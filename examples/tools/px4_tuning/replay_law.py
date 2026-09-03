#!/usr/bin/env python3
"""
| File: replay_law.py
| Description: Recompute what EigenController SHOULD have commanded, from the logged
|              state, and compare against what it DID command.

Why this exists
───────────────
Six mechanisms have been proposed for this controller's divergence and none survived.
Every one was inferred from PX4 logs alone, with nothing to compare against, so there
was no way to tell whether the controller was misbehaving or being asked for the wrong
thing.

This splits the problem in half without flying anything. Take the logged state, run the
law over it in Python, and compare to the logged torque:

  match    -> the law executes exactly as designed. The fault is downstream (actuation,
              MC_EIG_TRQ_MAX, allocation) or upstream (the setpoint it was given).
  diverge  -> intent and execution disagree, and WHICH term disagrees says where.

The reimplementation follows EigenController.cpp:221-305 term for term. Parameters come
from the log's own initial_parameters, so it cannot drift from the flight it is checking.

A caveat the numbers depend on: mc_controller_status is source-throttled to 50 Hz
(MulticopterController.cpp:681) while vehicle_angular_velocity logs at 10-20 ms, so the
inputs to the law and its logged outputs are sampled at different rates. Everything is
resampled onto the slower grid, which is adequate for modes below ~5 Hz and marginal
above. SDLOG_PROFILE=25 logs all of these unthrottled if better resolution is needed.

Usage
─────
  replay_law.py flight.ulg
  replay_law.py flight.ulg --window 34 50
"""
from __future__ import annotations

import argparse
import sys

import numpy as np

try:
    from pyulog import ULog
except ImportError:  # pragma: no cover
    sys.exit("pyulog not found. Try: /usr/bin/python3 -m pip install --user pyulog")


def _ds(u, name):
    try:
        return u.get_dataset(name)
    except Exception:
        return None


def _t(u, d):
    return (d.data["timestamp"] - u.start_timestamp) / 1e6


def _euler_from_q(w, x, y, z):
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1.0, 1.0))
    return roll, pitch


def replay(path: str, window=None):
    u = ULog(path)
    ip = u.initial_parameters

    P = {k: ip.get(k) for k in ("MC_EIG_ATT_P", "MC_EIG_ATT_D", "MC_EIG_WN", "MC_EIG_B",
                                "MC_EIG_ALPHA", "MC_EIG_BETA", "MC_EIG_IXX", "MC_EIG_IYY",
                                "MC_EIG_IZZ", "MC_EIG_TRQ_MAX")}
    if any(v is None for v in P.values()):
        raise RuntimeError(f"log lacks MC_EIG_* params: {P}")

    att, av = _ds(u, "vehicle_attitude"), _ds(u, "vehicle_angular_velocity")
    asp, tq = _ds(u, "vehicle_attitude_setpoint"), _ds(u, "vehicle_torque_setpoint")
    if any(d is None for d in (att, av, asp, tq)):
        raise RuntimeError("log lacks vehicle_attitude / angular_velocity / "
                           "attitude_setpoint / torque_setpoint")

    t_att, t_av, t_asp, t_tq = (_t(u, d) for d in (att, av, asp, tq))

    lo = window[0] if window else max(t_att[0], t_av[0], t_asp[0], t_tq[0])
    hi = window[1] if window else min(t_att[-1], t_av[-1], t_asp[-1], t_tq[-1])

    # Replay on the TORQUE timebase: that is the thing being predicted, and resampling
    # the prediction rather than the target avoids smearing what we are testing.
    m = (t_tq >= lo) & (t_tq < hi)
    grid = t_tq[m]
    if len(grid) < 100:
        raise RuntimeError(f"window {lo:.1f}-{hi:.1f}s has only {len(grid)} torque samples")

    def on_grid(td, v):
        return np.interp(grid, td, v)

    roll, pitch = _euler_from_q(*(att.data[f"q[{i}]"] for i in range(4)))
    roll, pitch = on_grid(t_att, roll), on_grid(t_att, pitch)
    roll_sp, pitch_sp = _euler_from_q(*(asp.data[f"q_d[{i}]"] for i in range(4)))
    roll_sp, pitch_sp = on_grid(t_asp, roll_sp), on_grid(t_asp, pitch_sp)
    p = on_grid(t_av, av.data["xyz[0]"])
    q = on_grid(t_av, av.data["xyz[1]"])
    r = on_grid(t_av, av.data["xyz[2]"])

    # --- EigenController::angleToRateSetpoint (:221-245) ---
    roll_err = np.arctan2(np.sin(roll_sp - roll), np.cos(roll_sp - roll))
    pitch_err = np.arctan2(np.sin(pitch_sp - pitch), np.cos(pitch_sp - pitch))
    # The flight code differentiates only on a fresh attitude sample and holds between;
    # on this grid a plain gradient is the closest faithful equivalent.
    d_roll = np.gradient(roll_err, grid)
    d_pitch = np.gradient(pitch_err, grid)
    rate_sp_roll = P["MC_EIG_ATT_P"] * roll_err + P["MC_EIG_ATT_D"] * d_roll
    rate_sp_pitch = P["MC_EIG_ATT_P"] * pitch_err + P["MC_EIG_ATT_D"] * d_pitch

    # --- EigenController::rateToTorque (:265-305) ---
    e_p = rate_sp_roll - p
    e_q = rate_sp_pitch - q
    wn, b, al, be = (P["MC_EIG_WN"], P["MC_EIG_B"], P["MC_EIG_ALPHA"], P["MC_EIG_BETA"])
    roll_acc = wn * e_p + b * e_q + al * p
    pitch_acc = wn * e_q - b * e_p + al * q
    yaw_acc = be * 0.0          # Attitude/Trajectory level commands zero yaw rate

    Ix, Iy, Iz = P["MC_EIG_IXX"], P["MC_EIG_IYY"], P["MC_EIG_IZZ"]
    tau_x = Ix * roll_acc + (Iz - Iy) * q * r
    tau_y = Iy * pitch_acc + (Ix - Iz) * p * r
    tau_z = Iz * yaw_acc + (Iy - Ix) * p * q

    trq = P["MC_EIG_TRQ_MAX"]
    pred = np.column_stack([tau_x, tau_y, tau_z]) / trq
    pred = np.clip(pred, -1.0, 1.0)
    logged = np.column_stack([tq.data[f"xyz[{i}]"][m] for i in range(3)])

    return dict(grid=grid, pred=pred, logged=logged, params=P,
                terms=dict(roll_err=roll_err, pitch_err=pitch_err,
                           rate_sp_roll=rate_sp_roll, rate_sp_pitch=rate_sp_pitch,
                           e_p=e_p, e_q=e_q, p=p, q=q, r=r,
                           gyro_x=(Iz - Iy) * q * r, gyro_y=(Ix - Iz) * p * r,
                           ctrl_x=Ix * roll_acc, ctrl_y=Iy * pitch_acc))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("ulog")
    ap.add_argument("--window", nargs=2, type=float, metavar=("START", "END"))
    a = ap.parse_args()

    R = replay(a.ulog, tuple(a.window) if a.window else None)
    pred, log, T = R["pred"], R["logged"], R["terms"]

    print(f"log     : {a.ulog}")
    print(f"window  : {R['grid'][0]:.1f} - {R['grid'][-1]:.1f} s  ({len(R['grid'])} samples)")
    print("params  : " + "  ".join(f"{k.replace('MC_EIG_','')}={v:g}" for k, v in R["params"].items()))

    print(f"\n{'axis':>6} {'corr':>7} {'slope':>8} {'rms pred':>10} {'rms log':>9} {'rms err':>9}")
    for i, nm in ((0, "roll"), (1, "pitch")):
        pv, lv = pred[:, i], log[:, i]
        c = np.corrcoef(pv, lv)[0, 1]
        s = float(np.linalg.lstsq(pv[:, None], lv, rcond=None)[0][0])
        print(f"{nm:>6} {c:7.3f} {s:8.3f} {pv.std():10.5f} {lv.std():9.5f} {(lv-pv).std():9.5f}")

    print("\n--- term breakdown (rms, physical N.m before TRQ_MAX) ---")
    print(f"  control  roll {T['ctrl_x'].std():.5f}   pitch {T['ctrl_y'].std():.5f}")
    print(f"  gyro     roll {T['gyro_x'].std():.5f}   pitch {T['gyro_y'].std():.5f}")
    print(f"  rate err roll {T['e_p'].std():.5f}   pitch {T['e_q'].std():.5f}")
    print(f"  rate_sp  roll {T['rate_sp_roll'].std():.5f}   pitch {T['rate_sp_pitch'].std():.5f}")
    print(f"  body rate  p  {T['p'].std():.5f}     q  {T['q'].std():.5f}   r mean {T['r'].mean():+.3f}")

    print("\ncorr ~1 and slope ~1  -> law executes as designed; look downstream/upstream")
    print("slope != 1            -> a scale error; TRQ_MAX and the inertias are the candidates")
    print("corr  < 1             -> structural mismatch; the term breakdown localises it")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
