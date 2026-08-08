#!/usr/bin/env python3
"""
| File: ulog_to_npz.py
| Description: Convert a PX4 ulog (or one trial window of it) into the npz that
|              sweep_metrics.compute_metrics expects.

The measurement half of the PX4 tuning harness. ``compute_metrics`` takes a *path*
and ``np.load``s it, so the cheapest way to reuse the entire existing scoring stack
unchanged is to write an npz with exactly its keys rather than refactor it.

The one conversion that carries real risk is per-rotor thrust. ``actuator_motors``
is normalized [0, 1]; every effort metric in sweep_metrics is in newtons, and
``SAT_THRESH`` is 342.4 N². Feeding normalized values straight through would not
error - it would silently report a vehicle that never saturates and rank effort
nonsensically. ``U_MAX_PER_MOTOR`` is imported rather than hardcoded so this cannot
drift from the value the metrics are calibrated against:

    U_MAX_PER_MOTOR = 8.54858e-6 * 1100^2 = 10.344 N

which is the same rotor_constant and max rotor velocity behind single_vehicle.py's
``input_scaling = 1100`` / ``zero_position_armed = 0`` and the airframe's
``CA_ROTOR*_CT = 10.344``. With that mapping omega = 1100*u exactly, so

    F_i = rotor_constant * (1100*u_i)^2 = U_MAX_PER_MOTOR * u_i^2

Everything is resampled onto one uniform grid because compute_metrics requires all
arrays to share a length and infers dt from ``t[1] - t[0]``.

Usage
─────
  ulog_to_npz.py flight.ulg out.npz
  ulog_to_npz.py flight.ulg out.npz --window 42.0 62.0     # one trial
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

try:
    from pyulog import ULog
except ImportError:  # pragma: no cover
    sys.exit("pyulog not found. Try: /usr/bin/python3 -m pip install --user pyulog")

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "utils"))
from sweep_metrics import U_MAX_PER_MOTOR  # noqa: E402  (path set above)

GRID_HZ = 100.0


def _ds(ulog: ULog, name: str):
    try:
        return ulog.get_dataset(name)
    except Exception:
        return None


def _t(ulog: ULog, d) -> np.ndarray:
    return (d.data["timestamp"] - ulog.start_timestamp) / 1e6


def _resample(grid, t, v):
    """Linear resample, holding the endpoints. NaNs are dropped first: unset
    setpoint axes are NaN by convention in PX4 and would otherwise poison the
    interpolation of the axes that ARE set."""
    v = np.asarray(v, dtype=float)
    ok = np.isfinite(v)
    if ok.sum() < 2:
        return np.zeros_like(grid)
    return np.interp(grid, t[ok], v[ok])


def convert(ulog_path: str, window: tuple[float, float] | None = None) -> dict:
    ulog = ULog(ulog_path)

    lp = _ds(ulog, "vehicle_local_position")
    av = _ds(ulog, "vehicle_angular_velocity")
    am = _ds(ulog, "actuator_motors")
    if lp is None or av is None or am is None:
        raise RuntimeError("log lacks vehicle_local_position / vehicle_angular_velocity / "
                           "actuator_motors")
    sp = _ds(ulog, "trajectory_setpoint")
    land = _ds(ulog, "vehicle_land_detected")

    t_lp, t_av, t_am = _t(ulog, lp), _t(ulog, av), _t(ulog, am)

    if window is not None:
        lo, hi = window
    else:
        lo = float(max(t_lp[0], t_av[0], t_am[0]))
        if land is not None:
            t_ld = _t(ulog, land)
            air = np.where(land.data["landed"] == 0)[0]
            if len(air):
                lo = float(t_ld[air[0]])
        hi = float(min(t_lp[-1], t_av[-1], t_am[-1]))
    if hi - lo < 0.5:
        raise RuntimeError(f"window {lo:.2f}-{hi:.2f}s too short")

    grid = np.arange(lo, hi, 1.0 / GRID_HZ)

    pos = np.column_stack([_resample(grid, t_lp, lp.data[ax]) for ax in ("x", "y", "z")])
    if sp is not None:
        t_sp = _t(ulog, sp)
        ref = np.column_stack([_resample(grid, t_sp, sp.data[f"position[{i}]"])
                               for i in range(3)])
        # An all-NaN setpoint axis resamples to zero; fall back to holding the
        # first position so a missing reference reads as "no error" rather than
        # as a large error against the origin.
        for i in range(3):
            if not np.isfinite(sp.data[f"position[{i}]"]).any():
                ref[:, i] = pos[0, i]
    else:
        ref = np.tile(pos[0], (len(grid), 1))

    ep = ref - pos

    omega = np.column_stack([_resample(grid, t_av, av.data[f"xyz[{i}]"]) for i in range(3)])

    u = np.column_stack([_resample(grid, t_am, am.data[f"control[{i}]"]) for i in range(4)])
    u = np.clip(u, 0.0, 1.0)
    motor_thrusts = U_MAX_PER_MOTOR * u ** 2

    alt = -pos[:, 2]
    out = {
        "time": grid - grid[0],
        "ep": ep,
        "angular_velocity": omega,
        "yaw_rate": omega[:, 2],
        "motor_thrusts": motor_thrusts,
        "min_altitude": float(alt.min()),
        "max_excursion": float(np.abs(ep).max()),
    }
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("ulog")
    ap.add_argument("out")
    ap.add_argument("--window", nargs=2, type=float, metavar=("START", "END"))
    args = ap.parse_args()

    d = convert(args.ulog, tuple(args.window) if args.window else None)
    np.savez(args.out, **d)

    thr = d["motor_thrusts"]
    print(f"wrote {args.out}")
    print(f"  samples        : {len(d['time'])} @ {GRID_HZ:.0f} Hz "
          f"({d['time'][-1]:.1f} s)")
    print(f"  |ep| mean/max  : {np.linalg.norm(d['ep'],axis=1).mean():.3f} / "
          f"{np.linalg.norm(d['ep'],axis=1).max():.3f} m")
    print(f"  motor_thrusts  : mean {thr.mean():.2f} N  peak {thr.max():.2f} N "
          f"(ceiling {U_MAX_PER_MOTOR:.3f} N)")
    if thr.max() <= 1.0:
        print("  !! peak <= 1.0 N - looks like normalized values, not newtons")
    print(f"  |omega| max    : {np.linalg.norm(d['angular_velocity'],axis=1).max():.3f} rad/s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
