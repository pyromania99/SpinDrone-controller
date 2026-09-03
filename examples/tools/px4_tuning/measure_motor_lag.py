#!/usr/bin/env python3
"""
measure_motor_lag.py — measure the rotor/ESC time constant T and the total loop delay
tau_eff from a REAL hardware ulog (Kakute H7).

WHY THESE TWO NUMBERS
─────────────────────
Pegasus models the rotor as instantaneous - quadratic_thrust_curve.py:99 literally says
"instanenous model - no delay introduced" - so the simulated plant is missing what is
probably the largest lag in the hardware loop. Every gain found in sim so far was fitted
to a plant without it.

The stability model says the quantity that matters is

    tau_eff = tau_transport + T_rotor

and that gains scale roughly INVERSELY with it. Current numbers:

    sim now        tau_eff = 12 ms   (8 ms of which is a HIL artifact the board lacks)
    sim + rotor    tau_eff = 32 ms
    Kakute (est)   tau_eff = 23 ms

At an assumed T = 20 ms the model finds NO stable wn at r = 25 rad/s, which - if the real
motor is that slow - makes 25 rad/s an actuator-bandwidth limit rather than a tuning
problem. That conclusion is worth an afternoon on the bench before any more searching.

WHAT THIS READS
───────────────
  esc_status.esc_rpm     -> T directly, per motor, from a commanded step
  vehicle_torque_setpoint
  vehicle_angular_velocity -> tau_eff, via the same cross-correlation loop_latency.py uses

T needs bidirectional DShot (DSHOT_BIDIR_EN=1) and an ESC that supports it (BLHeli_32 /
AM32). Without it the script still reports tau_eff, which is the number the model actually
scales on - it just cannot split it into transport and motor.

BENCH PROCEDURE (props OFF for the step test - it is a bare motor spin-up)
──────────────────────────────────────────────────────────────────────────
    param set DSHOT_BIDIR_EN 1
    param set SDLOG_PROFILE 27
    param set SDLOG_MODE 2          # log from boot, not just when armed
    reboot
    logger start -f
    actuator_test set -m 1 -v 0.25 -t 3
    actuator_test set -m 1 -v 0.55 -t 3      # <- the step this fits
    actuator_test set -m 1 -v 0.25 -t 3
    logger stop

Repeat per motor if you want per-motor numbers. Steps in the 0.2-0.6 band keep the ESC out
of its startup region, where the response is not first-order at all.

Usage
─────
  measure_motor_lag.py hardware.ulg
  measure_motor_lag.py hardware.ulg --min-step 0.1
"""
from __future__ import annotations

import argparse
import sys

import numpy as np

try:
    from pyulog import ULog
except ImportError:
    sys.exit("pyulog not found: pip install pyulog")


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("ulog")
    p.add_argument("--min-step", type=float, default=0.08,
                   help="Smallest commanded jump in normalized output treated as a step.")
    p.add_argument("--settle", type=float, default=1.5,
                   help="Seconds after a step to fit over.")
    return p.parse_args()


ARGS = parse_args()


def fit_first_order(t: np.ndarray, y: np.ndarray) -> tuple[float, float]:
    """Fit y(t) = y_inf + (y0 - y_inf)*exp(-t/T). Returns (T, r2).

    Fitted on the LOG of the remaining error rather than by nonlinear least squares:
    ln|y - y_inf| is linear in t with slope -1/T, which needs no initial guess and cannot
    land in a local minimum. y_inf is taken as the mean of the last 25% of the window,
    which is why the window must be long enough to actually settle.
    """
    if len(t) < 20:
        return float("nan"), 0.0
    y_inf = float(np.mean(y[int(0.75 * len(y)):]))
    err = y_inf - y
    # Keep only the part of the transient that is still meaningfully away from settled;
    # once the residual is at the noise floor its log is noise with a slope of its own.
    span = abs(err[0])
    if span < 1e-6:
        return float("nan"), 0.0
    keep = np.abs(err) > 0.1 * span
    if keep.sum() < 10:
        return float("nan"), 0.0
    tt, ee = t[keep], np.abs(err[keep])
    slope, icept = np.polyfit(tt, np.log(ee), 1)
    if slope >= 0:
        return float("nan"), 0.0
    pred = np.exp(icept + slope * tt)
    ss_res = float(np.sum((ee - pred) ** 2))
    ss_tot = float(np.sum((ee - ee.mean()) ** 2))
    return float(-1.0 / slope), (1.0 - ss_res / ss_tot if ss_tot > 0 else 0.0)


def measure_T(u: ULog) -> None:
    d = {x.name: x for x in u.data_list}
    if "esc_status" not in d or "actuator_motors" not in d:
        print("  esc_status or actuator_motors missing -> cannot separate T.")
        print("  Enable bidirectional DShot (DSHOT_BIDIR_EN=1) on an ESC that supports it,")
        print("  or accept tau_eff below as the combined number.")
        return

    esc = d["esc_status"]; am = d["actuator_motors"]
    te = esc.data["timestamp"] * 1e-6
    ta = am.data["timestamp"] * 1e-6
    t0 = min(te[0], ta[0])
    te -= t0; ta -= t0

    n_esc = int(np.max(esc.data.get("esc_count", np.array([4]))))
    print(f"  esc_status: {len(te)} samples, {n_esc} ESCs reporting")

    any_fit = False
    for m in range(min(4, n_esc)):
        rpm_key = f"esc[{m}].esc_rpm"
        if rpm_key not in esc.data:
            continue
        rpm = esc.data[rpm_key].astype(float)
        if np.abs(rpm).max() < 100:
            continue                      # no telemetry on this channel
        cmd = am.data[f"control[{m}]"]
        jumps = np.where(np.abs(np.diff(cmd)) > ARGS.min_step)[0]
        for j in jumps:
            t_step = ta[j + 1]
            w = (te >= t_step) & (te <= t_step + ARGS.settle)
            if w.sum() < 20:
                continue
            T, r2 = fit_first_order(te[w] - t_step, rpm[w])
            if not np.isfinite(T) or r2 < 0.8:
                continue
            any_fit = True
            print(f"    motor {m}  step {cmd[j]:.2f}->{cmd[j+1]:.2f} at t={t_step:6.2f}s"
                  f"   T = {1000*T:6.1f} ms   r2 {r2:.3f}")
    if not any_fit:
        print("    no clean step found. Command one: actuator_test set -m 1 -v 0.55 -t 3")
    else:
        print("    NOTE: T here is the ROTOR VELOCITY time constant, which is exactly what")
        print("    quadratic_thrust_curve.py needs (it integrates velocity, then squares it")
        print("    for thrust). Do not convert it to a thrust time constant.")


def measure_tau_eff(u: ULog) -> None:
    d = {x.name: x for x in u.data_list}
    if "vehicle_torque_setpoint" not in d or "vehicle_angular_velocity" not in d:
        print("  need vehicle_torque_setpoint + vehicle_angular_velocity for tau_eff")
        return
    tq = d["vehicle_torque_setpoint"]; av = d["vehicle_angular_velocity"]
    t0 = min(tq.data["timestamp"][0], av.data["timestamp"][0]) * 1e-6
    tt = tq.data["timestamp"] * 1e-6 - t0
    tv = av.data["timestamp"] * 1e-6 - t0
    lo, hi = max(tt[0], tv[0]), min(tt[-1], tv[-1])
    dt = float(min(np.median(np.diff(tt)), np.median(np.diff(tv))))
    grid = np.arange(lo, hi, dt)
    for ax, name in ((0, "roll"), (1, "pitch")):
        tau_u = np.interp(grid, tt, tq.data[f"xyz[{ax}]"])
        key = f"xyz_derivative[{ax}]"
        alpha = (np.interp(grid, tv, av.data[key]) if key in av.data
                 else np.gradient(np.interp(grid, tv, av.data[f"xyz[{ax}]"]), dt))
        a = tau_u - tau_u.mean(); b = alpha - alpha.mean()
        if a.std() < 1e-6 or b.std() < 1e-6:
            print(f"  {name}: no excitation")
            continue
        nmax = int(0.12 / dt)
        den = np.sqrt((a ** 2).sum() * (b ** 2).sum())
        corr = np.array([float((a[:len(a) - k] * b[k:]).sum() / den) for k in range(nmax)])
        peak = corr.max()
        # Earliest strong peak, not argmax: a narrowband signal correlates again every
        # period, and argmax picks among those echoes at random.
        strong = np.flatnonzero(corr >= 0.95 * peak)
        local = [k for k in strong if (k == 0 or corr[k] >= corr[k - 1])
                 and (k == len(corr) - 1 or corr[k] >= corr[k + 1])]
        best = int(local[0]) if local else int(np.argmax(corr))
        print(f"  {name}: tau_eff = {best*dt*1e3:5.1f} ms   (peak r {corr[best]:.2f})")


def main() -> int:
    u = ULog(ARGS.ulog, ["esc_status", "actuator_motors", "vehicle_torque_setpoint",
                         "vehicle_angular_velocity"])
    P = u.initial_parameters
    print(f"=== {ARGS.ulog}")
    print(f"  DSHOT_BIDIR_EN={P.get('DSHOT_BIDIR_EN')}  SYS_AUTOSTART={P.get('SYS_AUTOSTART')}"
          f"  IMU_GYRO_RATEMAX={P.get('IMU_GYRO_RATEMAX')}  EKF2_PREDICT_US={P.get('EKF2_PREDICT_US')}")
    print("\nROTOR TIME CONSTANT T (from commanded steps):")
    measure_T(u)
    print("\nTOTAL LOOP DELAY tau_eff (torque command -> angular acceleration):")
    measure_tau_eff(u)
    print("\nFeed T into quadratic_thrust_curve.py as rotor_tau, and tau_eff into the")
    print("stability model / SensorLag. tau_eff on hardware should be roughly")
    print("tau_transport + T; if it is not, one of the two is mismeasured.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
