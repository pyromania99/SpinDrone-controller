#!/usr/bin/env python3
"""
| File: loop_latency.py
| Description: Measure rate-loop latency and control effectiveness from a PX4 ulog.

Why this exists
───────────────
The 12.7 Hz limit cycle that killed the Isaac-Sim-derived gains implied ~39 ms of
loop delay, but that number came from assuming ``period/2 == delay`` on the limit
cycle itself, which is a rule of thumb rather than a measurement. This measures it
directly.

Method
──────
A commanded torque at time t produces angular acceleration at t + delay. So
cross-correlate ``vehicle_torque_setpoint`` against the angular acceleration
derived from ``vehicle_angular_velocity``, and read the delay off the peak. Both
signals are resampled onto a uniform grid first — ulog topics arrive at different
and slightly irregular rates, and a lag estimate is only as good as its time base.

The same regression gives ``K_eff``, the physical torque per unit of normalized
torque command, which is the number that makes sim→hardware a rescale rather than
a re-tune. Note it needs *excitation*: a quiet hover yields a correlation around
0.01 and a tumbling flight around 0.2, neither of which identifies anything. Run
this on a log containing commanded steps.

Usage
─────
  loop_latency.py flight.ulg
  loop_latency.py flight.ulg --window 40 90     # restrict to a time range
"""
from __future__ import annotations

import argparse
import sys

import numpy as np

try:
    from pyulog import ULog
except ImportError:  # pragma: no cover
    sys.exit("pyulog not found. Try: /usr/bin/python3 -m pip install --user pyulog")

# COMPOSITE diagonal inertia, PER AXIS - the whole articulation about its own centre of
# mass (vehicle_params.composite_mass_properties), matching MC_EIG_IXX/IYY in the
# airframe. Only used to turn the measured alpha/tau slope into newton-metres; the
# latency result does not depend on it.
#
# This was previously a single 0.029125 applied to both axes - the /body LINK alone,
# excluding the four rotors and their parallel-axis contribution. That understated K_eff
# by 19% on roll and 9% on pitch, and because the two axes differ it manufactured a
# roll/pitch asymmetry out of nothing. FINDINGS section 9's open question ("K_eff measured
# 2.20 vs 4.4325 derived, a clean 0.50x, cause unresolved") is largely this bug plus
# saturated samples; with both fixed the axes agree to 2.8%.
I_AXIS = {"roll": 0.03477, "pitch": 0.03185}

# What PX4's own RPY normalization gives for the CA_ROTOR* geometry CURRENTLY in the
# none_iris airframe. The point of measuring K_eff is to confirm or refute this.
#
# Was 4.4325, which is the value for the STOCK rotor arms (PX 0.1515, PY 0.245/0.1875)
# that the airframe replaced with measured ones. Worth knowing: the step-ID measurement
# lands at 4.30 (roll) / 4.42 (pitch), i.e. on the STOCK figure rather than this one, so
# the geometry change is itself under suspicion - see derive_px4_params.py, which reads
# the arms out of the running simulation instead of trusting either.
K_DERIVED = 3.843

# Torque commands at the rail cannot be delivered, so including them drags the regression
# slope down and understates K_eff. Samples above this magnitude are dropped.
SATURATION_CLIP = 0.9


def _dataset(ulog: ULog, name: str):
    try:
        return ulog.get_dataset(name)
    except Exception:
        return None


def _seconds(ulog: ULog, dataset) -> np.ndarray:
    return (dataset.data["timestamp"] - ulog.start_timestamp) / 1e6


def _airborne_start(ulog: ULog, default: float) -> float:
    land = _dataset(ulog, "vehicle_land_detected")
    if land is None:
        return default
    t = _seconds(ulog, land)
    airborne = np.where(land.data["landed"] == 0)[0]
    return float(t[airborne[0]]) if len(airborne) else default


def measure_axis(t_tau, tau, t_w, w, lo, hi, inertia, alpha_src=None, max_lag_ms=120.0):
    """Cross-correlate commanded torque against resulting angular acceleration.

    Returns (lag_ms, peak_correlation, K_eff) or None when there is too little
    excitation to identify anything.
    """
    # Uniform grid at the finer of the two native rates.
    dt = min(np.median(np.diff(t_tau)), np.median(np.diff(t_w)))
    grid = np.arange(lo, hi, dt)
    if len(grid) < 256:
        return None

    tau_u = np.interp(grid, t_tau, tau)
    w_u = np.interp(grid, t_w, w)

    # Angular acceleration. Prefer vehicle_angular_velocity.xyz_derivative when the caller
    # supplies it: PX4 computes that on the raw gyro stream before any of this resampling.
    #
    # The fallback below smooths over 5 samples, which at 250 Hz is a 20 ms window - long
    # against the 12-16 ms delay being measured. On the step-ID flight that attenuation
    # dragged K_eff to 3.75 (roll) and 2.88 (pitch), where the same data through
    # xyz_derivative gives a flat 4.3-4.5 on BOTH axes across two decades of command
    # amplitude. The 0.5x roll/pitch asymmetry of FINDINGS section 9 is largely this.
    if alpha_src is not None:
        alpha = np.interp(grid, t_w, alpha_src)
    else:
        w_s = np.convolve(w_u, np.ones(5) / 5.0, mode="same")
        alpha = np.gradient(w_s, dt)

    a = tau_u - tau_u.mean()
    b = alpha - alpha.mean()
    if a.std() < 1e-6 or b.std() < 1e-6:
        return None

    max_lag = int(round((max_lag_ms / 1000.0) / dt))
    lags = np.arange(0, max_lag + 1)
    denom = np.sqrt((a ** 2).sum() * (b ** 2).sum())
    corr = np.array([float((a[: len(a) - k] * b[k:]).sum() / denom) for k in lags])

    # Take the EARLIEST strong peak, not the global maximum. Cross-correlation of
    # a narrowband signal is ambiguous modulo its period: a 12.7 Hz limit cycle
    # produces near-equal peaks at lag, lag+79 ms, lag+158 ms... and argmax picks
    # among them essentially at random (measured 0.951 at 16 ms vs 0.958 at 96 ms).
    # Physical delay is the first one; the rest are echoes of the same alignment.
    peak = float(corr.max())
    strong = np.flatnonzero(corr >= 0.95 * peak)
    local = [k for k in strong
             if (k == 0 or corr[k] >= corr[k - 1]) and (k == len(corr) - 1 or corr[k] >= corr[k + 1])]
    best = int(local[0]) if local else int(np.argmax(corr))
    lag_ms = lags[best] * dt * 1000.0

    # Effectiveness from the lag-aligned pair, so the delay does not bias the fit.
    a_al = tau_u[: len(tau_u) - lags[best]]
    b_al = alpha[lags[best]:]
    # Excited enough to carry information, and below the rail. A saturated command is not
    # delivered, so those samples pull the slope down: on the step-ID flight they moved
    # K_eff from 4.29 to 3.65 on roll, a 15% understatement, with correlation dropping
    # 0.99 -> 0.93. Verified stable from clip 0.7 down to 0.1.
    excited = (np.abs(a_al) > max(0.02, 0.1 * np.abs(a_al).std())) & (np.abs(a_al) < SATURATION_CLIP)
    if excited.sum() < 100:
        return lag_ms, float(corr[best]), None
    # WITH an intercept. Forcing the fit through the origin makes it absorb any DC offset
    # in either signal - a trim torque, or a bias on the angular-acceleration estimate -
    # as slope, which is exactly the term being measured. On the step-ID flight the
    # no-intercept form read 3.54/2.74 against 4.3/4.4 here.
    design = np.column_stack([a_al[excited], np.ones(int(excited.sum()))])
    slope = float(np.linalg.lstsq(design, b_al[excited], rcond=None)[0][0])
    return lag_ms, float(corr[best]), slope * inertia


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("ulog")
    ap.add_argument("--window", nargs=2, type=float, metavar=("START", "END"),
                    help="restrict analysis to this time range (s from log start)")
    args = ap.parse_args()

    ulog = ULog(args.ulog)
    av = _dataset(ulog, "vehicle_angular_velocity")
    tq = _dataset(ulog, "vehicle_torque_setpoint")
    if av is None or tq is None:
        return print("need vehicle_angular_velocity and vehicle_torque_setpoint") or 1

    t_w, t_tau = _seconds(ulog, av), _seconds(ulog, tq)
    if args.window:
        lo, hi = args.window
    else:
        lo = _airborne_start(ulog, t_w[0]) + 2.0
        hi = float(t_w[-1])

    print(f"log      : {args.ulog}")
    print(f"window   : {lo:.1f} - {hi:.1f} s")
    ip = ulog.initial_parameters
    print(f"gains    : MC_PD_ATT_P={ip.get('MC_PD_ATT_P')}  MC_PD_ATT_D={ip.get('MC_PD_ATT_D')}")
    print(f"filters  : IMU_GYRO_CUTOFF={ip.get('IMU_GYRO_CUTOFF')}  "
          f"IMU_GYRO_RATEMAX={ip.get('IMU_GYRO_RATEMAX')}")

    print(f"\n{'axis':>6} {'lag':>9} {'peak r':>8} {'K_eff':>10} {'vs derived':>11}")
    lags = []
    for idx, name in ((0, "roll"), (1, "pitch")):
        got = measure_axis(t_tau, tq.data[f"xyz[{idx}]"], t_w, av.data[f"xyz[{idx}]"], lo, hi,
                           I_AXIS[name], av.data.get(f"xyz_derivative[{idx}]"))
        if got is None:
            print(f"{name:>6} {'--':>9} {'--':>8} {'--':>10}   too little data")
            continue
        lag_ms, r, k = got
        lags.append(lag_ms)
        k_s = f"{k:10.3f}" if k is not None else f"{'--':>10}"
        ratio = f"{k / K_DERIVED:10.2f}x" if k is not None else f"{'--':>11}"
        print(f"{name:>6} {lag_ms:7.1f}ms {r:8.2f} {k_s} {ratio}")
        if r < 0.4:
            print(f"       ^ correlation {r:.2f} is too low to trust - the log needs "
                  f"commanded excitation (steps), not hover or a tumble")

    if lags:
        print(f"\nloop latency (mean over axes): {np.mean(lags):.1f} ms")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
