#!/usr/bin/env python3
"""
failure_metrics.py — score a level-2/3 run in which one rotor was stopped.

Reads the ulog, finds when the motor ACTUALLY stopped, and reports the flight either
side of that instant. Built for runs flown with `fly.py --fail-rotor`, which stops a
motor through PX4's own injector (FailureInjector -> failure_detector_status.
motor_stop_mask -> ControlAllocator).

Why the failure time is read from the log and not taken from --fail-at
──────────────────────────────────────────────────────────────────────
The MAVLink command is timed, but commander only republishes failure_detector_status
every 500 ms (Commander.cpp, the 2 Hz status block), so the allocator learns about the
stop up to half a second after the command was sent. At r ~ 25 rad/s that is two whole
revolutions. Scoring a window that starts at the commanded time would put up to 0.5 s
of healthy four-rotor flight inside the "after" window.

Two independent witnesses are printed and should agree to within a logging interval:
  motor_stop_mask    what commander told the allocator
  actuator_motors    the first NaN on that motor's channel - the command that Pegasus
                     actually stopped receiving

Survival is NOT the headline number. A quadrotor that survives in a coning limit cycle
passes a duration check while flying 13.7 deg off vertical (LEVEL3_HOVER.md §4), so
attitude after the failure is what this reports first.

Usage
  python3 experiments/tools/failure_metrics.py px4_runs/L3_eigen_2026... [more runs]
  python3 experiments/tools/failure_metrics.py --pre 10 --post-from 5 --post 20 RUN
"""
from __future__ import annotations

import argparse
import glob
import os
import sys

import numpy as np
import pyulog

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import plot_run                                            # noqa: E402


def failure_time(run_dir, t0_us):
    """(t_mask, t_nan, motor_index) on plot_run's clock, any of which may be None.

    t0_us is plot_run.load()'s time origin - the first ground-truth position sample -
    so these line up with every other series without a second convention to track.
    """
    ulg = (glob.glob(os.path.join(run_dir, "log", "*", "*.ulg"))
           + glob.glob(os.path.join(run_dir, "fs", "log", "*", "*.ulg")))
    if not ulg:
        raise SystemExit(f"no ulog under {run_dir}")
    u = pyulog.ULog(ulg[0])

    t_mask, idx = None, None
    tf, fd = plot_run._series(u, "failure_detector_status", ["motor_stop_mask"])
    if fd and "motor_stop_mask" in fd:
        mask = fd["motor_stop_mask"].astype(np.int64)
        hit = np.flatnonzero(mask != 0)
        if hit.size:
            t_mask = float((tf[hit[0]] - t0_us) / 1e6)
            idx = int(np.log2(mask[hit[0]] & -mask[hit[0]]))   # lowest set bit

    t_nan = None
    tm, am = plot_run._series(u, "actuator_motors",
                              [f"control[{i}]" for i in range(4)])
    if am:
        # Which channel went NaN, in case the mask was never logged. NaN here is how
        # ControlAllocator spells "this motor is stopped"; it is also what a disarmed
        # motor looks like, so only samples after takeoff count.
        for i in range(4):
            c = am.get(f"control[{i}]")
            if c is None:
                continue
            bad = np.isnan(c) & (tm > t0_us)
            if not bad.any():
                continue
            first = float((tm[np.flatnonzero(bad)[0]] - t0_us) / 1e6)
            if idx is not None and i != idx:
                continue
            if t_nan is None or first < t_nan:
                t_nan, idx = first, i
    return t_mask, t_nan, idx


def _win(t, v, lo, hi):
    m = (t >= lo) & (t < hi) & np.isfinite(v)
    return v[m] if m.any() else np.array([])


def stats(d, lo, hi):
    """The same numbers the hover experiments quote, over one window."""
    out = {}
    tilt = _win(d["t_att"], np.degrees(d["tilt"]), lo, hi) if "tilt" in d else np.array([])
    out["tilt"] = float(tilt.mean()) if tilt.size else float("nan")
    pq = _win(d["t_w"], d["pq"], lo, hi) if "pq" in d else np.array([])
    out["pq"] = float(pq.mean()) if pq.size else float("nan")
    r = _win(d["t_w"], d["r"], lo, hi) if "r" in d else np.array([])
    out["r"] = float(r.mean()) if r.size else float("nan")
    alt = _win(d["t_pos"], d["alt"], lo, hi)
    out["alt"] = float(alt.mean()) if alt.size else float("nan")
    out["alt_sd"] = float(alt.std()) if alt.size else float("nan")
    m = (d["t_pos"] >= lo) & (d["t_pos"] < hi)
    out["xy"] = (float(np.hypot(d["x"][m] - d["x"][m].mean(),
                                d["y"][m] - d["y"][m].mean()).mean())
                 if m.any() else float("nan"))
    if "seat_theta" in d and d["seat_theta"] is not None:
        th = _win(d["t_cs"], d["seat_theta"], lo, hi)
        out["theta_s"] = (float(th.mean()), float(th.std())) if th.size else None
    else:
        out["theta_s"] = None
    return out


def report(run_dir, pre, post_from, post):
    d = plot_run.load(run_dir)
    ulg = (glob.glob(os.path.join(run_dir, "log", "*", "*.ulg"))
           + glob.glob(os.path.join(run_dir, "fs", "log", "*", "*.ulg")))
    u = pyulog.ULog(ulg[0])
    tp, _ = plot_run._series(u, "vehicle_local_position_groundtruth", ["x"])
    t_mask, t_nan, idx = failure_time(run_dir, tp[0])

    t_fail = t_nan if t_nan is not None else t_mask
    t_end = float(d["t_pos"][-1])
    t_crash = plot_run.crash_time(d)

    print(f"\n=== {os.path.basename(run_dir)} ===")
    print(f"  seat mode      MC_SEAT_MODE={d['params'].get('MC_SEAT_MODE', 0):g}   "
          f"km={d['cfg'].get('motor_moment', float('nan')):.4g}   "
          f"rotor_tau={d['cfg'].get('rotor_tau', float('nan'))}")
    if t_fail is None:
        print("  NO FAILURE FOUND in this log - motor_stop_mask never set and no "
              "actuator_motors channel went NaN. Check CA_FAILURE_MODE and "
              "SYS_FAILURE_EN, and that the injection command was acked.")
        return
    print(f"  failure        motor index {idx} (PX4 instance {idx + 1})")
    print(f"    mask set at  {t_mask if t_mask is None else round(t_mask, 3)} s"
          f"   first NaN at {t_nan if t_nan is None else round(t_nan, 3)} s"
          + ("" if (t_mask is None or t_nan is None)
             else f"   (lag {t_nan - t_mask:+.3f} s)"))

    if t_crash is None:
        print(f"  outcome        NO CRASH - flew to the end of the log, "
              f"{t_end - t_fail:.1f} s after the failure")
    else:
        print(f"  outcome        CRASH at {t_crash:.1f} s, "
              f"{t_crash - t_fail:.1f} s after the failure")
    t_stop = t_crash if t_crash is not None else t_end

    # When the vehicle does not survive `post_from` seconds there is no new steady
    # state to measure, and skipping the transient would report nothing at all. Fall
    # back to the whole post-failure interval and say so, because that number is a
    # departure, not a hold, and must not be read as one.
    settled = (t_stop - t_fail) > (post_from + 1.0)
    after = (("after", min(t_fail + post_from, t_stop),
              min(t_fail + post_from + post, t_stop)) if settled
             else ("after*", t_fail, t_stop))
    rows = [("before", max(t_fail - pre, 0.0), t_fail), after]
    if not settled:
        print("  NOTE: it did not survive long enough for a settled window; 'after*' "
              "spans the whole post-failure interval and is a DEPARTURE, not a hold.")
    print(f"  {'window':8s} {'span [s]':>13s} {'tilt[deg]':>10s} {'|p,q|':>8s} "
          f"{'r':>8s} {'alt[m]':>8s} {'alt sd':>8s} {'xy sd[m]':>9s}  theta_s")
    for name, lo, hi in rows:
        if hi - lo < 0.2:
            print(f"  {name:8s} {'--- no window (the flight ended first) ---':>60s}")
            continue
        s = stats(d, lo, hi)
        th = ("--" if s["theta_s"] is None
              else f"{s['theta_s'][0]:+.3f} +/- {s['theta_s'][1]:.3f}")
        print(f"  {name:8s} {lo:6.1f}-{hi:6.1f} {s['tilt']:10.2f} {s['pq']:8.3f} "
              f"{s['r']:8.2f} {s['alt']:8.2f} {s['alt_sd']:8.3f} {s['xy']:9.3f}  {th}")


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("runs", nargs="+")
    p.add_argument("--pre", type=float, default=10.0,
                   help="Seconds of settled flight before the failure to average over.")
    p.add_argument("--post-from", type=float, default=5.0,
                   help="Skip this many seconds after the failure before measuring, so "
                        "the transient is not averaged into the new steady state.")
    p.add_argument("--post", type=float, default=20.0,
                   help="Length of the post-failure window.")
    a = p.parse_args()
    for r in a.runs:
        report(r.rstrip("/"), a.pre, a.post_from, a.post)
    return 0


if __name__ == "__main__":
    sys.exit(main())
