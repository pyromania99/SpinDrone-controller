#!/usr/bin/env python3
"""
run_step_id.py — fly a repeatable step sequence for per-axis system identification.

Purpose
───────
Every K_eff measurement so far has been taken from flight data that happened to exist,
and each was confounded: quiet hover gave correlation 0.01, a tumble gave 0.21, and the
gyroscopic term - which the commanded torque is partly cancelling - biased the regression
by about half until it was removed explicitly.

This produces excitation on purpose. A NORTH step tilts the vehicle nose-down and excites
PITCH; an EAST step rolls it and excites ROLL. Run separately, they identify each axis
without the other contributing, which is what the open question needs: roll reconciles
with the analytic 3.843 N.m while pitch measures 1.98, and nothing so far explains it.

Pair this with SDLOG_PROFILE=25 (set in the none_iris airframe) so actuator_motors and
the rate topics log unthrottled rather than at 10-50 Hz.

Caveat worth knowing: if the vehicle is spinning, a world-frame step does not isolate a
body axis - the tilt demand rotates through roll and pitch at the spin rate. For the
cleanest per-axis numbers set motor_moment = 0 for this run, identify K_eff, then restore
the spin. The gyro-corrected regression can cope with spin, but there is no reason to make
it work harder than necessary.

Usage
─────
  run_step_id.py                       # 3 m steps, default altitude
  run_step_id.py --step 4 --alt 6
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from px4_driver import PX4Driver, log  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--conn", default="udp:127.0.0.1:14540")
    ap.add_argument("--instance", type=int, default=0)
    ap.add_argument("--alt", type=float, default=5.0)
    ap.add_argument("--step", type=float, default=3.0)
    ap.add_argument("--hold", type=float, default=10.0, help="settle time after each step")
    a = ap.parse_args()

    with PX4Driver(a.conn, a.instance) as d:
        for k in ("MC_EIG_TRQ_MAX", "MC_EIG_WN", "MC_EIG_B", "MC_EIG_ATT_P",
                  "MC_EIG_ATT_D", "CA_ROTOR0_PX", "CA_ROTOR0_PY", "SDLOG_PROFILE"):
            log(f"  {k:16} = {d.param_get(k)}")

        if not d.takeoff(a.alt, climb_time=14.0):
            log("takeoff failed")
            return 1

        # Marker gap so the segments are trivially separable in the log.
        seq = [
            ("settle",      0.0,      0.0, 8.0),
            ("NORTH  +x  (excites PITCH)",  a.step,  0.0, a.hold),
            ("return",      0.0,      0.0, a.hold),
            ("EAST   +y  (excites ROLL)",   0.0,  a.step, a.hold),
            ("return",      0.0,      0.0, a.hold),
            ("NORTH  -x  (excites PITCH)", -a.step,  0.0, a.hold),
            ("return",      0.0,      0.0, a.hold),
            ("EAST   -y  (excites ROLL)",   0.0, -a.step, a.hold),
            ("return",      0.0,      0.0, a.hold),
        ]
        t0 = time.time()
        for name, n, e, hold in seq:
            log(f"t={time.time()-t0:6.1f}s  {name}")
            d.hold(n, e, -a.alt, hold)

        log(f"t={time.time()-t0:6.1f}s  landing")
        d.land()
        time.sleep(8)
        log("sequence complete")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
