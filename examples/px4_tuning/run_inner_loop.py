#!/usr/bin/env python3
"""
run_inner_loop.py — isolate which STAGE of the eigen controller loses stability under spin.

The spin ladder established that MC_CTRL_ALG=3 holds position cleanly at 4.9 rad/s and
diverges at 7.9 rad/s (growth +0.39 1/s, doubling every 1.8 s, mode at 11.3 rad/s
retrograde). It did not establish WHICH stage is responsible, and the frequency alone
cannot settle it: at r ~ 8 the spin rate and MC_EIG_ATT_P are numerically equal, so
sqrt(ATT_P^2 + r^2), sqrt(2)*r and several other candidates all land on ~11.3.

This isolates it structurally rather than spectrally, by flying the same vehicle at the
same spin rate through progressively less of the control law:

    OFFBOARD    ControlLevel::Trajectory   position stage + angle stage + eigen torque
    STABILIZED  ControlLevel::Attitude                     angle stage + eigen torque
    ACRO        ControlLevel::BodyRate                                   eigen torque

Read the result off the first mode that is stable:

    diverges in ACRO         -> the eigen rate law itself (MC_EIG_WN/B, inertias, TRQ_MAX)
    stable ACRO, bad STAB    -> angleToRateSetpoint: the Euler-kinematic coupling, i.e.
                                commanding BODY rates to correct EULER errors while
                                yawing. This is "Fix B" - invert the kinematics.
    stable STAB, bad OFFBOARD-> the trajectory stage: the cached roll/pitch setpoint goes
                                stale in azimuth at 125 Hz while the vehicle spins.
                                This is "Fix A".

ALTITUDE MODE IS NOT AN OPTION HERE. ALTCTL sets flag_control_altitude_enabled and
flag_control_climb_rate_enabled; Commander.cpp:2652 derives
flag_multicopter_position_control_enabled from those, so ControlLevelResolver returns
Trajectory exactly as it does for POSCTL. PX4's altitude hold *is* the outer loop. To keep
altitude while excluding it, PX4Driver.hold_stabilized() closes a deliberately slow
(<0.5 rad/s) loop on the throttle stick from out here, more than a decade below the
11 rad/s mode being measured.

Usage
─────
  # in another terminal, with the spin you want:
  isaac_run examples/px4_flight/single_vehicle.py --keep-log --motor-moment 1.6e-7 \
      --no-px4-autolaunch      # or let it autolaunch and just run this after

  python3 run_inner_loop.py --altitude 5 --hold 40
  python3 run_inner_loop.py --stages stab            # STABILIZED only
  python3 run_inner_loop.py --stages acro,stab       # rate law first, then add the angles
"""
from __future__ import annotations

import argparse
import sys
import time

from px4_driver import PX4Driver, log


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--altitude", type=float, default=5.0,
                   help="Hold altitude [m]. High enough that a departure is visible "
                        "before it reaches the ground.")
    p.add_argument("--hold", type=float, default=40.0,
                   help="Seconds per stage. Needs to be long against the observed "
                        "1.8 s doubling time - 40 s is ~22 doublings of headroom.")
    p.add_argument("--stages", type=str, default="stab",
                   help="Comma-separated subset of offboard,stab,acro, flown in the "
                        "order given. Default 'stab' answers the immediate question.")
    p.add_argument("--acro-seconds", type=float, default=10.0,
                   help="ACRO has no attitude feedback at all, so it will drift off "
                        "level on its own. Keep it short.")
    return p.parse_args()


ARGS = parse_args()


def main() -> int:
    stages = [s.strip().lower() for s in ARGS.stages.split(",") if s.strip()]
    unknown = set(stages) - {"offboard", "stab", "acro"}
    if unknown:
        sys.exit(f"unknown stage(s): {', '.join(sorted(unknown))}")

    with PX4Driver() as drv:
        # Refuses if the Pegasus airframe did not load - every ulog before 2026-08-07
        # was a Gazebo x500 for want of this check.
        drv.check_airframe()
        drv.enable_joystick()

        if not drv.takeoff(alt=ARGS.altitude, climb_time=14.0):
            log("takeoff failed")
            return 1

        # Let the climb transient decay before any stage is measured, otherwise the
        # first seconds of stage 1 are settling rather than the mode under test.
        drv.hold(0, 0, -ARGS.altitude, 8.0)
        log(f"settled at {ARGS.altitude:.1f} m; flying stages: {', '.join(stages)}")

        for stage in stages:
            if stage == "offboard":
                log("=== OFFBOARD (Trajectory: full law, the known-diverging case) ===")
                drv.hold(0, 0, -ARGS.altitude, ARGS.hold)

            elif stage == "stab":
                log("=== STABILIZED (Attitude: no position stage) ===")
                if not drv.hold_stabilized(ARGS.hold, ARGS.altitude):
                    return 1

            elif stage == "acro":
                log("=== ACRO (BodyRate: eigen torque only, no angle stage) ===")
                # Sticks centred is a ZERO RATE command, not level flight - nothing
                # returns this vehicle to level, so it leaves on whatever attitude it
                # had. Short, and recovered in OFFBOARD afterwards.
                thr = drv.param_get("MPC_THR_HOVER") or 0.58
                for _ in range(40):
                    drv.send_manual(0.0, 0.0, 0.0, thr)
                    time.sleep(0.02)
                if not drv.set_mode("ACRO"):
                    return 1
                end = time.time() + ARGS.acro_seconds
                while time.time() < end:
                    drv.send_manual(0.0, 0.0, 0.0, thr)
                    time.sleep(0.02)

            # Back to a controlled hover between stages so each one starts from the
            # same state rather than inheriting the previous stage's attitude.
            drv.enter_offboard(0, 0, -ARGS.altitude)
            drv.hold(0, 0, -ARGS.altitude, 6.0)

            if drv.is_diverging():
                log(f"DIVERGING after stage '{stage}' - landing")
                break

        drv.land()
        time.sleep(10)
        drv.disarm()

    log("done - run flight_health.py on the ulog and compare the stages by timestamp")
    return 0


if __name__ == "__main__":
    sys.exit(main())
