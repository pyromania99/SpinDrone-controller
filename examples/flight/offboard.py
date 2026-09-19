#!/usr/bin/env python3
"""
offboard.py — the flight profile: arm, take off, hold, land, then point at the ulog.

sim_px4.py spawns the vehicle and runs PX4 but never arms it, so a bare run just sits on
the ground. This is the half that flies it, over MAVLink, from outside the simulator.

``fly.py --level 2|3`` calls ``fly()`` here, which is the normal way in - it launches the
sim as well. Run this module directly when the sim is already up and you want to fly it
without fly.py owning the launch, e.g. to watch a headed run in the GUI.

Usage
-----
  # normally, just:
  fly.py --level 2 --hold 90

  # directly, against a sim you launched yourself:
  isaac_run examples/flight/sim_px4.py --keep-log px4_runs/mytest \
      --motor-moment 2.7e-7 --duration 400
  offboard.py --run-dir px4_runs/mytest --hold 90
  offboard.py --run-dir ... --ground-truth     # MC_CTRL_GT=1, isolate the control law
  offboard.py --run-dir ... --gains MC_EIG_WN=34.18 MC_EIG_B=0.0
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from px4_driver import PX4Driver, log            # noqa: E402


# MC_CTRL_ALG values, from PX4 src/modules/mc_controller/module.yaml.
CTRL_ALG_NAMES = {
    0: "stock_cascade",
    1: "cascaded_pd",
    2: "template",
    3: "eigen",
}


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--run-dir", required=True,
                   help="The --keep-log directory the sim was launched with; the ulog is "
                        "read from <run-dir>/log/*/*.ulg")
    p.add_argument("--altitude", type=float, default=6.0)
    p.add_argument("--hold", type=float, default=90.0,
                   help="Seconds to hold altitude. Longer than gain_search's 25 s default "
                        "on purpose - a divergence that takes a minute to develop is "
                        "exactly what a short window misses.")
    p.add_argument("--boot-timeout", type=float, default=240.0)
    p.add_argument("--ctrl-alg", type=int, default=None,
                   choices=(0, 1, 2, 3),
                   help="PX4 control law to fly: 0 stock cascade, 1 cascaded PD, "
                        "2 template, 3 eigen dynamics. Omit to fly whatever the "
                        "airframe booted with (10016_none_iris defaults to 1).")
    p.add_argument("--ground-truth", action="store_true",
                   help="MC_CTRL_GT=1: feed the controller simulator truth, separating "
                        "'these gains are wrong' from 'the state reaching them is wrong'.")
    p.add_argument("--gains", nargs="*", default=[],
                   help="Optional NAME=VALUE overrides applied live, e.g. MC_EIG_WN=34.18. "
                        "Omit to fly exactly what the airframe set.")
    p.add_argument("--abort-on-loss", action="store_true",
                   help="Cut the hold short if MAVLink telemetry says altitude was lost. "
                        "OFF by default: that telemetry proved unreliable enough to abort "
                        "two healthy flights. Judge from the ulog instead.")
    p.add_argument("--trajectory", default=None,
                   help="CSV of time,x,y,z (z POSITIVE up, converted to NED here) to fly "
                        "instead of holding the takeoff point. Same file format the level-1 "
                        "runs use, so the two rungs fly the SAME path.")
    p.add_argument("--waypoint-radius", type=float, default=0.0,
                   help="With --trajectory: advance to the next point on ARRIVAL within this "
                        "radius [m] instead of on the CSV clock. 0 = timed (use only for a "
                        "smooth densely-sampled path).")
    p.add_argument("--lateral-amp", type=float, default=0.0,
                   help="Fly a slow lateral square-wave of this half-amplitude [m] during "
                        "the hold instead of holding the takeoff point. Needed by laws "
                        "whose observable is a TRANSLATION direction (MC_POLE_EN=2): with "
                        "no lateral demand the commanded horizontal acceleration is zero "
                        "and there is no direction to measure. 0 keeps the archived "
                        "hold-the-point profile, so existing comparisons are unaffected.")
    p.add_argument("--lateral-period", type=float, default=20.0,
                   help="Seconds per lateral leg. Must be long compared with the position "
                        "loop and with MC_POLE_FDIT's dither period, or the two excitations "
                        "alias into each other.")
    p.add_argument("--fail-rotor", type=int, default=None, metavar="N",
                   help="Stop PX4 motor N (1-based, as the `failure` console command "
                        "numbers them; 0 = all) partway through the flight, using PX4's "
                        "own injector. This is the level-2/3 equivalent of level 1's "
                        "motor_failure.py, and unlike that one it goes through the "
                        "allocator rather than around it.")
    p.add_argument("--fail-at", type=float, default=20.0, metavar="S",
                   help="Seconds into the hold (or the trajectory) at which --fail-rotor "
                        "fires. Default 20, which is past the spin-up and inside the "
                        "settled window every hover experiment scores over. The command "
                        "is timed, not settle-triggered: commander republishes the stop "
                        "mask at 2 Hz, so the motor actually stops up to 0.5 s later - "
                        "take the failure time from the ulog, not from this number.")
    p.add_argument("--no-land", action="store_true",
                   help="Leave it hovering instead of landing, to keep watching.")
    return p.parse_args()



def fly(ARGS) -> int:
    """Fly the profile against an already-running sim.

    Takes its options as a namespace rather than reading a module-level ARGS, so
    fly.py can drive this without going through the command line.
    """
    run_dir = Path(ARGS.run_dir).expanduser().resolve()

    drv = PX4Driver()
    deadline = time.time() + ARGS.boot_timeout
    ready = False
    while time.time() < deadline:
        try:
            drv.connect(timeout=15.0)
            # A UDP endpoint "connects" with nothing behind it and pymavlink then reports a
            # phantom heartbeat from system 0, so connect() alone proves nothing. Require a
            # real parameter read before believing PX4 is up.
            if drv.param_get("SYS_AUTOSTART") is not None:
                ready = True
                break
        except Exception:
            pass
        time.sleep(3.0)
    if not ready:
        sys.exit("PX4 never appeared - is the sim actually running?")

    drv.check_airframe()

    # Fly whichever control law is selected, rather than demanding one. This used
    # to require MC_CTRL_ALG==3 and exit otherwise, which failed every level-2/3
    # flight before it armed: 10016_none_iris boots with
    # `param set-default MC_CTRL_ALG 1`, so a correctly loaded airframe was
    # rejected for holding its own documented default.
    #
    # --ctrl-alg switches it, and is safe to apply live:
    # MulticopterController::handleParameterUpdate() applies a change as soon as
    # the vehicle is disarmed, which is its state here, before takeoff. set_gains
    # reads back, so a rejected value raises rather than flying something other
    # than what was asked for.
    if ARGS.ctrl_alg is not None:
        want_alg = int(ARGS.ctrl_alg)
        current = drv.param_get("MC_CTRL_ALG")
        if current is None or int(current) != want_alg:
            log(f"MC_CTRL_ALG={current} -> setting {want_alg}")
            drv.set_gains(MC_CTRL_ALG=want_alg)
    alg = drv.param_get("MC_CTRL_ALG")
    if alg is None:
        sys.exit("MC_CTRL_ALG unreadable - is mc_controller running?")
    log(f"control law: {CTRL_ALG_NAMES.get(int(alg), '?')} (MC_CTRL_ALG={int(alg)})")

    if ARGS.ground_truth:
        drv.set_gains(MC_CTRL_GT=1)
    if ARGS.gains:
        over = {}
        for kv in ARGS.gains:
            k, _, v = kv.partition("=")
            over[k.strip()] = float(v)
        drv.set_gains(**over)

    if getattr(ARGS, "fail_rotor", None) is not None:
        # Both of these are read by PX4 in places that make a silent failure look
        # like a successful flight, so check rather than set-and-hope.
        en = drv.param_get("SYS_FAILURE_EN")
        if en is None or int(en) != 1:
            sys.exit(f"SYS_FAILURE_EN={en}, must be 1 for --fail-rotor. It is read once "
                     "in FailureInjector's constructor, so setting it now would not take "
                     "effect until PX4 restarts - set it before boot (init.d-posix/rcS "
                     "already defaults it to 1).")
        drv.set_gains(CA_FAILURE_MODE=1)
        fd_act = drv.param_get("FD_ACT_EN")
        if fd_act is not None and int(fd_act) != 0:
            log(f"WARNING: FD_ACT_EN={int(fd_act)} - the allocator will DETECT the stopped "
                "motor and re-allocate around it. That is a different experiment from the "
                "level-1 one, where the controller is never told.")
        log(f"failure injection armed: motor {ARGS.fail_rotor} off at t+{ARGS.fail_at:.0f}s "
            "of the hold (CA_FAILURE_MODE=1, SYS_FAILURE_EN=1)")

    flown = {k: drv.param_get(k) for k in
             ("MC_EIG_WN", "MC_EIG_B", "MC_EIG_ATT_P", "MC_EIG_ATT_D", "MC_CTRL_GT")}
    log("flying: " + "  ".join(f"{k}={v}" for k, v in flown.items()))
    sep = (flown["MC_EIG_WN"] / flown["MC_EIG_ATT_P"]
           if flown["MC_EIG_WN"] and flown["MC_EIG_ATT_P"] else float("nan"))
    log(f"cascade separation wn/ATT_P = {sep:.2f}" +
        ("" if sep >= 3.0 else "   <-- BELOW 3, the loops will interact"))

    if not drv.takeoff(alt=ARGS.altitude, climb_time=14.0):
        sys.exit("takeoff failed")

    # One closure, shared by the trajectory branch and the hold loop, so the two
    # profiles cannot drift into injecting the failure differently.
    _failed = {"done": False}

    def _maybe_fail(elapsed: float) -> None:
        if getattr(ARGS, "fail_rotor", None) is None or _failed["done"]:
            return
        if elapsed < ARGS.fail_at:
            return
        _failed["done"] = True
        acks = drv.inject_motor_failure(ARGS.fail_rotor)
        log(f"*** INJECTED motor {ARGS.fail_rotor} OFF at t+{elapsed:.1f}s "
            f"(acks={acks}; 1=UNSUPPORTED from commander's default branch is expected "
            "alongside 0=ACCEPTED from the injector)")

    if ARGS.trajectory:
        import numpy as _np
        _tr = _np.genfromtxt(ARGS.trajectory, delimiter=",")
        # CSV carries altitude POSITIVE UP (level-1 convention); MAVLink wants NED down.
        _traj = _np.column_stack([_tr[:, 0], _tr[:, 1], _tr[:, 2], -_tr[:, 3]])
        _mode = (f"arrival-based, radius {ARGS.waypoint_radius} m"
                 if ARGS.waypoint_radius > 0 else "timed")
        log(f"flying trajectory {ARGS.trajectory} ({len(_traj)} points, {_mode})")
        drv.fly_trajectory(_traj, waypoint_radius=ARGS.waypoint_radius,
                           max_seconds=max(ARGS.hold, 60.0),
                           on_sample=lambda el, _pos, _row: _maybe_fail(el))
        log("trajectory complete")
        if not ARGS.no_land:
            drv.land()
            time.sleep(8.0)
        drv.disarm()
        return 0

    if ARGS.lateral_amp > 0.0:
        log(f"holding {ARGS.altitude} m for {ARGS.hold:.0f} s, with a "
            f"+/-{ARGS.lateral_amp:.1f} m lateral square-wave every "
            f"{ARGS.lateral_period:.0f} s (translation-observable laws need it)")
    else:
        log(f"holding {ARGS.altitude} m for {ARGS.hold:.0f} s")
    # Split the hold so divergence is reported as it happens rather than after the fact -
    # the point of watching a headed run is seeing WHEN it lets go, not just that it did.
    step = 5.0
    t0 = time.time()
    while time.time() - t0 < ARGS.hold:
        # North setpoint alternates each leg when --lateral-amp is set. A square wave
        # rather than a smooth sweep on purpose: the position loop's response to a STEP is
        # what produces a sustained, clearly-directed horizontal acceleration demand, which
        # is the signal a translation-direction observable needs. A slow sinusoid spends
        # most of its time near zero demand, gated off.
        n_sp = 0.0

        # Fire before the step rather than after: hold() blocks for up to `step`
        # seconds, so testing afterwards would quantise the failure to a 5 s grid.
        _maybe_fail(time.time() - t0)

        if ARGS.lateral_amp > 0.0:
            leg = int((time.time() - t0) / max(ARGS.lateral_period, 1e-3))
            n_sp = ARGS.lateral_amp * (1.0 if (leg % 2 == 0) else -1.0)

        drv.hold(n_sp, 0.0, -ARGS.altitude, min(step, ARGS.hold - (time.time() - t0)))
        el = time.time() - t0

        # is_diverging() alone is NOT enough. It trips on tilt > 60 deg or rate > 3 rad/s,
        # and the 2026-08-16 rank-1 flight sat at 29 deg roll while sinking 6 m -> 1 m
        # without ever tripping it, which got reported as "held 90 s, no divergence".
        # Losing the altitude you were commanded to hold IS the failure, whatever the
        # attitude is doing, so track it directly.
        # DRAIN, don't sample. pymavlink queues incoming messages and recv_match returns
        # the OLDEST, so after a 5 s hold() the first read is ~5 s stale. That reported
        # alt = -0.01 m (the pre-takeoff value) while the vehicle was actually holding
        # 6.00 m, and aborted a healthy flight. Take the newest of what has piled up.
        pos = att = None
        while True:
            m = drv.m.recv_match(type=["LOCAL_POSITION_NED", "ATTITUDE"], blocking=False)
            if m is None:
                break
            if m.get_type() == "LOCAL_POSITION_NED":
                pos = (m.x, m.y, m.z)
            else:
                att = (m.roll, m.pitch, m.yaw, m.rollspeed, m.pitchspeed, m.yawspeed)
        if pos is None:
            pos = drv.local_position()      # nothing buffered - fall back to a blocking read
        if att is None:
            att = drv.attitude()
        alt = -pos[2] if pos else None
        desc = (f"  alt={alt:.2f}m" if alt is not None else "")
        if att:
            desc += f" roll={att[0]:+.2f} pitch={att[1]:+.2f} r={att[5]:+.2f}rad/s"
        log(f"  t+{el:5.1f}s{desc}")

        # DO NOT abort on this telemetry. The MAVLink stream here is not reliable enough
        # to gate a flight on: draining the queue still yielded alt = -0.00 m and
        # r = +0.01 rad/s while the ulog showed the vehicle holding 6.01 m against a 6.00 m
        # setpoint at 13.2 rad/s. Two good ground-truth flights were cut short at t+5 s by
        # exactly that read. The ulog is the source of truth for whether a flight was good;
        # these lines are a progress indicator only.
        if ARGS.abort_on_loss and alt is not None and abs(alt - ARGS.altitude) > 1.5:
            log(f"LOST ALTITUDE at t+{el:.0f}s: {alt:.2f} m vs {ARGS.altitude:.1f} m "
                f"commanded (telemetry - verify against the ulog before believing it)")
            break

    if not ARGS.no_land:
        log("landing")
        drv.land()

    # Give the logger a moment to close the ulog before anything reads it.
    time.sleep(5.0)
    logs = sorted(run_dir.glob("log/*/*.ulg"))
    if not logs:
        log(f"no ulog under {run_dir}/log/ - was the sim launched with --keep-log {run_dir}?")
        return 1
    log(f"ulog: {logs[-1]}")
    log(f"score it with:  flight_health.py {logs[-1]}")
    return 0


if __name__ == "__main__":
    sys.exit(fly(parse_args()))
