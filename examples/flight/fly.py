#!/usr/bin/env python3
"""
fly.py — pick a level of the simulation ladder and fly it.

The three levels
────────────────
  1  Pegasus + Python      The controller is a Python Backend reading simulator
                           state directly. No PX4, no estimator, no transport
                           delay.
  2  PX4 + ground truth    Real PX4 - mixer, CommandFrontEnd limits, allocator,
                           normalized torque - but MC_CTRL_GT=1 feeds it
                           vehicle_attitude_groundtruth, so the estimator is out.
  3  PX4, estimator in     MC_CTRL_GT=0. EKF2 back in the loop.

They are a ladder, and the value of each rung is its DIFFERENCE from the one
below. Level 2 minus level 1 is PX4's implementation cost; level 3 minus level 2
is the estimator's cost at that spin rate. A number from one level on its own
says very little.

Why this is a front door and not one process
────────────────────────────────────────────
Levels 1 and 2/3 put a vehicle in the air using Isaac, which needs Isaac's own
interpreter (python.sh). The offboard half of levels 2 and 3 drives PX4 over
MAVLink and needs system python3 with pymavlink. No single process can be both,
so this script stays plain python3 and launches the Isaac side as a subprocess -
the same thing gain_search.py has always done. sim_pegasus.py and sim_px4.py are
those Isaac-side halves; they are runnable on their own, but this is the way in.

Usage
-----
  fly.py --level 1                                  # watch it, prompts for the rest
  fly.py --level 1 --controller eigen --headless
  fly.py --level 2 --motor-moment 2.7e-7 --hold 90  # PX4 on ground truth
  fly.py --level 3 --motor-moment 2.7e-7 --hold 90  # same candidate, estimator in
  fly.py --level 2 --attach --run-dir px4_runs/mine # a sim you started yourself

  fly.py --level 2 --gains MC_EIG_WN=34.18 MC_EIG_B=0.0

For a batch of candidates rather than one flight, use gain_search.py. For a
rotor-loss study, use motor_failure.py.
"""
from __future__ import annotations

import argparse
import atexit
import signal
import sys
import time
from datetime import datetime
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import simlaunch                                          # noqa: E402
import offboard                                           # noqa: E402
from px4_driver import log                                # noqa: E402

# Sims this process started, so they can be stopped however we exit.
#
# simlaunch starts Isaac with start_new_session=True, which is what lets the
# whole tree be killed deliberately - but it also means the child does NOT die
# with its parent. Without the handlers below, a Ctrl-C or a kill left Isaac
# running: holding the GPU, holding TCP 4560, and colliding with the next run.
# Level 1 was the worse case, having had no cleanup path at all.
_LAUNCHED = []


def _stop_launched() -> None:
    while _LAUNCHED:
        proc = _LAUNCHED.pop()
        try:
            if proc.poll() is None:
                simlaunch.kill(proc)
        except Exception:
            pass


def _install_cleanup() -> None:
    """Stop the sim on Ctrl-C, on SIGTERM, and on any normal exit."""
    atexit.register(_stop_launched)

    def _handler(signum, _frame):
        log(f"signal {signal.Signals(signum).name}; stopping the sim")
        _stop_launched()
        raise SystemExit(128 + signum)

    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, _handler)


LEVELS = {
    1: "Pegasus + Python (no PX4, no estimator)",
    2: "PX4 + Pegasus, ground truth (MC_CTRL_GT=1)",
    3: "PX4 + Pegasus, estimator in the loop (MC_CTRL_GT=0)",
}


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--level", type=int, required=True, choices=(1, 2, 3),
                   help="Which rung of the ladder to fly. See the table above.")

    p.add_argument("--motor-moment", type=float, default=2.7e-7,
                   help="Rotor drag moment km, which sets the steady yaw spin "
                        "(r_ss ~ 4.9e7 * km). Default 2.7e-7 is ~13.2 rad/s, the spin "
                        "the existing GA front was searched at.")
    p.add_argument("--headless", action="store_true",
                   help="Run without a viewport. Every level is headed by default: "
                        "fly.py flies one flight, and the usual reason to fly one is "
                        "to watch it. Batch work (gain_search.py) stays headless.")
    p.add_argument("--duration", type=float, default=None,
                   help="Level 1: how long to fly, in simulated seconds - without it "
                        "the run never ends on its own. Levels 2 and 3: hard cap on "
                        "the sim process, defaulting to --hold + 180.")
    p.add_argument("--desaturation", choices=("uniform", "collective"), default="uniform",
                   help="What the mixer gives up when a rotor saturates. Keep 'uniform' "
                        "to stay comparable with every archived run.")

    g1 = p.add_argument_group("level 1")
    g1.add_argument("--controller", choices=("pd", "nonlinear", "eigen", "mpc"),
                    default=None, help="Control law. Prompted for if omitted.")
    g1.add_argument("--failure-mode", choices=("manual", "auto", "off"), default=None)
    g1.add_argument("--rotor", type=int, choices=[0, 1, 2, 3], default=None)
    g1.add_argument("--seed", type=int, default=None)
    g1.add_argument("--rotor-tau", type=float, default=None, metavar="S",
                    help="Level 1 rotor/ESC lag [s]; 0 for instant response, matching "
                         "archives older than 2026-08-15.")
    g1.add_argument("--controller-gains", nargs="*", default=[], metavar="K=V",
                    help="Level 1 controller gains, e.g. Kp_att=60 Kd_att=1.481. "
                         "Distinct from --gains, which sets PX4 parameters on "
                         "levels 2 and 3.")
    g1.add_argument("--lateral", action="store_true", default=False,
                    help="Translate to the goal after taking off. Off by default so the "
                         "profile matches levels 2 and 3, which climb and hold the "
                         "takeoff point; a level-1 run that also translates is not "
                         "comparable with them.")
    g1.add_argument("--no-takeoff-first", dest="takeoff_first", action="store_false",
                    default=True,
                    help="With --lateral, fly one diagonal instead of climbing first. "
                         "The archived sweeps in results/ were flown this way.")

    g2 = p.add_argument_group("levels 2 and 3")
    g2.add_argument("--altitude", type=float, default=6.0)
    g2.add_argument("--hold", type=float, default=90.0,
                    help="Seconds to hold altitude before landing.")
    g2.add_argument("--run-dir", default=None,
                    help="Where the ulog goes. Default: px4_runs/<level>_<timestamp>.")
    g2.add_argument("--attach", action="store_true",
                    help="Do not launch anything - fly a sim that is already running. "
                         "Pair with --run-dir pointing at the directory it was given.")
    g2.add_argument("--ctrl-alg", type=int, default=None, choices=(0, 1, 2, 3),
                    help="PX4 control law to fly: 0 stock cascade, 1 cascaded PD, "
                         "2 template, 3 eigen dynamics. Omit to fly whatever the "
                         "airframe booted with (10016_none_iris defaults to 1). The "
                         "law also names the run directory, so two laws never share one.")
    g2.add_argument("--gains", nargs="*", default=[],
                    help="PX4 parameter overrides, e.g. MC_EIG_WN=34.18.")
    g2.add_argument("--boot-timeout", type=float, default=240.0)

    g2.add_argument("--no-land", action="store_true")
    g2.add_argument("--abort-on-loss", action="store_true",
                    help="Stop early if telemetry shows altitude lost. Off by default: "
                         "the MAVLink stream is not reliable enough to gate a flight on "
                         "- the ulog is the source of truth.")
    return p.parse_args()


def fly_level_1(a: argparse.Namespace) -> int:
    """Hand off to the Isaac-side level-1 sim and wait for it."""
    args = ["--motor-moment", repr(a.motor_moment), "--desaturation", a.desaturation]
    for flag, val in (("--controller", a.controller),
                      ("--failure-mode", a.failure_mode),
                      ("--rotor", a.rotor), ("--seed", a.seed)):
        if val is not None:
            args += [flag, str(val)]
    if a.duration is not None:
        args += ["--duration", repr(a.duration)]
    if a.headless:
        args.append("--headless")
    if a.rotor_tau is not None:
        args += ["--rotor-tau", repr(a.rotor_tau)]
    if a.controller_gains:
        args += ["--gains"] + list(a.controller_gains)
    if a.lateral:
        args.append("--lateral")
    if not a.takeoff_first:
        args.append("--no-takeoff-first")

    # Not quiet, and not detached from stdin: level 1 prompts for whatever was not
    # passed, and its viewport output is the point of running it.
    log(f"level 1 - {LEVELS[1]}")
    proc = simlaunch.launch(simlaunch.SIM_PEGASUS, args, quiet=False)
    _LAUNCHED.append(proc)
    try:
        return proc.wait()
    finally:
        _stop_launched()


def _report_sim_failure(proc, sim_log: Path, tail: int = 25) -> None:
    """Print why the sim died, from its own log, rather than only that it did."""
    rc = proc.poll()
    log(f"--- sim exited with code {rc} ---" if rc is not None
        else "--- sim still running; it never became flyable ---")
    try:
        lines = sim_log.read_text(errors="replace").splitlines()
    except OSError as exc:
        log(f"could not read {sim_log}: {exc}")
        return
    if not lines:
        log(f"{sim_log} is empty - Isaac produced no output at all.")
        return
    log(f"last {min(tail, len(lines))} lines of {sim_log}:")
    for line in lines[-tail:]:
        log(f"  | {line}")


def fly_px4(a: argparse.Namespace) -> int:
    """Levels 2 and 3: put a PX4 sim up (or attach to one), then fly the profile."""
    ground_truth = (a.level == 2)
    log(f"level {a.level} - {LEVELS[a.level]}")

    # The control law goes in the directory name. Two laws flown back to back are
    # different experiments, and a name that records only the level would file
    # them together - which is how a cascaded_pd ulog ends up read as an eigen one.
    alg_tag = (offboard.CTRL_ALG_NAMES.get(a.ctrl_alg, str(a.ctrl_alg))
               if a.ctrl_alg is not None else "airframe")
    run_dir = Path(a.run_dir).expanduser().resolve() if a.run_dir else (
        simlaunch.REPO / "px4_runs" /
        f"L{a.level}_{alg_tag}_{datetime.now():%Y%m%d_%H%M%S}")

    sim_log = run_dir / "sim.log"
    proc = None
    if not a.attach:
        duration = a.duration if a.duration is not None else a.hold + 180.0
        log(f"launching sim -> {run_dir}")
        log(f"sim output    -> {sim_log}")
        proc = simlaunch.launch_px4_sim(
            run_dir, a.motor_moment, duration,
            extra=["--desaturation", a.desaturation],
            log_path=sim_log,
            headless=a.headless,
            on_port_busy=lambda port: log(
                f"WARNING: TCP {port} still held; PX4 will likely fail to bind and "
                f"report a bogus airframe mismatch"))
        _LAUNCHED.append(proc)
    else:
        if a.run_dir is None:
            sys.exit("--attach needs --run-dir: the ulog is read from the directory "
                     "the running sim was launched with.")
        log(f"attaching to a running sim, ulog expected under {run_dir}")

    # offboard.fly() reads its options off a namespace, so build the one it wants.
    opts = argparse.Namespace(
        run_dir=str(run_dir), altitude=a.altitude, hold=a.hold,
        boot_timeout=a.boot_timeout, ground_truth=ground_truth, gains=a.gains,
        abort_on_loss=a.abort_on_loss, no_land=a.no_land, ctrl_alg=a.ctrl_alg)
    try:
        return offboard.fly(opts)
    except SystemExit as exc:
        # offboard.fly() exits with a message when PX4 never shows up. That names
        # the symptom; the cause is almost always in the sim's own output, which
        # is on disk rather than on this terminal. Surface it here instead of
        # leaving the reader to guess which of Isaac, PX4 or the airframe failed.
        if proc is not None and sim_log.exists():
            _report_sim_failure(proc, sim_log)
        raise exc
    finally:
        if proc is not None:
            log("stopping sim")
            simlaunch.kill(proc)
        else:
            # An attached sim is the caller's to stop, but a PX4 orphaned by a
            # crash would still hold the lockstep port against the next run.
            time.sleep(0.5)


# Flags that only do something on one rung. Passing one to the other rung is
# silently ignored by argparse, which is how "--hold 90" on level 1 looks like a
# 90 second flight and produces an unbounded one.
_LEVEL1_ONLY = ("controller", "failure_mode", "rotor", "seed", "lateral",
                "rotor_tau", "controller_gains")
_LEVEL23_ONLY = ("hold", "altitude", "attach", "gains", "ctrl_alg", "run_dir",
                 "boot_timeout", "no_land", "abort_on_loss")


def _warn_wrong_level(a, argv=None) -> None:
    """Say so when a flag cannot apply to the level being flown.

    Reads argv rather than comparing against defaults: passing a flag its own
    default value is still passing it, and "--hold 90" - 90 being the default -
    is exactly the case worth catching.
    """
    argv = sys.argv[1:] if argv is None else argv
    typed = {tok.split("=", 1)[0] for tok in argv if tok.startswith("--")}
    wrong = _LEVEL23_ONLY if a.level == 1 else _LEVEL1_ONLY
    for dest in wrong:
        flag = "--" + dest.replace("_", "-")
        if flag not in typed:
            continue
        other = "levels 2 and 3" if a.level == 1 else "level 1"
        log(f"WARNING: {flag} applies to {other}, not level {a.level} - ignored."
            + ("  Use --duration to set the flight length."
               if dest == "hold" and a.level == 1 else ""))


def main() -> int:
    a = parse_args()
    _warn_wrong_level(a)
    _install_cleanup()
    if not simlaunch.ISAAC_PY.exists() and not a.attach:
        sys.exit(f"{simlaunch.ISAAC_PY} not found - set ISAACSIM_PATH")
    return fly_level_1(a) if a.level == 1 else fly_px4(a)


if __name__ == "__main__":
    sys.exit(main())
