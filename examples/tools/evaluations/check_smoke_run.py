#!/usr/bin/env python3
"""
check_smoke_run.py — assert the physical invariants on freshly written telemetry.

Prints a space-separated list of problem tags (empty when clean) so a shell
smoke harness can fold the result into its own pass/fail. Exit status is always
0; the output is the signal.

Checks, each corresponding to a bug that actually shipped:

  short-run      logged flight time missed --duration by more than the
                 tolerance. `--duration 25` silently delivered 6 s for an
                 entire suite, because the loops advanced a hand-rolled clock
                 at 1/60 s per world.step() while physics runs at 1/250 s.

  thrust-over    a per-rotor thrust exceeded the physical ceiling
                 (rotor_constant x max_rotor_velocity^2 = 10.34 N). Telemetry
                 used to hold a mixer *reconstruction* rather than the applied
                 force, with values as large as 3.6e5 N.

  air-start      the run began well off the floor when a ground start was
                 asked for, i.e. --spawn-altitude did not take effect. This
                 check used to be its inverse ("ground-start"), from when
                 sweeps spawned at the hover target; that task made initial
                 position error zero and left the score unable to distinguish
                 attitude gains at all. Runs now take off. Pass
                 --spawn-altitude to check an air-start run instead, and the
                 sense flips back.

  no-climb       NOT ONE run reached takeoff altitude. Individual failures to
                 take off are a legitimate result — smoke runs deliberately
                 include gains that cannot fly — so this fires only when every
                 run in the batch stayed down, which means the rig is broken
                 rather than the gains.

  no-telemetry   nothing was written at all.

The altitude checks read p[0] directly. That used to be unsafe: the opening
sample held the pose from before the reset was applied, so this file skipped
to t = 0.1 s to avoid reading it. The cause was `received_first_state` being
set once in the controller's __init__ and never cleared between passes; with
that fixed the first logged row is genuinely the first flown step, and for a
ground start it is the only place the spawn altitude is still visible — by
t = 0.1 s a healthy vehicle has already climbed off the floor.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "utils"))
from applied_thrust import max_rotor_force  # noqa: E402
import sweep_task as task  # noqa: E402

RESULTS = task.results_root()


def check_archives(args, now: float) -> int:
    """Validate a sweep that left only an archive CSV (the genetic searches).

    peak_effort is max_t Σᵢ uᵢ(t)². With the applied rotor forces each bounded
    by rotor_constant × max_rotor_velocity² = 10.34 N, it cannot legitimately
    exceed 4 × 10.34² = 428 N². The pre-fix telemetry held a mixer
    reconstruction instead and reached 3.6e5 N per rotor, so this single
    column is enough to tell the two apart without any npz.
    """
    import csv

    archives = [f for f in args.root.rglob("*archive*.csv")
                if now - f.stat().st_mtime <= args.max_age]
    if not archives:
        print("no-telemetry")
        return 0

    e_sat = 4.0 * max_rotor_force() ** 2
    rows = over = 0
    for path in archives:
        try:
            with open(path, newline="") as fh:
                for row in csv.DictReader(fh):
                    rows += 1
                    raw = row.get("peak_effort")
                    if raw in (None, "", "inf"):
                        continue
                    try:
                        if float(raw) > e_sat * 1.001:
                            over += 1
                    except ValueError:
                        continue
        except Exception:
            continue

    problems = []
    if rows == 0:
        problems.append("empty-archive")
    if over:
        problems.append(f"thrust-over({over}/{rows})")
    print(" ".join(problems))
    return 0


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--duration", type=float, required=True)
    p.add_argument("--tol", type=float, default=0.10,
                   help="allowed fractional deviation in flight time")
    p.add_argument("--max-age", type=float, default=900.0,
                   help="only inspect telemetry written in the last N seconds")
    p.add_argument("--root", type=Path, default=RESULTS)
    p.add_argument("--spawn-altitude", type=float, default=None,
                   help="Altitude the runs were launched from. Defaults to the "
                        "sweeps' own default (a ground start), which is what "
                        "decides whether a low or a high start is the fault.")
    p.add_argument("--target-altitude", type=float, default=1.5,
                   help="Goal altitude the runs were given; sets the height "
                        "that counts as having left the ground.")
    args = p.parse_args()

    spawn_altitude = task.resolve_spawn_altitude(args.spawn_altitude)
    ground_start = spawn_altitude <= 2.0 * task.GROUND_ALTITUDE
    takeoff_altitude = task.TAKEOFF_ALTITUDE_FRAC * args.target_altitude
    # Halfway between the floor and the takeoff line: high enough that a ground
    # start is never mistaken for an air start, low enough that an air start at
    # the goal altitude always trips it.
    start_line = 0.5 * (task.GROUND_ALTITUDE + takeoff_altitude)

    now = time.time()
    files = [f for f in args.root.rglob("*.npz")
             if now - f.stat().st_mtime <= args.max_age]
    if not files:
        # The genetic sweeps delete their per-run npz once scored, so absence of
        # telemetry is normal for them rather than a failure. Fall back to the
        # archive CSV, which still carries enough to test the thrust ceiling.
        return check_archives(args, now)

    ceiling = max_rotor_force() * 1.001  # allow float slop
    problems = []
    short = over = wrong_start = climbed = with_pos = 0

    for f in files:
        try:
            d = np.load(str(f))
        except Exception:
            continue
        keys = set(d.files)

        if "time" in keys:
            t = np.asarray(d["time"], dtype=float)
            if t.size >= 2:
                # A crashed run legitimately stops early; only flag runs that
                # ended without a recorded crash/OOB.
                ended_early = ("oob_time" in keys
                               and np.isfinite(float(d["oob_time"])))
                if not ended_early and abs(t[-1] - args.duration) > args.tol * args.duration:
                    short += 1

        if "motor_thrusts" in keys:
            u = np.asarray(d["motor_thrusts"], dtype=float)
            if u.size and np.nanmax(u) > ceiling:
                over += 1

        if "p" in keys:
            pos = np.asarray(d["p"], dtype=float)
            if pos.ndim == 2 and pos.shape[0] > 0:
                with_pos += 1
                z0 = float(pos[0, 2])
                # Started where it was told to? A ground start that begins in
                # the air, or an air start that begins on the floor, both mean
                # --spawn-altitude never reached the vehicle.
                if ground_start and z0 > start_line:
                    wrong_start += 1
                elif not ground_start and z0 < start_line:
                    wrong_start += 1
                if float(np.nanmax(pos[:, 2])) >= takeoff_altitude:
                    climbed += 1

    if short:
        problems.append(f"short-run({short}/{len(files)})")
    if over:
        problems.append(f"thrust-over({over}/{len(files)})")
    if wrong_start:
        tag = "air-start" if ground_start else "ground-start"
        problems.append(f"{tag}({wrong_start}/{len(files)})")
    # Only when nothing flew at all — see the module docstring.
    if with_pos and climbed == 0:
        problems.append(f"no-climb(0/{with_pos})")

    print(" ".join(problems))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
