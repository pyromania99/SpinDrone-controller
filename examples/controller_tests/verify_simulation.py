"""
| File: verify_simulation.py
| License: BSD-3-Clause. Copyright (c) 2026. All rights reserved.
| Description: End-to-end verification that the simulation runs and produces sane results.

The offline physics suite checks the model's mathematics without Isaac Sim. This checks
the other half: that every configuration actually SPAWNS, STEPS and SCORES inside the
simulator, which is where integration mistakes live -- a mis-wired flag, a missing dt, a
prim that never gets the wing's mass, a NaN reaching PhysX.

Run it after touching anything the simulator loads:

    python3 examples/controller_tests/verify_simulation.py
    python3 examples/controller_tests/verify_simulation.py --quick

Each case asserts three things: the run exits cleanly, it emits a finite score, and that
score falls in a physically plausible band. A run that completes but scores infinity has
diverged, which a plain exit code would not catch.
"""

from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
import time
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SWEEPS = "examples/controller_tests/sweeps"

SCORE_RE = re.compile(r"score\s*=\s*([0-9.eE+-]+)")
EFFORT_RE = re.compile(r"effort\s*=\s*([0-9.eE+-]+)")


@dataclass
class Case:
    """One simulator configuration to exercise."""

    name: str
    script: str
    args: str
    score_range: Tuple[float, float] = (0.0, 5.0)
    expect_log: Tuple[str, ...] = ()
    """Substrings that MUST appear in the output, e.g. the wing's mass being applied."""
    quick: bool = True


def build_cases(duration: float) -> List[Case]:
    d = f"--duration {duration} --headless"
    eig = f"{SWEEPS}/eigen_stability_sweep.py"
    common = f"--motor-moments 6e-7 --omega-n-values 2 --zeta 1.0 {d}"

    return [
        # -- baseline: the stock quadrotor path must still work and stay bounded --------
        Case("baseline no-wing", eig, f"{common} --no-wing"),
        Case("baseline, gyroscopics ablated", eig, f"{common} --no-wing --no-rotor-gyro"),

        # -- the wing, and that its structure really reaches the rigid body -------------
        Case("wing, default (bemt)", eig, f"{common} --wing",
             expect_log=("wing structure added",)),
        Case("wing, inertia ablated", eig, f"{common} --wing --wing-no-inertia"),

        # -- every inflow model ---------------------------------------------------------
        Case("inflow none", eig, f"{common} --wing --wing-inflow none"),
        Case("inflow glauert", eig, f"{common} --wing --wing-inflow glauert"),
        Case("inflow dynamic", eig, f"{common} --wing --wing-inflow dynamic"),

        # -- geometry and trim knobs ------------------------------------------------------
        Case("4 blades", eig, f"{common} --wing --wing-blades 4"),
        Case("collective +2 deg", eig, f"{common} --wing --wing-collective-deg 2.0"),
        Case("retargeted span", eig,
             f"{common} --wing --wing-root-radius 0.15 --wing-tip-radius 0.40"),
        Case("48 stations", eig, f"{common} --wing --wing-stations 48"),
        Case("wrong handedness (ablation)", eig,
             f"{common} --wing --wing-rotation-sense 1"),

        # -- several vehicles in one scene, which is how the sweeps parallelise ----------
        Case("six vehicles, no wing", eig,
             f"--motor-moments 6e-7 --omega-n-values 2 4 6 8 10 12 --zeta 1.0 {d} "
             f"--no-wing", quick=False),
        Case("six vehicles, wing", eig,
             f"--motor-moments 6e-7 --omega-n-values 2 4 6 8 10 12 --zeta 1.0 {d} "
             f"--wing", quick=False),

        # -- the other two sweep scripts ---------------------------------------------------
        Case("pd sweep, no wing", f"{SWEEPS}/pd_sweep.py", f"{d} --no-wing",
             score_range=(0.0, 20.0)),
        Case("pd sweep, wing", f"{SWEEPS}/pd_sweep.py", f"{d} --wing",
             score_range=(0.0, 20.0)),
        Case("mpc sweep, no wing", f"{SWEEPS}/mpc_sweep.py",
             f"{d} --q-values 10 --r-values 0.02 --no-wing", score_range=(0.0, 50.0)),
        Case("mpc sweep, wing", f"{SWEEPS}/mpc_sweep.py",
             f"{d} --q-values 10 --r-values 0.02 --wing", score_range=(0.0, 50.0),
             quick=False),
    ]


def run_case(case: Case, timeout: int = 900) -> dict:
    """Execute one case through ``isaac_run`` and judge the result."""
    cmd = (
        f"source ~/.isaac_env.sh && isaac_run {case.script} {case.args}"
    )
    t0 = time.time()
    proc = subprocess.run(
        ["bash", "-lc", cmd], cwd=REPO, capture_output=True, text=True, timeout=timeout
    )
    out = proc.stdout + proc.stderr
    elapsed = time.time() - t0

    scores = [float(m) for m in SCORE_RE.findall(out)]
    efforts = [float(m) for m in EFFORT_RE.findall(out)]
    missing = [s for s in case.expect_log if s not in out]

    problems: List[str] = []
    if proc.returncode != 0:
        problems.append(f"exit {proc.returncode}")
    if not scores:
        problems.append("no score emitted")
    else:
        for s in scores:
            if not (s == s) or s in (float("inf"), float("-inf")):
                problems.append(f"non-finite score {s}")
            elif not case.score_range[0] <= s <= case.score_range[1]:
                problems.append(
                    f"score {s:.4f} outside plausible band {case.score_range}"
                )
    if missing:
        problems.append(f"missing from log: {missing}")
    if "Traceback" in out:
        problems.append("python traceback")
    if re.search(r"\bnan\b", out, re.I) and "nan" not in case.name.lower():
        problems.append("NaN mentioned in output")

    return {
        "name": case.name,
        "ok": not problems,
        "problems": problems,
        "scores": scores,
        "efforts": efforts,
        "seconds": round(elapsed, 1),
        "tail": out[-1500:] if problems else "",
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--quick", action="store_true",
                    help="Skip the slow multi-vehicle cases.")
    ap.add_argument("--duration", type=float, default=5.0,
                    help="Simulated seconds per case.")
    ap.add_argument("--timeout", type=int, default=900)
    args = ap.parse_args()

    cases = [c for c in build_cases(args.duration) if c.quick or not args.quick]
    print(f"Verifying {len(cases)} simulator configurations "
          f"({args.duration:g} s each)\n")

    results = []
    for i, case in enumerate(cases, 1):
        print(f"[{i:2d}/{len(cases)}] {case.name:34s}", end="", flush=True)
        try:
            r = run_case(case, args.timeout)
        except subprocess.TimeoutExpired:
            r = {"name": case.name, "ok": False, "problems": ["timeout"],
                 "scores": [], "efforts": [], "seconds": args.timeout, "tail": ""}
        results.append(r)
        mark = "ok  " if r["ok"] else "FAIL"
        score = f"{r['scores'][0]:.4f}" if r["scores"] else "-"
        print(f" {mark}  score {score:>9}  {r['seconds']:5.1f}s")
        if not r["ok"]:
            for p in r["problems"]:
                print(f"         ! {p}")

    failed = [r for r in results if not r["ok"]]
    print(f"\n{len(results) - len(failed)}/{len(results)} configurations passed")
    if failed:
        print("\nFailures:")
        for r in failed:
            print(f"\n--- {r['name']} ---\n{r['tail'][-700:]}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
