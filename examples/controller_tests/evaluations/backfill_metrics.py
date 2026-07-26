#!/usr/bin/env python3
"""
backfill_metrics.py — recompute every metric from .npz telemetry already on disk.

Why
───
The sweeps now record an effort DC/AC split and the vehicle's spin rate (see
examples/utils/sweep_metrics.py), but the archives written before that change
carry only ``score`` and the legacy total-thrust ``effort``. The raw telemetry
those archives were derived from is still there, so the new columns can be
recovered without re-running a single simulation.

This is a plain-python tool: it imports numpy and sweep_metrics only, so it
runs in any interpreter — no Isaac Sim, no ``isaac_run``.

Usage
─────
    python3 examples/controller_tests/evaluations/backfill_metrics.py
    python3 .../backfill_metrics.py --root results/eigen_sweep/ab_sweep
    python3 .../backfill_metrics.py --duration 25.0 --out /tmp/backfill.csv

Output
──────
One long-format CSV row per .npz, holding every key in
sweep_metrics.METRIC_KEYS plus whatever run parameters could be recovered from
the file path. Parameter recovery is best-effort and layout-driven — the sweeps
encode gains in directory and file names, e.g.::

    eigen_sweep/ab_sweep/mm_4.0e-07/b_10.000/a12.500_statistics.npz
                         ^ km        ^ b_mdyn ^ wn_mdyn

Anything that cannot be parsed is left blank rather than guessed.
"""
from __future__ import annotations

import argparse
import csv
import re
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "utils"))
import sweep_metrics as sm  # noqa: E402

# Arena box used by every sweep's --bounds default. The bounds diagonal only
# affects the OOB padding value, so it must match what the sweeps scored with
# for backfilled numbers to be comparable to the archived ones.
DEFAULT_BOUNDS = [-12.0, 12.0, -12.0, 12.0, 0.0, 20.0]

# Run parameters recoverable from path components, in the naming conventions
# the sweep scripts actually use. Each entry: (column, compiled pattern).
# Directory patterns are matched against each parent directory name, file
# patterns against the .npz filename.
_DIR_PATTERNS = [
    ("motor_moment", re.compile(r"^mm_([-+0-9.eE]+)$")),
    ("b_mdyn",       re.compile(r"^b_([-+0-9.eE]+)$")),
    ("Kd_att",       re.compile(r"^kd_([-+0-9.eE]+)$")),
    ("zeta",         re.compile(r"^zeta_([-+0-9.eE]+)$")),
]
_FILE_PATTERNS = [
    ("wn_mdyn", re.compile(r"^a([-+0-9.eE]+)_statistics")),
    ("Kp_att",  re.compile(r"^kp_att_([-+0-9.eE]+)_statistics")),
    ("Kp_att",  re.compile(r"^kp_?([-+0-9.eE]+)_statistics")),
    ("alpha",   re.compile(r"^alpha([-+0-9.eE]+)_statistics")),
    ("wn_mdyn", re.compile(r"^wn([-+0-9.eE]+)_statistics")),
    ("mpc_q",   re.compile(r"^q([-+0-9.eE]+)_statistics")),
]

PARAM_COLUMNS = ["motor_moment", "wn_mdyn", "b_mdyn", "Kp_att", "Kd_att",
                 "zeta", "alpha", "mpc_q"]


def _as_float(text: str):
    try:
        return float(text)
    except ValueError:
        return None


def parse_params(npz_path: Path, root: Path) -> dict:
    """Best-effort recovery of run parameters from the file's location."""
    params = {}
    try:
        rel = npz_path.relative_to(root)
    except ValueError:
        rel = npz_path

    for part in rel.parts[:-1]:
        for column, pattern in _DIR_PATTERNS:
            match = pattern.match(part)
            if match:
                value = _as_float(match.group(1))
                if value is not None:
                    params[column] = value

    stem = rel.parts[-1]
    for column, pattern in _FILE_PATTERNS:
        if column in params:
            continue
        match = pattern.match(stem)
        if match:
            value = _as_float(match.group(1))
            if value is not None:
                params[column] = value
    return params


def logged_duration(npz_path: Path) -> float:
    """Wall duration actually present in the file, independent of --duration."""
    try:
        d = np.load(str(npz_path))
        if "time" not in d.files:
            return float("nan")
        t = np.asarray(d["time"], dtype=float)
        return float(t[-1]) if t.size else float("nan")
    except Exception:
        return float("nan")


def main() -> int:
    p = argparse.ArgumentParser(
        description="Recompute sweep metrics from existing .npz telemetry")
    default_root = Path(__file__).resolve().parents[1] / "results"
    p.add_argument("--root", type=Path, default=default_root,
                   help="Directory to walk for .npz files (default: results/)")
    p.add_argument("--out", type=Path, default=None,
                   help="Output CSV (default: <root>/backfilled_metrics.csv)")
    p.add_argument("--duration", type=float, default=25.0,
                   help="Intended run duration T used for OOB padding. Must "
                        "match the sweep's --duration for the backfilled "
                        "numbers to line up with the archived ones.")
    p.add_argument("--bounds", nargs=6, type=float, default=DEFAULT_BOUNDS,
                   metavar=("XMIN", "XMAX", "YMIN", "YMAX", "ZMIN", "ZMAX"))
    p.add_argument("--include-archived", action="store_true",
                   help="Also scan _archive*/ and results_pre_*/ paths. Those "
                        "hold telemetry from before the applied-thrust fix, "
                        "whose effort columns are NOT comparable with anything "
                        "measured after it — pass this only when deliberately "
                        "analysing an archive on its own, never when building "
                        "a CSV that also covers current runs.")
    args = p.parse_args()

    root = args.root.resolve()
    if not root.is_dir():
        print(f"error: --root is not a directory: {root}", file=sys.stderr)
        return 1

    out_path = args.out or (root / "backfilled_metrics.csv")
    bounds_diag = sm.bounds_diagonal(args.bounds)

    npz_files = sorted(root.rglob("*.npz"))

    # Never mix archived runs into a fresh backfill. Telemetry recorded before
    # the applied-thrust fix holds a *reconstruction* of per-rotor force rather
    # than what the simulator applied, so its effort columns are not comparable
    # with anything measured after — and nothing in the CSV would reveal that.
    skipped_archived = [] if args.include_archived else [
        p for p in npz_files if any(
            part.startswith("_archive") or part.startswith("results_pre_")
            for part in p.parts)]
    npz_files = [p for p in npz_files if p not in set(skipped_archived)]
    if skipped_archived:
        print(f"  skipping {len(skipped_archived)} .npz under archived paths "
              f"(_archive*/ or results_pre_*/) — effort there is not comparable")

    # Partially-written scratch files from an interrupted sweep.
    tmp_files = [p for p in npz_files if "_tmp_" in p.name]
    npz_files = [p for p in npz_files if p not in set(tmp_files)]
    if tmp_files:
        print(f"  skipping {len(tmp_files)} scratch *_tmp_*.npz files")

    if not npz_files:
        print(f"error: no .npz files found under {root}", file=sys.stderr)
        return 1

    print(f"scanning {len(npz_files)} .npz files under {root}")
    print(f"  T = {args.duration}s   bounds diagonal = {bounds_diag:.3f} m")

    rows = []
    n_scored = 0
    n_with_spin = 0
    for npz_path in npz_files:
        metrics = sm.compute_metrics(str(npz_path), args.duration, bounds_diag)
        params = parse_params(npz_path, root)
        if np.isfinite(metrics["score"]):
            n_scored += 1
        if np.isfinite(metrics["r_ss"]):
            n_with_spin += 1
        rows.append({
            "path": str(npz_path.relative_to(root)),
            "logged_duration": logged_duration(npz_path),
            **{c: params.get(c, "") for c in PARAM_COLUMNS},
            **{k: sm.format_metric(k, metrics[k]) for k in sm.METRIC_KEYS},
        })

    header = (["path", "logged_duration"] + PARAM_COLUMNS + sm.METRIC_KEYS)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=header)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)

    print(f"\nwrote {len(rows)} rows → {out_path}")
    print(f"  {n_scored}/{len(rows)} produced a finite stability score")
    print(f"  {n_with_spin}/{len(rows)} carried a yaw-rate trace (spin metrics)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
