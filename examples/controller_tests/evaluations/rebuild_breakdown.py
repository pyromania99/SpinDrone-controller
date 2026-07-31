#!/usr/bin/env python3
"""
rebuild_breakdown.py — recompute breakdown.csv from a robustness archive.

The archive holds every run; breakdown.csv is only a reduction of it. When the
reduction changes — a new failure criterion, a different grouping — nothing
needs re-flying, which is the whole reason the long-format archive exists.

Plain python: imports numpy and examples/utils/robustness.py only, so it runs
in any interpreter without Isaac Sim.

Grouping
────────
Archives written before the driver assigned a unique ``config_id`` are grouped
by their gain columns instead. ``config_rank`` cannot be used: pareto_top.json
numbers ranks within each (controller, motor_moment) set, and the driver pools
across motor moments, so two different gain sets routinely share a rank. On the
eigen inertia archive, rank 3 covered both (wn_mdyn=6.81, b_mdyn=0.31) and
(wn_mdyn=60.0, b_mdyn=2.52), and the original breakdown merged them into a
single row reporting n_runs=16 for what were two 8-run configurations.

Usage
─────
    python3 rebuild_breakdown.py                       # every archive found
    python3 rebuild_breakdown.py --root results/robustness/pd
    python3 rebuild_breakdown.py --dry-run
"""
from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "utils"))
import robustness as rb  # noqa: E402

RESULTS = Path(__file__).resolve().parents[1] / "results"

# Gain columns that, together, identify a configuration. Only those present in
# a given archive are used.
GAIN_COLUMNS = ["wn_mdyn", "b_mdyn", "Kp_att", "Kd_att", "Kd_yaw_rate",
                "mpc_q", "mpc_r"]

# Unperturbed magnitude per axis: the scale axes are matched at 1.0, the
# additive ones at 0.
ORIGINS = {"inertia": 1.0, "mass": 1.0, "lag": 0.0}


def origin_for(axis: str) -> float:
    return ORIGINS.get(axis.split("-")[0], 0.0)


def group_fields(fieldnames) -> tuple:
    """Columns identifying one configuration in this archive."""
    if "config_id" in fieldnames:
        return ("controller", "axis", "motor_moment", "config_id")
    present = [c for c in GAIN_COLUMNS if c in fieldnames]
    return ("controller", "axis", "motor_moment", *present)


def rebuild(archive: Path, dry_run: bool = False) -> dict:
    with open(archive, newline="") as f:
        reader = csv.DictReader(f)
        fields = reader.fieldnames or []
        rows = list(reader)
    if not rows:
        return {"path": archive, "rows": 0}

    axes = {r["axis"] for r in rows}
    if len(axes) != 1:
        raise ValueError(f"{archive} mixes axes {sorted(axes)}; "
                         f"the origin is per-axis so they cannot share a file")
    axis = axes.pop()
    keys = group_fields(fields)

    out = rb.breakdown_rows(rows, origin=origin_for(axis), key_fields=keys)

    # config_id is absent from older archives; number the groups so each row
    # still has a stable handle, and keep the (ambiguous) rank for provenance.
    for i, row in enumerate(out):
        if row["config_id"] in ("", None):
            row["config_id"] = i

    path = archive.parent / "breakdown.csv"
    if not dry_run:
        with open(path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(rb.BREAKDOWN_COLUMNS)
            for row in out:
                writer.writerow([_fmt(k, row[k]) for k in rb.BREAKDOWN_COLUMNS])
    return {"path": path, "rows": len(rows), "groups": len(out),
            "grouped_by": keys, "breakdowns": sum(1 for r in out if r["failure_mode"])}


def _fmt(key: str, value):
    """One cell. motor_moment never goes through a fixed-point format — '%.6f'
    renders every km below ~5e-7 as '0.000000', merging four of six spin rates
    into the km=0 bucket (see examples/utils/gain_seed.py:234)."""
    if value is None:
        return ""
    if key == "motor_moment":
        try:
            return repr(float(value))
        except (TypeError, ValueError):
            return str(value)
    if key.startswith(("below_", "above_")):
        return f"{float(value):g}"
    if isinstance(value, float):
        if not np.isfinite(value):
            return "nan" if np.isnan(value) else "inf"
        return f"{value:.6f}"
    return value


def main() -> int:
    p = argparse.ArgumentParser(description="Recompute robustness breakdowns")
    p.add_argument("--root", type=Path, default=RESULTS / "robustness")
    p.add_argument("--dry-run", action="store_true")
    args = p.parse_args()

    archives = sorted(Path(args.root).rglob("robustness_archive.csv"))
    if not archives:
        print(f"no robustness_archive.csv under {args.root}")
        return 1

    for archive in archives:
        info = rebuild(archive, dry_run=args.dry_run)
        rel = info["path"].relative_to(Path(args.root).parent)
        print(f"  {str(rel):<46s} {info['rows']:>4} runs → "
              f"{info['groups']:>2} configs, {info['breakdowns']} with a breakdown")
        print(f"      grouped by: {', '.join(info['grouped_by'])}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
