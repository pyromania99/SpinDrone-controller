#!/usr/bin/env python3
"""
select_reference.py — pick the operating point the ablation marches out from.

The ablation explorer holds every gene but one at a reference configuration and
steps that one outward until flight breaks. Everything it measures is relative
to that point, so choosing it badly makes every response curve a curve through
somewhere uninteresting.

Why the knee, not the best score
────────────────────────────────
Measured over the archived corpus, the middle 50 % of stable configurations
spans about 7 % of the median score, and repeated evaluations of an identical
gene tuple differ by ~1.5 %. Picking the single lowest score from a few hundred
individuals in a band that flat is largely picking the luckiest run — the
reference would move between reruns of the same search.

The knee is stable against that. Both objectives are normalised over the stable
Pareto front in log space and the point nearest the utopia corner is taken, so
the reference is the best accuracy/effort compromise rather than an extreme.
``--reference-rule min-score`` and ``min-effort`` are available when an extreme
is what you actually want.

Plain python — numpy and the stdlib only, no Isaac Sim.

Usage
─────
    python3 select_reference.py                       # every controller x mm
    python3 select_reference.py --root results/outer --only eigen
    python3 select_reference.py --reference-rule min-score --dry-run
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "utils"))
import sweep_task as task  # noqa: E402
import gene_space  # noqa: E402
import sweep_metrics as sm  # noqa: E402

RESULTS = task.results_root()

SCORE_KEY = "score"
EFFORT_KEY = "effort_ac_ses"


def fnum(raw):
    """Parse a CSV cell to float; non-numeric and non-finite both become nan."""
    if raw is None or raw == "":
        return math.nan
    try:
        value = float(raw)
    except ValueError:
        return math.nan
    return value if math.isfinite(value) else math.nan


def load_stable(path: Path, genes_def) -> list:
    """Stable rows from one archive, deduped on the gene tuple.

    Dedup matters: the genetic search re-evaluates its elites every generation,
    so a winning individual appears dozens of times and would dominate any
    front computed over the raw rows.
    """
    names = [g[0] for g in genes_def]
    best = {}
    with open(path, newline="") as f:
        for raw in csv.DictReader(f):
            score, effort = fnum(raw.get(SCORE_KEY)), fnum(raw.get(EFFORT_KEY))
            if not (math.isfinite(score) and math.isfinite(effort)):
                continue
            if score > sm.STABLE_SCORE_MAX or fnum(raw.get("stable")) != 1.0:
                continue
            genes = {n: fnum(raw.get(n)) for n in names}
            if any(not math.isfinite(v) for v in genes.values()):
                continue
            key = tuple(round(genes[n], 6) for n in names)
            row = {"genes": genes, "score": score, "effort": effort,
                   "metrics": {k: fnum(raw.get(k)) for k in sm.METRIC_KEYS
                               if k not in sm.STR_METRIC_KEYS},
                   "config": {k: fnum(raw.get(k)) for k in sm.CONFIG_KEYS},
                   "motor_moment": fnum(raw.get("motor_moment"))}
            if key not in best or score < best[key]["score"]:
                best[key] = row
    return list(best.values())


def pareto_front(rows: list) -> list:
    """Non-dominated set in (score, effort), both minimised."""
    front = []
    for candidate in rows:
        dominated = any(
            other["score"] <= candidate["score"]
            and other["effort"] <= candidate["effort"]
            and (other["score"] < candidate["score"]
                 or other["effort"] < candidate["effort"])
            for other in rows)
        if not dominated:
            front.append(candidate)
    return sorted(front, key=lambda r: r["score"])


def pick(front: list, rule: str) -> dict:
    """The reference point, by the requested rule."""
    if not front:
        raise ValueError("empty Pareto front")
    if rule == "min-score":
        return min(front, key=lambda r: r["score"])
    if rule == "min-effort":
        return min(front, key=lambda r: r["effort"])

    # knee: normalise both objectives over the front in log space, then take
    # the point closest to the utopia corner. Log space because effort spans
    # orders of magnitude across a front while score spans a few percent — in
    # linear space the effort axis would decide the answer by itself.
    s = np.log10(np.array([r["score"] for r in front]))
    e = np.log10(np.array([max(r["effort"], 1e-9) for r in front]))
    def unit(v):
        span = v.max() - v.min()
        return np.zeros_like(v) if span < 1e-12 else (v - v.min()) / span
    d = np.hypot(unit(s), unit(e))
    return front[int(np.argmin(d))]


def process(archive: Path, controller: str, rule: str) -> dict:
    genes_def = gene_space.genes_for(controller, sweep_outer=True)
    rows = load_stable(archive, genes_def)
    if not rows:
        # Fall back to the inner-only gene set, so an archive written before
        # --sweep-outer existed still yields a reference.
        genes_def = gene_space.genes_for(controller, sweep_outer=False)
        rows = load_stable(archive, genes_def)
    if not rows:
        raise ValueError(f"no stable rows in {archive}")

    front = pareto_front(rows)
    chosen = pick(front, rule)

    # Physical outer gains, derived from whatever coordinates the search used.
    # Downstream consumers (robustness_sweep's key check in particular) know
    # kp_xy/kd_xy and reject wn_xy/zeta_xy, so both are recorded.
    gains = dict(chosen["genes"])
    gains.update(gene_space.outer_gains(chosen["genes"]))

    return {
        "controller": controller,
        "motor_moment": chosen["motor_moment"],
        "rule": rule,
        "source": str(archive),
        "genes": chosen["genes"],
        "gene_box": {n: [lo, hi] for n, lo, hi in genes_def},
        "gains": gains,
        "metrics": chosen["metrics"],
        "config": chosen["config"],
        "pool_size": len(rows),
        "front_size": len(front),
        "front": [{"genes": r["genes"], "score": r["score"],
                   "effort": r["effort"]} for r in front],
    }


def at_wall(entry: dict) -> list:
    """Genes sitting on a box bound — the reference is not interior there.

    Worth surfacing loudly. Every free-outer winner measured so far pins at
    least one gene to a bound (wn_xy at its 2.0 ceiling for all three
    controllers), which means the search box, not the physics, chose that
    value. An ablation arm starting at a wall can only march inward on one
    side unless it is allowed past the box — see gene_space.ladder_values.
    """
    walls = []
    for name, value in entry["genes"].items():
        lo, hi = entry["gene_box"][name]
        if abs(value - lo) <= 1e-6 * max(1.0, abs(lo)):
            walls.append(f"{name}=lo({lo:g})")
        elif abs(value - hi) <= 1e-6 * max(1.0, abs(hi)):
            walls.append(f"{name}=hi({hi:g})")
    return walls


def main() -> int:
    p = argparse.ArgumentParser(description="Pick the ablation reference point")
    p.add_argument("--root", type=Path, default=RESULTS / "outer",
                   help="Tree holding genetic_<ctrl>_sweep/mm_*/genetic_archive.csv")
    p.add_argument("--only", type=str, default="",
                   help="Comma-separated controllers (default: all found)")
    p.add_argument("--reference-rule", choices=("knee", "min-score", "min-effort"),
                   default="knee")
    p.add_argument("--out-root", type=Path, default=RESULTS / "ablation")
    p.add_argument("--dry-run", action="store_true")
    args = p.parse_args()

    only = {c.strip() for c in args.only.split(",") if c.strip()}
    archives = sorted(Path(args.root).glob("genetic_*_sweep/mm_*/genetic_archive.csv"))
    if not archives:
        print(f"no genetic_archive.csv under {args.root}", file=sys.stderr)
        return 1

    written = 0
    for archive in archives:
        controller = archive.parts[-3].replace("genetic_", "").replace("_sweep", "")
        if only and controller not in only:
            continue
        try:
            entry = process(archive, controller, args.reference_rule)
        except (ValueError, KeyError) as exc:
            print(f"  {controller:<6} {archive.parts[-2]:<12} SKIPPED: {exc}")
            continue

        mm = entry["motor_moment"]
        walls = at_wall(entry)
        print(f"  {controller:<6} {archive.parts[-2]:<12} "
              f"pool {entry['pool_size']:>4}  front {entry['front_size']:>3}  "
              f"score {entry['metrics']['score']:.4f}  "
              f"effort {entry['metrics'][EFFORT_KEY]:.4f}")
        print("         genes " + "  ".join(
            f"{k}={v:g}" for k, v in entry["genes"].items()))
        if walls:
            print(f"         AT BOX BOUND: {', '.join(walls)} — the box chose "
                  f"these, not the physics; the ablation must march past it")

        if not args.dry_run:
            out = Path(args.out_root) / controller / f"mm_{mm:.1e}"
            out.mkdir(parents=True, exist_ok=True)
            (out / "reference.json").write_text(json.dumps(entry, indent=2))
            written += 1

    if not args.dry_run:
        print(f"\n  wrote {written} reference.json under {args.out_root}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
