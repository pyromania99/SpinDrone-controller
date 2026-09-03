#!/usr/bin/env python3
"""
ga_to_candidates.py — turn a Pegasus GA archive into a gain_search.py candidate list.

This is the Stage 1 -> Stage 3 bridge in notes/NEXT_STEPS.md. The GA searches Python gene names
(wn_mdyn, b_mdyn, Kp_att, Kd_att); gain_search.py sets PX4 parameters. The mapping is
one-to-one, but two things have to happen on the way across and neither is automatic:

  1. CASCADE SEPARATION wn/ATT_P >= 3 is enforced here, not searched. GA rank 3 sat at
     1.45, the rate and attitude loops interacted, and it departed in 12.9 s. The GA has no
     term for this, so it will happily return violating individuals.

  2. Kd_att DOES NOT TRANSFER. Every GA front pins it near its lower bound because the
     prototype differentiates an EXACT attitude with no transport lag. PX4 differentiates
     an ESTIMATE across ~12 ms, where the same gain drives the rate-setpoint feedback loop
     that FINDINGS section 8 documents (D/P = 4.3, 45.7 rad/s limit cycle from takeoff).
     So --att-d overrides it, defaulting to the airframe's 0.1 rather than the GA's number.

     If you want the GA's Kd_att anyway, pass --att-d -1 and it is carried through. That
     is a legitimate experiment - it just is not the default, because the one time it was
     taken at face value it cost a flight.

Usage
-----
  ga_to_candidates.py <genetic_archive.csv> -n 5 -o top5.json
  ga_to_candidates.py <archive.csv> -n 5 --min-gen 20 --att-d 0.1
  ga_to_candidates.py <archive.csv> --att-d -1        # trust the GA's Kd_att

Then:
  gain_search.py --candidates top5.json --ground-truth --motor-moment 2.7e-7
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path

GENE_TO_PX4 = {
    "wn_mdyn": "MC_EIG_WN",
    "b_mdyn": "MC_EIG_B",
    "Kp_att": "MC_EIG_ATT_P",
    "Kd_att": "MC_EIG_ATT_D",
}


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("archive", help="genetic_archive.csv from a GA run")
    p.add_argument("-n", "--top", type=int, default=5)
    p.add_argument("-o", "--out", default="candidates.json")
    p.add_argument("--min-sep", type=float, default=3.0,
                   help="Minimum wn/ATT_P. 0 disables the filter.")
    p.add_argument("--min-gen", type=int, default=0,
                   help="Ignore generations below this. Gen 0 is the raw LHS population "
                        "and is not a search result - its genes are effectively random.")
    p.add_argument("--att-d", type=float, default=0.1,
                   help="MC_EIG_ATT_D to write. -1 carries the GA's Kd_att through, which "
                        "is almost always wrong for PX4 - see the module docstring.")
    p.add_argument("--require-stable", action="store_true",
                   help="Keep only rows with stable=1. Off by default: the GA's stability "
                        "flag is a hover criterion, and rows that fail it can still be "
                        "worth flying in PX4. Turn it on to be strict.")
    p.add_argument("--dedupe-tol", type=float, default=0.02,
                   help="Relative tolerance for treating two gene vectors as the same "
                        "point. A converged GA returns many near-clones and they would "
                        "otherwise consume the whole candidate budget on one design.")
    return p.parse_args()


def main() -> int:
    a = parse_args()
    path = Path(a.archive)
    if not path.is_file():
        sys.exit(f"not found: {path}")

    with path.open() as fh:
        rows = list(csv.DictReader(fh))
    if not rows:
        sys.exit(f"{path} has no rows")

    missing = [g for g in GENE_TO_PX4 if g not in rows[0]]
    if missing:
        sys.exit(f"{path} is missing gene columns {missing} - is this an eigen archive?")

    def num(row, key, default=float("nan")):
        try:
            return float(row[key])
        except (KeyError, TypeError, ValueError):
            return default

    kept, drop_gen, drop_sep, drop_score, drop_stable = [], 0, 0, 0, 0
    for r in rows:
        if int(num(r, "gen", 0)) < a.min_gen:
            drop_gen += 1
            continue
        score = num(r, "score")
        if not math.isfinite(score):
            drop_score += 1
            continue
        if a.require_stable and num(r, "stable", 0) < 0.5:
            drop_stable += 1
            continue
        wn, att_p = num(r, "wn_mdyn"), num(r, "Kp_att")
        if a.min_sep > 0 and (att_p <= 0 or wn / att_p < a.min_sep):
            drop_sep += 1
            continue
        kept.append(r)

    if not kept:
        sys.exit("nothing survived filtering - relax --min-sep / --min-gen, or the GA "
                 "found no individual satisfying the cascade constraint (itself a result)")

    kept.sort(key=lambda r: num(r, "score"))   # genetic_sweep uses argmin: lower is better

    # Collapse near-clones. A converged front is mostly one design repeated.
    picked: list[dict] = []
    for r in kept:
        vec = [num(r, g) for g in GENE_TO_PX4]
        if any(all(abs(v - w) <= a.dedupe_tol * max(1.0, abs(w))
                   for v, w in zip(vec, [num(p, g) for g in GENE_TO_PX4]))
               for p in picked):
            continue
        picked.append(r)
        if len(picked) >= a.top:
            break

    cands = []
    for r in picked:
        c = {px4: round(num(r, gene), 4) for gene, px4 in GENE_TO_PX4.items()}
        if a.att_d >= 0:
            c["MC_EIG_ATT_D"] = a.att_d
        cands.append(c)

    Path(a.out).write_text(json.dumps(cands, indent=2) + "\n")

    print(f"=== {path}")
    print(f"  {len(rows)} rows -> {len(kept)} eligible -> {len(picked)} distinct candidates")
    print(f"  dropped: gen<{a.min_gen} {drop_gen}, non-finite score {drop_score}, "
          f"sep<{a.min_sep} {drop_sep}, unstable {drop_stable}")
    if a.att_d >= 0:
        gaks = [num(r, "Kd_att") for r in picked]
        print(f"  MC_EIG_ATT_D forced to {a.att_d} "
              f"(GA wanted {min(gaks):.3f}-{max(gaks):.3f}; see docstring)")
    print()
    for i, (c, r) in enumerate(zip(cands, picked), 1):
        print(f"  [{i}] wn={c['MC_EIG_WN']:7.3f}  b={c['MC_EIG_B']:7.3f}  "
              f"ATT_P={c['MC_EIG_ATT_P']:6.3f}  ATT_D={c['MC_EIG_ATT_D']:5.2f}  "
              f"sep={c['MC_EIG_WN']/c['MC_EIG_ATT_P']:5.2f}  "
              f"score={num(r,'score'):.4f}  gen={int(num(r,'gen',0))}")
    print(f"\nwrote {a.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
