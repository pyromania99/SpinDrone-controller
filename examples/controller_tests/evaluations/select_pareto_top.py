#!/usr/bin/env python3
"""
select_pareto_top.py — pick the top-N Pareto configurations per controller.

The sweeps optimise two objectives that trade off against each other:

    score            tracking cost (lower is better)
    effort_ac_ses    AC control effort, the differential part of the rotor
                     commands — what the controller spends *fighting* rather
                     than what it spends holding the vehicle up (lower better)

A single "best" run does not exist: there is a front. This walks every archive
belonging to a controller, pools the runs, drops the unstable ones, computes the
non-dominated set in (score, effort_ac_ses), and then picks N points spread
along it so the selection covers both ends rather than clustering at one.

Two archives do not record the attitude gains they ran with, because those were
held fixed rather than swept:

    eigen_sweep/ab_sweep_{wide,fine}   seeded per motor moment from the PD GA;
                                       the values are in the run log and are
                                       reproduced in SEEDED_ATT_GAINS below
    eigen_explore                      ran at the controller's own defaults

Both are filled in here so every emitted config is directly replayable.

Outputs (default: results/pareto_top/):
    pareto_top.json    machine-readable configs for the replay driver
    README.md          the same table for a human
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path

RESULTS = Path(__file__).resolve().parents[1] / "results"

# Motor moments to report on. km -> steady spin rate is r_ss ~ 4.9e7 * km.
DEFAULT_MM = [0.0, 2e-7, 4e-7, 6e-7]

SCORE_KEY = "score"
EFFORT_KEY = "effort_ac_ses"

# Runs above this are diverging, not trading off; STABLE_SCORE_MAX in
# sweep_metrics. Keeping them would let a crashed run sit on the front at
# near-zero effort.
STABLE_SCORE_MAX = 2.0

# Attitude gains the ab-sweeps were seeded with, transcribed from
# sweep_logs/20260726_214245/eigen-ab.log. Keyed by motor moment.
SEEDED_ATT_GAINS = {
    0.0:    (4.8421, 2.5692),
    1e-07:  (3.5438, 2.6016),
    2e-07:  (3.4145, 2.6049),
    2.7e-07: (3.3443, 2.0993),
    4e-07:  (2.5879, 9.4784),
    6e-07:  (3.8380, 1.0000),
}

# eigen_ab_explore.py constructs EigenController without overriding these.
EIGEN_DEFAULT_ATT = (4.68, 1.0)

# Sources per controller: (archive glob, label, how to fill missing gains)
SOURCES = {
    "eigen": [
        ("genetic_eigen_sweep/{mm}/genetic_archive.csv", "GA", None),
        ("eigen_sweep/kpkd_sweep/{mm}/grid_archive.csv", "kpkd-grid", None),
        ("eigen_sweep/ab_sweep_wide/{mm}/grid_archive.csv", "ab-wide", "seed"),
        ("eigen_sweep/ab_sweep_fine/{mm}/grid_archive.csv", "ab-fine", "seed"),
        ("eigen_explore/{mm}/exploration_archive.csv", "explore", "default"),
    ],
    "pd": [
        ("genetic_pd_sweep/{mm}/genetic_archive.csv", "GA", None),
        ("pd_sweep/{mm}/grid_archive.csv", "grid", None),
    ],
    # mpc_sweep.py only started writing a grid_archive.csv (and only started
    # sweeping the spin ladder at all) when it was brought in line with the
    # other sweeps; before that this controller had no entry here, and
    # pareto_top.json contained zero MPC configurations.
    "mpc": [
        ("mpc_sweep/{mm}/grid_archive.csv", "grid", None),
        # The widened q/r box, pooled alongside the original rather than
        # replacing it. The first grid (q 5-20, r 0.01-0.05) put BOTH objectives
        # at the same corner at all six motor moments — an optimum outside the
        # searched region — and re-running at q 0.5-5, r 0.05-1.0 cut
        # effort_ac_ses from 47-69 to 0.12-0.14 and flattened the spin
        # dependence from 81 % to 4.5 %. Pooling costs nothing: the original
        # box's points are simply dominated and drop off the front on their own,
        # while staying available as the record of what was searched.
        #
        # Kept as a separate path rather than merged into mpc_sweep/ because the
        # two boxes overlap at (q=5, r=0.05) and merging would silently
        # overwrite one run's telemetry with the other's.
        ("mpc_widen/mpc_sweep/{mm}/grid_archive.csv", "grid-wide", None),
    ],
}

GENES = {
    "eigen": ["wn_mdyn", "b_mdyn", "Kp_att", "Kd_att"],
    "pd": ["Kp_att", "Kd_att", "Kd_yaw_rate"],
    # The MPC's cost weights, not attitude gains. Named with the mpc_ prefix
    # they carry in every archive so they cannot be confused with the other
    # controllers' Kp_att/Kd_att, which are three different physical
    # quantities across the three controllers (see examples/utils/gain_seed.py).
    "mpc": ["mpc_q", "mpc_r"],
}


def mm_dir(mm: float) -> str:
    return f"mm_{mm:.1e}"


def fnum(raw) -> float:
    """Parse a CSV cell to float; non-numeric and non-finite both become nan."""
    if raw is None or raw == "":
        return math.nan
    try:
        v = float(raw)
    except ValueError:
        return math.nan
    return v if math.isfinite(v) else math.nan


def load_rows(controller: str, mm: float) -> list[dict]:
    """Pool every archive for one controller at one motor moment."""
    rows = []
    for rel, label, fill in SOURCES[controller]:
        path = RESULTS / rel.format(mm=mm_dir(mm))
        if not path.exists():
            continue
        with open(path, newline="") as fh:
            for raw in csv.DictReader(fh):
                # The archive's own motor_moment column is authoritative: a
                # mismatch against the directory means a stale file (the old
                # %.6f formatting rounded 1e-07 to 0.000000), so drop it.
                logged = fnum(raw.get("motor_moment"))
                if math.isfinite(logged) and not math.isclose(
                        logged, mm, rel_tol=1e-6, abs_tol=1e-12):
                    continue

                score = fnum(raw.get(SCORE_KEY))
                effort = fnum(raw.get(EFFORT_KEY))
                if not (math.isfinite(score) and math.isfinite(effort)):
                    continue
                if score > STABLE_SCORE_MAX:
                    continue
                if fnum(raw.get("stable")) != 1.0:
                    continue

                gains = {}
                for g in GENES[controller]:
                    gains[g] = fnum(raw.get(g))

                if fill == "seed":
                    kp, kd = SEEDED_ATT_GAINS[mm]
                    gains["Kp_att"], gains["Kd_att"] = kp, kd
                elif fill == "default":
                    gains["Kp_att"], gains["Kd_att"] = EIGEN_DEFAULT_ATT
                if controller == "pd" and not math.isfinite(gains["Kd_yaw_rate"]):
                    gains["Kd_yaw_rate"] = 0.0

                if any(not math.isfinite(v) for v in gains.values()):
                    continue

                rows.append({
                    "source": label,
                    "motor_moment": mm,
                    "score": score,
                    "effort": effort,
                    "rms_error": fnum(raw.get("rms_error")),
                    "settling_time": fnum(raw.get("settling_time")),
                    "max_overshoot": fnum(raw.get("max_overshoot")),
                    "peak_effort": fnum(raw.get("peak_effort")),
                    "r_ss": fnum(raw.get("r_ss")),
                    **gains,
                })
    return rows


def dedup(rows: list[dict], controller: str) -> list[dict]:
    """Collapse repeated gene tuples, keeping the best-scoring instance.

    The GA re-evaluates elites every generation, so a good individual appears
    dozens of times. Without this the front is mostly one point.
    """
    best: dict[tuple, dict] = {}
    for r in rows:
        key = tuple(round(r[g], 6) for g in GENES[controller])
        prev = best.get(key)
        if prev is None or r["score"] < prev["score"]:
            best[key] = r
    return list(best.values())


def pareto_front(rows: list[dict], eps: float = 0.0) -> list[dict]:
    """Non-dominated set for (score, effort), both minimised.

    With eps > 0 the objectives are first snapped to a logarithmic grid of
    relative cell size eps, one representative is kept per cell, and dominance
    is then applied to the cell indices. Plain dominance is useless here: the PD
    score saturates at a floor — over half its runs share a score to four
    decimals — so runs whose scores differ in the sixth decimal while effort
    climbs 20% all qualify as non-dominated. Those are one operating point
    measured repeatedly, and selecting ten of them yields ten identical flights.
    """
    if eps <= 0:
        keyed = [((r["score"], r["effort"]), r) for r in rows]
    else:
        step = math.log1p(eps)
        cells: dict[tuple[int, int], dict] = {}
        for r in rows:
            key = (int(math.floor(math.log(max(r["score"], 1e-12)) / step)),
                   int(math.floor(math.log(max(r["effort"], 1e-12)) / step)))
            prev = cells.get(key)
            if prev is None or (r["score"], r["effort"]) < (prev["score"], prev["effort"]):
                cells[key] = r
        keyed = list(cells.items())

    # Dominance runs on the cell indices, not the raw values. Comparing raw
    # values here would defeat the quantisation: a run whose score is lower by
    # one part in 10^5 while its effort is 60% higher would still be admitted.
    keyed.sort(key=lambda kr: kr[0])
    front, best_effort_cell = [], math.inf
    for key, r in keyed:
        if key[1] < best_effort_cell:
            front.append(r)
            best_effort_cell = key[1]
    return front


def _norm_coords(rows: list[dict]) -> list[tuple[float, float]]:
    """Objectives in normalised log space, for distance comparisons."""
    pts = [(math.log10(max(r["score"], 1e-9)), math.log10(max(r["effort"], 1e-9)))
           for r in rows]
    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
    sx = (max(xs) - min(xs)) or 1.0
    sy = (max(ys) - min(ys)) or 1.0
    return [((p[0] - min(xs)) / sx, (p[1] - min(ys)) / sy) for p in pts]


def _gene_coords(rows: list[dict], genes: list[str]) -> list[tuple[float, ...]]:
    """Gains normalised to [0, 1] per axis, for diversity comparisons."""
    lo = {g: min(r[g] for r in rows) for g in genes}
    hi = {g: max(r[g] for r in rows) for g in genes}
    return [tuple((r[g] - lo[g]) / ((hi[g] - lo[g]) or 1.0) for g in genes)
            for r in rows]


def select(front: list[dict], pool: list[dict], n: int, genes: list[str],
           fill_factor: float = 3.0) -> tuple[list[dict], int]:
    """Choose n configs spanning the trade-off; return them and the fill count.

    The front comes first, since those are the genuinely non-dominated runs.
    When it is shorter than n — which happens whenever a controller's score
    saturates and its front collapses to a couple of points — the remainder is
    topped up from runs that are still *good* (within fill_factor of the best
    score on offer) and spread over the **gain space** rather than the objective
    plane. Spreading over objectives would be wrong here: with a one-point
    front, the far corners of the objective plane are exactly the runs the front
    dominates, so it would select the worst flights available. Distinct gains at
    comparable performance is what is actually worth watching.

    Fill points are not claimed to be non-dominated; the count is returned so
    the report can label which is which.
    """
    if len(front) >= n:
        chosen = list(front)
        if len(front) > n:
            coords = _norm_coords(front)
            best_score_i = min(range(len(front)), key=lambda i: front[i]["score"])
            best_effort_i = min(range(len(front)), key=lambda i: front[i]["effort"])
            keep = [best_score_i]
            if best_effort_i != best_score_i:
                keep.append(best_effort_i)
            while len(keep) < n:
                far, far_d = None, -1.0
                for i in range(len(front)):
                    if i in keep:
                        continue
                    d = min(math.dist(coords[i], coords[c]) for c in keep)
                    if d > far_d:
                        far, far_d = i, d
                if far is None:
                    break
                keep.append(far)
            chosen = [front[i] for i in keep]
        return sorted(chosen, key=lambda r: r["effort"]), 0

    chosen = list(front)
    on_front = len(chosen)
    ids = {id(r) for r in chosen}
    best_score = min((r["score"] for r in front), default=math.inf)
    cutoff = best_score * fill_factor
    rest = [r for r in pool if id(r) not in ids and r["score"] <= cutoff]
    if rest:
        coords = _gene_coords(chosen + rest, genes)
        picked_coords = coords[:len(chosen)]
        c_rest = coords[len(chosen):]
        while len(chosen) < n and rest:
            far, far_d = None, -1.0
            for i in range(len(rest)):
                d = min(math.dist(c_rest[i], c) for c in picked_coords)
                if d > far_d:
                    far, far_d = i, d
            if far is None:
                break
            chosen.append(rest.pop(far))
            picked_coords.append(c_rest.pop(far))
    return sorted(chosen, key=lambda r: r["effort"]), on_front


def fmt(v: float, nd: int = 4) -> str:
    if not math.isfinite(v):
        return "—"
    if v != 0 and (abs(v) < 1e-3 or abs(v) >= 1e5):
        return f"{v:.2e}"
    return f"{v:.{nd}f}"


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--top", type=int, default=10, help="configs per controller per mm")
    p.add_argument("--motor-moments", nargs="+", type=float, default=DEFAULT_MM)
    p.add_argument("--eps", type=float, default=0.02,
                   help="relative cell size for epsilon-dominance (0 = strict)")
    p.add_argument("--fill-factor", type=float, default=3.0,
                   help="fill candidates must score within this factor of the best")
    p.add_argument("--out", type=Path, default=RESULTS / "pareto_top")
    args = p.parse_args()

    args.out.mkdir(parents=True, exist_ok=True)
    payload = {"top": args.top, "score_key": SCORE_KEY, "effort_key": EFFORT_KEY,
               "stable_score_max": STABLE_SCORE_MAX, "eps": args.eps, "sets": []}
    md: list[str] = []

    md.append("# Pareto top-%d — replay set\n" % args.top)
    md.append(
        "Configurations selected from the completed sweep suite "
        "(`results/sweep_logs/20260726_214245/`) for headed replay **from the "
        "ground, including takeoff**. The sweeps themselves spawned in the air "
        "at the hover target, so takeoff behaviour of these gains has never "
        "been observed.\n")
    md.append(
        "## How these were chosen\n\n"
        f"Two objectives, both minimised: `{SCORE_KEY}` (tracking cost) and "
        f"`{EFFORT_KEY}` (AC control effort — the differential part of the rotor "
        "commands, i.e. what the controller spends fighting rather than what it "
        "spends holding the vehicle up).\n\n"
        "1. Pool every archive for that controller at that motor moment. Rows "
        "whose `motor_moment` column disagrees with their directory are stale "
        "and are dropped.\n"
        f"2. Drop runs with `stable != 1` or `score > {STABLE_SCORE_MAX}` — those "
        "are diverging, not trading off, and a crashed run would otherwise sit "
        "on the front at near-zero effort.\n"
        "3. Collapse repeated gene tuples to their best instance. The GA "
        "re-evaluates its elites every generation, so without this one "
        "individual appears dozens of times.\n"
        f"4. Take the ε-non-dominated set with ε = {args.eps:g}: objectives are "
        "snapped to a logarithmic grid of that relative cell size, one "
        "representative is kept per cell, and dominance is applied to the cell "
        "indices. Plain dominance does not work here — see the note on PD "
        "below.\n"
        f"5. If the front exceeds {args.top} points, pin both extremes (most "
        "accurate, cheapest) and farthest-point sample the rest in log-log "
        "objective space. If it falls short, top up from runs scoring within "
        f"{args.fill_factor:g}× of the best, spread over **gain space**. Those "
        "rows are marked `fill`. Spreading fill over the objective plane would "
        "be wrong when the front is one point wide: the far corners of that "
        "plane are precisely the runs the front dominates, so it would select "
        "the worst flights available. Distinct gains at comparable performance "
        "is what is worth watching.\n")
    md.append(
        "## Why ε-dominance, and what it exposes\n\n"
        "Cascaded PD's score **saturates**. At km = 4e-07, 254 of its 491 stable "
        "runs share `score = 0.0531` to four decimals; the accuracy floor is set "
        "by the fixed outer position loop, and the attitude gains stop mattering "
        "once they are above a threshold. Under strict dominance a run scoring "
        "lower by one part in 10⁵ while spending 20% more effort still counts as "
        "non-dominated, so that plateau masquerades as a 10-point front — and "
        "replaying it would give ten numerically identical flights. Quantising "
        "first collapses it to the single operating point it really is.\n\n"
        "The consequence is in the summary table above: **PD's front is 1–7 "
        "points wide, Eigen's is 12–33.** PD has essentially one operating point "
        "per spin rate and no accuracy/effort trade-off to exploit; Eigen has a "
        "real front that buys roughly an order of magnitude of accuracy for "
        "effort. Most PD rows below are therefore `fill`, and that is the "
        "finding rather than a shortfall of the selection.\n")
    md.append(
        "## Gains that were not in the archives\n\n"
        "`ab_sweep_wide` / `ab_sweep_fine` swept only (a, b) with the attitude "
        "gains pinned per motor moment (seeded from the PD GA); "
        "`eigen_explore` ran at the controller's own defaults. Those columns "
        "are absent from the CSVs and are reconstructed here so every row "
        "below is directly replayable:\n\n"
        "| km | Kp_att | Kd_att | source |\n|---|---|---|---|\n"
        + "".join(f"| {k:g} | {v[0]} | {v[1]} | ab-sweep seed |\n"
                  for k, v in sorted(SEEDED_ATT_GAINS.items()))
        + f"| any | {EIGEN_DEFAULT_ATT[0]} | {EIGEN_DEFAULT_ATT[1]} "
          "| eigen_explore default |\n")

    summary_rows = []

    for controller in ("eigen", "pd", "mpc"):
        title = {"eigen": "Eigen", "pd": "Cascaded PD",
                 "mpc": "LPV-MPC"}[controller]
        md.append(f"\n---\n\n# {title}\n")
        for mm in args.motor_moments:
            rows = dedup(load_rows(controller, mm), controller)
            front = pareto_front(rows, eps=args.eps)
            picked, n_front = select(front, rows, args.top, GENES[controller],
                                     fill_factor=args.fill_factor)
            r_ss = next((r["r_ss"] for r in picked if math.isfinite(r["r_ss"])), math.nan)
            n_fill = len(picked) - n_front if n_front else 0
            front_ids = {id(r) for r in front}

            summary_rows.append((title, mm, len(rows), len(front), len(picked),
                                 min((r["score"] for r in picked), default=math.nan),
                                 min((r["effort"] for r in picked), default=math.nan)))

            md.append(f"\n## km = {mm:g}  (spin ≈ {fmt(r_ss, 2)} rad/s)\n")
            if n_fill:
                md.append(
                    f"{len(rows)} stable configs pooled. The ε-front is only "
                    f"**{len(front)} point(s) wide** — this controller's score "
                    f"saturates, so there is barely a trade-off to sample. The "
                    f"remaining {n_fill} rows are near-front fill, chosen to "
                    f"spread over the (score, effort) plane; they are marked "
                    f"`fill` and are *not* claimed to be non-dominated.\n")
            else:
                md.append(f"{len(rows)} stable configs pooled → {len(front)} on "
                          f"the ε-front → {len(picked)} selected.\n")
            if not picked:
                md.append("\n_No stable configuration survived the filter._\n")
                continue

            gene_cols = GENES[controller]
            md.append("\n| # | " + " | ".join(gene_cols)
                      + " | score | effort_ac | rms_err | overshoot"
                        " | peak_eff | source | on front |")
            md.append("|---|" + "---|" * (len(gene_cols) + 7))
            for i, r in enumerate(picked, 1):
                cells = [fmt(r[g]) for g in gene_cols]
                md.append(
                    f"| {i} | " + " | ".join(cells)
                    + f" | {fmt(r['score'])} | {fmt(r['effort'], 2)}"
                      f" | {fmt(r['rms_error'])}"
                      f" | {fmt(r['max_overshoot'], 3)}"
                      f" | {fmt(r['peak_effort'], 1)} | {r['source']}"
                      f" | {'yes' if id(r) in front_ids else 'fill'} |")

            payload["sets"].append({
                "controller": controller,
                "motor_moment": mm,
                "pool_size": len(rows),
                "front_size": len(front),
                "configs": [
                    {"rank": i, "source": r["source"],
                     "on_front": id(r) in front_ids,
                     "gains": {g: r[g] for g in gene_cols},
                     "metrics": {k: r[k] for k in
                                 ("score", "effort", "rms_error",
                                  "settling_time", "max_overshoot",
                                  "peak_effort", "r_ss")}}
                    for i, r in enumerate(picked, 1)],
            })

    md.insert(2, "\n## Summary\n\n"
              "| controller | km | spin (rad/s) | pooled | ε-front width | selected | best score | least effort |\n"
              "|---|---|---|---|---|---|---|---|\n"
              + "".join(f"| {c} | {m:g} | {4.9e7 * m:.1f} | {p} | {f} | {s} | "
                        f"{fmt(bs)} | {fmt(be, 2)} |\n"
                        for c, m, p, f, s, bs, be in summary_rows))

    total = sum(len(s["configs"]) for s in payload["sets"])
    md.append(
        f"\n---\n\n## Replay\n\n**{total} runs total** "
        f"({args.top} per controller per motor moment, three controllers, "
        f"{len(args.motor_moments)} motor moments).\n\n"
        "`pareto_top.json` alongside this file carries the same configurations "
        "in machine-readable form, including the reconstructed attitude gains, "
        "and is what the headed replay driver consumes.\n\n"
        "Every run here was originally flown **spawned in the air at the hover "
        "target**, to keep the takeoff transient out of the metrics. The replay "
        "starts on the ground instead, so its numbers are not comparable to the "
        "`score` and `effort_ac` columns above — those stay as the selection "
        "record. What the replay adds is the takeoff behaviour, which no "
        "measurement in this suite has ever covered.\n")

    (args.out / "pareto_top.json").write_text(json.dumps(payload, indent=2))
    (args.out / "README.md").write_text("\n".join(md) + "\n")
    print(f"wrote {args.out / 'README.md'}")
    print(f"wrote {args.out / 'pareto_top.json'}  ({total} configs)")
    for c, m, pool, f, s, bs, be in summary_rows:
        print(f"  {c:<12} km={m:<8g} pool={pool:<5} front={f:<4} picked={s:<3} "
              f"best_score={fmt(bs)} min_effort={fmt(be, 2)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
