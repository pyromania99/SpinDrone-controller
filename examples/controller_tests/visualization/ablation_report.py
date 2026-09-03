#!/usr/bin/env python3
"""
ablation_report.py — the per-parameter gradient, drawn and ranked.

Reads what sweeps/ablation_explore.py wrote and answers three questions:

    which parameters matter, and by how much   (sensitivity ranking)
    what does each one actually do             (response curves)
    how far can each one move before breaking  (stability limits)

The single most important thing this file does is **draw the noise band**.
Measured over 8 identical slots, the score spread is 0.0045 % and the effort
spread 0.45 %; the effect worth caring about is ~7 % of median score. Most
panels will nevertheless be flat, because the archived corpus shows Spearman
rho of wn_mdyn against score is -0.001 across a 45x range. A trend line drawn
through a flat panel is worse than no panel: it invents a finding. So every
curve that stays inside the replicate band is labelled "flat within noise"
rather than given a slope, which turns rho = -0.001 from a mystery into a
stated result.

Classification comes from robustness._failed — the same function that builds
breakdown.csv and the robustness report — never a second copy of the rules.

Plain python: numpy + stdlib. No Isaac Sim, no plotting library, no network.
The page embeds its own SVG, so it opens offline.

Usage
─────
    python3 visualization/ablation_report.py
    python3 visualization/ablation_report.py --root results/ablation
    python3 visualization/ablation_report.py --no-html
"""
from __future__ import annotations

import argparse
import csv
import html
import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "utils"))
import gene_space          # noqa: E402
import robustness as rb    # noqa: E402
import sweep_metrics as sm  # noqa: E402

RESULTS = Path(__file__).resolve().parents[1] / "results"

# Metrics worth a panel, in reading order. The full METRIC_KEYS set is in the
# archive; these are the ones that carry the argument.
PANEL_METRICS = [
    ("score",           "stability score [m]"),
    ("score_transient", "transient error [m]"),
    ("max_excursion",   "peak excursion [m]"),
    ("score_tail",      "settled error [m]"),
    ("effort_ac",       "differential effort [N²]"),
    ("oscillation",     "roll/pitch rate RMS [rad/s]"),
]

# A metric whose replicate band exceeds this cannot resolve a gradient smaller
# than itself, so it is excluded from the sensitivity ranking and marked in the
# report rather than quietly producing a number.
#
# Measured over the interleaved replicates of all six cases: max_excursion
# 0.00-0.06 %, score_transient 0.17-0.81 %, score 0.14-2.65 %, score_tail
# 0.20-13.3 % — all usable. But oscillation runs 47.8-173.8 % and effort_ac
# 1.8-318.5 %, because the differential effort depends on where in its limit
# cycle the attitude loop happens to sit. Ranking on those would rank noise.
#
# The 5x effort criterion that defines a breakdown is unaffected: it is far
# outside even the widest of those bands.
NOISE_BAND_USABLE_MAX: float = 0.25

# Categorical slots 1-3 of the reference palette, unchanged and in documented
# order; that file records this subset as clearing the all-pairs CVD and
# normal-vision gates in both modes. Used for the two motor moments plus the
# noise band, so at most three series share a panel.
SERIES = ["#2a78d6", "#eb6834", "#1baf7a"]

STATUS = {
    "":         ("#0ca30c", "ok",    "●"),
    "effort":   ("#fab219", "eff",   "▲"),
    "tracking": ("#ec835a", "trk",   "■"),
    "crash":    ("#d03b3b", "crash", "✕"),
}

OUTCOME_GLYPH = {
    "broken":            ("#d03b3b", "broke"),
    "censored_box":      ("#78766f", "hit box edge"),
    "censored_extended": ("#78766f", "never broke"),
}


def fnum(raw, default=math.nan):
    try:
        v = float(raw)
    except (TypeError, ValueError):
        return default
    return v


def status_for(mode: str):
    if not mode:
        return STATUS[""]
    for key in ("crash", "tracking", "effort"):
        if key in mode:
            return STATUS[key]
    return STATUS[""]


# ── Loading ──────────────────────────────────────────────────────────────────

def load_case(case_dir: Path) -> dict:
    """One (controller, motor moment): reference, calibration, runs, limits."""
    ref_path = case_dir / "reference.json"
    arch_path = case_dir / "ablation_archive.csv"
    if not (ref_path.is_file() and arch_path.is_file()):
        return None
    reference = json.loads(ref_path.read_text())

    rows = list(csv.DictReader(open(arch_path, newline="")))
    calib = []
    cal_path = case_dir / "calibration.csv"
    if cal_path.is_file():
        calib = list(csv.DictReader(open(cal_path, newline="")))

    limits = []
    lim_path = case_dir / "limits.csv"
    if lim_path.is_file():
        limits = list(csv.DictReader(open(lim_path, newline="")))

    # The noise band, from every run that flew the reference — the calibration
    # batch plus every padding replicate the march accumulated.
    band = {}
    ref_runs = calib + [r for r in rows if r.get("gene") == "_reference"]
    for key, _ in PANEL_METRICS:
        v = np.array([fnum(r.get(key)) for r in ref_runs])
        v = v[np.isfinite(v)]
        if v.size >= 2:
            band[key] = {"median": float(np.median(v)), "lo": float(v.min()),
                         "hi": float(v.max()), "n": int(v.size),
                         "rel": float(np.ptp(v) / max(abs(np.median(v)), 1e-12))}
    return {"dir": case_dir, "controller": case_dir.parts[-2],
            "mm": fnum(case_dir.parts[-1].replace("mm_", "")),
            "reference": reference, "rows": rows, "limits": limits,
            "band": band, "n_ref": len(ref_runs)}


def curves(case: dict, gene: str, metric: str) -> list:
    """(value, metric) points for one gene, both directions, sorted by value."""
    pts = []
    x0 = case["reference"]["genes"].get(gene)
    for r in case["rows"]:
        if r.get("gene") != gene:
            continue
        x, y = fnum(r.get("value")), fnum(r.get(metric))
        if np.isfinite(x) and np.isfinite(y):
            pts.append((x, y, r.get("failed_reason", "")))
    if x0 is not None and case["band"].get(metric):
        pts.append((float(x0), case["band"][metric]["median"], ""))
    return sorted(pts, key=lambda p: p[0])


def sensitivity(case: dict, gene: str) -> dict:
    """How far this gene moves before any metric leaves the noise band.

    Reported as a ratio to the reference value — scale-free, so genes with
    wildly different units (wn_mdyn spans 0.5-60, zeta_xy spans 0.5-1.3)
    compare on one axis. A large tolerance means the parameter barely matters;
    a small one means it is load-bearing.
    """
    x0 = case["reference"]["genes"].get(gene)
    if x0 in (None, 0):
        return {"tolerance": math.nan, "driver": "", "elasticity": {}}

    best_ratio, driver = math.inf, ""
    elasticity = {}
    for metric, _ in PANEL_METRICS:
        band = case["band"].get(metric)
        if not band or band["rel"] > NOISE_BAND_USABLE_MAX:
            continue
        pts = [(x, y) for x, y, _ in curves(case, gene, metric)]
        if len(pts) < 3:
            continue
        # First departure from the band, walking outward from the reference.
        for x, y in pts:
            if y < band["lo"] or y > band["hi"]:
                ratio = max(x / x0, x0 / x) if x > 0 else math.inf
                if ratio < best_ratio:
                    best_ratio, driver = ratio, metric
        # Local elasticity: d(log metric)/d(log value) over the nearest points
        # either side of the reference.
        near = sorted(pts, key=lambda p: abs(math.log(max(p[0], 1e-12)) -
                                             math.log(max(x0, 1e-12))))[:5]
        xs = np.array([p[0] for p in near]); ys = np.array([p[1] for p in near])
        ok = (xs > 0) & (ys > 0)
        if ok.sum() >= 3:
            try:
                slope = np.polyfit(np.log(xs[ok]), np.log(ys[ok]), 1)[0]
                elasticity[metric] = float(slope)
            except Exception:
                pass
    return {"tolerance": best_ratio, "driver": driver, "elasticity": elasticity}


# ── Terminal ─────────────────────────────────────────────────────────────────

def print_summary(cases: list) -> None:
    for case in cases:
        ref = case["reference"]
        print(f"\n{'='*80}\n  {case['controller'].upper()}  km={case['mm']:.1e}"
              f"   reference: " + "  ".join(f"{k}={v:g}" for k, v in ref["genes"].items())
              + f"\n{'='*80}")

        if case["band"]:
            print(f"  noise band over {case['n_ref']} reference runs:")
            for key, _ in PANEL_METRICS:
                b = case["band"].get(key)
                if b:
                    print(f"     {key:<16} median {b['median']:>10.6f}  "
                          f"spread {b['rel']:>8.4%}")

        genes = [g for g, _, _ in gene_space.genes_for(case["controller"], True)]
        sens = {g: sensitivity(case, g) for g in genes}
        ordered = sorted(genes, key=lambda g: sens[g]["tolerance"])
        print("\n  sensitivity — how far before any metric leaves the band"
              "  (smaller = more load-bearing)")
        print(f"     {'gene':<14} {'tolerance':>10}  {'driven by':<16} elasticity(score)")
        for g in ordered:
            s = sens[g]
            tol = ("never" if not np.isfinite(s["tolerance"])
                   else f"{s['tolerance']:.2f}x")
            el = s["elasticity"].get("score")
            print(f"     {g:<14} {tol:>10}  {s['driver']:<16} "
                  f"{'' if el is None else f'{el:+.3f}'}")

        if case["limits"]:
            print("\n  stability limits")
            print(f"     {'gene':<14} {'dir':<5} {'reference':>10} {'last ok':>10} "
                  f"{'first fail':>11} {'outcome':<18} mode")
            for lim in case["limits"]:
                print(f"     {lim['gene']:<14} {lim['direction']:<5} "
                      f"{lim['reference']:>10} {lim['last_surviving'] or '-':>10} "
                      f"{lim['first_failing'] or '-':>11} {lim['outcome']:<18} "
                      f"{lim['failure_mode']}")


# ── SVG ──────────────────────────────────────────────────────────────────────

def nice_ticks(lo, hi, n=4):
    if not (np.isfinite(lo) and np.isfinite(hi)) or hi <= lo:
        return [lo]
    raw = (hi - lo) / n
    mag = 10 ** math.floor(math.log10(raw))
    step = min((m * mag for m in (1, 2, 2.5, 5, 10)), key=lambda s: abs(s - raw))
    start = math.ceil(lo / step) * step
    out, v = [], start
    while v <= hi * (1 + 1e-9):
        out.append(v); v += step
    return out or [lo, hi]


def fmt(v):
    a = abs(v)
    if a == 0:
        return "0"
    if a >= 1000 or a < 0.01:
        return f"{v:.1e}"
    return f"{v:g}" if a >= 1 else f"{v:.3g}"


def curve_panel(cases_by_mm: dict, gene: str, metric: str, label: str,
                width=340, height=210) -> str:
    """One (gene x metric) panel, one line per motor moment, noise band shaded."""
    pad_l, pad_r, pad_t, pad_b = 52, 22, 26, 34
    pw, ph = width - pad_l - pad_r, height - pad_t - pad_b

    series = []
    for i, (mm, case) in enumerate(sorted(cases_by_mm.items())):
        pts = curves(case, gene, metric)
        if len(pts) >= 2:
            series.append((mm, case, pts, SERIES[i % len(SERIES)]))
    if not series:
        return ""

    xs = [p[0] for _, _, pts, _ in series for p in pts]
    ys = [p[1] for _, _, pts, _ in series for p in pts]
    bands = [c["band"].get(metric) for _, c, _, _ in series if c["band"].get(metric)]
    for b in bands:
        ys += [b["lo"], b["hi"]]
    ys = [y for y in ys if np.isfinite(y)]
    if not xs or not ys:
        return ""

    log_x = gene_space.GENE_SCALE.get(gene) == "log" and min(xs) > 0
    log_y = min(ys) > 0 and max(ys) / max(min(ys), 1e-12) > 50

    def tx(v):
        if log_x:
            lo, hi = math.log10(min(xs)), math.log10(max(xs))
            return pad_l + (math.log10(max(v, 1e-12)) - lo) / max(hi - lo, 1e-9) * pw
        lo, hi = min(xs), max(xs)
        return pad_l + (v - lo) / max(hi - lo, 1e-9) * pw

    y_lo, y_hi = min(ys), max(ys)
    pad = 0.08 * (y_hi - y_lo) if y_hi > y_lo else max(abs(y_hi) * 0.1, 1e-9)
    y_lo, y_hi = y_lo - pad, y_hi + pad

    def ty(v):
        if log_y:
            lo, hi = math.log10(max(y_lo, 1e-12)), math.log10(max(y_hi, 1e-12))
            return pad_t + ph - (math.log10(max(v, 1e-12)) - lo) / max(hi - lo, 1e-9) * ph
        return pad_t + ph - (v - y_lo) / max(y_hi - y_lo, 1e-9) * ph

    out = [f'<svg viewBox="0 0 {width} {height}" class="panel" role="img" '
           f'aria-label="{html.escape(gene)} versus {html.escape(label)}">']
    out.append(f'<text x="{pad_l}" y="13" class="panel-title">{html.escape(gene)}</text>')
    out.append(f'<text x="{width-pad_r}" y="13" class="panel-sub">{html.escape(label)}</text>')

    for v in nice_ticks(y_lo, y_hi):
        y = ty(v)
        if pad_t <= y <= pad_t + ph:
            out.append(f'<line x1="{pad_l}" y1="{y:.1f}" x2="{width-pad_r}" '
                       f'y2="{y:.1f}" class="grid"/>')
            out.append(f'<text x="{pad_l-5}" y="{y+3.5:.1f}" class="tick tick-y">{fmt(v)}</text>')

    # The noise band. Everything inside it is indistinguishable from a repeat
    # of the reference, so a curve that stays here has no measurable gradient.
    for _, case, _, _ in series[:1]:
        b = case["band"].get(metric)
        if b:
            y1, y2 = ty(b["hi"]), ty(b["lo"])
            out.append(f'<rect x="{pad_l}" y="{min(y1,y2):.1f}" width="{pw}" '
                       f'height="{max(abs(y2-y1), 1.2):.1f}" class="band"/>')

    x0 = None
    for _, case, _, _ in series[:1]:
        x0 = case["reference"]["genes"].get(gene)
    if x0 is not None and min(xs) <= x0 <= max(xs):
        out.append(f'<line x1="{tx(x0):.1f}" y1="{pad_t}" x2="{tx(x0):.1f}" '
                   f'y2="{pad_t+ph}" class="ref-line"/>')
        out.append(f'<text x="{tx(x0):.1f}" y="{pad_t-3}" class="anno anno-mid">ref</text>')

    for v in (min(xs), max(xs)):
        out.append(f'<text x="{tx(v):.1f}" y="{height-pad_b+14:.1f}" '
                   f'class="tick tick-x">{fmt(v)}</text>')

    flat_all = True
    for i, (mm, case, pts, colour) in enumerate(series):
        b = case["band"].get(metric)
        d = " ".join(("M" if k == 0 else "L") + f"{tx(x):.1f},{ty(y):.1f}"
                     for k, (x, y, _) in enumerate(pts))
        out.append(f'<path d="{d}" class="series" style="stroke:{colour}"/>')
        outside = False
        for x, y, mode in pts:
            if b and (y < b["lo"] or y > b["hi"]):
                outside = True
            fill = status_for(mode)[0] if mode else colour
            tip = (f'{gene}={fmt(x)} | {metric}={y:.6g} | km={mm:.1e}'
                   + (f' | {status_for(mode)[1]}' if mode else ''))
            out.append(f'<circle cx="{tx(x):.1f}" cy="{ty(y):.1f}" r="3.4" '
                       f'style="fill:{fill}" class="mark">'
                       f'<title>{html.escape(tip)}</title></circle>')
        if outside:
            flat_all = False

    if flat_all and bands:
        rel = max(b["rel"] for b in bands)
        n = max(c["n_ref"] for _, c, _, _ in series)
        out.append(f'<text x="{pad_l + pw/2:.1f}" y="{pad_t + ph/2:.1f}" '
                   f'class="flat">flat within noise (n={n}, ±{rel:.2%})</text>')

    out.append("</svg>")
    return "".join(out)


def limits_svg(case: dict) -> str:
    """Per gene: the box, the reference, and how far each direction survived."""
    lims = case["limits"]
    if not lims:
        return ""
    genes = []
    for lim in lims:
        if lim["gene"] not in genes:
            genes.append(lim["gene"])
    row_h, label_w, track_w = 30, 110, 380
    width, height = label_w + track_w + 130, 26 + row_h * len(genes)

    out = [f'<svg viewBox="0 0 {width} {height}" class="grid-svg" role="img" '
           f'aria-label="Per-parameter stability limits">']
    for i, gene in enumerate(genes):
        y = 20 + row_h * i
        arms = {l["direction"]: l for l in lims if l["gene"] == gene}
        ref = fnum(next(iter(arms.values()))["reference"])
        lo = fnum(next(iter(arms.values()))["reach_lo"])
        hi = fnum(next(iter(arms.values()))["reach_hi"])
        vals = [ref, lo, hi]
        for a in arms.values():
            vals += [fnum(a["last_surviving"]), fnum(a["first_failing"])]
        vals = [v for v in vals if np.isfinite(v)]
        vlo, vhi = min(vals), max(vals)

        def tx(v):
            return label_w + (v - vlo) / max(vhi - vlo, 1e-9) * track_w

        out.append(f'<text x="0" y="{y+4}" class="row-label">{html.escape(gene)}</text>')
        out.append(f'<line x1="{label_w}" y1="{y}" x2="{label_w+track_w}" y2="{y}" '
                   f'class="track"/>')
        out.append(f'<line x1="{tx(lo):.1f}" y1="{y-5}" x2="{tx(lo):.1f}" y2="{y+5}" class="box-tick"/>')
        out.append(f'<line x1="{tx(hi):.1f}" y1="{y-5}" x2="{tx(hi):.1f}" y2="{y+5}" class="box-tick"/>')

        left = fnum(arms.get("down", {}).get("last_surviving"), ref)
        right = fnum(arms.get("up", {}).get("last_surviving"), ref)
        left = left if np.isfinite(left) else ref
        right = right if np.isfinite(right) else ref
        out.append(f'<line x1="{tx(min(left,right)):.1f}" y1="{y}" '
                   f'x2="{tx(max(left,right)):.1f}" y2="{y}" class="survive"/>')
        out.append(f'<circle cx="{tx(ref):.1f}" cy="{y}" r="4" class="ref-dot"/>')

        notes = []
        for d in ("down", "up"):
            a = arms.get(d)
            if not a:
                continue
            colour, word = OUTCOME_GLYPH.get(a["outcome"], ("#78766f", a["outcome"]))
            if a["outcome"] == "broken" and a["first_failing"]:
                x = tx(fnum(a["first_failing"]))
                out.append(f'<circle cx="{x:.1f}" cy="{y}" r="4.5" '
                           f'style="fill:{colour}" class="break-dot"><title>'
                           f'{html.escape(d)} broke at {a["first_failing"]} '
                           f'({a["failure_mode"]})</title></circle>')
                notes.append(f'{d} {a["failure_mode"]}')
            else:
                notes.append(f'{d} {word}')
        out.append(f'<text x="{label_w+track_w+8}" y="{y+4}" class="limit-note">'
                   f'{html.escape("; ".join(notes))}</text>')
    out.append("</svg>")
    return "".join(out)


CSS = """
:root{color-scheme:light dark}
.viz-root{--surface-1:#fcfcfb;--surface-2:#f4f3f0;--text-primary:#0b0b0b;
  --text-secondary:#52514e;--text-muted:#78766f;--rule:#dcdbd6;--band:#c9c8c250}
@media (prefers-color-scheme:dark){:root:where(:not([data-theme="light"])) .viz-root{
  --surface-1:#1a1a19;--surface-2:#232320;--text-primary:#fff;
  --text-secondary:#c3c2b7;--text-muted:#96958c;--rule:#3a3a36;--band:#5a594f50}}
:root[data-theme="dark"] .viz-root{--surface-1:#1a1a19;--surface-2:#232320;
  --text-primary:#fff;--text-secondary:#c3c2b7;--text-muted:#96958c;
  --rule:#3a3a36;--band:#5a594f50}
body{margin:0;background:var(--surface-1)}
.viz-root{background:var(--surface-1);color:var(--text-primary);
  font:14px/1.55 ui-sans-serif,system-ui,-apple-system,sans-serif;
  padding:28px clamp(14px,4vw,48px);max-width:1240px;margin:0 auto}
h1{font-size:1.5rem;margin:0 0 2px}h2{font-size:1.1rem;margin:34px 0 4px}
h3{font-size:.9rem;margin:20px 0 6px;color:var(--text-secondary);font-weight:600}
p{color:var(--text-secondary);margin:6px 0 12px;max-width:76ch}
code{background:var(--surface-2);padding:1px 5px;border-radius:4px;font-size:.86em}
.panels{display:grid;gap:10px;grid-template-columns:repeat(auto-fill,minmax(320px,1fr))}
.panel{width:100%;height:auto;background:var(--surface-2);border-radius:8px}
.panel-title{fill:var(--text-primary);font-size:11px;font-weight:700}
.panel-sub{fill:var(--text-muted);font-size:9px;text-anchor:end}
.grid{stroke:var(--rule);stroke-width:1}
.band{fill:var(--band)}
.tick{fill:var(--text-muted);font-size:9px}
.tick-y{text-anchor:end}.tick-x{text-anchor:middle}
.anno{fill:var(--text-muted);font-size:9px}.anno-mid{text-anchor:middle}
.ref-line{stroke:var(--text-muted);stroke-width:1;stroke-dasharray:2 3;opacity:.8}
.series{fill:none;stroke-width:2;stroke-linejoin:round}
.mark{stroke:var(--surface-2);stroke-width:1.5}
.flat{fill:var(--text-muted);font-size:10px;text-anchor:middle;font-style:italic}
.grid-svg{width:100%;height:auto;background:var(--surface-2);border-radius:8px;padding:6px 0}
.row-label{fill:var(--text-secondary);font-size:11px}
.track{stroke:var(--rule);stroke-width:2}
.box-tick{stroke:var(--text-muted);stroke-width:1.5}
.survive{stroke:#1baf7a;stroke-width:6;stroke-linecap:round;opacity:.75}
.ref-dot{fill:var(--text-primary)}
.break-dot{stroke:var(--surface-2);stroke-width:1.5}
.limit-note{fill:var(--text-muted);font-size:10px}
table{border-collapse:collapse;width:100%;font-size:.82rem;margin-top:8px}
th,td{padding:5px 9px;text-align:right;border-bottom:1px solid var(--rule);white-space:nowrap}
th{background:var(--surface-2);color:var(--text-secondary);font-weight:600}
td:first-child,th:first-child{text-align:left}
.tablewrap{overflow-x:auto;border:1px solid var(--rule);border-radius:8px}
.note{border-left:3px solid var(--rule);padding:2px 0 2px 12px;margin:12px 0;
  color:var(--text-secondary);font-size:.88rem}
.bar{height:11px;border-radius:3px;background:#2a78d6;display:inline-block}
"""


def render_html(cases: list) -> str:
    by_ctrl = {}
    for c in cases:
        by_ctrl.setdefault(c["controller"], {})[c["mm"]] = c

    p = ['<div class="viz-root"><h1>Parameter ablation</h1>',
         '<p>Each tuning parameter marched outward from a reference '
         'configuration until flight broke, holding every other parameter '
         'fixed. For a controlled one-at-a-time design the response curve '
         '<em>is</em> the partial dependence — no estimator, no smoothing.</p>',
         f'<div class="note">The shaded strip on every panel is the '
         f'<strong>noise band</strong>: the spread across repeated flights of '
         f'the identical reference configuration. A curve inside it has no '
         f'measurable gradient, and is labelled as such rather than given a '
         f'slope. Breakdown is classified by the same rules as the robustness '
         f'study — crash, score &gt; {sm.STABLE_SCORE_MAX:g}, or effort above '
         f'{rb.EFFORT_BREAKDOWN_RATIO:g}× the reference.</div>']

    for ctrl, by_mm in sorted(by_ctrl.items()):
        any_case = next(iter(by_mm.values()))
        genes = [g for g, _, _ in gene_space.genes_for(ctrl, True)]
        p.append(f'<h2>{html.escape(ctrl)}</h2>')

        p.append('<h3>Sensitivity ranking</h3>')
        p.append('<p>How far each parameter can move from the reference, as a '
                 'multiple, before any metric leaves the noise band. Smaller '
                 'means more load-bearing. Scale-free, so parameters with '
                 'different units compare directly.</p>')
        rank = []
        for g in genes:
            s = sensitivity(any_case, g)
            rank.append((g, s))
        rank.sort(key=lambda t: t[1]["tolerance"])
        finite = [s["tolerance"] for _, s in rank if np.isfinite(s["tolerance"])]
        scale = max(finite) if finite else 1.0
        p.append('<div class="tablewrap"><table><thead><tr><th>parameter</th>'
                 '<th>tolerance</th><th>first metric to move</th>'
                 '<th>elasticity (score)</th><th></th></tr></thead><tbody>')
        for g, s in rank:
            tol = s["tolerance"]
            bar = 0 if not np.isfinite(tol) else max(2, 150 * (1 - min(tol / scale, 1)))
            el = s["elasticity"].get("score")
            p.append(
                f'<tr><td>{html.escape(g)}</td>'
                f'<td>{"never" if not np.isfinite(tol) else f"{tol:.2f}×"}</td>'
                f'<td>{html.escape(s["driver"] or "—")}</td>'
                f'<td>{"—" if el is None else f"{el:+.3f}"}</td>'
                f'<td><span class="bar" style="width:{bar:.0f}px"></span></td></tr>')
        p.append('</tbody></table></div>')

        p.append('<h3>Stability limits</h3>')
        p.append('<p>Green marks the surviving interval; the dot is the '
                 'reference, ticks are the search-box bounds. An arm that never '
                 'broke is said so, never left blank.</p>')
        p.append(limits_svg(any_case))

        for metric, label in PANEL_METRICS:
            p.append(f'<h3>{html.escape(label)}</h3><div class="panels">')
            for g in genes:
                p.append(curve_panel(by_mm, g, metric, label))
            p.append('</div>')

    p.append('</div>')
    return "".join(p)


def main() -> int:
    ap = argparse.ArgumentParser(description="Plot the ablation gradient")
    ap.add_argument("--root", type=Path, default=RESULTS / "ablation")
    ap.add_argument("--out", type=Path, default=None)
    ap.add_argument("--no-html", action="store_true")
    args = ap.parse_args()

    cases = []
    for ref in sorted(Path(args.root).glob("*/mm_*/reference.json")):
        case = load_case(ref.parent)
        if case:
            cases.append(case)
    if not cases:
        print(f"no ablation results under {args.root}")
        return 1

    print_summary(cases)
    if args.no_html:
        return 0
    out = args.out or (Path(args.root) / "ablation_report.html")
    page = ("<!doctype html><html><head><meta charset='utf-8'>"
            "<meta name='viewport' content='width=device-width,initial-scale=1'>"
            "<title>Parameter ablation</title><style>" + CSS + "</style></head>"
            "<body>" + render_html(cases) + "</body></html>")
    out.write_text(page)
    print(f"\n  wrote {out}   ({sum(len(c['rows']) for c in cases)} runs, "
          f"{len(cases)} cases)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
