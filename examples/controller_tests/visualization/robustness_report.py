#!/usr/bin/env python3
"""
robustness_report.py — read the robustness archives, classify them, plot them.

Produces two things from the same data, so the picture and the numbers cannot
disagree:

    a terminal summary   classification per configuration, readable over ssh
    report.html          self-contained page: status grid, response curves,
                         and the full run table

Classification comes from examples/utils/robustness.py — the same
``_failed`` used to build breakdown.csv — rather than a second copy of the
rules. A plot that classified runs differently from the archive would be worse
than no plot.

Plain python: numpy + the stdlib. No Isaac Sim, no plotting library, no network.
The page embeds its own SVG, so it opens offline and survives being emailed.

Usage
─────
    python3 visualization/robustness_report.py
    python3 visualization/robustness_report.py --root results/robustness
    python3 visualization/robustness_report.py --out /tmp/report.html
"""
from __future__ import annotations

import argparse
import csv
import html
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "utils"))
import robustness as rb  # noqa: E402
import sweep_metrics as sm  # noqa: E402

RESULTS = Path(__file__).resolve().parents[1] / "results"

GAIN_COLUMNS = ["wn_mdyn", "b_mdyn", "Kp_att", "Kd_att", "Kd_yaw_rate",
                "mpc_q", "mpc_r"]
ORIGINS = {"inertia": 1.0, "mass": 1.0, "lag": 0.0, "wind": 0.0}

# Axis labels, so a reader never has to guess what a magnitude means.
AXIS_UNITS = {
    "inertia": ("gamma (plant inertia / modelled)", "x"),
    "mass":    ("gamma (plant mass / modelled)", "x"),
    "lag":     ("position sensor delay", " ms"),
    "wind":    ("wind speed", " m/s"),
}

# Categorical slots 1-3 of the reference palette, unchanged and in documented
# order. That file records this three-slot subset as clearing the all-pairs
# gates in both modes (CVD dE 9.2 light / 9.4 dark, normal-vision 24.0 / 20.9),
# which is the case that applies here — configs are compared point-by-point,
# not just as adjacent stacked bands. Slot 3 (aqua) sits below 3:1 on the light
# surface, so the relief rule applies: every series is direct-labelled at its
# last point AND the full table view ships below the charts.
SERIES_LIGHT = ["#2a78d6", "#eb6834", "#1baf7a"]
SERIES_DARK = ["#3987e5", "#d95926", "#199e70"]

# Status palette — fixed, never themed. Always paired with a glyph and a word,
# so a failure mode never depends on colour alone.
STATUS = {
    "":         ("good",     "#0ca30c", "ok",    "●"),
    "effort":   ("warning",  "#fab219", "eff",   "▲"),
    "tracking": ("serious",  "#ec835a", "trk",   "■"),
    "crash":    ("critical", "#d03b3b", "crash", "✕"),
}


def status_for(mode: str) -> tuple:
    """Glyph/colour for a failure mode, including combined ones like crash+effort."""
    if not mode:
        return STATUS[""]
    for key in ("crash", "tracking", "effort"):     # worst wins
        if key in mode:
            return STATUS[key]
    return STATUS[""]


# ── Loading and classification ───────────────────────────────────────────────

def load(root: Path) -> list:
    rows = []
    for archive in sorted(Path(root).rglob("robustness_archive.csv")):
        with open(archive, newline="") as f:
            reader = csv.DictReader(f)
            fields = reader.fieldnames or []
            key = (["config_id"] if "config_id" in fields
                   else [c for c in GAIN_COLUMNS if c in fields])
            for raw in reader:
                raw["_config"] = "|".join(str(raw.get(k, "")) for k in key)
                raw["_archive"] = str(archive)
                rows.append(raw)
    return rows


def as_float(value, default=float("nan")) -> float:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return default
    return out


def build_groups(rows: list) -> list:
    """One entry per (controller, axis, km, config), magnitudes sorted."""
    buckets = {}
    for row in rows:
        key = (row["controller"], row["axis"], row["motor_moment"], row["_config"])
        buckets.setdefault(key, []).append(row)

    groups = []
    for key, runs in sorted(buckets.items(), key=lambda kv: (
            kv[0][0], kv[0][1], as_float(kv[0][2]), kv[0][3])):
        controller, axis, km, config = key
        origin = ORIGINS.get(axis.split("-")[0], 0.0)
        nominal = [r for r in runs if as_float(r["magnitude"]) == origin]
        nominal_effort = (float(np.median([as_float(r["effort_ac"]) for r in nominal]))
                          if nominal else float("nan"))
        nominal_score = (min(as_float(r["score"]) for r in nominal)
                         if nominal else float("nan"))

        points = []
        for run in sorted(runs, key=lambda r: as_float(r["magnitude"])):
            effort = as_float(run["effort_ac"])
            ratio = (effort / nominal_effort
                     if np.isfinite(nominal_effort) and nominal_effort > 0
                     else float("nan"))
            points.append({
                "magnitude": as_float(run["magnitude"]),
                "score": as_float(run["score"]),
                "effort": effort,
                "ratio": ratio,
                "mode": rb._failed(run, nominal_effort),
                "rank": run.get("config_rank", ""),
            })
        gains = {g: run.get(g, "") for g in GAIN_COLUMNS
                 if runs[0].get(g) not in (None, "")}
        # Human-readable identity. The raw _config key is a join over gain
        # columns, which for MPC renders as "|||||1.0|1.0" — every column the
        # controller does not use contributes an empty field.
        label = " ".join(f"{k.replace('mpc_', '')}={as_float(v):g}"
                         for k, v in gains.items()) or "default"
        groups.append({
            "controller": controller, "axis": axis, "km": as_float(km),
            "config": config, "label": label, "gains": gains, "points": points,
            "nominal_score": nominal_score, "nominal_effort": nominal_effort,
            "origin": origin,
            "modes": sorted({p["mode"] for p in points if p["mode"]}),
        })
    return groups


# ── Terminal summary ─────────────────────────────────────────────────────────

def print_summary(groups: list) -> None:
    by_axis = {}
    for g in groups:
        by_axis.setdefault(g["axis"], []).append(g)

    for axis, entries in sorted(by_axis.items()):
        label, unit = AXIS_UNITS.get(axis.split("-")[0], (axis, ""))
        print(f"\n{'=' * 78}\n  AXIS: {axis}   ({label})\n{'=' * 78}")
        controllers = sorted({g["controller"] for g in entries})
        for controller in controllers:
            subset = [g for g in entries if g["controller"] == controller]
            broke = [g for g in subset if g["modes"]]
            print(f"\n  {controller.upper():<6s} {len(subset)} configs, "
                  f"{len(broke)} with a breakdown")
            for g in subset:
                marks = []
                for p in g["points"]:
                    _, _, short, glyph = status_for(p["mode"])
                    marks.append(f"{p['magnitude']:g}{glyph}")
                mode = "+".join(g["modes"]) if g["modes"] else "none"
                worst = max((p["ratio"] for p in g["points"]
                             if np.isfinite(p["ratio"])), default=float("nan"))
                print(f"    km={g['km']:<9.1e} cfg={g["label"][:34]:<34s} "
                      f"{mode:<14s} worst effort {worst:6.0f}x")
                print(f"        {'  '.join(marks)}")
    print(f"\n  legend: {'  '.join(f'{v[3]} {v[2]}' for v in STATUS.values())}\n")


# ── SVG helpers ──────────────────────────────────────────────────────────────

def log_scale(values, lo=None, hi=None):
    finite = [v for v in values if np.isfinite(v) and v > 0]
    lo = lo if lo is not None else min(finite, default=1.0)
    hi = hi if hi is not None else max(finite, default=10.0)
    lo, hi = min(lo, hi * 0.999), max(hi, lo * 1.001)
    return math.log10(lo), math.log10(hi)


def nice_log_ticks(lo_exp: float, hi_exp: float, limit: int = 6) -> list:
    ticks = []
    for e in range(math.floor(lo_exp), math.ceil(hi_exp) + 1):
        for m in (1, 3):
            v = m * 10.0 ** e
            if 10 ** lo_exp * 0.999 <= v <= 10 ** hi_exp * 1.001:
                ticks.append(v)
    if len(ticks) > limit:
        ticks = ticks[:: max(1, len(ticks) // limit)]
    return ticks


def fmt_tick(v: float) -> str:
    if v >= 1000:
        return f"{v:.0f}"
    if v >= 1:
        return f"{v:g}"
    return f"{v:g}"


def panel_svg(group_set: list, metric: str, title: str, threshold: float,
              threshold_label: str, axis: str, width=340, height=210) -> str:
    """One small multiple: metric vs magnitude, one line per config."""
    pad_l, pad_r, pad_t, pad_b = 46, 24, 26, 34
    plot_w, plot_h = width - pad_l - pad_r, height - pad_t - pad_b

    xs = [p["magnitude"] for g in group_set for p in g["points"]]
    ys = [p[metric] for g in group_set for p in g["points"]]
    ys = [y for y in ys if np.isfinite(y) and y > 0] + [threshold]
    if not xs or not ys:
        return ""

    # X scale follows the axis, not a house style. The scale axes (inertia,
    # mass) are multiplicative about a matched value of 1.0, so log spacing is
    # what makes 0.5x and 2x read as equally far from matched. The additive
    # axes (lag, wind) start at 0 — which has no logarithm at all — and their
    # magnitudes are already evenly spaced, so they get a linear scale.
    origin = ORIGINS.get(axis.split("-")[0], 0.0)
    x_log = origin > 0 and min(xs) > 0
    if x_log:
        x_lo, x_hi = log_scale(xs)

        def sx(v):
            return pad_l + (math.log10(v) - x_lo) / max(x_hi - x_lo, 1e-9) * plot_w
    else:
        x_lo, x_hi = min(xs), max(xs)
        span = max(x_hi - x_lo, 1e-9)

        def sx(v):
            return pad_l + (v - x_lo) / span * plot_w

    y_lo, y_hi = log_scale(ys)
    y_lo, y_hi = y_lo - 0.08, y_hi + 0.12

    def sy(v):
        v = max(v, 10 ** y_lo)
        return pad_t + plot_h - (math.log10(v) - y_lo) / max(y_hi - y_lo, 1e-9) * plot_h

    out = [f'<svg viewBox="0 0 {width} {height}" role="img" '
           f'aria-label="{html.escape(title)}" class="panel">']
    out.append(f'<text x="{pad_l}" y="14" class="panel-title">{html.escape(title)}</text>')

    # Grid + y ticks
    for v in nice_log_ticks(y_lo, y_hi):
        y = sy(v)
        out.append(f'<line x1="{pad_l}" y1="{y:.1f}" x2="{width-pad_r}" y2="{y:.1f}" class="grid"/>')
        out.append(f'<text x="{pad_l-6}" y="{y+3.5:.1f}" class="tick tick-y">{fmt_tick(v)}</text>')
    # x ticks at the actual sampled magnitudes
    for v in sorted(set(xs)):
        x = sx(v)
        out.append(f'<text x="{x:.1f}" y="{height-pad_b+14:.1f}" class="tick tick-x">{v:g}</text>')

    # Matched-condition marker and the breakdown threshold
    if origin > 0 and min(xs) <= origin <= max(xs):
        out.append(f'<line x1="{sx(origin):.1f}" y1="{pad_t}" x2="{sx(origin):.1f}" '
                   f'y2="{pad_t+plot_h}" class="origin-line"/>')
        out.append(f'<text x="{sx(origin):.1f}" y="{pad_t-4:.1f}" class="anno anno-mid">matched</text>')
    ty = sy(threshold)
    out.append(f'<line x1="{pad_l}" y1="{ty:.1f}" x2="{width-pad_r}" y2="{ty:.1f}" class="thresh"/>')
    out.append(f'<text x="{width-pad_r}" y="{ty-4:.1f}" class="anno anno-end">{html.escape(threshold_label)}</text>')

    for i, g in enumerate(group_set[:3]):
        pts = [(p["magnitude"], p[metric], p) for p in g["points"]
               if np.isfinite(p[metric]) and p[metric] > 0]
        if not pts:
            continue
        d = " ".join(("M" if k == 0 else "L") + f"{sx(x):.1f},{sy(y):.1f}"
                     for k, (x, y, _) in enumerate(pts))
        out.append(f'<path d="{d}" class="series s{i}"/>')
        for x, y, p in pts:
            _, colour, short, _ = status_for(p["mode"])
            cls = "mark-fail" if p["mode"] else "mark"
            extra = f' style="fill:{colour}"' if p["mode"] else ""
            tip = (f'{g["controller"]} cfg {g["label"][:28]} | '
                   f'{AXIS_UNITS.get(axis.split("-")[0], ("x",""))[0]} {x:g} | '
                   f'{metric} {y:.4g}' + (f' | {short}' if p["mode"] else ''))
            out.append(f'<circle cx="{sx(x):.1f}" cy="{sy(y):.1f}" r="4" '
                       f'class="{cls} s{i}"{extra}><title>{html.escape(tip)}</title></circle>')
        # Direct label at the last point — the relief rule for the light surface.
        lx, ly, _ = pts[-1]
        out.append(f'<text x="{sx(lx)+7:.1f}" y="{sy(ly)+3:.1f}" '
                   f'class="direct s{i}">{i}</text>')

    out.append('</svg>')
    return "".join(out)


def status_grid_svg(groups: list, axis: str) -> str:
    """Classification at a glance: one row per config, one cell per magnitude."""
    entries = [g for g in groups if g["axis"] == axis]
    if not entries:
        return ""
    mags = sorted({p["magnitude"] for g in entries for p in g["points"]})
    cell, label_w, head_h = 34, 210, 26
    width = label_w + cell * len(mags) + 12
    height = head_h + cell * len(entries) + 10

    out = [f'<svg viewBox="0 0 {width} {height}" role="img" '
           f'aria-label="Failure classification for the {html.escape(axis)} axis" '
           f'class="grid-svg">']
    for j, m in enumerate(mags):
        out.append(f'<text x="{label_w + cell*j + cell/2:.1f}" y="{head_h-9}" '
                   f'class="tick tick-mid">{m:g}</text>')
    for i, g in enumerate(entries):
        y = head_h + cell * i
        name = f'{g["controller"]} km={g["km"]:.1e} #{g["config"][:10]}'
        out.append(f'<text x="0" y="{y + cell/2 + 4:.1f}" class="row-label">{html.escape(name)}</text>')
        for j, m in enumerate(mags):
            p = next((q for q in g["points"] if q["magnitude"] == m), None)
            x = label_w + cell * j
            if p is None:
                continue
            role, colour, short, glyph = status_for(p["mode"])
            tip = (f'{g["controller"]} | km={g["km"]:.1e} | x={m:g} | '
                   f'score {p["score"]:.4f} | effort {p["effort"]:.3f} '
                   f'({p["ratio"]:.1f}x) | {short}')
            out.append(
                f'<rect x="{x+2}" y="{y+2}" width="{cell-4}" height="{cell-4}" rx="4" '
                f'fill="{colour}" opacity="{0.22 if not p["mode"] else 0.85}" '
                f'class="cell"><title>{html.escape(tip)}</title></rect>')
            out.append(f'<text x="{x+cell/2:.1f}" y="{y+cell/2+4:.1f}" '
                       f'class="cell-glyph">{glyph}</text>')
    out.append('</svg>')
    return "".join(out)


# ── Page ─────────────────────────────────────────────────────────────────────

CSS = """
:root{color-scheme:light dark}
.viz-root{
  --surface-1:#fcfcfb; --surface-2:#f4f3f0;
  --text-primary:#0b0b0b; --text-secondary:#52514e; --text-muted:#78766f;
  --rule:#dcdbd6;
  --s0:#2a78d6; --s1:#eb6834; --s2:#1baf7a;
}
@media (prefers-color-scheme:dark){:root:where(:not([data-theme="light"])) .viz-root{
  --surface-1:#1a1a19; --surface-2:#232320;
  --text-primary:#fff; --text-secondary:#c3c2b7; --text-muted:#96958c;
  --rule:#3a3a36;
  --s0:#3987e5; --s1:#d95926; --s2:#199e70;
}}
:root[data-theme="dark"] .viz-root{
  --surface-1:#1a1a19; --surface-2:#232320;
  --text-primary:#fff; --text-secondary:#c3c2b7; --text-muted:#96958c;
  --rule:#3a3a36;
  --s0:#3987e5; --s1:#d95926; --s2:#199e70;
}
body{margin:0;background:var(--surface-1)}
.viz-root{background:var(--surface-1);color:var(--text-primary);
  font:14px/1.55 ui-sans-serif,system-ui,-apple-system,"Segoe UI",sans-serif;
  padding:28px clamp(14px,4vw,48px);max-width:1180px;margin:0 auto}
h1{font-size:1.5rem;margin:0 0 2px;letter-spacing:-.01em}
h2{font-size:1.05rem;margin:34px 0 4px;letter-spacing:-.005em}
h3{font-size:.9rem;margin:20px 0 6px;color:var(--text-secondary);font-weight:600}
p{color:var(--text-secondary);margin:6px 0 12px;max-width:74ch}
code{background:var(--surface-2);padding:1px 5px;border-radius:4px;font-size:.86em}
pre{background:var(--surface-2);padding:12px 14px;border-radius:8px;overflow-x:auto;
  font-size:.82rem;line-height:1.5;color:var(--text-primary)}
.panels{display:grid;gap:10px;grid-template-columns:repeat(auto-fill,minmax(320px,1fr))}
.panel{width:100%;height:auto;background:var(--surface-2);border-radius:8px}
.panel-title{fill:var(--text-primary);font-size:11px;font-weight:600}
.grid{stroke:var(--rule);stroke-width:1}
.tick{fill:var(--text-muted);font-size:9.5px}
.tick-y{text-anchor:end}.tick-x,.tick-mid{text-anchor:middle}
.anno{fill:var(--text-muted);font-size:9px}
.anno-mid{text-anchor:middle}.anno-end{text-anchor:end}
.origin-line{stroke:var(--text-muted);stroke-width:1;stroke-dasharray:2 3;opacity:.7}
.thresh{stroke:var(--text-secondary);stroke-width:1;stroke-dasharray:5 3;opacity:.55}
.series{fill:none;stroke-width:2;stroke-linejoin:round;stroke-linecap:round}
.mark{stroke:var(--surface-2);stroke-width:2}
.mark-fail{stroke:var(--surface-2);stroke-width:2}
.direct{font-size:10px;font-weight:700}
.s0{stroke:var(--s0)}.s0.mark,.s0.direct{fill:var(--s0)}
.s1{stroke:var(--s1)}.s1.mark,.s1.direct{fill:var(--s1)}
.s2{stroke:var(--s2)}.s2.mark,.s2.direct{fill:var(--s2)}
.grid-svg{width:100%;height:auto;background:var(--surface-2);border-radius:8px;padding:8px 0}
.row-label{fill:var(--text-secondary);font-size:10.5px}
.cell-glyph{fill:var(--text-primary);font-size:12px;text-anchor:middle;pointer-events:none}
.cell{cursor:default}
.legend{display:flex;flex-wrap:wrap;gap:14px;margin:10px 0 4px;
  color:var(--text-secondary);font-size:.83rem;align-items:center}
.key{display:inline-flex;align-items:center;gap:6px}
.dot{width:11px;height:11px;border-radius:3px;display:inline-block}
.swatch{width:16px;height:3px;border-radius:2px;display:inline-block}
.tablewrap{overflow-x:auto;border:1px solid var(--rule);border-radius:8px;margin-top:8px}
table{border-collapse:collapse;width:100%;font-size:.8rem;min-width:720px}
th,td{padding:5px 9px;text-align:right;white-space:nowrap;
  border-bottom:1px solid var(--rule)}
th{background:var(--surface-2);color:var(--text-secondary);font-weight:600;
  position:sticky;top:0;text-align:right}
td:first-child,th:first-child,td:nth-child(2),th:nth-child(2){text-align:left}
tr:last-child td{border-bottom:none}
.bad{color:#d03b3b;font-weight:600}
.note{border-left:3px solid var(--rule);padding:2px 0 2px 12px;margin:12px 0;
  color:var(--text-secondary);font-size:.88rem}
"""


def render_html(groups: list) -> str:
    axes = sorted({g["axis"] for g in groups})
    parts = ['<div class="viz-root"><h1>Controller robustness</h1>',
             '<p>How far each tuned configuration can be pushed before it stops '
             'flying. Generated from <code>robustness_archive.csv</code> by '
             '<code>visualization/robustness_report.py</code>; runs are classified by the '
             'same rules that build <code>breakdown.csv</code>.</p>']

    parts.append('<div class="legend"><strong style="color:var(--text-primary)">Failure mode</strong>')
    for mode, (role, colour, short, glyph) in STATUS.items():
        name = {"": "no breakdown", "effort": "effort spike",
                "tracking": "tracking loss", "crash": "crash / no takeoff"}[mode]
        parts.append(f'<span class="key"><span class="dot" style="background:{colour}"></span>'
                     f'{glyph} {html.escape(name)}</span>')
    parts.append('</div>')
    parts.append(f'<div class="note">A run is a breakdown if it crashed, if its stability '
                 f'score exceeded {sm.STABLE_SCORE_MAX:g}, or if its differential effort '
                 f'exceeded {rb.EFFORT_BREAKDOWN_RATIO:g}x the same configuration’s '
                 f'effort at the matched condition. The three are independent because the '
                 f'controllers do not fail the same way.</div>')

    for axis in axes:
        entries = [g for g in groups if g["axis"] == axis]
        label, unit = AXIS_UNITS.get(axis.split("-")[0], (axis, ""))
        parts.append(f'<h2>{html.escape(axis)} &mdash; {html.escape(label)}</h2>')

        parts.append('<h3>Classification</h3>')
        parts.append('<p>One row per configuration, one cell per perturbation '
                     'magnitude. Hover any cell for its numbers.</p>')
        parts.append(status_grid_svg(entries, axis))

        controllers = sorted({g["controller"] for g in entries})
        for metric, title, thresh, tlabel in (
                ("ratio", "Effort ratio vs matched", rb.EFFORT_BREAKDOWN_RATIO,
                 f"{rb.EFFORT_BREAKDOWN_RATIO:g}x breakdown"),
                ("score", "Stability score", sm.STABLE_SCORE_MAX,
                 f"score {sm.STABLE_SCORE_MAX:g}")):
            parts.append(f'<h3>{html.escape(title)}</h3>')
            parts.append('<div class="legend">')
            for i in range(3):
                parts.append(f'<span class="key"><span class="swatch" '
                             f'style="background:var(--s{i})"></span>config {i}</span>')
            parts.append('</div><div class="panels">')
            for controller in controllers:
                for km in sorted({g["km"] for g in entries if g["controller"] == controller}):
                    subset = [g for g in entries
                              if g["controller"] == controller and g["km"] == km]
                    parts.append(panel_svg(
                        subset, metric, f"{controller}  km={km:.1e}",
                        thresh, tlabel, axis))
            parts.append('</div>')

    # Table view — required so identity never rests on colour alone.
    parts.append('<h2>All runs</h2>')
    parts.append('<div class="tablewrap"><table><thead><tr>'
                 '<th>controller</th><th>config</th><th>km</th><th>magnitude</th>'
                 '<th>score</th><th>effort</th><th>ratio</th><th>mode</th>'
                 '</tr></thead><tbody>')
    for g in groups:
        for p in g["points"]:
            _, _, short, _ = status_for(p["mode"])
            bad = ' class="bad"' if p["mode"] else ''
            parts.append(
                f'<tr><td>{html.escape(g["controller"])}</td>'
                f'<td>{html.escape(g["label"][:40])}</td>'
                f'<td>{g["km"]:.1e}</td><td>{p["magnitude"]:g}</td>'
                f'<td>{p["score"]:.4f}</td><td>{p["effort"]:.3f}</td>'
                f'<td{bad}>{p["ratio"]:.1f}x</td>'
                f'<td{bad}>{html.escape(short if p["mode"] else "ok")}</td></tr>')
    parts.append('</tbody></table></div></div>')
    return "".join(parts)


def main() -> int:
    p = argparse.ArgumentParser(description="Plot and classify robustness archives")
    p.add_argument("--root", type=Path, default=RESULTS / "robustness")
    p.add_argument("--out", type=Path, default=None)
    p.add_argument("--no-html", action="store_true")
    args = p.parse_args()

    rows = load(args.root)
    if not rows:
        print(f"no robustness_archive.csv under {args.root}")
        return 1
    groups = build_groups(rows)
    print_summary(groups)

    if args.no_html:
        return 0
    out = args.out or (Path(args.root) / "report.html")
    page = ("<!doctype html><html><head><meta charset='utf-8'>"
            "<meta name='viewport' content='width=device-width,initial-scale=1'>"
            "<title>Controller robustness</title><style>" + CSS + "</style></head>"
            "<body>" + render_html(groups) + "</body></html>")
    out.write_text(page)
    print(f"  wrote {out}   ({len(rows)} runs, {len(groups)} configurations)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
