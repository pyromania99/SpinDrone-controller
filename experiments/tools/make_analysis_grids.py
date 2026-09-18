#!/usr/bin/env python3
"""
make_analysis_grids.py - tile the per-experiment figures into side-by-side comparison
sheets under ../analysis/.

Nothing here reads a ulog. Every tile is a PNG already committed in an experiment folder,
pasted unmodified (only scaled) into a labelled grid, so a sheet can never disagree with
the experiment it came from. Regenerate a tile by re-running the command in that
experiment's RUN_LOG, then re-run this.

    <view>_by_level.png          the default: the rungs side by side at the lowest delay
    <view>_by_level_and_lag.png  --all-conditions: rows are the rung, columns the delay

THE COLUMN LABELS ARE THE POINT
    The experiment figures are named `nolag` and `lag20ms`, which says how much *rotor*
    pole was modelled and says nothing about the rest of the loop. Those names are kept
    everywhere - they are what the files are called - but PX4 carries ~17 ms of transport
    delay plus ~14 ms of pipeline filtering whatever the rotor model says, so the columns
    here are read as ~17 ms and ~37 ms of total delay. Level 1 has no PX4 in it at all, so
    its 20 ms rotor pole really is the whole delay and it is NOT the same condition as the
    level-2/3 column it sits in. Each tile caption says which.

Usage:
    python3 experiments/tools/make_analysis_grids.py [--out experiments/analysis]
    python3 experiments/tools/make_analysis_grids.py --tile-width 900   # smaller sheets
"""
from __future__ import annotations

import argparse
import os
import textwrap

from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
EXP = os.path.dirname(HERE)

L1 = "hover_level1_km6e7_seat_off_fixed_adaptive"
H2 = "hover_level2_px4_groundtruth"
H3 = "hover_level3_px4_estimator"
C2 = "trajectory_level2_px4_groundtruth_fast_circle"
C3 = "trajectory_level3_px4_estimator_fast_circle"
C1 = "trajectory_level1_circle_seat"

FONT_DIR = "/usr/share/fonts/truetype/dejavu"
BG = (255, 255, 255)
INK = (24, 24, 24)
MUTED = (110, 110, 110)
ROW_BAND = (238, 241, 245)
COL_BAND = (248, 249, 251)
BORDER = (200, 204, 210)
MISSING_BG = (246, 246, 246)

VIEWS = {
    "compare": "flight comparison — altitude, |p,q|, attitude error, tilt vs commanded, position error, torque, spin + θ_s",
    "attitude": "attitude vs time — roll / pitch / yaw and the roll–pitch phase portrait",
    "seat": "θ_s on its own axes, with α and θ_b",
    "trajectory": "commanded vs flown path, following distance, speed, altitude",
    "attitude_adaptive": "the adaptive arm alone, on its own axes",
}


# ------------------------------------------------------------------ the default sheets
#
# One condition only - the `nolag` arms, i.e. ~17 ms of PX4 pipeline and no rotor pole -
# with level 2 and level 3 as the two COLUMNS, so the rung comparison is horizontal. The
# wider grids over lag and spin are still built by --all-conditions.

FOCUS_SUB = {
    "circle_l3_envelope":
        "Columns are conditions, not rungs — every cell is level 3 on the identical circle.\n"
        "Score these on FOLLOWING DISTANCE, not airborne time: the 20 ms seat-ON arm stays "
        "up longer than seat-OFF while ending 15 m off the path.",
    "hover": "Columns are the rung; the single row is the condition. Only the lowest-delay "
             "arm of each rung is shown — everything else is behind --all-conditions.\n"
             "Ground truth is the source for every trace at every rung: scoring level 3's "
             "estimator against its own estimate would be circular.",
    "circle": "Columns are the rung; the single row is the condition. Only the `nolag` arms "
              "are shown — everything else is behind --all-conditions.\n"
              "SPIN IS NOT MATCHED BETWEEN THE COLUMNS (r ≈ 20.7 vs r ≈ 25), so a level-2 / "
              "level-3 difference here confounds the rung with the spin rate.",
}

L1_COL = ("level 1 — Pegasus + the Python eigen law, NO PX4\n"
          "20 ms rotor pole and nothing else")
L2_COL = "level 2 — PX4 + Pegasus on simulator GROUND TRUTH  (MC_CTRL_GT=1)"
L3_COL = ("level 3 — PX4 + Pegasus on PX4's OWN ESTIMATOR  (MC_CTRL_GT=0)\n"
          "the hardware-relevant rung")
LEVEL_COLS = [L2_COL, L3_COL]


def hover_focus(view):
    """Level 1, 2 and 3 at ~the same total loop delay.

    Level 1 belongs on this sheet even though it has no `nolag` arm, because at r ≈ 29.5 its
    20 ms rotor pole and the PX4 rungs' transport+pipeline lag come to the same phase to
    within about 6%:  atan(r·T) = atan(29.5 × 0.020) = 0.533 rad against r·tau = 29.5 ×
    0.017 = 0.50 rad. Same size, different mechanism - a first-order pole, not a pure delay
    - which is why the caption says so on the tile.
    """
    row = ("~17-20 ms of total loop delay  ·  r ≈ 29.5  ·  climb and hold, no trajectory\n"
           "level 1: a 20 ms rotor pole, no PX4.  levels 2-3: the `nolag` arms — "
           "rotor_tau = 0, so PX4's ~17 ms pipeline and no rotor pole.")
    l2 = (f"{H2}/l2_hover_nolag_{view}.png",
          "r ≈ 29.4, hold 6 m. Both arms fly the whole run; neither is rescued at this delay.\n"
          "θ_s converges to −0.860 — PX4's own pipeline, found with no rotor pole present.\n"
          "alt sd 1.6 mm. α (circular mean) +1.04 rad at R 0.92 with the seat off;\n"
          "with the seat on it moves to −2.94 at R 0.37 — it does NOT shrink. See the README.")
    l3 = (f"{H3}/l3_hover_nolag_{view}.png",
          "r ≈ 29.4, hold 6 m. Both arms fly. θ_s → −0.782, 9% off level 2's −0.860.\n"
          "~5× noisier than level 2 on the same profile (alt sd 7.3 mm vs 1.6 mm)\n"
          "— that is the estimator in the loop, and it is the expected direction.")
    l1 = (f"{L1}/km6e7_refly_{view}.png",
          "r ≈ 29.5, hold 1.5 m, 120 s. THREE arms: seat OFF / FIXED at ψ_model / ADAPTIVE.\n"
          "OFF departs at 24.0 s and keeps re-departing · FIXED crashes ~4.4 s · "
          "ADAPTIVE flies the full 120 s.\n"
          "θ_s → 0.067 rad. Comparable in total phase to the two on the right "
          "(0.533 rad of pole vs ~0.50 rad\nof delay at this spin), NOT in mechanism: "
          "a first-order pole is not a pure transport delay.")
    return [L1_COL, L2_COL, L3_COL], [(row, [l1, l2, l3])]


def circle_focus(view):
    row = ("R = 10 m, 5 m/s circle, the same path at all three rungs  ·  lowest delay each rung has\n"
           "level 1: a 20 ms rotor pole, no PX4, r ≈ 25.  levels 2-3: the `nolag` arms (rotor_tau = 0, "
           "~17 ms of PX4 pipeline), r ≈ 20.7 and r ≈ 25.")
    l1 = (f"{C1}/l1_fast_circle_{view}.png",
          "r ≈ 25, accel clamp 3.0. BOTH ARMS FLY; this is a tracking result, not a rescue.\n"
          "Following distance 7.54 m (off) → 4.62 m (on); altitude hold ±473 mm → ±118 mm;\n"
          "tilt 16.61° → 11.79°. θ_s → +0.468. Flown 2026-09-18 — see that folder's TRAP\n"
          "section: without --max-lateral-accel (which defaults OFF) both arms crash instead.")
    return [L1_COL, L2_COL, L3_COL], [(row, [l1,
        (f"{C2}/l2_nolag_{view}.png",
         "r ≈ 20.7. Both arms fly the full ~52 s, 2 altitude crossings each, neither dips.\n"
         "θ_s → −0.401 with no rotor pole present. α falls +0.35 → −0.13 rad (R ≈ 0.8) with the seat on.\n"
         "RE-FLOWN 2026-09-18: the old run's seat-OFF double dip did NOT reproduce."),
        (f"{C3}/l3_nolag_{view}.png",
         "r ≈ 25 — NOT level 2's 20.7, so this pair confounds the rung with the spin rate.\n"
         "Three arms here: seat OFF / seat ON / seat + adaptive b. All three fly.\n"
         "θ_s −0.505 (ON) and −0.457 (ON+b); adding b moves tracking by less than the noise.\n"
         "α falls +0.45 → −0.13 rad (circular mean, R ≈ 0.8) with the seat on."),
    ])]



def l3_lag_spin(view):
    """Level 3, the same fast circle, three conditions - the lag/spin envelope on one row."""
    row = ("LEVEL 3 ONLY, R = 10 m 5 m/s circle, estimator in the loop  ·  "
           "left to right: raise the spin, then add a rotor pole")
    return ([ "r ≈ 25.9, NO rotor pole (~19.5 ms)\nboth arms fly — no rescue to see",
              "r ≈ 29.5, NO rotor pole (~19.5 ms)\nTHE TRAJECTORY RESCUE",
              "r ≈ 25.8, 20 ms rotor pole (~39.5 ms)\nneither arm survives" ],
        [(row, [
        (f"{C3}/l3_nolag_{view}.png",
         "Three arms: seat OFF / ON / ON+b, all fly the full ~52 s.\n"
         "follow 4.88 m (off) vs 5.01 m (on); att err 2.96° vs 4.13°.\n"
         "At this spin the seat slightly WORSENS tracking — there is nothing to rescue."),
        (f"{C3}/l3_r29_nolag_{view}.png",
         "seat OFF CRASHES at 35.4 s — 32.62 m off path, att err 39.27°.\n"
         "seat ON FLIES the full 52.2 s — follow 5.04 m, att err 4.48°, θ_s −0.552.\n"
         "Same spin, same path, only MC_SEAT_MODE differs. n=1 on both arms."),
        (f"{C3}/l3_lag20ms_{view}.png",
         "seat OFF crashes 13.9 s · seat ON crashes 22.4 s. The seat buys ~8 s and no more.\n"
         "A 20 ms pole adds amplitude DROOP (0.86 at this spin) on top of rotation, and\n"
         "the seat cancels rotation only. Slowing to 2 m/s does not help either."),
    ])])


# ------------------------------------------------------------------ the wider grids

def hover_spec(view):
    """(columns, rows) for a hover sheet. A cell is (png_path, caption) or (None, why)."""
    cols = [
        '"no lag"  (rotor_tau = 0)\nreads as ~17 ms: PX4 pipeline, no rotor pole',
        '"lag20ms"  (rotor_tau = 0.02)\nreads as ~37 ms: the same PX4 pipeline + a 20 ms rotor pole',
        '"r20, lag20ms"  (rotor_tau = 0.02)\nsame ~37 ms, lower spin',
    ]
    l1 = f"{L1}/km6e7_refly_{view}.png"
    l1_cap = ("≈20 ms — 20 ms rotor pole and NOTHING ELSE.\n"
              "Not the same condition as the cells below it: level 1 has no transport lag.\n"
              "Three arms: seat OFF / FIXED at ψ_model / ADAPTIVE.\n"
              "OFF departs at 24.0 s and keeps re-departing · FIXED crashes ~4.4 s · "
              "ADAPTIVE flies the full 120 s, θ_s → 0.067 rad.")
    rows = [
        ("level 1 — Pegasus + the Python eigen law, no PX4  (r ≈ 29.5)", [
            (None, 'no "no lag" column at level 1:\nthere is no PX4 pipeline to leave in'),
            (l1, l1_cap),
            (None, "the r ≈ 20 hover was not flown at level 1"),
        ]),
        ("level 2 — PX4 + Pegasus on simulator ground truth (MC_CTRL_GT=1)", [
            (f"{H2}/l2_hover_nolag_{view}.png",
             "r ≈ 29.4. Both arms fly the whole run; neither is rescued here.\n"
             "θ_s converges to −0.860 — PX4's own pipeline, found with no rotor pole present."),
            (f"{H2}/l2_hover_lag20ms_{view}.png",
             "r ≈ 29.5 (re-flown 2026-09-18). seat OFF crashes 11.9 s · seat ON crashes 21.4 s.\n"
             "The seat nearly doubles the time to departure but does not prevent it."),
            (f"{H2}/l2_hover_r20_lag20ms_{view}.png",
             "r ≈ 20.0 (the fast circle's spin), re-flown. seat OFF crashes 36.6 s · seat ON flies 46.8 s.\n"
             "Same rung and same delay as the middle column — only the spin is lower."),
        ]),
        ("level 3 — PX4 + Pegasus on PX4's own estimator (MC_CTRL_GT=0) — the hardware-relevant rung", [
            (f"{H3}/l3_hover_nolag_{view}.png",
             "r ≈ 29.4. Both arms fly. θ_s → −0.782, near level 2's −0.860 at the same condition.\n"
             "~5× noisier than level 2 on the same profile (alt sd 7.3 mm vs 1.6 mm) — that is the estimator."),
            (f"{H3}/l3_hover_lag20ms_{view}.png",
             "r ≈ 29.4. seat OFF crashes 12.7 s · seat ON flies the full 46 s. **n=5, all five flew**,\n"
             "θ_s −0.4731 ± 0.0009 (0.19%). THE DIVERGENCE: same condition as the cell above,\n"
             "opposite outcome on the seat-ON arm — and this is the tree's best-established result."),
            (f"{H3}/l3_hover_r20_lag20ms_{view}.png",
             "r ≈ 20, re-flown. Both survive ~47 s, but seat OFF does it inside a coning limit cycle:\n"
             "attitude error 13.40° against seat ON's 0.28° — 47.7× — at |p,q| 1.54 vs 0.088.\n"
             "Score attitude, not survival. The largest measured seat effect in the tree."),
        ]),
    ]
    return cols, rows


def circle_spec(view):
    cols = [
        '"no lag"  (rotor_tau = 0)\nreads as ~17 ms: PX4 pipeline, no rotor pole',
        '"lag20ms"  (rotor_tau = 0.02)\nreads as ~37 ms: the same PX4 pipeline + a 20 ms rotor pole',
    ]
    rows = [
        ("level 1 — Pegasus + the Python eigen law, no PX4", [
            (None, "level 1 never flew THIS circle.\n"
                   "Its trajectory work is a waypoint ellipse and a five-geometry sweep,\n"
                   "plotted with a different tool — see trajectory_level1_* .\n"
                   "Those figures are not tileable against these and are deliberately left out."),
            (None, "as left."),
        ]),
        ("level 2 — PX4 + Pegasus on simulator ground truth (MC_CTRL_GT=1)  ·  R = 10 m, 5 m/s circle, r ≈ 20.7", [
            (f"{C2}/l2_nolag_{view}.png",
             "Re-flown 2026-09-18: both arms fly ~52 s, 2 crossings each, neither dips.\n"
             "θ_s → −0.401. The old run's seat-OFF double dip did not reproduce."),
            (f"{C2}/l2_lag20ms_{view}.png",
             "RE-FLOWN 2026-09-18, and the old headline did not hold. seat OFF crashes 16.4 s ·\n"
             "seat ON crashes 19.9 s HERE, but flew in 1 of 4 repeats (see l2_lag20ms_repeats_*).\n"
             "The seat reliably buys time; the 48.7 s rescue reproduces once in four."),
        ]),
        ("level 3 — PX4 + Pegasus on PX4's own estimator (MC_CTRL_GT=0)  ·  same circle, r ≈ 25 (NOT level 2's 20.7)", [
            (f"{C3}/l3_nolag_{view}.png",
             "Three arms: seat OFF / seat ON / seat + adaptive b. All three fly.\n"
             "θ_s −0.505 (ON) and −0.457 (ON+b); adding b moves tracking by less than the noise."),
            (f"{C3}/l3_lag20ms_{view}.png",
             "NEITHER ARM SURVIVES, re-flown. seat OFF loses it at 13.9 s then bounces (8 crossings);\n"
             "seat ON crashes 22.4 s. The seat buys ~8 s and no more.\n"
             "This is the open item of the whole tree."),
        ]),
    ]
    return cols, rows


SHEETS = [
    ("hover", "Hover under sustained spin, r ≈ 29.5 — levels 1, 2 and 3 at ~17-20 ms",
     "Climb to 6 m and hold (1.5 m at level 1), no trajectory. Rows are the simulation rung, "
     "columns the loop delay - under the name the experiment files use, and the total it really is.\n"
     "Ground truth is the source for every trace, at all three levels — scoring level 3's "
     "estimator against its own estimate would be circular.",
     hover_spec, hover_focus,
     ["compare", "attitude", "seat", "trajectory", "attitude_adaptive"]),
    ("circle_l3_envelope",
     "Level 3 fast circle — how spin and rotor lag close the envelope",
     "All six arms are level 3 on the identical R = 10 m, 5 m/s circle with PX4's estimator "
     "in the loop.\nRaising the spin from 25.9 to 29.5 separates the arms; adding a 20 ms "
     "rotor pole kills both.",
     l3_lag_spin, l3_lag_spin, ["compare", "trajectory", "attitude", "seat"]),
    ("circle", "Fast circle (R = 10 m, 5 m/s) — level 2 vs level 3 at ~17 ms",
     "Rows are the simulation rung, columns the loop delay - under the name the experiment "
     "files use, and the total it really is. "
     "SPIN IS NOT MATCHED BETWEEN THE ROWS: level 2 flew this circle at r ≈ 20.7 and level 3 "
     "at r ≈ 25,\nso a level-2/level-3 difference on this sheet confounds the rung with the "
     "spin rate. The hover sheet does not have that problem — its rows were flown back to "
     "back with only MC_CTRL_GT different.",
     circle_spec, circle_focus,
     ["compare", "attitude", "seat", "trajectory", "attitude_adaptive"]),
]


# ------------------------------------------------------------------ rendering

def font(size, bold=False):
    name = "DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf"
    path = os.path.join(FONT_DIR, name)
    try:
        return ImageFont.truetype(path, size)
    except OSError:
        return ImageFont.load_default()


def text_h(draw, s, fnt, width):
    """Height of `s` drawn as wrapped/broken lines inside `width`."""
    return len(_lines(draw, s, fnt, width)) * (fnt.size + 6)


def _lines(draw, s, fnt, width):
    out = []
    for para in s.split("\n"):
        if not para:
            out.append("")
            continue
        # character-count wrap is close enough for DejaVu at these sizes
        approx = max(int(width / (fnt.size * 0.55)), 12)
        out.extend(textwrap.wrap(para, approx) or [""])
    return out


def draw_text(draw, xy, s, fnt, width, fill=INK, center=False):
    x, y = xy
    for line in _lines(draw, s, fnt, width):
        w = draw.textlength(line, font=fnt)
        draw.text((x + (width - w) / 2 if center else x, y), line, font=fnt, fill=fill)
        y += fnt.size + 6
    return y


def build(view, title, subtitle, cols, rows, out_path, tile_w):
    f_title, f_sub = font(38, True), font(23)
    f_col, f_row, f_cap = font(25, True), font(25, True), font(19)

    probe = Image.new("RGB", (10, 10))
    pd = ImageDraw.Draw(probe)

    # every tile in a sheet is the same view, so one aspect ratio governs
    first = next((c[0] for r in rows for c in r[1] if c[0]), None)
    if first is None:
        return None
    with Image.open(os.path.join(EXP, first)) as im:
        aspect = im.height / im.width
    tile_h = int(tile_w * aspect)

    pad, gap = 28, 20
    ncol = len(cols)
    grid_w = ncol * tile_w + (ncol - 1) * gap
    W = grid_w + 2 * pad

    head_h = pad + text_h(pd, title, f_title, W - 2 * pad) + 10 \
        + text_h(pd, subtitle, f_sub, W - 2 * pad) + 6 \
        + text_h(pd, VIEWS[view], f_sub, W - 2 * pad) + 18
    colhead_h = max(text_h(pd, c, f_col, tile_w - 20) for c in cols) + 20

    row_geo = []
    for label, cells in rows:
        rh = text_h(pd, label, f_row, W - 2 * pad - 20) + 18
        # a row with nothing to show collapses to its note: two page-tall empty boxes say
        # no more than one sentence does, and they push the real rows off the screen.
        empty = not any(p for p, _c in cells)
        ch = (text_h(pd, cells[0][1], f_cap, W - 2 * pad - 20) + 12 if empty
              else max(text_h(pd, cap, f_cap, tile_w - 16) for _p, cap in cells) + 12)
        row_geo.append((rh, ch, 0 if empty else tile_h))
    H = head_h + colhead_h + sum(rh + ch + th + gap * 2 for rh, ch, th in row_geo) + pad

    img = Image.new("RGB", (W, H), BG)
    d = ImageDraw.Draw(img)

    y = pad
    y = draw_text(d, (pad, y), title, f_title, W - 2 * pad) + 10
    y = draw_text(d, (pad, y), subtitle, f_sub, W - 2 * pad, fill=MUTED) + 6
    y = draw_text(d, (pad, y), f"This sheet: {VIEWS[view]}", f_sub, W - 2 * pad, fill=INK) + 18

    d.rectangle([pad, y, W - pad, y + colhead_h], fill=COL_BAND)
    for j, c in enumerate(cols):
        x = pad + j * (tile_w + gap)
        draw_text(d, (x + 10, y + 10), c, f_col, tile_w - 20, center=True)
    y += colhead_h

    for (label, cells), (rh, ch, th) in zip(rows, row_geo):
        y += gap
        d.rectangle([pad, y, W - pad, y + rh], fill=ROW_BAND)
        draw_text(d, (pad + 10, y + 9), label, f_row, W - 2 * pad - 20)
        y += rh
        if th == 0:
            draw_text(d, (pad + 10, y + 6), cells[0][1], f_cap, W - 2 * pad - 20, fill=MUTED)
            y += ch
            continue
        for j, (path, cap) in enumerate(cells):
            x = pad + j * (tile_w + gap)
            draw_text(d, (x + 8, y + 6), cap, f_cap, tile_w - 16,
                      fill=INK if path else MUTED)
            ty = y + ch
            if path and os.path.exists(os.path.join(EXP, path)):
                with Image.open(os.path.join(EXP, path)) as im:
                    img.paste(im.convert("RGB").resize((tile_w, tile_h), Image.LANCZOS), (x, ty))
                d.rectangle([x, ty, x + tile_w - 1, ty + tile_h - 1], outline=BORDER)
                draw_text(d, (x + 8, ty + tile_h + 3), path, font(15), tile_w - 16, fill=MUTED)
            else:
                d.rectangle([x, ty, x + tile_w - 1, ty + tile_h - 1],
                            fill=MISSING_BG, outline=BORDER)
                msg = "no figure" if path else "not flown"
                w = d.textlength(msg, font=f_col)
                d.text((x + (tile_w - w) / 2, ty + tile_h / 2), msg, font=f_col, fill=MUTED)
        y += ch + th
    img.save(out_path, optimize=True)
    return out_path


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default=os.path.join(EXP, "analysis"))
    ap.add_argument("--tile-width", type=int, default=1150)
    ap.add_argument("--all-conditions", action="store_true",
                    help="also build the wide grids over every lag and spin condition "
                         "(and, for hover, the level-1 row)")
    a = ap.parse_args()

    for folder, title, subtitle, spec, focus, views in SHEETS:
        dest = os.path.join(a.out, folder)
        os.makedirs(dest, exist_ok=True)
        pat = "{}_by_condition.png" if folder.endswith("envelope") else "{}_by_level.png"
        jobs = [(focus, pat, title, FOCUS_SUB[folder])]
        if a.all_conditions:
            jobs.append((spec, "{}_by_level_and_lag.png",
                         title + "  —  every lag and spin condition", subtitle))
        for build_spec, pattern, sheet_title, sheet_sub in jobs:
          for view in views:
            cols, rows = build_spec(view)
            got = build(view, sheet_title, sheet_sub, cols, rows,
                        os.path.join(dest, pattern.format(view)), a.tile_width)
            if got:
                missing = [p for _l, cs in rows for p, _c in cs
                           if p and not os.path.exists(os.path.join(EXP, p))]
                note = f"  (missing: {', '.join(missing)})" if missing else ""
                print(f"wrote {got}  {os.path.getsize(got) // 1024} KB{note}")


if __name__ == "__main__":
    main()
