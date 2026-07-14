#!/usr/bin/env python
"""
Regenerate eigen stability plots from existing scores.csv files.
No Isaac Sim required.

Handles both formats:
  - New format: omega_n \\ zeta, <zeta_value>  (1 column → bar chart)
  - Old format: wn \\ b, <b1>, <b2>, ...        (multi-column → heatmap)

Usage:
    python eigen_plot_heatmap.py                        # all mm dirs found
    python eigen_plot_heatmap.py --results-dir results/eigen_sweep/mm_2.7e-07
    python eigen_plot_heatmap.py --show                 # pop up windows too
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.colors as mc


def parse_args():
    p = argparse.ArgumentParser(description="Replot eigen stability scores from CSV")
    p.add_argument(
        "--results-dir", type=Path,
        default=Path(__file__).resolve().parent.parent / "results" / "eigen_sweep",
        help="Root dir to search for scores.csv files (default: results/eigen_sweep/)",
    )
    p.add_argument("--show", action="store_true", help="Pop up matplotlib windows")
    return p.parse_args()


# ─────────────────────────── CSV parsing ─────────────────────────────────────

def detect_format(csv_path: Path) -> str:
    """Return 'new' (omega_n × single zeta) or 'old' (wn × multi-b heatmap)."""
    with open(csv_path, newline="") as f:
        header = next(csv.reader(f))
    if header[0].strip().startswith("omega_n"):
        return "new"
    return "old"


def load_new_format(csv_path: Path):
    """New format: omega_n \\ zeta, <zeta_value>
    Returns (on_vals, zeta, scores_1d)."""
    with open(csv_path, newline="") as f:
        reader = list(csv.reader(f))
    zeta   = float(reader[0][1].strip())
    on_vals, scores = [], []
    for row in reader[1:]:
        on_vals.append(float(row[0].strip()))
        scores.append(np.inf if row[1].strip() == "inf" else float(row[1].strip()))
    return on_vals, zeta, np.array(scores)


def load_old_format(csv_path: Path):
    """Old format: wn \\ b, <b1>, <b2>, ...
    Returns (wn_vals, b_vals, scores_2d)."""
    with open(csv_path, newline="") as f:
        reader = list(csv.reader(f))
    header  = reader[0]
    b_vals  = [float(x) for x in header[1:]]
    wn_vals, rows = [], []
    for row in reader[1:]:
        wn_vals.append(float(row[0]))
        rows.append([np.inf if v.strip() == "inf" else float(v) for v in row[1:]])
    return wn_vals, b_vals, np.array(rows)


# ─────────────────────────── Bar chart (new format) ──────────────────────────

def plot_bar_chart(on_vals, zeta, scores, title: str, out_path: Path, show: bool):
    """Score vs ωn bar chart for fixed-ζ sweeps."""
    fig, ax = plt.subplots(figsize=(max(8, len(on_vals) * 1.2 + 3), 5))

    fin_mask = np.isfinite(scores)
    colors   = []
    best_i   = int(np.argmin(np.where(fin_mask, scores, np.inf))) if fin_mask.any() else None

    for i, s in enumerate(scores):
        if not np.isfinite(s):
            colors.append("dimgray")
        elif i == best_i:
            colors.append("gold")
        else:
            colors.append("steelblue")

    heights = np.where(fin_mask, scores, 0.0)
    ax.bar(range(len(on_vals)), heights, color=colors, edgecolor="black", linewidth=0.7)

    for i, s in enumerate(scores):
        if not np.isfinite(s):
            ax.text(i, heights.max() * 0.02 if fin_mask.any() else 0.05,
                    "∞\n(OOB)", ha="center", va="bottom",
                    fontsize=8, color="white", fontweight="bold")
        else:
            ax.text(i, s + heights.max() * 0.01, f"{s:.3f}",
                    ha="center", va="bottom", fontsize=7, color="black")

    # X tick labels: ωn + derived wn_mdyn / b_mdyn
    xlabels = []
    for on in on_vals:
        wn_m = zeta * on
        b_m  = on * math.sqrt(max(1.0 - zeta ** 2, 0.0))
        xlabels.append(f"ωn={on:.1f}\nwn={wn_m:.2f}  b={b_m:.2f}")

    ax.set_xticks(range(len(on_vals)))
    ax.set_xticklabels(xlabels, fontsize=8)
    ax.set_xlabel("Closed-loop natural frequency ωn  (→ M_dyn wn_mdyn, b_mdyn)", fontsize=11)
    ax.set_ylabel("Stability score  [future-biased mean ‖ep‖  m]", fontsize=11)
    ax.set_title(title, fontsize=11)
    ax.set_ylim(bottom=0)

    if best_i is not None:
        wn_best = zeta * on_vals[best_i]
        b_best  = on_vals[best_i] * math.sqrt(max(1.0 - zeta ** 2, 0.0))
        ax.bar(best_i, scores[best_i], color="gold", edgecolor="black", linewidth=1.2,
               zorder=5,
               label=(f"★ Best: ωn={on_vals[best_i]:.1f}  "
                      f"wn_mdyn={wn_best:.3f}  b_mdyn={b_best:.3f}  "
                      f"score={scores[best_i]:.4f}"))
        ax.legend(loc="upper right", fontsize=9)

        info = (
            f"★ Best params\n"
            f"  ωn      = {on_vals[best_i]:.1f}\n"
            f"  wn_mdyn = {wn_best:.4f}\n"
            f"  b_mdyn  = {b_best:.4f}\n"
            f"  ζ       = {zeta:.2f}\n"
            f"  score   = {scores[best_i]:.4f}"
        )
        ax.text(0.01, 0.99, info, transform=ax.transAxes,
                fontsize=8, verticalalignment="top",
                bbox=dict(boxstyle="round,pad=0.4", facecolor="lightyellow",
                          edgecolor="goldenrod", alpha=0.9))

    fig.tight_layout()
    fig.savefig(str(out_path), dpi=150, bbox_inches="tight")
    print(f"  saved → {out_path}")
    if show:
        plt.show()
    plt.close(fig)


# ─────────────────────────── Heatmap (old format) ────────────────────────────

def gains_for_wn(wn_desired: float):
    """Backward-compat gain law from the old sweep (wn_mdyn fixed at 7)."""
    _WN_MDYN    = 7.0
    _KP_DEFAULT = 0.7
    _KD_DEFAULT = 10.0
    _ALPHA      = 0.5
    import math as _m
    _omega_n_0  = _m.sqrt(_WN_MDYN * _KP_DEFAULT)
    _kd_eff_0   = _WN_MDYN * (1.0 + _KD_DEFAULT) - _ALPHA
    _ZETA_0     = _kd_eff_0 / (2.0 * _omega_n_0)
    kp = (wn_desired ** 2) / _WN_MDYN
    kd = (2.0 * _ZETA_0 * wn_desired + _ALPHA) / _WN_MDYN - 1.0
    return max(kp, 1e-4), max(kd, 0.01)


def plot_heatmap(wn_vals, b_vals, scores, title: str, out_path: Path, show: bool):
    """2-D heatmap for old-format CSVs."""
    n_wn, n_b = len(wn_vals), len(b_vals)
    fig, ax   = plt.subplots(figsize=(max(8, n_b * 1.2 + 3), max(6, n_wn * 1.0 + 2)))

    display  = scores.copy()
    fin_mask = np.isfinite(display)
    fin_vals = display[fin_mask]

    if fin_vals.size:
        vmin, vmax = max(fin_vals.min(), 1e-4), fin_vals.max() * 1.5
        sentinel   = vmax * 2.0
    else:
        vmin, vmax, sentinel = 1e-4, 10.0, 100.0

    display[~fin_mask] = sentinel
    norm = mc.LogNorm(vmin=vmin, vmax=vmax)
    cmap = plt.cm.RdYlGn_r.copy()
    cmap.set_over("dimgray")

    im = ax.imshow(display, aspect="auto", origin="lower", norm=norm, cmap=cmap)
    plt.colorbar(im, ax=ax,
                 label="Stability score  [future-biased mean ‖ep‖ m]",
                 extend="max")

    ax.set_xticks(range(n_b))
    ax.set_xticklabels([f"{v:.1f}" for v in b_vals], fontsize=9)
    ax.set_yticks(range(n_wn))
    ax.set_yticklabels([f"{v:.1f}" for v in wn_vals], fontsize=9)
    ax.set_xlabel("b  (M_dyn off-diagonal)", fontsize=11)
    ax.set_ylabel("wn  (M_dyn diagonal)", fontsize=11)
    ax.set_title(title, fontsize=12)

    for i in range(n_wn):
        for j in range(n_b):
            val = scores[i, j]
            kp_c, kd_c = gains_for_wn(wn_vals[i])
            if np.isfinite(val):
                text, col = f"{val:.3f}\nKp={kp_c:.3f} Kd={kd_c:.2f}", "black"
            else:
                text, col = "∞", "white"
            ax.text(j, i, text, ha="center", va="center",
                    fontsize=6, color=col, fontweight="bold", linespacing=1.4)

    if fin_mask.any():
        bi, bj = np.unravel_index(scores.argmin(), scores.shape)
        best_kp, best_kd = gains_for_wn(wn_vals[bi])
        ax.scatter(bj, bi, marker="*", s=400, color="gold",
                   edgecolors="black", linewidths=0.8, zorder=6,
                   label=(f"Best: wn={wn_vals[bi]:.1f}, b={b_vals[bj]:.1f}  "
                          f"Kp={best_kp:.4f}, Kd={best_kd:.3f}  "
                          f"(score={scores[bi,bj]:.4f})"))
        ax.legend(loc="upper right", fontsize=9)
        info = (f"★ Best params\n"
                f"  wn = {wn_vals[bi]:.1f}   b = {b_vals[bj]:.1f}\n"
                f"  Kp = {best_kp:.4f}\n  Kd = {best_kd:.4f}\n"
                f"  score = {scores[bi, bj]:.4f}")
        ax.text(0.01, 0.01, info, transform=ax.transAxes,
                fontsize=8, verticalalignment="bottom",
                bbox=dict(boxstyle="round,pad=0.4", facecolor="lightyellow",
                          edgecolor="goldenrod", alpha=0.9))

    fig.tight_layout()
    fig.savefig(str(out_path), dpi=150, bbox_inches="tight")
    print(f"  saved → {out_path}")
    if show:
        plt.show()
    plt.close(fig)


# ─────────────────────────── Main ────────────────────────────────────────────

def main():
    args = parse_args()

    if not args.show:
        matplotlib.use("Agg")

    root      = args.results_dir
    csv_files = sorted(root.rglob("scores.csv"))

    if not csv_files:
        print(f"No scores.csv found under {root}", file=sys.stderr)
        sys.exit(1)

    print(f"Found {len(csv_files)} scores file(s):")

    for csv_path in csv_files:
        mm_label = csv_path.parent.name
        fmt      = detect_format(csv_path)
        out_path = csv_path.parent / "heatmap.png"

        if fmt == "new":
            on_vals, zeta, scores = load_new_format(csv_path)
            fin = np.isfinite(scores)
            best_str = (
                f"best ωn={on_vals[int(np.argmin(np.where(fin, scores, np.inf)))]:.1f}"
                if fin.any() else "all OOB"
            )
            title = (
                f"Eigen M_dyn Eigenvalue Sweep   |   {mm_label}\n"
                f"ζ = {zeta:.2f} (fixed)   |   lower score = more stable   |   {best_str}"
            )
            print(f"  {mm_label}/scores.csv  ({len(on_vals)} ωn values, ζ={zeta:.2f})",
                  end="  →  ")
            plot_bar_chart(on_vals, zeta, scores, title, out_path, args.show)

        else:  # old heatmap format
            wn_vals, b_vals, scores = load_old_format(csv_path)
            fin = np.isfinite(scores)
            title = (
                f"Eigen Stability Sweep   |   {mm_label}\n"
                f"lower score = more stable   |   "
                + (f"best: wn={wn_vals[np.unravel_index(scores.argmin(), scores.shape)[0]]:.1f}, "
                   f"b={b_vals[np.unravel_index(scores.argmin(), scores.shape)[1]]:.1f}"
                   if fin.any() else "all OOB")
            )
            print(f"  {mm_label}/scores.csv  "
                  f"({len(wn_vals)} wn × {len(b_vals)} b)", end="  →  ")
            plot_heatmap(wn_vals, b_vals, scores, title, out_path, args.show)


if __name__ == "__main__":
    main()
