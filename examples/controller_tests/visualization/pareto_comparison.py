#!/usr/bin/env python3
"""
pareto_comparison.py  —  Fair multi-controller benchmarking via Pareto analysis.

Reads sweep data from:
  results/eigen_sweep/ab_sweep/mm_<km>/  scores.csv + effort.csv
  results/pd_sweep/                      scores.csv + effort.csv
  results/mpc_sweep/                     scores.csv + effort.csv

METRIC (shared by ALL sweep scripts and this visualiser)
--------------------------------------------------------
  score(t)  = Σ w(t)·‖ep(t)‖ / Σ w(t)     w(t) = t / T  (future-biased)
  effort(t) = Σ w(t)·(Σ_i u_i²) / Σ w(t)

WHY PARETO?
-----------
Each controller has a different parameterisation.
Comparing at a single tuning point is unfair.  The correct comparison is
a PARETO FRONTIER:

    "For each unit of control effort budget, which controller achieves
     the lowest tracking error?"

A controller whose entire Pareto frontier lies closer to the origin dominates.

FAIRNESS CAVEATS
----------------
  • Eigen currently has a 2D sweep (wn × b), while PD and MPC have 1D sweeps.
    A 1D sweep only traces a subset of the true Pareto frontier.
  • Outlier filtering uses the same 95th-percentile threshold for all controllers.
  • The iso-effort comparison (Figure 2) uses linear interpolation along each
    controller's Pareto frontier, not nearest-neighbour.

The script plots:
  Figure 1: Pareto plane with all frontiers overlaid
  Figure 2: Iso-effort bar chart — each frontier interpolated at every
            other frontier's median effort level (pairwise)

Usage
-----
  python3 examples/controller_tests/visualization/pareto_comparison.py
  python3 examples/controller_tests/visualization/pareto_comparison.py --no-show
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.colors as mc
from matplotlib.lines import Line2D
import csv

# ── paths ─────────────────────────────────────────────────────────────────────

RESULTS_DIR = Path(__file__).resolve().parent.parent / "results"

SINGLE_RUN_FILES = {
    "Cascaded PD": RESULTS_DIR / "controller_eval" / "cascaded_pd" / "statistics" / "cascaded_pd_statistics.npz",
    "LPV-MPC":     RESULTS_DIR / "controller_eval" / "lpv_mpc" / "statistics" / "lpv_mpc_statistics.npz",
    # Eigen single-run (used only if sweep data unavailable)
    "Eigen (single)": RESULTS_DIR / "controller_eval" / "eigen" / "statistics" / "eigen_statistics.npz",
}

# Sweep CSV directories (produced by pd_sweep.py / mpc_sweep.py)
SWEEP_CSV_DIRS = {
    "Cascaded PD": RESULTS_DIR / "pd_sweep",
    "LPV-MPC":     RESULTS_DIR / "mpc_sweep",
}

COLORS = {
    "Cascaded PD":    "#E74C3C",
    "LPV-MPC":        "#2980B9",
    "Eigen (single)": "#F39C12",
    "Eigen sweep":    "#2ECC71",
}

MARKERS = {
    "Cascaded PD":    "o",
    "LPV-MPC":        "^",
    "Eigen (single)": "s",
    "Eigen sweep":    "D",
}

# ── CLI ────────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--no-show", action="store_true")
    p.add_argument(
        "--sweep-dir",
        default=str(RESULTS_DIR / "eigen_sweep" / "ab_sweep" / "mm_2.7e-07"),
        help="Directory containing eigen ab_sweep scores.csv and effort.csv",
    )
    p.add_argument("--t-limit", type=float, default=None,
                   help="Clip single-run NPZ data to this many seconds")
    p.add_argument("--out-dir", default=str(RESULTS_DIR))
    p.add_argument(
        "--pd-sweep-dir",
        default=str(RESULTS_DIR / "pd_sweep"),
        help="Directory containing pd_sweep scores.csv and effort.csv",
    )
    p.add_argument(
        "--mpc-sweep-dir",
        default=str(RESULTS_DIR / "mpc_sweep"),
        help="Directory containing mpc_sweep scores.csv and effort.csv",
    )
    return p.parse_args()


# ── helpers ───────────────────────────────────────────────────────────────────

def _future_biased_mean(t: np.ndarray, values: np.ndarray) -> float:
    """Weighted mean with w(t) = t / T — same metric used by all sweep scripts."""
    T = float(t[-1]) if len(t) > 0 else 1.0
    if T < 1e-6:
        return float("nan")
    w = t / T
    w_sum = float(np.sum(w))
    if w_sum < 1e-12:
        return float("nan")
    return float(np.dot(w, values) / w_sum)


def operating_point_from_npz(path: Path, t_limit: float | None) -> tuple[float, float] | None:
    """Return (effort, score) from a single-run NPZ.

    Uses the SAME future-biased weighted metric as the sweep scripts:
      score  = Σ w(t)·‖ep(t)‖ / Σ w(t),   w(t) = t / T
      effort = Σ w(t)·(Σ_i u_i²) / Σ w(t)
    """
    if not path.exists():
        return None
    d = np.load(path)
    t  = d["time"].astype(float)
    ep = d["ep"].astype(float)
    mt = d.get("motor_thrusts")
    if mt is None:
        return None
    mt = mt.astype(float)

    mask = t <= t_limit if t_limit is not None else np.ones(len(t), bool)
    t, ep, mt = t[mask], ep[mask], mt[mask]
    if len(t) < 10:
        return None

    errors = np.linalg.norm(ep, axis=1)
    eff_per_step = np.sum(mt ** 2, axis=1)
    if not (np.all(np.isfinite(errors)) and np.all(np.isfinite(eff_per_step))):
        return None

    score  = _future_biased_mean(t, errors)
    effort = _future_biased_mean(t, eff_per_step)
    if not (np.isfinite(score) and np.isfinite(effort)):
        return None
    return effort, score


# ── Unified outlier filter (applied to ALL controllers equally) ──────────────

def _apply_outlier_filter(efforts: np.ndarray, scores: np.ndarray,
                          eff_cap: float = None, score_cap: float = None
                          ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return boolean mask of finite, non-outlier points.

    If caps are None, use 95th-percentile of the finite values as the cutoff.
    This ensures all controllers get the same style of filtering.
    """
    valid = np.isfinite(efforts) & np.isfinite(scores)
    if not valid.any():
        return efforts[valid], scores[valid], valid

    if eff_cap is None:
        eff_cap = min(float(np.percentile(efforts[valid], 95)) * 1.5, 3000.0)
    if score_cap is None:
        score_cap = min(float(np.percentile(scores[valid], 95)) * 1.5, 100.0)

    valid &= (efforts < eff_cap) & (scores < score_cap)
    return efforts[valid], scores[valid], valid


def read_1d_sweep_csv(csv_dir: Path) -> tuple[np.ndarray, np.ndarray] | None:
    """
    Read pd_sweep or mpc_sweep output CSVs.
    Returns (efforts, scores) arrays — one point per row, inf/outlier rows excluded.
    Expects:
      scores.csv  → columns: [param, ..., score]
      effort.csv  → columns: [param, ..., effort]
    """
    s_path = csv_dir / "scores.csv"
    e_path = csv_dir / "effort.csv"
    if not s_path.exists() or not e_path.exists():
        return None
    try:
        scores, efforts = [], []
        with open(str(s_path)) as f:
            reader = csv.reader(f)
            next(reader)  # skip header
            for row in reader:
                for v in row[1:]:
                    v = v.strip()
                    scores.append(float("inf") if v == "inf" else float(v))
        with open(str(e_path)) as f:
            reader = csv.reader(f)
            next(reader)
            for row in reader:
                for v in row[1:]:
                    v = v.strip()
                    efforts.append(float("inf") if v == "inf" else float(v))
        s_arr = np.array(scores)
        e_arr = np.array(efforts)
        e_filt, s_filt, _ = _apply_outlier_filter(e_arr, s_arr)
        if len(e_filt) == 0:
            return None
        return e_filt, s_filt
    except Exception as exc:
        print(f"[warn] could not read sweep CSVs in {csv_dir}: {exc}")
        return None


def read_sweep_csv(csv_path: Path) -> tuple[list, list, np.ndarray] | None:
    """
    Parse the eigen ab_sweep CSV (wn×b grid).
    Returns (a_vals, b_vals, values_2d).
    """
    if not csv_path.exists():
        return None
    with open(str(csv_path), newline="") as f:
        rows = list(csv.reader(f))
    if len(rows) < 2:
        return None
    b_vals = [float(x) for x in rows[0][1:]]
    a_vals, data = [], []
    for row in rows[1:]:
        if not row:
            continue
        a_vals.append(float(row[0]))
        data.append([float("inf") if x.strip() == "inf" else float(x) for x in row[1:]])
    return a_vals, b_vals, np.array(data)


def pareto_frontier(efforts: np.ndarray, scores: np.ndarray) -> np.ndarray:
    """
    Return boolean mask of Pareto-optimal points (lower effort AND lower score).
    A point is Pareto-optimal if no other point dominates it in both dimensions.
    """
    n = len(efforts)
    is_pareto = np.ones(n, dtype=bool)
    for i in range(n):
        if not is_pareto[i]:
            continue
        # dominate check: is there a j with effort[j] <= effort[i] AND score[j] <= score[i], j != i?
        dominated = (
            (efforts <= efforts[i]) & (scores <= scores[i]) &
            ~((efforts == efforts[i]) & (scores == scores[i]))
        )
        if dominated.any():
            is_pareto[i] = False
    return is_pareto


# ── plotting ───────────────────────────────────────────────────────────────────

def plot_pareto(sweep_data,
                single_points: dict[str, tuple[float, float]],
                sweep_frontiers: dict[str, tuple[np.ndarray, np.ndarray]],
                out_dir: Path, show: bool, mm_label: str = "",
                plot_name: str = "pareto_plane") -> None:

    bg = "#1A1D2E"
    grid_kw = dict(color="#2A2D3E", linestyle="--", linewidth=0.6, alpha=0.7)

    # ── Figure 1: Full Pareto scatter ───────────────────────────────────────
    fig1, ax1 = plt.subplots(figsize=(10, 8))
    fig1.patch.set_facecolor("#0F1117")
    fig1.suptitle(
        f"Controller Pareto Comparison {mm_label} — Tracking Error vs Actuation Effort",
        fontsize=13, fontweight="bold", color="white", y=0.96,
    )
    
    ax1.set_facecolor(bg)
    ax1.tick_params(colors="white", labelsize=9)
    for sp in ax1.spines.values():
        sp.set_edgecolor("#333650")
    ax1.grid(**grid_kw)
    ax1.xaxis.label.set_color("white")
    ax1.yaxis.label.set_color("white")
    ax1.title.set_color("white")

    legend_handles = []

    if sweep_data is not None:
        a_vals, b_vals, scores_2d, efforts_2d = sweep_data

        # Flatten, remove inf, apply unified outlier filter
        flat_eff   = efforts_2d.ravel()
        flat_score = scores_2d.ravel()
        a_flat     = np.repeat(a_vals, len(b_vals))
        b_flat     = np.tile(b_vals, len(a_vals))
        
        # Same percentile-based filter applied to all controllers
        _, _, valid = _apply_outlier_filter(flat_eff, flat_score)
        
        eff_v = flat_eff[valid]
        sc_v  = flat_score[valid]
        a_v, b_v = a_flat[valid], b_flat[valid]

        if len(sc_v) > 0:
            # Colour by (a, b) label
            norm_color = mc.Normalize(vmin=sc_v.min(), vmax=np.percentile(sc_v, 90))
            cmap       = plt.cm.YlOrRd

            sc_plot = ax1.scatter(eff_v, sc_v, c=sc_v, cmap=cmap, norm=norm_color,
                                 s=70, zorder=4, edgecolors="#555", linewidths=0.4,
                                 label="Eigen Configs (suboptimal)")
            cb = plt.colorbar(sc_plot, ax=ax1, label="Stability score (lower=better)", pad=0.01)
            cb.ax.yaxis.label.set_color("white")
            cb.ax.tick_params(colors="white")

            pf_mask  = pareto_frontier(eff_v, sc_v)
            pf_eff   = eff_v[pf_mask]
            pf_sc    = sc_v[pf_mask]
            pf_a     = a_v[pf_mask]
            pf_b     = b_v[pf_mask]
            sort_idx = np.argsort(pf_eff)
            pf_eff, pf_sc = pf_eff[sort_idx], pf_sc[sort_idx]
            pf_a, pf_b = pf_a[sort_idx], pf_b[sort_idx]

            ax1.plot(pf_eff, pf_sc, color=COLORS["Eigen sweep"], linewidth=2.0,
                    linestyle="-", zorder=5, marker="D", markersize=6,
                    label="Eigen Pareto frontier")
            legend_handles.append(Line2D([0],[0], color=COLORS["Eigen sweep"],
                                         linewidth=2, marker="D", markersize=6,
                                         label="Eigen Pareto frontier"))
                                         
            for e, s, a, b in zip(pf_eff, pf_sc, pf_a, pf_b):
                ax1.annotate(f"a={a:.0f}\nb={b:.0f}", (e, s),
                            textcoords="offset points", xytext=(4, 2),
                            fontsize=8, color="#DDDDDD", alpha=0.9, fontweight="bold")

    # ── PD and MPC sweep frontiers (from pd_sweep / mpc_sweep CSVs) ──────────
    sweep_line_styles = {"Cascaded PD": "-", "LPV-MPC": "-."}
    for name, frontier in sweep_frontiers.items():
        if frontier is None:
            continue
        eff_f, sc_f = frontier
        # Sort by effort for a clean line
        sort_i = np.argsort(eff_f)
        ax1.plot(eff_f[sort_i], sc_f[sort_i],
                color=COLORS[name], linewidth=2.0,
                linestyle=sweep_line_styles.get(name, "-"),
                marker=MARKERS.get(name, "o"), markersize=7, zorder=6,
                label=f"{name} Pareto frontier")
        legend_handles.append(Line2D([0],[0], color=COLORS[name],
                                     linewidth=2, marker=MARKERS.get(name, "o"),
                                     markersize=7, linestyle=sweep_line_styles.get(name, "-"),
                                     label=f"{name} Pareto frontier"))
        # Annotate individual points
        for e, s in zip(eff_f, sc_f):
            ax1.scatter(e, s, color=COLORS[name], s=60, zorder=7,
                       edgecolors="white", linewidths=0.5)

    # Single operating points (fallback when no sweep available)
    for name, pt in single_points.items():
        if pt is None:
            continue
        # Skip single point if a full frontier exists for this controller
        if name in sweep_frontiers and sweep_frontiers[name] is not None:
            continue
        if name == "Eigen (single)" and sweep_data is not None:
            continue
        eff, rmse = pt
        ax1.scatter(eff, rmse, color=COLORS[name], s=200, marker=MARKERS[name],
                   zorder=8, edgecolors="white", linewidths=1.2)
        ax1.axvline(eff,  color=COLORS[name], linestyle=":", linewidth=0.8, alpha=0.5)
        ax1.axhline(rmse, color=COLORS[name], linestyle=":", linewidth=0.8, alpha=0.5)
        ax1.annotate(f" {name}\n eff={eff:.1f}\n RMSE={rmse:.4f}",
                    (eff, rmse), textcoords="offset points", xytext=(8, -8),
                    fontsize=9, color=COLORS[name], fontweight="bold")
        legend_handles.append(Line2D([0],[0], color=COLORS[name],
                                     linestyle="none", marker=MARKERS[name],
                                     markersize=9, label=f"{name} (single run)"))

    # Iso-efficiency background gradient and hyperbolas: RMSE * effort = k
    if single_points:
        all_pts = [v for v in single_points.values() if v is not None]
        
        # Collect all points from all sweeps to set correct plot bounds
        all_effs = list(eff_v) if 'eff_v' in locals() else []
        all_scs  = list(sc_v) if 'sc_v' in locals() else []
        for name, frontier in sweep_frontiers.items():
            if frontier is not None:
                all_effs.extend(frontier[0])
                all_scs.extend(frontier[1])
        
        if all_effs and all_scs:
            e_min = max(1e-9, min(all_effs)*0.8)
            r_min = max(1e-9, min(all_scs)*0.8)
            e_max = max(all_effs)*1.1
            r_max = max(all_scs)*1.1
            
            # Cap the graph to some reasonable range for unstable runs
            e_p95 = float(np.percentile(all_effs, 95)) if all_effs else e_max
            r_p95 = float(np.percentile(all_scs, 95)) if all_scs else r_max
            
            if plot_name == "cascaded_pd":
                e_max = min(e_max, max(300.0, e_p95 * 1.5))
                r_max = min(r_max, r_p95 * 1.5)
            else:
                e_max = min(e_max, max(800.0, e_p95 * 1.5))
                r_max = min(r_max, r_p95 * 1.5)
                
            E, R = np.meshgrid(np.linspace(e_min, e_max, 100),
                               np.linspace(r_min, r_max, 100))
            Z = E * R
            
            # Low opacity gradient
            ax1.contourf(E, R, Z, levels=20, cmap="viridis", alpha=0.1, zorder=0)
            
            # Draw a few hyperbola lines
            k_vals = np.percentile(Z.ravel(), [10, 30, 50, 70, 90])
            for k in k_vals:
                ax1.plot(E[0], k / E[0], color="white", linewidth=0.5, linestyle=":", alpha=0.2, zorder=1)
                
            legend_handles.append(Line2D([0],[0], color="white", linewidth=0.5,
                                         linestyle=":", label="Iso-efficiency (Effort × Score)"))
            ax1.set_xlim(e_min, e_max)
            ax1.set_ylim(r_min, r_max)

    ax1.set_xlabel("Control Effort [future-biased mean Σ motor_thrust²]  (Lower is cheaper)", fontsize=11)
    ax1.set_ylabel("Stability Score [future-biased mean ‖ep‖  m]  (Lower is better)", fontsize=11)
    ax1.set_title("Pareto Plane: Each point is a different controller tuning\n"
                 "(Controllers closest to the bottom-left are most efficient)", fontsize=11)
    ax1.legend(handles=legend_handles, fontsize=9,
              facecolor="#1A1D2E", edgecolor="#333650", labelcolor="white")

    fig1.tight_layout(rect=[0, 0, 1, 0.97])
    pf_dir = Path(out_dir) / "pareto_front"
    pf_dir.mkdir(parents=True, exist_ok=True)
    pf_out = pf_dir / f"{plot_name}{mm_label.replace(' ', '_').replace('=', '')}.png"
    fig1.savefig(str(pf_out), dpi=150, bbox_inches="tight", facecolor=fig1.get_facecolor())
    print(f"PNG → {pf_out}")
    if not show:
        plt.close(fig1)

    # ── Figure 2: Iso-effort comparison using ALL available frontiers ────────
    fig2, ax2 = plt.subplots(figsize=(10, 7))
    fig2.patch.set_facecolor("#0F1117")
    ax2.set_facecolor(bg)
    ax2.tick_params(colors="white", labelsize=9)
    for sp in ax2.spines.values():
        sp.set_edgecolor("#333650")
    ax2.grid(**grid_kw)
    ax2.xaxis.label.set_color("white")
    ax2.yaxis.label.set_color("white")
    ax2.title.set_color("white")

    # Build a unified frontier dict: name → (sorted_efforts, sorted_scores)
    all_frontiers: dict[str, tuple[np.ndarray, np.ndarray]] = {}

    # Eigen sweep frontier
    if sweep_data is not None:
        a_vals_f, b_vals_f, scores_2d_f, efforts_2d_f = sweep_data
        flat_eff_f   = efforts_2d_f.ravel()
        flat_score_f = scores_2d_f.ravel()
        _, _, valid_f = _apply_outlier_filter(flat_eff_f, flat_score_f)
        eff_v_f, sc_v_f = flat_eff_f[valid_f], flat_score_f[valid_f]
        if len(eff_v_f) > 0:
            pf_m = pareto_frontier(eff_v_f, sc_v_f)
            pf_e, pf_s = eff_v_f[pf_m], sc_v_f[pf_m]
            si = np.argsort(pf_e)
            all_frontiers["Eigen sweep"] = (pf_e[si], pf_s[si])

    # PD and MPC sweep frontiers
    for name, frontier in sweep_frontiers.items():
        if frontier is None:
            continue
        eff_f, sc_f = frontier
        pf_m = pareto_frontier(eff_f, sc_f)
        pf_e, pf_s = eff_f[pf_m], sc_f[pf_m]
        si = np.argsort(pf_e)
        all_frontiers[name] = (pf_e[si], pf_s[si])

    # Add single-run points as degenerate 1-point "frontiers" if no sweep
    for name, pt in single_points.items():
        if pt is None:
            continue
        # Skip if a real frontier already exists for this controller base
        base = name.replace(" (single)", "")
        has_frontier = any(base in fn for fn in all_frontiers)
        if name == "Eigen (single)" and "Eigen sweep" in all_frontiers:
            continue
        if not has_frontier:
            all_frontiers[name] = (np.array([pt[0]]), np.array([pt[1]]))

    def _interpolate_frontier(frontier_eff, frontier_score, target_effort):
        """Linearly interpolate a frontier at a given effort level."""
        if len(frontier_eff) < 2:
            # Single point: nearest only
            return float(frontier_score[0]), float(frontier_eff[0])
        return float(np.interp(target_effort, frontier_eff, frontier_score)), target_effort

    # Pairwise iso-effort comparisons between ALL frontiers
    frontier_names = sorted(all_frontiers.keys())
    if len(frontier_names) >= 2:
        print(f"\n{'='*70}")
        print("ISO-EFFORT PAIRWISE FRONTIER COMPARISONS")
        print(f"{'='*70}")

        # For each pair, compare at the other's median effort
        for i, name_a in enumerate(frontier_names):
            eff_a, sc_a = all_frontiers[name_a]
            for name_b in frontier_names[i+1:]:
                eff_b, sc_b = all_frontiers[name_b]
                # Compare at both median efforts
                for ref_name, ref_eff in [(name_a, float(np.median(eff_a))),
                                          (name_b, float(np.median(eff_b)))]:
                    score_a_at, _ = _interpolate_frontier(eff_a, sc_a, ref_eff)
                    score_b_at, _ = _interpolate_frontier(eff_b, sc_b, ref_eff)
                    gap = score_a_at - score_b_at
                    print(f"  At {ref_name}'s median effort={ref_eff:.2f}:  "
                          f"{name_a}={score_a_at:.4f}  {name_b}={score_b_at:.4f}  "
                          f"gap={gap:+.4f}")

        # Bar chart: at each frontier's median effort, show all frontiers' interpolated scores
        ref_efforts = [(n, float(np.median(all_frontiers[n][0]))) for n in frontier_names]
        n_refs = len(ref_efforts)
        n_fronts = len(frontier_names)
        x = np.arange(n_refs)
        w = 0.8 / max(n_fronts, 1)

        for j, fname in enumerate(frontier_names):
            fe, fs = all_frontiers[fname]
            interp_scores = []
            for _, ref_e in ref_efforts:
                s_at, _ = _interpolate_frontier(fe, fs, ref_e)
                interp_scores.append(s_at)
            color = COLORS.get(fname, "#888888")
            bars = ax2.bar(x + j * w - (n_fronts - 1) * w / 2, interp_scores, w,
                           color=color, alpha=0.85, label=fname)
            for bar, v in zip(bars, interp_scores):
                ax2.text(bar.get_x() + bar.get_width()/2,
                         bar.get_height() + 0.0005,
                         f"{v:.4f}", ha="center", va="bottom",
                         fontsize=7, color="white")

        ax2.set_xticks(x)
        ax2.set_xticklabels([f"{n}\neff={e:.1f}" for n, e in ref_efforts],
                            fontsize=8, color="white")
        ax2.set_ylabel("Stability Score [m]  (interpolated from frontier)")
        ax2.set_title("Iso-effort comparison: all frontiers interpolated\n"
                      "at each controller's median effort level",
                      fontsize=11)
        ax2.legend(fontsize=8, facecolor="#1A1D2E",
                   edgecolor="#333650", labelcolor="white")
    else:
        ax2.text(0.5, 0.5, "Need ≥ 2 sweep frontiers\nfor iso-effort comparison",
                 ha="center", va="center", transform=ax2.transAxes,
                 color="white", fontsize=12)

    fig2.tight_layout()
    iso_dir = Path(out_dir) / "iso_effort"
    iso_dir.mkdir(parents=True, exist_ok=True)
    iso_out = iso_dir / f"iso_{plot_name}{mm_label.replace(' ', '_').replace('=', '')}.png"
    fig2.savefig(str(iso_out), dpi=150, bbox_inches="tight", facecolor=fig2.get_facecolor())
    print(f"PNG → {iso_out}")
    if show:
        plt.show()
    else:
        plt.close(fig2)


# ── instructions for getting full PD/MPC Pareto frontiers ────────────────────

SWEEP_INSTRUCTIONS = """
╔══════════════════════════════════════════════════════════════════════════════╗
║  PARETO FRONTIER FAIRNESS NOTES                                              ║
╠══════════════════════════════════════════════════════════════════════════════╣
║                                                                              ║
║  ⚠ SWEEP DIMENSIONALITY:                                                     ║
║    Eigen has a 2D sweep (wn × b = 30 configs). PD/MPC currently have         ║
║    1D sweeps (α or q_weight alone). A 1D slice can only trace a subset       ║
║    of a controller's true Pareto frontier. For a fair comparison,            ║
║    consider 2D sweeps for PD (Kp_xy × Kd_xy) and MPC (q × r).                ║
║                                                                              ║
║  METRIC CONSISTENCY:                                                         ║
║    All axes use the same future-biased weighted mean metric                  ║
║    (score = Σ w(t)·‖ep(t)‖ / Σ w(t), w(t) = t/T) as the sweep scripts.       ║
║    Outlier filtering uses the same 95th-percentile-based threshold           ║
║    for all controllers.                                                      ║
║                                                                              ║
║  CURRENT PD/MPC SWEEPS (1D):                                                 ║
║    mpc_sweep.py --q-values 5 10 15 20 30 --r-weight 0.02                     ║
║    pd_sweep.py  --alpha-values 0.3 0.5 0.75 1.0 1.5 2.0 3.0                  ║
║                                                                              ║
║  RECOMMENDED 2D SWEEPS (for fairness):                                       ║
║    pd_sweep.py  --kp-xy-values 0.3 0.5 0.7 --kd-xy-values 0.5 1.0 2.0        ║
║    mpc_sweep.py --q-values 5 10 20 --r-values 0.01 0.02 0.05                 ║
║    (These CLI flags require extending the sweep scripts.)                    ║
╚══════════════════════════════════════════════════════════════════════════════╝
"""


# ── main ──────────────────────────────────────────────────────────────────────

def main():
    args = parse_args()
    can_show = (not args.no_show) and bool(os.environ.get("DISPLAY"))
    if not can_show:
        matplotlib.use("Agg")

    print(SWEEP_INSTRUCTIONS)

    # Load eigen sweep grid
    sweep_dir = Path(args.sweep_dir)
    scores_raw  = read_sweep_csv(sweep_dir / "scores.csv")
    efforts_raw = read_sweep_csv(sweep_dir / "effort.csv")

    # Load single-run operating points
    single_points: dict[str, tuple | None] = {}
    for name, path in SINGLE_RUN_FILES.items():
        pt = operating_point_from_npz(path, args.t_limit)
        single_points[name] = pt
        if pt:
            print(f"  {name:20s}  effort={pt[0]:.3f}  RMSE_ss={pt[1]:.4f}")
        else:
            print(f"  {name:20s}  [not found]")

    # Find all mm_ directories in the parent ab_sweep dir
    ab_sweep_dir = RESULTS_DIR / "eigen_sweep" / "ab_sweep"
    if ab_sweep_dir.exists():
        mm_dirs = [d for d in ab_sweep_dir.iterdir() if d.is_dir() and d.name.startswith("mm_")]
    else:
        mm_dirs = [Path(args.sweep_dir)]

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    for mm_dir in mm_dirs:
        scores_raw  = read_sweep_csv(mm_dir / "scores.csv")
        efforts_raw = read_sweep_csv(mm_dir / "effort.csv")

        sweep_data = None
        if scores_raw is not None and efforts_raw is not None:
            a_vals, b_vals, scores_2d  = scores_raw
            _,      _,      efforts_2d = efforts_raw
            sweep_data = (a_vals, b_vals, scores_2d, efforts_2d)
            print(f"\nLoaded Eigen sweep: {len(a_vals)} × {len(b_vals)} grid from {mm_dir.name}")
        else:
            print(f"\n[warn] Eigen sweep CSVs not found at {mm_dir}")
        import subprocess
        
        mm_str = mm_dir.name.replace('mm_', '')
        
        pd_mm_dir = RESULTS_DIR / "pd_sweep" / mm_dir.name
        if not pd_mm_dir.exists() or not (pd_mm_dir / "scores.csv").exists():
            print(f"\n[pareto] PD grid not found for mm={mm_str}. Running pd_sweep...")
            cmd = f"bash -i -c 'isaac_run examples/controller_tests/sweeps/pd_sweep.py --kp-xy-values 0.3 0.5 0.7 1.0 1.5 --kd-xy-values 0.5 1.0 2.0 3.0 --headless --motor-moment {mm_str}'"
            subprocess.run(cmd, shell=True)
            
        mpc_mm_dir = RESULTS_DIR / "mpc_sweep" / mm_dir.name
        if not mpc_mm_dir.exists() or not (mpc_mm_dir / "scores.csv").exists():
            print(f"\n[pareto] MPC grid not found for mm={mm_str}. Running mpc_sweep...")
            cmd = f"bash -i -c 'isaac_run examples/controller_tests/sweeps/mpc_sweep.py --q-values 5 10 20 30 --r-values 0.01 0.02 0.05 0.1 --headless --motor-moment {mm_str}'"
            subprocess.run(cmd, shell=True)
            
        sweep_frontiers: dict[str, tuple | None] = {}
        pd_frontier = read_1d_sweep_csv(pd_mm_dir)
        if pd_frontier:
            sweep_frontiers["Cascaded PD"] = pd_frontier
            print(f"Loaded PD sweep for mm={mm_str}")
        
        mpc_frontier = read_1d_sweep_csv(mpc_mm_dir)
        if mpc_frontier:
            sweep_frontiers["LPV-MPC"] = mpc_frontier
            print(f"Loaded MPC sweep for mm={mm_str}")

        mm_label = f" (kt={mm_str})" if sweep_data else ""
        
        eigen_mpc_frontiers = {k: v for k, v in sweep_frontiers.items() if "Cascaded PD" not in k}
        pd_frontiers = {k: v for k, v in sweep_frontiers.items() if "Cascaded PD" in k}
        
        eigen_mpc_singles = {k: v for k, v in single_points.items() if "Cascaded PD" not in k}
        pd_singles = {k: v for k, v in single_points.items() if "Cascaded PD" in k}
        
        plot_pareto(sweep_data, eigen_mpc_singles, eigen_mpc_frontiers,
                    out_dir=out_dir, show=can_show, mm_label=mm_label + " (Eigen vs MPC)",
                    plot_name="eigen_mpc")
                    
        plot_pareto(None, pd_singles, pd_frontiers,
                    out_dir=out_dir, show=can_show, mm_label=mm_label + " (Cascaded PD)",
                    plot_name="cascaded_pd")

if __name__ == "__main__":
    main()
