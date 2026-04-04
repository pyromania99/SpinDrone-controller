#!/usr/bin/env python
"""
Post-sweep summary plots for the Eigen M_dyn eigenvalue stability sweep.
No Isaac Sim required — reads NPZ telemetry files produced by eigen_stability_sweep.py.

Generates (per motor_moment directory found):
  1. Score vs ωn — line/scatter plot, all mm on one axis
  2. Motor forces & torques over time — one figure per (mm, ωn) run
     • Total thrust  [N]
     • Roll / pitch / yaw torque  [N·m]
     • Per-motor thrusts u1–u4  [N]
  3. Cross-mm overlay — best ωn from each mm on one force/torque figure

Usage:
    python eigen_plot_summary.py
    python eigen_plot_summary.py --results-dir results/eigen_sweep
    python eigen_plot_summary.py --show
    python eigen_plot_summary.py --forces-only       # skip score plot
    python eigen_plot_summary.py --omega-n 6.0 8.0  # only these ωn values
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.cm as cm


# ─────────────────────────── CLI ─────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(
        description="Post-sweep summary: score vs ωn + motor forces/torques"
    )
    p.add_argument(
        "--results-dir", type=Path,
        default=Path(__file__).resolve().parent / "results" / "eigen_sweep",
        help="Root directory containing mm_* subdirectories",
    )
    p.add_argument("--show",        action="store_true",  help="Pop up matplotlib windows")
    p.add_argument("--forces-only", action="store_true",  help="Skip the score-vs-ωn plot")
    p.add_argument(
        "--omega-n", nargs="+", type=float, default=None,
        help="Only plot these ωn values (default: all found)",
    )
    p.add_argument(
        "--max-force-plots", type=int, default=6,
        help="Maximum per-run force plots per mm dir (avoid hundreds of files)",
    )
    return p.parse_args()


# ─────────────────────────── Helpers ─────────────────────────────────────────

def mm_label(mm_dir: Path) -> str:
    return mm_dir.name  # e.g. "mm_2.7e-07"


def find_mm_dirs(root: Path) -> list[Path]:
    return sorted(d for d in root.iterdir() if d.is_dir() and d.name.startswith("mm_"))


def find_npz_files(mm_dir: Path, omega_n_filter=None) -> list[tuple[float, Path]]:
    """Return [(omega_n, path), ...] sorted by omega_n."""
    results = []
    for npz in sorted(mm_dir.glob("on*_statistics.npz")):
        try:
            on = float(npz.stem.split("_")[0].lstrip("on"))
        except ValueError:
            continue
        if omega_n_filter is not None and on not in omega_n_filter:
            continue
        results.append((on, npz))
    results.sort(key=lambda x: x[0])
    return results


def load_scores_csv(mm_dir: Path):
    """Return (on_vals, zeta, scores) from scores.csv if present."""
    csv_path = mm_dir / "scores.csv"
    if not csv_path.exists():
        return None, None, None
    import csv
    with open(csv_path, newline="") as f:
        reader = list(csv.reader(f))
    if not reader[0][0].strip().startswith("omega_n"):
        return None, None, None  # old format — skip
    zeta     = float(reader[0][1].strip())
    on_vals, scores = [], []
    for row in reader[1:]:
        on_vals.append(float(row[0].strip()))
        scores.append(np.inf if row[1].strip() == "inf" else float(row[1].strip()))
    return on_vals, zeta, np.array(scores)


def save_fig(fig, path: Path, show: bool):
    fig.savefig(str(path), dpi=150, bbox_inches="tight")
    print(f"  saved → {path}")
    if show:
        plt.show()
    plt.close(fig)


# ─────────────────────────── 1. Score vs ωn (all mm) ─────────────────────────

def plot_score_summary(mm_dirs: Path, args):
    """One figure with all motor_moment curves on a shared ωn axis."""
    fig, ax = plt.subplots(figsize=(10, 5))
    plotted = 0

    cmap   = cm.get_cmap("tab10")
    colors = [cmap(i) for i in range(len(mm_dirs))]

    for idx, mm_dir in enumerate(mm_dirs):
        on_vals, zeta, scores = load_scores_csv(mm_dir)
        if on_vals is None:
            continue

        fin_mask = np.isfinite(scores)
        label    = mm_label(mm_dir)
        color    = colors[idx]

        # Draw finite points
        finite_on  = [on_vals[i] for i in range(len(on_vals)) if fin_mask[i]]
        finite_sc  = [scores[i]  for i in range(len(scores))  if fin_mask[i]]
        ax.plot(finite_on, finite_sc, marker="o", linewidth=1.8,
                markersize=5, color=color, label=label)

        # Mark OOB as X at plot bottom
        oob_on = [on_vals[i] for i in range(len(on_vals)) if not fin_mask[i]]
        if oob_on:
            ax.scatter(oob_on, [0] * len(oob_on), marker="x",
                       color=color, s=80, linewidths=2, zorder=5,
                       label=f"{label} OOB")

        # Star best
        if fin_mask.any():
            best_i = int(np.argmin(np.where(fin_mask, scores, np.inf)))
            ax.scatter(on_vals[best_i], scores[best_i], marker="*",
                       s=300, color=color, edgecolors="black",
                       linewidths=0.8, zorder=6)

        if zeta is not None and plotted == 0:
            ax.set_title(
                f"Eigen Stability — Score vs ωn   |   ζ = {zeta:.2f} (fixed)\n"
                f"Lower score = more stable   |   ✕ = OOB   |   ★ = best per motor_moment",
                fontsize=11,
            )
        plotted += 1

    if plotted == 0:
        print("  [summary] No scores.csv (new format) found — skipping score plot.")
        plt.close(fig)
        return

    ax.set_xlabel("Closed-loop natural frequency ωn", fontsize=12)
    ax.set_ylabel("Stability score  [future-biased mean ‖ep‖  m]", fontsize=12)
    ax.legend(fontsize=9, loc="upper left")
    ax.set_ylim(bottom=0)
    ax.grid(True, alpha=0.3)

    fig.tight_layout()
    out = args.results_dir / "score_vs_omega_n.png"
    save_fig(fig, out, args.show)


# ─────────────────────────── 2. Motor forces/torques ─────────────────────────

def plot_forces_for_run(on: float, npz_path: Path, out_path: Path,
                        title_prefix: str, show: bool):
    """4-panel force/torque figure for one (mm, ωn) run."""
    try:
        data = np.load(npz_path)
    except Exception as exc:
        print(f"  [forces] could not load {npz_path}: {exc}")
        return

    t = data.get("time", None)
    if t is None or len(t) < 2:
        print(f"  [forces] {npz_path.name}: no time data, skipping")
        return

    thrust  = data.get("thrust",        None)   # (N,)
    torque  = data.get("torque",         None)   # (N, 3)
    motors  = data.get("motor_thrusts",  None)   # (N, 4)
    ep      = data.get("ep",             None)   # (N, 3)

    has_forces = (thrust is not None and len(thrust) > 1)
    has_torque = (torque is not None and torque.ndim == 2 and torque.shape[0] > 1)
    has_motors = (motors is not None and motors.ndim == 2 and motors.shape[0] > 1)
    has_ep     = (ep     is not None and ep.ndim     == 2 and ep.shape[0]     > 1)

    n_panels = sum([has_forces, has_torque, has_motors, has_ep])
    if n_panels == 0:
        print(f"  [forces] {npz_path.name}: no force/torque data "
              "(old NPZ without tracking — re-run sweep to populate)")
        return

    fig, axes = plt.subplots(n_panels, 1,
                             figsize=(11, 3.2 * n_panels),
                             sharex=True)
    if n_panels == 1:
        axes = [axes]

    ax_iter = iter(axes)

    # ── 1. Total thrust ────────────────────────────────────────────────
    if has_forces:
        ax = next(ax_iter)
        ax.plot(t, thrust, color="royalblue", linewidth=1.2)
        ax.set_ylabel("Total thrust [N]", fontsize=10)
        ax.grid(True, alpha=0.3)
        ax.set_title(f"{title_prefix}  ωn={on:.1f}", fontsize=11)

    # ── 2. Roll / pitch / yaw torque ──────────────────────────────────
    if has_torque:
        ax    = next(ax_iter)
        t_trq = t[:torque.shape[0]]
        ax.plot(t_trq, torque[:, 0], label="Roll  [N·m]", linewidth=1.2)
        ax.plot(t_trq, torque[:, 1], label="Pitch [N·m]", linewidth=1.2)
        ax.plot(t_trq, torque[:, 2], label="Yaw   [N·m]", linewidth=1.2, linestyle="--")
        ax.set_ylabel("Torque [N·m]", fontsize=10)
        ax.legend(fontsize=8, ncol=3)
        ax.grid(True, alpha=0.3)

    # ── 3. Per-motor thrusts ───────────────────────────────────────────
    if has_motors:
        ax   = next(ax_iter)
        t_mt = t[:motors.shape[0]]
        for mi in range(motors.shape[1]):
            ax.plot(t_mt, motors[:, mi], label=f"Motor {mi+1}", linewidth=1.0)
        ax.set_ylabel("Motor thrust [N]", fontsize=10)
        ax.legend(fontsize=8, ncol=4)
        ax.grid(True, alpha=0.3)

    # ── 4. Position error magnitude ────────────────────────────────────
    if has_ep:
        ax      = next(ax_iter)
        t_ep    = t[:ep.shape[0]]
        err_mag = np.linalg.norm(ep, axis=1)
        ax.plot(t_ep, err_mag, color="firebrick", linewidth=1.2)
        ax.set_ylabel("‖position error‖ [m]", fontsize=10)
        ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel("Time [s]", fontsize=10)
    fig.tight_layout()
    save_fig(fig, out_path, show)


def plot_forces_all_runs(mm_dir: Path, args):
    """Generate one force/torque figure per NPZ in mm_dir."""
    runs = find_npz_files(mm_dir, omega_n_filter=args.omega_n)
    if not runs:
        print(f"  [forces] no NPZ files found in {mm_dir.name}")
        return

    label = mm_label(mm_dir)
    count = 0

    for on, npz_path in runs:
        if count >= args.max_force_plots:
            print(f"  [forces] limit of {args.max_force_plots} force plots reached "
                  f"for {label} — use --max-force-plots to increase")
            break
        out_path = mm_dir / f"forces_on{on:.1f}.png"
        print(f"  [forces] {label}  ωn={on:.1f}  →  ", end="")
        plot_forces_for_run(on, npz_path, out_path,
                            title_prefix=f"Motor forces/torques  |  {label}",
                            show=args.show)
        count += 1


# ─────────────────────────── 3. Cross-mm best overlay ────────────────────────

def plot_cross_mm_forces(mm_dirs: list[Path], args):
    """
    For each mm dir find the best-scoring ωn run and overlay their
    total-thrust and position-error traces on a single figure.
    """
    fig, (ax_thrust, ax_ep) = plt.subplots(2, 1, figsize=(11, 7), sharex=False)

    cmap   = cm.get_cmap("tab10")
    colors = [cmap(i) for i in range(len(mm_dirs))]
    plotted = 0

    for idx, mm_dir in enumerate(mm_dirs):
        on_vals, zeta, scores = load_scores_csv(mm_dir)
        if on_vals is None:
            continue

        fin_mask = np.isfinite(scores)
        if not fin_mask.any():
            continue

        best_i  = int(np.argmin(np.where(fin_mask, scores, np.inf)))
        best_on = on_vals[best_i]
        npz_path = mm_dir / f"on{best_on:.1f}_statistics.npz"
        if not npz_path.exists():
            continue

        try:
            data = np.load(npz_path)
        except Exception as exc:
            print(f"  [cross-mm] could not load {npz_path}: {exc}")
            continue

        t      = data.get("time",   None)
        thrust = data.get("thrust", None)
        ep     = data.get("ep",     None)

        color = colors[idx]
        label = f"{mm_label(mm_dir)}  ωn={best_on:.1f}  score={scores[best_i]:.4f}"

        if t is not None and thrust is not None and len(thrust) == len(t):
            ax_thrust.plot(t, thrust, color=color, linewidth=1.4, label=label)

        if t is not None and ep is not None and ep.ndim == 2:
            t_ep = t[:ep.shape[0]]
            ax_ep.plot(t_ep, np.linalg.norm(ep, axis=1), color=color,
                       linewidth=1.4, label=label)
        plotted += 1

    if plotted == 0:
        print("  [cross-mm] no valid data for overlay — skipping")
        plt.close(fig)
        return

    ax_thrust.set_ylabel("Total thrust [N]", fontsize=11)
    ax_thrust.set_xlabel("Time [s]", fontsize=10)
    ax_thrust.legend(fontsize=8)
    ax_thrust.grid(True, alpha=0.3)
    ax_thrust.set_title(
        "Cross motor_moment overlay — best ωn per mm\nTotal thrust [N]", fontsize=11
    )

    ax_ep.set_ylabel("‖position error‖ [m]", fontsize=11)
    ax_ep.set_xlabel("Time [s]", fontsize=10)
    ax_ep.legend(fontsize=8)
    ax_ep.grid(True, alpha=0.3)
    ax_ep.set_title("‖position error‖ [m]", fontsize=11)

    fig.tight_layout()
    out = args.results_dir / "cross_mm_best_forces.png"
    save_fig(fig, out, args.show)


# ─────────────────────────── Main ────────────────────────────────────────────

def main():
    args = parse_args()

    if not args.show:
        matplotlib.use("Agg")

    root    = args.results_dir
    mm_dirs = find_mm_dirs(root)

    if not mm_dirs:
        print(f"No mm_* directories found under {root}", file=sys.stderr)
        sys.exit(1)

    print(f"Found {len(mm_dirs)} motor_moment directory(ies): "
          + ", ".join(d.name for d in mm_dirs))

    # 1. Score vs ωn (all mm on one plot)
    if not args.forces_only:
        print("\n── Score vs ωn ─────────────────────────────────")
        plot_score_summary(mm_dirs, args)

    # 2. Per-run motor forces/torques
    print("\n── Motor forces / torques per run ──────────────")
    for mm_dir in mm_dirs:
        print(f"\n  {mm_dir.name}:")
        plot_forces_all_runs(mm_dir, args)

    # 3. Cross-mm best overlay
    print("\n── Cross-mm best-ωn overlay ─────────────────────")
    plot_cross_mm_forces(mm_dirs, args)

    print("\nDone.")


if __name__ == "__main__":
    main()
