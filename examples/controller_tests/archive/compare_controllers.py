#!/usr/bin/env python3
"""
Controller comparison: Eigen vs CascadedPD
Special metrics for a same-direction-spin drone (uncontrolled yaw via aerodynamic damping).

Key insight: with all motors spinning the same way, the drone develops ~28+ rad/s yaw rate.
The gyroscopic Euler cross-coupling terms (p*r, q*r) then perturb roll/pitch proportionally
to yaw rate. A controller that ignores this coupling accumulates position error at high spin.

Usage:
  /usr/bin/python3 examples/controller_tests/archive/compare_controllers.py
  /usr/bin/python3 examples/controller_tests/archive/compare_controllers.py --no-show
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec


RESULTS_DIR = Path(__file__).resolve().parent.parent / "results"

CONTROLLERS = {
    "Eigen":       RESULTS_DIR / "controller_eval" / "eigen" / "statistics" / "eigen_statistics.npz",
    "CascadedPD":  RESULTS_DIR / "controller_eval" / "cascaded_pd" / "statistics" / "cascaded_pd_statistics.npz",
}

COLORS = {
    "Eigen":      "#2196F3",
    "CascadedPD": "#F44336",
}

# ---- Metric helpers ---------------------------------------------------------

def pos_rmse(ep: np.ndarray) -> float:
    return float(np.sqrt(np.mean(np.linalg.norm(ep, axis=1) ** 2)))


def yaw_pos_correlation(yr: np.ndarray, ep: np.ndarray) -> float:
    """Pearson r between |yaw_rate| and |position_error|.

    Measures how tightly the spin-yaw disturbance couples into position.
    CascadedPD ≈ +0.95 (spin causes error); Eigen ≈ near-zero or negative (coupling broken).
    """
    pos_mag = np.linalg.norm(ep, axis=1)
    return float(np.corrcoef(np.abs(yr), pos_mag)[0, 1])


def spin_regime_rmse(yr: np.ndarray, ep: np.ndarray, threshold: float = 20.0):
    """Position RMSE restricted to samples where |yaw_rate| > threshold rad/s."""
    mask = np.abs(yr) > threshold
    if mask.sum() == 0:
        return float("nan"), 0
    return float(np.sqrt(np.mean(np.linalg.norm(ep[mask], axis=1) ** 2))), int(mask.sum())


def spin_resilience_score(yr: np.ndarray, ep: np.ndarray,
                           lo_thresh: float = 5.0, hi_thresh: float = 20.0):
    """RMSE_high_spin / RMSE_low_spin.

    < 1 means the controller actually improves at high spin (compensates coupling).
    > 1 means spin degrades tracking (disturbance is not modelled).
    NaN if one bin is empty.
    """
    pos_mag = np.linalg.norm(ep, axis=1)
    lo = pos_mag[np.abs(yr) < lo_thresh]
    hi = pos_mag[np.abs(yr) > hi_thresh]
    if len(lo) == 0 or len(hi) == 0:
        return float("nan")
    return float(np.sqrt(np.mean(hi ** 2)) / (np.sqrt(np.mean(lo ** 2)) + 1e-9))


def compute_metrics(data: dict) -> dict:
    t   = data["time"]
    ep  = data["ep"]
    yr  = data["yaw_rate"]

    n_ss = len(t) // 2          # steady-state window: last 50%
    ep_ss = ep[n_ss:]
    yr_ss = yr[n_ss:]

    hi_rmse, hi_n = spin_regime_rmse(yr, ep, threshold=20.0)

    return {
        "duration_s":        float(t[-1]),
        "n_samples":         len(t),
        "rmse_all":          pos_rmse(ep),
        "rmse_steady":       pos_rmse(ep_ss),
        "max_pos_err":       float(np.max(np.linalg.norm(ep, axis=1))),
        "yaw_rate_mean":     float(np.mean(np.abs(yr))),
        "yaw_rate_max":      float(np.max(np.abs(yr))),
        "yaw_pos_corr":      yaw_pos_correlation(yr, ep),
        "yaw_pos_corr_ss":   yaw_pos_correlation(yr_ss, ep_ss),
        "spin_rmse_hi":      hi_rmse,
        "spin_rmse_hi_n":    hi_n,
        "spin_resilience":   spin_resilience_score(yr, ep),
    }


def print_scorecard(metrics: dict[str, dict]) -> None:
    sep = "=" * 72
    print(sep)
    print("CONTROLLER SCORECARD  (spin-yaw drone — all motors same direction)")
    print(sep)

    fields = [
        ("Run duration [s]",              "duration_s",       ".1f"),
        ("Total pos RMSE [m]",            "rmse_all",         ".4f"),
        ("Steady-state pos RMSE [m]",     "rmse_steady",      ".4f"),
        ("Max position error [m]",        "max_pos_err",      ".4f"),
        ("Mean |yaw_rate| [rad/s]",       "yaw_rate_mean",    ".2f"),
        ("Max  |yaw_rate| [rad/s]",       "yaw_rate_max",     ".2f"),
        ("Corr(|yr|, |pos_err|) all",     "yaw_pos_corr",     ".4f"),
        ("Corr(|yr|, |pos_err|) SS",      "yaw_pos_corr_ss",  ".4f"),
        ("RMSE at |yr|>20 rad/s [m]",     "spin_rmse_hi",     ".4f"),
        ("Spin Resilience Score",          "spin_resilience",  ".3f"),
    ]

    names = list(metrics.keys())
    col_w = 14
    header = f"{'Metric':<40}" + "".join(f"{n:>{col_w}}" for n in names)
    print(header)
    print("-" * len(header))

    for label, key, fmt in fields:
        row = f"{label:<40}"
        for n in names:
            v = metrics[n].get(key, float("nan"))
            row += f"{format(v, fmt):>{col_w}}"
        print(row)

    print()
    print("Spin Resilience Score: RMSE(|yr|>20) / RMSE(|yr|<5). < 1 = spin helps.")
    print("Corr(|yr|,|pos_err|): +1 = spin causes error, 0/-1 = coupling broken.")
    print(sep)

    # Improvement ratios
    if "Eigen" in metrics and "CascadedPD" in metrics:
        e = metrics["Eigen"]
        c = metrics["CascadedPD"]
        print("\nImprovement (Eigen over CascadedPD):")
        if not np.isnan(c["rmse_steady"]) and c["rmse_steady"] > 1e-6:
            print(f"  Steady-state RMSE:    {c['rmse_steady']/e['rmse_steady']:.1f}x better")
        if not np.isnan(c["spin_rmse_hi"]) and not np.isnan(e["spin_rmse_hi"]):
            print(f"  Spin-regime RMSE:     {c['spin_rmse_hi']/e['spin_rmse_hi']:.1f}x better")
        print(f"  Run stability:        {e['duration_s']/c['duration_s']:.1f}x longer before divergence")
        print(f"  Cross-coupling corr:  {c['yaw_pos_corr']:.2f} -> {e['yaw_pos_corr']:.2f}  (near-zero = coupling compensated)")
    print(sep)


# ---- Plots ------------------------------------------------------------------

def plot_all(data_map: dict[str, dict], metrics: dict[str, dict],
             save_dir: Path | None, show: bool) -> None:

    fig = plt.figure(figsize=(16, 12))
    fig.suptitle("Controller Comparison — Spin-Yaw Drone\n(all motors same direction → passive yaw via aerodynamic damping)",
                 fontsize=13, fontweight="bold")
    gs = gridspec.GridSpec(3, 3, figure=fig, hspace=0.45, wspace=0.35)

    # --- 1. Position error magnitude vs time ---------------------------------
    ax1 = fig.add_subplot(gs[0, :2])
    for name, d in data_map.items():
        t  = d["time"]
        ep = d["ep"]
        ax1.plot(t, np.linalg.norm(ep, axis=1), color=COLORS[name], label=name, alpha=0.8)
    ax1.set_xlabel("Time [s]")
    ax1.set_ylabel("|Position Error| [m]")
    ax1.set_title("Position Error vs Time")
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    # --- 2. Yaw rate vs time -------------------------------------------------
    ax2 = fig.add_subplot(gs[0, 2])
    for name, d in data_map.items():
        ax2.plot(d["time"], np.abs(d["yaw_rate"]), color=COLORS[name], label=name, alpha=0.8)
    ax2.set_xlabel("Time [s]")
    ax2.set_ylabel("|Yaw Rate| [rad/s]")
    ax2.set_title("Yaw Rate (passive)")
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    # --- 3. Scatter: yaw rate vs position error (key coupling plot) ----------
    ax3 = fig.add_subplot(gs[1, :2])
    for name, d in data_map.items():
        yr  = np.abs(d["yaw_rate"])
        mag = np.linalg.norm(d["ep"], axis=1)
        ax3.scatter(yr, mag, s=1, color=COLORS[name], alpha=0.3, label=name)

        corr = metrics[name]["yaw_pos_corr"]
        # trend line
        z = np.polyfit(yr, mag, 1)
        p = np.poly1d(z)
        xs = np.linspace(yr.min(), yr.max(), 100)
        ax3.plot(xs, p(xs), color=COLORS[name], linewidth=2,
                 label=f"{name} trend (r={corr:.2f})")

    ax3.set_xlabel("|Yaw Rate| [rad/s]")
    ax3.set_ylabel("|Position Error| [m]")
    ax3.set_title("Cross-Coupling: |Yaw Rate| → |Position Error|\n"
                  "(slope near 0 = coupling compensated)")
    ax3.legend(markerscale=5)
    ax3.grid(True, alpha=0.3)

    # --- 4. Spin-regime RMSE bar chart ---------------------------------------
    ax4 = fig.add_subplot(gs[1, 2])
    spin_thresholds = [0, 10, 20, 25]
    names = list(data_map.keys())
    x = np.arange(len(spin_thresholds))
    w = 0.35
    for i, name in enumerate(names):
        d = data_map[name]
        yr = d["yaw_rate"]
        ep = d["ep"]
        rmses = []
        for th in spin_thresholds:
            mask = np.abs(yr) > th
            if mask.sum() > 5:
                rmses.append(pos_rmse(ep[mask]))
            else:
                rmses.append(float("nan"))
        ax4.bar(x + i * w, rmses, w, label=name, color=COLORS[name], alpha=0.8)

    ax4.set_xticks(x + w / 2)
    ax4.set_xticklabels([f"|yr|>{th}" for th in spin_thresholds], fontsize=8)
    ax4.set_ylabel("Pos RMSE [m]")
    ax4.set_title("RMSE by Spin Regime")
    ax4.legend()
    ax4.grid(True, alpha=0.3, axis="y")

    # --- 5. X/Y position traces ----------------------------------------------
    ax5 = fig.add_subplot(gs[2, :2])
    for name, d in data_map.items():
        p = d["p"]
        ax5.plot(p[:, 0], p[:, 1], color=COLORS[name], label=name, alpha=0.7)
    ax5.plot(*data_map[names[0]]["desired_p"][0, :2], "k*", markersize=12, label="target")
    ax5.set_xlabel("x [m]")
    ax5.set_ylabel("y [m]")
    ax5.set_title("XY Ground Track")
    ax5.axis("equal")
    ax5.legend()
    ax5.grid(True, alpha=0.3)

    # --- 6. Summary metric bar chart -----------------------------------------
    ax6 = fig.add_subplot(gs[2, 2])
    metric_labels = ["RMSE\n(all)", "RMSE\n(SS)", "Max\nErr", "Spin\nRMSE"]
    metric_keys   = ["rmse_all", "rmse_steady", "max_pos_err", "spin_rmse_hi"]
    x6 = np.arange(len(metric_labels))
    for i, name in enumerate(names):
        vals = [metrics[name].get(k, float("nan")) for k in metric_keys]
        ax6.bar(x6 + i * w, vals, w, label=name, color=COLORS[name], alpha=0.8)
    ax6.set_xticks(x6 + w / 2)
    ax6.set_xticklabels(metric_labels, fontsize=8)
    ax6.set_ylabel("Error [m]")
    ax6.set_title("Summary Metrics")
    ax6.legend()
    ax6.grid(True, alpha=0.3, axis="y")

    if save_dir:
        out = save_dir / "controller_comparison.png"
        fig.savefig(str(out), dpi=150, bbox_inches="tight")
        print(f"Saved: {out}")

    if show:
        plt.show()
    else:
        plt.close(fig)


# ---- Main -------------------------------------------------------------------

def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--no-show", action="store_true")
    p.add_argument("--save-dir", default=str(RESULTS_DIR))
    return p.parse_args()


def main():
    args = parse_args()
    can_show = (not args.no_show) and bool(os.environ.get("DISPLAY"))
    if not can_show:
        matplotlib.use("Agg")

    data_map = {}
    for name, path in CONTROLLERS.items():
        if not path.exists():
            print(f"[warn] missing: {path}, skipping {name}")
            continue
        d = np.load(path)
        data_map[name] = {
            "time":      d["time"],
            "p":         d["p"],
            "desired_p": d["desired_p"],
            "ep":        d["ep"],
            "yaw_rate":  d["yaw_rate"],
        }
        if "eR" in d:
            data_map[name]["eR"] = d["eR"]

    if not data_map:
        print("No result files found.")
        return

    metrics = {name: compute_metrics(d) for name, d in data_map.items()}
    print_scorecard(metrics)

    save_dir = Path(args.save_dir) if args.save_dir else None
    plot_all(data_map, metrics, save_dir=save_dir, show=can_show)


if __name__ == "__main__":
    main()
