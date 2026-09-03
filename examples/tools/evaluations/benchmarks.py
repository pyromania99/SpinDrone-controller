#!/usr/bin/env python3
"""
benchmarks.py  —  Compare Cascaded PD, Eigen, and LPV-MPC controllers.

Metrics
-------
  • Position RMSE (all / transient / steady-state)
  • Max position error
  • Settling time  (first t where |ep| stays < 0.10 m for 2 s)
  • Overshoot      (max |ep| after lift-off / initial |ep|)
  • Control effort (mean Σ motor_thrust²)
  • Peak motor thrust
  • Yaw-rate disturbance coupling (if yaw_rate present)

Outputs
-------
  results/benchmark_report.png   — 2-row, 4-col dashboard
  results/benchmark_metrics.csv  — tabular summary
  stdout                         — formatted scorecard

Usage
-----
  /usr/bin/python3.10 examples/tools/evaluations/benchmarks.py
  /usr/bin/python3.10 examples/tools/evaluations/benchmarks.py --no-show --t-limit 60
"""

from __future__ import annotations

import argparse
import csv
import os
from pathlib import Path

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.lines import Line2D

# ── paths ────────────────────────────────────────────────────────────────────

RESULTS_DIR = Path(__file__).resolve().parent.parent / "results"

CONTROLLERS: dict[str, Path] = {
    "Cascaded PD": RESULTS_DIR / "controller_eval" / "cascaded_pd" / "statistics" / "cascaded_pd_statistics.npz",
    "Eigen":       RESULTS_DIR / "controller_eval" / "eigen" / "statistics" / "eigen_statistics.npz",
    "LPV-MPC":     RESULTS_DIR / "controller_eval" / "lpv_mpc" / "statistics" / "lpv_mpc_statistics.npz",
}

COLORS: dict[str, str] = {
    "Cascaded PD": "#E74C3C",   # red
    "Eigen":       "#F39C12",   # amber
    "LPV-MPC":     "#2980B9",   # blue
}

MARKERS: dict[str, str] = {
    "Cascaded PD": "o",
    "Eigen":       "s",
    "LPV-MPC":     "^",
}

# ── CLI ──────────────────────────────────────────────────────────────────────

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="3-controller benchmark dashboard")
    p.add_argument("--no-show",  action="store_true",
                   help="Skip plt.show(); only save PNG")
    p.add_argument("--t-limit",  type=float, default=None,
                   help="Clip all data to this many seconds (default: full run)")
    p.add_argument("--settle-tol", type=float, default=0.10,
                   help="Position error threshold for settling time [m]")
    p.add_argument("--settle-dur", type=float, default=2.0,
                   help="How long error must stay below tol to count as settled [s]")
    p.add_argument("--liftoff-time", type=float, default=2.0,
                   help="Time [s] after which overshoot is calculated")
    p.add_argument("--out-dir", type=str, default=str(RESULTS_DIR))
    return p.parse_args()


# ── data loading ─────────────────────────────────────────────────────────────

def load(path: Path, t_limit: float | None) -> dict | None:
    """Load an npz file and clip to t_limit. Returns None if missing."""
    if not path.exists():
        print(f"  [warn] missing: {path}")
        return None
    raw = np.load(path)
    t   = raw["time"].astype(float)
    if t_limit is not None:
        mask = t <= t_limit
        t    = t[mask]
    else:
        mask = np.ones(len(t), dtype=bool)
    if len(t) < 10:
        print(f"  [warn] too few samples in {path.name}")
        return None

    d: dict = {"time": t}
    for key in raw.keys():
        arr = raw[key]
        if arr.ndim == 1 and arr.shape[0] == len(raw["time"]):
            d[key] = arr[mask]
        elif arr.ndim == 2 and arr.shape[0] == len(raw["time"]):
            d[key] = arr[mask]
        else:
            d[key] = arr          # scalar / differently-shaped arrays
    return d


# ── metric helpers ───────────────────────────────────────────────────────────

def pos_err(d: dict) -> np.ndarray:
    return np.linalg.norm(d["ep"], axis=1)


def rmse(v: np.ndarray) -> float:
    return float(np.sqrt(np.mean(v ** 2)))


def settling_time(t: np.ndarray, err: np.ndarray,
                  tol: float, dur: float) -> float:
    """First t where err stays < tol for `dur` seconds. Returns nan if never."""
    dt = float(t[1] - t[0]) if len(t) > 1 else 1e-3
    window = max(1, int(dur / dt))
    for i in range(len(err) - window):
        if np.all(err[i:i + window] < tol):
            return float(t[i])
    return float("nan")


def overshoot(err: np.ndarray, t: np.ndarray, liftoff_s: float) -> float:
    """
    Overshoot = max |ep| after liftoff / reference_dist.
    To avoid issues when starting at the target, we use max(err[0], 1.0).
    """
    post_mask = t > liftoff_s
    if post_mask.sum() == 0:
        return float("nan")
    ref_dist = max(float(err[0]), 1.0)
    return float(np.max(err[post_mask]) / ref_dist)


def control_effort(d: dict) -> float:
    """Mean Σ motor_thrust² across time."""
    mt = d.get("motor_thrusts")
    if mt is None or mt.ndim != 2:
        return float("nan")
    return float(np.mean(np.sum(mt ** 2, axis=1)))


def peak_motor_thrust(d: dict) -> float:
    mt = d.get("motor_thrusts")
    if mt is None:
        return float("nan")
    return float(np.max(mt))


def yaw_coupling(d: dict) -> float:
    """Pearson r between |yaw_rate| and |pos_err|. nan if no yaw_rate."""
    if "yaw_rate" not in d:
        return float("nan")
    yr  = np.abs(d["yaw_rate"])
    err = pos_err(d)
    if yr.std() < 1e-9:
        return float("nan")
    return float(np.corrcoef(yr, err)[0, 1])


def compute_metrics(d: dict, tol: float, dur: float, liftoff_s: float) -> dict:
    t   = d["time"]
    err = pos_err(d)
    n   = len(t)
    n_tr = n // 4          # fallback transient

    ts = settling_time(t, err, tol, dur)
    
    if np.isfinite(ts):
        n_ss = np.searchsorted(t, ts)
    else:
        n_ss = n // 2      # fallback steady-state

    # For safety if settling time is very late
    if n_ss >= n:
        n_ss = n - 1

    return {
        "duration_s":        float(t[-1]),
        "samples":           n,
        "rmse_all":          rmse(err),
        "rmse_transient":    rmse(err[:n_tr]),
        "rmse_steady":       rmse(err[n_ss:]),
        "max_pos_err":       float(np.max(err)),
        "final_pos_err":     float(err[-1]),
        "settling_time":     ts,
        "overshoot":         overshoot(err, t, liftoff_s),
        "control_effort":    control_effort(d),
        "peak_motor_thrust": peak_motor_thrust(d),
        "yaw_coupling":      yaw_coupling(d),
    }


# ── scorecard ────────────────────────────────────────────────────────────────

FIELDS = [
    ("Run duration [s]",               "duration_s",        ".1f"),
    ("Samples",                        "samples",           "d"),
    ("RMSE — full run [m]",            "rmse_all",          ".4f"),
    ("RMSE — transient (0–25%) [m]",   "rmse_transient",    ".4f"),
    ("RMSE — steady-state (50–) [m]",  "rmse_steady",       ".4f"),
    ("Max position error [m]",         "max_pos_err",       ".4f"),
    ("Final position error [m]",       "final_pos_err",     ".4f"),
    ("Settling time [s]",              "settling_time",     ".2f"),
    ("Overshoot ratio",                "overshoot",         ".3f"),
    ("Control effort [N²]",            "control_effort",    ".3f"),
    ("Peak motor thrust [N]",          "peak_motor_thrust", ".3f"),
    ("Yaw-coupling corr",              "yaw_coupling",      ".4f"),
]


def print_scorecard(metrics: dict[str, dict]) -> None:
    sep = "=" * 80
    names = list(metrics.keys())
    col_w = 16
    print(f"\n{sep}")
    print("CONTROLLER BENCHMARK SCORECARD")
    print(sep)
    header = f"{'Metric':<40}" + "".join(f"{n:>{col_w}}" for n in names)
    print(header)
    print("-" * len(header))
    for label, key, fmt in FIELDS:
        row = f"{label:<40}"
        for n in names:
            v = metrics[n].get(key, float("nan"))
            try:
                if fmt == "d":
                    row += f"{int(v):>{col_w}}"
                else:
                    row += f"{format(v, fmt):>{col_w}}"
            except (ValueError, TypeError):
                row += f"{'—':>{col_w}}"
        print(row)
    print(sep)
    # highlight winners
    print("\n★ Best per metric:")
    lower_is_better = {"rmse_all","rmse_transient","rmse_steady","max_pos_err",
                       "final_pos_err","settling_time","overshoot",
                       "control_effort","peak_motor_thrust"}
    for label, key, _ in FIELDS:
        vals = {n: metrics[n].get(key, float("nan")) for n in names}
        finite = {n: v for n, v in vals.items() if np.isfinite(v)}
        if not finite:
            continue
        if key == "yaw_coupling":
            winner = min(finite, key=lambda n: abs(finite[n]))
        elif key in lower_is_better:
            winner = min(finite, key=lambda n: finite[n])
        else:
            winner = max(finite, key=lambda n: finite[n])
        print(f"  {label:<42} → {winner}")
    print(sep + "\n")


def write_csv(metrics: dict[str, dict], out_path: Path) -> None:
    names = list(metrics.keys())
    with open(str(out_path), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["metric"] + names)
        for _, key, fmt in FIELDS:
            row = [key]
            for n in names:
                v = metrics[n].get(key, float("nan"))
                try:
                    row.append(format(v, fmt) if fmt != "d" else str(int(v)))
                except (ValueError, TypeError):
                    row.append("nan")
            w.writerow(row)
    print(f"CSV  → {out_path}")


# ── plotting ─────────────────────────────────────────────────────────────────

def _legend_handles(names):
    return [Line2D([0], [0], color=COLORS[n], linewidth=2.5, label=n) for n in names]


def plot_dashboard(data_map: dict[str, dict],
                   metrics:  dict[str, dict],
                   out_dir:  Path,
                   show:     bool) -> None:

    names = list(data_map.keys())
    fig = plt.figure(figsize=(20, 12))
    fig.patch.set_facecolor("#0F1117")
    fig.suptitle(
        "Controller Benchmark Dashboard  —  Cascaded PD  |  Eigen  |  LPV-MPC",
        fontsize=16, fontweight="bold", color="white", y=0.98,
    )
    gs = gridspec.GridSpec(2, 4, figure=fig, hspace=0.48, wspace=0.38,
                           left=0.06, right=0.97, top=0.93, bottom=0.07)

    bg  = "#1A1D2E"
    grid_kw = dict(color="#2A2D3E", linestyle="--", linewidth=0.6, alpha=0.7)

    def _ax(row, col, colspan=1):
        ax = fig.add_subplot(gs[row, col:col+colspan])
        ax.set_facecolor(bg)
        ax.tick_params(colors="white", labelsize=9)
        for spine in ax.spines.values():
            spine.set_edgecolor("#333650")
        ax.grid(**grid_kw)
        ax.title.set_color("white")
        ax.xaxis.label.set_color("white")
        ax.yaxis.label.set_color("white")
        return ax

    # ── 1. Position error magnitude vs time ──────────────────────────────────
    ax1 = _ax(0, 0, colspan=2)
    for n, d in data_map.items():
        err = pos_err(d)
        ax1.plot(d["time"], err, color=COLORS[n], linewidth=1.3, alpha=0.9, label=n)
    ax1.set_xlabel("Time [s]")
    ax1.set_ylabel("|Position Error| [m]")
    ax1.set_title("Position Error vs Time")
    ax1.legend(handles=_legend_handles(names), fontsize=9,
               facecolor="#1A1D2E", edgecolor="#333650", labelcolor="white")

    # ── 2. XY ground track ───────────────────────────────────────────────────
    ax2 = _ax(0, 2)
    for n, d in data_map.items():
        p = d["p"]
        ax2.plot(p[:, 0], p[:, 1], color=COLORS[n], linewidth=1.0, alpha=0.8)
        ax2.plot(p[0, 0], p[0, 1], "o", color=COLORS[n], markersize=5)
    # target
    target = list(data_map.values())[0]["desired_p"][0, :2]
    ax2.plot(*target, "w*", markersize=12, label="target", zorder=5)
    ax2.set_xlabel("x [m]")
    ax2.set_ylabel("y [m]")
    ax2.set_title("XY Ground Track")
    ax2.set_aspect("equal")
    ax2.legend(handles=_legend_handles(names) +
               [Line2D([0],[0],marker="*",color="w",linestyle="none",
                        markersize=10,label="target")],
               fontsize=8, facecolor="#1A1D2E", edgecolor="#333650", labelcolor="white")

    # ── 3. Altitude vs time ──────────────────────────────────────────────────
    ax3 = _ax(0, 3)
    for n, d in data_map.items():
        ax3.plot(d["time"], d["p"][:, 2], color=COLORS[n], linewidth=1.2, alpha=0.9)
    # target altitude line
    tgt_z = float(list(data_map.values())[0]["desired_p"][0, 2])
    ax3.axhline(tgt_z, color="white", linewidth=1.0, linestyle="--", alpha=0.6, label=f"target {tgt_z:.1f}m")
    ax3.set_xlabel("Time [s]")
    ax3.set_ylabel("Altitude z [m]")
    ax3.set_title("Altitude Tracking")
    ax3.legend(handles=_legend_handles(names) +
               [Line2D([0],[0],color="white",linestyle="--",label=f"target {tgt_z:.1f}m")],
               fontsize=8, facecolor="#1A1D2E", edgecolor="#333650", labelcolor="white")

    # ── 4. RMSE summary bar chart ─────────────────────────────────────────────
    ax4 = _ax(1, 0)
    metric_keys = ["rmse_transient", "rmse_steady", "rmse_all"]
    metric_lbls = ["Transient\n(0–25%)", "Steady-state\n(50%–end)", "Full run"]
    x4 = np.arange(len(metric_keys))
    w4 = 0.25
    for i, n in enumerate(names):
        vals = [metrics[n][k] for k in metric_keys]
        bars = ax4.bar(x4 + i * w4, vals, w4, color=COLORS[n], alpha=0.88, label=n)
        for bar, v in zip(bars, vals):
            ax4.text(bar.get_x() + bar.get_width() / 2,
                     bar.get_height() + 0.005,
                     f"{v:.3f}", ha="center", va="bottom",
                     fontsize=7, color="white")
    ax4.set_xticks(x4 + w4)
    ax4.set_xticklabels(metric_lbls, fontsize=9, color="white")
    ax4.set_ylabel("RMSE [m]")
    ax4.set_title("Position RMSE Breakdown")
    ax4.legend(fontsize=8, facecolor="#1A1D2E", edgecolor="#333650", labelcolor="white")

    # ── 5. Control effort vs time ─────────────────────────────────────────────
    ax5 = _ax(1, 1)
    for n, d in data_map.items():
        mt = d.get("motor_thrusts")
        if mt is not None and mt.ndim == 2:
            effort = np.sum(mt ** 2, axis=1)
            # smooth with rolling mean
            k = max(1, len(effort) // 500)
            smoothed = np.convolve(effort, np.ones(k) / k, mode="same")
            ax5.plot(d["time"], smoothed, color=COLORS[n], linewidth=1.2, alpha=0.9)
    ax5.set_xlabel("Time [s]")
    ax5.set_ylabel("Σ motor_thrust² [N²]")
    ax5.set_title("Control Effort (smoothed)")
    ax5.legend(handles=_legend_handles(names), fontsize=8,
               facecolor="#1A1D2E", edgecolor="#333650", labelcolor="white")

    # ── 6. Settling-time & Overshoot bar chart ────────────────────────────────
    ax6 = _ax(1, 2)
    settle_vals = [metrics[n]["settling_time"] for n in names]
    colors6 = [COLORS[n] for n in names]
    # replace nan with bar height 0 but annotate "—"
    display_vals = [v if np.isfinite(v) else 0.0 for v in settle_vals]
    bars6 = ax6.bar(names, display_vals, color=colors6, alpha=0.85)
    for bar, v in zip(bars6, settle_vals):
        label = f"{v:.1f}s" if np.isfinite(v) else "never"
        ax6.text(bar.get_x() + bar.get_width() / 2,
                 bar.get_height() + 0.3,
                 label, ha="center", va="bottom",
                 fontsize=10, color="white", fontweight="bold")
    ax6.set_ylabel("Settling Time [s]")
    ax6.set_title("Settling Time  (|ep| < 0.10 m for 2 s)")
    ax6.tick_params(axis="x", colors="white", labelsize=9)

    # ── 7. Comprehensive metric radar / spider — replaced by bar comparison ───
    ax7 = _ax(1, 3)
    # Normalise each metric to [0,1] (lower-is-better → invert)
    spider_keys = ["rmse_all", "max_pos_err", "control_effort", "settling_time", "peak_motor_thrust"]
    spider_lbls = ["RMSE\n(all)", "Max\nerror", "Ctrl\neffort", "Settle\ntime", "Peak\nthrust"]
    x7 = np.arange(len(spider_keys))
    w7 = 0.25

    # collect values; nan → worst finite value for display
    raw_vals: dict[str, list] = {n: [metrics[n].get(k, float("nan")) for k in spider_keys] for n in names}
    for ki in range(len(spider_keys)):
        col = [raw_vals[n][ki] for n in names]
        finite_col = [v for v in col if np.isfinite(v)]
        worst = max(finite_col) if finite_col else 1.0
        for n in names:
            if not np.isfinite(raw_vals[n][ki]):
                raw_vals[n][ki] = worst

    # normalise to [0,1]
    for ki in range(len(spider_keys)):
        col_max = max(raw_vals[n][ki] for n in names)
        col_min = min(raw_vals[n][ki] for n in names)
        span = col_max - col_min if col_max > col_min else 1.0
        for n in names:
            raw_vals[n][ki] = (raw_vals[n][ki] - col_min) / span  # 0=best, 1=worst

    for i, n in enumerate(names):
        ax7.bar(x7 + i * w7, raw_vals[n], w7, color=COLORS[n], alpha=0.85, label=n)

    ax7.set_xticks(x7 + w7)
    ax7.set_xticklabels(spider_lbls, fontsize=8, color="white")
    ax7.set_ylabel("Normalised score (lower = better)")
    ax7.set_title("Relative Performance\n(0 = best, 1 = worst)")
    ax7.legend(fontsize=8, facecolor="#1A1D2E", edgecolor="#333650", labelcolor="white")

    out_png = out_dir / "benchmark_report.png"
    fig.savefig(str(out_png), dpi=150, bbox_inches="tight", facecolor=fig.get_facecolor())
    print(f"PNG  → {out_png}")

    if show:
        plt.show()
    else:
        plt.close(fig)


# ── main ─────────────────────────────────────────────────────────────────────

def main() -> None:
    args = parse_args()
    can_show = (not args.no_show) and bool(os.environ.get("DISPLAY"))
    if not can_show:
        matplotlib.use("Agg")

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    data_map: dict[str, dict] = {}
    for name, path in CONTROLLERS.items():
        d = load(path, args.t_limit)
        if d is not None:
            data_map[name] = d

    if not data_map:
        print("No controller result files found.  Run each controller first.")
        return

    metrics = {
        n: compute_metrics(d, args.settle_tol, args.settle_dur, args.liftoff_time)
        for n, d in data_map.items()
    }

    print_scorecard(metrics)
    write_csv(metrics, out_dir / "benchmark_metrics.csv")
    plot_dashboard(data_map, metrics, out_dir=out_dir, show=can_show)


if __name__ == "__main__":
    main()
