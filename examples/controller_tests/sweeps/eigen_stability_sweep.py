#!/usr/bin/env python
"""
Eigen M_dyn eigenvalue-placement stability sweep in Isaac Sim.

M_dyn is constructed from (ωn, ζ) via standard second-order pole placement:

    wn_mdyn  = ζ · ωn          (real part of closed-loop eigenvalue)
    b_mdyn   = ωn · √(1 − ζ²)  (imaginary part; clamped to 0 when ζ ≥ 1)

    M_dyn = [[-wn_mdyn, -b_mdyn, 0],
             [ b_mdyn,  -wn_mdyn, 0],
             [0,        0,        -β]]

    Eigenvalues of the attitude block: λ = -ζ·ωn ± j·ωn·√(1−ζ²)

With ζ fixed (default 1.0 = critical damping), a sweep over ωn traces the
negative real axis of the complex plane, giving a clean 1-D stability profile.

Outer PD gains (Kp_att, Kd_att) are held at a fixed baseline throughout the
M_dyn sweep — this isolates M_dyn eigenvalue placement from the outer loop.

Parallel structure : one drone per ωn value, all simulated simultaneously.
Serial  structure  : one pass per motor_moment.

Stability score: future-biased mean ‖position error‖
    score = Σ w(t)·‖ep(t)‖ / Σ w(t),   w(t) = t / T
Lower score = better.  OOB drones score ∞.

Usage:
isaac_run examples/controller_tests/eigen_stability_sweep.py \\
    --motor-moments 1e-7 2.7e-7 4e-7 \\
    --omega-n-values 2 4 6 8 10 12 \\
    --zeta-values 0.7 1.0 1.3 \\
    --duration 15

isaac_run examples/controller_tests/eigen_stability_sweep.py \
    --a-values 2 4 6 8 10 \
    --b-values 0 2 4 6 8 10 \
    --motor-moments 1e-7 2.7e-7 4e-7 \
    --duration 15 \

Outputs (per motor_moment):
    results/eigen_sweep/mm_<km>/score_plot.png                      (single zeta mode)
    results/eigen_sweep/mm_<km>/scores.csv                          (single zeta mode)
    results/eigen_sweep/mm_<km>/on<ωn>_statistics.npz               (single zeta mode)

    results/eigen_sweep/mm_<km>/zeta_<ζ>/scores.csv                 (multi zeta mode)
    results/eigen_sweep/mm_<km>/zeta_<ζ>/on<ωn>_statistics.npz      (multi zeta mode)
    results/eigen_sweep/mm_<km>/eta_compare_heatmap.png             (multi zeta mode)
    results/eigen_sweep/mm_<km>/eta_compare_scores.csv              (multi zeta mode)
    results/eigen_sweep/mm_<km>/eta_compare_effort_heatmap.png      (multi zeta mode)
    results/eigen_sweep/mm_<km>/eta_compare_effort.csv              (multi zeta mode)
"""

from __future__ import annotations

import argparse
import math
import os
import sys
from pathlib import Path



import carb
from isaacsim import SimulationApp


# ─────────────────────────── CLI ────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(
        description="Eigen M_dyn eigenvalue-placement stability sweep"
    )
    p.add_argument(
        "--motor-moments", nargs="+", type=float,
        default=[1e-7, 2.7e-7, 4e-7],
        help="km (motor moment) values to sweep — outer serial loop",
    )
    p.add_argument(
        "--omega-n-values", nargs="+", type=float,
        default=[2.0, 4.0, 6.0, 8.0, 10.0, 12.0],
        help="Closed-loop natural frequency ωn values — parallel drone axis",
    )
    p.add_argument(
        "--zeta", type=float, default=1.0,
        help="Fixed damping ratio ζ for all runs (used when --zeta-values is not set)",
    )
    p.add_argument(
        "--zeta-values", "--eta-values", nargs="+", type=float, default=None,
        help="Optional damping-ratio list to sweep (aliases: --zeta-values, --eta-values)",
    )
    # ── Direct M_dyn (a, b) sweep ────────────────────────────────────────────
    p.add_argument(
        "--a-values", nargs="+", type=float, default=None,
        help="M_dyn diagonal values (wn_mdyn = a) to sweep — activates ab-sweep mode. "
             "Drones are spawned one-per-a (parallel). "
             "Example: --a-values 2 4 6 8",
    )
    p.add_argument(
        "--b-values", nargs="+", type=float, default=None,
        help="M_dyn off-diagonal values (b_mdyn = b) to sweep in ab-sweep mode — "
             "inner serial loop over b, parallel over a. "
             "Example: --b-values 0 2 4 6",
    )
    # ────────────────────────────────────────────────────────────────────────
    p.add_argument(
        "--kp-att", type=float, default=0.7,
        help="Fixed Kp_att outer attitude gain (baseline, not swept here)",
    )
    p.add_argument(
        "--kd-att", type=float, default=10.0,
        help="Fixed Kd_att outer attitude gain (baseline, not swept here)",
    )
    p.add_argument("--duration",        type=float, default=15.0,
                   help="Simulation seconds per motor_moment pass")
    p.add_argument("--spacing",         type=float, default=4.0,
                   help="Metres between drone spawn points along X")
    p.add_argument(
        "--bounds", nargs=6, type=float,
        default=[-12., 12., -12., 12., 0., 20.],
        metavar=("XMIN", "XMAX", "YMIN", "YMAX", "ZMIN", "ZMAX"),
    )
    p.add_argument("--target-altitude", type=float, default=1.5)
    p.add_argument(
        "--goal-mode", type=str,
        choices=["spawn_xy", "random_xy"], default="spawn_xy",
    )
    p.add_argument("--goal-radius",     type=float, default=2.0)
    p.add_argument("--goal-seed",       type=int,   default=7)
    p.add_argument("--headless",        action="store_true", default=False)
    p.add_argument("--no-headless",     dest="headless", action="store_false")
    return p.parse_args()


ARGS = parse_args()
simulation_app = SimulationApp(
    {
        "headless": ARGS.headless,
        "hide_ui": True if ARGS.headless else None,
        "disable_viewport_updates": True if ARGS.headless else False,
        "multi_gpu": False,
        "max_gpu_count": 1,
        "active_gpu": 0,

    }
)


# ─────────────────────────── M_dyn eigenvalue placement ──────────────────────

def mdyn_params_for(omega_n: float, zeta: float) -> tuple[float, float]:
    """
    Return (wn_mdyn, b_mdyn) so that M_dyn's 2×2 attitude block has eigenvalues:
        λ = -ζ·ωn ± j·ωn·√(1 − ζ²)

    For ζ ≥ 1 (overdamped / critical), b_mdyn is clamped to 0
    (purely real double eigenvalue at -ζ·ωn).
    """
    wn_mdyn = zeta * omega_n
    b_mdyn  = omega_n * math.sqrt(max(1.0 - zeta ** 2, 0.0))
    return wn_mdyn, b_mdyn


# ─────────────────────────── Goal builder ────────────────────────────────────

def build_target_position(spawn_pos: list, rng) -> tuple:
    if ARGS.goal_mode == "spawn_xy":
        return (spawn_pos[0], spawn_pos[1], float(ARGS.target_altitude))
    theta = float(rng.uniform(0.0, 2.0 * math.pi))
    r     = float(ARGS.goal_radius)
    return (
        spawn_pos[0] + r * math.cos(theta),
        spawn_pos[1] + r * math.sin(theta),
        float(ARGS.target_altitude),
    )


# ─────────────────────────── Late imports (after SimulationApp) ──────────────

import numpy as np
import omni.timeline

from isaacsim.core.api import World
from scipy.spatial.transform import Rotation

for _parent in Path(__file__).resolve().parents:
    _candidate = _parent / "extensions" / "pegasus.simulator"
    if _candidate.exists():
        if str(_candidate) not in sys.path:
            sys.path.insert(0, str(_candidate))
        break

from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.backends import Backend
from pegasus.simulator.logic.thrusters import QuadraticThrustCurve
from pegasus.simulator.logic.state import State

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "utils"))
from eigen_controller import EigenController


# ─────────────────────────── Telemetry / OOB backend ─────────────────────────

class BoundedTelemetryBackend(Backend):
    """Wraps EigenController; tracks OOB and position."""

    def __init__(self, inner: EigenController, results_file: str):
        super().__init__(config=None)
        self.inner        = inner
        self.results_file = results_file
        self.total_time   = 0.0
        self.oob_time     = float("nan")
        self.is_oob       = False
        self._stopped     = False
        self.position     = np.zeros(3)

    def initialize(self, vehicle):
        super().initialize(vehicle)
        self.inner.initialize(vehicle)

    def start(self):
        self.total_time = 0.0
        self.oob_time   = float("nan")
        self.is_oob     = False
        self._stopped   = False
        self.inner.start()

    def stop(self):
        if self._stopped:
            return
        self._stopped = True
        self.inner.stop()
        # Append oob_time to saved npz
        if os.path.isfile(self.results_file):
            try:
                data = dict(np.load(self.results_file))
                data["oob_time"] = np.array(self.oob_time)
                np.savez(self.results_file, **data)
            except Exception as exc:
                carb.log_warn(f"[sweep] oob_time append failed: {exc}")

    def reset(self):
        self.inner.reset()

    def update_sensor(self, sensor_type, data):
        if not self.is_oob:
            self.inner.update_sensor(sensor_type, data)

    def update_graphical_sensor(self, sensor_type, data):
        if not self.is_oob:
            self.inner.update_graphical_sensor(sensor_type, data)

    def update_state(self, state: State):
        self.position = state.position.copy()
        if not self.is_oob:
            self.inner.update_state(state)

    def input_reference(self):
        return [0.0, 0.0, 0.0, 0.0] if self.is_oob else self.inner.input_reference()

    def update(self, dt: float):
        if self.is_oob:
            return
        self.total_time += dt
        self.inner.update(dt)

    def check_oob(self, bounds) -> bool:
        if self.is_oob:
            return False
        x, y, z = self.position
        if not (bounds[0] < x < bounds[1]
                and bounds[2] < y < bounds[3]
                and bounds[4] < z < bounds[5]):
            self.is_oob   = True
            self.oob_time = self.total_time
            carb.log_warn(f"[sweep] OOB at {self.position} t={self.total_time:.2f}s")
            return True
        return False


# ─────────────────────────── Score computation ───────────────────────────────

# Worst-case position error sentinel: bounding-box diagonal.
# Used to penalise the missing tail when a drone goes OOB early.
_BOUNDS_DIAGONAL = float(
    np.linalg.norm(np.array([
        ARGS.bounds[1] - ARGS.bounds[0],
        ARGS.bounds[3] - ARGS.bounds[2],
        ARGS.bounds[5] - ARGS.bounds[4],
    ]))
)


def stability_score(npz_path: str, T_total: float | None = None) -> float:
    """
    Future-biased mean ‖position error‖ over the full simulation window.

    OOB penalty
    -----------
    If the drone went out-of-bounds before T_total, the missing tail
    [t_oob, T_total] is padded with the bounding-box diagonal (worst-case
    reachable error) before computing the weighted mean.  This guarantees
    that an early crash always scores *worse* than a drone that hovered
    imperfectly for the full duration.

    w(t) = t / T_total  (later timesteps weighted more).
    Returns inf on missing file, < 2 samples, or non-finite values.
    """
    try:
        data = np.load(npz_path)
        ep   = data["ep"]       # (N, 3)
        t    = data["time"]     # (N,)
        if len(t) < 2:
            return np.inf

        oob_time       = float(data["oob_time"]) if "oob_time" in data else float("nan")
        drone_went_oob = np.isfinite(oob_time)

        # Scoring window: prefer the caller-supplied duration; fall back to
        # the last recorded timestep when not available.
        if T_total is None or T_total < 1e-6:
            T_total = float(t[-1])
        if T_total < 1e-6:
            return np.inf

        errors  = np.linalg.norm(ep, axis=1)
        if not np.all(np.isfinite(errors)):
            return np.inf

        t_arr   = t.copy()
        err_arr = errors.copy()

        # Pad the tail with worst-case error if the drone exited early.
        if drone_went_oob and oob_time < T_total - 1e-6:
            dt_approx = float(t[1] - t[0]) if len(t) > 1 else (1.0 / 60.0)
            pad_t     = np.arange(oob_time + dt_approx, T_total, dt_approx)
            if len(pad_t) > 0:
                t_arr   = np.concatenate([t_arr,   pad_t])
                err_arr = np.concatenate([err_arr,
                                          np.full(len(pad_t), _BOUNDS_DIAGONAL)])

        w     = t_arr / T_total
        w_sum = float(np.sum(w))
        if w_sum < 1e-12:
            return np.inf
        score = float(np.dot(w, err_arr) / w_sum)
        return score if np.isfinite(score) else np.inf
    except Exception as exc:
        carb.log_warn(f"[sweep] score read failed ({npz_path}): {exc}")
        return np.inf


def control_effort_score(npz_path: str, T_total: float | None = None) -> float:
    """
    Future-biased mean control effort based on per-motor thrusts.
    effort(t) = sum_i u_i(t)^2, where u_i are motor thrusts [N].

    OOB penalty
    -----------
    The tail after t_oob is padded with the *maximum* finite effort seen
    during the run so that an early crash never looks artificially cheap.

    Returns inf on missing file, < 2 samples, or non-finite values.
    """
    try:
        data = np.load(npz_path)
        t    = data["time"]
        if len(t) < 2:
            return np.inf

        oob_time       = float(data["oob_time"]) if "oob_time" in data else float("nan")
        drone_went_oob = np.isfinite(oob_time)

        if T_total is None or T_total < 1e-6:
            T_total = float(t[-1])
        if T_total < 1e-6:
            return np.inf

        if "motor_thrusts" in data:
            u = np.asarray(data["motor_thrusts"], dtype=float)
            if u.ndim != 2 or u.shape[0] != len(t):
                return np.inf
            effort = np.sum(np.square(u), axis=1)
        elif "thrust" in data:
            thrust = np.asarray(data["thrust"], dtype=float)
            if thrust.shape[0] != len(t):
                return np.inf
            effort = np.square(thrust)
        else:
            return np.inf

        if not np.all(np.isfinite(effort)):
            return np.inf

        t_arr   = t.copy()
        eff_arr = effort.copy()

        # Pad the tail with peak observed effort if the drone exited early.
        if drone_went_oob and oob_time < T_total - 1e-6:
            dt_approx = float(t[1] - t[0]) if len(t) > 1 else (1.0 / 60.0)
            pad_t     = np.arange(oob_time + dt_approx, T_total, dt_approx)
            if len(pad_t) > 0:
                worst_eff = float(np.max(eff_arr)) if eff_arr.size else 0.0
                t_arr     = np.concatenate([t_arr,  pad_t])
                eff_arr   = np.concatenate([eff_arr,
                                            np.full(len(pad_t), worst_eff)])

        w     = t_arr / T_total
        w_sum = float(np.sum(w))
        if w_sum < 1e-12:
            return np.inf
        score = float(np.dot(w, eff_arr) / w_sum)
        return score if np.isfinite(score) else np.inf
    except Exception as exc:
        carb.log_warn(f"[sweep] effort score read failed ({npz_path}): {exc}")
        return np.inf


# ─────────────────────────── Score line plot ─────────────────────────────────

def save_score_plot(on_vals, scores, zeta, mm, out_dir: Path):
    """Bar/line chart of stability score vs ωn for one motor_moment."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, ax = plt.subplots(figsize=(max(8, len(on_vals) * 1.0 + 3), 5))

        finite_scores = [s if np.isfinite(s) else None for s in scores]
        colors = [
            "dimgray" if s is None else
            "green"   if s == min((x for x in finite_scores if x is not None), default=0) else
            "steelblue"
            for s in finite_scores
        ]

        bars = ax.bar(range(len(on_vals)), [s if s is not None else 0 for s in finite_scores],
                      color=colors, edgecolor="black", linewidth=0.7)

        # Mark OOB bars with a label
        for i, s in enumerate(scores):
            if not np.isfinite(s):
                ax.text(i, 0.05, "∞\n(OOB)", ha="center", va="bottom",
                        fontsize=9, color="white", fontweight="bold")

        ax.set_xticks(range(len(on_vals)))
        ax.set_xticklabels(
            [f"ωn={v:.1f}\nwn={zeta*v:.2f}\nb={v*math.sqrt(max(1-zeta**2,0)):.2f}"
             for v in on_vals],
            fontsize=8,
        )
        ax.set_xlabel("Closed-loop natural frequency ωn  (→ M_dyn wn_mdyn, b_mdyn)", fontsize=11)
        ax.set_ylabel("Stability score  [future-biased mean ‖ep‖  m]", fontsize=11)
        ax.set_title(
            f"Eigen M_dyn Eigenvalue Sweep   |   km = {mm:.2e}\n"
            f"ζ = {zeta:.2f} (fixed)   |   Kp_att = {ARGS.kp_att}   Kd_att = {ARGS.kd_att}   "
            f"|   lower score = more stable",
            fontsize=11,
        )

        # Annotate best bar
        fin_idx = [i for i, s in enumerate(scores) if np.isfinite(s)]
        if fin_idx:
            best_i = min(fin_idx, key=lambda i: scores[i])
            ax.bar(best_i, scores[best_i], color="gold", edgecolor="black",
                   linewidth=1.2, zorder=5,
                   label=f"★ Best: ωn={on_vals[best_i]:.1f}  score={scores[best_i]:.4f}")
            ax.legend(fontsize=9)

        ax.set_ylim(bottom=0)
        fig.tight_layout()
        path = out_dir / "score_plot.png"
        fig.savefig(str(path), dpi=150, bbox_inches="tight")
        plt.close(fig)
        carb.log_warn(f"[sweep] score plot → {path}")
    except Exception as exc:
        carb.log_warn(f"[sweep] score plot failed: {exc}")


def save_csv(on_vals, scores, zeta, out_dir: Path):
    import csv
    path = out_dir / "scores.csv"
    with open(str(path), "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([f"omega_n \\ zeta", f"{zeta:.2f}"])
        for i, on in enumerate(on_vals):
            writer.writerow([
                f"{on:.2f}",
                f"{scores[i]:.6f}" if np.isfinite(scores[i]) else "inf",
            ])
    carb.log_warn(f"[sweep] scores CSV → {path}")


def save_multi_zeta_heatmap(on_vals, zeta_vals, zeta_to_scores, mm, out_dir: Path):
    """Heatmap comparing score across ζ (rows) and ωn (cols)."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import matplotlib.colors as mc

        scores = np.array([zeta_to_scores[zeta] for zeta in zeta_vals], dtype=float)
        n_z, n_on = scores.shape

        display = scores.copy()
        finite = np.isfinite(display)
        finite_vals = display[finite]
        if finite_vals.size:
            vmin = max(float(np.min(finite_vals)), 1e-5)
            vmax = float(np.max(finite_vals))
            if vmax <= vmin:
                vmax = vmin * 1.01
            sentinel = vmax * 1.5
        else:
            vmin, vmax, sentinel = 1e-5, 1.0, 2.0

        display[~finite] = sentinel

        fig, ax = plt.subplots(figsize=(max(8, n_on * 1.3 + 3), max(4.8, n_z * 0.9 + 2)))

        cmap = plt.cm.RdYlGn_r.copy()
        cmap.set_over("dimgray")
        norm = mc.LogNorm(vmin=vmin, vmax=vmax)

        im = ax.imshow(display, aspect="auto", origin="lower", cmap=cmap, norm=norm)
        plt.colorbar(
            im,
            ax=ax,
            extend="max",
            label="Stability score  [future-biased mean ‖ep‖  m]",
        )

        ax.set_xticks(range(n_on))
        ax.set_xticklabels([f"ωn={v:.1f}" for v in on_vals], fontsize=9)
        ax.set_yticks(range(n_z))
        ax.set_yticklabels([f"ζ={z:.2f}" for z in zeta_vals], fontsize=9)
        ax.set_xlabel("Closed-loop natural frequency ωn", fontsize=11)
        ax.set_ylabel("Damping ratio ζ", fontsize=11)
        ax.set_title(
            f"Eta/Zeta Heatmap   |   km = {mm:.2e}\n"
            f"Kp_att = {ARGS.kp_att}   Kd_att = {ARGS.kd_att}   |   lower score = more stable",
            fontsize=11,
        )

        for i_z, zeta in enumerate(zeta_vals):
            for i_on, omega_n in enumerate(on_vals):
                val = scores[i_z, i_on]
                a_val = zeta * omega_n
                b_val = omega_n * math.sqrt(max(1.0 - zeta ** 2, 0.0))
                score_txt = "∞" if not np.isfinite(val) else f"{val:.3f}"
                text = f"{score_txt}\na={a_val:.2f}\nb={b_val:.2f}"
                text_color = "white" if not np.isfinite(val) else "black"
                ax.text(i_on, i_z, text, ha="center", va="center",
                        fontsize=7, color=text_color, fontweight="bold", linespacing=1.25)

        if finite.any():
            best_idx = np.argmin(np.where(np.isfinite(scores), scores, np.inf))
            best_z, best_on = np.unravel_index(best_idx, scores.shape)
            ax.scatter(best_on, best_z, marker="*", s=350, color="gold",
                       edgecolors="black", linewidths=0.9, zorder=6)

        fig.tight_layout()

        path = out_dir / "eta_compare_heatmap.png"
        fig.savefig(str(path), dpi=150, bbox_inches="tight")
        plt.close(fig)
        carb.log_warn(f"[sweep] multi-zeta heatmap → {path}")
    except Exception as exc:
        carb.log_warn(f"[sweep] multi-zeta heatmap failed: {exc}")


def save_multi_zeta_csv(on_vals, zeta_vals, zeta_to_scores, out_dir: Path):
    import csv

    path = out_dir / "eta_compare_scores.csv"
    with open(str(path), "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["omega_n"] + [f"zeta_{z:.3f}" for z in zeta_vals])
        for i, on in enumerate(on_vals):
            row = [f"{on:.2f}"]
            for zeta in zeta_vals:
                score = float(zeta_to_scores[zeta][i])
                row.append(f"{score:.6f}" if np.isfinite(score) else "inf")
            writer.writerow(row)
    carb.log_warn(f"[sweep] multi-zeta CSV → {path}")


def save_multi_zeta_effort_heatmap(on_vals, zeta_vals, zeta_to_efforts, mm, out_dir: Path):
    """Heatmap comparing control effort across ζ (rows) and ωn (cols)."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import matplotlib.colors as mc

        efforts = np.array([zeta_to_efforts[zeta] for zeta in zeta_vals], dtype=float)
        n_z, n_on = efforts.shape

        display = efforts.copy()
        finite = np.isfinite(display)
        finite_vals = display[finite]
        if finite_vals.size:
            vmin = max(float(np.min(finite_vals)), 1e-8)
            vmax = float(np.max(finite_vals))
            if vmax <= vmin:
                vmax = vmin * 1.01
            sentinel = vmax * 1.5
        else:
            vmin, vmax, sentinel = 1e-8, 1.0, 2.0

        display[~finite] = sentinel

        fig, ax = plt.subplots(figsize=(max(8, n_on * 1.3 + 3), max(4.8, n_z * 0.9 + 2)))
        cmap = plt.cm.viridis.copy()
        cmap.set_over("dimgray")
        norm = mc.LogNorm(vmin=vmin, vmax=vmax)

        im = ax.imshow(display, aspect="auto", origin="lower", cmap=cmap, norm=norm)
        plt.colorbar(
            im,
            ax=ax,
            extend="max",
            label="Control effort  [future-biased mean Σ motor_thrust^2]",
        )

        ax.set_xticks(range(n_on))
        ax.set_xticklabels([f"ωn={v:.1f}" for v in on_vals], fontsize=9)
        ax.set_yticks(range(n_z))
        ax.set_yticklabels([f"ζ={z:.2f}" for z in zeta_vals], fontsize=9)
        ax.set_xlabel("Closed-loop natural frequency ωn", fontsize=11)
        ax.set_ylabel("Damping ratio ζ", fontsize=11)
        ax.set_title(
            f"Eta/Zeta Control Effort Heatmap   |   km = {mm:.2e}\n"
            f"Kp_att = {ARGS.kp_att}   Kd_att = {ARGS.kd_att}   |   lower = less actuation effort",
            fontsize=11,
        )

        for i_z, zeta in enumerate(zeta_vals):
            for i_on, omega_n in enumerate(on_vals):
                val = efforts[i_z, i_on]
                a_val = zeta * omega_n
                b_val = omega_n * math.sqrt(max(1.0 - zeta ** 2, 0.0))
                effort_txt = "∞" if not np.isfinite(val) else f"{val:.3f}"
                text = f"{effort_txt}\na={a_val:.2f}\nb={b_val:.2f}"
                text_color = "white" if not np.isfinite(val) else "black"
                ax.text(i_on, i_z, text, ha="center", va="center",
                        fontsize=7, color=text_color, fontweight="bold", linespacing=1.25)

        if finite.any():
            best_idx = np.argmin(np.where(np.isfinite(efforts), efforts, np.inf))
            best_z, best_on = np.unravel_index(best_idx, efforts.shape)
            ax.scatter(best_on, best_z, marker="*", s=350, color="gold",
                       edgecolors="black", linewidths=0.9, zorder=6)

        fig.tight_layout()

        path = out_dir / "eta_compare_effort_heatmap.png"
        fig.savefig(str(path), dpi=150, bbox_inches="tight")
        plt.close(fig)
        carb.log_warn(f"[sweep] multi-zeta effort heatmap → {path}")
    except Exception as exc:
        carb.log_warn(f"[sweep] multi-zeta effort heatmap failed: {exc}")


def save_multi_zeta_effort_csv(on_vals, zeta_vals, zeta_to_efforts, out_dir: Path):
    import csv

    path = out_dir / "eta_compare_effort.csv"
    with open(str(path), "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["omega_n"] + [f"zeta_{z:.3f}" for z in zeta_vals])
        for i, on in enumerate(on_vals):
            row = [f"{on:.2f}"]
            for zeta in zeta_vals:
                effort = float(zeta_to_efforts[zeta][i])
                row.append(f"{effort:.6f}" if np.isfinite(effort) else "inf")
            writer.writerow(row)
    carb.log_warn(f"[sweep] multi-zeta effort CSV → {path}")


# ─────────────────────────── Helpers ─────────────────────────────────────────

def make_thrust_curve(mm: float) -> QuadraticThrustCurve:
    return QuadraticThrustCurve(config={"motor_moment": mm})


def results_dir(base: Path, mm: float) -> Path:
    d = base / "eigen_sweep" / f"mm_{mm:.1e}"
    d.mkdir(parents=True, exist_ok=True)
    return d


def zeta_results_dir(base: Path, mm: float, zeta: float, use_subdir: bool) -> Path:
    d = base / "eigen_sweep" / "wn_eta_sweep" / f"mm_{mm:.1e}" / f"zeta_{zeta:.3f}"
    d.mkdir(parents=True, exist_ok=True)
    return d


def ab_sweep_dir(base: Path, mm: float) -> Path:
    """Root output dir for an (a, b) direct-sweep run."""
    d = base / "eigen_sweep" / "ab_sweep" / f"mm_{mm:.1e}"
    d.mkdir(parents=True, exist_ok=True)
    return d


def ab_b_dir(base: Path, mm: float, b: float) -> Path:
    """Per-b sub-directory that holds one npz file per a value."""
    d = ab_sweep_dir(base, mm) / f"b_{b:.3f}"
    d.mkdir(parents=True, exist_ok=True)
    return d


# ─────────────────────── ab-sweep I/O helpers ────────────────────────────────

def save_ab_csv(a_vals, b_vals, scores_2d, out_dir: Path):
    """
    Write scores to a CSV in the *old* wn×b format so eigen_plot_heatmap.py
    can re-plot it without modification.

    Format:
        wn \\ b, b0, b1, b2, ...
        a0, score, score, ...
        a1, score, score, ...
    """
    import csv
    path = out_dir / "scores.csv"
    with open(str(path), "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["wn \\ b"] + [f"{b:.3f}" for b in b_vals])
        for i, a in enumerate(a_vals):
            row = [f"{a:.3f}"]
            for j in range(len(b_vals)):
                s = scores_2d[i, j]
                row.append(f"{s:.6f}" if np.isfinite(s) else "inf")
            writer.writerow(row)
    carb.log_warn(f"[ab-sweep] scores CSV → {path}")


def save_ab_effort_csv(a_vals, b_vals, efforts_2d, out_dir: Path):
    """Write control-effort scores to CSV (same layout as scores.csv)."""
    import csv
    path = out_dir / "effort.csv"
    with open(str(path), "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["wn \\ b"] + [f"{b:.3f}" for b in b_vals])
        for i, a in enumerate(a_vals):
            row = [f"{a:.3f}"]
            for j in range(len(b_vals)):
                e = efforts_2d[i, j]
                row.append(f"{e:.6f}" if np.isfinite(e) else "inf")
            writer.writerow(row)
    carb.log_warn(f"[ab-sweep] effort CSV → {out_dir / 'effort.csv'}")


def save_ab_heatmap(a_vals, b_vals, scores_2d, mm, out_dir: Path, label="score"):
    """2-D heatmap with a (wn_mdyn diagonal) on Y and b (off-diagonal) on X."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import matplotlib.colors as mc

        n_a, n_b = len(a_vals), len(b_vals)
        display  = scores_2d.copy().astype(float)
        finite   = np.isfinite(display)
        fin_vals = display[finite]

        if fin_vals.size:
            vmin = max(float(np.min(fin_vals)), 1e-5)
            vmax = float(np.max(fin_vals))
            if vmax <= vmin:
                vmax = vmin * 1.01
            sentinel = vmax * 1.5
        else:
            vmin, vmax, sentinel = 1e-5, 1.0, 2.0

        display[~finite] = sentinel

        is_effort = label == "effort"
        cmap = (plt.cm.viridis if is_effort else plt.cm.RdYlGn_r).copy()
        cmap.set_over("dimgray")
        norm = mc.LogNorm(vmin=vmin, vmax=vmax)

        fig, ax = plt.subplots(
            figsize=(max(8, n_b * 1.3 + 3), max(5, n_a * 0.9 + 2))
        )
        im = ax.imshow(display, aspect="auto", origin="lower", cmap=cmap, norm=norm)
        cb_label = (
            "Control effort  [future-biased mean Σ motor_thrust²]"
            if is_effort else
            "Stability score  [future-biased mean ‖ep‖  m]"
        )
        plt.colorbar(im, ax=ax, extend="max", label=cb_label)

        ax.set_xticks(range(n_b))
        ax.set_xticklabels([f"b={v:.2f}" for v in b_vals], fontsize=9)
        ax.set_yticks(range(n_a))
        ax.set_yticklabels([f"a={v:.2f}" for v in a_vals], fontsize=9)
        ax.set_xlabel("b  (M_dyn off-diagonal)", fontsize=11)
        ax.set_ylabel("a  (M_dyn diagonal  wn_mdyn)", fontsize=11)
        ax.set_title(
            f"M_dyn (a, b) Direct Sweep   |   km = {mm:.2e}\n"
            f"Kp_att = {ARGS.kp_att}   Kd_att = {ARGS.kd_att}   "
            f"|   lower = {'less effort' if is_effort else 'more stable'}",
            fontsize=11,
        )

        for i, a in enumerate(a_vals):
            for j, b in enumerate(b_vals):
                val = scores_2d[i, j]
                txt = "∞" if not np.isfinite(val) else f"{val:.3f}"
                col = "white" if not np.isfinite(val) else "black"
                ax.text(j, i, f"{txt}\na={a:.2f}\nb={b:.2f}",
                        ha="center", va="center",
                        fontsize=7, color=col, fontweight="bold", linespacing=1.25)

        if finite.any():
            best_idx = np.argmin(np.where(finite, scores_2d, np.inf))
            bi, bj   = np.unravel_index(best_idx, scores_2d.shape)
            ax.scatter(bj, bi, marker="*", s=350, color="gold",
                       edgecolors="black", linewidths=0.9, zorder=6)

        fig.tight_layout()
        fname = "effort_heatmap.png" if is_effort else "scores_heatmap.png"
        path  = out_dir / fname
        fig.savefig(str(path), dpi=150, bbox_inches="tight")
        plt.close(fig)
        carb.log_warn(f"[ab-sweep] {label} heatmap → {path}")
    except Exception as exc:
        carb.log_warn(f"[ab-sweep] heatmap failed ({label}): {exc}")


# ─────────────────────────── ab-sweep runner ─────────────────────────────────

def run_ab_sweep(
    world,
    timeline,
    a_vals: list,
    b_vals: list,
    mm_vals: list,
    results_base: Path,
):
    """
    Direct M_dyn (a, b) sweep.

    Parallelism layout
    ──────────────────
    • One drone per *a* value  →  all simulated simultaneously.
    • *b* is the inner serial loop  →  one simulation pass per b.
    • motor_moment is the outer serial loop.

    Directory structure
    ───────────────────
    results/eigen_sweep/ab_sweep/mm_<km>/
        scores.csv              ← 2-D table (wn\\b format for plot_heatmap.py)
        effort.csv
        scores_heatmap.png
        effort_heatmap.png
        b_<b>/
            a<a>_statistics.npz   ← raw telemetry per drone
    """
    render_sim  = not ARGS.headless
    bounds      = ARGS.bounds
    n_a         = len(a_vals)

    carb.log_warn("\n" + "="*60)
    carb.log_warn("[ab-sweep] Mode: direct M_dyn (a, b) sweep")
    carb.log_warn(f"[ab-sweep] a values (wn_mdyn diagonal): {a_vals}")
    carb.log_warn(f"[ab-sweep] b values (b_mdyn off-diag) : {b_vals}")
    carb.log_warn(f"[ab-sweep] motor_moments              : {mm_vals}")
    carb.log_warn(f"[ab-sweep] Kp_att = {ARGS.kp_att}   Kd_att = {ARGS.kd_att}")
    carb.log_warn(f"[ab-sweep] duration = {ARGS.duration} s per pass")
    carb.log_warn("="*60)

    # ── Drone spacing along X ────────────────────────────────────────────────
    x_min, x_max = bounds[0], bounds[1]
    x_margin     = 1.5
    x_usable     = x_max - x_min - 2.0 * x_margin
    auto_spacing = x_usable / (n_a - 1) if n_a > 1 else 0.0
    spacing      = min(ARGS.spacing, auto_spacing)
    x_start      = x_min + x_margin
    y_min, y_max = bounds[2], bounds[3]
    z_min, z_max = bounds[4], bounds[5]

    for i_a in range(n_a):
        sx = x_start + i_a * spacing
        if not (x_min < sx < x_max):
            raise ValueError(
                f"[ab-sweep] Drone {i_a} spawn X={sx:.2f} outside X bounds "
                f"[{x_min},{x_max}]. Reduce --spacing or widen --bounds."
            )

    spawn_y = -1.5
    spawn_z = 0.07
    half    = spacing / 2.0
    rng     = np.random.default_rng(ARGS.goal_seed)

    # Temp npz paths (one per drone slot; overwritten each pass)
    tmp_files = [
        str(results_base / f"_ab_sweep_tmp_{i}.npz") for i in range(n_a)
    ]

    # ── Spawn drones (one per a); b and mm are updated each pass ────────────
    drones: list[tuple] = []
    for i_a, a in enumerate(a_vals):
        spawn_x   = x_start + i_a * spacing
        spawn_pos = [spawn_x, spawn_y, spawn_z]
        target    = build_target_position(spawn_pos, rng)
        drone_bounds = [
            spawn_x - half, spawn_x + half,
            y_min, y_max,
            z_min, z_max,
        ]

        inner = EigenController(
            target_position=target,
            results_file=tmp_files[i_a],
            wn_mdyn=a,
            b_mdyn=b_vals[0],
            Kp_att=ARGS.kp_att,
            Kd_att=ARGS.kd_att,
            km=mm_vals[0],
        )
        backend = BoundedTelemetryBackend(inner, tmp_files[i_a])

        cfg              = MultirotorConfig()
        cfg.backends     = [backend]
        cfg.thrust_curve = make_thrust_curve(mm_vals[0])

        vehicle = Multirotor(
            f"/World/ab_a{i_a}",
            ROBOTS["Iris"],
            i_a,
            spawn_pos,
            Rotation.from_euler("XYZ", [0., 0., 0.], degrees=True).as_quat(),
            config=cfg,
        )
        drones.append((vehicle, backend, spawn_pos, drone_bounds))

    world.reset()
    for _ in range(10):
        world.step(render=render_sim)

    # ── Outer loop: motor_moment ─────────────────────────────────────────────
    for mm in mm_vals:
        root_dir = ab_sweep_dir(results_base, mm)
        # 2-D accumulators: rows = a, cols = b
        all_scores  = np.full((n_a, len(b_vals)), np.inf)
        all_efforts = np.full((n_a, len(b_vals)), np.inf)

        for j_b, b in enumerate(b_vals):
            carb.log_warn(f"\n{'='*60}")
            carb.log_warn(
                f"[ab-sweep] motor_moment = {mm:.2e}  |  b = {b:.3f}  "
                f"({j_b+1}/{len(b_vals)})"
            )
            carb.log_warn(f"{'='*60}")

            b_dir = ab_b_dir(results_base, mm, b)

            # Update every drone for this (mm, b) pass
            for i_a, (vehicle, backend, spawn_pos, drone_bounds) in enumerate(drones):
                a = a_vals[i_a]

                vehicle._thrusters        = make_thrust_curve(mm)
                backend.inner.wn_mdyn     = a
                backend.inner.b_mdyn      = b
                backend.inner.Kp_att      = ARGS.kp_att
                backend.inner.Kd_att      = ARGS.kd_att
                backend.inner.km          = mm

                vehicle.set_world_pose(
                    position=spawn_pos,
                    orientation=[0., 0., 0., 1.],
                )
                vehicle.set_linear_velocity([0., 0., 0.])
                vehicle.set_angular_velocity([0., 0., 0.])

                backend.reset()
                backend.start()
                carb.log_warn(
                    f"  [a={a:.2f}  b={b:.2f}]  drone {i_a} ready"
                )

            # Flush USD → physics
            for _ in range(5):
                world.step(render=render_sim)

            # ── Simulate ──────────────────────────────────────────────────
            timeline.play()
            sim_time = 0.0
            dt       = 1.0 / 60.0

            while simulation_app.is_running() and sim_time < ARGS.duration:
                world.step(render=render_sim)
                sim_time += dt

                all_oob = True
                for _, backend, _, drone_bounds in drones:
                    if backend.is_oob:
                        continue
                    all_oob = False
                    backend.check_oob(drone_bounds)

                if all_oob:
                    carb.log_warn("  [ab-sweep] all drones OOB — ending pass early")
                    break

            timeline.stop()

            # ── Stop, move npz, score ─────────────────────────────────────
            for i_a, (_, backend, _, _) in enumerate(drones):
                if not backend._stopped:
                    backend.stop()

                a   = a_vals[i_a]
                src = tmp_files[i_a]
                dst = b_dir / f"a{a:.3f}_statistics.npz"

                score  = stability_score(src,          T_total=ARGS.duration)
                effort = control_effort_score(src,      T_total=ARGS.duration)
                all_scores [i_a, j_b] = score
                all_efforts[i_a, j_b] = effort

                carb.log_warn(
                    f"  [a={a:.2f}  b={b:.2f}]  "
                    f"score={'∞' if not np.isfinite(score)  else f'{score:.4f}'}  "
                    f"effort={'∞' if not np.isfinite(effort) else f'{effort:.4f}'}"
                )

                if os.path.isfile(src):
                    try:
                        os.rename(src, str(dst))
                    except Exception as exc:
                        carb.log_warn(f"  move failed: {exc}")

        # ── Per-mm outputs ───────────────────────────────────────────────────
        save_ab_csv(a_vals, b_vals, all_scores,  root_dir)
        save_ab_effort_csv(a_vals, b_vals, all_efforts, root_dir)
        save_ab_heatmap(a_vals, b_vals, all_scores,  mm, root_dir, label="score")
        save_ab_heatmap(a_vals, b_vals, all_efforts, mm, root_dir, label="effort")

        # Summary
        fin_mask = np.isfinite(all_scores)
        if fin_mask.any():
            best_idx   = np.argmin(np.where(fin_mask, all_scores, np.inf))
            bi, bj     = np.unravel_index(best_idx, all_scores.shape)
            carb.log_warn(
                f"\n  ★ Best for mm={mm:.1e}: "
                f"a={a_vals[bi]:.3f}  b={b_vals[bj]:.3f}  "
                f"→ score={all_scores[bi,bj]:.4f}"
            )
        else:
            carb.log_warn(f"\n  ✗ All drones OOB for mm={mm:.1e}")

    # Cleanup temp files
    for f in tmp_files:
        try:
            os.remove(f)
        except FileNotFoundError:
            pass


# ─────────────────────────── Main ────────────────────────────────────────────

def main():
    timeline     = omni.timeline.get_timeline_interface()
    render_sim   = not ARGS.headless
    bounds       = ARGS.bounds
    mm_vals      = ARGS.motor_moments
    base_dir     = Path(__file__).resolve().parent.parent
    results_base = base_dir / "results"

    # ── Dispatch: direct (a, b) sweep if --a-values is provided ─────────────
    if ARGS.a_values is not None:
        if not ARGS.b_values:
            raise ValueError(
                "--b-values must be supplied together with --a-values for ab-sweep mode."
            )
        pg       = PegasusInterface()
        pg._world = World(**pg._world_settings)
        world    = pg.world
        pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])
        for _ in range(20):
            world.step(render=render_sim)

        run_ab_sweep(
            world=world,
            timeline=timeline,
            a_vals=ARGS.a_values,
            b_vals=ARGS.b_values,
            mm_vals=mm_vals,
            results_base=results_base,
        )

        carb.log_warn("\n=== ab-sweep complete ===")
        try:
            timeline.stop()
        except Exception:
            pass
        if world is not None:
            try:
                world.stop()
            except Exception:
                pass
            for fn in ("stop", "clear_all_callbacks", "clear"):
                try:
                    getattr(world, fn)()
                except Exception:
                    pass
        simulation_app.close()
        return

    # ── Original ωn / ζ sweep (unchanged below) ──────────────────────────────
    on_vals      = ARGS.omega_n_values
    zeta_vals    = ARGS.zeta_values if ARGS.zeta_values else [ARGS.zeta]
    n_on         = len(on_vals)

    carb.log_warn(f"[sweep] ωn  values : {on_vals}")
    carb.log_warn(f"[sweep] ζ values    : {zeta_vals}")
    carb.log_warn(f"[sweep] km  values : {mm_vals}")
    carb.log_warn(f"[sweep] Kp_att     : {ARGS.kp_att}  (baseline, fixed)")
    carb.log_warn(f"[sweep] Kd_att     : {ARGS.kd_att}  (baseline, fixed)")
    carb.log_warn(f"[sweep] duration   : {ARGS.duration}s per mm pass")

    for zeta in zeta_vals:
        carb.log_warn(f"[sweep] eigen map for ζ={zeta:.4f}")
        for on in on_vals:
            wn_m, b_m = mdyn_params_for(on, zeta)
            carb.log_warn(f"  ωn={on:.1f}  →  wn_mdyn={wn_m:.3f}  b_mdyn={b_m:.3f}"
                          f"  (λ = -{wn_m:.3f} ± j{b_m:.3f})")

    # ── World setup ─────────────────────────────────────────────────────────

    pg       = PegasusInterface()
    pg._world = World(**pg._world_settings)
    world    = pg.world

    pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])

    for _ in range(20):
        world.step(render=render_sim)

    # ── Compute X spacing (drones spread along X, one per ωn value) ─────────

    x_min, x_max = bounds[0], bounds[1]
    x_margin  = 1.5
    x_usable  = x_max - x_min - 2.0 * x_margin
    auto_spacing = x_usable / (n_on - 1) if n_on > 1 else 0.0
    spacing   = min(ARGS.spacing, auto_spacing)
    x_start   = x_min + x_margin

    for i_on in range(n_on):
        sx = x_start + i_on * spacing
        if not (x_min < sx < x_max):
            raise ValueError(
                f"Drone {i_on} spawn X={sx:.2f} outside X bounds [{x_min},{x_max}]. "
                f"Reduce --spacing or widen --bounds."
            )

    spawn_y  = -1.5
    spawn_z  = 0.07
    half     = spacing / 2.0
    y_min, y_max = bounds[2], bounds[3]
    z_min, z_max = bounds[4], bounds[5]

    rng = np.random.default_rng(ARGS.goal_seed)
    carb.log_warn(f"[sweep] goal mode  : {ARGS.goal_mode}  "
                  f"(alt={ARGS.target_altitude}m, radius={ARGS.goal_radius}m, "
                  f"seed={ARGS.goal_seed})")
    carb.log_warn(f"[sweep] X spacing  : {spacing:.2f} m  (start x={x_start:.2f})")

    # Temp npz path per drone slot (overwritten each mm pass, then moved)
    tmp_files = [str(results_base / f"_eigen_sweep_tmp_{i}.npz") for i in range(n_on)]

    # ── Spawn drones (one per ωn); parameters updated each mm pass ──────────

    drones: list[tuple[Multirotor, BoundedTelemetryBackend, list, list]] = []

    for i_on, on in enumerate(on_vals):
        wn_m, b_m = mdyn_params_for(on, zeta_vals[0])
        spawn_x   = x_start + i_on * spacing
        spawn_pos = [spawn_x, spawn_y, spawn_z]
        target    = build_target_position(spawn_pos, rng)

        drone_bounds = [
            spawn_x - half, spawn_x + half,
            y_min, y_max,
            z_min, z_max,
        ]

        inner = EigenController(
            target_position=target,
            results_file=tmp_files[i_on],
            wn_mdyn=wn_m,
            b_mdyn=b_m,
            Kp_att=ARGS.kp_att,
            Kd_att=ARGS.kd_att,
            km=mm_vals[0],
        )
        backend = BoundedTelemetryBackend(inner, tmp_files[i_on])

        cfg              = MultirotorConfig()
        cfg.backends     = [backend]
        cfg.thrust_curve = make_thrust_curve(mm_vals[0])

        vehicle = Multirotor(
            f"/World/eigen_on{i_on}",
            ROBOTS["Iris"],
            i_on,
            spawn_pos,
            Rotation.from_euler("XYZ", [0., 0., 0.], degrees=True).as_quat(),
            config=cfg,
        )
        drones.append((vehicle, backend, spawn_pos, drone_bounds))

    world.reset()
    for _ in range(10):
        world.step(render=render_sim)

    # ── Outer loop: motor_moment ─────────────────────────────────────────────

    for mm in mm_vals:
        zeta_to_scores = {}
        zeta_to_efforts = {}
        for zeta in zeta_vals:
            carb.log_warn(f"\n{'='*60}")
            carb.log_warn(f"[sweep] motor_moment = {mm:.2e}  |  zeta = {zeta:.3f}")
            carb.log_warn(f"{'='*60}")

            rdir   = zeta_results_dir(results_base, mm, zeta, use_subdir=(len(zeta_vals) > 1))
            scores = np.full(n_on, np.inf)
            efforts = np.full(n_on, np.inf)

            # Update every drone for this (mm, zeta) pass ───────────────
            for i_on, (vehicle, backend, spawn_pos, drone_bounds) in enumerate(drones):
                on      = on_vals[i_on]
                wn_m, b_m = mdyn_params_for(on, zeta)

                # Swap thrust model
                vehicle._thrusters = make_thrust_curve(mm)

                # Update M_dyn params (Kp/Kd stay at baseline)
                backend.inner.wn_mdyn = wn_m
                backend.inner.b_mdyn  = b_m
                backend.inner.Kp_att  = ARGS.kp_att
                backend.inner.Kd_att  = ARGS.kd_att
                backend.inner.km      = mm

                # Reset kinematics
                vehicle.set_world_pose(
                    position=spawn_pos,
                    orientation=[0., 0., 0., 1.],
                )
                vehicle.set_linear_velocity([0., 0., 0.])
                vehicle.set_angular_velocity([0., 0., 0.])

                backend.reset()
                backend.start()
                carb.log_warn(
                    f"  [ωn={on:.1f}]  wn_mdyn={wn_m:.3f}  b_mdyn={b_m:.3f}  drone {i_on} ready"
                )

            # Flush USD→physics
            for _ in range(5):
                world.step(render=render_sim)

            # ── Simulate ─────────────────────────────────────────────

            timeline.play()
            sim_time = 0.0
            dt       = 1.0 / 60.0

            while simulation_app.is_running() and sim_time < ARGS.duration:
                world.step(render=render_sim)
                sim_time += dt

                all_oob = True
                for _, backend, _, drone_bounds in drones:
                    if backend.is_oob:
                        continue
                    all_oob = False
                    backend.check_oob(drone_bounds)

                if all_oob:
                    carb.log_warn("  [mm,zeta] all drones OOB — ending early")
                    break

            timeline.stop()

            # ── Stop, save, score ─────────────────────────────────────

            for i_on, (_, backend, _, _) in enumerate(drones):
                if not backend._stopped:
                    backend.stop()

                on  = on_vals[i_on]
                src = tmp_files[i_on]
                dst = rdir / f"on{on:.1f}_statistics.npz"

                score  = stability_score(src,          T_total=ARGS.duration)
                effort = control_effort_score(src,      T_total=ARGS.duration)
                scores[i_on] = score
                efforts[i_on] = effort
                carb.log_warn(
                    f"  [ωn={on:.1f}]  wn_mdyn={zeta*on:.3f}  b_mdyn={on*math.sqrt(max(1-zeta**2,0)):.3f}"
                    f"  →  score = {'∞' if not np.isfinite(score) else f'{score:.4f}'}"
                    f"  | effort = {'∞' if not np.isfinite(effort) else f'{effort:.4f}'}"
                )

                if os.path.isfile(src):
                    try:
                        os.rename(src, str(dst))
                    except Exception as exc:
                        carb.log_warn(f"  move failed: {exc}")

            # ── Score outputs ─────────────────────────────────────────

            if len(zeta_vals) == 1:
                save_score_plot(on_vals, scores, zeta, mm, rdir)
            save_csv(on_vals, scores, zeta, rdir)

            # Summary
            fin_idx = [i for i in range(n_on) if np.isfinite(scores[i])]
            if fin_idx:
                best_i = min(fin_idx, key=lambda i: scores[i])
                on_best = on_vals[best_i]
                wn_best, b_best = mdyn_params_for(on_best, zeta)
                carb.log_warn(
                    f"\n  ★ Best for mm={mm:.1e}, zeta={zeta:.3f}: "
                    f"ωn={on_best:.1f}  wn_mdyn={wn_best:.3f}  b_mdyn={b_best:.3f}"
                    f"  →  score={scores[best_i]:.4f}"
                )
            else:
                carb.log_warn(f"\n  ✗ All drones OOB for mm={mm:.1e}, zeta={zeta:.3f}")

            zeta_to_scores[zeta] = scores.copy()
            zeta_to_efforts[zeta] = efforts.copy()

        if len(zeta_vals) > 1:
            mm_root = results_dir(results_base, mm)
            save_multi_zeta_heatmap(on_vals, zeta_vals, zeta_to_scores, mm, mm_root)
            save_multi_zeta_csv(on_vals, zeta_vals, zeta_to_scores, mm_root)
            save_multi_zeta_effort_heatmap(on_vals, zeta_vals, zeta_to_efforts, mm, mm_root)
            save_multi_zeta_effort_csv(on_vals, zeta_vals, zeta_to_efforts, mm_root)

    # ── Teardown ────────────────────────────────────────────────────────────

    carb.log_warn("\n=== eigen M_dyn eigenvalue sweep complete ===")

    for f in tmp_files:
        try:
            os.remove(f)
        except FileNotFoundError:
            pass

    try:
        timeline.stop()
    except Exception:
        pass

    if world is not None:
        try:
            world.stop()
        except Exception:
            pass
        for fn in ("stop", "clear_all_callbacks", "clear"):
            try:
                getattr(world, fn)()
            except Exception:
                pass

    simulation_app.close()


if __name__ == "__main__":
    main()
