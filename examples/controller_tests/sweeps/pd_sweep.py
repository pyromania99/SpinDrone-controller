#!/usr/bin/env python3
"""
pd_sweep.py  —  Cascaded PD 2D gain sweep for Pareto analysis.

A 2D sweep over combinations of Kp_xy and Kd_xy:
    Kp_xy  → from --kp-xy-values
    Kd_xy  → from --kd-xy-values

Attitude gains (Kp_att, Kd_att) are kept fixed — they are the inner-loop
and should not be swept here (would conflate two tuning axes).

Parallelism layout
──────────────────
• One drone per Kp_xy value  →  all simulated simultaneously in one pass.
• Outer loop over Kd_xy values → successive simulation passes.

Outputs  (in results/pd_sweep/)
────────
  scores.csv           future-biased mean ‖ep‖ per combination
  effort.csv           future-biased mean Σ motor_thrust² per combination
  kd_<kd>/...          raw telemetry per drone and kd step

Usage
─────
  isaac_run examples/controller_tests/sweeps/pd_sweep.py \
      --kp-xy-values 0.3 0.5 0.7 1.0 1.5 \
      --kd-xy-values 0.5 1.0 2.0 3.0 \
      --duration 20 --headless
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys
from pathlib import Path

import carb
from isaacsim import SimulationApp

# ── CLI (before SimulationApp) ────────────────────────────────────────────────

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Cascaded PD 2D sweep"
    )
    p.add_argument(
        "--kp-xy-values", nargs="+", type=float,
        default=[15.0, 20.0, 30.0],
        help="Proportional XY gains to sweep (parallel drones)",
    )
    p.add_argument(
        "--kd-xy-values", nargs="+", type=float,
        default=[1.0, 1.5, 2.0],
        help="Derivative XY gains to sweep (sequential passes)",
    )
    # ── Baseline position gains ──────────────────────────
    p.add_argument("--kp-z",   type=float, default=15.0)
    p.add_argument("--kd-z",   type=float, default=8.0)
    # ── Fixed attitude gains ──────────────────────────────────────────────────
    p.add_argument("--kp-att", type=float, default=0.65,
                   help="Fixed attitude proportional gain (not swept)")
    p.add_argument("--kd-att", type=float, default=10.0,
                   help="Fixed attitude derivative gain (not swept)")
    # ── Simulation ────────────────────────────────────────────────────────────
    p.add_argument("--duration",        type=float, default=20.0)
    p.add_argument("--target-altitude", type=float, default=1.5)
    p.add_argument("--spacing",         type=float, default=4.0)
    p.add_argument(
        "--bounds", nargs=6, type=float,
        default=[-12., 12., -12., 12., 0., 20.],
        metavar=("XMIN", "XMAX", "YMIN", "YMAX", "ZMIN", "ZMAX"),
    )
    p.add_argument("--goal-seed",  type=int,  default=7)
    p.add_argument("--motor-moment", type=float, default=0,
                   help="Motor moment constant km. If provided, overrides default.")
    p.add_argument("--headless",   action="store_true", default=False)
    p.add_argument("--no-headless", dest="headless", action="store_false")
    return p.parse_args()


ARGS = parse_args()
simulation_app = SimulationApp({
    "headless": ARGS.headless,
    "hide_ui":                   True if ARGS.headless else None,
    "disable_viewport_updates":  True if ARGS.headless else False,
    "multi_gpu":    False,
    "max_gpu_count": 1,
    "active_gpu":    0,
})

# ── Late imports ──────────────────────────────────────────────────────────────

import numpy as np
# pyrefly: ignore [missing-import]
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
from pegasus.simulator.logic.thrusters import QuadraticThrustCurve
from pegasus.simulator.logic.backends import Backend
from pegasus.simulator.logic.thrusters import QuadraticThrustCurve
from pegasus.simulator.logic.state import State

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "utils"))
from cascaded_pd_controller import CascadedPDController


# ── OOB-aware telemetry wrapper ───────────────────────────────────────────────

class BoundedPDBackend(Backend):
    """Wraps CascadedPDController; tracks OOB and gates all calls."""

    def __init__(self, inner: CascadedPDController, results_file: str):
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
        if os.path.isfile(self.results_file):
            try:
                data = dict(np.load(self.results_file))
                data["oob_time"] = np.array(self.oob_time)
                np.savez(self.results_file, **data)
            except Exception as exc:
                carb.log_warn(f"[pd_sweep] oob_time append failed: {exc}")

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
        return [0., 0., 0., 0.] if self.is_oob else self.inner.input_reference()

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
            carb.log_warn(f"[pd_sweep] OOB at {self.position} t={self.total_time:.2f}s")
            return True
        return False


# ── Score helpers (copied from eigen_stability_sweep) ─────────────────────────

_BOUNDS_DIAG = float(np.linalg.norm([
    ARGS.bounds[1] - ARGS.bounds[0],
    ARGS.bounds[3] - ARGS.bounds[2],
    ARGS.bounds[5] - ARGS.bounds[4],
]))


def stability_score(npz_path: str, T_total: float) -> float:
    try:
        d   = np.load(npz_path)
        ep  = d["ep"]
        t   = d["time"]
        if len(t) < 2:
            return np.inf
        oob_time       = float(d["oob_time"]) if "oob_time" in d else float("nan")
        drone_went_oob = np.isfinite(oob_time)
        if T_total < 1e-6:
            T_total = float(t[-1])
        errors  = np.linalg.norm(ep, axis=1)
        if not np.all(np.isfinite(errors)):
            return np.inf
        t_arr, err_arr = t.copy(), errors.copy()
        if drone_went_oob and oob_time < T_total - 1e-6:
            dt_a  = float(t[1] - t[0]) if len(t) > 1 else 1/60
            pad_t = np.arange(oob_time + dt_a, T_total, dt_a)
            if len(pad_t) > 0:
                t_arr   = np.concatenate([t_arr,   pad_t])
                err_arr = np.concatenate([err_arr, np.full(len(pad_t), _BOUNDS_DIAG)])
        w     = t_arr / T_total
        w_sum = float(np.sum(w))
        if w_sum < 1e-12:
            return np.inf
        s = float(np.dot(w, err_arr) / w_sum)
        return s if np.isfinite(s) else np.inf
    except Exception as exc:
        carb.log_warn(f"[pd_sweep] score read failed: {exc}")
        return np.inf


def effort_score(npz_path: str, T_total: float) -> float:
    try:
        d  = np.load(npz_path)
        t  = d["time"]
        if len(t) < 2:
            return np.inf
        oob_time       = float(d["oob_time"]) if "oob_time" in d else float("nan")
        drone_went_oob = np.isfinite(oob_time)
        if T_total < 1e-6:
            T_total = float(t[-1])
        mt = d.get("motor_thrusts")
        if mt is None or mt.ndim != 2 or mt.shape[0] != len(t):
            return np.inf
        eff = np.sum(np.square(mt.astype(float)), axis=1)
        if not np.all(np.isfinite(eff)):
            return np.inf
        t_arr, eff_arr = t.copy(), eff.copy()
        if drone_went_oob and oob_time < T_total - 1e-6:
            dt_a  = float(t[1] - t[0]) if len(t) > 1 else 1/60
            pad_t = np.arange(oob_time + dt_a, T_total, dt_a)
            if len(pad_t) > 0:
                worst = float(np.max(eff_arr)) if eff_arr.size else 0.0
                t_arr   = np.concatenate([t_arr,  pad_t])
                eff_arr = np.concatenate([eff_arr, np.full(len(pad_t), worst)])
        w     = t_arr / T_total
        w_sum = float(np.sum(w))
        if w_sum < 1e-12:
            return np.inf
        s = float(np.dot(w, eff_arr) / w_sum)
        return s if np.isfinite(s) else np.inf
    except Exception as exc:
        carb.log_warn(f"[pd_sweep] effort read failed: {exc}")
        return np.inf


# ── CSV helpers ────────────────────────────────────────────────────────

def save_kpkd_csv(kp_vals, kd_vals, data_2d, out_dir: Path, filename: str) -> None:
    path = out_dir / filename
    with open(str(path), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["kp \\ kd"] + [f"{kd:.4f}" for kd in kd_vals])
        for i, kp in enumerate(kp_vals):
            row = [f"{kp:.4f}"]
            for j in range(len(kd_vals)):
                v = data_2d[i, j]
                row.append(f"{v:.6f}" if np.isfinite(v) else "inf")
            w.writerow(row)
    carb.log_warn(f"[pd_sweep] CSV → {path}")

def save_heatmap(x_vals, y_vals, data_2d, out_dir: Path, filename: str, title: str, xlabel: str, ylabel: str, clabel: str, cmap_name="RdYlGn_r"):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import matplotlib.colors as mc

        n_x, n_y = len(x_vals), len(y_vals)
        display = data_2d.copy()
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
        fig, ax = plt.subplots(figsize=(max(8, n_x * 1.3 + 3), max(4.8, n_y * 0.9 + 2)))

        cmap = getattr(plt.cm, cmap_name).copy()
        cmap.set_over("dimgray")
        norm = mc.LogNorm(vmin=vmin, vmax=vmax)

        im = ax.imshow(display, aspect="auto", origin="lower", cmap=cmap, norm=norm)
        plt.colorbar(im, ax=ax, extend="max", label=clabel)

        ax.set_xticks(range(n_x))
        ax.set_xticklabels([f"{v:.4f}" for v in x_vals], fontsize=9)
        ax.set_yticks(range(n_y))
        ax.set_yticklabels([f"{v:.4f}" for v in y_vals], fontsize=9)
        ax.set_xlabel(xlabel, fontsize=11)
        ax.set_ylabel(ylabel, fontsize=11)
        ax.set_title(title, fontsize=11)

        for i_y, y in enumerate(y_vals):
            for i_x, x in enumerate(x_vals):
                val = data_2d[i_y, i_x]
                txt = "∞" if not np.isfinite(val) else f"{val:.3f}"
                text_color = "white" if not np.isfinite(val) else "black"
                ax.text(i_x, i_y, txt, ha="center", va="center",
                        fontsize=8, color=text_color, fontweight="bold")

        if finite.any():
            best_idx = np.argmin(np.where(np.isfinite(data_2d), data_2d, np.inf))
            best_y, best_x = np.unravel_index(best_idx, data_2d.shape)
            ax.scatter(best_x, best_y, marker="*", s=350, color="gold",
                       edgecolors="black", linewidths=0.9, zorder=6)

        fig.tight_layout()
        path = out_dir / filename
        fig.savefig(str(path), dpi=150, bbox_inches="tight")
        plt.close(fig)
        carb.log_warn(f"[sweep] heatmap → {path}")
    except Exception as exc:
        carb.log_warn(f"[sweep] heatmap failed: {exc}")


# ── Main simulation sweep ─────────────────────────────────────────────────────

def main():
    timeline    = omni.timeline.get_timeline_interface()
    render_sim  = not ARGS.headless
    bounds      = ARGS.bounds
    kp_vals     = ARGS.kp_xy_values
    kd_vals     = ARGS.kd_xy_values
    n_kp        = len(kp_vals)
    n_kd        = len(kd_vals)

    base_dir     = Path(__file__).resolve().parent.parent
    results_base = base_dir / "results" / "pd_sweep"
    if ARGS.motor_moment is not None:
        results_base = results_base / f"mm_{ARGS.motor_moment:.1e}"
    results_base.mkdir(parents=True, exist_ok=True)

    carb.log_warn("\n" + "="*60)
    carb.log_warn("[pd_sweep] Cascaded PD 2D Sweep")
    carb.log_warn(f"[pd_sweep] kp_xy values : {kp_vals}")
    carb.log_warn(f"[pd_sweep] kd_xy values : {kd_vals}")
    carb.log_warn(f"[pd_sweep] Kp_z base    : {ARGS.kp_z}   Kd_z base  : {ARGS.kd_z}")
    carb.log_warn(f"[pd_sweep] Kp_att fixed : {ARGS.kp_att}  Kd_att fixed: {ARGS.kd_att}")
    carb.log_warn(f"[pd_sweep] motor_moment : {ARGS.motor_moment}")
    carb.log_warn(f"[pd_sweep] duration     : {ARGS.duration} s")
    carb.log_warn("="*60)

    # ── World ────────────────────────────────────────────────────────────────
    pg        = PegasusInterface()
    pg._world = World(**pg._world_settings)
    world     = pg.world
    pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])
    for _ in range(20):
        world.step(render=render_sim)

    # ── X spacing ────────────────────────────────────────────────────────────
    x_min, x_max = bounds[0], bounds[1]
    x_margin  = 1.5
    x_usable  = x_max - x_min - 2.0 * x_margin
    auto_spacing = x_usable / (n_kp - 1) if n_kp > 1 else 0.0
    spacing   = min(ARGS.spacing, auto_spacing)
    x_start   = x_min + x_margin

    for i, kp in enumerate(kp_vals):
        sx = x_start + i * spacing
        if not (x_min < sx < x_max):
            raise ValueError(
                f"[pd_sweep] Drone {i} spawn X={sx:.2f} outside X bounds "
                f"[{x_min},{x_max}]. Reduce --spacing or widen --bounds."
            )

    spawn_y = -1.5
    spawn_z = 0.07
    half    = spacing / 2.0
    y_min, y_max = bounds[2], bounds[3]
    z_min, z_max = bounds[4], bounds[5]

    tmp_files = [str(results_base / f"_pd_sweep_tmp_{i}.npz")
                 for i in range(n_kp)]
    drones = []

    # ── Spawn one drone per kp ────────────────────────────────────────────────
    for i, kp in enumerate(kp_vals):
        spawn_x   = x_start + i * spacing
        spawn_pos = [spawn_x, spawn_y, spawn_z]
        target    = (spawn_x, spawn_y, float(ARGS.target_altitude))

        drone_bounds = [
            spawn_x - half, spawn_x + half,
            y_min, y_max,
            z_min, z_max,
        ]

        inner = CascadedPDController(
            target_position=target,
            results_file=tmp_files[i],
            Kp_xy=(kp, kp),
            Kd_xy=(kd_vals[0], kd_vals[0]),
            Kp_z=ARGS.kp_z,
            Kd_z=ARGS.kd_z,
            Kp_att=(ARGS.kp_att, ARGS.kp_att),
            Kd_att=(ARGS.kd_att, ARGS.kd_att),
        )
        backend = BoundedPDBackend(inner, tmp_files[i])

        cfg          = MultirotorConfig()
        cfg.backends = [backend]
        if ARGS.motor_moment is not None:
            cfg.thrust_curve = QuadraticThrustCurve(config={"motor_moment": ARGS.motor_moment})

        vehicle = Multirotor(
            f"/World/pd_kp{i}",
            ROBOTS["Iris"],
            i,
            spawn_pos,
            Rotation.from_euler("XYZ", [0., 0., 0.], degrees=True).as_quat(),
            config=cfg,
        )
        drones.append((vehicle, backend, spawn_pos, drone_bounds))
        carb.log_warn(f"  [Kp_xy={kp:.2f}]  drone {i} spawned at x={spawn_x:.1f}")

    world.reset()
    for _ in range(10):
        world.step(render=render_sim)

    all_scores  = np.full((n_kp, n_kd), np.inf)
    all_efforts = np.full((n_kp, n_kd), np.inf)

    # ── Simulate ──────────────────────────────────────────────────────────────
    for j_kd, kd in enumerate(kd_vals):
        carb.log_warn(f"\n[pd_sweep] Kd_xy = {kd:.4f}  ({j_kd+1}/{n_kd})")
        kd_dir = results_base / f"kd_{kd:.4f}"
        kd_dir.mkdir(parents=True, exist_ok=True)

        for i, (vehicle, backend, spawn_pos, _) in enumerate(drones):
            vehicle.set_world_pose(position=spawn_pos, orientation=[0., 0., 0., 1.])
            vehicle.set_linear_velocity([0., 0., 0.])
            vehicle.set_angular_velocity([0., 0., 0.])
            backend.inner.Kd_xy = np.array([kd, kd], dtype=float)
            backend.reset()
            backend.start()

        for _ in range(5):
            world.step(render=render_sim)

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
                carb.log_warn("  [pd_sweep] all drones OOB — ending pass early")
                break

        timeline.stop()

        for i, kp in enumerate(kp_vals):
            _, backend, _, _ = drones[i]
            if not backend._stopped:
                backend.stop()

            src = tmp_files[i]
            dst = kd_dir / f"kp_{kp:.4f}_statistics.npz"

            s = stability_score(src, ARGS.duration)
            e = effort_score(src, ARGS.duration)
            all_scores[i, j_kd] = s
            all_efforts[i, j_kd] = e

            s_str = "∞" if not np.isfinite(s) else f"{s:.4f}"
            e_str = "∞" if not np.isfinite(e) else f"{e:.4f}"
            carb.log_warn(f"  kp={kp:<8.2f} kd={kd:<8.2f} score={s_str:<12} effort={e_str:<12}")

            if os.path.isfile(src):
                try:
                    os.rename(src, str(dst))
                except Exception as exc:
                    carb.log_warn(f"  move failed: {exc}")

    carb.log_warn("="*60)

    # Save CSVs
    save_kpkd_csv(kp_vals, kd_vals, all_scores,  results_base, "scores.csv")
    save_kpkd_csv(kp_vals, kd_vals, all_efforts, results_base, "effort.csv")

    # Heatmaps
    save_heatmap(kd_vals, kp_vals, all_scores, results_base, "score_heatmap.png",
                 "Cascaded PD Stability Score Heatmap",
                 "Derivative Gain Kd_xy", "Proportional Gain Kp_xy", "Stability Score [m]", "RdYlGn_r")
    save_heatmap(kd_vals, kp_vals, all_efforts, results_base, "effort_heatmap.png",
                 "Cascaded PD Control Effort Heatmap",
                 "Derivative Gain Kd_xy", "Proportional Gain Kp_xy", "Control Effort [N²]", "YlOrRd")

    carb.log_warn(f"\n[pd_sweep] results → {results_base}")
    carb.log_warn("=== pd_sweep complete ===")

    # ── Teardown ──────────────────────────────────────────────────────────────
    for f in tmp_files:
        try:
            os.remove(f)
        except FileNotFoundError:
            pass
    try:
        timeline.stop()
    except Exception:
        pass
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
