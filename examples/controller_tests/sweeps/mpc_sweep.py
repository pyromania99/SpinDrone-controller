#!/usr/bin/env python3
"""
mpc_sweep.py  —  LPV-MPC q/r weight 2D sweep for Pareto analysis.

The MPC has two primary cost-tuning knobs:
    q_weight  — tracking error penalty  (higher → tighter tracking, more effort)
    r_weight  — control energy penalty  (higher → softer response, less effort)

This sweep varies both q_weight and r_weight.
Parallelism layout:
• One drone per q_weight  →  all simulated simultaneously in X.
• Serial outer loop over r_weight.

Outputs  (in results/mpc_sweep/)
────────
  scores.csv            future-biased mean ‖ep‖ (2D grid: q_weight x r_weight)
  effort.csv            future-biased mean Σ motor_thrust² (2D grid)
  r_<rval>/q<qval>_statistics.npz   raw telemetry per drone
"""

from __future__ import annotations

import argparse
import csv
import os
import sys
from pathlib import Path

# The rotating-wing command-line flags must be registered before parse_args() runs,
# which is well before SimulationApp starts. wing_cli defers every Isaac Sim import
# to spawn_vehicle(), so importing it this early is safe.
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "utils"))
from wing_cli import add_wing_arguments, spawn_vehicle, wing_run_tag, describe_wing_run

import numpy as np

import carb
from isaacsim import SimulationApp

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="LPV-MPC q/r 2D sweep for Pareto analysis")
    p.add_argument("--q-values", nargs="+", type=float, default=[5.0, 10.0, 20.0])
    p.add_argument("--r-values", nargs="+", type=float, default=[0.01, 0.02, 0.05])
    p.add_argument("--n-horizon", type=int,   default=12)
    p.add_argument("--kp-pos", type=float, default=0.50)
    p.add_argument("--kd-pos", type=float, default=1.00)
    p.add_argument("--kp-z",   type=float, default=15.0)
    p.add_argument("--kd-z",   type=float, default=8.0)
    p.add_argument("--duration",        type=float, default=20.0)
    p.add_argument("--target-altitude", type=float, default=1.5)
    p.add_argument("--spacing",         type=float, default=4.0)
    p.add_argument("--bounds", nargs=6, type=float, default=[-12., 12., -12., 12., 0., 20.])
    p.add_argument("--goal-seed",   type=int,  default=7)
    p.add_argument("--motor-moment", type=float, default=None,
                   help="Motor moment constant km. If provided, overrides default.")
    p.add_argument("--headless",    action="store_true", default=False)
    p.add_argument("--no-headless", dest="headless", action="store_false")
    add_wing_arguments(p)
    return p.parse_args()

ARGS = parse_args()

simulation_app = SimulationApp({
    "headless": ARGS.headless,
    "hide_ui": True if ARGS.headless else None,
    "disable_viewport_updates": True if ARGS.headless else False,
    "multi_gpu": False,
    "max_gpu_count": 1,
    "active_gpu": 0,
})

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

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "utils"))
from lpv_mpc_controller import LPVMPCController

_BOUNDS_DIAG = 45.0 # Precomputed diagonal for OOB penalty

def stability_score(npz_path: str, T_total: float) -> float:
    try:
        d = np.load(npz_path)
        ep = d["ep"]
        t = d["time"]
        if len(t) < 2: return np.inf
        oob_time = float(d.get("oob_time", float("nan")))
        drone_went_oob = np.isfinite(oob_time)
        if T_total < 1e-6: T_total = float(t[-1])
        errors = np.linalg.norm(ep, axis=1)
        if not np.all(np.isfinite(errors)): return np.inf
        t_arr, err_arr = t.copy(), errors.copy()
        if drone_went_oob and oob_time < T_total - 1e-6:
            dt_a = float(t[1] - t[0]) if len(t) > 1 else 1/60
            pad_t = np.arange(oob_time + dt_a, T_total, dt_a)
            if len(pad_t) > 0:
                t_arr = np.concatenate([t_arr, pad_t])
                err_arr = np.concatenate([err_arr, np.full(len(pad_t), _BOUNDS_DIAG)])
        w = t_arr / T_total
        w_sum = float(np.sum(w))
        if w_sum < 1e-12: return np.inf
        s = float(np.dot(w, err_arr) / w_sum)
        return s if np.isfinite(s) else np.inf
    except Exception as exc:
        carb.log_warn(f"[mpc_sweep] score read failed: {exc}")
        return np.inf

def effort_score(npz_path: str, T_total: float) -> float:
    try:
        d = np.load(npz_path)
        t = d["time"]
        if len(t) < 2: return np.inf
        oob_time = float(d.get("oob_time", float("nan")))
        drone_went_oob = np.isfinite(oob_time)
        if T_total < 1e-6: T_total = float(t[-1])
        mt = d.get("motor_thrusts")
        if mt is None or mt.ndim != 2 or mt.shape[0] != len(t): return np.inf
        eff = np.sum(np.square(mt.astype(float)), axis=1)
        if not np.all(np.isfinite(eff)): return np.inf
        t_arr, eff_arr = t.copy(), eff.copy()
        if drone_went_oob and oob_time < T_total - 1e-6:
            dt_a = float(t[1] - t[0]) if len(t) > 1 else 1/60
            pad_t = np.arange(oob_time + dt_a, T_total, dt_a)
            if len(pad_t) > 0:
                worst = float(np.max(eff_arr)) if eff_arr.size else 0.0
                t_arr = np.concatenate([t_arr, pad_t])
                eff_arr = np.concatenate([eff_arr, np.full(len(pad_t), worst)])
        w = t_arr / T_total
        w_sum = float(np.sum(w))
        if w_sum < 1e-12: return np.inf
        s = float(np.dot(w, eff_arr) / w_sum)
        return s if np.isfinite(s) else np.inf
    except Exception as exc:
        carb.log_warn(f"[mpc_sweep] effort read failed: {exc}")
        return np.inf


class BoundedMPCBackend(Backend):
    def __init__(self, inner: LPVMPCController, results_file: str):
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

    def update(self, dt: float):
        self.total_time += dt
        if self.is_oob:
            return np.zeros(4)
        thrusts = self.inner.update(dt)
        self.position = self.inner.p
        return thrusts

    def check_oob(self, bounds: list[float]) -> bool:
        if self.is_oob:
            return True
        x, y, z = self.position
        xmin, xmax, ymin, ymax, zmin, zmax = bounds
        if not (xmin < x < xmax and ymin < y < ymax and zmin < z < zmax):
            self.is_oob = True
            self.oob_time = self.total_time
            return True
        return False

    def reset(self):
        self.total_time = 0.0
        self.oob_time   = float("nan")
        self.is_oob     = False
        self._stopped   = False
        self.position   = np.zeros(3)
        self.inner.reset()

    def start(self):
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
                carb.log_warn(f"[mpc_sweep] oob_time append failed: {exc}")

    def update_sensor(self, sensor_type, data):
        if not self.is_oob:
            self.inner.update_sensor(sensor_type, data)

    def update_graphical_sensor(self, sensor_type, data):
        if not self.is_oob:
            self.inner.update_graphical_sensor(sensor_type, data)

    def update_state(self, state):
        self.position = state.position.copy()
        if not self.is_oob:
            self.inner.update_state(state)

    def input_reference(self):
        return [0., 0., 0., 0.] if self.is_oob else self.inner.input_reference()

def save_qr_csv(q_vals, r_vals, scores_2d, out_dir: Path):
    path = out_dir / "scores.csv"
    with open(str(path), "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["q \\ r"] + [f"{r:.4f}" for r in r_vals])
        for i, q in enumerate(q_vals):
            row = [f"{q:.4f}"]
            for j in range(len(r_vals)):
                s = scores_2d[i, j]
                row.append(f"{s:.6f}" if np.isfinite(s) else "inf")
            writer.writerow(row)
    carb.log_warn(f"[mpc_sweep] scores CSV → {path}")

def save_qr_effort_csv(q_vals, r_vals, efforts_2d, out_dir: Path):
    path = out_dir / "effort.csv"
    with open(str(path), "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["q \\ r"] + [f"{r:.4f}" for r in r_vals])
        for i, q in enumerate(q_vals):
            row = [f"{q:.4f}"]
            for j in range(len(r_vals)):
                e = efforts_2d[i, j]
                row.append(f"{e:.6f}" if np.isfinite(e) else "inf")
            writer.writerow(row)
    carb.log_warn(f"[mpc_sweep] effort CSV → {path}")

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

def main():
    timeline   = omni.timeline.get_timeline_interface()
    render_sim = not ARGS.headless
    bounds     = ARGS.bounds
    q_vals     = ARGS.q_values
    r_vals     = ARGS.r_values
    n_q        = len(q_vals)
    n_r        = len(r_vals)

    base_dir     = Path(__file__).resolve().parent.parent
    results_base = base_dir / "results" / "mpc_sweep"
    if ARGS.motor_moment is not None:
        results_base = results_base / f"mm_{ARGS.motor_moment:.1e}"
    results_base.mkdir(parents=True, exist_ok=True)

    carb.log_warn("\n" + "="*60)
    carb.log_warn("[mpc_sweep] LPV-MPC 2D Sweep")
    carb.log_warn(f"[mpc_sweep] q_values    : {q_vals}")
    carb.log_warn(f"[mpc_sweep] r_values    : {r_vals}")
    carb.log_warn(f"[mpc_sweep] motor_moment: {ARGS.motor_moment}")
    carb.log_warn(f"[mpc_sweep] duration    : {ARGS.duration} s")
    carb.log_warn("="*60)

    pg        = PegasusInterface()
    pg._world = World(**pg._world_settings)
    world     = pg.world
    pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])
    for _ in range(20):
        world.step(render=render_sim)

    x_min, x_max = bounds[0], bounds[1]
    x_margin  = 1.5
    x_usable  = x_max - x_min - 2.0 * x_margin
    # A lone drone has no neighbour to space from, so it is given the whole usable
    # width. Zero here would collapse its bounds box to zero width and flag it out of
    # bounds on the first step, silently invalidating every single-point sweep.
    auto_spacing = x_usable / (n_q - 1) if n_q > 1 else x_usable
    spacing   = min(ARGS.spacing, auto_spacing)
    x_start   = x_min + x_margin

    spawn_y  = -1.5
    spawn_z  = 0.07
    half     = spacing / 2.0
    y_min, y_max = bounds[2], bounds[3]
    z_min, z_max = bounds[4], bounds[5]

    tmp_files = [str(results_base / f"_mpc_sweep_tmp_{i}.npz") for i in range(n_q)]
    drones = []

    for i, q in enumerate(q_vals):
        spawn_x   = x_start + i * spacing
        spawn_pos = [spawn_x, spawn_y, spawn_z]
        target    = (spawn_x, spawn_y, float(ARGS.target_altitude))
        drone_bounds = [spawn_x - half, spawn_x + half, y_min, y_max, z_min, z_max]

        inner = LPVMPCController(
            target_position=target,
            results_file=tmp_files[i],
            q_weight=q,
            r_weight=r_vals[0], # will be overwritten
            N=ARGS.n_horizon,
            kp_pos=ARGS.kp_pos,
            kd_pos=ARGS.kd_pos,
            kp_z=ARGS.kp_z,
            kd_z=ARGS.kd_z,
        )
        backend = BoundedMPCBackend(inner, tmp_files[i])
        cfg          = MultirotorConfig()
        cfg.backends = [backend]
        if ARGS.motor_moment is not None:
            cfg.thrust_curve = QuadraticThrustCurve(config={"motor_moment": ARGS.motor_moment})

        vehicle = spawn_vehicle(
            f"/World/mpc_q{i}", ROBOTS["Iris"], i, spawn_pos,
            Rotation.from_euler("XYZ", [0., 0., 0.], degrees=True).as_quat(),
            config=cfg,
            args=ARGS,
        )
        drones.append((vehicle, backend, spawn_pos, drone_bounds))

    world.reset()
    for _ in range(10):
        world.step(render=render_sim)

    all_scores  = np.full((n_q, n_r), np.inf)
    all_efforts = np.full((n_q, n_r), np.inf)

    for j_r, r in enumerate(r_vals):
        carb.log_warn(f"\n[mpc_sweep] r_weight = {r:.4f}  ({j_r+1}/{n_r})")
        r_dir = results_base / f"r_{r:.4f}"
        r_dir.mkdir(parents=True, exist_ok=True)

        for i_q, (vehicle, backend, spawn_pos, _) in enumerate(drones):
            q = q_vals[i_q]
            backend.inner.q_weight = q
            backend.inner.r_weight = r
            
            vehicle.set_world_pose(position=spawn_pos, orientation=[0., 0., 0., 1.])
            vehicle.set_linear_velocity([0., 0., 0.])
            vehicle.set_angular_velocity([0., 0., 0.])

            backend.reset()
            backend.start()

        for _ in range(5):
            world.step(render=render_sim)

        timeline.play()
        sim_time = 0.0
        dt = 1.0 / 60.0

        while simulation_app.is_running() and sim_time < ARGS.duration:
            world.step(render=render_sim)
            sim_time += dt

            all_oob = True
            for _, backend, _, drone_bounds in drones:
                if not backend.is_oob:
                    all_oob = False
                    backend.check_oob(drone_bounds)

            if all_oob:
                break

        timeline.stop()

        for i_q, (_, backend, _, _) in enumerate(drones):
            if not backend._stopped:
                backend.stop()

            q = q_vals[i_q]
            src = tmp_files[i_q]
            dst = r_dir / f"q_{q:.2f}_statistics.npz"

            s = stability_score(src, ARGS.duration)
            e = effort_score(src, ARGS.duration)
            all_scores[i_q, j_r] = s
            all_efforts[i_q, j_r] = e

            carb.log_warn(f"  q={q:.2f} r={r:.4f} → score={s:.4f} effort={e:.4f}")

            if os.path.isfile(src):
                try:
                    os.rename(src, str(dst))
                except Exception:
                    pass

    save_qr_csv(q_vals, r_vals, all_scores, results_base)
    save_qr_effort_csv(q_vals, r_vals, all_efforts, results_base)

    # Heatmaps
    save_heatmap(r_vals, q_vals, all_scores, results_base, "score_heatmap.png",
                 "LPV-MPC Stability Score Heatmap",
                 "R Weight (Control Penalty)", "Q Weight (State Penalty)", "Stability Score [m]", "RdYlGn_r")
    save_heatmap(r_vals, q_vals, all_efforts, results_base, "effort_heatmap.png",
                 "LPV-MPC Control Effort Heatmap",
                 "R Weight (Control Penalty)", "Q Weight (State Penalty)", "Control Effort [N²]", "YlOrRd")

    print(f"\n[mpc_sweep] results → {results_base}")
    
    carb.log_warn("=== mpc_sweep complete ===")
    
    for f in tmp_files:
        if os.path.exists(f):
            os.remove(f)
    try: timeline.stop()
    except: pass
    try: world.stop()
    except: pass
    for fn in ("stop", "clear_all_callbacks", "clear"):
        try: getattr(world, fn)()
        except: pass
    simulation_app.close()

if __name__ == "__main__":
    main()
