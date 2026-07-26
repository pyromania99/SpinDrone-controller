#!/usr/bin/env python3
"""
pd_sweep.py  —  Cascaded PD 2D attitude-gain sweep for Pareto analysis.

A 2D sweep over combinations of Kp_att and Kd_att:
    Kp_att  → from --kp-att-values  (parallel drones, one per value)
    Kd_att  → from --kd-att-values  (sequential passes)

Position gains (Kp_xy, Kd_xy, Kp_z, Kd_z) are kept fixed.

Parallelism layout
──────────────────
• One drone per Kp_att value  →  all simulated simultaneously in one pass.
• Outer loop over Kd_att values → successive simulation passes.

Outputs  (in results/pd_sweep/)
────────
  scores.csv           future-biased mean ‖ep‖ per combination
  effort.csv           future-biased mean Σ motor_thrust² per combination
  kd_att_<kd>/...      raw telemetry per drone and kd_att step

Usage
─────
  isaac_run examples/controller_tests/sweeps/pd_sweep.py \
      --kp-att-values 0.3 0.5 0.65 1.0 1.5 \
      --kd-att-values 5.0 8.0 10.0 15.0 \
      --motor-moments 0.0 1e-7 2e-7 4e-7 \
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
        description="Cascaded PD 2D attitude-gain sweep"
    )
    # ── Swept attitude gains ───────────────────────────────────────────────────
    p.add_argument(
        "--kp-att-values", nargs="+", type=float,
        default=[10.0, 15.0, 20.0, 25.0, 30.0],
        help="Attitude proportional gains to sweep (parallel drones)",
    )
    p.add_argument(
        "--kd-att-values", nargs="+", type=float,
        default=[0.2, 0.5, 1.0, 1.5, 2.0, 3.0],
        help="Attitude derivative gains to sweep (sequential passes)",
    )
    # Kd_att box note: the previous ceiling of 1.0 excluded the spinning
    # optimum. Under spin the GA (which searches Kd_att up to 3.0) converges
    # on Kd_att ≈ 1.4 and scores ~0.070, while this grid — capped at 1.0 —
    # bottomed out at ~0.48. That gap was the box, not the search method. The
    # ceiling now matches PD_GENES in genetic_sweep.py so grid and GA cover
    # the same space and the comparison between them is about method alone.
    # ── Fixed position gains ─────────────────────────────────────────────────
    # Defaults MUST match CascadedPDController's own class defaults (and
    # EigenController's kp_pos/kd_pos), since genetic_sweep.py's PD search
    # never overrides these and relies on the controller's built-in values.
    # Diverging here silently makes this grid sweep's outer loop stronger
    # than every other PD/Eigen run, breaking cross-sweep comparability.
    p.add_argument("--kp-xy",  type=float, default=0.50,
                   help="Fixed proportional XY gain")
    p.add_argument("--kd-xy",  type=float, default=1.0,
                   help="Fixed derivative XY gain")
    p.add_argument("--kp-z",   type=float, default=15.0,
                   help="Fixed proportional Z gain")
    p.add_argument("--kd-z",   type=float, default=8.0,
                   help="Fixed derivative Z gain")
    # ── Simulation ────────────────────────────────────────────────────────────
    p.add_argument("--duration",        type=float, default=25.0,
                   help="Simulation seconds per run (default: 25.0 to ensure >20s of stable hover)")
    p.add_argument("--target-altitude", type=float, default=1.5)
    p.add_argument("--spacing",         type=float, default=4.0)
    p.add_argument(
        "--bounds", nargs=6, type=float,
        default=[-12., 12., -12., 12., 0., 20.],
        metavar=("XMIN", "XMAX", "YMIN", "YMAX", "ZMIN", "ZMAX"),
    )
    p.add_argument("--goal-seed",  type=int,  default=7)
    p.add_argument("--motor-moments", nargs="+", type=float,
                   default=[0.0, 1e-7, 2e-7, 4e-7],
                   help="km (motor moment) values to sweep — outer serial loop "
                        "(same convention as genetic_sweep.py / eigen_stability_sweep.py)")
    # ── Step-response validation of the (score, effort) Pareto front ────────
    # Same methodology/defaults as genetic_sweep.py and eigen_stability_sweep.py,
    # so results are comparable across GA and grid searches: after the grid is
    # complete, the points on its (score, effort) Pareto front get ONE
    # additional pass against a target offset from hover.
    p.add_argument("--validate-pareto", action="store_true", default=True,
                   help="Run the step-response validation pass on the grid's Pareto front")
    p.add_argument("--no-validate-pareto", dest="validate_pareto", action="store_false")
    p.add_argument("--validation-radius", type=float, default=2.0,
                   help="Lateral offset (m) of the validation target from the hover point")
    p.add_argument("--validation-threshold", type=float, default=2.0,
                   help="Max stability score during the validation step to count as feasible")
    p.add_argument("--validation-seed", type=int, default=123,
                   help="Separate RNG seed for validation target directions")
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
import sweep_metrics as sm


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
        # Check for crashing on the floor after initial takeoff
        if self.total_time > 2.0 and z < 0.15:
            self.is_oob   = True
            self.oob_time = self.total_time
            carb.log_warn(f"[pd_sweep] CRASH DETECTED at z={z:.3f} t={self.total_time:.2f}s")
            return True
        return False


# ── Score helpers (copied from eigen_stability_sweep) ─────────────────────────

_BOUNDS_DIAG = sm.bounds_diagonal(ARGS.bounds)


def _validation_target(spawn_pos, altitude, radius, rng):
    """Random-direction, fixed-radius offset from the hover point — same
    convention as genetic_sweep.py/eigen_stability_sweep.py's
    _validation_target, so step-response results are directly comparable
    across GA and grid searches for both controllers."""
    theta = float(rng.uniform(0.0, 2.0 * math.pi))
    return (
        spawn_pos[0] + radius * math.cos(theta),
        spawn_pos[1] + radius * math.sin(theta),
        float(altitude),
    )


def _pareto_mask(scores, efforts):
    """Boolean mask of Pareto-optimal (min score, min effort) points."""
    scores = np.asarray(scores, dtype=float)
    efforts = np.asarray(efforts, dtype=float)
    n = len(scores)
    mask = np.zeros(n, dtype=bool)
    for i in range(n):
        if not (np.isfinite(scores[i]) and np.isfinite(efforts[i])):
            continue
        dominated = False
        for j in range(n):
            if i == j or not (np.isfinite(scores[j]) and np.isfinite(efforts[j])):
                continue
            if scores[j] <= scores[i] and efforts[j] <= efforts[i] and \
               (scores[j] < scores[i] or efforts[j] < efforts[i]):
                dominated = True
                break
        mask[i] = not dominated
    return mask


def stability_score(npz_path: str, T_total: float) -> float:
    """Future-biased mean position error. See examples/utils/sweep_metrics.py."""
    return sm.compute_metrics(npz_path, T_total, _BOUNDS_DIAG)["score"]


def run_metrics(npz_path: str, T_total: float) -> dict:
    """Full metric dict for one run — score, the effort DC/AC split, spin rate."""
    return sm.compute_metrics(npz_path, T_total, _BOUNDS_DIAG)


def effort_score(npz_path: str, T_total: float) -> float:
    """Legacy total-thrust Saturated Effort Score.

    Retained so existing archives stay comparable, but it is ~95 % constant
    hover bias (the Iris hover floor alone is ~73.7 N²) and barely resolves
    control activity. Prefer the ``effort_ac_ses`` channel from run_metrics()
    for anything being optimised. See examples/utils/sweep_metrics.py.
    """
    return sm.compute_metrics(npz_path, T_total, _BOUNDS_DIAG)["effort"]


# ── CSV helpers ────────────────────────────────────────────────────────

def save_kpkd_csv(kp_vals, kd_vals, data_2d, out_dir: Path, filename: str) -> None:
    path = out_dir / filename
    with open(str(path), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["kp_att \\ kd_att"] + [f"{kd:.4f}" for kd in kd_vals])
        for i, kp in enumerate(kp_vals):
            row = [f"{kp:.4f}"]
            for j in range(len(kd_vals)):
                v = data_2d[i, j]
                row.append(f"{v:.6f}" if np.isfinite(v) else "inf")
            w.writerow(row)
    carb.log_warn(f"[pd_sweep] CSV → {path}")

def save_grid_archive(kp_vals, kd_vals, scores_2d, efforts_2d, mm, out_dir: Path,
                       step_scores=None, step_efforts=None, metrics=None) -> None:
    """Long-format grid_archive.csv for interactive_dashboard.py.

    Column names (Kp_att, Kd_att) match the gene names genetic_sweep.py
    uses for the PD controller so grid and GA points overlay on the same
    parallel-coordinates axes. The metric columns come from
    sweep_metrics.METRIC_KEYS, so this archive carries the same set — effort
    DC/AC split and spin rate included — as genetic_archive.csv.

    step_scores/step_efforts: optional {(kp, kd): value} dicts from the
    Pareto-front step-response validation pass — NaN for any (kp, kd) that
    wasn't on the front and so was never validated.
    metrics: optional {(kp, kd): metric dict} from run_metrics().
    """
    path = out_dir / "grid_archive.csv"
    with open(str(path), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["gen", "Kp_att", "Kd_att", "motor_moment"] + sm.METRIC_KEYS)
        for i, kp in enumerate(kp_vals):
            for j, kd in enumerate(kd_vals):
                m = dict(sm.METRIC_DEFAULTS)
                if metrics:
                    m.update(metrics.get((kp, kd), {}))
                # scores_2d/efforts_2d stay authoritative for the two primary
                # objectives — they are what the Pareto mask was computed on.
                m["score"] = scores_2d[i, j]
                m["effort"] = efforts_2d[i, j]
                if step_scores:
                    m["step_score"] = step_scores.get((kp, kd), float("nan"))
                if step_efforts:
                    m["step_effort"] = step_efforts.get((kp, kd), float("nan"))
                w.writerow(
                    [-1, f"{kp:.6f}", f"{kd:.6f}", mm]
                    + [sm.format_metric(k, m[k]) for k in sm.METRIC_KEYS]
                )
    carb.log_warn(f"[pd_sweep] archive CSV → {path}")


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


# ── Pareto-front step-response validation ─────────────────────────────────────

def run_pd_validation_pass(world, timeline, drones, tmp_files, pairs, val_targets):
    """Re-run exactly the (kp, kd) pairs in `pairs` (the grid's current
    Pareto-front points) against `val_targets` instead of their normal hover
    target, batching them through the existing n_kp drone slots. Returns
    {(kp, kd): (score, effort)}. Mirrors genetic_sweep.py's validate_subset
    and eigen_stability_sweep.py's run_ab_validation_pass, using the same SES
    scoring so results are directly comparable across GA and grid searches."""
    render_sim = not ARGS.headless
    n_slots = len(drones)
    results = {}
    pairs = list(pairs)

    for start in range(0, len(pairs), n_slots):
        chunk = pairs[start:start + n_slots]

        for slot in range(n_slots):
            vehicle, backend, spawn_pos, _ = drones[slot]
            kp, kd = chunk[slot] if slot < len(chunk) else chunk[0]  # harmless filler past chunk end

            backend.inner.Kp_att = np.array([kp, kp], dtype=float)
            backend.inner.Kd_att = np.array([kd, kd], dtype=float)
            backend.inner.hover_target = np.asarray(val_targets[slot], dtype=float)

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
                if backend.is_oob:
                    continue
                all_oob = False
                backend.check_oob(drone_bounds)
            if all_oob:
                break
        timeline.stop()

        for slot in range(len(chunk)):
            _, backend, _, _ = drones[slot]
            if not backend._stopped:
                backend.stop()
            m = run_metrics(tmp_files[slot], ARGS.duration)
            results[chunk[slot]] = (m["score"], m["effort"])

    return results


# ── Main simulation sweep ─────────────────────────────────────────────────────

def main():
    timeline    = omni.timeline.get_timeline_interface()
    render_sim  = not ARGS.headless
    bounds      = ARGS.bounds
    kp_vals     = ARGS.kp_att_values   # parallel axis
    kd_vals     = ARGS.kd_att_values   # sequential axis
    n_kp        = len(kp_vals)
    n_kd        = len(kd_vals)

    base_dir     = Path(__file__).resolve().parent.parent
    results_base = base_dir / "results" / "pd_sweep"
    results_base.mkdir(parents=True, exist_ok=True)

    carb.log_warn("\n" + "="*60)
    carb.log_warn("[pd_sweep] Cascaded PD 2D Attitude-Gain Sweep")
    carb.log_warn(f"[pd_sweep] kp_att values : {kp_vals}")
    carb.log_warn(f"[pd_sweep] kd_att values : {kd_vals}")
    carb.log_warn(f"[pd_sweep] Kp_xy fixed   : {ARGS.kp_xy}   Kd_xy fixed  : {ARGS.kd_xy}")
    carb.log_warn(f"[pd_sweep] Kp_z fixed    : {ARGS.kp_z}    Kd_z fixed   : {ARGS.kd_z}")
    carb.log_warn(f"[pd_sweep] motor_moments : {ARGS.motor_moments}")
    carb.log_warn(f"[pd_sweep] duration      : {ARGS.duration} s")
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
    mm_init = ARGS.motor_moments[0]

    # ── Spawn one drone per kp_att ────────────────────────────────────────────
    for i, kp_att in enumerate(kp_vals):
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
            Kp_xy=(ARGS.kp_xy, ARGS.kp_xy),
            Kd_xy=(ARGS.kd_xy, ARGS.kd_xy),
            Kp_z=ARGS.kp_z,
            Kd_z=ARGS.kd_z,
            Kp_att=(kp_att, kp_att),
            Kd_att=(kd_vals[0], kd_vals[0]),   # overwritten each pass
        )
        backend = BoundedPDBackend(inner, tmp_files[i])

        cfg          = MultirotorConfig()
        cfg.backends = [backend]
        cfg.thrust_curve = QuadraticThrustCurve(config={"motor_moment": mm_init})

        vehicle = Multirotor(
            f"/World/pd_kp{i}",
            ROBOTS["Iris"],
            i,
            spawn_pos,
            Rotation.from_euler("XYZ", [0., 0., 0.], degrees=True).as_quat(),
            config=cfg,
        )
        drones.append((vehicle, backend, spawn_pos, drone_bounds))
        carb.log_warn(f"  [Kp_att={kp_att:.4f}]  drone {i} spawned at x={spawn_x:.1f}")

    world.reset()
    for _ in range(10):
        world.step(render=render_sim)

    val_rng = np.random.default_rng(ARGS.validation_seed)

    # ── Outer loop: motor moments ────────────────────────────────────────────
    for mm in ARGS.motor_moments:
        carb.log_warn(f"\n{'='*60}")
        carb.log_warn(f"[pd_sweep] motor_moment = {mm:.2e}")
        carb.log_warn(f"{'='*60}")

        out_dir = results_base / f"mm_{mm:.1e}"
        out_dir.mkdir(parents=True, exist_ok=True)

        all_scores  = np.full((n_kp, n_kd), np.inf)
        all_efforts = np.full((n_kp, n_kd), np.inf)
        # Full metric dict per (kp, kd) — carries the effort DC/AC split and
        # the spin rate through to grid_archive.csv alongside score/effort.
        all_metrics = {}

        # ── Simulate ─────────────────────────────────────────────────────────
        for j_kd, kd_att in enumerate(kd_vals):
            carb.log_warn(f"\n[pd_sweep] Kd_att = {kd_att:.4f}  ({j_kd+1}/{n_kd})")
            kd_dir = out_dir / f"kd_att_{kd_att:.4f}"
            kd_dir.mkdir(parents=True, exist_ok=True)

            for i, (vehicle, backend, spawn_pos, _) in enumerate(drones):
                vehicle._thrusters = QuadraticThrustCurve(config={"motor_moment": mm})
                vehicle.set_world_pose(position=spawn_pos, orientation=[0., 0., 0., 1.])
                vehicle.set_linear_velocity([0., 0., 0.])
                vehicle.set_angular_velocity([0., 0., 0.])
                backend.inner.Kd_att = np.array([kd_att, kd_att], dtype=float)
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

            for i, kp_att in enumerate(kp_vals):
                _, backend, _, _ = drones[i]
                if not backend._stopped:
                    backend.stop()

                src = tmp_files[i]
                dst = kd_dir / f"kp_att_{kp_att:.4f}_statistics.npz"

                # One npz load for every metric — score, the effort DC/AC
                # split and the spin rate all come out of the same pass.
                m = run_metrics(src, ARGS.duration)
                s, e = m["score"], m["effort"]
                all_scores[i, j_kd] = s
                all_efforts[i, j_kd] = e
                all_metrics[(kp_att, kd_att)] = m

                s_str = "∞" if not np.isfinite(s) else f"{s:.4f}"
                e_str = "∞" if not np.isfinite(e) else f"{e:.4f}"
                ac_str = "∞" if not np.isfinite(m["effort_ac"]) else f"{m['effort_ac']:.4f}"
                r_str = "  n/a" if not np.isfinite(m["r_ss"]) else f"{m['r_ss']:5.2f}"
                carb.log_warn(f"  kp_att={kp_att:<8.4f} kd_att={kd_att:<8.4f} score={s_str:<12} "
                              f"effort={e_str:<12} effort_ac={ac_str:<10} r_ss={r_str}")

                if os.path.isfile(src):
                    try:
                        os.rename(src, str(dst))
                    except Exception as exc:
                        carb.log_warn(f"  move failed: {exc}")

        carb.log_warn("="*60)

        # Save CSVs
        save_kpkd_csv(kp_vals, kd_vals, all_scores,  out_dir, "scores.csv")
        save_kpkd_csv(kp_vals, kd_vals, all_efforts, out_dir, "effort.csv")

        # ── Pareto-front step-response validation (one pass, front only) ────
        step_scores, step_efforts = None, None
        if ARGS.validate_pareto:
            pareto = _pareto_mask(all_scores.ravel(), all_efforts.ravel()).reshape(all_scores.shape)
            pareto_pairs = [
                (kp_vals[i], kd_vals[j])
                for i in range(n_kp) for j in range(n_kd)
                if pareto[i, j]
            ]
            carb.log_warn(
                f"[pd_sweep] mm={mm:.1e}: step-response validation of {len(pareto_pairs)} "
                f"Pareto-front points (radius={ARGS.validation_radius}m, "
                f"threshold={ARGS.validation_threshold})..."
            )
            val_targets = [
                _validation_target(drones[i][2], ARGS.target_altitude,
                                    ARGS.validation_radius, val_rng)
                for i in range(n_kp)
            ]
            val_results = run_pd_validation_pass(
                world, timeline, drones, tmp_files, pareto_pairs, val_targets
            )
            step_scores  = {k: v[0] for k, v in val_results.items()}
            step_efforts = {k: v[1] for k, v in val_results.items()}
            n_feasible = sum(
                1 for s in step_scores.values()
                if np.isfinite(s) and s <= ARGS.validation_threshold
            )
            carb.log_warn(
                f"[pd_sweep] mm={mm:.1e}: {n_feasible}/{len(pareto_pairs)} Pareto-front "
                f"points passed step-response validation"
            )

        save_grid_archive(kp_vals, kd_vals, all_scores, all_efforts, mm,
                           out_dir, step_scores=step_scores, step_efforts=step_efforts,
                           metrics=all_metrics)

        # Heatmaps
        save_heatmap(kd_vals, kp_vals, all_scores, out_dir, "score_heatmap.png",
                     "Cascaded PD Stability Score Heatmap (Att Gains)",
                     "Derivative Attitude Gain Kd_att", "Proportional Attitude Gain Kp_att",
                     "Stability Score [m]", "RdYlGn_r")
        save_heatmap(kd_vals, kp_vals, all_efforts, out_dir, "effort_heatmap.png",
                     "Cascaded PD Control Effort Heatmap (Att Gains)",
                     "Derivative Attitude Gain Kd_att", "Proportional Attitude Gain Kp_att",
                     "Control Effort [N²]", "YlOrRd")

        carb.log_warn(f"\n[pd_sweep] mm={mm:.1e} results → {out_dir}")

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
