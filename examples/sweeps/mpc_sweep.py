#!/usr/bin/env python3
"""
mpc_sweep.py  —  LPV-MPC q/r weight 2D sweep for Pareto analysis.

The MPC has two primary cost-tuning knobs:
    q_weight  — tracking error penalty  (higher → tighter tracking, more effort)
    r_weight  — control energy penalty  (higher → softer response, less effort)

Parallelism layout:
• One drone per q_weight  →  all simulated simultaneously in X.
• Serial loop over r_weight, inside a serial outer loop over motor_moment.

Brought in line with the other sweeps
─────────────────────────────────────
This file used to be the odd one out in three ways, each of which quietly
excluded the MPC from any cross-controller analysis:

**It scored itself.** It carried private copies of ``stability_score`` and
``effort_score`` rather than importing examples/utils/sweep_metrics.py — and
the copies had drifted. Its ``effort_score`` returned ``max(effort)`` where
every other sweep reports a future-biased *mean*, so the MPC's effort numbers
were a different statistic wearing the same column name and were never
comparable to the numbers they were being placed beside.

**It wrote no archive.** The 2D ``scores.csv``/``effort.csv`` grids are
readable but are not the long-format one-row-per-run layout that
evaluations/select_pareto_top.py consumes, so ``pareto_top.json`` contained
zero MPC configurations and the MPC had no replayable operating points at all.
It now also writes ``grid_archive.csv`` with the shared METRIC_KEYS columns.

**It ran at one spin rate.** ``--motor-moment`` was singular and optional, and
run_all_sweeps.sh never passed it — so every MPC run inherited
QuadraticThrustCurve's default of km = 6e-7, the *fastest* spin on the ladder
(~29 rad/s), while the Eigen and PD sweeps covered six values from 0 up. Any
comparison against them was reading the MPC's worst-case spin against their
average. ``--motor-moments`` is now plural and is the outer serial loop.

Outputs  (in results/mpc_sweep/mm_<km>/)
────────
  grid_archive.csv      one row per run: mpc_q, mpc_r, motor_moment + METRIC_KEYS
  scores.csv            future-biased mean ‖ep‖ (2D grid: q_weight x r_weight)
  effort.csv            effort_ac_ses (2D grid)
  r_<rval>/q_<qval>_statistics.npz   raw telemetry per drone
"""

from __future__ import annotations

import argparse
import csv
import os
import sys
from pathlib import Path
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
    p.add_argument("--duration",        type=float, default=25.0,
                   help="Simulation seconds per run (default: 25.0 to ensure >20s of stable hover)")
    p.add_argument("--target-altitude", type=float, default=1.5)
    p.add_argument("--spacing",         type=float, default=4.0)
    p.add_argument("--bounds", nargs=6, type=float, default=[-12., 12., -12., 12., 0., 20.])
    p.add_argument("--goal-seed",   type=int,  default=7)
    p.add_argument("--goal-radius", type=float, default=None,
                   help="Lateral distance [m] from spawn XY to the goal. Every run "
                        "is a takeoff followed by a translation of this distance in "
                        "a random direction; see examples/utils/sweep_task.py.")
    p.add_argument("--motor-moments", nargs="+", type=float,
                   default=[0.0, 1e-7, 2e-7, 2.7e-7, 4e-7, 6e-7],
                   help="Motor moment constants km — the spin ladder, as the "
                        "outer serial loop. This was singular and defaulted to "
                        "None, which left every run at QuadraticThrustCurve's "
                        "6e-7 (the fastest spin) while the other sweeps covered "
                        "the whole ladder.")
    p.add_argument("--spawn-altitude", type=float, default=None,
                   help="Altitude [m] to spawn at. Defaults to a ground start, so "
                        "every run includes the takeoff transient. Pass the target "
                        "altitude for the old air start.")
    p.add_argument("--headless",    action="store_true", default=False)
    p.add_argument("--no-headless", dest="headless", action="store_false")
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

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "utils"))
from lpv_mpc_controller import LPVMPCController
import sim_clock
import sweep_metrics as sm
import sweep_task as task

# Resolved after the late imports so every code path below sees numbers. The
# task is a ground start followed by a translation to a laterally offset goal.
ARGS.spawn_altitude = task.resolve_spawn_altitude(ARGS.spawn_altitude)
if ARGS.goal_radius is None:
    ARGS.goal_radius = task.DEFAULT_GOAL_RADIUS

# The arena diagonal used to pad an early crash out to full duration. Computed
# from --bounds rather than the hardcoded 45.0 this file used to carry: the
# default box is 24x24x20, whose diagonal is 39.40 m, so the old constant
# charged crashed runs 14 % more than every other sweep charged them.
_BOUNDS_DIAG = sm.bounds_diagonal(ARGS.bounds)


class BoundedMPCBackend(Backend):
    def __init__(self, inner: LPVMPCController, results_file: str):
        super().__init__(config=None)
        self.inner        = inner
        self.results_file = results_file
        self.total_time   = 0.0
        self._stopped     = False
        self.position     = np.zeros(3)
        # Recorded into the npz so backfill_metrics.py can group runs by spin
        # rate without having to parse it back out of the directory name.
        self.mm           = float("nan")
        self.monitor      = task.FlightMonitor(
            ARGS.target_altitude, label="mpc_sweep", log=carb.log_warn)

    # is_oob/oob_time stay part of this class's surface — the sweep loop and
    # sweep_metrics both read them — but FlightMonitor is the single owner of
    # the flight-phase state behind them.
    @property
    def is_oob(self) -> bool:
        return self.monitor.failed

    @property
    def oob_time(self) -> float:
        return self.monitor.oob_time

    def initialize(self, vehicle):
        super().initialize(vehicle)
        self.inner.initialize(vehicle)

    def update(self, dt: float):
        # Gate BEFORE accumulating, matching the other four sweeps. This used to
        # add dt first, so a failed run kept advancing its clock and reported a
        # full-duration flight to check_elapsed even after it had stopped flying.
        if self.is_oob:
            return np.zeros(4)
        self.total_time += dt
        thrusts = self.inner.update(dt)
        self.position = self.inner.p
        return thrusts

    def check_oob(self, bounds: list[float]) -> bool:
        """Advance the flight-phase monitor. True once the run is over.

        Takeoff failures and crashes are distinguished in
        examples/utils/sweep_task.py, shared with every other sweep. This used
        to also terminate on the arena walls; that gate is gone here for the
        same reason it went from the others — it made pass/fail depend on
        spacing/2, i.e. on how many drones were packed into the arena rather
        than on control quality.
        """
        return self.monitor.update(self.total_time, self.position, bounds)

    def reset(self):
        self.total_time = 0.0
        self._stopped   = False
        self.position   = np.zeros(3)
        self.monitor.reset()
        self.inner.reset()

    def start(self):
        self.monitor.reset()
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
                # Flight-phase diagnostics. min_altitude is the transient-safety
                # number the future-biased score cannot express: it is measured
                # post-takeoff, so it says how close the vehicle came to the
                # floor while actually flying.
                data["takeoff_time"]  = np.array(self.monitor.takeoff_time)
                data["min_altitude"]  = np.array(self.monitor.min_altitude)
                data["max_excursion"] = np.array(self.monitor.max_excursion)
                data["failure"]       = np.array(self.monitor.failure)
                data["motor_moment"]  = np.array(self.mm)
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

def mm_dir(mm: float) -> str:
    """Directory name for one motor moment, matching every other sweep."""
    return f"mm_{mm:.1e}"


def save_grid_archive(rows: list, out_dir: Path) -> None:
    """One row per run, in the long format select_pareto_top.py consumes.

    The 2D scores.csv/effort.csv grids remain — they are the readable form —
    but nothing downstream can consume them: the Pareto selection, the
    tuning-equivalence check and gain_seed all expect one row per run with the
    shared METRIC_KEYS columns. Without this file the MPC had no replayable
    operating points and was absent from pareto_top.json entirely.

    Column names ``mpc_q``/``mpc_r`` rather than ``q``/``r`` so they survive
    being pooled with the other controllers' archives, and so they match what
    backfill_metrics.py recovers from the file layout.
    """
    path = out_dir / "grid_archive.csv"
    with open(str(path), "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["mpc_q", "mpc_r", "motor_moment"] + sm.METRIC_KEYS)
        for row in rows:
            writer.writerow(
                [f"{row['mpc_q']:.6f}", f"{row['mpc_r']:.6f}",
                 str(row["motor_moment"])]
                + [sm.format_metric(key, row[key]) for key in sm.METRIC_KEYS])
    carb.log_warn(f"[mpc_sweep] archive → {path}")


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

    mm_vals      = ARGS.motor_moments
    base_dir     = Path(__file__).resolve().parent.parent
    results_base = task.results_root(base_dir) / "mpc_sweep"
    results_base.mkdir(parents=True, exist_ok=True)

    carb.log_warn("\n" + "="*60)
    carb.log_warn("[mpc_sweep] LPV-MPC 2D Sweep")
    carb.log_warn(f"[mpc_sweep] q_values     : {q_vals}")
    carb.log_warn(f"[mpc_sweep] r_values     : {r_vals}")
    carb.log_warn(f"[mpc_sweep] motor_moments: {mm_vals}")
    carb.log_warn(f"[mpc_sweep] duration     : {ARGS.duration} s")
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
    auto_spacing = x_usable / (n_q - 1) if n_q > 1 else 0.0
    spacing   = min(ARGS.spacing, auto_spacing)
    x_start   = x_min + x_margin

    spawn_y  = -1.5
    # Ground start: every run begins on the floor and has to take off. The air
    # start this replaced put the vehicle exactly on its target, so initial
    # position error was zero and the parameters being swept could not affect
    # the score at all. See examples/utils/sweep_task.py.
    spawn_z  = float(ARGS.spawn_altitude)
    half     = spacing / 2.0
    y_min, y_max = bounds[2], bounds[3]
    z_min, z_max = bounds[4], bounds[5]

    # Random goal direction per drone, bounded in X so a drone's target stays
    # inside its own slice of the arena and neighbours cannot be sent into each
    # other. Seeded, so the same --goal-seed reproduces the same task.
    goal_rng = np.random.default_rng(ARGS.goal_seed)
    max_dx   = task.slice_max_dx(spacing)
    task.check_goal_reachable(spawn_y, ARGS.goal_radius, bounds, label="mpc_sweep")

    tmp_files = [str(results_base / f"_mpc_sweep_tmp_{i}.npz") for i in range(n_q)]
    drones = []

    for i, q in enumerate(q_vals):
        spawn_x   = x_start + i * spacing
        spawn_pos = [spawn_x, spawn_y, spawn_z]
        target    = task.offset_target(spawn_pos, ARGS.target_altitude,
                                      ARGS.goal_radius, goal_rng,
                                      max_dx=max_dx)
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
        cfg.thrust_curve = QuadraticThrustCurve(
            config={"motor_moment": mm_vals[0]})

        vehicle = Multirotor(
            f"/World/mpc_q{i}", ROBOTS["Iris"], i, spawn_pos,
            Rotation.from_euler("XYZ", [0., 0., 0.], degrees=True).as_quat(),
            config=cfg,
        )
        drones.append((vehicle, backend, spawn_pos, drone_bounds))

    world.reset()
    for _ in range(10):
        world.step(render=render_sim)

    # ── Outer loop: motor_moment ────────────────────────────────────────────
    for i_mm, mm in enumerate(mm_vals):
        mm_base = results_base / mm_dir(mm)
        mm_base.mkdir(parents=True, exist_ok=True)
        carb.log_warn(f"\n{'='*60}\n[mpc_sweep] motor_moment = {mm:.2e} "
                      f"({i_mm+1}/{len(mm_vals)})\n{'='*60}")

        all_scores  = np.full((n_q, n_r), np.inf)
        all_efforts = np.full((n_q, n_r), np.inf)
        archive_rows = []

        for j_r, r in enumerate(r_vals):
            carb.log_warn(f"\n[mpc_sweep] r_weight = {r:.4f}  ({j_r+1}/{n_r})")
            r_dir = mm_base / f"r_{r:.4f}"
            r_dir.mkdir(parents=True, exist_ok=True)

            for i_q, (vehicle, backend, spawn_pos, _) in enumerate(drones):
                q = q_vals[i_q]
                backend.inner.q_weight = q
                backend.inner.r_weight = r
                # Both sides of the spin knob: the plant's rolling moment and
                # the controller's own km. Swapping the thrust curve in place
                # is what the Eigen sweeps do, so the vehicles are built once
                # and re-used across the whole ladder.
                vehicle._thrusters = QuadraticThrustCurve(
                    config={"motor_moment": mm})
                backend.inner.km = mm
                backend.mm = mm

                vehicle.set_world_pose(position=spawn_pos, orientation=[0., 0., 0., 1.])
                vehicle.set_linear_velocity([0., 0., 0.])
                vehicle.set_angular_velocity([0., 0., 0.])

                backend.reset()
                backend.start()

            for _ in range(5):
                world.step(render=render_sim)

            timeline.play()
            # world.step() advances ONE physics step (physics_dt = 1/250),
            # not 1/60. Counting iterations at 1/60 delivered 6 s for a 25 s
            # request. steps_for_duration asks the world for its real dt.
            n_steps, step_dt = sim_clock.steps_for_duration(world, ARGS.duration)
            sim_time = 0.0
            for _ in range(n_steps):
                if not simulation_app.is_running():
                    break
                world.step(render=render_sim)
                sim_time += step_dt

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

                # One shared metric implementation, one npz load. The private
                # copies this replaced reported max(effort) under the name
                # every other sweep uses for a future-biased mean.
                metrics = sm.compute_metrics(src, ARGS.duration, _BOUNDS_DIAG)
                all_scores[i_q, j_r] = metrics["score"]
                all_efforts[i_q, j_r] = metrics["effort_ac_ses"]
                archive_rows.append({"mpc_q": q, "mpc_r": r,
                                     "motor_moment": mm, **metrics})

                carb.log_warn(
                    f"  q={q:.2f} r={r:.4f} → score={metrics['score']:.4f} "
                    f"effort_ac_ses={metrics['effort_ac_ses']:.4f} "
                    f"stable={metrics['stable']}")

                if os.path.isfile(src):
                    try:
                        os.rename(src, str(dst))
                    except Exception:
                        pass

        save_grid_archive(archive_rows, mm_base)
        save_qr_csv(q_vals, r_vals, all_scores, mm_base)
        save_qr_effort_csv(q_vals, r_vals, all_efforts, mm_base)

        save_heatmap(r_vals, q_vals, all_scores, mm_base, "score_heatmap.png",
                     f"LPV-MPC Stability Score (km={mm:.1e})",
                     "R Weight (Control Penalty)", "Q Weight (State Penalty)",
                     "Stability Score [m]", "RdYlGn_r")
        save_heatmap(r_vals, q_vals, all_efforts, mm_base, "effort_heatmap.png",
                     f"LPV-MPC AC Control Effort (km={mm:.1e})",
                     "R Weight (Control Penalty)", "Q Weight (State Penalty)",
                     "effort_ac_ses [N²]", "YlOrRd")

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
