#!/usr/bin/env python
"""
Eigen Kp_att sensitivity sweep — Step 3 of the tuning workflow.

After the M_dyn eigenvalue sweep (eigen_stability_sweep.py) identifies the
best (ωn, ζ) operating point for each motor_moment, this script holds M_dyn
fixed at that anchor and sweeps Kp_att across a range of values, finding
the outer attitude gain that minimises position error.

M_dyn anchor (ωn, ζ) → wn_mdyn = ζ·ωn, b_mdyn = ωn·√(1−ζ²)

Parallel structure: one drone per Kp_att value, all run simultaneously.
Serial structure:   one pass per motor_moment.

Usage:
    isaac_run examples/tools/archive/eigen_kp_sweep.py \\
        --omega-n 8.0 \\
        --zeta 1.0 \\
        --motor-moments 1e-7 2.7e-7 4e-7 \\
        --kp-values 0.5 1.0 2.0 4.0 8.0 \\
        --duration 15

Kd_att is derived from the same gain law used in the M_dyn sweep:
    Kd_att = 2 · ζ_pd · √(wn_mdyn · Kp_att)
where ζ_pd is a target damping ratio for the outer PD loop (default: 1.0).
To override Kd_att entirely, use --kd-fixed.

Outputs (per motor_moment):
    results/eigen_kp_sweep/mm_<km>/kp_sweep_plot.png
    results/eigen_kp_sweep/mm_<km>/kp_scores.csv
    results/eigen_kp_sweep/mm_<km>/kp{Kp}_statistics.npz
"""

from __future__ import annotations

import argparse
import math
import os
import sys
from pathlib import Path



# ─────────────────────────── CLI ────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(
        description="Eigen Kp_att sensitivity sweep (Step 3)"
    )
    p.add_argument(
        "--omega-n", type=float, required=True,
        help="M_dyn anchor: closed-loop natural frequency ωn",
    )
    p.add_argument(
        "--zeta", type=float, default=1.0,
        help="M_dyn anchor: damping ratio ζ (default 1.0 = critical)",
    )
    p.add_argument(
        "--motor-moments", nargs="+", type=float,
        default=[2.7e-7],
        help="km values to test (outer serial loop)",
    )
    p.add_argument(
        "--kp-values", nargs="+", type=float,
        default=[0.5, 1.0, 2.0, 4.0, 8.0],
        help="Kp_att values to sweep (parallel drone axis)",
    )
    p.add_argument(
        "--zeta-pd", type=float, default=1.0,
        help="Target damping ratio for outer PD loop (used to derive Kd_att)",
    )
    p.add_argument(
        "--kd-fixed", type=float, default=None,
        help="Override Kd_att with a fixed value instead of deriving from --zeta-pd",
    )
    p.add_argument("--duration",        type=float, default=15.0)
    p.add_argument("--spacing",         type=float, default=4.0)
    p.add_argument(
        "--bounds", nargs=6, type=float,
        default=[-12., 12., -12., 12., 0., 20.],
        metavar=("XMIN", "XMAX", "YMIN", "YMAX", "ZMIN", "ZMAX"),
    )
    p.add_argument("--target-altitude", type=float, default=1.5)
    p.add_argument(
        "--goal-mode", type=str,
        choices=["spawn_xy", "random_xy_2m"], default="spawn_xy",
    )
    p.add_argument("--goal-radius",     type=float, default=2.0)
    p.add_argument("--goal-seed",       type=int,   default=7)
    p.add_argument("--headless",        action="store_true", default=False)
    p.add_argument("--no-headless",     dest="headless", action="store_false")
    return p.parse_args()


ARGS = parse_args()
# Isaac imports are deferred to here so --help works without a simulator:
# argparse runs above, and booting Isaac to read a flag list is a bad trade.
import carb                            # noqa: E402
from isaacsim import SimulationApp     # noqa: E402

simulation_app = SimulationApp({"headless": ARGS.headless})


# ─────────────────────────── Gain helpers ────────────────────────────────────

def mdyn_params_for(omega_n: float, zeta: float) -> tuple[float, float]:
    """Return (wn_mdyn, b_mdyn) from (ωn, ζ) eigenvalue placement."""
    wn_mdyn = zeta * omega_n
    b_mdyn  = omega_n * math.sqrt(max(1.0 - zeta ** 2, 0.0))
    return wn_mdyn, b_mdyn


def kd_for_kp(kp: float, wn_mdyn: float, zeta_pd: float) -> float:
    """
    Derive Kd_att to achieve zeta_pd for the outer attitude PD loop.

    Linearised per-axis closed-loop:
        s² + [wn_mdyn·(1 + Kd) - α]·s + wn_mdyn·Kp = 0
        ωn_cl  = √(wn_mdyn · Kp)
        ζ_cl   = [wn_mdyn·(1 + Kd) - α] / (2·ωn_cl)

    Solving for Kd:
        Kd = (2·ζ_pd·ωn_cl + α) / wn_mdyn - 1
    """
    _ALPHA  = 0.5
    omega_n_cl = math.sqrt(max(wn_mdyn * kp, 1e-8))
    kd = (2.0 * zeta_pd * omega_n_cl + _ALPHA) / wn_mdyn - 1.0
    return max(kd, 0.01)


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


# ─────────────────────────── Late imports ────────────────────────────────────

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
        if os.path.isfile(self.results_file):
            try:
                data = dict(np.load(self.results_file))
                data["oob_time"] = np.array(self.oob_time)
                np.savez(self.results_file, **data)
            except Exception as exc:
                carb.log_warn(f"[kp_sweep] oob_time append failed: {exc}")

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
            carb.log_warn(f"[kp_sweep] OOB at {self.position} t={self.total_time:.2f}s")
            return True
        return False


# ─────────────────────────── Score ───────────────────────────────────────────

def stability_score(npz_path: str) -> float:
    try:
        data   = np.load(npz_path)
        ep     = data["ep"]
        t      = data["time"]
        if len(t) < 2:
            return np.inf
        T = float(t[-1])
        if T < 1e-6:
            return np.inf
        errors = np.linalg.norm(ep, axis=1)
        w      = t / T
        w_sum  = float(np.sum(w))
        if w_sum < 1e-12 or not np.all(np.isfinite(errors)):
            return np.inf
        score = float(np.dot(w, errors) / w_sum)
        return score if np.isfinite(score) else np.inf
    except Exception as exc:
        carb.log_warn(f"[kp_sweep] score read failed ({npz_path}): {exc}")
        return np.inf


# ─────────────────────────── Kp sweep plot ───────────────────────────────────

def save_kp_plot(kp_vals, kd_vals, scores, mm, wn_mdyn, b_mdyn,
                 omega_n, zeta, out_dir: Path):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, ax = plt.subplots(figsize=(9, 5))

        fin_mask = np.isfinite(scores)
        colors   = ["gold" if (fin_mask[i] and scores[i] == np.min(scores[fin_mask]))
                    else ("dimgray" if not fin_mask[i] else "steelblue")
                    for i in range(len(kp_vals))]

        ax.bar(range(len(kp_vals)), np.where(fin_mask, scores, 0.0),
               color=colors, edgecolor="black", linewidth=0.7)

        for i, (kp, kd, s) in enumerate(zip(kp_vals, kd_vals, scores)):
            if np.isfinite(s):
                ax.text(i, s + max(scores[fin_mask]) * 0.01 if fin_mask.any() else 0.02,
                        f"{s:.3f}", ha="center", va="bottom", fontsize=7)
            else:
                ax.text(i, 0.02, "∞\n(OOB)", ha="center", va="bottom",
                        fontsize=8, color="white", fontweight="bold")

        ax.set_xticks(range(len(kp_vals)))
        ax.set_xticklabels(
            [f"Kp={kp:.2f}\nKd={kd:.2f}" for kp, kd in zip(kp_vals, kd_vals)],
            fontsize=8,
        )
        ax.set_xlabel("Kp_att  (Kd derived from ζ_pd)", fontsize=11)
        ax.set_ylabel("Stability score  [future-biased mean ‖ep‖  m]", fontsize=11)
        ax.set_title(
            f"Kp_att Sensitivity Sweep   |   km={mm:.2e}\n"
            f"M_dyn anchor: ωn={omega_n:.1f}  ζ={zeta:.2f}  "
            f"→  wn_mdyn={wn_mdyn:.3f}  b_mdyn={b_mdyn:.3f}",
            fontsize=11,
        )
        ax.set_ylim(bottom=0)

        if fin_mask.any():
            best_i = int(np.argmin(np.where(fin_mask, scores, np.inf)))
            ax.bar(best_i, scores[best_i], color="gold", edgecolor="black",
                   linewidth=1.2, zorder=5,
                   label=f"★ Best: Kp={kp_vals[best_i]:.2f}  Kd={kd_vals[best_i]:.2f}"
                         f"  score={scores[best_i]:.4f}")
            ax.legend(fontsize=9)

        fig.tight_layout()
        path = out_dir / "kp_sweep_plot.png"
        fig.savefig(str(path), dpi=150, bbox_inches="tight")
        import matplotlib.pyplot as _plt
        _plt.close(fig)
        carb.log_warn(f"[kp_sweep] plot → {path}")
    except Exception as exc:
        carb.log_warn(f"[kp_sweep] plot failed: {exc}")


def save_kp_csv(kp_vals, kd_vals, scores, out_dir: Path):
    import csv
    path = out_dir / "kp_scores.csv"
    with open(str(path), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["Kp_att", "Kd_att", "score"])
        for kp, kd, s in zip(kp_vals, kd_vals, scores):
            w.writerow([f"{kp:.4f}", f"{kd:.4f}",
                        f"{s:.6f}" if np.isfinite(s) else "inf"])
    carb.log_warn(f"[kp_sweep] CSV → {path}")


# ─────────────────────────── Helpers ─────────────────────────────────────────

def make_thrust_curve(mm: float) -> QuadraticThrustCurve:
    return QuadraticThrustCurve(config={"motor_moment": mm})


def results_dir(base: Path, mm: float) -> Path:
    d = base / "eigen_kp_sweep" / f"mm_{mm:.1e}"
    d.mkdir(parents=True, exist_ok=True)
    return d


# ─────────────────────────── Main ────────────────────────────────────────────

def main():
    timeline    = omni.timeline.get_timeline_interface()
    render_sim  = not ARGS.headless
    bounds      = ARGS.bounds
    kp_vals     = ARGS.kp_values
    mm_vals     = ARGS.motor_moments
    omega_n     = ARGS.omega_n
    zeta        = ARGS.zeta
    n_kp        = len(kp_vals)
    base_dir    = Path(__file__).resolve().parent.parent
    results_base = base_dir / "results"

    wn_mdyn, b_mdyn = mdyn_params_for(omega_n, zeta)

    # Derive Kd for each Kp (or use fixed override)
    kd_vals = [
        ARGS.kd_fixed if ARGS.kd_fixed is not None
        else kd_for_kp(kp, wn_mdyn, ARGS.zeta_pd)
        for kp in kp_vals
    ]

    carb.log_warn(f"[kp_sweep] M_dyn anchor:  ωn={omega_n:.1f}  ζ={zeta:.2f}")
    carb.log_warn(f"[kp_sweep]               wn_mdyn={wn_mdyn:.3f}  b_mdyn={b_mdyn:.3f}")
    carb.log_warn(f"[kp_sweep] Kp values : {kp_vals}")
    carb.log_warn(f"[kp_sweep] Kd values : {[f'{k:.3f}' for k in kd_vals]}")
    carb.log_warn(f"[kp_sweep] km values : {mm_vals}")
    carb.log_warn(f"[kp_sweep] duration  : {ARGS.duration}s per mm pass")

    # ── World ────────────────────────────────────────────────────────────────

    pg       = PegasusInterface()
    pg._world = World(**pg._world_settings)
    world    = pg.world
    pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])

    for _ in range(20):
        world.step(render=render_sim)

    # ── Spacing ──────────────────────────────────────────────────────────────

    x_min, x_max = bounds[0], bounds[1]
    x_margin     = 1.5
    auto_spacing = (x_max - x_min - 2 * x_margin) / (n_kp - 1) if n_kp > 1 else 0.0
    spacing      = min(ARGS.spacing, auto_spacing)
    x_start      = x_min + x_margin
    half         = spacing / 2.0
    y_min, y_max = bounds[2], bounds[3]
    z_min, z_max = bounds[4], bounds[5]

    for i in range(n_kp):
        sx = x_start + i * spacing
        if not (x_min < sx < x_max):
            raise ValueError(
                f"Drone {i} spawn X={sx:.2f} outside X bounds [{x_min},{x_max}]."
            )

    spawn_y  = -1.5
    spawn_z  = 0.07
    rng      = np.random.default_rng(ARGS.goal_seed)
    tmp_files = [str(results_base / f"_kp_sweep_tmp_{i}.npz") for i in range(n_kp)]

    # ── Spawn drones (one per Kp_att value) ─────────────────────────────────

    drones: list = []

    for i_kp, kp in enumerate(kp_vals):
        kd        = kd_vals[i_kp]
        spawn_x   = x_start + i_kp * spacing
        spawn_pos = [spawn_x, spawn_y, spawn_z]
        target    = build_target_position(spawn_pos, rng)

        drone_bounds = [
            spawn_x - half, spawn_x + half,
            y_min, y_max, z_min, z_max,
        ]

        inner = EigenController(
            target_position=target,
            results_file=tmp_files[i_kp],
            wn_mdyn=wn_mdyn,
            b_mdyn=b_mdyn,
            Kp_att=kp,
            Kd_att=kd,
            km=mm_vals[0],
        )
        backend = BoundedTelemetryBackend(inner, tmp_files[i_kp])

        cfg              = MultirotorConfig()
        cfg.backends     = [backend]
        cfg.thrust_curve = make_thrust_curve(mm_vals[0])

        vehicle = Multirotor(
            f"/World/eigen_kp{i_kp}",
            ROBOTS["Iris"],
            i_kp,
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
        carb.log_warn(f"\n{'='*60}")
        carb.log_warn(f"[kp_sweep] motor_moment = {mm:.2e}")
        carb.log_warn(f"{'='*60}")

        rdir   = results_dir(results_base, mm)
        scores = np.full(n_kp, np.inf)

        for i_kp, (vehicle, backend, spawn_pos, drone_bounds) in enumerate(drones):
            kp = kp_vals[i_kp]
            kd = kd_vals[i_kp]

            vehicle._thrusters     = make_thrust_curve(mm)
            backend.inner.Kp_att   = kp
            backend.inner.Kd_att   = kd
            backend.inner.wn_mdyn  = wn_mdyn
            backend.inner.b_mdyn   = b_mdyn
            backend.inner.km       = mm

            vehicle.set_world_pose(position=spawn_pos, orientation=[0., 0., 0., 1.])
            vehicle.set_linear_velocity([0., 0., 0.])
            vehicle.set_angular_velocity([0., 0., 0.])
            backend.reset()
            backend.start()
            carb.log_warn(f"  Kp={kp:.2f}  Kd={kd:.3f}  drone {i_kp} ready")

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
                carb.log_warn("  all drones OOB — ending early")
                break

        timeline.stop()

        for i_kp, (_, backend, _, _) in enumerate(drones):
            if not backend._stopped:
                backend.stop()

            kp  = kp_vals[i_kp]
            src = tmp_files[i_kp]
            dst = rdir / f"kp{kp:.2f}_statistics.npz"

            score = stability_score(src)
            scores[i_kp] = score
            carb.log_warn(
                f"  Kp={kp:.2f}  Kd={kd_vals[i_kp]:.3f}  →  score = "
                f"{'∞' if not np.isfinite(score) else f'{score:.4f}'}"
            )

            if os.path.isfile(src):
                try:
                    os.rename(src, str(dst))
                except Exception as exc:
                    carb.log_warn(f"  move failed: {exc}")

        save_kp_plot(kp_vals, kd_vals, scores, mm, wn_mdyn, b_mdyn,
                     omega_n, zeta, rdir)
        save_kp_csv(kp_vals, kd_vals, scores, rdir)

        fin_idx = [i for i in range(n_kp) if np.isfinite(scores[i])]
        if fin_idx:
            best_i = min(fin_idx, key=lambda i: scores[i])
            carb.log_warn(
                f"\n  ★ Best: Kp={kp_vals[best_i]:.2f}  Kd={kd_vals[best_i]:.3f}"
                f"  →  score={scores[best_i]:.4f}"
            )
        else:
            carb.log_warn(f"\n  ✗ All OOB for mm={mm:.1e}")

    # ── Teardown ─────────────────────────────────────────────────────────────

    carb.log_warn("\n=== Kp_att sweep complete ===")

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
        for fn in ("stop", "clear_all_callbacks", "clear"):
            try:
                getattr(world, fn)()
            except Exception:
                pass

    simulation_app.close()


if __name__ == "__main__":
    main()
