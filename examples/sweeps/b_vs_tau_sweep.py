#!/usr/bin/env python
"""
b_vs_tau_sweep.py — how the ideal M_dyn cross-coupling b tracks rotor/ESC lag.

Why this exists
───────────────
FINDINGS §6c measured that `rotor_tau` decides whether the eigen controller
holds ~30 rad/s at all: at the 20 ms placeholder it departs at t≈12 s, at 0 it
flies indefinitely. `rotor_tau` is not a measured number for this vehicle - its
own comment calls it "the midpoint of the 10-30 ms range ... NOT a measured
value" - so the controller is currently tuned against a guess.

`b_mdyn` is the M_dyn off-diagonal term, the one that cross-couples roll and
pitch, and gene_space.py records the conjecture that it tracks the spin rate.
Actuator lag acts on the same axis: it delays exactly the differential thrust
that b commands. So the question this answers is whether the b that flies best
moves with tau, and how steeply.

What it does
────────────
For each rotor_tau, flies every b in parallel (one drone per b), scores each
run with sweep_metrics, and takes the best-scoring b as that tau's optimum.
Serial over tau, parallel over b, because the thrust curve is per-vehicle and
so tau has to be uniform within a batch.

Outputs, under results/b_vs_tau/mm_<km>/:
    b_vs_tau.csv        every (tau, b) run: score, effort, stability, spin
    ideal_b_vs_tau.png  the headline - best b against tau, with the score
                        surface behind it so a flat optimum is visible as flat
    score_heatmap.png   score over the full (tau, b) grid

Usage
─────
  isaac_run examples/sweeps/b_vs_tau_sweep.py --headless
  isaac_run examples/sweeps/b_vs_tau_sweep.py --headless \
      --tau-ms 0 5 10 15 20 25 30 --b-values 0 4 8 12 16 20 24 28 \
      --motor-moment 6e-7 --duration 30
"""
from __future__ import annotations

import argparse
import csv
import os
import sys
from pathlib import Path


def parse_args():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--tau-ms", type=float, nargs="+",
                   default=[0.0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0],
                   help="Rotor/ESC lag values to test [ms]. The default spans the "
                        "10-30 ms hardware range FINDINGS section 6 cites, plus 0 "
                        "for the instant-response plant older archives were flown on.")
    p.add_argument("--b-values", type=float, nargs="+",
                   default=[0.0, 4.0, 8.0, 12.0, 16.0, 20.0, 24.0, 28.0],
                   help="b_mdyn values to try at each tau. gene_space.py bounds b at "
                        "[0, 40]; the default covers the lower two thirds, where every "
                        "archived front sits.")
    p.add_argument("--motor-moment", type=float, default=6.0e-7,
                   help="km, which sets the steady spin (r_ss ~ 4.9e7 * km). Default "
                        "6e-7 is ~30 rad/s, the spin the tau effect was measured at.")
    p.add_argument("--wn-mdyn", type=float, default=None,
                   help="Hold wn_mdyn fixed at this value. Default: the "
                        "EigenController's own, so only b is varied.")
    p.add_argument("--duration", type=float, default=30.0,
                   help="Seconds per run. Keep >=25: the tau departure is late, "
                        "arriving only as the spin finishes ramping (FINDINGS 6c).")
    p.add_argument("--target-altitude", type=float, default=1.5)
    p.add_argument("--spawn-altitude", type=float, default=None)
    p.add_argument("--goal-radius", type=float, default=None)
    p.add_argument("--goal-seed", type=int, default=7,
                   help="Seeds the goal direction. Fixed across the whole sweep so "
                        "every (tau, b) cell flies the same task.")
    p.add_argument("--spacing", type=float, default=3.5)
    p.add_argument("--bounds", type=float, nargs=6,
                   default=[-25., 25., -25., 25., 0., 30.])
    p.add_argument("--run-tag", type=str, default=None,
                   help="Subdirectory for this run's output, so a sweep does not "
                        "overwrite the previous one.")
    p.add_argument("--headless", action="store_true", default=False)
    p.add_argument("--no-headless", dest="headless", action="store_false")
    return p.parse_args()


ARGS = parse_args()

# Isaac imports are deferred to here so --help works without a simulator:
# argparse runs above, and booting Isaac to read a flag list is a bad trade.
import carb                            # noqa: E402
from isaacsim import SimulationApp     # noqa: E402

simulation_app = SimulationApp({
    "headless": ARGS.headless,
    "hide_ui": True if ARGS.headless else None,
    "disable_viewport_updates": True if ARGS.headless else False,
    "multi_gpu": False, "max_gpu_count": 1, "active_gpu": 0,
})

import numpy as np                                          # noqa: E402
import omni.timeline                                        # noqa: E402
from isaacsim.core.api import World                         # noqa: E402
from scipy.spatial.transform import Rotation                # noqa: E402

for _parent in Path(__file__).resolve().parents:
    _candidate = _parent / "extensions" / "pegasus.simulator"
    if _candidate.exists():
        if str(_candidate) not in sys.path:
            sys.path.insert(0, str(_candidate))
        break

from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS   # noqa: E402
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface  # noqa: E402
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig  # noqa: E402
from pegasus.simulator.logic.thrusters import QuadraticThrustCurve     # noqa: E402
from pegasus.simulator.logic.backends import Backend                   # noqa: E402
from pegasus.simulator.logic.state import State                        # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "utils"))
from eigen_controller import EigenController   # noqa: E402
import sweep_metrics as sm                     # noqa: E402
import sim_clock                               # noqa: E402
import sweep_task as task                      # noqa: E402

ARGS.spawn_altitude = task.resolve_spawn_altitude(ARGS.spawn_altitude)
if ARGS.goal_radius is None:
    ARGS.goal_radius = task.DEFAULT_GOAL_RADIUS

# ── OOB-aware telemetry backend (same gating as the other sweeps) ─────────────
class BoundedBackend(Backend):
    def __init__(self, inner, results_file: str):
        super().__init__(config=None)
        self.inner = inner
        self.results_file = results_file
        self.total_time = 0.0
        self._stopped = False
        self.position = np.zeros(3)
        self.monitor = task.FlightMonitor(
            ARGS.target_altitude, label="b-vs-tau", log=carb.log_warn)


    # is_oob/oob_time stay part of this class's surface — the sweep loops and
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

    def start(self):
        self.total_time = 0.0
        self._stopped = False
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
                np.savez(self.results_file, **data)
            except Exception:
                pass

    def reset(self): self.inner.reset()

    def update_sensor(self, st, d):
        if not self.is_oob: self.inner.update_sensor(st, d)

    def update_graphical_sensor(self, st, d):
        if not self.is_oob: self.inner.update_graphical_sensor(st, d)

    def update_state(self, state: State):
        self.position = state.position.copy()
        if not self.is_oob: self.inner.update_state(state)

    def input_reference(self):
        return [0., 0., 0., 0.] if self.is_oob else self.inner.input_reference()

    def update(self, dt: float):
        if self.is_oob: return
        self.total_time += dt
        self.inner.update(dt)

    def check_oob(self, bounds) -> bool:
        """Advance the flight-phase monitor. True once the run is over.

        Takeoff failures and crashes are distinguished in
        examples/utils/sweep_task.py, shared with every other sweep.
        """
        return self.monitor.update(self.total_time, self.position, bounds)


EFFORT_DROP_THRESHOLD: float = 100.0  # drop finite-effort samples above this (SES, N^2)

_BOUNDS_DIAG = sm.bounds_diagonal(ARGS.bounds)




def run_metrics(npz_path: str, T_total: float) -> dict:
    """Score one run with the shared metric set, so numbers match the archives."""
    return sm.compute_metrics(npz_path, T_total, _BOUNDS_DIAG)


def make_thrust_curve(mm: float, tau_s: float) -> QuadraticThrustCurve:
    """The plant. tau_s is the ESC/motor first-order lag; 0 is instant response."""
    return QuadraticThrustCurve(config={"motor_moment": mm, "rotor_tau": tau_s})


def _as_cost(value) -> float:
    """Non-finite scores mean the run failed; rank them as worst, not as best."""
    try:
        v = float(value)
    except (TypeError, ValueError):
        return float("inf")
    return v if np.isfinite(v) else float("inf")


def pick_ideal(rows_for_tau: list) -> dict:
    """The best b at one tau.

    Stability first, score second. A diverged run can post a deceptively low
    score by ending early - sweep_metrics pads with the arena diagonal, but a
    run that never left the ground has little trajectory to be charged for -
    so an unstable cell must never outrank a stable one.
    """
    stable = [r for r in rows_for_tau if r["stable"]]
    pool = stable if stable else rows_for_tau
    best = min(pool, key=lambda r: _as_cost(r["score"]))
    return dict(best, any_stable=bool(stable))


def save_csv(rows: list, out_dir: Path) -> Path:
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / "b_vs_tau.csv"
    cols = ["tau_ms", "b_mdyn", "wn_mdyn", "motor_moment", "score", "effort",
            "stable", "r_ss", "rms_error", "failure"]
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols, extrasaction="ignore")
        w.writeheader()
        for r in rows:
            w.writerow(r)
    return path


def save_plots(rows: list, out_dir: Path, mm: float) -> None:
    """Two relationships, because they are not equally strong.

    The score-optimal b is a shallow bowl - a few percent across the whole
    stable range - so on its own it reads as noise. What moves hard with tau is
    the largest b that stays stable at all, and that is the line to design
    against. Both are drawn; only the stable cells are shaded, so the bowl is
    visible instead of being flattened by diverged cells two orders of
    magnitude worse.
    """
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    taus = sorted({r["tau_ms"] for r in rows})
    bs = sorted({r["b_mdyn"] for r in rows})
    stable_grid = np.full((len(bs), len(taus)), np.nan)
    for r in rows:
        if r["stable"]:
            stable_grid[bs.index(r["b_mdyn"]), taus.index(r["tau_ms"])] = _as_cost(r["score"])

    best_b, ceiling, no_stable = [], [], []
    for tau in taus:
        at_tau = [r for r in rows if r["tau_ms"] == tau]
        stable = [r for r in at_tau if r["stable"]]
        best = pick_ideal(at_tau)
        best_b.append(best["b_mdyn"] if stable else np.nan)
        ceiling.append(max(r["b_mdyn"] for r in stable) if stable else np.nan)
        no_stable.append(not stable)

    fig, ax = plt.subplots(figsize=(9, 5.8))
    finite = stable_grid[np.isfinite(stable_grid)]
    if finite.size:
        mesh = ax.pcolormesh(taus, bs, stable_grid, shading="nearest",
                             cmap="viridis_r", vmin=finite.min(), vmax=finite.max())
        cb = fig.colorbar(mesh, ax=ax)
        cb.set_label("score among STABLE runs (lower is better)")
    # Diverged cells are left blank and hatched: they are not "a worse score",
    # they are a different outcome, and colouring them swamps the scale.
    for j, tau in enumerate(taus):
        for i, b in enumerate(bs):
            if not np.isfinite(stable_grid[i, j]):
                ax.add_patch(plt.Rectangle(
                    (tau - 1.25, b - 2), 2.5, 4, hatch="xx", fill=False,
                    edgecolor="0.55", lw=0.0))

    ax.plot(taus, ceiling, "s--", color="darkorange", lw=2.4, ms=9,
            label="largest STABLE b (the binding limit)", zorder=6)
    ax.plot(taus, best_b, "o-", color="crimson", lw=2.2, ms=8,
            label="best-scoring b (shallow optimum)", zorder=7)
    for tau, none in zip(taus, no_stable):
        if none:
            ax.axvspan(tau - 1.25, tau + 1.25, color="crimson", alpha=0.10, zorder=0)
    if any(no_stable):
        ax.plot([], [], "s", color="crimson", alpha=0.3, ms=12,
                label="no b is stable at this tau")

    ax.set_xlabel("rotor_tau [ms]")
    ax.set_ylabel("b_mdyn")
    ax.set_title(f"M_dyn cross-coupling b vs rotor/ESC lag\n"
                 f"km = {mm:.1e}  (r_ss ≈ {4.9e7 * mm:.0f} rad/s), "
                 f"{ARGS.duration:g} s per run — hatched = diverged")
    ax.legend(loc="upper right", fontsize=9)
    ax.grid(alpha=0.3, ls=":")
    fig.tight_layout()
    fig.savefig(out_dir / "ideal_b_vs_tau.png", dpi=150)
    plt.close(fig)

    # Full grid including failures, on a log scale, for the whole shape.
    fig, ax = plt.subplots(figsize=(9, 5.8))
    grid = np.full((len(bs), len(taus)), np.nan)
    for r in rows:
        grid[bs.index(r["b_mdyn"]), taus.index(r["tau_ms"])] = _as_cost(r["score"])
    with np.errstate(invalid="ignore", divide="ignore"):
        shown = np.log10(np.where(np.isfinite(grid), grid, np.nan))
    mesh = ax.pcolormesh(taus, bs, shown, shading="nearest", cmap="magma_r")
    fig.colorbar(mesh, ax=ax, label="log10(score) — white cells never finished")
    ax.plot(taus, ceiling, "s--", color="cyan", lw=2, ms=8, label="largest stable b")
    ax.set_xlabel("rotor_tau [ms]"); ax.set_ylabel("b_mdyn")
    ax.set_title("Score over the full (tau, b) grid")
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(out_dir / "score_heatmap.png", dpi=150)
    plt.close(fig)


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    timeline = omni.timeline.get_timeline_interface()
    render_sim = not ARGS.headless
    bounds = ARGS.bounds
    mm = ARGS.motor_moment
    b_values = list(ARGS.b_values)
    tau_list = list(ARGS.tau_ms)
    pop_size = len(b_values)

    base_dir = Path(__file__).resolve().parent.parent
    results_base = task.results_root(base_dir)
    out_dir = results_base / "b_vs_tau" / f"mm_{mm:.1e}"
    if ARGS.run_tag:
        out_dir = out_dir / ARGS.run_tag

    carb.log_warn("\n" + "=" * 60)
    carb.log_warn("[b-vs-tau] ideal M_dyn b against rotor/ESC lag")
    carb.log_warn(f"[b-vs-tau] tau [ms]  : {tau_list}")
    carb.log_warn(f"[b-vs-tau] b values  : {b_values}  ({pop_size} drones in parallel)")
    carb.log_warn(f"[b-vs-tau] km        : {mm:.2e}  -> r_ss ~ {4.9e7 * mm:.1f} rad/s")
    carb.log_warn(f"[b-vs-tau] duration  : {ARGS.duration}s per run, "
                  f"{len(tau_list)} batches")
    carb.log_warn(f"[b-vs-tau] output    : {out_dir}")
    carb.log_warn("=" * 60)

    pg = PegasusInterface()
    pg._world = World(**pg._world_settings)
    world = pg.world
    pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])
    for _ in range(20):
        world.step(render=render_sim)

    # One drone per b, in a row along X, each inside its own lateral slice.
    x_min, x_max = bounds[0], bounds[1]
    x_margin = 1.5
    x_usable = x_max - x_min - 2.0 * x_margin
    auto_spacing = x_usable / (pop_size - 1) if pop_size > 1 else 0.0
    spacing = min(ARGS.spacing, auto_spacing) if pop_size > 1 else ARGS.spacing
    x_start = x_min + x_margin
    spawn_y, spawn_z = -1.5, float(ARGS.spawn_altitude)
    half = spacing / 2.0
    y_min, y_max, z_min, z_max = bounds[2], bounds[3], bounds[4], bounds[5]

    # One goal draw, reused for every tau: the task must not change underneath
    # the comparison, or a difference between taus could be a difference in
    # where the drones were asked to fly.
    goal_rng = np.random.default_rng(ARGS.goal_seed)
    max_dx = task.slice_max_dx(spacing)
    task.check_goal_reachable(spawn_y, ARGS.goal_radius, bounds, label="b-vs-tau")

    tmp_files = [str(results_base / f"_btau_tmp_{i}.npz") for i in range(pop_size)]

    drones = []
    for i in range(pop_size):
        spawn_x = x_start + i * spacing
        spawn_pos = [spawn_x, spawn_y, spawn_z]
        target = task.offset_target(spawn_pos, ARGS.target_altitude,
                                    ARGS.goal_radius, goal_rng, max_dx=max_dx)
        drone_bounds = [spawn_x - half, spawn_x + half, y_min, y_max, z_min, z_max]
        kwargs = dict(target_position=target, results_file=tmp_files[i],
                      b_mdyn=b_values[i], km=mm)
        if ARGS.wn_mdyn is not None:
            kwargs["wn_mdyn"] = float(ARGS.wn_mdyn)
        inner = EigenController(**kwargs)
        backend = BoundedBackend(inner, tmp_files[i])
        cfg = MultirotorConfig()
        cfg.backends = [backend]
        cfg.thrust_curve = make_thrust_curve(mm, 1e-3 * tau_list[0])
        vehicle = Multirotor(
            f"/World/btau_{i}", ROBOTS["Iris"], i, spawn_pos,
            Rotation.from_euler("XYZ", [0., 0., 0.], degrees=True).as_quat(),
            config=cfg)
        drones.append((vehicle, backend, spawn_pos, drone_bounds))

    world.reset()
    for _ in range(10):
        world.step(render=render_sim)

    def run_batch(tau_ms: float) -> list:
        """Fly every b once at this tau. Returns one metrics dict per b."""
        tau_s = 1e-3 * tau_ms
        for slot in range(pop_size):
            vehicle, backend, spawn_pos, _ = drones[slot]
            # The plant is per-vehicle, which is why tau is uniform in a batch.
            vehicle._thrusters = make_thrust_curve(mm, tau_s)
            backend.inner.b_mdyn = float(b_values[slot])
            backend.inner.km = mm
            vehicle.set_world_pose(position=spawn_pos, orientation=[0., 0., 0., 1.])
            vehicle.set_linear_velocity([0., 0., 0.])
            vehicle.set_angular_velocity([0., 0., 0.])
            backend.reset()
            backend.start()

        for _ in range(5):
            world.step(render=render_sim)

        timeline.play()
        n_steps, _ = sim_clock.steps_for_duration(world, ARGS.duration)
        for _ in range(n_steps):
            if not simulation_app.is_running():
                break
            world.step(render=render_sim)
            all_oob = True
            for _, backend, _, db in drones:
                if not backend.is_oob:
                    all_oob = False
                    backend.check_oob(db)
            if all_oob:
                break
        timeline.stop()

        out = []
        for slot in range(pop_size):
            _, backend, _, _ = drones[slot]
            if not backend._stopped:
                backend.stop()
            out.append(run_metrics(tmp_files[slot], ARGS.duration))
            try:
                os.remove(tmp_files[slot])
            except OSError:
                pass
        return out

    rows = []
    for tau_ms in tau_list:
        carb.log_warn(f"\n[b-vs-tau] tau = {tau_ms:g} ms  ({pop_size} b values)")
        metrics = run_batch(tau_ms)
        for b, m in zip(b_values, metrics):
            row = {
                "tau_ms": float(tau_ms), "b_mdyn": float(b),
                "wn_mdyn": (float(ARGS.wn_mdyn) if ARGS.wn_mdyn is not None
                            else float(drones[0][1].inner.wn_mdyn)),
                "motor_moment": mm,
                "score": m.get("score"), "effort": m.get("effort"),
                "stable": bool(m.get("stable", False)), "r_ss": m.get("r_ss"),
                "rms_error": m.get("rms_error"), "failure": m.get("failure", ""),
            }
            rows.append(row)
            carb.log_warn(f"    b={b:6.2f}  score={_as_cost(row['score']):9.4f}  "
                          f"stable={row['stable']}  r_ss={row['r_ss']}")
        best = pick_ideal([r for r in rows if r["tau_ms"] == tau_ms])
        carb.log_warn(f"  -> best b at tau={tau_ms:g} ms: {best['b_mdyn']:g}"
                      + ("" if best["any_stable"] else "   (NO stable b at this tau)"))

    csv_path = save_csv(rows, out_dir)
    carb.log_warn(f"\n[b-vs-tau] csv  -> {csv_path}")
    try:
        save_plots(rows, out_dir, mm)
        carb.log_warn(f"[b-vs-tau] plot -> {out_dir / 'ideal_b_vs_tau.png'}")
        carb.log_warn(f"[b-vs-tau] plot -> {out_dir / 'score_heatmap.png'}")
    except Exception as exc:
        carb.log_warn(f"[b-vs-tau] plotting failed ({exc}); the csv above still holds "
                      f"every run, so nothing is lost.")

    carb.log_warn("\n[b-vs-tau] ideal b by tau:")
    for tau_ms in tau_list:
        best = pick_ideal([r for r in rows if r["tau_ms"] == tau_ms])
        carb.log_warn(f"    tau={tau_ms:6.1f} ms  ->  b={best['b_mdyn']:6.2f}  "
                      f"score={_as_cost(best['score']):9.4f}"
                      + ("" if best["any_stable"] else "   (none stable)"))

    timeline.stop()
    simulation_app.close()


if __name__ == "__main__":
    main()
