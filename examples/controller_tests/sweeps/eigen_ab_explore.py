#!/usr/bin/env python3
"""
eigen_ab_explore.py — space-filling exploration of the Eigen M_dyn (a, b) plane.

Purpose
───────
This is a DELIBERATELY NON-CONVERGENT exploration tool, the opposite of
genetic_sweep.py. Where the genetic search drives the population toward the
Pareto front (so its samples cluster in the "good" region and tell you little
about the rest of the space), this script scatters samples across the whole
(a, b) rectangle to map how the raw landscape of stability score and control
effort actually varies with:

    a = wn_mdyn   (real part of the M_dyn attitude eigenvalue)
    b = b_mdyn    (imaginary part of the M_dyn attitude eigenvalue)

Everything else is held at the EigenController's OWN DEFAULT gains — the outer
position loop (kp_pos, kd_pos, kp_z, kd_z) and the attitude loop (Kp_att,
Kd_att) are NOT overridden here. That isolates the a/b relationship at the
controller's shipped operating point, rather than at some sweep-specific
baseline. (Contrast eigen_stability_sweep.py, which pins --kp-att/--kd-att.)

This is intentionally standalone and does NOT feed interactive_dashboard.py.
It writes its own CSV plus a set of insight plots. The CSV happens to use the
same column layout as the other sweeps' grid_archive.csv, so you *can* point
the dashboard at it later if you want — but that is not its job.

Scoring is byte-for-byte the same SES metric (stability_score /
control_effort_score) used by eigen_stability_sweep.py, so numbers are on the
same scale as every other sweep.

Usage
─────
  isaac_run examples/controller_tests/sweeps/eigen_ab_explore.py \
      --motor-moments 0.0 4e-7 \
      --samples 128 --sampling halton \
      --a-range 0.5 20 --b-range 0 15 \
      --pop-size 8 --headless

Outputs (per motor_moment, under results/eigen_explore/mm_<km>/):
    exploration_archive.csv   long-format (a, b, score, effort) rows
    ab_score_scatter.png      (a, b) plane coloured by stability score
    ab_effort_scatter.png     (a, b) plane coloured by control effort
    marginals.png             score-vs-a, score-vs-b, effort-vs-a, effort-vs-b
    ab_score_interactive.html   Plotly (a, b) scatter coloured by stability score
    ab_effort_interactive.html Plotly (a, b) scatter coloured by control effort

Samples with a finite control-effort score (SES) greater than EFFORT_DROP_THRESHOLD
(100) are dropped before anything is written or plotted — they are numerical
outliers (saturated/diverging runs) that otherwise blow out the colour scale
and axis ranges on every plot. OOB/non-finite (inf) samples are unaffected by
this threshold and still show up as the red "OOB / inf" markers.
"""
from __future__ import annotations
import argparse, csv, os, sys
from pathlib import Path
import carb
from isaacsim import SimulationApp


# ── CLI (before SimulationApp) ────────────────────────────────────────────────
def parse_args():
    p = argparse.ArgumentParser(description="Eigen (a, b) space-filling exploration")
    p.add_argument("--motor-moments", nargs="+", type=float,
                   default=[0.0, 4e-7],
                   help="km (motor moment) values — outer serial loop")
    p.add_argument("--samples", type=int, default=128,
                   help="Total number of (a, b) samples to scatter over the plane "
                        "(for --sampling grid this is rounded to the nearest square)")
    p.add_argument("--sampling", choices=["halton", "uniform", "grid"], default="halton",
                   help="halton = low-discrepancy space-filling (best coverage, default); "
                        "uniform = plain random; grid = regular n×n lattice")
    # Ranges match EIGEN_GENES in genetic_sweep.py. Both ceilings were raised
    # after the earlier box (a ≤ 20, b ≤ 15) proved too small: the GA pinned
    # wn_mdyn at exactly 20.000, and the logged spin rate reaches ~30 rad/s,
    # so a b_mdyn conjectured to track spin rate could not even be expressed.
    p.add_argument("--a-range", nargs=2, type=float, default=[0.5, 60.0],
                   metavar=("A_LO", "A_HI"),
                   help="wn_mdyn (a) sampling range — matches genetic EIGEN_GENES bounds")
    p.add_argument("--b-range", nargs=2, type=float, default=[0.0, 40.0],
                   metavar=("B_LO", "B_HI"),
                   help="b_mdyn (b) sampling range — matches genetic EIGEN_GENES bounds")
    p.add_argument("--pop-size", type=int, default=8,
                   help="Number of drones simulated in parallel per batch")
    p.add_argument("--duration", type=float, default=25.0,
                   help="Simulation seconds per run")
    p.add_argument("--target-altitude", type=float, default=1.5)
    p.add_argument("--spacing", type=float, default=4.0)
    p.add_argument("--bounds", nargs=6, type=float,
                   default=[-12., 12., -12., 12., 0., 20.],
                   metavar=("XMIN", "XMAX", "YMIN", "YMAX", "ZMIN", "ZMAX"))
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--spawn-altitude", type=float, default=None,
                   help="Altitude [m] to spawn at. Defaults to a ground start, so "
                        "every run includes the takeoff transient. Pass the target "
                        "altitude for the old air start.")
    p.add_argument("--goal-seed", type=int, default=7,
                   help="RNG seed for goal directions, kept separate from the "
                        "search RNG so the task stays fixed as the search changes")
    p.add_argument("--goal-radius", type=float, default=None,
                   help="Lateral distance [m] from spawn XY to the goal. Every run "
                        "is a takeoff followed by a translation of this distance in "
                        "a random direction; see examples/utils/sweep_task.py.")
    p.add_argument("--headless", action="store_true", default=False)
    p.add_argument("--no-headless", dest="headless", action="store_false")
    return p.parse_args()


ARGS = parse_args()

simulation_app = SimulationApp({
    "headless": ARGS.headless,
    "hide_ui": True if ARGS.headless else None,
    "disable_viewport_updates": True if ARGS.headless else False,
    "multi_gpu": False, "max_gpu_count": 1, "active_gpu": 0,
})

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
from pegasus.simulator.logic.thrusters import QuadraticThrustCurve
from pegasus.simulator.logic.backends import Backend
from pegasus.simulator.logic.state import State

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "utils"))
from eigen_controller import EigenController
import sweep_metrics as sm
import sim_clock
import sweep_task as task

# Resolved after the late imports so every code path below sees numbers. The
# task is a ground start followed by a translation to a laterally offset goal.
ARGS.spawn_altitude = task.resolve_spawn_altitude(ARGS.spawn_altitude)
if getattr(ARGS, "goal_radius", None) is None:
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
            ARGS.target_altitude, label="explore", log=carb.log_warn)


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
    """Every metric for one sample. See examples/utils/sweep_metrics.py."""
    return sm.compute_metrics(npz_path, T_total, _BOUNDS_DIAG)


# ── Sampling: space-filling, NON-convergent by construction ───────────────────
def sample_ab(n: int, a_range, b_range, mode: str, seed: int) -> np.ndarray:
    """Return an (M, 2) array of (a, b) points covering the rectangle.

    halton  : low-discrepancy quasi-random (best uniform coverage). No uniform
              fallback — if scipy.stats.qmc is missing this raises loudly rather
              than silently sampling with a different method.
    uniform : independent uniform random draws.
    grid    : deterministic n×n lattice (M = round(sqrt(n))² ≈ n).
    """
    a_lo, a_hi = a_range
    b_lo, b_hi = b_range
    rng = np.random.default_rng(seed)

    if mode == "grid":
        side = max(2, int(round(np.sqrt(n))))
        a_vals = np.linspace(a_lo, a_hi, side)
        b_vals = np.linspace(b_lo, b_hi, side)
        aa, bb = np.meshgrid(a_vals, b_vals)
        return np.column_stack([aa.ravel(), bb.ravel()])

    if mode == "halton":
        from scipy.stats import qmc
        unit = qmc.Halton(d=2, seed=seed).random(n)
    else:  # uniform
        unit = rng.random((n, 2))

    a = a_lo + unit[:, 0] * (a_hi - a_lo)
    b = b_lo + unit[:, 1] * (b_hi - b_lo)
    return np.column_stack([a, b])


# ── CSV + insight plots (standalone — no dashboard involvement) ───────────────
def save_csv(rows, mm, out_dir: Path):
    """rows: list of (a, b, score, effort, metrics). Long format, dashboard-
    compatible column names but written to its own file. Metric columns come
    from sweep_metrics.METRIC_KEYS so this archive matches the other sweeps'."""
    path = out_dir / "exploration_archive.csv"
    with open(str(path), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["gen", "wn_mdyn", "b_mdyn", "motor_moment"] + sm.METRIC_KEYS)
        for a, b, _s, _e, m in rows:
            w.writerow(
                [-1, f"{a:.6f}", f"{b:.6f}", mm]
                + [sm.format_metric(k, m[k]) for k in sm.METRIC_KEYS]
            )
    carb.log_warn(f"[explore] archive CSV → {path}")


def _scatter_ab(a, b, vals, title, clabel, path, cmap_name):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import matplotlib.colors as mc

    a = np.asarray(a); b = np.asarray(b); vals = np.asarray(vals, dtype=float)
    finite = np.isfinite(vals)

    fig, ax = plt.subplots(figsize=(8, 6.5))
    if finite.any():
        fv = vals[finite]
        vmin = max(float(np.min(fv)), 1e-5)
        vmax = float(np.max(fv))
        if vmax <= vmin: vmax = vmin * 1.01
        norm = mc.LogNorm(vmin=vmin, vmax=vmax)
        sc = ax.scatter(a[finite], b[finite], c=fv, cmap=cmap_name, norm=norm,
                        s=60, edgecolors="black", linewidths=0.4)
        plt.colorbar(sc, ax=ax, label=clabel)
        # Mark the best (lowest) sample with a gold star
        best = np.argmin(fv)
        ax.scatter(a[finite][best], b[finite][best], marker="*", s=320,
                   color="gold", edgecolors="black", linewidths=0.9, zorder=6,
                   label=f"best = {fv[best]:.3f}")
        ax.legend(fontsize=9, loc="upper right")
    # OOB / non-finite samples as hollow red X
    if (~finite).any():
        ax.scatter(a[~finite], b[~finite], marker="x", s=55, color="red",
                   linewidths=1.2, label="OOB / ∞")
    ax.set_xlabel("a = wn_mdyn  (real part of M_dyn eigenvalue)", fontsize=11)
    ax.set_ylabel("b = b_mdyn  (imag part of M_dyn eigenvalue)", fontsize=11)
    ax.set_title(title, fontsize=11)
    ax.grid(alpha=0.25)
    fig.tight_layout()
    fig.savefig(str(path), dpi=150, bbox_inches="tight")
    plt.close(fig)
    carb.log_warn(f"[explore] plot → {path}")


def save_plots(rows, mm, out_dir: Path):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        a = np.array([r[0] for r in rows])
        b = np.array([r[1] for r in rows])
        s = np.array([r[2] for r in rows], dtype=float)
        e = np.array([r[3] for r in rows], dtype=float)

        _scatter_ab(a, b, s,
                    f"Eigen (a, b) exploration — stability score  |  km={mm:.2e}\n"
                    f"(default gains; lower = more stable)",
                    "Stability score  [future-biased mean ‖ep‖  m]",
                    out_dir / "ab_score_scatter.png", "RdYlGn_r")
        _scatter_ab(a, b, e,
                    f"Eigen (a, b) exploration — control effort  |  km={mm:.2e}\n"
                    f"(default gains; lower = cheaper)",
                    "Control effort  [SES,  N²]",
                    out_dir / "ab_effort_scatter.png", "viridis")

        # ── Marginals: how each objective depends on a and on b separately ────
        fig, axes = plt.subplots(2, 2, figsize=(12, 9))
        sf = np.isfinite(s)
        ef = np.isfinite(e)
        panels = [
            (axes[0, 0], a[sf], s[sf], "a = wn_mdyn", "stability score", "steelblue"),
            (axes[0, 1], b[sf], s[sf], "b = b_mdyn", "stability score", "steelblue"),
            (axes[1, 0], a[ef], e[ef], "a = wn_mdyn", "control effort (SES)", "darkorange"),
            (axes[1, 1], b[ef], e[ef], "b = b_mdyn", "control effort (SES)", "darkorange"),
        ]
        for ax, xv, yv, xl, yl, col in panels:
            ax.scatter(xv, yv, s=28, color=col, alpha=0.7, edgecolors="black", linewidths=0.3)
            ax.set_xlabel(xl, fontsize=10)
            ax.set_ylabel(yl, fontsize=10)
            if len(yv) and np.nanmax(yv) / max(np.nanmin(yv), 1e-9) > 50:
                ax.set_yscale("log")
            ax.grid(alpha=0.25)
        fig.suptitle(f"Eigen (a, b) exploration — marginal relations  |  km={mm:.2e}",
                     fontsize=12)
        fig.tight_layout(rect=[0, 0, 1, 0.97])
        path = out_dir / "marginals.png"
        fig.savefig(str(path), dpi=150, bbox_inches="tight")
        plt.close(fig)
        carb.log_warn(f"[explore] plot → {path}")
    except Exception as exc:
        carb.log_warn(f"[explore] plotting failed: {exc}")


def _scatter_ab_interactive(a, b, vals, title, clabel, path, colorscale):
    import plotly.graph_objects as go

    a = np.asarray(a); b = np.asarray(b); vals = np.asarray(vals, dtype=float)
    finite = np.isfinite(vals)

    fig = go.Figure()
    if finite.any():
        fv = vals[finite]
        fa, fb = a[finite], b[finite]
        log_v = np.log10(np.clip(fv, 1e-5, None))
        fig.add_trace(go.Scatter(
            x=fa, y=fb, mode="markers",
            marker=dict(size=10, color=log_v, colorscale=colorscale,
                        colorbar=dict(title=f"log10({clabel})"),
                        line=dict(width=0.5, color="black")),
            text=[f"a={ai:.3f}  b={bi:.3f}<br>{clabel} = {vi:.4g}"
                  for ai, bi, vi in zip(fa, fb, fv)],
            hoverinfo="text", name="samples",
        ))
        best = int(np.argmin(fv))
        fig.add_trace(go.Scatter(
            x=[fa[best]], y=[fb[best]], mode="markers",
            marker=dict(size=18, symbol="star", color="gold",
                        line=dict(width=1.5, color="black")),
            text=[f"BEST  a={fa[best]:.3f}  b={fb[best]:.3f}  {clabel} = {fv[best]:.4g}"],
            hoverinfo="text", name="best",
        ))
    if (~finite).any():
        fig.add_trace(go.Scatter(
            x=a[~finite], y=b[~finite], mode="markers",
            marker=dict(size=9, symbol="x", color="red"),
            text=["OOB / inf"] * int((~finite).sum()),
            hoverinfo="text", name="OOB / inf",
        ))
    fig.update_layout(
        title=title,
        xaxis_title="a = wn_mdyn  (real part of M_dyn eigenvalue)",
        yaxis_title="b = b_mdyn  (imag part of M_dyn eigenvalue)",
        template="plotly_dark", width=900, height=650,
        margin=dict(l=60, r=40, t=60, b=60),
    )
    fig.write_html(str(path))
    carb.log_warn(f"[explore] interactive plot → {path}")


def save_interactive_plots(rows, mm, out_dir: Path):
    """Plotly HTML counterparts of the static ab_score/ab_effort scatter plots,
    with per-point hover (a, b, score/effort) and pan/zoom. Skipped (with a
    warning) if plotly isn't installed in the running interpreter."""
    try:
        import plotly  # noqa: F401
    except Exception as exc:
        carb.log_warn(f"[explore] plotly unavailable ({exc}); skipping interactive plots")
        return

    a = np.array([r[0] for r in rows])
    b = np.array([r[1] for r in rows])
    s = np.array([r[2] for r in rows], dtype=float)
    e = np.array([r[3] for r in rows], dtype=float)

    _scatter_ab_interactive(
        a, b, s,
        f"Eigen (a, b) exploration — stability score  |  km={mm:.2e} (interactive)",
        "stability score", out_dir / "ab_score_interactive.html", "RdYlGn_r",
    )
    _scatter_ab_interactive(
        a, b, e,
        f"Eigen (a, b) exploration — control effort  |  km={mm:.2e} (interactive)",
        "control effort (SES)", out_dir / "ab_effort_interactive.html", "Viridis",
    )


# ── Helpers ─────────────────────────────────────────────────────────────────
def make_thrust_curve(mm: float) -> QuadraticThrustCurve:
    return QuadraticThrustCurve(config={"motor_moment": mm})


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    timeline = omni.timeline.get_timeline_interface()
    render_sim = not ARGS.headless
    bounds = ARGS.bounds
    pop_size = ARGS.pop_size

    base_dir = Path(__file__).resolve().parent.parent
    results_base = task.results_root(base_dir)

    samples = sample_ab(ARGS.samples, ARGS.a_range, ARGS.b_range, ARGS.sampling, ARGS.seed)
    n_samples = len(samples)

    carb.log_warn("\n" + "=" * 60)
    carb.log_warn("[explore] Eigen M_dyn (a, b) space-filling exploration")
    carb.log_warn(f"[explore] sampling   : {ARGS.sampling}  ({n_samples} points)")
    carb.log_warn(f"[explore] a range     : {ARGS.a_range}   b range : {ARGS.b_range}")
    carb.log_warn(f"[explore] motor mm    : {ARGS.motor_moments}")
    carb.log_warn(f"[explore] gains       : EigenController DEFAULTS (not overridden)")
    carb.log_warn(f"[explore] pop-size    : {pop_size}   duration : {ARGS.duration}s")
    carb.log_warn("=" * 60)

    # ── World ────────────────────────────────────────────────────────────────
    pg = PegasusInterface()
    pg._world = World(**pg._world_settings)
    world = pg.world
    pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])
    for _ in range(20):
        world.step(render=render_sim)

    # ── X spacing (pop_size drones along X) ──────────────────────────────────
    x_min, x_max = bounds[0], bounds[1]
    x_margin = 1.5
    x_usable = x_max - x_min - 2.0 * x_margin
    auto_spacing = x_usable / (pop_size - 1) if pop_size > 1 else 0.0
    spacing = min(ARGS.spacing, auto_spacing)
    x_start = x_min + x_margin
    # Ground start: every run begins on the floor and has to take off. The air
    # start this replaced put the vehicle exactly on its target, so initial
    # position error was zero and the gains being searched could not affect the
    # score at all. See examples/utils/sweep_task.py.
    spawn_y, spawn_z = -1.5, float(ARGS.spawn_altitude)
    half = spacing / 2.0
    y_min, y_max = bounds[2], bounds[3]
    z_min, z_max = bounds[4], bounds[5]

    # Random goal direction per drone, bounded in X so a drone's target stays
    # inside its own slice of the arena and neighbours cannot be sent into each
    # other. Seeded, so the same --goal-seed reproduces the same task.
    goal_rng = np.random.default_rng(ARGS.goal_seed)
    max_dx   = task.slice_max_dx(spacing)
    task.check_goal_reachable(spawn_y, ARGS.goal_radius, bounds, label="explore")

    tmp_files = [str(results_base / f"_explore_tmp_{i}.npz") for i in range(pop_size)]
    mm_init = ARGS.motor_moments[0]
    a0 = float(np.mean(ARGS.a_range))
    b0 = float(np.mean(ARGS.b_range))

    # ── Spawn pop_size drones once; re-parameterised each batch ──────────────
    drones = []
    for i in range(pop_size):
        spawn_x = x_start + i * spacing
        spawn_pos = [spawn_x, spawn_y, spawn_z]
        target = task.offset_target(spawn_pos, ARGS.target_altitude,
                                    ARGS.goal_radius, goal_rng,
                                    max_dx=max_dx)
        drone_bounds = [spawn_x - half, spawn_x + half, y_min, y_max, z_min, z_max]

        # NOTE: Kp_att/Kd_att/kp_pos/... deliberately NOT passed → controller
        # defaults are used, per the exploration's purpose.
        inner = EigenController(
            target_position=target,
            results_file=tmp_files[i],
            wn_mdyn=a0, b_mdyn=b0, km=mm_init,
        )
        backend = BoundedBackend(inner, tmp_files[i])
        cfg = MultirotorConfig()
        cfg.backends = [backend]
        cfg.thrust_curve = make_thrust_curve(mm_init)

        vehicle = Multirotor(
            f"/World/explore_{i}", ROBOTS["Iris"], i, spawn_pos,
            Rotation.from_euler("XYZ", [0., 0., 0.], degrees=True).as_quat(),
            config=cfg,
        )
        drones.append((vehicle, backend, spawn_pos, drone_bounds))

    world.reset()
    for _ in range(10):
        world.step(render=render_sim)

    def run_batch(batch_ab, mm):
        """batch_ab: (k, 2) array, k <= pop_size. Returns list of (score, effort)."""
        k = len(batch_ab)
        for slot in range(pop_size):
            vehicle, backend, spawn_pos, _ = drones[slot]
            a, b = batch_ab[slot] if slot < k else batch_ab[0]  # filler past k; discarded
            vehicle._thrusters = make_thrust_curve(mm)
            backend.inner.wn_mdyn = float(a)
            backend.inner.b_mdyn = float(b)
            backend.inner.km = mm
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
            for _, backend, _, db in drones:
                if not backend.is_oob:
                    all_oob = False
                    backend.check_oob(db)
            if all_oob: break
        timeline.stop()

        out = []
        for slot in range(k):
            _, backend, _, _ = drones[slot]
            if not backend._stopped:
                backend.stop()
            out.append(run_metrics(tmp_files[slot], ARGS.duration))
            try: os.remove(tmp_files[slot])
            except: pass
        return out

    # ── Outer loop: motor moments ────────────────────────────────────────────
    for mm in ARGS.motor_moments:
        carb.log_warn(f"\n{'=' * 60}")
        carb.log_warn(f"[explore] mm={mm:.2e} | {n_samples} samples in "
                      f"{int(np.ceil(n_samples / pop_size))} batches")
        carb.log_warn(f"{'=' * 60}")

        out_dir = results_base / "eigen_explore" / f"mm_{mm:.1e}"
        out_dir.mkdir(parents=True, exist_ok=True)

        # (a, b, score, effort, metrics) — the first four are positional so the
        # plotting helpers keep working unchanged; the trailing dict carries
        # the full metric set through to the CSV.
        rows = []
        for start in range(0, n_samples, pop_size):
            batch = samples[start:start + pop_size]
            results = run_batch(batch, mm)
            for (a, b), m in zip(batch, results):
                s, e = m["score"], m["effort"]
                rows.append((float(a), float(b), s, e, m))
                s_str = "∞" if not np.isfinite(s) else f"{s:.4f}"
                e_str = "∞" if not np.isfinite(e) else f"{e:.4f}"
                ac_str = "∞" if not np.isfinite(m["effort_ac"]) else f"{m['effort_ac']:.4f}"
                r_str = "n/a" if not np.isfinite(m["r_ss"]) else f"{m['r_ss']:.2f}"
                carb.log_warn(
                    f"  a={a:6.2f} b={b:6.2f} → score={s_str:<10} "
                    f"effort={e_str:<10} effort_ac={ac_str:<10} r_ss={r_str}"
                )

        n_before = len(rows)
        rows = [r for r in rows if not (np.isfinite(r[3]) and r[3] > EFFORT_DROP_THRESHOLD)]
        n_dropped = n_before - len(rows)
        if n_dropped:
            carb.log_warn(f"[explore] dropped {n_dropped}/{n_before} samples with "
                          f"effort > {EFFORT_DROP_THRESHOLD:.0f} (SES)")

        save_csv(rows, mm, out_dir)
        save_plots(rows, mm, out_dir)
        save_interactive_plots(rows, mm, out_dir)

        fin = [r for r in rows if np.isfinite(r[2])]
        if fin:
            best = min(fin, key=lambda r: r[2])
            carb.log_warn(f"\n  ★ mm={mm:.1e} best score: a={best[0]:.2f} b={best[1]:.2f} "
                          f"→ score={best[2]:.4f} effort={best[3]:.4f}  "
                          f"({len(fin)}/{n_samples} finite)")
        else:
            carb.log_warn(f"\n  ✗ mm={mm:.1e}: all samples OOB")

    # ── Teardown ─────────────────────────────────────────────────────────────
    for f in tmp_files:
        try: os.remove(f)
        except: pass
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
