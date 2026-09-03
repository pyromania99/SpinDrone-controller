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
isaac_run examples/controller_tests/sweeps/eigen_stability_sweep.py \\
    --motor-moments 1e-7 2.7e-7 4e-7 \\
    --omega-n-values 2 4 6 8 10 12 \\
    --zeta-values 0.7 1.0 1.3 \\
    --duration 15

isaac_run examples/controller_tests/sweeps/eigen_stability_sweep.py \
    --a-values 2 4 6 8 10 \
    --b-values 0 2 4 6 8 10 \
    --motor-moments 1e-7 2.7e-7 4e-7 \
    --duration 25 \

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
        default=[1e-7 , 2.0e-7, 2.7e-7, 4e-7],
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
    # ── Direct (Kp_att, Kd_att) sweep at fixed M_dyn ─────────────────────────
    # Mirror of ab-sweep for the OTHER gene pair: sweeps the outer attitude
    # gains while wn_mdyn/b_mdyn are pinned (--fixed-wn / --fixed-b), so the
    # pure Kp_att/Kd_att effect can be read without M_dyn confounding it — the
    # causal complement to the ab-sweep's "wn,b at fixed Kp,Kd".
    p.add_argument(
        "--kp-values", nargs="+", type=float, default=None,
        help="Kp_att values to sweep — activates kpkd-sweep mode. "
             "Drones are spawned one-per-Kp (parallel). Example: --kp-values 2 3 4 5",
    )
    p.add_argument(
        "--kd-values", nargs="+", type=float, default=None,
        help="Kd_att values to sweep in kpkd-sweep mode — inner serial loop. "
             "Example: --kd-values 1 3 5 7 9 11",
    )
    p.add_argument(
        "--fixed-wn", type=float, default=18.0,
        help="wn_mdyn held fixed during kpkd-sweep (default: stable-basin optimum)",
    )
    p.add_argument(
        "--fixed-b", type=float, default=10.2,
        help="b_mdyn held fixed during kpkd-sweep (default: stable-basin optimum)",
    )
    # ────────────────────────────────────────────────────────────────────────
    # Pinned attitude gains. These defaults are a known-bad operating point:
    # mapping the (a, b) plane at Kp_att=0.7 left only 12 % of grid cells
    # flying, because the Eigen controller's searched optimum is Kp_att ≈ 4.
    # Prefer --seed-from, which reads the pinned values out of a prior eigen
    # archive so the operating point tracks what the searches have found.
    p.add_argument(
        "--kp-att", type=float, default=None,
        help="Fixed Kp_att outer attitude gain (used when Kp is not the swept "
             "axis). Overrides --seed-from. Default 0.7 if neither is given.",
    )
    p.add_argument(
        "--kd-att", type=float, default=None,
        help="Fixed Kd_att outer attitude gain (used when Kd is not the swept "
             "axis). Overrides --seed-from. Default 10.0 if neither is given.",
    )
    p.add_argument(
        "--seed-from", type=str, default=None, metavar="PATH",
        help="Archive CSV, or directory of them, to take the pinned "
             "Kp_att/Kd_att from — the best-scoring row wins. Must have been "
             "written by an EIGEN sweep: Kp_att means a different physical "
             "quantity in the Cascaded PD controller (thrust differential, "
             "N/rad, optimum 25-55) than here (commanded angular rate, 1/s, "
             "optimum 3-4), so a PD archive is rejected rather than applied.",
    )
    p.add_argument("--duration",        type=float, default=25.0,
                   help="Simulation seconds per run (default: 25.0 to ensure >20s of stable hover)")
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
        choices=["spawn_xy", "random_xy"], default="random_xy",
        help="random_xy (default) puts the goal --goal-radius metres from the "
             "spawn XY in a random direction, so every run is a takeoff plus a "
             "translation. spawn_xy restores the old zero-offset goal, which "
             "made the score unable to distinguish gains at all — see "
             "examples/utils/sweep_task.py.",
    )
    p.add_argument("--goal-radius",     type=float, default=2.0)
    p.add_argument("--goal-seed",       type=int,   default=7)
    # ── Step-response validation of the (score, effort) Pareto front ────────
    # Same methodology/defaults as genetic_sweep.py's --validate-every, so
    # results are comparable across GA and grid searches: after the ab-sweep
    # grid is complete for a motor_moment, the points on its (score, effort)
    # Pareto front get ONE additional pass against a target offset from
    # hover, proving position control rather than just hover stability.
    p.add_argument("--validate-pareto", action="store_true", default=True,
                   help="Run the step-response validation pass on the ab-sweep's Pareto front")
    p.add_argument("--no-validate-pareto", dest="validate_pareto", action="store_false")
    p.add_argument("--validation-radius", type=float, default=2.0,
                   help="Lateral offset (m) of the validation target from the hover point")
    p.add_argument("--validation-threshold", type=float, default=2.0,
                   help="Max stability score during the validation step to count as feasible")
    p.add_argument("--validation-seed", type=int, default=123,
                   help="Separate RNG seed for validation target directions")
    # ── Optional early exit along the b axis (ab-sweep mode) ────────────────
    p.add_argument(
        "--b-stop-after-fails", type=int, default=0, metavar="N",
        help="Stop the b loop once N CONSECUTIVE b values have had every "
             "a-value fail. 0 (default) disables it. Off by default because "
             "failure is not monotonic in b: measured at km=1e-7 all a fail "
             "for b=12,16,20,24 then recover at b=28,32,40, and at km=2e-7 "
             "the dead band is b=12..32 with recovery at 40. Any N below ~7 "
             "truncates that recovery region — i.e. exactly the b≈spin-rate "
             "band the sweep is meant to measure. Use only for a quick "
             "reconnaissance pass, and note that skipped cells are written as "
             "'inf' in the archive despite never having been run.",
    )
    p.add_argument(
        "--run-tag", type=str, default="", metavar="TAG",
        help="Suffix for the sweep's output directory (e.g. --run-tag fine "
             "writes results/eigen_sweep/ab_sweep_fine/). Use it when running "
             "the same plane twice with different settings so the second run "
             "lands beside the first instead of overwriting it.",
    )
    p.add_argument("--spawn-altitude", type=float, default=None,
                   help="Altitude [m] to spawn at. Defaults to a ground start, so "
                        "every run includes the takeoff transient. Pass the target "
                        "altitude for the old air start.")
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

def build_target_position(spawn_pos: list, rng, max_dx: float = None) -> tuple:
    if ARGS.goal_mode == "spawn_xy":
        return (spawn_pos[0], spawn_pos[1], float(ARGS.target_altitude))
    # Checked here rather than per sweep mode: all three (ab, kpkd, wn/zeta)
    # route their goals through this one function.
    task.check_goal_reachable(spawn_pos[1], ARGS.goal_radius, ARGS.bounds,
                              label="eigen_sweep")
    return task.offset_target(spawn_pos, ARGS.target_altitude,
                              ARGS.goal_radius, rng, max_dx=max_dx)


def _validation_target(spawn_pos, altitude, radius, rng, max_dx=None):
    """Second, independently drawn offset goal for the Pareto-front pass.

    Same builder as the main pass, drawn from a separate RNG stream and
    decoupled from --goal-mode, so validation is a consistent check regardless
    of how the sweep's own goal is configured."""
    return task.offset_target(spawn_pos, altitude, radius, rng, max_dx=max_dx)


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
import sweep_metrics as sm
import sim_clock
import sweep_task as task

# Resolved after the late imports so every code path below sees numbers. The
# task is a ground start followed by a translation to a laterally offset goal.
ARGS.spawn_altitude = task.resolve_spawn_altitude(ARGS.spawn_altitude)
if getattr(ARGS, "goal_radius", None) is None:
    ARGS.goal_radius = task.DEFAULT_GOAL_RADIUS
from gain_seed import load_best_gains_by_group, lookup_group, SeedError


# ── Resolve the pinned attitude gains ────────────────────────────────────────
# Order of precedence: explicit --kp-att/--kd-att, then --seed-from, then the
# legacy defaults. Seeding failures are fatal on purpose — silently falling
# back to Kp_att=0.7 after being asked to seed is how the ab-sweep ended up
# mapping a region where only 12 % of cells flew.
#
# Seeds are resolved PER MOTOR MOMENT, not once globally: the searched optimum
# moves with spin rate (Kd_att spans 4.45 at km=4e-7 to 11.98 at km=1e-7), so a
# single pin would map the fast- and slow-spin ends of the sweep at gains that
# belong to neither.
_KP_ATT_FALLBACK, _KD_ATT_FALLBACK = 0.7, 10.0
_SEED_TABLE = None

if ARGS.seed_from is not None:
    try:
        _SEED_TABLE = load_best_gains_by_group(
            ARGS.seed_from, "eigen", keys=("Kp_att", "Kd_att"))
    except SeedError as _exc:
        carb.log_error(f"[eigen-sweep] --seed-from failed: {_exc}")
        raise SystemExit(2)
    for _skipped in _SEED_TABLE.pop("_stale_skipped", []):
        carb.log_warn(f"[eigen-sweep] ignoring stale archive: {_skipped}")
    carb.log_warn("[eigen-sweep] per-motor-moment seeds:")
    for _mm in sorted(_SEED_TABLE):
        _e = _SEED_TABLE[_mm]
        carb.log_warn(f"    km={_mm:<10g} Kp_att={_e['Kp_att']:.4f} "
                      f"Kd_att={_e['Kd_att']:.4f}  (score {_e['_score']:.4f})")

# What the user asked for explicitly, captured BEFORE any default or
# representative value is filled in. attitude_gains_for() keys off these:
# reading ARGS afterwards cannot distinguish "user passed --kp-att 3.0" from
# "we back-filled a representative value", which would silently disable the
# per-motor-moment lookup entirely.
_KP_ATT_EXPLICIT = ARGS.kp_att
_KD_ATT_EXPLICIT = ARGS.kd_att

# True when at least one pin varies per motor moment — used to label plots
# honestly rather than quoting one km's value across all of them.
SEEDED_PER_MM = _SEED_TABLE is not None and (_KP_ATT_EXPLICIT is None
                                             or _KD_ATT_EXPLICIT is None)

if ARGS.kp_att is None and _SEED_TABLE is None:
    ARGS.kp_att = _KP_ATT_FALLBACK
if ARGS.kd_att is None and _SEED_TABLE is None:
    ARGS.kd_att = _KD_ATT_FALLBACK

if _SEED_TABLE is not None:
    # Representative values so drone construction and any log line without a
    # motor moment in scope still have a number. The real pin is applied per
    # motor moment inside the sweep loops via attitude_gains_for().
    _first = _SEED_TABLE[sorted(_SEED_TABLE)[0]]
    if ARGS.kp_att is None:
        ARGS.kp_att = _first["Kp_att"]
    if ARGS.kd_att is None:
        ARGS.kd_att = _first["Kd_att"]


def gains_label(mm: float = None) -> str:
    """Human-readable description of the pinned gains for a plot title."""
    if mm is not None:
        kp, kd = attitude_gains_for(mm)
        suffix = " (seeded for this km)" if SEEDED_PER_MM else ""
        return f"Kp_att = {kp:.3f}   Kd_att = {kd:.3f}{suffix}"
    if SEEDED_PER_MM:
        return "Kp_att, Kd_att seeded per motor moment"
    return f"Kp_att = {ARGS.kp_att}   Kd_att = {ARGS.kd_att}"


def attitude_gains_for(mm: float) -> tuple:
    """(Kp_att, Kd_att) to pin for this motor moment.

    An explicit --kp-att/--kd-att always wins. Otherwise the seed table is
    consulted; a motor moment the archive does not cover is fatal rather than
    silently substituted, since the whole point of seeding is that the wrong
    operating point produces a map of a dead region.
    """
    kp, kd = _KP_ATT_EXPLICIT, _KD_ATT_EXPLICIT
    if _SEED_TABLE is not None and (kp is None or kd is None):
        try:
            entry = lookup_group(_SEED_TABLE, mm)
        except SeedError as exc:
            carb.log_error(f"[eigen-sweep] {exc}")
            raise SystemExit(2)
        if kp is None:
            kp = entry["Kp_att"]
        if kd is None:
            kd = entry["Kd_att"]
    if kp is None:
        kp = ARGS.kp_att
    if kd is None:
        kd = ARGS.kd_att
    return float(kp), float(kd)


# ─────────────────────────── Telemetry / OOB backend ─────────────────────────

class BoundedTelemetryBackend(Backend):
    """Wraps EigenController; tracks OOB and position."""

    def __init__(self, inner: EigenController, results_file: str):
        super().__init__(config=None)
        self.inner        = inner
        self.results_file = results_file
        self.total_time   = 0.0
        self._stopped     = False
        self.position     = np.zeros(3)
        self.monitor = task.FlightMonitor(
            ARGS.target_altitude, label="sweep", log=carb.log_warn)
        # Labeling fields — same role as genetic_sweep.py's reliable oob_time
        # write: these must live on the backend itself (not backend.inner) so
        # stop() can always read them, even before the first update. Callers
        # set these directly (backend.omega_n = ..., etc.) alongside
        # backend.inner.<attr> whenever a pass's parameters change.
        self.omega_n  = float("nan")
        self.zeta     = float("nan")
        self.wn_mdyn  = float("nan")
        self.b_mdyn   = float("nan")
        self.mm       = float("nan")


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
        self._stopped   = False
        self.monitor.reset()
        self.inner.start()

    def stop(self):
        if self._stopped:
            return
        self._stopped = True
        self.inner.stop()
        # Append oob_time + run labels to the saved npz. Matches
        # genetic_sweep.py's BoundedBackend.stop() pattern: read the existing
        # npz, add fields, rewrite. Each label is read from self (set by the
        # caller each pass — see run_ab_sweep / main) rather than
        # backend.inner, so it's always defined and this write can't fail
        # partway through.
        if os.path.isfile(self.results_file):
            try:
                data = dict(np.load(self.results_file))
                data["oob_time"]      = np.array(self.oob_time)
                # Flight-phase diagnostics. min_altitude is the transient-safety
                # number the future-biased score cannot express: it is measured
                # post-takeoff, so it says how close the vehicle came to the
                # floor while actually flying.
                data["takeoff_time"]  = np.array(self.monitor.takeoff_time)
                data["min_altitude"]  = np.array(self.monitor.min_altitude)
                data["max_excursion"] = np.array(self.monitor.max_excursion)
                data["failure"]       = np.array(self.monitor.failure)
                data["omega_n"]       = np.array(self.omega_n)
                data["zeta"]          = np.array(self.zeta)
                data["a"]             = np.array(self.wn_mdyn)
                data["b"]             = np.array(self.b_mdyn)
                data["motor_moment"]  = np.array(self.mm)

                np.savez(self.results_file, **data)
            except Exception as exc:
                carb.log_warn(f"[sweep] oob_time/label append failed: {exc}")

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
        """Advance the flight-phase monitor. True once the run is over.

        Takeoff failures and crashes are distinguished in
        examples/utils/sweep_task.py, shared with every other sweep.
        """
        return self.monitor.update(self.total_time, self.position, bounds)


# ─────────────────────────── Score computation ───────────────────────────────

# Worst-case position error sentinel: bounding-box diagonal.
# Used to penalise the missing tail when a drone goes OOB early.
_BOUNDS_DIAGONAL = sm.bounds_diagonal(ARGS.bounds)


def run_metrics(npz_path: str, T_total: float | None = None) -> dict:
    """Every metric for one run — score, effort DC/AC split, spin rate.

    Implementation lives in examples/utils/sweep_metrics.py, shared with
    pd_sweep.py, genetic_sweep.py and eigen_ab_explore.py so the four cannot
    drift apart.
    """
    return sm.compute_metrics(npz_path, float(T_total or 0.0), _BOUNDS_DIAGONAL)


def stability_score(npz_path: str, T_total: float | None = None) -> float:
    """Future-biased mean ‖position error‖, with the post-OOB tail charged at
    the arena diagonal. See examples/utils/sweep_metrics.py."""
    return run_metrics(npz_path, T_total)["score"]


def control_effort_score(npz_path: str, T_total: float | None = None) -> float:
    """Legacy total-thrust Saturated Effort Score.

    Kept so existing archives stay comparable, but it is ~95 % constant hover
    bias (the Iris hover floor alone is ~73.7 N²) and barely resolves control
    activity. Prefer ``effort_ac_ses`` from run_metrics() for new work.
    See examples/utils/sweep_metrics.py.
    """
    return run_metrics(npz_path, T_total)["effort"]


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

        ax.bar(range(len(on_vals)), [s if s is not None else 0 for s in finite_scores],
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
            f"ζ = {zeta:.2f} (fixed)   |   {gains_label(mm)}   "
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
        writer.writerow(["omega_n \\ zeta", f"{zeta:.2f}"])
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
            f"{gains_label(mm)}   |   lower score = more stable",
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
            f"{gains_label(mm)}   |   lower = less actuation effort",
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
    """Root output dir for an (a, b) direct-sweep run.

    --run-tag suffixes the directory so a second ab-sweep (a refined b axis, a
    different pinned operating point) lands beside the first instead of
    overwriting it. Two sweeps of the same plane answer different questions and
    both are worth keeping.
    """
    name = "ab_sweep" + (f"_{ARGS.run_tag}" if ARGS.run_tag else "")
    d = base / "eigen_sweep" / name / f"mm_{mm:.1e}"
    d.mkdir(parents=True, exist_ok=True)
    return d


def ab_b_dir(base: Path, mm: float, b: float) -> Path:
    """Per-b sub-directory that holds one npz file per a value."""
    d = ab_sweep_dir(base, mm) / f"b_{b:.3f}"
    d.mkdir(parents=True, exist_ok=True)
    return d


def kpkd_sweep_dir(base: Path, mm: float) -> Path:
    """Root output dir for a (Kp_att, Kd_att) sweep at fixed M_dyn."""
    name = "kpkd_sweep" + (f"_{ARGS.run_tag}" if ARGS.run_tag else "")
    d = base / "eigen_sweep" / name / f"mm_{mm:.1e}"
    d.mkdir(parents=True, exist_ok=True)
    return d


def kpkd_kd_dir(base: Path, mm: float, kd: float) -> Path:
    """Per-Kd sub-directory that holds one npz file per Kp value."""
    d = kpkd_sweep_dir(base, mm) / f"kd_{kd:.3f}"
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
            f"{gains_label(mm)}   "
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


# ──────────────────── Pareto-front step-response validation ──────────────────

def run_ab_validation_pass(world, timeline, drones, tmp_files, pairs, mm,
                            val_targets, hover_targets):
    """Re-run exactly the (a, b) pairs in `pairs` (the ab-sweep's current
    Pareto-front points for this mm) against `val_targets` instead of their
    normal hover target, batching them through the existing n_a drone slots.
    Returns {(a, b): (score, effort)}. Mirrors genetic_sweep.py's
    validate_subset, and uses the same SES stability/effort scoring so
    results are directly comparable to the GA's step_score/step_effort."""
    render_sim = not ARGS.headless
    n_slots = len(drones)
    results = {}
    pairs = list(pairs)

    for start in range(0, len(pairs), n_slots):
        chunk = pairs[start:start + n_slots]

        for slot in range(n_slots):
            vehicle, backend, spawn_pos, _ = drones[slot]
            a, b = chunk[slot] if slot < len(chunk) else chunk[0]  # harmless filler past chunk end

            vehicle._thrusters   = make_thrust_curve(mm)
            backend.inner.wn_mdyn = a
            backend.inner.b_mdyn  = b
            _kp_v, _kd_v = attitude_gains_for(mm)
            backend.inner.Kp_att  = _kp_v
            backend.inner.Kd_att  = _kd_v
            backend.inner.km      = mm
            backend.inner.hover_target = np.asarray(val_targets[slot], dtype=float)

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
            _m = run_metrics(tmp_files[slot], T_total=ARGS.duration)
            score, effort = _m["score"], _m["effort"]
            results[chunk[slot]] = (score, effort)

    # Put the sweep's own goals back before anything else runs. The drone
    # objects outlive this function — they are reused for every motor moment —
    # so an offset target left installed here becomes the GRID's target on the
    # next pass, silently changing the task partway through the sweep.
    for slot, (_, backend, _, _) in enumerate(drones):
        backend.inner.hover_target = np.asarray(hover_targets[slot], dtype=float)

    return results


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
    carb.log_warn(f"[ab-sweep] {gains_label()}")
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
    # Ground start: every run begins on the floor and has to take off. The air
    # start this replaced put the vehicle exactly on its target, so initial
    # position error was zero and the gains being searched could not affect the
    # score at all. See examples/utils/sweep_task.py.
    spawn_z = float(ARGS.spawn_altitude)
    half    = spacing / 2.0
    rng     = np.random.default_rng(ARGS.goal_seed)
    val_rng = np.random.default_rng(ARGS.validation_seed)

    # Temp npz paths (one per drone slot; overwritten each pass)
    tmp_files = [
        str(results_base / f"_ab_sweep_tmp_{i}.npz") for i in range(n_a)
    ]

    # ── Spawn drones (one per a); b and mm are updated each pass ────────────
    drones: list[tuple] = []
    for i_a, a in enumerate(a_vals):
        spawn_x   = x_start + i_a * spacing
        spawn_pos = [spawn_x, spawn_y, spawn_z]
        target    = build_target_position(spawn_pos, rng,
                                          max_dx=task.slice_max_dx(spacing))
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

    # The sweep's own goals, read back off the controllers so this list can
    # never drift from what they were actually constructed with. Restored after
    # every validation pass — see run_ab_validation_pass.
    hover_targets = [tuple(d[1].inner.hover_target) for d in drones]

    world.reset()
    for _ in range(10):
        world.step(render=render_sim)

    # ── Outer loop: motor_moment ─────────────────────────────────────────────
    for mm in mm_vals:
        kp_att_mm, kd_att_mm = attitude_gains_for(mm)
        carb.log_warn(f"[ab-sweep] mm={mm:.2e}: pinned Kp_att={kp_att_mm:.4f} "
                      f"Kd_att={kd_att_mm:.4f}")
        root_dir = ab_sweep_dir(results_base, mm)
        # 2-D accumulators: rows = a, cols = b
        all_scores  = np.full((n_a, len(b_vals)), np.inf)
        all_efforts = np.full((n_a, len(b_vals)), np.inf)
        # Full metric dict per grid point — carries the effort DC/AC split and
        # the spin rate through to grid_archive.csv alongside score/effort.
        all_metrics = {}
        consecutive_dead_b = 0

        for j_b, b in enumerate(b_vals):
            # Optional early exit from the b axis (see --b-stop-after-fails).
            # OFF by default, and deliberately so: failure is NOT monotonic in
            # b. Measured at km=1e-7, every a fails for b = 12, 16, 20, 24 and
            # then recovers at b = 28, 32, 40; at km=2e-7 the dead band runs
            # b = 12…32 and recovers at 40. Stopping at the first all-fail b
            # would discard exactly the high-b region where b approaches the
            # spin rate — the region these sweeps exist to measure. Enable this
            # only for a coarse reconnaissance pass, never for a run whose
            # grid_archive.csv is going to be analysed.
            if (ARGS.b_stop_after_fails > 0
                    and consecutive_dead_b >= ARGS.b_stop_after_fails):
                skipped = [f"{v:g}" for v in b_vals[j_b:]]
                carb.log_warn(
                    f"[ab-sweep] mm={mm:.2e}: {consecutive_dead_b} consecutive "
                    f"all-fail b values — skipping remaining b = {', '.join(skipped)} "
                    f"(--b-stop-after-fails={ARGS.b_stop_after_fails}). "
                    f"These cells stay 'inf' in the archive; they were NOT measured."
                )
                break

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
                backend.inner.Kp_att      = kp_att_mm
                backend.inner.Kd_att      = kd_att_mm
                backend.inner.km          = mm

                # Labels for the npz (ab-sweep has no ωn/ζ axis; a/b are direct)
                backend.wn_mdyn = a
                backend.b_mdyn  = b
                backend.mm      = mm
                backend.omega_n = float("nan")
                backend.zeta    = float("nan")

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

                # One npz load for every metric.
                m = run_metrics(src, T_total=ARGS.duration)
                score, effort = m["score"], m["effort"]
                all_scores [i_a, j_b] = score
                all_efforts[i_a, j_b] = effort
                all_metrics[(a, b)] = m

                ac_str = "∞" if not np.isfinite(m["effort_ac"]) else f"{m['effort_ac']:.4f}"
                r_str  = "n/a" if not np.isfinite(m["r_ss"]) else f"{m['r_ss']:.2f}"
                carb.log_warn(
                    f"  [a={a:.2f}  b={b:.2f}]  "
                    f"score={'∞' if not np.isfinite(score)  else f'{score:.4f}'}  "
                    f"effort={'∞' if not np.isfinite(effort) else f'{effort:.4f}'}  "
                    f"effort_ac={ac_str}  r_ss={r_str}"
                )

                if os.path.isfile(src):
                    try:
                        os.rename(src, str(dst))
                    except Exception as exc:
                        carb.log_warn(f"  move failed: {exc}")

            # Track the all-fail streak along b for --b-stop-after-fails. A
            # column counts as dead only if every a in it failed; a single
            # surviving a resets the streak, since that is enough to show the
            # region is not uniformly hopeless.
            col = all_scores[:, j_b]
            if np.all(~np.isfinite(col) | (col > sm.STABLE_SCORE_MAX)):
                consecutive_dead_b += 1
                carb.log_warn(
                    f"  [ab-sweep] all {n_a} a-values failed at b={b:g} "
                    f"(consecutive dead b columns: {consecutive_dead_b})"
                )
            else:
                consecutive_dead_b = 0

        # ── Per-mm outputs ───────────────────────────────────────────────────
        save_ab_csv(a_vals, b_vals, all_scores,  root_dir)
        save_ab_effort_csv(a_vals, b_vals, all_efforts, root_dir)
        save_ab_heatmap(a_vals, b_vals, all_scores,  mm, root_dir, label="score")
        save_ab_heatmap(a_vals, b_vals, all_efforts, mm, root_dir, label="effort")

        # ── Pareto-front step-response validation (one pass, front only) ────
        step_scores, step_efforts = None, None
        if ARGS.validate_pareto:
            pareto = _pareto_mask(all_scores.ravel(), all_efforts.ravel()).reshape(all_scores.shape)
            pareto_pairs = [
                (a_vals[i], b_vals[j])
                for i in range(n_a) for j in range(len(b_vals))
                if pareto[i, j]
            ]
            carb.log_warn(
                f"[ab-sweep] mm={mm:.1e}: step-response validation of "
                f"{len(pareto_pairs)} Pareto-front points "
                f"(radius={ARGS.validation_radius}m, threshold={ARGS.validation_threshold})..."
            )
            val_targets = [
                _validation_target(drones[i][2], ARGS.target_altitude,
                                    ARGS.validation_radius, val_rng,
                                    max_dx=task.slice_max_dx(spacing))
                for i in range(n_a)
            ]
            val_results = run_ab_validation_pass(
                world, timeline, drones, tmp_files, pareto_pairs, mm, val_targets,
                hover_targets
            )
            step_scores  = {k: v[0] for k, v in val_results.items()}
            step_efforts = {k: v[1] for k, v in val_results.items()}
            n_feasible = sum(
                1 for s in step_scores.values()
                if np.isfinite(s) and s <= ARGS.validation_threshold
            )
            carb.log_warn(
                f"[ab-sweep] mm={mm:.1e}: {n_feasible}/{len(pareto_pairs)} "
                f"Pareto-front points passed step-response validation"
            )

        save_grid_archive(
            a_vals,
            b_vals,
            all_scores,
            all_efforts,
            mm,
            root_dir,
            step_scores=step_scores,
            step_efforts=step_efforts,
            metrics=all_metrics,
        )
        carb.log_warn("WRITING GRID ARCHIVE")
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


def save_kpkd_grid_archive(kp_vals, kd_vals, scores_2d, efforts_2d,
                           fixed_wn, fixed_b, mm, out_dir, metrics=None):
    """Write the Kp×Kd slice in grid_archive.csv form so interactive_dashboard.py
    picks it up automatically. wn_mdyn/b_mdyn are written as constant columns
    (the pinned operating point) — the dashboard's gene-space panel drops
    zero-variance genes, so only the Kp_att×Kd_att plane is shown.

    Metric columns come from sweep_metrics.METRIC_KEYS, matching every other
    sweep's archive. metrics: optional {(kp, kd): dict} from run_metrics()."""
    import csv
    path = out_dir / "grid_archive.csv"
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["gen", "wn_mdyn", "b_mdyn", "Kp_att", "Kd_att",
                    "motor_moment"] + sm.METRIC_KEYS)
        for i, kp in enumerate(kp_vals):
            for j, kd in enumerate(kd_vals):
                m = dict(sm.METRIC_DEFAULTS)
                if metrics:
                    m.update(metrics.get((kp, kd), {}))
                m["score"] = scores_2d[i, j]
                m["effort"] = efforts_2d[i, j]
                w.writerow(
                    [-1, fixed_wn, fixed_b, kp, kd, mm]
                    + [sm.format_metric(k, m[k]) for k in sm.METRIC_KEYS]
                )
    carb.log_warn(f"[kpkd-sweep] grid archive → {path}")


def run_kpkd_sweep(world, timeline, kp_vals, kd_vals, mm_vals, results_base):
    """(Kp_att, Kd_att) sweep at fixed M_dyn — the causal complement to
    run_ab_sweep. Structure mirrors it exactly:

    • One drone per Kp_att value  →  simulated simultaneously.
    • Kd_att is the inner serial loop  →  one pass per Kd.
    • motor_moment is the outer serial loop.
    • wn_mdyn / b_mdyn are pinned at ARGS.fixed_wn / ARGS.fixed_b throughout,
      isolating the outer-loop gains from the motor-dynamics block.

    Output: results/eigen_sweep/kpkd_sweep/mm_<km>/grid_archive.csv
            results/eigen_sweep/kpkd_sweep/mm_<km>/kd_<kd>/kp<kp>_statistics.npz
    """
    render_sim = not ARGS.headless
    bounds     = ARGS.bounds
    n_kp       = len(kp_vals)
    fixed_wn   = ARGS.fixed_wn
    fixed_b    = ARGS.fixed_b

    carb.log_warn("\n" + "=" * 60)
    carb.log_warn("[kpkd-sweep] Mode: (Kp_att, Kd_att) sweep at fixed M_dyn")
    carb.log_warn(f"[kpkd-sweep] Kp values (parallel) : {kp_vals}")
    carb.log_warn(f"[kpkd-sweep] Kd values (serial)   : {kd_vals}")
    carb.log_warn(f"[kpkd-sweep] fixed wn_mdyn={fixed_wn}  b_mdyn={fixed_b}")
    carb.log_warn(f"[kpkd-sweep] motor_moments        : {mm_vals}")
    carb.log_warn(f"[kpkd-sweep] duration = {ARGS.duration} s per pass")
    carb.log_warn("=" * 60)

    # ── Drone spacing along X (identical layout to ab-sweep) ────────────────
    x_min, x_max = bounds[0], bounds[1]
    x_margin     = 1.5
    x_usable     = x_max - x_min - 2.0 * x_margin
    auto_spacing = x_usable / (n_kp - 1) if n_kp > 1 else 0.0
    spacing      = min(ARGS.spacing, auto_spacing)
    x_start      = x_min + x_margin
    y_min, y_max = bounds[2], bounds[3]
    z_min, z_max = bounds[4], bounds[5]

    for i_kp in range(n_kp):
        sx = x_start + i_kp * spacing
        if not (x_min < sx < x_max):
            raise ValueError(
                f"[kpkd-sweep] Drone {i_kp} spawn X={sx:.2f} outside X bounds "
                f"[{x_min},{x_max}]. Reduce --spacing or widen --bounds."
            )

    spawn_y = -1.5
    # Ground start: every run begins on the floor and has to take off. The air
    # start this replaced put the vehicle exactly on its target, so initial
    # position error was zero and the gains being searched could not affect the
    # score at all. See examples/utils/sweep_task.py.
    spawn_z = float(ARGS.spawn_altitude)
    half    = spacing / 2.0
    rng     = np.random.default_rng(ARGS.goal_seed)

    tmp_files = [str(results_base / f"_kpkd_sweep_tmp_{i}.npz") for i in range(n_kp)]

    # ── Spawn drones (one per Kp); Kd and mm are updated each pass ──────────
    drones: list[tuple] = []
    for i_kp, kp in enumerate(kp_vals):
        spawn_x   = x_start + i_kp * spacing
        spawn_pos = [spawn_x, spawn_y, spawn_z]
        target    = build_target_position(spawn_pos, rng,
                                          max_dx=task.slice_max_dx(spacing))
        drone_bounds = [spawn_x - half, spawn_x + half, y_min, y_max, z_min, z_max]

        inner = EigenController(
            target_position=target,
            results_file=tmp_files[i_kp],
            wn_mdyn=fixed_wn,
            b_mdyn=fixed_b,
            Kp_att=kp,
            Kd_att=kd_vals[0],
            km=mm_vals[0],
        )
        backend = BoundedTelemetryBackend(inner, tmp_files[i_kp])

        cfg              = MultirotorConfig()
        cfg.backends     = [backend]
        cfg.thrust_curve = make_thrust_curve(mm_vals[0])

        vehicle = Multirotor(
            f"/World/kpkd_kp{i_kp}",
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

    # ── Outer loop: motor_moment ────────────────────────────────────────────
    for mm in mm_vals:
        root_dir    = kpkd_sweep_dir(results_base, mm)
        all_scores  = np.full((n_kp, len(kd_vals)), np.inf)
        all_efforts = np.full((n_kp, len(kd_vals)), np.inf)
        # Full metric dict per grid point — carries the effort DC/AC split and
        # the spin rate through to grid_archive.csv alongside score/effort.
        all_metrics = {}

        for j_kd, kd in enumerate(kd_vals):
            carb.log_warn(f"\n{'='*60}")
            carb.log_warn(f"[kpkd-sweep] motor_moment = {mm:.2e}  |  Kd = {kd:.3f}  "
                          f"({j_kd+1}/{len(kd_vals)})")
            carb.log_warn(f"{'='*60}")

            kd_dir = kpkd_kd_dir(results_base, mm, kd)

            for i_kp, (vehicle, backend, spawn_pos, drone_bounds) in enumerate(drones):
                kp = kp_vals[i_kp]

                vehicle._thrusters    = make_thrust_curve(mm)
                backend.inner.wn_mdyn = fixed_wn
                backend.inner.b_mdyn  = fixed_b
                backend.inner.Kp_att  = kp
                backend.inner.Kd_att  = kd
                backend.inner.km      = mm

                backend.wn_mdyn = fixed_wn
                backend.b_mdyn  = fixed_b
                backend.mm      = mm
                backend.omega_n = float("nan")
                backend.zeta    = float("nan")

                vehicle.set_world_pose(position=spawn_pos, orientation=[0., 0., 0., 1.])
                vehicle.set_linear_velocity([0., 0., 0.])
                vehicle.set_angular_velocity([0., 0., 0.])

                backend.reset()
                backend.start()
                carb.log_warn(f"  [Kp={kp:.2f}  Kd={kd:.2f}]  drone {i_kp} ready")

            for _ in range(5):
                world.step(render=render_sim)

            # ── Simulate ─────────────────────────────────────────────────
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
                    if backend.is_oob:
                        continue
                    all_oob = False
                    backend.check_oob(drone_bounds)
                if all_oob:
                    carb.log_warn("  [kpkd-sweep] all drones OOB — ending pass early")
                    break
            timeline.stop()

            # ── Stop, move npz, score ────────────────────────────────────
            for i_kp, (_, backend, _, _) in enumerate(drones):
                if not backend._stopped:
                    backend.stop()
                kp  = kp_vals[i_kp]
                src = tmp_files[i_kp]
                dst = kd_dir / f"kp{kp:.3f}_statistics.npz"

                m = run_metrics(src, T_total=ARGS.duration)
                score, effort = m["score"], m["effort"]
                all_scores [i_kp, j_kd] = score
                all_efforts[i_kp, j_kd] = effort
                all_metrics[(kp, kd)] = m

                ac_str = "∞" if not np.isfinite(m["effort_ac"]) else f"{m['effort_ac']:.4f}"
                r_str  = "n/a" if not np.isfinite(m["r_ss"]) else f"{m['r_ss']:.2f}"
                carb.log_warn(
                    f"  [Kp={kp:.2f}  Kd={kd:.2f}]  "
                    f"score={'∞' if not np.isfinite(score)  else f'{score:.4f}'}  "
                    f"effort={'∞' if not np.isfinite(effort) else f'{effort:.4f}'}  "
                    f"effort_ac={ac_str}  r_ss={r_str}"
                )
                if os.path.isfile(src):
                    try:
                        os.rename(src, str(dst))
                    except Exception as exc:
                        carb.log_warn(f"  move failed: {exc}")

        # ── Per-mm outputs ──────────────────────────────────────────────────
        save_kpkd_grid_archive(kp_vals, kd_vals, all_scores, all_efforts,
                               fixed_wn, fixed_b, mm, root_dir,
                               metrics=all_metrics)
        fin_mask = np.isfinite(all_scores)
        if fin_mask.any():
            bi, bj = np.unravel_index(
                np.argmin(np.where(fin_mask, all_scores, np.inf)), all_scores.shape)
            carb.log_warn(
                f"\n  ★ Best for mm={mm:.1e}: Kp={kp_vals[bi]:.3f}  Kd={kd_vals[bj]:.3f}  "
                f"→ score={all_scores[bi,bj]:.4f}  (wn={fixed_wn}, b={fixed_b} fixed)")
        else:
            carb.log_warn(f"\n  ✗ All drones OOB for mm={mm:.1e}")

    for f in tmp_files:
        try:
            os.remove(f)
        except FileNotFoundError:
            pass


def save_wn_zeta_archive(
    on_vals,
    zeta,
    scores,
    efforts,
    mm,
    out_dir,
):
    import csv
    import math

    path = out_dir / "grid_archive.csv"

    with open(path, "w", newline="") as f:
        writer = csv.writer(f)

        writer.writerow([
            "gen",
            "omega_n",
            "zeta",
            "wn_mdyn",
            "b_mdyn",
            "motor_moment",
            "score",
            "effort",
        ])

        for i, omega_n in enumerate(on_vals):
            wn_mdyn = zeta * omega_n
            b_mdyn = omega_n * math.sqrt(max(1.0 - zeta**2, 0.0))

            writer.writerow([
                -1,
                omega_n,
                zeta,
                wn_mdyn,
                b_mdyn,
                mm,
                scores[i] if np.isfinite(scores[i]) else "inf",
                efforts[i] if np.isfinite(efforts[i]) else "inf",
            ])

def save_grid_archive(a_vals, b_vals, scores_2d, efforts_2d, mm, out_dir,
                       step_scores=None, step_efforts=None, metrics=None):
    """Long-format (a, b) grid archive for interactive_dashboard.py.

    Metric columns come from sweep_metrics.METRIC_KEYS, so this archive carries
    the same set — effort DC/AC split and spin rate included — as every other
    sweep's.

    step_scores/step_efforts: optional {(a, b): value} dicts from the
    Pareto-front step-response validation pass — NaN for any (a, b) that
    wasn't on the front and so was never validated.
    metrics: optional {(a, b): metric dict} from run_metrics().
    """
    import csv

    path = out_dir / "grid_archive.csv"

    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["gen", "wn_mdyn", "b_mdyn", "motor_moment"] + sm.METRIC_KEYS)

        for i, a in enumerate(a_vals):
            for j, b in enumerate(b_vals):
                m = dict(sm.METRIC_DEFAULTS)
                if metrics:
                    m.update(metrics.get((a, b), {}))
                # scores_2d/efforts_2d stay authoritative for the two primary
                # objectives — they are what the Pareto mask was computed on.
                m["score"] = scores_2d[i, j]
                m["effort"] = efforts_2d[i, j]
                if step_scores:
                    m["step_score"] = step_scores.get((a, b), float("nan"))
                if step_efforts:
                    m["step_effort"] = step_efforts.get((a, b), float("nan"))
                writer.writerow(
                    [-1, a, b, mm]
                    + [sm.format_metric(k, m[k]) for k in sm.METRIC_KEYS]
                )


# ─────────────────────────── Main ────────────────────────────────────────────

def main():
    timeline     = omni.timeline.get_timeline_interface()
    render_sim   = not ARGS.headless
    bounds       = ARGS.bounds
    mm_vals      = ARGS.motor_moments
    base_dir     = Path(__file__).resolve().parent.parent
    results_base = task.results_root(base_dir)

    # ── Dispatch: (Kp_att, Kd_att) sweep if --kp-values is provided ─────────
    if ARGS.kp_values is not None:
        if not ARGS.kd_values:
            raise ValueError(
                "--kd-values must be supplied together with --kp-values for kpkd-sweep mode."
            )
        pg       = PegasusInterface()
        pg._world = World(**pg._world_settings)
        world    = pg.world
        pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])
        for _ in range(20):
            world.step(render=render_sim)

        run_kpkd_sweep(
            world=world,
            timeline=timeline,
            kp_vals=ARGS.kp_values,
            kd_vals=ARGS.kd_values,
            mm_vals=mm_vals,
            results_base=results_base,
        )

        carb.log_warn("\n=== kpkd-sweep complete ===")
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
    carb.log_warn(f"[sweep] attitude   : {gains_label()}")
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
    # Ground start: every run begins on the floor and has to take off. The air
    # start this replaced put the vehicle exactly on its target, so initial
    # position error was zero and the gains being searched could not affect the
    # score at all. See examples/utils/sweep_task.py.
    spawn_z  = float(ARGS.spawn_altitude)
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
        target    = build_target_position(spawn_pos, rng,
                                          max_dx=task.slice_max_dx(spacing))

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
                _kp_w, _kd_w = attitude_gains_for(mm)
                backend.inner.Kp_att  = _kp_w
                backend.inner.Kd_att  = _kd_w
                backend.inner.km      = mm

                # Labels for the npz
                backend.omega_n = on
                backend.zeta    = zeta
                backend.wn_mdyn = wn_m
                backend.b_mdyn  = b_m
                backend.mm      = mm

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

                _m = run_metrics(src, T_total=ARGS.duration)
                score, effort = _m["score"], _m["effort"]
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
            save_wn_zeta_archive(on_vals, zeta, scores, efforts, mm, rdir)

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