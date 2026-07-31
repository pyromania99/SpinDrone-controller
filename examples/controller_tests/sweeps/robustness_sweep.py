#!/usr/bin/env python3
"""
robustness_sweep.py — how far a tuned controller can be pushed before it fails.

The other sweeps search for gains. This one does not search: it takes gains that
are already known good and degrades the world around them, one axis at a time,
until they stop working. What it reports is the magnitude at which that happens.

    inertia   the vehicle's mass and inertia are scaled; the controller's model
              is not. A pure model mismatch — the plant is heavier or lighter
              than the thing the gains were tuned for.
    mass      mass only, inertia untouched. Separated from `inertia` because
              they fail through different channels: mass through the hover
              feedforward (altitude droop), inertia through the torque model
              (attitude bandwidth). A real airframe changes both at once, which
              is exactly why a combined axis cannot tell you which one broke.
    lag       the state reaching the controller is delayed. Position and
              attitude are delayed independently — see examples/utils/robustness.py
              for why a single lumped number would misattribute every failure.
    wind      an unmodelled external force, applied through the vehicle's own
              linear drag law. Constant, gust or seeded turbulence.

Layout
──────
One drone per perturbation magnitude, all simulated together across X; serial
outer loops over motor moment and over configuration. The inertia axis is set
once per drone before ``world.reset()``, which is the only point PhysX will
accept new mass properties (measured — see evaluations/probe_robustness.py).

The zero point is real
──────────────────────
A mismatch axis is only meaningful as a perturbation away from a *matched*
plant, and until examples/utils/vehicle_params.py landed there was no such
point: Eigen and PD modelled the vehicle 7.3 % light, MPC 25.9 % light, and all
three had the wrong inertia. ``--magnitudes 1.0`` is now genuinely unperturbed,
and is included by default as the control.

Outputs (results/robustness/)
─────────────────────────────
    robustness_archive.csv   one long-format row per run: controller, axis,
                             magnitude, motor moment, config, seed, gains and
                             every key in sweep_metrics.METRIC_KEYS
    breakdown.csv            reduced to one row per (controller, config, axis):
                             the magnitude at which the run stopped flying
    <axis>/mm_<km>/cfg<i>/mag_<m>_statistics.npz    raw telemetry per run

Usage
─────
    isaac_python robustness_sweep.py --controller eigen --axis inertia \\
        --magnitudes 1.0 1.2 1.5 2.0 --motor-moments 0.0 2.7e-7 --headless

    isaac_python robustness_sweep.py --controller pd --axis wind \\
        --wind-profile turbulence --magnitudes 0 2 4 6 --seeds 1 2 3 --headless
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import sys
from pathlib import Path

import numpy as np

import carb
from isaacsim import SimulationApp


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Controller robustness sweep")
    p.add_argument("--controller", choices=("eigen", "pd", "mpc"), required=True)
    p.add_argument("--axis", choices=("inertia", "mass", "lag", "wind"), required=True)
    p.add_argument("--magnitudes", nargs="+", type=float, default=None,
                   help="One drone per value. Meaning depends on --axis: a scale "
                        "factor for inertia/mass (1.0 = matched), milliseconds "
                        "for lag, m/s for wind.")

    p.add_argument("--configs-from", type=str, default=None,
                   help="pareto_top.json to take gains from (default: the one "
                        "under results/pareto_top/).")
    p.add_argument("--gains", type=str, default=None,
                   help="Explicit JSON list of gain dicts, bypassing "
                        "--configs-from. Use for controllers the Pareto "
                        "selection does not cover yet.")
    p.add_argument("--top", type=int, default=3,
                   help="Configs per controller to carry through the axis.")

    p.add_argument("--motor-moments", nargs="+", type=float, default=[0.0, 2.7e-7])

    # Lag axis: --magnitudes gives the POSITION delay in ms; the attitude delay
    # is a fixed fraction of it, since a real vehicle's IMU is roughly an order
    # of magnitude faster than its position estimate. Override to sweep the
    # attitude channel directly instead.
    p.add_argument("--att-delay-frac", type=float, default=0.1)
    p.add_argument("--att-delay-ms", type=float, default=None,
                   help="Fix the attitude delay [ms] instead of deriving it "
                        "from --att-delay-frac, e.g. to sweep position alone.")
    p.add_argument("--pos-tau-ms", type=float, default=0.0)
    p.add_argument("--att-tau-ms", type=float, default=0.0)

    # Wind axis
    p.add_argument("--wind-profile", choices=("constant", "gust", "turbulence"),
                   default="constant")
    p.add_argument("--wind-direction", type=float, default=0.0)
    p.add_argument("--wind-vertical", type=float, default=0.0)
    p.add_argument("--gust-start", type=float, default=5.0)
    p.add_argument("--gust-duration", type=float, default=1.0)
    p.add_argument("--turbulence-tau", type=float, default=1.0)
    p.add_argument("--seeds", nargs="+", type=int, default=[1],
                   help="Repeats for the stochastic wind profile. Ignored by "
                        "every other axis, which is deterministic.")

    p.add_argument("--duration", type=float, default=12.0)
    p.add_argument("--target-altitude", type=float, default=1.5)
    p.add_argument("--spacing", type=float, default=4.0)
    p.add_argument("--bounds", nargs=6, type=float,
                   default=[-12., 12., -12., 12., 0., 20.])
    p.add_argument("--goal-seed", type=int, default=7)
    p.add_argument("--goal-radius", type=float, default=None)
    p.add_argument("--spawn-altitude", type=float, default=None)
    p.add_argument("--out-dir", type=str, default=None)
    p.add_argument("--headless", action="store_true", default=False)
    p.add_argument("--no-headless", dest="headless", action="store_false")
    return p.parse_args()


ARGS = parse_args()

# Per-axis magnitude defaults. Chosen to straddle the interesting region rather
# than to look symmetric: 1.0 is always present as the matched control.
DEFAULT_MAGNITUDES = {
    # Scale factors, weighted toward gamma < 1 — the destabilising direction.
    #
    # The controller computes a torque from its own (nominal) inertia while the
    # plant has gamma x that. With gamma > 1 the commanded torque produces LESS
    # angular acceleration than intended: sluggish, but self-limiting. With
    # gamma < 1 it produces MORE, so the attitude loop over-rotates and the
    # error compounds — that is where a breakdown point exists to find.
    #
    # Measured on eigen at km=2.7e-7: gamma = 2.5 degraded the score only 6.9 %
    # (0.1226 -> 0.1310) and every run stayed stable, so sampling the gamma > 1
    # side densely just buys near-identical rows. The ladder below keeps three
    # points above 1.0 for the trend and puts the resolution below it.
    "inertia": [1.0, 0.75, 0.5, 0.35, 0.25, 1.5, 2.5, 4.0],
    "mass":    [1.0, 1.1, 1.25, 1.5, 0.9, 0.75],
    # Position delay in ms. 4 ms is one physics step; 200 ms is a bad GPS.
    "lag":     [0.0, 20.0, 50.0, 100.0, 150.0, 200.0],
    # Wind speed in m/s. 12 m/s is a strong breeze against a 1.6 kg airframe.
    "wind":    [0.0, 2.0, 4.0, 6.0, 9.0, 12.0],
}
if ARGS.magnitudes is None:
    ARGS.magnitudes = DEFAULT_MAGNITUDES[ARGS.axis]

simulation_app = SimulationApp({
    "headless": ARGS.headless,
    "hide_ui": True if ARGS.headless else None,
    "disable_viewport_updates": True if ARGS.headless else False,
    "multi_gpu": False, "max_gpu_count": 1, "active_gpu": 0,
})

import omni.timeline                                    # noqa: E402
from isaacsim.core.api import World                     # noqa: E402
from scipy.spatial.transform import Rotation            # noqa: E402

for _parent in Path(__file__).resolve().parents:
    _candidate = _parent / "extensions" / "pegasus.simulator"
    if _candidate.exists():
        if str(_candidate) not in sys.path:
            sys.path.insert(0, str(_candidate))
        break

from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS               # noqa: E402
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface   # noqa: E402
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig  # noqa: E402
from pegasus.simulator.logic.thrusters import QuadraticThrustCurve                 # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "utils"))
import robustness as rb          # noqa: E402
import sim_clock                 # noqa: E402
import sweep_metrics as sm       # noqa: E402
import sweep_task as task        # noqa: E402
import vehicle_params            # noqa: E402
from eigen_controller import EigenController              # noqa: E402
from cascaded_pd_controller import CascadedPDController   # noqa: E402
from lpv_mpc_controller import LPVMPCController           # noqa: E402

ARGS.spawn_altitude = task.resolve_spawn_altitude(ARGS.spawn_altitude)
if ARGS.goal_radius is None:
    ARGS.goal_radius = task.DEFAULT_GOAL_RADIUS

RESULTS = Path(__file__).resolve().parents[1] / "results"
PARETO_JSON = RESULTS / "pareto_top" / "pareto_top.json"


# ── Configurations ───────────────────────────────────────────────────────────

def load_configs() -> list:
    """Gain sets to carry through the perturbation axis.

    Prefers an explicit --gains list, then a pareto_top.json. Raises rather
    than falling back to a hardcoded default: a robustness result is only
    meaningful if you know which operating point it describes, and silently
    substituting one would produce a plausible-looking table about nothing.
    """
    if ARGS.gains:
        configs = json.loads(ARGS.gains)
        if not isinstance(configs, list) or not configs:
            raise ValueError("--gains must be a non-empty JSON list of gain dicts")
        return [{"rank": i + 1, "source": "explicit", "gains": g}
                for i, g in enumerate(configs[:ARGS.top])]

    path = Path(ARGS.configs_from) if ARGS.configs_from else PARETO_JSON
    if not path.is_file():
        raise FileNotFoundError(
            f"no configuration source: {path} does not exist. Run "
            f"evaluations/select_pareto_top.py, or pass --gains explicitly.")

    data = json.loads(path.read_text())
    pooled, seen = [], set()
    for entry in data.get("sets", []):
        if entry.get("controller") != ARGS.controller:
            continue
        for config in entry.get("configs", []):
            gains = config.get("gains", {})
            key = tuple(sorted(gains.items()))
            if key in seen:
                continue
            seen.add(key)
            pooled.append({"rank": config.get("rank"),
                           "source": config.get("source", "pareto"),
                           "score": (config.get("metrics") or {}).get("score"),
                           "gains": gains})
    if not pooled:
        raise ValueError(
            f"{path} holds no '{ARGS.controller}' configurations. "
            f"select_pareto_top.py covers "
            f"{sorted({e.get('controller') for e in data.get('sets', [])})}; "
            f"pass --gains for anything else.")

    # Spread the selection along the front rather than taking the N best
    # tracking scores, which would cluster at one end and say nothing about
    # whether low-effort tunings are more or less robust.
    pooled.sort(key=lambda c: (c["score"] if c["score"] is not None else np.inf))
    if len(pooled) > ARGS.top:
        idx = np.linspace(0, len(pooled) - 1, ARGS.top).round().astype(int)
        pooled = [pooled[i] for i in idx]

    # Unique id, assigned here rather than reusing pareto_top.json's `rank`.
    #
    # That rank is numbered 1..N *within* each (controller, motor_moment) set,
    # and this function pools across every motor moment — so two different gain
    # sets routinely arrive carrying the same rank. Grouping the results by it
    # then merges two distinct configurations into one row: measured on the
    # eigen inertia run, rank 3 covered both (wn=6.81, b=0.31) and
    # (wn=60.0, b=2.52), and its breakdown row reported n_runs=16 for what
    # should have been two separate 8-run configurations.
    #
    # `rank` and `source` are kept alongside for provenance.
    for i, config in enumerate(pooled):
        config["id"] = i
    return pooled


def make_controller(gains: dict, results_file: str, target, km: float):
    """Construct the inner controller for --controller with these gains."""
    g = dict(gains)
    if ARGS.controller == "eigen":
        return EigenController(
            results_file=results_file, target_position=target, km=km,
            wn_mdyn=g.get("wn_mdyn", 18.0), b_mdyn=g.get("b_mdyn", 10.2),
            Kp_att=g.get("Kp_att", 7.0), Kd_att=g.get("Kd_att", 1.0),
            alpha=g.get("alpha", 0.5), beta=g.get("beta", 0.6))
    if ARGS.controller == "pd":
        # Kp_att/Kd_att are per-axis pairs here, and are a thrust differential
        # [N/rad] rather than the commanded rate they are in the Eigen
        # controller. See examples/utils/gain_seed.py.
        kp = g.get("Kp_att", 1.6)
        kd = g.get("Kd_att", 1.8)
        return CascadedPDController(
            results_file=results_file, target_position=target,
            Kp_att=(kp, kp) if np.isscalar(kp) else tuple(kp),
            Kd_att=(kd, kd) if np.isscalar(kd) else tuple(kd),
            Kd_yaw_rate=g.get("Kd_yaw_rate", 0.0))
    return LPVMPCController(
        results_file=results_file, target_position=target, km=km,
        q_weight=g.get("mpc_q", g.get("q_weight", 10.0)),
        r_weight=g.get("mpc_r", g.get("r_weight", 0.01)),
        N=int(g.get("N", 12)))


def perturbation_for(magnitude: float, seed: int) -> dict:
    """Translate an axis magnitude into the objects that realise it."""
    dt = sim_clock.DEFAULT_PHYSICS_DT
    out = {"lag": None, "wind": rb.WindField("none"),
           "mass_scale": 1.0, "inertia_scale": 1.0}

    if ARGS.axis == "inertia":
        out["inertia_scale"] = float(magnitude)
    elif ARGS.axis == "mass":
        out["mass_scale"] = float(magnitude)
    elif ARGS.axis == "lag":
        att_ms = (ARGS.att_delay_ms if ARGS.att_delay_ms is not None
                  else ARGS.att_delay_frac * magnitude)
        out["lag"] = rb.SensorLag(dt,
                                  pos_delay=1e-3 * magnitude,
                                  att_delay=1e-3 * att_ms,
                                  pos_tau=1e-3 * ARGS.pos_tau_ms,
                                  att_tau=1e-3 * ARGS.att_tau_ms)
    else:
        # For gust and turbulence the magnitude is the gust/sigma amplitude on
        # top of no steady wind, so the axis isolates the transient.
        steady = magnitude if ARGS.wind_profile == "constant" else 0.0
        out["wind"] = rb.WindField(
            ARGS.wind_profile, speed=steady, gust_speed=magnitude,
            direction_deg=ARGS.wind_direction, vertical=ARGS.wind_vertical,
            gust_start=ARGS.gust_start, gust_duration=ARGS.gust_duration,
            turbulence_tau=ARGS.turbulence_tau, seed=seed)
    return out


# ── Output ───────────────────────────────────────────────────────────────────

GAIN_COLUMNS = ["wn_mdyn", "b_mdyn", "Kp_att", "Kd_att", "Kd_yaw_rate",
                "mpc_q", "mpc_r"]
ARCHIVE_COLUMNS = (["controller", "axis", "magnitude", "achieved_scale",
                    "motor_moment", "config_id", "config_rank", "config_source",
                    "seed"]
                   + GAIN_COLUMNS + ["failure"] + sm.METRIC_KEYS)


def write_archive(rows: list, out_dir: Path) -> Path:
    path = out_dir / "robustness_archive.csv"
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(ARCHIVE_COLUMNS)
        for row in rows:
            writer.writerow([
                sm.format_metric(key, row[key])
                if key in sm.METRIC_DEFAULTS else row.get(key, "")
                for key in ARCHIVE_COLUMNS])
    carb.log_warn(f"[robustness] archive → {path}")
    return path


# Columns that must NOT go through a fixed-point format. motor_moment spans
# 0 to 6e-7, and "%.6f" renders every value below ~5e-7 as "0.000000" — which
# would silently merge four of the six spin rates into the km=0 bucket. This is
# the same bug examples/utils/gain_seed.py:234 records having already been
# fixed once in the genetic archives; it reappeared here because a generic
# "format floats with .6f" branch caught this column too.
_BREAKDOWN_EXACT = frozenset({"motor_moment"})
_BREAKDOWN_COMPACT = frozenset({"last_surviving", "first_failing"})


def _format_breakdown(key: str, value):
    """One breakdown cell. Empty for a boundary that was never reached."""
    if value is None:
        return ""
    if key in _BREAKDOWN_EXACT:
        return repr(float(value))
    if key in _BREAKDOWN_COMPACT:
        return f"{value:g}"
    if isinstance(value, float):
        return f"{value:.6f}" if np.isfinite(value) else (
            "nan" if np.isnan(value) else "inf")
    return value


def write_breakdown(rows: list, out_dir: Path) -> Path:
    """One row per (config, motor moment): where the configuration gave out.

    The reduction itself lives in robustness.breakdown_rows — it is the
    analytical claim this whole sweep exists to make, and keeping it in a
    pure-numpy module is what lets it be unit-tested without booting Isaac.
    """
    origin = 1.0 if ARGS.axis in ("inertia", "mass") else 0.0
    path = out_dir / "breakdown.csv"
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(rb.BREAKDOWN_COLUMNS)
        for row in rb.breakdown_rows(rows, origin=origin):
            writer.writerow([_format_breakdown(key, row[key])
                             for key in rb.BREAKDOWN_COLUMNS])
    carb.log_warn(f"[robustness] breakdown → {path}")
    return path


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    timeline = omni.timeline.get_timeline_interface()
    render_sim = not ARGS.headless
    bounds = ARGS.bounds
    magnitudes = list(ARGS.magnitudes)
    n_drones = len(magnitudes)
    seeds = ARGS.seeds if (ARGS.axis == "wind"
                           and ARGS.wind_profile == "turbulence") else [0]

    configs = load_configs()

    out_dir = Path(ARGS.out_dir) if ARGS.out_dir else (
        RESULTS / "robustness" / ARGS.controller / ARGS.axis)
    out_dir.mkdir(parents=True, exist_ok=True)

    carb.log_warn("\n" + "=" * 64)
    carb.log_warn(f"[robustness] controller   : {ARGS.controller}")
    carb.log_warn(f"[robustness] axis         : {ARGS.axis}"
                  + (f" ({ARGS.wind_profile})" if ARGS.axis == "wind" else ""))
    carb.log_warn(f"[robustness] magnitudes   : {magnitudes}")
    carb.log_warn(f"[robustness] motor moments: {ARGS.motor_moments}")
    carb.log_warn(f"[robustness] configs      : {len(configs)}  seeds: {seeds}")
    for config in configs:
        carb.log_warn(f"    rank {config['rank']} ({config['source']}): {config['gains']}")
    carb.log_warn("=" * 64)

    pg = PegasusInterface()
    pg._world = World(**pg._world_settings)
    world = pg.world
    pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])
    for _ in range(20):
        world.step(render=render_sim)

    x_min, x_max = bounds[0], bounds[1]
    x_margin = 1.5
    x_usable = x_max - x_min - 2.0 * x_margin
    auto_spacing = x_usable / (n_drones - 1) if n_drones > 1 else 0.0
    spacing = min(ARGS.spacing, auto_spacing) if n_drones > 1 else ARGS.spacing
    x_start = x_min + x_margin
    spawn_y = -1.5
    spawn_z = float(ARGS.spawn_altitude)
    half = spacing / 2.0
    y_min, y_max, z_min, z_max = bounds[2], bounds[3], bounds[4], bounds[5]

    goal_rng = np.random.default_rng(ARGS.goal_seed)
    max_dx = task.slice_max_dx(spacing)
    task.check_goal_reachable(spawn_y, ARGS.goal_radius, bounds, label="robustness")

    tmp_files = [str(out_dir / f"_tmp_{i}.npz") for i in range(n_drones)]
    drones = []

    for i, magnitude in enumerate(magnitudes):
        spawn_x = x_start + i * spacing
        spawn_pos = [spawn_x, spawn_y, spawn_z]
        target = task.offset_target(spawn_pos, ARGS.target_altitude,
                                    ARGS.goal_radius, goal_rng, max_dx=max_dx)
        drone_bounds = [spawn_x - half, spawn_x + half, y_min, y_max, z_min, z_max]

        perturbation = perturbation_for(magnitude, seeds[0])
        inner = make_controller(configs[0]["gains"], tmp_files[i], target,
                                ARGS.motor_moments[0])
        monitor = task.FlightMonitor(ARGS.target_altitude,
                                     label=f"robustness[{magnitude:g}]",
                                     log=carb.log_warn)
        backend = rb.RobustnessBackend(inner, tmp_files[i], target, monitor,
                                       wind=perturbation["wind"],
                                       lag=perturbation["lag"],
                                       log=carb.log_warn)

        cfg = MultirotorConfig()
        cfg.backends = [backend]
        cfg.thrust_curve = QuadraticThrustCurve(
            config={"motor_moment": ARGS.motor_moments[0]})

        vehicle = Multirotor(
            f"/World/robust_{i}", ROBOTS["Iris"], i, spawn_pos,
            Rotation.from_euler("XYZ", [0., 0., 0.], degrees=True).as_quat(),
            config=cfg)
        drones.append({"vehicle": vehicle, "backend": backend,
                       "spawn": spawn_pos, "bounds": drone_bounds,
                       "target": target, "magnitude": magnitude})

    # ── Plant-side perturbation, before the reset that makes it real ─────────
    #
    # PhysX reads mass properties at scene construction. The write below is
    # accepted immediately and ignored until world.reset(), so this ordering is
    # load-bearing: reversed, every run would be nominal and the CSV would say
    # otherwise. Verified in evaluations/probe_robustness.py.
    nominals = {}
    if ARGS.axis in ("inertia", "mass"):
        world.reset()
        for _ in range(5):
            world.step(render=render_sim)
        for drone in drones:
            nominal = rb.nominal_decomposition(drone["vehicle"])
            if nominal is None:
                raise RuntimeError("could not read nominal mass properties")
            nominals[id(drone["vehicle"])] = nominal
            perturbation = perturbation_for(drone["magnitude"], seeds[0])
            rb.apply_plant_scale(drone["vehicle"],
                                 mass_scale=perturbation["mass_scale"],
                                 inertia_scale=perturbation["inertia_scale"],
                                 decomposition=nominal)

    world.reset()
    for _ in range(10):
        world.step(render=render_sim)

    # Measure what the perturbation actually achieved, and refuse to continue
    # if it silently did nothing — a whole sweep of "nominal" runs mislabelled
    # as perturbed is worse than no sweep.
    achieved = {}
    if ARGS.axis in ("inertia", "mass"):
        carb.log_warn("[robustness] plant perturbation, as achieved:")
        for drone in drones:
            perturbation = perturbation_for(drone["magnitude"], seeds[0])
            check = rb.verify_plant_scale(
                drone["vehicle"], nominals[id(drone["vehicle"])],
                perturbation["mass_scale"], perturbation["inertia_scale"])
            achieved[drone["magnitude"]] = check
            if not check.get("ok"):
                raise RuntimeError(f"could not verify plant scale: {check}")
            carb.log_warn(
                f"    requested {drone['magnitude']:g}x → mass "
                f"{check['mass_scale_achieved']:.4f}x  inertia "
                f"({', '.join(f'{v:.4f}' for v in check['inertia_scale_achieved'])})x"
                + ("" if check["within_tolerance"] else "   ** OUT OF TOLERANCE **"))
            if not check["within_tolerance"]:
                raise RuntimeError(
                    f"plant scale for magnitude {drone['magnitude']:g} missed its "
                    f"target by more than 1 %; refusing to record it as perturbed")

    rows = []
    for mm in ARGS.motor_moments:
        for config in configs:
            for seed in seeds:
                label = (f"mm={mm:.1e} cfg{config['id']} (rank {config['rank']})"
                         + (f" seed={seed}" if len(seeds) > 1 else ""))
                carb.log_warn(f"\n[robustness] {label}")

                leaf = f"cfg{config['id']}"
                if len(seeds) > 1:
                    leaf += f"_seed{seed}"
                run_dir = out_dir / f"mm_{mm:.1e}" / leaf
                run_dir.mkdir(parents=True, exist_ok=True)

                for i, drone in enumerate(drones):
                    backend = drone["backend"]
                    perturbation = perturbation_for(drone["magnitude"], seed)
                    backend.wind = perturbation["wind"]
                    backend.lag = perturbation["lag"]

                    # Swap the plant's motor moment and rebuild the controller
                    # so its gains and its km both come from this pass.
                    drone["vehicle"]._thrusters = QuadraticThrustCurve(
                        config={"motor_moment": mm})
                    backend.inner = make_controller(
                        config["gains"], tmp_files[i], drone["target"], mm)
                    backend.inner.initialize(drone["vehicle"])

                    drone["vehicle"].set_world_pose(position=drone["spawn"],
                                                    orientation=[0., 0., 0., 1.])
                    drone["vehicle"].set_linear_velocity([0., 0., 0.])
                    drone["vehicle"].set_angular_velocity([0., 0., 0.])
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
                    for drone in drones:
                        if drone["backend"].is_oob:
                            continue
                        all_oob = False
                        drone["backend"].check_oob(drone["bounds"])
                    if all_oob:
                        carb.log_warn("  all drones failed — ending pass early")
                        break
                timeline.stop()

                for i, drone in enumerate(drones):
                    backend = drone["backend"]
                    if not backend._stopped:
                        backend.stop()

                    src = tmp_files[i]
                    dst = run_dir / f"mag_{drone['magnitude']:g}_statistics.npz"
                    metrics = sm.compute_metrics(src, ARGS.duration,
                                                 sm.bounds_diagonal(bounds))

                    check = achieved.get(drone["magnitude"], {})
                    row = {
                        "controller": ARGS.controller,
                        "axis": (ARGS.axis if ARGS.axis != "wind"
                                 else f"wind-{ARGS.wind_profile}"),
                        "magnitude": drone["magnitude"],
                        "achieved_scale": (
                            f"{check['mass_scale_achieved']:.4f}"
                            if ARGS.axis == "mass" and check else
                            f"{check['inertia_scale_achieved'][0]:.4f}"
                            if ARGS.axis == "inertia" and check else ""),
                        "motor_moment": mm,
                        "config_id": config["id"],
                        "config_rank": config["rank"],
                        "config_source": config["source"],
                        "seed": seed if len(seeds) > 1 else "",
                        "failure": backend.monitor.failure,
                    }
                    for column in GAIN_COLUMNS:
                        row[column] = config["gains"].get(column, "")
                    row.update(metrics)
                    rows.append(row)

                    carb.log_warn(
                        f"    {drone['magnitude']:>7g} → score={metrics['score']:.4f}"
                        f"  effort_ac={metrics['effort_ac']:.4f}"
                        f"  stable={metrics['stable']}"
                        + (f"  [{backend.monitor.failure}]"
                           if backend.monitor.failure else ""))

                    if os.path.isfile(src):
                        try:
                            os.replace(src, str(dst))
                        except Exception:
                            pass

    write_archive(rows, out_dir)
    write_breakdown(rows, out_dir)

    for f in tmp_files:
        if os.path.exists(f):
            os.remove(f)
    carb.log_warn(f"\n[robustness] complete — {len(rows)} runs → {out_dir}")

    try:
        timeline.stop()
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
