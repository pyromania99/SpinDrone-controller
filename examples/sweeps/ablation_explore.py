#!/usr/bin/env python3
"""
ablation_explore.py — march each tuning parameter outward until flight breaks.

What this is for
────────────────
Reading a gradient. The genetic sweeps converge, which is the opposite of what
a gradient needs: they concentrate every sample near one point and say nothing
about the shape of the space around it. This holds every gene but one at a
reference configuration and steps that one outward in both directions,
producing a response curve per (gene x metric) — which for a controlled
one-at-a-time design IS the partial dependence, exactly, with no estimator and
no smoothing.

Why outward-until-broken rather than a fixed range
──────────────────────────────────────────────────
Two reasons, both measured.

Exploration among crashes is wasted: sampling a fixed box spends most of its
budget outside the stable basin once several genes are free. Marching outward
from a point known to fly puts every sample inside or at the edge of the basin
by construction.

And the boxes are too small. Every free-outer genetic winner pins at least one
gene to a bound — wn_xy sits at its 2.0 ceiling for all three controllers — so
an arm confined to the box would report "never broke" when it merely never left
the box. gene_space.ladder_values therefore marches past the box by
--extend-factor, stopping at a physical floor where one exists.

How the batching works
──────────────────────
The march is nominally sequential (you cannot know the next step until the
current one is scored) while the GPU wants 16 drones. Resolved by marching
BREADTH-FIRST: one round advances every live arm by one step, and the batch is
padded to full with replicates of the reference. Eigen has 6 genes -> 12 arms +
4 replicates; pd 10 + 6; mpc 14 + 2. Near-total slot use, no speculative waste,
and the padding is not filler — it accumulates the noise floor without which no
response curve can be read.

The noise floor matters more than it sounds. The effect being hunted is of the
order of 7 % of the median score; repeated evaluations of an identical gene
tuple differ by ~1.5 %. A curve that moves less than the replicate spread is
flat, and the report must be able to say so rather than draw a trend through
it.

Outputs (results/ablation/<controller>/mm_<km>/)
────────────────────────────────────────────────
    reference.json      written earlier by evaluations/select_reference.py
    calibration.csv     the reference flown in every slot — the noise floor
    ablation_archive.csv  one row per run, long format
    limits.csv          per (gene, direction): where it broke, or why it did not
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
from pathlib import Path



def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="One-at-a-time ablation march")
    p.add_argument("--controller", choices=("eigen", "pd", "mpc"), required=True)
    p.add_argument("--motor-moments", nargs="+", type=float, default=[0.0, 2.7e-7])
    p.add_argument("--reference-root", type=str, default=None,
                   help="Where reference.json lives (default results/ablation)")

    # Ladder shape
    p.add_argument("--ratio", type=float, default=1.25,
                   help="Multiplicative step for log-scaled genes")
    p.add_argument("--delta-frac", type=float, default=0.08,
                   help="Step for linear genes, as a fraction of the box width")
    p.add_argument("--max-steps", type=int, default=24,
                   help="Cap per arm, so a gene that never breaks still ends")
    p.add_argument("--extend-factor", type=float, default=2.0,
                   help="March past the box to [lo/f, hi*f]. 1.0 stays inside. "
                        "Needed because the free-outer winners pin to bounds.")
    p.add_argument("--break-confirm", type=int, default=2,
                   help="Consecutive broken steps before an arm is declared "
                        "dead, so one marginal run cannot truncate it")

    p.add_argument("--slots", type=int, default=16,
                   help="Drones simulated in parallel")
    p.add_argument("--calibration-batch", action="store_true", default=True,
                   help="Fly the reference in every slot first, for the noise floor")
    p.add_argument("--no-calibration-batch", dest="calibration_batch",
                   action="store_false")

    p.add_argument("--duration", type=float, default=12.0)
    p.add_argument("--target-altitude", type=float, default=1.5)
    p.add_argument("--spacing", type=float, default=3.5)
    p.add_argument("--row-pitch", type=float, default=6.0)
    p.add_argument("--bounds", nargs=6, type=float,
                   default=[-12., 12., -12., 12., 0., 20.])
    p.add_argument("--goal-seed", type=int, default=7)
    p.add_argument("--goal-radius", type=float, default=None)
    p.add_argument("--spawn-altitude", type=float, default=None)
    # Every point on a response curve lands in a different drone slot, and
    # offset_target draws a different goal direction per slot. That injects
    # slot-to-slot goal variance straight into a gradient whose expected size
    # is ~7 %. One direction for every slot removes the confound; slots then
    # differ only by arena position.
    p.add_argument("--fixed-goal", action="store_true", default=True)
    p.add_argument("--no-fixed-goal", dest="fixed_goal", action="store_false")
    p.add_argument("--out-dir", type=str, default=None)
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

import numpy as np                                    # noqa: E402
import omni.timeline                                  # noqa: E402
from isaacsim.core.api import World                   # noqa: E402
from scipy.spatial.transform import Rotation          # noqa: E402

for _parent in Path(__file__).resolve().parents:
    _candidate = _parent / "extensions" / "pegasus.simulator"
    if _candidate.exists():
        if str(_candidate) not in sys.path:
            sys.path.insert(0, str(_candidate))
        break

from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS                # noqa: E402
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface    # noqa: E402
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig  # noqa: E402
from pegasus.simulator.logic.thrusters import QuadraticThrustCurve                  # noqa: E402
from pegasus.simulator.logic.backends import Backend                                # noqa: E402
from pegasus.simulator.logic.state import State                                     # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "utils"))
import controller_factory as cf   # noqa: E402
import gene_space                 # noqa: E402
import robustness as rb           # noqa: E402
import sim_clock                  # noqa: E402
import sweep_metrics as sm        # noqa: E402
import sweep_task as task         # noqa: E402

ARGS.spawn_altitude = task.resolve_spawn_altitude(ARGS.spawn_altitude)
if ARGS.goal_radius is None:
    ARGS.goal_radius = task.DEFAULT_GOAL_RADIUS

RESULTS = task.results_root()
_BOUNDS_DIAG = sm.bounds_diagonal(ARGS.bounds)


class BoundedBackend(Backend):
    """Flight-phase gating, shared with every other sweep via FlightMonitor."""

    def __init__(self, inner, results_file: str):
        super().__init__(config=None)
        self.inner = inner
        self.results_file = results_file
        self.total_time = 0.0
        self._stopped = False
        self.position = np.zeros(3)
        self.monitor = task.FlightMonitor(
            ARGS.target_altitude, label="ablation", log=carb.log_warn)

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
                data["takeoff_time"] = np.array(self.monitor.takeoff_time)
                data["min_altitude"] = np.array(self.monitor.min_altitude)
                data["max_excursion"] = np.array(self.monitor.max_excursion)
                data["failure"] = np.array(self.monitor.failure)
                data["config_json"] = np.array(
                    json.dumps(sm.controller_config(self.inner)))
                np.savez(self.results_file, **data)
            except Exception:
                pass

    def reset(self):
        self.inner.reset()

    def update_sensor(self, st, d):
        if not self.is_oob:
            self.inner.update_sensor(st, d)

    def update_graphical_sensor(self, st, d):
        if not self.is_oob:
            self.inner.update_graphical_sensor(st, d)

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
        return self.monitor.update(self.total_time, self.position, bounds)


class Arm:
    """One gene marching in one direction, until it breaks or runs out of room."""

    def __init__(self, gene: str, direction: str, values: list):
        self.gene = gene
        self.direction = direction
        self.values = values
        self.index = 0
        self.consecutive_breaks = 0
        self.last_surviving = None
        self.first_failing = None
        self.failure_mode = ""
        self.outcome = None          # broken | censored_box | censored_extended
        self.n_runs = 0

    @property
    def live(self) -> bool:
        return self.outcome is None

    def next_value(self):
        return self.values[self.index] if self.index < len(self.values) else None

    def record(self, value: float, mode: str) -> None:
        """Fold one evaluated step back into the arm's state."""
        self.n_runs += 1
        self.index += 1
        if mode:
            self.consecutive_breaks += 1
            if self.first_failing is None:
                self.first_failing = value
                self.failure_mode = mode
            if self.consecutive_breaks >= ARGS.break_confirm:
                self.outcome = "broken"
        else:
            # A survivor after a single break clears it: the arm was still
            # flying, and one marginal run should not end the march.
            self.consecutive_breaks = 0
            self.first_failing = None
            self.failure_mode = ""
            self.last_surviving = value
        if self.outcome is None and self.index >= len(self.values):
            self.outcome = ("censored_extended" if ARGS.extend_factor > 1.0
                            else "censored_box")


def build_arms(reference: dict, genes_def) -> list:
    """Two arms per gene, up and down from the reference."""
    arms = []
    for name, lo, hi in genes_def:
        x0 = reference["genes"].get(name)
        if x0 is None:
            continue
        for direction in ("up", "down"):
            values = gene_space.ladder_values(
                name, float(x0), float(lo), float(hi), direction,
                ratio=ARGS.ratio, delta_frac=ARGS.delta_frac,
                max_steps=ARGS.max_steps, extend=ARGS.extend_factor)
            if values:
                arms.append(Arm(name, direction, values))
    return arms


def main():
    timeline = omni.timeline.get_timeline_interface()
    render_sim = not ARGS.headless
    bounds = ARGS.bounds
    controller = ARGS.controller
    genes_def = gene_space.genes_for(controller, sweep_outer=True)

    ref_root = Path(ARGS.reference_root) if ARGS.reference_root else (
        RESULTS / "ablation")
    out_root = Path(ARGS.out_dir) if ARGS.out_dir else (RESULTS / "ablation")

    carb.log_warn("\n" + "=" * 64)
    carb.log_warn(f"[ablation] controller {controller}, "
                  f"{len(genes_def)} genes -> {2 * len(genes_def)} arms")
    carb.log_warn(f"[ablation] ladder ratio {ARGS.ratio}, delta_frac "
                  f"{ARGS.delta_frac}, extend {ARGS.extend_factor}x, "
                  f"confirm {ARGS.break_confirm}")
    carb.log_warn(gene_space.describe_box(controller, True))
    carb.log_warn("=" * 64)

    pg = PegasusInterface()
    pg._world = World(**pg._world_settings)
    world = pg.world
    pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])
    for _ in range(20):
        world.step(render=render_sim)

    slots = ARGS.slots
    layout = task.grid_layout(slots, bounds, pitch_x=ARGS.spacing,
                              pitch_y=ARGS.row_pitch, label="ablation")
    if layout["rows"] > 1 and ARGS.row_pitch < 2.0 * ARGS.goal_radius:
        raise ValueError(
            f"[ablation] row pitch {ARGS.row_pitch:g} m is below 2x the goal "
            f"radius ({2.0 * ARGS.goal_radius:g} m); adjacent rows could be "
            f"sent to the same point.")
    carb.log_warn(f"[ablation] layout {layout['rows']}x{layout['columns']}, "
                  f"pitch {layout['spacing']:.2f} m, max_dx {layout['max_dx']:.2f} m")

    z_min, z_max = bounds[4], bounds[5]
    half, half_y = layout["spacing"] / 2.0, ARGS.row_pitch / 2.0
    goal_rng = np.random.default_rng(ARGS.goal_seed)
    extreme_y = max(abs(y) for _, y in layout["positions"])
    task.check_goal_reachable(extreme_y, ARGS.goal_radius, bounds, label="ablation")

    # One goal direction for every slot — see --fixed-goal.
    fixed_theta = float(goal_rng.uniform(0.0, 2.0 * math.pi)) if ARGS.fixed_goal else None

    results_base = task.results_root(Path(__file__).resolve().parent.parent)
    tmp_files = [str(results_base / f"_ablation_tmp_{i}.npz") for i in range(slots)]

    drones = []
    for i in range(slots):
        spawn_x, spawn_y = layout["positions"][i]
        spawn_pos = [spawn_x, spawn_y, float(ARGS.spawn_altitude)]
        if fixed_theta is not None:
            target = (spawn_x + ARGS.goal_radius * math.cos(fixed_theta),
                      spawn_y + ARGS.goal_radius * math.sin(fixed_theta),
                      ARGS.target_altitude)
        else:
            target = task.offset_target(spawn_pos, ARGS.target_altitude,
                                        ARGS.goal_radius, goal_rng,
                                        max_dx=layout["max_dx"])
        drone_bounds = [spawn_x - half, spawn_x + half,
                        spawn_y - half_y, spawn_y + half_y, z_min, z_max]

        inner = cf.make_controller(controller, {}, tmp_files[i], target,
                                   ARGS.motor_moments[0])
        backend = BoundedBackend(inner, tmp_files[i])
        cfg = MultirotorConfig()
        cfg.backends = [backend]
        cfg.thrust_curve = QuadraticThrustCurve(
            config={"motor_moment": ARGS.motor_moments[0]})
        vehicle = Multirotor(
            f"/World/abl_{i}", ROBOTS["Iris"], i, spawn_pos,
            Rotation.from_euler("XYZ", [0., 0., 0.], degrees=True).as_quat(),
            config=cfg)
        drones.append({"vehicle": vehicle, "backend": backend,
                       "spawn": spawn_pos, "bounds": drone_bounds,
                       "target": target})

    world.reset()
    for _ in range(10):
        world.step(render=render_sim)

    def fly(jobs, mm):
        """Evaluate up to `slots` gain dicts in parallel. Returns metric dicts."""
        for i, job in enumerate(jobs):
            drone = drones[i]
            drone["vehicle"]._thrusters = QuadraticThrustCurve(
                config={"motor_moment": mm})
            cf.apply_gains(drone["backend"].inner, controller, job["gains"], mm)
            drone["backend"].inner.hover_target = np.asarray(drone["target"],
                                                             dtype=float)
            drone["vehicle"].set_world_pose(position=drone["spawn"],
                                            orientation=[0., 0., 0., 1.])
            drone["vehicle"].set_linear_velocity([0., 0., 0.])
            drone["vehicle"].set_angular_velocity([0., 0., 0.])
            drone["backend"].start()

        for _ in range(5):
            world.step(render=render_sim)

        timeline.play()
        n_steps, _ = sim_clock.steps_for_duration(world, ARGS.duration)
        for _ in range(n_steps):
            if not simulation_app.is_running():
                break
            world.step(render=render_sim)
            done = True
            for i in range(len(jobs)):
                if drones[i]["backend"].is_oob:
                    continue
                done = False
                drones[i]["backend"].check_oob(drones[i]["bounds"])
            if done:
                break
        timeline.stop()

        out = []
        for i in range(len(jobs)):
            if not drones[i]["backend"]._stopped:
                drones[i]["backend"].stop()
            out.append(sm.compute_metrics(tmp_files[i], ARGS.duration,
                                          _BOUNDS_DIAG))
            if os.path.isfile(tmp_files[i]):
                os.remove(tmp_files[i])
        return out

    for mm in ARGS.motor_moments:
        ref_path = ref_root / controller / f"mm_{mm:.1e}" / "reference.json"
        if not ref_path.is_file():
            carb.log_warn(f"[ablation] no reference at {ref_path} — skipping "
                          f"km={mm:.1e}. Run evaluations/select_reference.py.")
            continue
        reference = json.loads(ref_path.read_text())
        ref_gains = {k: v for k, v in reference["gains"].items()
                     if k not in ("wn_xy", "zeta_xy")}

        out_dir = out_root / controller / f"mm_{mm:.1e}"
        out_dir.mkdir(parents=True, exist_ok=True)
        carb.log_warn(f"\n[ablation] km={mm:.1e}  reference "
                      + "  ".join(f"{k}={v:g}" for k, v in reference["genes"].items()))

        rows = []

        # ── Calibration: the reference in every slot ─────────────────────────
        # This is the noise floor. Without it a flat response curve and a real
        # one look identical, and the effect being measured is ~7 % of median.
        nominal_effort = float("nan")
        if ARGS.calibration_batch:
            cal = fly([{"gains": ref_gains}] * slots, mm)
            with open(out_dir / "calibration.csv", "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["slot"] + sm.METRIC_KEYS)
                for i, m in enumerate(cal):
                    w.writerow([i] + [sm.format_metric(k, m[k])
                                      for k in sm.METRIC_KEYS])
            eff = np.array([m["effort_ac"] for m in cal])
            eff = eff[np.isfinite(eff)]
            nominal_effort = float(np.median(eff)) if eff.size else float("nan")
            sc = np.array([m["score"] for m in cal])
            sc = sc[np.isfinite(sc)]
            if sc.size:
                spread = (sc.max() - sc.min()) / max(np.median(sc), 1e-12)
                # Reported to 4 significant figures, not 1 decimal percent:
                # the measured spread is ~0.005 % on score, and "0.0 %" would
                # hide the very number the report needs to draw its noise band.
                carb.log_warn(
                    f"[ablation] noise floor over {sc.size} slots: "
                    f"score median {np.median(sc):.6f}, spread {spread:.4%}, "
                    f"effort_ac median {nominal_effort:.4f}")
                if spread > 0.05:
                    carb.log_warn(
                        "[ablation] WARNING: slot spread exceeds 5 % of median "
                        "score. Response curves smaller than this cannot be "
                        "resolved — treat flat panels as uninformative, not flat.")

        # ── The march ───────────────────────────────────────────────────────
        arms = build_arms(reference, genes_def)
        carb.log_warn(f"[ablation] {len(arms)} arms, "
                      f"{sum(len(a.values) for a in arms)} steps queued")

        round_index = 0
        while any(a.live for a in arms):
            live = [a for a in arms if a.live and a.next_value() is not None]
            for a in arms:
                if a.live and a.next_value() is None:
                    a.outcome = ("censored_extended" if ARGS.extend_factor > 1.0
                                 else "censored_box")
            if not live:
                break

            jobs, owners = [], []
            for arm in live[:slots]:
                gains = dict(ref_gains)
                gains.pop(arm.gene, None)
                gains[arm.gene] = arm.next_value()
                # A changed wn_xy/zeta_xy must be re-derived into kp_xy/kd_xy,
                # or the arm would fly the reference's outer loop while
                # recording its own gene value.
                gains.update(gene_space.outer_gains(
                    {**reference["genes"], arm.gene: arm.next_value()}))
                jobs.append({"gains": gains})
                owners.append(arm)

            # Pad with reference replicates: free slots cost nothing and every
            # replicate tightens the noise floor.
            pad = slots - len(jobs)
            for _ in range(pad):
                jobs.append({"gains": ref_gains})
                owners.append(None)

            metrics = fly(jobs, mm)
            round_index += 1

            for job, arm, m in zip(jobs, owners, metrics):
                mode = rb._failed(m, nominal_effort)
                if arm is None:
                    rows.append({"gene": "_reference", "direction": "",
                                 "step": -1, "value": float("nan"),
                                 "post_break": 0, "failed_reason": mode,
                                 "gains": job["gains"], "metrics": m})
                    continue
                value = arm.next_value()
                rows.append({"gene": arm.gene, "direction": arm.direction,
                             "step": arm.index, "value": value,
                             "post_break": 0, "failed_reason": mode,
                             "gains": job["gains"], "metrics": m})
                arm.record(value, mode)

            alive = sum(1 for a in arms if a.live)
            carb.log_warn(f"  round {round_index:>3}: {len(live[:slots])} arms "
                          f"advanced, {pad} replicates, {alive} still live")

        # ── Write ───────────────────────────────────────────────────────────
        gene_names = [g[0] for g in genes_def]
        header = (["gene", "direction", "step", "value", "post_break",
                   "failed_reason", "motor_moment"]
                  + gene_names + sm.CONFIG_KEYS + sm.METRIC_KEYS)
        with open(out_dir / "ablation_archive.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(header)
            for r in rows:
                g = r["gains"]
                w.writerow(
                    [r["gene"], r["direction"], r["step"],
                     "" if not np.isfinite(r["value"]) else f"{r['value']:.6g}",
                     r["post_break"], r["failed_reason"], str(mm)]
                    + [f"{g[n]:.6f}" if n in g else "" for n in gene_names]
                    + [f"{g[k]:.6f}" if k in g else "" for k in sm.CONFIG_KEYS]
                    + [sm.format_metric(k, r["metrics"][k]) for k in sm.METRIC_KEYS])

        with open(out_dir / "limits.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["gene", "direction", "reference", "outcome",
                        "last_surviving", "first_failing", "failure_mode",
                        "n_runs", "reach_lo", "reach_hi"])
            for a in arms:
                lo, hi = reference["gene_box"][a.gene]
                w.writerow([a.gene, a.direction,
                            f"{reference['genes'][a.gene]:.6g}", a.outcome,
                            "" if a.last_surviving is None else f"{a.last_surviving:.6g}",
                            "" if a.first_failing is None else f"{a.first_failing:.6g}",
                            a.failure_mode, a.n_runs, f"{lo:g}", f"{hi:g}"])

        broke = sum(1 for a in arms if a.outcome == "broken")
        carb.log_warn(f"[ablation] km={mm:.1e} done: {len(rows)} runs, "
                      f"{broke}/{len(arms)} arms broke -> {out_dir}")

    for f in tmp_files:
        if os.path.exists(f):
            os.remove(f)
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
