#!/usr/bin/env python3
"""
genetic_sweep.py — NSGA-II-style genetic Pareto search for Eigen, PD and LPV-MPC.

Searches the full gain space of each controller to find Pareto-optimal
(score, effort) configurations that the coarse grid sweeps miss.

Population of N drones simulated in parallel per generation.
Results appended to the same CSV format used by pareto_comparison.py.

Usage:
  isaac_run examples/sweeps/genetic_sweep.py \
      --controller eigen --motor-moments 0.0 1e-7 2.7e-7 4e-7 \
      --pop-size 20 --generations 50 --headless

  isaac_run examples/sweeps/genetic_sweep.py \
      --controller pd --motor-moments 0.0 1e-7 2.7e-7 4e-7 \
      --pop-size 20 --generations 50 --headless
"""
from __future__ import annotations
import argparse, csv, json, math, os, sys
from pathlib import Path
import carb
from isaacsim import SimulationApp

# ── CLI ──────────────────────────────────────────────────────────────────────
def parse_args():
    p = argparse.ArgumentParser(description="Genetic Pareto sweep")
    p.add_argument("--controller", choices=["eigen", "pd", "mpc"], required=True)
    p.add_argument("--motor-moments", nargs="+", type=float,
                   default=[0.0, 1e-7, 2e-7, 2.7e-7, 4e-7])
    p.add_argument("--pop-size", type=int, default=20)
    p.add_argument("--generations", type=int, default=50)
    p.add_argument("--duration", type=float, default=25.0,
                   help="Simulation seconds per run (default: 25.0 to ensure >20s of stable hover)")
    p.add_argument("--target-altitude", type=float, default=1.5)
    # X pitch between neighbouring drones. 3.5 m matches what pd_sweep has
    # always used at 7 drones, and is the pitch the goal geometry was designed
    # around: it leaves max_dx = 1.0 m of lateral freedom. Held FIXED rather
    # than divided across the population — dividing is what collapsed it to
    # 1.40 m at pop 16 and silently changed the task. Populations that no
    # longer fit in one row wrap into additional rows; see task.grid_layout.
    p.add_argument("--spacing", type=float, default=3.5,
                   help="X pitch between drones [m]. Fixed, not divided across "
                        "the population; the layout adds rows instead.")
    p.add_argument("--row-pitch", type=float, default=6.0,
                   help="Y pitch between spawn rows [m]. Must exceed twice the "
                        "goal radius so neighbouring rows cannot collide.")
    p.add_argument("--bounds", nargs=6, type=float,
                   default=[-12., 12., -12., 12., 0., 20.])
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--sweep-outer", action="store_true", default=False,
                   help="Also search the outer position loop (kp_xy, kd_xy, "
                        "and for the MPC its integral gains). Off by default, "
                        "so a run without it reproduces the pinned-outer "
                        "searches exactly.")
    # ── Step-response validation (proves position control, not just hover) ──
    p.add_argument("--validate-every", type=int, default=3,
                   help="Run a step-response validation pass every N generations "
                        "(0 disables). Validated individuals that fail the "
                        "--validation-threshold are constraint-dominated: they "
                        "always lose survivor selection to any individual that "
                        "passed, regardless of hover score/effort.")
    p.add_argument("--validation-radius", type=float, default=2.0,
                   help="Lateral offset (m) of the validation target from the hover point")
    p.add_argument("--validation-threshold", type=float, default=2.0,
                   help="Max stability score during the validation step to count as feasible "
                        "(matches the 'stable' hover threshold)")
    p.add_argument("--validation-seed", type=int, default=123,
                   help="Separate RNG seed for validation target directions, so enabling/"
                        "disabling validation doesn't change the population RNG stream")
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
    # ── Robust tuning: search under inertia MISMATCH, not at nominal ────────
    #
    # Off by default, so a run without it reproduces the nominal searches
    # exactly. With it, every individual is flown once per scale and its
    # fitness is the WORST of those, so the search optimises the failure end
    # rather than the typical case.
    #
    # The scales are the destabilising side of robustness_sweep's own inertia
    # ladder. Its note explains why: the controller computes a torque from its
    # nominal model while the plant has gamma x that, so gamma > 1 makes the
    # commanded torque produce LESS angular acceleration than intended —
    # sluggish but self-limiting — while gamma < 1 produces MORE and the
    # attitude error compounds. gamma = 1.0 is kept as the matched control so a
    # robust run is still directly comparable to a nominal one.
    p.add_argument("--robust-inertia", nargs="*", type=float, default=None,
                   metavar="SCALE",
                   help="Evaluate each individual at these inertia scale "
                        "factors and take the worst as its fitness. The plant "
                        "is scaled while the controller's model is held at "
                        "nominal, so this is genuine model mismatch. Pass with "
                        "no values for the default ladder 1.0 0.5 0.25.")
    # ── transport lag ────────────────────────────────────────────────────────
    # Defaults 0 so every existing invocation reproduces exactly. Set them and the
    # front means something for PX4; leave them and it is a lag-free front, which is
    # what produced gains that limit-cycled on the real stack (FINDINGS section 5).
    p.add_argument("--pos-delay-ms", type=float, default=0.0,
                   help="Transport delay on position/velocity reaching the controller. "
                        "PX4+Pegasus measures ~12-24 ms end to end; a real GPS-aided "
                        "vehicle is 50-200 ms.")
    p.add_argument("--att-delay-ms", type=float, default=0.0,
                   help="Transport delay on attitude/rate. This is the channel the "
                        "coning mode lives in, so it matters far more than position: "
                        "the eigen law broke between 10 and 15 ms of it at 13 rad/s.")
    p.add_argument("--pos-tau-ms", type=float, default=0.0,
                   help="First-order lag on position, ON TOP of the pure delay. Delay "
                        "adds phase without attenuation and is what destabilises; a lag "
                        "attenuates. They are different failure modes, so separate knobs.")
    p.add_argument("--att-tau-ms", type=float, default=0.0)
    p.add_argument("--torque-max", type=float, default=3.843,
                   help="Per-axis torque ceiling [N.m] the controller saturates at, "
                        "matching PX4's MC_EIG_TRQ_MAX. PX4 clips normalized torque to "
                        "+/-1 and this loop historically did not clip at all, so without "
                        "it the search optimises against authority the real stack cannot "
                        "deliver. 0 disables (the old unlimited behaviour).")
    p.add_argument("--rotor-tau-ms", type=float, default=None,
                   help="Rotor/ESC spin-up time constant, applied in the shared plant "
                        "(quadratic_thrust_curve.py). Distinct from the delays above: "
                        "this is a POLE in the actuator, not a delay on the state, and no "
                        "gain choice removes it. Measure with measure_motor_lag.py.")
    p.add_argument("--desaturation", choices=("uniform", "collective"),
                   default="uniform",
                   help="What the mixer gives up when a rotor saturates. 'uniform' scales every rotor down together, preserving the thrust/torque ratio and collapsing the collective when one rotor pins; 'collective' holds the mean and shrinks only the moments. 'uniform' is the default and the historical behaviour - every archived sweep and tuned gain set was flown against it, so changing it moves the plant.")
    p.add_argument("--headless", action="store_true", default=False)
    p.add_argument("--no-headless", dest="headless", action="store_false")
    return p.parse_args()


# Default ladder for --robust-inertia when given no explicit values.
DEFAULT_ROBUST_SCALES = [1.0, 0.5, 0.25]

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
from pegasus.simulator.logic.thrusters import quadratic_thrust_curve as rotor_curve

# parse_args() ran above, before the Pegasus imports could exist (they require
# SimulationApp first), so the default is resolved here. It tracks the SAME constant
# sim_px4.py uses: the GA searches the Python controller and gain_search.py
# verifies on PX4, and if those two plants disagree the comparison is meaningless.
# --rotor-tau-ms 0 stays 0 - an explicit request for the instantaneous model.
if ARGS.rotor_tau_ms is None:
    ARGS.rotor_tau_ms = 1000.0 * rotor_curve.ROTOR_TAU_PLACEHOLDER
from pegasus.simulator.logic.backends import Backend
from pegasus.simulator.logic.state import State

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "utils"))
from eigen_controller import EigenController
from cascaded_pd_controller import CascadedPDController
from lpv_mpc_controller import LPVMPCController
import sweep_metrics as sm
import sim_clock
import sweep_task as task
import robustness as rb

# Resolved once so every code path sees a list (empty == nominal search).
ROBUST_SCALES: list = []
if ARGS.robust_inertia is not None:
    ROBUST_SCALES = [float(g) for g in ARGS.robust_inertia] or list(DEFAULT_ROBUST_SCALES)
    if any(g <= 0.0 for g in ROBUST_SCALES):
        raise ValueError(f"--robust-inertia scales must be positive: {ROBUST_SCALES}")

# Per-scale score columns appended to the archive when searching robustly, so
# the worst-case fitness can always be decomposed back into what produced it.
ROBUST_KEYS: list = [f"score_g{g:g}" for g in ROBUST_SCALES]

# Resolved after the late imports so every code path below sees numbers. The
# task is a ground start followed by a translation to a laterally offset goal.
ARGS.spawn_altitude = task.resolve_spawn_altitude(ARGS.spawn_altitude)
if getattr(ARGS, "goal_radius", None) is None:
    ARGS.goal_radius = task.DEFAULT_GOAL_RADIUS

_BOUNDS_DIAG = sm.bounds_diagonal(ARGS.bounds)

# ── Gene definitions ─────────────────────────────────────────────────────────
# Inner-loop gains, always swept. The outer position loop is appended by
# --sweep-outer (see OUTER_GENES); without that flag these lists are the whole
# gene set and a run reproduces the pinned-outer searches exactly.
# Each gene: (name, low, high)
#
# Box sizing note: the earlier boxes (wn ≤ 20, b ≤ 15) were too small and the
# GA ran into the walls — the best individuals at km=4e-7 pinned wn_mdyn at
# exactly 20.000. They also could not express the hypothesis under test, since
# b_mdyn is conjectured to track the spin rate and the logged spin rate reaches
# ~30 rad/s, outside the old b range entirely. Both upper bounds now clear the
# observed spin rate with room to spare.
#
# Kd_att's LOWER bound was the next wall to fall. Unlike the two above it had
# no sizing rationale — it arrived at 1.0 in a general "save, reordered all the
# sweeps" commit — and the outer-loop searches ran straight into it at every
# motor moment. Measured on the km=4e-7 and 6e-7 archives (496 evals each,
# 6-gene --sweep-outer): median Kd_att 1.0202 and 1.0461 against a floor of
# 1.0, with 53 % and 59 % of stable individuals sitting within 1 % of it, and
# 6/6 and 14/21 of the Pareto-front points ON it. The pre-fix runs pin the same
# way (76.5 % at km=0, 47.7 % at 2.7e-7), so this is not an artefact of the A_r
# sign fix. Best score by Kd_att bin at km=4e-7 is monotone toward the floor:
#
#     [1.0, 1.5)  0.0202     [3, 6)    0.1551
#     [1.5, 3)    0.0219     [6, 12)   0.1228
#                            [12, 30)  0.1116
#
# so 1.0000 was the edge of the box, not an optimum. There is no physical floor
# either: Kd_att multiplies a 250 Hz finite-difference attitude-error rate, and
# small is the well-behaved end of that — the noise-amplifying end is the top.
# 0.05 leaves 20x of headroom below where the search kept pressing.
# The NSGA-II second objective. sweep_metrics.py:48 is explicit - "Use effort_ac_ses
# as the optimisation target; effort is retained unchanged so old and new archives stay
# comparable" - but this search was still sorting on `effort`, which never got switched
# over. `effort` is dominated by effort_dc (the collective/hover channel): measured across
# the 121-member converged elite of the 2026-08-16 lagged run, effort spanned 0.72% while
# effort_ac spanned 250%. Since the attitude gains being searched act ONLY through
# differential thrust, the channel that responds to them was contributing under 1% of the
# number being optimised, and the Pareto front was effectively single-objective.
# effort_ac_ses additionally folds in saturation (effort_ac x (1 + SAT_ALPHA x sat_frac)),
# which is the control-authority term the front is supposed to trade against.
EFFORT_KEY = "effort_ac_ses"

EIGEN_GENES = [
    ("wn_mdyn", 0.5, 60.0),
    ("b_mdyn",  0.0, 40.0),
    ("Kp_att",  0.1, 5.0),
    ("Kd_att",  -0.1, 0.1),
]

PD_GENES = [
    ("Kp_att",      1.0, 60.0),
    ("Kd_att",      0.05, 3.0),
    ("Kd_yaw_rate", 0.0, 2.0),
]

# The MPC's cost weights and horizon. These are hyperparameters the solver
# never touches — it optimises the control INPUT given this cost, exactly as
# Kp_att is a free parameter for the PD controller — so they need searching
# like any other gain.
#
# Bounds come from the widened grid in results/mpc_widen, which was run because
# the original 3x3 box (q 5-20, r 0.01-0.05) put BOTH objectives at the same
# corner at all six motor moments: the textbook signature of an optimum outside
# the searched region. Re-running with q 0.5-5 and r 0.05-1.0 cut effort_ac_ses
# from 47-69 down to 0.12-0.14 — into line with Eigen and PD, whose best sit
# near 0.14 — and flattened the spin dependence from an 81 % degradation across
# the ladder to 4.5 %. Both of the MPC's apparent weaknesses were artefacts of
# searching the wrong region.
#
# q is bracketed: q=5 was the MINIMUM of the old box and the MAXIMUM of the
# widened one and won from both sides, so the optimum is genuinely interior
# around 2-5; the range below keeps headroom either side. r is flat above ~0.25
# (0.2242 / 0.2252 / 0.2268 at r = 1.0 / 0.5 / 0.25), so its upper bound is
# generous rather than tight.
#
# N has never been searched at all — no grid has ever varied it — which is the
# main reason this GA is worth running beyond refining q/r. It is a horizon
# length in steps, carried as a float gene and rounded at apply time, so
# crossover and mutation need no special integer handling.
#
# ``s_weight`` is deliberately NOT a gene. LPVMPCController accepts it and
# stores it (self.s_weight = float(s_weight)) but never reads it again: the
# cost is built from q_weight and r_weight alone. Searching it would spend a
# quarter of the population's dimensionality on a parameter with no effect on
# the plant, and would produce an archive column implying it had been tuned.
# If it is meant to be a slew-rate or terminal cost, wiring it into the cost is
# a change to the controller, not to this search.
MPC_GENES = [
    ("mpc_q", 0.5, 10.0),
    ("mpc_r", 0.05, 2.0),
    ("mpc_N", 5.0, 25.0),
]


# ── Outer position-loop genes, appended by --sweep-outer ────────────────────
#
# Why these exist: measured over 2135 archived runs, 99-100 % of the Eigen and
# Cascaded PD score is the takeoff-and-translate transient (median
# score_transient 1.10 and 1.03 against settled tails of 0.0076 and 0.0041),
# and that transient is governed by the outer loop — which no sweep had ever
# varied. The inner gains show almost no gradient as a direct result: Spearman
# rho of wn_mdyn against score is -0.001 across a 45x range, and none of PD's
# three genes exceeds |rho| = 0.13.
#
# Box sizing, from the second-order reading of the position loop
# (wn = sqrt(kp), zeta = kd / (2*sqrt(kp))): the pinned values kp=0.50,
# kd=1.0 give wn = 0.71 rad/s and zeta = 0.71. The ranges below span
# wn = 0.5-2.83 rad/s, i.e. up to 4x the current bandwidth. The upper end is
# where the outer loop starts to approach the inner loop's own bandwidth and
# the cascade separation (currently 30-50x) stops holding — which is exactly
# the region worth mapping.
#
# kp_z/kd_z are NOT included by default. Altitude is 0.15 m of a 0.97 m
# transient and 0.0000 m of the settled tail for Eigen and PD, so sweeping it
# doubles the dimensionality to chase ~2 % of the objective. The LPV-MPC is the
# exception — it carries a 0.23 m altitude standing error — so its list adds
# the integral terms that exist to remove exactly that.
# Parameterised as (natural frequency, damping ratio), not (kp, kd).
#
# kp and kd are not independent: the loop they define has wn = sqrt(kp) and
# zeta = kd / (2*sqrt(kp)), so sampling them independently varies damping as a
# side effect of varying bandwidth. Measured on an 8-point LHS over
# kp in [0.25, 8], kd in [0.5, 8]: kp=6.83, kd=1.19 lands at zeta=0.23 and
# kp=6.43, kd=0.95 at zeta=0.19 — badly underdamped loops that are not "faster",
# just worse, and every sampled point scored 1.3-32.8 against 0.13 for the
# pinned default. The good region is a curved ridge in (kp, kd), and SBX
# crossover and polynomial mutation are axis-aligned operators that search a
# box well and a ridge badly.
#
# In (wn, zeta) the good region IS a box, so the same operators become
# effective and the two effects can be attributed separately. kp = wn**2 and
# kd = 2*zeta*wn are derived at apply time, and both the genes and the derived
# gains are recorded in the archive.
#
# The pinned default sits at wn = 0.707 rad/s, zeta = 0.707. The box spans
# 0.5x to 2.8x that bandwidth; the upper end is where the outer loop starts to
# approach the inner loop's own, which is the region worth mapping.
#
# The eigen row's wn_xy ceiling went 2.0 -> 3.0 for the same reason: it was a
# wall the search was pressing against, not a bound it respected. At km=6e-7
# the knee sits at exactly 2.0000 and 7/21 Pareto points are on the ceiling
# (pre-fix: 6/8 at km=0 and 5/9 at 2.7e-7). It is NOT pinned at km=4e-7 post-fix
# — knee 1.6284, 2/6 on the wall — which is the interesting part: the faster the
# spin, the more outer bandwidth the search wants, so the ceiling was censoring
# exactly the high-spin end this study is about.
#
# The original comment below sized the top of the box as "where the outer loop
# starts to approach the inner loop's own bandwidth and the cascade separation
# stops holding". That reasoning still bounds it, just not at 2.0: the searched
# wn_mdyn lands near 27 at both moments, so wn_xy = 3.0 is a 9x separation —
# comfortably inside the 5-10x a cascade needs, where 2.0 was a 13x that gave
# the outer loop no room to be the thing that moves the score.
#
# Only the eigen row moves. The pd and mpc boxes have no equivalent evidence
# behind them — no wall-pinning measurement of their own — and widening them on
# the strength of an eigen archive would change their searches for no reason.
OUTER_GENES = {
    "eigen": [
        ("wn_xy", 0.35, 3.0),
        ("zeta_xy", 0.5, 1.3),
    ],
    "pd": [
        ("wn_xy", 0.35, 2.0),
        ("zeta_xy", 0.5, 1.3),
    ],
    "mpc": [
        ("wn_xy", 0.35, 2.0),
        ("zeta_xy", 0.5, 1.3),
        ("ki_xy", 0.0, 1.0),
        ("ki_z", 0.0, 8.0),
    ],
}

# The pinned operating point in gene coordinates. Generation 0 is seeded with
# it so the population always contains one individual known to fly. With the
# outer loop free the failure rate is far higher than in the inner-only search,
# and a generation 0 where everything crashes gives NSGA-II nothing to select
# on — the search then wanders instead of converging.
OUTER_SEED = {
    "wn_xy": math.sqrt(0.50),                     # 0.7071, from kp = 0.50
    "zeta_xy": 1.0 / (2.0 * math.sqrt(0.50)),     # 0.7071, from kd = 1.00
    "ki_xy": 0.05,
    "ki_z": 2.0,
}

# ── OOB Backend (shared) ────────────────────────────────────────────────────
class BoundedBackend(Backend):
    def __init__(self, inner, results_file: str, lag=None):
        super().__init__(config=None)
        self.inner = inner
        # robustness.SensorLag, or None for the original lag-free behaviour. Separate
        # position and attitude channels, because a real vehicle's are not remotely alike:
        # GPS-like 50-200 ms against IMU-like 2-10 ms.
        self.lag = lag
        self.results_file = results_file
        self.total_time = 0.0
        self._stopped = False
        self.position = np.zeros(3)
        self.monitor = task.FlightMonitor(
            ARGS.target_altitude, label="genetic", log=carb.log_warn)


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
        if self._stopped: return
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
                # The outer-loop configuration actually flown, read off the
                # live controller. This sweep DELETES the npz after scoring,
                # so without it the operating point of a genetic run is not
                # recoverable from anywhere once the run is over.
                data["config_json"] = np.array(
                    json.dumps(sm.controller_config(self.inner)))
                np.savez(self.results_file, **data)
            except Exception: pass

    def reset(self): self.inner.reset()
    def update_sensor(self, st, d): 
        if not self.is_oob: self.inner.update_sensor(st, d)
    def update_graphical_sensor(self, st, d):
        if not self.is_oob: self.inner.update_graphical_sensor(st, d)
    def update_state(self, state: State):
        # Truth first and unconditionally - the monitor scores against it, and must not
        # inherit whatever perturbation the controller is being fed.
        self.position = state.position.copy()
        if self.is_oob:
            return
        # SensorLag is the reason a front found here can mean anything for PX4. This loop
        # reads exact state in-process; PX4 carries ~12-24 ms to the controller. FINDINGS
        # section 5 measured a GA front point limit-cycling on PX4 from the instant of
        # takeoff and concluded "either tune on the PX4 stack directly, or add a
        # transport-delay model to the Pegasus sweeps so the fronts mean something".
        # This is that second option; it was never implemented, and rank 3 of the front it
        # produced departed in 12 s on the real stack.
        self.inner.update_state(self.lag.apply(state) if self.lag is not None else state)
    def input_reference(self):
        return [0.,0.,0.,0.] if self.is_oob else self.inner.input_reference()
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




# ── Scoring ──────────────────────────────────────────────────────────────────
# Implementation lives in examples/utils/sweep_metrics.py so that this sweep,
# pd_sweep.py, eigen_stability_sweep.py and eigen_ab_explore.py cannot drift
# apart. See that module for the effort DC/AC split and the spin-rate metrics.
def _score_from_npz(path, T):
    return sm.compute_metrics(path, T, _BOUNDS_DIAG)


def _as_cost(x) -> float:
    """A non-finite objective is a failure, and a failure is the worst case.

    max() over a list containing NaN returns NaN in Python, which would let a
    crashed pass silently dominate selection as an unordered value rather than
    as the worst one. Mapping non-finite to +inf keeps the ordering total.
    """
    try:
        v = float(x)
    except (TypeError, ValueError):
        return float("inf")
    return v if np.isfinite(v) else float("inf")


def _worst_case(per_scale: list, scales: list) -> dict:
    """Collapse one individual's per-scale metrics into a single fitness.

    score and effort are each taken as the worst over the scales — the standard
    robust-optimisation objective, and a conservative bound even when the two
    worsts occur at different scales. Every OTHER metric column is carried from
    the worst-SCORING pass, so the row describes one real flight rather than a
    per-column mixture that never happened.

    The per-scale scores are kept alongside as score_g<scale>, so a worst-case
    number can always be traced back to which mismatch produced it.
    """
    costs = [_as_cost(m["score"]) for m in per_scale]
    worst_i = int(np.argmax(costs))

    out = dict(per_scale[worst_i])
    out["score"] = max(costs)
    out["effort"] = max(_as_cost(m["effort"]) for m in per_scale)
    out[EFFORT_KEY] = max(_as_cost(m[EFFORT_KEY]) for m in per_scale)
    for g, m in zip(scales, per_scale):
        out[f"score_g{g:g}"] = m["score"]
    return out


# ── Step-response validation target ──────────────────────────────────────────
def _validation_target(spawn_pos, altitude, radius, rng, max_dx=None):
    """Second, independently drawn offset goal for the validation pass.

    Same builder as the main pass (sweep_task.offset_target), drawn from a
    separate RNG stream, so validation asks "does this individual generalise to
    a different heading" rather than "does it fly at all" — which the main pass
    now answers, since every run is a takeoff plus a translation."""
    return task.offset_target(spawn_pos, altitude, radius, rng, max_dx=max_dx)


# ── NSGA-II helpers ──────────────────────────────────────────────────────────
def _dominates(i, j, scores, efforts, violations):
    """Deb's constrained-domination rule. violations[k] == 0 means individual k
    is feasible (passed step-response validation, or wasn't validated this
    generation — treated as feasible by default). A feasible individual always
    dominates an infeasible one; between two infeasible individuals, the one
    with the smaller constraint violation dominates; otherwise falls back to
    plain (score, effort) Pareto dominance. When violations are all 0 (the
    common case — non-validation generations) this is identical to the
    original unconstrained check."""
    vi, vj = violations[i], violations[j]
    if vi != vj:
        return vi < vj
    return (scores[i] <= scores[j] and efforts[i] <= efforts[j] and
            (scores[i] < scores[j] or efforts[i] < efforts[j]))


def _fast_nondominated_sort(scores, efforts, violations=None):
    """Return list of fronts (each front is list of indices).
    violations: optional per-individual constraint-violation array (0 =
    feasible) for periodic step-response validation — see _dominates()."""
    n = len(scores)
    if violations is None:
        violations = np.zeros(n)
    S = [[] for _ in range(n)]  # dominated set
    n_dom = [0]*n               # domination count
    fronts = [[]]
    for p in range(n):
        for q in range(n):
            if p == q: continue
            if _dominates(p, q, scores, efforts, violations):
                S[p].append(q)
            elif _dominates(q, p, scores, efforts, violations):
                n_dom[p] += 1
        if n_dom[p] == 0:
            fronts[0].append(p)
    i = 0
    while fronts[i]:
        next_front = []
        for p in fronts[i]:
            for q in S[p]:
                n_dom[q] -= 1
                if n_dom[q] == 0:
                    next_front.append(q)
        i += 1
        fronts.append(next_front)
    return [f for f in fronts if f]

def _crowding_distance(front_indices, scores, efforts):
    n = len(front_indices)
    if n <= 2: return [float("inf")]*n
    dist = [0.0]*n
    for vals in [scores, efforts]:
        obj = [vals[i] for i in front_indices]
        order = sorted(range(n), key=lambda x: obj[x])
        dist[order[0]] = dist[order[-1]] = float("inf")
        span = obj[order[-1]] - obj[order[0]]
        if span < 1e-12: continue
        for k in range(1, n-1):
            dist[order[k]] += (obj[order[k+1]] - obj[order[k-1]]) / span
    return dist

def get_rank_and_crowding(fronts, scores, efforts):
    n = len(scores)
    ranks = np.zeros(n)
    crowding = np.zeros(n)
    for r, front in enumerate(fronts):
        for idx in front:
            ranks[idx] = r
        cd = _crowding_distance(front, scores, efforts)
        for i, idx in enumerate(front):
            crowding[idx] = cd[i]
    return ranks, crowding

def _tournament_selection(ranks, crowding, rng, k=2):
    n = len(ranks)
    p1 = rng.integers(n)
    for _ in range(k-1):
        p2 = rng.integers(n)
        if ranks[p2] < ranks[p1]:
            p1 = p2
        elif ranks[p2] == ranks[p1] and crowding[p2] > crowding[p1]:
            p1 = p2
    return p1

def _sbx_crossover(p1, p2, genes, rng, eta=2.0):
    child = np.empty(len(genes))
    for i, (_, lo, hi) in enumerate(genes):
        if rng.random() < 0.5:
            child[i] = p1[i]
        else:
            u = rng.random()
            if u <= 0.5:
                beta = (2*u)**(1/(eta+1))
            else:
                beta = (1/(2*(1-u)))**(1/(eta+1))
            child[i] = np.clip(0.5*((1+beta)*p1[i]+(1-beta)*p2[i]), lo, hi)
    return child

def _polynomial_mutation(ind, genes, rng, pm=0.2, eta_m=20.0):
    out = ind.copy()
    for i, (_, lo, hi) in enumerate(genes):
        if rng.random() < pm:
            u = rng.random()
            if u < 0.5:
                delta = (2*u)**(1/(eta_m+1)) - 1
            else:
                delta = 1 - (2*(1-u))**(1/(eta_m+1))
            out[i] = np.clip(ind[i] + delta*(hi-lo), lo, hi)
    return out


def _lhs_population(genes_def, n, seed):
    """Latin Hypercube initial population over the gene box.

    Stratifies each gene into n bins and places exactly one sample per bin
    (guaranteed per-gene coverage), which independent uniform sampling cannot
    promise at small population sizes — with only a handful of individuals in
    3-4D, uniform draws routinely leave whole regions of the gain space
    unrepresented in generation 0. LHS is preferred over the Halton sequence
    used by eigen_ab_explore.py here specifically because it stays well-spread
    at tiny n, where Halton shows low-count correlation artefacts.

    No uniform fallback: if scipy.stats.qmc is missing this raises loudly, so a
    degraded init can never run silently under the guise of LHS.
    """
    lo = np.array([g[1] for g in genes_def], dtype=float)
    hi = np.array([g[2] for g in genes_def], dtype=float)
    d = len(genes_def)
    from scipy.stats import qmc
    unit = qmc.LatinHypercube(d=d, seed=seed).random(n)
    return lo + unit * (hi - lo)

# ── Controller factories ─────────────────────────────────────────────────────
#
# Genes are looked up BY NAME rather than by position. The gene list is no
# longer fixed per controller — --sweep-outer appends the outer position gains
# — and positional indexing silently reinterprets every gene the moment the
# list changes length. A name lookup either finds the gene or falls back to the
# documented default, and cannot quietly apply b_mdyn as a Kp_att.

def _gene_map(genes_def, genes_vec) -> dict:
    """{gene name: value} for one individual."""
    return {g[0]: float(v) for g, v in zip(genes_def, genes_vec)}


def _outer_gains(g: dict) -> dict:
    """Derive the physical outer gains from whichever genes are present.

    (wn_xy, zeta_xy) -> kp = wn**2, kd = 2*zeta*wn. Passing kp_xy/kd_xy
    directly is still honoured, so an explicit box in physical coordinates
    remains usable without touching this code.
    """
    out = {k: v for k, v in g.items()
           if k in ("kp_xy", "kd_xy", "kp_z", "kd_z", "ki_xy", "ki_z",
                    "int_max_xy", "int_max_z", "max_tilt_deg", "kp_rate")}
    if "wn_xy" in g:
        wn = g["wn_xy"]
        out["kp_xy"] = wn * wn
        out["kd_xy"] = 2.0 * g.get("zeta_xy", 0.7071) * wn
    return out


def _outer_kwargs(g: dict, controller: str) -> dict:
    """Outer-loop constructor arguments for whichever outer genes are present.

    Absent genes are simply omitted, so each controller keeps its own defaults
    and a run with --sweep-outer off is byte-identical to one before this
    existed. Names are translated from the canonical CONFIG_KEYS spelling to
    each controller's own (CascadedPDController says Kp_xy where the others
    say kp_pos).
    """
    g = _outer_gains(g)
    if controller == "pd":
        out = {}
        if "kp_xy" in g:
            out["Kp_xy"] = (g["kp_xy"], g["kp_xy"])
        if "kd_xy" in g:
            out["Kd_xy"] = (g["kd_xy"], g["kd_xy"])
        if "kp_z" in g:
            out["Kp_z"] = g["kp_z"]
        if "kd_z" in g:
            out["Kd_z"] = g["kd_z"]
        if "max_tilt_deg" in g:
            out["max_tilt_deg"] = g["max_tilt_deg"]
        return out
    out = {}
    for gene, arg in (("kp_xy", "kp_pos"), ("kd_xy", "kd_pos"),
                      ("kp_z", "kp_z"), ("kd_z", "kd_z")):
        if gene in g:
            out[arg] = g[gene]
    if controller == "mpc":
        for gene, arg in (("ki_xy", "ki_pos"), ("ki_z", "ki_z"),
                          ("kp_rate", "kp_rate")):
            if gene in g:
                out[arg] = g[gene]
    return out


def _make_eigen(g, target, results_file, mm):
    return EigenController(
        target_position=target, results_file=results_file,
        wn_mdyn=g["wn_mdyn"], b_mdyn=g["b_mdyn"],
        Kp_att=g["Kp_att"], Kd_att=g["Kd_att"],
        km=mm, **_outer_kwargs(g, "eigen"),
        # PX4 saturates every torque axis at MC_EIG_TRQ_MAX; this loop historically did
        # not clip at all. 0 restores that, for reproducing archived fronts.
        torque_max=(ARGS.torque_max if ARGS.torque_max > 0 else None),
    )

def _make_pd(g, target, results_file):
    return CascadedPDController(
        target_position=target, results_file=results_file,
        Kp_att=(g["Kp_att"], g["Kp_att"]),
        Kd_att=(g["Kd_att"], g["Kd_att"]),
        Kd_yaw_rate=g["Kd_yaw_rate"],
        **_outer_kwargs(g, "pd"),
    )

def _make_mpc(g, target, results_file, mm):
    return LPVMPCController(
        target_position=target, results_file=results_file,
        q_weight=g["mpc_q"], r_weight=g["mpc_r"],
        N=int(round(g["mpc_N"])), km=mm, **_outer_kwargs(g, "mpc"),
    )


def _apply_outer(inner, g: dict, controller: str) -> None:
    """Set whichever outer gains are being swept, in place."""
    g = _outer_gains(g)
    if controller == "pd":
        if "kp_xy" in g:
            inner.Kp_xy = np.array([g["kp_xy"], g["kp_xy"]], dtype=float)
        if "kd_xy" in g:
            inner.Kd_xy = np.array([g["kd_xy"], g["kd_xy"]], dtype=float)
        if "kp_z" in g:
            inner.Kp_z = g["kp_z"]
        if "kd_z" in g:
            inner.Kd_z = g["kd_z"]
        if "max_tilt_deg" in g:
            inner.max_tilt = np.deg2rad(g["max_tilt_deg"])
        return
    for gene, attr in (("kp_xy", "kp_pos"), ("kd_xy", "kd_pos"),
                       ("kp_z", "kp_z"), ("kd_z", "kd_z"),
                       ("ki_xy", "ki_pos"), ("ki_z", "ki_z"),
                       ("kp_rate", "kp_rate")):
        if gene in g and hasattr(inner, attr):
            setattr(inner, attr, g[gene])


def _apply_genes_mpc(backend, g, mm):
    # Safe to set in place: LPVMPCController rebuilds its OSQP problem from
    # self.N/q_weight/r_weight on every update() call, so a changed horizon
    # takes effect on the next step with no reconstruction.
    backend.inner.q_weight = g["mpc_q"]
    backend.inner.r_weight = g["mpc_r"]
    backend.inner.N        = int(round(g["mpc_N"]))
    backend.inner.km       = mm
    _apply_outer(backend.inner, g, "mpc")


def _apply_genes_eigen(backend, g, mm):
    backend.inner.wn_mdyn = g["wn_mdyn"]
    backend.inner.b_mdyn  = g["b_mdyn"]
    backend.inner.Kp_att  = g["Kp_att"]
    backend.inner.Kd_att  = g["Kd_att"]
    backend.inner.km      = mm
    _apply_outer(backend.inner, g, "eigen")

def _apply_genes_pd(backend, g):
    backend.inner.Kp_att = np.array([g["Kp_att"], g["Kp_att"]], dtype=float)
    backend.inner.Kd_att = np.array([g["Kd_att"], g["Kd_att"]], dtype=float)
    backend.inner.Kd_yaw_rate = g["Kd_yaw_rate"]
    _apply_outer(backend.inner, g, "pd")

# ── CSV output ───────────────────────────────────────────────────────────────
def _save_pareto_csv(archive, genes_def, out_dir: Path, mm: float, config: dict = None):
    s_path = out_dir / "scores.csv"
    e_path = out_dir / "effort.csv"
    gene_names = [g[0] for g in genes_def]

    with open(str(s_path), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["idx", "score"])
        for i, (gen, genes, metrics) in enumerate(archive):
            sc = metrics["score"]
            w.writerow([i, f"{sc:.6f}" if np.isfinite(sc) else "inf"])

    with open(str(e_path), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["idx", EFFORT_KEY])
        for i, (gen, genes, metrics) in enumerate(archive):
            ef = metrics[EFFORT_KEY]
            w.writerow([i, f"{ef:.6f}" if np.isfinite(ef) else "inf"])

    detail_path = out_dir / "genetic_archive.csv"
    # Column set comes from sweep_metrics.METRIC_KEYS so every sweep's archive
    # carries the same metrics in the same order — including the effort DC/AC
    # split and the r_ss/r_drift spin-rate columns.
    # ROBUST_KEYS is empty for a nominal search, so the column set is unchanged
    # there; in a robust search it appends score_g<scale> per scale, which is
    # what lets a worst-case fitness be decomposed back into the mismatch that
    # produced it.
    metric_keys = ["motor_moment"] + sm.METRIC_KEYS + ROBUST_KEYS

    # The outer-loop configuration every row was flown at, written whether it
    # was swept or pinned. This sweep DELETES its npz after scoring, so the
    # config_json embedded in the telemetry does not survive — the CSV is the
    # only place a genetic run's operating point can be recovered from. A
    # pinned sweep writing its defaults here is what makes the archive
    # self-describing rather than dependent on the source at that commit.
    # Pinned values come from a live controller; swept ones are derived from
    # each row's own genes and OVERRIDE the snapshot.
    #
    # The snapshot alone is only correct while the outer loop is pinned. Once
    # it is swept, one controller's state at archive-writing time is whatever
    # the last-evaluated individual left behind — so every row would record the
    # same wrong gains. Measured before this fix: the individual seeded at
    # exactly kp_xy=0.50, kd_xy=1.00 was archived as 0.4348 / 1.6343.
    config = config or {}
    base = {k: config[k] for k in sm.CONFIG_KEYS if k in config}

    with open(str(detail_path), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["gen"] + gene_names + sm.CONFIG_KEYS + metric_keys)
        for gen, genes, metrics in archive:
            per_row = dict(base)
            per_row.update(_outer_gains(_gene_map(genes_def, genes)))
            config_row = [sm.format_metric(k, per_row[k]) if k in per_row else ""
                          for k in sm.CONFIG_KEYS]
            row = [str(gen)] + [f"{g:.6f}" for g in genes] + config_row
            for k in metric_keys:
                if k == "motor_moment":
                    # Raw repr, NOT "%.6f" — motor moments live in the 1e-7-ish
                    # range and "%.6f" silently rounds anything under ~5e-7 to
                    # "0.000000", making every non-zero km indistinguishable
                    # from km=0 in the archive. str() matches how the grid
                    # sweeps' save_grid_archive() already write this column.
                    row.append(str(mm))
                    continue
                # .get, not [k]: step_score/step_effort only exist on rows that
                # went through a validation generation, and the per-scale keys
                # only on rows from a robust search.
                row.append(sm.format_metric(k, metrics.get(k, float("nan"))))
            w.writerow(row)
    carb.log_warn(f"[genetic] archive CSV → {detail_path}")

# ── Main ─────────────────────────────────────────────────────────────────────
def main():
    timeline = omni.timeline.get_timeline_interface()
    render_sim = not ARGS.headless
    bounds = ARGS.bounds
    rng = np.random.default_rng(ARGS.seed)

    is_eigen = ARGS.controller == "eigen"
    is_mpc = ARGS.controller == "mpc"
    genes_def = MPC_GENES if is_mpc else (EIGEN_GENES if is_eigen else PD_GENES)
    if ARGS.sweep_outer:
        genes_def = genes_def + OUTER_GENES[ARGS.controller]
    n_genes = len(genes_def)
    pop_size = ARGS.pop_size
    n_gen = ARGS.generations
    ctrl_name = "LPV-MPC" if is_mpc else ("Eigen" if is_eigen else "Cascaded PD")

    base_dir = Path(__file__).resolve().parent.parent
    sweep_name = ("genetic_mpc_sweep" if is_mpc else
                  "genetic_eigen_sweep" if is_eigen else "genetic_pd_sweep")

    pg = PegasusInterface()
    pg._world = World(**pg._world_settings)
    world = pg.world
    pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])
    for _ in range(20):
        world.step(render=render_sim)

    # ── Spawn layout: rows, not one long line ───────────────────────────────
    #
    # A single row divides the usable width by pop_size-1, and past ~15 drones
    # in the default 24 m arena that falls below 2 x SLICE_CLEARANCE. At
    # pop_size=16 it produced spacing = 21/15 = 1.40 m and therefore
    # max_dx = max(0, 0.70 - 0.75) = 0 — so offset_target put the whole goal
    # radius into Y and EVERY genetic run ever performed flew a pure +/-Y
    # translation, while pd_sweep (7 drones, spacing 3.50) and the robustness
    # sweeps (8 drones, spacing 3.00) flew the uniform-on-the-circle task this
    # sweep advertises. select_pareto_top pools all of them into one front.
    #
    # A grid keeps the X pitch fixed regardless of population, so the task no
    # longer depends on how many drones were packed in, and grid_layout raises
    # rather than degrading if the pitch is ever too tight to be safe.
    z_min, z_max = bounds[4], bounds[5]
    spawn_z = float(ARGS.spawn_altitude)
    layout = task.grid_layout(pop_size, bounds, pitch_x=ARGS.spacing,
                              pitch_y=ARGS.row_pitch, label="genetic")
    spacing, max_dx = layout["spacing"], layout["max_dx"]
    half = spacing / 2.0
    half_y = 0.5 * ARGS.row_pitch
    carb.log_warn(f"[genetic] spawn layout: {layout['rows']} rows x "
                  f"{layout['columns']} cols, X pitch {spacing:.2f} m, "
                  f"max_dx {max_dx:.2f} m")

    # Rows must clear each other in Y for the same reason slices clear in X.
    # offset_target puts the WHOLE radius into Y whenever the X component is
    # clamped, so a row pitch below 2x the goal radius lets two rows be sent
    # into each other — the collision test slice_max_dx exists to prevent,
    # reintroduced along the other axis.
    if layout["rows"] > 1 and ARGS.row_pitch < 2.0 * ARGS.goal_radius:
        raise ValueError(
            f"[genetic] row pitch {ARGS.row_pitch:g} m is below 2x the goal "
            f"radius ({2.0 * ARGS.goal_radius:g} m), so drones in adjacent rows "
            f"can be sent to the same point. Raise --row-pitch, lower "
            f"--goal-radius, or reduce the population to a single row.")

    # Random goal direction per drone, bounded in X so a drone's target stays
    # inside its own slice of the arena and neighbours cannot be sent into each
    # other. Seeded, so the same --goal-seed reproduces the same task.
    goal_rng = np.random.default_rng(ARGS.goal_seed)
    # Checked at the outermost row, which is the one closest to the wall.
    extreme_y = max(abs(y) for _, y in layout["positions"])
    task.check_goal_reachable(extreme_y, ARGS.goal_radius, bounds, label="genetic")

    results_base = task.results_root(base_dir)
    tmp_files = [str(results_base / f"_genetic_tmp_{i}.npz") for i in range(pop_size)]

    drones = []
    mm_init = ARGS.motor_moments[0]
    
    # Spawn drones dummy initial
    init_dummy = np.zeros(n_genes)
    for j, (_, lo, hi) in enumerate(genes_def):
        init_dummy[j] = (lo + hi) / 2.0
        
    for i in range(pop_size):
        spawn_x, spawn_y = layout["positions"][i]
        spawn_pos = [spawn_x, spawn_y, spawn_z]
        target = task.offset_target(spawn_pos, ARGS.target_altitude,
                                    ARGS.goal_radius, goal_rng,
                                    max_dx=max_dx)
        # Each drone owns its own cell now, in Y as well as X — with rows, the
        # full arena height is no longer one drone's slice.
        drone_bounds = [spawn_x - half, spawn_x + half,
                        spawn_y - half_y, spawn_y + half_y, z_min, z_max]

        if is_mpc:
            inner = _make_mpc(_gene_map(genes_def, init_dummy), target,
                              tmp_files[i], mm_init)
        elif is_eigen:
            inner = _make_eigen(_gene_map(genes_def, init_dummy), target,
                                tmp_files[i], mm_init)
        else:
            inner = _make_pd(_gene_map(genes_def, init_dummy), target,
                             tmp_files[i])

        lag = None
        if any((ARGS.pos_delay_ms, ARGS.att_delay_ms, ARGS.pos_tau_ms, ARGS.att_tau_ms)):
            lag = rb.SensorLag(sim_clock.DEFAULT_PHYSICS_DT,
                               pos_delay=1e-3 * ARGS.pos_delay_ms,
                               att_delay=1e-3 * ARGS.att_delay_ms,
                               pos_tau=1e-3 * ARGS.pos_tau_ms,
                               att_tau=1e-3 * ARGS.att_tau_ms)
        backend = BoundedBackend(inner, tmp_files[i], lag=lag)
        cfg = MultirotorConfig()
        cfg.backends = [backend]
        cfg.desaturation = ARGS.desaturation
        cfg.thrust_curve = QuadraticThrustCurve(config={"motor_moment": mm_init,
                                            "rotor_tau": 1e-3 * ARGS.rotor_tau_ms})

        vehicle = Multirotor(
            f"/World/gen_{i}", ROBOTS["Iris"], i, spawn_pos,
            Rotation.from_euler("XYZ", [0.,0.,0.], degrees=True).as_quat(),
            config=cfg,
        )
        drones.append((vehicle, backend, spawn_pos, drone_bounds))

    # The sweep's own goals, read back off the controllers so this list can
    # never drift from what they were actually constructed with.
    hover_targets = [tuple(d[1].inner.hover_target) for d in drones]
    val_rng = np.random.default_rng(ARGS.validation_seed)

    world.reset()
    for _ in range(10):
        world.step(render=render_sim)

    # Nominal mass/inertia per vehicle, read ONCE while the plant is still
    # unperturbed. Every later scale is expressed relative to these, and the
    # controller's model is restored to them after each start(), so a long run
    # cannot drift by compounding one pass's scale onto the next's.
    nominals = {}
    if ROBUST_SCALES:
        for vehicle, _, _, _ in drones:
            nom = rb.nominal_decomposition(vehicle)
            if nom is None:
                raise RuntimeError(
                    "[genetic] --robust-inertia needs the vehicle's nominal mass "
                    "properties and they could not be read; refusing to run a "
                    "search that would silently be nominal.")
            nominals[id(vehicle)] = nom
        ref = nominals[id(drones[0][0])]
        carb.log_warn(
            f"[genetic] robust search over inertia scales {ROBUST_SCALES} "
            f"(worst case); nominal composite inertia "
            f"{np.array2string(ref['composite_inertia'], precision=6)}, "
            f"mass {ref['composite_mass']:.4f} kg")

    def _run_pass(pop_array, motor_moment, targets, gamma=None):
        """One batched evaluation of the whole population at one plant scale.

        gamma is None for the nominal search, which leaves this byte-identical
        to the pre-robustness code path — no scaling, no extra reset, and no
        model freeze.
        """
        if gamma is not None:
            # The vehicles persist across generations, so the previous pass's
            # scale is still on them: every pass sets its own explicitly rather
            # than assuming it starts from nominal. apply_plant_scale's write
            # is ignored until the next reset, hence the reset here.
            for vehicle, _, _, _ in drones:
                rb.apply_plant_scale(vehicle, mass_scale=1.0,
                                     inertia_scale=gamma,
                                     decomposition=nominals[id(vehicle)])
            world.reset()
            for _ in range(10):
                world.step(render=render_sim)

        for i in range(pop_size):
            vehicle, backend, spawn_pos, _ = drones[i]
            vehicle._thrusters = QuadraticThrustCurve(config={"motor_moment": motor_moment,
                                                    "rotor_tau": 1e-3 * ARGS.rotor_tau_ms})
            if is_mpc:
                _apply_genes_mpc(backend, _gene_map(genes_def, pop_array[i]),
                                 motor_moment)
            elif is_eigen:
                _apply_genes_eigen(backend, _gene_map(genes_def, pop_array[i]),
                                   motor_moment)
            else:
                _apply_genes_pd(backend, _gene_map(genes_def, pop_array[i]))
            backend.inner.hover_target = np.asarray(targets[i], dtype=float)

            vehicle.set_world_pose(position=spawn_pos, orientation=[0.,0.,0.,1.])
            vehicle.set_linear_velocity([0.,0.,0.])
            vehicle.set_angular_velocity([0.,0.,0.])
            backend.reset()
            backend.start()

            # AFTER start(): that is where the controller syncs its model from
            # the live vehicle, which is the SCALED one by this point. Without
            # this the model tracks the perturbation exactly and there is no
            # mismatch left to be robust to — see
            # robustness.freeze_model_at_nominal for the measurement.
            if gamma is not None:
                rb.freeze_model_at_nominal(backend.inner, nominals[id(vehicle)])

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
        
        metrics_list = []
        for i in range(pop_size):
            _, backend, _, _ = drones[i]
            if not backend._stopped:
                backend.stop()
            m = _score_from_npz(tmp_files[i], ARGS.duration)
            metrics_list.append(m)
            try: os.remove(tmp_files[i])
            except: pass
        return metrics_list

    def evaluate_pop(pop_array, motor_moment, targets, scales=None):
        """targets: list of length pop_size, one (x, y, z) hover/step target per
        drone slot for this pass — always set explicitly (never left over from
        a previous call) so a validation pass's offset target can never leak
        into the next generation's hover evaluation.

        scales: inertia scale factors to fly each individual at, worst case
        taken as its fitness. Defaults to ROBUST_SCALES; pass [] to force a
        single nominal pass regardless (the validation gate does this).
        """
        scales = ROBUST_SCALES if scales is None else scales
        if not scales:
            return _run_pass(pop_array, motor_moment, targets, None)

        per_scale = [_run_pass(pop_array, motor_moment, targets, g)
                     for g in scales]
        return [_worst_case([p[i] for p in per_scale], scales)
                for i in range(pop_size)]

    def validate_subset(idx_list, combined_pop, motor_moment, targets):
        """Run the step-response pass for exactly the individuals at idx_list
        (indices into combined_pop — the 2N parents+offspring pool), batching
        them into pop_size-sized chunks since only pop_size physical drone
        slots exist. Returns {idx: metrics}. Used to validate only the
        current hover Pareto front, not the whole population — keeps the
        extra simulation cost proportional to what actually matters."""
        results = {}
        idx_list = list(idx_list)
        for start in range(0, len(idx_list), pop_size):
            chunk = idx_list[start:start + pop_size]
            batch = np.zeros((pop_size, n_genes))
            for slot, gi in enumerate(chunk):
                batch[slot] = combined_pop[gi]
            for slot in range(len(chunk), pop_size):
                batch[slot] = combined_pop[chunk[0]]  # harmless filler; discarded below
            # Nominal only (scales=[]), even in a robust search. This gate asks
            # "does this individual still hold position toward a different
            # heading", which is a question about the controller, not about
            # mismatch — and running it at every scale would triple the cost of
            # a check whose threshold was calibrated on unperturbed runs.
            batch_metrics = evaluate_pop(batch, motor_moment, targets, scales=[])
            for slot, gi in enumerate(chunk):
                results[gi] = batch_metrics[slot]
        return results

    # Shared LHS gen-0 design, generated ONCE and reused for every motor moment
    # so each mm starts from an identical, well-spread population (cleaner
    # cross-mm comparison than the previous fresh-uniform-per-mm draw). The
    # evolutionary RNG (crossover/mutation/selection) still flows continuously,
    # so the search diverges per mm after gen 0 as it adapts to each landscape.
    init_population = _lhs_population(genes_def, pop_size, ARGS.seed)
    if ARGS.sweep_outer:
        # Overwrite one individual with the pinned outer loop, keeping its
        # LHS-sampled inner genes. Without an anchor a generation-0 population
        # can be entirely infeasible once the outer loop is free — measured on
        # an 8-point sample, every individual scored 1.3-32.8 against 0.13 for
        # the pinned default — and NSGA-II selecting among all-crashed
        # individuals has no gradient to follow.
        gene_index = {g[0]: i for i, g in enumerate(genes_def)}
        for name, value in OUTER_SEED.items():
            if name in gene_index:
                init_population[0, gene_index[name]] = value
        carb.log_warn("[genetic] seeded individual 0 with the pinned outer loop "
                      + ", ".join(f"{k}={v:.4f}" for k, v in OUTER_SEED.items()
                                  if k in gene_index))

    # ── Outer loop: motor moments ────────────────────────────────────────────
    for mm in ARGS.motor_moments:
        carb.log_warn(f"\n{'='*60}")
        carb.log_warn(f"[genetic] {ctrl_name} | mm={mm:.2e} | pop={pop_size} | gens={n_gen}")
        # The mixer policy is part of the plant, so a front is only comparable to
        # another front flown under the same one.
        carb.log_warn(f"[genetic] desaturation={ARGS.desaturation}")
        carb.log_warn(f"{'='*60}")

        out_dir = results_base / sweep_name / f"mm_{mm:.1e}"
        out_dir.mkdir(parents=True, exist_ok=True)

        archive = []  # list of (gen, genes_vec, metrics)

        # Initialize population from the shared LHS design (identical per mm)
        population = init_population.copy()

        # Initial evaluation
        carb.log_warn("\n[genetic] Evaluating initial population (Gen 0)...")
        pop_metrics = evaluate_pop(population, mm, hover_targets)
        pop_scores = np.array([m["score"] for m in pop_metrics])
        pop_efforts = np.array([m[EFFORT_KEY] for m in pop_metrics])
        
        for i in range(pop_size):
            archive.append((0, population[i].copy(), pop_metrics[i]))

        for gen in range(1, n_gen + 1):
            carb.log_warn(f"\n[genetic] Generation {gen}/{n_gen}")
            
            # Non-dominated sort on current parents for tournament selection
            fronts = _fast_nondominated_sort(pop_scores, pop_efforts)
            ranks, crowding = get_rank_and_crowding(fronts, pop_scores, pop_efforts)
            
            # Generate offspring
            offspring = np.zeros((pop_size, n_genes))
            for i in range(pop_size):
                p1_idx = _tournament_selection(ranks, crowding, rng)
                p2_idx = _tournament_selection(ranks, crowding, rng)
                child = _sbx_crossover(population[p1_idx], population[p2_idx], genes_def, rng)
                child = _polynomial_mutation(child, genes_def, rng)
                offspring[i] = child
                
            # Evaluate offspring
            off_metrics = evaluate_pop(offspring, mm, hover_targets)
            for i in range(pop_size):
                gene_str = " ".join(f"{genes_def[j][0]}={offspring[i,j]:.2f}" for j in range(n_genes))
                carb.log_warn(f"  [{i}] {gene_str} → score={off_metrics[i]['score']:.4f} effort={off_metrics[i]['effort']:.4f}")

            # NSGA-II 2N -> N Selection
            combined_pop = np.vstack([population, offspring])
            combined_metrics = pop_metrics + off_metrics

            comb_scores = np.array([m["score"] for m in combined_metrics])
            comb_efforts = np.array([m[EFFORT_KEY] for m in combined_metrics])

            # ── Periodic step-response validation ──────────────────────────
            # Every --validate-every generations, find who's CURRENTLY on the
            # hover Pareto front (score, effort) and only re-run those against
            # a target offset from hover — proves position control rather
            # than just hover stability, at a cost proportional to the front
            # size instead of the full 2N pool. Front members that fail
            # --validation-threshold are constraint-dominated in the survivor
            # cut below: they always lose to any individual that passed,
            # regardless of hover score/effort. Non-front individuals default
            # to violation=0 (untested == not penalized) — the gate only
            # scrutinizes the candidates that would otherwise be reported as
            # the search's best results.
            do_validate = ARGS.validate_every > 0 and gen % ARGS.validate_every == 0
            comb_violation = np.zeros(2 * pop_size)
            if do_validate:
                hover_fronts = _fast_nondominated_sort(comb_scores, comb_efforts)
                front0 = hover_fronts[0]
                val_targets = [
                    _validation_target(drones[i][2], ARGS.target_altitude,
                                        ARGS.validation_radius, val_rng,
                                        max_dx=max_dx)
                    for i in range(pop_size)
                ]
                carb.log_warn(
                    f"  [genetic] Gen {gen}: step-response validation of "
                    f"{len(front0)} Pareto-front individuals "
                    f"(radius={ARGS.validation_radius}m, threshold={ARGS.validation_threshold})..."
                )
                val_results = validate_subset(front0, combined_pop, mm, val_targets)
                for gi, m in val_results.items():
                    s = m["score"]
                    comb_violation[gi] = (
                        max(0.0, s - ARGS.validation_threshold) if np.isfinite(s) else np.inf
                    )
                    if gi >= pop_size:  # offspring half -> record for archival
                        off_metrics[gi - pop_size]["step_score"]  = m["score"]
                        off_metrics[gi - pop_size]["step_effort"] = m["effort"]
                n_feasible = int(np.sum(comb_violation[front0] == 0.0)) if front0 else 0
                carb.log_warn(
                    f"  [genetic] Gen {gen}: {n_feasible}/{len(front0)} "
                    f"Pareto-front individuals passed step-response validation"
                )

            for i in range(pop_size):
                archive.append((gen, offspring[i].copy(), off_metrics[i]))

            # Flush the archive every generation, not only at the end of the
            # motor moment. The CSV is a pure function of `archive`, so this is
            # a rewrite costing milliseconds against generations costing
            # minutes — and a robust search is 3x longer again.
            #
            # Written because a 3-hour run was killed externally at generation
            # 9/30 and left NOTHING behind: 26 minutes of simulation existed
            # only in memory. The search itself is unchanged; this only affects
            # when results become durable.
            _save_pareto_csv(archive, genes_def, out_dir, mm,
                             config=sm.controller_config(drones[0][1].inner))

            comb_fronts = _fast_nondominated_sort(comb_scores, comb_efforts, comb_violation)
            
            new_pop = []
            new_metrics = []
            
            for front in comb_fronts:
                if len(new_pop) + len(front) <= pop_size:
                    for idx in front:
                        new_pop.append(combined_pop[idx])
                        new_metrics.append(combined_metrics[idx])
                else:
                    cd = _crowding_distance(front, comb_scores, comb_efforts)
                    ranked = sorted(range(len(front)), key=lambda x: -cd[x])
                    remaining = pop_size - len(new_pop)
                    for r in ranked[:remaining]:
                        idx = front[r]
                        new_pop.append(combined_pop[idx])
                        new_metrics.append(combined_metrics[idx])
                    break
                    
            population = np.array(new_pop)
            pop_metrics = new_metrics
            pop_scores = np.array([m["score"] for m in pop_metrics])
            pop_efforts = np.array([m[EFFORT_KEY] for m in pop_metrics])

            fin = np.isfinite(pop_scores)
            if fin.any():
                best_i = np.argmin(pop_scores)
                carb.log_warn(f"  ★ Gen {gen} best: score={pop_scores[best_i]:.4f} effort={pop_efforts[best_i]:.4f}")

        # Save results
        # Read off a live controller, so the record is what was flown
        # rather than what the caller believes it configured.
        _save_pareto_csv(archive, genes_def, out_dir, mm,
                         config=sm.controller_config(drones[0][1].inner))

        arc_sc = np.array([a[2]["score"] for a in archive])
        arc_ef = np.array([a[2][EFFORT_KEY] for a in archive])
        fin_mask = np.isfinite(arc_sc) & np.isfinite(arc_ef)
        if fin_mask.any():
            fin_sc = arc_sc[fin_mask]
            fin_ef = arc_ef[fin_mask]
            pf_mask_arr = np.ones(len(fin_sc), dtype=bool)
            for i in range(len(fin_sc)):
                for j in range(len(fin_sc)):
                    if i == j: continue
                    if fin_sc[j] <= fin_sc[i] and fin_ef[j] <= fin_ef[i] and \
                       (fin_sc[j] < fin_sc[i] or fin_ef[j] < fin_ef[i]):
                        pf_mask_arr[i] = False
                        break
            n_pareto = int(pf_mask_arr.sum())
            carb.log_warn(f"\n[genetic] mm={mm:.2e}: {len(archive)} evals, {n_pareto} Pareto-optimal points")

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
