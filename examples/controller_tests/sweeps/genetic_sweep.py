#!/usr/bin/env python3
"""
genetic_sweep.py — NSGA-II-style genetic Pareto search for Eigen & Cascaded PD.

Searches the full gain space of each controller to find Pareto-optimal
(score, effort) configurations that the coarse grid sweeps miss.

Population of N drones simulated in parallel per generation.
Results appended to the same CSV format used by pareto_comparison.py.

Usage:
  isaac_run examples/controller_tests/sweeps/genetic_sweep.py \
      --controller eigen --motor-moments 0.0 1e-7 2.7e-7 4e-7 \
      --pop-size 20 --generations 50 --headless

  isaac_run examples/controller_tests/sweeps/genetic_sweep.py \
      --controller pd --motor-moments 0.0 1e-7 2.7e-7 4e-7 \
      --pop-size 20 --generations 50 --headless
"""
from __future__ import annotations
import argparse, csv, math, os, sys
from pathlib import Path
import carb
from isaacsim import SimulationApp

# ── CLI ──────────────────────────────────────────────────────────────────────
def parse_args():
    p = argparse.ArgumentParser(description="Genetic Pareto sweep")
    p.add_argument("--controller", choices=["eigen", "pd"], required=True)
    p.add_argument("--motor-moments", nargs="+", type=float,
                   default=[0.0, 1e-7, 2e-7, 2.7e-7, 4e-7])
    p.add_argument("--pop-size", type=int, default=20)
    p.add_argument("--generations", type=int, default=50)
    p.add_argument("--duration", type=float, default=25.0,
                   help="Simulation seconds per run (default: 25.0 to ensure >20s of stable hover)")
    p.add_argument("--target-altitude", type=float, default=1.5)
    p.add_argument("--spacing", type=float, default=4.0)
    p.add_argument("--bounds", nargs=6, type=float,
                   default=[-12., 12., -12., 12., 0., 20.])
    p.add_argument("--seed", type=int, default=42)
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
from cascaded_pd_controller import CascadedPDController
import sweep_metrics as sm

_BOUNDS_DIAG = sm.bounds_diagonal(ARGS.bounds)

# ── Gene definitions ─────────────────────────────────────────────────────────
# Only INNER-LOOP gains are swept. Outer position loop (kp_pos, Kd_pos,
# Kp_xy, Kd_xy, Kp_z, Kd_z) is kept fixed for strict cross-controller fairness.
# Each gene: (name, low, high)
#
# Box sizing note: the earlier boxes (wn ≤ 20, b ≤ 15) were too small and the
# GA ran into the walls — the best individuals at km=4e-7 pinned wn_mdyn at
# exactly 20.000. They also could not express the hypothesis under test, since
# b_mdyn is conjectured to track the spin rate and the logged spin rate reaches
# ~30 rad/s, outside the old b range entirely. Both upper bounds now clear the
# observed spin rate with room to spare.
EIGEN_GENES = [
    ("wn_mdyn", 0.5, 60.0),
    ("b_mdyn",  0.0, 40.0),
    ("Kp_att",  0.1, 5.0),
    ("Kd_att",  1.0, 30.0),
]

PD_GENES = [
    ("Kp_att",      1.0, 60.0),
    ("Kd_att",      0.05, 3.0),
    ("Kd_yaw_rate", 0.0, 2.0),
]

# ── OOB Backend (shared) ────────────────────────────────────────────────────
class BoundedBackend(Backend):
    def __init__(self, inner, results_file: str):
        super().__init__(config=None)
        self.inner = inner
        self.results_file = results_file
        self.total_time = 0.0
        self.oob_time = float("nan")
        self.is_oob = False
        self._stopped = False
        self.position = np.zeros(3)

    def initialize(self, vehicle):
        super().initialize(vehicle)
        self.inner.initialize(vehicle)

    def start(self):
        self.total_time = 0.0
        self.oob_time = float("nan")
        self.is_oob = False
        self._stopped = False
        self.inner.start()

    def stop(self):
        if self._stopped: return
        self._stopped = True
        self.inner.stop()
        if os.path.isfile(self.results_file):
            try:
                data = dict(np.load(self.results_file))
                data["oob_time"] = np.array(self.oob_time)
                np.savez(self.results_file, **data)
            except Exception: pass

    def reset(self): self.inner.reset()
    def update_sensor(self, st, d): 
        if not self.is_oob: self.inner.update_sensor(st, d)
    def update_graphical_sensor(self, st, d):
        if not self.is_oob: self.inner.update_graphical_sensor(st, d)
    def update_state(self, state: State):
        self.position = state.position.copy()
        if not self.is_oob: self.inner.update_state(state)
    def input_reference(self):
        return [0.,0.,0.,0.] if self.is_oob else self.inner.input_reference()
    def update(self, dt: float):
        if self.is_oob: return
        self.total_time += dt
        self.inner.update(dt)
    def check_oob(self, bounds) -> bool:
        if self.is_oob: return True
        x, y, z = self.position
        if not (bounds[0]<x<bounds[1] and bounds[2]<y<bounds[3] and bounds[4]<z<bounds[5]):
            self.is_oob = True
            self.oob_time = self.total_time
            return True
        # Check for crashing on the floor after initial takeoff (give it 2 seconds to climb)
        if self.total_time > 2.0 and z < 0.15:
            self.is_oob = True
            self.oob_time = self.total_time
            carb.log_warn(f"[genetic] CRASH DETECTED at z={z:.3f} t={self.total_time:.2f}s")
            return True
        return False




# ── Scoring ──────────────────────────────────────────────────────────────────
# Implementation lives in examples/utils/sweep_metrics.py so that this sweep,
# pd_sweep.py, eigen_stability_sweep.py and eigen_ab_explore.py cannot drift
# apart. See that module for the effort DC/AC split and the spin-rate metrics.
def _score_from_npz(path, T):
    return sm.compute_metrics(path, T, _BOUNDS_DIAG)


# ── Step-response validation target ──────────────────────────────────────────
def _validation_target(spawn_pos, altitude, radius, rng):
    """Random-direction, fixed-radius offset from the hover point (same pattern
    as eigen_stability_sweep.py's random_xy goal mode) — forces the drone to
    translate instead of just holding its spawn XY, so the step-response
    result actually exercises position control."""
    theta = float(rng.uniform(0.0, 2.0 * math.pi))
    return (
        spawn_pos[0] + radius * math.cos(theta),
        spawn_pos[1] + radius * math.sin(theta),
        float(altitude),
    )


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
# Outer-loop gains are FIXED at the same defaults used by all sweep scripts.
def _make_eigen(genes_vec, target, results_file, mm):
    return EigenController(
        target_position=target, results_file=results_file,
        wn_mdyn=genes_vec[0], b_mdyn=genes_vec[1],
        Kp_att=genes_vec[2], Kd_att=genes_vec[3],
        km=mm,
    )

def _make_pd(genes_vec, target, results_file):
    return CascadedPDController(
        target_position=target, results_file=results_file,
        Kp_att=(genes_vec[0], genes_vec[0]),
        Kd_att=(genes_vec[1], genes_vec[1]),
        Kd_yaw_rate=genes_vec[2],
    )

def _apply_genes_eigen(backend, genes_vec, mm):
    backend.inner.wn_mdyn = genes_vec[0]
    backend.inner.b_mdyn  = genes_vec[1]
    backend.inner.Kp_att  = genes_vec[2]
    backend.inner.Kd_att  = genes_vec[3]
    backend.inner.km      = mm

def _apply_genes_pd(backend, genes_vec):
    backend.inner.Kp_att = np.array([genes_vec[0], genes_vec[0]], dtype=float)
    backend.inner.Kd_att = np.array([genes_vec[1], genes_vec[1]], dtype=float)
    backend.inner.Kd_yaw_rate = genes_vec[2]

# ── CSV output ───────────────────────────────────────────────────────────────
def _save_pareto_csv(archive, genes_def, out_dir: Path, mm: float):
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
        w.writerow(["idx", "effort"])
        for i, (gen, genes, metrics) in enumerate(archive):
            ef = metrics["effort"]
            w.writerow([i, f"{ef:.6f}" if np.isfinite(ef) else "inf"])

    detail_path = out_dir / "genetic_archive.csv"
    # Column set comes from sweep_metrics.METRIC_KEYS so every sweep's archive
    # carries the same metrics in the same order — including the effort DC/AC
    # split and the r_ss/r_drift spin-rate columns.
    metric_keys = ["motor_moment"] + sm.METRIC_KEYS

    with open(str(detail_path), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["gen"] + gene_names + metric_keys)
        for gen, genes, metrics in archive:
            row = [str(gen)] + [f"{g:.6f}" for g in genes]
            for k in metric_keys:
                if k == "motor_moment":
                    # Raw repr, NOT "%.6f" — motor moments live in the 1e-7-ish
                    # range and "%.6f" silently rounds anything under ~5e-7 to
                    # "0.000000", making every non-zero km indistinguishable
                    # from km=0 in the archive. str() matches how the grid
                    # sweeps' save_grid_archive() already write this column.
                    row.append(str(mm))
                    continue
                row.append(sm.format_metric(k, metrics[k]))
            w.writerow(row)
    carb.log_warn(f"[genetic] archive CSV → {detail_path}")

# ── Main ─────────────────────────────────────────────────────────────────────
def main():
    timeline = omni.timeline.get_timeline_interface()
    render_sim = not ARGS.headless
    bounds = ARGS.bounds
    rng = np.random.default_rng(ARGS.seed)

    is_eigen = ARGS.controller == "eigen"
    genes_def = EIGEN_GENES if is_eigen else PD_GENES
    n_genes = len(genes_def)
    pop_size = ARGS.pop_size
    n_gen = ARGS.generations
    ctrl_name = "Eigen" if is_eigen else "Cascaded PD"

    base_dir = Path(__file__).resolve().parent.parent
    sweep_name = "genetic_eigen_sweep" if is_eigen else "genetic_pd_sweep"

    pg = PegasusInterface()
    pg._world = World(**pg._world_settings)
    world = pg.world
    pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])
    for _ in range(20):
        world.step(render=render_sim)

    # Spacing
    x_min, x_max = bounds[0], bounds[1]
    x_margin = 1.5
    x_usable = x_max - x_min - 2.0 * x_margin
    auto_spacing = x_usable / (pop_size - 1) if pop_size > 1 else 0.0
    spacing = min(ARGS.spacing, auto_spacing)
    x_start = x_min + x_margin
    spawn_y, spawn_z = -1.5, 0.07
    half = spacing / 2.0
    y_min, y_max = bounds[2], bounds[3]
    z_min, z_max = bounds[4], bounds[5]

    results_base = base_dir / "results"
    tmp_files = [str(results_base / f"_genetic_tmp_{i}.npz") for i in range(pop_size)]

    drones = []
    mm_init = ARGS.motor_moments[0]
    
    # Spawn drones dummy initial
    init_dummy = np.zeros(n_genes)
    for j, (_, lo, hi) in enumerate(genes_def):
        init_dummy[j] = (lo + hi) / 2.0
        
    for i in range(pop_size):
        spawn_x = x_start + i * spacing
        spawn_pos = [spawn_x, spawn_y, spawn_z]
        target = (spawn_x, spawn_y, float(ARGS.target_altitude))
        drone_bounds = [spawn_x-half, spawn_x+half, y_min, y_max, z_min, z_max]

        if is_eigen:
            inner = _make_eigen(init_dummy, target, tmp_files[i], mm_init)
        else:
            inner = _make_pd(init_dummy, target, tmp_files[i])

        backend = BoundedBackend(inner, tmp_files[i])
        cfg = MultirotorConfig()
        cfg.backends = [backend]
        cfg.thrust_curve = QuadraticThrustCurve(config={"motor_moment": mm_init})

        vehicle = Multirotor(
            f"/World/gen_{i}", ROBOTS["Iris"], i, spawn_pos,
            Rotation.from_euler("XYZ", [0.,0.,0.], degrees=True).as_quat(),
            config=cfg,
        )
        drones.append((vehicle, backend, spawn_pos, drone_bounds))

    hover_targets = [(d[2][0], d[2][1], float(ARGS.target_altitude)) for d in drones]
    val_rng = np.random.default_rng(ARGS.validation_seed)

    world.reset()
    for _ in range(10):
        world.step(render=render_sim)
        
    def evaluate_pop(pop_array, motor_moment, targets):
        """targets: list of length pop_size, one (x, y, z) hover/step target per
        drone slot for this pass — always set explicitly (never left over from
        a previous call) so a validation pass's offset target can never leak
        into the next generation's hover evaluation."""
        for i in range(pop_size):
            vehicle, backend, spawn_pos, _ = drones[i]
            vehicle._thrusters = QuadraticThrustCurve(config={"motor_moment": motor_moment})
            if is_eigen:
                _apply_genes_eigen(backend, pop_array[i], motor_moment)
            else:
                _apply_genes_pd(backend, pop_array[i])
            backend.inner.hover_target = np.asarray(targets[i], dtype=float)

            vehicle.set_world_pose(position=spawn_pos, orientation=[0.,0.,0.,1.])
            vehicle.set_linear_velocity([0.,0.,0.])
            vehicle.set_angular_velocity([0.,0.,0.])
            backend.reset()
            backend.start()
            
        for _ in range(5):
            world.step(render=render_sim)
            
        timeline.play()
        sim_time = 0.0
        dt = 1.0/60.0
        while simulation_app.is_running() and sim_time < ARGS.duration:
            world.step(render=render_sim)
            sim_time += dt
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
            batch_metrics = evaluate_pop(batch, motor_moment, targets)
            for slot, gi in enumerate(chunk):
                results[gi] = batch_metrics[slot]
        return results

    # Shared LHS gen-0 design, generated ONCE and reused for every motor moment
    # so each mm starts from an identical, well-spread population (cleaner
    # cross-mm comparison than the previous fresh-uniform-per-mm draw). The
    # evolutionary RNG (crossover/mutation/selection) still flows continuously,
    # so the search diverges per mm after gen 0 as it adapts to each landscape.
    init_population = _lhs_population(genes_def, pop_size, ARGS.seed)

    # ── Outer loop: motor moments ────────────────────────────────────────────
    for mm in ARGS.motor_moments:
        carb.log_warn(f"\n{'='*60}")
        carb.log_warn(f"[genetic] {ctrl_name} | mm={mm:.2e} | pop={pop_size} | gens={n_gen}")
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
        pop_efforts = np.array([m["effort"] for m in pop_metrics])
        
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
            comb_efforts = np.array([m["effort"] for m in combined_metrics])

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
                                        ARGS.validation_radius, val_rng)
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
            pop_efforts = np.array([m["effort"] for m in pop_metrics])

            fin = np.isfinite(pop_scores)
            if fin.any():
                best_i = np.argmin(pop_scores)
                carb.log_warn(f"  ★ Gen {gen} best: score={pop_scores[best_i]:.4f} effort={pop_efforts[best_i]:.4f}")

        # Save results
        _save_pareto_csv(archive, genes_def, out_dir, mm)

        arc_sc = np.array([a[2]["score"] for a in archive])
        arc_ef = np.array([a[2]["effort"] for a in archive])
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
