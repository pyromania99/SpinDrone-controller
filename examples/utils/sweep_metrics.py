#!/usr/bin/env python
"""
| File: sweep_metrics.py
| Description: Single shared scoring implementation for every controller sweep.

Why this module exists
──────────────────────
The stability/effort metrics were previously copy-pasted into four sweep
scripts (eigen_stability_sweep.py, pd_sweep.py, eigen_ab_explore.py,
genetic_sweep.py), each carrying a comment asserting the copies were
"identical". They were not required to stay that way by anything other than
discipline, and the archives they wrote had drifting column sets. Every sweep
now imports from here, so a metric change lands everywhere at once.

Deliberately pure-numpy: this module imports neither ``carb`` nor anything
from Isaac Sim, so it can also be run from a plain interpreter to recompute
metrics from ``.npz`` files that are already on disk (see
``examples/controller_tests/evaluations/backfill_metrics.py``).

What is measured
────────────────
**Stability score** (unchanged from the original implementation)
    Future-biased mean position error, with any post-OOB tail padded out to
    the bounds diagonal so an early crash cannot look cheap::

        score = Σ w(t)·‖ep(t)‖ / Σ w(t),   w(t) = t / T

**Effort — split into orthogonal collective and differential channels (new)**
    The previous single effort number was ``Σᵢ uᵢ(t)²``, which is dominated by
    the constant hover term: an Iris hovering at ~1.75 kg spends
    ``4·(mg/4)² ≈ 73.7 N²`` before any control action at all, so measured
    values of 73.5–77.4 N² are ~95 % hover bias and only ~5 % actual control
    activity. Optimising against it is nearly optimising against a constant,
    which is why Pareto fronts computed on it collapse to a single point.

    Writing ``ū(t)`` for the instantaneous mean across the four motors, the
    decomposition is exact (an orthogonal projection onto the all-ones vector
    and its complement)::

        E(t)     = Σᵢ uᵢ(t)²                    total
        E_dc(t)  = 4·ū(t)²                      collective → net thrust
        E_ac(t)  = Σᵢ (uᵢ(t) − ū(t))²           differential → control torque
        E_dc(t) + E_ac(t) == E(t)               exactly, at every timestep

    ``E_dc`` is the thrust channel (hover + altitude hold). ``E_ac`` is the
    torque channel — and since the attitude gains being swept act *only*
    through differential thrust, ``E_ac`` is the part that actually responds
    to them. Use ``effort_ac_ses`` as the optimisation target; ``effort`` is
    retained unchanged so old and new archives stay comparable.

**Spin rate (new)**
    This vehicle spins: ``rot_dir`` defaults to ``[-1, -1, -1, -1]`` (all four
    rotors turn the same way), so a non-zero ``motor_moment`` produces a net
    yaw torque that no allocation can cancel while still producing lift. The
    steady-state rate reaches ~30 rad/s in the logged runs and is set almost
    entirely by ``motor_moment`` — measured as ``r_ss ≈ 4.9e7 · km``, varying
    by under 0.02 rad/s across every attitude gain combination in the grid.
    That makes it a clean independent variable.

    Spin-up is first-order with τ ≈ 1.8 s, so the rate is flat well before the
    25 s runs end; ``r_drift`` reports the residual slope over the tail so any
    analysis regressing against spin rate can confirm that per run rather than
    assuming it.

**Truth vs. belief (``ep_true``)**
    Every metric derived from position error is computed from ``ep_true`` when
    the archive carries it, falling back to ``ep`` otherwise. The controllers
    log ``ep`` from their own state estimate, which is the truth in every
    nominal run and is *not* the truth under a sensor-lag perturbation. Scoring
    a lagged controller's own belief would reward it for being badly informed:
    it reports a small error while sitting far off target, and the score
    improves monotonically with added delay. See examples/utils/robustness.py.

**Roll/pitch settling (``oscillation``)**
    RMS of ‖(p, q)‖ over the settled tail of the run — how much roll/pitch rate
    is still present once the vehicle should have converged.

    This deliberately excludes the yaw rate ``r``. The metric used to be
    ``Σ‖(p, q, r)‖·dt`` over the whole run, which on this vehicle is ~97 % yaw:
    the airframe spins at up to 29 rad/s by design, so the number reported the
    motor moment back and told you nothing about control quality. Dropping r
    leaves the two axes the attitude loop actually stabilises.

    It is also the channel a gyroscopic coning divergence shows up in first. The
    ``(Iyy − Izz)·q·r`` cross terms couple roll into pitch with a gain set by the
    spin rate, so an underdamped configuration grows in p and q well before the
    position error moves — measured at km=2e-7, ‖eR‖ ran 0.012 → 0.091 → 0.619
    over three seconds while the vehicle still looked stationary. A settled run
    lands near zero here; one that is oscillating or diverging does not.

    Note the units changed with the definition: this is a rate (rad/s), not the
    integrated angle (rad) the old version produced. Values are not comparable
    across the change, and archives written before it carry the old quantity.
"""
from __future__ import annotations

import numpy as np

# ── Saturated Effort Score constants (physics-based, Iris quadrotor) ──────────
#   E_SAT      = 4 × u_max²                        (all four rotors at maximum)
#   SAT_THRESH = 80 % of E_SAT                     (near-saturation boundary)
#   SAT_ALPHA  = penalty weight: SES = E_mean × (1 + α × sat_frac)
#
# u_max comes from the rotor model, not a guess: QuadraticThrustCurve applies
# F = rotor_constant · ω², with rotor_constant 8.54858e-6 and a maximum rotor
# velocity of 1100 rad/s, giving 10.34 N per rotor and a thrust-to-weight of
# ~2.4 for the ~1.75 kg airframe.
#
# This previously read 5.0 N, which put SAT_THRESH at 4.47 N/rotor against a
# hover thrust of 4.29 N — only 4.2 % of headroom. The "near-saturation"
# penalty therefore fired on 100 % of well-behaved runs (mean 21 % of their
# timesteps), multiplying their effort by ~2 for reasons unrelated to
# saturation and corrupting the effort ranking of every archive written before
# this fix.
U_MAX_PER_MOTOR: float = 8.54858e-6 * 1100.0 ** 2     # 10.344 N
E_SAT:           float = 4.0 * U_MAX_PER_MOTOR ** 2   # 428.0 N²
SAT_THRESH:      float = 0.80 * E_SAT                 # 342.4 N²
SAT_ALPHA:       float = 5.0

# Fraction of the logged trajectory used as the "settled" tail when reducing
# yaw rate to a single steady-state number.
SPIN_TAIL_FRAC: float = 1.0 / 3.0

# Stability score at or above which a run is treated as a failure rather than
# degraded flight. A run that leaves the arena is charged the bounds diagonal
# (~39 m) for its missing tail, so failures land far above this; well-behaved
# hovers score ~0.1. Used for the `stable` flag and by callers that need to ask
# "did this configuration fly at all".
STABLE_SCORE_MAX: float = 2.0

# Every metric this module can produce, with the value used when a run is
# unusable. Sweeps build their CSV header from METRIC_KEYS so all archives
# carry the same columns in the same order.
METRIC_DEFAULTS: dict = {
    # Primary objectives
    "score":         np.inf,
    "effort":        np.inf,   # SES on total Σuᵢ² — unchanged, for continuity
    # Effort decomposition (new)
    "effort_dc":     np.inf,   # future-biased mean of 4·ū²   (thrust channel)
    "effort_ac":     np.inf,   # future-biased mean of Σ(uᵢ−ū)² (torque channel)
    "effort_ac_ses": np.inf,   # effort_ac × (1 + SAT_ALPHA × sat_frac)
    # Spin state (new)
    "r_ss":          np.nan,   # mean |yaw rate| over the settled tail [rad/s]
    "r_peak":        np.nan,   # max |yaw rate| over the run           [rad/s]
    "r_drift":       np.nan,   # |slope of |r|| over that tail; ~0 ⇒ settled [rad/s²]
    # Extended descriptive metrics
    "rise_time":     np.nan,
    "settling_time": np.nan,
    "max_overshoot": np.nan,
    "rms_error":     np.nan,
    "peak_effort":   np.nan,
    "total_energy":  np.nan,
    "stable":        0,
    # RMS of ‖(p, q)‖ over the settled tail [rad/s] — yaw deliberately excluded
    "oscillation":   np.nan,
    # Populated only on validation passes (step response to an offset target)
    "step_score":    np.nan,
    "step_effort":   np.nan,
}

METRIC_KEYS: list = list(METRIC_DEFAULTS.keys())

# Keys whose value is an integer flag rather than a float (CSV formatting).
INT_METRIC_KEYS: frozenset = frozenset({"stable"})


def bounds_diagonal(bounds) -> float:
    """Diagonal length of the [xmin xmax ymin ymax zmin zmax] arena box.

    Used as the padding value for the post-OOB tail of the stability score:
    a drone that leaves the arena is charged the worst error the arena can
    physically produce for every timestep it is missing.
    """
    return float(np.linalg.norm([
        bounds[1] - bounds[0],
        bounds[3] - bounds[2],
        bounds[5] - bounds[4],
    ]))


def _future_biased_mean(t: np.ndarray, values: np.ndarray, T: float) -> float:
    """Σ w·v / Σ w with w = t/T — weights late samples more than early ones."""
    w = t / T
    w_sum = float(np.sum(w))
    if w_sum < 1e-12:
        return np.inf
    return float(np.dot(w, values) / w_sum)


def _pad_tail(t: np.ndarray, values: np.ndarray, oob_time: float, T: float,
              pad_value: float) -> tuple:
    """Extend a truncated (OOB) run out to T, charging ``pad_value`` throughout.

    Returns the padded (t, values). No-op when the run reached T normally.
    """
    if not (np.isfinite(oob_time) and oob_time < T - 1e-6):
        return t, values
    dt = float(t[1] - t[0]) if len(t) > 1 else 1.0 / 60.0
    pad_t = np.arange(oob_time + dt, T, dt)
    if len(pad_t) == 0:
        return t, values
    return (np.concatenate([t, pad_t]),
            np.concatenate([values, np.full(len(pad_t), pad_value)]))


def spin_metrics(yaw_rate, time=None) -> dict:
    """Reduce a yaw-rate trace to (r_ss, r_peak, r_drift).

    ``r_ss``    [rad/s]  mean |r| over the last SPIN_TAIL_FRAC of the trace.
    ``r_peak``  [rad/s]  max |r| over the whole trace.
    ``r_drift`` [rad/s²] magnitude of the least-squares slope of |r| over that
                same tail — the *residual* drift rate, i.e. how far from
                settled the spin still is at the end of the run.

    r_drift deliberately measures the tail slope rather than the change from
    the start of the run. Spin-up here is first-order with τ ≈ 1.8 s, so a
    head-to-tail difference is dominated by the startup transient and stays
    large (~24 % of r_ss) even when the rate has been flat for 15 s — it would
    report "not settled" for runs that plainly are. Measured as a tail slope,
    every logged run at T = 25 s comes out below 0.025 %/s of r_ss.

    Any analysis treating spin rate as an independent variable should filter
    on r_drift, which is what makes it meaningful for that purpose.
    """
    out = {"r_ss": np.nan, "r_peak": np.nan, "r_drift": np.nan}
    if yaw_rate is None:
        return out
    r = np.abs(np.asarray(yaw_rate, dtype=float).ravel())
    if r.size < 3 or not np.all(np.isfinite(r)):
        return out

    n_tail = max(2, int(round(r.size * SPIN_TAIL_FRAC)))
    r_tail = r[-n_tail:]
    out["r_ss"] = float(np.mean(r_tail))
    out["r_peak"] = float(np.max(r))

    # Slope needs a time axis; fall back to sample index if none was logged,
    # which only rescales the units and still separates settled from drifting.
    if time is not None:
        t = np.asarray(time, dtype=float).ravel()
        t_tail = t[-n_tail:] if t.size == r.size else np.arange(n_tail, dtype=float)
    else:
        t_tail = np.arange(n_tail, dtype=float)
    if np.ptp(t_tail) < 1e-9:
        return out
    try:
        out["r_drift"] = abs(float(np.polyfit(t_tail, r_tail, 1)[0]))
    except Exception:
        pass
    return out


def pq_settling(angular_velocity) -> float:
    """RMS of the roll/pitch rate magnitude over the settled tail [rad/s].

    ``angular_velocity`` is the (N, 3) body-frame [p, q, r] trace the
    controllers log. Only the first two columns are used — see the module
    docstring for why yaw is excluded.

    Uses the same SPIN_TAIL_FRAC tail as ``spin_metrics`` so "settled" means the
    same window for both. Returns NaN rather than 0.0 for a trace that is
    missing, too short, or non-finite: a run with no data has not demonstrated
    that its roll/pitch rates settled, and must not be scored as if it had.
    """
    w = np.asarray(angular_velocity, dtype=float)
    if w.ndim != 2 or w.shape[0] < 3 or w.shape[1] < 2:
        return np.nan
    pq = np.linalg.norm(w[:, :2], axis=1)
    if not np.all(np.isfinite(pq)):
        return np.nan
    n_tail = max(2, int(round(pq.size * SPIN_TAIL_FRAC)))
    return float(np.sqrt(np.mean(np.square(pq[-n_tail:]))))


def effort_channels(motor_thrusts: np.ndarray) -> tuple:
    """Split per-timestep motor thrusts into (E_total, E_dc, E_ac).

    ``E_dc + E_ac == E_total`` exactly at every timestep — the collective and
    differential components are orthogonal, so the decomposition loses nothing.
    """
    u = np.asarray(motor_thrusts, dtype=float)
    u_bar = np.mean(u, axis=1, keepdims=True)
    e_total = np.sum(np.square(u), axis=1)
    e_dc = float(u.shape[1]) * np.square(u_bar).ravel()
    e_ac = np.sum(np.square(u - u_bar), axis=1)
    return e_total, e_dc, e_ac


def compute_metrics(npz_path: str, T_total: float, bounds_diag: float) -> dict:
    """Compute every metric for one run. Never raises — returns defaults instead.

    ``T_total`` is the intended run duration; a run that went OOB early is
    padded out to it so short crashes are penalised rather than rewarded.
    Pass ``T_total <= 0`` to use the last logged timestamp instead.
    """
    metrics = dict(METRIC_DEFAULTS)
    try:
        d = np.load(npz_path)
        keys = set(d.files)

        if "time" not in keys or "ep" not in keys:
            return metrics
        t = np.asarray(d["time"], dtype=float)
        # ``ep`` is what the CONTROLLER believed its position error was, since
        # every controller logs its own ``self.p``. With a perfect state
        # estimate those coincide, and every archive written before the
        # robustness study is in that regime. Under a sensor-lag perturbation
        # they do not: a controller fed a stale position logs a small error
        # while sitting a long way off target, and scoring its belief would
        # rank "badly informed" as "well controlled" — the more lag, the better
        # the score. ``ep_true`` is written by the perturbation backend from the
        # simulator's own state and is preferred wherever present.
        ep_key = "ep_true" if "ep_true" in keys else "ep"
        ep = np.asarray(d[ep_key], dtype=float)
        if len(t) < 2 or ep.ndim != 2 or ep.shape[0] != len(t):
            return metrics

        oob = float(d["oob_time"]) if "oob_time" in keys else float("nan")
        if T_total < 1e-6:
            T_total = float(t[-1])

        # ── Spin state — independent of everything else, so record it even if
        # the run is otherwise unusable. This is the variable the Eigen
        # controller's A_r term cancels against, and the one b_mdyn is
        # hypothesised to track.
        metrics.update(spin_metrics(
            d["yaw_rate"] if "yaw_rate" in keys else None, time=t))

        errors = np.linalg.norm(ep, axis=1)

        # ── Descriptive metrics, measured pre-padding on the real trajectory ──
        e0_thresh = max(errors[0] if errors.size else 0.0, 1.0)
        rise_idx = np.where(errors < 0.1 * e0_thresh)[0]
        if len(rise_idx) > 0:
            metrics["rise_time"] = float(t[rise_idx[0]])
            metrics["max_overshoot"] = float(np.max(errors[rise_idx[0]:]))

        settled_idx = np.where(errors > 0.05 * e0_thresh)[0]
        if len(settled_idx) == 0:
            metrics["settling_time"] = float(t[0])
        elif settled_idx[-1] < len(t) - 1:
            metrics["settling_time"] = float(t[settled_idx[-1] + 1])

        metrics["rms_error"] = float(np.sqrt(np.mean(np.square(errors))))

        # ── Effort channels ──────────────────────────────────────────────────
        # A missing or malformed motor_thrusts array must NOT read as zero
        # effort — that would make a broken run score as free.
        if "motor_thrusts" not in keys:
            return metrics
        mt = np.asarray(d["motor_thrusts"], dtype=float)
        if mt.ndim != 2 or mt.shape[0] != len(t):
            return metrics

        e_total, e_dc, e_ac = effort_channels(mt)
        metrics["peak_effort"] = float(np.max(e_total))
        dt_arr = np.diff(t, prepend=t[0])
        metrics["total_energy"] = float(np.sum(e_total * dt_arr))

        if "angular_velocity" in keys:
            metrics["oscillation"] = pq_settling(d["angular_velocity"])

        if not (np.all(np.isfinite(errors)) and np.all(np.isfinite(e_total))):
            return metrics

        # ── Padding: error → bounds diagonal, effort → worst observed ────────
        t_err, err_p = _pad_tail(t, errors, oob, T_total, bounds_diag)
        t_eff, tot_p = _pad_tail(t, e_total, oob, T_total, float(np.max(e_total)))
        _, dc_p = _pad_tail(t, e_dc, oob, T_total, float(np.max(e_dc)))
        _, ac_p = _pad_tail(t, e_ac, oob, T_total, float(np.max(e_ac)))

        score = _future_biased_mean(t_err, err_p, T_total)

        # sat_frac is a property of TOTAL thrust (motors saturate on their
        # absolute output, not on the differential part), so the same value
        # multiplies both the legacy SES and the new AC-channel SES.
        sat_frac = float(np.mean(tot_p > SAT_THRESH))
        sat_mult = 1.0 + SAT_ALPHA * sat_frac

        e_tot_mean = _future_biased_mean(t_eff, tot_p, T_total)
        e_dc_mean = _future_biased_mean(t_eff, dc_p, T_total)
        e_ac_mean = _future_biased_mean(t_eff, ac_p, T_total)

        metrics["score"] = score if np.isfinite(score) else np.inf
        metrics["effort"] = e_tot_mean * sat_mult if np.isfinite(e_tot_mean) else np.inf
        metrics["effort_dc"] = e_dc_mean if np.isfinite(e_dc_mean) else np.inf
        metrics["effort_ac"] = e_ac_mean if np.isfinite(e_ac_mean) else np.inf
        metrics["effort_ac_ses"] = (e_ac_mean * sat_mult
                                    if np.isfinite(e_ac_mean) else np.inf)
        metrics["stable"] = 1 if (not np.isfinite(oob)
                                  and score < STABLE_SCORE_MAX) else 0
        return metrics
    except Exception:
        return metrics


# ── Back-compatible single-value wrappers ────────────────────────────────────
# The sweep scripts call these as standalone functions in a few places; they
# are thin views onto compute_metrics() so there is still only one code path.

def stability_score(npz_path: str, T_total: float, bounds_diag: float) -> float:
    return compute_metrics(npz_path, T_total, bounds_diag)["score"]


def effort_score(npz_path: str, T_total: float, bounds_diag: float) -> float:
    """Legacy total-thrust SES. Prefer ``effort_ac_ses`` for new work."""
    return compute_metrics(npz_path, T_total, bounds_diag)["effort"]


def format_metric(key: str, value) -> str:
    """CSV cell for one metric — integer flags plain, NaN as 'nan', inf as 'inf'.

    NaN and inf mean different things and must not share a spelling. In this
    module's defaults NaN is "never measured" and inf is "measured, and the run
    was unusable". Both used to serialise to 'inf', which made the two
    indistinguishable once written.

    That was not cosmetic. ``step_score`` defaults to NaN and is only populated
    for the Pareto-front members a sweep chooses to validate — and two sweeps
    (eigen_ab_explore, mpc_sweep) plus eigen_stability's kpkd mode never run a
    validation pass at all. Across the archives that produced, 7980 of 8518
    rows read 'inf' in that column while only 538 had ever been flown against
    an offset target, of which 532 passed. Read at face value the column says
    94 % of every configuration failed a test that 94 % of them never took, and
    anything gating on it would have discarded almost the entire population.

    Readers are unaffected: every consumer parses with float()/pd.read_csv and
    filters on isfinite, all of which treat 'nan' and 'inf' alike.
    """
    if key in INT_METRIC_KEYS:
        return str(int(value))
    if np.isfinite(value):
        return f"{value:.6f}"
    return "nan" if np.isnan(value) else "inf"
