#!/usr/bin/env python
"""
| File: gene_space.py
| Description: The tuning parameters, their boxes, and how to step through them.

Why this module exists
──────────────────────
All of this lived inside ``examples/controller_tests/sweeps/genetic_sweep.py``,
which runs ``parse_args()`` and constructs ``SimulationApp(...)`` at module
scope. Importing it therefore boots Isaac Sim and consumes ``sys.argv``, so the
gene definitions were unreachable from any other process — an analysis script
could not so much as ask what the box for ``wn_mdyn`` is without starting a GPU
simulator.

Kept deliberately pure: numpy and the standard library only, no ``carb``, no
Isaac, no controller imports. That is what lets the report tool know each
axis's range and scale offline, the same discipline sweep_metrics.py and
robustness.py already keep. The controller-construction half lives next door in
controller_factory.py, which does need Isaac.

What a "gene" is
────────────────
A ``(name, low, high)`` triple. Names are the canonical spelling used in every
archive column, so a gene, a CSV header and a robustness ``--gains`` key are all
the same string.
"""
from __future__ import annotations

import math

import numpy as np

# ── Inner-loop genes ─────────────────────────────────────────────────────────
#
# Box sizing note: the earlier boxes (wn <= 20, b <= 15) were too small and the
# search ran into the walls — the best individuals at km=4e-7 pinned wn_mdyn at
# exactly 20.000. They also could not express the hypothesis under test, since
# b_mdyn is conjectured to track the spin rate and the logged spin rate reaches
# ~30 rad/s, outside the old b range entirely.
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

MPC_GENES = [
    ("mpc_q", 0.5, 10.0),
    ("mpc_r", 0.05, 2.0),
    ("mpc_N", 5.0, 25.0),
]

# ── Outer position-loop genes ────────────────────────────────────────────────
#
# Parameterised as (natural frequency, damping ratio), not (kp, kd), because
# the two are not independent: wn = sqrt(kp) and zeta = kd / (2*sqrt(kp)), so
# sampling kp and kd independently varies damping as a side effect of varying
# bandwidth. Measured on an 8-point sample over kp in [0.25,8], kd in [0.5,8]:
# kp=6.83,kd=1.19 lands at zeta=0.23 — a loop that is not faster, just undamped.
# The good region is a curved ridge in (kp, kd) and a box in (wn, zeta).
OUTER_GENES = {
    "eigen": [("wn_xy", 0.35, 2.0), ("zeta_xy", 0.5, 1.3)],
    "pd":    [("wn_xy", 0.35, 2.0), ("zeta_xy", 0.5, 1.3)],
    "mpc":   [("wn_xy", 0.35, 2.0), ("zeta_xy", 0.5, 1.3),
              ("ki_xy", 0.0, 1.0), ("ki_z", 0.0, 8.0)],
}

# The pinned operating point in gene coordinates, round-tripping to exactly
# kp_xy = 0.50 and kd_xy = 1.00 — the values every sweep before --sweep-outer
# flew at.
OUTER_SEED = {
    "wn_xy": math.sqrt(0.50),
    "zeta_xy": 1.0 / (2.0 * math.sqrt(0.50)),
    "ki_xy": 0.05,
    "ki_z": 2.0,
}

# ── How each gene is stepped ─────────────────────────────────────────────────
#
# An explicit table, not a heuristic on the box. Four genes have a lower bound
# of exactly 0 — b_mdyn, Kd_yaw_rate, ki_xy, ki_z — and a multiplicative ladder
# through 0 is undefined; guessing "log if lo > 0" would silently give zeta_xy
# a log ladder across a range spanning under 3x, where it buys nothing.
#
# "log"     step by a constant ratio; the natural scale for a gain spanning
#           decades, and the scale its response should be read on
# "linear"  step by a constant fraction of the box width
# "integer" step by +/-1 and de-duplicate (a prediction horizon is a count)
GENE_SCALE: dict = {
    "wn_mdyn":     "log",
    "b_mdyn":      "linear",
    "Kp_att":      "log",
    "Kd_att":      "log",
    "Kd_yaw_rate": "linear",
    "mpc_q":       "log",
    "mpc_r":       "log",
    "mpc_N":       "integer",
    "wn_xy":       "log",
    "zeta_xy":     "linear",
    "ki_xy":       "linear",
    "ki_z":        "linear",
}

# Genes whose lower bound is physically meaningful and must not be crossed even
# when marching outside the search box: a negative integral gain or a negative
# damping cross-term is a different controller, not a further ablation step.
GENE_FLOOR: dict = {
    "b_mdyn": 0.0, "Kd_yaw_rate": 0.0, "ki_xy": 0.0, "ki_z": 0.0,
    "zeta_xy": 0.05,
}

INNER_GENES: dict = {"eigen": EIGEN_GENES, "pd": PD_GENES, "mpc": MPC_GENES}


def genes_for(controller: str, sweep_outer: bool = False) -> list:
    """The gene list for one controller, with or without the outer loop."""
    if controller not in INNER_GENES:
        raise ValueError(f"unknown controller {controller!r}; "
                         f"expected one of {sorted(INNER_GENES)}")
    genes = list(INNER_GENES[controller])
    if sweep_outer:
        genes = genes + list(OUTER_GENES[controller])
    return genes


def gene_box(genes_def) -> dict:
    """{name: (lo, hi)} for a gene list."""
    return {g[0]: (float(g[1]), float(g[2])) for g in genes_def}


def gene_map(genes_def, genes_vec) -> dict:
    """{gene name: value} for one individual."""
    return {g[0]: float(v) for g, v in zip(genes_def, genes_vec)}


def gene_vector(genes_def, g: dict) -> np.ndarray:
    """The inverse of gene_map, in the gene list's own order."""
    return np.array([float(g[name]) for name, _, _ in genes_def], dtype=float)


def outer_gains(g: dict) -> dict:
    """Derive the physical outer gains from whichever genes are present.

    (wn_xy, zeta_xy) -> kp = wn**2, kd = 2*zeta*wn. Passing kp_xy/kd_xy
    directly is still honoured, so an explicit box in physical coordinates
    remains usable without touching this code.

    Anything consuming a gain dict downstream — robustness_sweep's
    ``check_gain_keys`` in particular — knows kp_xy/kd_xy and rejects
    wn_xy/zeta_xy, so this conversion is mandatory at that boundary.
    """
    out = {k: v for k, v in g.items()
           if k in ("kp_xy", "kd_xy", "kp_z", "kd_z", "ki_xy", "ki_z",
                    "int_max_xy", "int_max_z", "max_tilt_deg", "kp_rate")}
    if "wn_xy" in g:
        wn = g["wn_xy"]
        out["kp_xy"] = wn * wn
        out["kd_xy"] = 2.0 * g.get("zeta_xy", 0.7071) * wn
    return out


def outer_kwargs(g: dict, controller: str) -> dict:
    """Outer-loop constructor arguments for whichever outer genes are present.

    Absent genes are simply omitted, so each controller keeps its own defaults
    and a run without the outer loop swept is byte-identical to one from before
    it existed. Names are translated from the canonical CONFIG_KEYS spelling to
    each controller's own (CascadedPDController says Kp_xy where the others say
    kp_pos).
    """
    g = outer_gains(g)
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


def lhs_population(genes_def, n: int, seed: int) -> np.ndarray:
    """Latin Hypercube sample over the gene box.

    Stratifies each gene into n bins with exactly one sample per bin, which
    independent uniform sampling cannot promise at small n — in 3-4D a handful
    of uniform draws routinely leaves whole regions unrepresented.

    No uniform fallback: if scipy.stats.qmc is missing this raises loudly, so a
    degraded design can never run silently under the guise of LHS.
    """
    lo = np.array([g[1] for g in genes_def], dtype=float)
    hi = np.array([g[2] for g in genes_def], dtype=float)
    from scipy.stats import qmc
    unit = qmc.LatinHypercube(d=len(genes_def), seed=seed).random(n)
    return lo + unit * (hi - lo)


# ── The ablation ladder ──────────────────────────────────────────────────────

def ladder_values(name: str, x0: float, lo: float, hi: float, direction: str,
                  ratio: float = 1.25, delta_frac: float = 0.08,
                  max_steps: int = 24, extend: float = 2.0) -> list:
    """Values to try, marching outward from ``x0`` in one direction.

    Returns the ordered list of parameter values an ablation arm will visit,
    nearest first. The caller stops early when flight breaks; this only says
    where the steps are.

    Marching PAST the box is deliberate and not optional. The search boxes are
    known to be too small — the first free-outer GA winner pinned three of six
    genes to a bound (wn_xy at its ceiling of 2.0, Kp_att at 5.0, Kd_att at its
    floor of 1.0) — so an arm confined to the box would report "never broke"
    when it merely never left the box, and the two are not the same finding.
    ``extend`` widens the reachable range to [lo/extend, hi*extend]; pass 1.0
    to stay inside the box.

    ``GENE_FLOOR`` is a hard stop regardless of ``extend``: a negative integral
    gain or negative damping is a different controller, not a further step.
    """
    if direction not in ("up", "down"):
        raise ValueError(f"direction must be 'up' or 'down', got {direction!r}")

    scale = GENE_SCALE.get(name, "linear")
    reach_hi = hi * extend if extend > 1.0 else hi
    reach_lo = lo / extend if (extend > 1.0 and lo > 0) else lo
    floor = GENE_FLOOR.get(name)
    if floor is not None:
        reach_lo = max(reach_lo, floor)

    values = []
    if scale == "integer":
        step = 1.0
        for n in range(1, max_steps + 1):
            v = round(x0 + (step * n if direction == "up" else -step * n))
            if v < reach_lo or v > reach_hi:
                break
            if v not in values:
                values.append(float(v))
    elif scale == "log" and x0 > 0:
        for n in range(1, max_steps + 1):
            v = x0 * (ratio ** n) if direction == "up" else x0 / (ratio ** n)
            if v < reach_lo or v > reach_hi:
                break
            values.append(float(v))
    else:
        delta = delta_frac * (hi - lo)
        if delta <= 0:
            return []
        for n in range(1, max_steps + 1):
            v = x0 + delta * n if direction == "up" else x0 - delta * n
            if v < reach_lo or v > reach_hi:
                # Clamp onto the reachable edge once, so an arm always tests
                # its own limit rather than stopping just short of it.
                edge = reach_hi if direction == "up" else reach_lo
                if values and abs(values[-1] - edge) > 1e-12:
                    values.append(float(edge))
                elif not values:
                    values.append(float(edge))
                break
            values.append(float(v))
    return values


def describe_box(controller: str, sweep_outer: bool = True) -> str:
    """One line per gene: name, box, scale. For run logs."""
    lines = []
    for name, lo, hi in genes_for(controller, sweep_outer):
        lines.append(f"    {name:<14} [{lo:g}, {hi:g}]  {GENE_SCALE.get(name, 'linear')}")
    return "\n".join(lines)
