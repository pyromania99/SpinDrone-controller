#!/usr/bin/env python
"""
| File: controller_factory.py
| Description: One place that turns a gain dict into a configured controller.

Why this module exists
──────────────────────
There were two implementations of this mapping and they disagreed.
``genetic_sweep._make_eigen/_make_pd/_make_mpc`` plus ``_apply_genes_*`` built
controllers for the search; ``robustness_sweep.make_controller`` built them for
the replay. They accepted different key sets — robustness_sweep took ``alpha``,
``int_max_xy``, ``int_max_z``, ``kp_rate`` and ``max_tilt_deg``; genetic_sweep
took none of those — so a configuration the search could express was not
necessarily one the replay could fly, and nothing checked.

This module is the union of the two, and it is where any third caller should
come. It may import the controllers (and therefore ``carb`` and the Pegasus
extension); the gene definitions and the ladder live next door in
gene_space.py, which stays pure so analysis tools can read the parameter space
without booting Isaac.

Unknown keys raise
──────────────────
``check_gain_keys`` refuses a dict carrying anything the controller cannot
apply, rather than ignoring it. That guard was written for the robustness
replay — where silently dropping an outer gain would fly every configuration at
the defaults while archiving it as tuned — and it matters more here, because
the ablation explorer generates gain dicts programmatically by the hundred. A
typo'd key that flies at a default and is recorded as tuned is the failure mode
this whole file exists to prevent.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

if str(Path(__file__).resolve().parent) not in sys.path:
    sys.path.insert(0, str(Path(__file__).resolve().parent))

import gene_space

from eigen_controller import EigenController                # noqa: E402
from cascaded_pd_controller import CascadedPDController     # noqa: E402
from lpv_mpc_controller import LPVMPCController             # noqa: E402


# Inner-loop keys each controller understands. The union of what the two
# previous implementations accepted.
#
# `beta` is deliberately absent for eigen: it appears identically in M_dyn and
# A_r so M_eff[2,2] = 0, and corrections[2] is discarded when tau is built — it
# cannot affect the vehicle, and advertising it as a tunable invites tuning
# against a parameter with no effect.
INNER_KEYS: dict = {
    "eigen": {"wn_mdyn", "b_mdyn", "Kp_att", "Kd_att", "alpha"},
    "pd":    {"Kp_att", "Kd_att", "Kd_yaw_rate"},
    # q_weight/r_weight/N are the constructor spellings; mpc_* are the archive
    # spellings. Both accepted so an older config dict still replays.
    "mpc":   {"mpc_q", "mpc_r", "mpc_N", "q_weight", "r_weight", "N"},
}

# Outer position-loop keys, in the canonical CONFIG_KEYS spelling. Only what
# each controller actually has: integral action is the LPV-MPC's alone, the
# tilt limit is Cascaded PD's alone.
OUTER_KEYS: dict = {
    "eigen": {"kp_xy", "kd_xy", "kp_z", "kd_z"},
    "pd":    {"kp_xy", "kd_xy", "kp_z", "kd_z", "max_tilt_deg"},
    "mpc":   {"kp_xy", "kd_xy", "ki_xy", "kp_z", "kd_z", "ki_z",
              "int_max_xy", "int_max_z", "kp_rate"},
}

# Gene-coordinate spellings for the outer loop. Accepted on input and converted
# by gene_space.outer_gains before anything else looks at the dict.
GENE_COORD_KEYS: frozenset = frozenset({"wn_xy", "zeta_xy"})

# Bookkeeping that rides along in a config dict without being a gain.
IGNORED_KEYS: frozenset = frozenset({
    "_score", "_source", "rank", "source", "score", "motor_moment", "id",
    "config_id", "config_rank", "config_source",
})


def known_keys(controller: str) -> set:
    """Every key ``controller`` will accept."""
    return (INNER_KEYS[controller] | OUTER_KEYS[controller]
            | set(GENE_COORD_KEYS) | set(IGNORED_KEYS))


def check_gain_keys(gains: dict, controller: str) -> None:
    """Raise if the dict carries gains this controller cannot apply.

    See the module docstring: ignoring an unrecognised key means flying the
    default while reporting it as tuned, with nothing in the archive revealing
    it. A robustness or ablation number attached to the wrong operating point
    is worse than no number.
    """
    if controller not in INNER_KEYS:
        raise ValueError(f"unknown controller {controller!r}; "
                         f"expected one of {sorted(INNER_KEYS)}")
    unknown = sorted(set(gains) - known_keys(controller))
    if unknown:
        raise ValueError(
            f"[controller_factory] configuration carries gains the "
            f"'{controller}' controller cannot apply: {unknown}. Known inner "
            f"{sorted(INNER_KEYS[controller])}, outer "
            f"{sorted(OUTER_KEYS[controller])}, gene coords "
            f"{sorted(GENE_COORD_KEYS)}. Refusing to fly it at the defaults "
            f"and report it as tuned."
        )


def _prepare(gains: dict, controller: str) -> dict:
    """Validate, then convert gene coordinates to physical gains."""
    check_gain_keys(gains, controller)
    g = dict(gains)
    # wn_xy/zeta_xy -> kp_xy/kd_xy. Done here so every downstream consumer sees
    # one vocabulary regardless of which coordinates the caller searched in.
    g.update(gene_space.outer_gains(g))
    return g


def _outer(g: dict, name: str, default: float) -> float:
    value = g.get(name)
    return default if value in (None, "") else float(value)


def make_controller(controller: str, gains: dict, results_file, target,
                    km: float = None):
    """Construct ``controller`` configured by ``gains``.

    Outer-loop gains are applied when present and left at the controller's own
    defaults when absent, so a config dict written before the outer loop was
    swept still replays exactly as it did.
    """
    g = _prepare(gains, controller)

    if controller == "eigen":
        return EigenController(
            results_file=results_file, target_position=target,
            km=km if km is not None else 6.0e-7,
            wn_mdyn=g.get("wn_mdyn", 18.0), b_mdyn=g.get("b_mdyn", 10.2),
            Kp_att=g.get("Kp_att", 7.0), Kd_att=g.get("Kd_att", 1.0),
            alpha=g.get("alpha", 0.5),
            kp_pos=_outer(g, "kp_xy", 0.50), kd_pos=_outer(g, "kd_xy", 1.0),
            kp_z=_outer(g, "kp_z", 15.0), kd_z=_outer(g, "kd_z", 8.0),
            # None keeps the historical unlimited torque; callers that want to match
            # PX4's saturation pass MC_EIG_TRQ_MAX through the gains dict.
            torque_max=g.get("torque_max"))

    if controller == "pd":
        # Kp_att/Kd_att are per-axis pairs here, and are a thrust differential
        # [N/rad] rather than the commanded rate they are in the Eigen
        # controller. See examples/utils/gain_seed.py.
        kp = g.get("Kp_att", 1.6)
        kd = g.get("Kd_att", 1.8)
        kp_xy, kd_xy = _outer(g, "kp_xy", 0.50), _outer(g, "kd_xy", 1.0)
        return CascadedPDController(
            results_file=results_file, target_position=target,
            Kp_att=(kp, kp) if np.isscalar(kp) else tuple(kp),
            Kd_att=(kd, kd) if np.isscalar(kd) else tuple(kd),
            Kd_yaw_rate=g.get("Kd_yaw_rate", 0.0),
            Kp_xy=(kp_xy, kp_xy), Kd_xy=(kd_xy, kd_xy),
            Kp_z=_outer(g, "kp_z", 15.0), Kd_z=_outer(g, "kd_z", 8.0),
            max_tilt_deg=_outer(g, "max_tilt_deg", 30.0))

    return LPVMPCController(
        results_file=results_file, target_position=target,
        km=km if km is not None else 6.0e-7,
        q_weight=g.get("mpc_q", g.get("q_weight", 10.0)),
        r_weight=g.get("mpc_r", g.get("r_weight", 0.01)),
        N=int(round(g.get("mpc_N", g.get("N", 12)))),
        kp_pos=_outer(g, "kp_xy", 0.50), kd_pos=_outer(g, "kd_xy", 1.0),
        ki_pos=_outer(g, "ki_xy", 0.05),
        kp_z=_outer(g, "kp_z", 15.0), kd_z=_outer(g, "kd_z", 8.0),
        ki_z=_outer(g, "ki_z", 2.0),
        int_max_xy=_outer(g, "int_max_xy", 2.0),
        int_max_z=_outer(g, "int_max_z", 4.0),
        kp_rate=_outer(g, "kp_rate", 4.0))


def apply_gains(inner, controller: str, gains: dict, km: float = None) -> None:
    """Re-configure an existing controller in place.

    Used by every sweep that spawns its drones once and re-parameterises them
    between passes, which is all of them — constructing a Multirotor per
    evaluation would dominate the run time.

    Only keys present in ``gains`` are touched, so a partial dict is a partial
    update rather than a reset to defaults.
    """
    g = _prepare(gains, controller)

    if controller == "eigen":
        for key, attr in (("wn_mdyn", "wn_mdyn"), ("b_mdyn", "b_mdyn"),
                          ("Kp_att", "Kp_att"), ("Kd_att", "Kd_att"),
                          ("alpha", "alpha")):
            if key in g:
                setattr(inner, attr, float(g[key]))
    elif controller == "pd":
        if "Kp_att" in g:
            inner.Kp_att = np.array([g["Kp_att"], g["Kp_att"]], dtype=float)
        if "Kd_att" in g:
            inner.Kd_att = np.array([g["Kd_att"], g["Kd_att"]], dtype=float)
        if "Kd_yaw_rate" in g:
            inner.Kd_yaw_rate = float(g["Kd_yaw_rate"])
    else:
        if "mpc_q" in g or "q_weight" in g:
            inner.q_weight = float(g.get("mpc_q", g.get("q_weight")))
        if "mpc_r" in g or "r_weight" in g:
            inner.r_weight = float(g.get("mpc_r", g.get("r_weight")))
        if "mpc_N" in g or "N" in g:
            # Safe in place: LPVMPCController rebuilds its OSQP problem from
            # self.N/q_weight/r_weight every update(), so a changed horizon
            # takes effect on the next step with no reconstruction.
            inner.N = int(round(g.get("mpc_N", g.get("N"))))

    _apply_outer(inner, g, controller)

    if km is not None and hasattr(inner, "km"):
        inner.km = float(km)


def _apply_outer(inner, g: dict, controller: str) -> None:
    """Set whichever outer gains are present, in place."""
    if controller == "pd":
        if "kp_xy" in g:
            inner.Kp_xy = np.array([g["kp_xy"], g["kp_xy"]], dtype=float)
        if "kd_xy" in g:
            inner.Kd_xy = np.array([g["kd_xy"], g["kd_xy"]], dtype=float)
        if "kp_z" in g:
            inner.Kp_z = float(g["kp_z"])
        if "kd_z" in g:
            inner.Kd_z = float(g["kd_z"])
        if "max_tilt_deg" in g:
            inner.max_tilt = np.deg2rad(float(g["max_tilt_deg"]))
        return
    for key, attr in (("kp_xy", "kp_pos"), ("kd_xy", "kd_pos"),
                      ("kp_z", "kp_z"), ("kd_z", "kd_z"),
                      ("ki_xy", "ki_pos"), ("ki_z", "ki_z"),
                      ("int_max_xy", "int_max_xy"), ("int_max_z", "int_max_z"),
                      ("kp_rate", "kp_rate")):
        if key in g and hasattr(inner, attr):
            setattr(inner, attr, float(g[key]))
