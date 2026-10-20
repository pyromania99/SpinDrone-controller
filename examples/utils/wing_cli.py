"""
| File: wing_cli.py
| License: BSD-3-Clause. Copyright (c) 2026. All rights reserved.
| Description: Command-line plumbing shared by every sweep script that can fly the wing.

This module exists so that adding the rotating wing to a sweep script is two lines:

    from utils.wing_cli import add_wing_arguments, spawn_vehicle
    ...
    add_wing_arguments(parser)                       # in parse_args()
    vehicle = spawn_vehicle(prefix, usd, i, pos, quat, cfg, ARGS)   # instead of Multirotor(...)

RUNNING WITH AND WITHOUT THE WING
---------------------------------
Baseline, the stock quadrotor with no aerodynamic panels::

    isaac_run examples/controller_tests/sweeps/eigen_stability_sweep.py \\
        --motor-moments 6e-7 --omega-n-values 2 4 6 8 10 12 --duration 20 \\
        --headless --no-wing

With the as-designed GOE 210 wing::

    isaac_run examples/controller_tests/sweeps/eigen_stability_sweep.py \\
        --motor-moments 6e-7 --omega-n-values 2 4 6 8 10 12 --duration 20 \\
        --headless --wing

``--no-wing`` is the default, so an existing command line keeps its previous meaning and
every result recorded before the wing existed stays reproducible.

To measure what the WRONG handedness costs -- panels built to lift under a positive spin
on an airframe that spins negative::

    isaac_run examples/controller_tests/sweeps/eigen_stability_sweep.py \\
        --motor-moments 6e-7 --omega-n-values 2 --duration 20 \\
        --headless --wing --wing-rotation-sense 1

Geometry and aerodynamic parameters come from ``examples/utils/wing_params.py``. The flags
below cover only the handful of knobs worth sweeping from the command line; anything else
is edited in that file.
"""

from __future__ import annotations

import argparse
from typing import Any, Optional

# The sweep scripts put ``examples/utils`` itself on sys.path and import its modules by
# bare name, while the physics tests put ``examples`` on sys.path and import the package.
# Support both, so this module can be used from either.
try:
    from utils.wing_params import InflowModel, build_wing_config, describe
except ImportError:  # pragma: no cover - exercised only by the sweep scripts
    from wing_params import InflowModel, build_wing_config, describe

__all__ = [
    "add_wing_arguments",
    "wing_config_from_args",
    "spawn_vehicle",
    "wing_run_tag",
    "describe_wing_run",
]


def add_wing_arguments(parser: argparse.ArgumentParser) -> argparse.ArgumentParser:
    """Add the rotating-wing flags to an existing argument parser.

    Args:
        parser: The script's parser. Modified in place.

    Returns:
        The same parser, for chaining.
    """
    group = parser.add_argument_group(
        "rotating wing",
        "Airframe-mounted aerofoil panels driven by the vehicle's own spin. "
        "Geometry comes from examples/utils/wing_params.py.",
    )

    enable = group.add_mutually_exclusive_group()
    enable.add_argument(
        "--wing",
        dest="wing",
        action="store_true",
        help="Fly with the rotating wing enabled.",
    )
    enable.add_argument(
        "--no-wing",
        dest="wing",
        action="store_false",
        help="Baseline: stock quadrotor, no aerofoil panels (default).",
    )
    parser.set_defaults(wing=False)

    group.add_argument(
        "--wing-collective-deg",
        type=float,
        default=None,
        help="Rigid pitch offset applied to every blade station [deg]. The primary trim "
             "knob: positive increases angle of attack everywhere. Default: as built.",
    )
    group.add_argument(
        "--wing-blades",
        type=int,
        default=None,
        help="Number of aerofoil panels. Default: as configured (2).",
    )
    group.add_argument(
        "--wing-root-radius",
        type=float,
        default=None,
        help="Inboard end of the lifting surface [m]. Default: as configured (0.20).",
    )
    group.add_argument(
        "--wing-tip-radius",
        type=float,
        default=None,
        help="Outboard end of the lifting surface [m]. Default: as configured (0.50).",
    )
    group.add_argument(
        "--wing-inflow",
        type=str,
        choices=list(InflowModel.ALL),
        default=None,
        help="Induced-velocity model. 'bemt' resolves the radial distribution and is the "
             "default; 'glauert' is a cheaper uniform inflow; 'none' disables it.",
    )
    group.add_argument(
        "--wing-no-inertia",
        dest="wing_no_inertia",
        action="store_true",
        help="Do not add the wing's mass and inertia to the airframe rigid body. The "
             "panels stay aerodynamically active but become structurally massless. For "
             "ablation studies only: on the stock Iris the default panels add 9%% to mass "
             "and 28%% to I_zz, so omitting them makes the yaw-axis dynamics wrong.",
    )
    group.add_argument(
        "--no-rotor-gyro",
        dest="no_rotor_gyro",
        action="store_true",
        help="Disable the propellers' gyroscopic reaction on the airframe. This is a "
             "QUADROTOR term, not a wing term: because every motor turns the same way "
             "here, the four propeller momenta add to 16%% of the vehicle's angular "
             "momentum instead of cancelling. Use only to reproduce results recorded "
             "before the term existed, or to measure its size.",
    )
    group.add_argument(
        "--wing-rotation-sense",
        type=int,
        choices=(1, -1),
        default=None,
        help="Handedness of the panels: the sign of body yaw rate they are built to lift "
             "in. Must match the direction the airframe actually spins, which for this "
             "vehicle is negative. Default: as configured (-1). Set +1 only to measure "
             "what the wrong handedness costs.",
    )
    group.add_argument(
        "--wing-stations",
        type=int,
        default=None,
        help="Spanwise integration stations per blade. Default: as configured (24).",
    )

    return parser


def wing_config_from_args(args: Any):
    """Build the wing configuration implied by the parsed arguments.

    Args:
        args: Parsed arguments carrying at least a ``wing`` attribute. Scripts that have
            not adopted :func:`add_wing_arguments` are treated as wing-disabled, so this
            is safe to call unconditionally.

    Returns:
        A ``RotatingWingConfig``. Its ``enabled`` flag reflects ``--wing`` / ``--no-wing``.
    """
    overrides = {"enabled": bool(getattr(args, "wing", False))}

    for attr, field in (
        ("wing_collective_deg", "collective_pitch_offset_deg"),
        ("wing_blades", "num_blades"),
        ("wing_root_radius", "root_radius_m"),
        ("wing_tip_radius", "tip_radius_m"),
        ("wing_inflow", "inflow_model"),
        ("wing_stations", "num_stations"),
        ("wing_rotation_sense", "rotation_sense"),
    ):
        value = getattr(args, attr, None)
        if value is not None:
            overrides[field] = value

    if getattr(args, "wing_no_inertia", False):
        overrides["apply_inertia_to_body"] = False

    return build_wing_config(**overrides)


def spawn_vehicle(
    stage_prefix: str,
    usd_file: str,
    vehicle_id: int,
    init_pos,
    init_orientation,
    config,
    args: Any,
):
    """Create the vehicle the arguments ask for.

    With the wing disabled this returns a stock
    :class:`~pegasus.simulator.logic.vehicles.multirotor.Multirotor`, constructed exactly
    as it was before the wing existed. With the wing enabled it returns a
    :class:`~pegasus.simulator.logic.vehicles.multirotors.spinning_wing_multirotor.SpinningWingMultirotor`,
    which inherits all of that behaviour unchanged and adds the aerodynamic panels.

    The ``config`` object passed in is the script's own ``MultirotorConfig``, already
    carrying its thrust curve and backends. The wing configuration is attached to it
    rather than requiring the caller to build a different config class.

    Args:
        stage_prefix: Prim path to spawn at.
        usd_file: Airframe USD asset.
        vehicle_id: Pegasus vehicle id.
        init_pos: Initial position, inertial ENU [m].
        init_orientation: Initial attitude quaternion [qx, qy, qz, qw].
        config: The script's ``MultirotorConfig``.
        args: Parsed command-line arguments.

    Returns:
        The constructed vehicle.
    """
    # Imported lazily: these modules pull in Isaac Sim, which must not be a requirement
    # for the offline physics tests that import wing_params from this same package.
    from pegasus.simulator.logic.vehicles.multirotor import Multirotor

    # Airframe dynamics parameters come from wing_params.py, so that one file governs
    # every dynamics setting rather than some of them living as class defaults.
    try:
        from utils.wing_params import (
            ENABLE_ROTOR_GYROSCOPICS, ROTOR_INERTIA_KG_M2,
        )
    except ImportError:  # pragma: no cover - exercised only by the sweep scripts
        from wing_params import ENABLE_ROTOR_GYROSCOPICS, ROTOR_INERTIA_KG_M2

    config.enable_rotor_gyroscopics = bool(
        ENABLE_ROTOR_GYROSCOPICS and not getattr(args, "no_rotor_gyro", False)
    )
    config.rotor_inertia_kg_m2 = float(ROTOR_INERTIA_KG_M2)

    wing_config = wing_config_from_args(args)

    if not wing_config.enabled:
        return Multirotor(
            stage_prefix, usd_file, vehicle_id, init_pos, init_orientation, config=config
        )

    from pegasus.simulator.logic.vehicles.multirotors.spinning_wing_multirotor import (
        SpinningWingMultirotor,
    )

    # SpinningWingMultirotor reads the wing parameters off the config object, so the
    # script's existing MultirotorConfig can simply carry them.
    config.wing = wing_config
    return SpinningWingMultirotor(
        stage_prefix, usd_file, vehicle_id, init_pos, init_orientation, config=config
    )


def wing_run_tag(args: Any) -> str:
    """Short filesystem-safe tag identifying the wing configuration of a run.

    Intended for result directory names so that winged and wingless sweeps do not
    overwrite each other.

    Returns:
        ``"nowing"``, or ``"wing"`` with any non-default knobs appended, e.g.
        ``"wing_col-2.0_b4"``.
    """
    if not getattr(args, "wing", False):
        return "nowing"

    parts = ["wing"]
    collective = getattr(args, "wing_collective_deg", None)
    if collective:
        parts.append(f"col{collective:+.1f}")
    blades = getattr(args, "wing_blades", None)
    if blades is not None:
        parts.append(f"b{blades}")
    inflow = getattr(args, "wing_inflow", None)
    if inflow is not None:
        parts.append(str(inflow))
    sense = getattr(args, "wing_rotation_sense", None)
    if sense is not None:
        parts.append(f"sense{sense:+d}")
    if getattr(args, "wing_no_inertia", False):
        parts.append("noinertia")

    return "_".join(parts)


def describe_wing_run(args: Any) -> str:
    """Human-readable summary of the wing configuration, for the top of a run log.

    A run record that does not state its geometry cannot be reproduced, so sweep scripts
    should print this before starting.
    """
    return describe(wing_config_from_args(args))
