"""
| File: test_wing_config.py
| License: BSD-3-Clause. Copyright (c) 2026. All rights reserved.
| Description: Tests for the rotating-wing configuration schema and its derived quantities.

Configuration validation is not glamorous, but a silently-accepted bad parameter is worse
than a crash: it produces plausible-looking numbers that are wrong. Every invariant the
solver relies on is asserted here, from both directions -- the valid case is accepted and
the invalid case is refused.
"""

from __future__ import annotations

import numpy as np
import pytest

from _context import (
    InflowModel,
    RotatingWingConfig,
    build_wing_config,
)


# =======================================================================================
# Defaults
# =======================================================================================

def test_default_configuration_is_the_as_designed_wing():
    """The defaults must be the GOE 210 build schedule from the design report."""
    cfg = RotatingWingConfig()

    assert cfg.num_blades == 2
    assert cfg.root_radius_m == pytest.approx(0.200)
    assert cfg.tip_radius_m == pytest.approx(0.500)
    assert cfg.station_chord_m[0] == pytest.approx(0.300)
    assert cfg.station_pitch_deg[0] == pytest.approx(24.7)
    assert cfg.air_density_kg_m3 == pytest.approx(1.2)
    assert cfg.inflow_model == InflowModel.BEMT


def test_default_configuration_validates():
    """The shipped defaults must satisfy their own invariants."""
    RotatingWingConfig().validate()


def test_user_config_file_matches_the_schema_defaults():
    """``examples/utils/wing_params.py`` must not have drifted from the schema.

    The user-facing file is the single place a person edits. If someone changes a default
    in the schema without updating that file -- or vice versa -- results stop being
    reproducible from the documented parameters. This pins them together.
    """
    from_user_file = build_wing_config()
    from_schema = RotatingWingConfig()

    assert from_user_file.num_blades == from_schema.num_blades
    assert from_user_file.root_radius_m == pytest.approx(from_schema.root_radius_m)
    assert from_user_file.tip_radius_m == pytest.approx(from_schema.tip_radius_m)
    assert from_user_file.station_chord_m == from_schema.station_chord_m
    assert from_user_file.station_pitch_deg == from_schema.station_pitch_deg
    assert from_user_file.air_density_kg_m3 == pytest.approx(
        from_schema.air_density_kg_m3
    )
    assert from_user_file.use_root_loss == from_schema.use_root_loss
    assert from_user_file.inflow_model == from_schema.inflow_model


# =======================================================================================
# Derived quantities
# =======================================================================================

def test_disk_area_is_the_full_swept_circle():
    """The figure of merit is defined against the full disk, so this must be pi*R^2."""
    cfg = RotatingWingConfig(tip_radius_m=0.5)
    assert cfg.disk_area_m2() == pytest.approx(np.pi * 0.25)


def test_blade_area_integrates_the_station_table():
    """Blade area must be the chord integral times the blade count."""
    cfg = RotatingWingConfig(
        num_blades=2,
        root_radius_m=0.0,
        tip_radius_m=1.0,
        station_radius_m=[0.0, 1.0],
        station_chord_m=[0.2, 0.2],
        station_pitch_deg=[10.0, 10.0],
    )
    # Constant 0.2 m chord over a 1 m span, two blades.
    assert cfg.blade_area_m2() == pytest.approx(0.4)


def test_solidity_is_blade_area_over_disk_area():
    """Solidity is the ratio the induced velocity scales with; it must be consistent."""
    cfg = RotatingWingConfig()
    assert cfg.solidity() == pytest.approx(cfg.blade_area_m2() / cfg.disk_area_m2())
    # Sanity: a two-bladed wing of this planform is a high-solidity rotor, but not above 1.
    assert 0.05 < cfg.solidity() < 0.5


def test_uniform_blade_azimuths_are_generated_when_unspecified():
    """With no azimuths given, blades must be spread evenly around the disk."""
    assert RotatingWingConfig(num_blades=2).resolved_blade_azimuths_deg() == [0.0, 180.0]
    assert RotatingWingConfig(num_blades=4).resolved_blade_azimuths_deg() == [
        0.0, 90.0, 180.0, 270.0
    ]


def test_explicit_blade_azimuths_are_respected():
    """An asymmetric installation must be configurable."""
    cfg = RotatingWingConfig(num_blades=2, blade_azimuths_deg=[0.0, 90.0])
    assert cfg.resolved_blade_azimuths_deg() == [0.0, 90.0]


# =======================================================================================
# Validation: each invariant, refused
# =======================================================================================

@pytest.mark.parametrize(
    "kwargs, reason",
    [
        ({"num_blades": 0}, "zero blades"),
        ({"air_density_kg_m3": 0.0}, "zero air density"),
        ({"air_density_kg_m3": -1.0}, "negative air density"),
        ({"kinematic_viscosity_m2_s": 0.0}, "zero viscosity gives infinite Reynolds"),
        ({"root_radius_m": -0.1}, "negative root radius"),
        ({"tip_radius_m": 0.1, "root_radius_m": 0.2}, "tip inboard of root"),
        ({"num_stations": 1}, "cannot integrate with one station"),
        ({"inflow_model": "not_a_model"}, "unknown inflow model"),
        ({"inflow_relaxation": 0.0}, "zero relaxation never advances"),
        ({"inflow_relaxation": 1.5}, "relaxation above 1 diverges"),
        ({"inflow_max_iterations": 0}, "no iterations"),
        ({"tip_loss_floor": 0.0}, "zero floor allows unbounded induced velocity"),
        ({"tip_loss_floor": 1.5}, "loss factor above 1 adds energy"),
        ({"max_relative_speed_m_s": 0.0}, "zero speed clamp"),
        ({"moment_reference_offset_m": [0.0, 0.0]}, "offset is not a 3-vector"),
    ],
)
def test_invalid_configuration_is_refused(kwargs, reason):
    """Each physical invariant must be enforced at construction, not discovered later."""
    with pytest.raises(ValueError):
        RotatingWingConfig(**kwargs)


def test_station_table_must_be_consistent():
    """Mismatched or non-monotone station tables must be refused."""
    with pytest.raises(ValueError):  # lengths differ
        RotatingWingConfig(
            station_radius_m=[0.2, 0.3, 0.4],
            station_chord_m=[0.3, 0.2],
            station_pitch_deg=[20.0, 18.0, 16.0],
        )
    with pytest.raises(ValueError):  # radii not increasing
        RotatingWingConfig(
            station_radius_m=[0.2, 0.4, 0.3],
            station_chord_m=[0.3, 0.2, 0.1],
            station_pitch_deg=[20.0, 18.0, 16.0],
        )
    with pytest.raises(ValueError):  # non-positive chord
        RotatingWingConfig(
            station_radius_m=[0.2, 0.3],
            station_chord_m=[0.3, 0.0],
            station_pitch_deg=[20.0, 18.0],
        )
    with pytest.raises(ValueError):  # single station
        RotatingWingConfig(
            station_radius_m=[0.2], station_chord_m=[0.3], station_pitch_deg=[20.0]
        )


def test_blade_azimuth_count_must_match_blade_count():
    """An azimuth list of the wrong length is a silent geometry error; refuse it."""
    with pytest.raises(ValueError):
        RotatingWingConfig(num_blades=3, blade_azimuths_deg=[0.0, 180.0])


def test_unknown_override_is_rejected_by_the_user_config_builder():
    """A typo in an override must fail loudly rather than being ignored.

    ``build_wing_config(**overrides)`` is what command-line flags feed into. Silently
    dropping a misspelled field would mean a sweep quietly running the wrong geometry.
    """
    with pytest.raises(TypeError) as excinfo:
        build_wing_config(collective_pitch_offest_deg=2.0)  # deliberate typo
    assert "collective_pitch_offest_deg" in str(excinfo.value)


def test_overrides_take_effect():
    """Overrides must actually reach the configuration."""
    cfg = build_wing_config(
        num_blades=4, collective_pitch_offset_deg=-3.5, inflow_model=InflowModel.GLAUERT
    )
    assert cfg.num_blades == 4
    assert cfg.collective_pitch_offset_deg == pytest.approx(-3.5)
    assert cfg.inflow_model == InflowModel.GLAUERT


def test_each_builder_call_returns_an_independent_object():
    """Mutating one configuration must not affect the next.

    The station tables are lists; a shared mutable default would let one sweep's geometry
    leak into another's, which is exactly the kind of bug that takes a day to find.
    """
    first = build_wing_config()
    second = build_wing_config()

    first.station_chord_m[0] = 99.0
    first.collective_pitch_offset_deg = 12.0

    assert second.station_chord_m[0] == pytest.approx(0.300)
    assert second.collective_pitch_offset_deg == pytest.approx(0.0)


def test_polar_default_is_not_shared_between_configurations():
    """Two configurations must not share one polar object."""
    first = RotatingWingConfig()
    second = RotatingWingConfig()
    assert first.polar is not second.polar
