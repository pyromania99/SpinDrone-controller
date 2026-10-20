"""
| File: test_dynamic_inflow.py
| License: BSD-3-Clause. Copyright (c) 2026. All rights reserved.
| Description: Tests for first-order dynamic inflow.

WHY THE MODEL EXISTS
--------------------
Momentum theory says what induced velocity eventually balances a given thrust. It does not
say how quickly the flow gets there, and the answer is not "instantly" -- the air column
through the disc has mass. For this wing the resulting time constant is about 70 ms, which
is 0.54 of the roll timescale at 8 rad/s and 0.81 of a 12 rad/s controller bandwidth. An
instantaneous inflow therefore overstates aerodynamic damping and phase margin in exactly
the regime a controller study examines.

It is also the only inflow model here with a bounded per-step cost, because it advances a
state instead of iterating a fixed point.
"""

from __future__ import annotations

import numpy as np
import pytest

from _context import InflowModel, RotatingWing, build_wing_config

DT = 1.0 / 250.0
SPIN = np.array([0.0, 0.0, -18.0])
STILL = np.zeros(3)


def _run(wing, steps, v=STILL, om=SPIN, dt=DT):
    last = None
    for _ in range(steps):
        last = wing.compute_loads(
            linear_body_velocity_m_s=v, angular_velocity_body_rad_s=om, dt=dt
        )
    return last


def _dynamic(**kw):
    return RotatingWing(build_wing_config(inflow_model=InflowModel.DYNAMIC, **kw))


# =======================================================================================
# The steady limit
# =======================================================================================

def test_settles_to_the_uniform_inflow_steady_solution():
    """Left alone, the state must converge on what the algebraic uniform model gives.

    This is the central consistency check. Dynamic inflow is the Glauert uniform-inflow
    model with a lag attached, so once the lag has played out the two must agree. If they
    did not, the dynamics would be converging on the wrong equilibrium and every transient
    result would be wrong at its endpoint too.
    """
    settled = _run(_dynamic(), 400)
    glauert = RotatingWing(
        build_wing_config(inflow_model=InflowModel.GLAUERT)
    ).hover_loads(-18.0)

    assert settled.thrust_n == pytest.approx(glauert.thrust_n, rel=0.02)
    assert float(np.mean(settled.induced_velocity_m_s)) == pytest.approx(
        float(np.mean(glauert.induced_velocity_m_s)), rel=0.02
    )


def test_the_steady_state_is_a_fixed_point():
    """Once settled, further steps must not move it."""
    wing = _dynamic()
    _run(wing, 400)
    a = _run(wing, 1)
    b = _run(wing, 200)
    assert b.thrust_n == pytest.approx(a.thrust_n, rel=1e-9)


# =======================================================================================
# The transient
# =======================================================================================

def test_inflow_builds_up_rather_than_appearing_instantly():
    """The defining behaviour: at the first step the inflow is still near zero."""
    wing = _dynamic()
    first = _run(wing, 1)
    settled = _run(wing, 400)

    v_first = float(np.mean(first.induced_velocity_m_s))
    v_settled = float(np.mean(settled.induced_velocity_m_s))

    assert v_first < 0.15 * v_settled, (
        "inflow reached a large fraction of its final value in one 4 ms step, which is "
        "the instantaneous behaviour this model exists to remove"
    )
    assert v_settled > 0.5


def test_time_constant_matches_the_apparent_mass_derivation():
    """tau must equal m_a/(dT/dv), which in hover is 2R/(3 pi v).

    Checked against the closed form rather than a recorded number: the apparent mass of
    an impermeable disc is (8/3) rho R^3, and linearising T = 2 rho A v^2 in hover gives
    dT/dv = 4 rho A v.
    """
    wing = _dynamic()
    settled = _run(wing, 500)

    cfg = wing.config
    v_i = float(np.mean(settled.induced_velocity_m_s))
    expected = 2.0 * cfg.tip_radius_m / (3.0 * np.pi * v_i)

    assert wing.inflow_time_constant_s == pytest.approx(expected, rel=0.05)
    # And it must be the tens-of-milliseconds value that makes this matter at all.
    assert 0.03 < wing.inflow_time_constant_s < 0.20


def test_step_response_reaches_63_percent_in_one_time_constant():
    """The signature of a first-order lag, measured rather than assumed."""
    wing = _dynamic()
    _run(wing, 500)
    v_final = float(np.mean(wing.compute_loads(
        linear_body_velocity_m_s=STILL, angular_velocity_body_rad_s=SPIN, dt=DT
    ).induced_velocity_m_s))
    tau = wing.inflow_time_constant_s

    fresh = _dynamic()
    n_tau = max(1, int(round(tau / DT)))
    at_tau = float(np.mean(_run(fresh, n_tau).induced_velocity_m_s))

    # 1 - 1/e = 0.632. The tolerance covers tau drifting as the operating point moves
    # during the very transient being measured.
    assert at_tau / v_final == pytest.approx(0.632, abs=0.15)


def test_a_coarse_timestep_stays_stable():
    """A timestep far larger than tau must not destabilise the model.

    Forward Euler would go unstable for dt > 2 tau. The exponential update cannot, because
    ``1 - exp(-dt/tau)`` saturates at one instead of overshooting.

    What it DOES degenerate into at dt >> tau is an undamped fixed-point iteration -- the
    state simply jumps to the quasi-steady target each step, and the target then moves
    because the loads were evaluated at the old state. That still converges here, but over
    several steps rather than one, which is why this checks the settled value rather than
    the third step.
    """
    reference = _run(_dynamic(), 500).thrust_n

    for dt in (0.05, 0.5, 5.0):
        wing = _dynamic()
        history = [_run(wing, 1, dt=dt).thrust_n for _ in range(40)]
        assert np.all(np.isfinite(history)), f"dt={dt} produced non-finite thrust"
        assert history[-1] == pytest.approx(reference, rel=0.02), (
            f"dt={dt} settled at {history[-1]:.4f} N against {reference:.4f} N"
        )
        assert max(history) < 5.0 * abs(reference), f"dt={dt} overshot badly"


# =======================================================================================
# Cost: the other reason the model exists
# =======================================================================================

def test_the_solver_never_iterates():
    """Exactly one 'iteration' on every call, in every flight state.

    This is what distinguishes a bounded worst case from a bounded average. The BEMT path
    runs between one and thirty-two passes depending on the state, each re-evaluating the
    section polar at every element; this path does the same work every time.
    """
    wing = _dynamic()
    rng = np.random.default_rng(0)
    for _ in range(40):
        v = rng.uniform(-8.0, 8.0, 3)
        om = np.array([rng.uniform(-10, 10), rng.uniform(-10, 10), rng.uniform(-30, 30)])
        loads = wing.compute_loads(
            linear_body_velocity_m_s=v, angular_velocity_body_rad_s=om, dt=DT
        )
        assert loads.inflow_iterations == 1


# =======================================================================================
# Contract
# =======================================================================================

def test_missing_timestep_is_refused():
    """Falling back to a steady solve would silently behave as a different model."""
    wing = _dynamic()
    with pytest.raises(ValueError):
        wing.compute_loads(
            linear_body_velocity_m_s=STILL, angular_velocity_body_rad_s=SPIN
        )
    with pytest.raises(ValueError):
        wing.compute_loads(
            linear_body_velocity_m_s=STILL, angular_velocity_body_rad_s=SPIN, dt=0.0
        )


def test_the_algebraic_models_still_ignore_dt():
    """Passing dt to a steady model must be harmless, so callers need no special casing."""
    for model in (InflowModel.NONE, InflowModel.BEMT, InflowModel.GLAUERT):
        wing = RotatingWing(build_wing_config(inflow_model=model))
        with_dt = wing.compute_loads(
            linear_body_velocity_m_s=STILL, angular_velocity_body_rad_s=SPIN, dt=DT
        )
        wing.reset()
        without = wing.compute_loads(
            linear_body_velocity_m_s=STILL, angular_velocity_body_rad_s=SPIN
        )
        assert with_dt.thrust_n == pytest.approx(without.thrust_n, rel=1e-12)


def test_reset_clears_the_inflow_state():
    """A teleport or episode restart must not leak a converged inflow across it."""
    wing = _dynamic()
    settled = _run(wing, 400)
    wing.reset()
    first = _run(wing, 1)
    assert float(np.mean(first.induced_velocity_m_s)) < 0.15 * float(
        np.mean(settled.induced_velocity_m_s)
    )


def test_descent_and_climb_both_stay_finite():
    """Axial flight changes the sign of the inflow balance; it must stay well posed."""
    for vz in (-4.0, -1.0, 1.0, 4.0):
        loads = _run(_dynamic(), 400, v=np.array([0.0, 0.0, vz]))
        assert np.all(np.isfinite(loads.force_body_n))
        assert np.all(np.isfinite(loads.induced_velocity_m_s))
