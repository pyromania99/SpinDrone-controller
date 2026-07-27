#!/usr/bin/env python
"""
| File: sim_clock.py
| Description: Convert a requested duration into world.step() calls, correctly.

The bug this exists to prevent
──────────────────────────────
Every sweep drove its run loop like this::

    sim_time = 0.0
    dt = 1.0 / 60.0
    while simulation_app.is_running() and sim_time < ARGS.duration:
        world.step(render=render_sim)
        sim_time += dt

That hard-codes an assumption — one ``world.step()`` advances 1/60 s — which
is simply false here. PegasusInterface configures ``physics_dt = 1/250`` and
one ``world.step()`` advances one physics step, so each iteration is 1/250 s.
The loop counted 25 s worth of iterations at 1/60 and delivered 1500 / 250 =
**6.0 s** of flight. Nothing failed; the runs were silently a quarter as long
as requested, and the telemetry (1506 samples at 250 Hz, t_end = 6.02 s) was
the only place it showed.

That matters beyond run length: the vehicle's yaw spin-up is first-order with
τ ≈ 1.8 s, so a 25 s run reaches a settled spin rate while a 6 s run is caught
mid-transient — and any metric reduced over "the settled tail" is then
measuring the transient instead.

The fix is to ask the world what its timestep actually is, and to cross-check
the result against the time the vehicles themselves accumulated.
"""
from __future__ import annotations

DEFAULT_PHYSICS_DT: float = 1.0 / 250.0


def physics_dt(world, default: float = DEFAULT_PHYSICS_DT) -> float:
    """Seconds of simulated time advanced by one ``world.step()`` call."""
    for getter in ("get_physics_dt", "get_physics_time_step"):
        fn = getattr(world, getter, None)
        if fn is None:
            continue
        try:
            dt = float(fn())
        except Exception:
            continue
        if dt and dt > 0.0:
            return dt
    return default


def steps_for_duration(world, duration: float,
                       default: float = DEFAULT_PHYSICS_DT) -> tuple:
    """(n_steps, dt) — how many ``world.step()`` calls cover `duration` seconds.

    Use as::

        n_steps, step_dt = steps_for_duration(world, ARGS.duration)
        for _ in range(n_steps):
            if not simulation_app.is_running():
                break
            world.step(render=render_sim)
            ...

    A bounded loop rather than a ``while`` on an accumulator: it cannot run
    forever if every vehicle stops advancing its own clock (which is what the
    OOB gating does to ``total_time``).
    """
    dt = physics_dt(world, default)
    return max(1, int(round(float(duration) / dt))), dt


def elapsed_from_backends(backends) -> float:
    """Largest ``total_time`` across backends — simulated seconds actually flown.

    Ground truth, since it is accumulated from the dt the physics callback
    passes in rather than from any assumption this module makes.
    """
    best = 0.0
    for backend in backends or ():
        value = getattr(backend, "total_time", None)
        if value is None:
            inner = getattr(backend, "inner", None)
            value = getattr(inner, "total_time", None) if inner else None
        try:
            value = float(value)
        except (TypeError, ValueError):
            continue
        if value > best:
            best = value
    return best


def check_elapsed(backends, requested: float, log, tol: float = 0.10,
                  label: str = "sweep") -> float:
    """Warn when flown time misses `requested` by more than `tol` (fraction).

    Manual diagnostic — deliberately NOT called by the sweeps. It was written
    when runs started in the air and flew to completion, so a short pass meant a
    clock bug. Runs now start on the ground and a failing configuration is
    expected to crash, which ends the pass early for a legitimate reason: whole
    passes legitimately terminate short whenever every drone in them fails (an
    entire bad Kd_att column, say). Wired into the run loop it would fire on
    those, and a warning that cries wolf on normal results is worse than none.

    Reach for it when a pass looks suspiciously short and you want to know
    whether the clock or the controller is responsible. `steps_for_duration`
    above is the part that prevents the original bug, and that IS used
    everywhere.
    """
    flown = elapsed_from_backends(backends)
    if requested > 0 and abs(flown - requested) > tol * requested:
        log(f"[{label}] flew {flown:.2f}s of the {requested:.2f}s requested "
            f"({100.0 * flown / requested:.0f}%) — check physics_dt vs the "
            f"loop's step count if no vehicle stopped early.")
    return flown
