#!/usr/bin/env python
"""
| File: sweep_task.py
| Description: The flight task every controller sweep evaluates, in one place.

Why this module exists
──────────────────────
Every sweep decided three things for itself — where the drone starts, where it
is told to go, and when a run counts as failed. Five copies of ``check_oob``
and four of ``_validation_target`` had drifted apart, and the defaults they
shared turned out to make the sweeps unable to measure the thing they were
built to measure. This module owns all three so a change lands everywhere.

The task: take off, then translate
──────────────────────────────────
Runs used to spawn the vehicle **at** its target (``spawn_altitude`` defaulted
to ``target_altitude``, and the target's XY was the spawn XY). Initial position
error was therefore zero — measured at 3.6 µm on the archived runs — so the
attitude loop was never exercised and the stability score collapsed onto the
steady-state altitude droop, a constant no attitude gain can influence. On the
mm=0 PD grid every non-crashing cell scored 0.0516–0.0541: a 4.7 % spread
across a 5x range of Kp_att and a 15x range of Kd_att. Ranking on that is
ranking on noise, and it recommended Kp_att≈24, Kd_att≈0.53 — a configuration
that falls out of the sky nine seconds into a real flight.

What that configuration hits is a gyroscopic coning instability, not a takeoff
problem: it climbs to 1.448 m and holds for three seconds first. The airframe
spins up first-order toward its steady yaw rate (τ ≈ 1.8 s, ~9.8 rad/s at
km=2e-7), and the ``(Iyy − Izz)·q·r`` cross terms in Euler's equations couple
roll into pitch with a gain proportional to that rate. The inner loop damps each
axis independently, so past a critical spin rate the coning mode needs more
damping than Kd_att provides. Measured at km=2e-7: ``|eR|`` sits at 0.001 while
|r| < 6 rad/s, then runs 0.012 → 0.091 → 0.619 as |r| passes 8 rad/s, and the
vehicle tumbles. Kd_att=1.9 holds 1.448 m for 31 s under the same spin.

An unstable mode grows from whatever amplitude it is given, so a task that never
disturbs the vehicle lets it finish the window looking healthy. Every run now
starts on the ground at ``GROUND_ALTITUDE`` and is given a target offset
laterally from its spawn XY; the climb and the translation both excite the mode,
bringing the divergence forward by roughly 3 s. The growth is exponential, so a
bigger disturbance buys time only logarithmically — the onset itself is set by
the spin-up and no task change can move it. If a controller is swept whose
instability grows more slowly than this one, the window has to grow too, or the
metric has to stop requiring the crash to complete: a rising ``|eR|`` trend
condemns a configuration well before it reaches the floor.

Note for anyone reading old archives: this changes what ``score`` means. Runs
recorded before this module cannot be compared against runs recorded after it.
"""
from __future__ import annotations

import math
import os
from pathlib import Path

import numpy as np


# ── Where a sweep writes ─────────────────────────────────────────────────────

# Environment variable that redirects every sweep's output tree. Empty or unset
# means the normal ``examples/controller_tests/results``.
RESULTS_ROOT_ENV: str = "SWEEP_RESULTS_ROOT"


def results_root(base_dir) -> Path:
    """The directory a sweep should write its results into.

    Why this is not simply ``base_dir / "results"``
    ───────────────────────────────────────────────
    Five of the seven sweeps had no way to redirect their output, so every
    smoke run wrote straight into the real results tree at whatever motor
    moment it used. ``smoke_test.sh`` pins ``--motor-moments 4e-7``, so each
    smoke run silently replaced the km=4e-7 archives with its own 2x2 grid:
    measured after one such run, ``pd_sweep/mm_4.0e-07`` held 4 rows where the
    real grid had 42, and ``genetic_pd_sweep/mm_4.0e-07`` held 12 where the
    real search had written 496. The loss is silent and total — the smoke test
    reports success, and nothing downstream can tell a 4-row archive from a
    42-row one except by counting.

    ``eigen_stability_sweep.py`` was already safe because it takes a
    ``--run-tag`` that lands its output in a per-run subdirectory; this gives
    the other sweeps the same protection without each needing its own flag.

    Callers pass their own ``base_dir`` so a redirect is opt-in and the default
    is unchanged.
    """
    override = os.environ.get(RESULTS_ROOT_ENV, "").strip()
    if override:
        return Path(override)
    return Path(base_dir) / "results"

# ── Task geometry ────────────────────────────────────────────────────────────

# Spawn altitude for a ground start. The Iris rests on its legs at ~0.07 m;
# this is the value the sweeps used before the air start was introduced.
GROUND_ALTITUDE: float = 0.07

# Lateral distance from spawn XY to the target XY. 2.0 m matches the radius the
# step-response validation passes already used, so validation and the main pass
# now measure the same kind of manoeuvre.
DEFAULT_GOAL_RADIUS: float = 2.0

# Keep a drone's target inside its own lateral slice of the arena. Drones are
# spawned in a row along X at ``spacing`` apart; an unconstrained random
# direction lets two neighbours be sent toward each other (at spacing 3.5 and
# radius 2.0, adjacent targets can coincide), which turns a controller
# comparison into a collision test. See ``offset_target``.
SLICE_CLEARANCE: float = 0.75

# ── Flight-phase thresholds ──────────────────────────────────────────────────

# Fraction of target altitude that counts as "off the ground". A healthy run
# reaches 1.47 m of a 1.5 m target within 0.5 s, so this is not a tight gate.
TAKEOFF_ALTITUDE_FRAC: float = 0.5

# Seconds allowed to reach that altitude before the run is failed as a
# non-takeoff. Generous against the ~0.5 s a working controller needs.
TAKEOFF_TIMEOUT: float = 5.0

# Altitude below which an already-airborne vehicle is called crashed, as a
# fraction of target altitude and an absolute floor.
#
# This replaces a flat ``z < 0.15``, which was too low to be useful: it is
# under the 0.20 m floor of a drone that had already given up. In the archived
# mm=6e-7 PD grid, the Kd_att=0.2 run spent ten seconds skittering along the
# ground at z ≈ 0.26 and finished the run with ``oob_time = nan`` — recorded as
# degraded flight rather than the crash it was, and never terminated. At a
# 1.5 m target these values put the line at 0.375 m: a vehicle that has lost
# 75 % of its commanded altitude is not flying.
CRASH_ALTITUDE_FRAC: float = 0.25
CRASH_ALTITUDE_FLOOR: float = 0.30


def resolve_spawn_altitude(value) -> float:
    """Spawn altitude for a sweep, defaulting to a ground start.

    Sweeps pass their ``--spawn-altitude`` argument straight through. ``None``
    means "not specified", which now resolves to the ground rather than to the
    target altitude — see the module docstring for why the air start was
    removed. Pass ``--spawn-altitude 1.5`` to get the old behaviour back for a
    single run.
    """
    if value is None:
        return GROUND_ALTITUDE
    return float(value)


def offset_target(spawn_pos, altitude: float, radius: float, rng,
                  max_dx: float = None) -> tuple:
    """A target ``radius`` metres from ``spawn_pos`` in a random direction.

    The direction is uniform on the circle, so a sweep's drones do not all
    translate the same way and no gain can win by suiting one heading.

    ``max_dx`` bounds the X component so the target stays inside the drone's
    own lateral slice of the arena. The offset keeps its full ``radius`` — the
    X component is clamped and the Y component grown to compensate — so every
    drone still flies exactly the same distance. Without this, neighbouring
    drones in the spawn row can be sent to the same point and collide.
    """
    theta = float(rng.uniform(0.0, 2.0 * math.pi))
    dx = radius * math.cos(theta)
    dy = radius * math.sin(theta)

    if max_dx is not None and abs(dx) > max_dx:
        dx = math.copysign(max_dx, dx)
        # Preserve ‖(dx, dy)‖ == radius; keep the original Y heading, and pick
        # +Y when sin(theta) was exactly zero so the offset is never degenerate.
        dy_mag = math.sqrt(max(radius ** 2 - dx ** 2, 0.0))
        dy = math.copysign(dy_mag, dy if dy != 0.0 else 1.0)

    return (float(spawn_pos[0] + dx), float(spawn_pos[1] + dy), float(altitude))


def slice_max_dx(spacing: float, clearance: float = SLICE_CLEARANCE) -> float:
    """Largest X offset that keeps a target inside one drone's spawn slice.

    Returns 0.0 when the drones are packed too tightly for any X offset to be
    safe; ``offset_target`` then produces a pure ±Y offset, which is still the
    full radius and still never collides.

    **Zero is not a harmless degradation** — see ``require_lateral_freedom``.
    Any caller that packs a population densely enough to hit it must call that
    guard rather than silently accept a different task.
    """
    return max(0.0, 0.5 * float(spacing) - float(clearance))


def min_spacing_for_lateral_freedom(clearance: float = SLICE_CLEARANCE) -> float:
    """Spacing below which ``slice_max_dx`` returns 0 and the task degenerates."""
    return 2.0 * float(clearance)


def require_lateral_freedom(spacing: float, label: str = "sweep",
                            clearance: float = SLICE_CLEARANCE) -> float:
    """Return ``slice_max_dx(spacing)``, refusing to continue when it is 0.

    What this catches
    ─────────────────
    ``slice_max_dx`` clamps at 0 with ``max(0.0, ...)``, which looks defensive
    and is in fact a silent change of experiment: at ``max_dx = 0``
    ``offset_target`` clamps the X component to ±0 and puts the entire goal
    radius into Y, so every drone flies the *same* pure ±Y translation instead
    of the uniform-on-the-circle direction this module advertises.

    That is not hypothetical. At ``pop_size = 16`` in the default 24 m arena the
    genetic sweep computed ``spacing = 21/15 = 1.40 m`` and therefore
    ``max_dx = max(0, 0.70 - 0.75) = 0.0`` — so **every genetic run ever
    performed flew a pure ±Y translation**, while ``pd_sweep`` at 7 drones
    (spacing 3.50, max_dx 1.00) and the robustness sweeps (spacing 3.00,
    max_dx 0.75) got real directional diversity. Both populations were then
    pooled into the same Pareto front by ``select_pareto_top``, which is only
    valid if they flew the same task.

    Raising is the point: a sweep that cannot give its drones lateral freedom
    should be reconfigured — fewer drones per row, a wider arena, or a 2D
    layout — not quietly run against a different manoeuvre.
    """
    max_dx = slice_max_dx(spacing, clearance)
    if max_dx > 0.0:
        return max_dx
    raise ValueError(
        f"[{label}] spawn spacing {float(spacing):.3f} m leaves no lateral "
        f"freedom: max_dx = max(0, {0.5 * float(spacing):.3f} - "
        f"{float(clearance):.3f}) = 0, so every goal collapses to a pure ±Y "
        f"translation and the drones no longer fly the task this sweep claims. "
        f"Need spacing > {min_spacing_for_lateral_freedom(clearance):.2f} m — "
        f"use fewer drones per row, a wider --bounds, or a 2D spawn layout."
    )


def grid_layout(count: int, bounds, pitch_x: float = 3.5, pitch_y: float = 6.0,
                margin: float = 1.5, label: str = "sweep") -> dict:
    """Lay ``count`` drones out in rows so each keeps a usable X pitch.

    A single row is what collapses the spacing: fitting N drones across the
    arena divides the usable width by N-1, and past ~15 drones in a 24 m box
    that falls under ``2 × SLICE_CLEARANCE`` and the task degenerates. Wrapping
    into rows keeps the X pitch fixed at ``pitch_x`` regardless of population,
    which is what actually preserves the manoeuvre.

    Returns ``{"spacing", "max_dx", "columns", "rows", "positions"}`` where
    ``positions`` is a list of ``(x, y)`` spawn centres, and ``spacing`` is the
    X pitch actually used — the quantity that governs lateral freedom, since
    the slice being protected is the drone's own X slot.

    Raises via ``require_lateral_freedom`` if the requested pitch is too tight,
    and if the population needs more rows than the arena's Y extent allows.
    """
    x_min, x_max, y_min, y_max = (float(bounds[0]), float(bounds[1]),
                                  float(bounds[2]), float(bounds[3]))
    usable_x = (x_max - x_min) - 2.0 * margin
    usable_y = (y_max - y_min) - 2.0 * margin

    columns = max(1, min(int(count), int(usable_x // pitch_x) + 1))
    rows = int(np.ceil(count / columns))
    needed_y = (rows - 1) * pitch_y
    if needed_y > usable_y:
        raise ValueError(
            f"[{label}] {count} drones need {rows} rows at {pitch_y:g} m pitch "
            f"= {needed_y:.1f} m of Y, but only {usable_y:.1f} m is usable "
            f"inside {bounds[2]:g}..{bounds[3]:g}. Reduce the population, the "
            f"pitch, or widen --bounds."
        )

    max_dx = require_lateral_freedom(pitch_x, label=label)

    x_start = x_min + margin
    y_centre = 0.5 * (y_min + y_max)
    y_start = y_centre - 0.5 * (rows - 1) * pitch_y

    positions = []
    for i in range(int(count)):
        row, col = divmod(i, columns)
        positions.append((x_start + col * pitch_x, y_start + row * pitch_y))

    return {"spacing": float(pitch_x), "max_dx": float(max_dx),
            "columns": columns, "rows": rows, "positions": positions}


def check_goal_reachable(spawn_y: float, radius: float, bounds, label: str = "sweep") -> None:
    """Raise if a goal ``radius`` from ``spawn_y`` could leave the arena in Y.

    X needs no check: ``slice_max_dx`` already keeps the X component inside the
    drone's own slice, which is itself carved out of the arena. Y is the
    unconstrained axis — ``offset_target`` puts the *whole* radius into Y when
    the X component is clamped, so a large ``--goal-radius`` can walk a target
    through the wall of the room. Better to fail at startup with the numbers
    than to spend a GPU-hour scoring drones that flew into scenery.
    """
    y_min, y_max = float(bounds[2]), float(bounds[3])
    lo, hi = float(spawn_y) - float(radius), float(spawn_y) + float(radius)
    if lo < y_min or hi > y_max:
        raise ValueError(
            f"[{label}] goal radius {radius:g} m from spawn_y={spawn_y:g} reaches "
            f"y in [{lo:.2f}, {hi:.2f}], outside the arena's [{y_min:g}, {y_max:g}]. "
            f"Reduce --goal-radius or widen --bounds."
        )


class FlightMonitor:
    """Takeoff-aware flight-phase tracker, shared by every sweep's backend.

    Replaces five near-identical ``check_oob`` implementations whose rule was
    ``total_time > 2.0 and z < 0.15``. That rule was written for an air start
    and does not survive a ground start: the 2 s grace window exists purely to
    tolerate a vehicle legitimately sitting at z ≈ 0.07 during its climb, which
    also blinds it to a vehicle that never climbs at all.

    Two explicit phases instead:

    **Climbing** — until the vehicle first reaches ``takeoff_altitude``. Ground
    contact here is expected, not a crash. Failing to reach that altitude
    within ``takeoff_timeout`` ends the run as ``no_takeoff``.

    **Airborne** — after that point. Dropping below ``crash_altitude`` ends the
    run as ``crash``.

    Both failures set ``oob_time``, which is what ``sweep_metrics.compute_metrics``
    uses to pad the missing tail with the arena diagonal — so a run that never
    left the ground is charged for the flight it did not do, rather than
    scoring well for holding still.

    Arena walls are deliberately not a gate. A vehicle that drifts out of its
    lateral slice is still flying, and terminating it there made pass/fail
    depend on ``spacing/2`` — a function of how many drones were packed into
    the arena, not of control quality. Measured on a 576-run grid, 165 runs
    "failed" that way against exactly 1 real crash. The excursion is recorded
    as a metric instead and the run continues.
    """

    def __init__(self, target_altitude: float, label: str = "sweep",
                 takeoff_altitude: float = None, takeoff_timeout: float = None,
                 crash_altitude: float = None, log=None):
        self.target_altitude = float(target_altitude)
        if self.target_altitude <= 0.0:
            raise ValueError(f"[{label}] target_altitude must be > 0, got "
                             f"{self.target_altitude}")
        self.label = label
        self.takeoff_timeout = (
            float(takeoff_timeout) if takeoff_timeout is not None
            else TAKEOFF_TIMEOUT
        )

        # The two altitude thresholds must satisfy
        #     0 < crash_altitude < takeoff_altitude <= target_altitude
        # or the phase machine is nonsense. The fractions alone do not
        # guarantee it: CRASH_ALTITUDE_FLOOR is absolute, so at a 0.5 m target
        # the derived crash line (0.30 m) lands ABOVE the derived takeoff line
        # (0.25 m) and a vehicle would be declared crashed the instant it was
        # declared airborne. Derived values are clamped into order; explicit
        # ones are rejected, because silently moving a threshold the caller
        # asked for is worse than refusing to start.
        explicit = takeoff_altitude is not None or crash_altitude is not None
        crash = (float(crash_altitude) if crash_altitude is not None
                 else min(max(CRASH_ALTITUDE_FLOOR,
                              CRASH_ALTITUDE_FRAC * self.target_altitude),
                          0.6 * self.target_altitude))
        takeoff = (float(takeoff_altitude) if takeoff_altitude is not None
                   else min(max(TAKEOFF_ALTITUDE_FRAC * self.target_altitude,
                                1.25 * crash),
                            self.target_altitude))
        if not (0.0 < crash < takeoff <= self.target_altitude):
            raise ValueError(
                f"[{label}] need 0 < crash_altitude < takeoff_altitude <= "
                f"target_altitude, got crash={crash:g}, takeoff={takeoff:g}, "
                f"target={self.target_altitude:g}"
                + (" (from explicit arguments)" if explicit else "")
            )
        self.crash_altitude = crash
        self.takeoff_altitude = takeoff

        self._log = log
        self.reset()

    def reset(self) -> None:
        self.airborne = False
        self.failed = False
        self.failure = ""
        self.oob_time = float("nan")
        self.takeoff_time = float("nan")
        self.max_excursion = 0.0
        # Tracked only once airborne: with a ground start the pre-takeoff
        # minimum is always the spawn altitude, which says nothing about the
        # controller. Post-takeoff it is the transient-safety metric — how
        # close the vehicle came to the floor while actually flying.
        self.min_altitude = float("nan")

    def _fail(self, reason: str, total_time: float, z: float) -> None:
        self.failed = True
        self.failure = reason
        self.oob_time = float(total_time)
        if self._log is not None:
            self._log(f"[{self.label}] {reason.upper()} at z={z:.3f} "
                      f"t={total_time:.2f}s")

    def update(self, total_time: float, position, bounds) -> bool:
        """Advance the monitor one step. Returns True once the run is over.

        ``bounds`` is the drone's [xmin xmax ymin ymax zmin zmax] slice, used
        only to measure lateral excursion from its centre.
        """
        x, y, z = (float(position[0]), float(position[1]), float(position[2]))

        cx = 0.5 * (bounds[0] + bounds[1])
        cy = 0.5 * (bounds[2] + bounds[3])
        excursion = float(np.hypot(x - cx, y - cy))
        if excursion > self.max_excursion:
            self.max_excursion = excursion

        if self.failed:
            return True

        if not self.airborne:
            if z >= self.takeoff_altitude:
                self.airborne = True
                self.takeoff_time = float(total_time)
                self.min_altitude = z
            elif total_time >= self.takeoff_timeout:
                self._fail("no_takeoff", total_time, z)
                return True
            return False

        if not np.isfinite(self.min_altitude) or z < self.min_altitude:
            self.min_altitude = z
        if z < self.crash_altitude:
            self._fail("crash", total_time, z)
            return True
        return False
