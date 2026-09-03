#!/usr/bin/env python3
"""
motor_failure.py — kill one rotor once the vehicle is settled, and watch.

What this does
──────────────
Flies the EigenController (its own current defaults — the km = 6e-7 robust
operating point) to a hover target, waits until it is genuinely settled, then
sets ONE randomly chosen rotor to zero for the rest of the run.

Where the failure is injected
─────────────────────────────
``Multirotor.update`` reads its rotor commands from
``self._backends[0].input_reference()`` and hands them straight to
``QuadraticThrustCurve``. Zeroing one element of that vector is therefore
exactly a stopped motor: the thrust curve computes F = k * w^2 = 0 for it, and
it contributes nothing to the rolling moment either. Nothing else has to be
touched, and the controller is not told — it keeps commanding all four and only
sees the consequences through the state, which is the point.

Why settling is detected rather than timed
──────────────────────────────────────────
"Once it stabilises" is a property of the flight, not a wall-clock constant. A
fixed delay would fire mid-transient on a slow run and long after settling on a
fast one, and the two produce completely different failure responses. The
trigger here requires the position error to stay inside --settle-radius for
--settle-hold continuous seconds, so the vehicle is demonstrably steady when
the motor dies.

Note on this airframe: rot_dir is [-1, -1, -1, -1], i.e. every rotor turns the
same way. There is no yaw authority to begin with (the controller's tau[2] is
hardcoded 0), so the vehicle already spins up to r_ss ~ 5.16e7 * km. Losing a
rotor removes a quarter of the lift and unbalances roll/pitch hard.

Usage
  isaac_run examples/controller_tests/motor_failure.py                 # watch it
  isaac_run examples/controller_tests/motor_failure.py --headless
  isaac_run examples/controller_tests/motor_failure.py --rotor 2       # pick one
  isaac_run examples/controller_tests/motor_failure.py --seed 7
"""
from __future__ import annotations
import argparse, sys
from pathlib import Path
import carb
from isaacsim import SimulationApp


def parse_args():
    p = argparse.ArgumentParser(description="Single-rotor failure at steady state")
    p.add_argument("--rotor", type=int, default=None, choices=[0, 1, 2, 3],
                   help="Rotor to fail. Default: chosen at random.")
    p.add_argument("--seed", type=int, default=None,
                   help="RNG seed for the rotor choice, so a run reproduces. "
                        "Default: nondeterministic.")
    p.add_argument("--duration", type=float, default=30.0,
                   help="Total simulated seconds.")
    p.add_argument("--target-altitude", type=float, default=1.5)
    p.add_argument("--settle-radius", type=float, default=0.15,
                   help="Position error [m] that counts as settled.")
    p.add_argument("--settle-hold", type=float, default=1.5,
                   help="Seconds the error must stay inside --settle-radius "
                        "before the motor is cut.")
    p.add_argument("--settle-timeout", type=float, default=20.0,
                   help="Give up waiting for a settle and end the run. Prevents "
                        "a vehicle that never settles from silently producing a "
                        "run with no failure in it.")
    p.add_argument("--km", type=float, default=None,
                   help="Motor moment. Default: the controller's own (6e-7).")
    p.add_argument("--out", type=str, default=None,
                   help="npz path. Default: results/motor_failure/run.npz")
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

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "utils"))
from eigen_controller import EigenController
import sim_clock
import sweep_task as task
import rotor_failure


def main():
    timeline = omni.timeline.get_timeline_interface()
    render = not ARGS.headless
    rng = np.random.default_rng(ARGS.seed)

    base = Path(__file__).resolve().parent
    out = (Path(ARGS.out) if ARGS.out
           else task.results_root(base) / "motor_failure" / "run.npz")
    out.parent.mkdir(parents=True, exist_ok=True)

    pg = PegasusInterface()
    pg._world = World(**pg._world_settings)
    world = pg.world
    pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])
    for _ in range(20):
        world.step(render=render)

    spawn = [0.0, 0.0, task.resolve_spawn_altitude(None)]
    target = (spawn[0], spawn[1], ARGS.target_altitude)

    # Constructed with NO gain overrides: the point is to fly whatever
    # eigen_controller.py currently defaults to.
    inner = EigenController(target_position=target, results_file=str(out))
    if ARGS.km is not None:
        inner.km = float(ARGS.km)
    km = inner.km

    monitor = task.FlightMonitor(ARGS.target_altitude, label="motor-failure",
                                 log=carb.log_warn)
    backend = rotor_failure.RotorFailureBackend(
        inner, results_file=str(out), rotor=ARGS.rotor, rng=rng,
        auto_fail=True, settle_radius=ARGS.settle_radius,
        settle_hold=ARGS.settle_hold, monitor=monitor, log=carb.log_warn)
    cfg = MultirotorConfig()
    cfg.backends = [backend]
    cfg.thrust_curve = QuadraticThrustCurve(config={"motor_moment": km})

    Multirotor("/World/failure_test", ROBOTS["Iris"], 0, spawn,
               Rotation.from_euler("XYZ", [0., 0., 0.], degrees=True).as_quat(),
               config=cfg)

    world.reset()
    for _ in range(10):
        world.step(render=render)

    bounds = [-25., 25., -25., 25., 0., 30.]
    carb.log_warn(
        f"[motor-failure] km={km:.2e} (r_ss ~ {5.163e7 * km:.1f} rad/s), "
        f"target={target}, settle |e|<={ARGS.settle_radius} m for "
        f"{ARGS.settle_hold} s, rotor="
        + ("random" if ARGS.rotor is None else str(ARGS.rotor)))

    # Read once here rather than after the loop: the "never settled" branch
    # below reports the position error and runs before the summary does.
    tgt = np.asarray(inner.hover_target, dtype=float)

    timeline.play()
    n_steps, step_dt = sim_clock.steps_for_duration(world, ARGS.duration)
    for _ in range(n_steps):
        if not simulation_app.is_running():
            break
        world.step(render=render)
        if backend.check_oob(bounds):
            carb.log_warn(f"[motor-failure] run ended: "
                          f"{backend.monitor.failure} at t={backend.total_time:.2f}s")
            break
        # A vehicle that never settles would otherwise fly the whole duration
        # and produce a run with no failure in it, which looks like a result.
        if (backend.failed_rotor is None
                and backend.total_time > ARGS.settle_timeout):
            carb.log_warn(
                f"[motor-failure] never settled within {ARGS.settle_timeout}s "
                f"(|e| = {np.linalg.norm(tgt - backend.position):.3f} m) — "
                f"no motor was cut; ending.")
            break
    timeline.stop()
    backend.stop()

    # ── Summary ─────────────────────────────────────────────────────────────
    t = np.array(backend.t_hist)
    p = np.vstack(backend.pos_hist) if backend.pos_hist else np.zeros((0, 3))
    e = (tgt - p) if len(p) else np.zeros((0, 3))
    w = np.vstack(backend.w_hist) if backend.w_hist else np.zeros((0, 3))

    print("\n" + "=" * 68)
    print(f"  motor failure — rotor {backend.failed_rotor}, km = {km:.2e}")
    print("=" * 68)
    if backend.failure_time is None:
        print("  no failure injected (never settled)")
    else:
        pre = t < backend.failure_time
        post = ~pre
        print(f"  settled at        t = {backend.settle_time:.2f} s")
        print(f"  rotor cut at      t = {backend.failure_time:.2f} s")
        print(f"  survived after cut  {t[-1] - backend.failure_time:.2f} s "
              f"(run ended {t[-1]:.2f} s)")
        print(f"  outcome           {backend.monitor.failure or 'still flying'}")
        if pre.any():
            print(f"\n  before cut: |e| = {np.linalg.norm(e[pre], axis=1).mean():.4f} m mean, "
                  f"alt {p[pre, 2].mean():.3f} m, |r| = {np.abs(w[pre, 2]).mean():.2f} rad/s")
        if post.any():
            print(f"  after cut:  |e| = {np.linalg.norm(e[post], axis=1).max():.4f} m max, "
                  f"alt {p[post, 2].min():.3f} m min, "
                  f"|p,q| max = {np.abs(w[post, :2]).max():.2f} rad/s, "
                  f"|r| max = {np.abs(w[post, 2]).max():.2f} rad/s")
    print("=" * 68 + "\n")

    try: world.stop()
    except Exception: pass
    simulation_app.close()


if __name__ == "__main__":
    main()
