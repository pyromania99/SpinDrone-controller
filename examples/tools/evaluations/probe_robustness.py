#!/usr/bin/env python3
"""
probe_robustness.py — establish the three facts the robustness study depends on.

Everything the robustness sweep wants to do rests on assumptions about the
simulator that have never been checked. Each one, if wrong, fails *silently*:
the runs complete, the CSVs look fine, and the numbers mean something other
than what they claim. So check them before writing the driver, not after.

Q1  What mass and inertia is the vehicle ACTUALLY simulated with, and what do
    the three controllers believe?

    The three disagree by construction. EigenController syncs both mass and
    inertia from the USD, but only when the attribute carries an authored
    opinion; CascadedPDController syncs mass alone (it has no inertia term);
    LPVMPCController syncs nothing and runs on its constructor defaults of
    m=1.2 kg, diag(0.01, 0.01, 0.02). If the USD authors nothing, the three
    controllers are flying three different vehicles — and "robustness to
    inertia mismatch" has no zero point to measure from.

Q2  Does the sync actually fire?

    Reported as before/after on a live vehicle rather than inferred from the
    code, because ``HasAuthoredValueOpinion()`` is the kind of guard that
    quietly evaluates false forever.

Q3  Can the plant be perturbed at runtime, and does an external force applied
    from a Backend reach the rigid body?

    The robustness driver needs to scale mass/inertia per drone (inertia axis)
    and to inject a wind force from inside ``Backend.update`` (wind axis). USD
    attribute writes on an active PhysX body are exactly the kind of thing that
    is accepted and then ignored — see probe_reset.py, where ``set_world_pose``
    turned out not to reposition an active rigid body at all.

Run it with isaac_run / run_all_sweeps.sh's isaac_python. It prints tables
tagged PROBE and exits; it writes nothing.
"""
from __future__ import annotations

import sys
from pathlib import Path

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True, "hide_ui": True,
                                "disable_viewport_updates": True,
                                "multi_gpu": False, "max_gpu_count": 1,
                                "active_gpu": 0})

import carb                             # noqa: E402
import numpy as np                      # noqa: E402
import omni.timeline                    # noqa: E402
from isaacsim.core.api import World     # noqa: E402
from scipy.spatial.transform import Rotation  # noqa: E402

for _parent in Path(__file__).resolve().parents:
    _cand = _parent / "extensions" / "pegasus.simulator"
    if _cand.exists():
        sys.path.insert(0, str(_cand))
        break

from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS      # noqa: E402
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface  # noqa: E402
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig  # noqa: E402
from pegasus.simulator.logic.backends import Backend                      # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "utils"))
from eigen_controller import EigenController          # noqa: E402
from cascaded_pd_controller import CascadedPDController  # noqa: E402
from lpv_mpc_controller import LPVMPCController       # noqa: E402
import vehicle_params                                 # noqa: E402


GRAVITY = 9.81

# Scale factors applied in Q3. Deliberately not 1.0 and far enough from it that
# a "perturbation" that silently did nothing cannot be mistaken for one that
# worked and produced a small change.
MASS_SCALE = 1.40
INERTIA_SCALE = 2.00

# Q4 wind probe: a constant body-frame force, and how long to let it act.
PROBE_FORCE_N = 2.0
PROBE_FORCE_STEPS = 150     # at physics_dt = 1/250 this is 0.6 s


def log(msg: str) -> None:
    carb.log_warn(f"PROBE {msg}")
    print(f"PROBE {msg}", flush=True)


def rule(char: str = "-") -> None:
    log(char * 72)


# ── USD / PhysX inspection ───────────────────────────────────────────────────

def physics_attrs(prim) -> dict:
    """Every physics: attribute on a prim, with its authored-opinion flag."""
    out = {}
    for attr in prim.GetAttributes():
        name = attr.GetName()
        if not name.startswith("physics:"):
            continue
        try:
            out[name] = (attr.Get(), bool(attr.HasAuthoredValueOpinion()))
        except Exception as exc:
            out[name] = (f"<unreadable: {exc}>", False)
    return out


def dump_usd_mass_properties(vehicle) -> dict:
    """Authored mass properties for every prim under the vehicle.

    Returns {prim_path: {attr: (value, authored)}} for prims that carry any.
    """
    stage = vehicle._current_stage
    root = stage.GetPrimAtPath(vehicle._stage_prefix)
    found = {}
    if not root or not root.IsValid():
        log(f"!! vehicle root prim {vehicle._stage_prefix} is not valid")
        return found
    for prim in [root] + list(root.GetAllChildren()):
        attrs = physics_attrs(prim)
        # Recurse one more level: the Iris links sit under the articulation root.
        for child in prim.GetAllChildren():
            child_attrs = physics_attrs(child)
            if child_attrs:
                found[str(child.GetPath())] = child_attrs
        if attrs:
            found[str(prim.GetPath())] = attrs
    return found


def physx_body_properties(vehicle, body_part: str = "/body") -> dict:
    """Mass and inertia PhysX is actually simulating, via dynamic_control.

    This is the number that matters. The USD is what was authored; this is what
    survived import, unit conversion and any automatic mass computation the
    physics parser applied when an attribute was absent.
    """
    out = {}
    try:
        dc = vehicle.get_dc_interface()
        rb = dc.get_rigid_body(vehicle._stage_prefix + body_part)
        if rb is None:
            return {"error": f"no rigid body at {body_part}"}
        props = dc.get_rigid_body_properties(rb)
        if props is None:
            return {"error": "get_rigid_body_properties returned None"}
        for field in ("mass", "moment", "cMassLocalPose", "maxDepenetrationVelocity"):
            value = getattr(props, field, None)
            if value is None:
                continue
            try:
                out[field] = tuple(float(v) for v in value)
            except TypeError:
                out[field] = float(value)
    except Exception as exc:
        out["error"] = str(exc)
    return out


def total_simulated_mass(vehicle) -> float:
    """Sum of PhysX link masses — the mass the controller's mg term must match.

    The controllers all model the vehicle as a single body, so the quantity
    their hover feedforward has to reproduce is the TOTAL, not the mass of the
    /body link alone. The four rotors are separate links.
    """
    total = 0.0
    parts = ["/body"] + [f"/rotor{i}" for i in range(4)]
    for part in parts:
        props = physx_body_properties(vehicle, part)
        mass = props.get("mass")
        if isinstance(mass, float):
            total += mass
            log(f"    {part:<10s} mass = {mass:.6f} kg")
        else:
            log(f"    {part:<10s} mass = <unavailable: {props.get('error')}>")
    return total


# ── Q3 helper: a backend that just pushes ────────────────────────────────────

class ForceProbeBackend(Backend):
    """Applies a constant body-frame force from inside ``update``.

    This is the mechanism the wind axis would use: ``Multirotor.update`` calls
    every backend's ``update`` after applying its own drag and damping, so a
    force applied here has to accumulate into the same physics step rather than
    being overwritten by it.
    """

    def __init__(self, force):
        super().__init__(config=None)
        self.force = np.asarray(force, dtype=float)
        self.enabled = False
        self.position = np.zeros(3)
        self.applied_steps = 0

    def update_state(self, state):
        self.position = np.asarray(state.position, dtype=float).copy()

    def update(self, dt: float):
        if self.enabled and self.vehicle is not None:
            self.vehicle.apply_force(list(self.force), body_part="/body")
            self.applied_steps += 1
        return np.zeros(4)

    def input_reference(self):
        return [0.0, 0.0, 0.0, 0.0]

    def update_sensor(self, sensor_type, data): pass
    def update_graphical_sensor(self, sensor_type, data): pass
    def start(self): pass
    def stop(self): pass
    def reset(self): pass


def settle(world, steps=30):
    for _ in range(steps):
        world.step(render=False)


# ── Probes ───────────────────────────────────────────────────────────────────

def probe_q1_q2(world, vehicle) -> dict:
    """What the plant weighs vs. what each controller believes it weighs."""
    rule("=")
    log("Q1/Q2 — simulated mass & inertia vs. each controller's model")
    rule("=")

    log("")
    log("USD authored mass properties (value, authored-opinion):")
    usd = dump_usd_mass_properties(vehicle)
    if not usd:
        log("  (none — no prim under the vehicle carries a physics: attribute)")
    for path in sorted(usd):
        log(f"  {path}")
        for name in sorted(usd[path]):
            value, authored = usd[path][name]
            flag = "AUTHORED" if authored else "fallback"
            log(f"      {name:<34s} {str(value):<28s} [{flag}]")

    log("")
    log("PhysX simulated properties for /body:")
    body = physx_body_properties(vehicle, "/body")
    for key in sorted(body):
        log(f"      {key:<34s} {body[key]}")

    log("")
    log("PhysX per-link masses:")
    total_mass = total_simulated_mass(vehicle)
    log(f"    {'TOTAL':<10s} mass = {total_mass:.6f} kg"
        f"   (hover thrust = {total_mass * GRAVITY:.4f} N,"
        f" {total_mass * GRAVITY / 4.0:.4f} N/rotor)")

    # The composite properties the fix installs: whole articulation, inertia
    # about the composite COM. Printed next to the /body values the controllers
    # were using so the size of the correction is visible rather than asserted.
    log("")
    log("Composite mass properties (vehicle_params.composite_mass_properties):")
    composite = vehicle_params.composite_mass_properties(vehicle)
    if composite is None:
        log("  !! unavailable — the physics interface could not be read")
    else:
        body_moment = body.get("moment")
        log(f"    mass  {composite['mass']:.6f} kg")
        log(f"    com   ({composite['com'][0]:+.6f}, {composite['com'][1]:+.6f},"
            f" {composite['com'][2]:+.6f}) m  relative to the body frame")
        if isinstance(body_moment, tuple) and len(body_moment) >= 3:
            for axis, key in enumerate(("Ixx", "Iyy", "Izz")):
                body_value = float(body_moment[axis])
                value = composite[key]
                delta = 100.0 * (value - body_value) / body_value if body_value else float("nan")
                log(f"    {key}   {value:.6f}  vs /body {body_value:.6f}"
                    f"   → {delta:+.1f} % of what the controllers were using")
        log(f"    {vehicle_params.describe(composite)}")

    # What each controller ends up believing, measured rather than asserted.
    # Each is constructed exactly as the sweeps construct it, then given the
    # same sync opportunity the sweeps give it via start().
    log("")
    log("Controller models — constructor default → after start()/sync:")
    rule()
    log(f"  {'controller':<16s} {'m [kg]':<20s} {'Ixx':<18s} {'Iyy':<18s} {'Izz':<18s}")

    results = {"total_mass": total_mass, "composite": composite,
               "controllers": {}}

    eigen = EigenController(results_file=None, target_position=(0.0, 0.0, 1.5))
    before = (eigen.m, eigen.Ixx, eigen.Iyy, eigen.Izz)
    # initialize() rather than assigning .vehicle: Backend exposes vehicle as a
    # read-only property over _vehicle, so the direct assignment would raise.
    eigen.initialize(vehicle)
    eigen._sync_physical_params_from_vehicle()
    after = (eigen.m, eigen.Ixx, eigen.Iyy, eigen.Izz)
    log(f"  {'eigen':<16s} "
        f"{before[0]:.4f} → {after[0]:<11.4f} "
        f"{before[1]:.5f} → {after[1]:<9.5f} "
        f"{before[2]:.5f} → {after[2]:<9.5f} "
        f"{before[3]:.5f} → {after[3]:<9.5f}")
    results["controllers"]["eigen"] = {"before": before, "after": after}

    pd = CascadedPDController(results_file=None, target_position=(0.0, 0.0, 1.5))
    pd_before = pd.m
    pd.initialize(vehicle)
    pd._sync_physical_params_from_vehicle()
    log(f"  {'cascaded_pd':<16s} "
        f"{pd_before:.4f} → {pd.m:<11.4f} "
        f"{'n/a':<18s} {'n/a':<18s} {'n/a':<18s}   (no inertia in its model)")
    results["controllers"]["pd"] = {"before": (pd_before,), "after": (pd.m,)}

    mpc = LPVMPCController(results_file=None, target_position=(0.0, 0.0, 1.5))
    mpc_before = (mpc.m, mpc.Ixx, mpc.Iyy, mpc.Izz)
    mpc_c_before = mpc.C
    mpc.initialize(vehicle)
    mpc._sync_physical_params_from_vehicle()
    mpc_after = (mpc.m, mpc.Ixx, mpc.Iyy, mpc.Izz)
    log(f"  {'lpv_mpc':<16s} "
        f"{mpc_before[0]:.4f} → {mpc_after[0]:<11.4f} "
        f"{mpc_before[1]:.5f} → {mpc_after[1]:<9.5f} "
        f"{mpc_before[2]:.5f} → {mpc_after[2]:<9.5f} "
        f"{mpc_before[3]:.5f} → {mpc_after[3]:<9.5f}")
    log(f"  {'':<16s} C {mpc_c_before:.6f} → {mpc.C:.6f}"
        f"   (cached cross-coupling coefficient — must track the inertias)")
    results["controllers"]["mpc"] = {"before": mpc_before, "after": mpc_after}

    rule()
    log("")
    log("Model error vs. the composite truth — the mismatch the nominal")
    log("(unperturbed) runs already carry, before any perturbation is applied:")
    truth = composite or {}
    truth_mass = truth.get("mass", total_mass)
    for name, entry in results["controllers"].items():
        after = entry["after"]
        believed = after[0]
        err = 100.0 * (believed - truth_mass) / truth_mass if truth_mass > 0 else float("nan")
        line = (f"    {name:<14s} m {believed:.4f} kg vs {truth_mass:.4f} kg"
                f" → {err:+6.1f} %")
        if len(after) == 4 and composite:
            for value, key in zip(after[1:], ("Ixx", "Iyy", "Izz")):
                true_value = composite[key]
                pct = 100.0 * (value - true_value) / true_value if true_value else float("nan")
                line += f"   {key} {pct:+.1f} %"
        log(line)

    return results


def probe_q3_inertia(world, vehicle, baseline: dict) -> None:
    """Does writing physics:mass / physics:diagonalInertia actually take?"""
    rule("=")
    log("Q3a — can the plant's mass/inertia be perturbed?")
    rule("=")

    stage = vehicle._current_stage
    body_path = vehicle._stage_prefix + "/body"
    prim = stage.GetPrimAtPath(body_path)
    if not prim or not prim.IsValid():
        log(f"!! no body prim at {body_path} — cannot test")
        return

    before = physx_body_properties(vehicle, "/body")
    log(f"  PhysX before write : mass={before.get('mass')} moment={before.get('moment')}")

    # Write through UsdPhysics.MassAPI so the attribute is created when it does
    # not already exist. A plain GetAttribute().Set() is a no-op on an absent
    # attribute, which would look exactly like "the write was ignored".
    try:
        from pxr import UsdPhysics
        mass_api = UsdPhysics.MassAPI.Apply(prim)
        base_mass = before.get("mass")
        if not isinstance(base_mass, float) or base_mass <= 0.0:
            base_mass = 1.0
        base_moment = before.get("moment")
        if not (isinstance(base_moment, tuple) and len(base_moment) >= 3):
            base_moment = (0.01, 0.01, 0.02)

        new_mass = base_mass * MASS_SCALE
        new_moment = tuple(float(v) * INERTIA_SCALE for v in base_moment[:3])
        mass_api.CreateMassAttr().Set(new_mass)
        mass_api.CreateDiagonalInertiaAttr().Set(new_moment)
        log(f"  wrote via MassAPI  : mass={new_mass:.6f} "
            f"diagonalInertia={tuple(f'{v:.6g}' for v in new_moment)}")
    except Exception as exc:
        log(f"!! MassAPI write failed: {exc}")
        return

    after_write = physx_body_properties(vehicle, "/body")
    log(f"  PhysX after write  : mass={after_write.get('mass')} "
        f"moment={after_write.get('moment')}   (no reset yet)")

    world.reset()
    settle(world, 10)
    after_reset = physx_body_properties(vehicle, "/body")
    log(f"  PhysX after reset  : mass={after_reset.get('mass')} "
        f"moment={after_reset.get('moment')}")

    def took(before_val, after_val, scale) -> str:
        if not isinstance(before_val, float) or not isinstance(after_val, float):
            return "UNKNOWN (property unreadable)"
        if abs(after_val - before_val * scale) <= 1e-4 * max(1.0, abs(after_val)):
            return "YES — PhysX picked up the new value"
        if abs(after_val - before_val) <= 1e-9:
            return "NO — PhysX still reports the original value"
        return f"PARTIAL — changed to {after_val:.6f}, expected {before_val * scale:.6f}"

    rule()
    log(f"  mass perturbation    : {took(before.get('mass'), after_reset.get('mass'), MASS_SCALE)}")
    b_mom, a_mom = before.get("moment"), after_reset.get("moment")
    if isinstance(b_mom, tuple) and isinstance(a_mom, tuple):
        log(f"  inertia perturbation : {took(b_mom[0], a_mom[0], INERTIA_SCALE)}")
    else:
        log("  inertia perturbation : UNKNOWN (moment unreadable)")
    rule()

    # Restore, so the force probe that follows runs on the nominal vehicle.
    try:
        from pxr import UsdPhysics
        mass_api = UsdPhysics.MassAPI.Apply(prim)
        if isinstance(before.get("mass"), float):
            mass_api.CreateMassAttr().Set(float(before["mass"]))
        if isinstance(b_mom, tuple):
            mass_api.CreateDiagonalInertiaAttr().Set(tuple(float(v) for v in b_mom[:3]))
        world.reset()
        settle(world, 10)
        restored = physx_body_properties(vehicle, "/body")
        log(f"  restored           : mass={restored.get('mass')} "
            f"moment={restored.get('moment')}")
    except Exception as exc:
        log(f"!! restore failed: {exc}")


def probe_q3_force(world, force_backend, force_vehicle, control_vehicle) -> None:
    """Does a force applied from Backend.update reach the rigid body?"""
    rule("=")
    log("Q3b — does an external force applied from Backend.update move the vehicle?")
    rule("=")

    timeline = omni.timeline.get_timeline_interface()

    world.reset()
    settle(world, 10)
    timeline.play()

    x0_force = float(force_vehicle._state.position[0])
    x0_ctrl = float(control_vehicle._state.position[0])
    log(f"  start x: forced drone {x0_force:+.5f} m, control drone {x0_ctrl:+.5f} m")

    force_backend.enabled = True
    for _ in range(PROBE_FORCE_STEPS):
        if not simulation_app.is_running():
            break
        world.step(render=False)
    force_backend.enabled = False
    timeline.stop()

    dx_force = float(force_vehicle._state.position[0]) - x0_force
    dx_ctrl = float(control_vehicle._state.position[0]) - x0_ctrl

    log(f"  applied {PROBE_FORCE_N} N in +X body for {force_backend.applied_steps} steps")
    log(f"  Δx forced drone  : {dx_force:+.5f} m")
    log(f"  Δx control drone : {dx_ctrl:+.5f} m  (no force — this is the null)")

    rule()
    if force_backend.applied_steps == 0:
        log("  VERDICT: Backend.update never ran — the probe proved nothing")
    elif abs(dx_force - dx_ctrl) > 10.0 * max(abs(dx_ctrl), 1e-3):
        log("  VERDICT: YES — force from Backend.update reaches the body. The")
        log("           wind axis can be implemented without touching the extension.")
    else:
        log("  VERDICT: NO — the force did not move the vehicle. Wind would have")
        log("           to go into the Multirotor update path (a Drag subclass).")
    rule()


def main():
    pg = PegasusInterface()
    pg._world = World(**pg._world_settings)
    world = pg.world
    pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])
    settle(world, 20)

    # Drone 0: the subject of Q1/Q2/Q3a, and the forced drone in Q3b.
    force_backend = ForceProbeBackend([PROBE_FORCE_N, 0.0, 0.0])
    cfg = MultirotorConfig()
    cfg.backends = [force_backend]
    subject = Multirotor("/World/probe_subject", ROBOTS["Iris"], 0, [0.0, 0.0, 3.0],
                         Rotation.from_euler("XYZ", [0, 0, 0], degrees=True).as_quat(),
                         config=cfg)

    # Drone 1: identical, no force. Without it, drift from drag or from a
    # non-level spawn would read as a successful force application.
    cfg_ctrl = MultirotorConfig()
    cfg_ctrl.backends = []
    control = Multirotor("/World/probe_control", ROBOTS["Iris"], 1, [6.0, 0.0, 3.0],
                         Rotation.from_euler("XYZ", [0, 0, 0], degrees=True).as_quat(),
                         config=cfg_ctrl)

    world.reset()
    settle(world, 10)

    baseline = probe_q1_q2(world, subject)
    probe_q3_inertia(world, subject, baseline)
    probe_q3_force(world, force_backend, subject, control)

    log("")
    log("probe complete")

    try:
        omni.timeline.get_timeline_interface().stop()
    except Exception:
        pass
    for fn in ("stop", "clear_all_callbacks", "clear"):
        try:
            getattr(world, fn)()
        except Exception:
            pass
    simulation_app.close()


if __name__ == "__main__":
    main()
