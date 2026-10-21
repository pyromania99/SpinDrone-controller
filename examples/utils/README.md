# `examples/utils` — controllers, configuration and CLI plumbing

Shared modules the sweep scripts import. Two groups: the controllers under test, and the
rotating-wing configuration.

| File | Contents | Origin |
|---|---|---|
| `eigen_controller.py` | Eigenvalue-structure attitude controller | existing |
| `cascaded_pd_controller.py` | Cascaded PD baseline | existing |
| `lpv_mpc_controller.py` | LPV-MPC controller (needs `osqp`) | existing |
| `nonlinear_controller.py` | Nonlinear analysis backend | existing |
| **`wing_params.py`** | **The single file to edit to change any dynamics parameter** | added |
| `wing_cli.py` | `--wing` / `--no-wing` plumbing shared by every sweep script | added |

---

## `wing_params.py` — the configuration file

Everything about the vehicle's physics is set here. Every value carries a docstring saying
what it does and where the number came from, so the file doubles as the parameter
reference.

| Section | Covers |
|---|---|
| Master switch | `WING_ENABLED` |
| Atmosphere | `AIR_DENSITY_KG_M3`, `KINEMATIC_VISCOSITY_M2_S` |
| **Airframe dynamics** | `ENABLE_ROTOR_GYROSCOPICS`, `ROTOR_INERTIA_KG_M2`, `HOVER_ROTOR_SPEED_RAD_S` |
| Blade planform | `NUM_BLADES`, `ROTATION_SENSE`, `ROOT_RADIUS_M`, `TIP_RADIUS_M`, `BLADE_AZIMUTHS_DEG`, `BLADE_Z_OFFSET_M` |
| Build schedule | `STATION_RADIUS_M`, `STATION_CHORD_M`, `STATION_PITCH_DEG` |
| Trim | `COLLECTIVE_PITCH_OFFSET_DEG` |
| Structure | `BLADE_AREAL_DENSITY_KG_M2`, `BLADE_MASS_KG`, `APPLY_INERTIA_TO_BODY` |
| Aerofoil | `USE_GOE210_POLAR`, `POLAR_OVERRIDE` |
| Induced velocity | `INFLOW_MODEL`, iteration cap, tolerance, relaxation, tip/root loss |
| Numerics | `NUM_STATIONS`, `MAX_RELATIVE_SPEED_M_S`, `MOMENT_REFERENCE_OFFSET_M` |

Defaults are the as-designed GOE 210 wing from `foil_out/GOE210_DESIGN_REPORT.md`.

**Two parameters are worth reading before changing anything else:**

- **`ROTATION_SENSE = -1`** must match the direction the airframe actually spins. It does
  not scale anything; it is a handedness. Get it wrong and the panels push *down* — which
  is exactly what happened before the parameter existed, costing 2.2 N.
- **`STATION_PITCH_DEG`** is not decoration. Each station's twist places that section near
  its best lift-to-drag angle for the *designed* radii. Moving `ROOT_RADIUS_M` or
  `TIP_RADIUS_M` without re-deriving the twist does not give a smaller working blade; it
  gives the old twist at the wrong radii. Use `design_pitch_schedule()` to retarget.

### Printing the active configuration

```bash
python3 examples/utils/wing_params.py
```

```
Rotating wing: ENABLED
  blades            : 2 at [0.0, 180.0] deg
  span              : r = 0.200 to 0.500 m
  blade area        : 0.1383 m^2  (solidity 0.176)
  collective offset : +0.00 deg
  inflow model      : bemt
  handedness        : -1 (left-handed panels)
  hover @ -20 rad/s: T = 3.701 N (377 g), P = 7.00 W, Q_z = +0.3498 N*m, FM = 0.741
```

Sweep scripts print the same block at the start of a run, so a result record always states
the geometry that produced it.

### Importing it

`wing_params` loads the aerodynamics package **by path**, bypassing
`pegasus.simulator.logic.__init__` which needs Isaac Sim. That is why the offline physics
tests can import this module under plain Python, and why it works unchanged inside Isaac.

---

## `wing_cli.py` — adding the wing to a script

Two lines:

```python
from utils.wing_cli import add_wing_arguments, spawn_vehicle

add_wing_arguments(parser)                                        # in parse_args()
vehicle = spawn_vehicle(prefix, usd, i, pos, quat, cfg, ARGS)     # instead of Multirotor(...)
```

With the wing disabled, `spawn_vehicle` returns a stock `Multirotor` constructed exactly as
before, so `--no-wing` results stay bit-identical to pre-wing recordings. With it enabled
it returns a `SpinningWingMultirotor`, which inherits that behaviour unchanged and adds the
panels.

`wing_cli` also applies the airframe-dynamics settings from `wing_params.py`, so that one
file governs every dynamics parameter rather than some of them living as class defaults.

### Flags

| Flag | Effect |
|---|---|
| `--wing` / `--no-wing` | Enable the panels. **`--no-wing` is the default**, so any command line written before this work keeps its previous meaning |
| `--wing-inflow {none,bemt,glauert,dynamic}` | Induced-velocity model |
| `--wing-collective-deg` | Rigid pitch offset on every station — the primary trim knob |
| `--wing-blades` | Panel count |
| `--wing-root-radius`, `--wing-tip-radius` | Span (re-derive the twist if you move these) |
| `--wing-stations` | Spanwise integration stations |
| `--wing-rotation-sense {1,-1}` | Handedness; `1` is the wrong-way ablation |
| `--wing-no-inertia` | Panels stay aerodynamically active but structurally massless. Ablation only |
| `--no-rotor-gyro` | Disable the propellers' gyroscopic reaction. A **quadrotor** term, not a wing term |

`wing_run_tag()` turns the active configuration into a filesystem-safe tag
(`nowing`, `wing`, `wing_col+2.0_b4`, `wing_noinertia`) so winged and wingless sweeps do not
overwrite each other's results.

### Example

```bash
isaac_run examples/controller_tests/sweeps/eigen_stability_sweep.py \
    --motor-moments 6e-7 --omega-n-values 2 4 6 8 10 12 --duration 20 \
    --headless --wing --wing-inflow dynamic
```

---

## See also

| Document | Contents |
|---|---|
| `IMPLEMENTATION.md` | What was built, why, and every measured number |
| `.../logic/aerodynamics/README.md` | The model itself |
| `.../logic/dynamics/README.md` | Propeller gyroscopics |
| `examples/controller_tests/physics_tests/` | 184 offline tests |
