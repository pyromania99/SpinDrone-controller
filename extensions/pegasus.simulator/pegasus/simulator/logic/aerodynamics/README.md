# `aerodynamics` — rotating-wing model

Aerofoil panels bonded to the drone's arms. Because all four motors turn the same way,
their reaction torques add rather than cancel and the airframe spins; the panels are
therefore rotor blades driven by that spin, converting part of an otherwise parasitic yaw
into lift.

**This package imports nothing but NumPy.** That is a deliberate, load-bearing property:
it is why 184 physics tests run in under four seconds with no simulator, and why the model
can be probed, swept and debugged offline. If anything here ever imports `pxr`, `omni`,
`isaacsim` or `carb`, the offline test suite stops existing.

---

## Files

| File | Contents |
|---|---|
| `wing_config.py` | `RotatingWingConfig` — the parameter schema, with every invariant validated in `validate()`; `InflowModel` enumeration |
| `airfoil_polar.py` | `AirfoilPolar` interface, `AnalyticPolar`, `TablePolar`; Viterna–Corrigan extension to ±180°; `goe210_polar()` |
| `rotating_wing.py` | `RotatingWing` — the blade element solver, four inflow models, load integration, mass and inertia |
| `wing_design.py` | `design_pitch_schedule()` — re-derives the blade twist when the geometry changes |

Nothing here is edited to configure a run. **`examples/utils/wing_params.py` is the file to
edit**; this package only defines what the parameters mean.

---

## The model

Blade element theory closed with momentum theory. The span is divided into elements; each
element's velocity through the air is the **exact** rigid-body relation

```
v_element = v_body + ω × r_element
```

so all six vehicle rates enter without approximation — there is no small-angle or
axial-flow assumption in the kinematics. Section lift and drag come from a 2-D polar at the
local Reynolds number, and the induced velocity from an annulus-by-annulus momentum balance
with a Prandtl tip-loss factor.

```
U_T = (v_body + ω×r)·ê_t          chordwise
U_P = (v_body + ω×r)·ẑ + v_i      axial, including induced velocity
φ   = atan2(U_P, U_T)             inflow angle
α   = θ(r) − φ                    angle of attack

dF_z = dL cos φ − dD sin φ
dF_t = −(dL sin φ + dD cos φ)     moments from p × dF
```

Attitude and position do not enter at all, which is correct: the panels are body-fixed and
the air is still in the world frame, so only relative velocity matters.

### Inflow models

| `InflowModel` | Behaviour | Cost |
|---|---|---|
| `BEMT` (default) | Per-annulus momentum balance with Prandtl tip loss, solved to convergence. Resolves the radial distribution; reproduces the design report. | 1–32 passes |
| `DYNAMIC` | Inflow as a **state**: `dν/dt = (ν_steady − ν)/τ`, τ ≈ 77 ms from the air column's apparent mass. Adds the real lag *and* bounds the cost. | **always 1 pass** |
| `GLAUERT` | Uniform inflow with an edgewise term. Cheaper, better behaved in fast translation, no radial detail. | iterated |
| `NONE` | No induced velocity. Debugging and ablation only; over-predicts thrust. | 0 |

`DYNAMIC` needs `dt` passed to `compute_loads()`; the algebraic models ignore it.

### Handedness

A rotor blade is a handed object. Mirror it and it still lifts, but only turned the other
way; spin it the wrong way and it pushes **down**. `rotation_sense` defaults to `-1`
because this airframe spins negative (`rot_dir = [-1,-1,-1,-1]`). Mirroring is an exact
symmetry of the blade element equations, so the design report's numbers are reproduced at
−20 rad/s by a left-handed blade exactly as at +20 rad/s by a right-handed one.

This was a real defect, not a hypothetical: the panels were first built right-handed and
produced **−2.2 N of downforce** on top of 133 g of added mass.

---

## Use

```python
from examples.utils.wing_params import build_wing, build_wing_config
import numpy as np

wing = build_wing()                      # as-designed GOE 210 panels
loads = wing.hover_loads(-18.0)          # steady hover at the measured settled spin
print(loads.thrust_n, loads.torque_z_nm)

loads = wing.compute_loads(              # arbitrary flight state
    linear_body_velocity_m_s=np.array([2.0, 0.0, -0.5]),
    angular_velocity_body_rad_s=np.array([0.0, 1.5, -18.0]),
    dt=1/250.0,                          # required only by InflowModel.DYNAMIC
)
```

`WingLoads` carries the resultant `force_body_n` and `moment_body_nm` plus per-station
diagnostics: `alpha_rad`, `cl`, `cd`, `reynolds`, `induced_velocity_m_s`, and the inflow
solver's `residual`/`iterations`.

`RotatingWing.inertia_about_body_origin()` returns the panels' mass and inertia, which
`SpinningWingMultirotor` adds to the `/body` prim at spawn — the wing has no USD geometry,
so without this it would be aerodynamically present and structurally massless.

---

## Validation

Against `foil_out/GOE210_DESIGN_REPORT.md`, an independent BEMT optimisation using
NeuralFoil: thrust −1.0 %, power −4.2 %, figure of merit +2.6 %.

Against XFOIL polars in `lit_survey/`: Cl within 4–5 %, Cd within 1–5 %, and the best
lift-to-drag angle within **0.13°** across the operating Reynolds range.

Only the section polar was fitted. Thrust, power, figure of merit and the radial inflow
distribution are integrated outputs that were tuned to nothing.

Tests: `examples/controller_tests/physics_tests/` (184, no Isaac Sim).
Accuracy and real-time budget: `physics_tests/FIDELITY.md`.

---

## Known limitations

Deliberate, and documented in the module docstrings:

- **Quasi-steady inflow** under `BEMT`/`GLAUERT`. `DYNAMIC` adds the lag; its
  first-harmonic states are still absent.
- **Edgewise validity ends at μ ≈ 0.40**, reached at only 3.6 m/s sideways because the
  wing's root sits far out at 0.2 m. Above it, the per-annulus balance cannot represent the
  fore-aft inflow gradient.
- **No blade flexibility.** The design report measured 10–15° of aeroelastic twist against a
  4–5° design angle of attack; −10° costs 54 % of thrust. Represent a known mean twist with
  `COLLECTIVE_PITCH_OFFSET_DEG`.
- **No rotational augmentation** (Himmelskamp). The blade's aspect ratio is 1.3, which is
  where that effect is largest.
- **Post-stall is extrapolation.** The Viterna branch is structurally verified against the
  source paper and bounded by tests, but not validated against data.
