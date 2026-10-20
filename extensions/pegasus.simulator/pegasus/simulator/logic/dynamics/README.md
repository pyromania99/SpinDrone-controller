# `dynamics` — body forces and moments that are not rotor thrust

| File | Contents | Origin |
|---|---|---|
| `drag.py` | `Drag` base interface | upstream Pegasus |
| `linear_drag.py` | `LinearDrag` — `diag(dx,dy,dz)·Rᵀv` on the body | upstream Pegasus |
| `rotor_gyroscopics.py` | Propeller angular momentum and its gyroscopic reaction | **added for the spinning airframe** |

---

## `rotor_gyroscopics.py`

### Why it exists here and nowhere else in Pegasus

A conventional quadrotor sets `rot_dir = [-1, -1, +1, +1]`. The four propeller angular
momenta then cancel to essentially zero, and neglecting them is harmless — which is why
upstream Pegasus neglects them.

This airframe spins *because* every motor turns the same way, `rot_dir = [-1,-1,-1,-1]`, so
the four momenta **add**:

| | |
|---|---|
| rotor inertia about its spin axis | 7.65e-5 kg·m² (read from PhysX at runtime) |
| hover rotor speed | 684 rad/s, from `T = k ω²` |
| **net propeller momentum** | **0.209 kg·m²/s** |
| body momentum at 18 rad/s | 1.273 kg·m²/s |

That is **16 % of the whole vehicle's angular momentum**, and the moment it produces
exceeds the rotating wing's entire aerodynamic moment above roughly 1.5 rad/s of body rate:

| body rate | propeller gyro | wing moment | ratio |
|---|---|---|---|
| 2 rad/s | 0.419 N·m | 0.311 N·m | 1.3× |
| 4 rad/s | 0.837 N·m | 0.364 N·m | **2.3×** |

A term that large being absent would dominate any conclusion drawn about attitude dynamics.

### Why it has to be applied explicitly

The USD's rotor joints are driven kinematically at a **hardcoded 100 rad/s** by
`Multirotor.handle_propeller_visual()` — the comment says "for animation purposes only" —
against a real hover speed of 684 rad/s. The articulation therefore carries at most 15 % of
the true momentum, and none of it reliably. The moment is applied as an explicit body
torque instead.

### The model

```
h      = Σ I_rotor · ω_i · (−rot_dir_i)      about body +z
M_gyro = −ω_body × h
```

The sign convention matters and is easy to get backwards. Pegasus applies
`Σ c_i ω_i² rot_dir_i` to the body as a yaw torque, so a rotor whose *contribution* is
negative is itself spinning in the **positive** sense — the motor drives the propeller one
way and the airframe feels the opposite. The propeller momentum therefore carries the
opposite sign to `rot_dir`.

Two consequences worth knowing:

- **The moment has no yaw component.** `−ω × [0,0,h]` is identically zero in z, so the
  airframe's own large spin rate produces no spurious moment.
- **It does no work.** The moment is perpendicular to the rate, so it cannot add or remove
  rotational energy. A violation would show up as a slow unexplained drift in spin rate.

Roll rate produces a **pitch** moment and vice versa — the cross-coupling is the signature
of the effect, and the reason it matters for a controller that treats those axes as
independent.

### Configuration and ablation

`examples/utils/wing_params.py` holds `ENABLE_ROTOR_GYROSCOPICS` (default `True`) and
`ROTOR_INERTIA_KG_M2`. Per run:

```bash
isaac_run ... --no-rotor-gyro     # reproduces results recorded before the term existed
```

Verified: the ablated baseline matches the pre-change score and effort to all digits.

### Note on scope

This touches **stock quadrotor physics**, which the rotating-wing work otherwise leaves
alone. It is here because the fidelity question cannot be answered honestly without it, and
it is behind a flag so prior results stay reproducible.

### Tests

`examples/controller_tests/physics_tests/test_rotor_gyroscopics.py` — 15 tests, checked
against closed forms rather than recorded numbers: the cross-product form, the sign
convention, exact cancellation for a conventional quadrotor, zero yaw component, zero work
done, and the magnitude against the wing's own moment.

The module imports nothing but NumPy so those tests run without Isaac Sim.
