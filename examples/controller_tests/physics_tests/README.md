# Rotating-wing physics tests

Offline verification of the spinning-wing aerodynamic model and of the airframe dynamics
terms it depends on. **No Isaac Sim required** — the code under test is pure NumPy, so the
whole suite runs in about ten seconds on system Python. (Most of that is the dynamic-inflow
tests, which integrate real transients over several hundred timesteps; the rest of the
suite is still sub-second.)

```bash
# from the repository root
python3 -m pytest examples/controller_tests/physics_tests -q

# or, where pytest is not installed (e.g. Isaac Sim's bundled interpreter)
python3 examples/controller_tests/physics_tests/run_physics_tests.py
```

Run it after touching anything in
`extensions/pegasus.simulator/pegasus/simulator/logic/aerodynamics/` or
`examples/utils/wing_params.py`.

---

## What the model is

Fixed aerofoil panels are bonded to the drone's arms. Because all four motors turn the
same way, their reaction torques add rather than cancel and the airframe spins; the panels
are therefore rotor blades driven by that spin, converting part of an otherwise parasitic
yaw into lift.

The model is classical blade element theory closed with momentum theory (BEMT): the span
is divided into elements, each element's velocity through the air comes from the vehicle's
body-frame linear velocity and body rates, section lift and drag come from a 2-D polar at
the local Reynolds number, and the induced velocity is found from an annulus-by-annulus
momentum balance with a Prandtl tip-loss factor.

| File | Contents |
|---|---|
| `aerodynamics/airfoil_polar.py` | 2-D section polars, full ±180° via Viterna–Corrigan |
| `aerodynamics/rotating_wing.py` | Blade element solver, inflow models, load integration |
| `aerodynamics/wing_config.py` | Parameter schema and the as-designed defaults |
| `aerodynamics/wing_design.py` | Re-derives the pitch schedule when geometry changes |
| `vehicles/multirotors/spinning_wing_multirotor.py` | Simulator integration |
| `examples/utils/wing_params.py` | **The single file to edit** to change the physics |
| `examples/utils/wing_cli.py` | `--wing` / `--no-wing` command-line plumbing |

---

## Validation against the design report

`test_wing_validation.py` checks the model against `foil_out/GOE210_DESIGN_REPORT.md`, an
**independent** BEMT optimisation using different aerodynamic data (NeuralFoil polars,
cross-checked in XFOIL 6.99).

| Quantity | Report | Model | Error |
|---|---|---|---|
| Thrust @ 20 rad/s | 3.740 N | 3.701 N | −1.0 % |
| Thrust @ 15 rad/s | 211 g | 212 g | +0.6 % |
| Aero power @ 20 rad/s | 7.31 W | 7.01 W | −4.1 % |
| Figure of merit | 0.722 | 0.740 | +2.5 % |
| Station angle of attack | — | — | within **0.07°** |
| Station lift coefficient | — | — | within **0.01** |
| Station induced velocity | — | — | within **0.02 m/s** |

**What was and was not fitted.** The airfoil polar *was* calibrated to the report's
per-station section data (α, c_l, c_l/c_d, Re). The integrated quantities above — thrust,
power, figure of merit, and the radial induced-velocity distribution — were **not** fitted
to anything. They fall out of the blade element integration and the momentum balance, so
agreement there tests the solver rather than the curve fit.

---

## What the suite covers

**`test_airfoil_polar.py`** (31 tests) — the innermost layer.
Lift-curve slope, drag bucket, Reynolds scaling and clamping, Viterna matching at the
stall point, finiteness and boundedness over the full ±180° range, continuity across the
stall blend, tabulated-polar interpolation and clamping, statelessness.

**`test_wing_validation.py`** (19 tests) — agreement with the design report.
Integrated performance, station-by-station distributions, grid convergence, inflow
convergence, warm-start independence, tip-loss sensitivity, and a momentum-theory
cross-check.

**`test_wing_physics.py`** (39 tests) — invariants and probes.
The tests that catch sign, frame and factor errors:

- **Energy.** `P = −Q_z · Ω` to machine precision — power and torque are computed by
  separate routes, so their agreeing is a real check. The wing must always *absorb* power
  and always *brake* the spin, for either rotation direction.
- **Scaling.** `T ∝ Ω²` with and without induced inflow, `P ∝ Ω³`, `T ∝ ρ`.
- **Signs.** Positive spin lifts; reversed spin pushes down.
- **Symmetry.** Two opposed blades cancel in-plane forces and roll/pitch moments exactly;
  one blade does not. Blade count raises thrust *sub*-linearly, because solidity raises
  the induced velocity.
- **Null directions.** With blades at 0°/180° their motion is along ±y, so an
  x-translation and a roll rate are geometrically null inputs while a y-translation and a
  pitch rate are not. Both the sensitive *and* the null cases are asserted — a model that
  responded to a null direction would be forming the cross product wrongly.
- **Flight regimes.** Climb reduces thrust, descent increases it, edgewise flight raises
  thrust and produces the advancing/retreating moment, and the reversed-flow region is
  exercised and stays finite.
- **Robustness.** Non-finite vehicle state returns zero load instead of propagating NaN
  into PhysX; extreme rates stay bounded; repeated evaluation is deterministic; vortex
  ring state stays bounded.

**`test_wing_config.py`** (29 tests) — the parameter schema.
Every invariant refused from both directions, derived quantities, and a check that
`wing_params.py` has not drifted from the schema defaults.

**`test_wing_structure.py`** (15 tests) — mass and inertia.
The inertia integral is checked against closed-form results rather than recorded numbers:
the uniform-rod formula for I_zz, the parallel-axis theorem for a raised wing plane,
additivity over blades, and the point-mass-at-the-tip upper bound. Also that blades lying
along x contribute almost nothing to I_xx, and that a four-blade cross is inertially
symmetric in roll and pitch while two opposed blades are not.

**`test_wing_design.py`** (16 tests) — the pitch designer.
Headline test: running the designer on the geometry the report optimised must **recover
the report's own twist** (0.55° mean error over the loaded span). Also that the designed
blade lands in the reported 4.4–5.5° α band, that rescaling a span without re-pitching
measurably costs thrust, and that the stall-margin guard keeps the design well conditioned
at low Reynolds number.

---

## Measured operating point, and the drag budget

Everything above is stated at the *design* point, 20 rad/s. The vehicle does not fly
there by choice — it settles wherever the torque balance puts it. Measured from the eigen
sweep's recorded `yaw_rate` telemetry, 20 s run, `--motor-moment 6e-7`:

| | settled spin | wing lift | % of weight |
|---|---|---|---|
| no wing | −26.96 rad/s | — | — |
| wing | **−18.00 rad/s** | **3.00 N** | **18.7 %** |

The wing slows the spin by a third, and the mechanism is worth stating because it is not
the obvious one. Only part of it is the wing's own drag torque. The larger part is that
wing lift *unloads the rotors*: less rotor thrust is needed, so rotor ω² falls, so the
reaction torque that drives the spin in the first place falls with it. Lift and spin rate
are coupled through the motors, not just through the wing.

At −18 rad/s the blade sits at **α = 3.3–5.5°** across the span, clear of the 9° stall,
at **Re 52k–120k** — inside the band the polar was calibrated against. The wing is
operating where its aerodynamic data is valid, which is not something to assume.

**Drag budget at that operating point.** Three things resist the spin:

| Source | Yaw torque | Model |
|---|---|---|
| Body rotational damping | 0.630 N·m | `rotational_damping[2] = 0.035`, linear in ω |
| Wing profile + induced drag | 0.284 N·m | blade element, from the section polar |

The stock airframe's yaw damping is **2.2× larger than the wing's**, and it is a constant
chosen for numerical well-behavedness rather than derived from any geometry — linear in ω
where real aerodynamic damping goes as ω². It therefore dominates the spin equilibrium,
and so dominates the lift the wing can produce. This is stock quadrotor physics and is
left untouched deliberately, but any conclusion about spin rate or wing lift inherits that
constant, and is worth re-checking against a measured airframe before it is trusted.

The wing's own drag is cross-checked two ways: `P = −Q_z · Ω` to machine precision (power
and torque are computed by separate routes), and the wing must absorb power and brake the
spin for either rotation direction and either handedness.

---

## Measured airframe geometry

From `assets/Robots/Iris/iris.usd`, since several decisions depend on it:

| Property | Value |
|---|---|
| Rotor radius | 0.245–0.252 m (mean 0.249) |
| Rotor height | z = +0.023 m |
| Body mass | 1.5 kg |
| Body inertia | (0.0291, 0.0291, 0.0552) kg·m² |

**The wing is designed for a larger airframe than the Iris asset.** The GOE 210 blade
spans r = 0.2→0.5 m, so on the Iris it extends past the motors at 0.249 m. Fitting it
strictly inboard instead (0.06→0.24 m) gives only **0.31 N — 2% of vehicle weight** — at
Re 17k–35k, below where the foil data is valid, with stations past stall. The design
report's own vehicle is ~500 g with arms reaching 0.5 m; the Iris is a 1.5 kg stand-in, so
wing lift reads as 25% of weight in simulation where it is ~75% on the intended airframe.

Use `design_pitch_schedule()` if you retarget the span — moving the radii without
re-deriving the twist leaves every section at the wrong incidence.

**`test_rotor_gyroscopics.py`** (15 tests) — the propellers' angular momentum.
That four co-rotating propellers do not cancel while a conventional quadrotor's do, that
the momentum carries the opposite sign to `rot_dir`, that roll rate feeds a pitch moment
and vice versa, that yaw rate alone produces nothing, that the moment never does work on
the body, and that its magnitude exceeds the wing's own at realistic body rates — which is
the finding that justified adding the term.

**`test_dynamic_inflow.py`** (13 tests) — inflow as a state rather than a constraint.
That it settles on the algebraic uniform-inflow solution, that the time constant matches
the apparent-mass derivation `2R/(3πν)` rather than a recorded number, that the step
response reaches 63 % in one time constant, that a coarse timestep cannot destabilise it,
and that it performs **exactly one** inflow pass in every flight state.

**`test_wing_handedness.py`** (9 tests) — which way the panels are built.
That mirroring is an exact symmetry (so handedness costs no performance), that the
configured handedness produces lift at the spin rate the airframe *measurably* reaches,
that the wrong handedness produces downforce, that the wing brakes the spin either way,
and that the configured handedness still agrees with the motor directions in
`QuadraticThrustCurve`. That last one is a one-second check on the fact everything else
depends on.

---

## Four bugs this suite caught during development

Worth recording, because all four were silent:

1. **Viterna singularity at ±180°.** The post-stall lift expression contains a
   `cos²α / sin α` term that diverges as α → 0. In the folded coordinate that happens at
   *both* ends of the range, and the second one — a section at near-zero incidence in
   fully reversed flow — is reachable by a real spinning wing. `c_l` reached **57.7**.
   Fixed by evaluating Viterna only outboard of the match angle and ramping lift linearly
   to zero inboard of it.

2. **Prandtl root loss.** Applying a root-loss factor alongside tip loss put predicted
   thrust ~10 % below the report and made the inflow iteration oscillate without
   converging. The report applies tip loss only, and physically the blade root is bonded
   to the airframe arm rather than being a free end, so root loss is the wrong model here.
   It is now off by default.

3. **The panels were built the wrong way round.** The blades were handed for a positive
   spin, matching the design report's conventional rotor, but this airframe spins
   *negative* -- all four motors turn the same way, so their reaction torques add. The
   vehicle settled at -27 rad/s in simulation and the wing produced **-2.2 N**: two
   newtons of downforce, on top of 133 g of added mass. A wing that was purely a cost.

   Nothing caught it because every test evaluated the wing at a *positive* spin rate, so
   the suite validated a mirror image of the blade that actually flew. The model was
   correct throughout; the configuration described an unflyable installation. Fixed by
   adding `ROTATION_SENSE`, defaulting to -1, and by deriving the suite's design spin rate
   from the configured handedness instead of hard-coding +20 rad/s.

   Measured effect, 20 s run, same controller gains:

   | | stability score | control effort |
   |---|---|---|
   | no wing | 0.1032 | 63.0 |
   | wing, correct handedness | 0.1132 | **52.1** |
   | wing, wrong handedness | 0.2060 | 89.3 |

4. **Drag bucket optimum in the wrong place.** The first calibration fitted c_d at the
   operating point and reproduced thrust and power correctly — but put the polar's best
   lift-to-drag angle at 6.4° instead of the report's 3.8–4.6°. It was invisible until the
   pitch designer used it and twisted the blade 2–5° wrong. The cause was scaling the
   *whole* drag bucket with Reynolds number, which freezes the optimum's location. Only
   the parasite term should scale; the lift-dependent term should not. With that fixed the
   optimum moves correctly with Reynolds number (c_l = 0.96 at Re 100k → 0.89 at 200k,
   matching the report) and the designer recovers the report's twist.

---

## Primary sources, and what was checked against them

`resources/` holds the two source documents the model's non-fitted parts are answerable
to. Both were checked directly rather than cited from memory.

**`19830010962.pdf`** — Viterna, L. A. and Corrigan, R. D., *Fixed Pitch Rotor Performance
of Large Horizontal Axis Wind Turbines*, NASA Lewis Research Center, 1981. The origin of
the post-stall extrapolation. `viterna_coefficients()` was checked term by term against
its equations (8), (9), (12), (14), (17) and (18) and matches them exactly.

It also supplied something the implementation was missing. Equation (13) gives the
flat-plate drag ceiling as a function of the blade's aspect ratio:

    c_d,max = 1.11 + 0.018 * AR,   AR <= 50

This blade is stubby — 0.30 m of span at 0.23 m mean chord, **AR 1.30** — so the
correlation gives **1.133**. The code had a hardcoded **1.8**, the 2-D infinite-span
value, which the paper argues against directly: a flat plate at AR 8 already measures
below 1.3. Deeply stalled drag was therefore overstated by **28–37 %** across 15–90° of
incidence. Now derived from the planform by `viterna_cd_max()`.

The operating point is unaffected — the blade runs at 3.3–5.5°, far from stall, and the
in-simulation score moved from 0.1132 to 0.1131. It matters for stalled transients,
spin-up, and the reversed-flow region, none of which were right before.

One thing the paper does *not* cover: its constants are matched at the stall angle and the
model is intended for α above it. A section at near-zero incidence in **fully reversed**
flow — which a spinning wing reaches, and which produced the `c_l = 57.7` singularity
listed above — is outside its scope. The ramp-to-zero treatment there is an engineering
choice of this implementation, not something the source sanctions.

**`Low-Speed-Airfoil-Data-V1.pdf`** — Selig, Guglielmo, Broeren and Giguère, *Summary of
Low-Speed Airfoil Data*, Vol. 1, UIUC. The right series at the right Reynolds numbers, but
**GOE 210 is not in it**. Volume 1 covers E387, SD7037, RG15, FX63-137, S1223, SD7003,
NACA 6409 and others — about two dozen sections, none of them Göttingen. So it cannot
validate this blade's polar directly.

It remains useful two ways, neither yet done: as an *indirect* check, by running the polar
model against a cambered section that is in it (E387 is the best-characterised) at Re 60k,
100k and 200k, which would test the Reynolds-trend machinery against wind-tunnel data
rather than against another model; and as a *design* input, since an airfoil with measured
low-Re data would anchor the whole model in measurement in a way GOE 210 cannot.

---

## Accuracy and real-time budget

Measured numbers — how accurate the model is, how much of the 4 ms physics step it costs,
whether it survives a spinning-and-rolling vehicle, and why a lookup table does not help —
are in **[FIDELITY.md](FIDELITY.md)**.

---

## Known limitations

Deliberate, documented in the module docstrings, and **not** covered by tests because they
are not modelled:

- **No blade flexibility.** The design report measured 10–15° of aeroelastic twist on an
  under-stiff 5 mm rod. Represent a known mean twist error with
  `COLLECTIVE_PITCH_OFFSET_DEG`.
- **No unsteady aerodynamics** — no dynamic stall, no shed-wake memory. Inflow responds
  instantaneously to a change in loading. This is the model's largest fidelity gap for a
  manoeuvring vehicle: the inflow time constant is about 67 ms against a 125 ms roll
  timescale, so it is not a small correction. See [FIDELITY.md](FIDELITY.md) §3.
- **No spanwise flow** in the section force (blade element independence).
- **Wing mass and inertia ARE now modelled**, and added to the rigid body at spawn
  (`APPLY_INERTIA_TO_BODY`). On the stock Iris the default panels add 9% to mass, **28% to
  I_zz** and 50% to I_yy. Disable with `--wing-no-inertia` for ablation only. The panel
  mass comes from an areal-density estimate (1.0 kg/m² → 67 g per blade); replace
  `BLADE_MASS_KG` with a weighed value once hardware exists.
- **Post-stall accuracy.** The Viterna branch is an engineering extrapolation, not
  validated data. Its structure and its drag ceiling now follow the source paper exactly
  (see above), but the tests still check only that it is smooth and bounded, not that it
  is accurate.
- **The section polar is fitted to another model, not to measurement.** The GOE 210
  coefficients come from the design report's NeuralFoil output. `resources/` does not
  contain measured data for this section — see above.
- **BEMT assumes axial flow.** Accuracy degrades as edgewise speed approaches tip speed;
  use `--wing-inflow glauert` for fast translation.
