# Spinning-drone high-fidelity dynamics — implementation

What this branch adds to the fork, why each piece exists, and what is deliberately not
done. Every number quoted here was measured on this machine against this code.

---

## 1. In one page

The vehicle spins because all four motors turn the same way, so their reaction torques add
instead of cancelling. Three things follow, and this work addresses all three.

| Addition | Why | Status |
|---|---|---|
| **Rotating-wing aerodynamics** — aerofoil panels on the arms, driven by the vehicle's own spin | Converts part of an otherwise parasitic yaw into lift | **Working.** 3.00 N at the settled spin, 18.7 % of vehicle weight, control effort −17 % |
| **Propeller gyroscopic reaction** | Co-rotating propellers carry 16 % of the vehicle's angular momentum; the moment exceeds the wing's own above ~1.5 rad/s | **Working.** On by default, `--no-rotor-gyro` to ablate |
| **Dynamic inflow** | Inflow has mass; τ ≈ 77 ms is 0.81 of a 12 rad/s controller bandwidth. Also makes the per-step cost constant | **Working.** `--wing-inflow dynamic` |
| **CFD section-data pipeline** | The section polar is answerable only to two codes that agree with each other | **Partial** — pipeline runs and the structured mesh converges cleanly; absolute values not yet validated, so no table is generated (§6) |

**184 offline physics tests** (no Isaac Sim, ~4 s) plus **18 end-to-end simulator
configurations**, all passing. The stock quadrotor path is bit-identical to before when the
new terms are disabled. See §8 for what was verified and how.

---

## 2. Configuration

Two files, by design. Wing and airframe *physics* in one, CFD *solver* in the other.

### `examples/utils/wing_params.py` — all dynamics parameters

The single file to edit. Every value carries a docstring explaining what it does and where
the number came from.

| Section | Covers |
|---|---|
| Master switch | `WING_ENABLED` |
| Atmosphere | density, kinematic viscosity |
| **Airframe dynamics** | `ENABLE_ROTOR_GYROSCOPICS`, `ROTOR_INERTIA_KG_M2`, `HOVER_ROTOR_SPEED_RAD_S` |
| Blade planform | blade count, **handedness**, radii, azimuths, z-offset |
| Build schedule | per-station radius, chord and pitch |
| Trim | `COLLECTIVE_PITCH_OFFSET_DEG` |
| Structure | areal density or weighed mass, `APPLY_INERTIA_TO_BODY` |
| Aerofoil | `USE_GOE210_POLAR`, `POLAR_OVERRIDE` |
| Induced velocity | `INFLOW_MODEL`, iteration cap, tolerance, relaxation, tip/root loss |
| Numerics | station count, speed clamp, moment reference |

Anything in it can be overridden per run from the command line; the file holds the
defaults.

### `cfd/config/solver.yaml` — CFD only

Image, rank count, turbulence model, freestream turbulence targeting, mesh sizing, the
(α, Re) sweep, and the per-case acceptance criteria.

---

## 3. Rotating-wing aerodynamics

`extensions/.../logic/aerodynamics/`

| Module | Contents |
|---|---|
| `airfoil_polar.py` | 2-D section polars; full ±180° via Viterna–Corrigan |
| `rotating_wing.py` | Blade element solver, four inflow models, load integration, mass/inertia |
| `wing_config.py` | Parameter schema with every invariant validated |
| `wing_design.py` | Re-derives the pitch schedule when the geometry changes |
| `vehicles/multirotors/spinning_wing_multirotor.py` | Simulator integration |

Blade element theory closed with momentum theory. Element velocity is the exact rigid-body
relation `v = v_body + ω × r`, so all six vehicle rates enter without approximation. The
package imports nothing but NumPy, which is why the tests run without Isaac Sim.

**Validated against the design report** (`foil_out/GOE210_DESIGN_REPORT.md`, an independent
BEMT optimisation using NeuralFoil):

| | Report | Model | Error |
|---|---|---|---|
| Thrust @ 20 rad/s | 3.740 N | 3.701 N | −1.0 % |
| Aero power | 7.31 W | 7.00 W | −4.2 % |
| Figure of merit | 0.722 | 0.741 | +2.6 % |

**And against XFOIL**, fetched independently into `lit_survey/`:

| Re | XFOIL max L/D | model | XFOIL best α | model |
|---|---|---|---|---|
| 50k | 40.7 | 44.7 | 5.50° | 5.62° |
| 100k | 62.0 | 62.5 | 4.75° | 4.62° |
| 200k | 82.3 | 88.6 | 3.75° | 3.88° |

Cl within 4–5 %, Cd within 1–5 %, and the best lift-to-drag angle within **0.13°** across
the operating range. Only the section polar was fitted; thrust, power, figure of merit and
the radial inflow distribution are integrated outputs that were not tuned to anything.

---

## 4. Airframe dynamics

### 4.1 Propeller gyroscopic reaction

`extensions/.../logic/dynamics/rotor_gyroscopics.py`, applied in `Multirotor.update()`.

```
h      = Σ I_rotor · ω_i · (−rot_dir_i)      about body +z
M_gyro = −ω_body × h
```

| | |
|---|---|
| rotor inertia about its spin axis | 7.65e-5 kg·m² (read from PhysX at runtime) |
| hover rotor speed | 684 rad/s, from `T = k ω²` |
| **net propeller momentum** | **0.209 kg·m²/s** |
| body momentum at 18 rad/s | 1.273 kg·m²/s |

**This matters here and nowhere else in Pegasus.** A conventional quadrotor sets
`rot_dir = [-1,-1,+1,+1]` and the four momenta cancel; this fork sets `[-1,-1,-1,-1]` and
they add. Against the wing's own aerodynamic moment:

| body rate | propeller gyro | wing moment | ratio |
|---|---|---|---|
| 2 rad/s | 0.419 N·m | 0.311 N·m | 1.3× |
| 4 rad/s | 0.837 N·m | 0.364 N·m | **2.3×** |

Above roughly 1.5 rad/s the term exceeds the entire aerodynamic contribution of the wing
being studied, which is what justified adding it.

Note the USD's rotor joints are driven kinematically at a hardcoded 100 rad/s by
`handle_propeller_visual()` — "for animation purposes only" — so the articulation carried
at most 15 % of the true momentum and none of it reliably. The term is therefore applied
explicitly as a body torque.

This touches stock quadrotor physics. `--no-rotor-gyro` reproduces prior results exactly
(verified: effort 49.0251, matching the pre-change baseline to all digits).

### 4.2 Dynamic inflow

`InflowModel.DYNAMIC`, selected with `--wing-inflow dynamic`.

Momentum theory says what induced velocity *eventually* balances a given thrust. It does
not say how fast the flow gets there, and the answer is not "instantly" — the air column
through the disc has mass.

```
m_a   = (8/3) ρ R³                            apparent mass of the air column
τ     = m_a / (dT/dν)                         ≈ 77 ms, matching 2R/(3πν) exactly
ν    ← ν + (ν_steady − ν)(1 − exp(−dt/τ))     exact ODE solution, not forward Euler
```

The apparent mass is the impermeable-disc value, which Pitt & Peters (1980) — in
`lit_survey/ERF 1980-47.pdf` — confirm is exactly their M₁₁ term.

**Why τ matters:**

| | timescale | τ/T |
|---|---|---|
| roll at 8 rad/s | 125 ms | **0.54** |
| controller ωn = 12 rad/s | 83 ms | **0.81** |
| spin period at 18 rad/s | 349 ms | 0.19 |

Instantaneous inflow is not a conservative approximation: it makes the wing respond to
rate changes immediately, **overstating aerodynamic damping and phase margin** exactly
where a controller study looks.

**It is also the only inflow model with a bounded per-step cost**, because it advances a
state instead of iterating a fixed point. Measured during manoeuvre:

| model | mean | worst | inflow passes |
|---|---|---|---|
| bemt | 5678 µs | 40238 µs | up to **32** |
| glauert | 2310 µs | 9919 µs | up to 10 |
| **dynamic** | **1280 µs** | **5216 µs** | **always 1** |

4.4× cheaper in the mean, 7.7× better worst case. (Absolute values inflated by a
concurrent CFD job; the ratios are clean.)

The quasi-steady target solves Glauert's relation with a **fixed** three-step Newton on a
scalar — bounded work, unlike the blade-wise fixed point that re-evaluates the polar at
every element on every pass.

**Not included:** the first-harmonic inflow states, which need the full Pitt–Peters
L-matrix as a function of wake skew angle. Uniform component only.

`bemt` remains the default so existing results stay reproducible.

---

## 5. Real-time behaviour

Physics runs at 250 Hz → **4000 µs per step**, one vehicle.

| | mean | worst |
|---|---|---|
| hover | 487 µs (12 %) | 3179 µs (79 %) |
| manoeuvre, `bemt` | 2177 µs (54 %) | **11572 µs (289 %)** |
| manoeuvre, `dynamic` | constant-time, one pass | bounded |

**For offline sweeps and software-in-the-loop — what this repository does — timing is
irrelevant**: simulated `dt` is unaffected by how long a step takes to compute, so results
are identical either way. The bounded worst case matters only with hardware or an operator
in the loop, and `--wing-inflow dynamic` is what delivers it.

### A measured result worth recording: lookup tables do not buy speed here

Three attempts, all measured:

| attempt | result |
|---|---|
| tabulate the section polar `(α, Re)` | **no change** — 2135 µs → 2210 µs |
| tabulate the whole wing wrench over vehicle state | **failed**: 34 % worst-case thrust error on an 11×11×7×3×7 grid, *and* slower than direct evaluation |
| compile the inner loop (numba) | **9× faster, bit-identical results** |

With 48 blade elements the cost is NumPy per-call dispatch overhead, not arithmetic, so
replacing arithmetic with a table lookup breaks even. Compiled, the arithmetic dominates —
which is the regime where a lookup table *does* pay. **Compile first; the table then
becomes free.** (numba was measured and then removed: it pulled numpy 2.2.6 into user
site-packages and broke scipy. Not currently installed.)

---

## 6. CFD pipeline

`cfd/` — see `cfd/README.md` and `cfd/PLAN.md`.

**Working:** OpenFOAM v2506 via its official container (no sudo), gmsh via pip, case
generation, parallel solve, force extraction, y+ checking, `config/solver.yaml`.

The physics setup is demonstrably right: y+ max 1.33 / mean 0.36, `kOmegaSSTLM` transition
model, freestream turbulence back-solved so the requested intensity arrives **at the
aerofoil** (inlet 0.104 % → 0.100 %), and a mean Cl within **1.2 %** of XFOIL.

**The mesher now works.** Two meshers were written; the structured one is correct.

| mesh | max non-orth | negative cells | y+ max | Cl drift |
|---|---|---|---|---|
| hybrid (gmsh BL quads + triangles) | 85.7° → 78.2° | — | 1.33 | **±2.74 → ±1.39** |
| **structured C (4 transfinite blocks)** | 87.4° | **0** | **0.785** | **±0.0041** |

The hybrid mesh's bad cells sat where its structured layers met the unstructured region,
which is also where the separation bubble forms, and the solution oscillated by ±74 % about
a mean that was itself within 1.2 % of XFOIL. No solver setting fixes mesh topology.

`cfd/mesh/cmesh.py` replaces it with four transfinite blocks: the aerofoil split at the
leading edge to avoid a degenerate corner, the wake cut shared by both wake blocks so the
mesh is conformal across it, **34,560 cells against 333,000** — and solves **9× faster**,
0.073 s/iteration against 0.66.

Three defects were found and fixed getting there:

1. **A duplicate point at the trailing edge.** The outer arc was given its own centre point
   coincident with the TE, which gmsh meshes as a separate node. `checkMesh` named it
   exactly: "2 conflicting points to set nonManifoldPoints".
2. **A blunt trailing edge forced onto a sharp-TE topology.** GOE 210 ships with a 0.16 %
   chord gap; the lower surface was being started at the *upper* TE point.
3. **Clustering the far-field arc.** This was the one that mattered. Transfinite
   interpolation blends between opposite edges, so clustering *both* the aerofoil and the
   arc pinches the outer ends of the radial lines toward the same corners — and over 60
   chords neighbouring lines cross and the first cell layer inverts. Measured on the lower
   surface near x = 0.10: **303 inverted cells at Bump 0.08, 180 at 0.2, 47 at 0.5, and
   exactly 0 with the arc uniform.** Clustering the far field bought nothing anyway; it has
   no feature to resolve.

The solver also needed changing. SIMPLEC (`consistent yes`) with the aspect ratios a C-mesh
carries into the far wake left the pressure residual on a plateau at 1e-2 and the
coefficients alternating in sign between consecutive iterations. Plain SIMPLE with explicit
pressure relaxation, a tighter GAMG tolerance and relaxation at 0.5 converges cleanly.

### Status: converges reliably, not yet validated

The solver side works. What is not established is absolute accuracy.

| | CFD | XFOIL | difference |
|---|---|---|---|
| **E387**, α = 4°, Re = 2e5 | Cl 1.0381, Cd 0.01117 | Cl 0.8365, Cd 0.01230 | **Cl +24.1 %, Cd −9.2 %** |
| **GOE 210**, α = 5°, Re = 1e5 | Cl 1.236, Cd 0.0246 | Cl 0.951, Cd 0.0159 | Cl +30 %, Cd +56 % |

Both converge cleanly — E387 reports `settled: true`, Cl drift 0.00025, y+ max 0.82 — so
this is not a convergence artefact.

**It is not the patch bug either.** Re-running E387 on the corrected mesh changed the
answer by **Cl +0.00 %, Cd −0.06 %**: the first of the two duplicate `aerofoil` patches had
in fact contained the complete surface. The fix removes a genuine latent hazard but does
not explain the discrepancy.

**The signature points at transition.** Lift up *and* drag down together is what earlier
transition produces — a shorter separation bubble, higher effective camber, less pressure
drag. A scaling error, a reference-area mistake or a lift-direction mistake would move both
coefficients the same way. So the suspect is `kOmegaSSTLM`'s response to the conditions it
is being given, and the leading candidates are:

1. **`gammaInt` initialised to 1**, i.e. fully turbulent everywhere, requiring the model to
   *relaminarise* the attached region rather than transition into turbulence. The ERCOFTAC
   T3A tutorial does this, but T3A is a flat plate where the correlation forces transition
   anyway.
2. **Freestream turbulence at the low end of the model's calibration.** Tu = 0.1 % at the
   aerofoil matches a clean tunnel and XFOIL's Ncrit 9, but Langtry–Menter is calibrated
   mainly above a few tenths of a percent.
3. **The wall `omega` treatment** injecting eddy viscosity into the laminar region.

For reference, published measurements put E387 at Re 2e5, α = 4° near Cl 0.75–0.80, so
XFOIL is close to measurement and the CFD is the outlier. (The Selig volume is a scanned
PDF whose polar tables do not OCR reliably, so this is from the literature rather than
extracted here.)

**No table is generated and nothing reaches the simulator until this is closed.** The
aerodynamic model in use remains the one validated against the design report and XFOIL,
which is unaffected by any of this.

**Do not start the 234-case sweep until one case converges cleanly against XFOIL.** A week
of compute on meshes like these produces confident-looking drag that is wrong.

---

## 7. Tests

`examples/controller_tests/physics_tests/` — 184 tests, no Isaac Sim, ~4 s. Runs under pytest, under the standalone runner, and under Isaac Sim's own Python 3.11 (§8.2).

```bash
python3 -m pytest examples/controller_tests/physics_tests -q
python3 examples/controller_tests/physics_tests/run_physics_tests.py   # no pytest needed
```

| Module | Covers |
|---|---|
| `test_airfoil_polar.py` (31) | Lift slope, drag bucket, Reynolds scaling, Viterna matching, ±180° boundedness, the aspect-ratio drag ceiling |
| `test_wing_validation.py` (19) | Agreement with the design report, grid convergence, momentum cross-check |
| `test_wing_physics.py` (39) | Energy, scaling laws, symmetry, null directions, flight regimes, NaN robustness |
| `test_wing_config.py` (29) | Every schema invariant, from both directions |
| `test_wing_structure.py` (15) | Mass and inertia against closed forms, not recorded numbers |
| `test_wing_design.py` (16) | The pitch designer; recovers the report's own twist to 0.55° |
| `test_wing_handedness.py` (9) | Mirror symmetry; agreement with the motor directions |
| `test_rotor_gyroscopics.py` (15) | Cross-product form, sign convention, does no work, magnitude vs the wing |
| `test_dynamic_inflow.py` (13) | Steady limit, time constant vs the apparent-mass derivation, 63 % step response, exactly one pass |

Where a closed form exists it is tested against that rather than against the model's own
output — the uniform-rod inertia, the parallel-axis theorem, `P = −Q_z·Ω`, `−ω × h`.

### Defects these tests caught

1. **Viterna singularity at ±180°** — `cos²α/sin α` diverges in reversed flow, a state a
   spinning wing genuinely reaches. `c_l` hit 57.7.
2. **Prandtl root loss** applied alongside tip loss put thrust 10 % low and made the inflow
   iteration oscillate. The root is bonded to the arm, not free; now off.
3. **The panels were handed the wrong way.** Built for positive spin on an airframe that
   spins negative, they produced **−2.2 N — downforce** — plus 133 g of mass. Invisible
   because every test evaluated at positive spin. Fixed with `ROTATION_SENSE`, and the
   suite's design spin rate now derives from the configured handedness.
4. **Drag bucket optimum in the wrong place** — reproduced thrust and power but put best
   L/D at 6.4° instead of 3.8–4.6°, because the *whole* bucket was scaled with Reynolds
   number. Only the parasite term should scale. Later confirmed by XFOIL to 0.13°.
5. **Post-stall drag ceiling 60 % too high** — Viterna eq. (13) gives
   `c_d,max = 1.11 + 0.018·AR`, which for this stubby AR 1.3 blade is 1.13, not the 2-D
   value of 1.8.

---

## 8. Verification

Two layers, both repeatable. The offline suite checks the mathematics; the simulator
harness checks that every configuration actually spawns, steps and scores inside Isaac Sim,
which is where integration mistakes live -- a mis-wired flag, a missing `dt`, a prim that
never receives the wing's mass, a NaN reaching PhysX.

### 8.1 End-to-end simulator harness

`examples/controller_tests/verify_simulation.py` -- 18 configurations. Each asserts three
things: the run exits cleanly, it emits a **finite** score, and that score falls in a
physically plausible band. A run that completes but scores infinity has diverged, which an
exit code alone would not catch.

```bash
python3 examples/controller_tests/verify_simulation.py            # full, ~6 min
python3 examples/controller_tests/verify_simulation.py --quick    # skips multi-vehicle
```

Result: **18/18 passed.**

| # | configuration | score |
|---|---|---|
| 1 | baseline, no wing | 0.4220 |
| 2 | baseline, gyroscopics ablated | 0.4220 |
| 3 | wing, default (asserts the mass reaches the rigid body) | 0.4243 |
| 4 | wing, inertia ablated | 0.4151 |
| 5–7 | inflow `none` / `glauert` / `dynamic` | 0.4182 / 0.4231 / 0.4210 |
| 8–11 | 4 blades / collective +2° / retargeted span / 48 stations | 0.4343 / 0.4214 / 0.4272 / 0.4245 |
| 12 | wrong handedness (ablation) | 0.4490 |
| 13–14 | six vehicles in one scene, without and with wing | 0.4220 / 0.4243 |
| 15–16 | PD sweep, without and with wing | 0.4283 / 0.4275 |
| 17–18 | MPC sweep, without and with wing | 0.5177 / 0.5800 |

Case 3 additionally requires `"wing structure added"` in the log, so a silent failure to
apply the wing's mass and inertia to the `/body` prim fails the check rather than passing
with the aerodynamics alone.

### 8.2 Offline suite under all three interpreters

| path | result |
|---|---|
| `pytest`, system Python 3.10 | **184 passed** |
| standalone runner, system Python 3.10 | **184 passed** |
| standalone runner, **Isaac Sim's Python 3.11** | **184 passed** |

The third did not work until now. The runner itself needed no pytest, but the *test
modules* import it for `approx`, `raises`, `fixture` and `mark` -- so the documented
promise that the suite runs "where pytest is not installed, e.g. Isaac Sim's bundled
interpreter" was false and failed at import. A minimal shim now supplies exactly those four
features.

A shim that passed everything would be worse than none, so its semantics are checked
directly: `approx` accepts matches within tolerance and **rejects** differences outside it
(default, `rel=`, and `abs=` forms), `raises` raises `AssertionError` when nothing is
raised, and it propagates an exception of the wrong type rather than swallowing it.

### 8.3 Analysis and visualisation scripts

Run under Isaac's interpreter against real recorded results:

`benchmarks.py`, `tuning_equivalence.py`, `eigen_plot_heatmap.py`, `eigen_plot_summary.py`,
`pareto_comparison.py` -- all pass.

### 8.4 Modularity audit

| check | result |
|---|---|
| The six "pure" modules import no `pxr`, `omni`, `isaacsim` or `carb` | **clean** |
| The aerodynamics package imports standalone | **yes** -- zero Isaac modules pulled in |
| Circular imports between `logic/` subpackages | **none** |
| Layering | one-way: `vehicles` → {`aerodynamics`, `dynamics`, `thrusters`, `sensors`, `backends`}; `aerodynamics` depends on nothing outside itself |

That property is what makes 184 tests run in under four seconds without a simulator, and it
is worth protecting: the moment the aerodynamics package imports Isaac Sim, the offline
suite stops existing.

---

## 9. Known limitations

- **First-harmonic inflow states** absent (§4.2).
- **Edgewise validity ends at μ ≈ 0.40**, reached at only 3.6 m/s sideways because the
  wing's root sits far out at 0.2 m. Above that, BEMT's per-annulus balance cannot
  represent the fore-aft inflow gradient; the three inflow models disagree by 21–31 % in
  thrust. Coleman's linear inflow (`lit_survey/`, NACA ARR L5E10) is the analytic fix and
  is not yet implemented.
- **Rotational augmentation (Himmelskamp)** not modelled. The blade's aspect ratio is 1.3,
  which is where the effect is largest.
- **Blade flexibility** not modelled — and the design report measured **10–15° of
  aeroelastic twist** on a 5 mm rod, against a 4–5° design angle of attack. Measured
  sensitivity: −10° of twist costs **54 %** of thrust, −15° costs **77 %**. This is the
  largest single uncertainty in the whole model, 7–25× the aerodynamic-model uncertainty,
  and it sits upstream of it. Represent a known mean twist with
  `COLLECTIVE_PITCH_OFFSET_DEG`.
- **No measured aerofoil data exists for GOE 210.** Searches of the Göttingen *Ergebnisse*
  volumes and the NACA/NASA archives found none. The polar is validated against XFOIL,
  which is computation, not measurement.
- **The Iris asset is not the airframe this wing was designed for.** The report's vehicle
  is ~500 g with arms reaching 0.5 m; the asset is 1.5 kg with motors at 0.249 m. Wing lift
  reads as 19 % of weight in simulation where it would be ~55 % on the intended airframe.

---

## 10. Running it

```bash
# baseline: stock quadrotor, no panels
isaac_run examples/controller_tests/sweeps/eigen_stability_sweep.py \
    --motor-moments 6e-7 --omega-n-values 2 4 6 8 10 12 --duration 20 --headless --no-wing

# with the rotating wing
isaac_run examples/controller_tests/sweeps/eigen_stability_sweep.py \
    --motor-moments 6e-7 --omega-n-values 2 4 6 8 10 12 --duration 20 --headless --wing

# with dynamic inflow: adds the 77 ms lag and makes the cost constant-time
isaac_run ... --headless --wing --wing-inflow dynamic
```

`--no-wing` is the default, so any command line written before this work keeps its previous
meaning.

Other flags: `--wing-collective-deg`, `--wing-blades`, `--wing-root-radius`,
`--wing-tip-radius`, `--wing-inflow {none,bemt,glauert,dynamic}`, `--wing-stations`,
`--wing-rotation-sense`, `--wing-no-inertia`, `--no-rotor-gyro`.

### Measured effect of the wing, single vehicle

Measured at 20 s, one vehicle, `--motor-moments 6e-7 --omega-n-values 2`:

| configuration | score | effort |
|---|---|---|
| `--no-wing` | 0.1033 | 62.34 |
| `--no-wing --no-rotor-gyro` | 0.1032 | 63.00 |
| **`--wing`** | 0.1131 | **52.07 (−16.5 %)** |
| **`--wing --wing-inflow dynamic`** | 0.1332 | **49.61 (−20.4 %)** |
| `--wing --wing-rotation-sense 1` (wrong handedness) | 0.1940 | 87.44 |

| | no wing | wing |
|---|---|---|
| settled spin | −26.96 rad/s | **−18.00 rad/s** |
| wing lift | — | **3.00 N (18.7 % of weight)** |
| mass / I_zz | 1.500 kg / 0.0552 | 1.633 kg / 0.0707 |

The spin slows partly from wing drag, but mostly because lift unloads the rotors, cutting
the reaction torque that drives the spin in the first place.

---

## 11. Further reading in this repository

| Document | Contents |
|---|---|
| `examples/controller_tests/physics_tests/README.md` | What the model is, what the suite covers, the defects it caught |
| `examples/controller_tests/physics_tests/FIDELITY.md` | Accuracy and real-time budget, measured: spinning + rolling, edgewise limits, gyroscopics, polar validation |
| `examples/controller_tests/verify_simulation.py` | End-to-end simulator harness, 18 configurations |
| `cfd/README.md` | CFD pipeline, how to run it, measured cost |
| `cfd/PLAN.md` | Full CFD campaign plan and priority ordering |
| `lit_survey/` | Primary sources: Viterna & Corrigan 1981, Pitt & Peters 1980, Coleman ARR L5E10, Dodic 2023, XFOIL polars |
