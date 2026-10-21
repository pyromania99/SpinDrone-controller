# CFD-derived high-fidelity aerodynamics — implementation plan

Target: replace fitted/analytic section data with CFD, keep the simulation real-time
through lookup tables, and keep solver and wing-design parameters in configuration files.

---

## 0. Two corrections to the premise, before any work starts

Both are measured, not opinions. Neither blocks the plan; both change its ordering.

### 0.1 LUTs do not buy speed in this codebase

Tabulating the polar was measured (`physics_tests/FIDELITY.md` §4):

| polar | mean | worst |
|---|---|---|
| analytic (current) | 2135 µs | 7443 µs |
| LUT 361×8 | 2210 µs | 10143 µs |

**Every LUT tested was slightly slower.** The arrays are tiny — 2 blades × 24 stations =
48 elements — so cost is NumPy per-call dispatch overhead, not arithmetic. Bilinear
interpolation needs its own searchsorted and gather, costing roughly what the closed form
costs.

LUTs are still the right vehicle for CFD data: they are how CFD results get *into* a
real-time loop at all. But they buy **fidelity, not speed**. Real-time headroom comes from
removing the inflow iteration (§3), not from tabulation. Plan accordingly, or the first
CFD table will look like a performance regression and the reason will be unclear.

#### What DOES make it cheaper -- measured

Three things were tried. Only one worked.

| attempt | result |
|---|---|
| tabulate the section polar `(α, Re)` | **no speed change** (2135 → 2210 µs); measured three times |
| tabulate the whole wing wrench over vehicle state | **failed both ways** -- see below |
| **compile the inner loop (numba)** | **9× faster, bit-identical results** |

**The wrench LUT was tested and does not work.** Tabulating the complete force and moment
over the non-dimensional state (μx, μy, μz, p̂, q̂) on an 11×11×7×3×7 grid -- 17,787 states,
0.4 MB -- gave, at off-grid states:

| | mean error | p95 | max |
|---|---|---|---|
| thrust | 4.2 % | 14.8 % | **34.2 %** |
| pitching moment | 4.8 % | — | 36.1 % |

and the lookup was **slower than evaluating the model directly** (1433 µs vs 1275 µs),
because `scipy.interpolate.RegularGridInterpolator` carries the same per-call Python
overhead that dominates everything else here. A finer grid would fix the accuracy at the
cost of memory and build time, but nothing fixes the speed in pure Python -- and the whole
idea is strictly worse than compiling, because the blade element kinematics are *exact*
whereas an interpolated wrench is not.

**Compiling is the lever.** The same per-element work, 48 elements:

| | per call | per element |
|---|---|---|
| NumPy, vectorised | 38.95 µs | 811 ns |
| numba, compiled loop | **4.27 µs** | **89 ns** |

Results agree to 0.0e+00 — bit-identical. 811 ns per element for a dozen floating-point
operations is almost entirely NumPy dispatch overhead on tiny arrays, which is the same
diagnosis every other measurement in this file points at.

**And this is what finally makes a lookup table pay.** In NumPy, a table lookup costs about
what evaluating the polar costs, because both are overhead-bound. Compiled, the arithmetic
genuinely dominates — which is precisely the regime where replacing arithmetic with a
memory read wins. The order matters: **compile first, and the CFD table then becomes free.**

Caveats: numba must be installed into Isaac Sim's own interpreter (Python 3.11; untested
there), and `cache=True` is needed so the one-off JIT cost is not paid every run.

### 0.2 Section aerodynamics is not currently the dominant error

The fitted polar was validated against XFOIL (§3d of FIDELITY.md): **Cl within 4–5 %, Cd
within 1–5 %, best-L/D angle within 0.13°** across the operating Reynolds range, and
thrust within 3 % of two independent routes. CFD would plausibly improve that to 3–5 %.

Meanwhile, two terms are **entirely absent**:

| missing term | size |
|---|---|
| propeller gyroscopic momentum | **2.3× the wing's whole aerodynamic moment** at 4 rad/s body rate |
| dynamic inflow lag (τ = 67 ms) | τ/T = **0.54** against roll, **0.81** against controller bandwidth |

For sim2real in *controller* development these dominate, because both are dynamic terms
that shift phase and damping — exactly what a controller is tuned against. A CFD campaign
that improves a 5 % static error while a 230 % dynamic error remains will not move
sim2real.

**Recommended ordering: do §3 first (days, no CFD), then the CFD campaign.** The CFD work
below is specified in full regardless, and §3 is a prerequisite for the real-time budget
the CFD tables will have to live inside.

### 0.3 The dominant aerodynamic uncertainty is the blade's shape, not the flow model

`foil_out/GOE210_DESIGN_REPORT.md` section 2 records a measurement, not a prediction:

> "On the 5 mm rod the wing twisted about **10–15°** in a real test."

The design pitch schedule is 16.5–24.7° and the target angle of attack is 4.4–5.5°. A
twist of 10–15° is therefore **larger than the entire angle-of-attack budget**. Measured
against the model at the design point:

| rigid twist offset | thrust | α across the span |
|---|---|---|
| 0° (as designed) | 3.701 N | +3.3 to +5.5° |
| −5° | 2.654 N (**−28 %**) | +1.0 to +2.6° |
| −10° | 1.698 N (**−54 %**) | −1.2 to +0.4° |
| −15° | 0.861 N (**−77 %**) | −3.1 to −1.6° |
| +10° | 3.766 N | +8.4 to +16.2° — **stalled** |

Against the section-aerodynamics uncertainty already quantified:

| source | effect on thrust |
|---|---|
| XFOIL vs the fitted polar | **3.0 %** |
| a 4.5× error in section drag | **7.5 %** |
| **the measured aeroelastic twist** | **28–77 %** |

The geometry uncertainty is **7 to 25 times** the entire aerodynamic-model uncertainty, and
it sits *upstream* of it: CFD on the as-designed rigid blade computes the flow around a
shape the hardware does not hold. Nose-down twist drives the sections to zero or negative
incidence; nose-up twist stalls them. Neither is a refinement — both are the difference
between a wing that lifts and one that does not.

The report is aware of this and mitigates by placing the rod at 35–40 % chord, where lift
and section moment nearly cancel at design C_L, and by tapering it 10 → 8 → 5 mm. It also
states plainly: *"Measure the real torsional stiffness of the finished rod before trusting
the model."* That instruction is still outstanding, and no amount of CFD substitutes for
it.

**Consequence for this plan: an aeroelastic twist model outranks the entire CFD
campaign**, and a bench measurement of the finished rod's GJ outranks both. Both are
cheap. See §8.

---

## 1. The architectural decision: tabulate the section, not the wrench

The instinctive reading of "CFD instead of BET" is to compute the wing's total force and
moment with CFD and tabulate *that* against vehicle state. **That is not feasible, and the
arithmetic is worth seeing once.**

The wing's loads depend on `v_body` (3) and `ω_body` (3). Non-dimensionalised that is

```
mu_x, mu_y, mu_z   advance ratios          (3)
p_hat, q_hat       rates normalised by Om  (2)
Re_tip             tip Reynolds number     (1)
```

six dimensions, before any design parameter. At 8 samples per dimension that is 262,144
CFD cases; at 10, a million. Each 3-D unsteady rotating case is hours to days. Two-blade
symmetry halves it. It remains impossible — by three to four orders of magnitude, not by a
factor that better hardware fixes.

**The resolution is to put CFD where the modelling error is and keep BET where it is
exact.** Blade element theory's state dependence is pure rigid-body kinematics —
`v_element = v_body + ω × r` — which has *no* modelling error. Its error is entirely in
(a) the section coefficients and (b) the induced inflow. So:

| layer | fidelity source | table dimensions |
|---|---|---|
| section coefficients | **2-D CFD** (§2) | `(α, Re)` → 2-D |
| rotational augmentation | 3-D CFD, a few cases → correction model (§5) | `(α, Re, r/R)` → 3-D |
| induced inflow in edgewise flight | analytic Coleman, then mid-fidelity wake (§4) | `(μ, χ)` → 2-D |
| state dependence | BET kinematics, exact | none |
| dynamic response | Pitt–Peters states (§3) | none |

Every table is 2-D or 3-D. All of them together are under a megabyte. This is how
production rotorcraft comprehensive codes are built (CAMRAD, RCAS): CFD or tables for the
section, lifting-line/BET for integration, CFD–CSD coupling only at the top end.

**The codebase already supports this.** `AirfoilPolar.evaluate(alpha_rad, reynolds)` is an
abstract interface with a working `TablePolar` implementation; `_solve_inflow` dispatches
on an enum. CFD data enters as a new `AirfoilPolar` subclass and a new `InflowModel`
member. No change to the solver, the vehicle class, or any sweep script.

---

## 2. Stage 1 — 2-D section CFD (feasible on this machine)

### 2.1 The Reynolds-number trap

The blade runs at **Re 5×10⁴ to 2.4×10⁵**. In that range the boundary layer is laminar
over much of the chord and a **laminar separation bubble** governs both lift and drag.

A fully turbulent RANS solution at Re 10⁵ is not merely approximate — it is
*qualitatively* wrong, and will be **worse than the XFOIL data already in the repository**,
because XFOIL's e^N method does model transition. Any CFD campaign here must use a
transition-sensitive model:

- **Langtry–Menter SST γ–Re_θ** (four-equation) — the standard choice, available in SU2 and
  OpenFOAM.
- **SA-BCM** or **γ-only SST** — cheaper, usually adequate.

If a transition model is not used, stop: the result will not be an improvement.

### 2.2 Solver choice — OpenFOAM or SU2

Revised recommendation: **if Stage 3 (3-D rotating) is actually coming, use OpenFOAM
throughout.** Learning one solver well beats learning two, and OpenFOAM's 2-D airfoil
capability is adequate — transitional low-Reynolds airfoil work including the E387
separation bubble has been published in OpenFOAM-v2312.

| | OpenFOAM | SU2 |
|---|---|---|
| 2-D low-Re airfoil | adequate; more files, more care | faster to a correct answer |
| transition models | `kOmegaSSTLM`, γ-SST, `kkLOmega` | Langtry–Menter, SA-BCM |
| **steady rotating hover (MRF)** | **mature — the standard path** | available, less travelled |
| **unsteady edgewise (sliding mesh / AMI)** | **mature — `pimpleFoam` + `dynamicMeshDict`** | weak |
| overset bodies | yes (ESI builds) | limited |
| meshing real geometry from STL | `snappyHexMesh` | needs external mesher |
| Python driver | shell / PyFoam | first-class `pysu2` |
| learning curve | weeks | days |

The decisive column is the rotating one. This vehicle's whole airframe spins, so hover is
naturally an MRF case with the entire vehicle in a rotating frame, and edgewise flight —
where periodicity is destroyed and the model's validity limit lies (§4) — requires a true
sliding-mesh unsteady solve. That is OpenFOAM's well-trodden ground and SU2's weak spot.

**Use SU2 only if** Stage 1 is the whole campaign and 3-D is definitely out of scope.

#### Which OpenFOAM

Two diverged distributions:

- **ESI / openfoam.com** (`v2412`, `v2506`, …) — better support for rotating machinery,
  AMI and overset. **Recommended here.**
- **Foundation / openfoam.org** (`12`, `13`, …) — also has AMI; smaller rotating-machinery
  ecosystem.

**Do not use Ubuntu's own `openfoam` package** — it is `1912.200626`, from 2019. Add the
upstream apt repository instead.

#### The five ways a low-Reynolds OpenFOAM case goes wrong

In rough order of how often they do:

1. **Freestream turbulence decay.** `kOmegaSSTLM` keys transition off the local turbulence
   intensity, and `k`/`omega` *decay* between the inlet and the aerofoil. Setting the
   inlet values to the tunnel's measured intensity puts the wrong value at the leading
   edge, which moves the transition point and invalidates the result. Choose inlet values
   so the decayed value **at the aerofoil** matches the reference condition, and verify by
   probing upstream of the leading edge in a blank run.
2. **Turbulent wall treatment.** Transition models require `y+ < 1`, 30–50 prism layers and
   a growth ratio ≤ 1.15. Wall functions silently defeat the model.
3. **`snappyHexMesh` for a 2-D aerofoil.** It is the wrong tool — hex-dominant meshing
   struggles at a sharp trailing edge and layer coverage collapses there. Use `blockMesh`
   with a C-grid, or `gmsh`, for sections; keep `snappyHexMesh` for Stage 3 geometry.
4. **Farfield too close.** 2-D aerofoil needs 50–100 chords with freestream (not
   zeroGradient) boundaries, or the lift is wrong by percent-level amounts that look like
   physics.
5. **Model choice by default.** `kOmegaSSTLM` is the well-documented four-equation option;
   recent work finds **γ-SST** gives the best accuracy-to-cost balance on E387
   specifically. `kkLOmega` is a reasonable third option but has been found to
   under-predict lift and delay reattachment at Re 6×10⁴ on SD7003.

### 2.3 Cost on the available hardware

8 physical cores, 30 GB RAM. A 2-D C-grid of 150–300k cells converges in **5–15 minutes**.

| sweep | cases | wall time |
|---|---|---|
| α −8…+20° at 1°, Re ∈ {50, 75, 100, 140, 200, 240}k | ~170 | ~1–2 days |
| plus deep stall to +40° for the Viterna match | ~60 | +half a day |

Entirely feasible. Run it once, commit the table.

### 2.4 Validate the CFD setup before trusting it

`lit_survey/` already contains Selig et al., *Summary of Low-Speed Airfoil Data* Vol. 1.
It has no GOE 210 — but it has **measured wind-tunnel data for E387 at Re 60k–300k**, the
canonical low-Re validation case.

**Run the E387 case first and reproduce the measured polar.** Only then run GOE 210. This
converts the Selig volume from a dead end into the campaign's validation anchor, and it is
the difference between "we ran CFD" and "we ran CFD that is known to work at this Reynolds
number".

Acceptance: Cl within 3 % and Cd within 8 % of the tunnel data over −4…+10°, with the
bubble's drag signature reproduced.

### 2.5 Deliverables

```
cfd/
  config/
    solver.yaml        mesh density, turbulence+transition model, CFL, convergence,
                       alpha list, Reynolds list, farfield distance
    cases.yaml         which aerofoils to run (goe210, e387 validation)
  geometry/            coordinates, from foil_out/database/
  mesh/                generator (2-D C-grid; analytic, no mesher dependency)
  run/                 case generation + batch driver + restart/resume
  post/                force-coefficient extraction, convergence checks, table assembly
  tables/
    goe210_cfd_v1.npz  (alpha, Re) -> cl, cd, cm  + provenance metadata
```

and in the simulator:

```python
# aerodynamics/airfoil_polar.py
class CFDTablePolar(TablePolar):
    """Section polar from a CFD campaign. Carries provenance; extends to +/-180 deg
    with Viterna using the measured stall point as the match."""
```

A new `POLAR_SOURCE = "cfd"` switch in `examples/utils/wing_params.py` selects it, so the
existing single-file configuration requirement is preserved and the analytic polar stays
available for comparison.

---

## 3. Stage 0 — the dynamic terms (do this first, no CFD needed)

Both are small, both are the dominant sim2real errors, and the second is a **prerequisite
for the real-time budget** that CFD tables must fit inside.

### 3.1 Propeller gyroscopic momentum — a few lines

`handle_propeller_visual()` drives the rotor joints kinematically at a hardcoded 100 rad/s;
real hover speed is 684 rad/s. Because this fork sets `rot_dir = [-1,-1,-1,-1]`, the four
propeller momenta **add** instead of cancelling: 0.209 kg·m²/s, 16 % of the whole vehicle's
angular momentum.

The thrust curve already computes each ωᵢ. One extra torque on `/body`:

```
h     = sum_i  I_rotor * omega_i * rot_dir_i   (about +z)
M_gyro = -omega_body  x  h
```

Same `apply_torque` pattern the wing already uses. Note this touches stock quadrotor
physics, which has been off-limits for the wing work — it needs an explicit decision.

### 3.2 Pitt–Peters dynamic inflow — three states

`lit_survey/ERF 1980-47.pdf` is the primary source, complete. Replacing the converge-every-
step inflow with three first-order states:

- fixes the τ = 67 ms lag, which is 0.54 of the roll timescale and 0.81 of controller
  bandwidth;
- **removes the fixed-point iteration**, collapsing cost from `266 + 215·n_iter` µs to a
  constant ~266 µs and eliminating the 11.6 ms worst-case spike.

Accuracy and real-time, one change. This is what makes room for CFD tables.

---

## 4. Stage 2 — induced inflow in edgewise flight

The measured validity limit is **μ = 0.40, reached at only 3.6 m/s sideways** (tip speed
is 9 m/s, root speed 3.6 m/s). Above it, BEMT's per-annulus momentum balance cannot
represent the fore-aft inflow gradient, and the three inflow models in the code disagree
by 21–31 % in thrust.

Two steps, cheapest first:

1. **Coleman linear inflow** — `ν = ν₀(1 + k_x·(r/R)·cos ψ)`, with `k_x` a function of wake
   skew angle χ. `lit_survey/Unknown_n_d_ARE_No_L_E10/` is Coleman, Feingold & Stempin,
   NACA ARR L5E10, complete, with the elliptic-integral solution. **Analytic, no CFD, and
   it addresses the actual deficiency.** Do this before any 3-D CFD.
2. **Tabulate `k_x, k_y` against (μ, χ) from a wake solver** if step 1 proves insufficient.
   A 2-D table. Generate it with **DUST** (Politecnico di Milano, free, mid-fidelity vortex
   particle, built for rotorcraft) rather than RANS — minutes per case on a laptop, and it
   resolves the wake geometry that sets the gradient.

### Solver convergence, while here

`lit_survey/Dodic_2023_.../` establishes that the Glauert inflow equation can be solved in
**one iteration** with the Sharma–Sharma, KT Diff or E5 methods given a λ_md initial guess.
The measured solver currently runs 8.8 iterations mean, 32 worst, and **3.9 % of
manoeuvring steps fail to converge**. Worth adopting for whichever inflow path survives.

---

## 5. Stage 3 — 3-D rotating CFD (needs hardware this machine does not have)

What 2-D section CFD cannot give: rotational augmentation (Himmelskamp effect — delayed
stall on rotating blades, significant at this blade's low aspect ratio), 3-D root and tip
flow, and the reverse-flow region in edgewise flight.

**Honest hardware assessment:**

| case | cells | feasible here? |
|---|---|---|
| 2-D section | 0.15–0.3 M | yes, minutes |
| 3-D periodic sector, hover, 1 blade | 3–10 M | marginal — 12–48 h/case, a handful only |
| 3-D full-annulus unsteady edgewise | 10–30 M, multi-revolution | **no — needs a cluster** |

Edgewise flight destroys periodicity, so the expensive case is exactly the one that matters
for §4. Two routes:

- **Institutional HPC.** Worth checking what the department provides before scoping this.
- **Avoid it.** Use a handful of feasible 3-D *hover* cases to calibrate a rotational-
  augmentation correction (Snel, Du–Selig, or Chaviaropoulos–Hansen — all are algebraic
  corrections to the 2-D polar as a function of `c/r`), and leave edgewise inflow to the
  mid-fidelity wake of §4. This gets most of the fidelity for a small fraction of the cost
  and keeps everything inside the existing table architecture.

---

## 6. Configuration surface

Requirement: solver and wing-design parameters editable from configuration.

| file | owns |
|---|---|
| `examples/utils/wing_params.py` | wing design — already the single edit point; unchanged |
| `cfd/config/solver.yaml` | mesh, turbulence + transition model, CFL, convergence, sweep ranges |
| `cfd/config/cases.yaml` | which sections to run, validation cases |
| `cfd/tables/*.npz` | generated artefacts, each carrying provenance |

The existing `RotatingWingConfig` already validates every wing parameter and the physics
suite already asserts `wing_params.py` has not drifted from the schema. Extending it with
`polar_source` and a table path keeps one editable file for design and adds one for the
solver, which is the requested split.

---

## 7. Validation that must accompany any of this

The current 158-test suite is the reason the model's errors are known rather than
suspected. Each stage needs its own acceptance test before it is trusted:

| stage | test |
|---|---|
| 1 | CFD reproduces E387 measured data (§2.4) before GOE 210 is run |
| 1 | CFD table and XFOIL agree within 10 % in the operating band — a large disagreement means the CFD setup is wrong, not that XFOIL was |
| 3.1 | gyroscopic torque matches `ω × h` analytically; zero when `rot_dir` cancels |
| 3.2 | dynamic inflow reproduces the steady solution in the limit; step response has τ ≈ 67 ms |
| 4 | Coleman inflow reduces to uniform at μ = 0; thrust asymmetry grows with μ |
| all | existing 158 tests still pass; `--no-wing` still bit-identical |

---

## 8. Summary of recommended order

Ranked by fidelity gained per unit of effort, which is not the order the work was first
proposed in.

| # | item | cost | fixes | CFD? |
|---|---|---|---|---|
| 1 | **Measure the finished rod's torsional stiffness** on a bench | an afternoon, a load cell | the dominant unknown (§0.3) | no |
| 2 | **Aeroelastic twist model** — `θ(r) = θ_design(r) + ∫T/GJ`, using the report's own section-moment formula | ~40 lines, 1 day | a 28–77 % thrust error | no |
| 3 | **Propeller gyroscopic momentum** | ~20 lines, hours | a moment 2.3× the wing's own | no |
| 4 | **Pitt–Peters dynamic inflow** (paper in `lit_survey/`) | ~100 lines, 1–2 days | τ = 67 ms lag; also makes cost constant-time | no |
| 5 | **Coleman linear inflow** (paper in `lit_survey/`) | ~50 lines, 1 day | edgewise validity above μ = 0.4 | no |
| 6 | **Rotational augmentation correction** (Snel / Du–Selig, algebraic in c/r) | ~30 lines, 1 day | delayed stall on a stubby AR 1.3 blade | no |
| 7 | **One-iteration inflow solve** (Dodic, in `lit_survey/`) | half a day | 3.9 % non-convergence, worst-case spike | no |
| 8 | **2-D CFD section tables**, E387-validated | 1–2 weeks + 1–2 days compute | post-stall branch; section data to ~3 % | yes |
| 9 | Mid-fidelity free wake (DUST) for inflow coefficients | 1–2 weeks | edgewise inflow field | mid-fi |
| 10 | 3-D rotating RANS | needs a cluster | rotational augmentation from first principles | yes |

**Items 1–7 require no CFD, no new software, and about two weeks.** Between them they
address every error larger than 10 % that has been measured in this model. Items 8–10 are
worth doing afterwards and are specified above, but they refine what is already the most
accurate part of the system.

### Where CFD genuinely earns its place

Not in attached flow — XFOIL already covers that to a few percent. Two places:

1. **Post-stall.** During manoeuvre **100 % of blade elements go past stall**, and the
   post-stall branch is Viterna extrapolation: structurally verified against the source
   paper, never validated against anything. CFD is the only practical way to replace a
   guess with a computation here.
2. **Rotational augmentation.** This blade has an aspect ratio of **1.3** — extremely
   stubby, which is precisely where the Himmelskamp effect (centrifugal pumping delaying
   separation) is largest and where 2-D section data is least applicable. An algebraic
   correction (item 6) is the cheap approximation; a handful of 3-D hover cases is how the
   correction gets calibrated for this specific blade.

Both are real, and both become worth the effort **once items 1–7 are done** — not before,
because until then they are second-order against errors that are 2–25× larger.
