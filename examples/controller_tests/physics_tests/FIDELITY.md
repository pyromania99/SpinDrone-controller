# Rotating-wing model: accuracy and real-time budget

Measured, not estimated. Every number here comes from a run on this machine against this
code; the scripts are reproducible from the method notes at the end.

**Short answer.** Accurate to about 1 % in steady flight and 3–10 % in aggressive
manoeuvre, dominated by one approximation (instantaneous inflow) that is *not* valid for
a rolling vehicle. Real-time for one vehicle with 3× margin; **not** real-time for six.
A lookup table does **not** help — the cost is solver iterations, not polar arithmetic.

---

## 1. What the model sees

The wing consumes exactly two state vectors, both body-frame FLU, matching the stock
`Multirotor` convention:

| Input | Source |
|---|---|
| `linear_body_velocity_m_s` | `State.linear_body_velocity` |
| `angular_velocity_body_rad_s` | `State.angular_velocity` — full `[p, q, r]` |

Element velocity is `v_element = v_body + ω × r_element`, the exact rigid-body relation
with no small-angle or axial-flow assumption. **Roll and pitch rates enter the kinematics
exactly.** Attitude and position do not enter at all, which is correct: the panels are
body-fixed and the air is still in the world frame, so only relative velocity matters.

The honest limitation is not in the kinematics. It is that the *inflow* is solved as if
the vehicle had been in its current state forever.

---

## 2. Accuracy

### 2.1 Against the design report (steady, design point)

| Quantity | Report | Model | Error |
|---|---|---|---|
| Thrust @ 20 rad/s | 3.740 N | 3.701 N | −1.0 % |
| Aero power | 7.31 W | 7.00 W | −4.2 % |
| Figure of merit | 0.722 | 0.741 | +2.6 % |
| Station α / c_l / v_i | — | — | 0.07° / 0.01 / 0.02 m/s |

Only the section polar was fitted. Thrust, power, FM and the radial inflow distribution
are integrated outputs and were not tuned to anything.

### 2.2 Spanwise discretisation, under manoeuvre

Against a 384-station reference, on a trajectory of spin −20 ± 4 rad/s, roll/pitch 8 rad/s,
6 m/s edgewise:

| Stations | mean error | worst error |
|---|---|---|
| 12 | 0.008 N | 3.00 % |
| **24 (default)** | **0.004 N** | **1.07 %** |
| 48 | 0.002 N | 0.73 % |
| 96 | 0.001 N | 0.45 % |

24 stations is a sound default. Going to 48 costs 6 % more time for 0.3 % accuracy.

### 2.3 Inflow convergence — the real accuracy limit

The inflow is a damped fixed-point iteration (relaxation 0.5, cap 32, tolerance 1e-4 m/s).
On the manoeuvring trajectory above, **3.9 % of steps fail to converge**, worst residual
0.105 m/s against an induced velocity of ~1.6 m/s — a 6 % error on inflow in the worst
steps.

Error against a hard-converged reference, as a function of the iteration cap:

| cap | mean error | **worst error** | worst-case time |
|---|---|---|---|
| 1 | 0.5 % | 19.2 % | 0.9 ms |
| 4 | 0.15 % | 10.4 % | 2.0 ms |
| 8 | 0.08 % | 7.6 % | 4.7 ms |
| 32 (default) | 0.02 % | **3.3 %** | 10.9 ms |

Two things to read here. The *mean* error is excellent everywhere — the model is accurate
almost all the time. The *worst case* is poor even at 32 iterations, because the solver
converges slowly in hard states, and those are exactly the manoeuvring transients.

Relaxation sweep confirms the solver, not the physics, is the limit: 0.9 is better than
the 0.5 default (7.9 % vs 10.4 % worst at cap 4) but **1.0 diverges** (26 %). The
iteration is only marginally stable. A Newton or Anderson-accelerated solve would fix
accuracy and speed together; this is the single highest-value improvement available.

### 2.4 Post-stall

During the aggressive manoeuvre, **100 % of blade elements go past stall at some point**.
Post-stall is Viterna–Corrigan extrapolation: structurally verified term-by-term against
the source paper, with the drag ceiling now derived from the blade's aspect ratio
(eq. 13, c_d,max = 1.133 not 1.8) — but never validated against data. Treat any
deeply-stalled result as indicative only.

---

## 3. Spinning **and** rolling — the case that matters

The two-bladed wing is **inertially and aerodynamically asymmetric**, and this interacts
with the spin in a way worth stating explicitly.

In the **body** frame the blades always lie along ±x. So:

| | damping |
|---|---|
| body-frame roll (about x) | **exactly 0** |
| body-frame pitch (about y) | −0.091 N·m/(rad/s) |

A blade along x has no moment arm about x. This is correct, and the test suite asserts it
as a deliberate null direction.

But the question is about the **world** frame, and there the spin rescues it. With the body
spinning at −18 rad/s under a steady 4 rad/s world-frame roll rate, averaged over one
revolution:

| | value |
|---|---|
| world-frame roll damping | **−0.052 N·m/(rad/s)** |
| world-frame pitch damping | ~0 (mean), ±0.41 N·m oscillating |
| thrust ripple | **±32 %** at 2/rev (5.7 Hz) |

The spin averages the body-frame roll and pitch damping, so the wing **does** damp
world-frame roll — at about half the body-frame pitch value. The cost is a large 2/rev
ripple in thrust and moment.

**Four blades largely remove this:**

| | 2 blades | 4 blades |
|---|---|---|
| thrust ripple | ±32.4 % | **±3.9 %** |
| world roll damping | −0.052 | **−0.179** N·m/(rad/s) |

If roll behaviour under spin matters to the controller study, four panels are the better
configuration — `--wing-blades 4`, no code change. The 2/rev ripple is resolved by the
physics step (44 steps per ripple cycle at 250 Hz), so it is represented, not aliased.

### The approximation that actually limits this case

The model solves inflow to **equilibrium every step**. Real inflow has mass:

```
apparent mass of the air column  m_a = (8/3)ρR³  = 0.400 kg
inflow time constant             τ  = m_a/(4ρAv_i) = 67 ms
```

Against the competing timescales:

| | timescale | τ/T |
|---|---|---|
| roll at 8 rad/s | 125 ms | **0.54** |
| controller ωn = 12 rad/s | 83 ms | **0.81** |
| spin period at 18 rad/s | 349 ms | 0.19 |
| physics step | 4 ms | — |

**τ is comparable to both the roll timescale and the controller's own bandwidth.** The
quasi-steady assumption is not justified for a manoeuvring vehicle, and it is not
conservative: it makes the wing respond to rate changes instantly, overstating aerodynamic
damping and phase margin exactly where a controller study would care.

For high fidelity this is the first thing to add: a Pitt–Peters first-order dynamic inflow
state (`dν/dt = (ν_steady − ν)/τ`, three states) in place of the instantaneous solve. It
would *also* make the model cheaper, since the per-step inflow solve becomes one update
rather than an iteration to convergence.

---

## 3b. Sideways (edgewise) flight — advancing vs retreating blade

The asymmetry **is** produced: the kinematics are exact, so the advancing blade sees more
dynamic pressure than the retreating one and the lift difference appears as a pitching
moment. At 18 rad/s spin (tip speed 9.0 m/s, root speed 3.6 m/s):

| V sideways | thrust | My | what the two blades see |
|---|---|---|---|
| 0 | 3.00 N | 0 | symmetric |
| 2 m/s | 3.46 N | 1.07 N·m | both attached |
| 4 m/s | 4.90 N | 1.82 N·m | retreating root reversing |
| 6 m/s | 6.85 N | 2.53 N·m | retreating blade **100 % stalled** |

At 6 m/s the split between the two panels is extreme:

| blade | α range | Re | stalled |
|---|---|---|---|
| advancing (180°) | +4.7 to +8.8° | 86k–235k | 0 % |
| retreating (0°) | **−105.6 to −28.4°** | 24k–71k | **100 %** |

Because the panels are rigidly bonded — no flapping hinge, no cyclic — this asymmetry
goes straight into the airframe as a real moment the controller must fight. In the world
frame the spin carries it round, so it is a **1/rev disturbance**, which is physically
right and is exactly the kind of thing the controller study should see.

**But the validity limit arrives early.** The advance ratio is μ = V/(ΩR):

| V | μ | reverse flow | regime |
|---|---|---|---|
| 1 m/s | 0.11 | none | BEMT solid |
| 2 m/s | 0.22 | none | degrading |
| **3.6 m/s** | **0.40** | **starts at the root** | **helicopter validity limit** |
| 6 m/s | 0.67 | r < 0.33 m | BEMT invalid |
| 9 m/s | 1.00 | whole blade, part of each rev | BEMT invalid |

Reverse flow begins as soon as the sideways speed exceeds the **root** speed Ωr_root =
3.6 m/s — a low bar, because the wing's root is far out at 0.2 m and the spin is slow
compared with a real rotor.

Two things break above μ ≈ 0.3–0.4:

1. **BEMT's momentum balance assumes axisymmetric flow through each annulus.** In edgewise
   flight the inflow has a strong fore-aft gradient that the per-annulus balance cannot
   represent. A Glauert/Coleman linear inflow (ν = ν₀(1 + k_x r/R cos ψ)) is the standard
   first fix.
2. **The reversed-flow region is handled by Viterna extrapolation**, which is an
   engineering fit, not data.

As a crude bound on the modelling uncertainty, the three inflow models disagree by
**21–31 % in thrust at every speed**, including hover. That spread is the honest error bar
on edgewise results, and it does not shrink with more stations or tighter tolerance —
it is model-form uncertainty, not numerical error.

**Practical limit: trust edgewise results below about 2 m/s; treat 2–4 m/s as indicative;
do not trust above 4 m/s** without a better inflow model.

---

## 3c. Gyroscopic effects

Two distinct effects. One is modelled correctly, one is missing and is large.

### Body gyroscopic term ω × (Iω) — **modelled**

PhysX integrates it: `enableGyroscopicForces` is **True** (verified on every rigid body in
`iris.usd`). Because the wing's mass and inertia are added to the body prim at spawn, the
wing's own inertial asymmetry participates correctly:

```
Ixx 0.0299   Iyy 0.0438   Izz 0.0707  kg.m^2   (body + 2-blade wing)
```

Ixx ≠ Iyy, so a spinning vehicle genuinely experiences the coupling. Nothing to fix.

### Propeller spin angular momentum — **NOT modelled, and it is bigger than the wing**

The rotor joints are driven kinematically at a **hardcoded 100 rad/s** by
`handle_propeller_visual()` — the comment says "for animation purposes only". The real
hover speed is **684 rad/s** (from T = kω² with the asset's own k).

| | |
|---|---|
| rotor inertia about spin axis | 7.65e-5 kg·m² (read from PhysX at runtime) |
| hover rotor speed | 684 rad/s (6536 rpm) |
| **real total propeller momentum** | **0.209 kg·m²/s** |
| what the kinematic joints carry | 0.031 (15 %) |
| body momentum at 18 rad/s | 1.273 kg·m²/s |

**The propellers carry 16 % of the whole vehicle's angular momentum, and it is missing.**

This matters *here specifically* and nowhere else in Pegasus. A stock quadrotor sets
`rot_dir = [-1,-1,+1,+1]`, so the four propeller momenta cancel to essentially zero and
ignoring them is harmless. This fork sets `[-1,-1,-1,-1]` — that is the whole premise of
the spinning drone — so **they add**.

The resulting gyroscopic moment `M = ω_body × h_prop`, against the wing's own moment at
the same body rate:

| body rate | missing prop gyro | wing moment | ratio |
|---|---|---|---|
| 1 rad/s | 0.209 N·m | 0.322 N·m | 0.7× |
| 2 rad/s | 0.419 N·m | 0.311 N·m | 1.3× |
| 4 rad/s | 0.837 N·m | 0.364 N·m | **2.3×** |
| 8 rad/s | 1.675 N·m | 0.725 N·m | **2.3×** |

Above about 1.5 rad/s of roll or pitch rate, **the unmodelled propeller gyroscopic moment
exceeds the entire aerodynamic contribution of the wing being studied.** Any conclusion
about attitude dynamics under manoeuvre is therefore dominated by a term that is not in
the simulation.

**Fix, in order of effort.** This is a few lines, not a research project:

1. Track rotor speed ωᵢ (the thrust curve already computes it) and apply
   `M = −ω_body × h` with `h = Σ Iᵢωᵢ·rot_dirᵢ ẑ` as one extra `apply_torque` on `/body`.
   Same pattern the wing already uses. This captures ~all of it.
2. Optionally add first-order motor spin-up lag, which the kinematic joints also omit.

Note this is **stock Pegasus physics**, outside the "don't touch the quadcopter" boundary
for the wing work — it is listed here because the fidelity question cannot be answered
honestly without it, not because it has been changed.

---

## 3d. Independent validation of the section polar (XFOIL)

The fitted polar was calibrated to the design report's station table, which was itself
produced with NeuralFoil — a model fitted to a model. `lit_survey/` now holds XFOIL polars
for the same section from Airfoil Tools (Re 5e4–1e6, Ncrit 9), fetched directly. XFOIL is
an independent code with different physics, so this is a real check.

**The coordinates were verified first**: the downloaded `goe210_coords.dat` matches the
repository's own `foil_out/database/goe210.dat` exactly, so the polars belong to the
section the blade was designed around.

| Re | XFOIL max L/D | model | XFOIL best α | model best α |
|---|---|---|---|---|
| 50k | 40.7 | 44.7 (+10 %) | 5.50° | 5.62° |
| 100k | 62.0 | 62.5 (**+1 %**) | 4.75° | 4.62° |
| 200k | 82.3 | 88.6 (+8 %) | 3.75° | 3.88° |

Section values at the operating points agree to **4–5 % in Cl and 1–5 % in Cd**, and the
**best lift-to-drag angle to within 0.13° across the whole operating range**. That last
number matters disproportionately: it is the quantity bug 4 had wrong by 2–5°, and the fix
was reasoned from the drag bucket's Reynolds behaviour rather than fitted to data. XFOIL
confirms it independently.

Integrated at the design point, three independent routes converge:

| | thrust | power | FM | settled spin |
|---|---|---|---|---|
| fitted polar (ships) | 3.701 N | 7.00 W | 0.741 | 18.44 rad/s |
| XFOIL polar | 3.591 N | 6.73 W | 0.736 | 18.61 rad/s |
| design report (NeuralFoil) | 3.740 N | 7.31 W | 0.722 | — |

Thrust within 3 %, FM within 2.6 %, settled spin within 1 %.

**What this validation does not cover.**

- XFOIL is computation, not measurement. **No wind-tunnel data for GOE 210 could be
  found** — searches of the Göttingen *Ergebnisse* volumes and the NACA/NASA archives
  returned nothing for this section. Two codes agreeing is weaker than data.
- Ncrit = 9 assumes a clean tunnel. A real moulded blade is rougher; Ncrit 5–7 would be
  more representative and would lower L/D.
- **Above Re 2e5 the fitted polar is badly wrong** (+36 % at 5e5, +78 % at 1e6) because it
  was calibrated only over the blade's range. Harmless here — the blade peaks near Re 235k
  — but it must not be reused outside Re 5e4–2e5.
- XFOIL stalls at 10.2–12.8° against the model's 9.0°, with Cl_max 1.32–1.51. The model is
  the conservative one. This matters only past stall, i.e. in manoeuvre.

### A note on a discarded file

A polar file, `GOE_210_Polars_Re_1e5_to_1e6.csv`, briefly sat in `lit_survey/` before it
was identified as language-model output rather than data, and **has since been deleted**.
XFOIL settled it: that file reported max L/D of 15.2 at Re 1e5 where XFOIL gives 62.0 — a
factor of four — and put the optimum at 6.5° against 4.75°. It was never used by any code
path. It is recorded here only so the sensitivity figures below, which were computed with
it before its origin was known, are not mistaken for a validation.

### Residual value: a drag sensitivity bound

The numbers it produced remain useful for one thing. Because that polar differed from the
fitted one mostly in drag (4.5× on L/D), running the model on it bounds how much section
drag actually matters:

| | fitted polar | arbitrary high-drag polar (perturbation only) |
|---|---|---|
| Thrust @ 20 rad/s | 3.701 N | 3.424 N |
| Power | 7.00 W | 7.55 W |
| Figure of merit | 0.741 | 0.611 |
| settled spin | 18.4 rad/s | 18.1 rad/s |
| lift at equilibrium | 19.6 % of weight | 17.5 % of weight |

**A factor-of-four error in section drag moves thrust, settled spin and lift fraction by
single-digit percentages.** The inflow angle is about 13° at this operating point, so the
induced term `dL·sinφ` is roughly seventeen times the profile term `dD·cosφ`: torque, and
hence the spin equilibrium, is induced-dominated. **Figure of merit is the exception**
(0.741 vs 0.611) because it is a ratio that reports profile drag directly.

---

## 4. Real-time budget

Physics runs at 250 Hz → **4000 µs per step**, shared with PhysX and everything else.

Measured cost model (2 blades, 24 stations):

```
cost ≈ 266 µs  +  215 µs × n_inflow_iterations
```

| Condition | mean | worst | worst as % of step |
|---|---|---|---|
| steady spin, hover | 487 µs | 3179 µs | 79 % |
| spin + roll 5 rad/s | 511 µs | 1459 µs | 36 % |
| spin-up transient | 603 µs | 3079 µs | 77 % |
| **aggressive manoeuvre** | **2177 µs** | **11572 µs** | **289 %** |
| discontinuous state | 3808 µs | 12740 µs | 319 % |

In hover the warm start holds the iteration count at 1 and the model is comfortable. In
manoeuvre the iteration count rises to a mean of 8.8 and hits the cap of 32, and the
worst case is nearly 3× the step budget.

### In the actual simulator

Marginal cost per extra vehicle, from timed 10 s runs:

| | per step per vehicle |
|---|---|
| no wing | 126 µs |
| **wing** | **1302 µs** |

| Vehicles | wing cost per step | verdict |
|---|---|---|
| 1 | 1.3 ms / 4 ms | **real-time, 3× margin** |
| 2 | 2.6 ms / 4 ms | marginal |
| **6 (README default)** | **7.8 ms / 4 ms** | **~2× slower than real time** |

Six-drone sweeps still *run* — Isaac does not require wall-clock real time — but they no
longer run in real time, and a hardware-in-the-loop or operator-in-the-loop setup would
miss its deadline.

### Do we need LUTs? No.

The intuition is reasonable — 68 % of runtime is inside the airfoil polar, with
`_viterna_evaluate` the single hottest function — so tabulating it looks like the obvious
fix. It was measured, and it does not work:

| Polar | mean | worst | thrust error vs analytic |
|---|---|---|---|
| analytic (current) | 2135 µs | 7443 µs | — |
| LUT 181×6 (17 KiB) | 2222 µs | 8398 µs | 15.1 % worst |
| LUT 361×8 (45 KiB) | 2210 µs | 10143 µs | 2.9 % worst |
| LUT 721×12 (135 KiB) | 2253 µs | 10228 µs | 2.0 % worst |

**Every LUT is slightly slower**, and costs accuracy. The reason: the arrays are tiny
(2 blades × 24 stations = 48 elements), so the cost is NumPy per-call dispatch overhead,
not arithmetic. Bilinear interpolation needs its own searchsorted and gather, which costs
about what the closed-form evaluation costs. Replacing arithmetic with memory lookup only
pays when the arithmetic dominates, and here it does not.

### What would actually work, in order of value

1. **Dynamic inflow (Pitt–Peters).** Removes the iteration entirely — one state update per
   step, so cost collapses to the ~266 µs fixed part and becomes *constant-time*, which is
   what hard real time needs. Simultaneously fixes the largest fidelity gap in §3. Both
   problems, one change.
2. **A better steady solver** (Newton / Anderson) if the quasi-steady model is kept.
   Targets 2–3 iterations instead of 8.8 mean / 32 worst.
3. **Cap iterations for determinism.** `--wing-inflow glauert` already gives a
   non-iterative uniform inflow at 448 µs, constant time, at the cost of radial detail.
   Available today with no code change.
4. **Reduce per-call overhead.** `np.clip` dispatch alone is ~15 % of runtime on this
   NumPy version. Fusing the section-state computation and avoiding `np.clip` would cut
   the 266 µs floor without touching the physics.

Note that 1 and 3 are the only options that give a *bounded* worst case. Items 2 and 4
lower the average but leave the tail.

---

## 5. Verdict against "high fidelity"

| Aspect | State |
|---|---|
| Rigid-body kinematics | Exact, all six rates |
| Steady aerodynamics at design point | ~1 %, validated independently |
| Spanwise discretisation | 1 % at 24 stations |
| Section polar | **Validated against XFOIL** to 4–5 % Cl, 1–5 % Cd, 0.13° on best-L/D α over Re 5e4–2e5 (§3d). No measured data exists for this section. |
| Inflow dynamics | **Available** — `--wing-inflow dynamic`, τ ≈ 77 ms from apparent mass; BEMT/Glauert remain quasi-steady |
| Edgewise flight | Asymmetry exact; inflow invalid above μ≈0.4 (3.6 m/s) |
| Body gyroscopic ω×(Iω) | Modelled by PhysX, wing inertia included |
| Propeller gyroscopic momentum | **Modelled** — `rotor_gyroscopics.py`, on by default |
| Unsteady aerodynamics / dynamic stall | Absent |
| Post-stall | Structurally correct, unvalidated |
| Blade flexibility | Absent (report measured 10–15° aeroelastic twist) |
| Spanwise flow | Absent (blade element independence) |

It is a good quasi-steady BEMT model, which is the right class of model for controller
work, and it is honest about the design point. It is **not yet** high fidelity for a
manoeuvring spinning vehicle, and the gap is specific and fixable: dynamic inflow first,
then a measured polar.

---

## Method

All timings on this machine (Ryzen 7 4800H, Python 3.10, single thread), physics step
1/250 s from `params.py`. Benchmarks drive `RotatingWing.compute_loads` directly, so they
exclude PhysX. The in-simulator figures are marginal costs from timed 10 s headless
`eigen_stability_sweep.py` runs at 1 and 6 vehicles, which cancels Isaac's startup.

The manoeuvring trajectory used throughout is

```python
v  = [6cos(2t), 6sin(2t), 0.5sin(3t)]              # m/s, body frame
ω  = [8sin(4t), 8cos(4t), -20 + 4sin(t)]           # rad/s, body frame
```

chosen to keep the state continuous (so the warm start behaves as it does in simulation)
while exercising spin, roll, pitch and edgewise flow together.
