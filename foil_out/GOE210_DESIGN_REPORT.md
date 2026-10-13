# Spinning-wing blade design: GOE210 version

Full record of the analysis done in this working session: methods, data, corrections, and the chosen design, with enough detail to re-check every number and rerun every step.

**Contents**

1. [Decision and design sheet](#1-decision-and-design-sheet)
2. [Context: the paper and the new blade](#2-context-the-paper-and-the-new-blade)
3. [Tools and environment](#3-tools-and-environment)
4. [Methods](#4-methods)
5. [Results: 2D airfoil screening](#5-results-2d-airfoil-screening)
6. [Results: XFoil verification](#6-results-xfoil-verification)
7. [Results: rotor (BEM) designs](#7-results-rotor-bem-designs)
8. [GOE210: detailed design data](#8-goe210-detailed-design-data)
9. [Thickness and the carbon rod](#9-thickness-and-the-carbon-rod)
10. [Torsion analysis](#10-torsion-analysis)
11. [Checking the paper's wing](#11-checking-the-papers-wing)
12. [What to expect on hardware](#12-what-to-expect-on-hardware)
13. [Concepts explained](#13-concepts-explained)
14. [Corrections log](#14-corrections-log)
15. [Assumptions, limits, and open questions](#15-assumptions-limits-and-open-questions)
16. [File index and how to rerun](#16-file-index-and-how-to-rerun)

---

## 1. Decision and design sheet

**Chosen foil: GOE210.** Reasons, all from the analysis below:

- At best CL/CD at every station, all nine screened foils give nearly the same figure of merit (0.72–0.73) and lift per watt (50–54 g/W). So the foil is chosen on structure, not on aerodynamics (section 7.3).
- GOE210 is about 6.6% thick and holds that thickness from about 15% to 40% chord. At the full 0.30 m chord that is 19.7 mm, wide enough for the carbon rod at the chordwise position where the aerodynamic torque is lowest (section 9, section 10).
- Its power cost is the same as AS6091 in the thrust-targeted blade runs (3.92 W against 3.89 W per blade at 2 N per blade).
- It was verified in XFoil: 11 of 12 cases within 5% of NeuralFoil, and one spurious XFoil point (section 6).

**Blade layout (hover, 20 rad/s, r = 0.2–0.5 m, 2 blades, every station at its best CL/CD):**

| Quantity | Value |
|---|---|
| Aero lift, 2 blades | 3.74 N (382 g) |
| Aero power, 2 blades | 7.31 W (3.66 W per blade) |
| Figure of merit | 0.72 |
| Lift per watt | 52 g/W aero |
| Chord | 0.30 m from r = 0.20 to 0.30 m, tapering to 0.08 m at the tip |
| Pitch needed | about 25° at the root, about 17° at r = 0.45 m |

Build schedule (pitch is the angle of the chord line to the rotor plane; ignore the model's tip spike and hold about 16.5° out to the tip):

| r (m) | Chord (mm) | Pitch (deg) | Rod ceiling at 37% chord* |
|---|---|---|---|
| 0.200 | 300 | 24.7 | 10 mm |
| 0.250 | 300 | 22.6 | 10 mm |
| 0.300 | 297 | 20.8 | 10 mm |
| 0.350 | 280 | 19.4 | 10 mm |
| 0.400 | 197 | 17.6 | 10 mm (limit) |
| 0.425 | 153 | 16.3 | 8 mm (limit) |
| 0.450 | 124 | 16.3 | 5 mm |
| 0.475 | 99 | 16.6 | 5 mm (limit) |
| 0.500 | 80 | 16.5 (hold) | below 5 mm |

*Rod diameter that still leaves 1.5 mm of skin or fairing at 37% chord. Section 9.

**Spin-rate robustness (section 8.3):** this same built geometry works from 15 to 20 rad/s. Lift goes as Ω², about 211 g at 15 rad/s and 377 g at 20 rad/s (aero, 2 blades), while α stays at 4.4–5.5° along the loaded span, FM stays 0.73–0.74, and the minimum stall margin is 4.1° at 15 rad/s. Re-optimising for the whole band gave no worthwhile improvement.

**Rod and structure recommendations (section 10 explains why):**

1. Put the rod at about 35–40% chord. This is where the lift and the section moment cancel at design CL, so the torque on the rod is almost zero.
2. Taper the rod with the thickness: about 10 mm to r = 0.40 m, 8 mm to 0.425 m, 5 mm to 0.475 m.
3. Stiffen any bare length of rod between the clamp and the wing with a larger or ±45° carbon tube. Twist adds up over the whole length.
4. Lock every section to the rod (square holes or bonded), so ribs cannot rotate.
5. Measure the real torsional stiffness of the finished rod (section 10.5) before trusting the model.

![GOE210 design](results/plots/8_goe210_design.png)
![GOE210 profile](results/plots/9_goe210_profile.png)

---

## 2. Context: the paper and the new blade

### 2.1 The paper

*Spinning Quadrotor: Hover Thrust Augmentation with Passive Lifting Surfaces* (Parkala and Kandath, ICUAS draft v2). Main claims relevant here:

- Quadrotor with radially mounted passive wings; motors canted 10° so the whole vehicle spins steadily.
- Wings at r = 0.40–0.65 m. Spin rate in tests about 8–10 rad/s. Stable spinning hover was demonstrated only up to about 8–10 rad/s.
- Airfoil screening for Re 40–90k gave a custom "Franky" airfoil (26% blend of S1223 and SD7037). Table I used α = 10° at every radius and took chords of 0.10 to 0.29 m.
- Lift of the 4 wings: BEMT simulation 5.27 N, CFD (Fluent MRF, 10 rad/s) 4.50 N, hardware 2.10 N (214 g).
- Hardware: 4-wing 960 g vehicle, 22.3% of weight carried by wings; 2-wing 658 g vehicle, 22.3%, per-wing 73.4 g against 53.5 g per wing for four wings.
- Idealised power budget: 52.3 W for the spinning vehicle against 122 W for a motor-only baseline (57% reduction), with motor efficiency 0.65 assumed throughout.

### 2.2 The new request in this session

The wing is to start at r = 0.2 m and end at r = 0.5 m at 20 rad/s, giving 4–10 m/s, with a maximum chord of 0.30 m. The first request was "check optimal values of the airfoils over a 5–10 m/s range for a chord ≤ 0.3 m". It then grew into designing a rotor-style blade, estimating hardware thrust, planning the carbon rod, and estimating torsion.

Vehicle information given in the chat: the drone is about 500 g without the wings, and the carbon rods are 10 × 10 mm (first written as cm, then corrected to mm) and 5 × 5 mm. On the 5 mm rod the wing twisted about 10–15° in a real test. The torque was applied along the tip end of the rod, with 10 to 40 cm of the rod from the root clamp under torsion.

---

## 3. Tools and environment

| Item | Detail |
|---|---|
| Working folder | `C:\Users\anike\Desktop\RESEARCH\foils` |
| Python environment | `v\` inside the folder (Python 3.12). A system-wide `pip install neuralfoil` failed with a Windows long-path error, so a venv with a short path was used. |
| Packages | neuralfoil (with aerosandbox), numpy, scipy, pandas, matplotlib, tabulate |
| XFoil | XFoil 6.99, official MIT Windows executable, `tools\xf\xfoil.exe` |
| Existing material | `database\` (1,665 UIUC-style airfoil files), `foils\as6091-3.dat`, `temp\` (be50, be50sm, goe210, goe346, goe57, hq308, sa7024), `s1010/s1012/s2048/s2050/s3021.dat` in the root |
| Fixed constants | ρ = 1.2 kg/m³, kinematic viscosity ν = 1.5e-5 m²/s |

Not used: Flow5/XFLR5 (GUI only, no scripting), OpenFOAM (discussed only, section 15).

---

## 4. Methods

### 4.1 2D sweep (`chord_velocity_opt.py`)

- 15 candidate airfoils: AS6091/2/3, BE50, BE50SM, GOE210, GOE346, GOE57, HQ308, SA7024, S1010, S1012, S2048, S2050, S3021.
- For each foil: V = 5, 6, 7, 8, 9, 10 m/s; chord = 0.05, 0.10, 0.15, 0.20, 0.25, 0.30 m; NCrit = 9 (calm air) and 5 (disturbed air).
- Reynolds number Re = V·c / ν (range 17k to 200k).
- Polars from NeuralFoil (xlarge model) at α = −4° to 15° in 0.25° steps. Points with analysis confidence below 0.5 were excluded.
- Result per case: maximum CL/CD, and the α, CL and CD where it occurs.
- Table values labelled "avg NCrit" average the two NCrit cases.

### 4.2 XFoil verification (`verify_xfoil.py`)

- XFoil 6.99 driven by script (stdin commands), with graphics off.
- Cases: AS6091, AS6092, GOE210 × V = 5, 7, 10 m/s × chord = 0.15, 0.30 m × NCrit = 9, 5 (36 cases).
- Setup per case: viscous mode at the Re, Mach 0, N_crit set, 300 iterations, α sweep −2° to 12° in 0.25° steps, then the maximum CL/CD found and compared with NeuralFoil.
- Output: `results/xfoil_verify.csv`.

### 4.3 Blade-element momentum model (`blade_design.py`)

The blade is cut into 25–31 strips along r = 0.2–0.5 m. At each strip:

1. Tangential speed U = Ωr; induced velocity v (unknown).
2. Inflow angle φ = atan(v / U). Relative speed V = √(U² + v²). Angle of attack α = θ − φ, where θ is the blade pitch.
3. Re = V·c / ν. CL and CD come from interpolation tables built from NeuralFoil (α −6° to 18° in 0.5° steps, Re 8k to 400k, log spaced, NCrit 9).
4. Lift per metre: ½ρV²·c·CL. Drag per metre: ½ρV²·c·CD.
5. Thrust per metre: dT/dr = dL·cosφ − dD·sinφ. Torque per metre: dQ/dr = r·(dL·sinφ + dD·cosφ). Power = Ω × integral of dQ.
6. Momentum theory closes the loop: B·dT/dr = 4π·ρ·r·F·v², with B blades and a Prandtl tip-loss factor F (clamped to 0.05–1). For each strip v is found by bisection so the blade-element thrust matches the momentum thrust.

Assumptions: hover only (no axial or forward flight), rigid blade, steady, 2D polars applied to 3D strips, no hub, body, wing-to-wing or propeller interference.

Set-up: Ω = 20 rad/s, R0 = 0.2 m, R1 = 0.5 m, B = 2, chord limit 0.30 m.

### 4.4 Optimisation: two versions

**(a) Thrust-targeted (`blade_design.py`).** Unknowns: chord at 5 nodes (30–300 mm) and pitch at 5 nodes (−5° to 40°), joined by smooth Pchip curves. Constraint: thrust per blade equals 1 N or 2 N. Goal: minimum power. Optimiser: SLSQP from two starting shapes. Compared against fixed chord laws (constant 0.30 m, linear taper 0.30 → 0.12 m, 1/r with 0.30 m at the root), each with only its pitch optimised.

**(b) Best CL/CD at every station (`design_max_ld.py`).** At each Re, the α with the highest CL/CD (searched over −2° to 12°) is taken from the polar table. Pitch is then set so α equals that value (θ = α_best + φ, iterated 14 times). Only the chord (5 nodes, 80–300 mm) is optimised, by Powell search, to maximise the figure of merit FM = T^1.5 / (√(2ρA)·P), with A = π·R1² (full disk) and total thrust and power of both blades. There is no thrust target and no requirement for equal lift along the span. The 80 mm lower chord bound avoids the very small tip chords the model otherwise favours.

### 4.5 Torsion estimates (`torsion_check.py`, `bare_rod_twist.py`)

- Section moment about the quarter chord: CM ≈ −0.12 (nose-down), taken from NeuralFoil output for AS6091 at the design α and Re 40–150k (−0.11 to −0.14).
- Moment per metre of span about a rod at chordwise position x_rod: m = ½ρV²·c²·[CM + CL·(x_rod/c − 0.25)] (nose-up positive). Torque at the wing root is the integral of m along the span.
- Twist = integral of T/GJ along the rod, where GJ is the torsional rigidity. Stiffness values assumed: G = 4.5 GPa for unidirectional carbon, solid square J ≈ 0.141·a⁴; thin-wall UD square tube with 0.5 mm wall, 4.0 GPa; ±45° layups are several times stiffer (not computed in detail).
- Twist from the bare rod length L between the clamp and the wing root = T_root·L / GJ.

### 4.6 Paper check (`paper_check.py`)

The same BEM with the paper's geometry: r = 0.40–0.65 m, ω = 9.27 rad/s, 4 wings, Table I chords, a Franky foil rebuilt as 26% S1223 + 74% SD7037 (my interpolation, since the original coordinates were not available). The 2D check at Re 85k, α = 10° gave CL 1.54, CD 0.03, against the paper's CL 1.79 and CD about 0.045, so my polars are somewhat lower than the paper's.

---

## 5. Results: 2D airfoil screening

### 5.1 Mean maximum CL/CD over V = 5–10 m/s (average of both NCrit)

| Foil | c = 0.05 | 0.10 | 0.15 | 0.20 | 0.25 | 0.30 | Worst case (any NCrit) |
|---|---|---|---|---|---|---|---|
| as6091 | 30.4 | 49.8 | 62.5 | 71.1 | 77.2 | 81.8 | 18.2 |
| as6092 | 28.9 | 48.3 | 60.4 | 68.3 | 74.0 | 78.2 | 16.8 |
| goe210 | 21.4 | 41.4 | 53.7 | 62.7 | 69.9 | 75.6 | 10.5 |
| hq308 | 21.6 | 40.7 | 52.3 | 61.0 | 68.0 | 73.7 | 10.0 |
| goe57 | 20.3 | 41.1 | 53.6 | 62.1 | 68.6 | 73.7 | 10.0 |
| as6093 | 27.3 | 46.7 | 57.6 | 64.9 | 69.9 | 73.6 | 15.1 |
| goe346 | 23.0 | 41.0 | 52.0 | 60.1 | 66.5 | 71.6 | 11.4 |
| sa7024 | 21.5 | 40.2 | 51.3 | 59.2 | 65.3 | 70.2 | 10.5 |
| be50 | 21.0 | 40.1 | 51.5 | 59.3 | 65.2 | 69.9 | 11.0 |
| be50sm | 20.9 | 39.9 | 51.2 | 58.9 | 64.8 | 69.6 | 11.0 |
| s3021 | 14.0 | 33.5 | 46.0 | 54.2 | 60.4 | 65.4 | 6.2 |
| s2048 | 21.2 | 37.0 | 46.3 | 53.3 | 58.9 | 63.4 | 8.6 |
| s2050 | 19.9 | 33.8 | 41.6 | 47.1 | 51.3 | 54.7 | 7.5 |
| s1012 | 15.8 | 24.9 | 29.8 | 33.5 | 35.7 | 37.4 | 5.5 |
| s1010 | 16.1 | 21.5 | 25.8 | 29.1 | 31.7 | 33.8 | 13.3 |

Main findings:

- L/D increases steadily with chord for every foil because Re scales with chord. There is no optimum chord below the 0.30 m limit; 0.20 m still gives about 90% of the 0.30 m value for the top foils.
- At very small chord (Re 17–33k) L/D collapses (AS6091 down to about 18 in disturbed air), and the thicker foils suffer most.
- The S1010, S1012, S2048, S2050, S3021 foils are clearly worse.

### 5.2 Maximum CL/CD at chord 0.30 m by velocity (avg NCrit)

| Foil | 5 m/s | 6 | 7 | 8 | 9 | 10 |
|---|---|---|---|---|---|---|
| as6091 | 72.4 | 77.3 | 80.9 | 83.9 | 86.9 | 89.1 |
| as6092 | 69.5 | 73.8 | 77.5 | 80.6 | 83.0 | 84.9 |
| as6093 | 65.7 | 70.1 | 73.2 | 75.6 | 77.6 | 79.4 |
| goe210 | 63.6 | 69.5 | 74.4 | 78.6 | 82.1 | 85.2 |
| goe346 | 60.9 | 66.2 | 70.5 | 74.2 | 77.4 | 80.2 |
| goe57 | 62.9 | 68.3 | 72.7 | 76.4 | 79.7 | 82.4 |
| hq308 | 61.7 | 67.4 | 72.4 | 76.7 | 80.4 | 83.8 |
| sa7024 | 60.0 | 65.1 | 69.2 | 72.8 | 75.8 | 78.5 |
| be50 | 60.1 | 65.0 | 69.0 | 72.4 | 75.3 | 77.9 |
| be50sm | 59.7 | 64.5 | 68.5 | 71.9 | 75.1 | 77.9 |
| s3021 | 54.9 | 60.1 | 64.4 | 68.0 | 71.0 | 73.8 |
| s2048 | 53.9 | 58.5 | 62.4 | 65.8 | 68.7 | 71.3 |
| s2050 | 47.7 | 51.2 | 54.0 | 56.4 | 58.4 | 60.2 |
| s1012 | 34.0 | 36.3 | 37.6 | 37.8 | 38.8 | 40.0 |
| s1010 | 29.4 | 31.7 | 33.3 | 34.8 | 36.2 | 37.7 |

### 5.3 The same at chord 0.10 m (avg NCrit)

| Foil | 5 m/s | 6 | 7 | 8 | 9 | 10 |
|---|---|---|---|---|---|---|
| as6091 | 38.8 | 43.8 | 48.3 | 52.4 | 56.1 | 59.5 |
| as6092 | 37.3 | 42.5 | 46.9 | 50.9 | 54.5 | 57.8 |
| as6093 | 35.9 | 41.1 | 45.5 | 49.3 | 52.6 | 55.5 |
| goe210 | 30.2 | 35.7 | 40.2 | 44.0 | 47.5 | 50.7 |
| goe346 | 31.3 | 35.9 | 39.9 | 43.3 | 46.5 | 49.4 |
| goe57 | 28.2 | 35.2 | 40.3 | 44.4 | 47.8 | 50.9 |
| hq308 | 30.2 | 35.2 | 39.4 | 43.2 | 46.5 | 49.5 |
| sa7024 | 30.0 | 35.0 | 39.1 | 42.7 | 45.8 | 48.7 |
| be50 | 28.7 | 34.7 | 39.2 | 42.9 | 46.1 | 49.0 |
| be50sm | 28.6 | 34.6 | 39.1 | 42.8 | 45.9 | 48.8 |

### 5.4 GOE210 specifically (avg NCrit)

Maximum CL/CD against chord and velocity:

| Chord (m) | 5 m/s | 6 | 7 | 8 | 9 | 10 |
|---|---|---|---|---|---|---|
| 0.05 | 13.9 | 16.5 | 18.8 | 22.4 | 26.7 | 30.2 |
| 0.10 | 30.2 | 35.7 | 40.2 | 44.0 | 47.5 | 50.7 |
| 0.15 | 42.2 | 47.5 | 52.2 | 56.4 | 60.2 | 63.6 |
| 0.20 | 50.7 | 56.4 | 61.3 | 65.7 | 69.5 | 72.8 |
| 0.25 | 57.7 | 63.6 | 68.6 | 72.8 | 76.6 | 79.8 |
| 0.30 | 63.6 | 69.5 | 74.4 | 78.6 | 82.1 | 85.2 |

Operating point at chord 0.30 m:

| V (m/s) | α at max L/D (deg) | CL | Max CL/CD (avg) | NCrit 9 | NCrit 5 |
|---|---|---|---|---|---|
| 5 | 4.6 | 0.96 | 63.6 | 64.0 | 63.3 |
| 6 | 4.4 | 0.94 | 69.5 | 70.5 | 68.5 |
| 7 | 4.1 | 0.92 | 74.4 | 76.0 | 72.8 |
| 8 | 4.0 | 0.91 | 78.6 | 80.8 | 76.3 |
| 9 | 3.8 | 0.89 | 82.1 | 85.1 | 79.2 |
| 10 | 3.8 | 0.89 | 85.2 | 88.7 | 81.8 |

AS6091 for comparison at chord 0.30 m: α 4.0° at 5 m/s easing to 3.1° at 10 m/s, CL 1.00 falling to 0.92, CD 0.014 falling to 0.010, CL/CD 72 rising to 89. The optimum in α is a flat plateau; within about ±1° L/D changes by only a few percent.

---

## 6. Results: XFoil verification

36 cases (AS6091, AS6092, GOE210; V = 5, 7, 10 m/s; chord 0.15, 0.30 m; NCrit 9, 5). All 36 converged.

- Median difference in max CL/CD between NeuralFoil and XFoil: **1.1%**.
- 35 of 36 within 15% (the worst of those 35: 5.0%).
- Optimal α agreed to about 0.5° and CL to about 0.05.
- AS6091 stayed best in XFoil.

GOE210 cases in detail:

| V | Chord | NCrit | Re | XFoil L/D | XFoil α | XFoil CL | NeuralFoil L/D | NF α | NF CL | Error (NF vs XFoil) |
|---|---|---|---|---|---|---|---|---|---|---|
| 5 | 0.15 | 9 | 50k | 41.6 | 6.00 | 1.09 | 41.1 | 6.00 | 1.08 | −1.3% |
| 5 | 0.15 | 5 | 50k | 43.8 | 5.25 | 0.98 | 43.3 | 5.50 | 1.00 | −1.2% |
| 5 | 0.30 | 9 | 100k | 64.5 | 4.75 | 0.98 | 64.0 | 5.00 | 1.00 | −0.9% |
| 5 | 0.30 | 5 | 100k | 62.0 | 4.00 | 0.90 | 63.3 | 4.25 | 0.92 | +2.0% |
| 7 | 0.15 | 9 | 70k | 53.0 | 5.25 | 1.02 | 51.8 | 5.50 | 1.04 | −2.2% |
| 7 | 0.15 | 5 | 70k | 52.6 | 4.75 | 0.95 | 52.6 | 4.75 | 0.95 | 0.0% |
| 7 | 0.30 | 9 | 140k | **564** | 1.75 | 0.66 | 76.0 | 4.50 | 0.96 | spurious |
| 7 | 0.30 | 5 | 140k | 70.3 | 3.50 | 0.85 | 72.8 | 3.75 | 0.88 | +3.6% |
| 10 | 0.15 | 9 | 100k | 64.5 | 4.75 | 0.98 | 64.0 | 5.00 | 1.00 | −0.9% |
| 10 | 0.15 | 5 | 100k | 62.0 | 4.00 | 0.90 | 63.3 | 4.25 | 0.92 | +2.0% |
| 10 | 0.30 | 9 | 200k | 86.0 | 4.00 | 0.91 | 88.7 | 4.25 | 0.94 | +3.1% |
| 10 | 0.30 | 5 | 200k | 77.9 | 2.75 | 0.78 | 81.8 | 3.25 | 0.83 | +5.0% |

The one outlier (GOE210, 7 m/s, c = 0.30 m, NCrit 9: XFoil L/D 564 at 1.75°) is almost certainly a spurious near-zero-drag point from a laminar bubble artefact. NeuralFoil's 76 at 4.5° is the value to trust.

AS6091 at 10 m/s, c = 0.30 m: XFoil gave 94 (calm) and 80 (disturbed); NeuralFoil 97 and 81. Quote 80–90 rather than 97.

![XFoil vs NeuralFoil](results/plots/4_xfoil_vs_neuralfoil.png)

---

## 7. Results: rotor (BEM) designs

### 7.1 Power at fixed thrust per blade (thrust-targeted runs, section 4.4a)

| Foil | Thrust/blade | Optimised chord + pitch | Constant 0.30 m | Linear taper 0.30→0.12 m | 1/r (0.30 at root) |
|---|---|---|---|---|---|
| AS6091 | 1 N | 1.403 W (area 0.038 m²) | 1.639 W (0.090 m²) | 1.503 W | 1.481 W |
| AS6091 | 2 N | 3.890 W (0.070 m²) | 4.015 W | 3.993 W | 4.228 W |
| AS6093 | 1 N | 1.419 W | 1.635 W | 1.536 W | 1.508 W |
| AS6093 | 2 N | 3.909 W | 4.020 W | 3.993 W | 4.607 W |
| GOE210 | 1 N | 1.434 W | 1.721 W | 1.493 W | 1.489 W |
| GOE210 | 2 N | 3.916 W | 4.030 W | 3.985 W | 4.108 W |

- Optimised chord at 1 N per blade (AS6091): about 232 mm at the root tapering to about 96 mm at r = 0.4 m. 15% less power than constant 0.30 m with 58% less area.
- Optimised chord at 2 N per blade: 0.30 m out to about r = 0.3 m, then tapering (221 mm at r = 0.4 m, 138 mm at 0.45 m). The chord laws differ by only 3–4% at 2 N, so any sensible taper works.
- An exploratory first run at 0.5 N per blade (AS6091) gave 0.563 W for the optimised chord, 0.954 W for constant chord and 0.723 W for linear taper (chord 125 mm at the root, shrinking to the lower bound at the tip).
- SLSQP is a local optimiser; differences below about 3% should be treated as noise.

AS6091 optimised station values:

| | r = 0.20 | 0.25 | 0.30 | 0.35 | 0.40 | 0.45 | 0.50 |
|---|---|---|---|---|---|---|---|
| **T = 1 N/blade** chord (mm) | 232 | 188 | 152 | 123 | 96 | 66 | 30 |
| pitch (deg) | 22.2 | 18.9 | 16.4 | 14.6 | 13.8 | 12.5 | 9.5 |
| alpha (deg) | 4.4 | 4.5 | 4.5 | 4.5 | 5.0 | 4.9 | −0.1 |
| CL | 1.05 | 1.06 | 1.06 | 1.06 | 1.10 | 1.08 | 0.30 |
| Re (k) | 65 | 65 | 62 | 58 | 52 | 40 | 20 |
| **T = 2 N/blade** chord (mm) | 300 | 300 | 297 | 278 | 221 | 138 | 30 |
| pitch (deg) | 30.6 | 25.0 | 21.4 | 19.0 | 16.8 | 15.7 | 15.3 |
| alpha (deg) | 8.0 | 5.9 | 4.5 | 3.8 | 3.5 | 4.2 | 2.1 |
| CL | 1.34 | 1.16 | 1.05 | 0.99 | 0.96 | 1.03 | 0.57 |
| Re (k) | 87 | 106 | 124 | 135 | 121 | 84 | 21 |

Constant 0.30 m chord, AS6091: the outer blade runs at low CL because the blade is oversized for the thrust. At T = 1 N the CL runs 0.74 (r = 0.20), 0.41 (0.35), 0.28 (0.45), 0.04 (0.50), with α from 2.0° down to −2.9°. At T = 2 N: CL 1.30, 0.92, 0.48, 0.05 and α from 7.4° to −2.8°.

![Blade distributions, thrust-targeted](results/plots/5_blade_distributions.png)

### 7.2 Hand check of one strip (AS6091, T = 1 N per blade, r = 0.30 m)

c = 0.152 m, θ = 16.4°, α = 4.5°, CL = 1.06, CL/CD = 57.

| Step | Calculation | Result |
|---|---|---|
| Blade speed | U = 20 × 0.3 | 6.0 m/s |
| Inflow angle | φ = θ − α | 11.9° |
| Induced velocity | v = U·tan φ | 1.26 m/s |
| Relative speed | √(6² + 1.26²) | 6.13 m/s |
| Reynolds number | 6.13 × 0.152 / 1.5e-5 | 62,000 |
| CD | 1.06 / 57 | 0.0186 |
| Lift per metre | ½ · 1.2 · 6.13² · 0.152 · 1.06 | 3.63 N/m |
| Drag per metre | ½ · 1.2 · 6.13² · 0.152 · 0.0186 | 0.064 N/m |
| Thrust per metre | 3.63·cos 11.9° − 0.064·sin 11.9° | 3.54 N/m |
| Momentum check (2 blades) | 4π·1.2·0.3·1.26² = 7.18 vs 2 × 3.54 = 7.08 | agrees within 1.4% |

### 7.3 Every station at best CL/CD, chord optimised (section 4.4b), nine foils

| Foil | Lift (g) | Aero power (W) | Lift per watt (g/W) | FM | 5 mm rod fits (span) | 10 mm rod fits (span) | Const 0.30 m chord: thrust / power / FM |
|---|---|---|---|---|---|---|---|
| as6091 | 380 | 7.19 | 52.9 | 0.729 | 0% | 0% | 5.76 N / 16.8 W / 0.60 |
| as6092 | 399 | 7.76 | 51.4 | 0.726 | 64% | 0% | 6.13 N / 18.5 W / 0.60 |
| hq308 | 366 | 6.83 | 53.5 | 0.724 | 100% | 72% | 5.25 N / 14.2 W / 0.62 |
| as6093 | 412 | 8.19 | 50.3 | 0.722 | 72% | 0% | 6.21 N / 18.8 W / 0.60 |
| goe210 | 382 | 7.31 | 52.2 | 0.722 | 92% | 68% | 5.48 N / 15.4 W / 0.61 |
| goe346 | 355 | 6.56 | 54.1 | 0.722 | 88% | 64% | 4.97 N / 13.1 W / 0.62 |
| be50 | 353 | 6.50 | 54.3 | 0.722 | 96% | 72% | 4.93 N / 12.9 W / 0.62 |
| goe57 | 381 | 7.31 | 52.2 | 0.721 | 92% | 68% | 5.31 N / 14.5 W / 0.62 |
| sa7024 | 356 | 6.59 | 54.0 | 0.721 | 96% | 72% | 4.94 N / 12.7 W / 0.63 |

"Rod fits" is the share of span where the foil's thickness at 30% chord is at least rod size + about 1 mm of skin (6 mm and 12 mm thresholds).

Observations:

- The optimiser keeps nearly full chord: 0.30 m to about r = 0.30 m, about 0.28 m at mid-span, 0.08 m at the tip (the lower bound).
- The AS series has the highest section CL/CD (up to about 83 at r ≈ 0.35 m, against 70–75 for the thicker foils), but lift per watt ends up about equal, and the thicker foils are slightly better (54 against 50–53 g/W) at 5–10% less total lift.
- Taking full chord everywhere (0.30 m) raises thrust to 4.9–6.2 N but lowers FM to about 0.60–0.63 and needs 12.7–18.8 W, about 35% less lift per watt.

![Spanwise design, nine foils](results/plots/6_spanwise_design.png)
![Totals per foil](results/plots/7_totals_per_foil.png)

### 7.4 Hover capacity of a full 0.30 m chord, AS6091 (constant chord, one α everywhere)

| α everywhere | Blades | Thrust per blade | Total | Mass equivalent | Aero power (all blades) |
|---|---|---|---|---|---|
| 5° | 2 | 3.31 N | 6.63 N | 676 g | 22.8 W |
| 5° | 3 | 3.42 N | 10.26 N | 1046 g | 44.3 W |
| 5° | 4 | 3.51 N | 14.02 N | 1429 g | 69.2 W |
| 7° | 2 | 3.89 N | 7.79 N | 794 g | 30.3 W |
| 7° | 3 | 4.04 N | 12.12 N | 1236 g | 59.6 W |
| 7° | 4 | 4.13 N | 16.53 N | 1685 g | 88.6 W |
| 9° | 2 | 4.42 N | 8.85 N | 902 g | 38.4 W |
| 9° | 3 | 4.59 N | 13.76 N | 1402 g | 72.8 W |
| 9° | 4 | 4.64 N | 18.54 N | 1890 g | 106.4 W |

Sanity check: momentum theory for 4 N total thrust over a 1 m diameter disk (area about 0.8 m²) gives about 5.8 W, and the BEM gave 7.8 W (2 × 3.9 W) at 2 N per blade, consistent with large slow rotors.

---

## 8. GOE210: detailed design data

### 8.1 Station table (every station at best CL/CD, chord optimised)

| r (m) | Chord (mm) | Pitch (deg) | Inflow φ (deg) | α (deg) | CL | CL/CD | Re (k) | V_rel (m/s) | Induced v (m/s) | dT/dr (N/m) | Thickness at 37% chord (mm) |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 0.200 | 299.7 | 24.7 | 19.7 | 5.1 | 1.01 | 57.8 | 84.9 | 4.25 | 1.43 | 3.05 | 19.7 |
| 0.225 | 299.7 | 23.4 | 18.5 | 4.9 | 0.99 | 61.6 | 94.8 | 4.75 | 1.51 | 3.80 | 19.7 |
| 0.250 | 299.7 | 22.6 | 17.7 | 4.9 | 1.00 | 65.3 | 104.8 | 5.25 | 1.60 | 4.68 | 19.7 |
| 0.275 | 299.7 | 21.7 | 16.9 | 4.8 | 0.99 | 68.3 | 114.8 | 5.75 | 1.67 | 5.58 | 19.7 |
| 0.300 | 297.1 | 20.8 | 16.2 | 4.6 | 0.97 | 71.0 | 123.7 | 6.25 | 1.74 | 6.45 | 19.5 |
| 0.325 | 290.1 | 20.0 | 15.5 | 4.5 | 0.96 | 73.0 | 130.5 | 6.75 | 1.80 | 7.28 | 19.1 |
| 0.350 | 279.9 | 19.4 | 15.0 | 4.4 | 0.96 | 74.3 | 135.2 | 7.25 | 1.87 | 8.13 | 18.4 |
| 0.375 | 248.8 | 18.5 | 14.0 | 4.5 | 0.96 | 72.5 | 128.2 | 7.73 | 1.87 | 8.31 | 16.4 |
| 0.400 | 197.1 | 17.6 | 12.7 | 5.0 | 1.00 | 66.5 | 107.8 | 8.20 | 1.80 | 7.74 | 13.0 |
| 0.425 | 153.0 | 16.3 | 11.3 | 5.0 | 1.00 | 59.4 | 88.4 | 8.67 | 1.70 | 6.76 | 10.1 |
| 0.450 | 123.8 | 16.3 | 10.9 | 5.5 | 1.04 | 54.2 | 75.6 | 9.16 | 1.73 | 6.33 | 8.1 |
| 0.475 | 98.9 | 16.6 | 11.2 | 5.5 | 1.04 | 48.6 | 63.8 | 9.68 | 1.87 | 5.64 | 6.5 |
| 0.500 | 80.0 | 32.4* | 27.7* | 4.7 | 0.97 | 43.3 | 60.2 | 11.30 | 5.25* | 5.20 | 5.3 |

\*The last station is an artefact of the tip-loss factor (the required pitch explodes there). Use the build schedule in section 1 (about 16.5° out to the tip) instead. Stations are shown every other point (spacing 0.0125 m, 25 stations).

Totals: thrust 3.74 N (382 g), aero power 7.31 W, FM 0.722, per blade 1.87 N and 3.66 W.

Observations: α runs 4.4–5.5° everywhere, section CL/CD peaks at about 74 near r = 0.35 m and falls to 43 at the tip because Re falls to 60k; thrust per metre peaks at 8.3 N/m near r = 0.375 m. The inflow angle falls from about 20° at the root to about 11° at 0.45 m.

### 8.2 Alternative: thrust-targeted GOE210 designs

| Thrust/blade | Chord root / mid / tip | Power per blade |
|---|---|---|
| 1 N | 242 / 127 / 31 mm | 1.434 W |
| 2 N | 300 / 288 / 36 mm | 3.916 W |

(Constant 0.30 m chord needs 1.721 W and 4.030 W for the same thrust.) Use these if you need a specific thrust and want the lowest power for it; use section 8.1 if you want the best figure of merit with no thrust target.

### 8.3 Robustness across 15–20 rad/s (fixed built geometry)

Requirement: one fixed blade (chord and pitch built in) that is reasonable from 15 rad/s (already achieved) to 20 rad/s (expected upper end). Script: `robust_design.py`. Hover BEM, 2 blades, GOE210, NCrit 9. Stall margin means α(CLmax) at the local Re minus the local α.

**The section 8.1 design (pitch tip held at 16.5°) evaluated at three spin rates:**

| Ω (rad/s) | Tip speed (m/s) | Lift, 2 blades | Aero power | FM | α range (loaded span) | Min stall margin | Thrust-weighted CL/CD |
|---|---|---|---|---|---|---|---|
| 15.0 | 7.5 | 2.07 N (211 g) | 2.99 W | 0.728 | 1.0–5.6° | 4.1° | 54 |
| 17.5 | 8.75 | 2.83 N (288 g) | 4.72 W | 0.734 | 1.0–5.6° | 4.9° | 60 |
| 20.0 | 10.0 | 3.70 N (377 g) | 7.02 W | 0.738 | 0.8–5.6° | 5.3° | 65 |

Why it holds up: when Ω changes, the induced velocity scales about the same way (v ∝ Ω), so the inflow angle φ = atan(v / Ωr) barely changes. The built pitch therefore still gives the same α (4.4–5.5° at the loaded stations), and only the Reynolds number (about 25% lower at 15 rad/s) and the thrust change. Lift scales as Ω² (211 g against 377 g, a ratio of 0.56), not because the blade is working worse, which is why FM is nearly unchanged (0.728 against 0.738).

Station values at the two ends (design as built):

| r (m) | α at 15 / 20 | CL at 15 / 20 | CL/CD at 15 | CL/CD at 20 | Re at 15 (k) | Re at 20 (k) |
|---|---|---|---|---|---|---|
| 0.20 | 5.1 / 5.1° | 1.01 / 1.01 | 47 | 58 | 64 | 85 |
| 0.25 | 5.0 / 4.9° | 1.00 / 1.00 | 55 | 65 | 79 | 105 |
| 0.30 | 4.6 / 4.6° | 0.97 / 0.97 | 60 | 71 | 93 | 124 |
| 0.35 | 4.5 / 4.4° | 0.96 / 0.96 | 63 | 74 | 101 | 135 |
| 0.40 | 5.0 / 5.0° | 1.00 / 1.00 | 56 | 66 | 81 | 108 |
| 0.45 | 5.5 / 5.4° | 1.04 / 1.04 | 44 | 54 | 57 | 76 |
| 0.50 | 1.0 / 0.8° | 0.30 / 0.31 | 11 | 11 | 42 | 55 |

(The last station is the unloaded tip and should be ignored.)

**Optimising one geometry for the whole band.** I let the optimiser vary chord and pitch (5 nodes each) to maximise the average FM at 15, 17.5 and 20 rad/s with a penalty for stall margin below 3°. Result: FM 0.733 / 0.739 / 0.743 against 0.728 / 0.734 / 0.738 for the section 8.1 design, an improvement of only about 0.005 (under 1%), and it gets there by loading the root harder (α up to 8.1°, stall margin down to 3.0°, thrust-weighted CL/CD lower: 51 / 57 / 61). That is within the noise of the model and trades away stall margin, so **it is not worth adopting.** The section 8.1 design stays as the build.

**Sensitivity to a uniform pitch error** (an error you could get from build tolerance or elastic twist), design as built:

| Pitch error | 15 rad/s: lift / FM / min stall margin | 20 rad/s: lift / FM / min stall margin |
|---|---|---|
| −4° | 161 g / 0.70 / 5.8° | 292 g / 0.73 / 7.3° |
| −2° | 187 g / 0.72 / 5.1° | 335 g / 0.74 / 6.4° |
| 0° | 211 g / 0.73 / 4.1° | 377 g / 0.74 / 5.3° |
| +2° | 235 g / 0.72 / 2.9° | 418 g / 0.73 / 4.1° |
| +4° | 258 g / 0.71 / 1.7° | 460 g / 0.72 / 2.9° |

Each degree of pitch error is worth about 12 g at 15 rad/s and about 21 g at 20 rad/s (about 6% and 5.5% of lift). A pitch error of +4° at 15 rad/s leaves only 1.7° of stall margin, so a wing that gets built or loaded with too much pitch stalls first at the low spin rate (lower Re, stall at a lower α). Errors toward lower pitch are safe for stall but cost lift.

**What this means for the plan:**

- Lift of the 2-blade set as designed: about 210 g at 15 rad/s up to about 380 g at 20 rad/s. With the 50–70% hardware knock-down in section 12, that is roughly 105–150 g at 15 rad/s (the already achieved case) and 190–265 g at 20 rad/s.
- If more lift is needed at 15 rad/s, add blades or add a little pitch (about +2° gives +24 g at 15 rad/s, stall margin 2.9°); do not add pitch beyond about +2°.
- Because α barely changes with Ω, there is nothing to re-tune between 15 and 20 rad/s.

### 8.4 Why section CL/CD falls outboard of r = 0.35 m

Re = V·c / ν. From r = 0.35 m to the tip V rises by 1.56× (7.25 → 11.3 m/s) while the chord falls by 3.5× (280 → 80 mm), so Re falls from its peak of 135k to 60k (about 42k at 15 rad/s), and CL/CD falls from 74 to 43. The GOE210 polar table shows the same dependence on Re alone: best CL/CD 36 at Re 43k, 48 at 62k, 67 at 109k, 73 at 131k, 87 at 190k, with the best α rising from 4.0° to 6.0° as Re falls. (An earlier version of this section said the optimiser tapers the chord because profile power goes as c·V³. The split below shows that is not the main reason; see the power split.)

Test: chord ramp from 0.28 m at r = 0.35 m to a wider tip chord (hover, 20 rad/s, best-CL/CD stations):

| Tip chord | Lift | Aero power | Lift per watt | FM | Outer-span CL/CD (r ≥ 0.4) | Outer Re |
|---|---|---|---|---|---|---|
| 80 mm | 400 g | 7.84 W | 51 g/W | 0.721 | 60 | 60–117k |
| 150 mm | 448 g | 9.92 W | 45 g/W | 0.677 | 69 | 110–130k |
| 200 mm | 482 g | 11.53 W | 42 g/W | 0.649 | 75 | 139–171k |
| 300 mm | 545 g | 14.97 W | 36 g/W | 0.602 | 84 | 159–274k |

Widening the outer chord raises local CL/CD but costs more power per unit of lift. A 150 mm tip chord is a reasonable compromise if low-Re margin at 15 rad/s matters more than efficiency (+12% lift for +25% power).

**Power split (induced-type vs profile).** Power = Ω × ∫ r·(dL·sinφ + dD·cosφ) dr. The first term is lift tilted by the inflow angle (induced-type power, the cost of accelerating air downward); the second is section drag (profile power):

| Tip chord | Lift | Total power | Induced-type | Profile | Step from the previous row |
|---|---|---|---|---|---|
| 80 mm | 400 g | 7.84 W | 7.40 W | 0.44 W (6%) | — |
| 150 mm | 448 g | 9.92 W | 9.45 W | 0.47 W (5%) | +49 g for +2.07 W (23 g/W) |
| 200 mm | 482 g | 11.53 W | 11.04 W | 0.49 W (4%) | +33 g for +1.61 W (21 g/W) |
| 300 mm | 545 g | 14.97 W | 14.44 W | 0.53 W (4%) | +64 g for +3.44 W (19 g/W) |

So 95% of the aero power is induced power, and the extra chord adds almost no profile drag (0.44 → 0.53 W). The last ~145 g of lift costs about 7.1 W, about 20 g/W, against 51 g/W on average, because induced power grows faster than lift (roughly T^1.5: (545/400)^1.5 = 1.59, while power grew 1.91×; the remaining factor of about 1.2 comes from loading the outer blade, which is a less efficient distribution than the one the optimiser found). This is also why the foil choice barely changed the figure of merit in section 7.3: section CL/CD only affects the small profile share.

### 8.5 Torque: who pays for the power

Aero power equals torque times spin rate, Q = P / Ω, so the wing power is a drag torque against the rotation:

| Case | Aero power | Torque Q | Electrical at η = 0.65 (paper's assumption) |
|---|---|---|---|
| Current design, 20 rad/s | 7.31 W | 0.365 N·m | 11.2 W |
| Current design, 15 rad/s | 2.99 W | 0.199 N·m | 4.6 W |
| Tip chord 150 mm, 20 rad/s | 9.92 W | 0.496 N·m | 15.3 W |
| Tip chord 200 mm, 20 rad/s | 11.53 W | 0.577 N·m | 17.7 W |
| Tip chord 300 mm, 20 rad/s | 14.97 W | 0.749 N·m | 23.0 W |

The last ~145 g of lift (80 → 300 mm tip chord) needs about 0.36 N·m more torque. Wing torque scales as Ω² for a fixed blade (0.199 against 0.365 N·m).

Who bears it, in the paper's architecture: the wing drag torque resists the spin, and steady spin is reached when the motors' yaw torque equals it (the paper's torque balance, Eq. 17). That yaw torque comes almost entirely from the tangential component of the canted motors' thrust (M_ψ ≈ Σ f_i·sin α·l, Eq. 13; propeller reaction torque was only about 1.5% in the paper's simulation). So the cost is paid by the motors and propellers as electrical yaw power (τω/η, Eq. 22), and by tilting thrust away from vertical (factor cos α). If a wing had its own drive instead, this torque would load that drive's shaft or gearbox, with an equal reaction on the airframe.

Rough check of whether 10° cant can supply 20 rad/s (order of magnitude only; paper's Configuration II: motors at 0.3 m arm, about 7.4 N total motor thrust at hover):

| Cant angle | Motor yaw torque available |
|---|---|
| 10° | 0.39 N·m |
| 20° | 0.76 N·m |
| 30° | 1.11 N·m |

The current design needs 0.365 N·m at 20 rad/s, which is about the full capacity at 10° cant, with no margin for propeller drag, body drag or other losses. At 15 rad/s it needs only 0.199 N·m. A wider tip chord (0.50–0.75 N·m) would not be reachable at 10° cant. Steady spin rate is set by where the motors' torque meets the wing torque curve (Q ∝ Ω²), so cant angle is the knob that decides how fast it can spin. For comparison, the paper's simulation used 44° tilt and reached 1.03 N·m.

---

## 9. Thickness and the carbon rod

### 9.1 Thickness of the candidate foils

| Foil | Max thickness (at x/c) | t at 25% chord | t at 35% chord | t at 40% chord |
|---|---|---|---|---|
| AS6091 | 4.05% (0.10) | 2.39% | 1.38% | 1.02% |
| AS6092 | 5.05% (0.12) | 3.51% | 2.29% | 1.80% |
| AS6093 | 6.06% (0.13) | 4.58% | 3.14% | 2.52% |
| GOE210 | 6.63% (0.30) | 6.50% | 6.59% | 6.55% |

In millimetres at chord 0.30 m / 0.20 m:

| Foil | 25% chord | 35% chord | 40% chord |
|---|---|---|---|
| AS6091 | 7.2 / 4.8 | 4.1 / 2.8 | 3.0 / 2.0 |
| AS6092 | 10.5 / 7.0 | 6.9 / 4.6 | 5.4 / 3.6 |
| AS6093 | 13.7 / 9.2 | 9.4 / 6.3 | 7.6 / 5.0 |
| GOE210 | 19.5 / 13.0 | 19.8 / 13.2 | 19.7 / 13.1 |

GOE210 thickness fractions: 6.07% at 15% chord, 6.63% at 30% chord, 6.58% at 37% chord, 6.08% at 50% chord, 4.18% at 70% chord.

### 9.2 Rod fit along the GOE210 blade (rod at 37% chord, 1.5 mm skin or fairing)

| Rod | Fits out to r = |
|---|---|
| 10 mm | 0.400 m |
| 8 mm | 0.425 m |
| 5 mm | 0.475 m |

So a tapered rod (10 mm → 8 mm → 5 mm) follows the thickness, and the bending moment also falls toward the tip. The AS-series foils cannot hold even a 5 mm rod over most of the span at these chordwise positions (AS6091: 0% of the span).

### 9.3 Fairing options if the rod protrudes

- Taper the rod with the thickness (preferred).
- Use a thicker foil where the rod is (GOE210 and similar).
- Blend a gradual fairing around the rod (ramp length at least 8–10 times the step height; a 4 mm step needs about 30–40 mm).
- Put any unavoidable bulge on the lower surface, since the suction surface is more sensitive (heuristic, not tested).
- Avoid a thin flat strip: bending stiffness goes as thickness cubed, so a 10 × 2 mm strip is about 125 times less stiff in bending than a 10 × 10 mm square.
- Weight: a solid 10 × 10 mm carbon rod is about 160 g/m; a hollow tube is far lighter. Wing lift measured on the paper's hardware was only 53–73 g per wing, so heavy rods can cancel the benefit.

---

## 10. Torsion analysis

### 10.1 Why the wing twists

Two aerodynamic moments act about the rod:

- The cambered section's own moment: CM ≈ −0.12 about the quarter chord (nose-down), measured from NeuralFoil for AS6091 at the design α (−0.109 to −0.141 over Re 40–150k).
- The lift, acting at the aerodynamic centre (about 25% chord), has a moment about the rod if the rod is not at 25% chord.

The centre of pressure at design CL ≈ 1.05 is at 0.25c − CM/CL ≈ **0.36c**. A rod there sees almost no net torque.

### 10.2 Rod only, wing span only (AS6091 T = 2 N design, solid carbon rod, rod 10 mm)

| Rod position | Root torque | Tip twist, solid UD 10 mm square | Tip twist, UD tube 10/8 mm |
|---|---|---|---|
| 25% chord | −58 mN·m | −0.08° | −0.17° |
| 35% chord | −7 mN·m | −0.01° | −0.03° |
| 40% chord | +18 mN·m | +0.02° | +0.05° |
| 50% chord | +68 mN·m | +0.08° | +0.19° |

For a 10 mm rod, aerodynamic twist along the wing span is a fraction of a degree if the sections are firmly locked to it.

### 10.3 Adding the bare length between clamp and wing (the missing piece)

Your observation: the wing twisted about 10–15° on a 5 mm rod, with the torque carried over the stretch of rod from the clamp, 10 to 40 cm out. My first estimates counted only the wing span. Torque is carried unchanged along the whole bare section, so twist = T_root × L / GJ adds to the twist of the wing itself.

Estimate for the optimised GOE210 blade at design conditions. Torsional rigidity values: solid 5 mm UD square 0.397 N·m²; thin-wall (0.5 mm) UD 5 mm square tube 0.182 N·m²; solid 10 mm square 6.35 N·m². Negative = nose-down (lowers α).

| Rod position | Root torque | Rod | Wing part | Bare 10 cm | Bare 20 cm | Bare 30 cm |
|---|---|---|---|---|---|---|
| 10% chord | −122 mN·m | 5 mm solid | −2.4° | −1.8° | −3.5° | −5.3° |
| | | 5 mm thin-wall tube | −5.2° | −3.8° | −7.7° | −11.5° |
| | | 10 mm solid | −0.1° | −0.1° | −0.2° | −0.3° |
| 25% chord | −55 mN·m | 5 mm solid | −1.1° | −0.8° | −1.6° | −2.4° |
| | | 5 mm thin-wall tube | −2.3° | −1.7° | −3.4° | −5.2° |
| | | 10 mm solid | −0.1° | 0.0° | −0.1° | −0.1° |
| 37% chord | −1 mN·m | all | about 0° | about 0° | about 0° | about 0° |

Tip twist is the bare-length value plus the wing part. A thin-wall 5 mm unidirectional tube with a rod at 10% chord and 30 cm bare gives about 17°, and the solid rod about 8°. This brackets your measured 10–15° and shows the effect is plausible; it is not a proof, because the real rod material, wall thickness and rod position are not known precisely.

### 10.4 Why it matters

From the paper check (section 11), thrust changes by about 0.2 N per degree of pitch for a 4-wing set, roughly 8–10% per degree at the 2 N level. A 10–15° nose-down twist therefore removes most of the lift.

### 10.5 Fixes, in order of effect

1. Move the rod to 35–40% chord, which cuts the torque by roughly 50× or more (from −55 to −122 mN·m down to about −1 mN·m in the estimate above). Needs a foil that is thick there: GOE210, HQ308 and BE50 qualify.
2. Stiffen the bare section: a larger-diameter tube over the bare 10–40 cm (stiffness goes as diameter⁴, so 10 mm is 16× stiffer than 5 mm), or ±45° carbon wrap (common pultruded rods and tubes have axial fibres, which are poor in torsion).
3. Shorten the bare section: start the wing closer to the clamp.
4. Lock sections to the rod: a round rod in round holes lets ribs rotate. Use square holes or bond them.
5. Do not use fishing line: it needs preload, creeps, adds drag, and only helps if anchored far from the rod axis.
6. D-box: a thin skin (0.1–0.2 mm carbon or glass cloth, or thin foam/balsa) from the rod forward around the leading edge makes a closed section that resists twist strongly. Rough order of magnitude: 5–10 g per wing.
7. Measure: clamp the root, hang a known weight on a short lever at the free end, and measure the angle. GJ_eff = torque × length / angle. Aim for under about 0.5° at design loading. Alternatively film a marker on the tip while spinning.

---

## 11. Checking the paper's wing

Using Table I chords (0.10, 0.19, 0.29, 0.27, 0.26, 0.24 m at r = 0.40 to 0.65 m), ω = 9.27 rad/s, 4 wings, rebuilt Franky foil (26% S1223), total lift of the 4 wings:

| Build | Total lift | Notes |
|---|---|---|
| Flat 10° pitch (α_design used as built incidence) | 1.32 N (135 g) | actual α about −3° to +3°, CL ≈ 0.4 |
| Flat 14° pitch | 2.03 N | close to the hardware's 2.10 N |
| Flat 18° pitch | 2.78 N | |
| Flat 22° pitch | 3.50 N | |
| Pitch = α + φ so α = 10° everywhere | 5.96 N (608 g) | close to the paper's BEMT 5.27 N |

For the α = 10° case the required pitch was about 20° (r = 0.40 m), 28° (0.45 m), 33° (0.50–0.55 m); the outermost station is an unphysical artefact (tip loss) and should be ignored. Induced velocities came out at 0.5–2.6 m/s at the stations, giving inflow angles of 10–25°.

What this implies: Table I picks α = 10° from 2D polars and states that this directly informs blade pitch and twist. If the wings were built with incidence near the design α, with no allowance for the inflow angle, the real α would be only a few degrees. The 2.5× gap in the paper's Table III (BEMT 5.27 N, CFD 4.50 N, hardware 2.10 N) is largely explained by that, if that is what was built. The as-built incidence is not known, so this stays a hypothesis to verify on the hardware.

Caveats: my polars are lower than the paper's (CL 1.54 against 1.79 at α = 10°, Re 85k); the Franky coordinates are my own blend; other effects reduce hardware lift (flexible film wings, wing-to-wing interference, propeller wash).

---

## 12. What to expect on hardware

The paper's own data point: hardware delivered about 40% of the BEMT lift and about 47% of the CFD lift (2.10 N against 5.27 N and 4.50 N).

Main contributors, roughly in order:

- As-built pitch and twist, about 8–10% of lift per degree of error (section 11).
- Flexibility: the wing untwisting or bending under load, especially thin film wings between ribs.
- Wing-to-wing interference: in the paper's hardware, 53.5 g per wing with four wings against 73.4 g per wing with two (a 27% penalty).
- Polar and Reynolds uncertainty: roughly ±10–20% in lift at Re 20–100k.
- Spin rate: lift goes as ω², so 5% low rate costs about 10%.

Planning numbers (my estimates, not measurements): 50–70% of the BEM prediction if the twist is built accurately and the structure is stiff; 30–50% if the twist is only approximate or the wing is flexible. For the GOE210 design (382 g predicted) that is roughly 190–270 g in the stiff case and 115–190 g otherwise, for 2 blades.

Power context: the 7.3 W is aerodynamic shaft power. Electrical power is higher once motor, ESC and propeller losses are included, roughly 2–2.5× for typical efficiencies (about 60–70% overall plus extra losses at low rpm and high torque). Comparing 30 W aero with 50 W electrical (as was done once in the chat) is not like for like; with 65% efficiency the 7.8 N aero case is about 45–50 W electrical, which is about 17 g/W against about 10 g/W for a typical drone, a real but smaller gain.

Other practical points raised in the chat:

- Torque and yaw: two blades at 30 W and 20 rad/s need about 1.5 N·m of shaft torque and push the same reaction torque onto the airframe. The paper's spinning drone uses canted motors for this (the motor yaw torque balanced the wing drag torque at 1.03 vs 1.04 N·m in simulation).
- Measure the real knock-down with a load cell: spin the wing alone at several speeds and plot thrust against ω².

---

## 13. Concepts explained

### 13.1 Pitch, inflow angle and angle of attack

**θ (pitch) = φ (inflow angle) + α (angle of attack).**

- Pitch θ is the geometric angle between the chord line and the rotor plane. You build it in.
- The air a strip sees is the vector sum of the blade's own speed Ωr (tangential) and the downwash v (perpendicular to the rotor plane). It arrives at an angle φ = atan(v / Ωr) below the rotor plane.
- α is the angle between the chord and that incoming air, α = θ − φ. You cannot set α directly; you set θ and the flow sets φ.
- Pitch equals the inflow angle only if α = 0, which gives little or no lift. A cambered foil like AS6091 still produces CL ≈ 0.5 at α = 0 (its zero-lift angle is about −5°).
- φ itself depends on thrust. More pitch gives more thrust, more downwash, a bigger φ, and a smaller α. The BEM solves this loop.

Example (AS6091 T = 2 N design): at r = 0.2 m, pitch 30.6° = φ 22.6° + α 8.0°; at r = 0.3 m, 21.4° = 16.9° + 4.5°; at r = 0.45 m, 15.7° = 11.5° + 4.2°.

### 13.2 Why this rotor's pitch differs a lot from alpha but a helicopter's does not

A helicopter blade moves at about 100–200 m/s with downwash of 5–15 m/s, so φ is only 2–5° and pitch is almost α. Here the blade moves at 4–10 m/s and the downwash is 1.2–2 m/s, so φ is about 7–23°. The induced velocity is set mainly by thrust and disk area (v ≈ √(T / 2ρA), about 1.5 m/s for 4 N over 0.8 m²), and it does not shrink just because the blade is slow. That is also why the twist is so large (12–15° root to tip for this design).

### 13.3 Why CL drops toward the tip on a constant-chord blade

At fixed total thrust, a blade of constant 0.30 m chord is oversized. Lift per metre grows with V², about 6× more at the tip than at the root. The optimiser gives up CL where the penalty is smallest, on the outboard span, which also reduces profile power (which goes as c·V³). Near the very tip, the Prandtl tip-loss factor in the model forces loading toward zero.

### 13.4 Why a narrow root chord helps at low thrust

Root dynamic pressure is about 6× lower than at the tip (4 against 10 m/s), so a full-chord root adds little lift for its drag. Only at higher thrust does the optimum chord fill the 0.30 m limit near the root.

---

## 14. Corrections log

These are statements made earlier in the session that were later corrected or refined:

1. **Maximum hover thrust of a constant 0.30 m chord.** An early hand estimate said about 2.8 N per blade. The BEM gives 3.3 N per blade at α = 5° (3.9 N at 7°). Use section 7.4.
2. **Rod at "max thickness for these foils, 20–30% chord".** Wrong for the AS series: their thickest point is at 10–13% chord, and they are only 3–7 mm thick at 25–35% chord for c = 0.30 m. A 10 mm rod does not fit in AS6091/2/3 at the chordwise positions that matter. GOE210 holds about 19.7 mm there (section 9).
3. **Optimal α plot (2D), mixing NCrit.** The alpha-versus-velocity plot averaged the argmax angles of two NCrit cases (for example 3.75° and 2.5° at 10 m/s gave 3.1°), which is not the optimum for either; the optimum is also a flat plateau sampled at 0.25° steps, so the curve is jagged, and the narrow y-axis exaggerates small changes. Use the NCrit-split table in section 5.4.
4. **Tip behaviour in plots.** The sharp drop of α and CL, and the rise of pitch, in the last about 10% of span come from the tip-loss model and are not physical.
5. **30 W against 50 W.** The first comparison mixed aerodynamic and electrical power (section 12).
6. **Torsion estimate.** The first estimate counted only the twist along the wing span; it left out the bare rod between the clamp and the wing (section 10.3), which explains the 10–15° observed twist.
7. **Rod size.** "10×10 cm" was a typo for 10 × 10 mm.
8. **The paper.** The paper design used α from 2D polars as if it were pitch (section 11).
9. **Why the outer chord tapers.** I first said it was because profile power goes as c·V³. The power split in section 8.4 shows profile power is only 4–6% of the total; the real cost of extra outboard lift is induced power (it grows faster than lift, roughly as T^1.5, and outer-heavy loading is less efficient than the optimised distribution).

---

## 15. Assumptions, limits, and open questions

**Model limits:**

- Hover only; no climb or forward flight (the inflow angle and therefore the twist would change).
- Rigid blade; no flap, flex or centrifugal effects.
- 2D polars from NeuralFoil (trained on XFoil) used in a 3D strip model; transition behaviour at Re 20–100k is uncertain; hardware data would be needed to tighten it.
- No hub, body, wing-to-wing, or propeller wake interference. Four wings would need extra knock-down.
- Tip-loss factor is a simple Prandtl correction; tip values are unreliable.
- ρ = 1.2, ν = 1.5e-5.
- Torsion numbers use assumed rod stiffness (G = 4.5 GPa unidirectional, J formulas for thin-wall sections) and CM = −0.12; they should be replaced by measured values.
- SLSQP and Powell are local optimisers from a few starting shapes; differences under about 3% are noise.
- FM uses the full disk area π R1², not the annulus.

**Open issues to resolve:**

1. **Spin rate.** The paper reports stable spinning hover only up to about 8–10 rad/s, but this design assumes 20 rad/s (4–10 m/s at r = 0.2–0.5 m). Yaw-rate authority at 20 rad/s has not been demonstrated.
2. **Number of blades and the vehicle weight.** The design above is for 2 blades. At 382 g predicted aero lift (maybe 190–270 g real) it augments a 500 g vehicle but cannot carry it. Three blades give about 60% more lift. Needs the actual vehicle mass including blades.
3. **Is the wing driven or passive?** If driven by its own motor, torque (about 1.5 N·m for 30 W at 20 rad/s) and motor losses at low rpm matter. In the paper the wing is spun by canted motors, so wing drag torque is paid by motor yaw power.
4. **As-built incidence** of the existing paper wings (section 11).
5. **Rod details**: position in the chord, solid or tube, ply layup, joint design between rod and ribs.
6. **CFD.** OpenFOAM (free) can do the job. Use the `kOmegaSSTLM` transition model for the Re 40–100k range (more important than the choice of code), one wing with periodic boundaries to cut cost by 4×, steady MRF, WSL2 or Docker on Windows. SU2 is an alternative. Check 2D (XFoil or 2D CFD) on the faired rod section first. CFD only helps if the geometry (as-built pitch included) matches what is built. The paper's CFD gave 4.5 N while the hardware gave 2.1 N.

**Suggested next steps:**

1. Fix twist (rod at 35–40% chord, stiffer bare section, locked sections, optionally a D-box), then measure GJ.
2. Build the GOE210 blade to the schedule in section 1.
3. Run the load-cell spin test at several ω and compare thrust with the BEM scaling.
4. Re-run the design with the real vehicle mass, blade count and rod stiffness.

---

## 16. File index and how to rerun

All paths are in `C:\Users\anike\Desktop\RESEARCH\foils`. Use the folder's own Python: `v\Scripts\python.exe`.

| File | Purpose |
|---|---|
| `chord_velocity_opt.py` | 2D sweep for 15 foils (section 4.1) → `results/chord_velocity_opt.csv` |
| `verify_xfoil.py` | XFoil cross-check (section 4.2) → `results/xfoil_verify.csv` |
| `make_report_plots.py` | Plots 1–4 in `results/plots/` |
| `blade_design.py` | BEM model and thrust-targeted optimiser (section 4.3, 4.4a). Slow (about an hour for all foils). Run for one foil with `blade_design.py goe210`. → `results/blade_design_<foil>.csv` |
| `plot_blade.py` | Plot 5 (thrust-targeted blade distributions) |
| `design_max_ld.py` | Best-CL/CD design with optimised chord (section 4.4b); run for example `design_max_ld.py goe210` → `results/_mld_<foil>.pkl` |
| `plot_design.py` | Plots 6 and 7 and `results/design_max_ld_summary.csv` |
| `goe210_tables.py` | GOE210 tables and plots 8 and 9 (section 8) |
| `robust_design.py` | 15–20 rad/s robustness check and band-optimised comparison (section 8.3). Note: it has no `__main__` guard, so importing it reruns the optimisation. |
| `paper_check.py` | Paper wing check (section 4.6, 11) |
| `torsion_check.py`, `bare_rod_twist.py` | Torsion estimates (section 10) |
| `tools\xf\xfoil.exe` | XFoil 6.99 (official MIT download) |
| `REPORT.md`, `BLADE_DESIGN.md` | Earlier, shorter reports; this file supersedes them where they differ |

Plots: `results/plots/1_LD_vs_chord.png`, `2_LD_vs_velocity.png`, `3_optimal_alpha_CL.png`, `4_xfoil_vs_neuralfoil.png`, `5_blade_distributions.png`, `6_spanwise_design.png`, `7_totals_per_foil.png`, `8_goe210_design.png`, `9_goe210_profile.png`.

To change one input and see the effect: edit `ct` (chords) or the pitch loop in `paper_check.py`; edit `CASES` or `FOILS` in `verify_xfoil.py`; change `Ω`, `R0`, `R1`, `B` and `CMAX` at the top of `blade_design.py` (and the foil dictionary in `design_max_ld.py`).
