# Blade as a rotor: r = 0.2–0.5 m, 20 rad/s, c ≤ 0.30 m

**Model:** hover BEM (no axial inflow, 2 blades, Prandtl tip loss, ρ = 1.2) using NeuralFoil polar tables
(Re 8k–400k, NCrit 9). Chord (5 nodes, ≤ 0.30 m) and pitch (5 nodes) optimised for minimum power at a fixed
thrust per blade. Scripts: `blade_design.py`, `plot_blade.py`. Results: `results/blade_design_*.csv`.

![Blade distributions](results/plots/5_blade_distributions.png)

## Answer to "full chord at the root has diminishing returns"

Correct for low thrust, and the effect is strong. Per-blade lift scales as c·V², and the root sees V = 4 m/s versus 10 m/s at the tip
(6× less dynamic pressure). The optimiser only fills the 0.30 m chord near the root when the thrust demand is high.

| AS6091, power per blade | T = 1 N | T = 2 N |
|---|---|---|
| Optimised chord + pitch | **1.40 W**, area 0.038 m² | **3.89 W**, area 0.070 m² |
| Constant 0.30 m (pitch optimised) | 1.64 W, area 0.090 m² | 4.02 W, area 0.090 m² |
| Linear taper 0.30→0.12 m | 1.50 W | 3.99 W |
| c ∝ 1/r (0.30 at root) | 1.48 W | 4.23 W (poor) |

(Thrust per blade; 2 blades → total thrust is double. Max useful thrust with 0.30 m everywhere is ≈ 2.8 N per blade.)

- **T = 1 N/blade:** chord 230 mm at the root tapering to ~100 mm at r = 0.4 m. 15 % less power than a constant 0.30 m blade with 58 % less area.
- **T = 2 N/blade:** the blade stays at 0.30 m out to r ≈ 0.3 m, then tapers (220 mm at 0.4 m, ~140 mm at 0.45 m).
  Differences between chord laws are only 3–4 %, so any sensible taper works.
- Chord is *not* what limits efficiency; matching alpha and twist is.

## What to build (AS6091, T ≈ 1–2 N per blade)

| r (m) | chord (mm) T=1 / T=2 | pitch (deg) T=1 / T=2 | alpha (deg) | CL | Re (k) | L/D |
|---|---|---|---|---|---|---|
| 0.20 | 232 / 300 | 22 / 31 | 4.4 / 8.0 | 1.05 / 1.34 | 65 / 87 | 58 / 35 |
| 0.30 | 152 / 297 | 16 / 21 | 4.5 / 4.5 | 1.06 / 1.05 | 62 / 124 | 57 / 80 |
| 0.40 | 96 / 221 | 14 / 17 | 5.0 / 3.5 | 1.10 / 0.96 | 52 / 121 | 51 / 80 |
| 0.45 | 66 / 138 | 12.5 / 16 | 4.9 / 4.2 | 1.08 / 1.03 | 40 / 84 | 40 / 67 |

- **Twist:** about 12–15° from root to tip (pitch measured from the rotation plane, includes ~inflow angle). Strong twist is the main efficiency lever.
- **Sections** run at CL ≈ 1.0–1.1 and alpha ≈ 4–5°, close to the 2D max L/D point found earlier.
- At T = 2 the root (alpha 8°, CL 1.3) is near the edge of the polar; give it a thicker foil or reduce root pitch.
- **Tip:** the optimiser shrinks chord to its 30 mm bound over the last ~10 %; that comes from the tip-loss model. In practice use a rounded tip with chord ≥ ~60–80 mm.
- Constant-Re logic: Re stays 40–135k along the span, well above the 20k region where AS6091 collapses.

## Caveats

- Assumes **static thrust (no axial/forward speed)**. A climbing/forward-flight rotor needs a different inflow angle, hence different twist.
- Thrust per blade (and blade count) is not fixed by you yet; pick the thrust, then use the matching row.
- AS6091 is ~4 % thick (≈ 9 mm at 0.23 m, ≈ 3 mm at 0.08 m): check bending stiffness and centrifugal loads. AS6093 (6 %) gave 1.42 / 3.91 W,
  nearly identical to AS6091's 1.40 / 3.89 W, so it is the safer structural choice. GOE210 gave 1.43 / 3.92 W.
- SLSQP is a local optimiser from two starting shapes; treat differences below ~3 % as noise. Not checked in XFoil/XFLR5 yet.
