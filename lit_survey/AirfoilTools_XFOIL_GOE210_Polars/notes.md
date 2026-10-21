# XFOIL polars for GOE 210 (DAIMLER) — Airfoil Tools

Re 5e4 / 1e5 / 2e5 / 5e5 / 1e6, Ncrit 9, Mach 0. Fetched 2026-10-15 from
<http://airfoiltools.com/airfoil/details?airfoil=goe210-il>.

## Why it matters here

This is the **first independent check on the aerofoil data** behind the rotating-wing
model. Until now the section polar in
`extensions/.../aerodynamics/airfoil_polar.py::goe210_polar()` was fitted to the station
table in `foil_out/GOE210_DESIGN_REPORT.md`, which was itself produced with NeuralFoil —
a model fitted to a model, with nothing outside the repository to check it against.

XFOIL is an independent code using different physics (panel method with an integral
boundary layer and an e^N transition criterion, against NeuralFoil's neural surrogate).
Agreement between them is meaningful.

**The coordinates were checked first**: `data/goe210_coords.dat` matches the repository's
own `foil_out/database/goe210.dat` exactly, so these polars are for the same section the
blade was designed around and not a different Göttingen profile.

### Result: the fitted polar validates, inside its operating range

| Re | XFOIL max L/D | model max L/D | XFOIL best α | model best α |
|---|---|---|---|---|
| 50k | 40.7 | 44.7 (+10 %) | 5.50° | 5.62° (+0.12°) |
| 100k | 62.0 | 62.5 (**+1 %**) | 4.75° | 4.62° (−0.13°) |
| 200k | 82.3 | 88.6 (+8 %) | 3.75° | 3.88° (+0.13°) |

Section values at the operating points: **Cl within 4–5 %, Cd within 1–5 %.**

The **best lift-to-drag angle agrees to within 0.13° across the whole operating range.**
That is the quantity a development bug had wrong by 2–5° (see `physics_tests/README.md`,
bug 4), and it was corrected by reasoning about the drag bucket's Reynolds behaviour
rather than against data. This dataset confirms the correction independently.

Integrated at the design point, three independent routes agree:

| | thrust | power | FM |
|---|---|---|---|
| fitted polar (ships) | 3.701 N | 7.00 W | 0.741 |
| XFOIL polar | 3.591 N | 6.73 W | 0.736 |
| design report (NeuralFoil BEMT) | 3.740 N | 7.31 W | 0.722 |

Settled spin rate 18.44 rad/s (fitted) vs 18.61 (XFOIL) — under 1 % apart.

### Limits of this validation

- **XFOIL is computation, not measurement.** No wind-tunnel data for GOE 210 could be
  found; searches of the Göttingen *Ergebnisse* volumes and the NACA/NASA archives turned
  up nothing for this section. The agreement above is between two codes, not against
  reality.
- **Ncrit = 9 is a clean-tunnel transition assumption.** A real moulded or printed blade
  is rougher; Ncrit 5–7 would be more representative and would lower L/D.
- **Above Re 2e5 the fitted polar diverges badly** — +36 % at 5e5 and +78 % at 1e6 —
  because it was calibrated only over the blade's own range. The blade reaches Re 235k at
  most, so this does not affect results, but the fitted polar must not be reused outside
  Re 5e4–2e5.
- **Stall angle differs**: XFOIL stalls at 10.2–12.8° against the model's 9.0°, and XFOIL's
  Cl_max is 1.32–1.51. The model is the conservative one. This matters only in manoeuvre,
  where the blade goes past stall.

### Why this dataset was fetched

A file named `GOE_210_Polars_Re_1e5_to_1e6.csv` was briefly added here as GOE 210 polar
data. It was language-model output, not data, and has been **deleted**. These XFOIL
polars were fetched to replace it with something traceable, and they settle what the
fabricated file got wrong: it gave max L/D of 15.2 at Re 1e5 where XFOIL gives **62.0**,
a factor-of-four error, and placed the optimum at 6.5° against XFOIL's 4.75°.

The episode is the argument for `provenance.yaml`: every number in this directory can be
traced to a URL, a method and a date.
