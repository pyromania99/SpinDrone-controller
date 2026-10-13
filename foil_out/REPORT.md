# Airfoil optimum over 5–10 m/s, chord ≤ 0.30 m

**Method:** NeuralFoil (xlarge) polars for 15 candidate foils at V = 5–10 m/s, chord 0.05–0.30 m,
Re = V·c/ν with ν = 1.5e-5 m²/s, NCrit 9 (calm) and 5 (disturbed). Top foils cross-checked in XFoil 6.99.
Max L/D below is the average of NCrit 9 and 5 unless noted.

## Recommendation

- **Chord: use the full 0.30 m.** L/D rises with chord for every foil (higher Re) and never peaks.
  0.20 m still gives ~90 % of the 0.30 m L/D for the top foils.
- **Foil: AS6091** – best at every velocity and chord tested.
  Runners-up: AS6092, then GOE210 (close to AS6091 at 10 m/s).
- **Angle of attack:** ~4° at 5 m/s easing to ~3° at 10 m/s (a fixed ~3.5° stays near-optimal across the range).

| Foil | Mean L/D, c = 0.30 m | Mean L/D, c = 0.10 m |
|---|---|---|
| as6091 | 81.8 | 49.8 |
| as6092 | 78.2 | 48.3 |
| goe210 | 75.6 | 41.4 |
| hq308 | 73.7 | 40.7 |
| goe57 | 73.7 | 41.1 |
| as6093 | 73.6 | 46.7 |

S1010, S1012, S2048, S2050, S3021 are clearly worse (L/D 34–65 at 0.30 m).

## AS6091 operating point, chord 0.30 m

| V (m/s) | alpha (deg) | CL | CD | L/D |
|---|---|---|---|---|
| 5 | 4.0 | 1.00 | 0.014 | 72 |
| 6 | 4.0 | 1.00 | 0.013 | 77 |
| 7 | 3.9 | 0.99 | 0.012 | 81 |
| 8 | 3.3 | 0.93 | 0.011 | 84 |
| 9 | 3.3 | 0.93 | 0.011 | 87 |
| 10 | 3.1 | 0.92 | 0.010 | 89 |

## Plots

![L/D vs chord](results/plots/1_LD_vs_chord.png)
![L/D vs velocity](results/plots/2_LD_vs_velocity.png)
![Optimal alpha and CL](results/plots/3_optimal_alpha_CL.png)
![XFoil vs NeuralFoil](results/plots/4_xfoil_vs_neuralfoil.png)

## Verification (XFoil 6.99)

36 cases (AS6091, AS6092, GOE210 × V 5/7/10 × chord 0.15/0.30 × NCrit 9/5), all converged.
Median L/D difference 1.1 %; 35/36 within 15 % (worst of those 5 %). Optimal alpha agrees to ~0.5°, CL to ~0.05.
AS6091 stays best in XFoil. At 10 m/s, c = 0.30 m, XFoil gives L/D 94 (calm) / 80 (rough) versus NeuralFoil 97 / 81 –
so quote ~80–90, not 97.

One outlier: GOE210, 7 m/s, c = 0.30 m, NCrit 9 – XFoil returned L/D 564 at 1.75° (spurious near-zero-drag point);
NeuralFoil's 76 at 4.5° is the value to trust.

## Caveats

- At small chord (0.05 m, Re 17–33 k) L/D collapses (AS6091 drops to ~18 in rough air); thick foils suffer most.
- Predictions only; XFoil is not a substitute for tunnel data at Re < 100 k.
- 2D section results; no finite-span / tip losses.

## Reproduce

```
v\Scripts\python.exe chord_velocity_opt.py     # sweep -> results/chord_velocity_opt.csv
v\Scripts\python.exe verify_xfoil.py           # XFoil check -> results/xfoil_verify.csv
v\Scripts\python.exe make_report_plots.py      # plots -> results/plots/
```
