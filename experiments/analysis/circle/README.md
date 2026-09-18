# circle — levels 1, 2 and 3 on the same path

R = 10 m, 5 m/s, altitude 6 m. Columns are the rung; the single row is the condition —
the lowest delay each rung has (levels 2–3 `rotor_tau = 0`; level 1 a 20 ms rotor pole and
no PX4). Read [`../README.md`](../README.md) first for what `nolag` means — **it is
~17 ms, not 0 ms.**

| sheet | the question it answers in one image |
|---|---|
| [`compare_by_level.png`](compare_by_level.png) | did it fly, and how did it fail? |
| [`attitude_by_level.png`](attitude_by_level.png) | what shape is the attitude? |
| [`seat_by_level.png`](seat_by_level.png) | what did θ_s converge to? |
| [`trajectory_by_level.png`](trajectory_by_level.png) | did it actually track the circle? |
| [`attitude_adaptive_by_level.png`](attitude_adaptive_by_level.png) | what does the seat-on arm look like alone? |

Sources: [`../../trajectory_level2_px4_groundtruth_fast_circle/`](../../trajectory_level2_px4_groundtruth_fast_circle/),
[`../../trajectory_level3_px4_estimator_fast_circle/`](../../trajectory_level3_px4_estimator_fast_circle/).

---

## Read the column confound first

**Spin is not matched between the columns: level 2 flew this circle at r ≈ 20.7 and level 3
at r ≈ 25.** Unlike the hover pair — flown back to back with only `MC_CTRL_GT` different —
every level-2/level-3 difference here confounds the rung with the spin rate, and spin alone
is known to flip outcomes in this regime. Use [`../hover/`](../hover/) for the clean rung
comparison.

**Level 1's cell is the only one where the seat is measured on tracking rather than
survival** — both its arms fly, so it answers a different question from the cells beside it.
Its numbers: following distance 7.54 → 4.62 m and altitude hold ±473 → ±118 mm with the
seat on.

**Level 3 has three arms and level 2 has two.** Level 3 adds `seat + adaptive b`, so its
third colour has no counterpart on the left.

**Level 1 now has a column** — it flew this exact path on 2026-09-18, at r ≈ 25 with a
20 ms rotor pole and the outer-loop acceleration clamp at 3.0. Both its arms fly, so its
cell is a tracking comparison rather than a survival one:
[`../../trajectory_level1_circle_seat/`](../../trajectory_level1_circle_seat/). Its older
trajectory work — a waypoint ellipse and a five-geometry sweep — uses different panels and
stays out of these sheets.

## What the pair shows

Both columns are from the **2026-09-18 re-flight** that added α on every arm.

| | level 2, r ≈ 20.7 | level 3, r ≈ 25 |
|---|---|---|
| seat OFF | up 52.1 s, 2 crossings, never dips | up 51.6 s, 1 crossing |
| seat ON | up 52.0 s, 2 crossings | up 52.2 s |
| seat + adaptive b | not flown | up 52.3 s, θ_s **−0.457** |
| θ_s, seat ON | **−0.401** | **−0.505** |
| circular-mean α, OFF → ON | **+0.35 → −0.13** | **+0.45 → −0.13** |
| its resultant length R | 0.76 → 0.80 | 0.82 → 0.82 |

1. **This is where α behaves as the law intends, and the sheet can finally show it.** The
   seat-off arms carry a standing misalignment of **+0.35 rad** (level 2) and **+0.45 rad**
   (level 3); with the seat on both land near **−0.13 rad**, and the resultant length stays
   at R ≈ 0.8 throughout, so these are real directions and not averaged noise. Before 2026-09-18 the seat-off
   trace did not exist and this comparison could not be made at all. It is the opposite of
   what [`../hover/`](../hover/) shows, and the difference is excitation — the circle drives
   the channel, hover starves it.
2. **θ_s is non-zero at both rungs with no rotor pole in the model** (−0.401 and −0.505).
   The seat is finding PX4's own pipeline on a moving trajectory, not just in hover.
3. **Adding adaptive b changes θ_s by 0.05 rad and tracking by less than the run-to-run
   noise.** The seat is load-bearing; b is a refinement on top of it.

## What this pair cannot tell you

**The condition that fails is not on these sheets.** At `rotor_tau = 0.02`, level 3's arms
*both* die — the open item of the whole project — and level 2's seat-ON arm was written up
as the clearest rescue in the tree, 7.8 s → 48.7 s. **That rescue reproduces once in four.**
Flying it four more times on the identical command gave crash 19.9 s, flew 52.2 s, crash
22.4 s, crash 24.6 s; the seat-OFF arm reproduces exactly, so the direction holds and the
reliability does not. The repeats are plotted in the experiment folder as
`l2_lag20ms_repeats_compare.png`. Build the ~37 ms cells with:

```bash
python3 experiments/tools/make_analysis_grids.py --all-conditions
```

## Traps

- **A documented result did not reproduce here.** The previous level-2 seat-OFF run dipped
  to 0.11 m twice (4 altitude crossings), and that was written up as the seat-off signature
  at this condition. The re-flight of the same command **stayed above 4.02 m throughout, 2
  crossings**. The dip belonged to that run, not to the condition. Old ulog:
  `px4_runs/L2_eigen_20260913_200501`.
- **α on a seat-ON arm is not a residual.** Both implementations build the observer's
  commanded vector from the torque **before** the seat rotates it, so α on a seat-ON arm is
  the misalignment the seat is correcting, not what is left after it.
- **Duration cannot tell a flight from a bounce sequence.** Count crossings; the plotting
  tool prints them and the tile captions quote them. This has already produced one wrong
  result in this project.
- **Everything is scored on ground truth, including level 3** — scoring the estimator
  against its own estimate would be circular.
