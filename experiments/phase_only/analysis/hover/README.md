# hover — levels 1, 2 and 3 at ~17–20 ms

Climb and hold, no trajectory, r ≈ 29.5. Columns are the rung; the single row is the
condition. Read [`../README.md`](../README.md) first for what `nolag` means — **it is
~17 ms, not 0 ms**, and that is what makes level 1 belong on the same sheet.

| sheet | the question it answers in one image |
|---|---|
| [`compare_by_level.png`](compare_by_level.png) | did it fly, and how did it fail? |
| [`attitude_by_level.png`](attitude_by_level.png) | what shape is the attitude? |
| [`seat_by_level.png`](seat_by_level.png) | what did θ_s converge to? |
| [`trajectory_by_level.png`](trajectory_by_level.png) | how well did it hold station? (levels 2–3 — level 1 has no station-keeping view) |
| [`attitude_adaptive_by_level.png`](attitude_adaptive_by_level.png) | what does the seat-on arm look like alone? |

Sources: [`../../hover_level1_km6e7_seat_off_fixed_adaptive/`](../../hover_level1_km6e7_seat_off_fixed_adaptive/)
(the **2026-09-17 re-flight**, not the archived 2026-09-10 set),
[`../../hover_level2_px4_groundtruth/`](../../hover_level2_px4_groundtruth/) and
[`../../hover_level3_px4_estimator/`](../../hover_level3_px4_estimator/) — runs
runs `L2_airframe_20260917_234829` / `_235022` and `L3_eigen_20260917_235215` / `_235407`
(the 2026-09-18 re-flight), each pair flown back to back with **only `MC_CTRL_GT`
different**.

---

## Why level 1 is on the same sheet

Level 1 has no `nolag` arm — it has no PX4, so there is nothing to switch off. What it has
is a 20 ms rotor pole, and at this spin that is **the same phase as the PX4 rungs' pipeline
lag to within about 6 %**:

| | mechanism | phase at r = 29.5 |
|---|---|---|
| level 1, `rotor_tau = 0.02` | first-order rotor pole | atan(29.5 × 0.020) = **0.533 rad** |
| levels 2–3, `nolag` | PX4 transport ~17 ms (+ ~14 ms pipeline) | 29.5 × 0.017 = **~0.50 rad** |

**Same size, different mechanism.** A first-order pole is not a pure transport delay — it
rolls off amplitude as well as shifting phase — so the columns are comparable in how much
phase the seat has to cancel, not identical in what it is cancelling.

## What the three columns show

| | level 1 (no PX4) | level 2 (ground truth) | level 3 (estimator) |
|---|---|---|---|
| arms | OFF / **FIXED at ψ_model** / ADAPTIVE | OFF / ON | OFF / ON |
| seat OFF | departs 24.0 s, then re-departs repeatedly | flies | flies |
| seat FIXED | **crashes ~4.4 s** — worst of the three | not flown | not flown |
| seat ON (adaptive) | flies the full 120 s | flies | flies |
| θ_s converged | **0.067** | **−0.860** | **−0.782** |
| alt sd | — | 1.6 mm | 7.3 mm |
| circular-mean α, seat OFF (R) | −0.60 (0.93) | **+1.04 (0.92)** | **+1.08 (0.84)** |
| hold altitude | 1.5 m, 120 s | 6 m, 60 s | 6 m, 60 s |

The level-2/3 numbers are from the **2026-09-18 re-flight** that added α on every arm. θ_s
moved 5 % at level 2 (−0.906 → −0.860) and 9 % at level 3 (−0.870 → −0.782) purely as
run-to-run scatter; everything else held. See each experiment's RUN_LOG.

1. **At this delay the PX4 rungs are unstressed — and the seat still converges.** Both arms
   fly at levels 2 and 3, so nothing is being rescued. Yet θ_s settles to −0.860 and −0.782
   with **no rotor pole anywhere in the model**. That is the adaptation finding PX4's own
   pipeline unaided.
2. **The two PX4 rungs agree to 9 % on θ_s** — and run-to-run scatter on a single arm is 5 %,
   so most of that gap is scatter rather than the estimator.
3. **α is now on the seat-OFF arm, and in hover it does not say what you would hope.**
   These are circular statistics; R is the resultant length (1 = concentrated, 0 = uniform),
   and a mean direction at low R is not a direction:

   | | circular-mean α | R |
   |---|---|---|
   | L2 seat OFF | **+1.04** | 0.92 |
   | L2 seat ON | **−2.94** | 0.37 |
   | L3 seat OFF | **+1.08** | 0.84 |
   | L3 seat ON | **−2.84** | 0.37 |

   With the seat on the misalignment does **not** shrink toward zero — it moves to near ±π,
   roughly anti-aligned, and concentrates less than half as well. **The circle sheet is
   where α behaves as intended** (0.35 → −0.13 at R ≈ 0.8). Hover is the starved-channel
   case `Seat.adapt`'s docstring describes, and CROSS converges on a magnitude-weighted
   average rather than by driving α to zero — but that is the design rationale, not a
   measurement, and nothing here tests whether it explains these two columns.
4. **Level 1 is where the seat is load-bearing, because level 1 is the only column under
   stress.** Its OFF arm departs and its FIXED arm — holding the *model* angle ψ_model =
   0.533 rad — dies fastest of all three. Only the adapted 0.067 rad flies. The model is
   available, exact, and still wrong.
5. **Level 3 is ~5× noisier than level 2 on the same profile** (alt sd 7.3 mm vs 1.6 mm).
   That is the estimator in the loop, and it is the expected direction, not worse control.

## Traps

- **α on a seat-ON arm is not a residual.** Both implementations build the observer's
  commanded vector from the torque **before** the seat rotates it
  (`xd = tau/I` precedes `seat.rotate`; PX4's `Vector2f xd(...)` precedes `_seat.apply`).
  So α on a seat-ON arm measures the misalignment the seat is correcting, not what is left
  after it, and "the seat should drive α to zero" is not a claim these traces can settle.
- **Do not compare θ_s across the level-1 boundary.** 0.067 against −0.78 is not a 12×
  disagreement about the same quantity; whether the Python law and the PX4 implementation
  define θ_s with the same sign and scale is **not established anywhere in this tree**, and
  nothing on this sheet tests it. Compare within a rung.
- **The columns are not the same flight profile.** Level 1 holds 1.5 m for 120 s, levels 2
  and 3 hold 6 m for 60 s. Time axes and altitude axes do not line up.
- **Level 1 carries three arms, the PX4 rungs carry two.** The third colour on the left has
  no counterpart to its right.
- **Level 1's numbers are from the re-flight, and the archived 2026-09-10 figures disagree
  with it.** The re-flight matches the code in the tree; do not mix in an archived number.
- **Survival is not success.** Nothing crashes in the two right-hand columns, so read the
  attitude panels, not the altitude ones.
- **Everything is scored on ground truth, including level 3** — scoring the estimator
  against its own estimate would be circular, and `vehicle_attitude` was unusable above
  ~20 rad/s before the EKF2 output-predictor fix.

## What this sheet cannot tell you

**The interesting disagreement is at the other condition, which is not shown.** At
`rotor_tau = 0.02` and the same spin, levels 2 and 3 still agree with the seat off (crash
12.1 s vs 12.8 s) but diverge completely with it on — level 2 crashes at 22.1 s, level 3
flies the full 46.7 s. Build that with:

```bash
python3 experiments/tools/make_analysis_grids.py --all-conditions
```
