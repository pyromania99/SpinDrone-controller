# Experiments — seat (θ_s) and adaptive pole angle (θ_b) under sustained spin

One folder per experiment. Each folder holds **the figures, a findings document, and a
`RUN_LOG.md`** recording how the flights were produced and where the raw data is.

Folder naming: `{profile}_{level}_{what varied}`, where profile is `hover` or
`trajectory` and level is the simulation rung:

| level | what closes the loop |
|---|---|
| 1 | Pegasus + the Python eigen law. No PX4, no estimator. |
| 2 | PX4 + Pegasus on simulator ground truth (`MC_CTRL_GT=1`). |
| 3 | PX4 + Pegasus on PX4's own estimator (`MC_CTRL_GT=0`). **The hardware-relevant rung.** |

---

## The experiments

| folder | date | condition | the one-line result |
|---|---|---|---|
| [`hover_level1_km6e7_seat_off_fixed_adaptive`](hover_level1_km6e7_seat_off_fixed_adaptive/) | 2026-09-10/11 | L1 hover, r ≈ 29.5 rad/s, `rotor_tau=0.02` | Seat off crashes at 13.7 s, **model-computed** seat crashes at 5.3 s, **adaptive** seat flies 120 s. |
| [`trajectory_level1_waypoint_ellipse_2x2_arms`](trajectory_level1_waypoint_ellipse_2x2_arms/) | 2026-09-13 | L1 waypoint ellipse, 25 rad/s, `rotor_tau=0.02` | 2×2 over {seat, b}: **the seat is load-bearing (−36.4 % tracking error), b is a 0.7-point refinement on top.** b alone still crashes. |
| [`trajectory_level1_seat_b_geometry_sweep`](trajectory_level1_seat_b_geometry_sweep/) | 2026-09-13 | L1, five trajectory geometries, 25 rad/s, `rotor_tau=0.02` | No standing directional bias at ~1 m/s; a real speed-scaled one at 6.4 m/s. θ_b converges correctly everywhere and **never moves tracking outside noise**. |
| [`hover_level2_px4_groundtruth`](hover_level2_px4_groundtruth/) | 2026-09-17 | L2 hover, r≈29.5, 2×2 over {lag, seat} | Seat off crashes at 12.1 s; seat on reaches 22.1 s — **not rescued**. θ_s tracks the lag cleanly to r≈28.6 then rails at r≈29.2. |
| [`hover_level3_px4_estimator`](hover_level3_px4_estimator/) | 2026-09-17 | L3 hover, same 2×2 | **The seat rescues hover: 12.8 s → the full 46.7 s.** Same condition as level 2, opposite outcome. |
| [`trajectory_level2_px4_groundtruth_fast_circle`](trajectory_level2_px4_groundtruth_fast_circle/) | 2026-09-13 | L2, R=10 m 5 m/s circle, ~20.7 rad/s | **The seat rescues level 2 at 20 ms lag: 7.8 s → 48.7 s airborne.** θ_s finds PX4's own pipeline lag (−0.41 rad) even with no rotor lag. |
| [`trajectory_level3_px4_estimator_fast_circle`](trajectory_level3_px4_estimator_fast_circle/) | 2026-09-14 | L3, R=10 m 5 m/s circle, ~25 rad/s | Level 3 works post-EKF2-fix. **L2 and L3 diverge reproducibly** at the seat-off boundary. Neither arm survives `rotor_tau=0.02` yet — the open item. |
| [`supporting_b_pole_theory_and_archive`](supporting_b_pole_theory_and_archive/) | — | not a flight | The analytic reason the original θ_b law can't work, and the archived static-b sweep that supplies the "predicted band" the other folders plot against. |

## The level-2 / level-3 divergence

The hover pair was flown back to back with **only `MC_CTRL_GT` different**, which the
circle experiments could not do (they ran level 2 at r = 20.7 and level 3 at r = 25, so rung
and spin were confounded):

| hover, r ≈ 29.5, `rotor_tau = 0.02` | seat OFF | seat ON |
|---|---|---|
| level 2 (ground truth) | crash 12.1 s | **crash 22.1 s** |
| level 3 (estimator) | crash 12.8 s | **flew 46.7 s** |

The seat-off arms agree; the seat-on arms do not. This is the same divergence
[level 3 §3](trajectory_level3_px4_estimator_fast_circle/LEVEL3_TRACKING.md) found on the
circle with the seat off, and it has the sign that section's explanation predicts — a small
spin-synchronous phase offset in the estimator acting as an inadvertent partial seat.
**Hardware always runs level-3-like**, so the level-2 failure is an artifact of an idealised
scaffold. The confirming experiment (inject the phase offset into ground truth and see if
the outcome flips) is still unrun.

## Reading order

1. **`hover_level1_...`** — the cleanest demonstration that the seat does something, on the
   simplest possible profile. Start here.
2. **`trajectory_level1_waypoint_ellipse_2x2_arms`** — the factorial that separates the
   seat's contribution from b's. This is the attribution result.
3. **`trajectory_level1_seat_b_geometry_sweep`** — what the tracking actually looks like
   across five geometries, and the measurement traps hit getting there.
4. **`trajectory_level2_...`** then **`trajectory_level3_...`** — the same questions on PX4,
   first on ground truth, then on the estimator (the rung that resembles hardware).
5. **`supporting_b_pole_theory_and_archive`** — background for the θ_b half of all of the above.

## The standard figure set

Levels 2 and 3 carry the same four views, produced by
[`tools/plot_run.py`](tools/plot_run.py) so they are directly comparable:

| figure | what it answers |
|---|---|
| `*_compare.png` | 8 panels — altitude, \|p,q\|, tilt, position error, torque, spin + θ_s. *Did it fly, and how did it fail?* |
| `*_attitude.png` | roll / pitch / yaw / roll–pitch phase portrait. *What shape was the failure?* |
| `*_seat.png` | θ_s alone, with α and θ_b. *What did the adaptation converge to?* |
| `*_trajectory.png` | flown vs commanded path, following distance, speed, altitude. *Did it track?* |

All three levels now have all four. Level 1's set exists twice: the archived
`km6e7_*` figures from 2026-09-10, and `km6e7_refly_*` from the 2026-09-17 re-flight.
**They disagree, and the re-flown set is the one that matches the current code** — see
[level 1's re-flight section](hover_level1_km6e7_seat_off_fixed_adaptive/KM6E7_LEVEL1.md#re-flight-2026-09-17--read-this-before-quoting-any-number-above).

Figures are trimmed to the flight by default: data before takeoff and after a terminal loss
of altitude is not plotted, because ground contact produces values an order of magnitude
larger than anything in flight and sets the axis limits of every panel. `--no-crash-cut`
keeps it, for the cases where the ground-contact behaviour *is* the evidence (counting
bounces at level 3 §4). **A dip that recovers is not a crash** — the cut requires a terminal
loss, after an earlier version deleted a whole level-2 flight over a recovered transient.

### A cross-level result that only appeared once the levels were plotted alike

The **roll–pitch phase portrait is an annulus at every level** — a near-constant
*world-frame* tilt with the body spinning underneath it. Only the radius changes, and the
radius is just the commanded tilt: **0.3° in level-1 hover, ~19° on the level-2 and level-3
circles.** It is the same physical picture at three rungs.

**θ_s converges to well under atan(r·T) everywhere — between an eighth and a third of it.**
An earlier version of this file claimed a constant ~30 % across levels; that was tested by
re-flying level 1 on 2026-09-17 and **withdrawn**:

| | θ_s | atan(r·T) | ratio |
|---|---|---|---|
| L1 archived 2026-09-10 (pixel-read, superseded build) | 0.157 | 0.533 | 0.295 |
| **L1 re-flown 2026-09-17 (measured)** | **0.0672** | 0.533 | **0.126** |
| **L2 lag increment (measured)** | **0.118** | 0.392 | **0.301** |

The apparent agreement was between a pixel-read number from an older controller build and a
real one. What survives is the direction and rough size of the shortfall, not a constant —
and the fact that *holding* the model value outright is worse than no seat at all.

---

## Cross-cutting facts worth knowing before reading any of them

- **Level 1 has no transport delay.** The only actuator lag is the first-order rotor pole
  (`--rotor-tau`). Levels 2/3 add PX4's own ~17 ms transport + ~14 ms pipeline lag on top.
  **Do not compare θ_s values across levels without accounting for this.**
- **`ψ_model = r·tau_a + atan(r·T)` is not the angle that flies.** Measured at level 1:
  model 0.533 rad, converged 0.157 rad. Holding the model value is *worse than no seat*.
- **Every level-2 result dated before 2026-09-13 is void** — the ground-truth position fed
  to the controller was 57.2958× too small (radians/degrees). See the project memory note.
- **`vehicle_attitude` was unusable above ~20 rad/s** until the EKF2 output-predictor fix
  (`notes/EKF2_OUTPUT_PREDICTOR_SPIN_FIX.md`). Analysis before that fix must use
  `vehicle_attitude_groundtruth`.
- **"trajectory complete" from the PX4 driver does not mean the vehicle flew.** It streams
  setpoints on a timer with no crash detection. Confirm altitude from
  `vehicle_local_position_groundtruth`.
- **A first/last-time-above-threshold "airborne duration" cannot tell a stable flight from
  a bounce sequence.** Count threshold crossings.

---

## Raw data, and what survives

| experiment | raw data | status |
|---|---|---|
| level 1, archived figures | `examples/results/controller_eval/eigen/statistics/eigen_statistics.npz` | **Overwritten by every subsequent level-1 run.** None of the original level-1 logs exist. Each `RUN_LOG.md` carries the commands to regenerate. |
| level 1, km6e7 re-flight | `examples/results/controller_eval/eigen/statistics/km6e7_refly/eigen_{off,fixed,adaptive}.npz` | **Intact** — copied out after each arm, which is the practice the overwrite problem demands. |
| level 2 | `px4_runs/L2_eigen_20260913_*/log/*/*.ulg` | **Intact.** 21 valid post-GT-fix runs; the four used are mapped in that folder's `RUN_LOG.md`. |
| level 3 (and its level-2 comparisons) | `px4_runs/L{2,3}_*_20260914_*/log/*/*.ulg` | **Intact**, ~30 MB each. Mapped run-by-run in that folder's `RUN_LOG.md`. |
| archived b-vs-lag sweep | `examples/results/b_vs_tau/mm_6.0e-07/b_vs_tau.csv` | Intact. |

**If you re-run anything at level 1, copy the npz out immediately** — the next run destroys
it. This is the single most annoying property of this setup and it has already cost every
log in three of these five folders.

---

## Related documents outside this tree

- `notes/EKF2_OUTPUT_PREDICTOR_SPIN_FIX.md` — why level 3 became usable at all.
- `notes/FINDINGS.md` — the running project log, including the archived sweep's writeup.
- `control_doc.md` — the control-law derivations.
- `examples/utils/seat.py` — `Seat`, `WrenchObserver`, `PoleAdapter`. The docstrings carry
  the design rationale and several of the measured numbers quoted in these documents.
