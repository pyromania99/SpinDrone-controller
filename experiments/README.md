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
| [`trajectory_level1_circle_seat`](trajectory_level1_circle_seat/) | 2026-09-18 | L1, the same R=10 m 5 m/s circle levels 2–3 fly, plus the validated R=3 m slow circle | **The seat improves circle tracking at level 1 at both speeds** — following distance 2.39→1.38 m slow, 7.54→4.62 m fast; altitude hold 45× tighter on the slow circle. Both arms fly. |
| [`trajectory_level1_seat_b_geometry_sweep`](trajectory_level1_seat_b_geometry_sweep/) | 2026-09-13 | L1, five trajectory geometries, 25 rad/s, `rotor_tau=0.02` | No standing directional bias at ~1 m/s; a real speed-scaled one at 6.4 m/s. θ_b converges correctly everywhere and **never moves tracking outside noise**. |
| [`hover_level2_px4_groundtruth`](hover_level2_px4_groundtruth/) | 2026-09-17 | L2 hover, r≈29.5, 2×2 over {lag, seat} | Seat off crashes at 12.1 s; seat on reaches 22.1 s — **not rescued**. θ_s tracks the lag cleanly to r≈28.6 then rails at r≈29.2. |
| [`hover_level3_px4_estimator`](hover_level3_px4_estimator/) | 2026-09-17 | L3 hover, same 2×2 | **The seat rescues hover: 12.8 s → the full 46.7 s.** Same condition as level 2, opposite outcome. |
| [`trajectory_level2_px4_groundtruth_fast_circle`](trajectory_level2_px4_groundtruth_fast_circle/) | 2026-09-13, re-flown 2026-09-18 | L2, R=10 m 5 m/s circle, ~20.7 rad/s | **The 20 ms-lag "rescue" reproduces 1 run in 4** — see below. θ_s finds PX4's own pipeline lag (−0.40 rad) even with no rotor lag. |
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

## The level-2 circle "rescue" is a coin flip (2026-09-18)

**`trajectory_level2_.../LEVEL2_TRACKING.md` reports the clearest rescue in this tree —
seat off 7.8 s, seat on 48.7 s on the `rotor_tau=0.02` circle. Flying that seat-on arm four
more times on the identical command produced:**

| run | outcome | θ_s in cruise |
|---|---|---|
| original, 2026-09-13 | flew 48.7 s | −0.5051 |
| re-fly 1 | **crash 19.9 s** | −0.8484 |
| re-fly 2 | flew 52.2 s, no crash | −0.5046 |
| re-fly 3 | **crash 22.4 s** | −0.5611 |
| re-fly 4 | **crash 24.6 s** | −0.5355 |

**One run in four survives.** The condition is bimodal, not a reliable rescue, and the
single run the document was built on is the minority outcome. The two that flew both
converged to θ_s = −0.505; the three that crashed went to −0.54, −0.56 and −0.85, so the
split may be which fixed point the adaptation finds — untested.

The seat-off arm at the same condition reproduces exactly (16.5 s → 16.4 s), so the
*direction* of the effect is not in doubt; its size and reliability are. See
[`l2_lag20ms_repeats_compare.png`](trajectory_level2_px4_groundtruth_fast_circle/l2_lag20ms_repeats_compare.png),
which plots all four.

**Nothing else in the tree moved this way.** The other eleven re-flown arms reproduced their
outcome, most within a few tenths of a second of their crash time.

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
| `*_compare.png` | 8 panels — altitude, \|p,q\|, **attitude error**, tilt (flown vs commanded), position error, torque, spin + θ_s. *Did it fly, and how did it fail?* |
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

### α is measured on every arm as of 2026-09-18

The `*_seat.png` α panel used to be empty for every seat-OFF and seat-FIXED arm, and the
panel claimed the gaps were a frozen allocator gate. They were not — α had never been
computed. Both controllers logged the *law's* α, which is written only on a tick that
adapts, and neither ran the observer with the seat off. Both now measure it unconditionally
and log `seat_stepped` separately for the gate. The affected arms were re-flown; **level 1
came back bit-for-bit identical**, and the level-2/3 arms moved only by run-to-run scatter,
with one exception recorded in
[the level-2 circle RUN_LOG](trajectory_level2_px4_groundtruth_fast_circle/RUN_LOG.md) — the
seat-OFF double dip on the `nolag` circle **did not reproduce.** Details in
[`tools/README.md`](tools/README.md).

### The same figures, tiled for comparison

[`analysis/`](analysis/) holds the standard figure set laid out for comparison — **the
rungs as the columns, at the lowest delay each one has** — one sheet per view, built by
[`tools/make_analysis_grids.py`](tools/make_analysis_grids.py) from the committed PNGs
themselves. [`analysis/hover/`](analysis/hover/) puts levels 1, 2 and 3 side by side;
level 1 belongs there because its 20 ms rotor pole and the PX4 rungs' `nolag` pipeline lag
come to the same phase at this spin (0.533 rad vs ~0.50 rad).
[`analysis/circle/`](analysis/circle/) is levels 2 and 3 only — level 1 never flew that
circle. `--all-conditions` adds wider grids over every lag and spin condition. Each
folder's `README.md` carries the outcome table, what the sheets are good for, and the
confounds that make particular comparisons invalid — the circle columns are at r ≈ 20.7 and
r ≈ 25, so rung and spin are entangled there; the hover columns are not. Nothing there is a
new flight.


### Tilt is not the attitude error on a trajectory (corrected 2026-09-18)

The compare figure's tilt panel was labelled "attitude error". In hover that is fair, since
the commanded tilt is ~0. **On the circle it is not**: the vehicle is asked to lean ~21° to
make its lateral acceleration, so tilt is dominated by the command and reading it as error
overstates the error about fivefold.

| | commanded tilt | flown tilt | attitude error |
|---|---|---|---|
| level 3 circle | 21.11° | 19.52° | **3.78°** |
| level 2 hover | 0.06° | 0.12° | **0.17°** |
| level 1 hover | 0.375° | 0.299° | **0.471°** |

The figures now carry both: the tilt panel plots flown against commanded, and **the panel
that used to hold a `|p,q|` zoom now holds the attitude error** — the angle between the
commanded body z and the flown one, taken between axes so the deliberate yaw spin does not
count toward it. It comes from `vehicle_attitude_setpoint.q_d` at levels 2–3 and from
`desired_angles` at level 1.

**Any tilt figure quoted as tracking quality needs re-reading** — the cruise tables in the
circle documents report flown tilt, which is correct as tilt and is not an error.

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

- **`--max-lateral-accel` defaults to OFF at level 1, and every level-1 trajectory result
  depends on it.** `fly.py` passes it only when given; `EigenController`'s default is `0.0`,
  which the code reads as *no clamp*. Omitting it on the fast circle turns a clean 65 s
  flight into a crash 13 s in, for **both** seat arms — and produced, on 2026-09-18, a
  thoroughly convincing but entirely spurious "the seat is harmful at level 1 on circles",
  complete with a monotonic gain sweep. See
  [`trajectory_level1_circle_seat/RUN_LOG.md`](trajectory_level1_circle_seat/RUN_LOG.md).
  **If a level-1 trajectory result says the seat hurts, check the clamp first.**
- **Level 1 has no transport delay.** The only actuator lag is the first-order rotor pole
  (`--rotor-tau`). Levels 2/3 add PX4's own ~17 ms transport + ~14 ms pipeline lag on top.
  **Do not compare θ_s values across levels without accounting for this.**
- **`nolag` at levels 2 and 3 does not mean zero lag.** It means `rotor_tau = 0` — no rotor
  pole. The PX4 pipeline above is still there, so read those runs as **~17 ms of loop
  delay** and the `lag20ms` runs as **~37 ms**. The figure names are kept as they are; only
  the reading changes. [`analysis/`](analysis/) labels every column both ways.
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

Every experiment folder also carries a `logs/` directory holding **the flight data of each
run it uses** — the `.ulg` at levels 2 and 3, the `.npz` at level 1 — plus that run's
`run_config.json` and a `console.txt`: the Isaac + PX4 console reduced to the ~25 lines that
say what was flown. The raw consoles are not kept; they are almost entirely carb/GPU/ROS
boot noise, and each `console.txt` header counts what was dropped. The table below still
names where the originals live.

| experiment | raw data | status |
|---|---|---|
| level 1, archived figures | `examples/results/controller_eval/eigen/statistics/eigen_statistics.npz` | **Overwritten by every subsequent level-1 run.** None of the original level-1 logs exist. Each `RUN_LOG.md` carries the commands to regenerate. |
| level 1, km6e7 re-flight | `examples/results/controller_eval/eigen/statistics/km6e7_refly/eigen_{off,fixed,adaptive}.npz` | **Intact** — copied out after each arm, which is the practice the overwrite problem demands. |
| level 2 | `px4_runs/L2_eigen_20260913_*/log/*/*.ulg` | **Intact.** 21 valid post-GT-fix runs; the four used are mapped in that folder's `RUN_LOG.md`. |
| level 3 (and its level-2 comparisons) | `px4_runs/L{2,3}_*_20260914_*/log/*/*.ulg` | **Intact**, ~30 MB each. Mapped run-by-run in that folder's `RUN_LOG.md`. |
| archived b-vs-lag sweep | `examples/results/b_vs_tau/mm_6.0e-07/b_vs_tau.csv` | Intact. |

`px4_runs/`, `examples/results/` and anything named `logs/` or `*.log` are gitignored, so
none of that raw data is in the repository *except* the per-experiment `logs/` copies, which
an explicit un-ignore at the bottom of `.gitignore` keeps. **That is ~760 MB of ulog and
npz** — the price of each folder standing on its own. If cloning ever becomes painful, these
are the files to move to git-lfs; nothing else in the tree is large.

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
