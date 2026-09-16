# Run log — trajectory, level 1, seat + adaptive b across five geometries

**Date flown:** 2026-09-13, 01:48 – 04:53.
**Findings document:** [`TRAJECTORY_TRACKING.md`](TRAJECTORY_TRACKING.md).
**Commit the work landed in:** `8c48c10` — *"Fix the 57.3x ground-truth position bug, add
waypoint/smooth trajectory tracking, and document the PX4 estimator spin fix."*

---

## Condition, common to every run

| | |
|---|---|
| level | 1 — Pegasus + Python eigen law |
| controller | `eigen` |
| spin | **25 rad/s** → `--motor-moment 5.102e-7` |
| `--rotor-tau` | `0.020` throughout |
| seat | `--seat adaptive` (CROSS law) on every arm |
| altitude | 1.5 m |
| lateral clamp | `--max-lateral-accel` — **added during this work**; see Traps |

Adaptive-b arms add:

```
--adapt-b --pole-gains law=extremum k_es=0.3 dither_amp=0.05 dither_hz=0.4 \
          a_min=0.3 theta_b_max=0.6
```

## The five geometries

| § | geometry | trajectory CSV | advance mode | figures |
|---|---|---|---|---|
| 1 | straight line, 40 m, 21 waypoints 2 m apart | `straight_line.csv` (21 rows) | `--waypoint-radius 0.35` | `traj_straight_line_40m.png`, `traj_thetab_convergence.png` |
| 2 | single distant goal, 30 m, swept over speed | `far_goal.csv` (2 rows) | n/a — one goal | `traj_misalignment_vs_speed.png` |
| 3 | ellipse, 16 evenly arc-spaced waypoints/lap | `ellipse_even.csv` (65 rows = 4×16+1) | `--waypoint-radius 0.35` | `traj_ellipse_corner_cutting.png`, `traj_ellipse_thetab_speed.png`, `traj_heading_vs_desired.png`, `traj_heading_zoom_3wp.png` |
| 4 | the same ellipse at 5× scale (~6.4 m spacing) | `ellipse_5x.csv` (33 rows = 2×16+1) | `--waypoint-radius 0.35` | `traj_5x_lap.png`, `traj_5x_zoom3.png`, `traj_5x_thetab_speed.png` |
| 5 | smooth continuous circle, R=3 m, 1.2 m/s | `smooth_circle.csv` (2757 rows @ 20 ms) | **timed**, no waypoints | `traj_smooth_circle_path.png`, `traj_smooth_circle_misalign.png`, `traj_smooth_circle_zoom10s.png` |

All CSVs are in `examples/results/`. They were generated for these tests.

## Commands

```bash
COMMON="--level 1 --controller eigen --motor-moment 5.102e-7 --failure-mode off \
        --rotor-tau 0.020 --seat adaptive --max-lateral-accel 3.0"

# §1 straight line (seat only, and again with --adapt-b ...)
python3 examples/flight/fly.py $COMMON \
    --trajectory examples/results/straight_line.csv --waypoint-radius 0.35 --duration 80

# §3 ellipse / §4 5x ellipse — same shape, swap the CSV
python3 examples/flight/fly.py $COMMON \
    --trajectory examples/results/ellipse_even.csv --waypoint-radius 0.35 --duration 110

# §5 smooth circle — TIMED reference, so NO --waypoint-radius
python3 examples/flight/fly.py $COMMON \
    --trajectory examples/results/smooth_circle.csv --duration 60
```

**Reconstructed, not transcribed** — same caveat as the other level-1 folders. Durations
are inferred from the figures' time axes. The `--max-lateral-accel` value used is not
recorded; 3.0 m/s² is the value the findings document names as the clamp the §5 circle sits
inside (0.48 m/s² demanded), so it is an upper bound on what was set, not a confirmed value.

## Raw data — GONE

Same single-path overwrite problem as every level-1 experiment:
`examples/results/controller_eval/eigen/statistics/eigen_statistics.npz`. The file there
now is the last run of this session, not any particular arm.

### Fields these figures use

| quantity | field |
|---|---|
| flown path | `p[:,0:2]` |
| reference | the CSV, or `desired_p[:,0:2]` |
| ground speed | differenced `p`, or the backend's velocity |
| heading | `desired_angles[:,2]` (measured yaw), unwrapped |
| desired direction | for §5, the **analytic** circle tangent — not direction-to-aim-point |
| misalignment | heading − desired direction, wrapped to ±180° |
| θ_b | `pole_theta` |
| waypoint acceptances | reference index changes in `desired_p` |

## Figures not cited in the original write-up

Two figures in this folder were produced with the others but never referenced in
`TRAJECTORY_TRACKING.md`'s original text. Both have since been added to it as supplementary
figures:

- **`traj_thetab_convergence.png`** (§1) — θ_b and ground speed along the straight line.
  θ_b climbs from 0 into the archived predicted band over ~40 s, with the waypoint-weave
  ripple riding on it. Ground speed cycles 0.5–1.45 m/s once per 2 m leg.
- **`traj_heading_vs_desired.png`** (§3) — three stacked panels: θ_b, a tick per waypoint
  acceptance, and misalignment. This is the figure behind §3's parenthetical
  *"misalignment lines up with each waypoint acceptance and decays before the next: mean
  +2.2°, sd 26°"* — every excursion is aligned with a tick.

## Traps hit building these runs (the full list is in the findings document)

- **Time-based references run away under spin.** Use `--waypoint-radius` for discrete
  points, or keep a timed reference's demanded acceleration inside `--max-lateral-accel`.
- **Clamping position *error* breaks the D term** — it flattens `d(error)/dt` and removes
  damping (measured: 8.6 m/s on a leg meant to be flown at 1.2 m/s). Clamp the
  *acceleration output*, scaled as a vector so direction is preserved.
- **A shrinking position-error vector is ill-conditioned.** Near waypoint arrival the
  desired direction is noise. Gate on distance-to-target, not just speed.
- **Repo trajectory CSVs are stored in reverse time order** and are not arc-length even.
