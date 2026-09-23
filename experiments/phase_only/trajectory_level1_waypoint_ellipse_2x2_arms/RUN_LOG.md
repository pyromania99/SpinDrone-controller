# Run log — trajectory, level 1, waypoint ellipse, 2×2 over {seat, adaptive b}

**Date flown:** 2026-09-13 (figures written 01:48).
**Findings document:** [`WAYPOINT_ELLIPSE_ARMS.md`](WAYPOINT_ELLIPSE_ARMS.md).
**Commit the surrounding work landed in:** `8c48c10`.

---

## Condition

| | |
|---|---|
| level | 1 — Pegasus + Python eigen law |
| controller | `eigen` |
| spin | **25 rad/s** → `--motor-moment 5.102e-7` (r_ss ≈ 4.9e7·km) |
| `--rotor-tau` | `0.020` |
| trajectory | `examples/results/ellipse_waypoints.csv` — **4 laps × 16 waypoints** (69 rows) |
| `--waypoint-radius` | `0.35` m — arrival-based advance, **not** the clock |
| duration | 110 s |
| altitude | 1.5 m |
| scoring window | `t > 25 s` (excludes climb + first lap transient) |

## The four arms

```bash
COMMON="--level 1 --controller eigen --motor-moment 5.102e-7 --failure-mode off \
        --rotor-tau 0.020 --duration 110 \
        --trajectory examples/results/ellipse_waypoints.csv --waypoint-radius 0.35"

# neither
python3 examples/flight/fly.py $COMMON --seat off

# b only, no seat
python3 examples/flight/fly.py $COMMON --seat off --adapt-b \
    --pole-gains law=extremum k_es=0.3 dither_amp=0.05 dither_hz=0.4 a_min=0.3 theta_b_max=0.6

# seat only
python3 examples/flight/fly.py $COMMON --seat adaptive

# seat + b
python3 examples/flight/fly.py $COMMON --seat adaptive --adapt-b \
    --pole-gains law=extremum k_es=0.3 dither_amp=0.05 dither_hz=0.4 a_min=0.3 theta_b_max=0.6
```

`--adapt-b` is independent of `--seat`, which is what makes the `b only` cell possible —
pair it with `--seat off`.

**Reconstructed, not transcribed.** These lines are rebuilt from the recorded condition
(spin, lag, trajectory, waypoint radius, duration) and the b-law gains quoted in the
companion document's header. The verbatim invocations were not captured — the runs were
launched from a subshell that did not write to `~/.bash_history`. The `--seat-gains` used
for the adaptive arms is likewise unrecorded; `Seat`'s defaults are `k=2.0, theta_max=1.4,
r_min=5.0, law=cross, k_sign=+1`.

**The b-law gain window is narrow and was established by failure:** `k_es=2.0` with
`dither_amp=0.12` railed θ_b and crashed; `k_es=0.3 / dither_amp=0.05 / theta_b_max=0.6`
flew. Do not widen these casually.

## Raw data — GONE

All four arms wrote to
`examples/results/controller_eval/eigen/statistics/eigen_statistics.npz`, **each
overwriting the last**, and the 2026-09-13 trajectory work then overwrote that. Nothing
survives but the figures.

To redo this properly: copy the npz to a per-arm filename immediately after each run.

### Fields used by these figures

| figure | fields |
|---|---|
| `arms_altitude.png` | `time`, `p[:,2]` |
| `arms_position_error.png` | `time`, `norm(ep[:,:2], axis=1)` |
| `arms_xy_path.png` | `p[:,0:2]` vs. the CSV's waypoints, masked to `t > 25` |
| `arms_thetab_ellipse.png` | `time`, `pole_theta`; band from the archived sweep |

Spin rate for the headline table is `yaw_rate`; θ_b statistics are over `pole_theta` in the
scoring window.

## Inputs

- `examples/results/ellipse_waypoints.csv` — 69 rows, `t,x,y,z`, first point
  `(2.30, −1.50, 1.50)`. **Generated for this test**, not one of the repo's archived
  trajectories.
- **Repo trajectory CSVs in `examples/tools/trajectories/` are stored in reverse time
  order** (t descends) and are not evenly spaced in arc length. Using one as-is makes the
  controller hold position for the trajectory's duration. Resample by cumulative arc
  length before use.

## Why arrival-based advance, not a timed reference

A timed CSV marches the reference on its own clock; under spin the vehicle falls behind and
the error compounds without bound. Measured on this profile: the climb held to 5 mm, then
the timed translate put the vehicle on the ground by t = 20 s with 3.2 m of error. Arrival
-based advance caps the error at the acceptance radius by construction, and the same flight
then held 110 s. `--waypoint-radius 0` restores the old timed behaviour.

## Figures

| file | what it is |
|---|---|
| `arms_altitude.png` | altitude vs. time, all four arms — **read this first** |
| `arms_position_error.png` | \|position error\| vs. time, all four arms, scoring window marked |
| `arms_xy_path.png` | flown vs. reference path, `t > 25 s`, `neither` and `seat only` only |
| `arms_thetab_ellipse.png` | θ_b vs. time for the two b arms, against the archived predicted band |
