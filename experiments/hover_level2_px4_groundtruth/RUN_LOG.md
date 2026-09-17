# Run log — hover, level 2, PX4 + ground truth

**Date flown:** 2026-09-17 (this session).
**Findings document:** [`LEVEL2_HOVER.md`](LEVEL2_HOVER.md).
**Companion:** [`../hover_level3_px4_estimator/`](../hover_level3_px4_estimator/) — the
identical batch at level 3, and [`../hover_level1_km6e7_seat_off_fixed_adaptive/`](../hover_level1_km6e7_seat_off_fixed_adaptive/)
at level 1.

Flown specifically to fill the hover row at levels 2 and 3 — **no post-ground-truth-fix
hover runs existed**. All 21 valid level-2 runs from 2026-09-13 and the 2026-09-14 runs fly
the fast circle; none hovers.

---

## Condition

| | |
|---|---|
| level | 2 — PX4 + Pegasus on ground truth (`MC_CTRL_GT=1`) |
| controller | eigen (`MC_CTRL_ALG=3`) |
| profile | **climb to 6 m and hold — no trajectory.** This is what "hover" means at these rungs. |
| `--hold` | 60 s |
| `--motor-moment` | `6e-7` → r ≈ **29.5 rad/s** measured — chosen to match the level-1 km6e7 hover |
| `--rotor-tau` | `0.020` and `0` |
| seat | `MC_SEAT_MODE` 0 (off) and 2 (adaptive) |
| `MC_POLE_EN` | 0 throughout |
| `IMU_DGYRO_CUTOFF` | 60 |

## The 2×2

| run directory | `rotor_tau` | seat | outcome |
|---|---|---|---|
| `px4_runs/L2_airframe_20260917_034108` | 0.020 | off | **crash 12.1 s** |
| `px4_runs/L2_airframe_20260917_034255` | 0.020 | adaptive | **crash 22.1 s** |
| `px4_runs/L2_airframe_20260917_034444` | 0 | off | flies, no crash |
| `px4_runs/L2_airframe_20260917_034631` | 0 | adaptive | flies, no crash |

**The `airframe` in the directory name is cosmetic.** `fly.py` tags the run directory from
`--ctrl-alg`, which was not passed on this batch; `MC_CTRL_ALG=3` was set via `--gains` and
is confirmed present in every ulog. These are eigen runs.

**`MC_SEAT_MODE` is absent from the seat-off ulogs, and that is correct** — PX4 does not log
a parameter left at its default, and 0 is the default. Absent means off.

## Command

```bash
for TAU in 0.020 0; do for SEAT in 0 2; do
  python3 examples/flight/fly.py --level 2 --controller eigen \
      --motor-moment 6e-7 --rotor-tau "$TAU" --altitude 6 --hold 60 --headless \
      --gains MC_CTRL_ALG=3 MC_CTRL_GT=1 "MC_SEAT_MODE=$SEAT" \
              MC_POLE_EN=0 IMU_DGYRO_CUTOFF=60
done; done
```

Sequential, one sim at a time, ~1.8 min per run. Level 3 is the same with `MC_CTRL_GT=0`.

## Figures

```bash
python3 experiments/tools/plot_run.py \
    --arm "seat OFF:px4_runs/L2_airframe_20260917_034108" \
    --arm "seat ON (adaptive):px4_runs/L2_airframe_20260917_034255" \
    --out experiments/hover_level2_px4_groundtruth --prefix l2_hover_lag20ms \
    --zoom 0 25 --settle-from 13 \
    --title "Level 2 hover (PX4 + ground truth), km=6e-7 (r~29.5), rotor_tau 20 ms"
# ... and the same for the rotor_tau=0 pair with --prefix l2_hover_nolag --settle-from 20
```

Figures are trimmed to the flight: pre-takeoff and post-crash data is not plotted. Crossing
counts are computed **before** trimming and stay truthful.

## How the steady-state numbers were taken

Window = from 5 s after first reaching 4 m, to 1 s before the crash (or 3 s before the last
sample if there is none). For the `rotor_tau=0.02` seat-on arm that window is only 13–21 s,
which is why its numbers carry the departure and should not be read as cruise values.

| quantity | source |
|---|---|
| altitude, xy drift | `vehicle_local_position_groundtruth` |
| tilt | `arccos(1 − 2(q₁² + q₂²))` from `vehicle_attitude_groundtruth` |
| \|p,q\|, spin | `vehicle_angular_velocity_groundtruth` |
| θ_s | `mc_controller_status.seat_theta` |

## Known confound in this batch

**Spin rate is not matched to the fast-circle experiments.** This batch runs r ≈ 29.5
(km = 6e-7, chosen to match level 1's hover); the circle experiments run r ≈ 20.7 (level 2)
and r ≈ 25 (level 3). A separate batch at km = 4.0816e-7 was flown to separate "hover" from
"higher spin" — see the findings document.

---

## Flight logs, in this folder

Every run listed above now has **its ulog** in [`logs/<run>/`](logs/), beside a
`run_config.json` and a `console.txt` — the Isaac + PX4 console distilled to the ~25 lines
that say what was flown. The raw `sim.log` is not kept: it is ~75 KB of carb/GPU/ROS boot
noise and repeated MAVLink heartbeat warnings, and the counts of what was dropped are in
each `console.txt` header. This experiment no longer depends on `px4_runs/` surviving.

[`logs/README.md`](logs/README.md) maps each run to its arm and gives the snippet for
reading the parameters (`MC_SEAT_*`, `MC_POLE_*`) back out of a ulog — they are recorded
nowhere else.
