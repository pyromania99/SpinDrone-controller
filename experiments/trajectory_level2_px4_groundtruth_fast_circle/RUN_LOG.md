# Run log — trajectory, level 2, PX4 + ground truth, fast circle

**Dates flown:** 2026-09-13, 20:05 – 20:13 (the two pairs used here).
**Figures generated:** 2026-09-17, from the surviving ulogs, with
[`../tools/plot_run.py`](../tools/plot_run.py).
**Findings document:** [`LEVEL2_TRACKING.md`](LEVEL2_TRACKING.md).

**The raw data is intact.** Unlike every level-1 experiment in this tree, these figures were
made from the original logs, not reconstructed.

---

## Condition, common to every run

| | |
|---|---|
| level | 2 — PX4 + Pegasus on **simulator ground truth** (`MC_CTRL_GT=1`) |
| controller | eigen (`MC_CTRL_ALG=3`) |
| `--motor-moment` | `4.0816e-7` → r ≈ **20.7 rad/s** measured |
| trajectory | `examples/results/fast_circle_l2.csv` — R = 10 m, 5 m/s, altitude 6 m, 2501 rows @ 20 ms, cosine ramp on the orbital rate |
| `IMU_DGYRO_CUTOFF` | **60 Hz** (the PX4 default of 20 Hz adds ~0.33 rad of its own phase at a 3.3 Hz spin, which the seat would read as actuator lag) |
| `MC_POLE_EN` | 0 — adaptive pole disabled in all four runs |
| airframe | `none_iris`, physics 250 Hz |

## Run → figure map

| run directory | `rotor_tau` | `MC_SEAT_MODE` | outcome | figures |
|---|---|---|---|---|
| `px4_runs/L2_eigen_20260913_201119` | **0.02** | 0 (off) | **7.8 s above 4 m**, 2 crossings, never reached cruise | `l2_lag20ms_*` |
| `px4_runs/L2_eigen_20260913_201321` | **0.02** | 2 (adaptive) | **48.7 s**, 2 crossings, 3+ orbits | `l2_lag20ms_*` |
| `px4_runs/L2_eigen_20260913_200501` | 0 | 0 (off) | 48.8 s, **4 crossings** (dipped twice) | `l2_nolag_*` |
| `px4_runs/L2_eigen_20260913_200711` | 0 | 2 (adaptive) | 48.8 s, 2 crossings | `l2_nolag_*` |

Parameters were recovered from each ulog (`initial_parameters` overlaid with
`changed_parameters` — the driver sets most of them over MAVLink *after* boot, so reading
`initial_parameters` alone is misleading).

## Regenerating the figures

```bash
python3 experiments/tools/plot_run.py \
    --arm "seat OFF:px4_runs/L2_eigen_20260913_201119" \
    --arm "seat ON (adaptive):px4_runs/L2_eigen_20260913_201321" \
    --out experiments/trajectory_level2_px4_groundtruth_fast_circle \
    --prefix l2_lag20ms --zoom 0 25 --settle-from 25 \
    --title "Level 2 (PX4 + ground truth), fast circle, r=20 rad/s, rotor_tau 20 ms"

python3 experiments/tools/plot_run.py \
    --arm "seat OFF:px4_runs/L2_eigen_20260913_200501" \
    --arm "seat ON (adaptive):px4_runs/L2_eigen_20260913_200711" \
    --out experiments/trajectory_level2_px4_groundtruth_fast_circle \
    --prefix l2_nolag --zoom 0 25 --settle-from 25 \
    --title "Level 2, same circle, rotor_tau 0 (PX4 pipeline lag only)"
```

Deterministic — re-running reproduces the committed PNGs.

## Flight commands

```bash
python3 examples/flight/fly.py --level 2 --controller eigen \
    --motor-moment 4.0816e-7 --rotor-tau 0.020 \
    --gains MC_CTRL_ALG=3 MC_CTRL_GT=1 MC_SEAT_MODE=2 IMU_DGYRO_CUTOFF=60 \
    --l2-trajectory examples/results/fast_circle_l2.csv
```

`MC_SEAT_MODE=0` for the off arm, `--rotor-tau 0` for the no-lag pair.
**Reconstructed from the ulog parameter sets**, not transcribed from shell history — the
*parameter values* are read directly from the logs and are exact, but the trajectory flag
spelling and the hold/altitude arguments were not captured. Check `fly.py --help` against
the current tree before reusing.

## How the numbers in the findings document were computed

Cruise window = `ground_speed > 5 m/s`, then trimmed by +3 s at the start (to drop the
spiral-out) and −1 s at the end.

| quantity | how |
|---|---|
| orbit radius | **about the fitted centre** `(mean x, mean y)` over the cruise window — *not* the origin; this reference is centred near (0, 10) |
| altitude sd | sd of `-vehicle_local_position_groundtruth.z` over the cruise window |
| tilt | `arccos(1 − 2(q₁² + q₂²))` from `vehicle_attitude_groundtruth` — frame-independent, which matters when yaw is deliberately spinning |
| \|p,q\| | `hypot` of `vehicle_angular_velocity_groundtruth.xyz[0:2]` |
| θ_s, α | `mc_controller_status.seat_theta`, `.seat_alpha` |
| following distance | GT position vs `trajectory_setpoint`, interpolated, blanked while commanded altitude < 1 m |
| airborne / crossings | first and last sample above 4 m, plus the number of threshold crossings |

Ground truth is used for all attitude and position analysis. At level 2 that is also what
the controller flew on, so there is no estimator question here — but keeping the same
convention as level 3 makes the two directly comparable.

## Runs deliberately not used

`px4_runs/L2_eigen_20260913_{063149,063431,063828,064211,064659,064956,065508,065914,070206,070701,074410,191504,195804,210350,210801,211110,211337}`
and the four `L2_eigen_20260914_*` runs are all valid post-fix level-2 flights, but they fly
other trajectories, other spin rates, or are exploratory repeats. Two of them were checked
as candidate corroboration for §3's altitude result and fly a ~2 m radius path, not this
circle — which is why that result is flagged as unreplicated.

The `L2_eigen_20260914_203427/203618/203810` triple *is* a clean seat-off / seat-on /
seat+b comparison at km = 5.102e-7, and is the level-2 half of level 3's §6. It is plotted
in the level-3 folder rather than duplicated here.
