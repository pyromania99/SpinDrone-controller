# Run log — trajectory, level 3, PX4 + estimator, fast circle

**Date flown:** 2026-09-14, 19:10 – 21:47 (figures written 21:54).
**Findings document:** [`LEVEL3_TRACKING.md`](LEVEL3_TRACKING.md).

**This is the only experiment in `experiments/` whose raw data still exists.** All ulogs
are intact under `px4_runs/`, ~30 MB each.

---

## Condition, common to every run

| | |
|---|---|
| level | 3 — PX4 + Pegasus, **estimator in the loop** (`MC_CTRL_GT=0`) |
| `--motor-moment` | `5.102e-7` → `expected_spin_rad_s` 24.9998; actual 20–26 rad/s by run |
| trajectory | `examples/results/fast_circle_l2.csv` — R = 10 m, 5 m/s, altitude 6 m, 2501 rows @ 20 ms |
| start | cosine ramp on the **orbital rate**, so there is neither a position nor an acceleration step |
| `IMU_DGYRO_CUTOFF` | **60 Hz** on every run (PX4 default 20 Hz contributes ~0.33 rad of its own phase at a 4.6 Hz spin, which the seat would read as actuator lag) |
| `duration_s` | 270 (process cap; flights are ~50 s) |
| airframe | `none_iris`, physics 250 Hz |

**Every run here post-dates the EKF2 output-predictor spin fix**
(`notes/EKF2_OUTPUT_PREDICTOR_SPIN_FIX.md`). Before it, `vehicle_attitude` reported 86°
mean tilt where truth was 18.7°, and level 3 was unusable.

---

## Run → figure/section map

Recovered from each ulog's parameter set (`initial_parameters` overlaid with
`changed_parameters`, since the driver sets most of these over MAVLink after boot).

| run directory | `rotor_tau` | `MC_CTRL_ALG` | `MC_CTRL_GT` | `MC_SEAT_MODE` | `MC_POLE_EN` | used for |
|---|---|---|---|---|---|---|
| `L3_eigen_20260914_203001` | 0 | 3 (eigen) | 0 | **2** (adaptive) | **2** (extremum, `KES=0.3`, `MAX=0.6`) | **§1 headline + §2 zoom — both figures in this folder** |
| `L3_eigen_20260914_202618` | 0 | 3 | 0 | 0 (off) | 0 | §3 level-3 seat-off — flew 50.1 s |
| `L3_eigen_20260914_202810` | 0 | 3 | 0 | 2 | — | §3 level-3 seat-on |
| `L3_eigen_20260914_212400` | 0 | 3 | 0 | — (off) | — | §3 seat-off **repeat** (reproducibility check) |
| `L2_eigen_20260914_212209` | 0 | 3 | **1** (ground truth) | — (off) | — | §3 level-2 comparison — **crashed at t = 17 s** |
| `L3_eigen_20260914_214502` | **0.02** | 3 | 0 | — (off) | — | §4 — crashes ~16 s then bounces 6× |
| `L3_eigen_20260914_214653` | **0.02** | 3 | 0 | 2 | — | §4 — crashes once, cleanly, ~20–24 s |
| `L3_cascaded_pd_20260914_214031` | 0 | **1** (cascaded PD) | 0 | n/a | n/a | §5 baseline — crashed at 16.6 s |
| `L2_eigen_20260914_203427` | 0 | 3 | 1 | — (off) | — | §6 level-2 seat-off |
| `L2_eigen_20260914_203618` | 0 | 3 | 1 | 2 | — | §6 level-2 seat-only |
| `L2_eigen_20260914_203810` | 0 | 3 | 1 | 2 | **2** (`KES=0.3`) | §6 level-2 seat + b |
| `L3_eigen_20260914_191009`, `..._194849` | 0 | 3 | 0 | 2 | — | earlier exploratory runs, not used in the document |

Paths are relative to the repository root, e.g.
`px4_runs/L3_eigen_20260914_203001/log/15_54_15.ulg` (the inner directory is a date stamp;
glob it).

Each run directory also holds `run_config.json` (motor moment, rotor_tau, duration) and
`sim.log` (Isaac + PX4 console). **`sim.log` does not record the seat/pole parameters** —
those are only in the ulog.

## Extracting the parameters from a ulog

```python
import glob, pyulog
u = pyulog.ULog(glob.glob("px4_runs/L3_eigen_20260914_203001/log/*/*.ulg")[0],
                message_name_filter_list=[])          # params only, fast
params = dict(u.initial_parameters)
for ts, name, val in u.changed_parameters:            # the driver sets these post-boot
    params[name] = val
```

Reading `initial_parameters` alone is misleading — most runs boot with the seat disabled
and are configured afterwards.

## Commands

```bash
python3 examples/flight/fly.py --level 3 --controller eigen \
    --motor-moment 5.102e-7 --rotor-tau 0 \
    --gains MC_CTRL_ALG=3 MC_SEAT_MODE=2 MC_POLE_EN=2 MC_POLE_KES=0.3 MC_POLE_MAX=0.6 \
            IMU_DGYRO_CUTOFF=60 \
    --l2-trajectory examples/results/fast_circle_l2.csv
```

**Reconstructed from the ulog parameter sets**, not transcribed from history. The exact
trajectory flag spelling and the hold/altitude arguments were not captured; check
`fly.py --help` against the current tree before reusing. The *parameter values* in the
table above are read directly from the logs and are exact.

## Topics the figures are built from

| panel | uLog topic / field |
|---|---|
| θ_b, θ_seat | `mc_controller_status` |
| ground speed, position | `vehicle_local_position_groundtruth` |
| reference | `trajectory_setpoint` |
| attitude | `vehicle_attitude` (valid **only** post-EKF2-fix; else `vehicle_attitude_groundtruth`) |
| heading misalignment | measured heading vs. the **analytic circle tangent** (`φ + 90°` from the known centre) |

## Analysis traps specific to these logs

- **A garbage timestamp (`≈1.8e19` µs = `UINT64_MAX`) appears in `mc_controller_status` and
  `trajectory_setpoint`**, making the raw time array non-monotonic. `np.interp` requires
  sorted x and returns silent garbage (exact zeros in one case) if it isn't. **Filter to a
  sane time range and re-sort before interpolating either topic.**
- **Do not compute desired heading as direction-to-aim-point** on a laggy curved reference —
  a chord across an arc does not point along the arc's tangent. Use the analytic tangent;
  the gap is ~27° here. Differentiating the raw setpoint stream instead adds noise
  (sd 17° vs 0.4°) without removing the bias.
- **`MC_CTRL_GT=1` vs `0` changes the outcome deterministically** at this condition. Both
  outcomes reproduced exactly on repeat. Do not treat a level-2 result as a level-3
  prediction.
- **Airborne duration by first/last time above 5 m cannot tell a flight from a bounce
  sequence.** This produced a wrong "seat off flew 41.6 s" in an earlier draft of §4; the
  real trace has 12 altitude crossings. Count crossings.

## Figures

| file | what it is |
|---|---|
| `l3_combined_panels.png` | 5 stacked panels, settled cruise: θ_b, θ_seat, ground speed, distance from reference, heading misalignment |
| `l3_zoom_response.png` | t = 10–18 s of the same flight — heading vs. desired, and following distance |

---

## Addendum, 2026-09-17 — the standard figure set

`l3_nolag_*` and `l3_lag20ms_*` were generated from the same ulogs with
[`../tools/plot_run.py`](../tools/plot_run.py), so level 3 carries the same four views as
levels 1 and 2. The original `l3_combined_panels.png` / `l3_zoom_response.png` are kept —
they answer the trajectory-lag question, which the standard set does not.

```bash
python3 experiments/tools/plot_run.py \
    --arm "seat OFF:px4_runs/L3_eigen_20260914_202618" \
    --arm "seat ON:px4_runs/L3_eigen_20260914_202810" \
    --arm "seat + adaptive b:px4_runs/L3_eigen_20260914_203001" \
    --out experiments/trajectory_level3_px4_estimator_fast_circle \
    --prefix l3_nolag --zoom 0 25 --settle-from 25 \
    --title "Level 3 (PX4 + estimator), fast circle, r=25 rad/s, rotor_tau 0"

python3 experiments/tools/plot_run.py \
    --arm "seat OFF:px4_runs/L3_eigen_20260914_214502" \
    --arm "seat ON:px4_runs/L3_eigen_20260914_214653" \
    --out experiments/trajectory_level3_px4_estimator_fast_circle \
    --prefix l3_lag20ms --zoom 0 30 --settle-from 25 \
    --title "Level 3, same circle, rotor_tau 20 ms — neither arm survives"
```

**§4's bounce count is confirmed by the tool independently:** seat OFF at `rotor_tau=0.02`
reports *42.3 s above 4 m with 12 crossings*, seat ON *12.4 s with 2*. The 42.3 s number is
precisely the misreading §4 corrects — the crossing count is what distinguishes them.

Cruise statistics quoted in §1b (cruise = ground speed > 5 m/s, +3 s / −1 s trim, radius
about the **fitted** circle centre near (0, 10), not the origin):

| arm | θ_s | radius [m] | alt sd [mm] | tilt | \|p,q\| |
|---|---|---|---|---|---|
| seat OFF | 0 | 10.926 ± 0.426 | 17.9 | 18.78° | 0.187 |
| seat ON | −0.4888 ± 0.0169 | 11.256 ± 0.432 | 18.6 | 19.34° | 0.186 |
| seat + b | −0.5030 ± 0.0163 | 11.251 ± 0.368 | 17.3 | 19.35° | 0.185 |

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
