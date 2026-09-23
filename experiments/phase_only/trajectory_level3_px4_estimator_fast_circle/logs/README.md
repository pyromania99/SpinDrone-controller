# Flight logs — trajectory_level3_px4_estimator_fast_circle

One directory per run, holding **the ulog itself** plus the two small things worth keeping
beside it:

| file | what it is |
|---|---|
| `*.ulg` | the flight. Everything is in here — state, setpoints, and the only record of the seat/pole parameters (`MC_SEAT_*`, `MC_POLE_*`). |
| `console.txt` | the Isaac + PX4 console, distilled to the ~25 lines that say what was flown. The raw `sim.log` is ~75 KB of carb/GPU/ROS boot noise and repeated MAVLink heartbeat warnings; those are dropped and counted in the header instead. |
| `run_config.json` | motor moment, expected spin, `rotor_tau`, physics rate, duration. |

Reading a ulog's parameters:

```python
import glob, pyulog
u = pyulog.ULog(glob.glob("L3_eigen_20260914_203001/*.ulg")[0], message_name_filter_list=[])
params = dict(u.initial_parameters)
for ts, name, val in u.changed_parameters:     # the driver sets most of these post-boot
    params[name] = val
```

| run | arm / condition | `rotor_tau` | ulog |
|---|---|---|---|
| [`L3_eigen_20260914_203001/`](L3_eigen_20260914_203001/) | L3 seat + adaptive b — §1 headline + §2 figures | `0.0` | `15_00_15.ulg` (30 MB) |
| [`L3_eigen_20260914_202618/`](L3_eigen_20260914_202618/) | L3 seat OFF — §3, flew 50.1 s | `0.0` | `14_56_35.ulg` (30 MB) |
| [`L3_eigen_20260914_202810/`](L3_eigen_20260914_202810/) | L3 seat ON — §3 | `0.0` | `14_58_25.ulg` (30 MB) |
| [`L3_eigen_20260914_212400/`](L3_eigen_20260914_212400/) | L3 seat OFF — §3 reproducibility repeat | `0.0` | `15_54_15.ulg` (30 MB) |
| [`L2_eigen_20260914_212209/`](L2_eigen_20260914_212209/) | L2 seat OFF — §3 comparison, crashed 17 s | `0.0` | `15_52_23.ulg` (30 MB) |
| [`L3_eigen_20260914_214502/`](L3_eigen_20260914_214502/) | L3 seat OFF, `rotor_tau` 0.02 — §4, bounces 6× | `0.02` | `16_15_16.ulg` (30 MB) |
| [`L3_eigen_20260914_214653/`](L3_eigen_20260914_214653/) | L3 seat ON, `rotor_tau` 0.02 — §4, crash 20–24 s | `0.02` | `16_17_08.ulg` (30 MB) |
| [`L3_cascaded_pd_20260914_214031/`](L3_cascaded_pd_20260914_214031/) | L3 cascaded PD — §5 baseline, crash 16.6 s | `0.0` | `16_10_45.ulg` (30 MB) |
| [`L2_eigen_20260914_203427/`](L2_eigen_20260914_203427/) | L2 seat OFF — §6 | `0.0` | `15_04_41.ulg` (30 MB) |
| [`L2_eigen_20260914_203618/`](L2_eigen_20260914_203618/) | L2 seat only — §6 | `0.0` | `15_06_32.ulg` (30 MB) |
| [`L2_eigen_20260914_203810/`](L2_eigen_20260914_203810/) | L2 seat + b — §6 | `0.0` | `15_08_24.ulg` (30 MB) |

These are copies; the originals stay in `px4_runs/<run>/`, which is gitignored.

## Re-flown 2026-09-18 — the runs the current figures use

Added, not substituted. **The originals above are still here**, because the numbers quoted
in the findings document were measured on them; these are what the committed PNGs are drawn
from. The difference is that PX4 now measures the misalignment α on the seat-OFF arm, which
it never did before.

| run | arm / condition | ulog |
|---|---|---|
| [`L3_eigen_20260918_000004/`](L3_eigen_20260918_000004/) | seat OFF, rotor_tau 0 — re-flown 2026-09-18, α now measured | `18_30_26.ulg` (28 MB) |
| [`L3_eigen_20260918_000208/`](L3_eigen_20260918_000208/) | seat ON, rotor_tau 0 — re-flown 2026-09-18, θ_s −0.505 | `18_32_33.ulg` (29 MB) |
| [`L3_eigen_20260918_000420/`](L3_eigen_20260918_000420/) | seat + adaptive b, rotor_tau 0 — re-flown 2026-09-18, θ_s −0.457 | `18_34_45.ulg` (28 MB) |

## The `lag20ms` / r20 arms, re-flown 2026-09-18

Same reason: α was not measured on their seat-OFF arms either. Originals kept.

| run | arm / condition | ulog |
|---|---|---|
| [`L3_eigen_20260918_005324/`](L3_eigen_20260918_005324/) | seat OFF, rotor_tau 0.02 — re-flown 2026-09-18, crash 13.9 s (was 15.3) | `19_23_44.ulg` (28 MB) |
| [`L3_eigen_20260918_005519/`](L3_eigen_20260918_005519/) | seat ON, rotor_tau 0.02 — re-flown, crash 22.4 s (was 20.7) | `19_25_41.ulg` (29 MB) |
