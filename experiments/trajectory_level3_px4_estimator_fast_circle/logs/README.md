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
