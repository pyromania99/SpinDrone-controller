# Flight logs — trajectory_level2_px4_groundtruth_fast_circle

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
u = pyulog.ULog(glob.glob("L2_eigen_20260913_201119/*.ulg")[0], message_name_filter_list=[])
params = dict(u.initial_parameters)
for ts, name, val in u.changed_parameters:     # the driver sets most of these post-boot
    params[name] = val
```

| run | arm / condition | `rotor_tau` | ulog |
|---|---|---|---|
| [`L2_eigen_20260913_201119/`](L2_eigen_20260913_201119/) | seat OFF, `rotor_tau` 0.02 — 7.8 s above 4 m | `0.02` | `14_41_42.ulg` (31 MB) |
| [`L2_eigen_20260913_201321/`](L2_eigen_20260913_201321/) | seat ON, `rotor_tau` 0.02 — 48.7 s, 3+ orbits | `0.02` | `14_43_36.ulg` (34 MB) |
| [`L2_eigen_20260913_200501/`](L2_eigen_20260913_200501/) | seat OFF, `rotor_tau` 0 — 48.8 s, 4 crossings | `0.0` | `14_35_17.ulg` (30 MB) |
| [`L2_eigen_20260913_200711/`](L2_eigen_20260913_200711/) | seat ON, `rotor_tau` 0 — 48.8 s | `0.0` | `14_37_26.ulg` (30 MB) |

These are copies; the originals stay in `px4_runs/<run>/`, which is gitignored.
