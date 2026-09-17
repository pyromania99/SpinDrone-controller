# Flight logs — hover_level3_px4_estimator

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
u = pyulog.ULog(glob.glob("L3_airframe_20260917_034821/*.ulg")[0], message_name_filter_list=[])
params = dict(u.initial_parameters)
for ts, name, val in u.changed_parameters:     # the driver sets most of these post-boot
    params[name] = val
```

| run | arm / condition | `rotor_tau` | ulog |
|---|---|---|---|
| [`L3_airframe_20260917_034821/`](L3_airframe_20260917_034821/) | seat OFF, `rotor_tau` 0.02 — crash 12.8 s | `0.02` | `22_18_41.ulg` (28 MB) |
| [`L3_airframe_20260917_035010/`](L3_airframe_20260917_035010/) | seat ADAPTIVE, `rotor_tau` 0.02 — flies 46.7 s | `0.02` | `22_20_32.ulg` (28 MB) |
| [`L3_airframe_20260917_035201/`](L3_airframe_20260917_035201/) | seat OFF, `rotor_tau` 0 — flies | `0.0` | `22_22_24.ulg` (28 MB) |
| [`L3_airframe_20260917_035353/`](L3_airframe_20260917_035353/) | seat ADAPTIVE, `rotor_tau` 0 — flies | `0.0` | `22_24_15.ulg` (28 MB) |
| [`L3_eigen_20260917_035942/`](L3_eigen_20260917_035942/) | seat OFF, r≈20.8 — survives in a 13.7° coning limit cycle | `0.02` | `22_30_05.ulg` (28 MB) |
| [`L3_eigen_20260917_040133/`](L3_eigen_20260917_040133/) | seat ADAPTIVE, r≈20.1 — flies 47.7 s | `0.02` | `22_31_55.ulg` (28 MB) |

These are copies; the originals stay in `px4_runs/<run>/`, which is gitignored.
