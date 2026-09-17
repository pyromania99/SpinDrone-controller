# Flight logs — hover_level2_px4_groundtruth

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
u = pyulog.ULog(glob.glob("L2_airframe_20260917_034108/*.ulg")[0], message_name_filter_list=[])
params = dict(u.initial_parameters)
for ts, name, val in u.changed_parameters:     # the driver sets most of these post-boot
    params[name] = val
```

| run | arm / condition | `rotor_tau` | ulog |
|---|---|---|---|
| [`L2_airframe_20260917_034108/`](L2_airframe_20260917_034108/) | seat OFF, `rotor_tau` 0.02 — crash 12.1 s | `0.02` | `22_11_27.ulg` (29 MB) |
| [`L2_airframe_20260917_034255/`](L2_airframe_20260917_034255/) | seat ADAPTIVE, `rotor_tau` 0.02 — crash 22.1 s | `0.02` | `22_13_16.ulg` (29 MB) |
| [`L2_airframe_20260917_034444/`](L2_airframe_20260917_034444/) | seat OFF, `rotor_tau` 0 — flies | `0.0` | `22_15_04.ulg` (28 MB) |
| [`L2_airframe_20260917_034631/`](L2_airframe_20260917_034631/) | seat ADAPTIVE, `rotor_tau` 0 — flies | `0.0` | `22_16_52.ulg` (28 MB) |

These are copies; the originals stay in `px4_runs/<run>/`, which is gitignored.
