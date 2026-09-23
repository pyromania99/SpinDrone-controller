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

## Re-flown 2026-09-18 — the runs the current figures use

Added, not substituted. **The originals above are still here**, because the numbers quoted
in the findings document were measured on them; these are what the committed PNGs are drawn
from. The difference is that PX4 now measures the misalignment α on the seat-OFF arm, which
it never did before.

| run | arm / condition | ulog |
|---|---|---|
| [`L2_airframe_20260917_235601/`](L2_airframe_20260917_235601/) | seat OFF, rotor_tau 0 — re-flown 2026-09-18; the old run's double dip did NOT reproduce | `18_26_26.ulg` (29 MB) |
| [`L2_airframe_20260917_235804/`](L2_airframe_20260917_235804/) | seat ADAPTIVE, rotor_tau 0 — re-flown 2026-09-18, θ_s −0.401 | `18_28_26.ulg` (29 MB) |

## The `lag20ms` / r20 arms, re-flown 2026-09-18

Same reason: α was not measured on their seat-OFF arms either. Originals kept.

| run | arm / condition | ulog |
|---|---|---|
| [`L2_airframe_20260918_004932/`](L2_airframe_20260918_004932/) | seat OFF, rotor_tau 0.02 — re-flown 2026-09-18, crash 16.4 s (was 16.5) | `19_19_53.ulg` (28 MB) |
| [`L2_airframe_20260918_005128/`](L2_airframe_20260918_005128/) | seat ADAPTIVE, rotor_tau 0.02 — **the figure's arm**, crash 19.9 s | `19_21_50.ulg` (29 MB) |
| [`L2_airframe_20260918_005844/`](L2_airframe_20260918_005844/) | seat ADAPTIVE repeat 2 — **flew 52.2 s**, the 1 in 4 that survives | `19_29_06.ulg` (29 MB) |
| [`L2_airframe_20260918_010040/`](L2_airframe_20260918_010040/) | seat ADAPTIVE repeat 3 — crash 22.4 s | `19_31_00.ulg` (29 MB) |
| [`L2_airframe_20260918_010234/`](L2_airframe_20260918_010234/) | seat ADAPTIVE repeat 4 — crash 24.6 s | `19_32_54.ulg` (29 MB) |
