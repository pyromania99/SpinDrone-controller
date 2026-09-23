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

## Re-flown 2026-09-18 — the runs the current figures use

Added, not substituted. **The originals above are still here**, because the numbers quoted
in the findings document were measured on them; these are what the committed PNGs are drawn
from. The difference is that PX4 now measures the misalignment α on the seat-OFF arm, which
it never did before.

| run | arm / condition | ulog |
|---|---|---|
| [`L3_eigen_20260917_235215/`](L3_eigen_20260917_235215/) | seat OFF, rotor_tau 0 — re-flown 2026-09-18, α now measured | `18_22_38.ulg` (24 MB) |
| [`L3_eigen_20260917_235407/`](L3_eigen_20260917_235407/) | seat ADAPTIVE, rotor_tau 0 — re-flown 2026-09-18, θ_s −0.782 | `18_24_32.ulg` (26 MB) |

## The `lag20ms` / r20 arms, re-flown 2026-09-18

Same reason: α was not measured on their seat-OFF arms either. Originals kept.

| run | arm / condition | ulog |
|---|---|---|
| [`L3_eigen_20260918_003829/`](L3_eigen_20260918_003829/) | seat OFF, rotor_tau 0.02 — re-flown 2026-09-18, crash 12.7 s (was 12.8) | `19_08_48.ulg` (25 MB) |
| [`L3_eigen_20260918_004016/`](L3_eigen_20260918_004016/) | seat ADAPTIVE, rotor_tau 0.02 — re-flown, flies 46.0 s | `19_10_40.ulg` (25 MB) |
| [`L3_eigen_20260918_004550/`](L3_eigen_20260918_004550/) | seat OFF, r≈20 — re-flown, survives 47.5 s (still coning) | `19_16_12.ulg` (26 MB) |
| [`L3_eigen_20260918_004741/`](L3_eigen_20260918_004741/) | seat ADAPTIVE, r≈20 — re-flown, flies 45.8 s | `19_18_03.ulg` (25 MB) |
