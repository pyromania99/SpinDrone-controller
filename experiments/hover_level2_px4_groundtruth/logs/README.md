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

## Re-flown 2026-09-18 — the runs the current figures use

Added, not substituted. **The originals above are still here**, because the numbers quoted
in the findings document were measured on them; these are what the committed PNGs are drawn
from. The difference is that PX4 now measures the misalignment α on the seat-OFF arm, which
it never did before.

| run | arm / condition | ulog |
|---|---|---|
| [`L2_airframe_20260917_234829/`](L2_airframe_20260917_234829/) | seat OFF, rotor_tau 0 — re-flown 2026-09-18, α now measured | `18_18_53.ulg` (27 MB) |
| [`L2_airframe_20260917_235022/`](L2_airframe_20260917_235022/) | seat ADAPTIVE, rotor_tau 0 — re-flown 2026-09-18, θ_s −0.860 | `18_20_46.ulg` (24 MB) |

## The `lag20ms` / r20 arms, re-flown 2026-09-18

Same reason: α was not measured on their seat-OFF arms either. Originals kept.

| run | arm / condition | ulog |
|---|---|---|
| [`L2_airframe_20260918_003456/`](L2_airframe_20260918_003456/) | seat OFF, rotor_tau 0.02 — re-flown 2026-09-18, crash 11.9 s (was 12.1) | `19_05_16.ulg` (26 MB) |
| [`L2_airframe_20260918_003645/`](L2_airframe_20260918_003645/) | seat ADAPTIVE, rotor_tau 0.02 — re-flown, crash 21.4 s (was 22.1) | `19_07_02.ulg` (27 MB) |
| [`L2_airframe_20260918_004209/`](L2_airframe_20260918_004209/) | seat OFF, r≈20 — re-flown, crash 36.6 s (was 26.1) | `19_12_32.ulg` (26 MB) |
| [`L2_airframe_20260918_004401/`](L2_airframe_20260918_004401/) | seat ADAPTIVE, r≈20 — re-flown, flies 46.8 s | `19_14_22.ulg` (26 MB) |
