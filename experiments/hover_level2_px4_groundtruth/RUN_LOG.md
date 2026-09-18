# Run log — hover, level 2, PX4 + ground truth

**Date flown:** 2026-09-17 (this session).
**Findings document:** [`LEVEL2_HOVER.md`](LEVEL2_HOVER.md).
**Companion:** [`../hover_level3_px4_estimator/`](../hover_level3_px4_estimator/) — the
identical batch at level 3, and [`../hover_level1_km6e7_seat_off_fixed_adaptive/`](../hover_level1_km6e7_seat_off_fixed_adaptive/)
at level 1.

Flown specifically to fill the hover row at levels 2 and 3 — **no post-ground-truth-fix
hover runs existed**. All 21 valid level-2 runs from 2026-09-13 and the 2026-09-14 runs fly
the fast circle; none hovers.

---

## Condition

| | |
|---|---|
| level | 2 — PX4 + Pegasus on ground truth (`MC_CTRL_GT=1`) |
| controller | eigen (`MC_CTRL_ALG=3`) |
| profile | **climb to 6 m and hold — no trajectory.** This is what "hover" means at these rungs. |
| `--hold` | 60 s |
| `--motor-moment` | `6e-7` → r ≈ **29.5 rad/s** measured — chosen to match the level-1 km6e7 hover |
| `--rotor-tau` | `0.020` and `0` |
| seat | `MC_SEAT_MODE` 0 (off) and 2 (adaptive) |
| `MC_POLE_EN` | 0 throughout |
| `IMU_DGYRO_CUTOFF` | 60 |

## The 2×2

| run directory | `rotor_tau` | seat | outcome |
|---|---|---|---|
| `px4_runs/L2_airframe_20260917_034108` | 0.020 | off | **crash 12.1 s** |
| `px4_runs/L2_airframe_20260917_034255` | 0.020 | adaptive | **crash 22.1 s** |
| `px4_runs/L2_airframe_20260917_034444` | 0 | off | flies, no crash |
| `px4_runs/L2_airframe_20260917_034631` | 0 | adaptive | flies, no crash |

**The `airframe` in the directory name is cosmetic.** `fly.py` tags the run directory from
`--ctrl-alg`, which was not passed on this batch; `MC_CTRL_ALG=3` was set via `--gains` and
is confirmed present in every ulog. These are eigen runs.

**`MC_SEAT_MODE` is absent from the seat-off ulogs, and that is correct** — PX4 does not log
a parameter left at its default, and 0 is the default. Absent means off.

## Command

```bash
for TAU in 0.020 0; do for SEAT in 0 2; do
  python3 examples/flight/fly.py --level 2 --controller eigen \
      --motor-moment 6e-7 --rotor-tau "$TAU" --altitude 6 --hold 60 --headless \
      --gains MC_CTRL_ALG=3 MC_CTRL_GT=1 "MC_SEAT_MODE=$SEAT" \
              MC_POLE_EN=0 IMU_DGYRO_CUTOFF=60
done; done
```

Sequential, one sim at a time, ~1.8 min per run. Level 3 is the same with `MC_CTRL_GT=0`.

## Figures

```bash
python3 experiments/tools/plot_run.py \
    --arm "seat OFF:px4_runs/L2_airframe_20260917_034108" \
    --arm "seat ON (adaptive):px4_runs/L2_airframe_20260917_034255" \
    --out experiments/hover_level2_px4_groundtruth --prefix l2_hover_lag20ms \
    --zoom 0 25 --settle-from 13 \
    --title "Level 2 hover (PX4 + ground truth), km=6e-7 (r~29.5), rotor_tau 20 ms"

python3 experiments/tools/plot_run.py \
    --arm "seat OFF:px4_runs/L2_airframe_20260917_034444" \
    --arm "seat ON (adaptive):px4_runs/L2_airframe_20260917_034631" \
    --out experiments/hover_level2_px4_groundtruth --prefix l2_hover_nolag \
    --zoom 0 25 --settle-from 20 \
    --title "Level 2 hover, same spin, rotor_tau 0 (PX4 pipeline lag only)"
```

The r = 20 pair (`l2_hover_r20_lag20ms_*`, the addendum below) is:

```bash
python3 experiments/tools/plot_run.py \
    --arm "seat OFF:px4_runs/L2_eigen_20260917_035604" \
    --arm "seat ON (adaptive):px4_runs/L2_eigen_20260917_035752" \
    --out experiments/hover_level2_px4_groundtruth --prefix l2_hover_r20_lag20ms \
    --zoom 0 30 --settle-from 20 \
    --title "Level 2 hover at r=20 (the circle's spin), rotor_tau 20 ms — the seat RESCUES"
```

All three are also in [`../tools/make_all_figures.sh`](../tools/make_all_figures.sh),
which regenerates every figure in the tree in one pass, and all three were **verified by
reproduction on 2026-09-17** — each regenerates the committed PNG byte for byte.

**`nolag` does not mean zero lag.** It means `rotor_tau = 0` — no rotor pole. PX4 still
carries ~17 ms of transport delay plus ~14 ms of pipeline filtering, so read the `nolag`
arms as **~17 ms of loop delay** and the `lag20ms` arms as that same ~17 ms plus a 20 ms
rotor pole, i.e. **~37 ms**. The names are kept; only the reading changes. The side-by-side
sheets in [`../analysis/`](../analysis/) label the columns that way.

Figures are trimmed to the flight: pre-takeoff and post-crash data is not plotted. Crossing
counts are computed **before** trimming and stay truthful.

## How the steady-state numbers were taken

Window = from 5 s after first reaching 4 m, to 1 s before the crash (or 3 s before the last
sample if there is none). For the `rotor_tau=0.02` seat-on arm that window is only 13–21 s,
which is why its numbers carry the departure and should not be read as cruise values.

| quantity | source |
|---|---|
| altitude, xy drift | `vehicle_local_position_groundtruth` |
| tilt | `arccos(1 − 2(q₁² + q₂²))` from `vehicle_attitude_groundtruth` |
| \|p,q\|, spin | `vehicle_angular_velocity_groundtruth` |
| θ_s | `mc_controller_status.seat_theta` |

## Known confound in this batch

**Spin rate is not matched to the fast-circle experiments.** This batch runs r ≈ 29.5
(km = 6e-7, chosen to match level 1's hover); the circle experiments run r ≈ 20.7 (level 2)
and r ≈ 25 (level 3). A separate batch at km = 4.0816e-7 was flown to separate "hover" from
"higher spin" — see the findings document.

## Addendum — the r = 20 pair

Flown in the same session at `km = 4.0816e-7` (the fast circle's spin) to separate "hover"
from "higher spin", after the 2×2 above failed at r ≈ 29.5. Same `rotor_tau = 0.02`.

| run directory | spin | seat | outcome |
|---|---|---|---|
| `px4_runs/L2_eigen_20260917_035604` | 20.0 | off | crash 26.1 s |
| `px4_runs/L2_eigen_20260917_035752` | 20.0 | adaptive | flies 47.5 s |

**These two are not in [`logs/`](logs/)** — unlike every other run in this folder, their
ulogs live only in the gitignored `px4_runs/`. Copy them out before that tree is cleaned.

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

---

## Re-flight, 2026-09-18 — the `nolag` pair only

Re-flown so that **α is measured on the seat-OFF arm**. Until this date neither controller
computed it with the seat off (level 1 did not build the observer; PX4 skipped the block),
so the misalignment the seat exists to cancel was unrecorded on the arm you compare it
against. The measurement applies nothing — the level-1 re-flight came back bit-for-bit
identical — but PX4 is not reproducible to the bit, so these arms moved a little.

**The figures in this folder now come from the re-flown runs.** The `lag20ms` and r = 20
arms were not re-flown and keep their original logs.

| | old run (L2_airframe_20260917_034444 / L2_airframe_20260917_034631) | re-flown (L2_airframe_20260917_234829 / L2_airframe_20260917_235022) |
|---|---|---|
| θ_s, seat ON | −0.9061 ± 0.1204 | **−0.8601 ± 0.1178** |
| alt sd, off / on | 1.6 / 1.5 mm | 1.6 / 1.5 mm |
| tilt, off / on | 0.19° / 0.12° | 0.19° / 0.13° |
| \|p,q\|, off / on | 0.097 / 0.081 | 0.096 / 0.081 |
| α on the seat-OFF arm | **not recorded** | **mean \|α\| 1.05 rad** |

θ_s moved by 5 %; everything else is inside run-to-run scatter. Window and definitions
as in "How the steady-state numbers were taken" above. The old ulogs are still under
`px4_runs/` and the numbers they produced are the ones quoted in the findings document,
which has not been re-derived.
