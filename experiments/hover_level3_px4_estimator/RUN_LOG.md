# Run log — hover, level 3, PX4 + estimator

**Date flown:** 2026-09-17 (this session).
**Findings document:** [`LEVEL3_HOVER.md`](LEVEL3_HOVER.md).
**Companion:** [`../hover_level2_px4_groundtruth/`](../hover_level2_px4_groundtruth/) — the
identical batch on ground truth. The two were flown back to back specifically so the only
difference is `MC_CTRL_GT`.

---

## Condition

Identical to the level-2 hover batch except `MC_CTRL_GT=0`:

| | |
|---|---|
| level | 3 — PX4 + Pegasus on **PX4's own estimator** (`MC_CTRL_GT=0`) |
| controller | eigen (`MC_CTRL_ALG=3`) |
| profile | climb to 6 m and hold, no trajectory |
| `--hold` | 60 s |
| `--motor-moment` | `6e-7` → r ≈ **29.4 rad/s** measured |
| `--rotor-tau` | `0.020` and `0` |
| seat | `MC_SEAT_MODE` 0 / 2 |
| `MC_POLE_EN` | 0 |
| `IMU_DGYRO_CUTOFF` | 60 |

Post-dates the EKF2 output-predictor spin fix (`notes/EKF2_OUTPUT_PREDICTOR_SPIN_FIX.md`);
without it `vehicle_attitude` is unusable at this spin and level 3 does not fly at all.

## The 2×2

| run directory | `rotor_tau` | seat | outcome |
|---|---|---|---|
| `px4_runs/L3_airframe_20260917_034821` | 0.020 | off | **crash 12.8 s** |
| `px4_runs/L3_airframe_20260917_035010` | 0.020 | adaptive | **flies 46.7 s, no crash** |
| `px4_runs/L3_airframe_20260917_035201` | 0 | off | flies, no crash |
| `px4_runs/L3_airframe_20260917_035353` | 0 | adaptive | flies, no crash |

`airframe` in the directory name is cosmetic (`--ctrl-alg` not passed); `MC_CTRL_ALG=3` is
confirmed in every ulog. `MC_SEAT_MODE` absent = left at its default of 0 = off.

## Command

```bash
for TAU in 0.020 0; do for SEAT in 0 2; do
  python3 examples/flight/fly.py --level 3 --controller eigen --ctrl-alg 3 \
      --motor-moment 6e-7 --rotor-tau "$TAU" --altitude 6 --hold 60 --headless \
      --gains MC_CTRL_ALG=3 MC_CTRL_GT=0 "MC_SEAT_MODE=$SEAT" \
              MC_POLE_EN=0 IMU_DGYRO_CUTOFF=60
done; done
```

## Measured steady state

Window: from 5 s after first reaching 4 m to 1 s before crash (or 3 s before the last
sample). Ground truth for all analysis, as everywhere in this tree — scoring the estimator
against its own estimate would be circular.

| `rotor_tau` | seat | alt sd | xy drift | tilt | \|p,q\| | θ_s |
|---|---|---|---|---|---|---|
| 0.020 | off | — | — | — | — | — (crashed at 12.8 s) |
| 0.020 | adaptive | 6.8 mm | 19.7 mm | 0.21° | 0.107 | **−0.474 ± 0.101** |
| 0 | off | 7.6 mm | 24.8 mm | 0.19° | 0.098 | 0 |
| 0 | adaptive | 7.4 mm | 28.9 mm | 0.13° | 0.081 | **−0.870 ± 0.099** |

**Level 3 is about 5× noisier than level 2 on the same profile** (alt sd 7.6 mm vs 1.6 mm,
xy 25 mm vs 1.4 mm). That is the estimator in the loop and it is the expected direction.

## An anomaly worth chasing

**θ_s gets *smaller* when rotor lag is added: −0.870 at `rotor_tau=0`, −0.474 at
`rotor_tau=0.02`.** More actuator lag should mean more phase to cancel, and on the level-2
*circle* it behaves that way (−0.411 → −0.530). Here it goes the other way, by a lot.

Not explained. Candidates, none tested:
- The CROSS law's fixed point is where the time-average of `|xd||xa| sin α` vanishes, not
  where α vanishes (`Seat.adapt` docstring). Adding lag changes the magnitude weighting as
  well as the phase, so the fixed point can move non-monotonically.
- The `rotor_tau=0` arm may be converging against PX4's pipeline lag alone in a regime where
  the observable is better conditioned, and the lagged arm may simply not have settled in
  the 46 s available.

**Do not build an argument on the sign of dθ_s/dτ from this data.** Re-fly with a τ sweep
if it matters.

---

## Addendum — the r = 20 pair

Flown in the same session to separate "hover" from "higher spin", after the level-2 batch
failed at r ≈ 29.5 and the fast circle succeeded at r ≈ 20.7.

| run directory | spin | seat | outcome |
|---|---|---|---|
| `px4_runs/L3_eigen_20260917_035942` | 20.8 | off | survives 47.4 s — **in a 13.7° coning limit cycle**, \|p,q\| 1.602 |
| `px4_runs/L3_eigen_20260917_040133` | 20.1 | adaptive | survives 47.7 s, tilt 0.22°, \|p,q\| 0.088, θ_s −0.505 ± 0.061 |

```bash
python3 examples/flight/fly.py --level 3 --controller eigen --ctrl-alg 3 \
    --motor-moment 4.0816e-7 --rotor-tau 0.020 --altitude 6 --hold 60 --headless \
    --gains MC_CTRL_ALG=3 MC_CTRL_GT=0 MC_SEAT_MODE=<0|2> MC_POLE_EN=0 IMU_DGYRO_CUTOFF=60
```

Statistics over t = 20–45 s (steady hold, after spin-up, before the landing descent).

**The seat-off arm here is why a duration check is not enough.** It never drops below 4 m,
holds altitude to 6.5 mm and position to 6 cm, and would pass any "did it fly" test — while
coning at 13.7° with 1.6 rad/s body rates. Score attitude, not just survival.

## The full matrix flown this session (`rotor_tau = 0.02` throughout)

| level | profile | spin | seat off | seat on |
|---|---|---|---|---|
| 2 | hover | 20.0 | crash 26.1 s | flies 47.5 s |
| 2 | hover | 29.5 | crash 12.1 s | crash 22.1 s |
| 3 | hover | 20.8 | survives, 13.7° cone | flies clean 47.7 s |
| 3 | hover | 29.4 | crash 12.8 s | flies 46.7 s |

Plus, from the archived logs: level 2 circle r = 20.7 (crash 16.5 s → flies 48.7 s) and
level 3 circle r = 25 (crash 15.3 s → crash 20.7 s).

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
