# L3 waypoint ellipse — the actuator lead on a setpoint trajectory

**Flown 2026-09-20.** 20 runs, level 3 (`MC_CTRL_GT=0`), the same 69-point arrival-based
ellipse the [`../../trajectory_level3_waypoint_ellipse/`](../../trajectory_level3_waypoint_ellipse/)
baseline flies, at a rotor pole where the seat alone cannot hold it.

## Headline

| `rotor_tau` | ceiling | what carries it |
|---|---|---|
| **0 ms** | **≥ 29.5 rad/s** | nothing much — it flies **seat-off** |
| 20 ms | 24.0 flew (archive, 69 s exposure only) | seat |
| **30 ms** | **22.0 rad/s** | seat **and** lead; seat alone dies at 18 s |

The profile is not the limit — **the rotor pole is**. Removing it moves the ceiling from 22
to at least 29.5 on the identical trajectory. 29.5 was not exceeded because ~30 rad/s is the
vehicle's own ceiling, so the zero-lag figure is a **lower bound, not a measured maximum**.

## The command

```bash
python3 examples/flight/fly.py --level 3 --controller eigen \
  --motor-moment <KM> --rotor-tau <TAU> --headless --duration 400 --hold <HOLD> \
  --l2-trajectory examples/results/ellipse_waypoints.csv --l2-waypoint-radius 0.6 \
  --gains MC_CTRL_ALG=3 MC_CTRL_GT=0 MC_SEAT_MODE=<0|2> MC_POLE_EN=0 \
          IMU_DGYRO_CUTOFF=60 MC_SEAT_SATGATE=1 \
          MC_LEAD_EN=<0|1> MC_LEAD_TC=<TC> MC_LEAD_WN=250 MC_LEAD_DMAX=25
```

Spin is set through `--motor-moment` on the archive's calibration `r = 4.92e7 × km`
(20.1/4.0816e-7, 25.1/5.102e-7, 29.5/6e-7, agreeing to 0.2 %): r=22 → 4.4715e-7,
r=23 → 4.6748e-7, r=24 → 4.8771e-7. The `r_ss` column is the **measured** spin.

## ⚠ Three scoring traps this profile sets

Scored with [`../../tools/score_waypoint_ellipse.py`](../../tools/score_waypoint_ellipse.py),
which exists because of these.

1. **`AUTO_LAND` is not a crash.** `nav_state` goes 14 → 18 at the end and the altitude walks
   down on purpose. A crash is ground contact **while still in OFFBOARD**; anything after is
   the profile landing itself. A naive "altitude below 1 m" test fails every good run.
2. **"trajectory complete" on the console does not mean the path was flown.** The commanded
   ellipse is **85.3 m** and the best run here covers 75 % of it. The message means the
   streaming loop exited.
3. **Duration is NOT controlled, so survival time is not comparable.** The loop's deadline is
   `time.time() + max_seconds` — **wall clock** — so how much *simulated* flight a run gets
   depends on how fast the sim happened to run. Observed range on nominally identical configs:
   **69 s to 148 s**. Score on **path flown**, which is what the tables below report.

Trap 3 is not academic: `C_xyp025_a` and `C_xyp025_b` are the same configuration, and one
"survived" because it was stopped at 69 s having done 17 m while the other was allowed 144 s,
did 32 m, and **crashed**. `--hold 200` (group B) is what makes the exposure long enough to
trust.

## Reproducing: set `MC_LEAD_DMAX` explicitly

Every run here booted firmware whose `MC_LEAD_DMAX` default was still **200**; the default was
lowered to 25 in source afterwards and SITL was not rebuilt. A `DMAX` of 200 in the tables
means the parameter was never set. Same caveat as
[`../hover_level3_actuator_lead/RUN_LOG.md`](../hover_level3_actuator_lead/RUN_LOG.md).

## The runs

### A — The lead's effect on the profile — short (69 s) exposure

| label | `rotor_tau` | r_ss | lead | `TC` | `DMAX` | seat | `XY_P` | outcome | path flown | tilt p95 | θ_s sd |
|---|---|---|---|---|---|---|---|---|---|---|---|
| `A_r20_seatlead_a` | 0.030 | 20.1 | on | 30 ms | 25 | adaptive | 0.5 | **flew 69s** | 28.4 m (33%) | 3.94° | 0.187 |
| `A_r20_seatlead_b` | 0.030 | 20.1 | on | 30 ms | 25 | adaptive | 0.5 | **flew 69s** | 27.7 m (33%) | 4.03° | 0.173 |
| `A_r20_seatonly` | 0.030 | 19.4 | off | — | — | adaptive | 0.5 | **CRASH 18s** | 6.8 m (8%) | 9.13° | 0.411 |
| `A_r22_seatlead` | 0.030 | 22.0 | on | 30 ms | 25 | adaptive | 0.5 | **flew 69s** | 27.9 m (33%) | 4.48° | 0.251 |
| `A_r22_seatonly` | 0.030 | 20.4 | off | — | — | adaptive | 0.5 | **CRASH 14s** | 4.1 m (5%) | 9.54° | 0.403 |
| `A_r24_seatlead_a` | 0.030 | 23.0 | on | 30 ms | 25 | adaptive | 0.5 | **CRASH 17s** | 5.5 m (7%) | 7.24° | 0.338 |
| `A_r24_seatlead_b` | 0.030 | 23.2 | on | 30 ms | 25 | adaptive | 0.5 | **CRASH 18s** | 6.9 m (8%) | 8.86° | 0.418 |
| `A_r24_seatonly` | 0.030 | 21.3 | off | — | — | adaptive | 0.5 | **CRASH 12s** | 2.3 m (3%) | 7.24° | 0.264 |

### B — Long exposure (`--hold 200`) — where the ceiling actually is

| label | `rotor_tau` | r_ss | lead | `TC` | `DMAX` | seat | `XY_P` | outcome | path flown | tilt p95 | θ_s sd |
|---|---|---|---|---|---|---|---|---|---|---|---|
| `B_r22_long_a` | 0.030 | 22.0 | on | 30 ms | 25 | adaptive | 0.5 | **flew 142s** | 61.9 m (73%) | 3.74° | 0.197 |
| `B_r22_long_b` | 0.030 | 22.0 | on | 30 ms | 25 | adaptive | 0.5 | **flew 140s** | 60.6 m (71%) | 3.83° | 0.184 |
| `B_r22_long_c` | 0.030 | 22.0 | on | 30 ms | 25 | adaptive | 0.5 | **flew 140s** | 60.5 m (71%) | 3.86° | 0.188 |
| `B_r23_long_a` | 0.030 | 22.1 | on | 30 ms | 25 | adaptive | 0.5 | **CRASH 18s** | 6.4 m (7%) | 7.20° | 0.363 |
| `B_r23_long_b` | 0.030 | 23.0 | on | 30 ms | 25 | adaptive | 0.5 | **CRASH 56s** | 24.6 m (29%) | 5.42° | 0.365 |
| `B_r24_long_a` | 0.030 | 23.6 | on | 30 ms | 25 | adaptive | 0.5 | **CRASH 23s** | 11.4 m (13%) | 9.81° | 0.497 |

### C — Does slowing navigation save r = 24? (`MC_EIG_XY_P`)

| label | `rotor_tau` | r_ss | lead | `TC` | `DMAX` | seat | `XY_P` | outcome | path flown | tilt p95 | θ_s sd |
|---|---|---|---|---|---|---|---|---|---|---|---|
| `C_xyp0125` | 0.030 | 24.0 | on | 30 ms | 25 | adaptive | 0.125 | **flew 69s** | 10.0 m (12%) | 0.88° | 0.050 |
| `C_xyp025_a` | 0.030 | 24.0 | on | 30 ms | 25 | adaptive | 0.25 | **flew 69s** | 16.7 m (20%) | 1.91° | 0.124 |
| `C_xyp025_b` | 0.030 | 24.0 | on | 30 ms | 25 | adaptive | 0.5 | **CRASH 144s** | 32.1 m (38%) | 1.83° | 0.109 |

### D — Zero rotor pole — is the PROFILE the limit, or the lag?

| label | `rotor_tau` | r_ss | lead | `TC` | `DMAX` | seat | `XY_P` | outcome | path flown | tilt p95 | θ_s sd |
|---|---|---|---|---|---|---|---|---|---|---|---|
| `D_nolag_noseat` | 0.000 | 29.5 | off | — | — | off | 0.5 | **flew 141s** | 61.8 m (72%) | 4.19° | 0.000 |
| `D_nolag_seat_a` | 0.000 | 29.5 | off | — | — | adaptive | 0.5 | **flew 141s** | 64.4 m (75%) | 3.44° | 0.112 |
| `D_nolag_seat_b` | 0.000 | 29.5 | off | — | — | adaptive | 0.5 | **flew 141s** | 63.5 m (74%) | 3.39° | 0.128 |
---

## Reading the groups

### A + B — the ceiling at `rotor_tau = 0.030`

| r | seat only | seat + lead |
|---|---|---|
| 20.1 | crash 18–24 s | **flew** ×2 |
| 22.0 | crash 18 s | **flew ×3 at long exposure** — 60.5 / 60.6 / 61.9 m, 140–142 s |
| 23.0 | — | **crash ×2** — 6.4 m and 24.6 m |
| 24.0 | crash 17 s | **crash ×3** — 5.5 / 6.9 / 11.4 m |

**22 flies, 23 does not, and the arms do not overlap**: the better of the two r=23 runs died
having covered 24.6 m, less than half what r=22 does without trouble. r=22 was re-flown at
double the exposure (140 s, 61 m) precisely because a 69 s result could not be trusted after
trap 3 — it held 3/3.

Note the r=23 pair disagree on *when* they die (18 s vs 56 s) while agreeing that they do.
That variance near a boundary is why every cell here that decides something has n ≥ 2.

**The lead does not fully recover the extra 10 ms of pole.** The archive flies r = 24 at
`rotor_tau = 0.020` with no lead at all; with the lead at 0.030 the ceiling is 22. So the
lead buys back most of a 10 ms pole increase on this profile, not all of it — a more modest
reading than the hover result gives, and the right one for trajectory work. (The archive's
r=24 cell is a 69 s exposure and has not been re-flown long.)

### C — slowing navigation does not fix r = 24

The profile has **no speed parameter**: in arrival mode the reference *waits* until the
vehicle is within 0.6 m, so the vehicle sets its own pace and `MC_EIG_XY_P` is what sets it.

| `XY_P` | fate | path before failure |
|---|---|---|
| 0.5 | crash ×3 | 5.5 / 6.9 / 11.4 m |
| 0.25 | crash (given 144 s) | **32.1 m** |
| 0.125 | stopped at 69 s | 10.0 m, only 12 % of the path |

Halving the position gain takes distance-to-failure from ~6 m to 32 m — a **5× improvement** —
and **does not remove the failure**. The one slow run allowed to keep flying crashed. Read on
time alone it looks like a rescue; read on distance it is a postponement. This agrees with
[`../../phase_only/analysis/SESSION_2026-09-18.md`](../../phase_only/analysis/SESSION_2026-09-18.md)
§2d, where a 6× cut in centripetal demand on the circle changed nothing: **the mechanism is
not a speed or acceleration limit.**

### D — the profile is not the limit

At `rotor_tau = 0`, r = 29.5, the identical trajectory flies **3/3 for 141 s at 0.40 m/s**,
covering 72–75 % of the path — and it flies **with the seat off**, at 4.19° tilt p95 against
3.39–3.44° with it.

So at zero rotor pole the seat is **not load-bearing on this profile**. That is consistent,
not contradictory: with no pole the only residual is the ~23 ms PX4 pipeline delay, and that
alone does not destabilise the vehicle here. The seat earns its place when a real actuator
pole sits underneath — which is the hardware case.

## Failure signature, common to every crashed cell

- **saturation**: 76–86 % of whole-log ticks on the failures, 10–12 % on the flights
- **θ_s sd**: ±0.36–0.50 on the failures, ±0.11–0.25 when flying
- **speed**: failures sit near-stationary (median 0.08–0.10 m/s) with lurches to 10–20× that;
  healthy runs cruise steadily at 0.37–0.41 m/s

The failures are saturation-dominated, which `../../../notes/HARDWARE_READINESS.md` §6 already
names as the mechanism the current theory does not model.

## What this does NOT establish

1. **No cell flies the whole 85.3 m ellipse.** Best is 75 %. The limit is "flies well for
   140 s", not "completes the route".
2. **The zero-lag ceiling is a lower bound.** 29.5 was not exceeded because it is the
   vehicle's own ceiling, not because anything failed.
3. **`MC_LEAD_TC` is matched to the pole** in every lead cell. The mismatched sweep
   (`ACTUATOR_LEAD.md` §3c) was run in hover only.
4. **One profile.** The fast circle, which the archive shows is harder, is untouched.
