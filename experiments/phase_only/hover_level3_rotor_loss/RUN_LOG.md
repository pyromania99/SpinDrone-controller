# Run log — hover, level 3, one rotor stopped

**Date flown:** 2026-09-19 (this session).
**Findings document:** [`LEVEL3_ROTOR_LOSS.md`](LEVEL3_ROTOR_LOSS.md).

The question this was flown to answer: **what base spin rate does hover need to survive a
single rotor loss at level 3 — is it at or above 25 rad/s?**

---

## How the rotor is stopped

Levels 2 and 3 had **no failure injection at all** before this session:
`rotor_failure.RotorFailureBackend` is a Pegasus `Backend` wrapper used only by level 1's
`motor_failure.py`, and `sim_px4.py` never referenced it. The PX4 clone's `motor_failure`
branch carries two hooks, and this experiment uses the first:

| hook | what it is | why |
|---|---|---|
| **`SYS_FAILURE_EN` + `MAV_CMD_INJECT_FAILURE`** | upstream PX4: `FailureInjector` → `failure_detector_status.motor_stop_mask` → `ControlAllocator` NaNs that entry of `actuator_motors` | deterministic, instance-selectable, works over the MAVLink link the harness already holds |
| `RC_MAP_KILL_SW2` / `RC_KILLSW2_TH` | the branch's own kill switch, NaNing an output inside `mixer_module.cpp` | built for the KakuteH7. It picks `j = rand() % 4 + 4`, i.e. **output indices 4–7**, and the SITL airframe puts motors on `PWM_MAIN_FUNC1..4` = indices 0–3, so on SITL it kills nothing. Unseeded `rand()`, redrawn on every OFF→ON edge. Not usable for a logged experiment. |

`fly.py --fail-rotor N --fail-at S` (added this session) drives the first one.
`px4_driver.inject_motor_failure()` sends the command; `offboard.fly()` arms it.

**Two preconditions, both of which fail silently if missed:**

- `SYS_FAILURE_EN=1` — read **once**, in `FailureInjector`'s constructor, so it must be set
  before commander starts. `init.d-posix/rcS` already defaults it to 1 in SITL; the
  KakuteH7 board config does not build the `failure` command at all.
- `CA_FAILURE_MODE≥1` — `ControlAllocator::check_for_motor_failures()` returns immediately
  at its default of **0 = Ignore**, discarding the stop mask. `offboard.fly()` sets it.

`offboard.fly()` checks the first and refuses to fly without it, rather than producing a
run in which nothing was ever stopped.

**The failure time is not the commanded time.** Commander republishes
`failure_detector_status` in its 2 Hz block, so the allocator can learn of the stop up to
500 ms after the command. Measured lag between the mask appearing and the motor's
`actuator_motors` channel going NaN: **4–8 ms** across all nine runs, so in practice the
two witnesses agree; the 500 ms is the bound, not the observed number.
`experiments/tools/failure_metrics.py` reads both out of the ulog and scores the flight
either side of the later one.

## Condition

| | |
|---|---|
| level | 3 — PX4 + Pegasus on PX4's own estimator (`MC_CTRL_GT=0`) |
| controller | eigen (`MC_CTRL_ALG=3`) |
| profile | climb to 6 m, hold, stop one rotor 20 s into the hold |
| `--rotor-tau` | `0.020` throughout |
| `--motor-moment` | `4.0816e-7` / `5.102e-7` / `6e-7` → r ≈ **20.1 / 25.1 / 29.5** measured |
| seat | `MC_SEAT_MODE` 0 / 2 |
| `MC_POLE_EN` | 0 |
| `IMU_DGYRO_CUTOFF` | 60 |
| stopped rotor | PX4 instance 1 (motor index 0) in every run |

## The grid — allocator NOT told (`FD_ACT_EN=0`, the default)

This is the level-1 convention: the motor stops, and the controller finds out only through
the state. `CA_FAILURE_MODE=1` stops the motor; re-allocation needs a *detected* failure,
which `FD_ACT_EN=0` prevents.

| run directory | r | seat | t_fail [s] | outcome |
|---|---|---|---|---|
| `px4_runs/L3_eigen_20260919_075716` | 20.8 | off | 28.92 | crash 31.1 s — **2.2 s after** |
| `px4_runs/L3_eigen_20260919_075910` | 20.1 | adaptive | 28.94 | crash 30.9 s — **1.9 s after** |
| `px4_runs/L3_eigen_20260919_080103` | 25 | off | 29.43 | **crashed at 15.8 s, 13.6 s BEFORE the injection** |
| `px4_runs/L3_eigen_20260919_080255` | 25.1 | adaptive | 28.76 | crash 31.6 s — **2.8 s after** |
| `px4_runs/L3_eigen_20260919_080448` | 29.4 | off | 28.36 | **crashed at 12.7 s, 15.7 s BEFORE the injection** |
| `px4_runs/L3_eigen_20260919_080641` | 29.5 | adaptive | 28.90 | crash 30.8 s — **1.9 s after** |

The two seat-off cells at r ≥ 25 are **not fault-tolerance results**: those arms never
survive to the failure. That they crash at 15.8 s and 12.7 s reproduces the archived
seat-off level-3 hover (12.8 s at r = 29.4), so the runs are sound — they simply cannot
answer this question, and the seat-ON row is the only one that can at those spins.

`px4_runs/L3_eigen_20260919_073938` is a seventh run, the smoke test, identical to the
r = 20 adaptive cell: crash **1.8 s** after the failure. It is the n = 2 on that cell.

## The second arm — allocator TOLD (`FD_ACT_EN=1`)

| run directory | r | outcome | what the detector did |
|---|---|---|---|
| `px4_runs/L3_eigen_20260919_083108` | 20.1 | crash 2.5 s after the failure | flagged **all four** motors at t = 10.0 s (`motor_failure_mask = 15`) |
| `px4_runs/L3_eigen_20260919_083306` | 25 | **fell at 9.9 s, 18 s before the injection** | flagged **one** motor at t = 8.7 s (`mask = 2`) — and the allocator removed that healthy motor |
| `px4_runs/L3_eigen_20260919_083458` | 29.5 | crash 1.9 s after the failure | flagged all four at t = 15.0 s |

`px4_runs/L3_eigen_20260919_080926` is a fourth, from a contaminated batch (below): r = 20,
mask = 15 at 9.59 s, crash 1.9 s after the failure — the same number as its clean repeat.

**`FD_ACT_EN` does not work in this rig.** `updateMotorStatus()` compares commanded thrust
against `esc_status` current through `FD_ACT_MOT_C2T` (35 A per unit, the default), and the
simulated ESC current does not follow that model, so the detector fires on healthy motors
within 10 s of takeoff every time. When it flags all four, `CA_FAILURE_MODE`'s
`REMOVE_FIRST_FAILING_MOTOR` declines to act (`num_motors_failed == 1` is required) and the
run degenerates to the `FD_ACT_EN=0` arm — which the matching crash times confirm. When it
flags exactly one, the allocator **removes a healthy motor from the effectiveness matrix**
and the vehicle falls out of the sky on its own.

## Contaminated runs — kept, not deleted

`px4_runs/L3_eigen_20260919_081116` (0.9 s of log) and `L3_eigen_20260919_081119` (no ulog)
come from two driver scripts running at once after a shell-quoting mistake, i.e. two sims
contending for the lockstep port. They are not in any table above.
`L3_eigen_20260919_080926` is from the same batch but had the machine to itself and is
reported. The directories stay as they are.

## Commands

```bash
# the grid (allocator not told)
for KM in 4.0816e-7 5.102e-7 6e-7; do for SEAT in 0 2; do
  python3 examples/flight/fly.py --level 3 --ctrl-alg 3 \
      --motor-moment "$KM" --rotor-tau 0.020 --altitude 6 --hold 60 --headless \
      --fail-rotor 1 --fail-at 20 \
      --gains MC_CTRL_ALG=3 MC_CTRL_GT=0 "MC_SEAT_MODE=$SEAT" MC_POLE_EN=0 \
              IMU_DGYRO_CUTOFF=60
done; done

# the second arm: same, seat adaptive only, plus FD_ACT_EN=1
```

One sim at a time: the driver waits on TCP 4560 (`simlaunch.port_busy()`) between cells.
A `pgrep`-based guard is not safe here — the pattern matches the driver script's own text.

## Scoring

```bash
python3 experiments/tools/failure_metrics.py px4_runs/L3_eigen_20260919_0759*
```

Windows: 10 s before the failure, and either the settled window 5–25 s after it or, when
the vehicle does not last that long, the whole post-failure interval, printed as `after*`
and labelled a departure rather than a hold.

## Figures

```bash
python3 experiments/tools/plot_run.py \
    --arm "seat OFF:px4_runs/L3_eigen_20260919_075716" \
    --arm "seat ON (adaptive):px4_runs/L3_eigen_20260919_075910" \
    --out experiments/hover_level3_rotor_loss --prefix l3_loss_r20 \
    --zoom 25 35 --settle-from 19 \
    --title "Level 3 hover, r=20, rotor 1 stopped at t=29 s"

python3 experiments/tools/plot_run.py \
    --arm "r=20:px4_runs/L3_eigen_20260919_075910" \
    --arm "r=25:px4_runs/L3_eigen_20260919_080255" \
    --arm "r=29.4:px4_runs/L3_eigen_20260919_080641" \
    --out experiments/hover_level3_rotor_loss --prefix l3_loss_spin_sweep \
    --zoom 25 35 --settle-from 19 \
    --title "Level 3 hover, seat ON, rotor 1 stopped: three base spin rates"
```

`plot_run.py` knows nothing about the failure, so it marks the crash and not the rotor
stop; on these figures the two are 2 s apart and the departure is unmistakable.
