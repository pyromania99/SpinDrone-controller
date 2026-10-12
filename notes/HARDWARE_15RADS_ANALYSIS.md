# Hardware analysis — ~15 rad/s build (3 × 10° + 1 × 20°), 7 Oct 2026

The first flights at ~15 rad/s, and the first long flight logged at full rate. Every number is
reproducible from the logs with the scripts in `examples/tools/px4_tuning/` (commands in §7).
Earlier builds: [`HARDWARE_2TILT_ANALYSIS.md`](HARDWARE_2TILT_ANALYSIS.md) (8.6 rad/s) and
[`HARDWARE_TILT10_SPIN_ANALYSIS.md`](HARDWARE_TILT10_SPIN_ANALYSIS.md) (spin per tilted motor).

## 1. Data

| Log | Airborne | Spin r | Ended by | Usable for |
|---|---|---|---|---|
| `3_10_1_20/log_9_UnknownDate.ulg` | ~6 s | 14.6 rad/s | **log stops while flying** (§6) | stability only |
| `3_10_1_20/log_1_2026-10-7-23-55-48.ulg` | ~50 s | 14.6–15.5 rad/s | kill switch after landing | **everything below** |

**Build.** The folder name is read as **3 motors at 10° + 1 at 20°**; confirm it. The tilt model
predicts 2.84 + 3·2.45 + 2.45·(tan 20°/tan 10°) ≈ 15.2 rad/s, and the flights measured 14.6–15.5.

**Setup, both logs.** Eigen (`MC_CTRL_ALG 3`, `fallback_active 0`), seat off, lead off, POSCTL.
`WN 30`, `ATT_P 4.2`, `ATT_D 0.05`, `XY_P 0.5`, `XY_D 2.0`, `TRQ_MAX 2.11`,
`IXX/IYY/IZZ 0.017/0.015/0.015`, `EKF2_EV_DELAY 15`, `IMU_GYRO_CUTOFF 90`. The two logs have the
same params apart from `BAT1_A_PER_V`.

**Logging.** Both used the first SD-card topic file (45 topics, `SDLOG_PROFILE 0`), not the trimmed
one. Gyro, torque setpoint and motor commands at ~280 Hz, EV at ~121 Hz, `esc_status` at ~100 Hz.
log_1: 136 KB/s, CPU 0.89 armed (max 0.99), 2 dropouts totalling 0.1 s.

## 2. Flight performance (log_1)

| Phase | Spin r | Tilt error rms / p95 | Motor p99 | Height |
|---|---|---|---|---|
| Hover (9–15 s) | 15.0–15.5 | 1.1° / 1.4° | 0.40–0.43 | 1.0 m |
| Stick inputs (15–34 s) | 15.0–15.4 | 1.3–3.7° / up to 6.8° | up to 0.49 | 1.0 → 0.6 m |
| Quiet hover (36–54 s) | 14.6–15.1 | 1.1–1.3° / 1.5–1.9° | 0.41–0.43 | 0.6–0.75 m |
| Descent and kill (54–60 s) | → 0 | | | ground |

- **Stable, not degrading.** The quiet hover at the end matches the hover at the start. Tilt error
  goes back to ~1.1° within about a second of every stick input.
- **No saturation at any point.** Sustained motor commands stayed within 0.24–0.51 (§3).
- Mean motor command rose 0.364 → 0.379 as the battery sagged 11.5 → 11.1 V. That is voltage
  compensation, not degradation.
- **|ω_xy| ≈ 15–16 °/s** in hover is coning: a ~1.1° tilt turning at 15 rad/s.
- The EV yaw innovation has a mean of **+2° to +8°** at spin, and ~0° on the ground (§4).
- **The spin tracks height.** It reads 15.5 at 1.0 m, 14.7 at 0.6 m and 15.1 at 0.75 m. A height
  dependence like that hints at ground effect, but it isn't established.
- Battery current is invalid; the pin reads voltage (`HARDWARE_TILT10_SPIN_ANALYSIS.md` §6).

## 3. Stick inputs and motor headroom

In POSCTL the stick commands velocity (`MPC_POS_MODE 0`, `MPC_VEL_MANUAL 10`), so every input reaches
the attitude loop through the position controller. The inputs were 0.08–0.26 of stick, giving up to
5.9° of tilt and 0.48 m/s.

Motor commands, averaged over 25 ms (about the motor time constant, so this is what the motors can
follow), with ESC RPM:

| Phase | Command min | Command max | RPM min |
|---|---|---|---|
| Hover (9–15 s) | 0.31 | 0.45 | 16,100 |
| Stick inputs (15–34.5 s) | **0.24** | **0.51** | 14,750 |
| Quiet hover (36–54 s) | 0.31 | 0.43 | 16,100 |

Hover RPM is ~18,000.

- **There is plenty of headroom on both rails.** The inputs used ~0.13 of the ~0.37 room below
  hover, and ~0.13 of the ~0.6 above it.
- The lower rail is still the nearer one, and it shrinks roughly with input size.
- **Single-sample command spikes don't count.** The raw command touched 0.058 for one or two
  samples (3 of 12,857, each 3–7 ms) and the ESCs never followed it: RPM stayed above 14,750.
  Judge headroom on the 25 ms average, not the raw minimum.

**For identification flights:**
- 0.2–0.25 stick was already enough for the fits in §4. Taps of ~0.3 s, on both axes and in both
  directions.
- Larger inputs are allowed: go up in steps (0.3, then 0.4).
- After each flight, check the 25 ms-averaged minimum motor command over the inputs. Stay above ~0.1.

## 4. Measurements (log_1, full rate)

### EV delay — 18.8 ms

`ev_delay_long_baseline.py`: **18.8 ms** (+2.5 ms gyro filter included; block-bootstrap 95% CI
18.5–26.5). An injected +20 ms reads back as 37.1, a gain of 0.92.

- **This supports raising `EKF2_EV_DELAY` from 15 to ~19.** The yaw innovation's +2° to +8° mean
  at 15 rad/s matches a ~4 ms under-setting.
- After the change, the yaw innovation mean should drop to ~0°.
- The 3.4 ms from log_9 is withdrawn (§8).

### Izz, gain, coupling

`coupling_fit.py`, 0.3–8 Hz, best lag 30 ms:

| | value | note |
|---|---|---|
| **Izz_eff** | **0.0223** | 0.0223–0.0225 over lags of 25–35 ms |
| κ (net gyroscopic coupling) | +0.40 | the controller assumes (0.015 − 0.016)/0.016 ≈ −0.06 |
| gain (actual/model authority) | 0.80 | the plant gives ~80% of the torque the model expects |
| R² | 0.36 | 0.33 with κ forced to the controller's value |

- **Supports `MC_EIG_IZZ` 0.015 → 0.022.** It is an effective value that includes rotor angular
  momentum. It feeds only the gyroscopic feedforward.
- R² 0.36 is the best yet. log_9 gave 0.08, too short and without inputs.
- It sits just above the 2-tilt estimate of 0.016–0.021.
- The 0.80 gain says `TRQ_MAX/I` is ~20% optimistic. Not acted on yet.

**Seat observable** (0.3–8 Hz band): mean misalignment **12°**, coherence R **0.42**. The lag model
predicts r·35 ms = 30°. That is the most coherent reading so far (log_1 at 8.6 rad/s: R 0.21), but
it is not yet the gate in `SPIN_RAMP_STEPS.md` §6.

### Motor lag — about 30 ms

`rpm_lag_from_bdshot.py` (command → RPM, first-order with dead time):

| Motor | Dead time L | Time constant T | L+T within 5% of best |
|---|---|---|---|
| 0 | 5.0 ms | 25 ms | 22–38 ms |
| 1 | 7.5 ms | 25 ms | 28–38 ms |
| 2 | 7.5 ms | 20 ms | 20–38 ms |
| 3 | 5.0 ms | 20 ms | 17–38 ms |

- All four motors agree; motor 1 is no longer the slow one it looked like on 2 Oct.
- The fit is broad because hover gives small command changes. Larger stick taps (§3) would
  tighten it.

## 5. Does the logging rate matter?

Each fit was re-run on log_1 with topics thinned using `thin_rates.py`:

| | Full rate | Default-set rates | Trimmed file (ESC 20 Hz, motors 125 Hz) |
|---|---|---|---|
| EV delay | 18.8 ms | 24.8 ms, CI 4.5–28.5 | (EV unchanged) |
| Izz_eff | 0.0223 (R² 0.36) | 0.0267 (R² 0.32) | (gyro/torque unchanged) |
| Motor lag | L 5–7.5, T 20–25 ms | fails (motor commands at 10 Hz break it) | **same as full rate** |

- **High-rate gyro, torque and EV were necessary.** At default rates Izz reads ~20% high and the EV
  delay CI is too wide to use. The 15–16 ms delays measured on 5 Hz EV logs (Sep 30 – Oct 6) are
  probably biased by this.
- **The trimmed topic file loses nothing.** It keeps gyro, torque and EV at full rate, and motor lag
  is unchanged with `esc_status` at 20 Hz.
- **CPU follows log rate.** Default set 43 KB/s → 0.70 armed; first topic file 125–136 KB/s → 0.89;
  profile 19 → 0.99. The trimmed file (~75 KB/s) should come back to ~0.75–0.8.

## 6. The log_9 cutoff

log_9 stops at 13.7 s while armed and flying. There is no disarm, failsafe or logger error, and the
file ends on a whole message.

- **Not the logging.** log_1 ran the same topic file at the same CPU load for a minute without a
  problem, and profile 19 ran 83 s at 0.99.
- **Most likely power:** a battery lead or power-module connector, or the FC supply.
- Unresolved: check the card for `fault_*.log` (one means a reset; none means power was lost).

## 7. Reproduce

```bash
cd examples/tools/px4_tuning
L=~/Downloads/logs/3_10_1_20/log_1_2026-10-7-23-55-48.ulg
python3 ev_delay_long_baseline.py $L [--inject 0.02]
python3 coupling_fit.py $L
python3 rpm_lag_from_bdshot.py $L       # now handles gap-free logs (1 s pieces)
python3 thin_rates.py --default-set coupling_fit.py $L
python3 thin_rates.py --rate esc_status=20 --rate actuator_motors=125 rpm_lag_from_bdshot.py $L
```

## 8. Withdrawn

- **"EV delay 3.4 ms" (log_9).** Only ~6 s of flight, mostly spin-up. log_1 gives 18.8 ms.
- **"Battery at ~37 A."** The current pin reads voltage.
- **"The motor-lag fit fails at 10 Hz ESC."** That test also thinned the motor commands to 10 Hz,
  which is what broke it. ESC at 10 Hz alone was not tested; 20 Hz works.
- **"15.7 rad/s is the EKF2 limit."** That is the limit for *stock* PX4 (`(π/2)/τ` at
  `EKF2_DELAY_MAX` 100 ms). The flashed build `d58c73df` has the output-predictor fix
  (`f7953526`, `10c40849`), so passing 15.7 tests the fix rather than hitting a wall.

## 9. Next

1. `EKF2_EV_DELAY 15 → 19` and `MC_EIG_IZZ 0.015 → 0.022`. First hop: the tilt error should be ≤ 1.1°
   rms, and the yaw innovation mean ~0°.
2. Copy the trimmed `logger_topics_spin.txt` to the card.
3. 30 s or more of hover plus stick taps, building from 0.25 to 0.4 (§3), then re-run §7.
4. Next tilt step: above 15.7 rad/s, check that attitude stays clean (`HARDWARE_READINESS.md` §0 and
   §5 B.10).
