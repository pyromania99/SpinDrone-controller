# Spin ramp — steps left

Starting point: `~/Downloads/logs/30th_2tilt/log_1_2026-9-30-21-50-08.ulg` (2026-09-30).
Eigen (ALG=3), seat off, lead off, EKF height from EV, POSCTL, all `CA_ROTOR*_KM = +0.05`.

| | log_1 value |
|---|---|
| Spin rate r | +8.6 rad/s |
| Tilt error rms | 0.6° → 1.2° over the flight |
| Motors mean / p99 | 0.35 / 0.385, no saturation |
| Fitted lag | 35 ms |
| Fitted gyroscopic coupling κ | ≈ −0.1 (controller assumes +1.0) |
| Seat observable | incoherent (circular mean 88°, R 0.21) |
| EV delay | unmeasurable (EV logged at 5 Hz) |

---

## 1. Firmware (needs a reflash)

- [ ] **Log-writer stack.** Source already edited: `src/modules/logger/log_writer_file.cpp`,
      `PX4_STACK_ADJUSTED(1170)` → `1600`. Not built yet. log_1 warned
      `log_writer_file low on stack! (272 bytes left)`.
- [ ] **Decide on the EV throttle.** The working tree has an uncommitted edit that sets
      `add_topic("vehicle_visual_odometry", 200)` in `logged_topics.cpp`. Either revert it in the
      same build, or leave it and rely on `SDLOG_PROFILE 19` (step 2).
- [ ] `make holybro_kakuteh7_default`, then flash.
- [ ] **Verify:** `top` in the shell shows `log_writer_file` stack used/total with margin. The
      low-stack warning is gone after an armed logging run.

## 2. Parameters

- [ ] `SDLOG_PROFILE 19` (1 default + 2 estimator replay + 16 high rate). Replay re-adds EV with
      interval 0, which overrides the 200 ms. It also brings the `estimator_*` topics.
  - For short ID flights only: `275` (+256 raw gyro FIFO), which puts the motor-rotation line in the
    gyro spectrum.
- [ ] `MC_EIG_IZZ 0.032` → `0.015`. This is an effective value. It only feeds the gyroscopic
      feedforward and the seat's model; yaw torque is commanded zero. Valid near r ≈ 8.6, so re-fit
      it as the spin goes up.
- [ ] **Keep** `EKF2_EV_DELAY 12` until a full-rate EV log measures it.
- [ ] **Keep** `MC_SEAT_MODE 0`, `MC_LEAD_EN 0`.
- [ ] **BDShot for motor RPM:** `PWM_MAIN_TIM2 -6`, `PWM_MAIN_TIM3 -6`. Motors are on outputs 5–8,
      which are Timer5 and Timer8.
- [ ] Set `DSHOT_MOT_POL1..4` to the real motor pole count. It is 14 now and unchecked.

## 3. Bench checks (props off)

- [ ] **ESC firmware.** Read the Tekko32 version in BLHeliSuite32 or esc-configurator.com.
      BLHeli_32 ≥ 32.7 or AM32 means BDShot is supported.
- [ ] **BDShot works.** After reboot, `esc_status` shows non-zero RPM and the ESCs online. If not,
      check `dshot status` for BDShot errors. Capture on Timer5/Timer8 on this board is unverified.
- [ ] **No topic-file override.** `/fs/microsd/etc/logging/logger_topics.txt` must not exist,
      otherwise the logger ignores `SDLOG_PROFILE`.
- [ ] **Log rate.** Short armed log: `vehicle_visual_odometry` comes out at ~119 Hz, not 5 Hz, and
      `logger_status` shows no dropouts.
- [ ] **Optional: motor lag on the bench.** Do a throttle step and record it with a microphone
      (blade-pass spectrogram), or put it on a thrust stand. This gives command → RPM lag at
      millisecond resolution.
  - This is the fallback if BDShot does not work.

## 4. Re-fly at the current spin (≈ 8.6 rad/s)

Same setup as log_1, with the step 2 parameters.

- [ ] 30 s or more of steady hover.
- [ ] A few sharp roll/pitch stick pulses. These are needed to separate lag from coupling, which
      are confounded when the content sits near the spin frequency.
- [ ] **Analysis:**
  - [ ] `examples/tools/px4_tuning/ev_delay_from_dynamics.py` → measured EV delay; set
        `EKF2_EV_DELAY`.
  - [ ] Joint fit (gain, κ, lag). κ should now come out near the model value, i.e. the
        cancellation matches the plant.
  - [ ] Tilt error rms/p95 and |ω_xy| against log_1.
  - [ ] Seat observable: coherence and mean misalignment against r·τ.
  - [ ] EKF health: `estimator_status.output_tracking_error`, EV innovations.
  - [ ] If RPM is available: command → RPM lag, and rotor speed (rotor angular momentum).

## 5. Spin ramp

Steps of about **8.6 → 12 → 15 → 18 rad/s**. Each step gets 20–30 s of steady hover plus stick
pulses.

**Record for every step:**

| r | tilt err rms / p95 | \|ω_xy\| | motor p99 | sat. frac | lag | κ | gain | seat R / misalign | heading resid vs EV | battery V / A |
|---|---|---|---|---|---|---|---|---|---|---|
| 8.6 (log_1) | 0.9° / 7° max | 4–8°/s | 0.385 | 0 | 35 ms | −0.1 | 0.79 | 0.21 / 88° | – | 12.2→10.8 / 33 |

**Stop climbing when any of these happens:**
- motor p99 > ~0.8;
- sustained non-zero saturation;
- tilt error roughly doubles from the previous step.

**Hard limits to watch:**
- **Gyro range:** the ICM-42688P is ±2000 °/s = **34.9 rad/s** on the vector norm, so p and q wobble
  counts on top of r.
- **EV heading error:** ≈ r × (delay error), so about 1°/ms at 17 rad/s.
- **Lag rotation:** ≈ r·τ, which passes 30° at about 15 rad/s at 35 ms.
- **Rotor pole:** in sim the actuator pole set the spin ceiling, and hover overstates the
  trajectory ceiling by about 7 rad/s.

## 6. Seat: before it flies on hardware

- [ ] **In sim, give the seat a band-limited observable.** About half the |xa| power in log_1 is
      above 8 Hz.
- [ ] **Understand the broadband reading.** With the observable band-limited, log_1 reads ~60°
      misalignment against a lag model of 17–25°. The motion is broadband (2–10 Hz), not
      spin-synchronous, and one fixed rotation is only correct at one frequency.
- [ ] **Gate for `MC_SEAT_MODE 2` on hardware:** the seat observable is coherent on a seat-off
      hardware log, and its mean sits near the lag rotation at that spin rate. Watch for seat
      runaway, which was the failure mode in sim.

## 7. Tooling

- [ ] Move the joint fit and seat-coherence scripts into `examples/tools/px4_tuning/`. They exist
      only in a session scratchpad today.
- [ ] Write a per-step comparison script: it takes a list of ulogs and prints the step 5 table.
