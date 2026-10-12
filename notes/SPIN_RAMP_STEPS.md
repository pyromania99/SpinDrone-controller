# Spin ramp — steps left

Starting point: `~/Downloads/logs/30th_2tilt/log_1_2026-9-30-21-50-08.ulg` (2026-09-30).
Eigen (ALG=3), seat off, lead off, EKF height from EV, POSCTL, all `CA_ROTOR*_KM = +0.05`.
Latest flight: `~/Downloads/logs/3_10_1_20/log_1_2026-10-7-23-55-48.ulg` (2026-10-07), r ≈ 15 rad/s, 50 s
airborne; see step 5 and `HARDWARE_15RADS_ANALYSIS.md`.

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

- [x] **Log-writer stack.** (built and flashed 2026-10-01) Source already edited: `src/modules/logger/log_writer_file.cpp`,
      `PX4_STACK_ADJUSTED(1170)` → `1600`. Not built yet. log_1 warned
      `log_writer_file low on stack! (272 bytes left)`.
- [ ] **Decide on the EV throttle.** The working tree has an uncommitted edit that sets
      `add_topic("vehicle_visual_odometry", 200)` in `logged_topics.cpp`. Either revert it in the
      same build, or leave it and rely on `SDLOG_PROFILE 19` (step 2).
- [x] `make holybro_kakuteh7_default`, then flash. (done 2026-10-01)
- [x] **Verify:** 2026-10-02 log: `log_writer_file` 880/1584 used (704 B free, was 272), no
      low-stack warnings.

## 2. Parameters

- [x] ~~`SDLOG_PROFILE 19`~~ **Tried 2026-10-02 and it failed.** With BDShot on, `esc_status`
      (496 B, logged every 5 ms by the high-rate bit) plus the replay topics asked ~300 KB/s. The card
      tops out near 190 KB/s, so 45% of the log was dropped: 90 ms kept, 80 ms lost, repeating.
      CPU sat at 99.7%.
- [x] **Use the topic file instead:** copy `examples/tools/px4_tuning/logger_topics_spin.txt` to the
      SD card as `/etc/logging/logger_topics.txt` and set `SDLOG_PROFILE 0`. In this PX4 the file is
      applied AFTER the profile (additive, it overrides intervals), so with profile 0 it is the whole
      set. (flown 2026-10-07: no dropouts, gyro/torque/motors at ~278 Hz, EV ~123 Hz)
  - **Trimmed 2026-10-07 to ~75 KB/s; the trimmed file has not been flown yet.** The first version
    logged 125 KB/s and armed CPU went to 0.89 (default set: 0.70). `esc_status` at 10 ms alone was
    47 KB/s. It is now 50 ms, `actuator_motors` 8 ms, `vehicle_attitude` 20 ms,
    `mc_controller_status` 50 ms. Gyro, torque setpoint and EV stay at full rate.
- [x] `MC_EIG_IZZ 0.032` → `0.015` (in the 2026-10-07 logs). This is an effective value. It only feeds the gyroscopic
      feedforward and the seat's model; yaw torque is commanded zero. Valid near r ≈ 8.6, so re-fit
      it as the spin goes up.
- [x] `EKF2_EV_DELAY 12` → `15`. Measured 2026-10-02 with
      `examples/tools/px4_tuning/ev_delay_long_baseline.py`: log_1 15.8 ms (95% 12.5–22.5), log_2 11.8
      (4.5–24.5), validated by injection. Re-check each step; it is cheap now.
  - [ ] **→ 19 (2026-10-08).** The first long full-rate log (`3_10_1_20/log_1`) measures **18.8 ms**,
    and its yaw innovation bias fits 15 being ~4 ms short. The 3.4 ms from log_9 is withdrawn (too
    short). See `HARDWARE_15RADS_ANALYSIS.md` §4.
- [ ] `MC_EIG_IZZ 0.015` → **`0.022`** (2026-10-08): Izz_eff 0.0223 at ~15 rad/s, R² 0.36.
- [x] **Outer loop: `MC_EIG_XY_D 1.0 → 2.0`, `XY_P` kept at 0.5.** Set between 2 Oct and 6 Oct;
      every log from `3_tilt spin` on has it. That gives ωn 0.71 rad/s and ζ 0.71 → **1.41**
      (overdamped). The 1.0 / 1.6 pair proposed here earlier was not used.
  - **Stopping after stick release got shorter.** Median per log, measured from POSCTL releases at
    ≥ 0.15 m/s with no stick input for the next 4 s:

    | Log | `XY_D` | Release speed | Stop distance | Per m/s | Time to stop | Bounce-back |
    |---|---|---|---|---|---|---|
    | 30th_2tilt/log_1 | 1.0 | 0.19 m/s | 0.23 m | 1.31 m | 1.27 s | 0.20 m |
    | 2_10 log_2 | 1.0 | 0.61 | 0.47 | 0.78 | 1.30 | 0.31 |
    | 3_tilt log_5 | 2.0 | 0.27 | 0.12 | 0.48 | 0.96 | 0.11 |
    | 4_tilt log_8 | 2.0 | 0.59 | 0.34 | 0.58 | 0.92 | 0.24 |
    | 3_10_1_20 log_1 | 2.0 | 0.20 | 0.11 | 0.55 | 1.08 | 0.06 |

    About half the stopping distance per m/s and ~0.3 s quicker. But there are only 1–4 releases
    per log and the spin differs between builds, so treat the numbers as indicative.
  - No sign of trouble at ~15 rad/s: position held within ~0.35 m through the stick inputs.
  - Not tried: a higher `XY_P`. With ζ already 1.41, raising `XY_P` is what would speed up position
    recovery, if it is needed. Alternatively `MPC_POS_MODE 4` (brakes before locking).
    `MPC_VEL_MANUAL` is 10 m/s at full stick.
- [ ] **Keep** `MC_SEAT_MODE 0`, `MC_LEAD_EN 0` until the two changes above have flown once.
- [ ] **Then, at the same build (~15 rad/s): lead on, then seat on (2026-10-08 plan).** Seat on
      at a new, faster build would mix two unknowns. Lead values, updated from `HARDWARE_READINESS.md`
      §1 (which used the August pole spread and `TRQ_MAX 2.265`):

      | Param | Value | How |
      |---|---|---|
      | `MC_LEAD_EN` | 1 | |
      | `MC_LEAD_TC` | **0.015** | The lead inverts the pole T only, not the dead time. The gap-free fit (`HARDWARE_15RADS_ANALYSIS.md` §4) gives T = 20–25 ms. Set it **below** the bottom of that: under-inversion flies, over-inversion crashed in sim (coin flip at +33 %, fatal at +67 %). |
      | `MC_LEAD_DMAX` | **28** | `0.4 · (0.5 · TRQ_MAX) / TC` = 0.4 · 0.5 · 2.11 / 0.015. Always change `TC` and `DMAX` together; `TC · DMAX` is the lead's authority, and the clamp carried the sim result. |
      | `MC_LEAD_WN` | **150**, then 250 | 150 is the default and the quietest; 250 is what level 3 flew. Raise it after a clean first hop. |
      | `MC_LEAD_DLTA` | 0 | 0 means the loop period. |

  - After the lead flight: tilt error and motor p99 against the seat-off 15 rad/s log, and the
    25 ms-averaged minimum motor command again (the lead adds command noise).
  - Seat flight: re-derive `MC_SEAT_K` for this airframe first (§6); watch θ_s for railing at
    `MC_SEAT_THMAX`; keep `MC_SEAT_MODE 0` on a switch. With the lead on, θ_s repeatability is
    **not** a health indicator (`HARDWARE_READINESS.md` §5 item 12). For that check, fly the
    first seat flight with the lead off.
  - If `TRQ_MAX` is changed for the ~0.80 authority gain, recompute `DMAX`.
- [x] **BDShot for motor RPM:** `PWM_MAIN_TIM2 -6`, `PWM_MAIN_TIM3 -6`. Motors are on outputs 5–8,
      which are Timer5 and Timer8.
- [x] `DSHOT_MOT_POL1..4` = 12 (set 2026-10-02). Confirm it matches the magnet count, since RPM scales with it.

## 3. Bench checks (props off)

- [ ] **ESC firmware.** Read the Tekko32 version in BLHeliSuite32 or esc-configurator.com.
      BLHeli_32 ≥ 32.7 or AM32 means BDShot is supported.
- [x] **BDShot works.** 2026-10-02: ~18,000 rpm on all four, published at ~250 Hz.
      `esc_online_flags` reads 0 even so; ignore it.
- [x] **Topic file in place** (see step 2). Correction: the file does NOT replace the profile; it is
      applied after it.
- [ ] **Log rate.** Short armed log: `ulog_info` says **No Dropouts**, EV at ~120 Hz,
      `logger_status.write_rate_kb_s` well under 190, `cpuload` near log_1's ~80%.
  - 2026-10-07, first file: no dropouts and EV 123 Hz, but 143 KB/s, CPU 0.89 and 11k
    `message_gaps` in 13 s. **Redo with the trimmed file.**
- [ ] **Fault log.** The 2026-10-07 log stops at 13.7 s while armed and flying (see step 5). Look on
      the card for `fault_*.log` and for a log after `sess007/log100.ulg`. A fault log means a reset;
      none means power was lost. Then check the battery lead and power-module connector.
- [ ] **Optional: motor lag on the bench.** Do a throttle step and record it with a microphone
      (blade-pass spectrogram), or put it on a thrust stand. This gives command → RPM lag at
      millisecond resolution.
  - This is the fallback if BDShot does not work.

## 4. Re-fly at the current spin (≈ 8.6 rad/s)

**2026-10-02 attempt (`2_10_2026_2_tilt/log_2`):** flew well, but the 45% dropouts make most of the
analysis impossible. What it gave:
- **Motor lag (command → RPM, fitted inside the 90 ms stretches):** first-order T ≈ 25 ms with ~2.5 ms
  dead time on motors 0, 2, 3; motor 1 is slower (~40 ms). The fit is broad: L+T within 5% of best spans
  17–42 ms. Tentative until a gap-free log confirms it.
  - This is consistent with the 35 ms torque → angular-acceleration figure once the gyro and
    derivative filters (~6 ms) are added.
- **Not measurable:** EV delay (95% CI −146..+213 ms) and coupling κ. A validation (log_1 cut into the
  same 90 ms pieces) gave κ anywhere from +0.65 to −0.86, so a number from log_2 would mean nothing.
- `MC_EIG_IZZ` was still 0.032, so this was a repeat of the log_1 configuration.
- Pulses ran 10–60 s. The quiet hover at 60–70 s gave tilt error 0.5° rms (log_1: 0.6°).

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

Spin is raised **mechanically**: eigen commands zero yaw torque, so r is set by the motor tilt and the
prop direction. Each step is a slightly different airframe.

**Before each step's flight:**
- Write down the tilt angle and which motors are tilted, in the log folder name or here. Without them,
  the per-step numbers cannot be interpreted.
- First hop: a low, short hover. Check hover throttle and motor p99 before going on.

**2026-10-07 step (`3_10_1_20/log_9`), 3 × 10° + 1 × 20° (read from the folder name; confirm):**
- Spin 14.6 rad/s steady, against a model prediction of ~15.2. It skipped the 12 rad/s step.
- **Stable and settling:** tilt error 1.1° rms, motor p99 0.389, no saturation, height converging.
  None of the stop criteria fired.
- **The log stops at 13.7 s**, armed and flying, with no disarm or error. Only ~4.5 s are steady, so
  this step needs a re-fly. Lag, κ, gain and seat could not be fitted on so little data.
- r is now 1 rad/s under 15.7 rad/s, where *stock* EKF2 diverges (`HARDWARE_READINESS.md` §0). The
  flashed build has the fix, so the next step is its first hardware test above that speed.

**2026-10-07 re-fly (`3_10_1_20/log_1`), same build:** 50 s airborne at 14.6–15.5 rad/s, stable,
with stick inputs. EV delay 18.8 ms, Izz_eff 0.0223, motor lag ~30 ms. Stick inputs of ~0.25 kept
motors within 0.24–0.51 (25 ms average), so there is room to build taps up to ~0.4. Full write-up: `HARDWARE_15RADS_ANALYSIS.md`.

**Record for every step:**

| tilt (motors, °) | r | tilt err rms / p95 | \|ω_xy\| | motor p99 | sat. frac | lag | κ | gain | seat R / misalign | heading resid vs EV | battery V / A |
|---|---|---|---|---|---|---|---|---|---|---|
| 2 motors, ? | 8.6 (log_1) | 0.9° / 7° max | 4–8°/s | 0.385 | 0 | 35 ms | −0.1 | 0.79 | 0.21 / 88° | – | 12.2→10.8 / 33 |
| 3 × 10° + 1 × 20°? | 14.6 (3_10_1_20/log_9) | 1.1° / 1.6° | 15 / 19°/s | 0.389 | 0 | – | – | – | – | EV yaw innov +4.7° mean | 12.5→11.6 / invalid |
| 3 × 10° + 1 × 20°? | 14.6–15.5 (3_10_1_20/log_1) | 1.1–1.3° / 1.9° (hover) | 15–16°/s | 0.42 (0.49 with sticks) | 0 (motors 0.24–0.51) | ~30 ms | +0.40 | 0.80 | 0.42 / 12° | EV yaw innov +2…+8° | 11.5→11.1 / invalid |

The battery current column is invalid on every log: the current pin reads voltage
(`HARDWARE_TILT10_SPIN_ANALYSIS.md` §6). log_1's "33" is that artefact, not amps.

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
- [ ] **Understand the broadband reading** (causes and fixes: 6a). With the observable band-limited, log_1 reads ~60°
      misalignment against a lag model of 17–25°. The motion is broadband (2–10 Hz), not
      spin-synchronous, and one fixed rotation is only correct at one frequency.
  - 2026-10-08, 15 rad/s seat-off log (`3_10_1_20/log_1`): band-limited reading **12°**, coherence
    0.42, against a lag model of ~30°. So 60° over at 8.6 rad/s and 12° under at 15 rad/s.

- [ ] **Gate for `MC_SEAT_MODE 2` on hardware:** the seat observable is coherent on a seat-off
      hardware log, and its mean sits near the lag rotation at that spin rate (see 6a). Watch for seat
      runaway, which was the failure mode in sim.

### 6a. The broadband reading: what it is, likely causes, proposed fixes (2026-10-08)

**What it is (established).** The seat's angle `arg(xa/xd)` equals the lag rotation r·τ only when
the motion sits at one body-frame frequency. World-fixed demand (the tilt holding a lateral
acceleration) turns at exactly **−r** in the body frame, and that is the component the seat exists
to cancel. Hover motion is spread over 1.8–10 Hz (75% of p's power). A delay L shifts each frequency
by its own phase ω·L: at 5 Hz and 35 ms that is 63°, against 17° (8.6 rad/s) or 30° (15 rad/s) at
the spin frequency. The single angle the seat reads is a power-weighted average over that spectrum.

**Likely causes of the over/under reading (hypotheses, untested):**
1. **Opposite rotation directions cancel.** Content turning one way in the body frame (+ω) is
   rotated by +ω·L, the other way (−ω) by −ω·L. The mix decides whether the average lands above
   the lag (60°) or below it (12°).
2. **Disturbance bias in hover.** `xa` includes the disturbance torque, which sits at the same
   frequency as world-fixed demand, so no filter removes it. The harness needed commanded content
   ~100× the disturbance before θ_s converged. In hover the command *is* the response to the
   disturbance, so the ratio is ~1: hover is the worst case.

**Proposed fixes:**
- **Measure only the spin-frequency component (lock-in).** Write `xd`, `xa` as complex
  (x + i·y), rotate both into the world frame by the heading, low-pass both (below ~0.5–1 Hz), and
  take `arg` of the averaged `xa_w · conj(xd_w)`. World-fixed content becomes near-DC and everything
  else is rejected. This is a complex band-pass centred on −r that follows the spin and separates the
  two rotation directions; the current 0.3–8 Hz real band-pass does neither. A constant heading error
  rotates both signals equally and cancels in the angle, so the EV delay does not bias it. Cheap in
  PX4 (two complex first-order low-passes).
- **Give it signal.** Sustained world-fixed lateral demand: a slow circle or a held stick, not
  station-keeping. Broadband hover motion is the lead's job, since a pole is frequency-dependent and
  one rotation cannot match it.

**Offline checks before any seat flight (on `30th_2tilt/log_1` and `3_10_1_20/log_1`):**
- [ ] Cross-spectrum phase of `xa` vs `xd` against **signed** body frequency, with coherence.
      If the lag model holds it follows ω·L + atan(ωT) + gyro/D-term filter phase, a near-straight
      line; that would confirm the broadband reading is weighting, not a broken mechanism. The phase
      at ω = −r is what the seat should read.
- [ ] The lock-in observable above, on both logs, against r·τ.
- [ ] **Predict the seat's fixed point.** The cross law settles where the mean of
      `|xd||xa|·sin ε` is zero; compute that from the seat-off log. If it says ~12° where the lag
      says ~30°, the seat will only partly compensate, before it ever flies.


## 7. Tooling

- [ ] Move the joint fit and seat-coherence scripts into `examples/tools/px4_tuning/`. They exist
      only in a session scratchpad today.
- [ ] Write a per-step comparison script: it takes a list of ulogs and prints the step 5 table.
