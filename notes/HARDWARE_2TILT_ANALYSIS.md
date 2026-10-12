# Hardware analysis — tilted-motor eigen flights, 30 Sep – 2 Oct 2026

Methods and results from the first eigen flights of the tilted-motor spin airframe. Every number
here is reproducible from the logs with the scripts in `examples/tools/px4_tuning/`. Claims
made during the analysis and later withdrawn are listed in §10, so they are not carried forward.

## 1. Data

| Log | Date | Config | Spin r | Usable for |
|---|---|---|---|---|
| `eigen/log_6_eigen_spin.ulg` | 26 Sep | eigen, no tilt (probably), `KM −`, ground effect (12–18 cm) | −2.7 rad/s | cross-log comparison |
| `30th_2tilted_props_against_tilt/log_12_UnknownDate.ulg` | 30 Sep | eigen, 2 motors tilted, `KM −` | +4.0 | cross-log comparison |
| `30th_2tilt/log_1_2026-9-30-21-50-08.ulg` | 30 Sep | eigen, 2 motors tilted, `KM +` | +8.6 | **reference: gap-free** |
| `2_10_2026_2_tilt/log_2_UnknownDate.ulg` | 2 Oct | as log_1, plus BDShot RPM and `SDLOG_PROFILE 19` | +8.5 | RPM, EV delay, outer loop; **45% of data dropped** |

**Common to all four:**
- Flight setup: `MC_CTRL_ALG 3` (eigen), seat off (`MC_SEAT_MODE 0`), lead off, EKF height from EV
  (`EKF2_HGT_REF 3`), POSCTL with `MAN_NORTH_REF 1`, `EKF2_EV_DELAY 12`.
- Controller model: `MC_EIG_IXX/IYY/IZZ 0.017/0.015/0.032`, `TRQ_MAX 2.11`, `WN 30`.
- All four `CA_ROTOR*_KM` have the same sign on all four logs (co-rotating props). The allocator has
  the props flat (`AX/AY = 0`), so the tilt is not modelled.
- The gyro and torque setpoint are logged at ~330 Hz (log_2: ~250 Hz).

**Which flight is which:** log_1 is the only flight that is both gap-free and at the current spin
rate. log_2 repeats its configuration (`MC_EIG_IZZ` was not changed), with RPM added.

## 2. Flight performance

| | log_1 | log_2 |
|---|---|---|
| Airborne | 41 s | 75 s |
| Spin r (steady) | 8.6 rad/s | 7.6–9.1 rad/s |
| Tilt error rms, quiet hover | 0.6–1.2° (rises through the flight) | 0.5° (60–70 s) |
| Tilt error rms, during stick pulses | — | 1.4–2.5° |
| Roll/pitch rate rms | 4–8 °/s | 4–11 °/s (pulses) |
| Motor command mean / p99 | 0.35 / 0.385 | 0.35–0.38 / 0.40–0.43 |
| Torque achieved | 100% | 99.5% |
| Fallbacks | 0 | 0 |
| Hover current | 33 A at ~11.5 V | 33–35 A |

**Ending of log_1:** pilot throttle −0.13 from t+30 s (a slow 4 cm/s descent), stick −1 at
t+40 s, then the kill switch. It was not a failure.

**Roll vibration (log_1 vs log_6):** roll torque in the 30–60 Hz band fell from 0.0117 to 0.0045. The
roll/pitch power ratio in that band fell from ~7 to ~1.6, so the roll amplification seen in log_6 is
gone.

## 3. Roll/pitch plant: authority gain, coupling, lag

**Script:** `coupling_fit.py`

### Method

Band-passed to 0.3–8 Hz, both axes stacked:

```
[ṗ; q̇] = g · [τx/Ixx; τy/Iyy](t − L) + κ · r · [−q; p] + a · [p; q]
```

- **Input:** τ is `vehicle_torque_setpoint × MC_EIG_TRQ_MAX`. The controller's own gyroscopic
  feedforward is inside τ, so κ is the plant's **net** coupling, `(Izz_eff − I)/I` with
  `I = (Ixx+Iyy)/2`. The controller assumes κ = `(MC_EIG_IZZ − I)/I` = +1.0.
- **Measured side:** ṗ and q̇ are PX4's `xyz_derivative`.
- **Fit:** L is scanned from 0 to 80 ms. g, κ and a are least squares at each L. The fit is repeated
  with κ forced to the assumed value.

### Results

| Log | r | best L | gain g | κ | Izz_eff | R² free κ | R² κ = +1.0 |
|---|---|---|---|---|---|---|---|
| log_6 | −2.7 | 40 ms | 1.00 | +0.12 | 0.018 | 0.753 | 0.737 (at 40 ms) |
| log_12 | +4.0 | 30 ms | 0.93 | +0.34 | 0.021 | 0.839 | 0.828 |
| **log_1** | **+8.6** | **35 ms** | **0.79** | **−0.12** | **0.014** | **0.547** | **0.484** |

**Reading the table:**
- **The controller over-cancels in log_1.** Forcing κ = +1.0 costs R² 0.547 → 0.484 at the best lag,
  and κ is negative at every lag from 25 to 45 ms (−0.09 to −0.34). This is the evidence behind
  `MC_EIG_IZZ 0.032 → 0.015`.
- **At low spin, κ is weakly determined.** The κ·r term is small, and forcing κ changes R² by only
  ~0.01–0.02 on log_6 and log_12. Treat the log_6 and log_12 κ values as soft.
- **Lag and coupling are confounded.** At a single frequency, a time delay and a rotation look the
  same, and the body-frame content sits near the spin frequency (1.4 Hz) and at 2–10 Hz. In per-axis
  fits, κ swung from +1.6 to −1.3 over L = 0–80 ms. The joint fit is better behaved, but κ still
  depends on L.
- **Authority gain:** 0.79 in log_1 against 0.93–1.00 earlier, i.e. ~20% less torque per command than
  the `TRQ_MAX/I` model. It is part airframe and part fit (the lag/coupling tangle); see §9.
- **Lag:** 35 ms best fit, matching the 35 ms from earlier PD-era flights.

### Why Izz_eff moved

Unresolved; see §9.
- **Rotor angular momentum** enters like an Izz shift: `Izz_eff = Izz + h_rotor/r`. It would flip
  sign with prop direction, which is consistent with log_12 → log_1, where `KM` flipped and the spin
  doubled.
- **But no single model fits all three logs.** The fitted values imply only ~0.02 N·m·s of total
  rotor momentum, and that model does not predict log_6.

### Gap-free data is required

Validation: log_1 cut into log_2's 90 ms on / 80 ms off pattern (per-run intercepts, no band-pass)
gave κ from +0.65 to −0.86 depending on L, with no R² peak. **No κ is reported for log_2.**

## 4. Seat observable

**Script:** `coupling_fit.py`

### Method

- **Per-tick quantity:** the seat measures the angle between `xd = τ/I` (expected acceleration) and
  `xa = ω̇ − drift(assumed coupling)` (measured acceleration).
- **Recomputed offline:** amplitude-weighted circular mean and coherence R, raw and band-passed.
- **Compared with:** the logged `mc_controller_status.seat_alpha`.

### Results (log_1)

| Signal | Mean misalignment | Coherence R |
|---|---|---|
| logged `seat_alpha` | 88° | 0.21 |
| recomputed, raw | 91° | 0.34 |
| recomputed, 0.3–8 Hz | 59° | 0.73 |
| lag model, r · 35 ms | 17° | — |

- **The raw observable is noise.** The code's own diagnostic, |α| near 90°, means uncorrelated.
  About half of |xa|'s power is above 8 Hz.
- **Band-limited, it is coherent but reads the wrong angle.** It reads ~60°, not the 17–25° the lag
  predicts. The motion is broadband (75% of p's power at 1.8–10 Hz), not spin-synchronous, and one
  static rotation is only right at one frequency, which is the harness finding.
- **The coupling fix doesn't help.** Using `Izz_eff` 0.014–0.016 in the drift leaves the band-limited
  reading at ~60°.
- **The low-spin logs** show the same pattern: logged R 0.18–0.23, band-passed 34–38° against a lag
  model of 5–8°.

**Conclusion:** the adaptive seat is not ready for hardware. It needs a band-limited observable, and
an explanation of the broadband reading, before `MC_SEAT_MODE 2` flies.

## 5. EV delay

**Script:** `ev_delay_long_baseline.py`

### Short-baseline method (`ev_delay_from_dynamics.py`) is unsuitable under steady spin

It compares attitude increments over EV intervals under 0.3 s. Under steady spin, the angular
velocity barely changes across such intervals, so there is almost no information:

| Log | Result | 95% CI |
|---|---|---|
| log_1 (EV at 5 Hz) | 8.2 ms | −169..+387 ms |
| log_2 (gap-aware wrapper, 334 usable intervals) | 30.9 ms | −146..+213 ms |

### Long-baseline method

- **Principle:** a delay τ makes EV attitude lag the gyro. Over a baseline, the increment mismatch
  grows with τ × (change in angular velocity), which is large across spin-rate changes, spin-up
  especially.
- **Pairs:** EV sample pairs 0.5–3 s apart (~9,000–28,000 pairs per log).
- **Mismatch:** for each trial τ, the body-frame mismatch `rotvec(Δq_gyro(t1−τ, t2−τ)⁻¹ · Δq_EV)`.
- **Nuisance terms:** gyro bias (∝ Δt) and scale (∝ gyro increment), fitted per axis by least squares.
- **Estimate:** τ at the cost minimum, parabola-refined. CI from a 2 s time-block bootstrap.
- **Gaps:** gyro gaps are bridged by linear interpolation on a 1 ms grid. The gyro low-pass group
  delay (2.5 ms at 90 Hz) is added.

### Validation

| Test | Result |
|---|---|
| log_1, +20 ms injected into the EV stamps | +19.4 ms recovered |
| log_2, +20 ms injected | +16.1 ms recovered |
| log_1 gyro cut to the 90/80 ms gap pattern | estimate moves 0.3 ms |

### Results

| Log | EV delay | 95% CI |
|---|---|---|
| log_1 | **15.8 ms** | 12.5–22.5 |
| log_2 | **11.8 ms** | 4.5–24.5 |

**Recommendation:** `EKF2_EV_DELAY 12 → 15`.
- **What it buys:** heading error ≈ r × delay error, so 3 ms is ~1.5° at 8.6 rad/s and ~2.5° at
  17 rad/s.
- **Caveat:** timesync is not running (`timestamp_sample == timestamp`), so the figure includes link
  latency and may shift if the link changes.

## 6. Motor lag from BDShot RPM (log_2)

**Script:** `rpm_lag_from_bdshot.py`

### Method

- **Model:** first order plus dead time, `T · dy/dt = K · u(t − L) + b − y`, with u the
  `actuator_motors` command and y the `esc_status` RPM.
- **Data used:** only inside unbroken log stretches. Each is simulated from the measured RPM at
  stretch start + 45 ms; 393 stretches of ~90 ms.
- **Fit:** linear in (K, b) for fixed (L, T); (L, T) on a grid; minimum mean squared output error.

### Results

| Motor | L | T | K (rpm per unit command) | L+T within 5% of the best fit |
|---|---|---|---|---|
| 0 | 2.5 ms | 25 ms | 21,400 | 17–42 ms |
| 1 | 0 ms | 40 ms | 25,300 | 30–60 ms |
| 2 | 2.5 ms | 25 ms | 21,900 | 20–40 ms |
| 3 | 2.5 ms | 25 ms | 22,700 | 17–42 ms |

- **Hover speed:** ~18,000 rpm (≈1,885 rad/s) with `DSHOT_MOT_POL 12`. RPM scales with the pole
  count, so it needs confirming.
- **Consistent with §3:** ~27 ms command → RPM, plus ~6 ms of gyro and derivative filtering, gives
  ~33 ms against the 35 ms torque → angular-acceleration fit.
- **Motor 1 is slower than the others.** Cause unknown: tilted motor, different motor or prop?
- **Tentative:** the fit surface is broad, and the stretches are short. Repeat on a gap-free log.

## 7. Outer loop (log_2 stick releases)

### Method

- **Events:** stick releases after a pulse longer than 0.3 s at peak speed above 0.15 m/s; 5 events.
- **Compared:** `vehicle_local_position` against `vehicle_local_position_setpoint` through each
  release.

### Results

- **What PX4 does at release:** in `MPC_POS_MODE 0`, it locks the position setpoint at the release
  point while the vehicle is still moving.
  - Largest event: released at 0.72 m/s, braked at ~0.6 m/s², stopped 0.57 m past the lock, then
    crept back over ~4 s with no oscillation.
  - Median over the 5 events: 0.44 m creep-back after stopping, peak 0.52 m/s.
- **It's soft PD, not underdamping.** `MC_EIG_XY_P 0.5`, `XY_D 1.0` give ωn = 0.71 rad/s and
  ζ = 0.71. That predicts 0.46 m past the lock from 0.72 m/s.
- **Comparison:** stock PX4's position loop with the logged `MPC_*` settings runs at ωn ≈ 1.3 rad/s.

**Options (sim first):**

| Option | Change | Expected stop past lock from 0.7 m/s |
|---|---|---|
| A | `XY_P 1.0`, `XY_D 1.6` (ωn 1.0, ζ 0.8) | ~0.3 m |
| B | `XY_P 1.7`, `XY_D 2.2` (ωn 1.3) | ~0.23 m |
| C | `MPC_POS_MODE 4` | brakes before locking |

- **Constraint:** keep ωn well under the ~4 rad/s attitude loop.
- **Also check:** `MPC_VEL_MANUAL` is 10 m/s at full stick.

## 8. Logging bandwidth (log_2)

- **Symptom:** 945 dropouts, 38 s of 83 s. Pattern ~90 ms kept, ~80 ms lost, repeating. The 64 KB
  buffer was full, and CPU sat at 99.7% (`log_writer_file` 38%, `logger` 14%).
- **Cause:**
  - `esc_status` (496 B per message, mostly empty ESC slots) is logged every 5 ms by the high-rate
    bit, ~100 KB/s on its own. It was dormant before BDShot.
  - The profile-19 replay topics add to that.
  - Total demand ~300 KB/s against a measured card ceiling of ~190 KB/s. log_1 ran at 120 KB/s with no
    drops.
- **Fix:** `examples/tools/px4_tuning/logger_topics_spin.txt` (~130 KB/s, ID signals at full rate,
  `esc_status` at 10 ms), copied to `/etc/logging/logger_topics.txt` on the SD card, with
  `SDLOG_PROFILE 0`. In this PX4 the topic file is applied after the profile, so with profile 0 it is
  the whole set.
- **Stack fix verified:** after `log_writer_file` was raised 1170 → 1600, it shows 880/1584 used
  (704 B free, was 272), with no warnings.

## 9. Open questions

1. **Why Izz_eff moved** from ~0.02 to 0.014. Decided by two flights in the log_1 configuration at
   different spin rates, plus RPM and prop inertia for the rotor term. Also unanswered: were the props
   physically reversed between log_12 and log_1?
2. **The authority gain of 0.79:** airframe (tilt), fit artefact, or both. Change `TRQ_MAX` only after
   two clean flights agree.
3. **Motor 1's slower response.**
4. **The seat observable's broadband reading** (§4).

## 10. Withdrawn during the analysis

- **"A rotor angular momentum of ~0.04 N·m·s per rotor explains Izz_eff"**, and the Izz_eff-vs-spin
  projection. Both assumed a real Izz of 0.031, which no log has measured.
- **"EV delay is unmeasurable from log_1 (5 Hz EV)."** It was unmeasurable by the short-baseline
  method only (§5).
- **"`logger_topics.txt` replaces `SDLOG_PROFILE`."** In this PX4 it is applied after the profile.

## 11. Scripts

All in `examples/tools/px4_tuning/`:

| Script | What it does |
|---|---|
| `coupling_fit.py` | §3, §4 — gain, κ and lag fit; seat observable coherence. Needs gap-free data. |
| `ev_delay_long_baseline.py` | §5 — long-baseline EV delay; `--chop` and `--inject S` for validation |
| `ev_delay_from_dynamics.py` | §5 — short-baseline method (unsuitable under steady spin) |
| `rpm_lag_from_bdshot.py` | §6 — command → RPM lag |
| `logger_topics_spin.txt` | §8 — logging set |
