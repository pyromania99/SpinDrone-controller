# Hardware readiness — PX4 build, what the eigen controller needs, and pre-flight checks

Written 2026-09-19, before the first hardware flight of the eigen + seat controller.
Target board: **Holybro KakuteH7**. Everything below was read out of the repos, not
recalled — re-verify after any branch change.

Companion documents: [`EKF2_OUTPUT_PREDICTOR_SPIN_FIX.md`](EKF2_OUTPUT_PREDICTOR_SPIN_FIX.md)
for the estimator bug, [`AIRFRAME_ID_METHOD.md`](AIRFRAME_ID_METHOD.md) for how §2's
numbers were got out of a flight log, and
[`../experiments/analysis/SESSION_2026-09-18.md`](../experiments/phase_only/analysis/SESSION_2026-09-18.md)
for the simulated envelope these checks are calibrated against.

---

## 0. Build state — VERIFIED 2026-09-19

| check | state |
|---|---|
| branch | **`motor_failure`** ✅ |
| `src/modules/mc_controller/` present | ✅ 15 entries |
| EKF2 output-predictor fix in history | ✅ 2 commits |
| `CONFIG_MODULES_MC_CONTROLLER=y` in `boards/holybro/kakuteh7/default.px4board` | ✅ |

**Why the branch matters.** `ekf2-output-predictor-spin-fix` and `main` carry the EKF2 fix
but **do not contain the controller at all** — `src/modules/mc_controller/` does not exist
on them. Only `motor_failure` has both. If the checkout ever moves, re-run:

```bash
cd ~/PX4-Autopilot
git branch --show-current                       # must be motor_failure
ls src/modules/mc_controller/ | wc -l           # must be non-zero
git log --oneline HEAD -- src/modules/ekf2/EKF/output_predictor/output_predictor.cpp
```

Build the board target, not SITL:

```bash
make holybro_kakuteh7_default
```

**The EKF2 fix is not upstreamed.** It exists only in this clone. A fresh checkout, a CI
build, or a colleague's machine will not have it, and without it `vehicle_attitude` diverges
above **r_crit = (π/2)/τ = 15.7 rad/s** — well inside the intended operating range.

Submodule pointers show as modified in `git status`. Confirm that is intentional before a
release build.

---

## 1. THE BIG ONE — all the gains live in a SITL-only file

Every eigen/seat parameter is set in:

```
ROMFS/px4fmu_common/init.d-posix/airframes/10016_none_iris
```

**`init.d-posix` is SITL only. It is never loaded on hardware.** No airframe under
`init.d/airframes/` sets `MC_CTRL_ALG`. A fresh flash therefore runs **firmware defaults**,
which are not what was flown:

| parameter | firmware default | flown in sim | note |
|---|---|---|---|
| `MC_CTRL_ALG` | 1 (cascaded PD) | **3 (eigen)** | PD crashed at every condition tested |
| `MC_EIG_WN` | 18.0 | **34.1837** | |
| `MC_EIG_B` | 10.2 | **0** | |
| `MC_EIG_ALPHA` | 0.5 | **0** | |
| `MC_EIG_BETA` | 0.6 | 0.55858 | |
| `MC_EIG_ATT_P` | 7.0 | **4.1186** | |
| `MC_EIG_ATT_D` | 1.0 | **0.05** | |
| `MC_EIG_Z_P` / `Z_D` | 10.0 / 5.3 | 9.27 / 4.94 | |
| `MC_EIG_XY_P` / `XY_D` | 0.5 / 1.0 | 0.5 / 1.0 | only pair that matches |
| `MC_EIG_IXX/IYY/IZZ` | 0.01 / 0.01 / 0.02 | **0.03477 / 0.03126 / 0.06266** | airframe-specific; hardware values in §2 |
| `MC_EIG_TRQ_MAX` | 1.0 | **3.843** | derived; hardware value 2.265, see §2 |
| `IMU_DGYRO_CUTOFF` | 20 | **60** | 20 Hz adds ~0.33 rad of phase at spin |
| `MC_LEAD_EN` | 0 | **1** | off by default; the 2026-09-20 result needs it on |
| `MC_LEAD_TC` | 0.020 | **≈0.0125** | the BOTTOM of the measured 12.5–40 ms spread, not the 25 ms median — over-inversion is a coin flip at +33 % and fatal at +67 % |
| `MC_LEAD_WN` | 150 | **250** | |
| `MC_LEAD_DMAX` | 25 | **≈36** | `0.4·(0.50·TRQ_MAX)/TC`, and TRQ_MAX is **2.265 on hardware, not 3.843** — the sim value 25 is wrong here. At TC = 0.0125 the rule gives 36.2. |
| `MC_CTRL_GT` | 0 | 1 in sim, **must be 0 on hardware** | §3 |

**Action:** write a hardware airframe file under `init.d/airframes/`, or a loadable `.params`
set. Do not set these by hand at the bench.

---

## 2. Three numbers that must be re-derived, not copied

*Updated 2026-09-19: flight logs now bracket `TRQ_MAX`, `IXX` and `IYY`. `IZZ` still open.*

### `MC_EIG_TRQ_MAX` — a derived quantity, not a tuning knob

`examples/tools/px4_tuning/derive_px4_params.py` computes it as **1/scale**, replicating
PX4's own `ControlAllocationPseudoInverse::updateControlAllocationMatrixScale()` for the
airframe's `CA_ROTOR*_CT/KM/PX/PY`. It is *the physical N·m that one unit of normalised
torque produces*. 3.843 belongs to the simulated Iris and to nothing else.

That script also records a structural fact worth carrying: **the allocation matrix is
rank 3** whenever all four `CA_ROTOR*_KM` share a sign — yaw is collinear with thrust and is
not independently controllable. That *is* the spin premise, encoded in the allocator.
**Canting the motors changes this** and will also require `CA_ROTOR*_AX/AY/AZ` for the
tilted axes.

**Working hardware value, 2026-09-19.** Derived in
[`AIRFRAME_ID_METHOD.md`](AIRFRAME_ID_METHOD.md), which documents every check that had to
pass; reproduce with:

```bash
python3 examples/tools/px4_tuning/identify_from_ulog.py \
        ~/Downloads/logs/ev_delay.ulg --mass 0.500 --arm 0.50
```

The airframe is a **plus layout**, not an X. The `4050_generic_250` airframe file sets an X,
but the stored parameters override it with a plus and the KM signs pair correctly with that;
the measured roll/pitch cross-coupling (3.3 % / 1.3 %, against 100 % predicted for an X)
confirms the frame matches. **This is worth 29 % on `TRQ_MAX`** — do not take the layout
from the airframe name.

`CT` comes from the log's own hover: over 8042 steady samples the four normalised motor
commands sum to **1.531 ± 0.062**, so `CT = m·g / 1.531`. With takeoff mass ≈ 500 g
(−50 / +15 g) and arm ≈ 0.50 m centre-to-motor (± 3 cm):

| quantity | value | range over the stated tolerances |
|---|---|---|
| `CA_ROTOR*_CT` | **3.20 N** | 2.88 – 3.30 |
| `MC_EIG_TRQ_MAX` | **2.265** | 1.92 – 2.47 |

Thrust-to-weight is 2.61, which is sane. Caveat: `CT` assumes PX4's linear control→thrust
map and the board has `THR_MDL_FAC = 0.3`, so do not trust it better than ~10 %.

**`CA_ROTOR*_PX/PY` must be corrected before flashing.** They currently hold unit arms
(1.0 m), which is why the parameters as logged imply `TRQ_MAX = 9.19`. The mixer still flew
because it is normalised — only the *ratios* of the positions reach the mix — but
`MC_EIG_TRQ_MAX` reads the magnitudes, and so does anything that computes a physical torque.

**`KM` does not reach `TRQ_MAX`** on this layout: the plus mixer puts roll on one rotor pair
and pitch on the other, so the roll and pitch columns of the pseudo-inverse are exactly
decoupled from the yaw row. Flipping all four `KM` to one sign — the spin conversion — moves
`rank(B)` from **4 to 3** and leaves `TRQ_MAX` at 2.2649 unchanged. The roll/pitch numbers
measured below therefore survive the prop flip.

### What `ev_delay.ulg` fixes, and what it cannot — added 2026-09-19

`~/Downloads/logs/ev_delay.ulg` is a real KakuteH7 flight (branch `motor_failure`, build
`154dc85`, ~65 s airborne) and it is **the only log in that folder with usable dynamics**: it
logs `vehicle_angular_velocity` at 1 kHz — 337 Hz average after 53 s of dropouts — where
`log_0/3/4/5/7/30` log it at 50 Hz, which aliases `xyz_derivative` into noise (R² ≈ 0.01 on
every axis of every one of them).

Two things it is *not*:

- Its parameter dump contains **no `MC_EIG_*`, no `MC_CTRL_ALG`, no `MC_SEAT_*` at all**. The
  flashed binary did not carry `mc_controller`; that flight was stock cascaded PD. §0 checks
  the *repo*, not the board — after flashing, re-verify against a log's own params.
- Its `CA_ROTOR*` values are the untouched `4050_generic_250` defaults: `CT = 6.5`,
  `KM = ±0.05`, `PX/PY` a **plus layout with 1.0 m arms**. `px4_torque_scale()` on them
  returns `TRQ_MAX = 9.192`, describing no real vehicle. **A ulog cannot hand you
  `MC_EIG_TRQ_MAX`; it only echoes whatever was typed into `CA_ROTOR*`.**

What it does fix is the *ratio*, from regressing `vehicle_angular_velocity.xyz_derivative`
on `vehicle_torque_setpoint` over the airborne window (26.5–89.1 s, n = 21121):

| axis | `TRQ_MAX/I` [rad/s² per unit] | best lag | R² | slope, 1st vs 2nd half |
|---|---|---|---|---|
| roll | **117.0** | 35 ms | 0.744 | 18 % apart |
| pitch | **152.8** | 35 ms | 0.713 | 2 % apart |
| yaw | 5.3 | — | **0.008** | not identified |

`TRQ_MAX/I` *is* the eigen law's NDI loop gain — the controller multiplies by `I` and divides
by `TRQ_MAX`, so an error in one cancels an error in the other as long as the ratio holds.
**Carry the ratio, not the numerator.** The 35 ms is actuator-plus-filter lag, and is an
independent measurement of the delay the seat has to lead.

### `MC_EIG_IXX / IYY / IZZ` — two of three now bracketed

Dividing the `TRQ_MAX` above by the measured ratio:

| | derived | range | sim value it replaces |
|---|---|---|---|
| `MC_EIG_IXX` | **0.01936** | 0.01638 – 0.02113 | 0.03477 |
| `MC_EIG_IYY` | **0.01483** | 0.01254 – 0.01619 | 0.03126 |
| `MC_EIG_IZZ` | **0.03419** *assumed* | upper bound | 0.06266 |

Independent check, not part of the derivation: `I = 2·μ·R²` holds for both X and plus, so
`Ixx = 0.01936` at `R = 0.5 m` is **39 g of motor + prop + ESC per arm** — the right order
for this airframe. The log-derived inertia and the mass budget agree without having been
fitted to each other.

**`IZZ` is assumed, not measured (2026-09-24).** `Izz = Ixx + Iyy = 0.01936 + 0.01483 =
0.03419` by the perpendicular-axis theorem. That is exact only for a planar lamina: any mass
above or below the rotor plane (battery, FC stack) adds to `Ixx`/`Iyy` but not to `Izz`, so
the true value is **at most** this. Replace it once a yaw-excited log exists
(`log_2_2026-9-24-07-34-36.ulg` has τ_z ≤ 0.069 and ω_z ≤ 0.31 rad/s — nothing to fit).

**The roll/pitch split is soft — do not rebuild the frame over it.** The time-domain fit
says `Ixx/Iyy = 1.31`, but the whole asymmetry appears only once the 35 ms lag correction is
applied (at zero lag the two axes are 103.6 and 105.2, identical), and the delay-immune
cross-spectrum gives 1.19 over 0.5–3 Hz and wanders between 1.07 and 1.26 across bands where
a rigid body must give one constant. Best estimate ≈ **15 %, ± most of itself**.
[`AIRFRAME_ID_METHOD.md`](AIRFRAME_ID_METHOD.md) Check 5 has the full tables.

Geometry cannot be the cause. For a plus layout, moving one rotor pair out by δ raises its
lever linearly and its inertia quadratically, so `c` falls as `1/δ` — and the arithmetic
gives:

| rotor-pair asymmetry | predicted `c_roll/c_pitch` |
|---|---|
| +2 cm | 0.96 – 1.00 |
| +3 cm | 0.94 – 0.99 |
| needed to explain the measurement | **+10 to +34 cm**, depending on how much inertia sits in the body |

A 2–3 cm build tolerance buys 1–4 %. It is not the explanation, and it is far inside the
measurement's own error. Note also that an arm asymmetry produces **zero** roll/pitch
cross-coupling, so it cannot be detected or excluded by the layout test — only by the
magnitudes.

**Does the asymmetry matter at spin?** Only through the gyroscopic term. The torque path
is self-correcting: `I = TRQ_MAX/c` is built from the measured accel-per-command, so the NDI
gain is right on both axes whatever the physical cause. But `ω × Iω` uses the values as
*true* inertias, and if the split is a measurement artefact the feedforward carries a
`ΔI·ω_tilt·Ω` error:

| spin rate Ω | error at ω_tilt = 1 rad/s | the gyroscopic term it perturbs |
|---|---|---|
| 5 rad/s | 1.0 % of authority | 3.8 % |
| 10 rad/s | 2.0 % | 7.6 % |
| 15 rad/s | 3.0 % | 11.3 % |
| 20 rad/s | 4.0 % | 15.1 % |

So a wrong split is a ~27 % error on the term the feedforward exists to cancel, and the seat
absorbs the remainder. The alternative — using the mean `I⊥ = 0.0171` on both axes — costs
−12 % / +15 % of torque-path gain instead. Neither is clearly better at present, which is
another reason to fly the doublet rather than pick.

**`Izz` is in none of these logs.** In `ev_delay.ulg`, `τ_z` never exceeds 0.096 and `ω_z`
never exceeds 0.265 rad/s — there is nothing to fit. `log_4` and `log_5` do have real yaw
excitation (`τ_z` rms 0.68, `ω_z` to 1.6 rad/s) but only 50 Hz gyro. So
`λ = (Izz − I⊥)/I⊥` — which sets the gyroscopic precession rate the NDI feedforward cancels,
and so feeds straight into the residual the seat must absorb (`control_doc.md` Part V.7,
row 5 of the Part VII table) — **still needs new data: a yaw doublet with
`vehicle_angular_velocity` logged at ≥ 250 Hz, twenty seconds of it.** The weight-accurate
CAD model remains worth building, but with `I⊥ ≈ 0.024` bracketed it now has only one number
left to get right.

### `MC_SEAT_K` — dimensional, will not transfer

`MC_SEAT_K = 0.02` is the CROSS-law gain and **absorbs the units of |xd||xa|**, so its
effective bandwidth scales with operating amplitude. A different airframe's torques and
inertias move that amplitude.

The warning is already in the record: level 1's `Seat()` default of **k = 2.0**, tuned on a
1.5 m hover, **railed θ_s on 95 % of ticks** when flown on a 5 m/s circle. Expect to retune,
and watch for railing (§5).

---

---

## 2b. The lag budget — and the half of it that is not measured

Added 2026-09-19, after the question "is the 35 ms total lag including mocap delay?"

**No. The 35 ms is inner-loop only.** It was measured between `vehicle_torque_setpoint` and
`vehicle_angular_velocity.xyz_derivative`. Both sit inside the rate loop, the gyro is a
direct onboard sensor, and mocap appears nowhere in that path. Decomposed:

| term | delay | source |
|---|---|---|
| `_lp_filter_velocity`, `LowPassFilter2p` (2nd order) @ `IMU_GYRO_CUTOFF` = 40 Hz | 5.63 ms | `√2/(2πfc)` |
| `_lp_filter_acceleration`, `AlphaFilter` (1st order) @ `IMU_DGYRO_CUTOFF` = 50 Hz | 3.18 ms | `1/(2πfc)` |
| ESC + motor spin-up + loop discretisation | **26.19 ms** | remainder |
| **measured total** | **35 ms** | |

Notches are off (`IMU_GYRO_NF0_FRQ` = `NF1_FRQ` = 0, `IMU_GYRO_DNF_EN` = 0), so they add
nothing here. **About 4 ms is free:** `IMU_GYRO_CUTOFF` is at 40 where the airframe file asks
for 90 (−3.1 ms), and `IMU_DGYRO_CUTOFF` at 50 where the sim flew 60 (−0.6 ms). The
remaining 26 ms is physical and only new ESCs or props will move it.

**But do not credit that 4 ms to the seat.** Both filters are in the *measurement* path, so
they cost feedback margin, not wrench rotation — the next section separates the two, and the
35 ms above is a point estimate of a quantity that is really a range.

### The actuator: a range, not a number — and droop is the small term

**Superseded 2026-09-19.** An earlier version of this section fitted a single (L, T) pair,
concluded droop was what kills level 3, and proposed a `MC_SEAT_GLAW` gain term to recover
it. That was wrong twice over and the gain code has been reverted. What follows is what the
data actually supports.

Fitting the same model on **8 s windows with 4 s overlap** across the airborne stretch of
`ev_delay.ulg`, both axes, keeping windows with R² > 0.45 (n = 20):

| quantity | median | range |
|---|---|---|
| transport delay `L` | 15.0 ms | 0 – 40 ms |
| pole `T` | 25.0 ms | 12.5 – 40 ms |
| `L`, actuator only (less 5.63 ms gyro LPF) | 9.4 ms | 0 – 34.4 ms |
| `T`, actuator only (less 3.18 ms dgyro filter) | 21.8 ms | 9.3 – 36.8 ms |

**`L` and `T` trade off — both produce phase — so the split is poorly determined.** Quoting
"L = 11 ms, T = 23 ms" as though it were measured overstates what one flight can tell you.
What *is* determined is the effective delay `τ_eff = φ/Ω`: **28–31 ms median, range 9–53 ms**.

### Phase is the problem; droop is a rounding error next to it

| spin Ω | phase to cancel, `ΩL + atan(ΩT)` | droop loss, `1 − 1/√(1+(ΩT)²)` |
|---|---|---|
| 10 rad/s | 17.7° [5.3 – 30.6] | 2.3 % [0.4 – 6.2] |
| 15 rad/s | 25.8° [8.0 – 45.7] | 5.0 % [1.0 – 12.5] |
| **20 rad/s** | **33.4°** [10.6 – 60.5] | **8.3 %** [1.7 – 19.5] |
| 25 rad/s | 40.3° [13.1 – 75.0] | 12.2 % [2.6 – 26.4] |
| 30 rad/s | 45.8° [15.6 – 89.2] | 16.3 % [3.7 – 32.9] |

At the 20 rad/s target the seat has 33° of rotation to cancel and the uncompensated droop
costs 8 %. Level-3 hover runs at 5–20 % of the real authority ceiling (below), so an 8 % loss
of commanded wrench has an order of magnitude of margin underneath it.

**Do not net the filter saving off the seat's lag.** Raising `IMU_GYRO_CUTOFF` 40 → 90 saves
3.1 ms and `IMU_DGYRO_CUTOFF` 50 → 60 another 0.6 ms, but both sit in the **measurement**
path. They buy feedback margin. The wrench never sees them.

**`MC_SEAT_THMAX = 1.4 rad` (80°) is a real ceiling.** It binds at 55 rad/s on the median
lag, 28.7 rad/s at the 90th percentile and 21.4 rad/s on the worst window. Not binding at 20,
worth watching above it.

### Why droop is not the mechanism — and what is

[`analysis/SESSION_2026-09-18.md`](../experiments/phase_only/analysis/SESSION_2026-09-18.md)
§3c measured a **23× separation in torque demand** between flying and failing arms, and says
plainly: *"Rotor-pole attenuation gives 1.14× at r = 27.4 and misdirection by ψ gives ~2×;
neither reaches 23×."* Droop is 1.14× of a 23× effect whose mechanism is **underived**. That
is the open problem; the gain term was aimed at the wrong term.

Two corrections follow from the same section, and they matter more than the droop tables:

**`MC_EIG_TRQ_MAX` is only the normaliser.** `τ_xy^max = 1.91 N·m` against a normaliser of
3.843, so *"the real saturation threshold is ≈ 0.50 of normalised torque, not 1.0"*. Anything
computed as `TRQ_MAX / I` is **2× optimistic**.

**The archive's droop ordering does not generalise.** §3d finds droop orders the hover data
but records that *"the circle contradicts it (L1 flies at droop 0.888, L3 fails at 0.970),
and every cell is n = 1"*. An earlier version of this note claimed "no level-3 run survives
this actuator above ~22 rad/s" on the strength of that ordering. **Withdrawn** — it rested on
a correlation the source already flags as contradicted.

### Authority: hardware is the stronger vehicle

Using the real ceiling rather than the normaliser:

| | sim | hardware (`ev_delay.ulg`) |
|---|---|---|
| real `τ_xy^max` | 1.91 N·m (0.50 of normaliser) | ~1.60 N·m (0.71) — see caveat |
| roll angular-accel cap | 54.9 rad/s² | **82.7 rad/s²** |
| L3 hover demand, p95 `|xd|` | 5.1 – 16.2 rad/s² | 7 – 20 % of the hardware cap |

So hardware carries ~1.5× the sim's angular-acceleration authority and hover sits well inside
it. **Caveat:** the 1.60 N·m figure is derived here for a *plus* layout (one rotor pair to the
rails, the other carrying thrust); §3c's formula is for an X. Re-derive before quoting it —
the "hardware is stronger" claim rests on it.

**On hardware, `T` is a prop property.** Cutting it to ~13 ms would hold droop ≥ 0.95 at
25 rad/s. With 0.5 m arms the props are large and `J_prop` dominates, so smaller and lighter
props — or more torque, from higher KV or more than the present 3S — is the lever. Worth
doing for the phase, not for the droop.

### The mocap path — uncompensated, but NOT on the path to a stabilised hover

| finding | value |
|---|---|
| `EKF2_EV_DELAY` | **0.0 ms** |
| `vehicle_visual_odometry.timestamp_sample − timestamp` | **0.00 ms** (max 0.12 ms over 2147 samples) |
| `EKF2_EV_CTRL` | 11 = bits 0+1+3 = horizontal position + vertical position + **yaw** |

The second row is the defect: the bridge stamps the sample time with the *arrival* time, so
the capture time is discarded before PX4 sees it, and with `EKF2_EV_DELAY = 0` the EKF
believes the pose is current. The ESP32 bridge has since been fixed to carry a capture stamp.

**Correction 2026-09-24: the bridge did NOT answer TIMESYNC until today.** The 2026-09-22
sketch never read its UART and still described T4 as optional. PX4's `Timesync::sync_stamp()`
returns the *arrival* time until 500 TIMESYNC replies have been accepted, so in
`log_2_2026-9-24-07-34-36.ulg` every EV sample has `timestamp_sample == timestamp` exactly,
and `ev_delay_from_dynamics.py` re-measures the old bridge's delay (12.4 ms, CI 9.1–18.5).
`serviceFcRx()` now answers the requests, and all stamps come from one 64-bit
`esp_timer_get_time()` clock. Needs the T4 → ESP32 GPIO16 wire; sync takes ~50 s after both
ends are up (10 Hz requests, 500 samples, RTT < 10 ms each). Check it in the next log: the
logger records `timesync_status` again, and EV `timestamp − timestamp_sample` must be non-zero.

**Scope it correctly, though.** Two facts, both verified rather than assumed:

- **Roll and pitch are never fused from EV.** `EKF2_EV_CTRL = 11` is bits 0, 1 and 3 —
  horizontal position, vertical position, yaw (`src/modules/ekf2/params_external_vision.yaml`).
  Attitude comes from accel + gyro.
- **Attitude does not wait on a pose.** In `ev_delay.ulg`, `vehicle_attitude` publishes at a
  **median 5.02 ms (≈ 200 Hz)** against mocap's true 119 Hz. That is the output predictor
  running at IMU rate — exactly what
  [`EKF2_OUTPUT_PREDICTOR_SPIN_FIX.md`](EKF2_OUTPUT_PREDICTOR_SPIN_FIX.md) bought.

So for the first hardware flight — spin, hold attitude, hold height — the mocap path touches
only the yaw datum and the position loop. The seat and the rate loop never see it. At the
expected **11–16 ms** the yaw datum error at 20 rad/s is **12.6–18.3°**, and in hover the
horizontal acceleration command that error misdirects is ≈ 0.

| spin Ω | τ = 11 ms | 16 ms | 30 ms |
|---|---|---|---|
| 10 rad/s | 6.3° | 9.2° | 17.2° |
| 15 rad/s | 9.5° | 13.7° | 25.8° |
| 20 rad/s | 12.6° | 18.3° | 34.4° |

Where it **does** bite is translation — and that term is already a named suspect in the 23×
demand question: `SESSION_2026-09-18.md` §3c lists *"misdirection by ψ gives ~2×"*. Measuring
the EV delay therefore belongs with the translation work and the 23× derivation, not ahead of
a stabilised hover.

### Three attempts to measure it from `ev_delay.ulg`, all of which failed

1. **Timestamps** — destroyed by the bridge, as above.
2. **Innovation against velocity** (`innov = −v·τ`). Mocap is the *only* horizontal aiding
   source, so the filter simply settles onto the delayed measurement and the steady-state
   innovation goes to zero regardless of τ. Fit gives −3 ms, R² = 0.048, over 70 points at
   under 1.2 m/s. No information, and structurally so — this method needs a second,
   non-delayed position source.
3. **Mocap-derived body rate against the gyro** — independent and non-circular, the right
   method. Gave +9.8 / −14.4 / −39.7 ms on x/y/z with peak correlations 0.69 / 0.66 / 0.38.
   Two of the three are negative, which is physically impossible. Cause: the logger dropped
   72 % of the EV stream, so differentiating consecutive quaternions across the gaps is
   mostly noise.

**The mocap link itself is healthy.** The stream runs at 119 Hz, and all 350 of the EV gaps
longer than 50 ms coincide with a logger dropout — 100 %, not a majority. What looks like a
flaky mocap feed in this log is the SD logger, not the network.

### CLOSED 2026-09-22 — measured from the dynamics: **12.0 ms (95 % CI 10.0–15.2)**

`examples/tools/px4_tuning/ev_delay_from_dynamics.py`, run on `ev_delay.ulg`.

**Method.** A mocap sample stamped `t` describes the vehicle at `t − τ`, so the attitude
*increment* mocap reports between two of its samples must equal the increment the gyro
integrates over the same interval shifted back by τ. The gyro is integrated **once** into a
cumulative attitude `Q(t)`; `Q(a)⁻¹Q(b)` is exactly the increment over `[a,b]`, so bias
accumulated before `a` cancels and a trial τ costs a lookup. Sensitivity is carried by
angular **acceleration** — `d/dτ ∫ω dt = ω(t1−τ) − ω(t2−τ)` — so intervals whose rate does
not change are dropped (`|Δω| > 0.02 rad/s`); 822 of 2146 EV pairs survive that and the
outlier cut.

**Why this succeeds where attempt 3 failed.** Attempt 3 finite-differenced consecutive EV
samples to get a rate. 30 % of EV intervals here are gaps > 12 ms and the contiguous runs
have a **median length of 2 samples**, so it was differentiating across holes — hence its
physically impossible negative delays. This method differentiates nothing on the mocap side:
a gap is a longer baseline, and a 200 ms increment is better conditioned than an 8 ms one.

**The answer is a null measurement, not a scan minimum.** Near zero the cost surface is
shallow and the estimate is compressed — measured gain ≈ 0.3 below 10 ms — so reading off
the minimum understates it. Instead: inject known shifts into the EV timestamps, find the
injection at which the estimator reads **zero**, negate it. Validated by injection over
0–80 ms: `recovered = 0.918 × injected + offset`, R² = 0.99. Independent check: the SSE
improvement is **exactly 0.0 %** at the null point, which is what a correct alignment must
give.

**Two corrections that are not optional.**

1. `vehicle_angular_velocity` is low-passed by a **2-pole** `LowPassFilter2p` at
   `IMU_GYRO_CUTOFF` (`sensors/vehicle_angular_velocity/VehicleAngularVelocity.hpp:132`).
   Group delay `√2/(2πfc)` = **5.63 ms** at 40 Hz, and it is **added**, because the scan
   measures `τ_EV − τ_gyro`. Using the 1-pole formula understates the answer by 1.6 ms.
2. `sensor_gyro_fifo` is *not* a better source despite being unfiltered: it carries only
   **4 valid samples per 1 ms record** across two device scales, so unpacking it naively
   produces a fictitious 45 kHz stream.

| | |
|---|---|
| relative to the filtered gyro | +6.3 ms (95 % 4.4–9.5) |
| gyro 2-pole LPF at 40 Hz | +5.63 ms |
| **EV delay** | **12.0 ms, 95 % CI 10.0–15.2** |

This lands inside the **11–16 ms** expected in this section and it is a **direct measurement**
rather than an expectation.

**⚠ DO NOT SET `EKF2_EV_DELAY = 12`.** `ev_delay.ulg` is dated **2026-08-22** and the bridge
was rewritten to carry the true capture time on **2026-09-22**. So 12.0 ms is **what the OLD
bridge cost**, i.e. the size of the problem the fix was built to remove — not a value to
apply on top of the fix. Applying it now would over-correct by roughly its own size.

The fixed bridge's own diagnostics, running 2026-09-22, are consistent with that reading:
capture→send `delay last` runs **1.3–8.5 ms** typical against the 12.0 ms total measured
before, with `stamp rej: 0` (the clock filter is accepting every stamp). Re-measure with this
tool on a log from the new bridge and set `EKF2_EV_DELAY` from **that**; expect a small
residual, not 12 ms.

At 20 rad/s a 12.0 ms datum error is **13.8°** of yaw (95 % CI 11.5–17.4°) — the same order
as the estimate that section already carried, so nothing downstream changes.

**This cannot be repeated on the flight logs.** `vehicle_visual_odometry` is logged **only**
in `ev_delay.ulg`; `log_0/3/4/5/7/30` do not have it, and their `estimator_aid_src_ev_*`
topics run at **2 Hz**, which cannot resolve a 10 ms delay. **Add `vehicle_visual_odometry`
to the logged topic set** — at 119 Hz it costs a fraction of `sensor_gyro_fifo`, which is
being dropped anyway.

### To close it (superseded by the above; kept for the reasoning)

1. **Fix the bridge to put the true capture time in `timestamp_sample`.** This is the real
   repair; `EKF2_EV_DELAY` then becomes a small residual trim rather than the whole
   correction.
2. If that is not possible, measure τ with method 3 above — but **fix the logging dropouts
   first**, or raise the `vehicle_visual_odometry` rate and log to RAM. Wave the vehicle by
   hand with sharp yaw reversals, cross-correlate against the gyro, and subtract 5.63 ms for
   the gyro filter. Then set `EKF2_EV_DELAY`.
3. Until one of those is done, **treat every spin result as carrying an unknown yaw datum
   error of order Ω·τ**, and do not attribute residuals to the seat.

## 3. Things that silently do not exist on hardware

- **`MC_CTRL_GT` must be 0.** At 1 the controller subscribes to
  `vehicle_attitude_groundtruth` / `vehicle_local_position_groundtruth`, whose only publisher
  anywhere in the PX4 tree is `ReplayEkf2.cpp`. On a real vehicle `.update()` never returns
  true, state is never refreshed, and the 50 ms `MC_CTRL_WD_MS` watchdog drops into the
  fallback cascade. **Level 2 of the simulation ladder does not exist on hardware** — every
  hardware flight is level-3-style.
- **The `gps.py` radians/degrees fix is simulator-only.** Real receivers emit degE7 and PX4's
  driver has always handled it; there is no reprojection step to get wrong.
- **Spin rate probably will not transfer.** The simulated `--motor-moment` (6e-7, giving
  k_m/k_f = 70.2 mm) is **~4.4× a physical propeller's drag coefficient** (~16 mm). All-CW
  rotation on real hardware yields roughly a quarter of the simulated yaw torque. Equivalent
  cant angles, if the cant supplies the shortfall:

  | target r | cant | vertical thrust lost |
  |---|---|---|
  | 29.4 rad/s | 24.6° | 9.1 % |
  | 25.0 rad/s | 19.6° | 5.8 % |
  | 20.0 rad/s | 14.1° | 3.0 % |

  That lift loss lands on the constraint that is already binding (§5).

---

## 4. What the eigen controller uses

### Parameters it reads

| group | parameters |
|---|---|
| eigen law | `MC_EIG_XY_P`, `XY_D`, `Z_P`, `Z_D`, `ATT_P`, `ATT_D`, `WN`, `B`, `ALPHA`, `BETA`, `IXX`, `IYY`, `IZZ`, `TRQ_MAX` |
| framework | `MC_CTRL_ALG`, `MC_CTRL_GT`, `MC_CTRL_WD_MS` |
| seat | `MC_SEAT_MODE`, `LAW`, `K`, `THMAX`, `RMIN`, `KSIGN`, `TAU_A`, `T` |
| adaptive pole | `MC_POLE_EN`, `K`, `MAX`, `RMIN`, `DITH`, `FDIT`, `KES`, `AMIN` |
| outer loop | `MC_OL_XY_I`, `MC_OL_Z_I`, `MC_OL_I_LIM` |
| indirect (sets `TRQ_MAX`) | `CA_ROTOR*_PX/PY/CT/KM`, `CA_ROTOR_COUNT`, `CA_AIRFRAME`, `MPC_THR_HOVER` |
| filtering | `IMU_DGYRO_CUTOFF` |

### uORB subscriptions

`vehicle_angular_velocity` (callback work item — the control trigger), `vehicle_attitude`,
`vehicle_local_position`, `vehicle_control_mode`, `vehicle_status`, `vehicle_land_detected`,
`trajectory_setpoint`, `vehicle_constraints`, `manual_control_setpoint`,
`vehicle_attitude_setpoint`, `vehicle_rates_setpoint`, `hover_thrust_estimate`,
`control_allocator_status`, `battery_status`, `autotune_attitude_control_status`,
`parameter_update`.

Plus `vehicle_attitude_groundtruth` and `vehicle_local_position_groundtruth`, used **only**
when `MC_CTRL_GT = 1` (SITL).

### uORB publications

`vehicle_torque_setpoint`, `vehicle_thrust_setpoint`, `vehicle_attitude_setpoint`,
`vehicle_rates_setpoint`, `vehicle_local_position_setpoint`, `takeoff_status`,
`actuator_controls_status_0`, `mc_controller_status`, `rate_ctrl_status`.

`mc_controller_status` is the diagnostic topic: `seat_theta`, `seat_alpha`, `seat_stepped`,
`pole_theta_b`, `pole_alpha_b`, `torque_sp[3]`, `algorithm`, `fallback_active`, and
`debug[5..7]` = allocator-saturated flag, |xd|, |xa|.

**Note:** `seat_stepped` is a **new field added 2026-09-18** — an ABI change to a logged
message. Anything parsing older logs must tolerate its absence.

---

## 5. Pre-flight checks

### A — bench, no props

1. `param show MC_EIG_*`, `MC_SEAT_*`, `MC_CTRL_ALG`, `MC_CTRL_GT`, `IMU_DGYRO_CUTOFF`.
   Confirm intended values, **not defaults** (§1).
2. Confirm the eigen law is actually running: `mc_controller_status.algorithm == 3` and
   `fallback_active == 0`. This field is trustworthy — it correctly reported `1` throughout
   the cascaded-PD baseline runs.
3. Re-derive `MC_EIG_TRQ_MAX` and the inertias for the real airframe (§2). Do not inherit
   3.843.
4. **Measure the real `rotor_tau`.** The simulated envelope says level-3 trajectory tracking
   dies between **5 and 10 ms** of rotor pole on top of transport. Where the real ESC/motor
   pair sits on that axis is currently unknown and is the difference between flying and not.
5. **Measure the real transport delay** and compare against the **19.5 ms** the seat inferred
   in simulation (recovered independently at two rungs, agreeing to 0.1 ms). If hardware is
   materially higher, the whole envelope shifts.
6. Confirm `mc_controller_status` is logged at a useful rate; it is the only record of
   `MC_SEAT_*` / `MC_POLE_*` behaviour.

### B — tethered, props on, **seat OFF**

7. Spin up at **low spin first**. Seat-off is stable below the boundary, which isolates
   airframe, allocation and estimator problems from the seat.
8. **Watch `torque_sp` against ≈ 0.5 normalised, not 1.0.** `MC_EIG_TRQ_MAX` is the
   *normaliser*; the airframe delivers about half of it
   (τ_xy^max = k_f·b·(2U − (S − 2U)) ≈ 1.91 N·m against 3.843). In simulation the failing
   arms sat at **0.92–0.94 with a motor pinned 87 % of ticks**; the flying arms at **0.04**.
   **If hover demand exceeds ~0.3, stop and investigate.**
9. Check `debug[5]` (allocator saturated). Sustained saturation is the measured failure mode,
   not a symptom of it.
10. Raise spin past 15 rad/s and compare `vehicle_attitude` against an external reference. If
    it diverges, the EKF2 fix is not in the flashed binary (§0).
11. Record actual `r_ss`. Do not assume the simulated spin transfers (§3).

### C — seat ON

12. Watch θ_s converge, and **check repeatability across runs**. Simulation gives
    **0.19 % spread (n = 5)** where the seat works and **40 %** where it fails — the cheapest
    in-flight health indicator available.

    **⚠ THIS INDICATOR IS VOID IF `MC_LEAD_EN = 1`.** Measured 2026-09-20, n = 5 with the
    lead on and the vehicle flying perfectly (tilt 0.16–0.18°): θ_s spread is **16.6 %**,
    with one run excursing to −1.04 and returning. The lead removes most of the phase, so α
    is small and the seat's gradient is noise-dominated — a loose θ_s under the lead means
    *nothing is left for the seat to grip*, not that the seat is failing. Judging a
    lead-on flight by this criterion would abort a healthy vehicle. If you want the
    indicator, fly the check with `MC_LEAD_EN = 0`.
13. **Confirm θ_s is not railing at `MC_SEAT_THMAX` = 1.4.** Railing means `MC_SEAT_K` is
    wrong for this airframe, and is the most likely hardware surprise (§2).
14. Keep a switch to `MC_SEAT_MODE = 0` reachable in flight. Below `MC_SEAT_RMIN` = 5 rad/s
    the seat is inert regardless.
15. Score attitude, not survival. The r ≈ 20 seat-off case survives a full flight **inside a
    13.4° coning limit cycle**; it would pass any "did it fly" test.

---

## 5b. Where the envelope sits after the actuator lead — 2026-09-20

The lag argument changed sign today. Effective delay at r = 29.5, computed as
`tau_eff = (23 ms pipeline) + atan(r·T)/r` — an arithmetic that reproduces the measured
40.5–40.8 ms at `rotor_tau = 0.020` to within 0.6 ms:

| condition | `tau_eff` | outcome |
|---|---|---|
| sim `rotor_tau = 0.020` | 41.1 ms | flies, seat alone |
| sim `rotor_tau = 0.025` | **44.5 ms** | **seat alone crashes at 18.3 s** |
| **hardware, retuned filters (90/60)** | **43.1 ms** | — |
| **hardware, as flown today (40/50)** | **46.8 ms** | — |
| sim `rotor_tau = 0.030` | 47.6 ms | **seat + lead flies 5/5** |
| sim `rotor_tau = 0.040` | 52.4 ms | seat + lead flies |
| sim `rotor_tau = 0.050` | 56.1 ms | seat + lead flies |

**Before the lead, hardware sat 1.4 ms below the measured seat-alone crash boundary** — i.e.
on it, with no margin and inside the measurement's own error. That is what the "hardware
straddles the boundary" language in §2b was describing.

**With the lead the flying envelope reaches 56.1 ms**, putting hardware's retuned 43.1 ms
**13 ms / 30 % inside** it, and even the un-retuned 46.8 ms inside it. This is the first time
any configuration in this tree has flown a plant laggier than the hardware.

Three qualifications, none of which are small:

1. **Hover only.** No trajectory profile has been flown with the lead on at any lag.
2. **`TC` matched.** The 0.040 and 0.050 cells set `MC_LEAD_TC` to the true pole. Hardware
   cannot. The mismatched sweep (`ACTUATOR_LEAD.md` §3c) was run only at a true pole of
   0.030, so the 52/56 ms cells are matched-model and bound the best case.
3. **The margin is against a lag figure with 6× spread.** `tau_eff` on hardware ranges
   9–53 ms across windows; 43.1 is the retuned median, not a worst case. At the p90 the
   margin is gone.

---

## 6. Open items that bear on hardware

- **No validated analytic limit.** The linear cascade predicts r_crit ≈ 48; simulation
  measures 27.4, invariant across a 4.3× rate-loop bandwidth sweep. Two candidate mechanisms
  were proposed and refuted on 2026-09-18. The failures are **saturation-driven**, which the
  current theory does not model.
- **Most envelope cells are n = 1**, and one condition is confirmed 1-in-4 bimodal.
- **Margin is thin.** 93 % of actuator ceiling in the failure cases. On hardware that margin
  is worse, not better — battery sag, motor derating, wind.
