# How the airframe numbers were got out of a ulog, and every check that had to pass

Written 2026-09-19, alongside [`HARDWARE_READINESS.md`](HARDWARE_READINESS.md) §2, which
carries the resulting numbers. This file carries the *reasoning*, because most of the work
was not computing anything — it was establishing that each number meant what it looked like.
Everything here is reproducible with
[`../examples/tools/px4_tuning/identify_from_ulog.py`](../examples/tools/px4_tuning/identify_from_ulog.py).

---

## The claim being made, stated narrowly

From `ev_delay.ulg` plus two tape-measure numbers, we obtain `MC_EIG_TRQ_MAX`, `MC_EIG_IXX`
and `MC_EIG_IYY` for the real KakuteH7 airframe. We do **not** obtain `MC_EIG_IZZ`, and
therefore not `λ = (Izz − I⊥)/I⊥` either.

---

## Check 0 — is the log physically capable of answering this?

`vehicle_angular_velocity.xyz_derivative` is the signal the whole identification rests on.
PX4 computes it at the gyro rate and then logs it at whatever rate the logger profile says.

| log | rate | verdict |
|---|---|---|
| `ev_delay.ulg` | 1 kHz nominal, 337 Hz average after 53 s of dropouts | usable |
| `log_0/3/4/5/7/30` | 50 Hz | **aliased — R² ≈ 0.01 on every axis of every one** |

This is not a subtle degradation. At 50 Hz a derivative whose content reaches 20 Hz folds
onto itself, and the regression returns noise. The R² ≈ 0.01 across six logs and three axes
*is* the evidence: a real rigid body cannot fail to correlate torque with acceleration, so
an R² that low means the signal is destroyed, not that the vehicle is strange.

**Rule:** print the sample rate before believing anything downstream. The script refuses
below 200 Hz.

---

## Check 1 — was the controller under test even running?

The parameter dump in `ev_delay.ulg` has **no `MC_EIG_*`, no `MC_CTRL_ALG`, no
`MC_SEAT_*`**. Not "set to defaults" — absent. A parameter that does not exist in a log did
not exist in the flashed binary, so `mc_controller` was not compiled into that build and the
flight was stock cascaded PD.

This matters twice. It means the identification is of the *airframe*, uncontaminated by the
new controller, which is what we want. And it means §0 of the readiness note — which checks
the repo checkout — does not tell you what is on the board. **Verify the binary from a log's
parameter list, not from `git branch`.**

---

## Check 2 — what geometry was the firmware actually mixing with?

This is the step that is easy to skip and changes the answer by 29 %.

The logged `CA_ROTOR*` values are:

```
PX = (-1, 0, 0, +1)     KM = (-0.05, +0.05, +0.05, -0.05)     CT = 6.5
PY = ( 0, +1, -1,  0)
```

Two separate facts fall out.

**(a) It is a plus layout, not an X.** The airframe file `4050_generic_250` sets an X
(`PX/PY = ±1, ±1`, commented "Square quadrotor X"), but `param set-default` only applies to
parameters the user has not set. These were set, deliberately, to a plus — and the KM signs
pair correctly with it (the two x-axis rotors share one direction, the two y-axis rotors the
other). My first pass assumed X from the airframe name and got `TRQ_MAX = 3.20`; the plus
gives **2.26**. The airframe *file* is not evidence. The *log's parameters* are.

**(b) The arms are 1.0 m, which is a lie.** Nobody edited the magnitudes. So
`1/scale = 9.1924` from these parameters describes a two-metre-span quadrotor. **A ulog can
never hand you `MC_EIG_TRQ_MAX`** — it only echoes whatever was typed into `CA_ROTOR*`, and
here that was a placeholder.

---

## Check 3 — is the mixer's layout the same shape as the real frame?

Check 2 says the parameters are right in *direction* and wrong in *scale*. That is an
assumption, and it is testable, because PX4 normalises the mixer using the configured
geometry and then drives the real vehicle with it. The realised wrench per unit command is

```
    realised = B_real @ mix(B_configured)          NOT  1/scale(B_real)
```

and its **off-diagonal terms** are a fingerprint of layout disagreement. Predicted against
measured:

| hypothesis | roll←pitch | pitch←roll |
|---|---|---|
| frame is plus, matching the config | 0.0 % | 0.0 % |
| frame is plus but rotated 10° | 17.6 % | 17.6 % |
| frame is actually an X | **100 %** | **100 %** |
| plus with y-arms 3 cm longer | 0.0 % | 0.0 % |
| **measured in `ev_delay.ulg`** | **3.3 %** | **1.3 %** |

The frame is a plus, aligned with the mixer to within about 2°. An X would have been
unmissable. Note the last row of the table: an arm-length asymmetry produces **zero**
cross-coupling, so this test constrains the layout and the rotation, and says nothing about
arm lengths — which is exactly why the asymmetry question needs a different argument (below).

Independent corroboration: the 2×2 principal axes of the measured effectiveness lie at
178.4° from body x, i.e. along the body axes.

---

## Check 4 — the quantity that needs no airframe knowledge at all

Regress `xyz_derivative` on `vehicle_torque_setpoint` over the airborne window:

```
    c_axis = d(omega)/dt / tau_normalised        [rad/s^2 per unit command]
```

`c` is an input–output fact. It needs no mass, no arm, no `CA_ROTOR*`. It is also **exactly
the eigen law's NDI loop gain**, because the controller multiplies by `I` and divides by
`TRQ_MAX`. So a consistent error in both cancels, and *the ratio is the thing to carry*.

| axis | c [rad/s² per unit] | lag | R² | split-half |
|---|---|---|---|---|
| roll | 117.0 | 35 ms | 0.744 | 18 % |
| pitch | 152.8 | 35 ms | 0.713 | 2 % |
| yaw | 5.3 | — | **0.008** | — |

**On the error bar.** The formal standard error of the slope is ±2 %, and it is worthless:
residuals at 1 kHz are autocorrelated (ρ = 0.81) and the data is closed loop, so the usual
formula counts 21 000 independent samples where there are a few hundred. Splitting the
flight in half and refitting is the honest bar, and it says 18 % on roll.

**Yaw is not a measurement.** `τ_z` never exceeds 0.096 and `ω_z` never exceeds 0.265 rad/s
in this flight. `log_4`/`log_5` do have genuine yaw excitation (`τ_z` rms 0.68, `ω_z` to
1.6 rad/s) — and are the 50 Hz logs killed by Check 0. So `Izz` is in no log we hold.

---

## Check 5 — is the lag correction inventing the asymmetry?

The time-domain fit must guess an input delay, and the slope is sensitive to that guess:

| assumed lag | c_roll | c_pitch | ratio |
|---|---|---|---|
| 0 ms | 103.6 | 105.2 | **0.986** |
| 20 ms | 113.8 | 144.9 | 0.785 |
| 35 ms (best R²) | 116.8 | 152.6 | 0.766 |
| 80 ms | 126.8 | 146.6 | 0.865 |

At zero lag the two axes are identical. **The entire roll/pitch asymmetry appears as the lag
correction is applied.** That is reason enough not to trust it without a second method.

The second method is the cross-spectrum. A pure input delay rotates the phase of `Pxy` but
cannot change `|Pxy| / Pxx`, so a band-limited gain is immune to the lag guess entirely:

| band [Hz] | roll | pitch | ratio | coherence |
|---|---|---|---|---|
| 0.6 – 1.0 | 109.7 | 117.5 | 0.934 | 0.72 / 0.63 |
| 1.0 – 1.5 | 130.9 | 149.9 | 0.873 | 0.79 / 0.75 |
| 1.5 – 2.2 | 119.3 | 150.5 | 0.793 | 0.79 / 0.83 |
| 2.2 – 3.2 | 116.1 | 131.3 | 0.884 | 0.76 / 0.67 |

A rigid body has one inertia ratio at all frequencies. This one wanders between 0.79 and
0.93 across the bands where coherence is good, and the phases agree between axes (−25.1°
vs −22.8°), so it is not a per-axis delay difference either. **The asymmetry is real in sign
but poorly determined in size: somewhere around 15 %, not the 31 % the time-domain fit
reports.** The script prints both and complains when they disagree by more than 0.05.

---

## Check 6 — the absolute scale, and why the sanity check is independent

`CT` comes from the log's own hover, not from a datasheet: over 8042 steady samples
(airborne, |vz| < 0.3, |az| < 0.7) the four normalised motor commands sum to 1.531 ± 0.062,
so `CT = m·g / 1.531 = 3.203 N`, giving thrust-to-weight 2.61 — sane, which is itself a
weak check that the mass is right.

Then `I = realised_torque / c` gives `Ixx = 0.0194`, `Iyy = 0.0148`.

The check that makes this trustworthy: `Ixx = 2·μ·R²` for **both** X and plus layouts (four
motors at lever `R/√2`, or two at lever `R` — same answer), so `Ixx = 0.0194` at `R = 0.5 m`
implies **39 g per motor + prop + ESC**. That is the right order for this airframe, and it
was never fitted to anything — it comes from a torque-vs-acceleration regression on one
side and a mass budget on the other. Two independent routes agreeing is the strongest
single piece of evidence here.

If that number had come out at 200 g or 5 g, the arm length or the layout would have been
wrong.

---

## What each input's error does

| input | error | effect |
|---|---|---|
| mass 500 g, −50/+15 g | −10 / +3 % | `TRQ_MAX`, `Ixx`, `Iyy` all scale with it |
| arm 0.50 m ± 3 cm | ±6 % | same — `TRQ_MAX ∝ R` |
| both, combined | | `TRQ_MAX` 1.92 – 2.47, `Ixx` 0.0164 – 0.0211 |
| **layout X vs plus** | | **29 % on `TRQ_MAX`** — settled by Check 3, not by assumption |
| **the ratio `c`** | | **unaffected by all of the above** |

The last row is the point. Mass and arm set the *split* between numerator and denominator;
they do not touch the loop gain. The dangerous error is not a sloppy tape measure, it is
taking `TRQ_MAX` from one source and `I` from another that was built at a different scale.

---

## Check 7 — `MC_EIG_TRQ_MAX` is a normaliser, not the airframe's torque limit

Added 2026-09-19. Everything above derives `TRQ_MAX` as *the physical N·m one unit of
normalised torque produces*, which is correct and is what PX4's allocator scales by. It is
**not** the most torque the airframe can make, and treating it as such is 2× optimistic.

[`experiments/phase_only/analysis/SESSION_2026-09-18.md`](../experiments/phase_only/analysis/SESSION_2026-09-18.md)
§3c measures the real ceiling for the simulated vehicle:

> `τ_xy^max = k_f · b · (2U − (S − 2U)),  U = min(S/2, ω_max²),  S = mg/k_f`
> = **1.91 N·m** … against `MC_EIG_TRQ_MAX` = 3.843 N·m which is only the *normaliser*.
> **So the real saturation threshold is ≈ 0.50 of normalised torque, not 1.0.**

The reason is the thrust budget: roll torque comes from *differential* thrust, and the
differential available is bounded by the hover thrust the rotors are already holding, not by
the allocator's normalisation.

For this airframe (plus layout, arm 0.50 m, `CT` 3.20 N, hover Σu = 1.531), one rotor pair
can go to the rails while the other pair carries the thrust:

```
τ_roll^max = CT · R · 1.0 = 1.60 N·m        = 0.71 of the 2.265 N·m normaliser
roll  cap  = 1.60 / 0.01936 = 82.7 rad/s²
pitch cap  = 1.60 / 0.01483 = 108.0 rad/s²
```

against the sim's 1.91 / 0.03477 = **54.9 rad/s²** — i.e. this airframe carries roughly
**1.5× the sim's angular-acceleration authority**, and archived level-3 hover demand (p95
`|xd|` 5.1–16.2 rad/s²) sits at 7–20 % of it.

**Verify before quoting.** The 1.60 N·m figure is derived here for a plus layout; §3c's
formula is for an X, where all four rotors contribute to each axis. Re-derive it in §3c's
form for this geometry — the "hardware is the stronger vehicle" claim rests on it, and it is
the one number in this file that has not been checked two ways.
