# The actuator lead — inverting the rotor pole so the seat is left with a pure delay

Added 2026-09-20. After Lee et al., *Toward Stable Position Control for High-Rate Spinning
Small-Sized Quadrotors via a Practical Loop Shaping Scheme*, IEEE/ASME T-MECH 30(6)
5295–5306, 2025.

**Default is off.** `MC_LEAD_EN = 0` and `ActuatorLead(enabled=False)` are bit-identical to
the controller before this existed.

---

## 1. Why this and not more seat

The actuator between a commanded and a delivered torque is roughly

```
Ga(s) = e^{-sL} / (1 + sT)        L transport delay, T rotor/ESC pole
```

Under spin at rate `r`, **both** terms rotate the delivered wrench in the body frame. The
seat cancels that rotation geometrically, and that is all it can do — in particular it
cannot recover the pole's amplitude droop `1/√(1+(rT)²)`.

A lead does the other half. Modelling the rotor as `τ̇ = (u − τ)/T`, the exact inverse of
the pole is

```
u = τ_des + T·d(τ_des)/dt          i.e. multiply by (1 + T s)
```

which cancels the pole **entirely** — phase *and* droop — leaving the actuator looking like
the bare delay `e^{-sL}`. A delay is non-minimum phase and no causal filter inverts it, so
what remains is exactly the seat's job:

| | handles | why |
|---|---|---|
| **lead** | the pole | invertible, needs `T` |
| **seat** | the transport delay | not invertible, needs no model |

**Measured split on this airframe** (`AIRFRAME_ID_METHOD.md`, `ev_delay.ulg`): at 20 rad/s
the plant carries **43.8° of phase — 17.2° delay, 26.6° pole** — and the pole droops 10.6%.

**It should help twice.** The seat is measured to cancel **1.14–1.32** of the true phase
against a pure delay but only **0.39–0.63** once a pole is present. Removing the pole also
moves the seat into the regime where it works.

**It does not compete with the adaptation.** The lead acts on the torque signal, not on
`θ_s`. The seat forms `xd` from the *pre-seat* torque and `xa` from the measurement, so it
simply sees a faster actuator and adapts to the smaller residual. That is the difference
from a static extra rotation, which the adaptation would integrate straight back out.

---

## 2. The paper's published gains do not work

This is the main finding from implementing it, and it is not a detail.

Linearising the source's (19) inside the dead zone, with `k = 1/√δ`:

```
z2/u = β₂k·s / (s² + β₁s + β₂k)
```

a differentiator rolled off at `ωₙ = √(β₂k)` with damping `ζ = β₁/(2ωₙ)`. Its **Table I**
values, `β₁ = 1, β₂ = 500`, give:

| dt | ωₙ | ζ | resonant peak |
|---|---|---|---|
| 1 ms | 125.7 rad/s (20.0 Hz) | **0.0040** | **×126** |
| 4 ms | 88.9 rad/s (14.2 Hz) | **0.0056** | **×89** |

Driven against a known first-order plant that reads **0.894 / −24.5°** uncompensated, those
gains produce **1.410 / −74.7°** — far worse than no compensation.

(The source is also internally inconsistent: its (21) prescribes `β₂ = 1/(2h^0.5)` = 15.8 at
h = 1 ms, against Table I's 500. Both are underdamped; 15.8 puts `ωₙ` at 3.6 Hz, right on
the spin frequency.)

**So this implementation is parameterised by bandwidth**, deriving

```
β₁ = 2ωₙ   (ζ = 1, critically damped)        β₂ = ωₙ²√δ
```

`MC_LEAD_WN` is the knob.

---

## 3. What it buys and what it costs

Against that same plant (T = 25 ms, 20 rad/s, ideal 1.000 / 0°, uncompensated 0.894 / −24.5°),
at 2 kHz:

| `MC_LEAD_WN` | gain | phase | reads as |
|---|---|---|---|
| 100 rad/s | 0.981 | −25.8° | barely helps |
| 150 (default) | 1.046 | −23.7° | marginal |
| 250 | 1.136 | −16.1° | recovers a third of the pole phase |
| 400 | 1.106 | **−5.7°** | recovers three quarters |

Noise cost at 2 kHz with 1% torque noise, commanded sd against a clean 0.707:

| `WN` | 100 | 150 | 250 | 400 |
|---|---|---|---|---|
| sd increase | +10% | +17% | +27% | +27% |

**There is no setting that both inverts cleanly and stays quiet.** The inversion is partial
and bounded by `ωₙ`, which is bounded by noise. **Sweep this in simulation before flying
it — the default of 150 is deliberately conservative and is not a tuned value.**

`(1 + Ts)` itself has gain 1.1× at the spin rate, **7.9× at 50 Hz and 15.7× at 100 Hz**,
which is why the differentiator and `MC_LEAD_DMAX` are load-bearing rather than cosmetic.

---

## 3b. The level-3 result, and the parameter that actually produces it

**Condition.** L3 hover, `rotor_tau = 0.030`, r = 29.5 rad/s — a lag at which the seat alone
crashes. `MC_CTRL_ALG=3 MC_SEAT_MODE=2 MC_POLE_EN=0 IMU_DGYRO_CUTOFF=60`, lead arm
`MC_LEAD_TC=0.030 MC_LEAD_WN=250 MC_LEAD_DMAX=25`, 60 s hold, scored on a 25 s window
opening 10 s after 5.5 m.

**The full 2×2, n = 12.** Neither tool is sufficient alone; the pair is.

| | seat OFF | seat ON |
|---|---|---|
| **lead OFF** | crash **11.3 s** (n=1) | crash 15.8 / 16.4 / 16.5 s (n=3) |
| **lead ON** | crash 14.5 / 14.6 / 15.1 s (n=3) | **FLEW 5/5**, 60 s, tilt 0.16–0.18°, alt sd 5.7–8.5 mm, 0% sat |

The separation is total; no run crosses over, and the within-arm spread is under 1 s. **The
interaction is superadditive**: against the 11.3 s floor the lead alone buys +3.5 s and the
seat alone +4.9 s, so an additive model predicts ~19.7 s — the pair instead flies indefinitely.
They are not two partial fixes for the same deficit, and neither is redundant given the other.

**Every failure is on the spin-up ramp**, short of the commanded 29.5 rad/s:

| arm | r reached at death |
|---|---|
| neither | 25.5 |
| lead alone | 28.1 / 28.4 / 28.8 |
| seat alone | 28.9 / 29.2 |
| both | **29.5, held** |

Each tool carries the vehicle further up the ramp and only the pair clears it. Note this does
**not** close as a pure uncancelled-phase threshold: if the lead inverted the pole outright,
lead-alone would depart with only the ~23 ms pipeline left (37° at r = 28.4) against the
neither-arm's 71°, and it would have flown. Either the in-flight inversion is well short of
complete, or the departure criterion is not phase alone — which is the same unresolved
question as the 23× demand growth in `SESSION_2026-09-18.md` §3c.

**The boundary moved further than one cell.** With `MC_LEAD_TC` matched to the pole, the
whole band where the seat alone dies now flies:

| `rotor_tau` | seat alone | seat + lead | θ_s |
|---|---|---|---|
| 0.020 | flew | — | — |
| 0.025 | crash 18.3 s (θ_s railed at `MC_SEAT_THMAX`) | — | — |
| 0.030 | crash 16.4 s | **flew**, tilt 0.17° | −0.48 |
| 0.040 | crash 13.2 s | **flew**, tilt 0.17° | −0.40 |
| 0.050 | crash 13.8 s | **flew**, tilt 0.21° | −0.30 |

θ_s falls monotonically as the pole grows because the lead is absorbing the added phase and
leaving the seat less to do. The two compose rather than compete. **Caveat:** these cells were
flown with `MC_LEAD_TC = rotor_tau` exactly — they are *matched-model*, and hardware will not
know `T` that well (12.5–40 ms spread). They bound the best case, not the expected case.

### `MC_LEAD_DMAX` is the mechanism, not `MC_LEAD_TC`

The first working run changed **two** things at once — it lowered `DMAX` 200 → 25 and set
`TC = 0.030` — and the result was first attributed to under-inversion, with a Bode argument
built for it. That attribution is wrong. The control run at `TC = 0.030` with `DMAX = 200`
does **not** fly, and four values of `TC` (8, 15, 20, 30 ms) **all** fly at `DMAX = 25`.
The clamp carries the entire effect; `TC` is second order within this range.

Why 200 was wrong: it came from `TRQ_MAX/TC`, i.e. "the derivative has no reason to exceed
full torque within one TC". But `TC * DMAX` is the cap on the lead's *own* contribution, so
that rule lets the feed-forward term alone command the full normaliser — and `MC_EIG_TRQ_MAX`
is only a normaliser, with the allocator saturating near **0.50** of it. The old default
licensed a one-tick transient of roughly **2× the torque the airframe can deliver**, from a
differentiator. Sizing rule now in `lead_params.c`:

```
DMAX ~= f * (0.50 * MC_EIG_TRQ_MAX) / MC_LEAD_TC,   f ~ 0.4     ->  26 here
```

The failure is **nonlinear** — unclamped ESO spikes — so it is invisible in the Bode plot of
`(1 + Ts)`. The linear weak-point analysis of the lead remains true; it was simply not the
operative mechanism. Default changed 200 → **25** in `lead_params.c` and in the level-1 twin.

### θ_s does NOT converge tightly with the lead on

Worth stating because the tree's best-known result is the opposite. The archive's L3 hover at
r = 29.4 gives **θ_s = −0.4731 ± 0.0009 (0.19%, n = 5)**, read as the adaptation landing on
the same fixed point every time. With the lead on, the same n = 5:

| | a | b | c | d | e |
|---|---|---|---|---|---|
| θ_s at 20 s | −0.593 | −0.358 | −0.517 | −0.265 | −0.468 |
| θ_s at 50 s | −0.361 | −0.533 | −0.506 | **−1.044** | −0.377 |
| θ_s final | −0.478 | −0.500 | −0.461 | −0.335 | −0.372 |

**−0.429 ± 0.071 (16.6%)** — two orders of magnitude looser, and run *d* excursions to −1.04
mid-flight and comes back. The vehicle does not care (tilt 0.16–0.18° on every one), but the
reading matters: the lead removes most of the phase, so α is small, so the gradient the seat
descends is weak and noise-dominated. **θ_s is not usable as a lag observable when the lead is
on**, and any future test that reads convergence tightness must be flown lead-off.

The magnitude is consistent with the model, but the convergence ratio is a **bound, not a
measurement**. 0.429 rad at 29.5 rad/s is a residual τ_eff of **14.6 ms**. What it is a
fraction *of* depends on how much of the 30 ms pole the lead actually removed in flight, and
that was measured on a bench plant, not here:

| if the lead inverted… | residual truth | ratio |
|---|---|---|
| all of the pole (23 ms pipeline only) | 0.68 rad | **0.63** |
| a third of it (§3's `WN = 250` bench figure) | 1.27 rad | **0.34** |

So the ratio sits somewhere in **0.34–0.63** — i.e. still inside the documented 0.39–0.63
band for a pole-bearing plant, and nowhere near the 1.14–1.32 of a pure delay. Closing this
needs the in-flight phase measured with the lead on, not inferred.

---

## 3c. Mismatched `TC` — the asymmetry is the operational result

Every cell in §3b was flown with `MC_LEAD_TC` equal to the true pole. Hardware will never
have that. So: **true pole held at 30 ms, believed pole swept** across the 12.5–40 ms spread
`ev_delay.ulg` actually gave, plus 50 ms for badly over-inverted.

**`DMAX` was scaled with `TC` by the §3b sizing rule**, holding `TC*DMAX` at 0.769 N·m. This
is not a convenience: `TC*DMAX` *is* the cap on the lead's own torque contribution, so
sweeping `TC` at fixed `DMAX` would vary the inversion amount and the feed-forward authority
together, and nothing could be attributed. It is also what an operator does — the rule gets
applied to whatever `T` they believe. The clean experiment and the realistic one coincide.

| believed `TC` | `DMAX` | error | outcome | tilt | alt sd | \|p,q\| | rp-sat |
|---|---|---|---|---|---|---|---|
| 12.5 ms | 61.5 | −58% | **FLEW** ×2 | 0.15–0.17° | 5.8–7.0 mm | 0.095–0.099 | 0% |
| 20 ms | 38.4 | −33% | **FLEW** ×2 | 0.17–0.18° | 6.5–7.8 mm | 0.106–0.109 | 0% |
| 30 ms | 25.6 | matched | **FLEW** ×5 | 0.16–0.18° | 5.7–8.5 mm | ~0.100 | 0% |
| 40 ms | 19.2 | +33% | **MARGINAL — 2 crashes / 4 flights, n=6** | 0.17° when it flies | 5.8–10.6 mm | 0.108–0.113 | 0% |
| 50 ms | 15.4 | +67% | **crash 18.1 / 20.7 s** | 13.7° | 702 mm | 8.41 | 85% |

**Under-inversion is free; over-inversion is not.** 12.5 ms is wrong by more than half and is
indistinguishable from matched on every metric. 50 ms fails hard and repeatably.

### 40 ms is a 1-in-3 coin flip with no warning

At n = 6 the cell splits **4 flew / 2 crashed** — crashes at 39.6 s and 44.4 s, flights clean
to 60 s. The first write-up of this row called it a clean failure from n = 1 and had to be
corrected twice; treat single runs near a boundary as uninformative.

**The failure is abrupt and the survivors give no precursor.** Median tilt in 5 s bins from
the moment altitude is reached:

| run | fate | 0–5 | 5–10 | 10–15 | 15–20 | 20–25 | 25–30 | 30–35 | 35–40 | 40–45 |
|---|---|---|---|---|---|---|---|---|---|---|
| a | **crash 44 s** | 0.23 | 0.16 | 0.18 | 0.16 | 0.19 | 0.16 | 0.51 | **31.2** | 32.2 |
| e | **crash 40 s** | 0.23 | 0.17 | 0.17 | 0.17 | 0.21 | 2.94 | **27.5** | 33.2 | 13.7 |
| b2 | flew | 0.23 | 0.17 | 0.17 | 0.16 | 0.17 | 0.17 | 0.17 | 0.18 | 0.16 |
| c | flew | 0.27 | 0.17 | 0.17 | 0.17 | 0.16 | 0.17 | 0.17 | 0.16 | 0.17 |
| d | flew | 0.25 | 0.17 | 0.16 | 0.18 | 0.16 | 0.18 | 0.17 | 0.16 | 0.15 |
| f | flew | 0.24 | 0.17 | 0.17 | 0.17 | 0.16 | 0.17 | 0.18 | 0.17 | 0.16 |
| matched | flew | 0.22 | 0.17 | 0.16 | 0.16 | 0.16 | 0.16 | 0.17 | 0.16 | 0.16 |

The four survivors are **flat at 0.16–0.18° for the entire flight** — identical to matched,
with nothing distinguishing them from the two that died. The two that died were also flat,
until they were not: one bin at 0.51°, then 31°.

**This retracts the "over-inversion warns first" claim.** That came from the crashed run's
*window-averaged* 0.51° tilt and 3× metrics, but the window (10–35 s) clipped the start of an
already-running departure — it was measuring the crash, not a precursor to it. There is no
abort signature. Do not plan a hardware flight around spotting one.

A cell that flies 4 times in 6 with no observable difference between the good and bad runs is
the worst case for a flight test: it passes, repeatedly, and then does not.

This **confirms the "err low" advice** in `lead_params.c`, which until now was reasoning, not
measurement. It also makes it quantitative and changes the recommended setting:

> **Set `MC_LEAD_TC` to the BOTTOM of the measured spread, not the median.** For this airframe
> that is ~12.5 ms, not the 25 ms median. The penalty for being far too low is nil; the
> penalty for being too high is a coin-flip at +33% and the vehicle at +67%.

Note the symmetry that does **not** hold: 20 ms and 40 ms are near-mirror phase errors
(+12.5° under against −12.1° over, by the arithmetic in `lead_params.c`), and 20 ms flies
2/2 clean while 40 ms is a coin-flip. Whatever governs the failure is not the magnitude of
the phase error — the same open question §3b's ramp data raised.

**Not covered:** mis-setting `TC` while leaving `DMAX` at its default. That breaks the
constant-contribution property and is a real operator error mode; untested.

**One void run:** the first 40 ms repeat never took off (`Preflight Fail: heading estimate not
stable`), a launch-time EKF failure unrelated to the compensator. Re-flown.

---

## 4. What is not implemented

The source also closes a PID loop on the torque **error** `τ_des − τ_B` (its eq. 17), which
needs the **delivered** torque fed back. Without rotor RPM telemetry (`DSHOT_BIDIR_EN = 0`
here) the only route to `τ_B` is `ω̇ − drift`, and that reconstruction is exactly what
refuted the seat's magnitude term — it read 0.044 to 1.225 where the truth was 1.000.

Only the feed-forward half is implemented, and that is the half carrying the result: the
source's Fig. 4(a) reaches **1.5 Hz without** the derivative term and **5.5 Hz with** it.

---

## 5. Parameters and cost

| | default | |
|---|---|---|
| `MC_LEAD_EN` | 0 | off / on |
| `MC_LEAD_TC` | 0.020 s | the pole being inverted — **measure it**, 25 ms median here with a 12.5–40 ms spread |
| `MC_LEAD_WN` | 150 rad/s | differentiator bandwidth — sweep it |
| `MC_LEAD_DLTA` | 0 | fal() dead zone; 0 = use dt, which makes tuning rate-dependent |
| `MC_LEAD_DMAX` | **25** | derivative clamp — **this is the parameter that carries the level-3 result**, see §3b |

`MC_LEAD_TC` is forgiving of error but **err low**: against a true 25 ms pole at 20 rad/s,
assuming 12 ms leaves +12.5° under-inverted and assuming 40 ms leaves −12.1° over-inverted.
Over-inversion adds lead the plant does not have and amplifies noise for nothing.

**Board cost, measured:** flash 1 706 160 → 1 707 720 B, **+1560 bytes**, 92.98% → 93.06%,
leaving 124 KB free. CPU, from the source's Table II on the same MCU family: **+830 ns per
cycle**, 0.17% at 2 kHz. RAM ~36 bytes.

---

## 6. Files

| | |
|---|---|
| `PX4 controllers/ActuatorLead.{hpp,cpp}` | the compensator |
| `PX4 controllers/lead_params.c` | `MC_LEAD_*` |
| `PX4 controllers/EigenController.cpp` | applied last, after the seat, before normalisation |
| `examples/utils/actuator_lead.py` | level-1 twin — keep in step, the ladder depends on both rungs running the same law |

Both implementations verified against the same plant and agree to three decimals.

---

## 7. Before this flies

1. **Sweep `MC_LEAD_WN`** at level 1, then level 3, scoring phase recovery against noise.
   Nothing here is tuned.
2. **Measure `MC_LEAD_TC`** on the vehicle it will fly on, at the spin rate it will fly at.
   The 12.5–40 ms spread from a gentle non-spinning log is the dominant uncertainty.
3. ~~**Re-measure the seat's convergence ratio with the lead on.**~~ **Done, §3b:** it is
   **0.63** — the top of the 0.39–0.63 band, not the 1.14–1.32 the pure-delay case shows. The
   inference that removing the pole moves the seat into the better regime is **not supported**.
   What removing the pole does is make the seat's fixed point *loose* (16.6% vs 0.19%), because
   there is little phase left for it to grip. Flight quality is unaffected.
4. ~~**Mismatched `TC`.**~~ **Done, §3c:** under-inversion is free (−58% flies clean),
   over-inversion is fatal (+33% crashes). Set `MC_LEAD_TC` to the **bottom** of the measured
   spread, ~12.5 ms here, not the 25 ms median.
5. **`TC` mis-set with `DMAX` left at default.** §3c held `TC*DMAX` constant by the sizing
   rule; an operator who changes one and not the other is untested.
