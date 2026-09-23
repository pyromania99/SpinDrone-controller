# One rotor stopped in hover — level 3

> **The question:** what base spin rate does level-3 hover need to survive a single rotor
> loss — is it at or above 25 rad/s?
>
> **The answer: there isn't one in this range.** 20, 25 and 29.5 rad/s all depart within
> 1.9–2.8 s of the rotor stopping, with no trend in spin rate. 25 is not a threshold, and
> raising the spin does not buy fault tolerance. What fails is not the spin — it is that
> the allocator goes on commanding a motor that is no longer there.

Flown 2026-09-19. Nine runs. Method, parameters and the two PX4 hooks involved are in
[`RUN_LOG.md`](RUN_LOG.md); this is what they say.

---

## 1. Every spin rate fails, and fails the same way

![three spin rates](l3_loss_spin_sweep_compare.png)

Seat ON (adaptive), `rotor_tau = 0.02`, rotor 1 stopped ~29 s into each flight:

| r [rad/s] | tilt before | \|p,q\| before | survives the loss by | tilt after |
|---|---|---|---|---|
| 20.1 | 0.22° | 0.087 | **1.9 s** | 11.70° |
| 25.1 | 0.17° | 0.091 | **2.8 s** | 7.07° |
| 29.5 | 0.18° | 0.103 | **1.9 s** | 25.47° |

A repeat of the r = 20 cell gives 1.8 s. The three altitude traces in the figure lie on top
of each other through the departure: **the descent is indifferent to the spin rate.**

All three were in the known-good hover when the motor died — 0.17–0.22° of tilt, θ_s
settled at −0.48 to −0.50 rad, altitude held to 5–7 mm. These reproduce
[`../hover_level3_px4_estimator/`](../hover_level3_px4_estimator/LEVEL3_HOVER.md) at
matched spin (0.22°, \|p,q\| 0.088, θ_s −0.505 there against −0.499 here), so the vehicle
was healthy and the seat had converged before anything was broken.

**There is no ceiling to find here, because nothing survives.** The premise of the question
— that fault tolerance switches on above some base spin — does not hold for this stack.

## 2. What actually kills it: the allocator keeps commanding a dead motor

Normalised motor commands across the failure at r = 20 (seat ON):

| window | motor 1 (stopped) | motor 2 | motor 3 | motor 4 |
|---|---|---|---|---|
| 2 s before | 0.601 | 0.637 | 0.600 | 0.638 |
| 0 – 0.5 s after | NaN | 0.439 (**max 1.000**) | 0.540 (**max 1.000**) | 0.725 (**max 1.000**) |
| 0.5 – 1.5 s after | NaN | 0.473 (max 1.000) | 0.493 (max 1.000) | 0.647 (max 1.000) |

Descent rate over the same interval: **+1.22 m/s** in the first second, **+3.63 m/s** in
the second.

The three survivors saturate within half a second and stay there, while their *mean*
command stays near the pre-failure value. That is the signature of a mixer thrashing on
roll/pitch, not of a vehicle short of thrust:

- the four rotors were holding hover on a command sum of **2.476** out of a possible 4.0,
- so three rotors at full command — 3.0 — exceed the hover requirement by **21 %**.

**The lift to hover on three rotors exists.** It is not delivered, because
`ControlAllocator`'s effectiveness matrix still contains the dead rotor: every roll/pitch
solution it computes assigns part of the job to a motor producing nothing, and the
correction it then demands from the other three saturates them.

This is the honest configuration — it is level 1's convention, where the controller "is not
told, and only sees the consequences through the state" (`motor_failure.py`). It is also
exactly the configuration in which no amount of spin can help, because the defect is in the
allocation, not in the attitude dynamics that spin stiffens.

## 3. Telling the allocator does not work in this rig — and can make it worse

`CA_FAILURE_MODE=1` stops the motor but only *re-allocates* around a failure the detector
has found, which needs `FD_ACT_EN=1`. Three more runs with it on:

| r | what the detector did | outcome |
|---|---|---|
| 20.1 | flagged **all four** motors at t = 10.0 s | crash 2.5 s after the failure |
| 25 | flagged **one** motor at t = 8.7 s | **fell at 9.9 s — 18 s before the injection** |
| 29.5 | flagged all four at t = 15.0 s | crash 1.9 s after the failure |

`FailureDetector::updateMotorStatus()` scores commanded thrust against `esc_status` current
through `FD_ACT_MOT_C2T` (35 A per unit of thrust). The simulated ESC current does not obey
that model, so **the detector fires on healthy motors within 10–15 s of every takeoff.**

The two ways that plays out are both useless and one is dangerous:

- **all four flagged** → `REMOVE_FIRST_FAILING_MOTOR` requires `num_motors_failed == 1` and
  declines, so the run degenerates to §1's arm. The matching crash times (1.9 s at r = 29.5
  either way) confirm nothing else changed.
- **exactly one flagged** → the allocator removes a **healthy** motor from the effectiveness
  matrix, and the vehicle drops from 5.97 m to the ground in about a second, with no
  injected failure anywhere near it.

So re-allocation is not reachable through the detector here. Reaching it needs the injector
to set `motor_failure_mask` as well as `motor_stop_mask` — a PX4 change, not a parameter —
or an `FD_ACT_MOT_C2T` calibrated to the simulated ESC. **Until then, "PX4 re-allocates
around a lost rotor" is untested on this vehicle, and `FD_ACT_EN=1` should stay off.**

## 4. The seat-off arms cannot answer the question above r = 20

![r=20, seat off vs on](l3_loss_r20_compare.png)

| r | seat off, before the failure | reached the failure? |
|---|---|---|
| 20.8 | survives in the **13.66°** coning limit cycle, \|p,q\| 1.598 | yes — then crashes 2.2 s after it |
| 25 | — | **no: crashed at 15.8 s, 13.6 s early** |
| 29.4 | — | **no: crashed at 12.7 s, 15.7 s early** |

Those two are not failures of the experiment: 12.7 s reproduces the archived seat-off
level-3 hover (12.8 s at r = 29.4) almost exactly. They simply mean the seat-off arm has no
flight left to lose at those spins, so **at r ≥ 25 the seat-ON row is the only one that can
be read as a fault-tolerance result at all.**

At r = 20, where both arms do reach the failure, they die 2.2 s and 1.9 s after it. The seat
is worth 62× in attitude before the loss and **nothing after it** — which is the expected
result, not a disappointment: θ_s cancels an actuator *phase*, and a missing rotor is not a
phase error.

## 5. What would have to change

Fault tolerance in the sense the literature means it — Mueller's relaxed hover, where a
quadrotor with three rotors gives up attitude about two axes and controls only the direction
of a primary axis — **is not implemented anywhere in this stack.** The eigen controller
commands full roll and pitch, and the allocator solves for them with a stale effectiveness
matrix. Nothing in the nine runs tests a fault-tolerant control law, because there isn't one
to test; they test what the existing spinning controller does when a rotor stops, and the
answer is that it falls in about two seconds at every spin rate tried.

Three things, in the order they would have to happen:

1. **Make re-allocation reachable.** Either have the injector set `motor_failure_mask`
   directly, or calibrate `FD_ACT_MOT_C2T` to the simulated ESC. Without this, §3's arm
   cannot be flown at all and the "PX4 handles it" claim stays untested.
2. **Re-fly §1's grid with the allocator actually told.** Only then is the spin-rate
   question meaningful: a three-rotor allocation on a rank-3 mixer is exactly determined,
   and whether it holds may well depend on the gyroscopic stiffening that spin provides.
   That is the experiment that could produce a required spin rate.
3. **If it still fails, the relaxed-hover law is the missing piece, not a gain.** That is a
   different controller, not a tuning of this one.

---

## Conclusions

1. **No base spin rate between 20 and 29.5 rad/s makes level-3 hover survive a single rotor
   loss.** Time to departure is 1.9 / 2.8 / 1.9 s at 20 / 25 / 29.5, with no trend, and
   1.8 s on a repeat. **25 rad/s is not a threshold and spin is not the variable.**
2. **The mechanism is allocation, not dynamics.** The three surviving rotors saturate within
   0.5 s while the mixer still assigns work to the dead one, although three rotors at full
   command exceed the measured hover requirement by 21 %.
3. **The seat is irrelevant to this failure**, as it should be: it converged to −0.48…−0.50
   rad beforehand and bought 62× in attitude, and a missing rotor is not a phase error.
4. **`FD_ACT_EN=1` is unusable in SITL** and can crash a healthy vehicle by removing a good
   motor from the mix. Re-allocation is untested on this airframe.
5. **The seat-off arm cannot be read at r ≥ 25**: it crashes 13–16 s before the injection,
   reproducing the archived seat-off hover.
