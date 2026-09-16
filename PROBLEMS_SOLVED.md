# What it took to fly a spinning quadrotor

A summary of the problems that had to be solved, in the order the stack hits them, to get a
quadrotor with all four rotors turning the same way flying at 20–30 rad/s of body spin.
Each entry is one paragraph; the full working record is in
[notes/FINDINGS.md](notes/FINDINGS.md),
[notes/EKF2_OUTPUT_PREDICTOR_SPIN_FIX.md](notes/EKF2_OUTPUT_PREDICTOR_SPIN_FIX.md) and
[notes/NEXT_STEPS.md](notes/NEXT_STEPS.md).

The work is organised as a **ladder**, and most of these problems were only separable
because of it — the value of a rung is its *difference* from the one below:

| level | what is in the loop |
|---|---|
| **1** — Pegasus + Python | Control law and plant, nothing else. No PX4, no estimator, no transport delay. |
| **2** — PX4 + ground truth | Real PX4 mixer, limits, allocator, normalized torque. `MC_CTRL_GT=1`, estimator out. |
| **3** — PX4, estimator in | `MC_CTRL_GT=0`. EKF2 back in the loop. |

Roughly: **level 1 answered whether the law can hold the spin, level 2 answered what PX4's
implementation costs, and level 3 was blocked on two genuine PX4 estimator bugs.**

---

## Summary

| # | problem | bit at | fix | effect |
|---|---|---|---|---|
| 1 | Cascaded PD cannot hold high spin | L1 | eigen law, `M_eff = M − A_r` | +0.224 1/s growth → +0.0001 1/s |
| 2 | `A_r` gyroscopic sign backwards | L1 | sign from real inertia difference | rate-loop gain flat in `r` instead of 0.16 at r=60 |
| 3 | `b` believed to be damping | L1 | corrected: `b` is a phase, not decay | stopped tuning the wrong knob |
| 4 | Actuator/rotor lag decides everything | L1–L3 | modelled as `rotor_tau`; then compensated | 12.9 s → 90 s at 30 rad/s |
| 5 | Lag rotates the command frame | L1–L3 | **adaptive seat** (wrench rotation `θ_s`) | L2: 8.0 s → **48.9 s** airborne at 20 rad/s |
| 6 | `M_dyn` pole angle `b`, adaptively | L1–L2 | extremum seeking (the ad-hoc law had no fixed point) | −0.7 pt on top of the seat; cannot replace it |
| 7 | PX4's thrust model was wrong | L2 | `input_scaling`/`CA_ROTOR*_CT` matched | 1.6–2.3× torque error → ratio 1.0000 |
| 8 | Vertical accel coupled into tilt | L2 | gravity-only `body_z` | takeoff worked at all |
| 9 | Nothing you set persisted; the wrong vehicle flew | L2 | airframe `set-default` + `check_airframe()` gate | 34 logs were of a Gazebo x500 |
| 10 | Ground-truth position 57.2958× too small | L2–L3 | radians → degrees in `gps.py` | this was the position **feedback**, not just the logs |
| 11 | Timed trajectories run away under spin | L1–L3 | arrival-based waypoint advance | error capped by the acceptance radius |
| 12 | EKF2 filtering at 100 Hz vs a 250 Hz IMU | L3 | `EKF2_PREDICT_US 10000 → 4000` | ~12 s departure → 163 s at 7.9 rad/s |
| 13 | EKF2 output predictor diverges under spin | L3 | frame rotation + exact angle extraction | 68.21° → **0.76°**; L3 flies at 25.9 rad/s |

---

## 1–3. The control law (level 1)

**Cascaded PD is structurally wrong for this vehicle.** At ~29.5 rad/s it diverges at
+0.224 1/s (doubling every 3.1 s) while holding altitude perfectly — it is *station* that
is lost. The mode is identified, not guessed: its frequency is 30.58 rad/s against a body
spin of 29.69, i.e. it rotates with the body. That is the `(Iyy − Izz)·q·r` Euler coupling,
which on this airframe carries 26.8 1/s of roll↔pitch cross-gain. PD damps each axis
independently and has no term of that form, so no `Kd_att` recovers it. The eigen law's
`M_eff = M − A_r` cancels exactly that term and holds the same plant at +0.0001 1/s.
(§6b)

**`A_r` had the wrong sign, which made it worse than nothing.** The off-diagonal block was
hardcoded `[[·,+r],[−r,·]]` — the sign for a rod, `Izz < Ixx`. Every planar multirotor has
`Izz ≈ Ixx + Iyy`, so subtracting that block *applied the coupling a second time*. The
error is proportional to `r`, invisible at hover: the rate loop's delivered-to-commanded
gain fell from 1.0 to 0.40 at r = 20 and 0.16 at r = 60, and by r = 60 the pitch channel
had the wrong sign outright. (commit `593decc`)

**`b` is a phase, not damping.** With the gyroscopic term cancelled the rate error obeys
`Ω_err(t) = Ω_err(0)·e^{−wn·t}·e^{+i·b·t}`: `b` is the imaginary part of a *first-order*
eigenvalue, so it sets the direction the error spirals while `wn` alone sets the decay. It
cannot stabilise the coning mode, and it costs margin, since delay acts at
`|wn − i·b| = √(wn² + b²)`. Months of tuning `b` as if it were a damping term were
retracted on this argument and confirmed in flight (`b` = +19.3 and −5.0 both diverged
where `b` = +10.2 flew). `M_dyn` was later reparameterised as `a·R(θ)` so magnitude and
phase are separate knobs. (§8, commit `92c12a2`)

## 4–6. Actuator lag — the thing that actually sets the ceiling

**Lag, not spin rate, is the binding constraint.** Level 1, eigen, same command, 30 rad/s:
`rotor_tau = 0` flies 90 s with `|eR| ≈ 0.005`; `rotor_tau = 20 ms` departs at 12.9 s with
`|eR|` 300× worse. One unmeasured number decides whether the vehicle flies. Loop latency at
level 2 measures 12 ms, which against `MC_EIG_WN = 24` is 16.5° of phase versus 2.8° for
250 Hz discretization — **delay dominates sample rate by ~6×**, so margin is won on latency,
not loop rate. (§6, §6c)

**`b` cannot buy the lag back.** A 56-flight sweep of `b_mdyn` × `rotor_tau` shows the
largest stable `b` collapsing 28 → 12 → 4 → nothing as `tau` goes 5 → 10 → 15 → 20 ms. `b`
and `tau` do not trade off; the lag simply closes the window `b` lives in. A `b` chosen on
the instant-response plant is not merely detuned on a lagged one, it is unflyable. (§6d)

**The fix that worked is the seat: rotate the commanded roll/pitch wrench by `θ_s`.** Under
spin a delay is, to first order, a *frame rotation* — state is measured at `t − tau`, torque
lands at `t`, and the body has turned `r·tau` in between, so every correction is misdirected
by that angle. `θ_s` is driven adaptively by the angle between the lagless-expected angular
acceleration and the measured one. In a reduced-model harness `θ_s` converges to the
actuator's exact discrete phase (4 decimals) and recovers 76–90% of the lag's tracking
damage. In flight:

| test | seat off | seat on |
|---|---|---|
| L1, waypoint ellipse, 25 rad/s, 20 ms lag | crashed, \|ep\| 1.417 | flew 110 s, \|ep\| 0.900 (**−36.4%**) |
| L2, fast circle, 20 rad/s, 20 ms lag | **8.0 s** airborne, ground at 11.4 m/s | **48.9 s**, 3.3 orbits, altitude sd 1 mm |

At `rotor_tau = 0` the seat is correctly **inert** (statistically identical flight) — it
finds the level-2 pipeline lag, converging to θ_s ≈ 0.42–0.55 rad against a computed 0.615,
and does nothing else. The variable that makes the seat matter is **lag, not speed or
lateral demand**, which is worth knowing before designing another "push it harder" test.
Straight lines are a weak probe (commanded tilt ~0.6°); use circles.

Getting it onto PX4 took two bugs that were only found by reconstructing the observable
offline from a healthy log where the answer is known: the expected acceleration `xd` was
double-counting the gyroscopic feedforward (88% of the observable was bookkeeping error,
which railed `θ_s`), and `MC_SEAT_K = 2.0` was inherited from an angle-form law where `k` is
dimensionally different by ~200×. Also `IMU_DGYRO_CUTOFF` at its 20 Hz default contributes
~0.33 rad of its own phase at a 4.56 Hz spin, which the seat reads as actuator lag — runs
use 60 Hz.

**Adapting the pole angle `b` is a real but small win, and only on top of a stabilised
flight.** The first adaptive-`b` law was shown to be ill-formed — its error signal is
strictly positive for all `|θ_b| < π/2` and zero only *at* the realizability rail, so it
converges robustly to a useless place; it destroyed a 90-s-stable level-2 flight in 3–9 s.
Replacing it with extremum seeking against an exogenous reference (dither + spin-synchronous
averaging + demodulated gradient) works, and is worth 0.7 points on top of the seat's 36.4 —
but `b`-only still crashes, so it is a refinement, not a substitute.

## 7–11. Porting to PX4 (level 2)

**PX4's model of the vehicle was wrong, and the error moved with throttle.** `CA_ROTOR*_CT`
said `thrust = 6.5·u²` while Pegasus produced `8.54858e-6·(1000u + 100)²` — 2.34× at u=0.3,
1.81× at hover, 1.59× at full. Altitude looked fine because `mc_hover_thrust_estimator`
self-calibrates the collective; there is no equivalent for torque, so the attitude loop
absorbed the whole error. Fixed in two halves (`input_scaling = 1100` so the curve passes
through the origin, `CA_ROTOR*_CT = 10.344`), now agreeing to ratio 1.0000 at every
throttle. (§3)

**Vertical acceleration must not drive tilt.** PX4 commands `acceleration[2] = 100` as a
pre-takeoff sentinel; the port computed `body_z = (−ax, −ay, G − az)`, so `9.81 − 100`
**inverted** the attitude target, hit `limitTilt`'s degenerate branch, and commanded 12° of
pitch while sitting on the pad. Stock avoids this via `MPC_ACC_DECOUPLE`, which defaults on.
This was never only a takeoff bug — any commanded descent above 1 g does the same. (§4)

**Nothing set at runtime persisted, and for a while the wrong vehicle was flying.** PX4 is
launched with a `TemporaryDirectory` rootfs, so `param set` never survives and the ulog is
deleted with it; and `rcS` checks `PX4_SYS_AUTOSTART` *before* `PX4_SIM_MODEL`, so a
leftover environment variable silently outranked the requested airframe — all 34 retained
ulogs turned out to be **Gazebo x500 flights**, saying nothing about this vehicle. Fixed by
`param set-default` in the airframe file plus a `check_airframe()` gate that refuses to fly
rather than flying the wrong thing. (§1, §2)

**The ground-truth position was 57.2958× too small.** `gps.py`'s reprojection returns
radians; the noisy lat/lon were converted with `np.degrees()` and the ground-truth ones were
not, while both MAVLink backends do `int(x * 1e7)` expecting degE7. Because levels 2 and 3
fly on `MC_CTRL_GT=1`, **this was the position feedback, not a logging artifact**: the
position loop ran 57.3× detuned and chased every waypoint 57.3× too far away. Confirmed
exactly against the logged acceleration setpoint. Every level-2 conclusion drawn before it —
including "nothing survives 20 ms of rotor lag" and "level 2 cannot translate" — was
measuring the bug, and had to be re-flown. (`8c48c10`)

**Time-based trajectories run away under spin.** A timed CSV marches the reference on its
own clock, so when the vehicle falls behind the error compounds — a climb held to 5 mm, then
the timed translate put it on the ground by t = 20 s with 3.2 m of error. Arrival-based
advance (advance only inside `waypoint_radius`) caps the error at the radius by
construction; the same flight then held 110 s. Two related traps: pacing off wall-clock time
ran the reference ~1.5× too fast because the sim is slower than real time (use PX4's
`time_boot_ms`), and clamping *position error* rather than the *acceleration output* zeroes
`d(error)/dt` and removes the damping term entirely.

## 12–13. The estimator (level 3)

**The spin ceiling was never a control problem.** Above ~7 rad/s the vehicle diverged under
every control law and every gain set — and the mode did not move when gains changed by 4×,
when `b` flipped sign, or when the entire law was swapped. EKF2 was running its filter at
100 Hz against a 250 Hz IMU: 4.5° of vehicle rotation per filter step at 7.9 rad/s. The
attitude estimate walked 20–80° from truth while EKF2's own innovation test ratios read
0.03–0.08 against a 1.0 rejection threshold — confidently, silently wrong.
`EKF2_PREDICT_US 10000 → 4000` fixed it; across 26 logged flights above 6.5 rad/s, every
flight that set it flew clean (68–163 s) and every one of the 23 that did not diverged. It
also explained why results had been non-deterministic: two flights with *zero* parameter
differences had given opposite outcomes, which invalidated most gain conclusions drawn from
single flights. (§10)

**That moved the ceiling to ~13 rad/s rather than removing it, and the second failure was a
genuine PX4 bug — in the output predictor, not the filter.** At 20 rad/s the EKF's internal
quaternion tracked truth to 0.81° while `vehicle_attitude`, which every consumer reads, was
off by 68.7°. The output predictor measures its tracking error in the body frame at the
*delayed fusion horizon* and then applies the correction to delta-angles sampled at the
*current* time, with no rotation between them — 2.07 rad (118°) of accumulated body rotation
at 20 rad/s over the stock 100 ms buffer. Past 90° of mismatch the loop's negative feedback
becomes positive and the published attitude diverges while the EKF stays correct. A second
defect made it unrecoverable: the correction extracted the angle as `2·q.vec` (small-angle,
i.e. `2·sin(θ/2)`), which past 180° *shrinks* as the error grows. Fixing both — rotate the
error into the current body frame using the predictor's own attitude history, and use exact
angle-axis — took the `vehicle_attitude` tilt error from **68.21° to 0.76°**, and let
level 3 fly the full trajectory closed-loop on the estimator at **20.7 and 25.9 rad/s**.
The fix is identity when the vehicle is not rotating, which is why nobody has hit it.
(notes/EKF2_OUTPUT_PREDICTOR_SPIN_FIX.md)

A related attempt that did **not** work, recorded because it looked obviously right: running
the whole chain at 560 Hz (physics, `IMU_INTEG_RATE`, `EKF2_PREDICT_US` together) is
achievable and measures cleanly, but EKF2 then refuses to arm — "heading estimate not
stable" — and wanders ±12 m of altitude while stationary. Reverted to the 250 Hz chain.

---

## Method lessons worth keeping

- **To tell a plant mode from a loop mode, check whether its frequency tracks the spin
  rate** — not whether the frequency *equals* something suggestive. Reason from how a
  frequency moves.
- **Resolve innovations in time before blaming the estimator.** Summary statistics over a
  full log show enormous test ratios that are pure post-crash effect.
- **Check what actually flew**: `SYS_AUTOSTART`, `changed_parameters` (not
  `initial_parameters`), `mc_controller_status.algorithm`. Several reboot-required
  parameters were "tested" without ever taking effect.
- **"Trajectory complete" does not mean the vehicle flew.** The driver streams setpoints on
  a timer with no crash detection; confirm altitude from
  `vehicle_local_position_groundtruth`.
- **Compare against a zero-lag reference run, component-wise.** An RMS magnitude hides the
  seat's effect entirely (reads −2.6% to +8% where the true vector error reads −15% to
  −520%) — and the reference must be the fixed nominal design, not the arm's own re-tuned
  plant, or the comparison is circular.
- **One simulator at a time.** Overlapping instances contend for the GPU and for PX4's fixed
  ports, and a second run either fails to bind or silently attaches to the first's state.

## Still open

- **`rotor_tau` on real hardware is still a placeholder (20 ms).** It is the single number
  that decides whether 30 rad/s is reachable, and props-off bench measurement gives the
  wrong quantity. The plan is to identify it in flight from the converged `b*`.
- **At `rotor_tau = 20 ms` and 20 rad/s the vehicle is marginal, not reliably stable.** The
  clean 48.9 s seat-on flight is a demonstration, not a reliability claim; A/B work belongs
  in the `rotor_tau = 0` regime where flight is repeatable.
- **The output-predictor fix has not been upstreamed**, its two halves have not been ablated
  separately, and it is validated on one trajectory at n=1–2 per condition.
- **Descending while spinning** was the failure mode in the marginal-lag case, and has not
  been studied on its own.
