# EKF2's output predictor diverges under spin — root cause and fix

Working notes on a PX4 core bug found while getting level 3 (PX4 + estimator, as opposed
to level 2's PX4 + simulator ground truth) flying at 20+ rad/s. Confidence marked
throughout: **[verified]** measured in flight, **[derived]** computed from source,
**[open]** unresolved.

Repo: `/home/ani/PX4-Autopilot` (separate git repo from this workspace).
File: `src/modules/ekf2/EKF/output_predictor/output_predictor.cpp`,
`OutputPredictor::correctOutputStates()`.

---

## TL;DR

**EKF2's own filter is fine under spin. The stage that publishes `vehicle_attitude` is
not.** `estimator_states` (the filter's internal quaternion, at its delayed fusion
horizon) tracked ground truth to 0.81° at 20 rad/s. `vehicle_attitude` — what every
consumer, including the flight controller, actually reads — was off by 68.7° (max 94.6°).
**\[verified\]**

Cause: the output predictor measures its own tracking error in the body frame at the
*delayed* fusion horizon, then applies the correction to delta-angles sampled in the body
frame at the *current* time, with no rotation between the two. Under spin those frames
differ by `r * delay` — 2.07 rad (118°) at 20 rad/s and the stock 100 ms delay buffer.
Past 90° of that mismatch the correction's projection onto the true error flips sign:
negative feedback becomes positive. **\[derived, and the predicted onset matched what was
measured\]**

Fix: rotate the error into the current body frame before applying it, using the
predictor's own attitude history (no assumption about spin rate or a constant rotation —
whatever the vehicle actually did during the delay is what gets used). Also replaced a
small-angle angle extraction that saturates and loses gradient near 180° with the exact
one. **\[verified: fixed the divergence, cost nothing in flight behavior\]**

---

## 1. Symptom

Flying the eigen controller in a circle (R=10 m, ~5 m/s, ~14° sustained tilt) at spin
rates above roughly 15-19 rad/s, `vehicle_attitude` diverges from truth while the vehicle
keeps flying normally (it's on `MC_CTRL_GT=1` / ground truth at level 2, so the controller
never sees the bad attitude — this corrupts *analysis*, silently, not control). Logged
`estimator_status.output_tracking_error[0]` — the attitude tracking error the output
predictor itself computes — sits at ~0.0007 rad below 12 rad/s, then jumps to ~1.9-2.0 rad
by ~20 rad/s and stays pinned there for the rest of the flight. 1.9-2.0 is suspicious: the
signal is bounded at 2.0 by construction (see §3).

## 2. Isolating the fault: estimator vs output predictor

Two attitude sources exist in every log, and they are not the same thing:

- `estimator_states[0..3]` — the core EKF's quaternion, valid at the delayed fusion
  horizon (`time_delayed_us`), used internally for the next prediction/fusion step.
- `vehicle_attitude` (`quat_nominal` in `_output_new`) — the output predictor's real-time
  quaternion, built by integrating raw gyro samples forward from the delayed solution,
  corrected periodically to track the EKF.

Comparing both against `vehicle_attitude_groundtruth` on the same flight, same instants
**\[verified\]**:

| | mean tilt error vs GT | max |
|---|---|---|
| `estimator_states` (EKF internal) | 0.81° | ~1.4° |
| `vehicle_attitude` (output predictor) | 68.73° | 94.59° |

Same filter, same flight. The defect is entirely in the stage between them.

## 3. Root cause: a frame mismatch

`correctOutputStates()`:

```cpp
const outputSample &output_delayed = _output_buffer.get_oldest();   // predictor's own
                                                                      // attitude at t-tau
const Quatf q_error((quat_state.inversed() * output_delayed.quat_nominal).normalized());
const float scalar = (q_error(0) >= 0.0f) ? -2.f : 2.f;
const Vector3f delta_ang_error{scalar * q_error(1), scalar * q_error(2), scalar * q_error(3)};
...
_delta_angle_corr = delta_ang_error * att_gain;
```

`q_error` is the small rotation between the EKF's attitude and the predictor's own
buffered attitude, **both at time `t - tau`** (tau = the fusion delay: `EKF2_DELAY_MAX`,
stock 100 ms). Its vector part is naturally expressed in the body frame **at `t - tau`**.

That correction is consumed one tick later, here, in `calculateOutputStates()`:

```cpp
const Vector3f delta_angle_corrected(delta_angle - delta_angle_bias_scaled + _delta_angle_corr);
const Quatf dq(AxisAnglef{delta_angle_corrected});
_output_new.quat_nominal = _output_new.quat_nominal * dq;
```

`dq` is composed on the right of the current attitude — a standard body-frame integration
step, so `delta_angle_corrected` is implicitly a rotation vector **in the current body
frame, at `t_now`**. A vector computed in the `t - tau` frame is being added as though it
were already in the `t_now` frame, with nothing rotating it in between.

Under spin, these frames are not the same. Restricting to the roll/pitch plane
(perpendicular to the spin axis, where a tilt error lives), the body frame turns by

```
phi = r * tau
```

between the two instants. Let `e` be the true error, expressed in `t - tau` coordinates.
In `t_now` coordinates the same physical vector is `Rot(phi) * e`. The code applies `e`
directly. The correction still restores the error whenever

```
e . Rot(phi)*e  =  |e|^2 * cos(phi)   >   0
```

— which flips sign at `phi = pi/2`. Past that, the "correction" has negative projection
onto the true error and actively drives it further out. This gives a critical spin rate
for a fixed delay:

```
r_crit = (pi/2) / tau
```

At the stock `EKF2_DELAY_MAX = 100 ms` (25-sample IMU buffer at 4 ms): `r_crit = 15.7
rad/s`. **\[derived\]** This matches what was measured: `output_tracking_error` was
essentially zero through r = 12-18.5 rad/s and pinned by the next sample at r = 19.9
rad/s. **\[verified\]** At the flight spin rate of 20.66 rad/s, `phi = 2.07 rad = 118°`,
`cos(phi) = -0.47` — solidly inverted, consistent with a loop that never recovers on its
own once it crosses the threshold.

This is invisible on a non-spinning airframe: `phi -> 0`, `cos(phi) -> 1`, full restoring
gain, no defect to find. It has presumably been sitting in EKF2 unnoticed because almost
nothing that runs this estimator spins at 15+ rad/s.

## 4. Secondary defect: the angle extraction saturates

`2 * q_error.vec = 2*sin(theta/2) * axis`. With the shortest-path sign already handled
(the `q_error(0) >= 0` check), `theta` is correctly in `[0, pi]`, but `2*sin(theta/2)`
saturates toward 2 as `theta -> pi` with vanishing slope (`d/dtheta = cos(theta/2) -> 0`):

| true angle | 2*sin(theta/2) | error |
|---|---|---|
| 90° (1.571 rad) | 1.414 | -10% |
| 150° (2.618 rad) | 1.900 | -27% (matches the logged 1.90 exactly) |

Not the trigger — the frame mismatch above is — but once that pushed `theta` near `pi`,
this term's near-zero gradient meant the correction could no longer pull it back.

## 5. The fix

Two changes in `correctOutputStates()`, applied together (not ablated separately — see
Limitations):

**(a) Exact angle extraction**, replacing the small-angle formula:

```cpp
Quatf q_err_shortest(q_error);
if (q_err_shortest(0) < 0.f) { q_err_shortest = -q_err_shortest; }
const Vector3f q_err_vec{q_err_shortest(1), q_err_shortest(2), q_err_shortest(3)};
const float q_err_vec_norm = q_err_vec.norm();
Vector3f delta_ang_error{};
if (q_err_vec_norm > 1e-9f) {
    const float angle = 2.f * atan2f(q_err_vec_norm, q_err_shortest(0));
    delta_ang_error = q_err_vec * (-angle / q_err_vec_norm);
}
```

`theta = 2*atan2(|vec|, w)` recovers the true angle exactly; gradient stays 1 everywhere.

**(b) Rotate the error into the current body frame** before it's used:

```cpp
const Quatf q_delayed_to_now((_output_new.quat_nominal.inversed() *
                              output_delayed.quat_nominal).normalized());
delta_ang_error = Dcmf(q_delayed_to_now) * delta_ang_error;
```

`q_o(t_now)^-1 * q_o(t-tau)` is exactly `Rot(phi)` from §3 — the rotation carrying
`t - tau`-frame coordinates to `t_now`-frame coordinates for the same physical vector.
Both quaternions come from the predictor's own continuous gyro integration, so this uses
whatever the vehicle actually did during the delay (precession, noise, non-constant spin
included) rather than an idealized `r * tau` model — that formula was only ever used
above to *diagnose* the failure and predict `r_crit`; the code itself doesn't compute or
assume it. When the vehicle isn't rotating, `q_o(t_now) ~= q_o(t-tau)` and this reduces to
identity, which is why the defect never surfaced before.

Every step in (b) is exact linear/quaternion algebra — quaternion composition, the
closed-form quaternion-to-DCM identity, a matrix-vector product — none of it is a
small-angle approximation. The only place approximation error enters at all is upstream:
`q_o_delayed` and `q_o_now` are themselves noisy (integrated from a real, noisy gyro), not
that computing the rotation between them is approximate.

## 6. Validation

Controlled A/B, same trajectory (fast circle, R=10 m, 5 m/s), same spin rate, only the
binary changed, in the regime known to fly reliably (`rotor_tau=0`) so the comparison
isn't confounded by a marginal flight condition: **\[verified\]**

| | pre-fix | post-fix |
|---|---|---|
| attitude tracking error | 1.9057 rad | 0.0010 rad |
| `vehicle_attitude` tilt error vs GT | 68.21° | 0.76° |
| orbit radius (cmd 10 m) | 11.34 ± 0.01 m | 11.33 ± 0.00 m |
| altitude sd | 0.0213 m | 0.0011 m |
| motor saturation | 0.6% | 0.0% |
| \|p,q\| | 0.18 rad/s | 0.18 rad/s |

Flight is unchanged (expected: the controller was on `MC_CTRL_GT=1`, so it never saw the
bad attitude) — only the estimator's own accuracy improved, by ~1900x on tracking error.

Confirmed at a second, independent spin rate with the controller **actually closing the
loop on the fixed estimator** (`MC_CTRL_GT=0`, level 3) rather than ground truth:

| | 20.67 rad/s | 25.86 rad/s |
|---|---|---|
| airborne | 50.2 s | 50.4 s |
| orbit radius (cmd 10 m) | 11.136 ± 0.044 | 11.326 ± 0.038 |
| altitude sd | 0.0060 m | 0.0088 m |
| tracking error | 0.0010 rad | 0.0011 rad |
| `vehicle_attitude` tilt error vs GT | 0.69° | 0.89° |
| estimator position error vs GT | 0.068 ± 0.011 m | — |

Both flew the full 3.3-orbit trajectory cleanly, no saturation, and were confirmed
visually (headed run) as well as by telemetry — including a controlled, non-violent
descent afterward (steady 0.5-0.7 m/s, motors mid-range, position essentially fixed),
not the fall that an earlier, unrelated marginal-lag test case showed.

## 7. A noise caveat on the "exact" rotation

The quaternion algebra in (b) is exact given its inputs, but the inputs are
gyro-integrated and therefore noisy. Using this project's `EKF2_GYR_NOISE = 0.015 rad/s`
as a noise density, the angle random walk over the `tau = 100 ms` window is
`~0.015 * sqrt(0.1) ~= 0.0047 rad ~= 0.27°` **\[derived, not directly measured\]** — a
noise floor on the rotation itself, well below the measured 0.69-0.89° post-fix residual
(most of which is already accounted for by the EKF's own internal 0.81° error, not this
stage). Three orders of magnitude below the ~100° error being removed either way.

## 8. Limitations / open items

- (a) and (b) above were applied and validated together. Their individual contributions
  have not been isolated — an ablation (frame-fix only, angle-fix only) hasn't been run.
- Validated at `rotor_tau = 0` (pipeline lag only) and two spin rates, one trajectory
  (the fast circle). Not yet validated with additional injected rotor lag, other
  trajectories, or repeated trials for statistics — each condition above is n=1 or n=2.
- Not upstreamed. This is a genuine PX4 bug (reduces to a no-op below `r_crit` for any
  vehicle, which is why it's gone unnoticed) and is worth submitting independently of this
  project.

See also `notes/FINDINGS.md` (the level-2 port history) and this session's memory files
`level2-bad-state-sources.md`, `ekf2-output-predictor-spin-fix.md`,
`seat-lag-compensation.md` for the surrounding context (the 57.3x ground-truth position
bug, the seat/pole adaptation work, and the level-1/2/3 test ladder).
