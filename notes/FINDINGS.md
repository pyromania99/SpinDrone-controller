# Porting the Isaac Sim controllers onto PX4 — what actually went wrong

Working notes from debugging why `CascadedPdController` flew badly under PX4 + Pegasus
while its Isaac Sim prototype flew fine. Most of these cost hours to find and none are
obvious from the code.

Confidence is marked throughout: **[verified]** means measured in flight or in a test,
**[derived]** means computed from source, **[open]** means unresolved.

---

## TL;DR

The prototype and the port implement the same law, but three things differed, in
descending order of impact:

1. **PX4's model of the vehicle was wrong** — `CA_ROTOR*_CT` said each rotor produced
   1.6-2.3× less thrust than it did, *and the error moved with throttle*. Every torque
   command landed harder than intended by a factor that changed as the vehicle climbed.
2. **Vertical acceleration was coupled into the tilt setpoint**, which inverted the
   attitude target on PX4's pre-takeoff sentinel and broke takeoff outright.
3. **The gains cannot be inherited from the Isaac Sim sweeps at all** — that loop has
   zero transport lag; this one has 16 ms, and derivative gain is what delay punishes.

1 and 2 are fixed and verified. 3 is structural: the sweep archives are not a source of
PX4 gains, and treating them as one produces a limit cycle.

> **Read §1 first (updated 2026-08-07).** Every ulog retained on disk is a **Gazebo x500**
> (`SYS_AUTOSTART = 4001`), not the Pegasus Iris, so none of them carries the tuning
> described below. The three findings above were derived from those logs and need
> re-confirming on a verified-`10016` flight. The loop rates are the exception — they are
> the same in every log and were re-measured directly: inner **250 Hz**, position stage
> **125 Hz**, `OuterLoop` dormant for `MC_CTRL_ALG=3`. Sample rate is not a constraint at
> 8–10 rad/s; transport delay is, by roughly 8×.

> ## ⚠ SUPERSEDED 2026-08-08 — the spin failure was the ESTIMATOR, not the gains
>
> Everything below about gains not transferring, limit cycles, and cross-coupling
> authority was written while chasing a control problem. It was not one. **`EKF2` was
> running its filter at 100 Hz against a 250 Hz IMU**; at 7.9 rad/s that is 4.5° of vehicle
> rotation per EKF step, and the attitude estimate walked away from ground truth by
> 20–80° while EKF2's own innovation test ratios sat at 0.05 against a 1.0 rejection
> threshold — confidently, silently wrong. See **§10**.
>
> `EKF2_PREDICT_US 10000 → 4000` fixes it. Read §10 before trusting §5, §8 or §9.

---

## 1. Nothing you set at runtime persists  **[verified]**

`PX4LaunchTool` hands PX4 a fresh `tempfile.TemporaryDirectory()` as its rootfs on every
launch and calls `root_fs.cleanup()` on teardown
([px4_launch_tool.py:54,113](../extensions/pegasus.simulator/pegasus/simulator/logic/backends/tools/px4_launch_tool.py#L54)).
So `param set` never survives, `param save` is pointless — **and the ulog is deleted with
the directory.**

This silently reverted `MC_CTRL_ALG` to 0 on every run, meaning the stock cascade flew
while it looked like the custom controller was under test.

**The mechanism that does work:** `param set-default` in the airframe file. PX4 symlinks
`tempdir/etc → ROMFS/px4fmu_common/` at startup
(`main.cpp:398-420`), and `rcS` sources the airframe at line 224, well before
`rc.vehicle_setup` → `rc.mc_apps` at line 289. **No rebuild needed** — the file is read
live from the checkout.

> **Correction (2026-08-07): "no rebuild needed" holds only on the autolaunch path.**
> It is true because `PX4LaunchTool.launch_px4()` passes the *source*
> `ROMFS/px4fmu_common/` as `argv[1]`, so the `etc` symlink lands in the checkout. A PX4
> launched **by hand out of `build/px4_sitl_default/rootfs`** has
> `etc → build/px4_sitl_default/etc`, a build artifact that only refreshes on `make`. That
> copy of `10016_none_iris` is currently two days stale and contains **zero** `MC_EIG_*`
> params. So the advice below to hand-launch for a persistent log silently costs you the
> airframe. Use `sim_px4.py --keep-log` instead: it keeps autolaunch (and therefore
> the source ROMFS) and keeps the ulog.

**And the log you keep may be of the wrong vehicle entirely  [verified]**

Every ulog retained under `PX4-Autopilot/build/px4_sitl_default/rootfs/log/` — all 34 —
reports **`SYS_AUTOSTART = 4001`**, i.e. `4001_gz_x500`. `rootfs/boot.log` confirms it:
`found model autostart file as SYS_AUTOSTART=4001` / `gz_bridge world: default, model:
x500_0`. None of them loaded `10016_none_iris`; all show `CA_ROTOR0_CT = 6.5`,
`IMU_GYRO_CUTOFF = 40`, and the `MC_EIG_*` values are `eigen_params.c`'s `PARAM_DEFINE`
defaults verbatim. **They are Gazebo x500 flights and say nothing about the Pegasus Iris.**

Two independent causes, both silent — PX4 boots, arms and flies either way:

1. **`rcS:41` checks `PX4_SYS_AUTOSTART` *before* `PX4_SIM_MODEL`.** An exported value (a
   normal leftover from any `gz_x500` workflow) outranks `--px4-airframe`, and
   `PX4LaunchTool` inherited `os.environ` wholesale. Now popped in `PX4LaunchTool.__init__`,
   which also copies the environment rather than aliasing it — the old code mutated the
   Isaac process's own `PX4_SIM_MODEL`, so a second vehicle overwrote the first's.
2. **A hand-launched PX4 reads the stale build copy of the airframe** (see the correction
   above).

`sim_px4.py` was never at fault: its `--px4-airframe none_iris` →
`px4_vehicle_model` → `PX4LaunchTool` → `PX4_SIM_MODEL` chain is sound. It simply was not
what produced these logs — `PX4LaunchTool` runs PX4 with `cwd=TemporaryDirectory()`, and
the logger writes to `./log` relative to cwd, so an autolaunched PX4 *cannot* write into
the build rootfs.

**Guard:** `px4_driver.py` now calls `check_airframe()` from `arm()` and refuses to fly
unless `SYS_AUTOSTART == 10016`, `CA_ROTOR0_CT == 10.344`, `IMU_GYRO_CUTOFF == 100` and
`MC_EIG_ATT_D == 0.1`. `flight_health.py` reports the same check on any ulog. **Check
`SYS_AUTOSTART` before drawing a conclusion from a log** — nothing else in this document
is meaningful without it, and §5–§9 below should be re-confirmed on a verified-10016
flight before being relied on.

Related gotchas:
- **SITL has no `reboot`.** Only `shutdown` is registered, and it calls `system_exit(0)`
  (`platforms/common/shutdown.cpp:206`). It kills the process; Pegasus must relaunch.
- **`MC_CTRL_ALG` is read at boot** by `rc.mc_apps:20` (`param greater -s MC_CTRL_ALG 0`)
  to decide which modules start. Changing it live does nothing if `mc_controller` was
  never started.
- **Ordinary gains DO apply live.** `MulticopterController::handleParameterUpdate`
  defers only `MC_CTRL_ALG` to disarm; `updateParams()` re-reads `MC_PD_*` / `MC_EIG_*`
  on every `parameter_update`. This is what makes a one-flight sweep possible.
- **To reach a shell without a console:** PX4 is launched with `-d` (pxh off), but the
  daemon socket at `/tmp/px4-sock-<instance>` is always open.
  `build/px4_sitl_default/bin/px4-param --instance 0 set FOO 1` works from anywhere.
- **To keep a log**, use `sim_px4.py --keep-log [DIR]`. This gives `PX4LaunchTool` a
  persistent rootfs instead of a `TemporaryDirectory`, so the ulog survives while PX4 is
  still launched with the source ROMFS. Hand-launching with `--no-px4-autolaunch` also
  keeps the log but loads the stale build copy of the airframe — see the correction above.

## 2. Use `none_iris`, not `gazebo-classic_iris`  **[verified]**

Pegasus-specific tuning belongs in
`ROMFS/px4fmu_common/init.d-posix/airframes/10016_none_iris` — the "external simulator"
airframe, which is exactly what Pegasus is. Both it and `10015_gazebo-classic_iris` fall
through to the same `px4-rc.mavlinksim` branch (only sihsim / gz / jmavsim differ), so
they are interchangeable for this purpose — but putting Pegasus values in the Gazebo
airframe corrupts real Gazebo runs, whose Iris has a different rotor model
(`motorConstant 5.84e-06` vs Pegasus's `8.54858e-06`).

Also note the SDF is **not the model in use**: Pegasus loads `ROBOTS["Iris"] = .../iris.usd`
([params.py:31](../extensions/pegasus.simulator/pegasus/simulator/params.py#L31)).
`PX4_SIM_MODEL` only selects which *param file* loads.

## 3. The thrust-model mismatch — the big one  **[verified]**

Your Python controller doesn't model the vehicle, it **reads** it:
`force_and_torques_to_velocities` builds the allocation matrix at runtime from the actual
USD rotor positions and thrust curve
([multirotor.py:212-231](../extensions/pegasus.simulator/pegasus/simulator/logic/vehicles/multirotor.py#L212-L231)).
It cannot be wrong about the airframe.

PX4 can only use `CA_ROTOR*`, which said `thrust = 6.5·u²` while Pegasus produced
`8.54858e-6·(1000u + 100)²`:

| throttle | PX4 believed | Pegasus delivered | ratio |
|---|---|---|---|
| u = 0.30 | 0.59 N | 1.37 N | **2.34×** |
| u = 0.58 (hover) | 2.19 N | 3.95 N | **1.81×** |
| u = 1.00 | 6.50 N | 10.34 N | **1.59×** |

The error is *throttle-dependent* because the `+100` offset means Pegasus's curve misses
the origin while `CT·u²` passes through it. A constant error you can absorb into gains
without noticing; this one moves as the vehicle climbs.

**Why altitude looked fine anyway:** the collective self-calibrates through
`mc_hover_thrust_estimator`. There is no equivalent for torque, so the attitude loop
absorbed the entire error uncorrected.

**Fix (two halves — the airframe alone cannot do it):**
1. `sim_px4.py`: `input_scaling = 1100`, `zero_position_armed = 0` → `omega = 1100u`,
   so the curve passes through the origin and is genuinely of the form `k·u²`
2. `10016_none_iris`: `CA_ROTOR*_CT = 10.344` = `8.54858e-6 × 1100²`

Verified: the two models now agree to **ratio 1.0000 at every throttle**.

## 4. Vertical acceleration must not drive tilt  **[verified]**

`CommandFrontEnd` commands `acceleration[2] = 100.f` as a pre-takeoff "make no thrust"
sentinel (`CommandFrontEnd.cpp:224-233`). The port computed
`body_z = (-ax, -ay, G - az)`, so `9.81 − 100 = −90.19` — the attitude target **inverted**.

Exactly anti-parallel to world down, that hits `limitTilt`'s degenerate branch, where the
rejection vector is zero and gets forced to `(1,0,0)` — yielding a **12° forward pitch
commanded while sitting on the pad**. Thrust is correctly zero there, so nothing moves
until the ramp starts, at which point the vehicle accelerates sideways instead of climbing.

Stock avoids this with `MPC_ACC_DECOUPLE`, which **defaults on**
(`multicopter_position_control_limits_params.c:152`); the port hardcoded the non-default
coupled branch. Fixed at
`CascadedPdController.cpp:250-269` — `body_z` now uses gravity alone. 20/20 unit tests pass.

**This was never only a takeoff bug.** Any commanded downward acceleration above 1 g
inverts the target the same way. With `MC_PD_Z_D = 8`, a 1.2 m/s climb-rate error does
it — inside `MPC_Z_VEL_MAX_DN`. Firm descents were hitting this too.

## 5. Sweep gains do not transfer — this is structural  **[verified]**

> **Heavily qualified 2026-08-08.** The 12.7 Hz limit cycle described here is real and was
> measured, but the flight it came from was a **Gazebo x500** (§1) and the conclusion drawn
> from it — that lag makes the Isaac Sim fronts untransferable — is now doubtful. The same
> `Kd_att = 1.0` that "self-excites here" is in the Pegasus-only gain set that flies this
> vehicle cleanly at 9.8 rad/s. And with `EKF2_PREDICT_US` corrected (§10), `MC_CTRL_ALG=3`
> flies 163 s at 7.86 rad/s. Most of what was attributed to transport lag was an estimator
> running at 100 Hz. **Re-derive before treating the archives as unusable.**

The mm=2e-7 Pareto front's best point (`Kp_att 5.283, Kd_att 3.000`) converts to
`MC_PD_ATT_P 1.192 / MC_PD_ATT_D 0.677` and **fails**: a 12.7 Hz limit cycle from the
instant of takeoff, attitude error 0.03 rad while torque sits at 0.79 of full authority —
essentially all damping term. It departs at t≈27 s once the spin adds to it.

```
  t     cone     r    attErr  |tau|   gps_hpos  grav_z
 19.0   1.229   7.00   0.031  0.788     0.00     0.00
 24.0   1.256   9.53   0.059  0.804     0.00     0.00
 27.0   2.135   9.80   0.677  0.961     0.00     0.09   <- departure
 30.0  21.840   1.00   0.998  1.000     0.02    40.50   <- innovations blow up AFTER
```

**The EKF is not implicated** — innovations are exactly zero through the whole
pre-departure window. (Summary statistics over the full log show `gps_hpos` test ratios
of 5×10¹¹, which is pure post-crash effect. Always resolve innovations in time before
blaming the estimator.)

**Why:** the Python controller reads exact state in-process with no transport lag. Its
Pareto front is optimal for a lag-free plant. `Kd = 3.0` N·m/(rad/s) is fine there and
guarantees a limit cycle through 16 ms of delay. Derivative gain is precisely what delay
destabilises, and the sweep raised it:

| | `Kd` normalized | saturates above |
|---|---|---|
| known-flying | 0.400 | 2.50 rad/s |
| sweep-derived | 0.677 | 1.48 rad/s |

**Consequence for the research plan:** the Isaac Sim archives are not a source of PX4
gains. Either tune on the PX4 stack directly, or add a transport-delay model to the
Pegasus sweeps so the fronts mean something.

## 6. Loop latency: 12 ms, and mostly not what you'd guess  **[verified]**

Measured by cross-correlating `vehicle_torque_setpoint` against angular acceleration —
[loop_latency.py](../examples/tools/px4_tuning/loop_latency.py).

**Updated 2026-08-07: 12.0 ms**, both axes agreeing at r = 0.90/0.89, on the first
verified-`10016` flight (`px4_runs/20260807_211118`). The earlier 16 ms was measured at
`IMU_GYRO_CUTOFF = 40`; the airframe now sets 100, and the ~4 ms drop is that change,
matching the budget below. Note `K_eff` is not identifiable from a quiet hover — there is
no excitation to regress against, and the tool correctly reports `--` on roll. Re-measure
it from a commanded step, not a hover.

Against `MC_EIG_WN = 24`, `wn*tau` = 0.29 rad = **16.5 deg** of phase (was 22 deg at
16 ms), versus `wn*T/2` = **2.8 deg** for 250 Hz discretization. **Delay still dominates
sample rate by ~6x**, so latency is where the margin is won.

The original measurement, for reference: both axes agreed at 16 ms, r = 0.95.

**A measurement trap worth knowing:** cross-correlation of a narrowband signal is
ambiguous modulo its period. The 12.7 Hz limit cycle produced near-equal peaks at 16, 96
and 176 ms (0.951 / 0.958 / 0.954) and `argmax` picked 96. The physical delay is the
*earliest* strong peak. The tool now handles this; two earlier estimates of mine (39 ms
from `period/2`, then 96 ms from `argmax`) were both wrong.

Budget **[derived]**:

| source | ~ms | sim artifact? |
|---|---|---|
| HIL transport — Pegasus applies actuators before `backend.update()`, ~2 physics steps | ~8 | **yes** |
| `IMU_GYRO_CUTOFF` 40 Hz Butterworth @ 12.7 Hz | ~6 | no — hardware needs it *more* |
| uORB hops + scheduling | ~1-2 | no |
| motor/ESC spin-up | **0** | **hardware-only** — the sim has none |

All rate-loop lag is **upstream of `mc_controller`**: `VehicleStateProvider.cpp:277-289`
copies `angular_velocity` raw, and the "velocity notch/LPF chain" is linear velocity only.
**EKF2 is not in the rate path**, so no `EKF2_*_DELAY` parameter is relevant to it.

**Important for sim→hardware:** roughly half the lag is a simulation artifact, but
hardware *adds* motor spin-up (10-30 ms) that the sim omitted entirely — it set rotor
velocity instantly. So hardware is probably **not** faster, and reducing sim lag may make
gains that don't transfer.

> **Since addressed (6a39ec5).** The shared thrust curve now models rotor/ESC spin-up as a
> first-order lag
> ([quadratic_thrust_curve.py:93,138-145](../extensions/pegasus.simulator/pegasus/simulator/logic/thrusters/quadratic_thrust_curve.py#L138-L145)).
> It stays off by default (`rotor_tau` 0.0) so archived sweeps are unaffected; the entry
> points opt in via `ROTOR_TAU_PLACEHOLDER` (20 ms). Gains tuned before that commit were
> tuned against the instant-response plant described above.

## 6b. Cascaded PD cannot hold 30 rad/s; the mode is gyroscopic coning  **[measured]**

Level 1, `km = 6e-7` (~29.5 rad/s steady), `--rotor-tau 0`, vertical-only
profile, 30 s. Same plant for both laws; only the controller differs.

`cascaded_pd` diverges. Attitude error grows exponentially at **+0.224 1/s**
(doubling every 3.1 s), and the roll/pitch rate envelope grows monotonically
per 5 s window: 0.038 → 0.041 → 0.049 → 0.072 → 0.157 → 0.282 rad/s. Altitude
holds 1.48 m the whole time; it is *station* that is lost, reaching 7.66 m of
lateral error by t = 30 s and still accelerating.

The mode is identified, not guessed. Its dominant frequency is **30.58 rad/s**
against a body spin of **29.69 rad/s** — a ratio of **1.030**, i.e. it rotates
with the body. That is the coning mode `sweep_task.py` predicts: the
`(Iyy - Izz)·q·r` Euler cross terms couple roll into pitch with a gain
proportional to spin. On the measured composite inertia,
`(Iyy - Izz)/Ixx = -0.901`, so at 29.7 rad/s the plant carries **26.8 1/s** of
roll↔pitch coupling. Cascaded PD damps each axis independently and has no term
of that form, so this is structural, not a tuning failure — no `Kd_att`
recovers it, which is consistent with the archive, where the whole stable
family at this spin sits at the *low* end of `Kp_att`.

`eigen` holds it, on the identical plant (same spin to within 0.15 rad/s).
`M_eff = M - A_r` cancels exactly this coupling (see §the A_r sign fix,
`593decc`). Measured over 30 s: `|eR|` stays bounded at 0.0069 mean / 0.0098
max, with a fitted growth rate of **+0.0001 1/s** — flat for practical
purposes, and 2000x below PD's +0.224. The roll/pitch envelope is flat at
0.025-0.031 rad/s across every window, altitude sits at 1.495 m against a
1.50 m target, and lateral error peaks at 7.1 mm and ends at 0.44 mm.

Stated precisely because it is easy to overclaim: the mode is not *decaying*,
it is not growing at a rate that matters. At +0.0001 1/s the doubling time is
about two hours, against a 30 s observation, so this says the coning mode is
cancelled — not that the flight is proven stable indefinitely.

Practical consequence: a PD-vs-eigen comparison at high spin is not measuring
tuning quality, it is measuring whether the law cancels the gyroscopic term.
Note also that a short run hides this — the archived `controller_eval`
cascaded_pd log is 6.96 s, and at a 3.1 s doubling time the mode is still
small there.

## 6c. The 20 ms rotor_tau placeholder decides whether eigen holds 30 rad/s  **[measured]**

Level 1, eigen, `km = 6e-7` (~29.5 rad/s), vertical-only, same command both
times. The only variable is `rotor_tau`.

    rotor_tau = 20 ms (ROTOR_TAU_PLACEHOLDER, the default)
        flies cleanly to t = 8 s, |eR| ~ 0.005, altitude 1.494 m
        departs as the spin finishes ramping: |eR| 0.0052 -> 1.497 by t = 12 s,
        altitude 1.494 -> 0.453 m. Run over at 12.86 s.

    rotor_tau = 0 (instant response)
        90.00 s, altitude 1.495 m, |eR| 0.0056 at the end. No departure.

That is a 300x difference in attitude error from one unmeasured number.
`ROTOR_TAU_PLACEHOLDER` documents itself as "the midpoint of the 10-30 ms range
FINDINGS section 6 cites for hardware, NOT a measured value for this vehicle",
and the mismatch ledger in NEXT_STEPS.md lists rotor/ESC lag as "modelled,
value guessed" - one of only two entries still open.

It is not merely open, it is decisive at this spin. Whether the eigen
controller can hold 30 rad/s is currently a property of a placeholder, so
`measure_motor_lag.py` against a hardware ulog is the measurement that settles
it. Note also the failure is spin-dependent and late: it survives the first
8 s at a lower instantaneous spin and only departs as r approaches its steady
value, so a short run at this km will not show it.

## 6d. b cannot buy back rotor lag; the usable b range collapses with tau  **[measured]**

`b_vs_tau_sweep.py`, km = 6e-7 (~29.5 rad/s), 30 s per run, 8 values of
`b_mdyn` (0-28) at each of 7 lag values. 56 flights.

    tau [ms]     stable b values        largest stable b    best-scoring b
       0         all of 0..28                  28                 16
       5         0..24 (28 marginal)           28                 16
      10         0..12                         12                  4
      15         0..4                           4                  4
      20         none                           -                  -
      25         none                           -                  -
      30         none                           -                  -

Two things follow, and the second is the one that matters.

**The optimum in b is shallow.** Across every stable cell at tau = 0 the score
spans 0.01205 to 0.01244 - 3.2% - with a weak minimum near b = 16. At that
depth "ideal b" is barely distinguishable from run-to-run variation, so a
best-score b read off one sweep should not be treated as a tuned value.

**The stability ceiling is not shallow.** The largest b that stays airborne
falls 28 -> 12 -> 4 -> nothing as tau goes 5 -> 10 -> 15 -> 20 ms, and beyond
about 15 ms no b in the range holds the spin at all. Whatever b is doing, it
is not compensating for actuator lag: b and tau do not trade off, the lag
simply closes the window b lives in.

The practical consequence for tuning: if the real rotor_tau is near the 20 ms
placeholder, the eigen controller does not need a different b at 30 rad/s, it
needs something other than b. And a b chosen at tau = 0 (say 16) is outside
the stable set by tau = 10 ms, so a gain set tuned on the instant-response
plant is not merely detuned on a lagged one - it is unflyable.

## 7. Yaw spin is by design; its rate is `motor_moment`  **[verified]**

`rot_dir` defaults to `[-1,-1,-1,-1]` — all four rotors turn the same way, so reaction
torques add instead of cancelling. This is intentional for the fault-tolerance research:
the controllers surrender yaw to keep position authority.

Steady-state rate is `r_ss ≈ 4.9e7 · km`
([sweep_metrics.py:51-58](../examples/utils/sweep_metrics.py#L51-L58)), confirmed in flight:
`km = 2.0e-7` → predicted 9.8, measured 9.8; `km = 1.0e-7` → measured 4.9.

Because all rotors share a direction, `CA_ROTOR*_KM` should share a sign too — that makes
PX4's model match the vehicle. It leaves the yaw row collinear with thrust, which is
harmless only because these laws command no yaw torque.

## 8. Eigen controller ships constructor defaults, not GA results  **[verified]**

> **The `MC_EIG_B` reasoning below is wrong (2026-08-08).** With the gyroscopic term
> cancelled, the rate error obeys `Om_err(t) = Om_err(0)·exp(-wn·t)·exp(+i·b·t)`: `b` is the
> imaginary part of a complex FIRST-ORDER eigenvalue, so it sets the direction the error
> spirals, not the decay rate — which is `wn` whatever `b` is. **`b` cannot stabilise the
> coning mode or anything else**, and it costs margin, since the loop delay acts at
> `|wn - i·b| = sqrt(wn² + b²)` which `b` only raises. The explicit `omega x (I·omega)` term
> does the cancellation, not `b`. Confirmed in flight: `b` = +19.3 and −5.0 both diverged,
> while Pegasus-only flies the same vehicle at 9.8 rad/s with `b` = +10.2. `@min` is now
> −10.0 so the sign is at least testable.
>
> The "rank3 crashed, frequency tracked spin at corr +0.72" evidence is also from an x500.

`eigen_params.c` values are the literal defaults in `eigen_controller.py`'s constructor
signature ([eigen_controller.py:70-94](../examples/utils/eigen_controller.py#L70-L94)) — the
un-tuned starting point. The GA evaluated that exact point:

```
rank6  wn=18.000  b=10.200  Kp_att=7.000   score=0.2130  effort=48.8903   off-front
rank3  wn= 6.807  b= 0.309  Kp_att=4.680   score=0.1071  effort= 0.1481   FRONT, best score
```

**330× the effort.** `Kp_att = 4.68, Kd_att = 1.0` recur on essentially every front entry
at both swept motor moments.

Beyond gains, two physical-property errors — and this law is a feedback linearization, so
these are measurements, not knobs:
- **`MC_EIG_TRQ_MAX = 1.0`** — same class of bug as `CT`. Measured 2.20 N·m.
- **Inertias 3.2-3.5× too small** — `0.01/0.01/0.02` vs composite `0.03477/0.03185/0.06359`.
  Note the rotor yaw exception: roll/pitch take the full parallel-axis term (rigid joint),
  yaw takes only `m(x²+y²)` because the joint is free about the rotor's own z.
- **Z gains normalized by 1.5 kg**, but the prototype's own default is 1.2 and the real
  vehicle is **1.6186**.

Corrected set applied to `10016_none_iris`, at **rank 2** (`wn 16.336, b 13.149`).

`wn = 6.807` (rank 3) was tried first and crashed. `MC_EIG_B` is what stabilises the
coning mode; at 0.309 against a 6.5 rad/s mode it is 5% of what it must cancel, which
the parameter's own doc describes as recovering "a conventional rate law" — i.e. the
eigen controller with its eigen-ness switched off, on a vehicle chosen for this law
*because* it spins.

**Outcome [verified].** Flies stably: 54 s of cruise at coning 0.101 rad/s with no trend,
x/y hold 0.017/0.013 m rms — better than the cascaded-PD reference — torque peak 0.032 of
1.0, motors 0.550-0.675. It took two fixes found in sequence, each revealed by the
previous one:

| | `b` | `ATT_D` | failure |
|---|---|---|---|
| rank3 | 0.309 | 1.0 | gyroscopic coning — freq tracked spin (corr **+0.72**) |
| rank2 | 13.149 | 1.0 | derivative self-excitation at ~39 rad/s, freq **flat** (corr **−0.01**) |
| rank2 + fix | 13.149 | **0.1** | **stable** |

The second failure was only visible once the first was fixed. `MC_EIG_ATT_D` is a finite
difference of the angle error, so its gain rises with frequency: at 1.0 it produced
**8.4×/9.3×** more rate setpoint than the proportional path, reconstructing the commanded
rate to corr +0.99. At 0.1 that ratio is 0.41×/0.14×. `Kd_att = 1.0` is on every GA front
entry and is wrong here for the same reason everything else is — the GA differentiates an
exact attitude in a loop with no lag.

*Caveat:* the successful flight also halved the spin (9.8 → 4.9 rad/s), so it does not
isolate the `ATT_D` change. The P/D ratio measurement does.

**Method worth reusing:** to tell a plant mode from a control-loop mode, check whether
the oscillation frequency tracks the spin rate. Here it does — corr **+0.72**, slope
**0.74** rad/s per rad/s against the **0.83** rigid-body coupling predicts, where a
fixed loop resonance would give slope **0**. I first blamed cascade separation on the
strength of the frequency *value* alone (12.8 rad/s ≈ 2× the rate pole); that was
numerology, and this test refuted it in one plot. Reason from how a frequency *moves*,
not from what it equals.

## 9. Open  **[open]**

- ~~**`K_eff` measured 2.20 N·m vs 4.4325 derived**, a clean 0.50× on both axes~~
  **RESOLVED 2026-08-08 — it was a measurement bug, not a vehicle property.**
  `loop_latency.py` used `I_ROLL_PITCH = 0.029125`, the **/body link alone**, for *both*
  axes (the composite is 0.03485/0.03126, and the axes differ); it included saturated
  samples, where the command is not delivered; it differentiated a 5-sample moving
  average, a 20 ms window against a 12–16 ms delay; and it regressed through the origin.
  With all four fixed, a step-ID flight at `motor_moment 0` gives **roll 4.30, pitch 4.42
  N·m, flat across two decades of command amplitude** — the axes agree to 2.8% and there
  is no asymmetry to explain. `derive_px4_params.py` independently confirms the analytic
  3.843 matches the simulation's own geometry to 4 decimal places.
- **`MC_PD_XY_D = 1.6`** was raised from 1.0 to counter EKF velocity lag (PX4 damps on
  filtered estimator velocity; the prototype used an exact numerical derivative). Set
  under the old wrong torque scale and never re-checked.
- **`MC_EIG_ALPHA` / `MC_EIG_BETA`** — now derivable rather than guessed.
  `multirotor.py:145` applies `-[0.02, 0.02, 0.035]·ω`, so the values that cancel it
  exactly are `ALPHA = 0.02/Ixx = 0.574` and `BETA = 0.035/Izz = 0.559`. BETA is applied;
  ALPHA is deliberately left at 0, because cancelling real damping *removes* stabilising
  margin and over-estimating it adds energy.
- **Which half of `EKF2_PREDICT_US` does the work** (§10) — it halves the rotation per
  filter step *and* doubles the position-stage rate from 125 to 250 Hz. Not separated.
- **`EKF2_DELAY_MAX = 100`** is carried in the airframe but only took effect from boot on
  2026-08-08; 100 ms is still 79° of rotation at 7.9 rad/s and the largest delay that
  actually applies here is GPS at 10 ms. There is likely room to go lower.

---

## 10. THE SPIN FAILURE WAS THE ESTIMATOR  **[verified]**

The vehicle diverged above ~7 rad/s under *every* control law and *every* gain set tried.
It was not a control problem at all.

**What ruled the controller out.** The divergent mode sat at ~1.4·r, retrograde, and did
not move when `MC_EIG_ATT_P` went 8→2, `MC_EIG_WN` 24→20, `MC_EIG_B` +19.3→−5.0 (a sign
flip), or when the entire law was swapped for the cascaded PD with its SO(3) geometric
error. Frequency changed by 2.4% across all of it. And flying the **same** Isaac Sim
`EigenController` in Pegasus-only at the same spin gave coning of 0.006 rad/s rms,
*decaying* — 40× quieter, no mode at all. The prototype reads exact state; PX4 estimates it.

**What it actually was.** With ground truth logged, the attitude estimate walks away from
truth while the rate estimate stays near-perfect (2.8–3.8% error, flat):

| r | t+6 | t+8 | t+10 | t+12 |
|---|---|---|---|---|
| 7.9 | 0.07° | 0.12° | 0.27° | 0.63° |
| 9.8 | 0.90° | 6.70° | **48.80°** | **82.72°** |

The error **leads** the coning — at t+8 the estimate is 6.7° wrong while the vehicle is
still quiet at 0.141 rad/s — and EKF2's innovation test ratios read 0.03–0.08 against a
1.0 rejection threshold throughout. The filter has no idea. The accelerometer meanwhile
agrees with the specific force predicted from truth to ~0.3 m/s², so the **sensors are
faithful and the fusion is not**.

**The fix.** `EKF2_PREDICT_US 10000 → 4000`. A 100 Hz filter update against a 250 Hz IMU
is 4.5° of vehicle rotation per EKF step at 7.9 rad/s; at 250 Hz it is 1.8°.

Across 26 logged flights at r > 6.5, every flight that set it flew clean and every flight
that did not diverged — both control laws on both sides:

```
CLEAN     4000 set   alg=1  68 s | alg=1  78 s | alg=3 163 s | alg=1 124 s
          coning 0.010-0.021 rms, growth ~0.000/s at r = 7.85
DIVERGED  23 flights, no EKF2 change, coning 0.25-1.96 rms, growth +0.04 to +0.27/s
```

**Two traps this investigation walked into, worth not repeating:**

*The failure was non-deterministic before the fix.* Two flights with **zero** parameter
differences gave opposite outcomes (coning 0.019 flat vs 0.246 growing). Single flights
could not distinguish gain sets, which invalidated most gain conclusions drawn from them.
A long settle before takeoff (189 s) helped and 89 s did not — the settle was only ever
partially compensating for a filter running too slow. After the fix, an 11 s settle flies
flat for 124 s.

*`initial_parameters` is not what flew.* `MC_CTRL_ALG` is deferred to disarm, and params
set live from QGC appear only in `changed_parameters`. Read `mc_controller_status.algorithm`
(the instance actually built) and check `fallback_active`. `flight_health.py` now does both,
restricted to the airborne window.

---

## Measured constants

| quantity | value | source |
|---|---|---|
| total mass | 1.618615 kg | `vehicle_params.py:13-20` |
| hover thrust | 15.879 N → 3.970 N/rotor | same |
| hover throttle | ~0.58 | `MPC_THR_HOVER` |
| per-rotor ceiling | 10.344 N = `8.54858e-6 × 1100²` | thrust curve |
| composite inertia | 0.03477 / 0.03185 / 0.06359 kg·m² | computed, §8 |
| torque per unit normalized | 2.20 N·m (measured) / 4.4325 (derived) | §9 |
| loop latency | 16 ms | §6 |
| spin rate | `r_ss ≈ 4.9e7 · km` | `sweep_metrics.py:56` |

## Verified-good reference flight

`MC_PD_ATT_P 0.36 / MC_PD_ATT_D 0.40`, `km = 1.0e-7`, post-`CT`-fix: **789 s continuous**,
coning **decaying** 0.123 → 0.076 rad/s, position hold 2.1/2.2 cm rms, torque peak 0.036
of 1.0, zero saturation. Use this as the control when something regresses.

## Tooling

- [`flight_health.py`](../examples/tools/px4_tuning/flight_health.py) — the first thing to run on any ulog. Checks the
  airframe actually loaded, then reports loop rates (inner from the `update_count` slope,
  not the logged message rate, which the logger throttles 5×), the angle stage's D/P split,
  coning trend, torque authority, motor saturation and position hold. Classifies the
  dominant roll/pitch mode by whether its frequency **tracks the spin rate** — the one test
  that separates a plant mode from a loop mode — using the spectrum of the complex `p + jq`
  rather than of `hypot(p, q)`, which is rectified and reports double the true frequency.
- [`loop_latency.py`](../examples/tools/px4_tuning/loop_latency.py) — loop delay + `K_eff` from a ulog
- [`px4_driver.py`](../examples/flight/px4_driver.py) — MAVLink arm/offboard/setpoint + live param set with
  read-back. Encodes two SITL facts: headless refuses to arm without a GCS heartbeat, and
  `MAV_CMD_NAV_TAKEOFF` is ACKed but doesn't climb, so OFFBOARD must precede arming.
- [`ulog_to_npz.py`](../examples/tools/px4_tuning/ulog_to_npz.py) — ulog → the npz `sweep_metrics.compute_metrics`
  expects, so the whole existing scoring stack is reusable. **Converts motor commands to
  newtons** (`F = U_MAX_PER_MOTOR · u²`); feeding normalized values through would not
  error, it would silently report a vehicle that never saturates.
