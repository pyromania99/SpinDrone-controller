# Where this stands, and what to run next

Companion to [FINDINGS.md](FINDINGS.md), which records *why* each of these exists.
FINDINGS is the archaeology; this is the plan.

---

## The one-paragraph state

The spin ceiling was **never a control problem**. EKF2 was running its filter at 100 Hz
against a 250 Hz IMU, and above ~7 rad/s the attitude estimate walked 20–80° away from
truth while the filter's own innovation ratios read 0.05 against a 1.0 rejection
threshold — confidently, silently wrong. `EKF2_PREDICT_US 10000 → 4000` fixed it:
`MC_CTRL_ALG=3` now flies **163 s at 7.86 rad/s** and 104 s at 9.8, where it previously
departed in ~12 s. What remains at ~14 rad/s **is** a control problem — the estimate stays
0.05–0.11° accurate through the onset — so gain tuning is now a meaningful activity for
the first time.

---

## Run these, in this order

Each stage isolates one thing. **The value of a stage is its difference from the previous
one**, so don't skip forward — a number with nothing to compare it against is not a result.

### Stage 0 — REPLACED (2026-08-16): identify tau from the controller, do not guess it

**Decision: stop guessing the actuator lag, and stop blocking on a bench measurement.**
`ROTOR_TAU_PLACEHOLDER = 0.020` was never measured, and the bench route is worse than it
looks:

- **Bidirectional DShot is unavailable.** `esc_status` is the only RPM source in a PX4 log
  and only `DShot.cpp` populates it. The Tekko32 ESCs do not support it - enabling
  `DSHOT_BIDIR_EN` inverts the signal line, and a non-supporting ESC then sees no valid
  throttle frames at all (no arming beeps, no `actuator_test` response). Reflashing to AM32
  is the only route, and it buys the wrong quantity anyway:
- **Props-off `T` is not flight `T`.** The documented bench step runs with props removed for
  safety, but a prop changes both terms of `T = J/damping` - roughly 3x the rotational
  inertia plus aero damping that does not exist bare. The measured number would be
  systematically short.

**Instead: make the controller identify tau, using `MC_EIG_B`.**

On a vehicle spinning at a known rate the dominant effect of a delay is a FRAME ROTATION:
the state is measured at `t - tau`, the torque lands at `t`, and the body has turned
`r*tau` in between, so every correction is misdirected by that angle. `b` supplies exactly
that rotation - the eigen rate error obeys `omega_err_dot = -(wn - i*b) * omega_err`, so the
complex loop gain `(wn - i*b)` carries phase `-atan(b/wn)`. Cancelling `r*tau` gives

```
    b* = wn * tan(r * tau)              and, inverted, the measurement:

    tau = atan(b* / wn) / r
```

Predicted optima at `wn = 34.18`, `r = 13.2 rad/s` - well separated, so a coarse sweep
resolves tau to a few ms:

| tau | r*tau | predicted b* |
|---|---|---|
| 10 ms | 7.6 deg | 4.5 |
| 20 ms | 15.1 deg | 9.2 |
| 28 ms | 21.2 deg | 13.3 |
| 40 ms | 30.3 deg | 20.0 |
| 50 ms | 37.8 deg | 26.5 |

#### Stage 0a — VALIDATE the method in SITL, where tau is known

This is the step that makes the whole idea trustworthy, and it costs no hardware.

1. Set `rotor_tau` to a KNOWN value (say 20 ms), `--att-delay-ms 8`, so true tau = 28 ms.
2. Fly `MC_CTRL_GT=1` (ground truth - keeps the estimator out of it) at `mm 2.7e-7`,
   sweeping `MC_EIG_B` over 0 / 5 / 10 / 15 / 20 / 25 with everything else fixed.
3. Score each flight on attitude error rms from the ulog.
4. **The test:** does the minimum land near `b* = 13.3`, and does `atan(b*/wn)/r` return
   ~28 ms?

If yes, the identifier works and Stage 0b is justified. If the curve is flat in `b`, the
rotation is already being absorbed by the loop and this whole approach is dead - which is
worth knowing in one afternoon rather than after building an adaptive law.

**Second, stronger check:** `b*` should scale with `r`. Repeat at a different motor moment
and confirm `b*/tan(r*tau)` is constant. A relation that holds at one spin rate could be
coincidence; one that tracks `r` is the mechanism.

#### Stage 0b — measure tau on hardware with the same sweep

Same procedure on the vehicle, where tau is unknown. Read tau off the converged `b*` via
`tau = atan(b*/wn)/r`. This gives tau **with props on, in flight, at operating
temperature** - the quantity that actually matters, and the one no props-off bench test can
produce.

#### Stage 0c — live adaptation (only after 0a and 0b)

Once the static relation is confirmed, `b` can be adapted online: perturb it slowly, follow
the gradient of attitude-error rms downhill, and let it track tau as the battery sags and
the motors heat. That also turns `b` into a continuous lag telemetry channel.

**Do not start here.** Extremum-seeking on a live flight control gain is the highest-risk
version of this idea, and its value is entirely contingent on 0a showing a minimum exists.

#### Caveats to hold

- The derivation treats `b` as a static phase on the loop gain. That is a first-order
  argument: `b` actually makes the closed-loop eigenvalue complex, which is a continuous
  rotation rather than a fixed offset. **The relation is a hypothesis to be tested by 0a,
  not a proof.**
- `b` costs margin - the delay acts at `|wn - i*b| = sqrt(wn^2 + b^2)` - so the optimum is
  a trade, not a free correction. Expect the minimum to sit below the pure-rotation
  prediction.
- FINDINGS is still right that **`b` cannot damp**. Decay is `wn` regardless. This is a
  different job for the same term, and the two claims are consistent.
- The existing gene bound `b_mdyn (0.0, 40.0)` already spans the useful range; the sign
  convention needs settling in 0a, and `MC_EIG_B` accepts -10 to +10 in PX4, which may need
  widening if `b*` lands high.

<details>
<summary><b>Stage 0 (OLD, bench route) — SUPERSEDED 2026-08-16, kept for reference</b></summary>

> Superseded by the `MC_EIG_B` identification above. Kept because the `tau_eff` half is
> still valid and still worth taking from any real flight log - only the `T` half (which
> needed bidirectional DShot, and would have measured a props-off constant) is dead.

> **(Historical note.) Everything below ran on `T = 20 ms` and
> `tau_eff = 23 ms`, both GUESSES.** Stages 1–5 are proceeding on
> them deliberately, because the *shape* of the answer — which stage costs what,
> which mismatch dominates — does not depend on the constants being exact, and
> the whole ladder can be re-run in an evening once they are measured.
>
> What this does mean: **no absolute number below is trustworthy, and no gain set
> below should go near hardware until this stage runs.** Treat Stage 1–5 output
> as a ranking, not as values.
>
> Two specific things that will move when `T` is measured:
> - the **stability ceiling** (`T = 20 ms` admits no stable `wn` at 25 rad/s;
>   `10 ms` does), so the 25 rad/s verdict is entirely hostage to this number
> - the **gain scale**, which goes roughly as `1/tau_eff`

Everything below runs against guessed constants until this is done.

```sh
# props OFF - this is a bare motor spin-up
param set DSHOT_BIDIR_EN 1      # needs a BLHeli_32 / AM32 ESC
param set SDLOG_PROFILE 27
param set SDLOG_MODE 2          # log from boot, not just when armed
reboot
logger start -f
actuator_test set -m 1 -v 0.25 -t 3
actuator_test set -m 1 -v 0.55 -t 3      # <- the step that gets fitted
actuator_test set -m 1 -v 0.25 -t 3
logger stop

python3 measure_motor_lag.py <log>.ulg
```

Gives **`T`** (rotor/ESC time constant) and **`tau_eff`** (total loop delay). Then:

- replace `ROTOR_TAU_PLACEHOLDER = 0.020` in `quadratic_thrust_curve.py`
- use `--att-delay-ms ≈ tau_eff − 5` for the GA (see the ZOH note below)

Without bidirectional DShot you still get `tau_eff`, which is what the stability model
scales on — you just can't split transport from motor, so the sim's `rotor_tau` stays a
guess.

</details>

### Stage 1 — GA, Python, parallel

```sh
genetic_sweep.py --motor-moments 2.7e-7 \
                 --rotor-tau-ms <T> --att-delay-ms <tau_eff-5> --pos-delay-ms <same> \
                 --torque-max 3.843
```

Hundreds of evaluations in minutes, because it flies a population per generation. Take the
top ~5.

Use **2.7e-7** (r ≈ 13.2 rad/s): it is the spin the existing GA front was searched at, and
where the vehicle currently fails. You have a prior and a known failure to compare against.

### Stage 2 — ~~`controllers.py` confirmation~~ **DROPPED, 2026-08-16**

I proposed this stage; measuring it killed it. Two reasons, both found while wiring it up:

1. **`controllers.py` cannot fly a chosen gain set.** It is fully interactive (stdin
   prompts) and takes no gain arguments — it uses hardcoded registry values. Using it as
   the Stage 2 vehicle would have meant building a CLI onto a file that is uncommitted and
   under active edit.
2. **Stage 3 is cheap enough to absorb it.** A `--dry-run` puts 5 PX4 candidates at
   **~2.3 min each, ~12 min total**. Spending an hour building a Pegasus-only confirmation
   to protect a 12-minute PX4 step is backwards.

What Stage 2 was *for* — catching GA-scoring artifacts — is partly covered already: the GA
runs `--validate-every 3`, a step-response pass against an **independently drawn** target
with its own RNG stream, and individuals that fail it are constraint-dominated. That is the
generalisation check; what it does not cover is duration (the GA window is 25 s).

**So: go Stage 1 → Stage 3 directly**, and buy the duration coverage inside Stage 3 with
`--hold` raised (it defaults to 25 s, the same window). If a candidate survives 25 s in the
GA and dies at 90 s in PX4, that is a real finding and Stage 3 will show it.

### Stage 3 — PX4 with ground truth

```sh
isaac_run examples/flight/single_vehicle.py --keep-log --headless --motor-moment 2.7e-7
gain_search.py --candidates top5.json --ground-truth --motor-moment 2.7e-7
```

**Difference from Stage 2 = PX4's implementation cost.** `CommandFrontEnd` limits, allocator
desaturation, hover-thrust normalisation, normalized torque — the three mismatches
deliberately left unmodelled (see the ledger) turn into a measured number here instead of
an assumption.

### Stage 4 — PX4 without ground truth

Same candidates, `MC_CTRL_GT=0`.

**Difference from Stage 3 = the estimator's cost at that spin.** This is the number worth
having before committing to hardware.

### Stage 5 — spin ladder on the winner

Walk `--motor-moment` up with GT on *and* off. Whichever ceiling you hit first says where
the next effort goes: estimator or control.

### Stage 6 — hardware

Scale `wn` by `τ_sim/τ_hw` and **verify — do not re-tune**. At 25 rad/s a departure is
destructive, so the sim's job was to have found the cliffs already.

---

## What changed, by file

### PX4-Autopilot

| file | change |
|---|---|
| `ROMFS/.../10016_none_iris` | **`EKF2_PREDICT_US 4000`** — the root-cause fix. Plus `DELAY_MAX 100`, `GPS_DELAY 30`, `SDLOG_PROFILE 27` (logs `*_groundtruth`), verified `MPC_THR_HOVER 0.61949` / `MC_EIG_IYY` / `IZZ` / `BETA`, and GA rank-2 gains. Two now-wrong comments corrected in place. |
| `mc_controller/module.yaml` | new **`MC_CTRL_GT`** parameter |
| `MulticopterController.{cpp,hpp}` | `MC_CTRL_GT=1` swaps the state source to `vehicle_attitude_groundtruth` / `vehicle_local_position_groundtruth`. Announced loudly at startup. |

### PegasusSimulator

| file | change |
|---|---|
| `quadratic_thrust_curve.py` | **first-order rotor lag** (exact ZOH `1-exp(-dt/tau)`, not the `dt/(tau+dt)` approximation — that biases 20 ms to 21.9). `ROTOR_TAU_PLACEHOLDER = 0.020`, the single constant every entry point resolves from. Class default stays `0.0`. |
| `px4_mavlink_backend.py` | `send_ground_truth()` was **defined but never called**; now wired in, with the scaled-int casts it needed. Persistent-rootfs support so the ulog survives. |
| `px4_launch_tool.py` | optional persistent rootfs; pops `PX4_SYS_AUTOSTART` (which outranks `PX4_SIM_MODEL` at `rcS:41`); copies `os.environ` instead of aliasing it |
| `single_vehicle.py` | `--keep-log`, `--motor-moment`, `--rotor-tau`; writes `run_config.json` recording the simulator-side settings that appear nowhere in the ulog |
| `genetic_sweep.py` | `--pos/att-delay-ms`, `--pos/att-tau-ms` → `SensorLag`; `--rotor-tau-ms`; `--torque-max` |
| `eigen_controller.py` | `torque_max` — saturates per axis like PX4 |
| `flight/controllers.py` | now carries `rotor_tau` (was the only entry point without it) |

### New tools in `examples/px4_tuning/`

| tool | what it answers |
|---|---|
| `measure_motor_lag.py` | `T` and `tau_eff` from a hardware ulog. **Stage 0.** |
| `flight_health.py` | Is this flight healthy, and which mode is eating it? Run it on everything first. |
| `mode_compare.py` | Same mode measurement across a PX4 `.ulg` and a Pegasus `.npz`, handling the FLU↔FRD flip |
| `derive_px4_params.py` | Reads mass/inertia/geometry out of the running sim and emits the PX4 values |
| `gain_search.py` | Relaunch-per-candidate verification with ulog scoring. **Stages 3–4.** |
| `loop_latency.py` | Loop delay + `K_eff` (four fitting bugs fixed — see FINDINGS §9) |

### Committed vs not, so nothing gets lost

Each commit is **one mismatch**, so any of them can be reverted alone.

```
PegasusSimulator                        PX4-Autopilot
  5560e9d  backend survivable            1d8e8ee  MC_CTRL_GT
  5b9633f  px4_tuning harness            9e9f4b2  airframe: EKF2 fix + params
  6a39ec5  rotor lag
  9dcf764  torque saturation           uncommitted (yours, WIP):
  b72d634  knobs on both entry points    TiltEigenController.*  (skipped)
```

**Since committed.** What this section listed as uncommitted — `desaturation.py`,
`multirotor.py`, `test_rotor_failure.py`, `motor_failure.py` and `flight/controllers.py` —
now lands as `5fbe368` (the mixer policy), `5077622` (its checks), `d92a267` (the policy on
both rotor-failure entry points) and `39bc57a` (`rotor_tau` into `flight/controllers.py`),
keeping the one-mismatch-per-commit rule above. `593decc` (gyroscopic sign) and `175373b`
predate this work.

---

## Mismatch ledger: Pegasus GA vs true PX4

| mismatch | state | knob | now |
|---|---|---|---|
| **rotor/ESC lag** | modelled, **value guessed** | `--rotor-tau-ms` | 20 ms |
| **state transport delay** | modelled, **OFF** | `--att-delay-ms` | **0** |
| torque saturation | matched | `--torque-max` | 3.843 |
| gyroscopic term | matched | — | — |
| loop rates | matched | — | 250 Hz both |
| estimator | removable in PX4 | `MC_CTRL_GT` | 0 |
| thrust/tilt limits | not modelled | — | collective/trajectory only |
| hover-thrust normalisation | not modelled | — | collective only |
| allocator desaturation | **modelled, selectable** | `--desaturation` | `uniform` (unchanged); binds 0.2% of samples |
| slew limit | not modelled | `CA_R*_SLEW` | 0 = off in PX4 too |

**Only the top two are actually open.** The bottom four are deliberate: they touch the
collective and saturated regimes, not the small-signal attitude boundary being searched.

---

## Traps that cost real time here

**`initial_parameters` is not what flew.** `MC_CTRL_ALG` is deferred to disarm and live QGC
sets appear only in `changed_parameters`. Read `mc_controller_status.algorithm` and check
`fallback_active`. `flight_health.py` does both, restricted to the airborne window.

**Check `SYS_AUTOSTART` before believing any log.** Every ulog on disk before 2026-08-07 was
`4001` — a Gazebo x500, not this vehicle. `px4_driver.check_airframe()` now refuses to arm
otherwise.

**Don't pass the full `tau_eff` as `--att-delay-ms`.** The Python controller is a 250 Hz
physics callback and already carries its own ZOH — 4–6 ms of PX4's 12–24. Passing the full
number searches a plant ~50% laggier than PX4.

**`EKF2_DELAY_MAX` is `reboot_required`.** Setting it live from QGC writes the param and
never reaches the running estimator. It has to come from the airframe.

**Single flights are weak evidence.** Before the EKF2 fix, *identical* parameters gave
opposite outcomes. Prefer repeats, especially for anything at the ceiling.

**`b` cannot damp anything.** `Om_err(t) = Om_err(0)·exp(-wn·t)·exp(+i·b·t)` — it sets the
direction the error spirals, not the decay rate, which is `wn` regardless. It costs margin,
since the loop delay acts at `sqrt(wn²+b²)`.

**Cascade separation `wn/ATT_P >= 3`.** GA rank 3 sat at 1.45, the loops interacted, and it
departed in 12.9 s. Enforced in `gain_search.py`'s grid rather than searched.

---

## Run log

**2026-08-16 — Stage 1 launched** on placeholders (`T = 20 ms`, `att/pos delay 8 ms`,
`torque_max 3.843`), eigen, `mm = 2.7e-7`, pop 16 × 30 gens — matching the July run's budget
so the two fronts are comparable. Log: `/home/ani/ga_logs/`.

The pre-existing lag-free front was copied to
`results/genetic_eigen_sweep_NOLAG_baseline_20260727/` first, because the GA **overwrites**
`mm_2.7e-07/` in place. That baseline is the comparison, so losing it would have cost the
whole point of the run.

### The July archive is NOT a valid control — corrected 2026-08-16

I initially planned to difference the lagged front against
`genetic_eigen_sweep_NOLAG_baseline_20260727/`. **That comparison is void**, and it would
have attributed three changes to one cause:

| | July baseline | today |
|---|---|---|
| archive schema | 24 columns | 42 columns |
| **task** | spawn **at** the target, zero initial position error | ground start + lateral goal |
| scoring | pre-`fc8d206` | `score_transient` / `score_tail` / `failure` |

The task change is `d3d119b` (2026-07-27 14:58) — "Runs previously spawned the vehicle AT
its target … the attitude loop was never exercised." The baseline directory predates it.
So those gains were tuned for a task that barely used the attitude loop, and their `score`
is a different quantity computed on a different flight.

**The control has to be today's code with the lag knobs zeroed.** That run is chained to
start automatically when the lagged run exits (`/home/ani/ga_logs/chain_control.sh`), same
seed, same budget, same scoring — one variable. Results land in:

```
results/ga_LAG_20260816/mm_2.7e-07/          <- rotor 20 ms, att/pos delay 8 ms
results/ga_NOLAG_control_20260816/mm_2.7e-07 <- all lag knobs 0
```

The July archive is still worth keeping, but only as a record of what the old task
rewarded — never as the other half of a delta.

### Prediction, registered before the result — **WRONG, and instructively so**

Predicted: *"`wn` should fall substantially in the lagged front."* Reasoning: phase loss at
the rate pole is `wn·tau_eff`, so `wn = 46` at `tau_eff ≈ 28 ms` costs ~74°.

**Measured at gen 8 (n=144): `corr(wn, score) = -0.002`. `wn` is invisible to the score.**
Best score per bin is 0.0094 / 0.0095 / 0.0095 across `wn` from 20 to 60.

The prediction failed because it named the wrong binding parameter, **not** because the lag
was absent — the lag was verified independently (below).

```
correlation with score, n=144 scored individuals
  Kd_att     r = +0.814     <- dominant
  b_mdyn     r = +0.740     <- higher b is WORSE
  Kp_att     r = -0.608     <- higher Kp is better
  wn_mdyn    r = -0.002     <- invisible
```

| `Kd_att` | n | best | median |
|---|---|---|---|
| 0.00–0.25 | 17 | **0.0094** | 0.0098 |
| 0.25–1.0 | 9 | 0.0485 | 0.0897 |
| 1.0–3.0 | 34 | 0.0308 | 0.3113 |
| 3–10 | 30 | 0.4967 | 2.845 |
| 10–31 | 54 | 2.8007 | 37.51 |

Monotone across **300×**.

**`Kd_att`'s optimum sat ON the 0.05 gene floor** — the search was pressing the bound again,
exactly as the `EIGEN_GENES` comment describes it doing at the old floor of 1.0. The true
optimum may be below 0.05, or zero (pure-P attitude). `MC_EIG_ATT_D 0.1` is in the right
region; the GA's historical `1.0` was not.

> **Acted on (`3372385`).** The box is now `[-0.1, 0.1]`, so the search brackets zero
> instead of approaching it from above and can go negative rather than piling up on a
> bound. The next GA run is the first that can actually answer where the optimum is. Note
> the `EIGEN_GENES` comment still describes the old `[0.05, 30.0]` box.

#### `b`'s correlation was a confound — do not read it as evidence

`b_mdyn` shows `r = +0.740` raw, and it is tempting to call that a case against
`MC_EIG_B 13.15`. It is not. **`corr(b, Kd_att) = +0.493`** — the genes move together, and
`b`'s apparent effect is mostly `Kd`'s shadow:

```
within the Kd <= 0.25 stratum (n=44, Kd spread 0.050-0.197)
  corr(b, score) = +0.030          <- essentially zero
    b [0.0, 0.5)  n=28  best=0.0093  median=0.0096
    b [0.5, 2.0)  n= 3  best=0.0095  median=0.0103
    b [2.0, 5.0)  n=13  best=0.0094  median=0.0095
```

Worse, that stratum contains **no individual with `b > 5`** — so the value the airframe
actually flies is not merely unsupported here, it is **untested**. Any gene correlation off
this archive must be checked within a stratum before it is believed; the population is
selected, not designed, so the genes are not independent.

The real case against `b` remains what it always was, and is independent of this run: the
theory (`|wn - i·b| = sqrt(wn²+b²)`, so `b` only spends margin) and the flight evidence that
`b = +19.3` and `b = -5.0` both diverged.

**Still open:** whether `Kd`'s dominance is a *lag* effect or a property of the task. Only
the no-lag control separates those, which is what makes it worth its 100 minutes.

### The result: lag shrinks the basin, it does not lower the ceiling

Gen 0 is a controlled experiment obtained for free. Both runs use `--seed 42`, so the
initial LHS population is **byte-identical** (verified: the 16 gene vectors match exactly).
That makes gen 0 the same genes evaluated on two plants — the only truly clean comparison
in this whole exercise, because nothing has been selected yet.

```
median lag cost across the identical population:  6.72x
worse with lag: 12/16      better: 0/16

the one individual that was genuinely good without lag:
  wn=39.70  b=1.08  Kp=3.43  Kd=1.71
     NO-LAG  0.0124
     LAG     0.1743        <- 14.1x worse, identical gains
```

Four of the sixteen sit near ratio 1.0, but those were already failing on **both** plants
(~37-39 either way) — saturated at the score ceiling, so their ratio carries no information
and drags the 6.72x median *down*. The cost among flyable gains is higher.

Meanwhile the **converged** optimum is the same on both plants (~0.0093 lagged vs ~0.0093
unlagged at matched generations, effort 63.14 either way).

**So: lag does not make the problem harder to solve — it makes the solution harder to
find.** A gain set tuned on a lag-free plant is roughly an order of magnitude worse once
the lag is there, even though an equally good gain set exists for the lagged plant.

This is the answer to the original question — *why does PX4 with lag fail where Pegasus
flies?* Not because lag lowers what is achievable. Because the gains were found on a plant
without it, and that basin does not transfer. It also explains why the airframe's
GA-derived gains never worked well: **every archive on disk was searched on a lag-free
prototype**, so all of them are drawn from the wrong basin.

The practical consequence for Stage 3: candidates must come from the **lagged** front. Gains
from any pre-2026-08-16 archive are not merely stale, they are from a different problem.

### Stage 3/4 RESULT (2026-08-16): the estimator is the limit at 13.2 rad/s, not the gains

Flown in PX4 with the lagged GA rank-1 gains written into the airframe
(`WN 34.1837, B 0.0, ATT_P 4.1186, ATT_D 0.05`, separation 8.30), `mm 2.7e-7`,
90 s commanded hold at 6 m. **Only `MC_CTRL_GT` differs between these two flights.**

| | ground truth | estimator |
|---|---|---|
| outcome | **held 90 s at 5.96 m** | **inverted at t+15 s** |
| airborne | 84.5 s | 27 s |
| roll err rms | 0.047 rad | 1.674 rad (96 deg) — **36x** |
| pitch err rms | 0.048 rad | 0.712 rad |
| coning rms | 0.250 rad/s | 1.447 rad/s |
| coning peak | 3.773 rad/s | 43.688 rad/s |
| torque x rms | 0.070 | 0.478 |
| motors on a rail | 0.2% | 23.0% — **100x** |
| lateral setpoint | — | `x 0 cm, y 0 cm` (never moved) |

The estimator run ended at `roll = -3.08 rad` (-176 deg, inverted) with spin at ~0, and sat
there for 70 s while the driver kept commanding a 6 m hold. The lateral setpoint never
moved, so nothing commanded the departure.

**`EKF2_PREDICT_US 4000` did not fix the ceiling, it moved it.** There is a second
estimator failure below 13.2 rad/s, and it is not the 100 Hz-filter problem that was fixed
in §10 — that one is gone (estimate and truth agree to 0.01 m on the ground-truth run).

Note the ground-truth run achieves this while `vehicle_attitude_groundtruth` publishes at
**50 Hz against the estimate's 250 Hz** — the control law wins through a 5x slower state
update. Sample rate is not what separates these two flights.

**Not yet established: that the gains are GOOD.** Even on truth, coning grows
(`+0.113 per min`), torque touches the rail (0.2%), and the 0.047 rad error over 84 s is
20x the 0.0022 rad measured over the first 8 s. "Flies on truth" is proven; "converged
hover" is not.

**Consequence for the ladder:** Stage 4 is now the interesting one, and the question is no
longer "what do the gains cost" but "what does EKF2 do wrong between 10 and 13 rad/s".

### 560 Hz attempt (2026-08-16): rate chain works, EKF2 will not arm — REVERTED

The Kakute H7's ICM-42688-P runs an 8 kHz internal ODR but transfers its FIFO every
1250 us = **800 Hz**, which is what reaches EKF2. 70% of that is 560 Hz. Rotation per
filter step would have gone 3.03 deg -> 1.35 deg at 13.2 rad/s, below the 1.81 deg that
flew 163 s after the last EKF2 fix.

**What worked.** All three rates have to move together, and the middle one is the trap:

```
Pegasus --physics-rate 560   ->  HIL_SENSOR at 560 Hz
IMU_INTEG_RATE  250 -> 560   ->  VehicleIMU delta-angles at 560 Hz
EKF2_PREDICT_US 4000 -> 1786 ->  filter steps at 560 Hz
```

`IMU_INTEG_RATE` must be `param set`, NOT `param set-default`: px4-rc.simulator:5 sets it
to 250 AFTER the airframe is sourced (rcS:224), so a set-default is silently overwritten.
With that fixed, measured 560.2 Hz on `vehicle_angular_velocity`, `vehicle_attitude` and
`vehicle_local_position`, zero lockstep poll timeouts, and **cleaner** timing than 250 Hz
(sensor dt p95 1.785 ms vs 8.000 ms).

**This corrects single_vehicle.py:189-210.** "PX4 emits HIL_ACTUATOR_CONTROLS at 250 Hz no
matter how fast HIL_SENSOR arrives" was never a protocol limit - it was this same
overridden `IMU_INTEG_RATE` pinning `vehicle_angular_velocity`, hence the rate loop, hence
`actuator_outputs`. Feeding 560 Hz of sensor data into a 250 Hz reply rate starves the
lockstep: actuator_outputs collapsed to 10 Hz and 3.7 s of sim took ~90 s of wall clock.

**Why it was reverted anyway.** With the chain genuinely at 560 Hz:

```
WARN [health_and_arming_checks] Preflight Fail: heading estimate not stable
WARN [commander] Arming denied: Resolve system health failures first
```

The vehicle never flew because it was never allowed to arm - and on the run before, the
estimate wandered +-12 m of altitude with the vehicle STATIONARY. EKF2 does not converge at
560 Hz in this setup. That is a filter problem, not a spin problem, and unrelated to the
13.2 rad/s divergence this was meant to fix.

`EKF2_DELAY_MAX 100 -> 40` was tried as a fix (the IMU buffer is
`ceil(EKF2_DELAY_MAX / filter_period)`, so 25 entries at 250 Hz becomes 56 at 560 Hz) but
**it never took: EKF2_DELAY_MAX is `reboot_required` and read back as 100.0**. So that
hypothesis is still untested, not refuted.

**Status: airframe reverted to the 250 Hz chain** (`EKF2_PREDICT_US 4000`,
`EKF2_DELAY_MAX 100.0`, `IMU_INTEG_RATE` line commented out). The GA rank-1 gains are
unchanged. Anyone retrying this needs to solve "heading estimate not stable" first -
`EKF2_MAG_TYPE` is 0 (automatic), and a magnetometer on a vehicle spinning at 13 rad/s is
the obvious suspect.

### Void-check — the version that actually holds

An earlier "the lag is reaching the plant" claim compared lagged gen-0 scores against the
July baseline's, which is the same cross-task cross-scoring comparison discarded above. It
proved nothing.

The check that does hold is direct: stepping `QuadraticThrustCurve` with
`rotor_tau = 0.020` and `dt = 1/250` reaches **63.2 % of the step at exactly 20 ms**, and
`rotor_tau = 0` reaches 100 % on the first tick. The lag is real and correctly scaled.
Reach for this test, not a cross-run score comparison.

Also worth noting: **cascade separation dropped 0 of 176** converged individuals in the
July archive (all sat at 7–12×). The constraint that killed rank 3 is not binding on
converged fronts — it was a hazard of picking off an early front, not of the search itself.
Enforcing it in `ga_to_candidates.py` costs nothing, but it is not doing real work.

### Provenance gap found while wiring this up

The airframe's current gains (`MC_EIG_WN 16.34`, `MC_EIG_B 13.15`, `ATT_P 4.68`) appear in
**no `genetic_archive.csv` on disk**. FINDINGS §8 cites them as rank 2 of a Pareto front,
but that archive is not among the ones retained, and every surviving eigen archive wants
`b ≈ 0–1.2`, not 13.15. Treat the current airframe gains as **unreproducible**, not as a
validated baseline — which is a further reason to let Stage 1 supersede them rather than
tune around them.

---

## Open questions

- **`T` and `tau_eff` are unmeasured.** Everything downstream is provisional until Stage 0.
- **Is 25 rad/s reachable?** At `T = 20 ms` the stability model finds *no stable `wn`*; at
  10 ms it does. That is an actuator-bandwidth question, answerable on the bench before any
  parts decision.
- **Which half of `EKF2_PREDICT_US` does the work** — it halves rotation-per-filter-step
  *and* doubles the position stage 125→250 Hz. Not separated.
- **`EKF2_DELAY_MAX = 100`** only ever applied from boot once. 100 ms is still 79° of
  rotation at 7.9 rad/s and the largest delay that actually applies here is GPS at 10 ms.
