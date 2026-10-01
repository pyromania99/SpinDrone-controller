# Phase-only archive — every experiment run before the seat had a magnitude term

Archived 2026-09-19. Nothing in this folder was edited on the way in; the run directories
keep their original names, figures and numbers, and only the relative links were repointed
one level deeper.

## What "phase only" means

Every run below was flown with a seat that **rotated** the commanded roll/pitch wrench and
nothing else. The actuator at the spin frequency is

```
Ga(iω) = e^{-iω·τ_a} / (1 + iω·T)
```

and a rotation inverts only its argument. The modulus `1/sqrt(1+(ωT)^2)` was left
uncompensated in all of these runs. That is not a detail — it is the finding that
[`trajectory_level3_px4_estimator_fast_circle/RUN_LOG.md`](trajectory_level3_px4_estimator_fast_circle/RUN_LOG.md)
recorded:

> Add a 20 ms pole and you add attenuation (droop 0.86 at this spin), which the seat cannot
> undo: at 20 ms **both** arms die at every speed from 2 to 5 m/s. The seat's contribution
> is phase, not authority.

So the seat-on/seat-off comparisons in here are comparisons of *phase compensation*, and
every "neither arm survives `rotor_tau=0.02`" result is a statement about a compensator that
was only ever inverting half of `Ga`.

## Why they have to be rerun rather than reinterpreted

A magnitude term was added on 2026-09-19 to compensate that droop and **reverted the same
day**. The premise was that `|xa|/|xd|` measures the actuator's `|Ga(iω)|`; it does not.
`xa` is a reconstruction (`ω̇_meas` minus the *modelled* drift) and under spin the `λr`
precession dominates that subtraction, so the ratio carries model error, not actuator
response. Measured across the 32 runs in here flown at `rotor_tau = 0`, where the true
`|Ga|` is exactly **1.000**: the ratio reads **0.044 to 1.225**, and exceeds 1, which an
attenuating actuator cannot do. See `notes/HARDWARE_READINESS.md` §2b.

So these runs remain phase-only, and that is not currently a deficiency to be fixed — droop
is 1.14× of a 23× demand effect whose mechanism is still underived
([`analysis/SESSION_2026-09-18.md`](analysis/SESSION_2026-09-18.md) §3c).

**These runs are still the baseline; they have not been superseded.** What changed underneath
them is the saturation gate, below.

At **level 1** these runs reproduce exactly — nothing in the level-1 seat path changed.

At **levels 2 and 3** they reproduce with **`MC_SEAT_SATGATE = 1`**, which is the default.
That parameter exists because of what follows.

## The angle law was frozen for much of these flights

Until 2026-09-19, PX4's `Seat::adapt()` gated the angle adaptation on allocator saturation
unconditionally, while level 1 never drove that flag at all. It is now
`MC_SEAT_SATGATE` (PX4) / `--seat-satgate` (level 1), **default 1 = gated**, so this archive
stays re-derivable and the ungated case becomes a measurable experiment rather than a silent
change.

The gate was not idle here. Across **all 51** archived level-2/3 ulogs, roll/pitch allocator
saturation occurs on every single one, from 10 % of ticks to 95 %. On the arms that were
adapting, the gate explains the freezing and nothing else does:

| log | rp-saturated | seat gated | gated given saturated | gated given NOT saturated |
|---|---|---|---|---|
| `19_07_02` | 80.1% | 80.3% | **99.3%** | 1.6% |
| `18_20_46` | 13.1% | 13.6% | **100%** | 0.0% |
| `19_14_22` | 12.7% | 13.1% | **100%** | 0.0% |
| `19_10_40` | 12.8% | 13.5% | **100%** | 0.2% |

And the skipped ticks were **not a random sample**: mean `|xd|` on them ran **1.3× to 32×**
the ticks the law did use, so the gate removed exactly the ticks `MC_SEAT_LAW = cross`
weights most heavily.

So where a run in here concludes that the adaptive seat could not rescue an arm, part of what
it observed is a seat switched off for between 12 % and 80 % of the flight. That is a second
qualification on this folder, alongside the phase-only one, and it was not known when these
were written. It is **not** a reason to discount the results — the tree's best-established
result (L3 hover, r = 29.4, n = 5, θ_s = −0.4731 ± 0.0009) was obtained gated — but it is a
reason to re-run the comparison with `SATGATE = 0` before concluding the seat has a ceiling.

## What is in here

| folder | rung |
|---|---|
| [`hover_level1_km6e7_seat_off_fixed_adaptive/`](hover_level1_km6e7_seat_off_fixed_adaptive/) | L1 hover, the 2×2 |
| [`trajectory_level1_circle_seat/`](trajectory_level1_circle_seat/) | L1 circle |
| [`trajectory_level1_seat_b_geometry_sweep/`](trajectory_level1_seat_b_geometry_sweep/) | L1 geometry sweep |
| [`trajectory_level1_waypoint_ellipse_2x2_arms/`](trajectory_level1_waypoint_ellipse_2x2_arms/) | L1 waypoint ellipse |
| [`hover_level2_px4_groundtruth/`](hover_level2_px4_groundtruth/) | L2 hover |
| [`trajectory_level2_px4_groundtruth_fast_circle/`](trajectory_level2_px4_groundtruth_fast_circle/) | L2 fast circle |
| [`hover_level3_px4_estimator/`](hover_level3_px4_estimator/) | L3 hover |
| [`hover_level3_rotor_loss/`](hover_level3_rotor_loss/) | L3 rotor loss |
| [`trajectory_level3_px4_estimator_fast_circle/`](trajectory_level3_px4_estimator_fast_circle/) | L3 fast circle |
| [`supporting_b_pole_theory_and_archive/`](supporting_b_pole_theory_and_archive/) | pole-angle theory |
| [`analysis/`](analysis/) | the cross-cutting sheets for all of the above |

The tooling did **not** move: [`../tools/`](../tools/) is shared with the reruns, so the new
figures are produced by the same scripts as these.

## Reading anything in here afterwards

Take every number as conditional on "phase compensation only". Where a run concludes that the
seat could not rescue an arm, the honest reading is that a *rotation* could not, which is a
narrower claim than it looks — and the one the gain term exists to test.
