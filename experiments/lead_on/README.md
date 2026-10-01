# Lead-on — experiments flown with the actuator lead compensator active

Opened 2026-09-20. Everything in here ran with `MC_LEAD_EN = 1` for at least one arm, i.e.
with the commanded body torque passed through

```
u = tau + MC_LEAD_TC * d(tau)/dt
```

which inverts the rotor's first-order pole `1/(1 + sT)` — phase **and** droop — and leaves
the actuator looking like its bare transport delay. It implements the feed-forward half of
Lee et al., IEEE/ASME T-MECH 30(6) 5295–5306, 2025. The compensator is documented in
[`../../notes/ACTUATOR_LEAD.md`](../../notes/ACTUATOR_LEAD.md).

## Why this is a separate folder from `phase_only/`

[`../phase_only/`](../phase_only/) is the archive of everything flown when the **seat was the
only lag tool in the stack**. A seat rotates the commanded wrench and nothing else, so it
inverts only the *argument* of `Ga(iω) = e^{−iωτ_a}/(1 + iωT)`; the modulus went
uncompensated in every run in there. That archive's own README records the consequence:

> Add a 20 ms pole and you add attenuation, which the seat cannot undo: at 20 ms **both**
> arms die at every speed from 2 to 5 m/s. The seat's contribution is phase, not authority.

The runs in *this* folder are the first that address the pole directly. They are therefore
not comparable cell-for-cell with `phase_only/` — the plant the controller sees is different
— and they do not supersede it either. `phase_only/` remains the seat's own envelope, and
several of its runs are re-used here as the **control arm** that the lead is measured
against.

## The division of labour, which is the point

A transport delay is non-minimum phase: `e^{+sL}` is non-causal and no feed-forward can
produce it. A pole is invertible. So the lead takes the pole, the seat takes the delay, and
the 2×2 in `hover_level3_actuator_lead/` shows that **neither alone is sufficient** at a lag
where the pair flies. The lead does not demote the seat; it removes the half of the plant the
seat structurally cannot reach.

## What is in here

| folder | rung | what it establishes |
|---|---|---|
| [`hover_level3_actuator_lead/`](hover_level3_actuator_lead/) | L3 hover | the 2×2, the `MC_LEAD_DMAX` control, and the mismatched-`TC` sweep — 38 runs |
| [`trajectory_level3_actuator_lead/`](trajectory_level3_actuator_lead/) | L3 waypoint ellipse | the spin ceiling on a setpoint profile: **22 rad/s at a 30 ms pole, ≥29.5 with no pole** — 20 runs |

**Hover and trajectory do not give the same ceiling.** At `rotor_tau = 0.030` hover flies
5/5 at r = 29.5 while the ellipse tops out at 22. Quote the profile with the number.

The trajectory folder also carries three scoring traps this profile sets — `AUTO_LAND` is not
a crash, "trajectory complete" does not mean the path was flown, and the flight duration is
bounded by **wall clock** so survival time is not comparable between runs. The shared scorer
[`../tools/score_waypoint_ellipse.py`](../tools/score_waypoint_ellipse.py) encodes all three.

## A trap to know about before reproducing anything in here

**The `MC_LEAD_DMAX` default changed after these runs flew**, from 200 to 25, on the strength
of what they show — and SITL was not rebuilt in between. Any run whose ulog does not record a
`MC_LEAD_DMAX` change took the **old default of 200**, not today's 25. Each experiment's
`RUN_LOG.md` states this and its manifest resolves the value per run from the ulog itself.

Tooling in [`../tools/`](../tools/) is shared with `phase_only/` and did not move.
