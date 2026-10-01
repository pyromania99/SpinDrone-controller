#!/usr/bin/env python
"""
| File: actuator_lead.py
| Description: Inverts the rotor's first-order pole so the seat is left with a pure
               delay. Level-1 twin of PX4's ActuatorLead (MC_LEAD_*).
               After Lee et al., IEEE/ASME T-MECH 30(6) 5295-5306, 2025.

WHAT IT IS, AND WHY IT IS NOT THE SEAT
──────────────────────────────────────
The actuator between a commanded and a delivered torque is roughly

    Ga(s) = e^{-s L} / (1 + s T)        L transport delay, T rotor/ESC pole

and under a spin at rate r both terms ROTATE the delivered wrench in the body
frame. The seat cancels that rotation geometrically, and that is all it can do -
in particular it cannot recover the pole's amplitude droop 1/sqrt(1+(rT)^2).

This does the other half. Modelling the rotor as tau_dot = (u - tau)/T, the exact
inverse of the pole is

    u = tau_des + T d(tau_des)/dt        i.e. multiply by (1 + T s)

which cancels the pole ENTIRELY, phase and droop, leaving the actuator looking
like the bare delay e^{-s L}. A delay is non-minimum phase and no causal filter
inverts it, so what remains is exactly the seat's job:

    lead handles the pole      invertible, needs T
    seat handles the delay     not invertible, needs no model

MEASURED ON THE HARDWARE (notes/AIRFRAME_ID_METHOD.md, ev_delay.ulg): at 20 rad/s
the plant carries 43.8 deg of phase - 17.2 deg delay, 26.6 deg pole - and the pole
droops 10.6 %. Inverting it leaves 17.2 deg for the seat. That helps twice, because
the seat is measured to cancel 1.14-1.32 of the true phase against a PURE DELAY and
only 0.39-0.63 once a pole is present. Removing the pole also moves the seat into
the regime where it works.

WHY A TRACKING DIFFERENTIATOR AND NOT A DIFFERENCE
──────────────────────────────────────────────────
(1 + T s) has gain |1 + i w T|: 1.1x at a 20 rad/s spin, 7.9x at 50 Hz, 15.7x at
100 Hz. A naive difference of tau_des injects that much noise into the motors. The
ESO tracking differentiator (the source's Algorithm 1) estimates the derivative
while filtering, and is the reason the derivative is usable at all.

WHAT IS NOT IMPLEMENTED
───────────────────────
The source also closes a PID loop on the torque ERROR tau_des - tau_B, which needs
the DELIVERED torque. Without rotor RPM there is no route to tau_B here except
omega_dot minus the modelled drift, and that reconstruction is exactly what refuted
the seat's magnitude term - it read 0.044 to 1.225 where the truth was 1.000. Only
the feed-forward half is implemented, and that is the half carrying the result: the
source's Fig. 4(a) reaches 1.5 Hz without the derivative term and 5.5 Hz with it.

ORDER
─────
Apply LAST, after the seat. The actuator then receives lead(seat(tau)) and delivers
e^{-sL} seat(tau). The seat's observable is unaffected: it builds xd from the
PRE-seat torque and xa from the measurement, so it simply sees a faster actuator and
adapts to the smaller residual. Nothing here competes with the adaptation - unlike a
static extra rotation, which the adaptation would integrate straight back out.

Keep in step with PX4's ActuatorLead.cpp: the point of the ladder is that level 2
minus level 1 is PX4's cost, which only means something if both rungs run the same
law.
"""
from __future__ import annotations

import math

import numpy as np


class ActuatorLead:
    """u = tau + T * d(tau)/dt, with an ESO-filtered derivative. Three axes."""

    def __init__(self, enabled: bool = False, tc: float = 0.020,
                 wn: float = 150.0, delta: float = 0.0, rate_max: float = 25.0):
        #: Off by default, so a run without it is bit-identical to before this existed.
        self.enabled = bool(enabled)
        #: [s] the pole being inverted. MEASURE it - see notes/AIRFRAME_ID_METHOD.md.
        #: Hardware reads 25 ms median with a 12.5-40 ms spread, and that spread is the
        #: dominant uncertainty in the whole scheme. Err LOW: over-inversion adds lead
        #: the plant does not have and amplifies noise for nothing.
        self.tc = max(float(tc), 0.0)
        #: [rad/s] differentiator bandwidth - the real knob. Both ESO gains derive from
        #: it: beta1 = 2 wn (critically damped), beta2 = wn^2 sqrt(delta).
        #:
        #: DO NOT use the source's Table I values (beta1 = 1, beta2 = 500). Linearised
        #: inside the dead zone its (19) is a differentiator rolled off at
        #: wn = sqrt(beta2/sqrt(delta)) with damping beta1/(2 wn), and those values put
        #: the damping at 0.004 - an 89x resonant peak. Against a known plant reading
        #: 0.894/-24.5 deg uncompensated they give 1.410/-74.7 deg, WORSE than nothing.
        self.wn = max(float(wn), 0.0)
        #: fal() dead-zone half-width. 0 means "use dt", which is what the source
        #: prescribes - but that ties the nonlinearity to the loop rate, which is why the
        #: same wn measures differently at 250 Hz and 2 kHz. Set it explicitly (roughly
        #: the torque noise floor) to make the tuning portable across the ladder.
        self.delta = max(float(delta), 0.0)
        #: Hard clamp on d(tau)/dt before it is scaled by tc. Not cosmetic - it is the
        #: only guard between the differentiator and the mixer.
        self.rate_max = max(float(rate_max), 0.0)
        self.reset()

    def reset(self):
        self.z1 = np.zeros(3)   # tracked torque
        self.z2 = np.zeros(3)   # tracked derivative - the output
        self._initialised = False

    @staticmethod
    def _fal(e: float, alpha: float, delta: float) -> float:
        """Han's nonlinear gain: linear inside the dead zone, sqrt outside.

        The linear interior stops the observer chattering once the error is at the
        noise floor; the sub-linear exterior converges fast on a large error without a
        correspondingly large gain. The source's (20).
        """
        if delta <= 0.0:
            return e
        if abs(e) <= delta:
            return e / (delta ** (1.0 - alpha))
        return math.copysign(math.sqrt(abs(e)), e)

    def apply(self, torque, dt: float):
        """Return u = tau + tc * d(tau)/dt. Pass-through when disabled."""
        tau = np.asarray(torque, dtype=float)
        if not self.enabled or not dt > 0.0 or not np.all(np.isfinite(tau)):
            return tau

        # First tick: start the observer ON the signal. Starting at zero emits a large
        # spurious derivative while z1 catches up, which on a vehicle is a torque spike.
        if not self._initialised:
            self.z1 = tau[:3].copy()
            self.z2 = np.zeros(3)
            self._initialised = True
            return tau

        delta = self.delta if self.delta > 0.0 else dt
        beta1 = 2.0 * self.wn                      # zeta = 1
        beta2 = self.wn * self.wn * math.sqrt(delta)
        for i in range(min(3, len(tau))):
            e = self.z1[i] - tau[i]
            z1_dot = self.z2[i] - beta1 * e
            z2_dot = -beta2 * self._fal(e, 0.5, delta)
            self.z1[i] += dt * z1_dot
            self.z2[i] += dt * z2_dot
            if not (np.isfinite(self.z1[i]) and np.isfinite(self.z2[i])):
                self.z1[i], self.z2[i] = tau[i], 0.0
            if self.rate_max > 0.0:
                self.z2[i] = float(np.clip(self.z2[i], -self.rate_max, self.rate_max))

        out = tau.copy()
        out[:3] = tau[:3] + self.z2 * self.tc
        return out

    def describe(self) -> str:
        if not self.enabled:
            return "lead=off"
        return (f"lead=on tc={self.tc*1e3:.1f}ms wn={self.wn:g} delta={self.delta:g} "
                f"dmax={self.rate_max:g}")
