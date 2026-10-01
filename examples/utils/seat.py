#!/usr/bin/env python
"""
| File: seat.py
| Description: The command-frame rotation ("seat") that cancels actuator lag's
               phase rotation on a spinning multirotor. Brief v2.

THE MECHANISM
─────────────
Pack roll/pitch as a complex number zeta = p + iq; then S (multiplication by i)
is just i, every (aI + bS) matrix is the complex scalar a + ib, and R(theta) is
multiplication by e^{i theta}. In that form:

    plant     zeta_dot = A zeta + tau_delivered/J + d,   A = a + i*lam*r
    command   tau_cmd  = J (M - A) zeta                  M = designed error dyn
    seat      u        = e^{i theta_s} tau_cmd
    actuator  tau_del  = Ga[u],  Ga(s) = e^{-s tau_a}/(1+sT)

The observable compares the acceleration a LAGLESS actuator would have produced
against the one actually produced:

    xd = tau_cmd/J = (M - A) zeta
    xa = tau_delivered/J

For single-frequency content zeta = Z e^{i w t}, Ga acts as multiplication by
Ga(iw), so (M - A) zeta cancels between the two and

    alpha = arg(xa/xd) = theta_s + arg Ga(iw)      =>   d(alpha)/d(theta_s) = 1

M (hence b*), A (hence lam), J and the state's own amplitude and phase all cancel
identically. What is left is the actuator's phase and nothing else. That exact
unit derivative is what makes  theta_s += -k*alpha*dt  a well-posed law: it is the
condition under which "reduce alpha" has an answer at all. An observable whose
derivative is not 1 has no theta_s that zeroes alpha, and the integrator rails.

WHAT A VEHICLE CAN ACTUALLY MEASURE  (see WrenchObserver)
────────────────────────────────────────────────────────
xa above is tau_delivered/J, which no vehicle can measure in isolation. From the
plant it is recoverable as

    tau_delivered/J = zeta_dot_meas - A zeta - d

so subtracting the MODELLED drift from the measured angular acceleration gets
there, up to the disturbance d. d does not cancel, and it sits at the same
frequency as the command (-r in the body frame), so no filter separates them.

    KNOWN LIMIT, measured in seat_harness.py: this needs commanded content at
    least ~100x the disturbance. At 30x the gain is 0.34 and it does not
    converge; at 100x it is 0.92 and it does. A station-keeping vehicle sits near
    1x, because there the command IS the response to the disturbance. Expect this
    to work while manoeuvring and not while merely holding position.

Validated in examples/tools/evaluations/seat_harness.py, which imports this file
so that what flies is what was tested.
"""
from __future__ import annotations

import math

import numpy as np


def R(theta: float) -> np.ndarray:
    """Rotation by theta in the roll/pitch plane."""
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s], [s, c]])


def signed_angle(xd, xa, eps: float = 1e-12) -> float:
    """Signed angle from xd to xa [rad], NaN if either lacks a usable direction."""
    if not (np.all(np.isfinite(xd)) and np.all(np.isfinite(xa))):
        return float("nan")
    if np.hypot(*xd) < eps or np.hypot(*xa) < eps:
        return float("nan")
    return math.atan2(xd[0] * xa[1] - xd[1] * xa[0],
                      xd[0] * xa[0] + xd[1] * xa[1])


class Seat:
    """One scalar of state: the angle the commanded roll/pitch wrench is rotated by."""

    OFF = 0       #: pass the wrench through untouched
    FIXED = 1     #: theta_s = r*tau_a + atan(r*T), from the live spin rate
    ADAPTIVE = 2  #: theta_s driven by the measured misalignment

    MODES = {"off": OFF, "fixed": FIXED, "adaptive": ADAPTIVE}

    #: |theta_s| < pi/2 is a realizability limit, not a safety margin: holding an
    #: angle theta costs a pole magnitude growing as 1/cos(theta).
    THETA_CEILING = 1.5

    ANGLE = "angle"   #: theta_s -= k * alpha * dt              (alpha from atan2)
    CROSS = "cross"   #: theta_s -= k * (xd x xa) * dt          = k |xd||xa| sin(alpha)

    def __init__(self, mode=ADAPTIVE, k=2.0, theta_max=1.4, r_min=5.0,
                 eps=1e-9, tau_a=0.002, T=0.020, k_sign=1.0, law=CROSS,
                 sat_gate=True):
        self.mode = self.MODES[mode] if isinstance(mode, str) else int(mode)
        self.k = max(float(k), 0.0)
        self.theta_max = float(np.clip(theta_max, 0.0, self.THETA_CEILING))
        self.r_min = max(float(r_min), 0.0)
        self.eps = float(eps)
        self.tau_a = max(float(tau_a), 0.0)
        self.T = max(float(T), 0.0)
        self.k_sign = -1.0 if k_sign < 0 else 1.0
        if law not in (self.ANGLE, self.CROSS):
            raise ValueError(f"unknown law {law!r}")
        self.law = law
        #: Whether allocator saturation freezes the update. PX4's MC_SEAT_SATGATE, which
        #: defaults to 1 there because every archived PX4 run was flown gated.
        #:
        #: This does NOT break the level-1 archive. eigen_controller.py now sets `saturated`
        #: from its torque re-clip, but that clip is behind `torque_max`, which is None
        #: unless a run passes --controller-gains torque_max=..., and no archived level-1 run
        #: does - so the flag is never assigned there and this is inert either way.
        self.sat_gate = bool(sat_gate)
        self.saturated = False
        self.reset()

    def reset(self):
        self.theta_s = 0.0
        self.alpha = float("nan")

    def psi_model(self, r: float) -> float:
        """The lag rotation magnitude at spin rate r."""
        return r * self.tau_a + math.atan(r * self.T)

    def rotate(self, v):
        """Apply the rotation. Stateless per tick - no dt, no memory."""
        return R(self.theta_s) @ np.asarray(v, dtype=float)[:2]

    def update_fixed(self, r: float):
        """FIXED mode: recompute theta_s from the live spin rate. No measurement."""
        self.theta_s = float(np.clip(self.k_sign * self.psi_model(r),
                                     -self.theta_max, self.theta_max))
        self.alpha = float("nan")

    def adapt(self, alpha: float, r: float, dt: float, drive=None) -> bool:
        """Advance theta_s. Returns whether it stepped.

        Two laws, and CROSS is the default:

          ANGLE  theta_s -= k * alpha * dt.  alpha comes from atan2, which
                 normalises the magnitudes away - so the step is full-size on every
                 tick regardless of how little signal produced it. Measured: the
                 noise drive has sd 1.81 independent of signal strength.
          CROSS  theta_s -= k * (xd x xa) * dt = k |xd||xa| sin(alpha).  Near
                 convergence sin(a) ~ a, so it is the same gradient and converges
                 to the same angle; away from it the step is weighted by the signal
                 magnitudes, so a starved channel suppresses its own update.

        Why CROSS by default: on the vehicle the observed failure is a starved
        channel (hover, command:disturbance = 0.61) where theta_s random-walked with
        sd 0.206 and applied essentially arbitrary rotations. CROSS makes that
        structurally impossible - no threshold to tune - turning the seat from
        harmful-when-unexcited into inert-when-unexcited. It also removes the eps
        gate and the atan2 NaN path entirely.

        NOT a bias fix. Both drives are zero-mean under noise (measured over 2e6
        samples: atan2 mean +0.0004 +/- 0.0013); mean|alpha| = pi/2 is just the mean
        ABSOLUTE value of a uniform distribution, not a drift. The win is variance,
        not bias - and CROSS actually carries MORE bias, because its fixed point is
        where the time-average of |xd||xa| sin(alpha) vanishes rather than where
        alpha does. Measured in the harness: sd 8.7x better, drift 3x worse.

        CAUTION: k is DIMENSIONAL for CROSS. It absorbs the units of |xd||xa|, so
        the effective bandwidth scales with signal amplitude and k must be retuned
        if the operating amplitude changes. In the harness, k_cross ~ 0.01 matched
        k_angle = 2.0 - a factor of ~200 - and that factor is not portable.
        """
        if (self.sat_gate and self.saturated) or abs(r) <= self.r_min or not dt > 0.0:
            self.alpha = float("nan")
            return False

        if self.law == self.CROSS:
            if drive is None or not math.isfinite(drive):
                self.alpha = float("nan")
                return False
            step = drive
        else:
            if not math.isfinite(alpha):
                self.alpha = float("nan")
                return False
            step = alpha

        self.theta_s = float(np.clip(self.theta_s - self.k_sign * self.k * step * dt,
                                     -self.theta_max, self.theta_max))
        self.alpha = alpha
        return True

    def describe(self) -> str:
        name = {v: k for k, v in self.MODES.items()}[self.mode]
        if self.mode == self.OFF:
            return "seat=off (wrench passed through)"
        if self.mode == self.FIXED:
            return (f"seat=fixed tau_a={self.tau_a*1e3:.1f}ms T={self.T*1e3:.1f}ms "
                    f"thmax={self.theta_max:.2f} ksign={self.k_sign:+.0f}")
        return (f"seat=adaptive law={self.law} k={self.k:g} thmax={self.theta_max:.2f} "
                f"rmin={self.r_min:g} ksign={self.k_sign:+.0f}")


class WrenchObserver:
    """Rebuilds the wrench-induced acceleration from what a vehicle can measure.

        xa = zeta_dot_meas - A_model zeta

    zeta_dot_meas comes from a backward difference of the body rate, so it
    describes the interval [t-dt, t] and must be compared against the command from
    the tick that CAUSED it, not the current one. Skipping that alignment leaves a
    spurious r*dt in the converged angle - 0.08 rad at r=20 and 250 Hz, and it
    grows with spin rate.

    Note the difference is not divided by dt: atan2 is invariant to positive
    scaling of either vector, so dividing would only inject dt noise. The drift
    term is therefore scaled by dt to match.

    CONSEQUENCE, recorded 2026-09-19 after it cost a day: last_xa is NOT in rad/s^2
    and last_xd is. Every consumer this class has takes an ANGLE between them, which
    is invariant to that, so it has never mattered - but anything comparing their
    LENGTHS must divide last_xa by dt first. Related and more important: last_xa is a
    RECONSTRUCTION, w_dot_meas minus the MODELLED drift, and under spin the lam*r
    precession dominates that subtraction. A modest error in lam leaves a residual the
    same order as the delivered wrench, so |last_xa|/|last_xd| is NOT the actuator's
    magnitude response. Measured across 32 archived runs flown at rotor_tau = 0, where
    the true |Ga| is exactly 1.000: the ratio reads 0.044 to 1.225, and exceeds 1,
    which an attenuating actuator cannot do. The ANGLE survives this contamination;
    a magnitude would not.

    TWO MODES, and the choice matters more than any gain:

      ABSOLUTE     compare xd against xa directly. LEVELS, no differencing -
                   this is brief v3's observable, and it is the default. In the
                   disturbance-free case it is exact: theta_s converges to minus
                   the actuator's discrete phase. With disturbance it needs
                   commanded content >= ~100x that disturbance.
      INCREMENTAL  compare their increments. Rejects disturbances constant in the
                   body frame and needs only >= ~5x, but differencing amplifies
                   content at w by ~w*dt - which made it WORSE on the real vehicle,
                   whose dominant residual sits at ~4x the spin rate. v3 §3 forbids
                   differencing for this reason; kept only for comparison.
    """

    ABSOLUTE = "absolute"        #: compare xd against xa directly
    INCREMENTAL = "incremental"  #: compare their INCREMENTS (INDI-style)

    #: How the angular acceleration is obtained when the platform has no measured one.
    BACKWARD = "backward"   #: (w_k - w_{k-1})/dt, attributed to t-dt. Default.
    CENTERED = "centered"   #: (w_{k+1} - w_{k-1})/2dt, evaluated one tick late.

    def __init__(self, mode: str = ABSOLUTE, xdot_mode: str = BACKWARD,
                 eps_cmd: float = 1e-6, eps_resp: float = 1e-4):
        if mode not in (self.ABSOLUTE, self.INCREMENTAL):
            raise ValueError(f"unknown observer mode {mode!r}")
        self.mode = mode
        self.xdot_mode = xdot_mode
        self.eps_cmd = float(eps_cmd)
        self.eps_resp = float(eps_resp)
        self.reset()

    def reset(self):
        self._w_prev = None
        self._xd_prev = None
        self.last_xa = None      # the achieved-accel vector, for PoleAdapter
        self.last_xd = None      # the aligned commanded-accel vector
        self._dxd_prev = None
        self._xa_prev = None
        self._hist = []          # (w, xd) ring for the centered estimate

    def observe(self, xd, w_xy, drift_x, dt: float, xdot=None) -> float:
        """Signed misalignment [rad], or NaN when it is not measurable.

        Args:
            xd: commanded roll/pitch angular acceleration this tick [rad/s^2]
            w_xy: measured roll/pitch body rate (p, q) [rad/s]
            drift_x: the modelled drift A_model @ (p, q) [rad/s^2]
            dt: timestep [s]
            xdot: MEASURED roll/pitch angular acceleration [rad/s^2], when the
                platform provides one (PX4 publishes it as
                vehicle_angular_velocity.xyz_derivative). Strongly preferred in
                INCREMENTAL mode: without it the acceleration comes from a
                backward difference of the rate, and differencing THAT is a second
                difference of w - order dt^2 against a rate of order 1, which is
                mostly cancellation error. With it, incremental needs only one
                difference and is well conditioned.
        """
        xd = np.asarray(xd, dtype=float)[:2]
        w_xy = np.asarray(w_xy, dtype=float)[:2]
        if not (np.all(np.isfinite(xd)) and np.all(np.isfinite(w_xy))) or not dt > 0.0:
            return float("nan")

        if self._w_prev is None:
            self._w_prev = w_xy.copy()
            self._xd_prev = xd.copy()
            return float("nan")

        # Pick the acceleration estimate and, critically, the command and drift
        # samples that BELONG TO THE SAME INSTANT as it.
        #
        # A backward difference w_k - w_{k-1} is the acceleration over [t-dt, t],
        # i.e. the acceleration at t-dt - not at t. Subtracting drift_x taken at t
        # mixes two instants, and that error is 62% of |xa| at r=20, dt=4ms. Worse,
        # INCREMENTAL then differences xa, which shrinks the signal by ~r*dt while
        # leaving the misalignment error alone, pushing it to 79%. Aligning all
        # three - acceleration, command, drift - is what makes the differenced path
        # usable at all.
        if xdot is not None:
            xa = (np.asarray(xdot, dtype=float)[:2] - np.asarray(drift_x, dtype=float)[:2]) * dt
            xd_use = xd                                   # all at t
            self._w_prev = w_xy.copy()
        else:
            drift = np.asarray(drift_x, dtype=float)[:2]
            self._hist.append((w_xy.copy(), xd.copy(), drift.copy()))
            need = 3 if self.xdot_mode == self.CENTERED else 2
            if len(self._hist) < need:
                return float("nan")
            del self._hist[:-need]

            if self.xdot_mode == self.CENTERED:
                (w0, _, _), (_, xd1, dr1), (w2, _, _) = self._hist
                dw = (w2 - w0) * 0.5                      # centred on t1
            else:
                (w0, xd1, dr1), (w1, _, _) = self._hist
                dw = w1 - w0                              # spans [t0, t1], i.e. at t0
                xd1, dr1 = self._hist[0][1], self._hist[0][2]
            xa = dw - dr1 * dt
            xd_use = xd1

        self.last_xa = xa.copy()
        self.last_xd = np.asarray(xd_use, dtype=float).copy()

        if self.mode == self.INCREMENTAL:
            # INDI-style: difference both sides, so anything CONSTANT IN THE BODY
            # FRAME drops out - CoM offset, thrust asymmetry, a stopped rotor. It
            # does not reject a world-fixed disturbance, which sits at -r in the
            # body frame alongside the command, where differencing scales both by
            # r*dt equally.
            #
            # The angle survives: differencing content at frequency w multiplies it
            # by (e^{i w dt} - 1), the same complex factor on both sides, so it
            # cancels out of the angle between them.
            prev_xd, prev_xa = self._dxd_prev, self._xa_prev
            self._dxd_prev, self._xa_prev = np.asarray(xd_use).copy(), xa.copy()
            if prev_xd is None:
                return float("nan")
            xd_use, xa = xd_use - prev_xd, xa - prev_xa

        if np.hypot(*xd_use) < self.eps_cmd or np.hypot(*xa) < self.eps_resp * dt:
            return float("nan")
        return signed_angle(xd_use, xa)


class PoleAdapter:
    """Adapts theta_b, the pole angle inside M_dyn = -rho*R(theta_b). Brief v3 §4.

    Separate from the seat: it applies NO rotation to the wrench. It changes where
    the designed closed-loop pole sits, by moving M_dyn's angle.

        alpha_b = signed angle from (M_dyn . x) to xa
        theta_b += -k_b * alpha_b * dt

    NOTE, measured in seat_harness_v3.py: this alpha_b has NO interior fixed point.
    |alpha_b| stayed >= 1.12 however slow theta_b was made, so theta_b is driven
    monotonically to its clip and slowing it only reduces the damage - at
    k_s/k_b = 1000 the arm converges on "do nothing". Brief v3 §4 anticipates this
    and says the signal is what to redesign. Kept as specified so the vehicle result
    can be compared against the harness rather than pre-judged.

    theta_b is an OFFSET from the controller's configured pole angle, so a run with
    k_b = 0 reproduces the unmodified controller exactly.
    """

    GRADIENT = "gradient"   #: brief v3 s4. Ill-formed - see the note above.
    EXTREMUM = "extremum"   #: dithered gradient on the LATERAL TRACKING cost.

    def __init__(self, k_b: float = 0.2, theta_b_max: float = 1.4,
                 r_min: float = 5.0, eps: float = 1e-9, law: str = GRADIENT,
                 dither_amp: float = 0.12, dither_hz: float = 0.4,
                 k_es: float = 2.0, a_min: float = 0.3):
        self.k_b = max(float(k_b), 0.0)
        self.theta_b_max = float(np.clip(theta_b_max, 0.0, Seat.THETA_CEILING))
        self.r_min = max(float(r_min), 0.0)
        self.eps = float(eps)
        self.saturated = False
        if law not in (self.GRADIENT, self.EXTREMUM):
            raise ValueError(f"unknown pole law {law!r}")
        self.law = law
        self.dither_amp = max(float(dither_amp), 0.0)
        self.dither_w = 2.0 * math.pi * max(float(dither_hz), 0.0)
        self.k_es = max(float(k_es), 0.0)
        self.a_min = max(float(a_min), 0.0)
        self.reset()

    def reset(self):
        self.theta_b = 0.0        # offset APPLIED (includes the dither in EXTREMUM)
        self.theta_b_hat = 0.0    # the converged estimate, dither excluded
        self.alpha_b = float("nan")
        self.cost = float("nan")
        self._phase = 0.0
        self._cost_lp = float("nan")
        self._grad_lp = 0.0
        self._acmd_lp = None
        self._ames_lp = None

    def adapt_extremum(self, a_cmd, a_meas, r: float, dt: float) -> bool:
        """EXTREMUM law. Descends the lateral-tracking cost by a MEASURED gradient.

        Why this is not brief v3 s4 again, in one line each:

          EXOGENOUS REFERENCE. a_cmd is the position loop's commanded horizontal
          acceleration. It does not rotate when theta_b rotates, so it cannot
          cancel itself out of the cost the way s4's M_dyn.x reference does -
          which is the structural reason s4's alpha_b is strictly positive in the
          interior and zero only at +/-pi/2.

          MEASURED SIGN. theta_b is dithered and the cost demodulated against that
          dither, so d(cost)/d(theta_b) is estimated, sign included. Nothing here
          assumes which way b should move. s4 derived a direction and it pointed
          at the rail; at level 2 it drove theta_b NEGATIVE while the archived
          b_vs_tau sweep says the optimum is POSITIVE (+0.11 to +0.40 rad).

        SPIN-SYNCHRONOUS REJECTION is not optional. Measured at level 2, r=29.5:
        commanded |a| averaged 0.064 m/s^2 while ACHIEVED averaged 4.40 - 70x
        larger, and almost all of it a residual tilt sweeping round at the spin
        frequency. An angle between the raw pair measures the spin, not the
        tracking. Both vectors are therefore averaged over ~4 spin periods first:
        the spin-synchronous part integrates away over a revolution, the
        translational response does not.
        """
        a_cmd = np.asarray(a_cmd, dtype=float)[:2]
        a_meas = np.asarray(a_meas, dtype=float)[:2]
        if not (np.all(np.isfinite(a_cmd)) and np.all(np.isfinite(a_meas))) or not dt > 0.0:
            self.cost = float("nan")
            return False

        self._phase = (self._phase + self.dither_w * dt) % (2.0 * math.pi)
        dither = self.dither_amp * math.sin(self._phase)

        spin_T = 2.0 * math.pi / max(abs(r), 1e-3)
        a_lp = float(np.clip(dt / max(4.0 * spin_T, 0.2), 0.0, 1.0))
        if self._acmd_lp is None:
            self._acmd_lp, self._ames_lp = a_cmd.copy(), a_meas.copy()
        self._acmd_lp += (a_cmd - self._acmd_lp) * a_lp
        self._ames_lp += (a_meas - self._ames_lp) * a_lp

        self.cost = float("nan")
        excited = (np.hypot(*self._acmd_lp) > self.a_min
                   and np.hypot(*self._ames_lp) > self.a_min)
        stepped = False
        if (not self.saturated) and abs(r) > self.r_min and excited:
            J = abs(signed_angle(self._acmd_lp, self._ames_lp, self.eps))
            if math.isfinite(J):
                self.cost = J
                lp = float(np.clip(dt / 2.0, 0.0, 1.0))
                if not math.isfinite(self._cost_lp):
                    self._cost_lp = J
                self._cost_lp += lp * (J - self._cost_lp)
                grad = (J - self._cost_lp) * math.sin(self._phase)
                gl = float(np.clip(dt / 1.0, 0.0, 1.0))
                self._grad_lp += gl * (grad - self._grad_lp)
                self.theta_b_hat = float(np.clip(
                    self.theta_b_hat - self.k_es * self._grad_lp * dt,
                    -self.theta_b_max, self.theta_b_max))
                stepped = True

        self.theta_b = float(np.clip(self.theta_b_hat + dither,
                                     -self.theta_b_max, self.theta_b_max))
        return stepped

    def adapt(self, M_dyn_x, xa, r: float, dt: float) -> bool:
        if xa is None or not dt > 0.0 or abs(r) <= self.r_min:
            self.alpha_b = float("nan")
            return False
        ab = signed_angle(np.asarray(M_dyn_x, dtype=float)[:2],
                          np.asarray(xa, dtype=float)[:2], self.eps)
        if not math.isfinite(ab):
            self.alpha_b = float("nan")
            return False
        self.theta_b = float(np.clip(self.theta_b - self.k_b * ab * dt,
                                     -self.theta_b_max, self.theta_b_max))
        self.alpha_b = ab
        return True

    def describe(self) -> str:
        return (f"pole_adapter k_b={self.k_b:g} theta_b_max={self.theta_b_max:.2f} "
                f"(offset from the configured pole angle)")
