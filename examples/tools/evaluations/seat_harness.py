#!/usr/bin/env python3
"""
seat_harness.py — the adaptive command-frame rotation ("seat"), in the reduced
                  complex-plane model, with no Isaac and no PX4.

WHAT THIS IS FOR
────────────────
Implementation brief v2 asks one question and nothing else:

    does theta_s converge to psi_model = r*tau_a + atan(r*T) ?

If it does, the seat mechanism works and PX4 integration is worth writing. If it
rails or lands far from psi_model, it does not — and we want to know that here,
in a script that runs in a second, rather than after a flight.

THE MODEL (brief §6)
────────────────────
Roll/pitch packed as 2-vectors, x = (p, q). S is multiplication by i:

    S = [[0, -1], [1, 0]],    R(t) = cos(t)*I + sin(t)*S

    plant     xdot = (a*I + lam*r*S) @ x + (1/I_perp) * tau_delivered  +  d(t)
    actuator  tau_delivered = Ga * u,   Ga = e^{-s*tau_a} / (1 + s*T)
    command   u = R(theta_s) @ wrench,  wrench = (A_r + M_dyn) @ x
                                        A_r = lam*r*S

WHY THE OBSERVABLE ISOLATES LAG (brief §2)
──────────────────────────────────────────
    xd = wdot_expect = (1/I_perp) * wrench      # what a LAGLESS actuator would give
    xa = wdot_meas                              # what actually happened
    alpha = signed angle from xd to xa

Both are built from the same M_dyn, so whatever b* does appears in both vectors
identically and cancels out of the ANGLE between them. What is left is the
actuator's rotation.

The property that matters is d(alpha)/d(theta_s) = 1: it is what makes "reduce
alpha" a well-posed instruction. An observable that does not have it has no
theta_s that zeroes alpha, and the integrator rails instead of converging. §6.7
logs the empirical value so the assumption stays checkable.

    theta_s += -k * alpha * dt,   clipped to +/- theta_max

is gradient descent on J = alpha^2/2 under d(alpha)/d(theta_s) = 1. This pass
TRUSTS that unit gain (brief §8) and only *logs* the empirical value (§6.7).

THE ONE OPEN QUESTION THIS HARNESS SETTLES FIRST
────────────────────────────────────────────────
xd is the accel due to the COMMANDED WRENCH ALONE, while xa is the TOTAL measured
accel — which in the plant above also carries (a*I + lam*r*S)x and the disturbance.
Those extra terms do not vanish at zero lag, so taken literally the angle between
xd and xa is not zero when there is nothing to compensate.

Rather than quietly pick a reading, both are implemented and selectable:

    OBSERVABLE_TOTAL   xa = xdot                          (brief §2, literal; default)
    OBSERVABLE_WRENCH  xa = (1/I_perp) * tau_delivered    (like-for-like)

and `test_zero_lag_nulls` asserts alpha == 0 at T = tau_a = 0 for whichever is
selected. That test is the arbiter, and it is why the observable is a swappable
function (brief §9) rather than three lines inline.

EXCITATION
──────────
The reduced loop has no reference input and a stable M_dyn, so x decays to zero
and the observable starves (|xd|, |xa| under the epsilon gate). A spinning
vehicle physically sees a disturbance that rotates in the body frame at -r, which
is what `disturbance` supplies. Set its amplitude to 0 for a free-decay run.

Run:  python3 examples/tools/evaluations/seat_harness.py            # tests + sweep
      python3 examples/tools/evaluations/seat_harness.py --plot out.png
"""
from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass, field, replace
from pathlib import Path

import numpy as np

# The seat itself lives in examples/utils/seat.py and is imported rather than
# copied, so the harness validates the same code the vehicle flies.
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "utils"))
from seat import R, Seat, WrenchObserver, signed_angle   # noqa: E402

S_MAT = np.array([[0.0, -1.0], [1.0, 0.0]])


# ── configuration ───────────────────────────────────────────────────────────
@dataclass
class Config:
    # plant
    a: float = -0.5          # [1/s]   rate damping
    lam: float = -0.91       # [-]     gyroscopic coefficient, (Iyy-Izz)/Ixx on the Iris
    lam_model: float | None = None  # what the FEEDFORWARD believes lam is; None = exact.
    #                          Brief §4: lam rotates wdot_expect, so a wrong one biases
    #                          theta_s like a lag would. It is the only parameter the
    #                          observable depends on for DIRECTION.
    I_perp: float = 0.0349   # [kg m^2]
    r: float = 30.0          # [rad/s] spin rate

    # designed error dynamics, M_dyn = -rho * R(theta_pole)   (control_doc.md V.7)
    rho: float = 18.0        # [1/s] pole magnitude
    theta_pole: float = 0.515  # [rad] pole angle; atan(10.2/18), the shipped MC_EIG_B

    # actuator, Ga = e^{-s*tau_a} / (1 + s*T)
    T: float = 0.020         # [s] first-order lag (rotor/ESC), ROTOR_TAU_PLACEHOLDER
    tau_a: float = 0.002     # [s] transport delay

    # seat
    k: float = 2.0           # [1/s] adaptation gain
    theta_max: float = 1.4   # [rad] < pi/2
    r_min: float = 5.0       # [rad/s] spin gate
    eps: float = 1e-9        # magnitude gate for atan2

    # Excitation: a disturbance rotating at dist_dir*r in the body frame. -1 is the
    # physical case - a body frame spinning at +r sees a world-fixed direction going
    # round at -r - and it is what sets the SIGN of the angle the seat must apply.
    dist_amp: float = 2.0    # [rad/s^2]
    dist_dir: float = -1.0

    # Commanded angular acceleration, added to the NDI output. This is the vehicle
    # being TOLD to do something, as opposed to merely rejecting d.
    #
    # The distinction decides whether the flyable observable works at all. A
    # disturbance enters the plant AFTER the actuator, so it lands in xa and not in
    # xd and does not cancel. A command enters through tau_cmd, so it appears in
    # both and cancels exactly like M and A do. In a purely disturbance-driven loop
    # the contaminant is the same size as the signal no matter how small d is - the
    # loop is linear, so scaling d scales the wrench with it (measured ratio 1.2,
    # flat across two decades of dist_amp).
    cmd_amp: float = 0.0     # [rad/s^2]

    # Where the disturbance is fixed. "world" rotates at dist_dir*r in the body
    # frame (wind, translational drag); "body" is constant in the body frame (CoM
    # offset, thrust asymmetry, a failed rotor). The distinction decides whether an
    # INDI-style incremental observable can reject it: differencing kills anything
    # slow in the BODY frame, and a world-fixed disturbance is not slow there - it
    # sits at exactly the spin frequency the seat works at.
    dist_frame: str = "world"      # "world" | "body" | "harmonic"
    dist_harmonic: float = 4.0     # multiple of r, for dist_frame="harmonic"

    # integration
    dt: float = 0.001        # [s]
    t_end: float = 20.0      # [s]

    seat_on: bool = True

    # DIAGNOSTIC ONLY - brief v2 §8 excludes the split from the implementation.
    #
    # False: the seat rotates the whole wrench (M_dyn - drift)x, which is what the
    # brief specifies and what is measured to be net harmful everywhere.
    # True: rotate only the gyroscopic feedforward -drift*x, leaving the designed
    # error dynamics M_dyn*x untouched. This exists to TEST the claim that the harm
    # comes from rotating the feedback term, rather than to assert it.
    seat_split: bool = False

    # Whether the platform supplies a MEASURED angular acceleration. True models
    # PX4 (vehicle_angular_velocity.xyz_derivative); False models Pegasus level 1,
    # whose State carries only the rate, so the observer must difference it.
    measured_xdot: bool = True
    xdot_mode: str = "centered"   # when measured_xdot is False
    observable: str = "wrench"  # "wrench" (isolates lag) or "total" (brief §2 literal)

    # Brief §2 writes wrench = (A_r + M_dyn)x with A_r = +lam*r*S, and
    # wdot_expect = wrench/I_perp. Taken literally that is dimensionally and
    # sign-wise inconsistent with the §6 plant, and the loop diverges:
    #
    #   UNITS  (A_r + M_dyn)x is [1/s][rad/s] = rad/s^2 already, so dividing by
    #          I_perp again scales it by 1/0.0349 = 28.7 - and the plant divides
    #          tau_delivered by I_perp a second time. Closed-loop |lambda| lands
    #          near 1150 rad/s, which a 20 ms lag cannot hold.
    #   SIGN   the feedforward must CANCEL the plant's gyroscopic term, so it
    #          needs -lam*r*S. With +lam*r*S it doubles it. PX4's own
    #          EigenController uses M_eff = M - A_r for exactly this reason, and
    #          its comments record the same sign bug in the prototype.
    #
    # Default is therefore the consistent NDI form, wrench = I_perp*(M_dyn - drift)x,
    # for which the zero-lag closed loop is exactly xdot = M_dyn x. Set this True to
    # reproduce the literal reading and watch it diverge.
    ndi_literal: bool = False

    def M_dyn(self) -> np.ndarray:
        return -self.rho * R(self.theta_pole)

    def A_r(self) -> np.ndarray:
        return self.lam * self.r * S_MAT

    def drift(self) -> np.ndarray:
        """The plant's own rate dynamics."""
        return self.a * np.eye(2) + self.lam * self.r * S_MAT

    def drift_model(self) -> np.ndarray:
        """What the feedforward BELIEVES the plant's rate dynamics are."""
        lam = self.lam if self.lam_model is None else self.lam_model
        return self.a * np.eye(2) + lam * self.r * S_MAT

    def psi_model(self) -> float:
        """Lag rotation magnitude, brief §3: psi = r*tau_a + atan(r*T)."""
        return self.r * self.tau_a + math.atan(self.r * self.T)

    def psi_target(self) -> float:
        """The SIGNED angle the seat should converge to.

        The command content sits at the excitation frequency w = dist_dir*r, and
        Ga(jw) has phase -(w*tau_a + atan(w*T)). Cancelling it means
        theta_s = -phase = dist_dir * psi_model, since atan is odd. Brief §3 writes
        the magnitude; this carries the sign that goes with the rotation direction,
        which is the thing that flips if the frame handedness does.
        """
        return self.dist_dir * self.psi_model()


# ── actuator: Ga = e^{-s*tau_a} / (1 + s*T) ─────────────────────────────────
class Actuator:
    """Transport delay in series with a first-order lag. Linear, so order is free.

    The lag uses 1 - exp(-dt/T) rather than the bilinear dt/(dt+T): the
    exponential preserves the pole's time constant exactly for a fixed step and
    does not drift if the step is ever varied.
    """

    def __init__(self, T: float, tau_a: float, dt: float):
        self.T = float(T)
        self.dt = float(dt)
        self.n_delay = int(round(tau_a / dt)) if dt > 0 else 0
        self.tau_a_effective = self.n_delay * dt
        self._queue = [np.zeros(2) for _ in range(self.n_delay)]
        self._lp = np.zeros(2)

    def reset(self):
        self._queue = [np.zeros(2) for _ in range(self.n_delay)]
        self._lp = np.zeros(2)

    def step(self, u: np.ndarray) -> np.ndarray:
        if self.n_delay > 0:
            self._queue.append(u.copy())
            delayed = self._queue.pop(0)
        else:
            delayed = u
        if self.T > 0.0:
            beta = 1.0 - math.exp(-self.dt / self.T)
            self._lp += (delayed - self._lp) * beta
            return self._lp.copy()
        return delayed.copy()


# ── seat ────────────────────────────────────────────────────────────────────
def make_seat(cfg: "Config") -> Seat:
    """The shared Seat (examples/utils/seat.py), configured from the harness."""
    return Seat(mode=Seat.ADAPTIVE, k=cfg.k, theta_max=cfg.theta_max,
                r_min=cfg.r_min, eps=cfg.eps, tau_a=cfg.tau_a, T=cfg.T)


# ── observables (swappable, brief §9) ───────────────────────────────────────
def observable_total(sig, eps):
    """Brief §2, literal: measured TOTAL angular acceleration. Does not null."""
    return _angle_gated(sig["xd"], sig["xdot"], eps)


def observable_wrench(sig, eps):
    """Like-for-like, but NOT FLYABLE.

    Uses tau_delivered/J directly. Nulls exactly, and it is what proves the
    mechanism - but no vehicle can measure the wrench-induced part of its own
    acceleration in isolation. Kept as the idealised reference.
    """
    return _angle_gated(sig["xd"], sig["wrench_accel"], eps)


def _angle_gated(xd, xa, eps):
    """Signed angle from xd to xa, gated on both having a usable direction."""
    if np.linalg.norm(xd) < eps or np.linalg.norm(xa) < eps:
        return float("nan")
    return signed_angle(xd, xa)


def observable_measured(sig, eps):
    """WrenchObserver in ABSOLUTE mode - the flyable path, un-differenced.

    Delegates to examples/utils/seat.py rather than reimplementing, so the harness
    validates the code the vehicle runs. The observer takes the RATE and forms its
    own backward difference, exactly as it does on the vehicle.
    """
    return sig["obs_absolute"].observe(sig["xd"], sig["x"], sig["drift_x"], sig["dt"],
                                       xdot=sig["xdot_meas"] if sig["use_xdot"] else None)


def observable_incremental(sig, eps):
    """WrenchObserver in INCREMENTAL mode - INDI-style, also flyable.

    Differences both sides, so any disturbance constant in the body frame cancels.
    Same delegation as above.
    """
    return sig["obs_incremental"].observe(sig["xd"], sig["x"], sig["drift_x"], sig["dt"],
                                          xdot=sig["xdot_meas"] if sig["use_xdot"] else None)


OBSERVABLES = {"total": observable_total, "wrench": observable_wrench,
               "measured": observable_measured,
               "incremental": observable_incremental}


# ── simulation ──────────────────────────────────────────────────────────────
def simulate(cfg: Config, x0=(0.05, 0.0)) -> dict:
    """Integrate the reduced closed loop. Returns per-tick logs."""
    obs = OBSERVABLES[cfg.observable]
    obs_absolute = WrenchObserver(mode=WrenchObserver.ABSOLUTE, xdot_mode=cfg.xdot_mode)
    obs_incremental = WrenchObserver(mode=WrenchObserver.INCREMENTAL, xdot_mode=cfg.xdot_mode)
    act = Actuator(cfg.T, cfg.tau_a, cfg.dt)
    seat = make_seat(cfg)

    M_dyn, A_r = cfg.M_dyn(), cfg.A_r()
    drift = cfg.drift()                 # the plant's truth
    # Feedback-linearizing map from state to COMMANDED ANGULAR ACCELERATION, built
    # from what the controller BELIEVES the plant is. With lam_model = None the two
    # agree and the zero-lag closed loop is exactly xdot = M_dyn x.
    ndi = A_r + M_dyn if cfg.ndi_literal else M_dyn - cfg.drift_model()
    psi = cfg.psi_target()

    n = int(round(cfg.t_end / cfg.dt))
    x = np.array(x0, dtype=float)
    log = {key: np.full(n, np.nan) for key in
           ("t", "theta_s", "alpha", "x_mag", "adapted")}
    # The state vector itself, so a trajectory can be compared against a reference
    # component-wise. |x| alone hides a phase change that leaves the magnitude alone.
    log["x"] = np.full((n, 2), np.nan)
    log["psi_target"] = np.full(n, psi)

    for i in range(n):
        t = i * cfg.dt

        # What the controller asks for, as a TORQUE, and the acceleration a lagless
        # actuator would produce from it. I_perp cancels between the two by
        # construction, which is what makes brief §4's claim exactly true: a wrong
        # I_perp cannot bias an angle.
        w_exc_c = cfg.dist_dir * cfg.r
        w_ref = cfg.cmd_amp * np.array([math.cos(w_exc_c * t), math.sin(w_exc_c * t)])
        accel_cmd = (ndi @ x) + w_ref
        wrench = accel_cmd if cfg.ndi_literal else (cfg.I_perp * accel_cmd)
        # For the split diagnostic, keep the two halves separable.
        fb_part = cfg.I_perp * (M_dyn @ x)
        ff_part = wrench - fb_part
        wdot_expect = wrench / cfg.I_perp

        # the seat, then the actuator
        if not cfg.seat_on:
            u = wrench
        elif cfg.seat_split:
            u = fb_part + seat.rotate(ff_part)      # feedback left alone
        else:
            u = seat.rotate(wrench)
        tau_delivered = act.step(u)

        # plant. The disturbance rotates at -r in the body frame, which is what a
        # body-fixed frame sees of anything fixed in the world while spinning.
        w_exc = cfg.dist_dir * cfg.r
        if cfg.dist_frame == "harmonic":
            # Measured in Isaac: the dominant roll/pitch residual sits at ~4r, the
            # four-rotor per-revolution forcing of a same-direction rotor set. It is
            # neither body-fixed (DC) nor world-fixed (-r), and differencing
            # AMPLIFIES it relative to the command by roughly (4r)/r.
            wh = cfg.dist_harmonic * w_exc
            d = cfg.dist_amp * np.array([math.cos(wh * t), math.sin(wh * t)])
        elif cfg.dist_frame == "body":
            d = cfg.dist_amp * np.array([1.0, 0.0])       # constant in the body frame
        else:
            d = cfg.dist_amp * np.array([math.cos(w_exc * t), math.sin(w_exc * t)])
        wrench_accel = tau_delivered / cfg.I_perp
        xdot = drift @ x + wrench_accel + d

        drift_x = cfg.drift_model() @ x
        alpha = obs({"xd": wdot_expect, "xdot": xdot, "wrench_accel": wrench_accel,
                     "drift_x": drift_x, "x": x, "dt": cfg.dt,
                     "obs_absolute": obs_absolute,
                     "obs_incremental": obs_incremental,
                     "xdot_meas": xdot, "use_xdot": cfg.measured_xdot}, cfg.eps)
        adapted = seat.adapt(alpha, cfg.r, cfg.dt) if cfg.seat_on else False

        log["t"][i] = t
        log["theta_s"][i] = seat.theta_s
        log["alpha"][i] = alpha
        log["x_mag"][i] = np.linalg.norm(x)
        log["x"][i] = x
        log["adapted"][i] = float(adapted)

        x = x + xdot * cfg.dt

        if not np.all(np.isfinite(x)) or np.linalg.norm(x) > 1e6:
            for key in log:
                log[key] = log[key][:i + 1]
            log["diverged"] = True
            return log

    log["diverged"] = False
    return log


def settled(log: dict, frac: float = 0.5):
    """Mask for the last `frac` of the run, where transients are gone."""
    n = len(log["t"])
    return slice(int(n * (1.0 - frac)), n)


def empirical_gain(log: dict) -> float:
    """Least-squares d(alpha)/d(theta_s) over the adaptation transient.

    DIAGNOSTIC ONLY (brief §6.7 and §8): the law assumes this is 1 and this says
    whether that held. It is deliberately not fed back into the law.

    Both series are smoothed over a window much longer than the excitation period
    before differencing. Without that the estimate is meaningless: per tick,
    alpha's oscillation at the excitation frequency is ~3x its slow trend
    (std(d_alpha) = 5.2e-4 vs std(d_theta) = 1.7e-4 at r = 30), so a raw
    regression fits noise uncorrelated with d_theta and reads ~0 whatever the true
    gain is. The window also has to start after alpha's initial rise, while the
    actuator's delay line is still filling.
    """
    th, al, t = log["theta_s"], log["alpha"], log["t"]
    m = np.isfinite(th) & np.isfinite(al)
    th, al = th[m], al[m]
    if th.size < 200 or t.size < 2:
        return float("nan")

    dt = float(t[1] - t[0])
    w = max(1, int(round(0.5 / dt)))          # >> 2*pi/r for every r swept here
    if th.size <= w + 10:
        return float("nan")
    kern = np.ones(w) / w
    ths = np.convolve(th, kern, mode="valid")
    als = np.convolve(al, kern, mode="valid")

    # From alpha's peak onward, and only while it still has something to say.
    peak = int(np.argmax(np.abs(als)))
    ths, als = ths[peak:], als[peak:]
    if als.size < 20:
        return float("nan")

    dth, dal = np.diff(ths), np.diff(als)
    keep = np.abs(als[:-1]) > 0.05 * np.max(np.abs(als))
    good = keep & (np.abs(dth) > 1e-12) & np.isfinite(dth) & np.isfinite(dal)
    if good.sum() < 10:
        return float("nan")
    return float(np.linalg.lstsq(dth[good, None], dal[good], rcond=None)[0][0])


# ── checks ──────────────────────────────────────────────────────────────────
FAILURES: list[str] = []


def check(cond: bool, msg: str):
    print(f"  {'ok   ' if cond else 'FAIL '} {msg}")
    if not cond:
        FAILURES.append(msg)


def test_zero_lag_nulls(cfg: Config):
    """THE ARBITER for which observable is right.

    With no lag there is nothing to compensate, so with theta_s pinned at 0 the
    measured response must sit exactly on the expected one and alpha must be 0.
    An observable that fails this is not isolating lag, whatever else it does.
    """
    print("\n1  zero lag => alpha == 0  (decides the observable)")
    for name in OBSERVABLES:
        c = replace(cfg, T=0.0, tau_a=0.0, observable=name, seat_on=False)
        log = simulate(c)
        a = log["alpha"][settled(log)]
        a = a[np.isfinite(a)]
        worst = np.max(np.abs(a)) if a.size else float("nan")
        ok = a.size > 0 and worst < 1e-9
        print(f"  {'ok   ' if ok else 'note '} observable={name!r}: "
              f"max|alpha| = {worst:.3e} rad at zero lag")
        if name == cfg.observable and not ok:
            FAILURES.append(f"observable {name!r} does not null at zero lag")


def test_convergence(cfg: Config):
    """Brief §3: the pass/fail of the whole exercise."""
    print("\n2  theta_s -> psi_model, alpha -> 0")
    log = simulate(cfg)
    check(not log["diverged"], "the closed loop stayed bounded")
    if log["diverged"]:
        return log

    sl = settled(log)
    th = np.nanmean(log["theta_s"][sl])
    psi = cfg.psi_target()
    al = log["alpha"][sl]
    al = al[np.isfinite(al)]
    mean_abs_alpha = np.mean(np.abs(al)) if al.size else float("nan")

    check(abs(th - psi) < 0.05,
          f"theta_s -> psi_target: {th:+.4f} vs {psi:+.4f} rad (err {th - psi:+.4f})")
    check(mean_abs_alpha < 0.05,
          f"alpha -> 0: mean|alpha| = {mean_abs_alpha:.4f} rad")
    check(abs(th) < cfg.theta_max - 1e-6,
          f"theta_s is not on its rail (|{th:.4f}| < {cfg.theta_max})")

    g = empirical_gain(log)
    print(f"  info  empirical d(alpha)/d(theta_s) = {g:.3f}  "
          f"(law assumes 1.0; logged only, per brief §6.7/§8)")
    # Not acted on (brief §8 forbids feeding it back), but asserted: an observable
    # whose derivative is not ~1 has no fixed point, and this is where that shows up.
    check(np.isfinite(g) and abs(g - 1.0) < 0.2,
          f"the unit-gain assumption holds here ({g:.3f} vs 1.0)")
    return log


def test_decay_rate(cfg: Config):
    """alpha should decay ~ exp(-k t), so k is the adaptation bandwidth."""
    print("\n3  the decay rate follows k")
    prev = None
    for k in (1.0, 2.0, 4.0):
        log = simulate(replace(cfg, k=k))
        th = log["theta_s"]
        psi = cfg.psi_target()
        # time to cover 63% of the way to psi
        target = 0.632 * psi
        idx = np.argmax(np.abs(th) >= abs(target)) if np.any(np.abs(th) >= abs(target)) else -1
        tau_meas = log["t"][idx] if idx > 0 else float("nan")
        print(f"  info  k={k:>4}: time to 63% of psi = {tau_meas:.3f} s "
              f"(1/k = {1.0 / k:.3f} s)")
        if prev is not None and np.isfinite(tau_meas) and np.isfinite(prev):
            check(tau_meas < prev, f"k={k} converges faster than k={k / 2}")
        prev = tau_meas


def test_gates(cfg: Config):
    """Brief §5."""
    print("\n4  gates and safeguards")
    below = simulate(replace(cfg, r=1.0, r_min=5.0, t_end=5.0))
    check(np.allclose(below["theta_s"], 0.0),
          "below r_min the angle is held (never adapts)")

    off = simulate(replace(cfg, seat_on=False, t_end=5.0))
    check(np.allclose(off["theta_s"], 0.0), "seat off leaves theta_s at 0")

    tight = simulate(replace(cfg, theta_max=0.05))
    check(np.max(np.abs(tight["theta_s"])) <= 0.05 + 1e-12,
          "theta_s is clamped to theta_max")

    s = make_seat(cfg)
    s.theta_s = 0.7
    s.saturated = True
    s.adapt(0.5, cfg.r, cfg.dt)
    check(s.theta_s == 0.7, "allocator saturation freezes the adaptation")
    s.saturated = False
    s.adapt(0.5, cfg.r, cfg.dt)
    check(s.theta_s != 0.7, "and releasing it resumes")
    s.reset()
    check(s.theta_s == 0.0, "reset zeroes theta_s")


def test_inertia_does_not_bias(cfg: Config):
    """Brief §4: I_perp scales magnitude only, so it cannot bias an ANGLE."""
    print("\n5  I_perp does not bias theta_s, lam does")
    ref = simulate(cfg)
    base = float(np.nanmean(ref["theta_s"][settled(ref)]))
    for scale in (0.5, 2.0):
        th = simulate(replace(cfg, I_perp=cfg.I_perp * scale))
        v = np.nanmean(th["theta_s"][settled(th)])
        check(abs(v - base) < 0.02,
              f"I_perp x{scale}: theta_s {v:+.4f} vs {base:+.4f} (angle is scale-free)")

    # Brief §4 expects a wrong feedforward lam to rotate wdot_expect and so bias
    # theta_s. That holds for the 'total' observable. It does NOT hold for the
    # 'wrench' one, and the reason is the same reason b* cancels: xd and xa are both
    # built from the SAME commanded wrench, so alpha is the angle from that wrench to
    # its own delivered version - the actuator phase, and nothing else. Whatever lam
    # does to the wrench appears identically in both vectors.
    #
    # What remains is indirect: a wrong lam changes the closed loop, hence the
    # spectrum of x, hence the frequency at which Ga's phase is evaluated. Reported
    # rather than asserted, because the size of that is a property of the loop, not
    # of the observable.
    for scale in (0.5, 1.5):
        lg = simulate(replace(cfg, lam_model=cfg.lam * scale))
        v = float(np.nanmean(lg["theta_s"][settled(lg)]))
        print(f"  info  feedforward lam x{scale}: theta_s {v:+.4f} vs {base:+.4f} "
              f"(shift {v - base:+.4f} rad, indirect via the closed-loop spectrum)")
    print("  note  lam does NOT rotate this observable directly - it cancels between")
    print("        xd and xa exactly as b* does. That is stronger than brief §4 assumed.")


def run_sweep(cfg: Config, spins=(8.0, 13.0, 20.0, 30.0, 45.0, 60.0)):
    """Brief §4/§6.5: theta_s(r) vs psi_model(r) separates lag from lam-error.

    Clean lag gives a curve through the origin (slope ~tau_a plus the atan bend).
    A nonzero intercept as r -> 0 is lam-leakage, not lag.
    """
    print("\n6  spin-rate sweep: theta_s(r) vs psi_target(r)")
    print(f"  {'r':>7} {'psi_target':>12} {'theta_s':>10} {'gap':>9} "
          f"{'mean|alpha|':>12} {'d(al)/d(th)':>12}")
    rows = []
    for r in spins:
        c = replace(cfg, r=r, r_min=min(cfg.r_min, r * 0.5))
        log = simulate(c)
        sl = settled(log)
        th = float(np.nanmean(log["theta_s"][sl]))
        al = log["alpha"][sl]
        al = al[np.isfinite(al)]
        ma = float(np.mean(np.abs(al))) if al.size else float("nan")
        psi = c.psi_target()
        g = empirical_gain(log)
        flag = "  DIVERGED" if log["diverged"] else ""
        print(f"  {r:7.1f} {psi:12.4f} {th:10.4f} {th - psi:+9.4f} "
              f"{ma:12.4f} {g:12.3f}{flag}")
        rows.append((r, psi, th, ma, g, log["diverged"]))
    low = [(r_, abs(t - p)) for r_, p, t, _, _, dv in rows if not dv and r_ <= 30.0]
    high = [(r_, abs(t - p)) for r_, p, t, _, _, dv in rows if not dv and r_ > 30.0]
    if low:
        check(max(g for _, g in low) < 0.05,
              f"theta_s tracks psi_target for r <= 30 (worst gap "
              f"{max(g for _, g in low):.4f} rad)")
    if high:
        # Brief §7 predicted exactly this: because the seat rotates the whole wrench
        # it also rotates the feedback term M_dyn*x, so part of what it nulls is
        # pole distortion it is itself causing. theta_s then settles SHORT of psi and
        # the gap opens with r. This is the readout that says when to split the seat.
        print(f"  note  gap opens above r=30: " +
              ",  ".join(f"r={r_:.0f} -> {g:+.3f} rad" for r_, g in high))
        print("        that is brief §7's whole-wrench coupling becoming visible,")
        print("        not a failure of the adaptation - alpha stops nulling there.")
    return rows


def run_baseline(cfg: Config):
    """Brief §6.6: quantify the seat against a real seat-off arm."""
    print("\n7  seat off vs seat on")
    print(f"  {'r':>7} {'|x| off':>10} {'|x| on':>10} {'change':>9}")
    for r in (13.0, 30.0, 60.0):
        c = replace(cfg, r=r, r_min=min(cfg.r_min, r * 0.5))
        off = simulate(replace(c, seat_on=False))
        on = simulate(c)
        so, sn = settled(off), settled(on)
        xo = float(np.sqrt(np.nanmean(off["x_mag"][so] ** 2)))
        xn = float(np.sqrt(np.nanmean(on["x_mag"][sn] ** 2)))
        ch = "n/a" if not np.isfinite(xo) or xo == 0 else f"{100 * (xn - xo) / xo:+.1f}%"
        note = ""
        if off["diverged"]:
            note += "  (off DIVERGED)"
        if on["diverged"]:
            note += "  (on DIVERGED)"
        print(f"  {r:7.1f} {xo:10.4f} {xn:10.4f} {ch:>9}{note}")
    print("  note  |x| is the regulation error in this rate-only reduced model;")
    print("        there is no separate attitude state to form an att_err from.")


def make_plot(cfg: Config, rows, path: str):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    r = np.array([x[0] for x in rows])
    psi = np.array([x[1] for x in rows])
    th = np.array([x[2] for x in rows])
    fine = np.linspace(0, r.max() * 1.05, 200)
    # The SIGNED target: the command content sits at dist_dir*r, so the angle that
    # cancels Ga's phase is dist_dir*psi_model. Plotting the unsigned psi_model here
    # would show two mirrored curves and read as a failure when it is a convention.
    psi_fine = cfg.dist_dir * (fine * cfg.tau_a + np.arctan(fine * cfg.T))

    fig, ax = plt.subplots(figsize=(7.5, 4.5), constrained_layout=True)
    ax.axhline(0, color="0.7", lw=0.8)
    ax.plot(fine, psi_fine, "-", lw=1.5,
            label=r"$\psi_{target}=\mathrm{dir}\cdot(r\tau_a+\arctan(rT))$")
    ax.plot(r, th, "o--", lw=1.2, ms=6, label=r"converged $\theta_s$")
    good = r <= 30.0
    if np.any(~good):
        ax.plot(r[~good], th[~good], "o", ms=11, mfc="none", mec="crimson", mew=1.6,
                label=r"whole-wrench coupling dominates (brief §7)")
    ax.set_xlabel("spin rate r [rad/s]")
    ax.set_ylabel("angle [rad]")
    ax.set_title(f"seat convergence  (T={cfg.T*1e3:.0f} ms, "
                 r"$\tau_a$=" f"{cfg.tau_a*1e3:.0f} ms, k={cfg.k}, "
                 f"observable={cfg.observable!r})")
    ax.grid(alpha=0.3)
    ax.legend()
    fig.savefig(path, dpi=140)
    print(f"\nplot written to {path}")


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--observable", choices=tuple(OBSERVABLES), default="wrench",
                   help="which observable to run the checks with. 'wrench' nulls at "
                        "zero lag and is the default; 'total' is the brief's literal "
                        "reading and does not null - test 1 shows both either way.")
    p.add_argument("--ndi-literal", action="store_true",
                   help="use brief §2's literal wrench = (A_r + M_dyn)x. Diverges; "
                        "see the Config comment for the units and sign reasons.")
    p.add_argument("--plot", metavar="PATH", default=None,
                   help="write the theta_s(r) vs psi_model(r) plot here")
    p.add_argument("--r", type=float, default=30.0, help="spin rate for the single-point tests")
    a = p.parse_args()

    cfg = Config(observable=a.observable, r=a.r, ndi_literal=a.ndi_literal)
    print(f"seat harness - observable={a.observable!r}  r={cfg.r} rad/s  "
          f"T={cfg.T*1e3:.0f} ms  tau_a={cfg.tau_a*1e3:.0f} ms  "
          f"psi_model={cfg.psi_model():+.4f} rad")

    test_zero_lag_nulls(cfg)
    test_convergence(cfg)
    test_decay_rate(cfg)
    test_gates(cfg)
    test_inertia_does_not_bias(cfg)
    rows = run_sweep(cfg)
    run_baseline(cfg)

    if a.plot:
        make_plot(cfg, rows, a.plot)

    print()
    if FAILURES:
        print(f"{len(FAILURES)} FAILED:")
        for f in FAILURES:
            print(f"  - {f}")
        return 1
    print("all checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
