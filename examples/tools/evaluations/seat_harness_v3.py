#!/usr/bin/env python3
"""
seat_harness_v3.py — minimal existence proof for the two-integrator scheme.

Brief v3. Isotropic, body-torque level, no allocation, no disturbance. One
question:

    do theta_s (fast, lag aim) and theta_b (slow, pole angle) converge to a
    JOINT resting point, with theta_b absorbing the pole shift theta_s causes?

MODEL (complex-packed zeta = p + iq internally; S is multiplication by i)

    plant      zeta_dot = A x + tau_delivered/I_perp,   A = a*I + lam*r*S
    control    tau      = I_perp * ((M_dyn - A_r) x + w_cmd)
    seat       u        = R(theta_s) @ tau
    actuator   tau_del  = Ga[u],   Ga(s) = e^{-s tau_a}/(1+sT)

with M_dyn = rho*R(theta_b) and A_r = lam*r*S.

OBSERVABLES (§3, §4) - both LEVELS, no differencing anywhere:

    xd      = tau/I_perp                      computed lagless-expected accel
    xa      = wdot_meas - A x                 = tau_delivered/I_perp
    alpha   = signed angle xd -> xa           drives theta_s (fast)
    alpha_b = signed angle (M_dyn x) -> xa    drives theta_b (slow)

alpha is blind to M_dyn: it appears in xd and xa identically and cancels out of
the angle, leaving only the actuator's phase. That is the property the whole
scheme rests on, and `check_b_cancels_from_alpha` asserts it.

THREE DEVIATIONS FROM THE BRIEF AS WRITTEN, each flagged where it occurs:

  4. §1's "M_dyn = rho*R(theta_b)" contradicts its own "a_pole = -rho*cos(theta_b)".
     The second is Hurwitz (and matches control_doc.md V.7); the first puts the
     lagless closed-loop pole at +15.2+8.9j and everything diverges. -rho used.
  1. §3 writes wdot_expect = (A_r + M_dyn)x/I_perp while §2 forms
     tau = (M_dyn - A_r)x. Those are different vectors. For M_dyn to cancel out
     of alpha, xd must be THE SAME WRENCH xa measures, so xd is computed from
     §2's tau. `--xd-literal` reproduces §3's form; it does not converge.
  2. (M_dyn - A_r)x is already rad/s^2, so dividing tau_delivered by I_perp a
     second time (§2's plant line) scales the loop by 1/I_perp = 29x. tau is
     therefore formed as I_perp*(...)x. `--tau-literal` reproduces the literal
     reading; it diverges.
  3. §5 asks for "a held or slowly-varying desired direction". Held in the BODY
     frame is DC, and a lag does not ROTATE DC - arg Ga(0) = 0 - so there would
     be nothing for theta_s to find and psi_model would not be its target. The
     command is therefore world-fixed by default, which in the body frame
     rotates at -r, putting the content at the spin frequency the theory is
     written about. `--cmd-frame body` gives the DC case for comparison.

Run:  python3 examples/tools/evaluations/seat_harness_v3.py
      python3 examples/tools/evaluations/seat_harness_v3.py --plot v3.png
"""
from __future__ import annotations

import argparse
import math
from dataclasses import dataclass, replace

import numpy as np

S_MAT = np.array([[0.0, -1.0], [1.0, 0.0]])


def R(theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s], [s, c]])


def signed_angle(xd, xa, eps: float) -> float:
    """Signed angle from xd to xa [rad], NaN when either lacks a direction."""
    if not (np.all(np.isfinite(xd)) and np.all(np.isfinite(xa))):
        return float("nan")
    if np.hypot(*xd) < eps or np.hypot(*xa) < eps:
        return float("nan")
    return math.atan2(xd[0] * xa[1] - xd[1] * xa[0],
                      xd[0] * xa[0] + xd[1] * xa[1])


# ── configuration ───────────────────────────────────────────────────────────
@dataclass
class Config:
    # plant (isotropic: one scalar inertia, no allocation)
    a: float = -0.5           # [1/s]     aero damping, < 0
    I_perp: float = 0.0349    # [kg m^2]  roll = pitch (mean, when aniso != 0)
    # BEYOND BRIEF v3 §8, which mandates isotropy. Added to test the one assumption
    # §0 says licenses rotating the whole wrench at the output: with Ixx != Iyy the
    # torque->accel map is a non-uniform scaling, which does NOT commute with R(θ),
    # so "rotate the output" stops being equivalent to "rotate upstream".
    # Isaac: Ixx=0.034853, Iyy=0.031256 -> aniso = (Ixx-Iyy)/mean = 0.109
    aniso: float = 0.0

    # OUTER CASCADE. Beyond brief v3, which drives the rate channel directly. Isaac
    # runs position PD -> desired roll/pitch -> attitude PD -> RATE SETPOINT -> the
    # eigen inner loop, and the harness has no attitude state at all. With this on,
    # the inner loop tracks a rate setpoint produced by an attitude loop, which is
    # the structure the vehicle actually has:
    #     phi_dot   = x                          (small-angle attitude)
    #     x_ref     = Kp_att (phi_ref - phi)
    #     accel_cmd = M_dyn (x - x_ref) - A_r x
    # Does the observer get a MEASURED angular acceleration, or must it difference
    # the rate? v3 hands it the exact wdot_meas, which no vehicle has: Pegasus State
    # carries only the rate, and PX4's xyz_derivative is itself filtered. Differencing
    # is what Isaac actually does, so this is the honest setting for comparing to it.
    exact_wdot: bool = True

    # MATCHED low-pass on xd and xa, cutoff in rad/s (0 = off). Differencing the
    # rate is a high-pass, so it amplifies the 4r contaminant ~4x against the
    # command at r. A low-pass between r and 4r suppresses that - but on xa alone
    # it would inject its own phase and bias theta_s. Applying the IDENTICAL filter
    # to both leaves the ANGLE between them untouched for content at any one
    # frequency, while changing the relative weight of r vs 4r. Same argument as a
    # matched washout.
    obs_lp_wc: float = 0.0

    # Demodulating notch on xa at a KNOWN multiple of the spin rate. The 4r
    # contaminant is the four-rotor per-revolution forcing, so its frequency is
    # known exactly from r - no identification needed. Estimate its quadrature
    # components by demodulation and subtract; unlike a low-pass this removes 4r
    # without attenuating or phase-shifting the command at r.
    notch_harmonic: float = 0.0   # multiple of r to notch out (0 = off)
    notch_beta: float = 0.02      # demodulator low-pass coefficient

    outer_loop: bool = False
    Kp_att: float = 7.0        # matches the eigen controller's default
    phi_ref_amp: float = 0.10  # [rad] world-fixed attitude target -> rotates at -r
    I_z: float = 0.0627       # [kg m^2]  only to form lam
    r: float = 20.0           # [rad/s]   spin rate, constant per run

    # designed error dynamics, M_dyn = rho * R(theta_b)
    rho: float = 18.0         # [1/s] pole magnitude, fixed
    theta_b0: float = 0.515   # [rad] initial pole angle (the shipped b/wn = 10.2/18)

    # actuator, Ga = e^{-s tau_a}/(1+sT)
    T: float = 0.020          # [s]
    tau_a: float = 0.002      # [s]

    # integrators. k_s >= 10*k_b is mandatory (§4).
    k_s: float = 2.0          # [1/s] fast, theta_s
    k_b: float = 0.2          # [1/s] slow, theta_b
    theta_max: float = 1.4
    theta_b_max: float = 1.4
    r_min: float = 5.0
    eps: float = 1e-9

    # excitation (§5). No disturbance this pass.
    cmd_amp: float = 20.0     # [rad/s^2] commanded angular acceleration
    cmd_frame: str = "world"  # "world" -> rotates at -r in body; "body" -> DC

    seat_on: bool = True
    adapt_b: bool = False

    # Update law for theta_s.
    #   "angle" : theta_s -= k * alpha * dt              (brief v3 §4)
    #   "cross" : theta_s -= k * (xd x xa) * dt          = k*|xd||xa|*sin(alpha)
    # The cross form is the angle form weighted by |xd||xa|, so near convergence
    # (sin a ~ a) it is the same gradient with a signal-dependent gain. k therefore
    # has different units and needs rescaling by ~1/(|xd||xa|) to match.
    law: str = "angle"

    # Gyro measurement noise [rad/s], 1-sigma per axis. The control law and the
    # observer see x + noise; the plant integrates the true state. This is the
    # low-SNR case the cross form is meant to survive - v3 §8 excludes disturbance,
    # so it is off by default and used only to compare the two laws.
    noise_std: float = 0.0
    seed: int = 0

    # Disturbance. Beyond brief v3 §8, which excludes it - added ONLY to compare
    # the angle and cross update laws against the contaminant the vehicle actually
    # has. Measured in Isaac: dominant residual at ~4r (four-rotor per-revolution
    # forcing), with RMS command : RMS residual = 0.61.
    dist_amp: float = 0.0        # [rad/s^2]
    dist_harmonic: float = 4.0   # multiple of r

    dt: float = 0.004         # Isaac's rate
    t_end: float = 40.0

    # literal-brief reproductions, see the module docstring
    xd_literal: bool = False
    tau_literal: bool = False

    @property
    def lam(self) -> float:
        return (self.I_z - self.I_perp) / self.I_perp

    def I_vec(self) -> np.ndarray:
        """(Ixx, Iyy). Equal when aniso = 0, which is brief v3's isotropic case."""
        return self.I_perp * np.array([1.0 + self.aniso/2, 1.0 - self.aniso/2])

    def A(self) -> np.ndarray:
        return self.a * np.eye(2) + self.lam * self.r * S_MAT

    def A_r(self) -> np.ndarray:
        return self.lam * self.r * S_MAT

    def psi_model(self) -> float:
        """Lag rotation at the spin frequency (§1)."""
        return self.r * self.tau_a + math.atan(self.r * self.T)

    def psi_target(self) -> float:
        """Continuous-time target: -psi_model for a world-fixed command."""
        return -self.psi_model() if self.cmd_frame == "world" else 0.0

    def psi_target_discrete(self) -> float:
        """What theta_s can ACTUALLY reach: minus the DISCRETE actuator phase.

        psi_model = r*tau_a + atan(r*T) is the continuous-time phase, and the
        harness does not run a continuous actuator. Two gaps at dt = 4 ms:

          * the transport delay is a ring buffer of round(tau_a/dt) samples, so
            tau_a = 2 ms rounds to ZERO - the delay is simply absent;
          * the one-pole lag y += alpha_f*(u - y) has phase
            -arg(1 - (1-alpha_f) e^{-j w dt}), not atan(wT).

        At r = 13 that difference is 0.2288 vs psi_model's 0.2804, and theta_s
        converges to 0.2292 - the discrete value, to within 4e-4. Comparing against
        psi_model would read as a 20% error in a mechanism that is in fact exact.
        Make tau_a a multiple of dt if you want the delay to be honoured.
        """
        if self.cmd_frame != "world":
            return 0.0
        w = -self.r
        z_inv = np.exp(-1j * w * self.dt)
        H = 1.0 + 0j
        n_delay = int(round(self.tau_a / self.dt))
        H *= z_inv ** n_delay                       # transport delay
        if self.T > 0.0:
            af = 1.0 - math.exp(-self.dt / self.T)
            H *= af / (1.0 - (1.0 - af) * z_inv)    # one-pole lag
        return -float(np.angle(H))


# ── swappable pieces (§9) ───────────────────────────────────────────────────
class Actuator:
    """Ga(s) = e^{-s tau_a} / (1 + sT): ring-buffer delay then one-pole lag."""

    def __init__(self, T: float, tau_a: float, dt: float):
        self.T, self.dt = float(T), float(dt)
        self.n_delay = int(round(tau_a / dt)) if dt > 0 else 0
        self._q = [np.zeros(2) for _ in range(self.n_delay)]
        self._lp = np.zeros(2)

    def apply(self, u: np.ndarray, dt: float) -> np.ndarray:
        if self.n_delay:
            self._q.append(u.copy())
            u = self._q.pop(0)
        if self.T > 0.0:
            self._lp += (u - self._lp) * (1.0 - math.exp(-dt / self.T))
            return self._lp.copy()
        return u.copy()


class Observer:
    """alpha: signed angle from the commanded accel to the achieved accel.

    Both arguments are LEVELS. Differencing them would reintroduce the ~90 deg
    rate-vs-derivative offset the earlier observables died on, so it is not done
    anywhere in this file.
    """

    def __init__(self, eps: float):
        self.eps = eps

    def __call__(self, xd, xa) -> float:
        return signed_angle(xd, xa, self.eps)


class PoleObserver:
    """alpha_b: misalignment on the ERROR dynamics, driving theta_b.

    NOTE (brief §4): the right signal here is a design choice. This is the
    brief's starting suggestion, angle(M_dyn x -> xa). It differs from alpha only
    by whether A_r sits in the first argument, so the two are NOT independent -
    they share xa entirely. If theta_s and theta_b wander together, this is the
    thing to change, not the scheme. `wander_together` in the summary reports it.
    """

    def __init__(self, eps: float):
        self.eps = eps

    def __call__(self, M_dyn_x, xa) -> float:
        return signed_angle(M_dyn_x, xa, self.eps)


class Loop:
    """Ties plant, actuator, observers and the two integrators together."""

    def __init__(self, cfg: Config):
        self.cfg = cfg
        self.act = Actuator(cfg.T, cfg.tau_a, cfg.dt)
        self.obs = Observer(cfg.eps)
        self.pole_obs = PoleObserver(cfg.eps)
        self.theta_s = 0.0
        self.theta_b = cfg.theta_b0
        self.alpha = float("nan")
        self.alpha_b = float("nan")
        self._rng = np.random.default_rng(cfg.seed)
        self.phi = np.zeros(2)
        self._x_prev = None
        self._xd_prev = None
        self._Ax_prev = None
        self._xd_lp = np.zeros(2)
        self._xa_lp = np.zeros(2)
        self._nI = np.zeros(2)
        self._nQ = np.zeros(2)

    def command(self, t: float) -> np.ndarray:
        """Commanded angular acceleration (§5). No disturbance this pass."""
        if self.cfg.cmd_amp == 0.0:
            return np.zeros(2)
        w = -self.cfg.r if self.cfg.cmd_frame == "world" else 0.0
        return self.cfg.cmd_amp * np.array([math.cos(w * t), math.sin(w * t)])

    def step(self, x_true: np.ndarray, t: float) -> tuple:
        cfg = self.cfg
        # Everything the controller and observer see is the MEASURED state.
        x = x_true + (self._rng.normal(0.0, cfg.noise_std, 2)
                      if cfg.noise_std > 0.0 else 0.0)
        # DEVIATION 4: brief §1 writes M_dyn = rho*R(theta_b), but its own
        # parenthetical gives a_pole = -rho*cos(theta_b) - i.e. the NEGATIVE form,
        # which is also control_doc.md V.7's. With +rho the lagless closed loop is
        # a*I + M_dyn, whose pole sits at a + rho*e^{i theta_b} = +15.2 + 8.9j at
        # the shipped rho and theta_b: unstable, and every arm diverges. The
        # parenthetical is the Hurwitz reading, so -rho it is.
        M_dyn = -cfg.rho * R(self.theta_b)
        A, A_r = cfg.A(), cfg.A_r()

        if cfg.outer_loop:
            w = -cfg.r
            phi_ref = cfg.phi_ref_amp * np.array([math.cos(w * t), math.sin(w * t)])
            x_ref = cfg.Kp_att * (phi_ref - self.phi)
            accel_cmd = M_dyn @ (x - x_ref) - A_r @ x
        else:
            accel_cmd = (M_dyn - A_r) @ x + self.command(t)
        # DEVIATION 2: I_perp makes tau a torque, so the plant's /I_perp closes.
        Iv = cfg.I_vec()
        tau = accel_cmd if cfg.tau_literal else Iv * accel_cmd

        # FRAME-FIX DEFERRED ---------------------------------------------------
        # The whole wrench is rotated, so theta_s rotates the FEEDBACK term
        # M_dyn.x as well as the gyroscopic feedforward. That is what shifts the
        # closed-loop poles and what adaptive theta_b exists to absorb. The clean
        # fix is to rotate only the feedforward path; deferred by brief v3 §8.
        u = R(self.theta_s) @ tau if cfg.seat_on else tau
        # ----------------------------------------------------------------------

        tau_del = self.act.apply(u, cfg.dt)
        d = np.zeros(2)
        if cfg.dist_amp != 0.0:
            wh = cfg.dist_harmonic * (-cfg.r)
            d = cfg.dist_amp * np.array([math.cos(wh * t), math.sin(wh * t)])
        wdot_true = A @ x_true + tau_del / Iv + d
        wdot_meas = A @ x + tau_del / Iv + d      # what the observer sees

        # DEVIATION 1: xd is the wrench xa measures, so M_dyn cancels from alpha.
        xd = ((A_r + M_dyn) @ x / Iv) if cfg.xd_literal else (tau / Iv)
        if cfg.exact_wdot:
            xa = wdot_meas - A @ x                        # = tau_del / I
        else:
            # Backward difference of the rate. dw spans [t-dt, t], so it is the
            # acceleration at t-dt and must be paired with xd and A.x from that
            # same tick - the alignment lesson from the vehicle-side observer.
            if self._x_prev is None:
                self._x_prev, self._xd_prev, self._Ax_prev = x.copy(), xd.copy(), (A @ x)
                return wdot_true, xd, np.zeros(2)
            dw = (x - self._x_prev) / cfg.dt
            xa = dw - self._Ax_prev
            xd_now = xd.copy()          # store THIS tick's xd, not the aligned one
            xd = self._xd_prev
            self._x_prev, self._xd_prev, self._Ax_prev = x.copy(), xd_now, (A @ x)

        if cfg.obs_lp_wc > 0.0:
            bq = 1.0 - math.exp(-cfg.dt * cfg.obs_lp_wc)
            self._xd_lp = self._xd_lp + (xd - self._xd_lp) * bq
            self._xa_lp = self._xa_lp + (xa - self._xa_lp) * bq
            xd, xa = self._xd_lp.copy(), self._xa_lp.copy()

        if cfg.notch_harmonic > 0.0:
            wn_ = cfg.notch_harmonic * (-cfg.r)
            cn, sn = math.cos(wn_ * t), math.sin(wn_ * t)
            b2 = cfg.notch_beta
            self._nI = self._nI + b2 * (xa * cn - self._nI)
            self._nQ = self._nQ + b2 * (xa * sn - self._nQ)
            xa = xa - 2.0 * (self._nI * cn + self._nQ * sn)

        gated = abs(cfg.r) < cfg.r_min
        self.alpha = float("nan") if gated else self.obs(xd, xa)
        self.alpha_b = float("nan") if gated else self.pole_obs(M_dyn @ x, xa)

        if cfg.seat_on:
            if cfg.law == "cross":
                # Raw cross product: |xd||xa| sin(alpha). No atan2, so the
                # magnitudes are never normalised away, and a quiet channel takes
                # proportionally smaller steps. Also needs no eps gate.
                drive = float(xd[0] * xa[1] - xd[1] * xa[0])
                if math.isfinite(drive):
                    self.theta_s = float(np.clip(self.theta_s - cfg.k_s * drive * cfg.dt,
                                                 -cfg.theta_max, cfg.theta_max))
            elif math.isfinite(self.alpha):
                self.theta_s = float(np.clip(self.theta_s - cfg.k_s * self.alpha * cfg.dt,
                                             -cfg.theta_max, cfg.theta_max))
        if cfg.adapt_b and math.isfinite(self.alpha_b):
            self.theta_b = float(np.clip(self.theta_b - cfg.k_b * self.alpha_b * cfg.dt,
                                         -cfg.theta_b_max, cfg.theta_b_max))

        if cfg.outer_loop:
            self.phi = self.phi + x * cfg.dt
        return wdot_true, xd, xa


def run(cfg: Config, x0=(0.05, 0.0)) -> dict:
    assert cfg.k_s >= 10.0 * cfg.k_b - 1e-12, "brief §4: k_s must be >= 10*k_b"
    loop = Loop(cfg)
    n = int(round(cfg.t_end / cfg.dt))
    log = {k: np.full(n, np.nan) for k in
           ("t", "theta_s", "theta_b", "alpha", "alpha_b", "x_mag", "att_err")}
    log["x"] = np.full((n, 2), np.nan)
    x = np.array(x0, dtype=float)

    for i in range(n):
        t = i * cfg.dt
        wdot, xd, xa = loop.step(x, t)
        log["t"][i] = t
        log["theta_s"][i] = loop.theta_s
        log["theta_b"][i] = loop.theta_b
        log["alpha"][i] = loop.alpha
        log["alpha_b"][i] = loop.alpha_b
        log["x_mag"][i] = np.linalg.norm(x)
        log["x"][i] = x
        log["att_err"][i] = np.nan   # filled by tracking_error() against a zero-lag run
        x = x + wdot * cfg.dt
        if not np.all(np.isfinite(x)) or np.linalg.norm(x) > 1e7:
            for k in log:
                log[k] = log[k][:i + 1]
            log["diverged"] = True
            return log
    log["diverged"] = False
    return log


def tracking_error(cfg: Config, log: dict) -> float:
    """RMS vector deviation of x from the SAME arm flown with a perfect actuator.

    This is the metric that matters. |alpha| is not: theta_s drives alpha to zero
    by construction, so reporting it says only that the integrator worked, not
    that the vehicle tracked better.
    """
    # The reference is the DESIGNED loop: perfect actuator, no seat, nominal b.
    # Using a zero-lag run of the same ARM would let the +b reference drift its own
    # poles, making that arm's score partly self-referential.
    ideal = run(replace(cfg, T=0.0, tau_a=0.0, seat_on=False, adapt_b=False))["x"]
    xa_ = log["x"]
    n = min(len(ideal), len(xa_))
    sl = slice(int(n * 0.6), n)
    d = xa_[sl] - ideal[sl]
    v = np.sqrt(np.nanmean(np.sum(d * d, axis=1)))
    return float(v) if np.isfinite(v) else float("inf")


def settled(log, frac=0.4):
    n = len(log["t"])
    return slice(int(n * (1 - frac)), n)


def empirical_gain(log) -> float:
    """d(alpha)/d(theta_s), smoothed. Diagnostic only (§6) - never fed back."""
    th, al, t = log["theta_s"], log["alpha"], log["t"]
    m = np.isfinite(th) & np.isfinite(al)
    th, al = th[m], al[m]
    if th.size < 200 or t.size < 2:
        return float("nan")
    w = max(1, int(round(0.5 / float(t[1] - t[0]))))
    if th.size <= w + 10:
        return float("nan")
    k = np.ones(w) / w
    ths, als = np.convolve(th, k, "valid"), np.convolve(al, k, "valid")
    pk = int(np.argmax(np.abs(als)))
    ths, als = ths[pk:], als[pk:]
    if als.size < 20:
        return float("nan")
    dth, dal = np.diff(ths), np.diff(als)
    good = (np.abs(als[:-1]) > 0.05 * np.max(np.abs(als))) & (np.abs(dth) > 1e-12)
    if good.sum() < 10:
        return float("nan")
    return float(np.linalg.lstsq(dth[good, None], dal[good], rcond=None)[0][0])


# ── arms and acceptance (§7) ────────────────────────────────────────────────
ARMS = (("baseline",   dict(seat_on=False, adapt_b=False)),
        ("theta_s only", dict(seat_on=True,  adapt_b=False)),
        ("theta_s + b",  dict(seat_on=True,  adapt_b=True)))

FAILURES: list[str] = []


def check(cond, msg):
    print(f"  {'ok   ' if cond else 'FAIL '} {msg}")
    if not cond:
        FAILURES.append(msg)


def summarize(cfg: Config, log: dict) -> dict:
    sl = settled(log)
    al = log["alpha"][sl]
    al = al[np.isfinite(al)]
    return dict(
        diverged=log["diverged"],
        theta_s=float(np.nanmean(log["theta_s"][sl])),
        theta_s_sd=float(np.nanstd(log["theta_s"][sl])),
        theta_b=float(np.nanmean(log["theta_b"][sl])),
        theta_b_sd=float(np.nanstd(log["theta_b"][sl])),
        alpha=float(np.mean(np.abs(al))) if al.size else float("nan"),
        att_err=tracking_error(cfg, log),
        x_mag=float(np.sqrt(np.nanmean(log["x_mag"][sl] ** 2))),
        gain=empirical_gain(log),
    )


def check_b_cancels_from_alpha(cfg: Config):
    """§3: alpha must not see theta_b. Sweep theta_b, alpha must not move."""
    print("\n1  b cancels out of alpha (the property the split rests on)")
    vals = []
    for tb in (0.0, 0.3, 0.515, 0.8):
        c = replace(cfg, theta_b0=tb, adapt_b=False, seat_on=False, t_end=6.0)
        log = run(c)
        al = log["alpha"][settled(log)]
        al = al[np.isfinite(al)]
        vals.append(float(np.mean(al)) if al.size else float("nan"))
        print(f"  info  theta_b={tb:.3f} -> mean alpha {vals[-1]:+.5f}")
    spread = np.nanmax(vals) - np.nanmin(vals)
    check(spread < 0.02, f"alpha is blind to theta_b (spread {spread:.5f} rad over 0.8 rad of b)")


def run_arms(cfg: Config, label: str):
    print(f"\n{label}")
    print(f"  {'arm':<14}{'att_err':>9}{'|alpha|':>9}{'theta_s':>9}{'(sd)':>8}"
          f"{'theta_b':>9}{'(sd)':>8}{'|x|':>9}{'gain':>7}")
    out = {}
    for name, over in ARMS:
        c = replace(cfg, **over)
        s = summarize(c, run(c))
        out[name] = s
        flag = "  DIVERGED" if s["diverged"] else ""
        print(f"  {name:<14}{s['att_err']:9.4f}{s['alpha']:9.4f}{s['theta_s']:9.4f}"
              f"{s['theta_s_sd']:8.4f}{s['theta_b']:9.4f}{s['theta_b_sd']:8.4f}"
              f"{s['x_mag']:9.4f}{s['gain']:7.2f}{flag}")
    print(f"  psi_target: continuous {cfg.psi_target():+.4f} | "
          f"DISCRETE {cfg.psi_target_discrete():+.4f} rad  (the reachable one)")
    return out


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--plot", metavar="PATH", default=None)
    p.add_argument("--dt", type=float, default=0.004)
    p.add_argument("--cmd-frame", choices=("world", "body"), default="world")
    p.add_argument("--xd-literal", action="store_true", help="brief §3's (A_r+M_dyn) form")
    p.add_argument("--tau-literal", action="store_true", help="brief §2's tau without I_perp")
    a = p.parse_args()

    cfg = Config(dt=a.dt, cmd_frame=a.cmd_frame,
                 xd_literal=a.xd_literal, tau_literal=a.tau_literal)
    print(f"v3 harness - isotropic, no disturbance, cmd_frame={cfg.cmd_frame!r}, "
          f"dt={cfg.dt*1e3:.0f} ms")
    print(f"  lam = (I_z - I_perp)/I_perp = {cfg.lam:+.3f}   k_s/k_b = {cfg.k_s/cfg.k_b:.0f}"
          f"   rho = {cfg.rho}   T = {cfg.T*1e3:.0f} ms   tau_a = {cfg.tau_a*1e3:.0f} ms")

    check_b_cancels_from_alpha(cfg)

    print("\n2  three arms, spin-rate sweep (§7)")
    per_r = {}
    for r in (13.0, 20.0, 30.0):
        per_r[r] = run_arms(replace(cfg, r=r), f"  --- r = {r:.0f} rad/s ---")

    print("\n3  acceptance (§7)")
    for r, arms in per_r.items():
        base, only, both = arms["baseline"], arms["theta_s only"], arms["theta_s + b"]
        tgt = replace(cfg, r=r).psi_target_discrete()
        print(f"  r={r:.0f}:")
        check(abs(only["theta_s"] - tgt) < 0.02,
              f"    r={r:.0f}: theta_s reaches the discrete target ({only['theta_s']:+.4f} vs {tgt:+.4f})")
        # does adaptive b move theta_b away from its start to absorb the shift?
        moved = abs(both["theta_b"] - cfg.theta_b0)
        print(f"         theta_b moved {moved:+.4f} rad from its start ({cfg.theta_b0})")
        # joint resting point: both integrators quiet in the settled window
        quiet = both["theta_s_sd"] < 0.05 and both["theta_b_sd"] < 0.05
        check(quiet, f"    r={r:.0f}: theta_s and theta_b settle "
                     f"(sd {both['theta_s_sd']:.4f}, {both['theta_b_sd']:.4f})")
        if not quiet:
            print("         ^ brief §7.4: widen k_s/k_b before changing anything else")
        # wander-together diagnostic (§4 note)
        print(f"         att_err  baseline {base['att_err']:.4f} | "
              f"theta_s only {only['att_err']:.4f} | theta_s+b {both['att_err']:.4f}")
        check(both["att_err"] <= base["att_err"] + 1e-9,
              f"    r={r:.0f}: theta_s+b improves on baseline")

    if a.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(2, 1, figsize=(8, 7), sharex=True, constrained_layout=True)
        c = replace(cfg, r=20.0, seat_on=True, adapt_b=True)
        log = run(c)
        axes[0].axhline(c.psi_target(), color="0.6", ls="--", label=r"$\psi_{target}$")
        axes[0].plot(log["t"], log["theta_s"], label=r"$\theta_s$ (fast)")
        axes[0].plot(log["t"], log["theta_b"], label=r"$\theta_b$ (slow)")
        axes[0].axhline(cfg.theta_b0, color="0.8", ls=":", label=r"$\theta_b$ initial")
        axes[0].set_ylabel("angle [rad]"); axes[0].legend(); axes[0].grid(alpha=.3)
        axes[0].set_title(f"joint convergence, r={c.r:.0f} rad/s, "
                          f"$k_s/k_b$={c.k_s/c.k_b:.0f}")
        axes[1].plot(log["t"], log["alpha"], label=r"$\alpha$ (drives $\theta_s$)")
        axes[1].plot(log["t"], log["alpha_b"], label=r"$\alpha_b$ (drives $\theta_b$)")
        axes[1].set_xlabel("t [s]"); axes[1].set_ylabel("[rad]")
        axes[1].legend(); axes[1].grid(alpha=.3)
        fig.savefig(a.plot, dpi=140)
        print(f"\nplot written to {a.plot}")

    print()
    if FAILURES:
        print(f"{len(FAILURES)} FAILED:")
        for f in FAILURES:
            print(f"  - {f}")
        return 1
    print("all checks passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
