#!/usr/bin/env python3
"""Measure the external-vision (mocap) delay from the DYNAMICS, not from timestamp fields.

    python3 ev_delay_from_dynamics.py <log.ulg>

Needs `vehicle_visual_odometry` (the raw EV stream) and `vehicle_angular_velocity` in the
log. NOTE the estimator_aid_src_ev_* topics are NOT a substitute: they log at 2 Hz, which
cannot resolve a 10 ms delay.

THE IDEA
--------
A mocap sample stamped t actually describes the vehicle at t - tau. So the attitude INCREMENT
mocap reports between two of its samples must equal the increment the gyro integrates over
the same interval shifted back by tau. Scan tau; keep the tau that makes them agree.

Sensitivity comes from angular ACCELERATION:
    d/dtau  integral_{t1-tau}^{t2-tau} w dt  =  w(t1-tau) - w(t2-tau)
so an interval over which the rate does not change carries no information about tau at all.
Those intervals are dropped (|dw| > 0.02 rad/s).

WHY THIS WORKS WHERE DIFFERENCING QUATERNIONS FAILED
----------------------------------------------------
The earlier attempt (HARDWARE_READINESS.md 2b, attempt 3) finite-differenced consecutive EV
samples to get a body rate and got physically impossible NEGATIVE delays on two axes. In
ev_delay.ulg 30 % of EV intervals are gaps > 12 ms and the contiguous runs have a MEDIAN
LENGTH OF 2 SAMPLES, so it was differentiating across holes. Here nothing is differentiated
on the mocap side: a gap is not a hole, it is a longer baseline, and the attitude increment
across 200 ms is better conditioned than across 8 ms.

The gyro is integrated ONCE into a cumulative attitude Q(t). Q(a)^-1 Q(b) is exactly the
increment over [a,b], so gyro bias accumulated before `a` cancels, and a trial tau costs a
lookup rather than a re-integration.

HOW THE ANSWER IS EXTRACTED
---------------------------
Not by reading off the scan minimum - near zero the cost surface is shallow and the estimate
is compressed (measured gain ~0.3 below 10 ms). Instead a NULL measurement: inject known
shifts into the EV timestamps, find the injection at which the estimator reads zero, and
negate it. Validated by injection - recovered = 0.918 x injected + offset, R^2 0.99 over
0-80 ms.

TWO CORRECTIONS THAT ARE NOT OPTIONAL
-------------------------------------
1. vehicle_angular_velocity is low-passed by a 2-POLE LowPassFilter2p at IMU_GYRO_CUTOFF
   (PX4 sensors/vehicle_angular_velocity). Its group delay sqrt(2)/(2*pi*fc) is 5.63 ms at
   40 Hz and is ADDED, because the scan measures (tau_EV - tau_gyro). Using the 1-pole
   formula here understates the answer by 1.6 ms.
2. The result is the delay relative to the message timestamp, so it is what EKF2_EV_DELAY
   should be set to - not a residual on top of it, unless EKF2_EV_DELAY was already nonzero
   when the log was taken (it is reported).
"""
import sys
import numpy as np
from pyulog import ULog

# ---- quaternion helpers (w, x, y, z) ---------------------------------------------------
def qmul(a, b):
    w1, x1, y1, z1 = a[..., 0], a[..., 1], a[..., 2], a[..., 3]
    w2, x2, y2, z2 = b[..., 0], b[..., 1], b[..., 2], b[..., 3]
    return np.stack([w1*w2 - x1*x2 - y1*y2 - z1*z2,
                     w1*x2 + x1*w2 + y1*z2 - z1*y2,
                     w1*y2 - x1*z2 + y1*w2 + z1*x2,
                     w1*z2 + x1*y2 - y1*x2 + z1*w2], -1)

def qconj(q):
    o = q.copy(); o[..., 1:] *= -1; return o

def qn(q):
    return q / np.linalg.norm(q, axis=-1, keepdims=True)

def rotvec(q):
    q = np.where(q[..., :1] < 0, -q, q)
    v = q[..., 1:]
    s = np.linalg.norm(v, axis=-1, keepdims=True)
    ang = 2 * np.arctan2(s[..., 0], q[..., 0])
    return np.where(s > 1e-12, v / np.maximum(s, 1e-30) * ang[..., None], 2 * v)

# ---- load / integrate ------------------------------------------------------------------
def load(path):
    u = ULog(path, ['vehicle_visual_odometry', 'vehicle_angular_velocity'])
    d = {m.name: m.data for m in u.data_list}
    if 'vehicle_visual_odometry' not in d:
        raise SystemExit("no vehicle_visual_odometry in this log - the raw EV stream is not "
                         "logged, and estimator_aid_src_ev_* at 2 Hz cannot substitute.")
    ev = d['vehicle_visual_odometry']
    tev = np.asarray(ev['timestamp_sample'], float) / 1e6
    if np.ptp(tev) < 1:
        tev = np.asarray(ev['timestamp'], float) / 1e6
    qev = qn(np.vstack([ev['q[%d]' % i] for i in range(4)]).T.astype(float))
    rc = np.asarray(ev.get('reset_counter', np.zeros(len(tev))), int)
    av = d['vehicle_angular_velocity']
    tg = np.asarray(av['timestamp'], float) / 1e6
    w = np.vstack([av['xyz[%d]' % i] for i in range(3)]).T.astype(float)
    k = np.argsort(tg)
    return tev, qev, rc, tg[k], w[k], u.initial_parameters

def integrate(tg, w):
    dt = np.diff(tg); wm = 0.5 * (w[:-1] + w[1:])
    n = np.linalg.norm(wm, axis=1); th = n * dt
    ax = np.where(n[:, None] > 1e-12, wm / np.maximum(n, 1e-30)[:, None], 0.0)
    dq = np.concatenate([np.cos(th / 2)[:, None], ax * np.sin(th / 2)[:, None]], 1)
    Q = np.empty((len(tg), 4)); Q[0] = [1, 0, 0, 0]
    for i in range(len(dt)):
        Q[i + 1] = qmul(Q[i], dq[i])
    return qn(Q)

def qat(tg, Q, t):
    i = np.clip(np.searchsorted(tg, t) - 1, 0, len(tg) - 2)
    f = ((t - tg[i]) / (tg[i + 1] - tg[i]))[:, None]
    a, b = Q[i], Q[i + 1]
    dot = np.sum(a * b, 1, keepdims=True)
    b = np.where(dot < 0, -b, b); dot = np.abs(dot)
    om = np.arccos(np.clip(dot, -1, 1)); so = np.sin(om)
    return qn(np.where(om < 1e-6, a * (1 - f) + b * f,
                       a * np.sin((1 - f) * om) / np.where(so == 0, 1, so)
                       + b * np.sin(f * om) / np.where(so == 0, 1, so)))

# ---- estimator -------------------------------------------------------------------------
TAUS = np.arange(-0.040, 0.061, 0.0005)
SHIFTS = np.array([-0.015, -0.010, -0.005, 0.0, 0.005])

def surfaces(tev, qev, rc, tg, Q, w, shift):
    t = tev + shift
    dt = np.diff(t)
    ok = (dt > 0.004) & (dt < 0.30) & (np.diff(rc) == 0)
    ok &= (t[:-1] > tg[0] + 0.4) & (t[1:] < tg[-1] - 0.4)
    t1, t2 = t[:-1][ok], t[1:][ok]
    dev = rotvec(qmul(qconj(qev[:-1][ok]), qev[1:][ok]))
    dg0 = rotvec(qmul(qconj(qat(tg, Q, t1)), qat(tg, Q, t2)))
    wi = lambda x: np.stack([np.interp(x, tg, w[:, j]) for j in range(3)], -1)
    dw = wi(t1) - wi(t2)
    r0 = np.linalg.norm(dev - dg0, axis=1)
    keep = (r0 < 3 * np.median(r0) * 1.4826 / 0.6745) & (np.linalg.norm(dw, axis=1) > 0.02)
    t1, t2, dev = t1[keep], t2[keep], dev[keep]
    E = np.empty((len(TAUS), len(t1)))
    for i, tt in enumerate(TAUS):
        dq = qmul(qconj(qat(tg, Q, t1 - tt)), qat(tg, Q, t2 - tt))
        E[i] = np.sum((rotvec(dq) - dev) ** 2, axis=1)
    return E

def null_crossing(P, sel=None):
    """the injection that nulls the estimator is s0 = -b/g; the log's own delay is -s0"""
    rec = np.array([TAUS[int(np.argmin(E.mean(1) if sel is None else E[:, sel].mean(1)))]
                    for E in P])
    A = np.vstack([SHIFTS, np.ones_like(SHIFTS)]).T
    g, b = np.linalg.lstsq(A, rec, rcond=None)[0]
    return b / g if g != 0 else np.nan

def main(path, nboot=300):
    tev, qev, rc, tg, w, par = load(path)
    print("EV   n=%d  %.1f Hz  span %.1f s" % (len(tev), 1 / np.median(np.diff(tev)), tev[-1] - tev[0]))
    print("gyro n=%d  %.1f Hz  |w| rms %.3f rad/s"
          % (len(tg), 1 / np.median(np.diff(tg)), np.sqrt((w ** 2).sum(1).mean())))
    gaps = np.diff(tev)
    print("EV gaps > 12 ms: %.0f%% of intervals - this is what killed the differencing method"
          % (100 * (gaps > 0.012).mean()))
    Q = integrate(tg, w)
    P = [surfaces(tev, qev, rc, tg, Q, w, s) for s in SHIFTS]
    pt = null_crossing(P)
    rng = np.random.default_rng(3)
    nmin = min(E.shape[1] for E in P)
    bs = np.array([null_crossing(P, rng.integers(0, nmin, nmin)) for _ in range(nboot)])
    bs = bs[np.isfinite(bs)]
    fc = par.get('IMU_GYRO_CUTOFF', 40.0)
    Tg = np.sqrt(2) / (2 * np.pi * fc)          # 2-pole LowPassFilter2p group delay
    lo, hi = np.percentile(bs, [2.5, 97.5])
    print("\nrelative to the filtered gyro: %+.1f ms  (95%% %.1f .. %.1f, n=%d boot)"
          % (1000 * pt, 1000 * lo, 1000 * hi, len(bs)))
    print("gyro 2-pole LPF at %.0f Hz adds %.2f ms" % (fc, 1000 * Tg))
    print("\n  EV DELAY = %.1f ms    95%% CI %.1f .. %.1f ms" % (1000 * (pt + Tg), 1000 * (lo + Tg), 1000 * (hi + Tg)))
    print("  EKF2_EV_DELAY in this log was %.1f ms" % par.get('EKF2_EV_DELAY', 0.0))

if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "/home/ani/Downloads/logs/ev_delay.ulg")
