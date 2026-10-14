#!/usr/bin/env python3
"""Seat observable diagnostics on a seat-off eigen log: why arg(xa/xd) does not read the lag
rotation, and whether a lock-in (world-frame, low-passed) observable does.
SPIN_RAMP_STEPS.md 6a; write-up in notes/SEAT_OBSERVABLE_ANALYSIS.md.

    python3 seat_observable.py <log.ulg> [<log.ulg> ...]
        [--seg NAME=T0-T1[+T2-T3...]] ...   log-relative seconds; applied to every log given
        [--izz X ...]                       extra MC_EIG_IZZ values for the drift (sensitivity)
        [--fig-dir DIR]                     write the three figures (all logs on one figure)

Signals, matched to EigenController.cpp tick by tick (checked against the logged seat_alpha,
debug[6] |xd| and debug[7] |xa| to float precision):
    xd = TRQ_MAX * (torque_sp - torque_trim) / (Ixx, Iyy)       pre-seat, seat and lead off
    xa = xyz_derivative - drift, drift = (Iyy-Izz)/Ixx r q, (Izz-Ixx)/Iyy r p   (MC_EIG_ALPHA 0)
paired by timestamp_sample. Both as complex x + i*y in FRD body axes (x roll, y pitch).

SIGN CONVENTION. Body frequency w is signed: content e^{iwt}. With FRD yaw positive (x turns
toward y) a world-fixed vector seen from the body turns at w = -r. A delay L multiplies content
at w by e^{-iwL}, so arg(xa conj xd) = -wL - atan(wT) + filter phase, which at w = -r is
+r*L + atan(r*T) + |filter phase(r)| = +psi (the lag rotation, positive for r > 0).
The seat's measure() is atan2(xd x xa, xd . xa) = arg(xa conj xd): the same angle.

The logged ticks are ~1 in 3 of the ~1 kHz control ticks (not anti-aliased). Sums and content
below 10 Hz are unaffected.
"""
import argparse
import os
import sys
import numpy as np
from pyulog import ULog

GAP = 0.020           # s, split contiguous pieces at gaps longer than this
FS_U = 300.0          # Hz, uniform grid for spectra
TOPICS = ['vehicle_angular_velocity', 'vehicle_torque_setpoint', 'vehicle_attitude',
          'vehicle_land_detected', 'mc_controller_status', 'manual_control_setpoint',
          'control_allocator_status']


# ----------------------------------------------------------------------------- loading
def load(path):
    u = ULog(path, TOPICS)
    P = u.initial_parameters
    t0 = u.start_timestamp
    d = dict(path=path, name=os.path.basename(path), P=P)
    d['Ixx'], d['Iyy'], d['Izz'] = P['MC_EIG_IXX'], P['MC_EIG_IYY'], P['MC_EIG_IZZ']
    d['TRQ'] = P['MC_EIG_TRQ_MAX']
    d['gyro_fc'], d['dgyro_fc'] = P['IMU_GYRO_CUTOFF'], P['IMU_DGYRO_CUTOFF']
    d['imu_rate'] = float(P.get('IMU_GYRO_RATEMAX', 1000) or 1000)
    if P.get('MC_LEAD_EN', 0) != 0 or P.get('MC_EIG_ALPHA', 0.0) != 0:
        print(f'WARNING {d["name"]}: lead/alpha not off - xd is not the pre-seat torque as assumed')

    w = u.get_dataset('vehicle_angular_velocity').data
    ts = u.get_dataset('vehicle_torque_setpoint').data
    _, iw, it = np.intersect1d(w['timestamp_sample'], ts['timestamp_sample'], return_indices=True)
    d['match_frac'] = len(iw) / len(ts['timestamp'])
    d['t'] = (w['timestamp_sample'][iw] - t0) / 1e6
    d['p'], d['q'], d['r'] = (w[f'xyz[{i}]'][iw] for i in range(3))
    d['pd'], d['qd'] = w['xyz_derivative[0]'][iw], w['xyz_derivative[1]'][iw]
    tx, ty = ts['xyz[0]'][it].astype(float), ts['xyz[1]'][it].astype(float)
    mc = u.get_dataset('mc_controller_status').data
    if 'torque_trim[0]' in mc:      # the logged setpoint is after the trim; xd is before it
        tm = (mc['timestamp'] - t0) / 1e6
        tx = tx - np.interp(d['t'], tm, np.nan_to_num(mc['torque_trim[0]']))
        ty = ty - np.interp(d['t'], tm, np.nan_to_num(mc['torque_trim[1]']))
    d['clip_frac'] = np.mean((np.abs(tx) >= 0.999) | (np.abs(ty) >= 0.999))
    d['seat_on'] = P.get('MC_SEAT_MODE', 0) != 0
    if d['seat_on']:   # logged torque is post-seat: undo the logged rotation to get the seat's xd
        tm = (mc['timestamp'] - t0) / 1e6; th = np.nan_to_num(mc['seat_theta'])
        k = np.clip(np.searchsorted(tm, d['t'], side='right') - 1, 0, len(th) - 1)
        c = (tx + 1j * ty) * np.exp(-1j * th[k]); tx, ty = c.real, c.imag
        print(f'NOTE {d["name"]}: seat on - xd un-rotated by logged seat_theta (50 Hz, last value held)')
    d['xd'] = d['TRQ'] * tx / d['Ixx'] + 1j * d['TRQ'] * ty / d['Iyy']
    d['xa'] = xa_for(d, d['Izz'])

    # firmware check: logged seat_alpha against arg(xa conj xd) on ticks that were logged
    tpub = ts['timestamp'][it]
    k = np.searchsorted(tpub, mc['timestamp'], side='right') - 1
    ok = (k >= 0) & (mc['timestamp'] - tpub[np.maximum(k, 0)] < 200) & np.isfinite(mc['seat_alpha'])
    dif = np.angle(np.exp(1j * (mc['seat_alpha'][ok] - np.angle(d['xa'][k[ok]] * np.conj(d['xd'][k[ok]])))))
    d['fw_check'] = (ok.sum(), np.max(np.abs(np.degrees(dif))) if ok.any() else np.nan)

    # heading (vehicle_attitude yaw, unwrapped) and gyro-integrated heading
    att = u.get_dataset('vehicle_attitude').data
    q0, q1, q2, q3 = (att[f'q[{i}]'] for i in range(4))
    yaw = np.unwrap(np.arctan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 ** 2 + q3 ** 2)))
    d['psi_att'] = np.interp(d['t'], (att['timestamp_sample'] - t0) / 1e6, yaw)

    # contiguous pieces
    br = np.where(np.diff(d['t']) > GAP)[0] + 1
    d['chunks'] = [(a, b) for a, b in zip(np.r_[0, br], np.r_[br, len(d['t'])]) if b - a > 10]
    psi_g = np.empty_like(d['t'])
    for a, b in d['chunks']:
        tt = d['t'][a:b]; rr = d['r'][a:b]
        psi_g[a:b] = d['psi_att'][a] + np.r_[0, np.cumsum(0.5 * (rr[1:] + rr[:-1]) * np.diff(tt))]
    d['psi_gyro'] = psi_g

    ld = u.get_dataset('vehicle_land_detected').data
    air = (ld['timestamp'][ld['landed'] == 0] - t0) / 1e6
    d['air'] = (air[0], air[-1])
    names = {x.name for x in u.data_list}
    if 'manual_control_setpoint' in names:
        ms = u.get_dataset('manual_control_setpoint').data
        d['stick'] = np.interp(d['t'], (ms['timestamp'] - t0) / 1e6, np.hypot(ms['roll'], ms['pitch']))
    else:
        d['stick'] = np.zeros(len(d['t']))
    if 'control_allocator_status' in names:
        ca = u.get_dataset('control_allocator_status').data
        unal = (np.abs(ca['unallocated_torque[0]']) > 1e-3) | (np.abs(ca['unallocated_torque[1]']) > 1e-3)
        kk = np.clip(np.searchsorted((ca['timestamp'] - t0) / 1e6, d['t'], side='right') - 1, 0, len(unal) - 1)
        d['unalloc'] = unal[kk]
    else:
        d['unalloc'] = np.zeros(len(d['t']), bool)
    d['_u'], d['_t0'] = u, t0
    return d


def xa_for(d, izz):
    """xa with the drift evaluated at yaw inertia izz (the firmware uses MC_EIG_IZZ)."""
    dr0 = (d['Iyy'] - izz) / d['Ixx'] * d['r'] * d['q']
    dr1 = (izz - d['Ixx']) / d['Iyy'] * d['r'] * d['p']
    return (d['pd'] - dr0) + 1j * (d['qd'] - dr1)


def seg_mask(d, spans):
    m = np.zeros(len(d['t']), bool)
    for a, b in spans:
        m |= (d['t'] >= a) & (d['t'] < b)
    return m


# ----------------------------------------------------------------------------- models
def filt_H(w, d):
    """xa's measurement chain relative to the true acceleration: 2-pole Butterworth at
    IMU_GYRO_CUTOFF (the D-term differentiates the low-passed gyro), backward difference
    (half a sample at the IMU rate), 1-pole AlphaFilter at IMU_DGYRO_CUTOFF. xd is unfiltered."""
    s = 1j * np.asarray(w, float)
    wg, wd = 2 * np.pi * d['gyro_fc'], 2 * np.pi * d['dgyro_fc']
    return wg ** 2 / (s ** 2 + np.sqrt(2) * wg * s + wg ** 2) * wd / (s + wd) * np.exp(-s / (2 * d['imu_rate']))


def G_model(w, d, L, T):
    """xa/xd at signed body frequency w [rad/s]: dead time L, first-order pole T, filters."""
    w = np.asarray(w, float)
    return np.exp(-1j * w * L) / (1 + 1j * w * T) * filt_H(w, d)


def K_coupling(w, r, k_assumed, k_true):
    """Real factor from a drift built on the wrong yaw inertia (Ixx = Iyy approximation):
    measured wdot = tau/I + i k_t r omega, drift removes i k_a r omega, omega = wdot/(i w):
        xa/xd_delayed = (1 - k_a r / w) / (1 - k_t r / w)
    Real at every w; negative (a 180 deg flip) for w between k_a r and k_t r. At w = -r it is
    (1 + k_a)/(1 + k_t) > 0, so the lock-in angle is untouched."""
    w = np.asarray(w, float)
    with np.errstate(divide='ignore', invalid='ignore'):
        return (1 - k_assumed * r / w) / (1 - k_true * r / w)


LAG_GRID = [(L, T) for L in (0.005, 0.0075) for T in (0.020, 0.025)]


def psi_model(d, r):
    """Lag rotation the seat should read at spin r, over the motor-lag envelope [deg]."""
    v = [np.degrees(np.angle(G_model(-r, d, L, T))) for L, T in LAG_GRID]
    return min(v), max(v)


# ----------------------------------------------------------------------------- helpers
def circ(z, blocks=None, nboot=300, seed=0):
    """arg of the summed product, R = |sum| / sum|.|, and a block-bootstrap 95% CI [deg]."""
    S = z.sum(); ang = np.degrees(np.angle(S)); R = abs(S) / np.abs(z).sum()
    ci = (np.nan, np.nan)
    if blocks is not None and len(np.unique(blocks)) > 4:
        rng = np.random.default_rng(seed)
        ub = np.unique(blocks); sums = np.array([z[blocks == b].sum() for b in ub])
        bs = [np.degrees(np.angle(np.exp(-1j * np.radians(ang)) *
                                  sums[rng.integers(0, len(ub), len(ub))].sum())) for _ in range(nboot)]
        ci = (ang + np.percentile(bs, 2.5), ang + np.percentile(bs, 97.5))
    return ang, R, ci


def blocks_of(t, length=2.0):
    return np.floor(t / length).astype(int)


def resample_chunk(t, *xs):
    tu = np.arange(t[0], t[-1], 1 / FS_U)
    return (tu,) + tuple(np.interp(tu, t, x.real) + 1j * np.interp(tu, t, x.imag) for x in xs)


def chunks_in(d, mask, min_len):
    """Contiguous pieces restricted to mask, each at least min_len seconds."""
    out = []
    for a, b in d['chunks']:
        m = mask[a:b]
        if not m.any():
            continue
        idx = np.where(m)[0] + a
        br = np.where(np.diff(idx) > 1)[0] + 1
        for s in np.split(idx, br):
            if d['t'][s[-1]] - d['t'][s[0]] >= min_len:
                out.append(s)
    return out


# ----------------------------------------------------------------------------- 1. reproduce
def reproduce_coupling_fit(d):
    """The 0.3-8 Hz number exactly as coupling_fit.py computes it (window air+-3 s, torque
    interpolated by publish time, brick-wall FFT band-pass across the whole record)."""
    u, t0 = d['_u'], d['_t0']
    w = u.get_dataset('vehicle_angular_velocity').data; ts = u.get_dataset('vehicle_torque_setpoint').data
    ld = u.get_dataset('vehicle_land_detected').data; air = ld['timestamp'][ld['landed'] == 0]
    m = (w['timestamp'] > air[0] + 3e6) & (w['timestamp'] < air[-1] - 3e6)
    t = w['timestamp'][m] / 1e6; fs = 1 / np.median(np.diff(t)); tt = ts['timestamp'] / 1e6
    p, q, r = w['xyz[0]'][m], w['xyz[1]'][m], w['xyz[2]'][m]
    pd, qd = w['xyz_derivative[0]'][m], w['xyz_derivative[1]'][m]

    def bp(x):
        X = np.fft.rfft(x - x.mean()); f = np.fft.rfftfreq(len(x), 1 / fs)
        X[(f < 0.3) | (f > 8.0)] = 0
        return np.fft.irfft(X, len(x))
    dd = np.interp(t, tt, ts['xyz[0]']) * d['TRQ'] / d['Ixx'] + 1j * np.interp(t, tt, ts['xyz[1]']) * d['TRQ'] / d['Iyy']
    a0 = pd - (d['Iyy'] - d['Izz']) / d['Ixx'] * r * q; a1 = qd - (d['Izz'] - d['Ixx']) / d['Iyy'] * r * p
    out = {}
    for lab, f_ in (('raw', lambda x: x), ('bp', bp)):
        z = (f_(a0) + 1j * f_(a1)) * np.conj(f_(dd.real) + 1j * f_(dd.imag)); S = z.sum()
        out[lab] = (np.degrees(np.angle(S)), abs(S) / np.abs(z).sum())
    out['window'] = ((air[0] - t0) / 1e6 + 3, (air[-1] - t0) / 1e6 - 3)
    return out


def kappa_fit(d):
    """coupling_fit.py's regression (same window and band-pass), best L by R^2 -> kappa_true."""
    u = d['_u']
    w = u.get_dataset('vehicle_angular_velocity').data; ts = u.get_dataset('vehicle_torque_setpoint').data
    ld = u.get_dataset('vehicle_land_detected').data; air = ld['timestamp'][ld['landed'] == 0]
    m = (w['timestamp'] > air[0] + 3e6) & (w['timestamp'] < air[-1] - 3e6)
    t = w['timestamp'][m] / 1e6; fs = 1 / np.median(np.diff(t)); tt = ts['timestamp'] / 1e6
    p, q, r = w['xyz[0]'][m], w['xyz[1]'][m], w['xyz[2]'][m]

    def bp(x):
        X = np.fft.rfft(x - x.mean()); f = np.fft.rfftfreq(len(x), 1 / fs)
        X[(f < 0.3) | (f > 8.0)] = 0
        return np.fft.irfft(X, len(x))
    Y = np.r_[bp(w['xyz_derivative[0]'][m]), bp(w['xyz_derivative[1]'][m])]
    best = None
    for L in np.arange(0.0, 0.081, 0.005):
        Tx = np.interp(t - L, tt, ts['xyz[0]']) * d['TRQ'] / d['Ixx']
        Ty = np.interp(t - L, tt, ts['xyz[1]']) * d['TRQ'] / d['Iyy']
        X = np.stack([np.r_[bp(Tx), bp(Ty)], np.r_[bp(-r * q), bp(r * p)], np.r_[bp(p), bp(q)]], 1)
        b, *_ = np.linalg.lstsq(X, Y, rcond=None); R2 = 1 - ((Y - X @ b) ** 2).sum() / (Y ** 2).sum()
        if best is None or R2 > best[0]:
            best = (R2, L, b)
    I = 0.5 * (d['Ixx'] + d['Iyy'])
    return dict(L=best[1], gain=best[2][0], kappa=best[2][1], izz_eff=I * (1 + best[2][1]), R2=best[0],
                kappa_assumed=(d['Izz'] - I) / I)


# ----------------------------------------------------------------------------- 2. spectra
def cross_spectrum(d, mask, xa, nper_s=4.0):
    """Welch cross-spectrum of complex xa against xd on signed frequency, per contiguous piece."""
    n = int(nper_s * FS_U); win = np.hanning(n); f = np.fft.fftshift(np.fft.fftfreq(n, 1 / FS_U))
    Sdd = np.zeros(n); Saa = np.zeros(n); Sad = np.zeros(n, complex); nseg = 0
    for s in chunks_in(d, mask, nper_s):
        _, xdu, xau = resample_chunk(d['t'][s], d['xd'][s], xa[s])
        for k in range(0, len(xdu) - n + 1, n // 2):
            D = np.fft.fftshift(np.fft.fft(win * (xdu[k:k + n] - xdu[k:k + n].mean())))
            A = np.fft.fftshift(np.fft.fft(win * (xau[k:k + n] - xau[k:k + n].mean())))
            Sdd += abs(D) ** 2; Saa += abs(A) ** 2; Sad += A * np.conj(D); nseg += 1
    coh = abs(Sad) ** 2 / (Sdd * Saa)
    return dict(f=f, Sad=Sad, Sdd=Sdd, Saa=Saa, coh=coh, nseg=nseg)


def phase_at(cs, f0, half=0.2):
    m = np.abs(cs['f'] - f0) <= half
    S = cs['Sad'][m].sum()
    return np.degrees(np.angle(S)), abs(S) ** 2 / (cs['Sdd'][m].sum() * cs['Saa'][m].sum())


def model_residual(cs, d, side, lo=0.3, hi=8.0, cmin=0.3):
    """Measured phase minus the lag model on the coherent bins of one side [deg]: the
    |S|-weighted circular mean, the rms, and the share of bins inside the envelope +-10 deg.
    The model is not fitted - L, T are the bench envelope, the filters are the flown params."""
    f = cs['f']; m = (np.abs(f) >= lo) & (np.abs(f) <= hi) & (np.sign(f) == side) & (cs['coh'] > cmin)
    if m.sum() < 3:
        return dict(n=int(m.sum()), mean=np.nan, rms=np.nan, inside=np.nan)
    w = 2 * np.pi * f[m]
    env = np.array([np.angle(G_model(w, d, L, T)) for L, T in LAG_GRID])
    mid = np.angle(G_model(w, d, 0.00625, 0.0225))
    res = np.angle(cs['Sad'][m] * np.exp(-1j * mid))
    meas = np.angle(cs['Sad'][m])
    inside = (meas >= env.min(0) - np.radians(10)) & (meas <= env.max(0) + np.radians(10))
    return dict(n=int(m.sum()), mean=np.degrees(np.angle((abs(cs['Sad'][m]) * np.exp(1j * res)).sum())),
                rms=np.degrees(np.sqrt(np.mean(res ** 2))), inside=inside.mean())


# ----------------------------------------------------------------------------- 3. +/- split
def band_decomposition(d, mask, xa, lo=0.3, hi=8.0, edges=None):
    """Exact split of the band-passed observable sum(xa_bp conj xd_bp) over signed frequency
    (Parseval per contiguous piece: sum_t a conj d = (1/N) sum_f A conj D)."""
    if edges is None:
        edges = np.r_[-hi, -np.arange(7, 0, -1), -lo, lo, np.arange(1, 8), hi]
    contrib = np.zeros(len(edges) - 1, complex); total = 0j; absz = 0.0
    for s in chunks_in(d, mask, 4.0):
        _, xdu, xau = resample_chunk(d['t'][s], d['xd'][s], xa[s])
        N = len(xdu); f = np.fft.fftfreq(N, 1 / FS_U)
        D = np.fft.fft(xdu - xdu.mean()); A = np.fft.fft(xau - xau.mean())
        Z = A * np.conj(D) / N
        band = (np.abs(f) >= lo) & (np.abs(f) <= hi)
        total += Z[band].sum()
        Db = D.copy(); Db[~band] = 0; Ab = A.copy(); Ab[~band] = 0
        absz += np.abs(np.fft.ifft(Ab) * np.conj(np.fft.ifft(Db))).sum()
        for i in range(len(edges) - 1):
            mm = band & (f >= edges[i]) & (f < edges[i + 1])
            contrib[i] += Z[mm].sum()
    return dict(edges=edges, contrib=contrib, total=total, R=abs(total) / absz)


# ----------------------------------------------------------------------------- 5. lock-in
def lowpass(x, t, fc, order=1):
    """First-order IIR with the per-tick dt (as the firmware would run it), cascaded order times."""
    tau = 1 / (2 * np.pi * fc); dt = np.r_[0, np.diff(t)]; a = dt / (tau + dt)
    for _ in range(order):
        y = np.empty_like(x); acc = x[0]
        for i in range(len(x)):
            acc = acc + a[i] * (x[i] - acc); y[i] = acc
        x = y
    return x


def lockin_signals(d, xa, fc, heading='att', order=1):
    """World-frame, low-passed xd and xa on every tick; NaN until 5 time constants into a piece."""
    psi = d['psi_att'] if heading == 'att' else d['psi_gyro']
    rot = np.exp(1j * psi)
    xdw = np.full(len(d['t']), np.nan + 0j); xaw = xdw.copy()
    settle = 5 * order / (2 * np.pi * fc)
    for a, b in d['chunks']:
        tt = d['t'][a:b]
        yd = lowpass(d['xd'][a:b] * rot[a:b], tt, fc, order); ya = lowpass(xa[a:b] * rot[a:b], tt, fc, order)
        ok = tt - tt[0] >= settle
        xdw[a:b][ok] = yd[ok]; xaw[a:b][ok] = ya[ok]
    return xdw, xaw


def lockin(d, mask, xa, fc, heading='att', order=1):
    xdw, xaw = lockin_signals(d, xa, fc, heading, order)
    m = mask & np.isfinite(xdw)
    z = xaw[m] * np.conj(xdw[m])
    ang, R, ci = circ(z, blocks_of(d['t'][m]))
    return dict(ang=ang, R=R, ci=ci, n=m.sum(), amp_d=np.sqrt(np.mean(abs(xdw[m]) ** 2)),
                amp_a=np.sqrt(np.mean(abs(xaw[m]) ** 2)), z=z, t=d['t'][m])


# ----------------------------------------------------------------------------- 6. fixed point
def cross_fixed_point(z):
    """MC_SEAT_LAW 1 (cross), KSIGN +1: theta_s -= k Im(conj(xd) e^{i theta} xa) dt. With the seat
    rotating the command by e^{i theta}, the mean step is -k |S| sin(arg S + theta): its stable
    zero is theta* = -arg S, where S is the sum of the observable's product. The seat then
    believes the lag rotation is arg S. First order: assumes the rotation does not reshape the
    closed-loop spectra."""
    S = z.sum()
    th = np.radians(np.arange(-180, 180, 0.25))
    F = np.array([np.mean(np.imag(np.exp(1j * x) * z)) for x in th[::8]])     # sanity: sign change
    return -np.degrees(np.angle(S)), F


def angle_fixed_point(alpha):
    """MC_SEAT_LAW 0: theta where mean(wrap(alpha + theta)) = 0, stable branch."""
    th = np.radians(np.arange(-180, 180, 0.5))
    F = np.array([np.mean(np.angle(np.exp(1j * (alpha + x)))) for x in th])
    zc = np.where((F[:-1] > 0) & (F[1:] <= 0))[0]
    return [np.degrees(th[i]) for i in zc]


# ----------------------------------------------------------------------------- report
def bp_obs(d, mask, xa, lo=0.3, hi=8.0):
    """Band-passed product, brick-wall per contiguous piece ON A UNIFORM TIME GRID. The logged
    ticks are every 3rd or 4th 1 kHz tick, so an index-based FFT (coupling_fit.py, fs = 1/median
    dt) puts the band edges at the wrong true frequency: x mean/median rate (A 1.12, B 0.86)."""
    zs = []; ts_ = []
    for s in chunks_in(d, mask, 4.0):
        tu, xdu, xau = resample_chunk(d['t'][s], d['xd'][s], xa[s])

        def bp(v):
            X = np.fft.fft(v - v.mean()); f = np.fft.fftfreq(len(v), 1 / FS_U)
            X[(np.abs(f) < lo) | (np.abs(f) > hi)] = 0
            return np.fft.ifft(X)
        zs.append(bp(xau) * np.conj(bp(xdu))); ts_.append(tu)
    return np.concatenate(zs), np.concatenate(ts_)


def demand_bins(d, mask, xa, fc=0.5, win=2.0):
    """Hypothesis 2: lock-in angle per window against the window's world-fixed commanded demand
    |xd_w| and stick. Returns per-window (t, |xd_w| rms, stick mean, sum z, sum |z|)."""
    xdw, xaw = lockin_signals(d, xa, fc)
    m = mask & np.isfinite(xdw); z = xaw * np.conj(xdw)
    b = blocks_of(d['t'], win); rows = []
    for k in np.unique(b[m]):
        mm = m & (b == k)
        if mm.sum() < 0.8 * win * 250:
            continue
        rows.append((d['t'][mm].mean(), np.sqrt(np.mean(abs(xdw[mm]) ** 2)), d['stick'][mm].mean(),
                     z[mm].sum(), abs(z[mm]).sum()))
    return rows


def default_segments(d):
    a, b = d['air'][0] + 3, d['air'][1] - 3
    m = (d['t'] > a) & (d['t'] < b)
    rmed = np.median(d['r'][m]); ok = m & (np.abs(d['r'] - rmed) < 0.1 * abs(rmed))
    idx = np.where(ok)[0]
    return {'steady': [(d['t'][idx[0]], d['t'][idx[-1]])]}


def analyse(d, segs, izz_extra):
    out = dict(name=d['name'], segs={})
    print(f'\n================ {d["name"]}')
    print(f'params: IXX/IYY/IZZ {d["Ixx"]:.3f}/{d["Iyy"]:.3f}/{d["Izz"]:.3f}  TRQ_MAX {d["TRQ"]:.2f}  '
          f'IMU_GYRO_CUTOFF {d["gyro_fc"]:.0f}  IMU_DGYRO_CUTOFF {d["dgyro_fc"]:.0f}  '
          f'IMU_GYRO_RATEMAX {d["imu_rate"]:.0f}  SEAT_MODE {d["P"].get("MC_SEAT_MODE")}  '
          f'LEAD_EN {d["P"].get("MC_LEAD_EN")}')
    print(f'pairing: {100*d["match_frac"]:.1f}% of torque samples matched by timestamp_sample; '
          f'firmware check: {d["fw_check"][0]} logged seat_alpha samples, max |diff| {d["fw_check"][1]:.2e} deg; '
          f'clipped {100*d["clip_frac"]:.2f}%')
    rep = reproduce_coupling_fit(d); out['repro'] = rep
    print(f'1. REPRODUCE (coupling_fit.py method, {rep["window"][0]:.1f}-{rep["window"][1]:.1f} s): '
          f'0.3-8 Hz {rep["bp"][0]:.1f} deg R {rep["bp"][1]:.2f} | raw {rep["raw"][0]:.1f} deg R {rep["raw"][1]:.2f}')
    wm = seg_mask(d, [rep['window']]); dts = np.diff(d['t'][wm]); dts = dts[dts < GAP]
    k = np.median(dts) / np.mean(dts); out['band_scale'] = k
    zw, _ = bp_obs(d, wm, d['xa'], 0.3 * k, 8.0 * k); zt, _ = bp_obs(d, wm, d['xa'])
    print(f'   index-based FFT: fs taken {1/np.median(dts):.0f} Hz, true mean {1/np.mean(dts):.0f} Hz, so its band '
          f'is really {0.3*k:.2f}-{8*k:.2f} Hz. Time-correct, that band: {circ(zw)[0]:.1f} deg R {circ(zw)[1]:.2f}; '
          f'true 0.3-8 Hz: {circ(zt)[0]:.1f} deg R {circ(zt)[1]:.2f}')
    out['repro_tc'] = (circ(zw)[:2], circ(zt)[:2])
    kf = kappa_fit(d); out['kfit'] = kf
    print(f'   coupling fit: best L {kf["L"]*1e3:.0f} ms gain {kf["gain"]:.2f} kappa {kf["kappa"]:+.2f} '
          f'(assumed {kf["kappa_assumed"]:+.2f}) Izz_eff {kf["izz_eff"]:.4f} R2 {kf["R2"]:.2f}')
    izzs = [d['Izz'], round(kf['izz_eff'], 4)] + [z for z in izz_extra if z not in (d['Izz'],)]
    for name, spans in segs.items():
        mask = seg_mask(d, spans)
        if mask.sum() < 1000:
            continue
        r = np.mean(d['r'][mask]); fr = -r / (2 * np.pi)
        lo, hi = psi_model(d, r)
        so = dict(r=r, psi=(lo, hi), stick=np.mean(d['stick'][mask]), dur=mask.sum() / 330)
        print(f'\n--- segment {name} {spans}  r {r:.2f} rad/s (-r = {fr:+.2f} Hz)  stick mean {so["stick"]:.3f}  '
              f'lag model psi(-r) {lo:.1f}-{hi:.1f} deg  r*35ms {np.degrees(r*0.035):.1f}')
        zb, tb = bp_obs(d, mask, d['xa']); a, R, ci = circ(zb, blocks_of(tb))
        zr = (d['xa'] * np.conj(d['xd']))[mask]; ar, Rr, cir = circ(zr, blocks_of(d['t'][mask]))
        so['bp'] = (a, R, ci); so['raw'] = (ar, Rr, cir)
        print(f'   observable 0.3-8 Hz (firmware pairing): {a:6.1f} deg [{ci[0]:.0f},{ci[1]:.0f}] R {R:.2f} | '
              f'raw (what the firmware integrates) {ar:6.1f} deg [{cir[0]:.0f},{cir[1]:.0f}] R {Rr:.2f}')
        # 2. cross-spectrum
        cs = cross_spectrum(d, mask, d['xa']); so['cs'] = cs
        ph, co = phase_at(cs, fr)
        rn = model_residual(cs, d, -1); rp = model_residual(cs, d, +1)
        so['ph_r'] = (ph, co); so['resid'] = (rn, rp)
        print(f'2. cross-spectrum ({cs["nseg"]} x 4 s): phase at -r {ph:6.1f} deg coherence^2 {co:.2f}')
        for lab, rr_ in (('w<0', rn), ('w>0', rp)):
            print(f'   vs lag model, coherent bins (coh^2>0.3) {lab}: n {rr_["n"]:2d}  weighted mean residual '
                  f'{rr_["mean"]:6.1f} deg  rms {rr_["rms"]:5.1f}  inside envelope+-10deg {rr_["inside"]:.2f}')
        fine = []
        for f0 in np.arange(-r / (2 * np.pi) - 1.5, -r / (2 * np.pi) + 1.51, 0.25):
            p_, c_ = phase_at(cs, f0, 0.13); fine.append((f0, p_, c_))
        print('   around -r: ' + '  '.join(f'{f0:+.2f}:{p_:.0f}/{c_:.2f}' for f0, p_, c_ in fine))
        for f0 in (-7, -6, -5, -4, -3, -2, -1, 1, 2, 3, 4, 5, 6, 7):
            p_, c_ = phase_at(cs, f0, 0.25)
            mdl = [np.degrees(np.angle(G_model(2 * np.pi * f0, d, L, T))) for L, T in LAG_GRID]
            print(f'      f {f0:+3d} Hz: phase {p_:7.1f}  coh^2 {c_:.2f}   model {min(mdl):6.1f}..{max(mdl):6.1f}')
        # 3. split by rotation direction
        bd = band_decomposition(d, mask, d['xa']); so['bands'] = bd
        e = bd['edges']; neg = bd['contrib'][e[:-1] < 0].sum(); pos = bd['contrib'][e[:-1] > 0].sum()
        tot = abs(bd['contrib']).sum()
        print(f'3. split (0.3-8 Hz, sum = {np.degrees(np.angle(bd["total"])):.1f} deg R {bd["R"]:.2f}): '
              f'w<0 |S| {abs(neg)/abs(bd["total"]):.2f} at {np.degrees(np.angle(neg)):6.1f} deg, '
              f'w>0 |S| {abs(pos)/abs(bd["total"]):.2f} at {np.degrees(np.angle(pos)):6.1f} deg  (|S|/|total|)')
        for i in range(len(e) - 1):
            c = bd['contrib'][i]
            print(f'      {e[i]:+5.1f}..{e[i+1]:+5.1f} Hz: share {abs(c)/tot:5.2f}  angle {np.degrees(np.angle(c)):7.1f}')
        so['split'] = (neg, pos)
        # 5. lock-in
        so['lock'] = {}
        for fc in (0.3, 0.5, 1.0):
            for hd in ('att', 'gyro'):
                L_ = lockin(d, mask, d['xa'], fc, hd); so['lock'][(fc, hd)] = L_
                print(f'5. lock-in fc {fc:.1f} Hz heading {hd:4s}: {L_["ang"]:6.1f} deg '
                      f'[{L_["ci"][0]:.0f},{L_["ci"][1]:.0f}] R {L_["R"]:.2f}  |xd_w| {L_["amp_d"]:.2f} '
                      f'|xa_w| {L_["amp_a"]:.2f} rad/s^2  (model {lo:.0f}-{hi:.0f})')
        for fc in (0.5, 1.0):
            L_ = lockin(d, mask, d['xa'], fc, order=2); so['lock'][(fc, '2nd')] = L_
            print(f'5. lock-in fc {fc:.1f} Hz 2nd order    : {L_["ang"]:6.1f} deg '
                  f'[{L_["ci"][0]:.0f},{L_["ci"][1]:.0f}] R {L_["R"]:.2f}')
        so['izz'] = {}
        for izz in izzs:
            xa2 = xa_for(d, izz); zb2, tb2 = bp_obs(d, mask, xa2)
            L2 = lockin(d, mask, xa2, 0.5); ph2, _ = phase_at(cross_spectrum(d, mask, xa2), fr)
            so['izz'][izz] = (circ(zb2)[0], L2['ang'], ph2)
            print(f'   drift Izz {izz:.4f}: 0.3-8 Hz {circ(zb2)[0]:6.1f} deg | lock-in 0.5 Hz {L2["ang"]:6.1f} | '
                  f'cross-spectrum at -r {ph2:6.1f}')
        # 6. fixed point
        th_raw, _ = cross_fixed_point(zr); th_bp, _ = cross_fixed_point(zb)
        th_li, _ = cross_fixed_point(so['lock'][(0.5, 'att')]['z'])
        ns = ~d['unalloc'][mask]; th_raw_sg, _ = cross_fixed_point(zr[ns])
        alpha = np.angle(zr); afp = angle_fixed_point(alpha)
        so['fp'] = dict(raw=th_raw, bp=th_bp, lock=th_li, raw_satgate=th_raw_sg, angle_law=afp)
        print(f'6. cross-law fixed point theta_s* (target -psi = {-hi:.0f}..{-lo:.0f}): raw {th_raw:6.1f}  '
              f'raw w/o unallocated ticks ({100*(1-ns.mean()):.0f}% dropped) {th_raw_sg:6.1f}  '
              f'0.3-8 Hz {th_bp:6.1f}  lock-in 0.5 Hz {th_li:6.1f} | angle law (raw) {afp}')
        out['segs'][name] = so
    # 4. hypothesis 2 across the first (whole) segment: 2 s windows binned by commanded demand
    name0 = next(iter(segs)); rows = demand_bins(d, seg_mask(d, segs[name0]), d['xa'])
    if rows:
        a = np.array([(x[1], x[2]) for x in rows]); zsum = np.array([x[3] for x in rows])
        zabs = np.array([x[4] for x in rows]); q = np.quantile(a[:, 0], [0, 1 / 3, 2 / 3, 1])
        out['demand'] = []
        print(f'\n4. demand bins ({name0}, 2 s windows, lock-in 0.5 Hz): angle and R of the pooled sum per bin')
        for i in range(3):
            m = (a[:, 0] >= q[i]) & (a[:, 0] <= q[i + 1])
            S = zsum[m].sum(); ang = np.degrees(np.angle(S)); R = abs(S) / zabs[m].sum()
            out['demand'].append(('|xd_w| tercile %d' % (i + 1), q[i], q[i + 1], m.sum(), ang, R))
            print(f'   |xd_w| {q[i]:.2f}-{q[i+1]:.2f} rad/s^2 ({m.sum():2d} windows, stick {a[m,1].mean():.3f}): '
                  f'{ang:6.1f} deg R {R:.2f}')
        for lab, m in (('stick == 0  ', a[:, 1] < 0.005), ('stick > 0.05', a[:, 1] > 0.05)):
            if m.any():
                S = zsum[m].sum()
                print(f'   {lab} ({m.sum():2d} windows, |xd_w| {a[m,0].mean():.2f}): '
                      f'{np.degrees(np.angle(S)):6.1f} deg R {abs(S)/zabs[m].sum():.2f}')
        out['demand_rows'] = rows
    return out


# ----------------------------------------------------------------------------- figures
INK, INK2, GRID, SURF = '#0b0b0b', '#52514e', '#e4e3df', '#fcfcfb'
C1, C2, C3 = '#2a78d6', '#eb6834', '#1baf7a'


def style(ax):
    ax.set_facecolor(SURF)
    for s in ('top', 'right'):
        ax.spines[s].set_visible(False)
    for s in ('left', 'bottom'):
        ax.spines[s].set_color(INK2)
    ax.tick_params(colors=INK2, labelsize=8)
    ax.grid(True, color=GRID, lw=0.6); ax.set_axisbelow(True)


def figures(results, logs, fig_dir, seg_main):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    os.makedirs(fig_dir, exist_ok=True)
    n = len(results)

    # Fig 1: phase vs signed frequency with the lag model
    fig, axs = plt.subplots(2, n, figsize=(5.2 * n, 5.6), sharex=True,
                            gridspec_kw=dict(height_ratios=[3, 1]), squeeze=False, facecolor=SURF)
    for j, (res, d) in enumerate(zip(results, logs)):
        so = res['segs'][seg_main]; cs = so['cs']; f = cs['f']; r = so['r']
        m = (np.abs(f) >= 0.25) & (np.abs(f) <= 10)
        ax = axs[0, j]; style(ax)
        ff = np.linspace(-10, 10, 2001); ff = ff[np.abs(ff) > 0.02]; w = 2 * np.pi * ff
        env = np.array([np.degrees(np.angle(G_model(w, d, L, T))) for L, T in LAG_GRID])
        ax.fill_between(ff, env.min(0), env.max(0), color=C1, alpha=0.25, lw=0,
                        label='lag model: L 5-7.5 ms, T 20-25 ms, + gyro/D-term filters')
        ax.plot(ff, np.degrees(-w * 0.035), color=INK2, lw=1, ls='--', label='pure 35 ms delay')
        kf = res['kfit']
        Gc = G_model(w, d, 0.00625, 0.0225) * K_coupling(w, r, kf['kappa_assumed'], kf['kappa'])
        pc = np.degrees(np.angle(Gc)); pc[1:][np.abs(np.diff(pc)) > 90] = np.nan    # break at the wrap
        ax.plot(ff, pc, color=C2, lw=1.5,
                label=f'lag model mid + drift-Izz mismatch (κ {kf["kappa_assumed"]:+.2f} assumed, {kf["kappa"]:+.2f} fitted)')
        sz = 4 + 40 * cs['coh'][m]
        ax.scatter(f[m], np.degrees(np.angle(cs['Sad'][m])), s=sz, color=INK, alpha=0.75, lw=0,
                   label='measured arg(S_ad), size ∝ coherence²')
        ax.axvline(-r / (2 * np.pi), color=C3, lw=1.5)
        ph, co = so['ph_r']
        ax.annotate(f'ω = −r: {ph:.0f}°\n(model {so["psi"][0]:.0f}–{so["psi"][1]:.0f}°)',
                    (-r / (2 * np.pi), ph), xytext=(-9.6, -150), fontsize=8, color=INK,
                    arrowprops=dict(arrowstyle='-', color=C3, lw=1))
        ax.set_ylim(-185, 185); ax.set_yticks(range(-180, 181, 60))
        ax.set_title(f'{d["name"][:22]}  (r = {r:.1f} rad/s, segment "{seg_main}")', fontsize=9, color=INK)
        if j == 0:
            ax.set_ylabel('arg(xa·conj xd)  [deg]', color=INK, fontsize=9)
        ax2 = axs[1, j]; style(ax2)
        ax2.plot(f[m & (f < 0)], cs['coh'][m & (f < 0)], color=INK, lw=1)
        ax2.plot(f[m & (f > 0)], cs['coh'][m & (f > 0)], color=INK, lw=1)
        ax2.axvline(-r / (2 * np.pi), color=C3, lw=1.5)
        ax2.set_ylim(0, 1); ax2.set_xlabel('signed body frequency [Hz]  (world-fixed content at −r/2π)',
                                           color=INK, fontsize=9)
        if j == 0:
            ax2.set_ylabel('coherence²', color=INK, fontsize=9)
    axs[0, 0].legend(fontsize=7, loc='upper right', frameon=False, labelcolor=INK)
    fig.tight_layout(); fig.savefig(os.path.join(fig_dir, 'fig1_phase_signed_freq.png'), dpi=150)

    # Fig 2: lock-in angle against the lag rotation, per log and segment
    fig, axs = plt.subplots(1, n, figsize=(5.2 * n, 4.4), squeeze=False, facecolor=SURF, sharey=True)
    for j, (res, d) in enumerate(zip(results, logs)):
        ax = axs[0, j]; style(ax); names = list(res['segs'])
        for k, name in enumerate(names):
            so = res['segs'][name]; lo, hi = so['psi']
            ax.fill_between([k - 0.4, k + 0.4], lo, hi, color=C1, alpha=0.25, lw=0,
                            label='lag model ψ(−r): L 5–7.5, T 20–25 ms + filters' if k == 0 else None)
            ax.plot([k - 0.4, k + 0.4], [np.degrees(so['r'] * 0.035)] * 2, color=INK2, lw=1, ls='--',
                    label='r·35 ms' if k == 0 else None)
            for dx, fc, mk, lab in ((-0.2, 0.3, 'o', 'lock-in 0.3 Hz'), (0.0, 0.5, 'o', 'lock-in 0.5 Hz (95% CI)'),
                                    (0.2, 1.0, 'o', 'lock-in 1 Hz')):
                L_ = so['lock'][(fc, 'att')]; ci = L_['ci']
                if np.all(np.isfinite(ci)):
                    ax.plot([k + dx] * 2, ci, color=C1, lw=1.2)
                ax.plot(k + dx, L_['ang'], mk, ms={0.3: 4, 0.5: 7, 1.0: 4}[fc], color=C1, mec=SURF, mew=1.2,
                        label=(lab if (k == 0 and fc == 0.5) else None))
            ax.plot(k + 0.32, so['bp'][0], 's', ms=6, color=C2, mec=SURF, mew=1.2,
                    label='current observable, 0.3–8 Hz' if k == 0 else None)
            ax.text(k, -168, f'R {so["lock"][(0.5, "att")]["R"]:.2f}', ha='center', fontsize=7, color=INK2)
        ax.axhline(0, color=INK2, lw=0.6)
        ax.set_xticks(range(len(names))); ax.set_xticklabels(names, fontsize=8, color=INK)
        ax.set_ylim(-180, 120); ax.set_yticks(range(-180, 121, 30))
        ax.set_title(f'{d["name"][:22]}  (r ≈ {res["segs"][names[0]]["r"]:.1f} rad/s)', fontsize=9, color=INK)
        if j == 0:
            ax.set_ylabel('misalignment arg(xa·conj xd) [deg]', color=INK, fontsize=9)
    axs[0, 0].legend(fontsize=7, loc='upper left', frameon=False, labelcolor=INK, ncol=2)
    fig.text(0.5, 0.005, 'small dots: lock-in 0.3 Hz (left) and 1 Hz (right); R is the 0.5 Hz lock-in coherence',
             ha='center', fontsize=7, color=INK2)
    fig.tight_layout(rect=(0, 0.03, 1, 1)); fig.savefig(os.path.join(fig_dir, 'fig2_lockin_vs_lag.png'), dpi=150)

    # Fig 3: how the single angle arises - head-to-tail sum over signed bands
    fig, axs = plt.subplots(1, n, figsize=(5 * n, 4.6), squeeze=False, facecolor=SURF)
    for j, (res, d) in enumerate(zip(results, logs)):
        so = res['segs'][seg_main]; bd = so['bands']; e = bd['edges']; c = bd['contrib'] / abs(bd['total'])
        ax = axs[0, j]; style(ax); ax.set_aspect('equal')
        order = [i for i in range(len(c)) if e[i] < 0] + [i for i in range(len(c)) if e[i] > 0]
        pos = 0j
        for i in order:
            col = C1 if e[i] < 0 else C2
            ax.annotate('', xy=(pos + c[i]).real * np.r_[1, 0] + (pos + c[i]).imag * np.r_[0, 1],
                        xytext=(pos.real, pos.imag),
                        arrowprops=dict(arrowstyle='-|>', color=col, lw=1.6, shrinkA=0, shrinkB=0))
            if abs(c[i]) > 0.2:
                mid = pos + c[i] / 2
                ax.text(mid.real, mid.imag, f' {e[i]:+.0f}..{e[i+1]:+.0f}', fontsize=7, color=INK2)
            pos += c[i]
        ax.annotate('', xy=(pos.real, pos.imag), xytext=(0, 0),
                    arrowprops=dict(arrowstyle='-|>', color=INK, lw=2, shrinkA=0, shrinkB=0))
        psi = np.radians(np.mean(so['psi'])); Lr = 1.3 * max(abs(np.cumsum(c[order])).max(), 1)
        ax.plot([0, Lr * np.cos(psi)], [0, Lr * np.sin(psi)], color=C3, lw=1.5, ls='--')
        ax.text(Lr * np.cos(psi), Lr * np.sin(psi), f' lag ψ(−r) {np.degrees(psi):.0f}°', fontsize=8, color=INK)
        ax.text(pos.real, pos.imag, f'  sum {np.degrees(np.angle(pos)):.0f}°', fontsize=8, color=INK, va='bottom')
        ax.plot([], [], color=C1, lw=2, label='bands at ω < 0 (turning with world-fixed demand)')
        ax.plot([], [], color=C2, lw=2, label='bands at ω > 0')
        ax.axhline(0, color=INK2, lw=0.6); ax.axvline(0, color=INK2, lw=0.6)
        ax.set_title(f'{d["name"][:22]}: 0.3–8 Hz product, 1 Hz signed bands (Hz)', fontsize=9, color=INK)
        ax.set_xlabel('Re  (fraction of |sum|)', fontsize=8, color=INK); ax.set_ylabel('Im', fontsize=8, color=INK)
        ax.legend(fontsize=7, loc='lower left', frameon=False, labelcolor=INK)
        ax.autoscale_view(); xl = ax.get_xlim(); yl = ax.get_ylim()
        lim = max(map(abs, xl + yl)) * 1.1
        ax.set_xlim(-lim, lim); ax.set_ylim(-lim, lim)
    fig.tight_layout(); fig.savefig(os.path.join(fig_dir, 'fig3_signed_band_sum.png'), dpi=150)
    print(f'\nfigures written to {fig_dir}')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('ulogs', nargs='+')
    ap.add_argument('--seg', action='append', default=[], help='NAME=T0-T1[+T2-T3] (log-relative s)')
    ap.add_argument('--seg-for', action='append', default=[],
                    help='INDEX:NAME=T0-T1[+...]  segment for one log only (0-based index in ulogs)')
    ap.add_argument('--izz', type=float, action='append', default=[])
    ap.add_argument('--fig-dir')
    ap.add_argument('--fig-seg', default='all', help='segment used for figures 1 and 3')
    a = ap.parse_args()

    def parse(spec):
        name, rng = spec.split('=')
        return name, [tuple(map(float, x.split('-'))) for x in rng.split('+')]
    logs, results = [], []
    for i, path in enumerate(a.ulogs):
        d = load(path)
        segs = dict(parse(s) for s in a.seg)
        segs.update(dict(parse(s.split(':', 1)[1]) for s in a.seg_for if int(s.split(':')[0]) == i))
        if not segs:
            segs = default_segments(d)
        segs.setdefault(a.fig_seg, default_segments(d)['steady'])
        segs = {a.fig_seg: segs.pop(a.fig_seg), **segs}
        results.append(analyse(d, segs, a.izz)); logs.append(d)
    if a.fig_dir:
        figures(results, logs, a.fig_dir, a.fig_seg)


if __name__ == '__main__':
    sys.exit(main())
