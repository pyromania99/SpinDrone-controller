#!/usr/bin/env python3
"""Roll/pitch plant fit under spin: authority gain, gyroscopic coupling and lag, plus the
seat observable's coherence.

    python3 coupling_fit.py <log.ulg>

Model, 0.3-8 Hz band-passed, both axes stacked:
    [pdot; qdot] = g * [Tx/Ixx; Ty/Iyy](t-L) + kappa * r * [-q; p] + a * [p; q]
T is vehicle_torque_setpoint * MC_EIG_TRQ_MAX, so the controller's own gyroscopic
feedforward is INSIDE T and kappa is the plant's net coupling, (Izz_eff - I)/I with
I = (Ixx+Iyy)/2. The controller assumes kappa = (MC_EIG_IZZ - I)/I.
L is scanned; g, kappa, a are least squares per L. The fit is also repeated with kappa
forced to the assumed value. NEEDS GAP-FREE DATA: cutting log_1 into 90/80 ms pieces
made kappa meaningless (+0.65..-0.86).
"""
import sys
import numpy as np
from pyulog import ULog

u = ULog(sys.argv[1]); P = u.initial_parameters
Ixx, Iyy, Izz, TRQ = P['MC_EIG_IXX'], P['MC_EIG_IYY'], P['MC_EIG_IZZ'], P['MC_EIG_TRQ_MAX']
alpha = P.get('MC_EIG_ALPHA', 0.0); I = 0.5 * (Ixx + Iyy)
w = u.get_dataset('vehicle_angular_velocity').data; ts = u.get_dataset('vehicle_torque_setpoint').data
ld = u.get_dataset('vehicle_land_detected').data; air = ld['timestamp'][ld['landed'] == 0]
m = (w['timestamp'] > air[0] + 3e6) & (w['timestamp'] < air[-1] - 3e6)
t = w['timestamp'][m] / 1e6
p, q, r = w['xyz[0]'][m], w['xyz[1]'][m], w['xyz[2]'][m]
pd, qd = w['xyz_derivative[0]'][m], w['xyz_derivative[1]'][m]
fs = 1 / np.median(np.diff(t)); tt = ts['timestamp'] / 1e6
if np.max(np.diff(t)) > 0.02:
    print('WARNING: gaps in vehicle_angular_velocity (max %.0f ms) - results are not valid'
          % (1e3 * np.max(np.diff(t))))

def bp(x, lo=0.3, hi=8.0):
    X = np.fft.rfft(x - x.mean()); f = np.fft.rfftfreq(len(x), 1 / fs)
    X[(f < lo) | (f > hi)] = 0
    return np.fft.irfft(X, len(x))

k_assumed = (Izz - I) / I
print(f'fs {fs:.0f} Hz  r mean {r.mean():.2f} rad/s  span {t[-1]-t[0]:.0f} s  '
      f'assumed kappa {k_assumed:.3f} (MC_EIG_IZZ {Izz})')
best = None
for L in np.arange(0.0, 0.081, 0.005):
    Tx = np.interp(t - L, tt, ts['xyz[0]']) * TRQ / Ixx
    Ty = np.interp(t - L, tt, ts['xyz[1]']) * TRQ / Iyy
    Y = np.r_[bp(pd), bp(qd)]
    X = np.stack([np.r_[bp(Tx), bp(Ty)], np.r_[bp(-r * q), bp(r * p)], np.r_[bp(p), bp(q)]], 1)
    b, *_ = np.linalg.lstsq(X, Y, rcond=None); R2 = 1 - ((Y - X @ b) ** 2).sum() / (Y ** 2).sum()
    Y2 = Y - k_assumed * X[:, 1]
    b2, *_ = np.linalg.lstsq(X[:, [0, 2]], Y2, rcond=None)
    R2f = 1 - ((Y2 - X[:, [0, 2]] @ b2) ** 2).sum() / (Y ** 2).sum()
    print(f'L {L*1e3:3.0f} ms  gain {b[0]:.2f}  kappa {b[1]:6.3f}  Izz_eff {I*(1+b[1]):.4f}  '
          f'R2 {R2:.3f} | R2 with assumed kappa {R2f:.3f}')
    if best is None or R2 > best[0]:
        best = (R2, L, b)
print(f'BEST: L {best[1]*1e3:.0f} ms  gain {best[2][0]:.2f}  kappa {best[2][1]:.3f}  '
      f'Izz_eff {I*(1+best[2][1]):.4f}  R2 {best[0]:.3f}')

# Seat observable (EigenController: xd = tau/I, xa = omega_dot - assumed drift)
Txr = np.interp(t, tt, ts['xyz[0]']) * TRQ; Tyr = np.interp(t, tt, ts['xyz[1]']) * TRQ
d = Txr / Ixx + 1j * Tyr / Iyy
for lab, f_ in (('raw', lambda x: x), ('0.3-8 Hz', bp)):
    dr0 = -alpha * p + (Iyy - Izz) / Ixx * r * q; dr1 = -alpha * q + (Izz - Ixx) / Iyy * r * p
    a = f_(pd - dr0) + 1j * f_(qd - dr1); dd = f_(d.real) + 1j * f_(d.imag)
    z = a * np.conj(dd); zz = z.sum() / np.abs(z).sum()
    print(f'seat observable {lab:8s}: mean misalignment {np.degrees(np.angle(zz)):6.1f} deg  '
          f'coherence R {abs(zz):.2f}   (lag model r*35ms = {np.degrees(r.mean()*0.035):.1f} deg)')
mc = u.get_dataset('mc_controller_status').data
mm = (mc['timestamp'] > air[0] + 3e6) & (mc['timestamp'] < air[-1] - 3e6)
sa = mc['seat_alpha'][mm]; sa = sa[np.isfinite(sa)]
if len(sa):
    zs = np.exp(1j * sa).mean()
    print(f'logged seat_alpha: circular mean {np.degrees(np.angle(zs)):.1f} deg  R {abs(zs):.2f}  n {len(sa)}')
