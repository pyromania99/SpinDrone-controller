#!/usr/bin/env python3
"""Spin contribution of the props vs the 10-degree motor tilt, from steady hover segments.

    python3 tilt_spin_analysis.py [--out notes/figures/tilt10]

Writes the figures used by notes/HARDWARE_TILT10_SPIN_ANALYSIS.md and prints every number
quoted there. Steady = motors at hover (mean command > 0.3), spin settled (|r_dot| < 0.3
rad/s^2 over 1 s), height held (|dz/dt| < 0.05 m/s from the mocap z), runs >= 2 s with
1 s trimmed at each end. Where the mocap height is valid, z > 0.3 m (no ground effect).
"""
import sys, os
import numpy as np
from pyulog import ULog
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

LOGS = os.path.expanduser('~/Downloads/logs')
OUT = sys.argv[sys.argv.index('--out') + 1] if '--out' in sys.argv else \
    os.path.expanduser('~/PegasusSimulator/notes/figures/tilt10')
os.makedirs(OUT, exist_ok=True)

# (label, n_tilted, prop sense vs tilt (+1 with, -1 against), path, mocap z usable)
FLIGHTS = [
    ('0 tilted  dunowhatthisis', 0, +1, 'eigen/dunowhatthisis.ulg', True),
    ('0 tilted  spin_24/log_1', 0, +1, 'spin_24/log_1_2026-9-24-12-10-04.ulg', False),
    ('0 tilted  log_3', 0, +1, '../log_3_UnknownDate.ulg', True),
    ('2 tilted  log_1', 2, +1, '30th_2tilt/log_1_2026-9-30-21-50-08.ulg', True),
    ('2 tilted  log_2', 2, +1, '2_10_2026_2_tilt/log_2_UnknownDate.ulg', True),
    ('2 tilted  log_12 (props against)', 2, -1, '30th_2tilted_props_against_tilt/log_12_UnknownDate.ulg', True),
    ('3 tilted  log_5', 3, +1, '3_tilt spin/log_5_UnknownDate.ulg', True),
    ('4 tilted  log_8', 4, +1, '4_tilt/log_8_UnknownDate.ulg', True),
]

def mavg(x, n): return np.convolve(x, np.ones(n) / n, 'same')

def load(path, zok):
    u = ULog(os.path.join(LOGS, path)); t0 = u.start_timestamp
    g = lambda n: u.get_dataset(n).data
    av = g('vehicle_angular_velocity'); ta = (av['timestamp'] - t0) / 1e6
    tu = np.arange(ta[0], ta[-1], 0.02)
    r = np.abs(np.interp(tu, ta, av['xyz[2]']))          # magnitude: untilted builds spin negative
    m = g('actuator_motors')
    cmd = np.interp(tu, (m['timestamp'] - t0) / 1e6,
                    np.nan_to_num(np.mean([m['control[%d]' % i] for i in range(4)], 0)))
    lp = g('vehicle_local_position'); z = -np.interp(tu, (lp['timestamp'] - t0) / 1e6, lp['z'])
    rs = mavg(r, 50); rd = np.gradient(mavg(r, 50), 0.02); dz = np.gradient(mavg(z, 50), 0.02)
    ok = (cmd > 0.3) & (np.abs(rd) < 0.3) & (np.abs(dz) < 0.05) & (rs > 1)
    if zok: ok &= z > 0.3
    sel = np.zeros_like(ok); i = 0
    while i < len(ok):
        if ok[i]:
            j = i
            while j < len(ok) and ok[j]: j += 1
            if (j - i) * 0.02 >= 4: sel[i + 50:j - 50] = True
            i = j
        else: i += 1
    return tu - tu[0], rs, z, sel

rows = []
for lab, n, s, path, zok in FLIGHTS:
    t, rs, z, sel = load(path, zok)
    rows.append(dict(lab=lab, n=n, s=s, t=t, r=rs, z=z, sel=sel,
                     rss=np.median(rs[sel]), sd=rs[sel].std(), dur=sel.sum() * 0.02,
                     zm=np.median(z[sel]) if zok else np.nan))
    print('%-34s steady %5.1f s  r = %5.2f +- %.2f rad/s  z = %s' % (
        lab, rows[-1]['dur'], rows[-1]['rss'], rows[-1]['sd'],
        '%.2f m' % rows[-1]['zm'] if zok else 'n/a (mocap frame offset)'))

# per-build steady spin (flights pooled, props-with only)
build = {}
for n in (0, 2, 3, 4):
    v = [x['rss'] for x in rows if x['n'] == n and x['s'] > 0 and x['dur'] > 0]
    build[n] = np.mean(v)
against = [x['rss'] for x in rows if x['s'] < 0][0]
print('\nbuild means:', {k: round(v, 2) for k, v in build.items()}, ' props-against:', round(against, 2))

# drag = r + k r^2 (units of drive/linear-drag coeff); drive = n*A + B
ns = np.array(sorted(build)); rr = np.array([build[n] for n in ns])
X = np.column_stack([ns, np.ones_like(ns), -rr ** 2]); A, B, k = np.linalg.lstsq(X, rr, rcond=None)[0]
def solve(drive, k):
    return drive if k <= 0 else (-1 + np.sqrt(1 + 4 * k * drive)) / (2 * k)
print('mixed-drag fit: A = %.2f rad/s per tilted motor, B = %.2f rad/s props, k = %.4f' % (A, B, k))
print('  predicted', [round(float(solve(n * A + B, k)), 2) for n in ns], ' measured', np.round(rr, 2))
print('  props-against predicted %.2f, measured %.2f' % (solve(2 * A - B, k), against))
per_motor = (build[4] - build[0]) / 4
print('model-free average per tilted motor (4-tilt minus untilted)/4 = %.2f rad/s' % per_motor)
print('steps: 0->2 %+.2f, 2->3 %+.2f, 3->4 %+.2f' % (build[2] - build[0], build[3] - build[2], build[4] - build[3]))

# projection to larger tilt: tilt drive ~ tan(theta) (T = mg/(4 cos), lateral = T sin),
# prop yaw drive ~ Q cos(theta) with Q ~ T ~ 1/cos -> constant.
th = np.radians(np.linspace(5, 35, 301)); ratio = np.tan(th) / np.tan(np.radians(10))
r_lin = build[0] + 4 * per_motor * ratio                                 # drag linear
r_mix = solve(B + 4 * A * ratio, k)                                      # fitted
B2 = build[0] ** 2; A2 = (build[4] ** 2 - B2) / 4
r_quad = np.sqrt(B2 + 4 * A2 * ratio)                                    # drag quadratic
deg = np.degrees(th)
def angle_for(target, curve): return np.interp(target, curve, deg)
print('\ntilt needed      linear   fitted   quadratic')
for tgt in (16, 20, 22, 24):
    print('  %2d rad/s      %5.1f°   %5.1f°   %5.1f°' % (tgt, angle_for(tgt, r_lin), angle_for(tgt, r_mix), angle_for(tgt, r_quad)))
i20 = np.argmin(abs(deg - 20))
print('at 20°: tan20/tan10 = %.2f  ->  %.1f / %.1f / %.1f rad/s; extra thrust per motor 1/cos = %+.1f%%' % (
    ratio[i20], r_lin[i20], r_mix[i20], r_quad[i20], 100 * (1 / np.cos(np.radians(20)) - 1)))

# ---------------------------------------------------------------- figures
INK, MUTED, GRID, SURF = '#1f1f1e', '#6b6a63', '#e4e3dc', '#fcfcfb'
C = {0: '#2a78d6', 2: '#eb6834', 3: '#1baf7a', 4: '#eda100'}
plt.rcParams.update({'font.size': 10, 'axes.edgecolor': MUTED, 'axes.labelcolor': INK,
                     'xtick.color': MUTED, 'ytick.color': MUTED, 'axes.spines.top': False,
                     'axes.spines.right': False, 'figure.facecolor': SURF, 'axes.facecolor': SURF,
                     'axes.grid': True, 'axes.axisbelow': True, 'grid.color': GRID, 'grid.linewidth': 0.8, 'lines.linewidth': 2})

# 1 - spin time series, one representative flight per build, steady part bold
fig, ax = plt.subplots(figsize=(9, 4.2))
rep = {0: '0 tilted  dunowhatthisis', 2: '2 tilted  log_1', 3: '3 tilted  log_5', 4: '4 tilted  log_8'}
for n, lab in rep.items():
    x = next(x for x in rows if x['lab'] == lab)
    ax.plot(x['t'], x['r'], color=C[n], lw=1, alpha=0.35)
    rs = np.where(x['sel'], x['r'], np.nan); ax.plot(x['t'], rs, color=C[n], lw=2.2,
                                                     label='%d tilted (%s)' % (n, lab.split()[-1]))
    j = np.where(x['sel'])[0][-1]
    ax.annotate('%d tilted  %.1f rad/s' % (n, build[n]), (x['t'][j], build[n]), xytext=(6, 0),
                textcoords='offset points', va='center', color=INK, fontsize=9)
ax.set_xlabel('time in log [s]'); ax.set_ylabel('|yaw rate| [rad/s]')
ax.set_title('Spin per build (10° tilt) - bold = steady segments used', loc='left', color=INK)
ax.legend(frameon=False, loc='upper left', fontsize=8.5); ax.set_xlim(0, 72)
fig.tight_layout(); fig.savefig(f'{OUT}/fig1_spin_timeseries.png', dpi=150); plt.close(fig)

# 2 - steady spin vs number of tilted motors
fig, ax = plt.subplots(figsize=(6.5, 4.2))
nn = np.linspace(0, 4, 50)
ax.plot(nn, build[0] + per_motor * nn, color=MUTED, lw=1.5, ls='--', zorder=1)
ax.text(1.0, build[0] + per_motor * 1.0 + 1.6, '%.2f + %.2f·n  (spin adds)' % (build[0], per_motor),
        color=MUTED, fontsize=9, rotation=27)
for x in rows:
    if x['s'] < 0: continue
    ax.plot(x['n'] + 0.06 * (hash(x['lab']) % 3 - 1), x['rss'], 'o', ms=6, color=C[x['n']],
            mec=SURF, mew=1.5, alpha=0.6, zorder=2)
for n in build:
    ax.plot(n, build[n], 'o', ms=10, color=C[n], mec=SURF, mew=2, zorder=3)
    ax.annotate('%.2f' % build[n], (n, build[n]), xytext=(10, -4), textcoords='offset points',
                color=INK, fontsize=9)
ax.plot(2, against, 'o', ms=10, mfc=SURF, mec=C[2], mew=2, zorder=3)
ax.annotate('props against tilt %.2f\n(model says ~%.1f - unexplained)' % (against, solve(2 * A - B, k)),
            (2, against), xytext=(12, -16), textcoords='offset points', color=MUTED, fontsize=8.5)
ax.set_xticks([0, 1, 2, 3, 4]); ax.set_xlabel('motors tilted 10°'); ax.set_ylabel('steady spin [rad/s]')
ax.set_ylim(0, 14.5); ax.set_xlim(-0.3, 4.6)
ax.set_title('Steady spin vs tilted motors', loc='left', color=INK)
fig.tight_layout(); fig.savefig(f'{OUT}/fig2_spin_vs_tilted_motors.png', dpi=150); plt.close(fig)

# 3 - stacked contribution bars (additive, linear-drag reading)
fig, ax = plt.subplots(figsize=(6.5, 3.6))
for i, n in enumerate((0, 2, 3, 4)):
    ax.barh(i, build[0], color='#2a78d6', height=0.55, edgecolor=SURF, lw=2)
    ax.barh(i, build[n] - build[0], left=build[0], color='#eb6834', height=0.55, edgecolor=SURF, lw=2)
    ax.text(build[n] + 0.2, i, '%.2f' % build[n], va='center', color=INK, fontsize=9)
    if n: ax.text(build[0] + (build[n] - build[0]) / 2, i, '%d × %.2f' % (n, (build[n] - build[0]) / n),
                  va='center', ha='center', color='white', fontsize=8.5)
ax.text(build[0] / 2, 0, 'props %.1f' % build[0], va='center', ha='center', color='white', fontsize=8.5)
ax.set_yticks(range(4)); ax.set_yticklabels(['no tilt', '2 tilted', '3 tilted', '4 tilted'])
ax.set_xlabel('steady spin [rad/s]'); ax.grid(axis='y', visible=False); ax.set_xlim(0, 14)
ax.legend(handles=[plt.Rectangle((0, 0), 1, 1, color='#2a78d6'), plt.Rectangle((0, 0), 1, 1, color='#eb6834')],
          labels=['4 co-rotating props (reaction torque)', 'tilted thrust (per motor shown)'],
          frameon=False, fontsize=8.5, loc='lower right')
ax.set_title('Where the spin comes from (10° tilt)', loc='left', color=INK)
fig.tight_layout(); fig.savefig(f'{OUT}/fig3_contribution.png', dpi=150); plt.close(fig)

# 4 - projection to larger tilt
fig, ax = plt.subplots(figsize=(7, 4.4))
ax.axhspan(20, 24, color='#eb6834', alpha=0.10, lw=0)
ax.text(5.5, 22, 'target 20-24 rad/s', color=INK, fontsize=9, va='center')
ax.fill_between(deg, r_quad, r_lin, color='#2a78d6', alpha=0.12, lw=0)
ax.plot(deg, r_lin, color='#2a78d6', lw=1.5, ls='--'); ax.plot(deg, r_mix, color='#2a78d6', lw=2.2)
ax.plot(deg, r_quad, color='#2a78d6', lw=1.5, ls=':')
ax.annotate('drag linear', (21, np.interp(21, deg, r_lin)), xytext=(-62, 4), textcoords='offset points', color=MUTED, fontsize=8.5)
ax.annotate('fitted', (31, np.interp(31, deg, r_mix)), xytext=(0, -12), textcoords='offset points', color=INK, fontsize=8.5)
ax.annotate('drag quadratic (pessimistic)', (24, np.interp(24, deg, r_quad)), xytext=(0, -14), textcoords='offset points', color=MUTED, fontsize=8.5)
ax.plot(10, build[4], 'o', ms=9, color='#eda100', mec=SURF, mew=2)
ax.annotate('measured 4×10°: %.1f' % build[4], (10, build[4]), xytext=(8, -12), textcoords='offset points', color=INK, fontsize=9)
ax.set_xlabel('tilt angle, all 4 motors [deg]'); ax.set_ylabel('predicted steady spin [rad/s]')
ax.set_xlim(5, 35); ax.set_ylim(0, 36)
ax.set_title('Extrapolating the 10° data: tilt torque ∝ tan θ, prop torque constant', loc='left', color=INK, fontsize=10)
fig.tight_layout(); fig.savefig(f'{OUT}/fig4_tilt_projection.png', dpi=150); plt.close(fig)
print('\nfigures ->', OUT)
