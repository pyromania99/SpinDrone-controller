#!/usr/bin/env python3
# Run another analysis script on a ulog with chosen topics thinned, to ask "would a lower log
# rate have given the same answer?". Usage:
#   thin_rates.py [--rate topic=Hz ...] [--default-set] <script.py> <script args...>
# --default-set thins to the rates SDLOG_PROFILE 0 logged on the KakuteH7 (gyro/torque 50 Hz,
# motors/esc 10 Hz, EV 5 Hz, attitude 20 Hz). Used 2026-10-08: thinning 3_10_1_20/log_1 moved
# Izz_eff 0.0223 -> 0.0267 and EV delay 18.8 -> 24.8 ms (CI 4.5-28.5).
import sys, os, runpy, numpy as np, pyulog
DEFAULT_SET = {'vehicle_angular_velocity': 50, 'vehicle_torque_setpoint': 50, 'actuator_motors': 10,
               'esc_status': 10, 'vehicle_visual_odometry': 5, 'vehicle_attitude': 20}
args = sys.argv[1:]; rates = {}
while args and args[0].startswith('--'):
    flag = args.pop(0)
    if flag == '--default-set': rates.update(DEFAULT_SET)
    elif flag == '--rate': n, hz = args.pop(0).split('='); rates[n] = float(hz)
    else: sys.exit(f'unknown flag {flag}')
_init = pyulog.ULog.__init__
def _thinned(self, *a, **k):
    _init(self, *a, **k)
    for d in self.data_list:
        if d.name in rates:
            t = d.data['timestamp']; keep = np.r_[True, np.diff(t // int(1e6 / rates[d.name])) > 0]
            d.data = {kk: v[keep] for kk, v in d.data.items()}
pyulog.ULog.__init__ = _thinned
sys.argv = args; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
runpy.run_path(args[0], run_name='__main__')
