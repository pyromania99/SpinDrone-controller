"""Distance-to-failure scorer for the waypoint-ellipse profile.

A crash is ground contact while STILL in OFFBOARD; touchdown after OFFBOARD ends is the
profile's own AUTO_LAND and is not a failure. Duration is NOT comparable across runs -
the streaming loop stops on a WALL-CLOCK budget, so a slower or a faster-simulating run
gets a different amount of flight. Score on path flown.
"""
import glob,sys,numpy as np
from pyulog import ULog
L_CMD=85.3
def score(d,lab):
    fs=sorted(glob.glob(d+"/**/*.ulg",recursive=True))
    if not fs: print("%-16s no ulog"%lab); return
    u=ULog(fs[0],['vehicle_local_position_groundtruth','vehicle_attitude_groundtruth',
                  'vehicle_status','mc_controller_status'])
    dd={m.name:m.data for m in u.data_list}
    lp=dd['vehicle_local_position_groundtruth']; t0=lp['timestamp'][0]
    t=(lp['timestamp']-t0)/1e6; alt=-np.asarray(lp['z'])
    st=dd['vehicle_status']; tn=(np.asarray(st['timestamp'])-t0)/1e6; ns=np.asarray(st['nav_state'])
    ob=np.where(ns==14)[0]
    if not len(ob): print("%-16s never OFFBOARD"%lab); return
    t0b,t1b=tn[ob[0]],tn[ob[-1]]
    down=np.where((t>t0b+5)&(t<t1b)&(alt<0.5))[0]
    tf=t[down[0]] if len(down) else t1b
    fate="CRASH" if len(down) else "flew to end"
    m=(t>t0b)&(t<tf)
    path=np.sum(np.hypot(np.diff(lp['x'][m]),np.diff(lp['y'][m])))
    spd=np.median(np.hypot(lp['vx'],lp['vy'])[m])
    at=dd.get('vehicle_attitude_groundtruth'); tilt=np.nan
    if at is not None:
        q=np.vstack([at['q[%d]'%i] for i in range(4)]).T
        ta=(np.asarray(at['timestamp'])-t0)/1e6
        z=np.degrees(np.arccos(np.clip(1-2*(q[:,1]**2+q[:,2]**2),-1,1)))
        k=(ta>t0b+3)&(ta<tf-1)
        if k.any(): tilt=np.percentile(z[k],95)
    th=np.nan; mcs=dd.get('mc_controller_status')
    if mcs is not None and 'seat_theta' in mcs:
        tm=(np.asarray(mcs['timestamp'])-t0)/1e6; v=np.asarray(mcs['seat_theta'],float)
        k=(tm>t0b+3)&(tm<tf-1)&np.isfinite(v)
        if k.any(): th=v[k].std(ddof=1)
    print("%-16s %-12s %6.0fs  path %6.1f m (%3.0f%%)  spd %.2f  tilt_p95 %5.2f  th_sd %.3f"
          %(lab,fate,tf-t0b,path,100*path/L_CMD,spd,tilt,th))
if __name__=="__main__":
    for a in sys.argv[1:]:
        d,l=a.split("=",1); score(d,l)
