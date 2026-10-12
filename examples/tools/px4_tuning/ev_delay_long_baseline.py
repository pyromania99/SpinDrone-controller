#!/usr/bin/env python3
# Validated 2026-10-02: log_1 -> 15.8 ms; +20 ms injected -> +19.4 recovered; cutting log_1 to
# 90/80 ms gaps changes it by 0.3 ms. Usage: ev_delay_long_baseline.py <log.ulg> [--chop] [--inject S]
# EV delay from LONG baselines. A delay tau makes EV attitude lag the gyro by an angle
# whose rate-of-change is set by changes in omega; over 0.5-3 s baselines with the spin
# varying, the increment mismatch is ~tau*|dw|. Gyro z bias and scale are fitted per tau
# (linear in dt and in the gyro increment). Gaps in the gyro are bridged by linear
# interpolation on a 1 ms grid. Options: --chop (cut gap-free log 90/80 ms), --inject S
# (add S seconds to EV stamps, to check recovery).
import sys, numpy as np
sys.path.insert(0,'/home/ani/PegasusSimulator/examples/tools/px4_tuning')
import ev_delay_from_dynamics as E
args=sys.argv[1:]; path=args[0]; chop='--chop' in args
inj=float(args[args.index('--inject')+1]) if '--inject' in args else 0.0
tev,qev,rc,tg,w,par=E.load(path); tev=tev+inj
from pyulog import ULog
ld=ULog(path,['vehicle_land_detected']).get_dataset('vehicle_land_detected').data
air=ld['timestamp'][ld['landed']==0]/1e6
if chop:
  k=((tg-tg[0])%0.17)<0.09; tg,w=tg[k],w[k]
grid=np.arange(tg[0],tg[-1],0.001)
wg=np.stack([np.interp(grid,tg,w[:,j]) for j in range(3)],1)
Q=E.integrate(grid,wg)
# EV samples inside the flight with gyro coverage nearby
m=(tev>air[0]-1)&(tev<air[-1]-1)&(tev>grid[0]+0.2)&(tev<grid[-1]-0.2)
tev,qev=tev[m],qev[m]
rng=np.random.default_rng(0)
n=len(tev); i1=rng.integers(0,n,40000); dtp=rng.uniform(0.5,3.0,40000)
i2=np.searchsorted(tev,tev[i1]+dtp); ok=i2<n; i1,i2=i1[ok],i2[ok]
ok=np.abs(tev[i2]-tev[i1]-dtp[ok])<0.05; i1,i2=i1[ok],i2[ok]
dev=E.qmul(E.qconj(qev[i1]),qev[i2])
TA=np.arange(-0.02,0.0801,0.002); cost=[]; sel=None
def resid(tau):
  dg=E.qmul(E.qconj(E.qat(grid,Q,tev[i1]-tau)),E.qat(grid,Q,tev[i2]-tau))
  err=E.rotvec(E.qmul(E.qconj(dg),dev))      # body-frame mismatch at t1
  ginc=E.rotvec(dg); T=(tev[i2]-tev[i1])
  out=[]
  for j in range(3):                          # bias*dt + scale*increment per axis
    X=np.stack([T,ginc[:,j]],1); c,*_=np.linalg.lstsq(X,err[:,j],rcond=None); out.append(err[:,j]-X@c)
  return np.stack(out,1)
R0=resid(0.012); good=np.linalg.norm(R0,axis=1)<5*np.median(np.linalg.norm(R0,axis=1))
for tau in TA:
  r=resid(tau)[good]; cost.append(np.mean(np.sum(r**2,1)))
cost=np.array(cost); k=np.argmin(cost)
# parabola refine
if 0<k<len(TA)-1:
  a,b,c=np.polyfit(TA[k-1:k+2],cost[k-1:k+2],2); tbest=-b/(2*a)
else: tbest=TA[k]
# bootstrap over 2 s time blocks
blk=np.floor((tev[i1][good]-tev[0])/2).astype(int); ub=np.unique(blk)
R=np.stack([np.sum(resid(t)[good]**2,1) for t in TA],0)
bs=[]
for _ in range(200):
  pick=rng.choice(ub,len(ub)); wts=np.bincount(np.searchsorted(ub,pick),minlength=len(ub))[np.searchsorted(ub,blk)]
  cb=(R*wts).sum(1); kk=np.argmin(cb); bs.append(TA[kk])
fc=par.get('IMU_GYRO_CUTOFF',40.0); Tg=np.sqrt(2)/(2*np.pi*fc)
lo,hi=np.percentile(bs,[2.5,97.5])
print(f'{path.split("/")[-1]} chop={chop} inject={inj*1e3:.0f}ms  pairs {good.sum()}  EV {1/np.median(np.diff(tev)):.0f} Hz')
print(f'  vs filtered gyro: {tbest*1e3:5.1f} ms (block-boot 95% {lo*1e3:.0f}..{hi*1e3:.0f})  +LPF {Tg*1e3:.1f} -> EV DELAY {(tbest+Tg)*1e3:5.1f} ms   [EKF2_EV_DELAY {par.get("EKF2_EV_DELAY")}]')
print('  cost curve (ms:rel)',' '.join(f'{t*1e3:.0f}:{c/cost.min():.2f}' for t,c in zip(TA[::5],cost[::5])))
