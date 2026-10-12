#!/usr/bin/env python3
# Usage: rpm_lag_from_bdshot.py <log.ulg>   (needs esc_status from BDShot and actuator_motors)
# Motor lag from actuator_motors -> esc rpm, fitted INSIDE contiguous log runs only.
# Validated 2026-10-08 on 3_10_1_20/log_1: same L+T with esc_status thinned to 50 or 20 Hz.
# Model: T*dy/dt = K*u(t-L) + b - y. Output-error simulation from the measured y at
# (run start + Lmax), linear in (K,b) for fixed (L,T), so those are solved per grid cell.
import sys, numpy as np
from pyulog import ULog
u=ULog(sys.argv[1]); g=lambda n:u.get_dataset(n).data
ld=g('vehicle_land_detected'); air=ld['timestamp'][ld['landed']==0]; A0,A1=(air[0]+3e6)/1e6,(air[-1]-5e6)/1e6
mo=g('actuator_motors'); es=g('esc_status')
tm=mo['timestamp']/1e6; te=es['timestamp']/1e6
GAP=0.02; LMAX=0.045
br=np.where(np.diff(tm)>GAP)[0]; st=np.r_[tm[0],tm[br+1]]; en=np.r_[tm[br],tm[-1]]
# Gap-free logs are one long run: clip every run to the airborne window and cut it into 1 s pieces.
runs=[(x,min(x+1.0,b,A1)) for a,b in zip(st,en) for x in np.arange(max(a,A0),min(b,A1),1.0)]
runs=[(a,b) for a,b in runs if b-a>LMAX+0.03]
print('runs used',len(runs),'mean len ms',np.mean([b-a for a,b in runs])*1e3)
dt=0.001
Ls=np.arange(0,0.0451,0.0025); Ts=np.array([0.005,0.01,0.015,0.02,0.025,0.03,0.04,0.05,0.06,0.08])
for i in range(4):
  U=mo[f'control[{i}]']; Y=es[f'esc[{i}].esc_rpm'].astype(float)
  best=None; grid=np.zeros((len(Ls),len(Ts)))
  for li,L in enumerate(Ls):
    for ti,T in enumerate(Ts):
      rows_y=[];rows_x=[]
      for a,b in runs:
        ts=np.arange(a+LMAX,b,dt)
        ue=np.interp(ts-L,tm,U)
        ye=te[(te>=a+LMAX)&(te<=b)]
        if len(ye)<4: continue
        y0=np.interp(ts[0],te,Y)
        al=dt/T; fu=np.zeros_like(ts); f1=np.zeros_like(ts); phi=np.ones_like(ts)
        for k in range(1,len(ts)):
          fu[k]=fu[k-1]+al*(ue[k-1]-fu[k-1]); f1[k]=f1[k-1]+al*(1-f1[k-1]); phi[k]=phi[k-1]*(1-al)
        yo=Y[(te>=a+LMAX)&(te<=b)]
        rows_y.append(yo-y0*np.interp(ye,ts,phi)); rows_x.append(np.stack([np.interp(ye,ts,fu),np.interp(ye,ts,f1)],1))
      Yv=np.concatenate(rows_y); Xv=np.concatenate(rows_x)
      c,*_=np.linalg.lstsq(Xv,Yv,rcond=None); res=Yv-Xv@c
      # score on the change from y0 only
      grid[li,ti]=np.mean(res**2)
      if best is None or grid[li,ti]<best[0]: best=(grid[li,ti],L,T,c)
  li,ti=np.unravel_index(np.argmin(grid),grid.shape)
  # effective delay (L+T) spread among cells within 5% of best
  near=[(Ls[a]+Ts[b]) for a in range(len(Ls)) for b in range(len(Ts)) if grid[a,b]<=1.05*grid.min()]
  print(f'motor {i}: best L {best[1]*1e3:4.1f} ms T {best[2]*1e3:4.0f} ms  K {best[3][0]:7.0f} rpm/unit  rms res {np.sqrt(best[0]):5.0f} rpm | L+T within 5%: {min(near)*1e3:.0f}-{max(near)*1e3:.0f} ms')
