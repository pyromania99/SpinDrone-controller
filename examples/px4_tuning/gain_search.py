#!/usr/bin/env python3
"""
gain_search.py — headless automatic gain search for MC_CTRL_ALG=3, one FRESH LAUNCH per
candidate.

WHY RELAUNCH RATHER THAN ONE LONG FLIGHT
────────────────────────────────────────
MC_EIG_* gains apply live, so a single flight *could* evaluate every candidate in ~18 s
each. It is rejected here for three reasons that matter more than the wall clock:

  1. **Identical initial state.** A candidate evaluated after a bad one inherits its
     attitude, its position error and its EKF covariance. On this airframe two flights
     with ZERO parameter differences once gave opposite outcomes, so trial order would
     become a hidden variable.
  2. **No recovery problem.** The continuous version had to fly a known-good gain set
     between trials and abort the sweep if the vehicle would not re-stabilise - which
     means one early bad candidate can end the run.
  3. **A ulog per trial.** Scoring reads the log at 250 Hz with the same reduction
     flight_health.py uses, instead of a few hundred MAVLink samples. Coning growth,
     torque saturation and attitude-vs-ground-truth all become measurable per candidate
     rather than approximated.

Cost: a trial is a launch, not a hold window - minutes rather than seconds. Budget the
sweep accordingly, and prefer a coarse grid first.

GROUND TRUTH
────────────
--ground-truth sets MC_CTRL_GT=1, so the controller consumes vehicle_attitude_groundtruth
and vehicle_local_position_groundtruth instead of EKF2's output. This airframe's spin
ceiling was an ESTIMATOR failure that looked exactly like a control problem for days;
searching against truth means a bad score is genuinely a bad candidate. Re-fly the winner
with MC_CTRL_GT=0 afterwards - the difference between the two scores is what the estimator
costs, which is worth measuring rather than assuming.

Usage
─────
  /home/ani/miniconda3/bin/python3.13 gain_search.py --grid --ground-truth
  /home/ani/miniconda3/bin/python3.13 gain_search.py --candidates c.json --motor-moment 2.7e-7
  /home/ani/miniconda3/bin/python3.13 gain_search.py --grid --dry-run     # print the plan
"""
from __future__ import annotations

import argparse
import itertools
import json
import os
import shutil
import signal
import socket
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from px4_driver import PX4Driver, log  # noqa: E402

REPO = Path(__file__).resolve().parents[2]
ISAAC_PY = Path(os.environ.get("ISAACSIM_PATH", Path.home() / "isaacsim")) / "python.sh"
SIM = REPO / "examples/px4_flight/single_vehicle.py"

GAIN_KEYS = ("MC_EIG_ATT_P", "MC_EIG_ATT_D", "MC_EIG_WN", "MC_EIG_B")


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--motor-moment", type=float, default=2.7e-7,
                   help="r_ss ~ 4.9e7*km. Default 2.7e-7 = 13.2 rad/s, the spin the GA "
                        "front was searched at and where the vehicle currently fails.")
    p.add_argument("--ground-truth", action="store_true",
                   help="MC_CTRL_GT=1 - fly on simulator truth, so a bad score is the gains.")
    p.add_argument("--altitude", type=float, default=6.0)
    p.add_argument("--hold", type=float, default=25.0,
                   help="Seconds held at altitude and scored. Long against the growth "
                        "rates here (+0.05 to +0.5 /s) so a slow divergence is visible.")
    p.add_argument("--boot-timeout", type=float, default=180.0)
    p.add_argument("--grid", action="store_true")
    p.add_argument("--candidates", type=str)
    p.add_argument("--out", type=str, default="gain_search_results.json")
    p.add_argument("--runs-dir", type=str, default="gain_search_runs")
    p.add_argument("--dry-run", action="store_true",
                   help="Print the candidate list and the time budget, launch nothing.")
    p.add_argument("--keep-logs", action="store_true",
                   help="Keep every trial's rootfs. Default deletes all but the ulog.")
    return p.parse_args()


ARGS = parse_args()


def coarse_grid() -> list[dict]:
    """Cascade separation wn/ATT_P >= 3 is enforced rather than searched: GA rank 3 sat at
    1.45, the rate and attitude loops interacted, and it departed in 12 s. That is a
    structural constraint, not something worth spending trials rediscovering."""
    out = []
    for att_p, wn, b, att_d in itertools.product((3.0, 4.68, 7.0), (12.0, 16.34, 24.0),
                                                 (0.0, 13.15), (0.1, 0.4)):
        if wn / att_p < 3.0:
            continue
        out.append({"MC_EIG_ATT_P": att_p, "MC_EIG_WN": wn,
                    "MC_EIG_B": b, "MC_EIG_ATT_D": att_d})
    return out


SIM_TCP_PORT = 4560          # PX4 <-> simulator lockstep socket


def _port_busy(port: int = SIM_TCP_PORT) -> bool:
    """True if anything still holds the lockstep port."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            s.bind(("127.0.0.1", port))
            return False
        except OSError:
            return True


def _reap_px4() -> None:
    """Kill PX4 instances left behind by a previous trial.

    PX4LaunchTool starts px4 in its OWN session, so kill_sim's killpg on the Isaac tree
    never reaches it. The orphan keeps TCP 4560 bound, and because the sweep launches
    trials back-to-back the next one cannot bind - PX4 never boots, every parameter reads
    None, and check_airframe reports "wrong airframe" for what is really a stale socket.
    That misdiagnosis is expensive: the error names the airframe, so it sends you looking
    at SYS_AUTOSTART instead of at process cleanup.
    """
    subprocess.run(["pkill", "-f", r"bin/px4"], check=False,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def _wait_port_free(timeout: float = 60.0) -> bool:
    _reap_px4()
    deadline = time.time() + timeout
    while time.time() < deadline:
        if not _port_busy():
            return True
        time.sleep(2.0)
        _reap_px4()
    return not _port_busy()


def launch_sim(run_dir: Path, duration: float) -> subprocess.Popen:
    """Start Isaac + PX4 headless. --duration bounds the trial so a wedged lockstep
    cannot stall the sweep; start_new_session lets us kill the whole tree, since Isaac
    spawns children that outlive a bare terminate()."""
    if not _wait_port_free():
        log(f"      WARNING: TCP {SIM_TCP_PORT} still held; this trial will likely fail "
            f"to bind and report a bogus airframe mismatch")
    run_dir.mkdir(parents=True, exist_ok=True)
    cmd = [str(ISAAC_PY), str(SIM), "--headless", "--keep-log", str(run_dir),
           "--motor-moment", repr(ARGS.motor_moment), "--duration", repr(duration)]
    return subprocess.Popen(cmd, cwd=str(REPO), start_new_session=True,
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def kill_sim(proc: subprocess.Popen) -> None:
    if proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        proc.wait(timeout=25)
    except Exception:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except Exception:
            pass
    # PX4 lives in its own session and survives the killpg above.
    _reap_px4()


def score_ulog(run_dir: Path) -> dict:
    """Score from the ulog at full rate - the reason for relaunching in the first place."""
    from pyulog import ULog
    logs = sorted(run_dir.glob("log/*/*.ulg"))
    if not logs:
        return {"ok": False, "reason": "no ulog", "cost": float("inf")}

    want = ["vehicle_angular_velocity", "vehicle_local_position", "vehicle_torque_setpoint",
            "actuator_motors", "mc_controller_status", "vehicle_attitude",
            "vehicle_attitude_groundtruth"]
    u = ULog(str(logs[-1]), want)
    d = {x.name: x for x in u.data_list}
    if "vehicle_local_position" not in d or "vehicle_angular_velocity" not in d:
        return {"ok": False, "reason": "topics missing", "cost": float("inf")}

    lp = d["vehicle_local_position"]
    t0 = lp.data["timestamp"][0] * 1e-6
    tl = lp.data["timestamp"] * 1e-6 - t0
    air = tl[lp.data["z"] < -1.0]
    if len(air) < 100:
        return {"ok": False, "reason": "never airborne", "cost": float("inf")}

    lo, hi = air[0] + 2.0, air[-1]
    av = d["vehicle_angular_velocity"]
    t = av.data["timestamp"] * 1e-6 - t0
    m = (t > lo) & (t < hi)
    if m.sum() < 500:
        return {"ok": False, "reason": f"airborne only {air[-1]-air[0]:.1f}s",
                "cost": float("inf"), "airborne": float(air[-1] - air[0])}

    env = np.hypot(av.data["xyz[0]"][m], av.data["xyz[1]"][m])
    coning = float(env.std())
    growth = float(np.polyfit(t[m], np.log(np.maximum(env, 1e-9)), 1)[0])

    tq = d.get("vehicle_torque_setpoint")
    tpk = tprs = float("nan")
    if tq is not None:
        tt = tq.data["timestamp"] * 1e-6 - t0
        mt = (tt > lo) & (tt < hi)
        if mt.sum():
            a = np.abs(tq.data["xyz[0]"][mt]); b = np.abs(tq.data["xyz[1]"][mt])
            tpk = float(max(a.max(), b.max())); tprs = float(max(a.std(), b.std()))

    rails = float("nan")
    am = d.get("actuator_motors")
    if am is not None:
        ta = am.data["timestamp"] * 1e-6 - t0
        ma = (ta > lo) & (ta < hi)
        if ma.sum():
            mo = np.vstack([am.data[f"control[{i}]"][ma] for i in range(4)])
            rails = float(((mo <= 1e-3) | (mo >= 0.999)).mean())

    # Attitude estimate error, when ground truth is present. Not part of the cost - a
    # search against MC_CTRL_GT=1 has no estimator in the loop - but it is what says
    # whether a MC_CTRL_GT=0 result was the gains or the estimate.
    att_err = float("nan")
    if "vehicle_attitude" in d and "vehicle_attitude_groundtruth" in d:
        at, gt = d["vehicle_attitude"], d["vehicle_attitude_groundtruth"]
        ta = at.data["timestamp"] * 1e-6 - t0
        tg = gt.data["timestamp"] * 1e-6 - t0
        ma = (ta > lo) & (ta < hi)
        if ma.sum() > 50:
            qe = np.vstack([at.data[f"q[{i}]"][ma] for i in range(4)])
            qt = np.vstack([np.interp(ta[ma], tg, gt.data[f"q[{i}]"]) for i in range(4)])
            qt /= np.linalg.norm(qt, axis=0)
            dot = np.abs((qe * qt).sum(axis=0)).clip(0, 1)
            att_err = float(np.degrees(2 * np.arccos(dot)).mean())

    alg = fallback = None
    if "mc_controller_status" in d:
        st = d["mc_controller_status"]
        ts = st.data["timestamp"] * 1e-6 - t0
        ms = (ts > air[0]) & (ts < hi)
        if ms.sum():
            alg = sorted({int(a) for a in st.data["algorithm"][ms]})
            fallback = int(max(st.data["fallback_active"][ms]))

    # Growth dominates: a large but steady coning is survivable, a small growing one is
    # not. Saturation is penalised hard because a railed actuator means the score no
    # longer reflects the gains at all.
    cost = (max(growth, 0.0) * 100.0 + coning * 10.0
            + (0.0 if np.isnan(rails) else rails * 200.0)
            + (0.0 if np.isnan(tpk) else max(tpk - 0.5, 0.0) * 20.0))
    if fallback:
        cost += 1000.0

    return {"ok": True, "cost": float(cost), "coning_rms": coning, "growth": growth,
            "torque_peak": tpk, "torque_rms": tprs, "motors_railed": rails,
            "att_err_deg": att_err, "airborne": float(air[-1] - air[0]),
            "algorithm_ran": alg, "fallback": fallback, "ulog": str(logs[-1])}


def run_trial(cand: dict, run_dir: Path) -> dict:
    duration = ARGS.boot_timeout + ARGS.hold + 90.0     # hard bound on a wedged trial
    proc = launch_sim(run_dir, duration)
    try:
        drv = PX4Driver()
        deadline = time.time() + ARGS.boot_timeout
        while time.time() < deadline:                   # PX4 takes a while behind Isaac
            try:
                drv.connect(timeout=15.0)
                break
            except Exception:
                time.sleep(3.0)
        else:
            return {"ok": False, "reason": "PX4 never appeared", "cost": float("inf")}

        drv.check_airframe()
        alg = drv.param_get("MC_CTRL_ALG")
        if alg is None or int(alg) != 3:
            return {"ok": False, "reason": f"MC_CTRL_ALG={alg}, expected 3 (set it in the "
                                           f"airframe; it is deferred to disarm)",
                    "cost": float("inf")}
        if ARGS.ground_truth:
            drv.set_gains(MC_CTRL_GT=1)
        drv.set_gains(**cand)

        if not drv.takeoff(alt=ARGS.altitude, climb_time=14.0):
            return {"ok": False, "reason": "takeoff failed", "cost": float("inf")}
        drv.hold(0, 0, -ARGS.altitude, ARGS.hold)
        drv.land()
        time.sleep(8)
        drv.disarm()
        drv.close()
    except Exception as exc:
        return {"ok": False, "reason": f"{type(exc).__name__}: {exc}", "cost": float("inf")}
    finally:
        kill_sim(proc)
        time.sleep(4)          # let the ulog flush and the ports free before the next run

    return score_ulog(run_dir)


def main() -> int:
    if ARGS.candidates:
        cands = json.loads(Path(ARGS.candidates).read_text())
    elif ARGS.grid:
        cands = coarse_grid()
    else:
        sys.exit("give --grid or --candidates")

    per = ARGS.boot_timeout * 0.4 + ARGS.hold + 40.0     # rough, launch dominates
    log(f"{len(cands)} candidates, one LAUNCH each (~{per/60:.1f} min/trial, "
        f"~{len(cands)*per/3600:.1f} h total)")
    log(f"spin: motor_moment {ARGS.motor_moment:.2e} -> r_ss ~ {4.9e7*ARGS.motor_moment:.1f} rad/s")
    log(f"state source: {'GROUND TRUTH (MC_CTRL_GT=1)' if ARGS.ground_truth else 'EKF2 estimate'}")
    if ARGS.dry_run:
        for i, c in enumerate(cands, 1):
            log(f"  [{i:2d}] {c}  sep={c['MC_EIG_WN']/c['MC_EIG_ATT_P']:.2f}")
        return 0
    if not ISAAC_PY.exists():
        sys.exit(f"{ISAAC_PY} not found - set ISAACSIM_PATH")

    root = Path(ARGS.runs_dir)
    results = []
    for i, cand in enumerate(cands, 1):
        run_dir = root / f"trial_{i:03d}"
        log(f"[{i}/{len(cands)}] {cand}")
        res = run_trial(cand, run_dir)
        res["gains"] = cand
        results.append(res)
        if res.get("ok"):
            log(f"      cost {res['cost']:8.2f}  growth {res['growth']:+.3f}/s  "
                f"coning {res['coning_rms']:.4f}  tau_pk {res['torque_peak']:.3f}  "
                f"railed {100*res['motors_railed']:.1f}%  air {res['airborne']:.0f}s")
        else:
            log(f"      FAILED: {res.get('reason')}")
        Path(ARGS.out).write_text(json.dumps(results, indent=2))

        if not ARGS.keep_logs:      # keep the ulog, drop the rest of the rootfs
            for sub in ("eeprom", "dataman", "parameters.bson", "parameters_backup.bson"):
                p = run_dir / sub
                shutil.rmtree(p, ignore_errors=True) if p.is_dir() else p.unlink(missing_ok=True)

    ranked = sorted((r for r in results if r.get("ok")), key=lambda r: r["cost"])
    log(f"\n{len(ranked)}/{len(results)} trials scored. Best:")
    for r in ranked[:5]:
        log(f"   cost {r['cost']:7.2f}  growth {r['growth']:+.3f}/s  "
            f"coning {r['coning_rms']:.4f}  att_err {r['att_err_deg']:.2f}d  {r['gains']}")
    if ARGS.ground_truth and ranked:
        log("\nRe-fly the winner with MC_CTRL_GT=0; the score difference is the "
            "estimator's cost at this spin.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
