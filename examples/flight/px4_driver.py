#!/usr/bin/env python3
"""
| File: px4_driver.py
| Description: MAVLink control of a PX4 SITL vehicle, plus live parameter access.

The flying half of the tuning harness. The MAVLink sequencing here is lifted from
PX4-Autopilot/src/modules/mc_controller/tools/flight_profile.py, which encodes two
environment facts that are painful to rediscover:

  1. Headless SITL will not arm unless something is heartbeating as a GCS, or
     preflight fails with "No connection to the GCS".
  2. MAV_CMD_NAV_TAKEOFF is ACKed but does not climb - it drops into AUTO.LOITER
     and auto-disarms. OFFBOARD must be entered BEFORE arming, with setpoints
     already streaming, and the climb flown as an offboard setpoint.

Parameters go over the px4-* daemon socket rather than MAVLink PARAM_SET, because
the socket path is synchronous and gives an immediate read-back. Gains apply live:
MulticopterController::handleParameterUpdate defers only MC_CTRL_ALG to disarm,
while CascadedPdController::updateParams re-reads MC_PD_* on every parameter_update.
That is what lets one continuous flight serve a whole sweep, with no respawn and no
EKF-disrupting teleport between trials.

arm() refuses unless check_airframe() confirms 10016_none_iris actually loaded. Every
ulog retained on disk so far is SYS_AUTOSTART=4001 (a Gazebo x500 on eigen_params.c
defaults), which flies and logs perfectly happily while telling you nothing about the
Pegasus vehicle. Pass strict=False to check_airframe() to warn instead of raise.
"""
from __future__ import annotations

import subprocess
import threading
import time

from pymavlink import mavutil

DEFAULT_CONN = "udp:127.0.0.1:14540"
PX4_BIN = "/home/ani/PX4-Autopilot/build/px4_sitl_default/bin"

# position + yaw; velocity/accel/yawrate ignored
_MASK_POS_YAW = 0b0000101111111000
# Position AND velocity, yaw held. Bits are "ignore" flags: clearing the three velocity
# bits sends a velocity FEEDFORWARD alongside the position setpoint.
#
# Without it the controller's D term has no velocity reference and acts as pure damping
# against the motion, so sustaining a v m/s orbit needs a standing position error of
# (kd*v + a_centripetal)/kp. At level 2's MC_EIG_XY_P=0.5 / XY_D=1.0 that is ~3.4 m for a
# 1.2 m/s circle - larger than the 3 m radius, i.e. the vehicle simply cannot follow and
# sits near the origin while the reference orbits away (measured 2026-09-13).
_MASK_POSVEL_YAW = 0b0000101111000000

# PX4 main modes for MAV_CMD_DO_SET_MODE param2 (custom mode enabled).
_CUSTOM_MODE_ACRO = 5
_CUSTOM_MODE_OFFBOARD = 6
_CUSTOM_MODE_STABILIZED = 7

# Which mc_controller ControlLevel each mode resolves to, per
# ControlLevelResolver::resolveLevel() and Commander.cpp:2652. The level is what
# decides how much of the control law is under test:
#
#   STABILIZED -> Attitude   angleToRateSetpoint + eigenTorque    (no position stage)
#   ACRO       -> BodyRate   eigenTorque only                     (no angle stage either)
#   OFFBOARD   -> Trajectory the whole law
#
# ALTCTL is NOT a way to drop the position stage: it sets flag_control_altitude_enabled
# and flag_control_climb_rate_enabled, and Commander derives
# flag_multicopter_position_control_enabled from those, so it resolves to Trajectory just
# like POSCTL. Altitude hold in PX4 *is* the outer loop. To hold altitude without it, fly
# STABILIZED and close a slow loop on the throttle stick from here - see hold_stabilized().
MODE_LEVEL = {"STABILIZED": "Attitude", "ACRO": "BodyRate", "OFFBOARD": "Trajectory"}


def log(msg: str) -> None:
    print(f"[px4] {msg}", flush=True)


class PX4Driver:
    """Drives one SITL instance: connect, arm, fly offboard setpoints, set params."""

    def __init__(self, conn: str = DEFAULT_CONN, instance: int = 0):
        self.thrust_scale = 1.0   # set by offboard.fly() when --thrust-scale is used
        self.instance = instance
        self.conn_str = conn
        self.m = None
        self._stop_hb = False
        self._hb_thread = None

    # ---- lifecycle ------------------------------------------------------

    def connect(self, timeout: float = 60.0) -> "PX4Driver":
        log(f"connecting to {self.conn_str}")
        self.m = mavutil.mavlink_connection(self.conn_str)
        self.m.wait_heartbeat(timeout=timeout)
        log(f"heartbeat from system {self.m.target_system}")
        self._start_gcs_heartbeat()
        return self

    def _start_gcs_heartbeat(self) -> None:
        def _hb():
            while not self._stop_hb:
                self.m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                          mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                time.sleep(1.0)

        self._hb_thread = threading.Thread(target=_hb, daemon=True)
        self._hb_thread.start()

    def close(self) -> None:
        self._stop_hb = True
        if self.m is not None:
            self.m.close()

    def __enter__(self):
        return self.connect()

    def __exit__(self, *exc):
        self.close()
        return False

    # ---- parameters (daemon socket, not MAVLink) ------------------------

    def param_set(self, name: str, value: float) -> None:
        subprocess.run([f"{PX4_BIN}/px4-param", "--instance", str(self.instance),
                        "set", name, str(value)],
                       capture_output=True, text=True, timeout=10)

    def param_get(self, name: str) -> float | None:
        out = subprocess.run([f"{PX4_BIN}/px4-param", "--instance", str(self.instance),
                              "show", name],
                             capture_output=True, text=True, timeout=10).stdout
        for line in out.splitlines():
            if name in line and ":" in line:
                try:
                    return float(line.split(":")[-1].strip())
                except ValueError:
                    pass
        return None

    def set_gains(self, **gains: float) -> dict:
        """Set params and read them back. Read-back is the point: a silently
        rejected value (out of range, misspelled) would otherwise be scored as if
        it had been flown, quietly poisoning the archive."""
        for k, v in gains.items():
            self.param_set(k, v)
        time.sleep(0.3)
        got = {k: self.param_get(k) for k in gains}
        for k, want in gains.items():
            if got[k] is None or abs(got[k] - want) > 1e-3:
                raise RuntimeError(f"{k}: asked {want}, got {got[k]} - rejected?")
        return got

    # ---- preflight ------------------------------------------------------

    # What 10016_none_iris must have put in place, and what it means if it did not.
    # Values are (expected, tolerance, why it matters).
    # An expected value of None means "must EXIST", value unchecked.
    AIRFRAME_EXPECT = {
        "SYS_AUTOSTART": (10016, 0,
                          "wrong airframe entirely - none of the Pegasus tuning loaded"),
        "CA_ROTOR0_CT": (10.344, 1e-3,
                         "PX4's thrust model is wrong; torque lands 1.6-2.3x hot"),
        "IMU_GYRO_CUTOFF": (100.0, 1e-3,
                            "gyro low-pass at 40 Hz adds ~6 ms of phase to the rate loop"),
        # EXISTENCE ONLY, deliberately. This was pinned at 0.025 while the airframe set
        # 0.1, so the check failed on value for a correctly loaded airframe. It cannot be
        # a fingerprint in the first place: gain_search sweeps MC_EIG_ATT_D per candidate,
        # so any fixed expectation here is wrong for most of a sweep by construction.
        # What it usefully proves is that mc_controller started and registered its
        # parameters - which is the last thing to happen during boot, and therefore the
        # best single readiness signal we have.
        "MC_EIG_ATT_D": (None, 0.0,
                         "mc_controller never registered its params - module not started"),
    }

    def wait_params(self, names, timeout: float = 90.0) -> bool:
        """Block until every named parameter is readable.

        PX4 registers parameters progressively as modules start, so a param read that
        returns None early in boot means "not yet", not "wrong value". Reading once and
        judging produced exactly that confusion: consecutive trials failed on three
        params, then one, then none, purely as a function of how far boot had got.
        """
        deadline = time.time() + timeout
        while time.time() < deadline:
            if all(self.param_get(n) is not None for n in names):
                return True
            time.sleep(2.0)
        return False

    def check_airframe(self, strict: bool = True) -> dict:
        """Verify the Pegasus airframe actually loaded, before anything flies.

        Every ulog retained in PX4-Autopilot/build/px4_sitl_default/rootfs/log/ reports
        SYS_AUTOSTART=4001 - a Gazebo x500 running eigen_params.c's compiled defaults -
        because rcS:41 lets an exported PX4_SYS_AUTOSTART outrank PX4_SIM_MODEL, and
        because a hand-launched PX4 reads the BUILD copy of the airframe rather than the
        checkout. Both failures are silent: PX4 boots, arms and flies. This turns them
        into a refusal to arm instead of days of misattributed tuning.
        """
        # Wait for the params to exist before judging their values, so a still-booting
        # PX4 is not reported as a wrong airframe. Distinguishing the two matters: the
        # fixes are unrelated, and the wrong-airframe wording sends you to SYS_AUTOSTART.
        self.wait_params(list(self.AIRFRAME_EXPECT))

        got = {name: self.param_get(name) for name in self.AIRFRAME_EXPECT}
        missing = [n for n, v in got.items() if v is None]
        if missing:
            raise RuntimeError(
                "PX4 did not finish booting - these parameters never appeared: "
                + ", ".join(missing)
                + "\n  This is NOT an airframe mismatch. Check that PX4 actually started "
                  "(TCP 4560 free? stale px4 process holding it?), not SYS_AUTOSTART.")

        bad = []
        for name, (want, tol, why) in self.AIRFRAME_EXPECT.items():
            have = got[name]
            if want is None:            # existence-only entry, already satisfied above
                continue
            # CA_ROTOR*_CT is the one expectation a deliberate experiment moves:
            # sim_px4.py --thrust-scale K multiplies the plant's thrust constant, and
            # PX4 is then told CT = 10.344*K so its model still matches. Scale the
            # expectation with it, so the guard still catches an UNintended mismatch -
            # which is what it is for - without blocking an intended one.
            if name.startswith("CA_ROTOR") and name.endswith("_CT"):
                want = want * self.thrust_scale
                tol = max(tol, abs(want) * 1e-4)
            if abs(have - want) > tol:
                bad.append(f"  {name}: got {have}, expected {want} -> {why}")

        if not bad:
            log(f"airframe OK (SYS_AUTOSTART={got['SYS_AUTOSTART']:.0f})")
            return got

        detail = "\n".join(bad)
        msg = ("airframe check FAILED - this is not the Pegasus Iris:\n" + detail +
               "\n  fix: unset PX4_SYS_AUTOSTART, launch via sim_px4.py "
               "(--keep-log), and confirm PX4_SIM_MODEL=none_iris")
        if strict:
            raise RuntimeError(msg)
        log(msg)
        return got

    # ---- flight ---------------------------------------------------------

    def wait_ready(self, timeout: float = 180.0) -> bool:
        log("waiting for EKF / home position...")
        deadline = time.time() + timeout
        got_fix = False
        while time.time() < deadline:
            msg = self.m.recv_match(type=["LOCAL_POSITION_NED", "GPS_RAW_INT"],
                                    blocking=True, timeout=5)
            if msg is None:
                continue
            if msg.get_type() == "GPS_RAW_INT" and msg.fix_type >= 3:
                got_fix = True
            if got_fix and msg.get_type() == "LOCAL_POSITION_NED":
                log("EKF ready")
                time.sleep(5)
                return True
        log("TIMEOUT waiting for EKF")
        return False

    def _cmd(self, command, *params):
        self.m.mav.command_long_send(self.m.target_system, self.m.target_component,
                                     command, 0, *(list(params) + [0] * (7 - len(params))))
        return self.m.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)

    # ---- failure injection ----------------------------------------------

    # PX4's own hook, from msg/versioned/VehicleCommand.msg. Spelled numerically
    # because pymavlink's default dialect (ardupilotmega v1.0) does not carry
    # MAV_CMD_INJECT_FAILURE, and importing a second dialect to name a constant
    # is a worse trade than a comment.
    CMD_INJECT_FAILURE = 420
    FAILURE_UNIT_SYSTEM_MOTOR = 101
    FAILURE_TYPE_OK = 0
    FAILURE_TYPE_OFF = 1

    def inject_motor_failure(self, instance: int, off: bool = True) -> list:
        """Stop (or restore) one motor through PX4's failure injector.

        `instance` is PX4's 1-based motor number, matching the `-i` of the `failure`
        console command and the "motor N off" it prints; 0 means every motor.

        The chain is FailureInjector (commander) -> failure_detector_status.
        motor_stop_mask -> ControlAllocator, which NaNs that entry of
        actuator_motors. Two preconditions, both checked by offboard.fly():

          SYS_FAILURE_EN=1    read ONCE in FailureInjector's constructor, so it has
                              to be set before commander starts. init.d-posix/rcS
                              already defaults it to 1; hardware boards do not.
          CA_FAILURE_MODE>0   ControlAllocator::check_for_motor_failures() returns
                              immediately at the default of 0, silently dropping the
                              mask. This is the one that bites.

        Returns every COMMAND_ACK seen, because there are normally TWO: commander
        does not handle 420 and answers UNSUPPORTED from its default branch, while
        FailureInjector publishes its own ACCEPTED. Neither is proof on its own -
        the mask reaching the allocator is what matters, and that is visible in the
        ulog (failure_detector_status.motor_stop_mask, actuator_motors.control).
        """
        self.m.mav.command_long_send(
            self.m.target_system, self.m.target_component,
            self.CMD_INJECT_FAILURE, 0,
            self.FAILURE_UNIT_SYSTEM_MOTOR,
            self.FAILURE_TYPE_OFF if off else self.FAILURE_TYPE_OK,
            float(instance), 0, 0, 0, 0)

        # Collect for a bounded window rather than taking the first ack: the first
        # one back is usually commander's UNSUPPORTED, and reporting that alone
        # would read as a failed injection when the motor did in fact stop.
        acks, deadline = [], time.time() + 2.0
        while time.time() < deadline:
            ack = self.m.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.5)
            if ack is not None and ack.command == self.CMD_INJECT_FAILURE:
                acks.append(ack.result)
        return acks

    def send_setpoint(self, n: float, e: float, d: float, yaw: float = 0.0) -> None:
        self.m.mav.set_position_target_local_ned_send(
            int(time.time() * 1e3) & 0xFFFFFFFF,
            self.m.target_system, self.m.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            _MASK_POS_YAW, n, e, d, 0, 0, 0, 0, 0, 0, yaw, 0)

    def send_setpoint_vel(self, n: float, e: float, d: float,
                          vn: float, ve: float, vd: float, yaw: float = 0.0) -> None:
        """Position setpoint WITH a velocity feedforward. See _MASK_POSVEL_YAW."""
        self.m.mav.set_position_target_local_ned_send(
            int(time.time() * 1e3) & 0xFFFFFFFF,
            self.m.target_system, self.m.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            _MASK_POSVEL_YAW, n, e, d, vn, ve, vd, 0, 0, 0, yaw, 0)

    def hold(self, n: float, e: float, d: float, seconds: float, yaw: float = 0.0) -> None:
        """Stream a setpoint for `seconds`. Streaming must not stop while in
        OFFBOARD - PX4 fails safe out of the mode after ~0.5 s of silence."""
        end = time.time() + seconds
        while time.time() < end:
            self.send_setpoint(n, e, d, yaw)
            time.sleep(0.05)

    def fly_trajectory(self, traj, waypoint_radius: float = 0.0,
                      yaw: float = 0.0, timeout_pad: float = 30.0,
                      max_seconds: float = 240.0, on_sample=None) -> bool:
        """Stream a position trajectory. Mirrors the level-1 controller's two advance
        rules so the two rungs run the SAME test rather than two different ones.

        traj: (N, 4) array of [t, north, east, down]. `down` is NED, so altitude
              is negative.

        waypoint_radius == 0  TIMED: the reference marches on the CSV's own clock.
              Use only for a SMOOTH, densely-sampled path whose demanded speed the
              vehicle can actually hold - a timed reference does not wait, so if the
              vehicle falls behind the error compounds (measured at level 1: a timed
              translate put the vehicle on the ground with 3.2 m of error).

        waypoint_radius > 0   ARRIVAL: advance only once the vehicle is within the
              radius of the current point. The reference WAITS, which caps position
              error at roughly the radius by construction.

        Returns False if the vehicle is lost (altitude collapse) partway.
        """
        import numpy as _np
        traj = _np.asarray(traj, dtype=float)
        idx = 0
        n_pts = len(traj)
        t0 = time.time()
        # In ARRIVAL mode the CSV's first column is just an index - the vehicle sets the
        # pace, not the file - so the budget has to be a wall-clock cap, not the file's
        # own end time. Using the latter capped a 40 m run at 50 s and reported "complete"
        # after 2.5 m.
        deadline = t0 + (max_seconds if waypoint_radius > 0.0
                         else (float(traj[-1, 0]) + timeout_pad) * 4.0)

        # SIMULATION CLOCK, not wall clock. The sim does not run in real time - measured
        # 77 s of sim in ~150 s of wall - so pacing a timed trajectory off time.time()
        # marches the reference through the path ~1.5x too fast. The vehicle then cannot
        # follow by construction: the position-error vector spins, the demanded
        # acceleration averages out, and the vehicle sits near the middle looking like it
        # "cannot translate". PX4's own time_boot_ms is the clock the vehicle lives on.
        sim_ms0 = None
        sim_ms = None
        last_pos = None

        # Finite-difference the path once, up front, for the velocity feedforward.
        vel = _np.zeros((n_pts, 3))
        if n_pts > 2 and waypoint_radius <= 0.0:
            tt = traj[:, 0]
            for k in range(1, 4):
                vel[:, k - 1] = _np.gradient(traj[:, k], tt)

        while idx < n_pts and time.time() < deadline:
            row = traj[idx]
            if waypoint_radius > 0.0:
                # Arrival mode: the vehicle sets the pace, so the file's own derivative is
                # not the speed it should be flying - send position only.
                self.send_setpoint(row[1], row[2], row[3], yaw)
            else:
                v = vel[idx]
                self.send_setpoint_vel(row[1], row[2], row[3], v[0], v[1], v[2], yaw)

            if waypoint_radius > 0.0:
                # Arrival-based. Drain to the newest sample - pymavlink queues, and a
                # stale position here would advance the index on where the vehicle WAS.
                pos = None
                while True:
                    m = self.m.recv_match(type="LOCAL_POSITION_NED", blocking=False)
                    if m is None:
                        break
                    pos = (m.x, m.y, m.z)
                if pos is not None:
                    last_pos = pos
                if last_pos is not None:
                    # Advance past every point already satisfied, so a cluster of close
                    # waypoints does not take one streaming tick each.
                    while idx < n_pts - 1:
                        r = traj[idx]
                        if _np.hypot(last_pos[0] - r[1], last_pos[1] - r[2]) > waypoint_radius:
                            break
                        idx += 1
            else:
                # Drain to the newest sim timestamp available.
                while True:
                    m = self.m.recv_match(type="LOCAL_POSITION_NED", blocking=False)
                    if m is None:
                        break
                    sim_ms = m.time_boot_ms
                    if sim_ms0 is None:
                        sim_ms0 = sim_ms
                if sim_ms is None or sim_ms0 is None:
                    time.sleep(0.02)
                    continue
                elapsed = (sim_ms - sim_ms0) / 1000.0
                while idx < n_pts - 1 and elapsed >= traj[idx + 1, 0]:
                    idx += 1
                if idx >= n_pts - 1 and elapsed >= traj[-1, 0]:
                    break

            if on_sample is not None and last_pos is not None:
                on_sample(time.time() - t0, last_pos, row)

            time.sleep(0.05)

        return True

    def enter_offboard(self, n=0.0, e=0.0, d=0.0, yaw=0.0) -> bool:
        for _ in range(40):
            self.send_setpoint(n, e, d, yaw)
            time.sleep(0.05)
        ack = self._cmd(mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                        _CUSTOM_MODE_OFFBOARD, 0)
        if not ack or ack.result != 0:
            log(f"OFFBOARD REJECTED: {ack}")
            return False
        log("in OFFBOARD")
        return True

    # ---- stabilized / acro: isolating the inner loop ----------------------

    def send_manual(self, roll: float = 0.0, pitch: float = 0.0,
                    yaw: float = 0.0, throttle: float = 0.5) -> None:
        """One MANUAL_CONTROL message. Stick axes are -1..1, throttle 0..1.

        PX4 scales x/y/r by 1/1000 and maps z from 0..1000 onto throttle 0..1
        (mavlink_receiver.cpp, handle_message_manual_control). x is PITCH and y is
        ROLL - the order is not (roll, pitch)."""
        self.m.mav.manual_control_send(
            self.m.target_system,
            int(max(-1.0, min(1.0, pitch)) * 1000),
            int(max(-1.0, min(1.0, roll)) * 1000),
            int(max(0.0, min(1.0, throttle)) * 1000),
            int(max(-1.0, min(1.0, yaw)) * 1000),
            0)

    def enable_joystick(self) -> None:
        """Let MANUAL_CONTROL over mavlink stand in for an RC transmitter.

        Without this SITL has no stick source, so a manual mode either refuses to engage
        or drops straight into an RC-loss failsafe. NAV_RCL_ACT=0 stops a momentary gap
        in our own streaming from ending the run."""
        self.set_gains(COM_RC_IN_MODE=1, NAV_RCL_ACT=0)

    def hold_stabilized(self, seconds: float, altitude: float,
                        roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0,
                        rate_hz: float = 50.0) -> bool:
        """Fly STABILIZED for `seconds`, holding `altitude` from OUT HERE.

        This is the point of the whole method: PX4's own altitude hold routes through the
        position controller and therefore through EigenController's trajectory stage, so
        it cannot be used to isolate the inner loop. Closing the altitude loop on the
        throttle stick instead leaves PX4 at ControlLevel::Attitude - the position stage
        never runs - while keeping the vehicle in the air long enough to measure.

        The loop is deliberately slow. The mode under investigation sits near 11 rad/s
        (1.8 Hz); these gains put this loop below ~0.5 rad/s, more than an order of
        magnitude clear, so it cannot be mistaken for or interfere with what is being
        measured. Throttle authority is clamped near hover for the same reason.
        """
        thr_hover = self.param_get("MPC_THR_HOVER") or 0.58
        kp, kd, clamp = 0.05, 0.03, 0.15      # per metre, per m/s, throttle units

        # Stream before requesting the mode, exactly as OFFBOARD needs: PX4 will not
        # engage a manual mode with no stick source, and would failsafe if it did.
        for _ in range(40):
            self.send_manual(roll, pitch, yaw, thr_hover)
            time.sleep(1.0 / rate_hz)

        if not self.set_mode("STABILIZED"):
            return False

        log(f"STABILIZED for {seconds:.0f}s, external altitude hold at {altitude:.1f} m "
            f"(mc_controller level: {MODE_LEVEL['STABILIZED']} - position stage OFF)")

        end = time.time() + seconds
        throttle = thr_hover
        updates = 0
        while time.time() < end:
            # Drain everything queued and keep the LAST position sample. One
            # recv_match per cycle cannot keep up with the telemetry rate, so the
            # socket backs up and the loop ends up steering on stale data - or, as
            # happened on 2026-08-08, on no data at all: the throttle sat at
            # thr_hover for a 5 m descent into the ground while this loop believed
            # it was holding altitude.
            msg = None
            while True:
                got = self.m.recv_match(type="LOCAL_POSITION_NED", blocking=False)
                if got is None:
                    break
                msg = got
            if msg is not None:
                updates += 1
                error = altitude - (-msg.z)
                # NED: vz > 0 is DESCENDING, so damping the altitude rate needs +kd*vz
                # (climb rate is -vz, and the PD term is -kd*climb_rate). The sign was
                # inverted here, which fed the descent instead of arresting it.
                throttle = thr_hover + max(-clamp, min(clamp, kp * error + kd * msg.vz))
            self.send_manual(roll, pitch, yaw, throttle)
            time.sleep(1.0 / rate_hz)

        # An altitude loop that never saw the altitude is not a test result, it is a
        # crash with extra steps. Say so rather than let the log look like a finding.
        if updates < 0.2 * seconds * rate_hz:
            log(f"WARNING: only {updates} position updates in {seconds:.0f}s of STABILIZED "
                f"- the altitude hold was effectively open loop. Treat this run as void.")
            return False
        return True

    def set_mode(self, name: str) -> bool:
        custom = {"STABILIZED": _CUSTOM_MODE_STABILIZED,
                  "ACRO": _CUSTOM_MODE_ACRO,
                  "OFFBOARD": _CUSTOM_MODE_OFFBOARD}[name]
        ack = self._cmd(mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, custom, 0)
        if not ack or ack.result != 0:
            log(f"{name} REJECTED: {ack}")
            return False
        log(f"in {name} (mc_controller level: {MODE_LEVEL.get(name, '?')})")
        return True

    def arm(self) -> bool:
        # Gate every arm, not just takeoff(): a run that arms by any other path is just as
        # capable of producing an unusable log.
        self.check_airframe()
        log("arming...")
        ack = None
        for _ in range(10):
            ack = self._cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
            if ack and ack.result == 0:
                log("armed")
                return True
            time.sleep(2)
        log(f"ARM FAILED (last ack: {ack})")
        return False

    def disarm(self, force: bool = False) -> None:
        self._cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 21196 if force else 0)

    def land(self) -> None:
        log("landing")
        self._cmd(mavutil.mavlink.MAV_CMD_NAV_LAND)

    def takeoff(self, alt: float = 5.0, climb_time: float = 12.0) -> bool:
        """Offboard climb to `alt` metres. Order matters: setpoints streaming,
        then OFFBOARD, then arm - see the module docstring."""
        if not self.wait_ready():
            return False
        if not self.enter_offboard(0, 0, -alt):
            return False
        if not self.arm():
            return False
        self.hold(0, 0, -alt, climb_time)
        log(f"at {alt:.1f} m")
        return True

    # ---- telemetry ------------------------------------------------------

    def local_position(self, timeout: float = 2.0):
        msg = self.m.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=timeout)
        return (msg.x, msg.y, msg.z) if msg else None

    def attitude(self, timeout: float = 2.0):
        msg = self.m.recv_match(type="ATTITUDE", blocking=True, timeout=timeout)
        return (msg.roll, msg.pitch, msg.yaw, msg.rollspeed, msg.pitchspeed, msg.yawspeed) \
            if msg else None

    def is_diverging(self, tilt_limit_deg: float = 60.0, rate_limit: float = 3.0) -> bool:
        """Cheap abort check for a trial that is departing, so the sweep does not
        spend its budget watching a known-bad gain set tumble."""
        att = self.attitude(timeout=0.5)
        if att is None:
            return False
        roll, pitch, _, p, q, _ = att
        import math
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, math.cos(roll) * math.cos(pitch)))))
        return tilt > tilt_limit_deg or math.hypot(p, q) > rate_limit
