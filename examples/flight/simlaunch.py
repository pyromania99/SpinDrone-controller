#!/usr/bin/env python3
"""
| File: simlaunch.py
| Description: Starting and stopping an Isaac + PX4 simulator, in one place.

Both ``fly.py`` (one flight) and ``gain_search.py`` (one launch per candidate)
need the same three things: find Isaac's interpreter, get the lockstep port
clear before starting, and kill the whole process tree afterwards. Keeping one
copy matters because the hard-won part is the cleanup, and a second copy would
be the one that forgets it.
"""
from __future__ import annotations

import os
import signal
import socket
import subprocess
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
ISAAC_PY = Path(os.environ.get("ISAACSIM_PATH", Path.home() / "isaacsim")) / "python.sh"

SIM_PEGASUS = REPO / "examples/flight/sim_pegasus.py"   # level 1
SIM_PX4 = REPO / "examples/flight/sim_px4.py"           # levels 2 and 3

SIM_TCP_PORT = 4560          # PX4 <-> simulator lockstep socket


def port_busy(port: int = SIM_TCP_PORT) -> bool:
    """True if anything still holds the lockstep port."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            s.bind(("127.0.0.1", port))
            return False
        except OSError:
            return True


def reap_px4() -> None:
    """Kill PX4 instances left behind by a previous run.

    PX4LaunchTool starts px4 in its OWN session, so a killpg on the Isaac tree
    never reaches it. The orphan keeps TCP 4560 bound, and a back-to-back run
    then cannot bind - PX4 never boots, every parameter reads None, and
    check_airframe reports "wrong airframe" for what is really a stale socket.
    That misdiagnosis is expensive: the error names the airframe, so it sends you
    looking at SYS_AUTOSTART instead of at process cleanup.
    """
    subprocess.run(["pkill", "-f", r"bin/px4"], check=False,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def wait_port_free(timeout: float = 60.0) -> bool:
    reap_px4()
    deadline = time.time() + timeout
    while time.time() < deadline:
        if not port_busy():
            return True
        time.sleep(2.0)
        reap_px4()
    return not port_busy()


def launch(script: Path, args: list[str], quiet: bool = True,
           log_path: Path | None = None) -> subprocess.Popen:
    """Run an Isaac-side script under Isaac's own interpreter.

    ``start_new_session`` lets the whole tree be killed later: Isaac spawns
    children that outlive a bare terminate().

    ``log_path`` captures the sim's stdout and stderr to a file. Prefer it to
    ``quiet``: a sim that dies during boot is the single most likely failure
    here, and with the output on /dev/null the only symptom is the caller
    waiting out its boot timeout and then reporting that PX4 never appeared -
    which names the symptom and hides the cause.
    """
    if log_path is not None:
        log_path.parent.mkdir(parents=True, exist_ok=True)
        sink = open(log_path, "wb")
    else:
        sink = subprocess.DEVNULL if quiet else None
    return subprocess.Popen([str(ISAAC_PY), str(script), *args],
                            cwd=str(REPO), start_new_session=True,
                            stdout=sink, stderr=sink)


def launch_px4_sim(run_dir: Path, motor_moment: float, duration: float,
                   extra: list[str] | None = None, quiet: bool = True,
                   on_port_busy=None, log_path: Path | None = None,
                   headless: bool = True) -> subprocess.Popen:
    """Start Isaac + PX4 with its ulog kept under ``run_dir``.

    Headless by default: these runs are long and usually unattended. Pass
    ``headless=False`` to bring up the viewport and watch one.
    """
    if not wait_port_free() and on_port_busy is not None:
        on_port_busy(SIM_TCP_PORT)
    run_dir.mkdir(parents=True, exist_ok=True)
    args = ["--keep-log", str(run_dir),
            "--motor-moment", repr(motor_moment), "--duration", repr(duration)]
    if headless:
        args.insert(0, "--headless")
    return launch(SIM_PX4, args + list(extra or []), quiet=quiet, log_path=log_path)


def kill(proc: subprocess.Popen) -> None:
    """Stop a launched sim and everything it spawned."""
    if proc.poll() is not None:
        reap_px4()
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
    reap_px4()
