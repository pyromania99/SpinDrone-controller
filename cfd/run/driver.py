"""
| File: driver.py
| License: BSD-3-Clause. Copyright (c) 2026. All rights reserved.
| Description: Meshes, writes and runs aerofoil CFD cases, one mesh per Reynolds number.

The OpenFOAM solver runs inside a container, which is how OpenFOAM is distributed for
Linux. Only the solver is containerised: cases are written to, and read from, the host
filesystem, so results, logs and tables are ordinary files.
"""

from __future__ import annotations

import json
import os
import subprocess
import time
from typing import Dict, List, Optional

IMAGE = "opencfd/openfoam-default:2506"
REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _docker(
    workdir: str, command: str, timeout: int = 7200, extra_mounts: tuple = ()
) -> subprocess.CompletedProcess:
    """Run a command inside the OpenFOAM container.

    ``workdir`` and every path in ``extra_mounts`` are bind-mounted at their own absolute
    paths, so paths mean the same thing inside and outside. The run scripts live in the
    repository while cases live in a scratch directory, so both must be mounted.
    """
    mounts: List[str] = []
    for path in (workdir, *extra_mounts):
        mounts += ["-v", f"{path}:{path}"]
    return subprocess.run(
        [
            "docker", "run", "--rm",
            "-u", f"{os.getuid()}:{os.getgid()}",
            *mounts,
            "-w", workdir,
            IMAGE, "bash", "-lc", command,
        ],
        capture_output=True, text=True, timeout=timeout,
    )


def fix_boundary_types(case_dir: str) -> Dict[str, str]:
    """Correct the patch types that ``gmshToFoam`` cannot infer.

    gmshToFoam tags every physical surface as a generic ``patch``. Two corrections are
    mandatory and neither produces an error if omitted -- the run simply gives wrong
    answers:

    * ``front``/``back`` must be ``empty``, which is how OpenFOAM is told the case is
      two-dimensional. Left as ``patch``, the solver computes a three-dimensional flow one
      cell deep with slip on both faces.
    * ``aerofoil`` must be ``wall``. Wall functions, y+ reporting and the transition
      model's wall-distance treatment all key off the patch being a wall; as a plain
      ``patch`` it is silently treated as a generic boundary.

    Returns:
        Mapping of patch name to the type it was set to.
    """
    import re

    path = os.path.join(case_dir, "constant", "polyMesh", "boundary")
    if not os.path.isfile(path):
        raise FileNotFoundError(f"{path}: gmshToFoam did not produce a boundary file")

    text = open(path).read()
    applied: Dict[str, str] = {}
    for name, wanted in (("front", "empty"), ("back", "empty"),
                         ("aerofoil", "wall"), ("farfield", "patch"),
                         ("outlet", "patch")):
        pattern = r"(\b%s\s*\n\s*\{[^}]*?\btype\s+)\w+;" % re.escape(name)
        text, n = re.subn(pattern, r"\1%s;" % wanted, text, flags=re.S)
        if n:
            applied[name] = wanted
    open(path, "w").write(text)

    missing = {"front", "back", "aerofoil"} - set(applied)
    if missing:
        raise ValueError(
            f"{path}: patches {sorted(missing)} not found. The mesh's physical groups do "
            f"not match what the case expects, so the case would run with wrong boundary "
            f"conditions."
        )
    return applied


def run_case(case_dir: str, n_procs: int = 8, timeout: int = 7200) -> Dict:
    """Convert the mesh, correct boundary types, then solve.

    Args:
        case_dir: An existing case written by :func:`run.case.write_case`.
        n_procs: MPI ranks. Eight physical cores are available.
        timeout: Wall-clock limit for the solve, in seconds.

    Returns:
        Dict with timing, the checkMesh verdict and the solver's convergence state.
    """
    case_dir = os.path.abspath(case_dir)
    parent = os.path.dirname(case_dir)
    t0 = time.time()

    conv = _docker(
        parent, f"bash {REPO}/run/foam_mesh.sh {case_dir}",
        timeout=600, extra_mounts=(REPO,),
    )
    if "converted" not in conv.stdout:
        return {
            "case": case_dir, "stage": "gmshToFoam", "returncode": conv.returncode,
            "seconds": round(time.time() - t0, 1), "mesh_ok": False, "converged": False,
            "stderr_tail": conv.stderr[-800:],
        }

    patches = fix_boundary_types(case_dir)

    solve = _docker(
        parent, f"bash {REPO}/run/foam_solve.sh {case_dir} {n_procs}",
        timeout=timeout, extra_mounts=(REPO,),
    )
    elapsed = time.time() - t0

    def tail(name: str, n: int = 40) -> str:
        path = os.path.join(case_dir, name)
        if not os.path.isfile(path):
            return ""
        with open(path, errors="replace") as handle:
            return "".join(handle.readlines()[-n:])

    return {
        "case": case_dir,
        "stage": "solve",
        "patches": patches,
        "seconds": round(elapsed, 1),
        "returncode": solve.returncode,
        "mesh_ok": "Mesh OK" in tail("log.checkMesh", 400),
        "converged": "SIMPLE solution converged" in tail("log.simpleFoam", 600),
        "checkMesh_tail": tail("log.checkMesh", 14),
        "solver_tail": tail("log.simpleFoam", 25),
        "stderr_tail": solve.stderr[-800:],
    }
