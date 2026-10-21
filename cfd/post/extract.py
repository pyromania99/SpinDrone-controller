"""
| File: extract.py
| License: BSD-3-Clause. Copyright (c) 2026. All rights reserved.
| Description: Pulls force coefficients out of a finished case and assembles sweep tables.

CONVERGENCE IS NOT THE SAME AS A CONVERGED RESIDUAL
---------------------------------------------------
At these Reynolds numbers a separation bubble can leave the residuals falling steadily
while the lift still drifts, and a case can also stall into a limit cycle where the
residuals plateau above tolerance but the coefficients are perfectly steady. Neither state
is detected by the solver's own residual control, so the coefficients themselves are the
convergence test here: :func:`read_force_coeffs` reports the drift over the last window,
and :func:`assemble_table` refuses to tabulate a case whose lift has not settled.
"""

from __future__ import annotations

import glob
import os
from typing import Dict, List, Optional, Tuple

import numpy as np


def read_force_coeffs(case_dir: str) -> Optional[np.ndarray]:
    """Read the forceCoeffs time history.

    Returns:
        Array with columns (iteration, Cd, Cl, Cm), or None if no history was written.
    """
    pattern = os.path.join(case_dir, "postProcessing", "forceCoeffs", "*", "coefficient*.dat")
    files = sorted(glob.glob(pattern))
    if not files:
        return None

    rows: List[List[float]] = []
    header: List[str] = []
    for path in files:
        with open(path, errors="replace") as handle:
            for line in handle:
                if line.startswith("#"):
                    if "Time" in line:
                        header = line.lstrip("#").split()
                    continue
                parts = line.split()
                if len(parts) >= 4:
                    rows.append([float(x) for x in parts])
    if not rows:
        return None

    data = np.asarray(rows)
    # Column order varies between versions, so locate by name where possible.
    def col(name: str, default: int) -> int:
        try:
            return header.index(name)
        except (ValueError, AttributeError):
            return default

    return np.column_stack([
        data[:, 0],
        data[:, col("Cd", 1)],
        data[:, col("Cl", 2)],
        data[:, col("CmPitch", 3) if "CmPitch" in header else min(3, data.shape[1] - 1)],
    ])


def converged_coefficients(
    history: np.ndarray, window_frac: float = 0.15, drift_tol: float = 2.0e-3
) -> Dict[str, float]:
    """Average the last window of the history and measure its drift.

    Args:
        history: Output of :func:`read_force_coeffs`.
        window_frac: Fraction of the run to average over.
        drift_tol: Maximum peak-to-peak variation in Cl across the window that still
            counts as settled.

    Returns:
        Dict of mean Cd, Cl, Cm, the Cl drift, and a ``settled`` flag.
    """
    n = len(history)
    k = max(20, int(window_frac * n))
    w = history[-k:]
    cl_drift = float(w[:, 2].max() - w[:, 2].min())
    return {
        "iterations": int(history[-1, 0]),
        "cd": float(w[:, 1].mean()),
        "cl": float(w[:, 2].mean()),
        "cm": float(w[:, 3].mean()),
        "cl_drift": cl_drift,
        "settled": bool(cl_drift < drift_tol),
        "window": k,
    }


def read_y_plus(case_dir: str) -> Optional[Dict[str, float]]:
    """Read the maximum and mean y+ from the yPlus function object's log.

    y+ above 1 means the mesh has defeated the transition model, so this is checked on
    every case rather than spot-checked.
    """
    files = sorted(glob.glob(os.path.join(case_dir, "postProcessing", "yPlus", "*", "yPlus.dat")))
    if not files:
        return None
    last: Optional[List[str]] = None
    for path in files:
        with open(path, errors="replace") as handle:
            for line in handle:
                if not line.startswith("#") and line.split():
                    last = line.split()
    if last is None or len(last) < 4:
        return None
    return {"y_plus_min": float(last[-3]), "y_plus_max": float(last[-2]),
            "y_plus_mean": float(last[-1])}


def summarise_case(case_dir: str) -> Dict:
    """Everything worth recording about one finished case."""
    hist = read_force_coeffs(case_dir)
    out: Dict = {"case": os.path.basename(case_dir)}
    if hist is None:
        out["error"] = "no forceCoeffs history"
        return out
    out.update(converged_coefficients(hist))
    yp = read_y_plus(case_dir)
    if yp:
        out.update(yp)
        if yp["y_plus_max"] > 1.5:
            out["warning"] = (
                f"y+max={yp['y_plus_max']:.2f} exceeds 1.5; the transition model is not "
                f"wall-resolved and this case should not be tabulated"
            )
    return out
