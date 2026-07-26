#!/usr/bin/env python
"""
| File: gain_seed.py
| Description: Seed a sweep's pinned gains from the best row of a prior archive.

Why
───
Grid sweeps have to pin the genes they are not sweeping. Those pinned values
live as argparse defaults, and defaults go stale: the eigen ab-sweep shipped
``--kp-att 0.7`` while the genetic search had long since established that the
Eigen controller wants Kp_att ≈ 4. Mapping the (a, b) plane through that slice
produced a grid where only 12 % of cells flew at all — a map of a dead region
rather than of the parameter being studied.

This module lets a sweep read its pinned gains out of the best-scoring row of
an existing archive instead, so the operating point tracks what the searches
have actually found.

Cross-controller safety
───────────────────────
``Kp_att``/``Kd_att`` name *different physical quantities* in the two
controllers, and the column names do not warn you:

    EigenController      attitude error → desired angular RATE   [1/s]
                         (cascade continues through M_dyn/B to a torque)
                         searched optimum ≈ Kp 3-4,  Kd 4.6-8.5

    CascadedPDController attitude error → THRUST DIFFERENTIAL    [N/rad]
                         (applied directly as u1 = base ∓ tau_x ± tau_y)
                         searched optimum ≈ Kp 25-55, Kd 0.7-2.1

They differ by roughly an order of magnitude in opposite directions, so
seeding one controller from the other's archive silently reproduces exactly
the failure this module exists to prevent. ``load_best_gains`` therefore
requires the caller to declare which controller it wants and refuses an
archive whose columns say otherwise.
"""
from __future__ import annotations

import csv
import math
from pathlib import Path

# Columns that identify which controller wrote an archive. The Eigen sweeps
# record the M_dyn genes; the PD sweeps record the yaw-rate damping gene.
_EIGEN_MARKERS = ("wn_mdyn", "b_mdyn")
_PD_MARKERS = ("Kd_yaw_rate",)


class SeedError(RuntimeError):
    """Raised when an archive cannot be used to seed the requested gains."""


def _read_rows(path: Path) -> list:
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def _as_float(text):
    if text is None or text == "":
        return None
    try:
        value = float(text)
    except ValueError:
        return None
    return value if math.isfinite(value) else None


def detect_controller(fieldnames) -> str:
    """Return 'eigen', 'pd' or 'unknown' for an archive's column set."""
    names = set(fieldnames or ())
    if any(m in names for m in _PD_MARKERS):
        return "pd"
    if all(m in names for m in _EIGEN_MARKERS):
        return "eigen"
    return "unknown"


def find_archives(root: Path) -> list:
    """Every archive CSV under `root`, or `root` itself if it is a CSV."""
    root = Path(root)
    if root.is_file():
        return [root]
    found = []
    for pattern in ("**/genetic_archive.csv", "**/grid_archive.csv",
                    "**/exploration_archive.csv"):
        found.extend(sorted(root.glob(pattern)))
    return found


def load_best_gains(source, controller: str, keys=("Kp_att", "Kd_att"),
                    score_column: str = "score", max_score: float = None) -> dict:
    """Best-scoring row's `keys` from the archive(s) under `source`.

    source      archive CSV, or a directory searched for archives.
    controller  'eigen' or 'pd' — the controller the caller is configuring.
                An archive written by the other one is rejected rather than
                silently misapplied (see the module docstring).
    keys        gain columns to extract.
    max_score   optional ceiling; rows above it are ignored.

    Returns {key: value, '_score': ..., '_source': ...}. Raises SeedError if
    nothing usable is found — a sweep should stop rather than fall back to a
    stale default it was explicitly asked to replace.
    """
    if controller not in ("eigen", "pd"):
        raise SeedError(f"unknown controller {controller!r}")

    archives = find_archives(Path(source))
    if not archives:
        raise SeedError(f"no archive CSV found under {source}")

    best_row = None
    best_score = math.inf
    best_path = None
    wrong_controller = []

    for path in archives:
        try:
            rows = _read_rows(path)
        except Exception as exc:
            raise SeedError(f"could not read {path}: {exc}") from exc
        if not rows:
            continue

        found = detect_controller(rows[0].keys())
        if found != controller:
            wrong_controller.append(f"{path} (looks like '{found}')")
            continue
        if not all(k in rows[0] for k in keys):
            continue

        for row in rows:
            score = _as_float(row.get(score_column))
            if score is None:
                continue
            if max_score is not None and score > max_score:
                continue
            values = {k: _as_float(row.get(k)) for k in keys}
            if any(v is None for v in values.values()):
                continue
            if score < best_score:
                best_score, best_row, best_path = score, values, path

    if best_row is None:
        detail = ""
        if wrong_controller:
            detail = ("\n  archives found but written by a different "
                      "controller (Kp_att/Kd_att are NOT the same quantity "
                      "across controllers, so these cannot be used):\n    "
                      + "\n    ".join(wrong_controller))
        raise SeedError(
            f"no usable '{controller}' row with columns {list(keys)} under "
            f"{source}{detail}")

    result = dict(best_row)
    result["_score"] = best_score
    result["_source"] = str(best_path)
    return result
