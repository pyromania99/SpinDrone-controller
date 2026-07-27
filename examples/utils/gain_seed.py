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
import re
from pathlib import Path

# Columns that identify which controller wrote an archive. The Eigen sweeps
# record the M_dyn genes; the PD sweeps record the yaw-rate damping gene.
_EIGEN_MARKERS = ("wn_mdyn", "b_mdyn")
_PD_MARKERS = ("Kd_yaw_rate",)


_MM_DIR = re.compile(r"^mm_([-+0-9.eE]+)$")


def _close(a: float, b: float, rtol: float = 1e-6, atol: float = 1e-12) -> bool:
    return abs(a - b) <= atol + rtol * max(abs(a), abs(b))


def _group_from_path(path: Path, group_column: str):
    """Group value encoded in the archive's directory name, or None.

    Only motor_moment is encoded this way (results/<sweep>/mm_<value>/), so
    any other grouping column falls through to the CSV alone.
    """
    if group_column != "motor_moment":
        return None
    for part in path.parts:
        match = _MM_DIR.match(part)
        if match:
            try:
                return float(match.group(1))
            except ValueError:
                return None
    return None


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


def load_best_gains_by_group(source, controller: str, keys=("Kp_att", "Kd_att"),
                             group_column: str = "motor_moment",
                             score_column: str = "score",
                             max_score: float = None) -> dict:
    """Best-scoring row's `keys` for EACH distinct value of `group_column`.

    Returns {group_value: {key: value, ..., '_score': ..., '_source': ...}}.

    A single global best is the wrong pin when the optimum moves with the
    grouping variable. Measured on the Eigen genetic archives, the best
    Kd_att ranges from 4.45 at km=4e-7 to 11.98 at km=1e-7 — a factor of 2.7.
    Pinning every motor moment to whichever one happened to score best overall
    maps the high- and low-spin ends of the sweep at gains that are not theirs.

    Rows whose group value is missing or unparseable are skipped rather than
    pooled into an arbitrary bucket.
    """
    if controller not in ("eigen", "pd"):
        raise SeedError(f"unknown controller {controller!r}")

    archives = find_archives(Path(source))
    if not archives:
        raise SeedError(f"no archive CSV found under {source}")

    best = {}
    wrong_controller = []
    stale = []

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
        if not all(k in rows[0] for k in keys) or group_column not in rows[0]:
            continue

        # The sweeps write one archive per motor moment into an mm_<value>
        # directory, so the path is an independent record of the group value.
        # Archives written before genetic_sweep.py switched to str(mm)
        # formatted this column with "%.6f", which rounds every km below
        # ~5e-7 to "0.000000" — such a file would silently deposit its rows in
        # the km=0 bucket and outrank the real km=0 results. When the column
        # and the path disagree the archive is stale: skip it and carry on
        # with the others rather than seeding from data known to be mislabelled.
        path_group = _group_from_path(path, group_column)
        if path_group is not None and rows:
            column_group = _as_float(rows[0].get(group_column))
            if column_group is not None and not _close(path_group, column_group):
                stale.append(f"{path} (path says {path_group:g}, "
                             f"column says {column_group:g})")
                continue

        for row in rows:
            group = path_group if path_group is not None else _as_float(
                row.get(group_column))
            score = _as_float(row.get(score_column))
            if group is None or score is None:
                continue
            if max_score is not None and score > max_score:
                continue
            values = {k: _as_float(row.get(k)) for k in keys}
            if any(v is None for v in values.values()):
                continue
            if group not in best or score < best[group]["_score"]:
                entry = dict(values)
                entry["_score"] = score
                entry["_source"] = str(path)
                best[group] = entry

    if stale:
        best["_stale_skipped"] = stale

    if not [k for k in best if not isinstance(k, str)]:
        detail = ""
        if wrong_controller:
            detail = ("\n  archives found but written by a different "
                      "controller (Kp_att/Kd_att are NOT the same quantity "
                      "across controllers, so these cannot be used):\n    "
                      + "\n    ".join(wrong_controller))
        if stale:
            detail += ("\n  stale archives skipped (motor_moment column "
                       "disagrees with the mm_<value> directory, so their "
                       "rows cannot be grouped):\n    " + "\n    ".join(stale))
        raise SeedError(
            f"no usable '{controller}' rows with columns {list(keys)} grouped "
            f"by '{group_column}' under {source}{detail}")
    return best


def lookup_group(table: dict, value: float, rtol: float = 1e-6,
                 atol: float = 1e-12) -> dict:
    """Entry in a load_best_gains_by_group table matching `value`.

    Motor moments round-trip through the CSV as text, so keys are compared
    with a tolerance instead of by equality. Raises SeedError listing the
    available values when there is no match — a near-miss must not quietly
    resolve to the wrong operating point.
    """
    for key, entry in table.items():
        if abs(key - value) <= atol + rtol * abs(value):
            return entry
    available = ", ".join(f"{k:g}" for k in sorted(table))
    raise SeedError(
        f"no seed row for {group_repr(value)}; archive covers: {available}")


def group_repr(value: float) -> str:
    return f"{value:g}"
