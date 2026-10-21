"""
| File: airfoil_mesh.py
| License: BSD-3-Clause. Copyright (c) 2026. All rights reserved.
| Description: C-topology 2-D aerofoil mesh generator for OpenFOAM, built with gmsh.

WHY A C-MESH AND NOT AN O-MESH
------------------------------
The blade operates at Re 5e4 to 2.4e5, where a laminar separation bubble on the upper
surface governs both lift and drag. Resolving the bubble's reattachment, and the wake it
feeds, needs cells aligned with the flow downstream of the trailing edge. An O-mesh
wraps the far boundary around the aerofoil and leaves the wake under-resolved, which
shows up directly as drag error.

THE FIRST CELL HEIGHT IS NOT A FREE PARAMETER
---------------------------------------------
``kOmegaSSTLM`` is a low-Reynolds transition model: it integrates to the wall and needs
y+ < 1 with enough layers to resolve the viscous sublayer. Wall functions silently
defeat it -- the solution still converges and the transition location is simply wrong.
``first_layer_height()`` therefore derives y1 from a target y+ and the flat-plate skin
friction correlation rather than taking it as an input.

NON-DIMENSIONALISATION
----------------------
Chord 1 m, freestream 1 m/s, and nu = 1/Re. RANS is Reynolds-similar, so this gives the
requested Reynolds number with force coefficients that normalise by 1/2 rho U^2 c = 0.5
per unit span. One less place to make an arithmetic error.
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np


# =======================================================================================
# Aerofoil geometry
# =======================================================================================

def load_selig(path: str) -> np.ndarray:
    """Read a Selig-format aerofoil file.

    Selig order runs from the trailing edge, forward along the upper surface to the
    leading edge, then back along the lower surface to the trailing edge.

    Args:
        path: Path to the ``.dat`` file. The first line is a name, not data.

    Returns:
        Array of shape (n, 2) of (x, y) in chord units, in the file's own order.

    Raises:
        ValueError: If fewer than 10 coordinate pairs are found, or the chord is not
            normalised to roughly 1.
    """
    pts: List[Tuple[float, float]] = []
    with open(path) as handle:
        for line in handle.readlines()[1:]:
            parts = line.split()
            if len(parts) != 2:
                continue
            try:
                pts.append((float(parts[0]), float(parts[1])))
            except ValueError:
                continue

    if len(pts) < 10:
        raise ValueError(f"{path}: found only {len(pts)} coordinate pairs")

    arr = np.asarray(pts, dtype=float)
    chord = arr[:, 0].max() - arr[:, 0].min()
    if not 0.95 < chord < 1.05:
        raise ValueError(f"{path}: chord is {chord:.3f}, expected a unit-chord aerofoil")
    return arr


def resample_surface(points: np.ndarray, n_surface: int) -> np.ndarray:
    """Redistribute surface points with cosine clustering at the leading and trailing edges.

    The supplied coordinate files carry 30-60 points, far too few to resolve a separation
    bubble. Resampling is done on arc length so the aerofoil's own shape is preserved,
    with a cosine law that concentrates points where the curvature and the pressure
    gradient are largest.

    Args:
        points: (n, 2) surface coordinates in Selig order.
        n_surface: Number of points to produce around the whole section.

    Returns:
        (n_surface, 2) resampled coordinates, closed (first point repeated at the end is
        NOT included; the caller closes the loop).
    """
    closed = points
    if not np.allclose(closed[0], closed[-1], atol=1e-9):
        closed = np.vstack([points, points[0]])

    seg = np.linalg.norm(np.diff(closed, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    s /= s[-1]

    # Cosine clustering in normalised arc length. Both ends of the parameter range sit at
    # the trailing edge, so this clusters at the TE; the midpoint is the leading edge,
    # which the squared sine term clusters as well.
    t = np.linspace(0.0, 1.0, n_surface, endpoint=False)
    s_new = t - (np.sin(2.0 * np.pi * t)) / (2.0 * np.pi) * 0.55

    return np.column_stack([
        np.interp(s_new, s, closed[:, 0]),
        np.interp(s_new, s, closed[:, 1]),
    ])


# =======================================================================================
# Boundary-layer sizing
# =======================================================================================

def first_layer_height(reynolds: float, y_plus: float = 0.7) -> float:
    """First cell height, in chord units, for a target y+.

    Uses the 1/7-power flat-plate correlation Cf = 0.026 Re^(-1/7), which is adequate for
    sizing: the point is to land below y+ = 1, not to predict Cf.

        u_tau = U sqrt(Cf/2),   y1 = y+ nu / u_tau

    With U = 1 and nu = 1/Re this reduces to y1 = y+ / (Re sqrt(Cf/2)).

    Args:
        reynolds: Chord Reynolds number.
        y_plus: Target y+ at the wall. Below 1 for a transition-resolving model.

    Returns:
        First layer height in chord units.
    """
    cf = 0.026 * reynolds ** (-1.0 / 7.0)
    u_tau = np.sqrt(0.5 * cf)
    return float(y_plus / (reynolds * u_tau))


def layer_count(y1: float, growth: float, target_thickness: float) -> int:
    """Number of layers needed to cover a given thickness at a fixed growth ratio."""
    if growth <= 1.0:
        raise ValueError("growth must exceed 1")
    n = np.log1p(target_thickness * (growth - 1.0) / y1) / np.log(growth)
    return int(np.ceil(n))


# =======================================================================================
# Mesh
# =======================================================================================

@dataclass
class MeshConfig:
    """Mesh generation parameters. All lengths in chord units."""

    n_surface: int = 320
    """Points around the aerofoil surface. Drives streamwise resolution of the bubble."""

    bl_thickness_chord: float = 0.08
    """Thickness of the structured layer stack, in chord units.

    The layer COUNT is derived from this, y1 and the growth ratio -- not specified. A
    fixed count with a y1 that shrinks as Re rises silently produces a stack one to two
    chords thick, which wastes cells and distorts the farfield. At Re 1e5 this value
    gives about 38 layers, comfortably covering a boundary layer of roughly 0.02 chord
    plus the separated shear layer above it.
    """

    growth: float = 1.12
    """Layer-to-layer growth ratio. Above about 1.2 the transition model degrades."""

    y_plus: float = 0.7
    """Target y+ for the first cell. Must stay below 1."""

    farfield_radius: float = 60.0
    """Distance to the far boundary ahead of and above the aerofoil."""

    wake_length: float = 80.0
    """Distance to the outlet behind the trailing edge."""

    wake_cells: int = 140
    """Streamwise cells in the wake refinement region."""

    far_cell_size: float = 6.0
    """Target cell size at the far boundary."""

    wake_cell_size: float = 0.012
    """Target cell size in the near wake, where the shear layer rolls up."""

    te_cell_size: float = 0.0015
    """Target cell size at the trailing edge."""

    def validate(self) -> None:
        if self.y_plus >= 1.0:
            raise ValueError(
                f"y_plus={self.y_plus} defeats the low-Re transition model; use < 1"
            )
        if not 1.0 < self.growth <= 1.25:
            raise ValueError(f"growth={self.growth} outside the usable range (1, 1.25]")
        if not 0.02 <= self.bl_thickness_chord <= 0.4:
            raise ValueError(
                f"bl_thickness_chord={self.bl_thickness_chord} is outside the sensible "
                "range; the attached boundary layer here is about 0.02 chord"
            )
        if self.n_surface < 120:
            raise ValueError("n_surface below 120 cannot resolve a separation bubble")
        if self.farfield_radius < 20.0:
            raise ValueError(
                f"farfield_radius={self.farfield_radius} is too close; lift will be wrong"
            )


def build_mesh(
    airfoil_path: str,
    reynolds: float,
    out_dir: str,
    config: Optional[MeshConfig] = None,
    write_msh: str = "mesh.msh",
) -> dict:
    """Generate a 2-D C-topology mesh around an aerofoil and write it as gmsh .msh2.

    The mesh is a single cell thick in z with front and back patches tagged ``empty``,
    which is how OpenFOAM represents a 2-D case.

    Args:
        airfoil_path: Selig ``.dat`` coordinate file.
        reynolds: Chord Reynolds number, used only to size the first cell.
        out_dir: Directory to write into. Created if absent.
        config: Mesh parameters. Defaults to :class:`MeshConfig`.
        write_msh: File name for the mesh inside ``out_dir``.

    Returns:
        A dict of mesh statistics and the derived boundary-layer sizing, for the run
        record.
    """
    import gmsh

    cfg = config or MeshConfig()
    cfg.validate()

    y1 = first_layer_height(reynolds, cfg.y_plus)
    n_normal = layer_count(y1, cfg.growth, cfg.bl_thickness_chord)
    bl_thickness = y1 * (cfg.growth ** n_normal - 1.0) / (cfg.growth - 1.0)

    raw = load_selig(airfoil_path)
    surf = resample_surface(raw, cfg.n_surface)

    os.makedirs(out_dir, exist_ok=True)
    gmsh.initialize()
    gmsh.option.setNumber("General.Terminal", 0)
    gmsh.model.add("airfoil")

    # -- aerofoil surface -------------------------------------------------------------
    pts = [gmsh.model.geo.addPoint(x, y, 0.0, cfg.te_cell_size) for x, y in surf]
    lines = [
        gmsh.model.geo.addLine(pts[i], pts[(i + 1) % len(pts)]) for i in range(len(pts))
    ]
    foil_loop = gmsh.model.geo.addCurveLoop(lines)

    # -- far boundary: a C, i.e. a semicircle upstream closed by a rectangle downstream --
    R = cfg.farfield_radius
    L = cfg.wake_length
    h = cfg.far_cell_size
    c_in = gmsh.model.geo.addPoint(0.0, 0.0, 0.0, h)
    p_top = gmsh.model.geo.addPoint(0.0, R, 0.0, h)
    p_bot = gmsh.model.geo.addPoint(0.0, -R, 0.0, h)
    p_up = gmsh.model.geo.addPoint(-R, 0.0, 0.0, h)
    p_top_out = gmsh.model.geo.addPoint(L, R, 0.0, h)
    p_bot_out = gmsh.model.geo.addPoint(L, -R, 0.0, h)

    arc_top = gmsh.model.geo.addCircleArc(p_top, c_in, p_up)
    arc_bot = gmsh.model.geo.addCircleArc(p_up, c_in, p_bot)
    l_bot = gmsh.model.geo.addLine(p_bot, p_bot_out)
    l_out = gmsh.model.geo.addLine(p_bot_out, p_top_out)
    l_top = gmsh.model.geo.addLine(p_top_out, p_top)

    far_loop = gmsh.model.geo.addCurveLoop([arc_top, arc_bot, l_bot, l_out, l_top])
    plane = gmsh.model.geo.addPlaneSurface([far_loop, foil_loop])
    gmsh.model.geo.synchronize()

    # -- refinement: wake, then the boundary layer ------------------------------------
    # A distance-from-the-aerofoil field, thresholded, keeps cells small through the
    # bubble and the near wake and lets them coarsen towards the far boundary.
    f_dist = gmsh.model.mesh.field.add("Distance")
    gmsh.model.mesh.field.setNumbers(f_dist, "CurvesList", lines)
    gmsh.model.mesh.field.setNumber(f_dist, "Sampling", 600)

    f_thr = gmsh.model.mesh.field.add("Threshold")
    gmsh.model.mesh.field.setNumber(f_thr, "InField", f_dist)
    gmsh.model.mesh.field.setNumber(f_thr, "SizeMin", cfg.wake_cell_size)
    gmsh.model.mesh.field.setNumber(f_thr, "SizeMax", h)
    gmsh.model.mesh.field.setNumber(f_thr, "DistMin", 0.05)
    gmsh.model.mesh.field.setNumber(f_thr, "DistMax", 0.5 * R)

    # A box along the wake line, so the shear layer stays resolved downstream.
    f_box = gmsh.model.mesh.field.add("Box")
    gmsh.model.mesh.field.setNumber(f_box, "VIn", cfg.wake_cell_size * 1.5)
    gmsh.model.mesh.field.setNumber(f_box, "VOut", h)
    gmsh.model.mesh.field.setNumber(f_box, "XMin", 0.9)
    gmsh.model.mesh.field.setNumber(f_box, "XMax", 0.35 * L)
    gmsh.model.mesh.field.setNumber(f_box, "YMin", -0.6)
    gmsh.model.mesh.field.setNumber(f_box, "YMax", 0.6)
    gmsh.model.mesh.field.setNumber(f_box, "Thickness", 2.0)

    f_min = gmsh.model.mesh.field.add("Min")
    gmsh.model.mesh.field.setNumbers(f_min, "FieldsList", [f_thr, f_box])
    gmsh.model.mesh.field.setAsBackgroundMesh(f_min)

    bl = gmsh.model.mesh.field.add("BoundaryLayer")
    gmsh.model.mesh.field.setNumbers(bl, "CurvesList", lines)
    gmsh.model.mesh.field.setNumber(bl, "Size", y1)
    gmsh.model.mesh.field.setNumber(bl, "Ratio", cfg.growth)
    gmsh.model.mesh.field.setNumber(bl, "Thickness", bl_thickness)
    gmsh.model.mesh.field.setNumber(bl, "Quads", 1)
    gmsh.model.mesh.field.setNumber(bl, "NbLayers", n_normal)
    # No trailing-edge fan: on a sharp TE it produces the worst cells in the mesh.
    gmsh.model.mesh.field.setAsBoundaryLayer(bl)

    gmsh.option.setNumber("Mesh.Algorithm", 5)            # Delaunay
    gmsh.option.setNumber("Mesh.RecombineAll", 0)
    # Smoothing and optimisation. The first case run without these reached a maximum
    # non-orthogonality of 85.7 deg -- localised where the boundary layer meets the
    # unstructured region -- and the steady solver diverged on it.
    gmsh.option.setNumber("Mesh.Smoothing", 20)
    gmsh.option.setNumber("Mesh.Optimize", 1)
    gmsh.option.setNumber("Mesh.OptimizeNetgen", 1)
    gmsh.option.setNumber("Mesh.MshFileVersion", 2.2)     # gmshToFoam wants v2
    gmsh.model.mesh.generate(2)

    # -- extrude one cell in z, which is how OpenFOAM does 2-D ------------------------
    ov = gmsh.model.geo.extrude([(2, plane)], 0, 0, 0.05, [1], recombine=True)
    gmsh.model.geo.synchronize()
    gmsh.model.mesh.generate(3)

    vol = [e[1] for e in ov if e[0] == 3]
    side_surfaces = [e[1] for e in ov if e[0] == 2]

    # Front/back are the original surface and its extruded image; the rest are the
    # boundaries carried up from the 2-D curves.
    gmsh.model.addPhysicalGroup(3, vol, name="internal")
    gmsh.model.addPhysicalGroup(2, [plane], name="back")
    gmsh.model.addPhysicalGroup(2, [side_surfaces[0]], name="front")

    # Identify side patches by the curve they were extruded from, using geometry: the
    # aerofoil curves are the ones inside the far boundary.
    wall_surfs, far_surfs, out_surfs = [], [], []
    for s in side_surfaces[1:]:
        xmin, ymin, _, xmax, ymax, _ = gmsh.model.getBoundingBox(2, s)
        cx, cy = 0.5 * (xmin + xmax), 0.5 * (ymin + ymax)
        if abs(cx - L) < 1e-6 * max(1.0, L):
            out_surfs.append(s)
        elif (cx ** 2 + cy ** 2) ** 0.5 > 0.5 * R:
            far_surfs.append(s)
        else:
            wall_surfs.append(s)

    gmsh.model.addPhysicalGroup(2, wall_surfs, name="aerofoil")
    gmsh.model.addPhysicalGroup(2, far_surfs, name="farfield")
    gmsh.model.addPhysicalGroup(2, out_surfs, name="outlet")

    path = os.path.join(out_dir, write_msh)
    gmsh.write(path)

    n_nodes = len(gmsh.model.mesh.getNodes()[0])
    _, elems, _ = gmsh.model.mesh.getElements(3)
    n_cells = sum(len(e) for e in elems)
    gmsh.finalize()

    return {
        "path": path,
        "reynolds": reynolds,
        "nodes": int(n_nodes),
        "cells": int(n_cells),
        "y1_chord": y1,
        "bl_thickness_chord": bl_thickness,
        "n_normal": int(n_normal),
        "growth": cfg.growth,
        "y_plus_target": cfg.y_plus,
        "n_surface": cfg.n_surface,
        "wall_patches": len(wall_surfs),
        "far_patches": len(far_surfs),
        "outlet_patches": len(out_surfs),
    }
