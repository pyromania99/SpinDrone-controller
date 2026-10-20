"""
| File: cmesh.py
| License: BSD-3-Clause. Copyright (c) 2026. All rights reserved.
| Description: Fully structured C-topology quad mesh for a 2-D aerofoil, built with gmsh
|              transfinite blocks.

WHY THIS REPLACED THE HYBRID MESH
---------------------------------
The first mesher grew structured quad layers off the aerofoil and filled the rest of the
domain with unstructured triangles. It produced cells at **85.7 degrees** of
non-orthogonality (78.2 after smoothing and removing the trailing-edge fan), concentrated
exactly where the two meshing strategies meet -- which is also where the laminar separation
bubble sits.

Finite volume evaluates a face gradient assuming the line joining the two cell centres is
perpendicular to the shared face. Where it is not, a correction term is added; past roughly
70 degrees that correction exceeds the term it corrects and the scheme loses its footing.
Measured consequence: the solution oscillated by +/-74 percent about a mean that was itself
within 1.2 percent of XFOIL. The physics setup was right; the mesh was not.

No solver setting fixes mesh topology, so the topology changed.

THE DECOMPOSITION
-----------------
Four transfinite blocks, every one a genuine quadrilateral with four distinct corners::

      (1-R,0)         outer arc, centred on the trailing edge
         +-------------------+ (1,+R) ----------------+ (L,+R)
         |                   |                        |
         |   UPPER AEROFOIL  |      UPPER WAKE        |
         |                   |                        |
         +---LE         TE---+------- wake cut -------+ (L,y_te)
         |                   |                        |
         |   LOWER AEROFOIL  |      LOWER WAKE        |
         |                   |                        |
         +-------------------+ (1,-R) ----------------+ (L,-R)

Splitting the aerofoil at the LEADING edge is what avoids a degenerate block. The obvious
single-block C has the trailing edge at two of its four corners, which a transfinite
surface cannot represent.

The wake cut is ONE curve shared by both wake blocks, so the mesh is conformal across it:
cells above and below the cut are true neighbours rather than two coincident boundaries.
That is the property that makes a C-mesh a C-mesh rather than two separate grids.

CELL COUNT
----------
A structured mesh of this quality needs far fewer cells than the hybrid one did -- roughly
40k against 333k for better resolution where it matters, because none are wasted on an
unstructured filler region. Solves are correspondingly faster.
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np

from .airfoil_mesh import first_layer_height, layer_count, load_selig


@dataclass
class CMeshConfig:
    """Structured C-mesh parameters. All lengths in chord units."""

    n_surface_half: int = 110
    """Cells along ONE side of the aerofoil, trailing edge to leading edge."""

    n_wake: int = 70
    """Cells from the trailing edge to the outlet."""

    growth: float = 1.12
    """Radial cell-to-cell growth ratio. Above about 1.2 the transition model degrades."""

    y_plus: float = 0.7
    """Target y+ for the first cell. Must stay below 1 for a wall-resolved model."""

    farfield_radius: float = 60.0
    """Outer arc radius, centred on the trailing edge [chords]."""

    wake_length: float = 80.0
    """Trailing edge to outlet [chords]."""

    surface_bump: float = 0.08
    """Transfinite 'Bump' coefficient on the surface curves. Below 1 clusters points at
    both ends, which is what the leading and trailing edges need."""

    wake_progression: float = 1.045
    """Streamwise growth along the wake. Keeps the near wake fine where the shear layer
    rolls up, and lets it coarsen towards the outlet."""

    def validate(self) -> None:
        """Check the invariants the mesh and the turbulence model depend on."""
        if self.y_plus >= 1.0:
            raise ValueError(
                f"y_plus={self.y_plus} defeats the wall-resolved transition model; use < 1"
            )
        if not 1.0 < self.growth <= 1.25:
            raise ValueError(f"growth={self.growth} outside the usable range (1, 1.25]")
        if self.n_surface_half < 40:
            raise ValueError(
                f"n_surface_half={self.n_surface_half} cannot resolve a separation bubble"
            )
        if self.farfield_radius < 20.0:
            raise ValueError(
                f"farfield_radius={self.farfield_radius} is too close; lift will be wrong"
            )
        if self.wake_length < 5.0:
            raise ValueError("wake_length must reach well past the trailing edge")
        if not 0.0 < self.surface_bump < 1.0:
            raise ValueError(
                "surface_bump must lie in (0, 1); only values below 1 cluster at the ends"
            )


def _split_surface(points: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Split a Selig-ordered aerofoil into upper and lower surfaces at the leading edge.

    Selig order runs from the trailing edge forward along the upper surface to the leading
    edge, then back along the lower surface.

    Returns:
        ``(upper, lower)``, each ordered from the TRAILING edge to the LEADING edge, so
        both blocks are built the same way round.
    """
    i_le = int(np.argmin(points[:, 0]))
    upper = np.array(points[: i_le + 1], dtype=float)       # TE -> LE
    lower = np.array(points[i_le:][::-1], dtype=float)      # TE -> LE after reversing
    if len(upper) < 5 or len(lower) < 5:
        raise ValueError("could not split the aerofoil into two surfaces")

    # CLOSE A BLUNT TRAILING EDGE.
    #
    # GOE 210 is supplied with a finite trailing-edge thickness: the coordinate file ends
    # at y = +0.0008 on the upper surface and y = -0.0008 on the lower, a gap of 0.16 % of
    # chord. A sharp-trailing-edge C topology has one point there, so leaving the two
    # apart produces two coincident-but-distinct corners -- checkMesh reports them exactly,
    # as "2 conflicting points to set nonManifoldPoints", along with the collapsed cells
    # that follow.
    #
    # Both surfaces are snapped to the midpoint. At 0.16 % chord this is far below the
    # resolution of the aerodynamics and is what a panel method does with the same
    # geometry anyway; modelling the blunt edge properly would need a fifth block carrying
    # cells across the base, which buys nothing at this thickness.
    te_mid = 0.5 * (upper[0] + lower[0])
    upper[0] = te_mid
    lower[0] = te_mid

    return upper, lower


def build_cmesh(
    airfoil_path: str,
    reynolds: float,
    out_dir: str,
    config: Optional[CMeshConfig] = None,
    write_msh: str = "mesh.msh",
) -> Dict:
    """Generate a structured C-mesh and write it as gmsh ``.msh`` v2.

    One cell thick in z with the two faces tagged for OpenFOAM's 2-D treatment.

    Args:
        airfoil_path: Selig ``.dat`` coordinate file, unit chord.
        reynolds: Chord Reynolds number. Sets the first cell height only.
        out_dir: Directory to write into; created if absent.
        config: Mesh parameters. Defaults to :class:`CMeshConfig`.
        write_msh: Mesh file name inside ``out_dir``.

    Returns:
        Mesh statistics and the derived radial sizing, for the run record.
    """
    import gmsh

    cfg = config or CMeshConfig()
    cfg.validate()

    y1 = first_layer_height(reynolds, cfg.y_plus)
    R = cfg.farfield_radius
    L = cfg.wake_length
    # Radial cells to span the whole domain at the chosen growth, starting from y1.
    n_normal = layer_count(y1, cfg.growth, R)

    raw = load_selig(airfoil_path)
    upper, lower = _split_surface(raw)
    te = raw[0]
    y_te = float(te[1])

    os.makedirs(out_dir, exist_ok=True)
    gmsh.initialize()
    gmsh.option.setNumber("General.Terminal", 0)
    gmsh.model.add("cmesh")
    geo = gmsh.model.geo

    # -- points ------------------------------------------------------------------------
    p_te = geo.addPoint(float(te[0]), y_te, 0.0)
    p_le = geo.addPoint(float(upper[-1][0]), float(upper[-1][1]), 0.0)

    def surface_points(arr):
        """Interior points of a surface, TE and LE supplied separately."""
        return [geo.addPoint(float(x), float(y), 0.0) for x, y in arr[1:-1]]

    up_mid = surface_points(upper)
    lo_mid = surface_points(lower)

    # Outer arc is centred ON the trailing-edge point itself. Creating a second point at
    # the same location, as a separate "centre", leaves a duplicate node there and
    # checkMesh reports open cells.
    p_far_up = geo.addPoint(float(te[0]), y_te + R, 0.0)
    p_far_lo = geo.addPoint(float(te[0]), y_te - R, 0.0)
    p_far_fwd = geo.addPoint(float(te[0]) - R, y_te, 0.0)

    p_out_cut = geo.addPoint(L, y_te, 0.0)
    p_out_up = geo.addPoint(L, y_te + R, 0.0)
    p_out_lo = geo.addPoint(L, y_te - R, 0.0)

    # -- curves ---------------------------------------------------------------------
    c_up_surf = geo.addSpline([p_te] + up_mid + [p_le])      # TE -> LE, upper
    c_lo_surf = geo.addSpline([p_te] + lo_mid + [p_le])      # TE -> LE, lower

    c_arc_up = geo.addCircleArc(p_far_up, p_te, p_far_fwd)   # (1,+R) -> (1-R,0)
    c_arc_lo = geo.addCircleArc(p_far_lo, p_te, p_far_fwd)   # (1,-R) -> (1-R,0)

    c_rad_le = geo.addLine(p_le, p_far_fwd)                  # LE outward
    c_rad_te_up = geo.addLine(p_te, p_far_up)                # TE outward, upper
    c_rad_te_lo = geo.addLine(p_te, p_far_lo)                # TE outward, lower

    c_cut = geo.addLine(p_te, p_out_cut)                     # the wake cut, SHARED
    c_far_up = geo.addLine(p_far_up, p_out_up)
    c_far_lo = geo.addLine(p_far_lo, p_out_lo)
    c_out_up = geo.addLine(p_out_cut, p_out_up)
    c_out_lo = geo.addLine(p_out_cut, p_out_lo)

    # -- surfaces ----------------------------------------------------------------------
    def quad(edges, corners):
        loop = geo.addCurveLoop(edges)
        surf = geo.addPlaneSurface([loop])
        geo.mesh.setTransfiniteSurface(surf, "Left", corners)
        geo.mesh.setRecombine(2, surf)
        return surf

    # Every loop is wound counter-clockwise. The upper aerofoil and lower wake blocks are
    # traversed in the reverse of the natural curve directions to achieve it; winding two
    # of the four the other way inverts their extruded cells.
    s_up_foil = quad([c_rad_te_up, c_arc_up, -c_rad_le, -c_up_surf],
                     [p_te, p_far_up, p_far_fwd, p_le])
    s_lo_foil = quad([c_lo_surf, c_rad_le, -c_arc_lo, -c_rad_te_lo],
                     [p_te, p_le, p_far_fwd, p_far_lo])
    s_up_wake = quad([c_cut, c_out_up, -c_far_up, -c_rad_te_up],
                     [p_te, p_out_cut, p_out_up, p_far_up])
    s_lo_wake = quad([c_rad_te_lo, c_far_lo, -c_out_lo, -c_cut],
                     [p_te, p_far_lo, p_out_lo, p_out_cut])

    # -- node distributions -------------------------------------------------------------
    # Radial: geometric from the wall outward, so the first cell lands on the y+ target.
    for c in (c_rad_le, c_rad_te_up, c_rad_te_lo, c_out_up, c_out_lo):
        geo.mesh.setTransfiniteCurve(c, n_normal + 1, "Progression", cfg.growth)

    # The aerofoil surface is clustered at both ends -- Bump below 1 -- to put points where
    # the curvature and the pressure gradient are.
    for c in (c_up_surf, c_lo_surf):
        geo.mesh.setTransfiniteCurve(
            c, cfg.n_surface_half + 1, "Bump", cfg.surface_bump
        )

    # The far-field arc opposite it is UNIFORM, and this is not a free choice.
    #
    # Transfinite interpolation blends between opposite edges, so clustering BOTH the
    # aerofoil and the arc pinches the outer ends of the radial grid lines towards the
    # same two corners. Over a 60-chord span that is enough for neighbouring lines to
    # cross, and the first cell layer inverts. Measured on the lower surface near
    # x = 0.10: 303 cells of negative area with matched Bump 0.08, falling to 180 at
    # Bump 0.2, 47 at Bump 0.5 -- and exactly ZERO with the arc uniform.
    #
    # Clustering the arc bought nothing in the first place: the far field has no feature
    # to resolve.
    for c in (c_arc_up, c_arc_lo):
        geo.mesh.setTransfiniteCurve(c, cfg.n_surface_half + 1, "Progression", 1.0)

    # Wake: fine at the trailing edge, coarsening downstream.
    for c in (c_cut, c_far_up, c_far_lo):
        geo.mesh.setTransfiniteCurve(
            c, cfg.n_wake + 1, "Progression", cfg.wake_progression
        )

    geo.synchronize()
    gmsh.option.setNumber("Mesh.RecombineAll", 1)
    gmsh.option.setNumber("Mesh.RecombinationAlgorithm", 1)
    gmsh.option.setNumber("Mesh.MshFileVersion", 2.2)
    gmsh.model.mesh.generate(2)

    # -- extrude one cell, which is how OpenFOAM represents 2-D --------------------------
    extruded = geo.extrude(
        [(2, s_up_foil), (2, s_lo_foil), (2, s_up_wake), (2, s_lo_wake)],
        0.0, 0.0, 0.05, [1], recombine=True,
    )
    geo.synchronize()
    gmsh.model.mesh.generate(3)

    volumes = [e[1] for e in extruded if e[0] == 3]
    front_faces = [e[1] for e in extruded if e[0] == 2][:0]  # filled below

    # Classify every boundary surface by WHICH CURVE IT WAS EXTRUDED FROM, not by where
    # it sits.
    #
    # The bounding-box heuristic this replaces produced DUPLICATE patch names -- two
    # "aerofoil" patches of 220 and 102 faces and two "farfield" patches -- because a
    # surface near the aerofoil in x and y is not necessarily part of it. OpenFOAM then
    # integrated forces over only the first patch of that name, which is a silent and
    # entirely plausible-looking error.
    #
    # Every extruded side surface carries its source curve in its boundary, so the mapping
    # is exact rather than inferred.
    wall_curves = {c_up_surf, c_lo_surf}
    far_curves = {c_arc_up, c_arc_lo, c_far_up, c_far_lo}
    outlet_curves = {c_out_up, c_out_lo}

    walls, far, outlet, front, back = [], [], [], [], []
    for dim, tag in gmsh.model.getEntities(2):
        xmin, ymin, zmin, xmax, ymax, zmax = gmsh.model.getBoundingBox(dim, tag)
        if abs(zmax - zmin) < 1e-12:                   # planar in z: a front or back face
            (back if abs(zmin) < 1e-12 else front).append(tag)
            continue
        # The wake cut is shared by both wake blocks, so its extruded surface lies
        # BETWEEN two volumes and is interior, not a boundary. That sharing is the whole
        # point of a C-mesh -- cells above and below the cut are true neighbours -- so the
        # surface must be skipped rather than given a patch.
        if len(gmsh.model.getAdjacencies(2, tag)[0]) == 2:
            continue

        curves = {
            abs(c[1])
            for c in gmsh.model.getBoundary([(2, tag)], oriented=False, recursive=False)
        }
        if curves & wall_curves:
            walls.append(tag)
        elif curves & far_curves:
            far.append(tag)
        elif curves & outlet_curves:
            outlet.append(tag)
        else:
            # The wake cut is interior: both sides of it are real cells, not a boundary.
            # Anything landing here would be an unclassified boundary, which must not be
            # silently dropped into the wrong patch.
            raise RuntimeError(
                f"surface {tag} could not be classified from its curves {sorted(curves)}"
            )

    expected_walls = 2
    if len(walls) != expected_walls:
        raise RuntimeError(
            f"expected {expected_walls} aerofoil surfaces (upper and lower), got "
            f"{len(walls)}; force integration would be over the wrong faces"
        )

    gmsh.model.addPhysicalGroup(3, volumes, name="internal")
    gmsh.model.addPhysicalGroup(2, walls, name="aerofoil")
    gmsh.model.addPhysicalGroup(2, far, name="farfield")
    gmsh.model.addPhysicalGroup(2, outlet, name="outlet")
    gmsh.model.addPhysicalGroup(2, front, name="front")
    gmsh.model.addPhysicalGroup(2, back, name="back")

    path = os.path.join(out_dir, write_msh)
    gmsh.write(path)

    n_nodes = len(gmsh.model.mesh.getNodes()[0])
    _, elems, _ = gmsh.model.mesh.getElements(3)
    n_cells = sum(len(e) for e in elems)
    gmsh.finalize()

    return {
        "path": path,
        "topology": "structured C, 4 transfinite blocks",
        "reynolds": reynolds,
        "nodes": int(n_nodes),
        "cells": int(n_cells),
        "y1_chord": y1,
        "n_normal": int(n_normal),
        "n_surface_half": cfg.n_surface_half,
        "n_wake": cfg.n_wake,
        "growth": cfg.growth,
        "wall_patches": len(walls),
        "far_patches": len(far),
        "outlet_patches": len(outlet),
    }
