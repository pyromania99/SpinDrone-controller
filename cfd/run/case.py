"""
| File: case.py
| License: BSD-3-Clause. Copyright (c) 2026. All rights reserved.
| Description: Writes a steady incompressible OpenFOAM case for a 2-D aerofoil at a given
|              angle of attack and Reynolds number, using the Langtry-Menter transition
|              model.

ONE MESH SERVES EVERY ANGLE OF ATTACK
-------------------------------------
Incidence is imposed by rotating the freestream velocity vector, not the mesh. The far
boundary is a circle, so this is exact, and it means the mesh is generated once per
Reynolds number instead of once per (alpha, Re) pair -- roughly a thirty-fold saving on
meshing for a typical sweep.

THE FREESTREAM TURBULENCE TRAP
------------------------------
``kOmegaSSTLM`` triggers transition from the LOCAL turbulence intensity, and k and omega
decay between the inlet and the aerofoil. Setting the inlet values to the reference
turbulence intensity therefore puts the wrong intensity at the leading edge, which moves
the transition point and quietly invalidates the whole result. This is the single most
common way a low-Reynolds transitional case goes wrong.

:func:`freestream_turbulence` inverts the analytic decay so that the requested intensity
is achieved AT THE AEROFOIL, and reports both ends so the run record shows the decay that
was applied.
"""

from __future__ import annotations

import os
import shutil
from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple

import numpy as np

# SST freestream decay constants.
BETA = 0.0828
BETA_STAR = 0.09


# =======================================================================================
# Inflow turbulence
# =======================================================================================

def re_theta_t_correlation(turbulence_intensity_pct: float) -> float:
    """Langtry-Menter inlet transition-momentum-thickness Reynolds number.

    The empirical correlation the model is built around:

        Tu <= 1.3 :  ReThetat = 1173.51 - 589.428 Tu + 0.2196 / Tu^2
        Tu >  1.3 :  ReThetat = 331.50 (Tu - 0.5658)^-0.671

    Args:
        turbulence_intensity_pct: Freestream turbulence intensity in PERCENT.

    Returns:
        ReThetat for the inlet boundary condition.
    """
    tu = float(turbulence_intensity_pct)
    if tu <= 0.0:
        raise ValueError("turbulence intensity must be positive")
    if tu <= 1.3:
        return 1173.51 - 589.428 * tu + 0.2196 / (tu * tu)
    return 331.50 * (tu - 0.5658) ** -0.671


def freestream_turbulence(
    reynolds: float,
    target_intensity_pct: float,
    inlet_viscosity_ratio: float,
    decay_distance_chord: float,
    velocity: float = 1.0,
) -> Dict[str, float]:
    """Inlet k and omega that DECAY to the requested intensity at the aerofoil.

    The SST freestream decay, with x measured from the inlet, is

        omega(x) = omega_in / F,          F = 1 + beta omega_in x / U
        k(x)     = k_in F^(-beta*/beta)

    Note what this implies: ``omega`` at the aerofoil is bounded above by ``U/(beta L)``
    however large the inlet value, because a larger inlet omega decays proportionally
    faster. Specifying a target eddy-viscosity ratio AT THE AEROFOIL is therefore not
    always satisfiable -- at Re 2e5 over 60 chords the smallest reachable ratio is about
    1.5. The ratio is consequently specified at the INLET, which is both standard practice
    and always well posed, and the value arriving at the aerofoil is reported.

    With ``k_in = R nu omega_in`` for inlet ratio R, the intensity at the aerofoil is

        k_le(omega_in) = R nu omega_in (1 + beta omega_in L / U)^(-beta*/beta)

    which rises linearly for small omega_in and falls slowly beyond a maximum. The lower
    root is taken: it is the solution with the least decay, hence the least sensitivity to
    the inlet position.

    Args:
        reynolds: Chord Reynolds number (nu = 1/Re with U = c = 1).
        target_intensity_pct: Turbulence intensity wanted AT THE AEROFOIL, in percent.
        inlet_viscosity_ratio: nu_t/nu imposed at the inlet.
        decay_distance_chord: Inlet-to-aerofoil distance in chords.
        velocity: Freestream speed.

    Returns:
        Dict of inlet and aerofoil k, omega, intensity and viscosity ratio.

    Raises:
        ValueError: If the target intensity exceeds what this inlet ratio can deliver,
            with the attainable maximum quoted.
    """
    from scipy.optimize import brentq

    nu = velocity / reynolds  # chord = 1
    tu = target_intensity_pct / 100.0
    k_le_target = 1.5 * (velocity * tu) ** 2
    exponent = BETA_STAR / BETA
    L = decay_distance_chord

    def k_at_aerofoil(omega_in: float) -> float:
        F = 1.0 + BETA * omega_in * L / velocity
        return inlet_viscosity_ratio * nu * omega_in * F ** (-exponent)

    # Bracket the maximum, then solve on the rising branch below it.
    grid = np.geomspace(1.0e-6, 1.0e6, 2000)
    vals = np.array([k_at_aerofoil(w) for w in grid])
    i_peak = int(np.argmax(vals))
    if vals[i_peak] < k_le_target:
        tu_max = 100.0 * np.sqrt(2.0 * vals[i_peak] / 3.0) / velocity
        raise ValueError(
            f"Tu={target_intensity_pct}% at the aerofoil is unreachable with an inlet "
            f"eddy-viscosity ratio of {inlet_viscosity_ratio} over {L} chords; the "
            f"maximum attainable is {tu_max:.4f}%. Raise inlet_viscosity_ratio or "
            f"shorten decay_distance_chord."
        )

    omega_in = brentq(
        lambda w: k_at_aerofoil(w) - k_le_target, grid[0], grid[i_peak], xtol=1e-14,
        rtol=1e-12,
    )
    k_in = inlet_viscosity_ratio * nu * omega_in
    F = 1.0 + BETA * omega_in * L / velocity
    omega_le = omega_in / F
    k_le = k_in * F ** (-exponent)
    tu_inlet_pct = 100.0 * np.sqrt(2.0 * k_in / 3.0) / velocity

    return {
        "nu": nu,
        "k_inlet": k_in,
        "omega_inlet": omega_in,
        "k_aerofoil": k_le,
        "omega_aerofoil": omega_le,
        "intensity_inlet_pct": tu_inlet_pct,
        "intensity_aerofoil_pct": 100.0 * np.sqrt(2.0 * k_le / 3.0) / velocity,
        "viscosity_ratio_inlet": inlet_viscosity_ratio,
        "viscosity_ratio_aerofoil": k_le / omega_le / nu,
        "ReThetat_inlet": re_theta_t_correlation(tu_inlet_pct),
    }


# =======================================================================================
# Case writing
# =======================================================================================

_HEADER = """/*--------------------------------*- C++ -*----------------------------------*\\
  Generated by cfd/run/case.py -- do not edit by hand.
\\*---------------------------------------------------------------------------*/
FoamFile
{{
    version     2.0;
    format      ascii;
    class       {cls};
    object      {obj};
}}
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //

"""


def _write(path: str, cls: str, obj: str, body: str) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as handle:
        handle.write(_HEADER.format(cls=cls, obj=obj))
        handle.write(body)
        handle.write("\n// ***************************************************** //\n")


def _scalar_field(path: str, name: str, dims: str, internal: str, walls: str,
                  inlet: str, outlet: str) -> None:
    _write(path, "volScalarField", name, f"""dimensions      {dims};

internalField   uniform {internal};

boundaryField
{{
    aerofoil
    {{
{walls}
    }}
    farfield
    {{
{inlet}
    }}
    outlet
    {{
{outlet}
    }}
    "(front|back)"
    {{
        type            empty;
    }}
}}
""")


@dataclass
class CaseConfig:
    """Solver settings for one aerofoil case."""

    alpha_deg: float
    reynolds: float
    velocity: float = 1.0
    turbulence_intensity_pct: float = 0.10
    """Intensity AT THE AEROFOIL. 0.1 % is a clean low-turbulence tunnel, which is the
    condition the UIUC low-speed aerofoil data was taken in."""
    inlet_viscosity_ratio: float = 10.0
    """nu_t/nu imposed at the INLET, not at the aerofoil -- see freestream_turbulence."""
    decay_distance_chord: float = 60.0
    n_iterations: int = 6000
    residual_tolerance: float = 1.0e-7
    write_interval: int = 1000
    consistent: bool = False
    """Use SIMPLEC (``consistent yes``) instead of SIMPLE.

    SIMPLEC converges faster on a good mesh but is less forgiving on stretched cells. With
    the aspect ratios a C-mesh carries into the far wake it left the pressure residual on a
    plateau at 1e-2 and the force coefficients alternating in sign between consecutive
    iterations. Plain SIMPLE with explicit pressure relaxation is slower and stable, which
    is the right trade for generating a table.
    """

    relaxation_p: float = 0.3
    relaxation_U: float = 0.7
    relaxation_turb: float = 0.7
    n_nonorth: int = 3
    """Non-orthogonal correctors. The hybrid gmsh mesh reaches 78 deg maximum
    non-orthogonality where the boundary-layer quads meet the unstructured region, which
    needs more than the single corrector a good structured mesh would."""
    laplacian_limiter: float = 0.33
    """Non-orthogonal correction limiter: 1.0 is uncorrected, 0.33 is strongly limited and
    stable on poor cells at the cost of some formal accuracy."""


def write_case(case_dir: str, mesh_msh: str, cfg: CaseConfig) -> Dict[str, float]:
    """Write a complete OpenFOAM case directory.

    Args:
        case_dir: Directory to create. Overwritten if it exists.
        mesh_msh: Path to the gmsh ``.msh`` file to convert in place.
        cfg: Case settings.

    Returns:
        The freestream turbulence record, for the run log.
    """
    if os.path.exists(case_dir):
        shutil.rmtree(case_dir)
    os.makedirs(case_dir)

    turb = freestream_turbulence(
        cfg.reynolds, cfg.turbulence_intensity_pct, cfg.inlet_viscosity_ratio,
        cfg.decay_distance_chord, cfg.velocity,
    )

    a = np.deg2rad(cfg.alpha_deg)
    ux, uy = cfg.velocity * np.cos(a), cfg.velocity * np.sin(a)
    shutil.copy(mesh_msh, os.path.join(case_dir, "mesh.msh"))

    # -- 0/ ---------------------------------------------------------------------------
    _write(f"{case_dir}/0/U", "volVectorField", "U", f"""dimensions      [0 1 -1 0 0 0 0];

internalField   uniform ({ux:.10f} {uy:.10f} 0);

boundaryField
{{
    aerofoil
    {{
        type            noSlip;
    }}
    farfield
    {{
        type            freestreamVelocity;
        freestreamValue uniform ({ux:.10f} {uy:.10f} 0);
    }}
    outlet
    {{
        type            freestreamVelocity;
        freestreamValue uniform ({ux:.10f} {uy:.10f} 0);
    }}
    "(front|back)"
    {{
        type            empty;
    }}
}}
""")

    _write(f"{case_dir}/0/p", "volScalarField", "p", """dimensions      [0 2 -2 0 0 0 0];

internalField   uniform 0;

boundaryField
{
    aerofoil
    {
        type            zeroGradient;
    }
    farfield
    {
        type            freestreamPressure;
        freestreamValue uniform 0;
    }
    outlet
    {
        type            freestreamPressure;
        freestreamValue uniform 0;
    }
    "(front|back)"
    {
        type            empty;
    }
}
""")

    # Wall treatment: low-Re, integrate to the wall. omegaWallFunction is the correct
    # low-Re form in OpenFOAM -- it blends to the analytic viscous-sublayer value when
    # y+ < 1, which is where this mesh sits.
    _scalar_field(
        f"{case_dir}/0/k", "k", "[0 2 -2 0 0 0 0]", f"{turb['k_inlet']:.6e}",
        "        type            fixedValue;\n        value           uniform 1e-14;",
        f"        type            freestream;\n"
        f"        freestreamValue uniform {turb['k_inlet']:.6e};",
        f"        type            freestream;\n"
        f"        freestreamValue uniform {turb['k_inlet']:.6e};",
    )
    _scalar_field(
        f"{case_dir}/0/omega", "omega", "[0 0 -1 0 0 0 0]", f"{turb['omega_inlet']:.6e}",
        "        type            omegaWallFunction;\n        blended         true;\n"
        f"        value           uniform {turb['omega_inlet']:.6e};",
        f"        type            freestream;\n"
        f"        freestreamValue uniform {turb['omega_inlet']:.6e};",
        f"        type            freestream;\n"
        f"        freestreamValue uniform {turb['omega_inlet']:.6e};",
    )
    _scalar_field(
        f"{case_dir}/0/nut", "nut", "[0 2 -1 0 0 0 0]", "0",
        "        type            nutLowReWallFunction;\n        value           uniform 0;",
        "        type            calculated;\n        value           uniform 0;",
        "        type            calculated;\n        value           uniform 0;",
    )
    _scalar_field(
        f"{case_dir}/0/gammaInt", "gammaInt", "[0 0 0 0 0 0 0]", "1",
        "        type            zeroGradient;",
        "        type            fixedValue;\n        value           uniform 1;",
        "        type            zeroGradient;",
    )
    _scalar_field(
        f"{case_dir}/0/ReThetat", "ReThetat", "[0 0 0 0 0 0 0]",
        f"{turb['ReThetat_inlet']:.4f}",
        "        type            zeroGradient;",
        f"        type            fixedValue;\n"
        f"        value           uniform {turb['ReThetat_inlet']:.4f};",
        "        type            zeroGradient;",
    )

    # -- constant/ ---------------------------------------------------------------------
    _write(f"{case_dir}/constant/transportProperties", "dictionary",
           "transportProperties",
           f"transportModel  Newtonian;\n\nnu              {turb['nu']:.10e};\n")
    _write(f"{case_dir}/constant/turbulenceProperties", "dictionary",
           "turbulenceProperties",
           "simulationType  RAS;\n\nRAS\n{\n    RASModel        kOmegaSSTLM;\n"
           "    turbulence      on;\n    printCoeffs     on;\n}\n")

    # -- system/ -----------------------------------------------------------------------
    _write(f"{case_dir}/system/controlDict", "dictionary", "controlDict", f"""application     simpleFoam;
startFrom       startTime;
startTime       0;
stopAt          endTime;
endTime         {cfg.n_iterations};
deltaT          1;
writeControl    timeStep;
writeInterval   {cfg.write_interval};
purgeWrite      2;
writeFormat     binary;
writePrecision  10;
timeFormat      general;
runTimeModifiable false;

functions
{{
    forceCoeffs
    {{
        type            forceCoeffs;
        libs            (forces);
        writeControl    timeStep;
        writeInterval   1;
        patches         (aerofoil);
        rho             rhoInf;
        rhoInf          1;
        liftDir         ({-np.sin(a):.10f} {np.cos(a):.10f} 0);
        dragDir         ({np.cos(a):.10f} {np.sin(a):.10f} 0);
        CofR            (0.25 0 0);
        pitchAxis       (0 0 1);
        magUInf         {cfg.velocity};
        lRef            1;
        Aref            0.05;   // chord x span; span is the 1-cell extrusion depth
    }}

    yPlus
    {{
        type            yPlus;
        libs            (fieldFunctionObjects);
        writeControl    writeTime;
    }}

    residuals
    {{
        type            solverInfo;
        libs            (utilityFunctionObjects);
        fields          (U p k omega);
        writeControl    timeStep;
        writeInterval   1;
    }}
}}
""")

    # Built as a plain string with one substitution: the body is full of literal braces,
    # which an f-string would require doubling throughout for no benefit.
    fv_schemes = """ddtSchemes
{
    default         steadyState;
}

gradSchemes
{
    default         cellLimited Gauss linear 1;
}

divSchemes
{
    default         none;
    div(phi,U)      bounded Gauss linearUpwind grad(U);
    div(phi,k)      bounded Gauss limitedLinear 1;
    div(phi,omega)  bounded Gauss limitedLinear 1;
    div(phi,gammaInt) bounded Gauss limitedLinear 1;
    div(phi,ReThetat) bounded Gauss limitedLinear 1;
    div((nuEff*dev2(T(grad(U))))) Gauss linear;
}

laplacianSchemes
{
    default         Gauss linear limited corrected __LIM__;
}

interpolationSchemes
{
    default         linear;
}

snGradSchemes
{
    default         limited corrected __LIM__;
}

wallDist
{
    method          meshWave;
}
""".replace("__LIM__", f"{cfg.laplacian_limiter}")
    _write(f"{case_dir}/system/fvSchemes", "dictionary", "fvSchemes", fv_schemes)

    _write(f"{case_dir}/system/fvSolution", "dictionary", "fvSolution", f"""solvers
{{
    p
    {{
        solver          GAMG;
        smoother        GaussSeidel;
        nPreSweeps      0;
        nPostSweeps     2;
        cacheAgglomeration true;
        nCellsInCoarsestLevel 50;
        tolerance       1e-9;
        relTol          0.001;
    }}

    "(U|k|omega|gammaInt|ReThetat)"
    {{
        solver          PBiCGStab;
        preconditioner  DILU;
        tolerance       1e-10;
        relTol          0.01;
    }}
}}

SIMPLE
{{
    nNonOrthogonalCorrectors {cfg.n_nonorth};
    consistent      {"yes" if cfg.consistent else "no"};

    residualControl
    {{
        p               {cfg.residual_tolerance:.1e};
        U               {cfg.residual_tolerance:.1e};
        "(k|omega|gammaInt|ReThetat)" {cfg.residual_tolerance:.1e};
    }}
}}

relaxationFactors
{{
    fields
    {{
        p               {1.0 if cfg.consistent else cfg.relaxation_p};
    }}
    equations
    {{
        U               {cfg.relaxation_U};
        ".*"            {cfg.relaxation_turb};
    }}
}}
""")

    _write(f"{case_dir}/system/decomposeParDict", "dictionary", "decomposeParDict",
           "numberOfSubdomains 8;\nmethod          scotch;\n")

    return turb
