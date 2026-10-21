# CFD-derived aerodynamics for the spinning wing

Working OpenFOAM pipeline producing section polars for the blade, to replace the analytic
polar fitted to the design report.

## Status

| piece | state |
|---|---|
| OpenFOAM v2506 (container, no sudo needed) | **working** |
| gmsh mesher via pip (no sudo) | **working** — 333k cells in 18 s, `Mesh OK` |
| C-mesh generator with derived y1 | **working** |
| Case generator with turbulence-decay back-solve | **working** |
| Container run driver | **working** — 0.66 s/iteration on 8 ranks |
| Force-coefficient post-processor | written |
| E387 validation case | geometry fetched, not yet run |
| GOE 210 sweep | not yet run |

## Quick start

```bash
python3 -c "
import sys; sys.path.insert(0,'.')
from mesh.airfoil_mesh import build_mesh, MeshConfig
from run.case import write_case, CaseConfig
from run.driver import run_case
from post.extract import summarise_case

m = build_mesh('geometry/goe210.dat', 1.0e5, '/tmp/cfd/mesh_Re100k', MeshConfig())
write_case('/tmp/cfd/a5_Re100k', m['path'], CaseConfig(alpha_deg=5.0, reynolds=1.0e5))
print(run_case('/tmp/cfd/a5_Re100k', n_procs=8))
print(summarise_case('/tmp/cfd/a5_Re100k'))
"
```

Everything else is in `config/solver.yaml`. Wing *design* stays in
`examples/utils/wing_params.py`; this directory owns the solver only.

## Why it is built this way

**The section is tabulated, not the whole wing.** Tabulating the wing's force and moment
against vehicle state needs six non-dimensional inputs (μx, μy, μz, p̂, q̂, Re_tip) — about
262,000 cases at 8 samples per dimension. Tabulating the *section* needs (α, Re) — a few
hundred. Blade element theory then supplies the state dependence, which is exact
kinematics with no modelling error. CFD goes where the error actually is. See `PLAN.md` §1.

**`kOmegaSSTLM` is mandatory, not a preference.** The blade runs at Re 5×10⁴–2.4×10⁵ where
a laminar separation bubble governs lift and drag. A fully turbulent RANS solution there is
qualitatively wrong and would be *worse* than the XFOIL data already in `lit_survey/`.

**The first cell height is derived, not configured.** `first_layer_height()` computes y1
from a target y+ and the flat-plate skin-friction correlation. Wall functions silently
defeat a transition model — the case still converges and the transition location is simply
wrong.

**Freestream turbulence is specified at the aerofoil, not the inlet.** k and ω decay
between the two, so setting inlet values to the reference intensity puts the wrong
intensity at the leading edge. `freestream_turbulence()` inverts the analytic decay. It
also enforces a real constraint: ω at the aerofoil is bounded above by `U/(βL)` however
large the inlet value, which is why the eddy-viscosity ratio is specified at the inlet
where the problem is always well posed.

**E387 runs before GOE 210.** It is the only section in `lit_survey/` with measured
wind-tunnel data at these Reynolds numbers (Selig et al. Vol. 1), so it is the only case
that can show whether this setup is trustworthy. GOE 210 has no measured data anywhere, so
validating against it is impossible by construction.

## Measured cost

0.66 s/iteration for 333k cells on 8 MPI ranks. At 4000 iterations that is **~45 min per
case**, which matches the one-hour-per-case expectation.

The sweep in `config/solver.yaml` is 39 angles × 6 Reynolds numbers = **234 cases ≈ 7 days**
continuous. Ways to cut it, in order of how little they cost in fidelity:

| change | cases | estimate |
|---|---|---|
| as configured | 234 | ~7 days |
| 4 Reynolds numbers instead of 6 | 156 | ~4.7 days |
| + mesh to ~200k cells | 156 | ~3 days |
| + 2° steps in deep stall only | 134 | ~2.6 days |
| + residual-converged early exit | 134 | less, run-dependent |

Most cases should hit `residualControl` well before 4000 iterations; the first case will
establish that.

## Layout

```
config/solver.yaml     solver, mesh, freestream, sweep, acceptance criteria
geometry/              section coordinates (goe210 verified against foil_out/database/)
mesh/airfoil_mesh.py   gmsh C-mesh; y1 and layer count derived from Re and y+
run/case.py            writes an OpenFOAM case; turbulence-decay back-solve
run/driver.py          mesh conversion, boundary-type correction, parallel solve
run/foam_*.sh          the parts that must execute inside the container
post/extract.py        force coefficients, convergence test, y+ check
tables/                generated (alpha, Re) -> cl, cd, cm with provenance
```

Only the solver is containerised. Cases, logs and tables are ordinary files on the host.

## Known gaps

- `CFDTablePolar` (the simulator-side consumer) is not written yet. `TablePolar` already
  exists and works, so this is a thin subclass carrying provenance plus a Viterna
  extension to ±180° anchored on the CFD stall point.
- Incidence is imposed by rotating the freestream, so the wake-refinement box (aligned
  with x) drifts off the wake above roughly 15°. Deep-stall cases need the box widened or
  the mesh rotated.
- 3-D rotational augmentation (Himmelskamp) is not addressed here. The blade's aspect
  ratio is 1.3, which is where that effect is largest; see `PLAN.md` §5.
