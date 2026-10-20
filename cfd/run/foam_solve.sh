#!/bin/bash
# Check the mesh and solve. Runs INSIDE the OpenFOAM container, after the host has
# corrected the boundary types that gmshToFoam cannot know about.
set -e
CASE="$1"; NP="${2:-8}"
cd "$CASE"
checkMesh -constant > log.checkMesh 2>&1 || true
if [ "$NP" -gt 1 ]; then
  sed -i "s/numberOfSubdomains .*/numberOfSubdomains $NP;/" system/decomposeParDict
  decomposePar -force > log.decomposePar 2>&1
  mpirun -np "$NP" --allow-run-as-root simpleFoam -parallel > log.simpleFoam 2>&1 || true
  reconstructPar -latestTime > log.reconstructPar 2>&1 || true
else
  simpleFoam > log.simpleFoam 2>&1 || true
fi
echo "solved"
