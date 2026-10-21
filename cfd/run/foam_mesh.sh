#!/bin/bash
# Convert the gmsh mesh and check it. Runs INSIDE the OpenFOAM container.
set -e
cd "$1"
gmshToFoam mesh.msh > log.gmshToFoam 2>&1
echo "converted"
