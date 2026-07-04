#pragma once

#include "horizon/simulation/TetMesh.h"

namespace hz::sim {

/// Builds a structured tetrahedral mesh of an axis-aligned box spanning
/// [0, sizeX] x [0, sizeY] x [0, sizeZ], subdivided into nx*ny*nz cells with each
/// cell split into 6 conforming tetrahedra (a Kuhn / shared-main-diagonal
/// decomposition, so faces match across neighbouring cells).
///
/// Nodes are laid out on the grid with index i + (nx+1)*(j + (ny+1)*k) for grid
/// coordinate (i, j, k). Returns an empty mesh if any subdivision count is < 1 or
/// any dimension is <= 0.
///
/// This is a simple built-in volume mesher for boxes — enough to run analyses on
/// simple geometry and to validate the solver; general B-Rep tetrahedralization
/// is a separate concern.
TetMesh meshBox(double sizeX, double sizeY, double sizeZ, int nx, int ny, int nz);

}  // namespace hz::sim
