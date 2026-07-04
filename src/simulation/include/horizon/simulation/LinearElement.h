#pragma once

#include <array>

#include "horizon/simulation/Material.h"
#include "horizon/simulation/TetMesh.h"

namespace hz::sim {

/// Signed volume of a linear tetrahedron: det([p1-p0, p2-p0, p3-p0]) / 6.
/// Positive for a counter-clockwise (canonically oriented) element.
double tetVolume(const TetMesh& mesh, const Tet4& element);

/// The 12x12 stiffness matrix of a constant-strain linear tetrahedron, in
/// row-major order. DOF order is [n0x, n0y, n0z, n1x, n1y, n1z, ...] following
/// the element's node order.
///
/// Ke = V * B^T * D * B, where B is the (constant) strain-displacement matrix,
/// D the isotropic elasticity matrix, and V the element volume. The result is
/// symmetric and has the six rigid-body modes (three translations, three
/// rotations) in its null space. A degenerate (near-zero-volume) element or an
/// invalid material yields an all-zero matrix.
std::array<double, 144> elementStiffness(const TetMesh& mesh, const Tet4& element,
                                         const ElasticMaterial& material);

/// The constant stress in a linear tetrahedron given its 12 nodal displacements
/// (order [n0x, n0y, n0z, n1x, ...]). Returns the Voigt stress vector
/// [sxx, syy, szz, sxy, syz, szx] = D * B * u. A degenerate element or invalid
/// material yields all zeros.
std::array<double, 6> elementStress(const TetMesh& mesh, const Tet4& element,
                                    const ElasticMaterial& material,
                                    const std::array<double, 12>& displacements);

/// The von Mises equivalent stress of a Voigt stress vector
/// [sxx, syy, szz, sxy, syz, szx].
double vonMises(const std::array<double, 6>& stress);

}  // namespace hz::sim
