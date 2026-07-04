#include "horizon/simulation/LinearElement.h"

#include <Eigen/Dense>
#include <cmath>

namespace hz::sim {

namespace {

/// Node positions of an element as an Eigen 4x3 (row i = node i).
Eigen::Matrix<double, 4, 3> nodePositions(const TetMesh& mesh, const Tet4& element) {
    Eigen::Matrix<double, 4, 3> p;
    for (int i = 0; i < 4; ++i) {
        const math::Vec3& v = mesh.nodes[element.nodes[i]].position;
        p(i, 0) = v.x;
        p(i, 1) = v.y;
        p(i, 2) = v.z;
    }
    return p;
}

/// Isotropic elasticity matrix D (6x6) in Voigt notation with strain order
/// [exx, eyy, ezz, gxy, gyz, gzx].
Eigen::Matrix<double, 6, 6> elasticityMatrix(const ElasticMaterial& m) {
    const double E = m.youngsModulus;
    const double nu = m.poissonRatio;
    const double lambda = E * nu / ((1.0 + nu) * (1.0 - 2.0 * nu));
    const double mu = E / (2.0 * (1.0 + nu));

    Eigen::Matrix<double, 6, 6> D = Eigen::Matrix<double, 6, 6>::Zero();
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            D(i, j) = lambda;
        }
        D(i, i) = lambda + 2.0 * mu;
    }
    D(3, 3) = mu;
    D(4, 4) = mu;
    D(5, 5) = mu;
    return D;
}

}  // namespace

double tetVolume(const TetMesh& mesh, const Tet4& element) {
    const Eigen::Matrix<double, 4, 3> p = nodePositions(mesh, element);
    Eigen::Matrix3d J;
    J.row(0) = p.row(1) - p.row(0);
    J.row(1) = p.row(2) - p.row(0);
    J.row(2) = p.row(3) - p.row(0);
    return J.determinant() / 6.0;
}

std::array<double, 144> elementStiffness(const TetMesh& mesh, const Tet4& element,
                                         const ElasticMaterial& material) {
    std::array<double, 144> out{};  // zero-initialised
    if (!material.isValid()) return out;

    const Eigen::Matrix<double, 4, 3> p = nodePositions(mesh, element);

    // Shape-function coefficients: N_i = a_i + b_i x + c_i y + d_i z with
    // N_i(p_j) = delta_ij. Stacking [1, x, y, z] over the four nodes gives C, and
    // the coefficient columns are C^{-1}. Rows 1..3 of C^{-1} are the (constant)
    // spatial gradients of the shape functions.
    Eigen::Matrix4d C;
    C.col(0).setOnes();
    C.block<4, 3>(0, 1) = p;

    const double detC = C.determinant();
    const double volume = std::abs(detC) / 6.0;
    if (volume < 1e-18) return out;  // degenerate element

    const Eigen::Matrix4d Cinv = C.inverse();
    // grad(3x4): column i holds (dN_i/dx, dN_i/dy, dN_i/dz).
    const Eigen::Matrix<double, 3, 4> grad = Cinv.block<3, 4>(1, 0);

    // Strain-displacement matrix B (6x12).
    Eigen::Matrix<double, 6, 12> B = Eigen::Matrix<double, 6, 12>::Zero();
    for (int i = 0; i < 4; ++i) {
        const double bx = grad(0, i);
        const double by = grad(1, i);
        const double bz = grad(2, i);
        const int c = 3 * i;
        B(0, c + 0) = bx;
        B(1, c + 1) = by;
        B(2, c + 2) = bz;
        B(3, c + 0) = by;
        B(3, c + 1) = bx;
        B(4, c + 1) = bz;
        B(4, c + 2) = by;
        B(5, c + 0) = bz;
        B(5, c + 2) = bx;
    }

    const Eigen::Matrix<double, 6, 6> D = elasticityMatrix(material);
    const Eigen::Matrix<double, 12, 12> Ke = volume * (B.transpose() * D * B);

    for (int r = 0; r < 12; ++r) {
        for (int col = 0; col < 12; ++col) {
            out[r * 12 + col] = Ke(r, col);
        }
    }
    return out;
}

}  // namespace hz::sim
