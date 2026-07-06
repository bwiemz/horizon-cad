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

/// The (constant) strain-displacement matrix B (6x12) and absolute volume of a
/// linear tetrahedron. `valid` is false for a degenerate (near-zero-volume)
/// element, in which case B is left zero.
struct ElementGeom {
    Eigen::Matrix<double, 6, 12> B = Eigen::Matrix<double, 6, 12>::Zero();
    double volume = 0.0;
    bool valid = false;
};

ElementGeom computeGeom(const TetMesh& mesh, const Tet4& element) {
    ElementGeom g;
    std::array<math::Vec3, 4> grad;
    if (!tetShapeGradients(mesh, element, grad, g.volume)) return g;

    for (int i = 0; i < 4; ++i) {
        const double bx = grad[i].x;
        const double by = grad[i].y;
        const double bz = grad[i].z;
        const int c = 3 * i;
        g.B(0, c + 0) = bx;
        g.B(1, c + 1) = by;
        g.B(2, c + 2) = bz;
        g.B(3, c + 0) = by;
        g.B(3, c + 1) = bx;
        g.B(4, c + 1) = bz;
        g.B(4, c + 2) = by;
        g.B(5, c + 0) = bz;
        g.B(5, c + 2) = bx;
    }
    g.valid = true;
    return g;
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

bool tetShapeGradients(const TetMesh& mesh, const Tet4& element,
                       std::array<math::Vec3, 4>& gradients, double& volume) {
    const Eigen::Matrix<double, 4, 3> p = nodePositions(mesh, element);

    // Shape-function coefficients: N_i = a_i + b_i x + c_i y + d_i z with
    // N_i(p_j) = delta_ij. Stacking [1, x, y, z] over the four nodes gives C, and
    // the coefficient columns are C^{-1}. Rows 1..3 of C^{-1} are the (constant)
    // spatial gradients of the shape functions.
    Eigen::Matrix4d C;
    C.col(0).setOnes();
    C.block<4, 3>(0, 1) = p;

    const double detC = C.determinant();
    const double vol = std::abs(detC) / 6.0;
    if (vol < 1e-18) return false;  // degenerate element

    const Eigen::Matrix4d Cinv = C.inverse();
    // grad(3x4): column i holds (dN_i/dx, dN_i/dy, dN_i/dz).
    const Eigen::Matrix<double, 3, 4> grad = Cinv.block<3, 4>(1, 0);
    for (int i = 0; i < 4; ++i) {
        gradients[i] = math::Vec3(grad(0, i), grad(1, i), grad(2, i));
    }
    volume = vol;
    return true;
}

std::array<double, 144> elementStiffness(const TetMesh& mesh, const Tet4& element,
                                         const ElasticMaterial& material) {
    std::array<double, 144> out{};  // zero-initialised
    if (!material.isValid()) return out;

    const ElementGeom g = computeGeom(mesh, element);
    if (!g.valid) return out;

    const Eigen::Matrix<double, 6, 6> D = elasticityMatrix(material);
    const Eigen::Matrix<double, 12, 12> Ke = g.volume * (g.B.transpose() * D * g.B);

    for (int r = 0; r < 12; ++r) {
        for (int col = 0; col < 12; ++col) {
            out[r * 12 + col] = Ke(r, col);
        }
    }
    return out;
}

std::array<double, 144> elementMass(const TetMesh& mesh, const Tet4& element, double density) {
    std::array<double, 144> out{};  // zero-initialised
    if (density <= 0.0) return out;

    std::array<math::Vec3, 4> grad;
    double volume = 0.0;
    if (!tetShapeGradients(mesh, element, grad, volume)) return out;

    // Consistent mass: diagonal node blocks weigh 2*c, off-diagonal 1*c, with
    // c = rho*V/20, each replicated on the three translational DOFs of a node.
    const double c = density * volume / 20.0;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            const double m = (i == j) ? 2.0 * c : c;
            for (int d = 0; d < 3; ++d) {
                const int r = 3 * i + d;
                const int col = 3 * j + d;
                out[r * 12 + col] = m;
            }
        }
    }
    return out;
}

std::array<double, 6> elementStress(const TetMesh& mesh, const Tet4& element,
                                    const ElasticMaterial& material,
                                    const std::array<double, 12>& displacements) {
    std::array<double, 6> out{};  // zero-initialised
    if (!material.isValid()) return out;

    const ElementGeom g = computeGeom(mesh, element);
    if (!g.valid) return out;

    Eigen::Matrix<double, 12, 1> ue;
    for (int i = 0; i < 12; ++i) ue(i) = displacements[i];

    const Eigen::Matrix<double, 6, 1> sigma = elasticityMatrix(material) * (g.B * ue);
    for (int i = 0; i < 6; ++i) out[i] = sigma(i);
    return out;
}

double vonMises(const std::array<double, 6>& s) {
    const double sxx = s[0], syy = s[1], szz = s[2];
    const double sxy = s[3], syz = s[4], szx = s[5];
    const double dev =
        (sxx - syy) * (sxx - syy) + (syy - szz) * (syy - szz) + (szz - sxx) * (szz - sxx);
    const double shear = sxy * sxy + syz * syz + szx * szx;
    return std::sqrt(0.5 * dev + 3.0 * shear);
}

}  // namespace hz::sim
