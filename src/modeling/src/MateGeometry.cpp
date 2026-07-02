#include "horizon/modeling/MateGeometry.h"

#include <Eigen/Dense>
#include <cmath>
#include <vector>

#include "horizon/geometry/surfaces/NurbsSurface.h"
#include "horizon/topology/Queries.h"

namespace hz::model {

using hz::math::Vec3;

MateFrame MateFrame::transformed(const math::Mat4& m) const {
    MateFrame out = *this;
    out.origin = m.transformPoint(origin);
    out.direction = m.transformDirection(direction).normalized();
    return out;
}

const topo::Face* MateGeometry::findFace(const topo::Solid& solid, const topo::TopologyID& id) {
    if (!id.isValid()) return nullptr;

    const topo::Face* descendant = nullptr;
    for (const auto& face : solid.faces()) {
        if (face.topoId == id) return &face;
        if (descendant == nullptr && face.topoId.isDescendantOf(id)) {
            descendant = &face;
        }
    }
    return descendant;
}

std::optional<MateFrame> MateGeometry::frameForFace(const topo::Face& face) {
    if (!face.surface) return std::nullopt;
    const auto& surface = *face.surface;

    const double u0 = surface.uMin(), u1 = surface.uMax();
    const double v0 = surface.vMin(), v1 = surface.vMax();

    // Sample surface points and normals on a 3x3 parameter grid.
    std::vector<Vec3> points;
    std::vector<Vec3> normals;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            const double u = u0 + (u1 - u0) * (0.5 * i);
            const double v = v0 + (v1 - v0) * (0.5 * j);
            points.push_back(surface.evaluate(u, v));
            Vec3 n = surface.normal(u, v);
            if (n.length() < 1e-12) continue;  // degenerate sample (apex etc.)
            normals.push_back(n.normalized());
        }
    }
    if (normals.size() < 4) return std::nullopt;

    // Planar test: all normals agree.
    constexpr double kPlanarTol = 1e-6;
    bool planar = true;
    for (const auto& n : normals) {
        if (n.dot(normals.front()) < 1.0 - kPlanarTol) {
            planar = false;
            break;
        }
    }

    if (planar) {
        MateFrame frame;
        frame.kind = MateFrameKind::Planar;
        frame.direction = normals.front();
        // Origin: centroid of the face's vertex loop (more representative
        // than the parameter-space midpoint for trimmed faces).
        auto verts = topo::faceVertices(&face);
        if (!verts.empty()) {
            Vec3 sum = Vec3::Zero;
            for (const auto* v : verts) sum = sum + v->point;
            frame.origin = sum * (1.0 / static_cast<double>(verts.size()));
        } else {
            frame.origin = surface.evaluate(0.5 * (u0 + u1), 0.5 * (v0 + v1));
        }
        return frame;
    }

    // Cylindrical extraction.
    // Axis direction: the vector orthogonal to all sampled normals — the
    // eigenvector of sum(n nᵀ) with the smallest eigenvalue.
    Eigen::Matrix3d scatter = Eigen::Matrix3d::Zero();
    for (const auto& n : normals) {
        Eigen::Vector3d en(n.x, n.y, n.z);
        scatter += en * en.transpose();
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(scatter);
    Eigen::Vector3d eAxis = eig.eigenvectors().col(0);
    // A genuine cylinder has one near-zero eigenvalue; reject cones/spheres.
    if (eig.eigenvalues()(0) > 1e-6 * eig.eigenvalues()(2)) return std::nullopt;
    Vec3 axis(eAxis.x(), eAxis.y(), eAxis.z());
    axis = axis.normalized();

    // Radius and axis point via least squares in the plane orthogonal to
    // the axis: for each sample, origin_perp + r * n_perp = p_perp.
    Vec3 e1 = axis.cross(std::abs(axis.x) < 0.9 ? Vec3(1, 0, 0) : Vec3(0, 1, 0)).normalized();
    Vec3 e2 = axis.cross(e1).normalized();

    const auto sampleCount = static_cast<Eigen::Index>(std::min(points.size(), normals.size()));
    Eigen::MatrixXd A(2 * sampleCount, 3);
    Eigen::VectorXd b(2 * sampleCount);
    for (Eigen::Index i = 0; i < sampleCount; ++i) {
        const Vec3& p = points[static_cast<size_t>(i)];
        const Vec3& n = normals[static_cast<size_t>(i)];
        A(2 * i, 0) = 1.0;
        A(2 * i, 1) = 0.0;
        A(2 * i, 2) = e1.dot(n);
        b(2 * i) = e1.dot(p);
        A(2 * i + 1, 0) = 0.0;
        A(2 * i + 1, 1) = 1.0;
        A(2 * i + 1, 2) = e2.dot(n);
        b(2 * i + 1) = e2.dot(p);
    }
    Eigen::Vector3d sol = A.colPivHouseholderQr().solve(b);

    MateFrame frame;
    frame.kind = MateFrameKind::Cylindrical;
    frame.direction = axis;
    frame.origin = e1 * sol(0) + e2 * sol(1);
    frame.radius = std::abs(sol(2));
    return frame;
}

}  // namespace hz::model
