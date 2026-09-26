#include "horizon/modeling/MateGeometry.h"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <vector>

#include "horizon/geometry/surfaces/NurbsSurface.h"
#include "horizon/modeling/EdgeProjection.h"
#include "horizon/modeling/Naming.h"
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

    const topo::Face* facet = nullptr;
    const topo::Face* descendant = nullptr;
    for (const auto& face : solid.faces()) {
        if (face.topoId == id) return &face;
        // A facet of the curved face it names (Stable names): its ideal is
        // the face's. Before a pattern copy's, which is a descendant too.
        if (facet == nullptr && logicalFace(face.topoId.tag()) == id.tag()) facet = &face;
        if (descendant == nullptr && face.topoId.isDescendantOf(id)) {
            descendant = &face;
        }
    }
    return facet != nullptr ? facet : descendant;
}

std::optional<MateFrame> MateGeometry::frameForFace(const topo::Face& face) {
    // Prefer the ideal surface the face approximates over its carrier.
    // Curved primitives are faceted, so a lateral facet's carrier is a plane
    // and would yield a planar mate frame; the analytic surface is what lets
    // one pick on that facet still resolve the cylinder it belongs to.
    const auto* carrier = face.analyticSurface ? face.analyticSurface.get() : face.surface.get();
    if (carrier == nullptr) return std::nullopt;
    const auto& surface = *carrier;

    const double u0 = surface.uMin(), u1 = surface.uMax();
    const double v0 = surface.vMin(), v1 = surface.vMax();

    // Sample surface points and normals on a 5x5 parameter grid, each point
    // with its normal: a degenerate sample (an apex, a pole) is left out
    // whole. Keeping its point and not its normal put the two lists out of
    // step, and every fit below reads them in pairs.
    std::vector<Vec3> sampled;
    std::vector<Vec3> sampledNormals;
    for (int i = 0; i < 5; ++i) {
        for (int j = 0; j < 5; ++j) {
            const double u = u0 + (u1 - u0) * (0.25 * i);
            const double v = v0 + (v1 - v0) * (0.25 * j);
            const Vec3 n = surface.normal(u, v);
            if (n.length() < 1e-12) continue;
            sampled.push_back(surface.evaluate(u, v));
            sampledNormals.push_back(n.normalized());
        }
    }
    // Nor a sample at a point the patch collapses to (a sphere's pole, a
    // cone's apex): several parameters meet there, and its normal, however
    // long, is none of the surface's.
    double extent = 0.0;
    for (const Vec3& p : sampled) extent = std::max(extent, (p - sampled.front()).length());
    const double coincident = 1e-9 * std::max(extent, 1.0);
    std::vector<Vec3> points;
    std::vector<Vec3> normals;
    for (size_t i = 0; i < sampled.size(); ++i) {
        const bool collapsed = std::any_of(sampled.begin(), sampled.end(), [&](const Vec3& q) {
            return &q != &sampled[i] && (q - sampled[i]).length() <= coincident;
        });
        if (collapsed) continue;
        points.push_back(sampled[i]);
        normals.push_back(sampledNormals[i]);
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

    const auto count = static_cast<Eigen::Index>(points.size());
    double scale = 0.0;
    for (const Vec3& p : points) scale = std::max(scale, (p - points.front()).length());
    const double fitTol = 1e-6 * std::max(scale, 1.0);

    // Sphere (Phase 160): every sample its centre plus radius times its
    // normal, one centre and one radius for all.
    {
        Eigen::MatrixXd A(3 * count, 4);
        Eigen::VectorXd b(3 * count);
        for (Eigen::Index i = 0; i < count; ++i) {
            const Vec3& p = points[static_cast<size_t>(i)];
            const Vec3& n = normals[static_cast<size_t>(i)];
            const double pc[3] = {p.x, p.y, p.z};
            const double nc[3] = {n.x, n.y, n.z};
            for (int k = 0; k < 3; ++k) {
                A.row(3 * i + k) << (k == 0 ? 1.0 : 0.0), (k == 1 ? 1.0 : 0.0),
                    (k == 2 ? 1.0 : 0.0), nc[k];
                b(3 * i + k) = pc[k];
            }
        }
        const Eigen::Vector4d fit = A.colPivHouseholderQr().solve(b);
        const double fitError = (A * fit - b).lpNorm<Eigen::Infinity>();
        if (std::abs(fit(3)) > fitTol && fitError < fitTol) {
            MateFrame frame;
            frame.kind = MateFrameKind::Spherical;
            frame.origin = Vec3(fit(0), fit(1), fit(2));
            frame.direction = Vec3::UnitZ;
            frame.radius = std::abs(fit(3));
            return frame;
        }
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
    if (eig.eigenvalues()(0) > 1e-6 * eig.eigenvalues()(2)) {
        return conicalFrame(points, normals, fitTol);  // not a cylinder: a cone, or nothing
    }
    Eigen::Vector3d eAxis = eig.eigenvectors().col(0);
    Vec3 axis(eAxis.x(), eAxis.y(), eAxis.z());
    axis = axis.normalized();

    // Radius and axis point via least squares in the plane orthogonal to
    // the axis: for each sample, origin_perp + r * n_perp = p_perp.
    Vec3 e1 = axis.cross(std::abs(axis.x) < 0.9 ? Vec3(1, 0, 0) : Vec3(0, 1, 0)).normalized();
    Vec3 e2 = axis.cross(e1).normalized();

    Eigen::MatrixXd A(2 * count, 3);
    Eigen::VectorXd b(2 * count);
    for (Eigen::Index i = 0; i < count; ++i) {
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

std::optional<MateFrame> MateGeometry::conicalFrame(const std::vector<Vec3>& points,
                                                    const std::vector<Vec3>& normals,
                                                    double tolerance) {
    // A cone's normals make one angle with its axis: their tips lie on a
    // circle of the unit sphere, in a plane facing along the axis.
    const auto count = static_cast<Eigen::Index>(normals.size());
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    for (const Vec3& n : normals) mean += Eigen::Vector3d(n.x, n.y, n.z);
    mean /= static_cast<double>(count);
    Eigen::Matrix3d spread = Eigen::Matrix3d::Zero();
    for (const Vec3& n : normals) {
        const Eigen::Vector3d d = Eigen::Vector3d(n.x, n.y, n.z) - mean;
        spread += d * d.transpose();
    }
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(spread);
    const Eigen::Vector3d e = eig.eigenvectors().col(0);
    Vec3 axis = Vec3(e.x(), e.y(), e.z()).normalized();
    const double along = axis.dot(Vec3(mean.x(), mean.y(), mean.z()));
    for (const Vec3& n : normals) {
        if (std::abs(n.dot(axis) - along) > 1e-6) return std::nullopt;
    }
    // Its apex is on every tangent plane: n . apex = n . p for each sample.
    Eigen::MatrixXd A(count, 3);
    Eigen::VectorXd b(count);
    for (Eigen::Index i = 0; i < count; ++i) {
        const Vec3& n = normals[static_cast<size_t>(i)];
        A.row(i) << n.x, n.y, n.z;
        b(i) = n.dot(points[static_cast<size_t>(i)]);
    }
    const Eigen::Vector3d apex = A.colPivHouseholderQr().solve(b);
    if ((A * apex - b).lpNorm<Eigen::Infinity>() > tolerance) return std::nullopt;
    MateFrame frame;
    frame.kind = MateFrameKind::Conical;
    frame.origin = Vec3(apex.x(), apex.y(), apex.z());
    // Pointing from the apex into the cone.
    Vec3 centroid;
    for (const Vec3& p : points) centroid = centroid + p;
    centroid = centroid * (1.0 / static_cast<double>(points.size()));
    if ((centroid - frame.origin).dot(axis) < 0.0) axis = axis * -1.0;
    frame.direction = axis;
    frame.angle = std::asin(std::clamp(std::abs(along), 0.0, 1.0));  // from the axis to the side
    return frame;
}

std::optional<MateFrame> MateGeometry::frameForEdge(const topo::Solid& solid,
                                                    const std::string& edge) {
    // Its chords' ends: every point of it the part keeps.
    std::vector<Vec3> points;
    for (const auto& e : solid.edges()) {
        const topo::HalfEdge* he = e.halfEdge;
        if (!e.topoId.isValid() || he == nullptr || he->origin == nullptr || he->next == nullptr ||
            he->next->origin == nullptr) {
            continue;
        }
        if (wholeEdgeName(e.topoId.tag()) != edge) continue;
        points.push_back(he->origin->point);
        points.push_back(he->next->origin->point);
    }
    if (points.size() < 2) return std::nullopt;
    Vec3 far = points.front();
    for (const Vec3& p : points) {
        if ((p - points.front()).length() > (far - points.front()).length()) far = p;
    }
    const double span = (far - points.front()).length();
    if (span < 1e-12) return std::nullopt;
    const Vec3 along = (far - points.front()) * (1.0 / span);
    const double tolerance = 1e-9 * std::max(span, 1.0);

    // Straight: a line, through its first point.
    Vec3 widest = points.front();
    double off = 0.0;
    for (const Vec3& p : points) {
        const Vec3 d = p - points.front();
        const double o = (d - along * d.dot(along)).length();
        if (o > off) {
            off = o;
            widest = p;
        }
    }
    if (off <= tolerance) {
        MateFrame frame;
        frame.kind = MateFrameKind::Line;
        frame.origin = points.front();
        frame.direction = along;
        return frame;
    }

    // Round: the circle through three points well apart, which the others
    // must be on.
    const Vec3 a = points.front();
    const Vec3 ab = far - a;
    const Vec3 ac = widest - a;
    const Vec3 normal = ab.cross(ac);
    const double n2 = normal.dot(normal);
    if (n2 < 1e-24) return std::nullopt;
    const Vec3 centre =
        a + (normal.cross(ab) * ac.dot(ac) + ac.cross(normal) * ab.dot(ab)) * (0.5 / n2);
    const double radius = (a - centre).length();
    const Vec3 unit = normal.normalized();
    for (const Vec3& p : points) {
        if (std::abs((p - centre).length() - radius) > 1e-6 * std::max(radius, 1.0) ||
            std::abs((p - centre).dot(unit)) > 1e-6 * std::max(radius, 1.0)) {
            return std::nullopt;  // neither straight nor round
        }
    }
    MateFrame frame;
    frame.kind = MateFrameKind::Circle;
    frame.origin = centre;
    frame.direction = unit;
    frame.radius = radius;
    return frame;
}

}  // namespace hz::model
