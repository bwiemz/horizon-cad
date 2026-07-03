#include "horizon/modeling/DrawingProjection.h"

#include <algorithm>
#include <cmath>

#include "horizon/geometry/curves/NurbsCurve.h"
#include "horizon/modeling/ExactPredicates.h"
#include "horizon/topology/HalfEdge.h"
#include "horizon/topology/Solid.h"

namespace hz::model {

using hz::math::Vec2;
using hz::math::Vec3;

namespace {

/// Orthonormal 2D basis on the view plane plus the normalized view direction.
struct Basis {
    Vec3 right;
    Vec3 upn;
    Vec3 dir;  ///< normalized view direction (into the screen)
};

Basis makeBasis(const ViewProjection& view) {
    Vec3 dir = view.dir.normalized();
    Vec3 right = dir.cross(view.up);
    if (right.length() < 1e-12) {
        // `up` is parallel to `dir`; pick any perpendicular reference instead.
        const Vec3 alt = (std::abs(dir.x) < 0.9) ? Vec3(1.0, 0.0, 0.0) : Vec3(0.0, 1.0, 0.0);
        right = dir.cross(alt);
    }
    right = right.normalized();
    const Vec3 upn = right.cross(dir).normalized();
    return {right, upn, dir};
}

Vec2 projectPoint(const Vec3& p, const ViewProjection& view, const Basis& b) {
    const Vec3 rel = p - view.origin;
    return Vec2(rel.dot(b.right), rel.dot(b.upn));
}

/// Möller–Trumbore ray/triangle test; forward hits only. Returns the hit
/// distance in @p t.
bool rayTriangle(const Vec3& origin, const Vec3& dir, const Vec3& v0, const Vec3& v1,
                 const Vec3& v2, double& t) {
    const Vec3 e1 = v1 - v0;
    const Vec3 e2 = v2 - v0;
    const Vec3 h = dir.cross(e2);
    const double a = e1.dot(h);
    if (std::abs(a) < 1e-15) return false;
    const double f = 1.0 / a;
    const Vec3 s = origin - v0;
    const double u = f * s.dot(h);
    if (u < 0.0 || u > 1.0) return false;
    const Vec3 q = s.cross(e1);
    const double v = f * dir.dot(q);
    if (v < 0.0 || u + v > 1.0) return false;
    t = f * e2.dot(q);
    return t > 0.0;
}

/// True if the ray from @p origin along @p dir hits the mesh at t > minT.
bool rayHitsMesh(const Vec3& origin, const Vec3& dir, const std::vector<Vec3>& mesh, double minT) {
    for (size_t i = 0; i + 2 < mesh.size(); i += 3) {
        double t = 0.0;
        if (rayTriangle(origin, dir, mesh[i], mesh[i + 1], mesh[i + 2], t) && t > minT) {
            return true;
        }
    }
    return false;
}

/// A segment is hidden if a ray from its midpoint toward the viewer strikes the
/// solid. Self-hits from the edge's own adjacent faces sit at t≈0 and are
/// skipped by @p minT.
bool segmentVisible(const Vec3& p0, const Vec3& p1, const Vec3& viewDir,
                    const std::vector<Vec3>& mesh, double minT) {
    const Vec3 mid = (p0 + p1) * 0.5;
    const Vec3 toViewer = -viewDir;  // out of the screen
    return !rayHitsMesh(mid, toViewer, mesh, minT);
}

/// Sample an edge into a polyline of 3D points (>= 2). Straight edges use their
/// two endpoints; curved edges are evaluated across their parameter domain.
std::vector<Vec3> sampleEdge(const topo::Edge& edge) {
    std::vector<Vec3> pts;
    const topo::HalfEdge* he = edge.halfEdge;
    if (he == nullptr || he->origin == nullptr || he->twin == nullptr ||
        he->twin->origin == nullptr) {
        return pts;
    }

    if (edge.curve && edge.curve->degree() > 1) {
        constexpr int kSamples = 16;
        const double t0 = edge.curve->tMin();
        const double t1 = edge.curve->tMax();
        pts.reserve(kSamples + 1);
        for (int i = 0; i <= kSamples; ++i) {
            const double t = t0 + (t1 - t0) * (static_cast<double>(i) / kSamples);
            pts.push_back(edge.curve->evaluate(t));
        }
    } else {
        pts.push_back(he->origin->point);
        pts.push_back(he->twin->origin->point);
    }
    return pts;
}

double meshDiagonal(const std::vector<Vec3>& mesh) {
    if (mesh.empty()) return 1.0;
    Vec3 lo = mesh[0];
    Vec3 hi = mesh[0];
    for (const Vec3& p : mesh) {
        lo = Vec3(std::min(lo.x, p.x), std::min(lo.y, p.y), std::min(lo.z, p.z));
        hi = Vec3(std::max(hi.x, p.x), std::max(hi.y, p.y), std::max(hi.z, p.z));
    }
    const double d = (hi - lo).length();
    return d > 1e-9 ? d : 1.0;
}

}  // namespace

std::vector<ProjectedEdge> DrawingProjection::project(const topo::Solid& solid,
                                                      const ViewProjection& view) {
    const Basis basis = makeBasis(view);

    // Build the occluder mesh once and reuse it across every visibility query.
    const std::vector<Vec3> mesh = ExactPredicates::tessellateSolid(solid);
    // Skip self-hits from the edge's own adjacent faces (which sit at t≈0)
    // without missing genuine occluders (at t ~ model scale).
    const double minT = meshDiagonal(mesh) * 1e-3;

    std::vector<ProjectedEdge> out;
    for (const auto& edge : solid.edges()) {
        const std::vector<Vec3> samples = sampleEdge(edge);
        if (samples.size() < 2) continue;

        for (size_t i = 0; i + 1 < samples.size(); ++i) {
            const Vec3& p0 = samples[i];
            const Vec3& p1 = samples[i + 1];

            ProjectedEdge pe;
            pe.a = projectPoint(p0, view, basis);
            pe.b = projectPoint(p1, view, basis);
            pe.sourceEdge = edge.topoId;
            pe.visibility = segmentVisible(p0, p1, basis.dir, mesh, minT)
                                ? ProjectedEdge::Visibility::Visible
                                : ProjectedEdge::Visibility::Hidden;
            out.push_back(pe);
        }
    }
    return out;
}

ViewProjection DrawingProjection::standardView(StandardView view) {
    switch (view) {
        case StandardView::Front:  // look along -Y; Z is up
            return {Vec3(0, 0, 0), Vec3(0, -1, 0), Vec3(0, 0, 1)};
        case StandardView::Top:  // look along -Z; Y is up
            return {Vec3(0, 0, 0), Vec3(0, 0, -1), Vec3(0, 1, 0)};
        case StandardView::Right:  // look along -X; Z is up
            return {Vec3(0, 0, 0), Vec3(-1, 0, 0), Vec3(0, 0, 1)};
        case StandardView::Isometric:  // look from (+,+,+) toward the origin
            return {Vec3(0, 0, 0), Vec3(-1, -1, -1), Vec3(0, 0, 1)};
    }
    return {};
}

}  // namespace hz::model
