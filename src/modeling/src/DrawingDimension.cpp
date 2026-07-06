#include "horizon/modeling/DrawingDimension.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include "horizon/geometry/curves/NurbsCurve.h"
#include "horizon/math/Vec3.h"
#include "horizon/topology/HalfEdge.h"
#include "horizon/topology/Solid.h"

namespace hz::model {

namespace {

/// Direction (unit vector) of the model edge with @p edgeId. Returns false if the
/// edge is missing or degenerate.
bool edgeDirection(const topo::Solid& solid, const topo::TopologyID& edgeId, math::Vec3& outDir) {
    if (!edgeId.isValid()) return false;
    for (const topo::Edge& e : solid.edges()) {
        if (!(e.topoId == edgeId)) continue;
        const topo::HalfEdge* he = e.halfEdge;
        if (he == nullptr || he->origin == nullptr || he->twin == nullptr ||
            he->twin->origin == nullptr) {
            return false;
        }
        const math::Vec3 d = he->twin->origin->point - he->origin->point;
        if (d.length() < 1e-12) return false;
        outDir = d.normalized();
        return true;
    }
    return false;
}

}  // namespace

bool DrawingDimensioner::measureEdge(const topo::Solid& solid, const topo::TopologyID& edgeId,
                                     double& outLength) {
    if (!edgeId.isValid()) return false;

    for (const topo::Edge& e : solid.edges()) {
        if (!(e.topoId == edgeId)) continue;
        const topo::HalfEdge* he = e.halfEdge;
        if (he == nullptr || he->origin == nullptr || he->twin == nullptr ||
            he->twin->origin == nullptr) {
            return false;
        }
        outLength = he->origin->point.distanceTo(he->twin->origin->point);
        return true;
    }
    return false;
}

bool DrawingDimensioner::dimensionEdge(const topo::Solid& solid, const topo::TopologyID& edgeId,
                                       LinearDimension& out) {
    double length = 0.0;
    if (!measureEdge(solid, edgeId, length)) return false;
    out.edge = edgeId;
    out.value = length;
    return true;
}

bool DrawingDimensioner::measureAngle(const topo::Solid& solid, const topo::TopologyID& edgeA,
                                      const topo::TopologyID& edgeB, double& outRadians) {
    math::Vec3 dirA;
    math::Vec3 dirB;
    if (!edgeDirection(solid, edgeA, dirA) || !edgeDirection(solid, edgeB, dirB)) {
        return false;
    }
    // Unsigned line-to-line angle: fold direction sign with abs(), clamp for
    // numerical safety.
    const double c = std::clamp(std::abs(dirA.dot(dirB)), 0.0, 1.0);
    outRadians = std::acos(c);
    return true;
}

bool DrawingDimensioner::dimensionAngle(const topo::Solid& solid, const topo::TopologyID& edgeA,
                                        const topo::TopologyID& edgeB, AngularDimension& out) {
    double radians = 0.0;
    if (!measureAngle(solid, edgeA, edgeB, radians)) return false;
    out.edgeA = edgeA;
    out.edgeB = edgeB;
    out.value = radians;
    return true;
}

bool DrawingDimensioner::measureRadius(const topo::Solid& solid, const topo::TopologyID& edgeId,
                                       double& outRadius) {
    if (!edgeId.isValid()) return false;

    const geo::NurbsCurve* curve = nullptr;
    for (const topo::Edge& e : solid.edges()) {
        if (!(e.topoId == edgeId)) continue;
        curve = e.curve.get();
        break;
    }
    if (curve == nullptr) return false;

    // Sample the curve and fit a circle through three spread samples, then
    // verify every sample sits on it — that separates true circles/arcs from
    // straight edges and free-form splines.
    constexpr int kSamples = 16;
    std::vector<math::Vec3> pts;
    pts.reserve(kSamples);
    const double t0 = curve->tMin();
    const double t1 = curve->tMax();
    for (int i = 0; i < kSamples; ++i) {
        const double t = t0 + (t1 - t0) * (static_cast<double>(i) / (kSamples - 1));
        pts.push_back(curve->evaluate(t));
    }

    const math::Vec3& a = pts[0];
    const math::Vec3& b = pts[kSamples / 3];
    const math::Vec3& c = pts[(2 * kSamples) / 3];

    // Circumcenter of (a, b, c) in their common plane.
    const math::Vec3 d1 = b - a;
    const math::Vec3 d2 = c - a;
    const double d11 = d1.dot(d1);
    const double d22 = d2.dot(d2);
    const double d12 = d1.dot(d2);
    const double denom = 2.0 * (d11 * d22 - d12 * d12);
    if (std::abs(denom) < 1e-12) return false;  // collinear — a straight edge
    const double u = (d22 * (d11 - d12)) / denom;
    const double v = (d11 * (d22 - d12)) / denom;
    const math::Vec3 center = a + d1 * u + d2 * v;

    const double radius = a.distanceTo(center);
    if (radius < 1e-12) return false;

    const double tol = std::max(1e-9, radius * 1e-6);
    for (const math::Vec3& p : pts) {
        if (std::abs(p.distanceTo(center) - radius) > tol) return false;  // not circular
    }

    outRadius = radius;
    return true;
}

bool DrawingDimensioner::dimensionRadius(const topo::Solid& solid, const topo::TopologyID& edgeId,
                                         bool diameter, RadialDimension& out) {
    double radius = 0.0;
    if (!measureRadius(solid, edgeId, radius)) return false;
    out.edge = edgeId;
    out.value = radius;
    out.diameter = diameter;
    return true;
}

}  // namespace hz::model
