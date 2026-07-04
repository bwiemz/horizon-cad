#include "horizon/modeling/DrawingDimension.h"

#include <algorithm>
#include <cmath>

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

}  // namespace hz::model
