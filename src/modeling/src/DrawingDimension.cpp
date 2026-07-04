#include "horizon/modeling/DrawingDimension.h"

#include "horizon/topology/HalfEdge.h"
#include "horizon/topology/Solid.h"

namespace hz::model {

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

}  // namespace hz::model
