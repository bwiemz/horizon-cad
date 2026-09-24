#include "horizon/modeling/Naming.h"

#include <map>
#include <string>
#include <utility>

#include "horizon/topology/HalfEdge.h"
#include "horizon/topology/TopologyID.h"

namespace hz::model {

void nameEdgesByFaces(topo::Solid& solid) {
    std::map<std::string, int> seen;
    for (auto& edge : solid.edges()) {
        const topo::HalfEdge* he = edge.halfEdge;
        if (he == nullptr || he->twin == nullptr || he->face == nullptr ||
            he->twin->face == nullptr || !he->face->topoId.isValid() ||
            !he->twin->face->topoId.isValid()) {
            continue;
        }
        std::string a = he->face->topoId.tag();
        std::string b = he->twin->face->topoId.tag();
        if (b < a) std::swap(a, b);

        // The source both faces share, if they share one.
        std::string tag;
        const size_t slash = a.find('/');
        if (slash != std::string::npos && b.compare(0, slash + 1, a, 0, slash + 1) == 0) {
            tag = a.substr(0, slash) + "/edge:" + a.substr(slash + 1) + "|" + b.substr(slash + 1);
        } else {
            tag = "edge:" + a + "|" + b;
        }
        const int k = seen[tag]++;
        if (k > 0) tag += "#" + std::to_string(k);
        edge.topoId = topo::TopologyID::fromTag(tag);
    }
}

void nameFacePieces(topo::Solid& solid) {
    std::map<std::string, int> count;
    for (const auto& face : solid.faces()) {
        if (face.topoId.isValid()) ++count[face.topoId.tag()];
    }
    std::map<std::string, int> next;
    for (auto& face : solid.faces()) {
        if (!face.topoId.isValid() || count[face.topoId.tag()] < 2) continue;
        const int k = next[face.topoId.tag()]++;
        face.topoId = face.topoId.child("piece", k);
    }
}

}  // namespace hz::model
