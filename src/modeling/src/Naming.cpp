#include "horizon/modeling/Naming.h"

#include <cctype>
#include <map>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

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

namespace {

/// The two face names an edge parts, in order; empty when either face has none.
std::pair<std::string, std::string> facePair(const topo::Edge& edge) {
    const topo::HalfEdge* he = edge.halfEdge;
    if (he == nullptr || he->twin == nullptr || he->face == nullptr || he->twin->face == nullptr ||
        !he->face->topoId.isValid() || !he->twin->face->topoId.isValid()) {
        return {};
    }
    std::string a = he->face->topoId.tag();
    std::string b = he->twin->face->topoId.tag();
    if (b < a) std::swap(a, b);
    return {a, b};
}

}  // namespace

namespace {

/// @p tag without its `/<part>:<k>` component, if it has one, and the
/// `/piece:<n>` of a piece of it: what else follows it (a pattern copy's
/// `/pattern:1`) is kept, so a copy's curve is its own.
std::string without(const std::string& tag, const char* part) {
    const size_t at = tag.find(part);
    if (at == std::string::npos) return tag;
    static constexpr std::string_view kPiece = "/piece:";
    size_t end = tag.find('/', at + 1);
    while (end != std::string::npos && tag.compare(end, kPiece.size(), kPiece) == 0) {
        end = tag.find('/', end + 1);
    }
    return tag.substr(0, at) + (end == std::string::npos ? std::string() : tag.substr(end));
}

}  // namespace

std::string logicalFace(const std::string& faceTag) {
    return without(faceTag, "/facet:");
}

std::string logicalEdge(const std::string& edgeTag) {
    return without(edgeTag, "/chord:");
}

void nameEdgesLogically(topo::Solid& solid) {
    // Each edge's base name, from its logical faces; how many share it.
    std::vector<std::string> base(solid.edges().size());
    std::map<std::string, int> count;
    size_t i = 0;
    for (const auto& edge : solid.edges()) {
        const auto [a, b] = facePair(edge);
        std::string& name = base[i++];
        if (a.empty()) continue;
        const std::string la = logicalFace(a);
        const std::string lb = logicalFace(b);
        if (la == lb) {
            name = la + "/seam";  // between two facets of one face
        } else {
            const size_t slash = la.find('/');
            if (slash != std::string::npos && lb.compare(0, slash + 1, la, 0, slash + 1) == 0) {
                name = la.substr(0, slash) + "/edge:" + la.substr(slash + 1) + "|" +
                       lb.substr(slash + 1);
            } else {
                name = "edge:" + la + "|" + lb;
            }
        }
        ++count[name];
    }
    std::map<std::string, int> next;
    i = 0;
    for (auto& edge : solid.edges()) {
        const std::string& name = base[i++];
        if (name.empty()) continue;
        const bool seam = name.size() > 5 && name.compare(name.size() - 5, 5, "/seam") == 0;
        if (!seam && count[name] == 1) {
            edge.topoId = topo::TopologyID::fromTag(name);
            continue;
        }
        const int k = next[name]++;
        edge.topoId = topo::TopologyID::fromTag(seam ? name + ":" + std::to_string(k)
                                                     : name + "/chord:" + std::to_string(k));
    }
}

void nameEdges(topo::Solid& solid, NamingScheme naming) {
    if (naming == NamingScheme::Stable) {
        nameEdgesLogically(solid);
    } else if (naming == NamingScheme::FromGeometry) {
        nameEdgesByFaces(solid);
    }
}

void nameFacetsLogically(topo::Solid& solid) {
    std::map<const void*, std::vector<topo::Face*>> bySurface;
    for (auto& face : solid.faces()) {
        if (face.analyticSurface && face.topoId.isValid()) {
            bySurface[face.analyticSurface.get()].push_back(&face);
        }
    }
    for (auto& [surface, facets] : bySurface) {
        if (facets.size() < 2) continue;
        // What their names share, less the trailing digits and underscores
        // that tell them apart.
        std::string shared = facets.front()->topoId.tag();
        for (const topo::Face* f : facets) {
            const std::string& tag = f->topoId.tag();
            size_t n = 0;
            while (n < shared.size() && n < tag.size() && shared[n] == tag[n]) ++n;
            shared.resize(n);
        }
        while (!shared.empty() &&
               (std::isdigit(static_cast<unsigned char>(shared.back())) || shared.back() == '_')) {
            shared.pop_back();
        }
        if (shared.empty() || shared.back() == '/') shared += "surface";
        for (size_t k = 0; k < facets.size(); ++k) {
            facets[k]->topoId = topo::TopologyID::fromTag(shared + "/facet:" + std::to_string(k));
        }
    }
}

void keepEdgeNames(topo::Solid& result, const topo::Solid& input) {
    // The input's edges, by the faces they part: kept only where one edge
    // parts them.
    std::map<std::pair<std::string, std::string>, std::string> before;
    std::map<std::pair<std::string, std::string>, int> countBefore;
    for (const auto& edge : input.edges()) {
        const auto pair = facePair(edge);
        if (pair.first.empty() || !edge.topoId.isValid()) continue;
        before[pair] = edge.topoId.tag();
        ++countBefore[pair];
    }
    std::map<std::pair<std::string, std::string>, int> countAfter;
    for (const auto& edge : result.edges()) {
        const auto pair = facePair(edge);
        if (!pair.first.empty()) ++countAfter[pair];
    }
    nameEdgesLogically(result);
    for (auto& edge : result.edges()) {
        const auto pair = facePair(edge);
        if (pair.first.empty() || countAfter[pair] != 1 || countBefore[pair] != 1) continue;
        edge.topoId = topo::TopologyID::fromTag(before[pair]);
    }
}

void nameBlendFaces(topo::Solid& solid, const std::string& prefix) {
    // In storage order, which the operation's input fixes.
    std::map<std::string, std::vector<topo::Face*>> byCurve;
    for (auto& face : solid.faces()) {
        const std::string& tag = face.topoId.tag();
        if (!face.topoId.isValid() || tag.compare(0, prefix.size(), prefix) != 0) continue;
        const size_t colon = tag.rfind(':');
        if (colon == std::string::npos || colon <= prefix.size()) continue;
        byCurve[logicalEdge(tag.substr(prefix.size(), colon - prefix.size()))].push_back(&face);
    }
    for (auto& [curve, faces] : byCurve) {
        const std::string name = prefix + curve;
        for (size_t k = 0; k < faces.size(); ++k) {
            faces[k]->topoId = topo::TopologyID::fromTag(
                faces.size() == 1 ? name : name + "/facet:" + std::to_string(k));
        }
    }
}

void scopeToFeature(topo::Solid& solid, const std::string& featureID) {
    const auto scoped = [&featureID](const topo::TopologyID& id) {
        const std::string& tag = id.tag();
        const size_t slash = tag.find('/');
        return topo::TopologyID::fromTag(
            featureID + "/" + (slash == std::string::npos ? tag : tag.substr(slash + 1)));
    };
    for (auto& face : solid.faces()) {
        if (face.topoId.isValid()) face.topoId = scoped(face.topoId);
    }
    for (auto& edge : solid.edges()) {
        if (edge.topoId.isValid()) edge.topoId = scoped(edge.topoId);
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
