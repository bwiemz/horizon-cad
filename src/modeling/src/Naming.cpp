#include "horizon/modeling/Naming.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <string_view>
#include <tuple>
#include <unordered_map>
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
/// `/piece:<n>` of any piece of it after that (a boolean appends it, after a
/// pattern copy's `/pattern:1` too): what else follows is kept, so a copy's
/// curve is its own.
std::string without(const std::string& tag, const char* part) {
    const size_t at = tag.find(part);
    if (at == std::string::npos) return tag;
    static constexpr std::string_view kPiece = "/piece:";
    std::string out = tag.substr(0, at);
    size_t from = tag.find('/', at + 1);
    while (from != std::string::npos) {
        const size_t next = tag.find('/', from + 1);
        if (tag.compare(from, kPiece.size(), kPiece) != 0) {
            out += tag.substr(from, next == std::string::npos ? std::string::npos : next - from);
        }
        from = next;
    }
    return out;
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

void nameBridgeEdges(topo::Solid& faceted, const topo::Solid& exact) {
    // Every name the exact solid's edges have: a chord of one keeps its name, and no line is
    // named so.
    std::set<std::string> exactNames;
    for (const auto& edge : exact.edges()) {
        if (edge.topoId.isValid()) exactNames.insert(edge.topoId.tag());
    }
    std::map<std::string, int> next;  // by face: the number its next seam may take
    for (auto& edge : faceted.edges()) {
        if (edge.topoId.isValid() && exactNames.count(logicalEdge(edge.topoId.tag())) != 0) {
            continue;  // a chord of an exact edge
        }
        const auto [a, b] = facePair(edge);
        if (a.empty()) continue;
        const std::string face = logicalFace(a);
        if (logicalFace(b) != face) continue;  // not inside one face
        int& n = next[face];
        std::string name = face + "/seam:" + std::to_string(n++);
        while (exactNames.count(name) != 0) name = face + "/seam:" + std::to_string(n++);
        edge.topoId = topo::TopologyID::fromTag(name);
    }
}

namespace {

/// The name nameEdgesLogically gives an edge between faces named @p a and @p b (its base, before
/// any `/chord:<k>` or `:<k>` of a seam's), and whether it is a seam.
std::pair<std::string, bool> logicalBase(std::string a, std::string b) {
    if (b < a) std::swap(a, b);
    const std::string la = logicalFace(a);
    const std::string lb = logicalFace(b);
    if (la == lb) return {la + "/seam", true};
    const size_t slash = la.find('/');
    if (slash != std::string::npos && lb.compare(0, slash + 1, la, 0, slash + 1) == 0) {
        return {la.substr(0, slash) + "/edge:" + la.substr(slash + 1) + "|" + lb.substr(slash + 1),
                false};
    }
    return {"edge:" + la + "|" + lb, false};
}

/// @p masters' points, found again near where they are: each within @p within of one.
class PointIndex {
public:
    PointIndex(const std::vector<math::Vec3>& points, double within)
        : points_(points), within_(within), cell_(std::max(within, 1e-300) * 4.0) {
        for (std::size_t i = 0; i < points.size(); ++i) grid_[keyOf(points[i])].push_back(i);
    }

    /// The point nearest @p p within the distance given, if one is.
    std::optional<std::size_t> find(const math::Vec3& p) const {
        const Key k = keyOf(p);
        std::optional<std::size_t> best;
        double nearest = within_;
        for (std::int64_t dx = -1; dx <= 1; ++dx) {
            for (std::int64_t dy = -1; dy <= 1; ++dy) {
                for (std::int64_t dz = -1; dz <= 1; ++dz) {
                    const auto found =
                        grid_.find({std::get<0>(k) + dx, std::get<1>(k) + dy, std::get<2>(k) + dz});
                    if (found == grid_.end()) continue;
                    for (std::size_t i : found->second) {
                        const double d = (points_[i] - p).length();
                        if (d <= nearest) {
                            nearest = d;
                            best = i;
                        }
                    }
                }
            }
        }
        return best;
    }

private:
    using Key = std::tuple<std::int64_t, std::int64_t, std::int64_t>;
    struct KeyHash {
        std::size_t operator()(const Key& k) const {
            const auto h = [](std::int64_t v) { return std::hash<std::int64_t>{}(v); };
            return h(std::get<0>(k)) * 73856093u ^ h(std::get<1>(k)) * 19349663u ^
                   h(std::get<2>(k)) * 83492791u;
        }
    };
    Key keyOf(const math::Vec3& p) const {
        return {static_cast<std::int64_t>(std::floor(p.x / cell_)),
                static_cast<std::int64_t>(std::floor(p.y / cell_)),
                static_cast<std::int64_t>(std::floor(p.z / cell_))};
    }
    const std::vector<math::Vec3>& points_;
    double within_;
    double cell_;
    std::unordered_map<Key, std::vector<std::size_t>, KeyHash> grid_;
};

using PointPair = std::pair<std::size_t, std::size_t>;

PointPair unordered(std::size_t a, std::size_t b) {
    return a < b ? PointPair{a, b} : PointPair{b, a};
}

/// Each of @p masters' edges' name, as nameEdgesLogically gave it: by the two faces it parts,
/// numbered in the order master made the edges.
std::map<PointPair, std::string> mastersEdgeNames(const MastersFacets& masters) {
    std::map<PointPair, std::vector<std::size_t>> facesOf;
    std::vector<PointPair> order;
    for (std::size_t f = 0; f < masters.faces.size(); ++f) {
        const auto& loop = masters.faces[f];
        for (std::size_t i = 0; i < loop.size(); ++i) {
            const PointPair e = unordered(loop[i], loop[(i + 1) % loop.size()]);
            auto& faces = facesOf[e];
            if (faces.empty() && masters.edges.empty()) order.push_back(e);
            faces.push_back(f);
        }
    }
    if (!masters.edges.empty()) {
        for (const auto& e : masters.edges) order.push_back(unordered(e.first, e.second));
    }
    std::vector<std::pair<std::string, bool>> base(order.size());
    std::map<std::string, int> count;
    for (std::size_t i = 0; i < order.size(); ++i) {
        const auto found = facesOf.find(order[i]);
        if (found == facesOf.end() || found->second.size() != 2) continue;
        base[i] =
            logicalBase(masters.faceNames[found->second[0]], masters.faceNames[found->second[1]]);
        ++count[base[i].first];
    }
    std::map<PointPair, std::string> names;
    std::map<std::string, int> next;
    for (std::size_t i = 0; i < order.size(); ++i) {
        const std::string& name = base[i].first;
        if (name.empty()) continue;
        if (!base[i].second && count[name] == 1) {
            names[order[i]] = name;
            continue;
        }
        const int k = next[name]++;
        names[order[i]] =
            base[i].second ? name + ":" + std::to_string(k) : name + "/chord:" + std::to_string(k);
    }
    return names;
}

/// Whether @p tag names a seam, a facet or a chord numbered `<prefix><k>`: its prefix, for
/// renumbering.
std::optional<std::string> numberedPrefix(const std::string& tag) {
    for (const char* part : {"/seam:", "/facet:", "/chord:"}) {
        const size_t at = tag.rfind(part);
        if (at == std::string::npos) continue;
        const size_t from = at + std::char_traits<char>::length(part);
        if (from < tag.size() &&
            std::all_of(tag.begin() + static_cast<std::ptrdiff_t>(from), tag.end(),
                        [](unsigned char c) { return std::isdigit(c) != 0; })) {
            return tag.substr(0, from);
        }
    }
    return std::nullopt;
}

}  // namespace

MastersFacets mastersFacets(const std::vector<SolidSewer::InputFace>& soup,
                            const std::string& curvedSide, const std::string& featureID,
                            const math::Mat4& placement) {
    MastersFacets out;
    // The sewer welds a point every face that meets there shares; the soup shares them exactly.
    std::map<std::tuple<double, double, double>, std::size_t> index;
    std::vector<std::size_t> curved;
    for (const auto& face : soup) {
        std::vector<std::size_t> loop;
        loop.reserve(face.points.size());
        for (const math::Vec3& p : face.points) {
            const auto inserted = index.emplace(std::make_tuple(p.x, p.y, p.z), index.size());
            if (inserted.second) out.points.push_back(placement.transformPoint(p));
            const std::size_t at = inserted.first->second;
            if (loop.empty() || loop.back() != at) loop.push_back(at);
        }
        while (loop.size() > 1 && loop.front() == loop.back()) loop.pop_back();
        if (loop.size() < 3) continue;
        const std::string& tag = face.topoId.tag();
        const size_t slash = tag.find('/');
        if (tag.find(curvedSide) != std::string::npos) curved.push_back(out.faces.size());
        out.faces.push_back(std::move(loop));
        out.faceNames.push_back(featureID + "/" +
                                (slash == std::string::npos ? tag : tag.substr(slash + 1)));
    }
    if (curved.size() < 2) return out;
    // nameFacetsLogically: what their names share, less the digits and underscores that tell them
    // apart, and each its facet in order.
    std::string shared = out.faceNames[curved.front()];
    for (std::size_t f : curved) {
        const std::string& tag = out.faceNames[f];
        size_t n = 0;
        while (n < shared.size() && n < tag.size() && shared[n] == tag[n]) ++n;
        shared.resize(n);
    }
    while (!shared.empty() &&
           (std::isdigit(static_cast<unsigned char>(shared.back())) || shared.back() == '_')) {
        shared.pop_back();
    }
    if (shared.empty() || shared.back() == '/') shared += "surface";
    for (std::size_t k = 0; k < curved.size(); ++k) {
        out.faceNames[curved[k]] = shared + "/facet:" + std::to_string(k);
    }
    return out;
}

void nameAsMasters(topo::Solid& bridge, topo::Solid& exact, const MastersFacets& masters) {
    if (masters.points.empty()) return;
    // Each of the bridge's vertices, master's point there: they are master's vertices (to the
    // rounding of the curves they were evaluated on).
    // Within 1e-9 of the construction's reach: its size, or how far it lies from the origin.
    math::Vec3 lo(std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(),
                  std::numeric_limits<double>::infinity());
    math::Vec3 hi = lo * -1.0;
    double reach = 0.0;
    for (const math::Vec3& p : masters.points) {
        lo = math::Vec3(std::min(lo.x, p.x), std::min(lo.y, p.y), std::min(lo.z, p.z));
        hi = math::Vec3(std::max(hi.x, p.x), std::max(hi.y, p.y), std::max(hi.z, p.z));
        reach = std::max(reach, p.length());
    }
    const double size = std::max((hi - lo).length(), reach);
    const PointIndex points(masters.points, 1e-9 * std::max(size, 1e-300));
    std::unordered_map<const topo::Vertex*, std::size_t> at;
    for (const auto& v : bridge.vertices()) {
        if (!v.alive) continue;
        if (const auto i = points.find(v.point)) at.emplace(&v, *i);
    }
    const auto pointOf = [&at](const topo::Vertex* v) -> std::optional<std::size_t> {
        const auto found = at.find(v);
        if (found == at.end()) return std::nullopt;
        return found->second;
    };

    // Facets: master's face with the same corners.
    std::map<std::vector<std::size_t>, std::size_t> faceAt;
    for (std::size_t f = 0; f < masters.faces.size(); ++f) {
        std::vector<std::size_t> corners = masters.faces[f];
        std::sort(corners.begin(), corners.end());
        faceAt.emplace(std::move(corners), f);
    }
    std::set<std::string> taken;  // the names given, which nothing else may keep
    std::vector<topo::Face*> unmatchedFaces;
    for (auto& face : bridge.faces()) {
        if (!face.alive || face.outerLoop == nullptr || !face.innerLoops.empty()) continue;
        std::vector<std::size_t> corners;
        bool all = true;
        const topo::HalfEdge* he = face.outerLoop->halfEdge;
        do {
            const auto p = pointOf(he->origin);
            if (!p) {
                all = false;
                break;
            }
            corners.push_back(*p);
            he = he->next;
        } while (he != face.outerLoop->halfEdge);
        std::sort(corners.begin(), corners.end());
        const auto found = all ? faceAt.find(corners) : faceAt.end();
        const std::string& name =
            found == faceAt.end() ? std::string() : masters.faceNames[found->second];
        if (found == faceAt.end() || logicalFace(name) != logicalFace(face.topoId.tag())) {
            unmatchedFaces.push_back(&face);
            continue;
        }
        face.topoId = topo::TopologyID::fromTag(name);
        taken.insert(name);
    }

    // Edges: master's edge between the same points.
    const auto names = mastersEdgeNames(masters);
    const auto mastersName = [&](const topo::Edge& e) -> std::string {
        const topo::HalfEdge* he = e.halfEdge;
        if (he == nullptr || he->twin == nullptr) return {};
        const auto a = pointOf(he->origin);
        const auto b = pointOf(he->twin->origin);
        if (!a || !b) return {};
        const auto found = names.find(unordered(*a, *b));
        return found == names.end() ? std::string() : found->second;
    };
    std::map<std::string, topo::Edge*> exactByName;
    for (auto& e : exact.edges()) {
        if (e.alive && e.topoId.isValid()) exactByName[e.topoId.tag()] = &e;
    }
    // Each exact edge's chords on the bridge, in the exact solid's order; the rest are lines of a
    // grid.
    std::unordered_map<const topo::Edge*, std::size_t> exactOrder;
    std::vector<topo::Edge*> exactEdges;
    for (auto& e : exact.edges()) {
        if (!e.alive || !e.topoId.isValid()) continue;
        exactOrder.emplace(&e, exactEdges.size());
        exactEdges.push_back(&e);
    }
    std::vector<std::vector<topo::Edge*>> chordsOf(exactEdges.size());
    std::vector<topo::Edge*> lines;
    for (auto& e : bridge.edges()) {
        if (!e.alive || !e.topoId.isValid()) continue;
        // An exact edge's one chord has its name; each of several, `<its name>/chord:<k>`.
        auto found = exactByName.find(e.topoId.tag());
        if (found == exactByName.end()) found = exactByName.find(logicalEdge(e.topoId.tag()));
        if (found != exactByName.end()) {
            chordsOf[exactOrder.at(found->second)].push_back(&e);
        } else {
            lines.push_back(&e);
        }
    }
    std::vector<topo::Edge*> unmatchedEdges;
    const auto rename = [&taken](topo::Edge& e, const std::string& name) {
        e.topoId = topo::TopologyID::fromTag(name);
        taken.insert(name);
    };
    for (std::size_t x = 0; x < exactEdges.size(); ++x) {
        const std::vector<topo::Edge*>& chords = chordsOf[x];
        if (chords.empty()) continue;
        topo::Edge& exactEdge = *exactEdges[x];
        const std::string name = exactEdge.topoId.tag();
        const topo::HalfEdge* he = exactEdge.halfEdge;
        const bool seam =
            he != nullptr && he->twin != nullptr && he->face != nullptr &&
            he->twin->face != nullptr &&
            logicalFace(he->face->topoId.tag()) == logicalFace(he->twin->face->topoId.tag());
        if (!seam && chords.size() == 1 && chords.front()->topoId.tag() == name) {
            // An edge of one chord between two faces: where two or more part the same faces
            // (`<edge>/chord:<k>`, a half disc's lines along the offset), master numbered them
            // as it made them, and the exact edge takes its chord's number.
            const std::string given = mastersName(*chords.front());
            if (!given.empty() && logicalEdge(given) == logicalEdge(name)) {
                exactEdge.topoId = topo::TopologyID::fromTag(given);
                rename(*chords.front(), given);
            } else {
                unmatchedEdges.push_back(chords.front());
            }
            continue;
        }
        if (!seam) {
            // A curve between two faces: master numbered its chords as it made them.
            for (topo::Edge* chord : chords) {
                const std::string given = mastersName(*chord);
                if (!given.empty() && logicalEdge(given) == name) {
                    rename(*chord, given);
                } else {
                    unmatchedEdges.push_back(chord);
                }
            }
            continue;
        }
        // A seam: named after master's edge where its first chord lies.
        topo::Edge* first = chords.front();
        for (topo::Edge* chord : chords) {
            if (chord->topoId.tag() == name || chord->topoId.tag() == name + "/chord:0") {
                first = chord;
            }
        }
        const std::string given = mastersName(*first);
        if (given.empty()) {
            for (topo::Edge* chord : chords) taken.insert(chord->topoId.tag());
            continue;
        }
        exactEdge.topoId = topo::TopologyID::fromTag(given);
        for (topo::Edge* chord : chords) {
            const std::string& tag = chord->topoId.tag();
            rename(*chord, tag.size() > name.size() ? given + tag.substr(name.size()) : given);
        }
    }
    std::set<std::string> exactNames;  // as they are now, a seam renamed
    for (const topo::Edge* e : exactEdges) exactNames.insert(e->topoId.tag());
    for (topo::Edge* line : lines) {
        const std::string given = mastersName(*line);
        if (given.empty() || exactNames.count(given) != 0) {
            unmatchedEdges.push_back(line);
        } else {
            rename(*line, given);
        }
    }

    // What has no place in master's construction keeps its name, renumbered past those taken.
    taken.insert(exactNames.begin(), exactNames.end());
    const auto renumber = [&taken](topo::TopologyID& id) {
        const std::string tag = id.tag();
        if (taken.count(tag) == 0) {
            taken.insert(tag);
            return;
        }
        const auto prefix = numberedPrefix(tag);
        if (!prefix) return;
        int n = 0;
        std::string name = *prefix + std::to_string(n);
        while (taken.count(name) != 0) name = *prefix + std::to_string(++n);
        id = topo::TopologyID::fromTag(name);
        taken.insert(name);
    };
    for (topo::Face* face : unmatchedFaces) renumber(face->topoId);
    for (topo::Edge* edge : unmatchedEdges) renumber(edge->topoId);
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
