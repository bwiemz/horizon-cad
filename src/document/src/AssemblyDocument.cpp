#include "horizon/document/AssemblyDocument.h"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <map>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "horizon/document/Document.h"
#include "horizon/math/Quaternion.h"
#include "horizon/modeling/InterferenceChecker.h"
#include "horizon/modeling/Pattern.h"
#include "horizon/topology/Solid.h"

namespace hz::doc {

InterferenceReport AssemblyDocument::findInterference() const {
    return measureInterference(interferenceInput());
}

AssemblyDocument::InterferenceInput AssemblyDocument::interferenceInput() const {
    InterferenceInput input;
    for (const auto& comp : m_components) {
        if (comp.suppressed) continue;
        const topo::Solid* solid = comp.solid();  // a part's, or a subassembly's
        if (solid == nullptr) {
            input.unchecked.push_back(comp.id);
            continue;
        }
        input.placed.push_back(model::Pattern::transformed(*solid, comp.transform));
        input.ids.push_back(comp.id);
    }
    return input;
}

std::string AssemblyDocument::namePrefix(uint64_t id) {
    return "c" + std::to_string(id) + "/";
}

bool ComponentInstance::isAssembly() const {
    std::string extension = std::filesystem::path(partPath).extension().string();
    std::transform(extension.begin(), extension.end(), extension.begin(),
                   [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
    return extension == ".hzasm";
}

const topo::Solid* ComponentInstance::solid() const {
    if (resolvedPart) return resolvedPart->solid();
    return assemblySolid.get();
}

std::shared_ptr<geo::MeshData> AssemblyDocument::drawingMesh() const {
    auto merged = std::make_shared<geo::MeshData>();
    bool any = false;
    for (const auto& comp : m_components) {
        if (comp.suppressed || !comp.cachedMesh) continue;
        const geo::MeshData& mesh = *comp.cachedMesh;
        const math::Mat4& placed = comp.transform;
        const std::string prefix = namePrefix(comp.id);
        const auto firstVertex = static_cast<uint32_t>(merged->positions.size() / 3);
        const auto firstFace = static_cast<uint32_t>(merged->faceTags.size());
        for (size_t i = 0; i + 2 < mesh.positions.size(); i += 3) {
            const math::Vec3 p = placed.transformPoint(
                math::Vec3(mesh.positions[i], mesh.positions[i + 1], mesh.positions[i + 2]));
            merged->positions.insert(
                merged->positions.end(),
                {static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z)});
        }
        // Its normals, one to each vertex: its own, or, for a mesh without
        // them (a part's cached tessellation may have none), made from its
        // triangles, so each vertex keeps its own and not the next part's.
        std::vector<math::Vec3> normals(mesh.positions.size() / 3);
        if (mesh.normals.size() == mesh.positions.size()) {
            for (size_t v = 0; v < normals.size(); ++v) {
                normals[v] = math::Vec3(mesh.normals[3 * v], mesh.normals[3 * v + 1],
                                        mesh.normals[3 * v + 2]);
            }
        } else {
            const auto at = [&mesh](uint32_t v) {
                return math::Vec3(mesh.positions[3 * v], mesh.positions[3 * v + 1],
                                  mesh.positions[3 * v + 2]);
            };
            for (size_t t = 0; t + 2 < mesh.indices.size(); t += 3) {
                const uint32_t a = mesh.indices[t];
                const uint32_t b = mesh.indices[t + 1];
                const uint32_t c = mesh.indices[t + 2];
                if (a >= normals.size() || b >= normals.size() || c >= normals.size()) continue;
                const math::Vec3 area = (at(b) - at(a)).cross(at(c) - at(a));  // weighted
                normals[a] = normals[a] + area;
                normals[b] = normals[b] + area;
                normals[c] = normals[c] + area;
            }
        }
        for (const math::Vec3& own : normals) {
            const math::Vec3 turned = placed.transformDirection(own);
            const math::Vec3 n = turned.length() > 0.0 ? turned.normalized() : turned;
            merged->normals.insert(
                merged->normals.end(),
                {static_cast<float>(n.x), static_cast<float>(n.y), static_cast<float>(n.z)});
        }
        for (const uint32_t index : mesh.indices) merged->indices.push_back(firstVertex + index);
        // Its faces' names, for a pick. A mesh without them leaves some
        // triangles unnamed, and the whole unnamed at the end.
        if (mesh.hasFaces()) {
            for (const uint32_t face : mesh.triangleFaces) {
                merged->triangleFaces.push_back(firstFace + face);
            }
            for (const auto& tag : mesh.faceTags) merged->faceTags.push_back(prefix + tag);
        }
        for (const auto& edge : mesh.edges) {
            geo::MeshData::Edge placedEdge;
            placedEdge.tag = prefix + edge.tag;
            for (size_t i = 0; i + 2 < edge.points.size(); i += 3) {
                const math::Vec3 q = placed.transformPoint(
                    math::Vec3(edge.points[i], edge.points[i + 1], edge.points[i + 2]));
                placedEdge.points.insert(
                    placedEdge.points.end(),
                    {static_cast<float>(q.x), static_cast<float>(q.y), static_cast<float>(q.z)});
            }
            merged->edges.push_back(std::move(placedEdge));
        }
        any = true;
    }
    // Names for some triangles and not others would name the wrong faces.
    if (!merged->hasFaces()) {
        merged->triangleFaces.clear();
        merged->faceTags.clear();
    }
    return any ? merged : nullptr;
}

std::unique_ptr<topo::Solid> AssemblyDocument::drawingSolid(
    const std::function<const topo::Solid*(const ComponentInstance&)>& partOf,
    std::vector<uint64_t>* missing) const {
    auto gathered = std::make_unique<topo::Solid>();
    bool any = false;
    for (const auto& comp : m_components) {
        if (comp.suppressed) continue;
        const topo::Solid* part = partOf(comp);
        if (part == nullptr) {
            if (missing != nullptr) missing->push_back(comp.id);
            continue;
        }
        // Placed straight into the one solid: each part copied once.
        const std::size_t facesBefore = gathered->faces().size();
        const std::size_t edgesBefore = gathered->edges().size();
        model::Pattern::append(*gathered, *part, comp.transform);
        // Its names its own: a balloon or dimension on one instance of a
        // part names that instance, not every instance of the part.
        const std::string prefix = namePrefix(comp.id);
        auto& faces = gathered->faces();
        for (std::size_t i = facesBefore; i < faces.size(); ++i) {
            faces[i].topoId = topo::TopologyID::fromTag(prefix + faces[i].topoId.tag());
        }
        auto& edges = gathered->edges();
        for (std::size_t i = edgesBefore; i < edges.size(); ++i) {
            edges[i].topoId = topo::TopologyID::fromTag(prefix + edges[i].topoId.tag());
        }
        any = true;
    }
    return any ? std::move(gathered) : nullptr;
}

std::size_t AssemblyDocument::InterferenceInput::faceCount() const {
    std::size_t faces = 0;
    for (const auto& solid : placed) faces += solid ? solid->faceCount() : 0;
    return faces;
}

InterferenceReport AssemblyDocument::measureInterference(const InterferenceInput& input,
                                                         const std::atomic<bool>* cancelled) {
    InterferenceReport report;
    report.unchecked = input.unchecked;
    std::vector<const topo::Solid*> solids;
    solids.reserve(input.placed.size());
    for (const auto& s : input.placed) solids.push_back(s.get());
    for (const auto& pair : model::InterferenceChecker::check(solids, cancelled)) {
        ComponentInterference ci;
        ci.componentA = input.ids[pair.indexA];
        ci.componentB = input.ids[pair.indexB];
        ci.volume = pair.volume;
        ci.volumeResolved = pair.volumeResolved;
        report.pairs.push_back(ci);
    }
    return report;
}

void AssemblyDocument::restore(AssemblyState state) {
    // A snapshot records where the components are and what they are, not
    // the geometry loaded for them: a component still here keeps what it has
    // now. (An undo after its part changed brought back the old mesh.)
    for (auto& comp : state.components) {
        const auto* now = component(comp.id);
        if (now == nullptr || now->partPath != comp.partPath) continue;
        comp.cachedMesh = now->cachedMesh;
        comp.resolvedPart = now->resolvedPart;
        comp.resolvedAssembly = now->resolvedAssembly;  // a subassembly's (Phase 159)
        comp.assemblySolid = now->assemblySolid;
        comp.state = now->state;
    }
    m_components = std::move(state.components);
    m_mates = std::move(state.mates);
    m_views = std::move(state.views);
    m_patterns = std::move(state.patterns);
    // Ids handed out since the snapshot are not reused.
    for (const auto& comp : m_components) {
        m_nextComponentId = std::max(m_nextComponentId, comp.id + 1);
    }
    for (const auto& mate : m_mates) m_nextMateId = std::max(m_nextMateId, mate.id + 1);
    for (const auto& view : m_views) m_nextViewId = std::max(m_nextViewId, view.id + 1);
    for (const auto& owner : m_patterns) {
        m_nextPatternId = std::max(m_nextPatternId, owner.id + 1);
    }
    if (explodedView(m_shownView) == nullptr) m_shownView = 0;  // it is gone
}

math::Mat4 ComponentPattern::instanceTransform(int k) const {
    const double at = static_cast<double>(k);
    if (kind == Kind::Linear) return math::Mat4::translation(direction * (spacing * at));
    return math::Mat4::translation(axisPoint) *
           math::Mat4::rotation(math::Quaternion::fromAxisAngle(direction, spacing * at)) *
           math::Mat4::translation(axisPoint * -1.0);
}

bool ComponentPattern::skips(int k) const {
    return std::find(skipped.begin(), skipped.end(), k) != skipped.end();
}

int ComponentPattern::kept() const {
    int n = 1;
    for (int k = 1; k < count; ++k) n += skips(k) ? 0 : 1;
    return n;
}

uint64_t AssemblyDocument::addPattern(ComponentPattern added) {
    if (added.id == 0 || pattern(added.id) != nullptr) added.id = m_nextPatternId;
    m_nextPatternId = std::max(m_nextPatternId, added.id + 1);
    const uint64_t id = added.id;
    m_patterns.push_back(std::move(added));
    m_dirty = true;
    return id;
}

bool AssemblyDocument::removePattern(uint64_t id) {
    const auto it = std::find_if(m_patterns.begin(), m_patterns.end(),
                                 [id](const ComponentPattern& p) { return p.id == id; });
    if (it == m_patterns.end()) return false;
    m_patterns.erase(it);
    m_dirty = true;
    updatePatterns();
    return true;
}

ComponentPattern* AssemblyDocument::pattern(uint64_t id) {
    const auto it = std::find_if(m_patterns.begin(), m_patterns.end(),
                                 [id](const ComponentPattern& p) { return p.id == id; });
    return it == m_patterns.end() ? nullptr : &*it;
}

const ComponentPattern* AssemblyDocument::pattern(uint64_t id) const {
    const auto it = std::find_if(m_patterns.begin(), m_patterns.end(),
                                 [id](const ComponentPattern& p) { return p.id == id; });
    return it == m_patterns.end() ? nullptr : &*it;
}

bool AssemblyDocument::updatePatterns() {
    bool changed = false;
    // Seeds that are components, and not instances: a pattern of an
    // instance would be placed from what it places.
    for (auto& owner : m_patterns) {
        const size_t had = owner.seeds.size();
        std::erase_if(owner.seeds, [this](uint64_t seed) {
            const ComponentInstance* comp = component(seed);
            return comp == nullptr || comp->isPatternInstance();
        });
        // Each once.
        std::vector<uint64_t> once;
        for (const uint64_t seed : owner.seeds) {
            if (std::find(once.begin(), once.end(), seed) == once.end()) once.push_back(seed);
        }
        owner.seeds = std::move(once);
        changed = changed || owner.seeds.size() != had;
    }
    changed =
        std::erase_if(m_patterns, [](const ComponentPattern& p) { return p.seeds.empty(); }) > 0 ||
        changed;

    // Each instance wanted: its pattern, seed and index, in order.
    using Key = std::tuple<uint64_t, uint64_t, int>;
    std::vector<Key> wanted;
    for (const auto& owner : m_patterns) {
        for (const uint64_t seed : owner.seeds) {
            for (int k = 1; k < owner.count; ++k) {
                if (!owner.skips(k)) wanted.emplace_back(owner.id, seed, k);
            }
        }
    }
    const std::set<Key> wantedSet(wanted.begin(), wanted.end());
    // Instances not wanted go; so does a second of one.
    std::set<Key> seen;
    std::vector<uint64_t> gone;
    std::erase_if(m_components, [&](const ComponentInstance& comp) {
        if (!comp.isPatternInstance()) return false;
        const Key key{comp.patternId, comp.seedId, comp.patternIndex};
        if (wantedSet.count(key) != 0 && seen.insert(key).second) return false;
        gone.push_back(comp.id);
        return true;
    });
    if (!gone.empty()) {
        changed = true;
        const auto isGone = [&gone](const Mate& m) {
            return std::find(gone.begin(), gone.end(), m.a.componentId) != gone.end() ||
                   std::find(gone.begin(), gone.end(), m.b.componentId) != gone.end();
        };
        std::erase_if(m_mates, isGone);
        for (auto& view : m_views) {
            for (auto& step : view.steps) {
                std::erase_if(step.components, [&gone](uint64_t id) {
                    return std::find(gone.begin(), gone.end(), id) != gone.end();
                });
            }
        }
    }

    // The instances made, then placed from their seeds.
    for (const Key& key : wanted) {
        // The key, not its parts: clang before 16 cannot capture a structured
        // binding of an aggregate, and this keeps the rule simple.
        const auto found = std::find_if(
            m_components.begin(), m_components.end(), [&key](const ComponentInstance& comp) {
                return Key{comp.patternId, comp.seedId, comp.patternIndex} == key;
            });
        if (found != m_components.end()) continue;
        const auto& [patternId, seedId, k] = key;
        const ComponentInstance* seed = component(seedId);
        ComponentInstance made;
        made.id = m_nextComponentId++;
        made.name = seed->name + " (" + std::to_string(k + 1) + ")";
        made.patternId = patternId;
        made.seedId = seedId;
        made.patternIndex = k;
        m_components.push_back(std::move(made));  // seed is not used after this
        changed = true;
    }
    std::map<uint64_t, const ComponentInstance*> seeds;
    for (const auto& comp : m_components) {
        if (!comp.isPatternInstance()) seeds[comp.id] = &comp;
    }
    for (auto& comp : m_components) {
        if (!comp.isPatternInstance()) continue;
        const ComponentInstance& seed = *seeds.at(comp.seedId);
        const math::Mat4 at =
            pattern(comp.patternId)->instanceTransform(comp.patternIndex) * seed.transform;
        bool differs = comp.suppressed != seed.suppressed || comp.partPath != seed.partPath;
        for (int r = 0; r < 4 && !differs; ++r) {
            for (int c = 0; c < 4 && !differs; ++c)
                differs = comp.transform.at(r, c) != at.at(r, c);
        }
        // The seed's geometry, when it has some: one part, one mesh.
        const bool share = seed.cachedMesh && comp.cachedMesh != seed.cachedMesh;
        if (!differs && !share) continue;
        if (differs) changed = true;
        if (comp.partPath != seed.partPath) {
            // Another part: what it had resolved is not it.
            comp.state = ComponentState::Lightweight;
            comp.cachedMesh.reset();
            comp.resolvedPart.reset();
            comp.resolvedAssembly.reset();
            comp.assemblySolid.reset();
        }
        comp.transform = at;
        comp.suppressed = seed.suppressed;
        comp.partPath = seed.partPath;
        if (seed.cachedMesh) {
            comp.state = seed.state;
            comp.cachedMesh = seed.cachedMesh;
            comp.resolvedPart = seed.resolvedPart;
            comp.resolvedAssembly = seed.resolvedAssembly;
            comp.assemblySolid = seed.assemblySolid;
        }
    }
    if (changed) m_dirty = true;
    return changed;
}

uint64_t AssemblyDocument::addExplodedView(ExplodedView view) {
    // None, or one taken (a file's two alike): the next.
    if (view.id == 0 || explodedView(view.id) != nullptr) view.id = m_nextViewId;
    m_nextViewId = std::max(m_nextViewId, view.id + 1);
    const uint64_t id = view.id;
    m_views.push_back(std::move(view));
    m_dirty = true;
    return id;
}

bool AssemblyDocument::removeExplodedView(uint64_t id) {
    const auto it = std::find_if(m_views.begin(), m_views.end(),
                                 [id](const ExplodedView& v) { return v.id == id; });
    if (it == m_views.end()) return false;
    m_views.erase(it);
    if (m_shownView == id) m_shownView = 0;
    m_dirty = true;
    return true;
}

ExplodedView* AssemblyDocument::explodedView(uint64_t id) {
    const auto it = std::find_if(m_views.begin(), m_views.end(),
                                 [id](const ExplodedView& v) { return v.id == id; });
    return it == m_views.end() ? nullptr : &*it;
}

const ExplodedView* AssemblyDocument::explodedView(uint64_t id) const {
    const auto it = std::find_if(m_views.begin(), m_views.end(),
                                 [id](const ExplodedView& v) { return v.id == id; });
    return it == m_views.end() ? nullptr : &*it;
}

bool AssemblyDocument::setShownView(uint64_t id) {
    if (id != 0 && explodedView(id) == nullptr) return false;
    if (m_shownView != id) m_dirty = true;
    m_shownView = id;
    return true;
}

math::Mat4 AssemblyDocument::displayTransform(const ComponentInstance& comp) const {
    const ExplodedView* view = explodedView(m_shownView);
    if (view == nullptr) return comp.transform;
    math::Vec3 moved;
    for (const auto& step : view->steps) {
        if (std::find(step.components.begin(), step.components.end(), comp.id) !=
            step.components.end()) {
            moved = moved + step.direction * step.distance;
        }
    }
    return math::Mat4::translation(moved) * comp.transform;
}

uint64_t AssemblyDocument::addComponent(ComponentInstance instance) {
    if (instance.id == 0) {
        instance.id = m_nextComponentId;
    }
    m_nextComponentId = std::max(m_nextComponentId, instance.id + 1);
    uint64_t id = instance.id;
    m_components.push_back(std::move(instance));
    m_dirty = true;
    return id;
}

bool AssemblyDocument::removeComponent(uint64_t id) {
    auto it = std::find_if(m_components.begin(), m_components.end(),
                           [id](const ComponentInstance& c) { return c.id == id; });
    if (it == m_components.end()) return false;
    // A pattern's instance: its index is left out, or it would be made
    // again (Phase 161).
    if (it->isPatternInstance()) {
        if (ComponentPattern* owner = pattern(it->patternId)) {
            if (!owner->skips(it->patternIndex)) owner->skipped.push_back(it->patternIndex);
        }
    }
    m_components.erase(it);
    // Its mates go with it: one left referring to it made every later solve
    // fail (InvalidReference).
    std::erase_if(m_mates,
                  [id](const Mate& m) { return m.a.componentId == id || m.b.componentId == id; });
    // And its place in any exploded view's steps.
    for (auto& view : m_views) {
        for (auto& step : view.steps) std::erase(step.components, id);
    }
    // A seed leaves its patterns, and its instances go (Phase 161).
    for (auto& owner : m_patterns) std::erase(owner.seeds, id);
    updatePatterns();
    m_dirty = true;
    return true;
}

ComponentInstance* AssemblyDocument::component(uint64_t id) {
    auto it = std::find_if(m_components.begin(), m_components.end(),
                           [id](const ComponentInstance& c) { return c.id == id; });
    return it == m_components.end() ? nullptr : &*it;
}

const ComponentInstance* AssemblyDocument::component(uint64_t id) const {
    auto it = std::find_if(m_components.begin(), m_components.end(),
                           [id](const ComponentInstance& c) { return c.id == id; });
    return it == m_components.end() ? nullptr : &*it;
}

uint64_t AssemblyDocument::addMate(Mate mate) {
    if (mate.id == 0) {
        mate.id = m_nextMateId;
    }
    m_nextMateId = std::max(m_nextMateId, mate.id + 1);
    uint64_t id = mate.id;
    m_mates.push_back(std::move(mate));
    m_dirty = true;
    return id;
}

bool AssemblyDocument::removeMate(uint64_t id) {
    auto it =
        std::find_if(m_mates.begin(), m_mates.end(), [id](const Mate& m) { return m.id == id; });
    if (it == m_mates.end()) return false;
    m_mates.erase(it);
    m_dirty = true;
    return true;
}

Mate* AssemblyDocument::mate(uint64_t id) {
    auto it =
        std::find_if(m_mates.begin(), m_mates.end(), [id](const Mate& m) { return m.id == id; });
    return it == m_mates.end() ? nullptr : &*it;
}

const Mate* AssemblyDocument::mate(uint64_t id) const {
    auto it =
        std::find_if(m_mates.begin(), m_mates.end(), [id](const Mate& m) { return m.id == id; });
    return it == m_mates.end() ? nullptr : &*it;
}

void AssemblyDocument::clear() {
    m_components.clear();
    m_mates.clear();
    m_nextComponentId = 1;
    m_nextMateId = 1;
    m_views.clear();
    m_nextViewId = 1;
    m_patterns.clear();
    m_nextPatternId = 1;
    m_shownView = 0;
    m_dirty = false;
    m_filePath.clear();
    m_lengthUnit = math::LengthUnit::Millimetre;
}

}  // namespace hz::doc
