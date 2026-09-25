#include "horizon/document/AssemblyDocument.h"

#include <algorithm>
#include <string>
#include <utility>

#include "horizon/document/Document.h"
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
        const topo::Solid* solid = comp.resolvedPart ? comp.resolvedPart->solid() : nullptr;
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

std::unique_ptr<topo::Solid> AssemblyDocument::drawingSolid(
    const std::function<const topo::Solid*(const ComponentInstance&)>& partOf,
    std::vector<uint64_t>* missing) const {
    std::unique_ptr<topo::Solid> gathered;
    for (const auto& comp : m_components) {
        if (comp.suppressed) continue;
        const topo::Solid* part = partOf(comp);
        if (part == nullptr) {
            if (missing != nullptr) missing->push_back(comp.id);
            continue;
        }
        auto placed = model::Pattern::transformed(*part, comp.transform);
        if (!placed) {
            if (missing != nullptr) missing->push_back(comp.id);
            continue;
        }
        // Its names its own: a balloon or dimension on one instance of a
        // part names that instance, not every instance of the part.
        const std::string prefix = namePrefix(comp.id);
        for (topo::Face& f : placed->faces()) {
            f.topoId = topo::TopologyID::fromTag(prefix + f.topoId.tag());
        }
        for (topo::Edge& e : placed->edges()) {
            e.topoId = topo::TopologyID::fromTag(prefix + e.topoId.tag());
        }
        gathered = gathered ? model::Pattern::collect(*gathered, *placed) : std::move(placed);
    }
    return gathered;
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
        comp.state = now->state;
    }
    m_components = std::move(state.components);
    m_mates = std::move(state.mates);
    // Ids handed out since the snapshot are not reused.
    for (const auto& comp : m_components) {
        m_nextComponentId = std::max(m_nextComponentId, comp.id + 1);
    }
    for (const auto& mate : m_mates) m_nextMateId = std::max(m_nextMateId, mate.id + 1);
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
    m_components.erase(it);
    // Its mates go with it: one left referring to it made every later solve
    // fail (InvalidReference).
    std::erase_if(m_mates,
                  [id](const Mate& m) { return m.a.componentId == id || m.b.componentId == id; });
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
    m_dirty = false;
    m_filePath.clear();
}

}  // namespace hz::doc
