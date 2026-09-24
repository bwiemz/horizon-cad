#include "horizon/document/AssemblyDocument.h"

#include <algorithm>
#include <utility>

#include "horizon/document/Document.h"
#include "horizon/modeling/InterferenceChecker.h"
#include "horizon/modeling/Pattern.h"

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

std::size_t AssemblyDocument::InterferenceInput::faceCount() const {
    std::size_t faces = 0;
    for (const auto& solid : placed) faces += solid ? solid->faceCount() : 0;
    return faces;
}

InterferenceReport AssemblyDocument::measureInterference(const InterferenceInput& input) {
    InterferenceReport report;
    report.unchecked = input.unchecked;
    std::vector<const topo::Solid*> solids;
    solids.reserve(input.placed.size());
    for (const auto& s : input.placed) solids.push_back(s.get());
    for (const auto& pair : model::InterferenceChecker::check(solids)) {
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
