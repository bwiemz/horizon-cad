#include "horizon/ui/Clipboard.h"

#include <set>
#include <string>

#include "horizon/drafting/BlockDefinition.h"
#include "horizon/drafting/DraftBlockRef.h"

namespace hz::ui {

namespace {

/// The layers @p entity is on: its own, and those of what is in its block.
void layersOf(const draft::DraftEntity& entity, std::set<std::string>& names,
              std::set<const draft::BlockDefinition*>& seen) {
    names.insert(entity.layer());
    const auto* ref = dynamic_cast<const draft::DraftBlockRef*>(&entity);
    if (ref == nullptr || !ref->definition() || !seen.insert(ref->definition().get()).second) {
        return;
    }
    for (const auto& inner : ref->definition()->entities) {
        if (inner) layersOf(*inner, names, seen);
    }
}

}  // namespace

void Clipboard::copy(const std::vector<std::shared_ptr<draft::DraftEntity>>& entities,
                     const draft::LayerManager* layers) {
    m_entities.clear();
    m_layers.clear();
    if (entities.empty()) return;

    // Compute centroid from bounding box centers.
    math::Vec2 sum{0.0, 0.0};
    int count = 0;
    for (const auto& e : entities) {
        auto bbox = e->boundingBox();
        if (bbox.isValid()) {
            auto lo = bbox.min();
            auto hi = bbox.max();
            sum.x += (lo.x + hi.x) * 0.5;
            sum.y += (lo.y + hi.y) * 0.5;
            ++count;
        }
    }
    m_centroid = (count > 0) ? math::Vec2{sum.x / count, sum.y / count} : math::Vec2{0.0, 0.0};

    // Clone all entities.
    for (const auto& e : entities) {
        m_entities.push_back(e->clone());
    }

    if (layers == nullptr) return;
    std::set<std::string> names;
    std::set<const draft::BlockDefinition*> seen;
    for (const auto& e : entities) layersOf(*e, names, seen);
    for (const std::string& name : names) {
        if (const auto* props = layers->getLayer(name)) m_layers.push_back(*props);
    }
}

void Clipboard::clear() {
    m_entities.clear();
    m_layers.clear();
    m_centroid = {0.0, 0.0};
}

}  // namespace hz::ui
