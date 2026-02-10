#include "horizon/ui/Clipboard.h"

namespace hz::ui {

void Clipboard::copy(const std::vector<std::shared_ptr<draft::DraftEntity>>& entities) {
    m_entities.clear();
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
}

void Clipboard::clear() {
    m_entities.clear();
    m_centroid = {0.0, 0.0};
}

}  // namespace hz::ui
