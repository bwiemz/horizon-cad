#pragma once

#include "horizon/drafting/DraftEntity.h"
#include "horizon/math/Vec2.h"
#include <memory>
#include <vector>

namespace hz::ui {

/// Internal clipboard for Copy/Cut/Paste of draft entities.
class Clipboard {
public:
    void copy(const std::vector<std::shared_ptr<draft::DraftEntity>>& entities);

    bool hasContent() const { return !m_entities.empty(); }
    const std::vector<std::shared_ptr<draft::DraftEntity>>& entities() const { return m_entities; }
    const math::Vec2& centroid() const { return m_centroid; }
    void clear();

private:
    std::vector<std::shared_ptr<draft::DraftEntity>> m_entities;
    math::Vec2 m_centroid;
};

}  // namespace hz::ui
