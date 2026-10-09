#pragma once

#include <memory>
#include <vector>

#include "horizon/drafting/DraftEntity.h"
#include "horizon/drafting/Layer.h"
#include "horizon/math/Vec2.h"

namespace hz::ui {

/// Internal clipboard for Copy/Cut/Paste of draft entities.
class Clipboard {
public:
    /// Copy @p entities, and from @p layers the layers they are on (and what
    /// is in their blocks is on): what a paste into a drawing that lacks
    /// them adds.
    void copy(const std::vector<std::shared_ptr<draft::DraftEntity>>& entities,
              const draft::LayerManager* layers = nullptr);

    bool hasContent() const { return !m_entities.empty(); }
    const std::vector<std::shared_ptr<draft::DraftEntity>>& entities() const { return m_entities; }
    const math::Vec2& centroid() const { return m_centroid; }
    /// The layers of what was copied, as they were then.
    const std::vector<draft::LayerProperties>& layers() const { return m_layers; }
    void clear();

private:
    std::vector<std::shared_ptr<draft::DraftEntity>> m_entities;
    std::vector<draft::LayerProperties> m_layers;
    math::Vec2 m_centroid;
};

}  // namespace hz::ui
