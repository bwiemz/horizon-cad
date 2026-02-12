#pragma once

#include "horizon/math/Vec2.h"
#include <cstdint>
#include <vector>

namespace hz::draft {
class DraftEntity;
}

namespace hz::ui {

/// Extracts editable grip points from entities and applies grip moves.
/// Centralizes grip-point logic without adding virtuals to DraftEntity.
class GripManager {
public:
    /// Get the editable grip points for an entity.
    static std::vector<math::Vec2> gripPoints(const draft::DraftEntity& entity);

    /// Move grip at the given index to a new position.
    /// Returns true if the move was applied.
    static bool moveGrip(draft::DraftEntity& entity, int gripIndex,
                         const math::Vec2& newPos);
};

}  // namespace hz::ui
