#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "horizon/math/Vec2.h"

namespace hz::draft {
class DraftEntity;
}

namespace hz::cstr {

/// Type of geometric feature on an entity.
enum class FeatureType {
    Point,   // A specific point (endpoint, center, vertex)
    Line,    // A linear edge or full line entity
    Circle,  // A circle or the underlying circle of an arc
};

/// References a specific geometric feature on a DraftEntity.
struct GeometryRef {
    uint64_t entityId = 0;
    FeatureType featureType = FeatureType::Point;
    int featureIndex = 0;

    bool operator==(const GeometryRef& o) const {
        return entityId == o.entityId && featureType == o.featureType &&
               featureIndex == o.featureIndex;
    }
    bool operator!=(const GeometryRef& o) const { return !(*this == o); }
    bool isValid() const { return entityId != 0; }
};

/// The world-space position of a Point feature on an entity, or nothing when
/// the ref does not fit it (another kind of entity, an index out of range).
std::optional<math::Vec2> pointOf(const GeometryRef& ref, const draft::DraftEntity& entity);

/// Line endpoints (start, end) of a Line feature, or nothing, as pointOf().
std::optional<std::pair<math::Vec2, math::Vec2>> lineOf(const GeometryRef& ref,
                                                        const draft::DraftEntity& entity);

/// Extract the world-space position of a Point feature from an entity.
/// Throws std::runtime_error when the ref does not fit it.
math::Vec2 extractPoint(const GeometryRef& ref, const draft::DraftEntity& entity);

/// Extract line endpoints (start, end) for a Line feature. Throws
/// std::runtime_error when the ref does not fit the entity.
std::pair<math::Vec2, math::Vec2> extractLine(const GeometryRef& ref,
                                              const draft::DraftEntity& entity);

/// Extract circle center and radius for a Circle feature.
std::pair<math::Vec2, double> extractCircle(const GeometryRef& ref,
                                            const draft::DraftEntity& entity);

/// Find the entity matching a GeometryRef's entityId.
const draft::DraftEntity* findEntity(
    uint64_t entityId, const std::vector<std::shared_ptr<draft::DraftEntity>>& entities);

}  // namespace hz::cstr
