#include "horizon/constraint/GeometryRef.h"

#include <stdexcept>

#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftEntity.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftPolyline.h"
#include "horizon/drafting/DraftRectangle.h"

namespace hz::cstr {

std::optional<math::Vec2> pointOf(const GeometryRef& ref, const draft::DraftEntity& entity) {
    if (auto* line = dynamic_cast<const draft::DraftLine*>(&entity)) {
        if (ref.featureIndex == 0) return line->start();
        if (ref.featureIndex == 1) return line->end();
    } else if (auto* circle = dynamic_cast<const draft::DraftCircle*>(&entity)) {
        if (ref.featureIndex == 0) return circle->center();
    } else if (auto* arc = dynamic_cast<const draft::DraftArc*>(&entity)) {
        if (ref.featureIndex == 0) return arc->center();
        if (ref.featureIndex == 1) return arc->startPoint();
        if (ref.featureIndex == 2) return arc->endPoint();
    } else if (auto* rect = dynamic_cast<const draft::DraftRectangle*>(&entity)) {
        auto c = rect->corners();
        if (ref.featureIndex >= 0 && ref.featureIndex < 4) return c[ref.featureIndex];
    } else if (auto* poly = dynamic_cast<const draft::DraftPolyline*>(&entity)) {
        auto& pts = poly->points();
        if (ref.featureIndex >= 0 && ref.featureIndex < static_cast<int>(pts.size()))
            return pts[ref.featureIndex];
    }
    return std::nullopt;
}

std::optional<std::pair<math::Vec2, math::Vec2>> lineOf(const GeometryRef& ref,
                                                        const draft::DraftEntity& entity) {
    if (auto* line = dynamic_cast<const draft::DraftLine*>(&entity)) {
        if (ref.featureIndex == 0) return std::pair{line->start(), line->end()};
    } else if (auto* rect = dynamic_cast<const draft::DraftRectangle*>(&entity)) {
        auto c = rect->corners();
        int i = ref.featureIndex;
        if (i >= 0 && i < 4) return std::pair{c[i], c[(i + 1) % 4]};
    } else if (auto* poly = dynamic_cast<const draft::DraftPolyline*>(&entity)) {
        auto& pts = poly->points();
        int i = ref.featureIndex;
        int n = static_cast<int>(pts.size());
        if (i >= 0 && i < n - 1) return std::pair{pts[i], pts[i + 1]};
        if (poly->closed() && i == n - 1) return std::pair{pts[n - 1], pts[0]};
    }
    return std::nullopt;
}

math::Vec2 extractPoint(const GeometryRef& ref, const draft::DraftEntity& entity) {
    if (auto point = pointOf(ref, entity)) return *point;
    throw std::runtime_error("extractPoint: invalid GeometryRef for entity " +
                             std::to_string(ref.entityId));
}

std::pair<math::Vec2, math::Vec2> extractLine(const GeometryRef& ref,
                                              const draft::DraftEntity& entity) {
    if (auto line = lineOf(ref, entity)) return *line;
    throw std::runtime_error("extractLine: invalid GeometryRef for entity " +
                             std::to_string(ref.entityId));
}

std::pair<math::Vec2, double> extractCircle(const GeometryRef& ref,
                                            const draft::DraftEntity& entity) {
    if (auto* circle = dynamic_cast<const draft::DraftCircle*>(&entity)) {
        if (ref.featureIndex == 0) return {circle->center(), circle->radius()};
    } else if (auto* arc = dynamic_cast<const draft::DraftArc*>(&entity)) {
        if (ref.featureIndex == 0) return {arc->center(), arc->radius()};
    }
    throw std::runtime_error("extractCircle: invalid GeometryRef for entity " +
                             std::to_string(ref.entityId));
}

const draft::DraftEntity* findEntity(
    uint64_t entityId, const std::vector<std::shared_ptr<draft::DraftEntity>>& entities) {
    for (const auto& e : entities) {
        if (e->id() == entityId) return e.get();
    }
    return nullptr;
}

}  // namespace hz::cstr
