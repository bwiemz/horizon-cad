#include "horizon/constraint/ParameterTable.h"
#include "horizon/constraint/ConstraintSystem.h"
#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftEntity.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftPolyline.h"
#include "horizon/drafting/DraftRectangle.h"
#include <set>
#include <stdexcept>

namespace hz::cstr {

int ParameterTable::registerEntity(const draft::DraftEntity& entity) {
    int startIdx = static_cast<int>(m_values.size());
    EntityParams ep;
    ep.entityId = entity.id();
    ep.startIndex = startIdx;

    if (auto* line = dynamic_cast<const draft::DraftLine*>(&entity)) {
        ep.paramCount = 4;
        ep.entityType = "line";
        m_values.conservativeResize(startIdx + 4);
        m_values(startIdx + 0) = line->start().x;
        m_values(startIdx + 1) = line->start().y;
        m_values(startIdx + 2) = line->end().x;
        m_values(startIdx + 3) = line->end().y;
    } else if (auto* circle = dynamic_cast<const draft::DraftCircle*>(&entity)) {
        ep.paramCount = 3;
        ep.entityType = "circle";
        m_values.conservativeResize(startIdx + 3);
        m_values(startIdx + 0) = circle->center().x;
        m_values(startIdx + 1) = circle->center().y;
        m_values(startIdx + 2) = circle->radius();
    } else if (auto* arc = dynamic_cast<const draft::DraftArc*>(&entity)) {
        ep.paramCount = 5;
        ep.entityType = "arc";
        m_values.conservativeResize(startIdx + 5);
        m_values(startIdx + 0) = arc->center().x;
        m_values(startIdx + 1) = arc->center().y;
        m_values(startIdx + 2) = arc->radius();
        m_values(startIdx + 3) = arc->startAngle();
        m_values(startIdx + 4) = arc->endAngle();
    } else if (auto* rect = dynamic_cast<const draft::DraftRectangle*>(&entity)) {
        ep.paramCount = 4;
        ep.entityType = "rectangle";
        m_values.conservativeResize(startIdx + 4);
        m_values(startIdx + 0) = rect->corner1().x;
        m_values(startIdx + 1) = rect->corner1().y;
        m_values(startIdx + 2) = rect->corner2().x;
        m_values(startIdx + 3) = rect->corner2().y;
    } else if (auto* poly = dynamic_cast<const draft::DraftPolyline*>(&entity)) {
        int n = static_cast<int>(poly->points().size());
        ep.paramCount = 2 * n;
        ep.entityType = "polyline";
        m_values.conservativeResize(startIdx + 2 * n);
        for (int i = 0; i < n; ++i) {
            m_values(startIdx + 2 * i + 0) = poly->points()[i].x;
            m_values(startIdx + 2 * i + 1) = poly->points()[i].y;
        }
    } else {
        // Unsupported entity type for constraints (dimensions, leaders, etc.)
        return -1;
    }

    m_entityParams.push_back(ep);
    return startIdx;
}

const ParameterTable::EntityParams* ParameterTable::findEntityParams(
    uint64_t entityId) const {
    for (const auto& ep : m_entityParams) {
        if (ep.entityId == entityId) return &ep;
    }
    return nullptr;
}

bool ParameterTable::hasEntity(uint64_t entityId) const {
    return findEntityParams(entityId) != nullptr;
}

int ParameterTable::parameterIndex(const GeometryRef& ref) const {
    const auto* ep = findEntityParams(ref.entityId);
    if (!ep) {
        throw std::runtime_error("ParameterTable: entity " +
                                 std::to_string(ref.entityId) + " not registered");
    }

    int base = ep->startIndex;

    if (ref.featureType == FeatureType::Point) {
        if (ep->entityType == "line") {
            // Point(0) = start [base+0, base+1], Point(1) = end [base+2, base+3]
            return base + ref.featureIndex * 2;
        } else if (ep->entityType == "circle") {
            // Point(0) = center [base+0, base+1]
            return base;
        } else if (ep->entityType == "arc") {
            if (ref.featureIndex == 0) return base;  // center
            // Point(1) = start point, Point(2) = end point — computed from angles
            // For solver purposes, arc start/end points are derived, not direct params.
            // We return the center index; the constraint should use the circle feature instead.
            return base;
        } else if (ep->entityType == "rectangle") {
            // Corner indices map into the 4 params [c1x, c1y, c2x, c2y]
            // BL=Point(0) => [c1x, c1y], BR=Point(1) => [c2x, c1y] -- not direct params!
            // For rectangles, only corner1 and corner2 are free params.
            // Point(0)=BL=corner1, Point(2)=TR=corner2 are direct.
            // Point(1)=BR and Point(3)=TL are derived.
            if (ref.featureIndex == 0) return base;      // corner1
            if (ref.featureIndex == 2) return base + 2;   // corner2
            // For derived corners, we'd need special handling.
            // For now, map to the closest independent corner.
            if (ref.featureIndex == 1) return base;       // BR uses c1y but c2x — complex
            if (ref.featureIndex == 3) return base;       // TL similar
            return base;
        } else if (ep->entityType == "polyline") {
            return base + ref.featureIndex * 2;
        }
    } else if (ref.featureType == FeatureType::Line) {
        if (ep->entityType == "line") {
            // Line(0) = the whole line [sx, sy, ex, ey]
            return base;
        } else if (ep->entityType == "rectangle") {
            // Rectangle edges: edge0 = BL→BR, edge1 = BR→TR, etc.
            // These are derived from corner1/corner2 — return base for now
            return base;
        } else if (ep->entityType == "polyline") {
            // Segment i: [pts[i], pts[i+1]]
            return base + ref.featureIndex * 2;
        }
    } else if (ref.featureType == FeatureType::Circle) {
        if (ep->entityType == "circle") {
            return base;  // [cx, cy, r]
        } else if (ep->entityType == "arc") {
            return base;  // [cx, cy, r, ...]
        }
    }

    return base;  // Fallback
}

math::Vec2 ParameterTable::pointPosition(const GeometryRef& ref) const {
    const auto* ep = findEntityParams(ref.entityId);
    if (!ep) {
        throw std::runtime_error("ParameterTable: entity " +
                                 std::to_string(ref.entityId) + " not registered");
    }
    int base = ep->startIndex;

    if (ep->entityType == "line") {
        int idx = base + ref.featureIndex * 2;
        return {m_values(idx), m_values(idx + 1)};
    } else if (ep->entityType == "circle") {
        return {m_values(base), m_values(base + 1)};
    } else if (ep->entityType == "arc") {
        if (ref.featureIndex == 0) return {m_values(base), m_values(base + 1)};
        // Start/end points derived from center + radius + angle
        double cx = m_values(base), cy = m_values(base + 1), r = m_values(base + 2);
        double angle = (ref.featureIndex == 1) ? m_values(base + 3) : m_values(base + 4);
        return {cx + r * std::cos(angle), cy + r * std::sin(angle)};
    } else if (ep->entityType == "rectangle") {
        double c1x = m_values(base), c1y = m_values(base + 1);
        double c2x = m_values(base + 2), c2y = m_values(base + 3);
        double minX = std::min(c1x, c2x), minY = std::min(c1y, c2y);
        double maxX = std::max(c1x, c2x), maxY = std::max(c1y, c2y);
        switch (ref.featureIndex) {
            case 0: return {minX, minY};  // BL
            case 1: return {maxX, minY};  // BR
            case 2: return {maxX, maxY};  // TR
            case 3: return {minX, maxY};  // TL
        }
    } else if (ep->entityType == "polyline") {
        int idx = base + ref.featureIndex * 2;
        return {m_values(idx), m_values(idx + 1)};
    }

    return {m_values(base), m_values(base + 1)};
}

std::pair<math::Vec2, math::Vec2> ParameterTable::lineEndpoints(
    const GeometryRef& ref) const {
    const auto* ep = findEntityParams(ref.entityId);
    if (!ep) {
        throw std::runtime_error("ParameterTable: entity " +
                                 std::to_string(ref.entityId) + " not registered");
    }
    int base = ep->startIndex;

    if (ep->entityType == "line") {
        return {{m_values(base), m_values(base + 1)},
                {m_values(base + 2), m_values(base + 3)}};
    } else if (ep->entityType == "rectangle") {
        // Reconstruct corners from params
        double c1x = m_values(base), c1y = m_values(base + 1);
        double c2x = m_values(base + 2), c2y = m_values(base + 3);
        double minX = std::min(c1x, c2x), minY = std::min(c1y, c2y);
        double maxX = std::max(c1x, c2x), maxY = std::max(c1y, c2y);
        math::Vec2 corners[4] = {{minX, minY}, {maxX, minY}, {maxX, maxY}, {minX, maxY}};
        int i = ref.featureIndex;
        return {corners[i % 4], corners[(i + 1) % 4]};
    } else if (ep->entityType == "polyline") {
        int idx = base + ref.featureIndex * 2;
        math::Vec2 s = {m_values(idx), m_values(idx + 1)};
        math::Vec2 e = {m_values(idx + 2), m_values(idx + 3)};
        return {s, e};
    }

    throw std::runtime_error("lineEndpoints: unsupported entity type " + ep->entityType);
}

std::pair<math::Vec2, double> ParameterTable::circleData(const GeometryRef& ref) const {
    const auto* ep = findEntityParams(ref.entityId);
    if (!ep) {
        throw std::runtime_error("ParameterTable: entity " +
                                 std::to_string(ref.entityId) + " not registered");
    }
    int base = ep->startIndex;

    if (ep->entityType == "circle" || ep->entityType == "arc") {
        return {{m_values(base), m_values(base + 1)}, m_values(base + 2)};
    }

    throw std::runtime_error("circleData: unsupported entity type " + ep->entityType);
}

void ParameterTable::applyToEntities(
    std::vector<std::shared_ptr<draft::DraftEntity>>& entities) const {
    for (const auto& ep : m_entityParams) {
        for (auto& entity : entities) {
            if (entity->id() != ep.entityId) continue;
            int base = ep.startIndex;

            if (auto* line = dynamic_cast<draft::DraftLine*>(entity.get())) {
                line->setStart({m_values(base), m_values(base + 1)});
                line->setEnd({m_values(base + 2), m_values(base + 3)});
            } else if (auto* circle = dynamic_cast<draft::DraftCircle*>(entity.get())) {
                circle->setCenter({m_values(base), m_values(base + 1)});
                circle->setRadius(m_values(base + 2));
            } else if (auto* arc = dynamic_cast<draft::DraftArc*>(entity.get())) {
                arc->setCenter({m_values(base), m_values(base + 1)});
                arc->setRadius(m_values(base + 2));
                arc->setStartAngle(m_values(base + 3));
                arc->setEndAngle(m_values(base + 4));
            } else if (auto* rect = dynamic_cast<draft::DraftRectangle*>(entity.get())) {
                rect->setCorner1({m_values(base), m_values(base + 1)});
                rect->setCorner2({m_values(base + 2), m_values(base + 3)});
            } else if (auto* poly = dynamic_cast<draft::DraftPolyline*>(entity.get())) {
                int n = ep.paramCount / 2;
                std::vector<math::Vec2> pts(n);
                for (int i = 0; i < n; ++i) {
                    pts[i] = {m_values(base + 2 * i), m_values(base + 2 * i + 1)};
                }
                poly->setPoints(pts);
            }
            break;
        }
    }
}

ParameterTable ParameterTable::buildFromEntities(
    const std::vector<std::shared_ptr<draft::DraftEntity>>& entities,
    const ConstraintSystem& constraints) {
    // Collect all entity IDs referenced by constraints
    std::set<uint64_t> neededIds;
    for (const auto& c : constraints.constraints()) {
        auto ids = c->referencedEntityIds();
        neededIds.insert(ids.begin(), ids.end());
    }

    ParameterTable table;
    for (const auto& entity : entities) {
        if (neededIds.count(entity->id())) {
            table.registerEntity(*entity);
        }
    }
    return table;
}

}  // namespace hz::cstr
