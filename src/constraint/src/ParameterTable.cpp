#include "horizon/constraint/ParameterTable.h"

#include <cmath>
#include <set>
#include <stdexcept>
#include <string>

#include "horizon/constraint/ConstraintSystem.h"
#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftEntity.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftPolyline.h"
#include "horizon/drafting/DraftRectangle.h"

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
        ep.closed = poly->closed();
        m_values.conservativeResize(startIdx + 2 * n);
        for (int i = 0; i < n; ++i) {
            m_values(startIdx + 2 * i + 0) = poly->points()[i].x;
            m_values(startIdx + 2 * i + 1) = poly->points()[i].y;
        }
    } else {
        // Unsupported entity type for constraints (dimensions, leaders, etc.)
        return -1;
    }

    m_byId.emplace(ep.entityId, m_entityParams.size());
    m_entityParams.push_back(ep);
    // An edge of the part projected (Phase 157) is where the part puts it:
    // the solver moves what is tied to it, never it.
    m_fixed.resize(static_cast<size_t>(m_values.size()), false);
    if (!entity.sourceEdge().empty()) {
        for (int k = 0; k < ep.paramCount; ++k) m_fixed[static_cast<size_t>(startIdx + k)] = true;
    }
    return startIdx;
}

const ParameterTable::EntityParams* ParameterTable::findEntityParams(uint64_t entityId) const {
    const auto it = m_byId.find(entityId);
    return it == m_byId.end() ? nullptr : &m_entityParams[it->second];
}

bool ParameterTable::hasEntity(uint64_t entityId) const {
    return findEntityParams(entityId) != nullptr;
}

std::pair<int, int> ParameterTable::parameterRange(uint64_t entityId) const {
    const auto* ep = findEntityParams(entityId);
    return ep ? std::pair<int, int>{ep->startIndex, ep->paramCount} : std::pair<int, int>{0, 0};
}

namespace {

/// How many points an entity of @p type with @p paramCount parameters has,
/// in the order its Point features number them.
int pointCount(const std::string& type, int paramCount) {
    if (type == "line") return 2;       // start, end
    if (type == "circle") return 1;     // centre
    if (type == "arc") return 3;        // centre, start, end
    if (type == "rectangle") return 4;  // BL, BR, TR, TL
    if (type == "polyline") return paramCount / 2;
    return 0;
}

/// How many segments it has, as its Line features number them: a closed
/// polyline's last runs back to its first point.
int segmentCount(const std::string& type, int paramCount, bool closed) {
    if (type == "line") return 1;
    if (type == "rectangle") return 4;
    if (type == "polyline") {
        const int n = paramCount / 2;
        return n < 2 ? 0 : (closed ? n : n - 1);
    }
    return 0;
}

std::runtime_error doesNotHave(const std::string& type, uint64_t id, const std::string& what) {
    return std::runtime_error(type + " " + std::to_string(id) + " has no " + what);
}

}  // namespace

const ParameterTable::EntityParams& ParameterTable::entityOf(const GeometryRef& ref) const {
    const auto* ep = findEntityParams(ref.entityId);
    if (!ep) {
        throw std::runtime_error("ParameterTable: entity " + std::to_string(ref.entityId) +
                                 " not registered");
    }
    return *ep;
}

math::Vec2 ParameterTable::point(const EntityParams& ep, int index, PointJacobian* jac) const {
    if (index < 0 || index >= pointCount(ep.entityType, ep.paramCount)) {
        throw doesNotHave(ep.entityType, ep.entityId, "point " + std::to_string(index));
    }
    const int base = ep.startIndex;
    // A point that is two of the table's parameters, x then y.
    const auto pair = [&](int at) {
        if (jac) {
            jac->add(at, {1.0, 0.0});
            jac->add(at + 1, {0.0, 1.0});
        }
        return math::Vec2{m_values(at), m_values(at + 1)};
    };
    if (ep.entityType == "line" || ep.entityType == "polyline") return pair(base + 2 * index);
    const bool centre = ep.entityType == "circle" || (ep.entityType == "arc" && index == 0);
    if (centre) return pair(base);
    if (ep.entityType == "arc") {
        // Its start or end: the centre plus the radius at that angle, so it
        // moves with all four.
        const double r = m_values(base + 2);
        const int angleAt = base + (index == 1 ? 3 : 4);
        const double c = std::cos(m_values(angleAt)), s = std::sin(m_values(angleAt));
        if (jac) {
            jac->add(base, {1.0, 0.0});
            jac->add(base + 1, {0.0, 1.0});
            jac->add(base + 2, {c, s});
            jac->add(angleAt, {-r * s, r * c});
        }
        return {m_values(base) + r * c, m_values(base + 1) + r * s};
    }
    // A rectangle's corner: its x is the left or the right of its two
    // corners' x, and its y the lower or the upper of their y. Each is the
    // one parameter it is equal to; corner 1 is taken as the left (lower)
    // on a tie, so a rectangle with no width can be given one.
    const bool c1Left = m_values(base) <= m_values(base + 2);
    const bool c1Low = m_values(base + 1) <= m_values(base + 3);
    const bool right = index == 1 || index == 2;
    const bool top = index == 2 || index == 3;
    const int xAt = base + (right == c1Left ? 2 : 0);
    const int yAt = base + 1 + (top == c1Low ? 2 : 0);
    if (jac) {
        jac->add(xAt, {1.0, 0.0});
        jac->add(yAt, {0.0, 1.0});
    }
    return {m_values(xAt), m_values(yAt)};
}

std::pair<math::Vec2, math::Vec2> ParameterTable::segment(const EntityParams& ep, int index,
                                                          PointJacobian* js,
                                                          PointJacobian* je) const {
    const int count = segmentCount(ep.entityType, ep.paramCount, ep.closed);
    if (index < 0 || index >= count) {
        throw doesNotHave(ep.entityType, ep.entityId, "line " + std::to_string(index));
    }
    if (ep.entityType == "line") return {point(ep, 0, js), point(ep, 1, je)};
    // A rectangle's edge k runs from its corner k to the next; a polyline's
    // segment i from its point i to the next, the last of a closed one back
    // to its first.
    const int points = pointCount(ep.entityType, ep.paramCount);
    return {point(ep, index, js), point(ep, (index + 1) % points, je)};
}

int ParameterTable::parameterIndex(const GeometryRef& ref) const {
    const EntityParams& ep = entityOf(ref);
    const int base = ep.startIndex;

    if (ref.featureType == FeatureType::Point) {
        point(ep, ref.featureIndex, nullptr);  // throws if it has none
        if (ep.entityType == "line" || ep.entityType == "polyline") {
            return base + ref.featureIndex * 2;
        }
        const bool centre =
            ep.entityType == "circle" || (ep.entityType == "arc" && ref.featureIndex == 0);
        if (centre) return base;
        throw std::runtime_error(ep.entityType + " " + std::to_string(ep.entityId) + "'s point " +
                                 std::to_string(ref.featureIndex) +
                                 " is not two of its parameters: see pointJacobian()");
    }
    if (ref.featureType == FeatureType::Line) {
        segment(ep, ref.featureIndex, nullptr, nullptr);  // throws if it has none
        const int n = ep.paramCount / 2;
        if (ep.entityType == "line") return base;
        if (ep.entityType == "polyline" && ref.featureIndex < n - 1) {
            return base + ref.featureIndex * 2;  // [pts[i], pts[i+1]]
        }
        throw std::runtime_error(ep.entityType + " " + std::to_string(ep.entityId) + "'s line " +
                                 std::to_string(ref.featureIndex) +
                                 " is not four of its parameters: see lineJacobian()");
    }
    circleData(ref);  // throws if it has none
    return base;      // [cx, cy, r]
}

math::Vec2 ParameterTable::pointPosition(const GeometryRef& ref) const {
    return point(entityOf(ref), ref.featureIndex, nullptr);
}

PointJacobian ParameterTable::pointJacobian(const GeometryRef& ref) const {
    PointJacobian jac;
    point(entityOf(ref), ref.featureIndex, &jac);
    return jac;
}

std::pair<math::Vec2, math::Vec2> ParameterTable::lineEndpoints(const GeometryRef& ref) const {
    return segment(entityOf(ref), ref.featureIndex, nullptr, nullptr);
}

std::pair<PointJacobian, PointJacobian> ParameterTable::lineJacobian(const GeometryRef& ref) const {
    std::pair<PointJacobian, PointJacobian> jac;
    segment(entityOf(ref), ref.featureIndex, &jac.first, &jac.second);
    return jac;
}

std::pair<math::Vec2, double> ParameterTable::circleData(const GeometryRef& ref) const {
    const EntityParams& ep = entityOf(ref);
    if ((ep.entityType != "circle" && ep.entityType != "arc") || ref.featureIndex != 0) {
        throw doesNotHave(ep.entityType, ep.entityId, "circle " + std::to_string(ref.featureIndex));
    }
    const int base = ep.startIndex;
    return {{m_values(base), m_values(base + 1)}, m_values(base + 2)};
}

void ParameterTable::applyToEntities(
    std::vector<std::shared_ptr<draft::DraftEntity>>& entities) const {
    for (auto& entity : entities) {
        if (entity) applyToEntity(*entity);
    }
}

void ParameterTable::applyToEntity(draft::DraftEntity& entity) const {
    const EntityParams* ep = findEntityParams(entity.id());
    if (!ep) return;
    const int base = ep->startIndex;
    if (auto* line = dynamic_cast<draft::DraftLine*>(&entity)) {
        line->setStart({m_values(base), m_values(base + 1)});
        line->setEnd({m_values(base + 2), m_values(base + 3)});
    } else if (auto* circle = dynamic_cast<draft::DraftCircle*>(&entity)) {
        circle->setCenter({m_values(base), m_values(base + 1)});
        circle->setRadius(m_values(base + 2));
    } else if (auto* arc = dynamic_cast<draft::DraftArc*>(&entity)) {
        arc->setCenter({m_values(base), m_values(base + 1)});
        arc->setRadius(m_values(base + 2));
        arc->setStartAngle(m_values(base + 3));
        arc->setEndAngle(m_values(base + 4));
    } else if (auto* rect = dynamic_cast<draft::DraftRectangle*>(&entity)) {
        rect->setCorner1({m_values(base), m_values(base + 1)});
        rect->setCorner2({m_values(base + 2), m_values(base + 3)});
    } else if (auto* poly = dynamic_cast<draft::DraftPolyline*>(&entity)) {
        const int n = ep->paramCount / 2;
        std::vector<math::Vec2> pts(static_cast<size_t>(n));
        for (int i = 0; i < n; ++i) {
            pts[static_cast<size_t>(i)] = {m_values(base + 2 * i), m_values(base + 2 * i + 1)};
        }
        poly->setPoints(pts);
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
