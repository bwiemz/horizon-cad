#include "horizon/drafting/DraftBlockRef.h"
#include "horizon/math/MathUtils.h"
#include <cmath>

namespace hz::draft {

DraftBlockRef::DraftBlockRef(std::shared_ptr<BlockDefinition> definition,
                             const math::Vec2& insertPos,
                             double rotation,
                             double uniformScale)
    : m_definition(std::move(definition))
    , m_insertPos(insertPos)
    , m_rotation(rotation)
    , m_uniformScale(uniformScale) {}

math::Vec2 DraftBlockRef::transformPoint(const math::Vec2& defPt) const {
    // worldPt = insertPos + rotate((defPt - basePoint) * scale, rotation)
    math::Vec2 local = (defPt - m_definition->basePoint) * m_uniformScale;
    double c = std::cos(m_rotation), s = std::sin(m_rotation);
    return {m_insertPos.x + local.x * c - local.y * s,
            m_insertPos.y + local.x * s + local.y * c};
}

math::Vec2 DraftBlockRef::inverseTransformPoint(const math::Vec2& worldPt) const {
    // Reverse: defPt = rotate(worldPt - insertPos, -rotation) / scale + basePoint
    math::Vec2 d = worldPt - m_insertPos;
    double c = std::cos(-m_rotation), s = std::sin(-m_rotation);
    math::Vec2 rotated = {d.x * c - d.y * s, d.x * s + d.y * c};
    double invScale = (std::abs(m_uniformScale) > 1e-12) ? (1.0 / m_uniformScale) : 0.0;
    return rotated * invScale + m_definition->basePoint;
}

math::BoundingBox DraftBlockRef::boundingBox() const {
    math::BoundingBox bbox;
    for (const auto& ent : m_definition->entities) {
        auto subBB = ent->boundingBox();
        if (!subBB.isValid()) continue;
        // Transform all 4 corners of the sub-entity bbox.
        math::Vec3 lo = subBB.min();
        math::Vec3 hi = subBB.max();
        math::Vec2 corners[4] = {
            {lo.x, lo.y}, {hi.x, lo.y}, {hi.x, hi.y}, {lo.x, hi.y}};
        for (const auto& corner : corners) {
            math::Vec2 w = transformPoint(corner);
            bbox.expand(math::Vec3(w.x, w.y, 0.0));
        }
    }
    return bbox;
}

bool DraftBlockRef::hitTest(const math::Vec2& point, double tolerance) const {
    // Inverse-transform point into definition space.
    math::Vec2 defPt = inverseTransformPoint(point);
    double defTolerance = (std::abs(m_uniformScale) > 1e-12)
                              ? (tolerance / std::abs(m_uniformScale))
                              : tolerance;
    for (const auto& ent : m_definition->entities) {
        if (ent->hitTest(defPt, defTolerance)) return true;
    }
    return false;
}

std::vector<math::Vec2> DraftBlockRef::snapPoints() const {
    std::vector<math::Vec2> points;
    // Insert point is always a snap point.
    points.push_back(m_insertPos);
    // Transformed snap points from sub-entities.
    for (const auto& ent : m_definition->entities) {
        for (const auto& sp : ent->snapPoints()) {
            points.push_back(transformPoint(sp));
        }
    }
    return points;
}

void DraftBlockRef::translate(const math::Vec2& delta) {
    m_insertPos += delta;
}

std::shared_ptr<DraftEntity> DraftBlockRef::clone() const {
    auto copy = std::make_shared<DraftBlockRef>(m_definition, m_insertPos, m_rotation,
                                                m_uniformScale);
    copy->setLayer(layer());
    copy->setColor(color());
    copy->setLineWidth(lineWidth());
    copy->setLineType(lineType());
    copy->setGroupId(groupId());
    return copy;
}

void DraftBlockRef::mirror(const math::Vec2& axisP1, const math::Vec2& axisP2) {
    // Mirror the insert point.
    math::Vec2 d = (axisP2 - axisP1).normalized();
    math::Vec2 v = m_insertPos - axisP1;
    m_insertPos = axisP1 + d * (2.0 * v.dot(d)) - v;

    // Mirroring negates the rotation and adds PI reflection.
    double axisAngle = std::atan2(d.y, d.x);
    m_rotation = 2.0 * axisAngle - m_rotation;
    m_rotation = math::normalizeAngle(m_rotation);

    // Mirroring flips scale sign to reflect sub-entities.
    m_uniformScale = -m_uniformScale;
}

void DraftBlockRef::rotate(const math::Vec2& center, double angle) {
    // Rotate the insert point around center.
    double c = std::cos(angle), s = std::sin(angle);
    math::Vec2 v = m_insertPos - center;
    m_insertPos = {center.x + v.x * c - v.y * s, center.y + v.x * s + v.y * c};
    m_rotation = math::normalizeAngle(m_rotation + angle);
}

void DraftBlockRef::scale(const math::Vec2& center, double factor) {
    m_insertPos = center + (m_insertPos - center) * factor;
    m_uniformScale *= factor;
}

}  // namespace hz::draft
