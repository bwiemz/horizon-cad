#pragma once

#include "DraftEntity.h"
#include "BlockDefinition.h"
#include "horizon/math/Vec2.h"
#include <memory>
#include <string>

namespace hz::draft {

/// A reference (instance) of a BlockDefinition with position, rotation, and uniform scale.
class DraftBlockRef : public DraftEntity {
public:
    DraftBlockRef(std::shared_ptr<BlockDefinition> definition,
                  const math::Vec2& insertPos,
                  double rotation = 0.0,
                  double uniformScale = 1.0);

    // -- DraftEntity pure virtuals --
    math::BoundingBox boundingBox() const override;
    bool hitTest(const math::Vec2& point, double tolerance) const override;
    std::vector<math::Vec2> snapPoints() const override;
    void translate(const math::Vec2& delta) override;
    std::shared_ptr<DraftEntity> clone() const override;
    void mirror(const math::Vec2& axisP1, const math::Vec2& axisP2) override;
    void rotate(const math::Vec2& center, double angle) override;
    void scale(const math::Vec2& center, double factor) override;

    // -- Accessors --
    const std::shared_ptr<BlockDefinition>& definition() const { return m_definition; }
    const std::string& blockName() const { return m_definition->name; }
    const math::Vec2& insertPos() const { return m_insertPos; }
    double rotation() const { return m_rotation; }
    double uniformScale() const { return m_uniformScale; }

    void setInsertPos(const math::Vec2& pos) { m_insertPos = pos; }
    void setRotation(double radians) { m_rotation = radians; }
    void setUniformScale(double s) { m_uniformScale = s; }

    /// Transform a point from definition space to world space.
    math::Vec2 transformPoint(const math::Vec2& defPt) const;

    /// Transform a point from world space to definition space.
    math::Vec2 inverseTransformPoint(const math::Vec2& worldPt) const;

private:
    std::shared_ptr<BlockDefinition> m_definition;
    math::Vec2 m_insertPos;
    double m_rotation;
    double m_uniformScale;
};

}  // namespace hz::draft
