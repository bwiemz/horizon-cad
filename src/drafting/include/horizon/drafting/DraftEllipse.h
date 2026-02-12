#pragma once

#include "DraftEntity.h"

namespace hz::draft {

/// An ellipse defined by center, semi-major radius, semi-minor radius, and rotation.
class DraftEllipse : public DraftEntity {
public:
    /// Construct an ellipse.
    /// @param center      Center point
    /// @param semiMajor   Half-length of the major axis
    /// @param semiMinor   Half-length of the minor axis
    /// @param rotation    Rotation of the major axis from the X-axis (radians)
    DraftEllipse(const math::Vec2& center, double semiMajor, double semiMinor,
                 double rotation = 0.0);

    math::BoundingBox boundingBox() const override;
    bool hitTest(const math::Vec2& point, double tolerance) const override;
    std::vector<math::Vec2> snapPoints() const override;
    void translate(const math::Vec2& delta) override;
    std::shared_ptr<DraftEntity> clone() const override;
    void mirror(const math::Vec2& axisP1, const math::Vec2& axisP2) override;
    void rotate(const math::Vec2& center, double angle) override;
    void scale(const math::Vec2& center, double factor) override;

    /// Generate points on the ellipse curve for rendering / intersection.
    std::vector<math::Vec2> evaluate(int segments = 64) const;

    // Accessors
    const math::Vec2& center() const { return m_center; }
    double semiMajor() const { return m_semiMajor; }
    double semiMinor() const { return m_semiMinor; }
    double rotation() const { return m_rotation; }

    void setCenter(const math::Vec2& center) { m_center = center; }
    void setSemiMajor(double v) { m_semiMajor = v; }
    void setSemiMinor(double v) { m_semiMinor = v; }
    void setRotation(double v) { m_rotation = v; }

private:
    math::Vec2 m_center;
    double m_semiMajor;
    double m_semiMinor;
    double m_rotation;  // radians
};

}  // namespace hz::draft
