#pragma once

#include "horizon/ui/Tool.h"
#include "horizon/constraint/Constraint.h"
#include "horizon/constraint/GeometryRef.h"
#include "horizon/math/Vec2.h"

namespace hz::ui {

/// Interactive tool for adding geometric constraints.
/// Supports all 10 constraint types via setMode().
class ConstraintTool : public Tool {
public:
    enum class Mode {
        Coincident,
        Horizontal,
        Vertical,
        Perpendicular,
        Parallel,
        Tangent,
        Equal,
        Fixed,
        Distance,
        Angle,
    };

    ConstraintTool();

    std::string name() const override { return "Constraint"; }

    void activate(ViewportWidget* viewport) override;
    void deactivate() override;

    bool mousePressEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool mouseMoveEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool mouseReleaseEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool keyPressEvent(QKeyEvent* event) override;
    void cancel() override;

    std::vector<std::pair<math::Vec2, math::Vec2>> getPreviewLines() const override;
    std::vector<std::pair<math::Vec2, double>> getPreviewCircles() const override;

    std::string promptText() const override;
    bool wantsCrosshair() const override;

    void setMode(Mode mode);
    Mode mode() const { return m_mode; }

private:
    enum class State { WaitingForFirst, WaitingForSecond };

    /// Detect the nearest compatible geometry feature at worldPos.
    cstr::GeometryRef detectFeature(const math::Vec2& worldPos) const;

    /// Create and commit the constraint.
    void commitConstraint();

    /// Check if a feature type is compatible with the current mode.
    bool isCompatibleFeature(cstr::FeatureType ft) const;

    /// Get the required feature type for the current mode.
    cstr::FeatureType requiredFeatureType() const;

    /// Whether this mode only requires a single reference (e.g., Fixed).
    bool isSingleRefMode() const;

    Mode m_mode = Mode::Coincident;
    State m_state = State::WaitingForFirst;

    cstr::GeometryRef m_firstRef;
    cstr::GeometryRef m_hoveredRef;
    math::Vec2 m_hoveredPos;
    math::Vec2 m_firstPos;
};

}  // namespace hz::ui
