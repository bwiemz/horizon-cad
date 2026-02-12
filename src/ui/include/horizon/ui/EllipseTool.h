#pragma once

#include "horizon/ui/Tool.h"
#include "horizon/math/Vec2.h"

namespace hz::ui {

/// Three-click ellipse drawing tool.
///
/// 1. Click center point
/// 2. Click major axis endpoint (defines semi-major radius and rotation)
/// 3. Click to set minor axis radius
/// Escape cancels at any stage.
class EllipseTool : public Tool {
public:
    std::string name() const override { return "Ellipse"; }

    void activate(ViewportWidget* viewport) override;
    void deactivate() override;

    bool mousePressEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool mouseMoveEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool mouseReleaseEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool keyPressEvent(QKeyEvent* event) override;
    void cancel() override;

    std::vector<std::pair<math::Vec2, math::Vec2>> getPreviewLines() const override;

    std::string promptText() const override;
    bool wantsCrosshair() const override;

private:
    enum class State { Center, MajorAxis, MinorAxis };

    void finishEllipse();

    /// Generate preview points for an ellipse.
    static std::vector<math::Vec2> evaluateEllipse(
        const math::Vec2& center, double semiMajor, double semiMinor,
        double rotation, int segments = 64);

    State m_state = State::Center;
    math::Vec2 m_center;
    math::Vec2 m_majorAxisPt;
    double m_semiMajor = 0.0;
    double m_rotation = 0.0;
    math::Vec2 m_currentPos;
};

}  // namespace hz::ui
