#pragma once

#include "Tool.h"
#include "horizon/drafting/DraftLinearDimension.h"

namespace hz::ui {

/// Tool for creating linear (horizontal/vertical/aligned) dimensions.
/// 3-click: point1, point2, dimension line position.
class LinearDimensionTool : public Tool {
public:
    std::string name() const override { return "Linear Dimension"; }

    void activate(ViewportWidget* viewport) override;
    void deactivate() override;

    bool mousePressEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool mouseMoveEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool mouseReleaseEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool keyPressEvent(QKeyEvent* event) override;
    void cancel() override;

    std::vector<std::pair<math::Vec2, math::Vec2>> getPreviewLines() const override;

private:
    enum class State { WaitingForPoint1, WaitingForPoint2, WaitingForDimLine };

    /// Auto-detect orientation from the third click position.
    draft::DraftLinearDimension::Orientation detectOrientation() const;

    State m_state = State::WaitingForPoint1;
    math::Vec2 m_point1;
    math::Vec2 m_point2;
    math::Vec2 m_currentPos;

    // Orientation override: -1 = auto-detect, 0/1/2 = H/V/Aligned
    int m_orientationOverride = -1;
};

}  // namespace hz::ui
