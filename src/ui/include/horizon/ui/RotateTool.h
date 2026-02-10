#pragma once

#include "horizon/ui/Tool.h"
#include "horizon/math/Vec2.h"

namespace hz::ui {

/// Rotate tool: pick center of rotation, then angle via mouse or typed degrees.
///
/// Requires entities to be selected before use.
/// - First click sets the rotation center
/// - Second click (or typed angle + Enter) sets the rotation angle and commits
class RotateTool : public Tool {
public:
    std::string name() const override { return "Rotate"; }

    bool mousePressEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool mouseMoveEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool mouseReleaseEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool keyPressEvent(QKeyEvent* event) override;
    void cancel() override;

    std::vector<std::pair<math::Vec2, math::Vec2>> getPreviewLines() const override;
    std::vector<std::pair<math::Vec2, double>> getPreviewCircles() const override;
    std::vector<ArcPreview> getPreviewArcs() const override;

private:
    enum class State { SelectCenter, SelectAngle };
    State m_state = State::SelectCenter;

    math::Vec2 m_center;
    math::Vec2 m_currentPos;
    std::string m_angleInput;
};

}  // namespace hz::ui
