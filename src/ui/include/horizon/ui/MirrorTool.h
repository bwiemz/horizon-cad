#pragma once

#include "horizon/ui/Tool.h"
#include "horizon/math/Vec2.h"

namespace hz::ui {

/// Mirror tool: pick two points defining a mirror axis, then mirror selected entities.
///
/// Requires entities to be selected before use.
/// - First click sets the first axis point
/// - Second click sets the second axis point and commits the mirror
class MirrorTool : public Tool {
public:
    std::string name() const override { return "Mirror"; }

    bool mousePressEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool mouseMoveEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool mouseReleaseEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool keyPressEvent(QKeyEvent* event) override;
    void cancel() override;

    std::vector<std::pair<math::Vec2, math::Vec2>> getPreviewLines() const override;

private:
    enum class State { SelectFirstPoint, SelectSecondPoint };
    State m_state = State::SelectFirstPoint;

    math::Vec2 m_axisP1;
    math::Vec2 m_currentPos;
};

}  // namespace hz::ui
