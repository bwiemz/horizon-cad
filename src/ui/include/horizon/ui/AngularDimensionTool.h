#pragma once

#include "Tool.h"

namespace hz::ui {

/// Tool for creating angular dimensions between two lines.
/// Click line1, click line2, click to position the dimension arc.
class AngularDimensionTool : public Tool {
public:
    std::string name() const override { return "Angular Dimension"; }

    void activate(ViewportWidget* viewport) override;
    void deactivate() override;

    bool mousePressEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool mouseMoveEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool mouseReleaseEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool keyPressEvent(QKeyEvent* event) override;
    void cancel() override;

    std::vector<std::pair<math::Vec2, math::Vec2>> getPreviewLines() const override;

private:
    enum class State { WaitingForLine1, WaitingForLine2, WaitingForArcPos };

    /// Compute the intersection of two infinite lines.
    static bool lineIntersection(const math::Vec2& a1, const math::Vec2& a2,
                                  const math::Vec2& b1, const math::Vec2& b2,
                                  math::Vec2& result);

    State m_state = State::WaitingForLine1;
    math::Vec2 m_line1Start, m_line1End;
    math::Vec2 m_line2Start, m_line2End;
    math::Vec2 m_vertex;
    math::Vec2 m_currentPos;
};

}  // namespace hz::ui
