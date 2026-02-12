#pragma once

#include "horizon/ui/Tool.h"
#include "horizon/math/Vec2.h"

namespace hz::ui {

/// Measure the angle formed by three points (vertex + two rays).
/// Click: vertex, point on first ray, point on second ray.
class MeasureAngleTool : public Tool {
public:
    std::string name() const override { return "MeasureAngle"; }

    void activate(ViewportWidget* viewport) override;
    void deactivate() override;

    bool mousePressEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool mouseMoveEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool mouseReleaseEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool keyPressEvent(QKeyEvent* event) override;
    void cancel() override;

    std::vector<std::pair<math::Vec2, math::Vec2>> getPreviewLines() const override;

private:
    enum class State { Vertex, Ray1, Ray2 };
    State m_state = State::Vertex;
    math::Vec2 m_vertex;
    math::Vec2 m_ray1Point;
    math::Vec2 m_currentPos;
};

}  // namespace hz::ui
