#pragma once

#include "horizon/ui/Tool.h"
#include "horizon/math/Vec2.h"

namespace hz::ui {

/// Scale tool: pick base point, then scale factor via mouse distance ratio or typed value.
///
/// Requires entities to be selected before use.
/// - First click sets the base point
/// - Mouse distance from base relative to selection centroid defines scale factor
/// - Or type scale factor + Enter
class ScaleTool : public Tool {
public:
    std::string name() const override { return "Scale"; }

    bool mousePressEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool mouseMoveEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool mouseReleaseEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool keyPressEvent(QKeyEvent* event) override;
    void cancel() override;

    std::vector<std::pair<math::Vec2, math::Vec2>> getPreviewLines() const override;
    std::vector<std::pair<math::Vec2, double>> getPreviewCircles() const override;
    std::vector<ArcPreview> getPreviewArcs() const override;

private:
    enum class State { SelectBasePoint, SelectScaleFactor };
    State m_state = State::SelectBasePoint;

    math::Vec2 m_basePoint;
    double m_referenceDist = 1.0;
    math::Vec2 m_currentPos;
    std::string m_factorInput;
};

}  // namespace hz::ui
