#pragma once

#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "horizon/math/Vec2.h"
#include "horizon/ui/Tool.h"

namespace hz::ui {

/// A regular polygon: the centre, then a corner, which sets its size and how
/// it is turned (AutoCAD's POLYGON, inscribed). Up and Down add and take
/// away a side (with Shift, ten), from 3 to 1,024; the count is kept for the
/// next polygon. Either point can be typed, and the corner as a length from
/// the centre toward the cursor. It is drawn as one closed polyline, which
/// extrudes into a prism.
class PolygonTool : public Tool {
public:
    static constexpr int kFewestSides = 3;
    static constexpr int kMostSides = 1024;

    std::string name() const override { return "Polygon"; }

    void activate(ViewportWidget* viewport) override;
    void deactivate() override;

    bool mousePressEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool mouseMoveEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool mouseReleaseEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool keyPressEvent(QKeyEvent* event) override;
    void cancel() override;

    std::vector<std::pair<math::Vec2, math::Vec2>> getPreviewLines() const override;

    bool acceptsTypedPoints() const override { return true; }
    std::optional<math::Vec2> basePoint() const override {
        if (m_state != State::WaitingForCorner) return std::nullopt;
        return m_center;
    }

    std::string promptText() const override;
    bool wantsCrosshair() const override { return true; }

    /// How many sides the next polygon has.
    int sides() const { return m_sides; }
    void setSides(int sides);

    /// The corners of the regular polygon about @p center with a corner at
    /// @p corner, counterclockwise from it; none when the two are one point
    /// or either is not a finite number.
    static std::vector<math::Vec2> corners(const math::Vec2& center, const math::Vec2& corner,
                                           int sides);

private:
    enum class State { WaitingForCenter, WaitingForCorner };
    State m_state = State::WaitingForCenter;
    math::Vec2 m_center;
    math::Vec2 m_currentPos;
    int m_sides = 6;
};

}  // namespace hz::ui
