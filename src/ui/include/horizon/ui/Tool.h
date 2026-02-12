#pragma once

#include <string>
#include <utility>
#include <vector>

#include "horizon/math/Vec2.h"

class QMouseEvent;
class QKeyEvent;

namespace hz::render {
class Camera;
}

namespace hz::ui {

class ViewportWidget;

/// Abstract base class for interactive drawing/editing tools.
class Tool {
public:
    virtual ~Tool() = default;

    /// Returns the display name of this tool.
    virtual std::string name() const = 0;

    /// Called when the tool becomes the active tool.
    virtual void activate(ViewportWidget* viewport) { m_viewport = viewport; }

    /// Called when the tool is deactivated.
    virtual void deactivate() { m_viewport = nullptr; }

    /// Handle a mouse press in the viewport.  Returns true if the event was consumed.
    virtual bool mousePressEvent(QMouseEvent* event, const math::Vec2& worldPos) = 0;

    /// Handle mouse movement in the viewport.  Returns true if the event was consumed.
    virtual bool mouseMoveEvent(QMouseEvent* event, const math::Vec2& worldPos) = 0;

    /// Handle a mouse release in the viewport.  Returns true if the event was consumed.
    virtual bool mouseReleaseEvent(QMouseEvent* event, const math::Vec2& worldPos) = 0;

    /// Handle a key press.  Returns true if the event was consumed.
    virtual bool keyPressEvent(QKeyEvent* event) { return false; }

    /// Cancel the current operation (e.g. when Escape is pressed).
    virtual void cancel() {}

    /// Return preview line segments (start, end) to draw while the tool is active.
    virtual std::vector<std::pair<math::Vec2, math::Vec2>> getPreviewLines() const { return {}; }

    /// Return preview circles (center, radius) to draw while the tool is active.
    virtual std::vector<std::pair<math::Vec2, double>> getPreviewCircles() const { return {}; }

    /// Arc preview data.
    struct ArcPreview {
        math::Vec2 center;
        double radius;
        double startAngle;
        double endAngle;
    };

    /// Return preview arcs to draw while the tool is active.
    virtual std::vector<ArcPreview> getPreviewArcs() const { return {}; }

    /// Returns the current tool prompt text for the status bar (e.g. "Specify first point").
    virtual std::string promptText() const { return ""; }

    /// Returns true if this tool wants a full-viewport crosshair cursor.
    virtual bool wantsCrosshair() const { return false; }

protected:
    ViewportWidget* m_viewport = nullptr;
};

}  // namespace hz::ui
