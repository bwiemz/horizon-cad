#pragma once

#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "horizon/math/Vec2.h"
#include "horizon/math/Vec3.h"

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
    virtual bool keyPressEvent(QKeyEvent* /*event*/) { return false; }

    /// Cancel the current operation (e.g. when Escape is pressed).
    virtual void cancel() {}

    /// Whether the tool places points, so a point typed at the keyboard
    /// (TypedPoint) can be given to it as a click. A tool that reads digits
    /// itself (a fillet radius) returns false.
    virtual bool acceptsTypedPoints() const { return false; }

    /// Whether the tool is part way through a value typed into it (a fillet
    /// radius, a rotation): its letters and spaces are the value's text
    /// ("5 mm", "0.5 rad"), not the window's one-key shortcuts (Phase 154).
    virtual bool typingText() const { return false; }

    /// The point the next one is measured from, if there is one: the start
    /// of the line being drawn, a circle's centre. Relative input
    /// ("@dx,dy"), a length typed alone, and ortho and polar tracking all
    /// work from it.
    virtual std::optional<math::Vec2> basePoint() const { return std::nullopt; }

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

    /// Returns the color for preview geometry (lines, circles, arcs).
    virtual math::Vec3 previewColor() const { return {0.0, 0.8, 1.0}; }

protected:
    ViewportWidget* m_viewport = nullptr;
};

}  // namespace hz::ui
