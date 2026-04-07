#pragma once

#include <QPoint>

class QMouseEvent;
class QWheelEvent;
class QKeyEvent;

namespace hz::ui {

class ViewportWidget;

/// Handles mouse and keyboard input for the viewport (pan, orbit, zoom, tool dispatch).
/// Extracted from ViewportWidget to keep the widget class focused on coordination.
class ViewportInputHandler {
public:
    ViewportInputHandler() = default;

    void handleMousePress(QMouseEvent* event, ViewportWidget* viewport);
    void handleMouseMove(QMouseEvent* event, ViewportWidget* viewport);
    void handleMouseRelease(QMouseEvent* event, ViewportWidget* viewport);
    void handleWheel(QWheelEvent* event, ViewportWidget* viewport);
    void handleKeyPress(QKeyEvent* event, ViewportWidget* viewport);

private:
    bool m_orbiting = false;
    bool m_panning = false;
    QPoint m_lastMousePos;
};

}  // namespace hz::ui
