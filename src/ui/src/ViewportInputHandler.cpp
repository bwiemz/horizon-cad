#include "horizon/ui/ViewportInputHandler.h"
#include "horizon/ui/ViewportWidget.h"
#include "horizon/ui/Tool.h"

#include <QMouseEvent>
#include <QWheelEvent>
#include <QKeyEvent>
#include <QOpenGLWidget>

namespace hz::ui {

void ViewportInputHandler::handleMousePress(QMouseEvent* event, ViewportWidget* viewport) {
    m_lastMousePos = event->pos();

    if (event->button() == Qt::MiddleButton) {
        if (event->modifiers() & Qt::ShiftModifier) {
            m_panning = true;
        } else {
            m_orbiting = true;
        }
        return;
    }

    if (event->button() == Qt::LeftButton && viewport->activeTool()) {
        math::Vec2 wp = viewport->worldPositionAtCursor(event->pos().x(), event->pos().y());
        if (viewport->activeTool()->mousePressEvent(event, wp)) {
            emit viewport->selectionChanged();
            viewport->update();
            return;
        }
    }

    // Let QOpenGLWidget handle unprocessed events.
    // (The base class call is handled by ViewportWidget's forwarding method.)
}

void ViewportInputHandler::handleMouseMove(QMouseEvent* event, ViewportWidget* viewport) {
    QPoint delta = event->pos() - m_lastMousePos;

    if (m_orbiting) {
        double yaw   = static_cast<double>(delta.x()) * 0.005;
        double pitch  = static_cast<double>(delta.y()) * 0.005;
        viewport->camera().orbit(yaw, pitch);
        m_lastMousePos = event->pos();
        viewport->update();
        return;
    }

    if (m_panning) {
        double dx = static_cast<double>(delta.x()) * 0.01;
        double dy = static_cast<double>(delta.y()) * 0.01;
        viewport->camera().pan(dx, -dy);  // negate Y because screen Y is flipped
        m_lastMousePos = event->pos();
        viewport->update();
        return;
    }

    // Compute the world position and emit signal for status bar.
    math::Vec2 wp = viewport->worldPositionAtCursor(event->pos().x(), event->pos().y());
    emit viewport->mouseMoved(wp);

    // Update crosshair position for overlay renderer.
    viewport->overlayRenderer().setCrosshairWorldPos(wp);

    // Delegate to tool.
    if (viewport->activeTool()) {
        if (viewport->activeTool()->mouseMoveEvent(event, wp)) {
            viewport->update();
        }
    }

    m_lastMousePos = event->pos();
}

void ViewportInputHandler::handleMouseRelease(QMouseEvent* event, ViewportWidget* viewport) {
    if (event->button() == Qt::MiddleButton) {
        m_orbiting = false;
        m_panning = false;
        return;
    }

    if (event->button() == Qt::LeftButton && viewport->activeTool()) {
        math::Vec2 wp = viewport->worldPositionAtCursor(event->pos().x(), event->pos().y());
        if (viewport->activeTool()->mouseReleaseEvent(event, wp)) {
            emit viewport->selectionChanged();
            viewport->update();
            return;
        }
    }

    // Let QOpenGLWidget handle unprocessed events.
}

void ViewportInputHandler::handleWheel(QWheelEvent* event, ViewportWidget* viewport) {
    double delta = event->angleDelta().y() / 120.0;
    double factor = 1.0 + delta * 0.1;
    viewport->camera().zoom(factor);
    viewport->update();
}

void ViewportInputHandler::handleKeyPress(QKeyEvent* event, ViewportWidget* viewport) {
    if (event->key() == Qt::Key_Escape && viewport->activeTool()) {
        viewport->activeTool()->cancel();
        viewport->update();
        return;
    }

    if (viewport->activeTool() && viewport->activeTool()->keyPressEvent(event)) {
        emit viewport->selectionChanged();
        viewport->update();
        return;
    }

    // Let QOpenGLWidget handle unprocessed events.
}

}  // namespace hz::ui
