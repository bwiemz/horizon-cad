#pragma once

#include "horizon/math/Vec2.h"
#include "horizon/drafting/SnapEngine.h"

#include <vector>

class QOpenGLExtraFunctions;

namespace hz::render {
class Camera;
class GLRenderer;
}  // namespace hz::render

namespace hz::ui {

/// Manages GL-based overlay rendering: crosshair, snap markers, axis indicator.
/// Called from ViewportWidget::paintGL() after the main scene.
class OverlayRenderer {
public:
    OverlayRenderer() = default;

    /// Render all overlays.  Call after main scene rendering, before text overlay.
    void render(QOpenGLExtraFunctions* gl,
                const render::Camera& camera,
                render::GLRenderer* renderer,
                int viewportWidth, int viewportHeight,
                double pixelScale);

    // ---- Snap markers ----

    void setSnapResult(const draft::SnapResult& snap) { m_snapResult = snap; }

    // ---- Crosshair ----

    void setCrosshairEnabled(bool enabled) { m_crosshairEnabled = enabled; }
    void setCrosshairWorldPos(const math::Vec2& worldPos) { m_crosshairWorld = worldPos; }

    // ---- Axis indicator ----

    void setAxisIndicatorEnabled(bool enabled) { m_axisEnabled = enabled; }

private:
    void renderSnapMarker(QOpenGLExtraFunctions* gl,
                          const render::Camera& camera,
                          render::GLRenderer* renderer,
                          double pixelScale);

    void renderCrosshair(QOpenGLExtraFunctions* gl,
                         const render::Camera& camera,
                         render::GLRenderer* renderer,
                         int vpW, int vpH,
                         double pixelScale);

    void renderAxisIndicator(QOpenGLExtraFunctions* gl,
                             const render::Camera& camera,
                             render::GLRenderer* renderer,
                             int vpW, int vpH,
                             double pixelScale);

    draft::SnapResult m_snapResult;
    bool m_crosshairEnabled = false;
    math::Vec2 m_crosshairWorld;
    bool m_axisEnabled = true;
};

}  // namespace hz::ui
