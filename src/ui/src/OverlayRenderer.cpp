#include "horizon/ui/OverlayRenderer.h"

#include <cmath>

#include "horizon/render/Camera.h"
#include "horizon/render/GLRenderer.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace hz::ui {

void OverlayRenderer::render(QOpenGLExtraFunctions* gl, const render::Camera& camera,
                             render::GLRenderer* renderer, int viewportWidth, int viewportHeight,
                             double pixelScale) {
    if (m_crosshairEnabled) {
        renderCrosshair(gl, camera, renderer, viewportWidth, viewportHeight, pixelScale);
    }

    if (m_snapResult.type != draft::SnapType::None) {
        renderSnapMarker(gl, camera, renderer, pixelScale);
    }

    if (m_axisEnabled) {
        renderAxisIndicator(gl, camera, renderer, viewportWidth, viewportHeight, pixelScale);
    }
}

// ---------------------------------------------------------------------------
// Snap markers — distinct shapes per snap type
// ---------------------------------------------------------------------------

// Helper: push a line segment with distance attribute (solid overlays use dist=0).
static void pushSeg(std::vector<float>& v, float x0, float y0, float x1, float y1) {
    float len = std::sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0));
    v.push_back(x0);
    v.push_back(y0);
    v.push_back(0.0f);
    v.push_back(0.0f);
    v.push_back(x1);
    v.push_back(y1);
    v.push_back(0.0f);
    v.push_back(len);
}

void OverlayRenderer::renderSnapMarker(QOpenGLExtraFunctions* gl, const render::Camera& camera,
                                       render::GLRenderer* renderer, double pixelScale) {
    double size = 8.0 * pixelScale;
    float cx = static_cast<float>(m_snapResult.point.x);
    float cy = static_cast<float>(m_snapResult.point.y);
    float s = static_cast<float>(size);

    std::vector<float> verts;
    math::Vec3 yellow{1.0, 1.0, 0.0};

    switch (m_snapResult.type) {
        case draft::SnapType::Endpoint: {
            // Square
            pushSeg(verts, cx - s, cy - s, cx + s, cy - s);
            pushSeg(verts, cx + s, cy - s, cx + s, cy + s);
            pushSeg(verts, cx + s, cy + s, cx - s, cy + s);
            pushSeg(verts, cx - s, cy + s, cx - s, cy - s);
            break;
        }
        case draft::SnapType::Midpoint: {
            // Triangle
            pushSeg(verts, cx, cy + s, cx - s, cy - s);
            pushSeg(verts, cx - s, cy - s, cx + s, cy - s);
            pushSeg(verts, cx + s, cy - s, cx, cy + s);
            break;
        }
        case draft::SnapType::Center: {
            // Circle (12 segments)
            const int segs = 12;
            for (int i = 0; i < segs; ++i) {
                double a1 = 2.0 * M_PI * i / segs;
                double a2 = 2.0 * M_PI * (i + 1) / segs;
                pushSeg(verts, cx + s * static_cast<float>(std::cos(a1)),
                        cy + s * static_cast<float>(std::sin(a1)),
                        cx + s * static_cast<float>(std::cos(a2)),
                        cy + s * static_cast<float>(std::sin(a2)));
            }
            break;
        }
        case draft::SnapType::Grid: {
            // X cross
            pushSeg(verts, cx - s, cy - s, cx + s, cy + s);
            pushSeg(verts, cx - s, cy + s, cx + s, cy - s);
            break;
        }
        case draft::SnapType::None:
        default:
            break;
    }

    if (!verts.empty()) {
        renderer->drawLines(gl, camera, verts, yellow, 2.0f);
    }
}

// ---------------------------------------------------------------------------
// Crosshair — full-viewport thin gray lines through cursor
// ---------------------------------------------------------------------------

void OverlayRenderer::renderCrosshair(QOpenGLExtraFunctions* gl, const render::Camera& camera,
                                      render::GLRenderer* renderer, int vpW, int vpH,
                                      double pixelScale) {
    double extent = std::max(vpW, vpH) * pixelScale * 2.0;

    float wx = static_cast<float>(m_crosshairWorld.x);
    float wy = static_cast<float>(m_crosshairWorld.y);
    float e = static_cast<float>(extent);

    std::vector<float> verts;
    pushSeg(verts, wx - e, wy, wx + e, wy);
    pushSeg(verts, wx, wy - e, wx, wy + e);

    math::Vec3 gray{0.45, 0.45, 0.45};
    renderer->drawLines(gl, camera, verts, gray, 1.0f);
}

// ---------------------------------------------------------------------------
// Axis triad — screen-anchored X/Y/Z orientation gizmo (bottom-left)
// ---------------------------------------------------------------------------

// Push a 3D segment with a distance attribute (solid overlays use dist=0).
static void pushSeg3(std::vector<float>& v, float x0, float y0, float z0, float x1, float y1,
                     float z1) {
    v.push_back(x0);
    v.push_back(y0);
    v.push_back(z0);
    v.push_back(0.0f);
    v.push_back(x1);
    v.push_back(y1);
    v.push_back(z1);
    v.push_back(0.0f);
}

void OverlayRenderer::renderAxisIndicator(QOpenGLExtraFunctions* gl, const render::Camera& camera,
                                          render::GLRenderer* renderer, int vpW, int vpH,
                                          double pixelScale) {
    const double axisLen = 30.0 * pixelScale;

    // Anchor the triad to the ground plane under a fixed bottom-left screen
    // point so it stays put while the user pans/zooms.
    auto [rayO, rayD] = camera.screenToRay(40.0, static_cast<double>(vpH) - 40.0, vpW, vpH);
    math::Vec3 origin;
    if (std::abs(rayD.z) > 1e-12) {
        origin = rayO + rayD * (-rayO.z / rayD.z);
    } else {
        origin = {rayO.x, rayO.y, 0.0};
    }

    const float ox = static_cast<float>(origin.x);
    const float oy = static_cast<float>(origin.y);
    const float oz = static_cast<float>(origin.z);
    const float len = static_cast<float>(axisLen);
    const float a = static_cast<float>(6.0 * pixelScale);

    const math::Vec3 red{0.92, 0.26, 0.26};
    const math::Vec3 green{0.30, 0.82, 0.30};
    const math::Vec3 blue{0.36, 0.56, 0.95};

    // X axis (+world X, red) with a chevron arrowhead in the XY plane.
    std::vector<float> xv;
    pushSeg3(xv, ox, oy, oz, ox + len, oy, oz);
    pushSeg3(xv, ox + len, oy, oz, ox + len - a, oy + a * 0.5f, oz);
    pushSeg3(xv, ox + len, oy, oz, ox + len - a, oy - a * 0.5f, oz);
    renderer->drawLines(gl, camera, xv, red, 2.2f);

    // Y axis (+world Y, green).
    std::vector<float> yv;
    pushSeg3(yv, ox, oy, oz, ox, oy + len, oz);
    pushSeg3(yv, ox, oy + len, oz, ox - a * 0.5f, oy + len - a, oz);
    pushSeg3(yv, ox, oy + len, oz, ox + a * 0.5f, oy + len - a, oz);
    renderer->drawLines(gl, camera, yv, green, 2.2f);

    // Z axis (+world Z / up, blue) — the new third axis that makes this a true
    // 3D orientation triad; arrowhead splayed in the XZ plane.
    std::vector<float> zv;
    pushSeg3(zv, ox, oy, oz, ox, oy, oz + len);
    pushSeg3(zv, ox, oy, oz + len, ox - a * 0.5f, oy, oz + len - a);
    pushSeg3(zv, ox, oy, oz + len, ox + a * 0.5f, oy, oz + len - a);
    renderer->drawLines(gl, camera, zv, blue, 2.2f);
}

}  // namespace hz::ui
