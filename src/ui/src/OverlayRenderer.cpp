#include "horizon/ui/OverlayRenderer.h"
#include "horizon/render/GLRenderer.h"
#include "horizon/render/Camera.h"

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace hz::ui {

void OverlayRenderer::render(QOpenGLExtraFunctions* gl,
                              const render::Camera& camera,
                              render::GLRenderer* renderer,
                              int viewportWidth, int viewportHeight,
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
    v.push_back(x0); v.push_back(y0); v.push_back(0.0f); v.push_back(0.0f);
    v.push_back(x1); v.push_back(y1); v.push_back(0.0f); v.push_back(len);
}

void OverlayRenderer::renderSnapMarker(QOpenGLExtraFunctions* gl,
                                        const render::Camera& camera,
                                        render::GLRenderer* renderer,
                                        double pixelScale) {
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
                pushSeg(verts,
                        cx + s * static_cast<float>(std::cos(a1)),
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

void OverlayRenderer::renderCrosshair(QOpenGLExtraFunctions* gl,
                                       const render::Camera& camera,
                                       render::GLRenderer* renderer,
                                       int vpW, int vpH,
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
// Axis indicator — small X/Y axes in the bottom-left corner
// ---------------------------------------------------------------------------

void OverlayRenderer::renderAxisIndicator(QOpenGLExtraFunctions* gl,
                                           const render::Camera& camera,
                                           render::GLRenderer* renderer,
                                           int vpW, int vpH,
                                           double pixelScale) {
    double margin = 35.0 * pixelScale;
    double axisLen = 30.0 * pixelScale;

    auto [rayO, rayD] = camera.screenToRay(35.0, static_cast<double>(vpH) - 35.0, vpW, vpH);

    math::Vec2 origin;
    if (std::abs(rayD.z) > 1e-12) {
        double t = -rayO.z / rayD.z;
        math::Vec3 hit = rayO + rayD * t;
        origin = {hit.x, hit.y};
    } else {
        origin = {rayO.x, rayO.y};
    }

    float ox = static_cast<float>(origin.x);
    float oy = static_cast<float>(origin.y);
    float len = static_cast<float>(axisLen);
    float arrow = static_cast<float>(6.0 * pixelScale);

    // X axis (red)
    std::vector<float> xVerts;
    pushSeg(xVerts, ox, oy, ox + len, oy);
    math::Vec3 red{0.9, 0.2, 0.2};
    renderer->drawLines(gl, camera, xVerts, red, 2.0f);

    // Y axis (green)
    std::vector<float> yVerts;
    pushSeg(yVerts, ox, oy, ox, oy + len);
    math::Vec3 green{0.2, 0.8, 0.2};
    renderer->drawLines(gl, camera, yVerts, green, 2.0f);

    // X arrowhead
    std::vector<float> xArrow;
    pushSeg(xArrow, ox + len, oy, ox + len - arrow, oy + arrow * 0.5f);
    pushSeg(xArrow, ox + len, oy, ox + len - arrow, oy - arrow * 0.5f);
    renderer->drawLines(gl, camera, xArrow, red, 2.0f);

    // Y arrowhead
    std::vector<float> yArrow;
    pushSeg(yArrow, ox, oy + len, ox - arrow * 0.5f, oy + len - arrow);
    pushSeg(yArrow, ox, oy + len, ox + arrow * 0.5f, oy + len - arrow);
    renderer->drawLines(gl, camera, yArrow, green, 2.0f);
}

}  // namespace hz::ui
