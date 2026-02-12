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

void OverlayRenderer::renderSnapMarker(QOpenGLExtraFunctions* gl,
                                        const render::Camera& camera,
                                        render::GLRenderer* renderer,
                                        double pixelScale) {
    double size = 8.0 * pixelScale;  // 8 pixels in world units
    float cx = static_cast<float>(m_snapResult.point.x);
    float cy = static_cast<float>(m_snapResult.point.y);
    float s = static_cast<float>(size);

    std::vector<float> verts;
    math::Vec3 yellow{1.0, 1.0, 0.0};

    switch (m_snapResult.type) {
        case draft::SnapType::Endpoint: {
            // Square
            verts = {
                cx - s, cy - s, 0.0f,  cx + s, cy - s, 0.0f,
                cx + s, cy - s, 0.0f,  cx + s, cy + s, 0.0f,
                cx + s, cy + s, 0.0f,  cx - s, cy + s, 0.0f,
                cx - s, cy + s, 0.0f,  cx - s, cy - s, 0.0f,
            };
            break;
        }
        case draft::SnapType::Midpoint: {
            // Triangle (pointing up)
            verts = {
                cx,     cy + s, 0.0f,   cx - s, cy - s, 0.0f,
                cx - s, cy - s, 0.0f,   cx + s, cy - s, 0.0f,
                cx + s, cy - s, 0.0f,   cx,     cy + s, 0.0f,
            };
            break;
        }
        case draft::SnapType::Center: {
            // Circle (12 segments)
            const int segs = 12;
            for (int i = 0; i < segs; ++i) {
                double a1 = 2.0 * M_PI * i / segs;
                double a2 = 2.0 * M_PI * (i + 1) / segs;
                verts.push_back(cx + s * static_cast<float>(std::cos(a1)));
                verts.push_back(cy + s * static_cast<float>(std::sin(a1)));
                verts.push_back(0.0f);
                verts.push_back(cx + s * static_cast<float>(std::cos(a2)));
                verts.push_back(cy + s * static_cast<float>(std::sin(a2)));
                verts.push_back(0.0f);
            }
            break;
        }
        case draft::SnapType::Grid: {
            // X cross
            verts = {
                cx - s, cy - s, 0.0f,  cx + s, cy + s, 0.0f,
                cx - s, cy + s, 0.0f,  cx + s, cy - s, 0.0f,
            };
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
    // Build two full-viewport lines through the cursor world position.
    // Extend far enough to cover the visible area.
    double extent = std::max(vpW, vpH) * pixelScale * 2.0;

    float wx = static_cast<float>(m_crosshairWorld.x);
    float wy = static_cast<float>(m_crosshairWorld.y);
    float e = static_cast<float>(extent);

    // Horizontal line (left to right at cursor Y).
    // Vertical line (bottom to top at cursor X).
    std::vector<float> verts = {
        wx - e, wy, 0.0f,   wx + e, wy, 0.0f,
        wx, wy - e, 0.0f,   wx, wy + e, 0.0f,
    };

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
    // Position: bottom-left corner, 35 pixels from edges.
    // Axis length: 30 pixels.
    double margin = 35.0 * pixelScale;
    double axisLen = 30.0 * pixelScale;

    // Get world coordinates of the bottom-left area.
    // The camera is perspective with Z-up convention.
    // We need the world position at the bottom-left corner of the viewport.
    // Use screenToRay to unproject the bottom-left corner.

    // Screen coords: (marginPx, vpH - marginPx) in Qt coords (0=top).
    auto [rayO, rayD] = camera.screenToRay(35.0, static_cast<double>(vpH) - 35.0, vpW, vpH);

    // Intersect with Z=0 plane.
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

    // X axis (red)
    std::vector<float> xVerts = {
        ox, oy, 0.0f,   ox + len, oy, 0.0f,
    };
    math::Vec3 red{0.9, 0.2, 0.2};
    renderer->drawLines(gl, camera, xVerts, red, 2.0f);

    // Y axis (green)
    std::vector<float> yVerts = {
        ox, oy, 0.0f,   ox, oy + len, 0.0f,
    };
    math::Vec3 green{0.2, 0.8, 0.2};
    renderer->drawLines(gl, camera, yVerts, green, 2.0f);

    // Small arrowheads.
    float arrow = static_cast<float>(6.0 * pixelScale);

    // X arrowhead
    std::vector<float> xArrow = {
        ox + len, oy, 0.0f,   ox + len - arrow, oy + arrow * 0.5f, 0.0f,
        ox + len, oy, 0.0f,   ox + len - arrow, oy - arrow * 0.5f, 0.0f,
    };
    renderer->drawLines(gl, camera, xArrow, red, 2.0f);

    // Y arrowhead
    std::vector<float> yArrow = {
        ox, oy + len, 0.0f,   ox - arrow * 0.5f, oy + len - arrow, 0.0f,
        ox, oy + len, 0.0f,   ox + arrow * 0.5f, oy + len - arrow, 0.0f,
    };
    renderer->drawLines(gl, camera, yArrow, green, 2.0f);
}

}  // namespace hz::ui
