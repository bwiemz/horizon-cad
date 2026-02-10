#include "horizon/render/Grid.h"

#include <QOpenGLExtraFunctions>

namespace hz::render {

// Embedded grid shader sources so we don't depend on file loading for the grid.
static const char* kGridVertSrc = R"glsl(
#version 330 core

layout(location = 0) in vec3 aPos;

out vec3 vNearPoint;
out vec3 vFarPoint;

uniform mat4 uInvViewProj;

// Unproject a point from NDC to world space.
vec3 unprojectPoint(vec3 ndc) {
    vec4 world = uInvViewProj * vec4(ndc, 1.0);
    return world.xyz / world.w;
}

void main() {
    vNearPoint = unprojectPoint(vec3(aPos.xy, -1.0));
    vFarPoint  = unprojectPoint(vec3(aPos.xy,  1.0));
    gl_Position = vec4(aPos.xy, 0.0, 1.0);
}
)glsl";

static const char* kGridFragSrc = R"glsl(
#version 330 core

in vec3 vNearPoint;
in vec3 vFarPoint;

out vec4 FragColor;

uniform mat4 uViewProj;
uniform float uNear;
uniform float uFar;

// Draw grid lines on the XY plane (Z = 0).
vec4 grid(vec3 fragPos3D, float scale) {
    vec2 coord = fragPos3D.xy * scale;
    vec2 derivative = fwidth(coord);
    vec2 grid = abs(fract(coord - 0.5) - 0.5) / derivative;
    float lineVal = min(grid.x, grid.y);
    float minimumz = min(derivative.y, 1.0);
    float minimumx = min(derivative.x, 1.0);
    vec4 color = vec4(0.35, 0.35, 0.35, 1.0 - min(lineVal, 1.0));

    // Highlight X axis (red)
    if (fragPos3D.y > -minimumz * 0.5 && fragPos3D.y < minimumz * 0.5) {
        color = vec4(0.8, 0.2, 0.2, 1.0 - min(lineVal, 1.0));
        color.a = max(color.a, 0.5);
    }
    // Highlight Y axis (green)
    if (fragPos3D.x > -minimumx * 0.5 && fragPos3D.x < minimumx * 0.5) {
        color = vec4(0.2, 0.8, 0.2, 1.0 - min(lineVal, 1.0));
        color.a = max(color.a, 0.5);
    }
    return color;
}

float computeDepth(vec3 pos) {
    vec4 clipPos = uViewProj * vec4(pos, 1.0);
    return (clipPos.z / clipPos.w) * 0.5 + 0.5;  // map to [0, 1]
}

float computeLinearDepth(vec3 pos) {
    vec4 clipPos = uViewProj * vec4(pos, 1.0);
    float clipDepth = clipPos.z / clipPos.w;
    clipDepth = clipDepth * 0.5 + 0.5;  // NDC to [0,1]
    float linearDepth = (2.0 * uNear * uFar) /
                        (uFar + uNear - (2.0 * clipDepth - 1.0) * (uFar - uNear));
    return linearDepth / uFar;
}

void main() {
    // Find where the ray from near to far intersects the XY plane (Z = 0)
    float t = -vNearPoint.z / (vFarPoint.z - vNearPoint.z);

    // Discard fragments that don't intersect the plane
    if (t < 0.0) discard;

    vec3 fragPos3D = vNearPoint + t * (vFarPoint - vNearPoint);

    // Write correct depth
    gl_FragDepth = computeDepth(fragPos3D);

    float linearDepth = computeLinearDepth(fragPos3D);
    float fadeFactor = max(0.0, 1.0 - linearDepth);

    // Two scales of grid: 1m and 10m
    vec4 gridSmall = grid(fragPos3D, 1.0);
    vec4 gridLarge = grid(fragPos3D, 0.1);

    // Blend the two grids
    FragColor = gridSmall;
    FragColor.a *= fadeFactor;

    // Add large grid on top
    FragColor = mix(FragColor, gridLarge, gridLarge.a * 0.6);
    FragColor.a *= fadeFactor;

    if (FragColor.a < 0.01) discard;
}
)glsl";

Grid::Grid() = default;
Grid::~Grid() = default;

void Grid::initialize(QOpenGLExtraFunctions* gl) {
    if (m_initialized) return;

    // Compile grid shader
    if (!m_shader.create(kGridVertSrc, kGridFragSrc)) {
        qWarning("Grid: failed to create grid shader");
        return;
    }

    // Create a full-screen triangle (covers the entire viewport with just 3 vertices)
    // Using the oversized-triangle trick: 3 vertices that cover [-1,1] NDC space.
    std::vector<float> quadPositions = {
        -1.0f, -1.0f, 0.0f,
         3.0f, -1.0f, 0.0f,
        -1.0f,  3.0f, 0.0f,
    };

    m_quad.createPositionOnly(gl, quadPositions);
    m_initialized = true;
}

void Grid::render(QOpenGLExtraFunctions* gl, const Camera& camera) {
    if (!m_initialized) return;

    math::Mat4 vp = camera.viewProjectionMatrix();
    math::Mat4 invVP = vp.inverse();

    m_shader.bind();
    m_shader.setUniform("uViewProj", vp);
    m_shader.setUniform("uInvViewProj", invVP);
    m_shader.setUniform("uNear", static_cast<float>(0.1));
    m_shader.setUniform("uFar", static_cast<float>(10000.0));

    gl->glEnable(GL_BLEND);
    gl->glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    gl->glEnable(GL_DEPTH_TEST);
    gl->glDepthFunc(GL_LEQUAL);

    m_quad.bind();
    m_quad.draw(gl);
    m_quad.release();

    gl->glDepthFunc(GL_LESS);

    m_shader.release();
}

}  // namespace hz::render
