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
uniform int uPerspective;

// How strongly the lines a distance `spacing` apart are drawn here: 1 on a
// line, falling to 0 a pixel away. `pixel` is how much of the plane a pixel
// covers along each axis.
float lines(vec2 p, vec2 pixel, float spacing) {
    vec2 coord = p / spacing;
    vec2 derivative = pixel / spacing;
    vec2 g = abs(fract(coord - 0.5) - 0.5) / derivative;
    return 1.0 - min(min(g.x, g.y), 1.0);
}

// The grid on the XY plane (Z = 0), in powers of ten that suit the zoom:
// lines are never drawn closer together than kMinPixels, so a view zoomed far
// out shows a coarser grid instead of a plane solid with lines, and one
// zoomed far in a finer grid instead of none. The finest level shown fades
// out as the view zooms out towards the next.
vec4 grid(vec3 fragPos3D) {
    const float kMinPixels = 8.0;
    vec2 pixel = max(fwidth(fragPos3D.xy), vec2(1e-30));
    float lod = log(max(pixel.x, pixel.y) * kMinPixels) / log(10.0);
    float fine = pow(10.0, floor(lod) + 1.0);
    // Full strength down to three times the closest spacing, then fading out
    // as the view zooms out to it, where the next level takes over.
    float spacingPixels = fine / max(pixel.x, pixel.y);
    float fade = clamp((spacingPixels - kMinPixels) / (2.0 * kMinPixels), 0.0, 1.0);

    // Every tenth line is drawn, and every hundredth drawn brighter.
    float a = max(lines(fragPos3D.xy, pixel, fine) * fade,
                  lines(fragPos3D.xy, pixel, fine * 10.0));
    float major = lines(fragPos3D.xy, pixel, fine * 100.0);
    vec4 color = vec4(mix(vec3(0.35), vec3(0.5), major), a);

    // The axes, two pixels wide at any zoom.
    if (abs(fragPos3D.y) < pixel.y) {
        color = vec4(0.8, 0.2, 0.2, max(a, 0.5));
    }
    if (abs(fragPos3D.x) < pixel.x) {
        color = vec4(0.2, 0.8, 0.2, max(a, 0.5));
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
    float denom = vFarPoint.z - vNearPoint.z;
    // Parallel to the plane, at any scale.
    if (abs(denom) <= 1e-7 * max(abs(vFarPoint.z), abs(vNearPoint.z))) discard;
    float t = -vNearPoint.z / denom;

    // Discard fragments that don't intersect the plane
    if (t < 0.0) discard;

    vec3 fragPos3D = vNearPoint + t * (vFarPoint - vNearPoint);

    // Write correct depth
    gl_FragDepth = computeDepth(fragPos3D);

    // A perspective view fades the grid out towards the far plane; an
    // orthographic one has no distance to fade it by.
    float fadeFactor = 1.0;
    if (uPerspective != 0) fadeFactor = max(0.0, 1.0 - computeLinearDepth(fragPos3D));

    FragColor = grid(fragPos3D);
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
        -1.0f, -1.0f, 0.0f, 3.0f, -1.0f, 0.0f, -1.0f, 3.0f, 0.0f,
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
    // The planes the camera projects with: the grid fades towards its far
    // plane, wherever the view places it.
    const bool perspective = camera.projectionType() == ProjectionType::Perspective;
    m_shader.setUniform("uNear", static_cast<float>(perspective ? camera.nearPlane() : 0.1));
    m_shader.setUniform("uFar", static_cast<float>(perspective ? camera.farPlane() : 10000.0));
    m_shader.setUniform("uPerspective", perspective ? 1 : 0);

    gl->glEnable(GL_BLEND);
    gl->glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    gl->glEnable(GL_DEPTH_TEST);
    gl->glDepthFunc(GL_LEQUAL);
    // The grid is a reference, drawn behind what is drawn after it, and
    // hides none of it: the drawing lies on its plane, at its depth, and
    // lost the depth test to it wherever its lines were close together.
    gl->glDepthMask(GL_FALSE);

    m_quad.bind();
    m_quad.draw(gl);
    m_quad.release();

    gl->glDepthMask(GL_TRUE);
    gl->glDepthFunc(GL_LESS);

    m_shader.release();
}

}  // namespace hz::render
