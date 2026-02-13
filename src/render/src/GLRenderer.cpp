#include "horizon/render/GLRenderer.h"

#include "horizon/math/Constants.h"
#include "horizon/math/Mat4.h"

#include <QOpenGLExtraFunctions>

namespace hz::render {

// ---- Embedded Phong Shader Sources ----

static const char* kPhongVertSrc = R"glsl(
#version 330 core

layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;

out vec3 vWorldPos;
out vec3 vWorldNormal;

uniform mat4 uMVP;
uniform mat4 uModel;
uniform mat4 uNormalMatrix;

void main() {
    vec4 worldPos = uModel * vec4(aPos, 1.0);
    vWorldPos = worldPos.xyz;
    vWorldNormal = normalize((uNormalMatrix * vec4(aNormal, 0.0)).xyz);
    gl_Position = uMVP * vec4(aPos, 1.0);
}
)glsl";

static const char* kPhongFragSrc = R"glsl(
#version 330 core

in vec3 vWorldPos;
in vec3 vWorldNormal;

out vec4 FragColor;

uniform vec3 uViewPos;
uniform vec3 uLightDir;
uniform vec3 uObjectColor;

void main() {
    vec3 normal = normalize(vWorldNormal);
    vec3 lightDir = normalize(uLightDir);

    // Ambient
    float ambientStrength = 0.15;
    vec3 ambient = ambientStrength * uObjectColor;

    // Diffuse
    float diff = max(dot(normal, lightDir), 0.0);
    vec3 diffuse = diff * uObjectColor;

    // Specular (Phong)
    float specularStrength = 0.5;
    vec3 viewDir = normalize(uViewPos - vWorldPos);
    vec3 reflectDir = reflect(-lightDir, normal);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), 32.0);
    vec3 specular = specularStrength * spec * vec3(1.0);

    vec3 result = ambient + diffuse + specular;
    FragColor = vec4(result, 1.0);
}
)glsl";

// ---- Embedded Fill Shader Sources (alpha-blended quads) ----

static const char* kFillVertSrc = R"glsl(
#version 330 core

layout(location = 0) in vec3 aPos;

uniform mat4 uMVP;

void main() {
    gl_Position = uMVP * vec4(aPos, 1.0);
}
)glsl";

static const char* kFillFragSrc = R"glsl(
#version 330 core

out vec4 FragColor;

uniform vec4 uFillColor;

void main() {
    FragColor = uFillColor;
}
)glsl";

// ---- Embedded Line Shader Sources ----

static const char* kLineVertSrc = R"glsl(
#version 330 core

layout(location = 0) in vec3 aPos;

uniform mat4 uMVP;

void main() {
    gl_Position = uMVP * vec4(aPos, 1.0);
}
)glsl";

static const char* kLineFragSrc = R"glsl(
#version 330 core

out vec4 FragColor;

uniform vec3 uLineColor;

void main() {
    FragColor = vec4(uLineColor, 1.0);
}
)glsl";

// ---- GLRenderer Implementation ----

GLRenderer::GLRenderer() = default;
GLRenderer::~GLRenderer() = default;

void GLRenderer::initialize(QOpenGLExtraFunctions* gl) {
    if (m_initialized) return;

    // Compile shaders
    if (!m_phongShader.create(kPhongVertSrc, kPhongFragSrc)) {
        qWarning("GLRenderer: failed to create Phong shader");
        return;
    }

    if (!m_lineShader.create(kLineVertSrc, kLineFragSrc)) {
        qWarning("GLRenderer: failed to create line shader");
        return;
    }

    if (!m_fillShader.create(kFillVertSrc, kFillFragSrc)) {
        qWarning("GLRenderer: failed to create fill shader");
        return;
    }

    // Initialize grid
    m_grid.initialize(gl);

    // Default OpenGL state
    gl->glEnable(GL_DEPTH_TEST);
    gl->glEnable(GL_MULTISAMPLE);
    gl->glEnable(GL_LINE_SMOOTH);

    m_initialized = true;
}

void GLRenderer::resize(QOpenGLExtraFunctions* gl, int width, int height) {
    m_viewportWidth = width > 0 ? width : 1;
    m_viewportHeight = height > 0 ? height : 1;
    gl->glViewport(0, 0, m_viewportWidth, m_viewportHeight);
}

void GLRenderer::renderScene(QOpenGLExtraFunctions* gl, const SceneGraph& scene,
                              const Camera& camera) {
    if (!m_initialized) return;

    // Clear
    gl->glClearColor(m_bgColor[0], m_bgColor[1], m_bgColor[2], 1.0f);
    gl->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Render grid first (with depth writing so objects occlude it)
    renderGrid(gl, camera);

    // Render scene objects
    gl->glEnable(GL_DEPTH_TEST);
    gl->glDepthFunc(GL_LESS);

    auto visibleNodes = scene.collectVisibleMeshNodes();

    math::Mat4 view = camera.viewMatrix();
    math::Mat4 proj = camera.projectionMatrix();
    math::Mat4 vp = proj * view;

    m_phongShader.bind();

    // Light direction: from upper-right-front (in world space)
    math::Vec3 lightDir = math::Vec3(0.3, 0.5, 0.8).normalized();
    m_phongShader.setUniform("uViewPos", camera.eye());
    m_phongShader.setUniform("uLightDir", lightDir);

    for (const SceneNode* node : visibleNodes) {
        // Ensure we have a GPU mesh buffer for this node
        if (m_meshCache.find(node->id()) == m_meshCache.end()) {
            uploadMesh(gl, node);
        }

        auto it = m_meshCache.find(node->id());
        if (it == m_meshCache.end() || !it->second || !it->second->isValid()) continue;

        math::Mat4 model = node->worldTransform();
        math::Mat4 mvp = vp * model;
        math::Mat4 normalMat = model.inverse().transposed();

        m_phongShader.setUniform("uMVP", mvp);
        m_phongShader.setUniform("uModel", model);
        m_phongShader.setUniform("uNormalMatrix", normalMat);
        m_phongShader.setUniform("uObjectColor", node->material().color);

        it->second->bind();
        it->second->draw(gl);
        it->second->release();
    }

    m_phongShader.release();
}

void GLRenderer::renderGrid(QOpenGLExtraFunctions* gl, const Camera& camera) {
    m_grid.render(gl, camera);
}

void GLRenderer::setBackgroundColor(float r, float g, float b) {
    m_bgColor[0] = r;
    m_bgColor[1] = g;
    m_bgColor[2] = b;
}

void GLRenderer::drawLines(QOpenGLExtraFunctions* gl, const Camera& camera,
                            const std::vector<float>& lineVertices,
                            const math::Vec3& color, float lineWidth) {
    if (!m_initialized || lineVertices.empty()) return;

    math::Mat4 vp = camera.projectionMatrix() * camera.viewMatrix();

    m_lineShader.bind();
    m_lineShader.setUniform("uMVP", vp);
    m_lineShader.setUniform("uLineColor", color);

    // Create a temporary VAO/VBO for the lines
    GLuint vao, vbo;
    gl->glGenVertexArrays(1, &vao);
    gl->glBindVertexArray(vao);

    gl->glGenBuffers(1, &vbo);
    gl->glBindBuffer(GL_ARRAY_BUFFER, vbo);
    gl->glBufferData(GL_ARRAY_BUFFER,
                     static_cast<GLsizeiptr>(lineVertices.size() * sizeof(float)),
                     lineVertices.data(), GL_STREAM_DRAW);

    gl->glEnableVertexAttribArray(0);
    gl->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);

    gl->glLineWidth(lineWidth);
    gl->glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(lineVertices.size() / 3));

    gl->glBindVertexArray(0);
    gl->glDeleteBuffers(1, &vbo);
    gl->glDeleteVertexArrays(1, &vao);

    m_lineShader.release();
}

void GLRenderer::drawCircle(QOpenGLExtraFunctions* gl, const Camera& camera,
                              const std::vector<float>& circleVertices,
                              const math::Vec3& color, float lineWidth) {
    drawLines(gl, camera, circleVertices, color, lineWidth);
}

void GLRenderer::drawFilledQuad(QOpenGLExtraFunctions* gl, const Camera& camera,
                                 const math::Vec2& corner1, const math::Vec2& corner2,
                                 const math::Vec4& color) {
    if (!m_initialized) return;

    float x0 = static_cast<float>(std::min(corner1.x, corner2.x));
    float y0 = static_cast<float>(std::min(corner1.y, corner2.y));
    float x1 = static_cast<float>(std::max(corner1.x, corner2.x));
    float y1 = static_cast<float>(std::max(corner1.y, corner2.y));

    // Two triangles forming a quad.
    float verts[] = {
        x0, y0, 0.0f,  x1, y0, 0.0f,  x1, y1, 0.0f,
        x0, y0, 0.0f,  x1, y1, 0.0f,  x0, y1, 0.0f,
    };

    math::Mat4 vp = camera.projectionMatrix() * camera.viewMatrix();

    m_fillShader.bind();
    m_fillShader.setUniform("uMVP", vp);
    m_fillShader.setUniform("uFillColor", color);

    GLuint vao, vbo;
    gl->glGenVertexArrays(1, &vao);
    gl->glBindVertexArray(vao);

    gl->glGenBuffers(1, &vbo);
    gl->glBindBuffer(GL_ARRAY_BUFFER, vbo);
    gl->glBufferData(GL_ARRAY_BUFFER, sizeof(verts), verts, GL_STREAM_DRAW);

    gl->glEnableVertexAttribArray(0);
    gl->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);

    gl->glEnable(GL_BLEND);
    gl->glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    gl->glDepthMask(GL_FALSE);

    gl->glDrawArrays(GL_TRIANGLES, 0, 6);

    gl->glDepthMask(GL_TRUE);
    gl->glDisable(GL_BLEND);
    gl->glBindVertexArray(0);
    gl->glDeleteBuffers(1, &vbo);
    gl->glDeleteVertexArrays(1, &vao);

    m_fillShader.release();
}

void GLRenderer::uploadMesh(QOpenGLExtraFunctions* gl, const SceneNode* node) {
    if (!node || !node->hasMesh()) return;

    const MeshData& meshData = node->mesh();
    auto buffer = std::make_unique<MeshBuffer>();
    buffer->create(gl, meshData.positions, meshData.normals, meshData.indices);
    m_meshCache.emplace(node->id(), std::move(buffer));
}

}  // namespace hz::render
