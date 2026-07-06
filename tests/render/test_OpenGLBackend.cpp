#include <gtest/gtest.h>

#include <QGuiApplication>
#include <QOffscreenSurface>
#include <QOpenGLContext>
#include <QOpenGLExtraFunctions>
#include <memory>

#include "horizon/render/OpenGLBackend.h"

using namespace hz::render;

namespace {

/// Offscreen GL fixture: skips (rather than fails) on machines/CI runners
/// without a usable OpenGL implementation.
class OpenGLBackendTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Strictly opt-in: Qt GUI initialization can HANG (not fail) in
        // headless/service sessions — observed on Windows with both the
        // native and offscreen platforms — and a hung test wedges CI. Set
        // HZ_RENDER_TESTS_PLATFORM (e.g. "windows", "xcb", "offscreen") on a
        // machine with a real session to exercise the GL backend; automated
        // coverage comes from the RAL contract tests and the Vulkan backend
        // tests.
        const QByteArray platform = qgetenv("HZ_RENDER_TESTS_PLATFORM");
        if (platform.isEmpty()) {
            GTEST_SKIP() << "GL runtime tests are opt-in: set HZ_RENDER_TESTS_PLATFORM";
        }

        if (QGuiApplication::instance() == nullptr) {
            qputenv("QT_QPA_PLATFORM", platform);
            static int argc = 1;
            static char arg0[] = "hz_render_tests";
            static char* argv[] = {arg0, nullptr};
            static QGuiApplication app(argc, argv);
            (void)app;
        }

        // Match the application's context profile (src/app/main.cpp): a
        // default-format context can silently be Compatibility profile, where
        // Core-only mistakes (like drawing without a VAO) go undetected.
        QSurfaceFormat fmt;
        fmt.setVersion(3, 3);
        fmt.setProfile(QSurfaceFormat::CoreProfile);

        m_context = std::make_unique<QOpenGLContext>();
        m_context->setFormat(fmt);
        if (!m_context->create()) {
            GTEST_SKIP() << "No OpenGL 3.3 Core context available";
        }
        m_surface = std::make_unique<QOffscreenSurface>();
        m_surface->setFormat(m_context->format());
        m_surface->create();
        if (!m_surface->isValid() || !m_context->makeCurrent(m_surface.get())) {
            GTEST_SKIP() << "No offscreen OpenGL surface available";
        }
        m_gl = m_context->extraFunctions();
    }

    void TearDown() override {
        if (m_context != nullptr && m_context->isValid()) {
            m_context->doneCurrent();
        }
    }

    std::unique_ptr<QOpenGLContext> m_context;
    std::unique_ptr<QOffscreenSurface> m_surface;
    QOpenGLExtraFunctions* m_gl = nullptr;
};

constexpr char kVertexSrc[] = R"(#version 330 core
layout(location = 0) in vec3 aPos;
void main() { gl_Position = vec4(aPos, 1.0); }
)";

constexpr char kFragmentSrc[] = R"(#version 330 core
out vec4 FragColor;
void main() { FragColor = vec4(1.0, 0.5, 0.2, 1.0); }
)";

}  // namespace

TEST_F(OpenGLBackendTest, ReportsVersionedName) {
    OpenGLBackend backend(m_gl);
    EXPECT_NE(backend.name().find("OpenGL"), std::string::npos);
}

TEST_F(OpenGLBackendTest, BufferLifecycle) {
    OpenGLBackend backend(m_gl);
    const float data[] = {1.0f, 2.0f, 3.0f};
    const BufferHandle handle = backend.createBuffer(BufferUsage::Vertex, data, sizeof(data));
    ASSERT_TRUE(handle.isValid());

    const float updated[] = {4.0f, 5.0f, 6.0f};
    backend.updateBuffer(handle, updated, sizeof(updated));
    backend.destroyBuffer(handle);

    EXPECT_FALSE(backend.createBuffer(BufferUsage::Vertex, nullptr, 0).isValid());
}

TEST_F(OpenGLBackendTest, ShaderCompileAndFailure) {
    OpenGLBackend backend(m_gl);
    const ShaderHandle good = backend.createShader(kVertexSrc, kFragmentSrc);
    EXPECT_TRUE(good.isValid());
    backend.destroyShader(good);

    const ShaderHandle bad = backend.createShader("not glsl at all", kFragmentSrc);
    EXPECT_FALSE(bad.isValid());
}

TEST_F(OpenGLBackendTest, DrawTriangleThroughRal) {
    OpenGLBackend backend(m_gl);

    const float vertices[] = {-0.5f, -0.5f, 0.0f, 0.5f, -0.5f, 0.0f, 0.0f, 0.5f, 0.0f};
    const BufferHandle vb = backend.createBuffer(BufferUsage::Vertex, vertices, sizeof(vertices));
    const ShaderHandle sh = backend.createShader(kVertexSrc, kFragmentSrc);
    ASSERT_TRUE(vb.isValid());
    ASSERT_TRUE(sh.isValid());

    RenderPassDesc pass;
    pass.clearColor[0] = 0.1f;
    backend.beginPass(pass);

    DrawCall call;
    call.shader = sh;
    call.vertexBuffer = vb;
    call.layout.attributeComponents[0] = 3;
    call.layout.attributeCount = 1;
    call.elementCount = 3;
    backend.draw(call);
    backend.endPass();

    EXPECT_EQ(m_gl->glGetError(), 0u) << "RAL draw left a GL error";

    backend.destroyShader(sh);
    backend.destroyBuffer(vb);
}

TEST_F(OpenGLBackendTest, TextureLifecycle) {
    OpenGLBackend backend(m_gl);
    const uint32_t pixels[4] = {0xFFFFFFFF, 0xFF000000, 0xFF0000FF, 0xFFFF0000};
    const TextureHandle tex = backend.createTexture(2, 2, pixels);
    ASSERT_TRUE(tex.isValid());
    backend.destroyTexture(tex);

    EXPECT_FALSE(backend.createTexture(0, 0, nullptr).isValid());
    EXPECT_EQ(m_gl->glGetError(), 0u);
}
