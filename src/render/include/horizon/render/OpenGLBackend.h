#pragma once

#include <memory>
#include <unordered_map>

#include "horizon/render/RenderBackend.h"

class QOpenGLExtraFunctions;
class QOpenGLShaderProgram;

namespace hz::render {

/// RenderBackend over the existing Qt OpenGL path (Phase 65).
///
/// Wraps QOpenGLExtraFunctions — the same entry points GLRenderer uses — so
/// RAL-based code and the legacy renderer can share one context. The caller
/// must keep the GL context current for the lifetime of every call.
class OpenGLBackend : public RenderBackend {
public:
    /// @p gl must outlive the backend and belong to a current context.
    explicit OpenGLBackend(QOpenGLExtraFunctions* gl);
    ~OpenGLBackend() override;

    OpenGLBackend(const OpenGLBackend&) = delete;
    OpenGLBackend& operator=(const OpenGLBackend&) = delete;

    std::string name() const override;

    BufferHandle createBuffer(BufferUsage usage, const void* data, size_t size) override;
    void updateBuffer(BufferHandle handle, const void* data, size_t size) override;
    void destroyBuffer(BufferHandle handle) override;

    TextureHandle createTexture(int width, int height, const void* rgbaPixels) override;
    void destroyTexture(TextureHandle handle) override;

    ShaderHandle createShader(const std::string& vertexSrc,
                              const std::string& fragmentSrc) override;
    void destroyShader(ShaderHandle handle) override;

    void beginPass(const RenderPassDesc& desc) override;
    void draw(const DrawCall& call) override;
    void endPass() override;

    bool submitCompute(const std::string& computeSrc, uint32_t groupsX, uint32_t groupsY,
                       uint32_t groupsZ) override;

private:
    struct BufferRecord {
        uint32_t glId = 0;
        uint32_t glTarget = 0;
        size_t size = 0;
    };

    QOpenGLExtraFunctions* m_gl;
    uint32_t m_nextHandle = 1;
    uint32_t m_vao = 0;  ///< Scratch VAO — Core Profile has no default VAO.
    std::unordered_map<uint32_t, BufferRecord> m_buffers;
    std::unordered_map<uint32_t, uint32_t> m_textures;  // handle → GL texture id
    std::unordered_map<uint32_t, std::unique_ptr<QOpenGLShaderProgram>> m_shaders;
    bool m_inPass = false;
};

}  // namespace hz::render
