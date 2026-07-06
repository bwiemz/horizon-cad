#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace hz::render {

/// Opaque GPU resource handles (0 = invalid). Handles are backend-scoped:
/// a handle from one backend must never be passed to another.
struct BufferHandle {
    uint32_t id = 0;
    bool isValid() const { return id != 0; }
};
struct TextureHandle {
    uint32_t id = 0;
    bool isValid() const { return id != 0; }
};
struct ShaderHandle {
    uint32_t id = 0;
    bool isValid() const { return id != 0; }
};

enum class BufferUsage { Vertex, Index, Uniform };
enum class PrimitiveTopology { Triangles, Lines };

/// One render pass: clear state for the bound framebuffer.
struct RenderPassDesc {
    float clearColor[4] = {0.0f, 0.0f, 0.0f, 1.0f};
    bool clearColorBuffer = true;
    bool clearDepthBuffer = true;
};

/// Vertex layout: interleaved float attributes, described by component counts
/// (e.g. {3, 3} = position vec3 + normal vec3). Max 4 attributes.
struct VertexLayout {
    int attributeComponents[4] = {0, 0, 0, 0};
    int attributeCount = 0;

    int stride() const {
        int total = 0;
        for (int i = 0; i < attributeCount; ++i) total += attributeComponents[i];
        return total * static_cast<int>(sizeof(float));
    }
};

/// One draw: shader + vertex buffer (+ optional index buffer) + topology.
struct DrawCall {
    ShaderHandle shader;
    BufferHandle vertexBuffer;
    BufferHandle indexBuffer;  ///< invalid → non-indexed draw
    VertexLayout layout;
    PrimitiveTopology topology = PrimitiveTopology::Triangles;
    uint32_t elementCount = 0;  ///< indices (indexed) or vertices (non-indexed)
};

/// Rendering Abstraction Layer (Phase 65, roadmap §7.1).
///
/// The interface every rendering backend implements. OpenGLBackend wraps the
/// existing Qt OpenGL path; VulkanBackend (behind HZ_ENABLE_VULKAN, staged
/// bring-up) targets the Vulkan migration. Callers own handle lifecycles:
/// every create must be paired with a destroy before the backend dies.
class RenderBackend {
public:
    virtual ~RenderBackend() = default;

    /// Human-readable backend identification ("OpenGL 3.3", "Vulkan — RTX …").
    virtual std::string name() const = 0;

    // -- Resources -------------------------------------------------------------

    virtual BufferHandle createBuffer(BufferUsage usage, const void* data, size_t size) = 0;

    /// Overwrite the first @p size bytes. @p size must not exceed the buffer's
    /// creation size — oversized updates are ignored on every backend (grow by
    /// destroying and recreating instead).
    virtual void updateBuffer(BufferHandle handle, const void* data, size_t size) = 0;
    virtual void destroyBuffer(BufferHandle handle) = 0;

    /// RGBA8 texture from tightly packed pixel data.
    virtual TextureHandle createTexture(int width, int height, const void* rgbaPixels) = 0;
    virtual void destroyTexture(TextureHandle handle) = 0;

    /// Compile + link a shader program from backend-native source (GLSL for
    /// the OpenGL backend). Invalid handle on compile failure.
    virtual ShaderHandle createShader(const std::string& vertexSrc,
                                      const std::string& fragmentSrc) = 0;
    virtual void destroyShader(ShaderHandle handle) = 0;

    // -- Frame -----------------------------------------------------------------

    virtual void beginPass(const RenderPassDesc& desc) = 0;
    virtual void draw(const DrawCall& call) = 0;
    virtual void endPass() = 0;

    // -- Compute (optional; Phase 66) -------------------------------------------

    /// Dispatch a compute shader from backend-native source. Returns false
    /// when the backend (or context version) has no compute support.
    virtual bool submitCompute(const std::string& computeSrc, uint32_t groupsX, uint32_t groupsY,
                               uint32_t groupsZ) {
        (void)computeSrc;
        (void)groupsX;
        (void)groupsY;
        (void)groupsZ;
        return false;
    }
};

}  // namespace hz::render
