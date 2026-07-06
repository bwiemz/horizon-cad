#pragma once

// Compiled only when the Vulkan SDK is found (HZ_ENABLE_VULKAN) — quiet
// detection mirrors HZ_ENABLE_SCRIPTING so default builds never require it.
#ifdef HZ_ENABLE_VULKAN

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include "horizon/render/RenderBackend.h"

// Forward-declare Vulkan handles to keep vulkan.h out of the public header.
// (Dispatchable and — on 64-bit targets, the only ones Horizon builds —
// non-dispatchable handles are pointers to these tag types.)
struct VkInstance_T;
struct VkPhysicalDevice_T;
struct VkDevice_T;
struct VkQueue_T;
struct VkBuffer_T;
struct VkDeviceMemory_T;

namespace hz::render {

/// Vulkan backend, staged bring-up (Phase 65, roadmap §7.1 migration
/// strategy: "build alongside OpenGL, feature-flag at startup").
///
/// Implemented today: instance + physical-device selection + logical device
/// with a graphics queue, capability reporting, and device-memory buffer
/// allocation (create/update/destroy through host-visible memory).
///
/// Staged (returns invalid handles / no-ops until the SPIR-V pipeline lands):
/// textures, shaders, render passes, draws, compute. The OpenGL backend
/// remains the production path; select backends at startup via
/// RenderBackendFactory-style wiring once draws are implemented.
class VulkanBackend : public RenderBackend {
public:
    VulkanBackend();
    ~VulkanBackend() override;

    VulkanBackend(const VulkanBackend&) = delete;
    VulkanBackend& operator=(const VulkanBackend&) = delete;

    /// True when an instance + logical device came up successfully.
    bool isAvailable() const { return m_device != nullptr; }

    /// "Vulkan <apiVersion> — <deviceName>" when available.
    std::string name() const override;

    BufferHandle createBuffer(BufferUsage usage, const void* data, size_t size) override;
    void updateBuffer(BufferHandle handle, const void* data, size_t size) override;
    void destroyBuffer(BufferHandle handle) override;

    /// One-shot compute dispatch from SPIR-V words (Phase 66). The storage
    /// buffers bind in order to descriptor bindings 0..N-1 of set 0; entry
    /// point is "main". Blocks until the GPU finishes. Returns false on any
    /// setup or submission failure.
    bool runComputeSpirv(const uint32_t* spirv, size_t wordCount,
                         const std::vector<BufferHandle>& storageBuffers, uint32_t groupsX,
                         uint32_t groupsY, uint32_t groupsZ);

    /// Read back the first @p size bytes of a buffer (host-visible memory).
    bool readBuffer(BufferHandle handle, void* out, size_t size);

    // -- Staged: not yet implemented (invalid handle / no-op) -------------------
    TextureHandle createTexture(int width, int height, const void* rgbaPixels) override;
    void destroyTexture(TextureHandle handle) override;
    ShaderHandle createShader(const std::string& vertexSrc,
                              const std::string& fragmentSrc) override;
    void destroyShader(ShaderHandle handle) override;
    void beginPass(const RenderPassDesc& desc) override;
    void draw(const DrawCall& call) override;
    void endPass() override;

private:
    struct BufferRecord {
        VkBuffer_T* buffer = nullptr;
        VkDeviceMemory_T* memory = nullptr;
        size_t size = 0;
    };

    VkInstance_T* m_instance = nullptr;
    VkPhysicalDevice_T* m_physicalDevice = nullptr;
    VkDevice_T* m_device = nullptr;
    VkQueue_T* m_queue = nullptr;
    uint32_t m_queueFamily = 0;
    std::string m_deviceName;
    uint32_t m_apiVersion = 0;
    /// propertyFlags per memory-type index — memory types are selected per
    /// allocation against the buffer's memoryTypeBits requirement.
    std::vector<uint32_t> m_memoryTypeFlags;

    uint32_t m_nextHandle = 1;
    std::unordered_map<uint32_t, BufferRecord> m_buffers;
};

}  // namespace hz::render

#endif  // HZ_ENABLE_VULKAN
