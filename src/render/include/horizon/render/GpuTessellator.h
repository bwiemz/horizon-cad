#pragma once

// GPU NURBS evaluation (Phase 66) — needs the Vulkan backend plus the
// build-time SPIR-V compiler (both provided by the Vulkan SDK).
#ifdef HZ_ENABLE_GPU_TESSELLATION

#include <cstdint>
#include <vector>

#include "horizon/math/Vec3.h"

namespace hz::geo {
class NurbsSurface;
}  // namespace hz::geo

namespace hz::render {

class VulkanBackend;

/// GPU-accelerated NURBS surface evaluation (roadmap §7.2: compute shaders
/// only — no hardware tessellation stages, so the path translates cleanly
/// through MoltenVK). Uploads the control net + knot vectors as storage
/// buffers and evaluates a parameter grid with a Cox–de Boor compute kernel.
class GpuTessellator {
public:
    /// @p backend must be available (isAvailable()) and outlive this object.
    explicit GpuTessellator(VulkanBackend& backend);

    /// Evaluate an @p nx × @p ny parameter grid over the surface's full
    /// domain on the GPU. Returns row-major points (ny rows × nx columns),
    /// or an empty vector on failure (no device, degree > 7, dispatch error).
    std::vector<math::Vec3> evaluateGrid(const geo::NurbsSurface& surface, uint32_t nx,
                                         uint32_t ny);

private:
    VulkanBackend& m_backend;
};

}  // namespace hz::render

#endif  // HZ_ENABLE_GPU_TESSELLATION
