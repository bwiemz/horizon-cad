// GPU NURBS evaluation vs the CPU reference (Phase 66) — compiled only when
// the Vulkan SDK + glslangValidator were found, skipped without a device.

#ifdef HZ_ENABLE_GPU_TESSELLATION

#include <gtest/gtest.h>

#include <cmath>

#include "horizon/geometry/surfaces/NurbsSurface.h"
#include "horizon/math/Vec3.h"
#include "horizon/render/GpuTessellator.h"
#include "horizon/render/VulkanBackend.h"

using hz::geo::NurbsSurface;
using hz::math::Vec3;
using hz::render::GpuTessellator;
using hz::render::VulkanBackend;

namespace {

/// Compare a GPU grid against CPU NurbsSurface::evaluate at the same params.
/// GPU evaluates in float; CPU in double — tolerance covers the gap.
void expectGridMatchesCpu(const NurbsSurface& surface, uint32_t nx, uint32_t ny, double tol) {
    VulkanBackend backend;
    if (!backend.isAvailable()) {
        GTEST_SKIP() << "No Vulkan device available";
    }
    GpuTessellator tess(backend);
    const auto gpu = tess.evaluateGrid(surface, nx, ny);
    ASSERT_EQ(gpu.size(), static_cast<size_t>(nx) * ny) << "GPU evaluation failed";

    double maxErr = 0.0;
    for (uint32_t j = 0; j < ny; ++j) {
        const double v = surface.vMin() + (surface.vMax() - surface.vMin()) * j / double(ny - 1);
        for (uint32_t i = 0; i < nx; ++i) {
            const double u =
                surface.uMin() + (surface.uMax() - surface.uMin()) * i / double(nx - 1);
            const Vec3 cpu = surface.evaluate(u, v);
            maxErr = std::max(maxErr, cpu.distanceTo(gpu[j * nx + i]));
        }
    }
    EXPECT_LT(maxErr, tol) << "GPU/CPU divergence over " << nx << "x" << ny << " grid";
}

}  // namespace

TEST(GpuTessellatorTest, PlaneMatchesCpu) {
    const NurbsSurface plane =
        NurbsSurface::makePlane(Vec3(0, 0, 0), Vec3(1, 0, 0), Vec3(0, 1, 0), 10.0, 5.0);
    expectGridMatchesCpu(plane, 16, 16, 1e-4);
}

TEST(GpuTessellatorTest, CylinderMatchesCpu) {
    const NurbsSurface cyl = NurbsSurface::makeCylinder(Vec3(0, 0, 0), Vec3(0, 0, 1), 5.0, 12.0);
    expectGridMatchesCpu(cyl, 33, 17, 1e-3);
}

TEST(GpuTessellatorTest, SphereMatchesCpu) {
    const NurbsSurface sphere = NurbsSurface::makeSphere(Vec3(1, 2, 3), 4.0);
    expectGridMatchesCpu(sphere, 32, 32, 1e-3);
}

TEST(GpuTessellatorTest, LargeGridMatchesCpuOnSamples) {
    VulkanBackend backend;
    if (!backend.isAvailable()) {
        GTEST_SKIP() << "No Vulkan device available";
    }
    const NurbsSurface torus = NurbsSurface::makeTorus(Vec3(0, 0, 0), Vec3(0, 0, 1), 10.0, 2.0);
    GpuTessellator tess(backend);
    constexpr uint32_t kN = 256;
    const auto gpu = tess.evaluateGrid(torus, kN, kN);
    ASSERT_EQ(gpu.size(), size_t{kN} * kN);

    // The contract is GPU ≡ CPU (NurbsSurface::evaluate), not the analytic
    // torus — the kernel's two-pass rational scheme deviates slightly from
    // exact analytic geometry on doubly-rational surfaces, and the GPU must
    // reproduce exactly that. Compare a subsample against the CPU.
    double maxErr = 0.0;
    for (uint32_t j = 0; j < kN; j += 16) {
        const double v = torus.vMin() + (torus.vMax() - torus.vMin()) * j / double(kN - 1);
        for (uint32_t i = 0; i < kN; i += 16) {
            const double u = torus.uMin() + (torus.uMax() - torus.uMin()) * i / double(kN - 1);
            maxErr = std::max(maxErr, torus.evaluate(u, v).distanceTo(gpu[j * kN + i]));
        }
    }
    EXPECT_LT(maxErr, 1e-3);
}

TEST(GpuTessellatorTest, RejectsUnsupportedInput) {
    VulkanBackend backend;
    if (!backend.isAvailable()) {
        GTEST_SKIP() << "No Vulkan device available";
    }
    GpuTessellator tess(backend);
    const NurbsSurface plane =
        NurbsSurface::makePlane(Vec3(0, 0, 0), Vec3(1, 0, 0), Vec3(0, 1, 0), 1.0, 1.0);
    EXPECT_TRUE(tess.evaluateGrid(plane, 1, 16).empty());  // grid too small
    EXPECT_TRUE(tess.evaluateGrid(plane, 16, 1).empty());
}

#endif  // HZ_ENABLE_GPU_TESSELLATION
