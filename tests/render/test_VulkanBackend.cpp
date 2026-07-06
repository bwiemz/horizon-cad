// Vulkan bring-up tests — compiled only when the SDK is present
// (HZ_ENABLE_VULKAN) and skipped at runtime when no Vulkan device exists.

#ifdef HZ_ENABLE_VULKAN

#include <gtest/gtest.h>

#include "horizon/render/VulkanBackend.h"

using namespace hz::render;

TEST(VulkanBackendTest, DeviceBringUp) {
    VulkanBackend backend;
    if (!backend.isAvailable()) {
        GTEST_SKIP() << "No Vulkan device available";
    }
    // "Vulkan <major>.<minor> — <device name>"
    const std::string name = backend.name();
    EXPECT_NE(name.find("Vulkan"), std::string::npos);
    EXPECT_GT(name.size(), std::string("Vulkan x.y — ").size());
}

TEST(VulkanBackendTest, DeviceMemoryBufferLifecycle) {
    VulkanBackend backend;
    if (!backend.isAvailable()) {
        GTEST_SKIP() << "No Vulkan device available";
    }

    const float data[] = {1.0f, 2.0f, 3.0f, 4.0f};
    const BufferHandle vb = backend.createBuffer(BufferUsage::Vertex, data, sizeof(data));
    ASSERT_TRUE(vb.isValid()) << "GPU buffer allocation failed";

    const float updated[] = {5.0f, 6.0f, 7.0f, 8.0f};
    backend.updateBuffer(vb, updated, sizeof(updated));
    backend.destroyBuffer(vb);

    const BufferHandle ib = backend.createBuffer(BufferUsage::Index, nullptr, 64);
    EXPECT_TRUE(ib.isValid());
    backend.destroyBuffer(ib);

    EXPECT_FALSE(backend.createBuffer(BufferUsage::Vertex, nullptr, 0).isValid());
}

TEST(VulkanBackendTest, StagedFeaturesReportUnavailable) {
    VulkanBackend backend;
    if (!backend.isAvailable()) {
        GTEST_SKIP() << "No Vulkan device available";
    }
    EXPECT_FALSE(backend.createShader("vs", "fs").isValid());
    EXPECT_FALSE(backend.createTexture(2, 2, nullptr).isValid());
    EXPECT_FALSE(backend.submitCompute("cs", 1, 1, 1));
}

#endif  // HZ_ENABLE_VULKAN
