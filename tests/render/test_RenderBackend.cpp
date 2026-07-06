#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "horizon/render/RenderBackend.h"

using namespace hz::render;

namespace {

/// Records every RAL call — validates the interface contract without a GPU.
class RecordingBackend : public RenderBackend {
public:
    std::vector<std::string> log;
    uint32_t next = 1;
    int livePasses = 0;

    std::string name() const override { return "Recording"; }

    BufferHandle createBuffer(BufferUsage, const void*, size_t size) override {
        if (size == 0) return {};
        log.push_back("createBuffer");
        return {next++};
    }
    void updateBuffer(BufferHandle h, const void*, size_t) override {
        if (h.isValid()) log.push_back("updateBuffer");
    }
    void destroyBuffer(BufferHandle h) override {
        if (h.isValid()) log.push_back("destroyBuffer");
    }
    TextureHandle createTexture(int w, int h, const void*) override {
        if (w <= 0 || h <= 0) return {};
        log.push_back("createTexture");
        return {next++};
    }
    void destroyTexture(TextureHandle h) override {
        if (h.isValid()) log.push_back("destroyTexture");
    }
    ShaderHandle createShader(const std::string&, const std::string&) override {
        log.push_back("createShader");
        return {next++};
    }
    void destroyShader(ShaderHandle h) override {
        if (h.isValid()) log.push_back("destroyShader");
    }
    void beginPass(const RenderPassDesc&) override {
        ++livePasses;
        log.push_back("beginPass");
    }
    void draw(const DrawCall&) override { log.push_back("draw"); }
    void endPass() override {
        --livePasses;
        log.push_back("endPass");
    }
};

}  // namespace

TEST(RenderBackendTest, HandlesDefaultToInvalid) {
    EXPECT_FALSE(BufferHandle{}.isValid());
    EXPECT_FALSE(TextureHandle{}.isValid());
    EXPECT_FALSE(ShaderHandle{}.isValid());
}

TEST(RenderBackendTest, VertexLayoutStride) {
    VertexLayout layout;
    layout.attributeComponents[0] = 3;  // position
    layout.attributeComponents[1] = 3;  // normal
    layout.attributeCount = 2;
    EXPECT_EQ(layout.stride(), static_cast<int>(6 * sizeof(float)));
}

TEST(RenderBackendTest, TypicalFrameSequence) {
    RecordingBackend backend;

    const float vertices[] = {0, 0, 0, 1, 0, 0, 0, 1, 0};
    const BufferHandle vb = backend.createBuffer(BufferUsage::Vertex, vertices, sizeof(vertices));
    const ShaderHandle sh = backend.createShader("vs", "fs");
    ASSERT_TRUE(vb.isValid());
    ASSERT_TRUE(sh.isValid());

    RenderPassDesc pass;
    backend.beginPass(pass);
    DrawCall call;
    call.shader = sh;
    call.vertexBuffer = vb;
    call.layout.attributeComponents[0] = 3;
    call.layout.attributeCount = 1;
    call.elementCount = 3;
    backend.draw(call);
    backend.endPass();
    EXPECT_EQ(backend.livePasses, 0);

    backend.destroyShader(sh);
    backend.destroyBuffer(vb);

    const std::vector<std::string> expected = {"createBuffer", "createShader", "beginPass",
                                               "draw",         "endPass",      "destroyShader",
                                               "destroyBuffer"};
    EXPECT_EQ(backend.log, expected);
}

TEST(RenderBackendTest, ZeroSizeBufferIsInvalid) {
    RecordingBackend backend;
    EXPECT_FALSE(backend.createBuffer(BufferUsage::Vertex, nullptr, 0).isValid());
}

TEST(RenderBackendTest, ComputeDefaultsToUnsupported) {
    RecordingBackend backend;
    EXPECT_FALSE(backend.submitCompute("shader", 1, 1, 1));
}
