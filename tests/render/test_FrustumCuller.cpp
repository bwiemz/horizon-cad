#include <gtest/gtest.h>

#include <cmath>
#include <memory>

#include "horizon/render/FrustumCuller.h"
#include "horizon/render/InstanceBatcher.h"
#include "horizon/render/SceneGraph.h"

using hz::math::BoundingBox;
using hz::math::Mat4;
using hz::math::Vec3;
using hz::render::FrustumCuller;
using hz::render::InstanceBatcher;
using hz::render::MeshData;
using hz::render::SceneGraph;
using hz::render::SceneNode;

namespace {

std::unique_ptr<MeshData> unitTriangle() {
    auto mesh = std::make_unique<MeshData>();
    mesh->positions = {-0.5f, -0.5f, 0.0f, 0.5f, -0.5f, 0.0f, 0.0f, 0.5f, 0.0f};
    mesh->normals = {0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f};
    mesh->indices = {0, 1, 2};
    return mesh;
}

std::shared_ptr<SceneNode> nodeAt(const std::string& name, const Vec3& pos) {
    auto node = std::make_shared<SceneNode>(name);
    node->setMesh(unitTriangle());
    node->setLocalTransform(Mat4::translation(pos));
    return node;
}

/// Camera at (0, 0, 10) looking down -Z at the origin, 60° vertical FOV.
Mat4 lookAtOriginViewProj() {
    const Mat4 view = Mat4::lookAt(Vec3(0, 0, 10), Vec3(0, 0, 0), Vec3(0, 1, 0));
    const Mat4 proj = Mat4::perspective(60.0 * 3.14159265358979 / 180.0, 1.0, 0.1, 100.0);
    return proj * view;
}

}  // namespace

TEST(FrustumCullerTest, BoxAtLookTargetIsInside) {
    const auto planes = FrustumCuller::extractPlanes(lookAtOriginViewProj());
    const BoundingBox box(Vec3(-1, -1, -1), Vec3(1, 1, 1));
    EXPECT_TRUE(FrustumCuller::intersects(box, planes));
}

TEST(FrustumCullerTest, BoxBehindCameraIsOutside) {
    const auto planes = FrustumCuller::extractPlanes(lookAtOriginViewProj());
    const BoundingBox behind(Vec3(-1, -1, 19), Vec3(1, 1, 21));
    EXPECT_FALSE(FrustumCuller::intersects(behind, planes));
}

TEST(FrustumCullerTest, BoxFarOffAxisIsOutsideButNearOffAxisIsNot) {
    const auto planes = FrustumCuller::extractPlanes(lookAtOriginViewProj());
    // At z = 0 the camera is 10 away; 60° FOV → half-height ≈ 5.77.
    const BoundingBox wayLeft(Vec3(-51, -1, -1), Vec3(-49, 1, 1));
    EXPECT_FALSE(FrustumCuller::intersects(wayLeft, planes));
    const BoundingBox nearLeft(Vec3(-5, -1, -1), Vec3(-3, 1, 1));
    EXPECT_TRUE(FrustumCuller::intersects(nearLeft, planes));
}

TEST(FrustumCullerTest, BoxPastFarPlaneIsOutside) {
    const auto planes = FrustumCuller::extractPlanes(lookAtOriginViewProj());
    // Far plane is 100 in front of the camera at z = 10 → z = -90.
    const BoundingBox pastFar(Vec3(-1, -1, -130), Vec3(1, 1, -110));
    EXPECT_FALSE(FrustumCuller::intersects(pastFar, planes));
}

TEST(FrustumCullerTest, StraddlingBoxIsKept) {
    const auto planes = FrustumCuller::extractPlanes(lookAtOriginViewProj());
    // Half in front of the camera, half behind — conservative culling keeps it.
    const BoundingBox straddle(Vec3(-1, -1, 5), Vec3(1, 1, 15));
    EXPECT_TRUE(FrustumCuller::intersects(straddle, planes));
}

TEST(FrustumCullerTest, MeshBoundsCoverAllVertices) {
    const auto mesh = unitTriangle();
    const BoundingBox box = FrustumCuller::meshBounds(*mesh);
    ASSERT_TRUE(box.isValid());
    EXPECT_NEAR(box.min().x, -0.5, 1e-12);
    EXPECT_NEAR(box.max().x, 0.5, 1e-12);
    EXPECT_NEAR(box.min().y, -0.5, 1e-12);
    EXPECT_NEAR(box.max().y, 0.5, 1e-12);
}

TEST(FrustumCullerTest, CullBatchKeepsArraysAligned) {
    SceneGraph scene;
    scene.addNode(nodeAt("visibleA", Vec3(0, 0, 0)));
    scene.addNode(nodeAt("gone", Vec3(200, 0, 0)));
    scene.addNode(nodeAt("visibleB", Vec3(2, 0, 0)));

    auto batches = InstanceBatcher::batch(scene);
    ASSERT_EQ(batches.size(), 1u);
    const BoundingBox local = FrustumCuller::meshBounds(*batches[0].mesh);
    const auto planes = FrustumCuller::extractPlanes(lookAtOriginViewProj());

    const size_t culled = FrustumCuller::cullBatch(batches[0], local, planes);
    EXPECT_EQ(culled, 1u);
    ASSERT_EQ(batches[0].nodes.size(), 2u);
    ASSERT_EQ(batches[0].transforms.size(), 2u);
    ASSERT_EQ(batches[0].materials.size(), 2u);
    EXPECT_EQ(batches[0].nodes[0]->name(), "visibleA");
    EXPECT_EQ(batches[0].nodes[1]->name(), "visibleB");
    // The survivor transforms still match their nodes' world transforms.
    const Vec3 b = batches[0].transforms[1].transformPoint(Vec3(0, 0, 0));
    EXPECT_NEAR(b.x, 2.0, 1e-12);
}

TEST(FrustumCullerTest, RotatedInstanceUsesWorldSpaceBox) {
    SceneGraph scene;
    auto node = std::make_shared<SceneNode>("spun");
    node->setMesh(unitTriangle());
    // 90° about Y then push left to the frustum edge: the world-space box
    // must be rebuilt from transformed corners, not the local box translated.
    node->setLocalTransform(Mat4::translation(Vec3(-5.5, 0, 0)) *
                            Mat4::rotationY(3.14159265358979 / 2.0));
    scene.addNode(node);

    auto batches = InstanceBatcher::batch(scene);
    ASSERT_EQ(batches.size(), 1u);
    const BoundingBox local = FrustumCuller::meshBounds(*batches[0].mesh);
    const auto planes = FrustumCuller::extractPlanes(lookAtOriginViewProj());
    FrustumCuller::cullBatch(batches[0], local, planes);
    EXPECT_EQ(batches[0].nodes.size(), 1u);  // still visible near the edge
}

TEST(FrustumCullerTest, TenThousandPartCullingIsCorrect) {
    SceneGraph scene;
    // 100×100 grid, 4-unit pitch, centered on the origin at z = 0.
    for (int i = 0; i < 100; ++i) {
        for (int j = 0; j < 100; ++j) {
            scene.addNode(
                nodeAt("p" + std::to_string(i * 100 + j), Vec3((i - 50) * 4.0, (j - 50) * 4.0, 0)));
        }
    }
    auto batches = InstanceBatcher::batch(scene);
    ASSERT_EQ(batches.size(), 1u);
    ASSERT_EQ(batches[0].transforms.size(), 10000u);

    const BoundingBox local = FrustumCuller::meshBounds(*batches[0].mesh);
    const auto planes = FrustumCuller::extractPlanes(lookAtOriginViewProj());
    const size_t culled = FrustumCuller::cullBatch(batches[0], local, planes);

    // Half-extent of the visible square at z = 0 is ~5.77 → only the parts
    // near the center survive; the vast majority of 10k instances drop.
    EXPECT_GT(culled, 9900u);
    EXPECT_GT(batches[0].transforms.size(), 0u);
    for (const auto* node : batches[0].nodes) {
        const Vec3 p = node->worldTransform().transformPoint(Vec3(0, 0, 0));
        EXPECT_LT(std::abs(p.x), 7.0);
        EXPECT_LT(std::abs(p.y), 7.0);
    }
}
