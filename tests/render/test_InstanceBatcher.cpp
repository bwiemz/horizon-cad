#include <gtest/gtest.h>

#include <memory>

#include "horizon/render/InstanceBatcher.h"
#include "horizon/render/SceneGraph.h"

using hz::math::Mat4;
using hz::math::Vec3;
using hz::render::InstanceBatcher;
using hz::render::Material;
using hz::render::MeshData;
using hz::render::SceneGraph;
using hz::render::SceneNode;

namespace {

std::unique_ptr<MeshData> makeTriangle(float scale) {
    auto mesh = std::make_unique<MeshData>();
    mesh->positions = {0.0f, 0.0f, 0.0f, scale, 0.0f, 0.0f, 0.0f, scale, 0.0f};
    mesh->normals = {0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f};
    mesh->indices = {0, 1, 2};
    return mesh;
}

std::shared_ptr<SceneNode> makeNode(const std::string& name, float scale,
                                    const Vec3& translation = Vec3(0, 0, 0)) {
    auto node = std::make_shared<SceneNode>(name);
    node->setMesh(makeTriangle(scale));
    node->setLocalTransform(Mat4::translation(translation));
    return node;
}

}  // namespace

TEST(InstanceBatcherTest, IdenticalMeshesShareOneBatch) {
    SceneGraph scene;
    scene.addNode(makeNode("boltA", 1.0f, {0, 0, 0}));
    scene.addNode(makeNode("boltB", 1.0f, {5, 0, 0}));
    scene.addNode(makeNode("boltC", 1.0f, {0, 5, 0}));

    const auto batches = InstanceBatcher::batch(scene);
    ASSERT_EQ(batches.size(), 1u);
    EXPECT_EQ(batches[0].transforms.size(), 3u);
    EXPECT_EQ(batches[0].materials.size(), 3u);
    EXPECT_EQ(batches[0].nodes.size(), 3u);
}

TEST(InstanceBatcherTest, DistinctMeshesGetDistinctBatches) {
    SceneGraph scene;
    scene.addNode(makeNode("small", 1.0f));
    scene.addNode(makeNode("large", 2.0f));
    scene.addNode(makeNode("small2", 1.0f));

    const auto batches = InstanceBatcher::batch(scene);
    ASSERT_EQ(batches.size(), 2u);
    // First-seen order: "small"'s geometry batch first, then "large"'s.
    EXPECT_EQ(batches[0].transforms.size(), 2u);
    EXPECT_EQ(batches[1].transforms.size(), 1u);
    EXPECT_NE(batches[0].contentHash, batches[1].contentHash);
}

TEST(InstanceBatcherTest, TransformsCaptureWorldNotLocal) {
    SceneGraph scene;
    auto parent = std::make_shared<SceneNode>("assembly");
    parent->setLocalTransform(Mat4::translation(Vec3(10, 0, 0)));
    auto child = makeNode("part", 1.0f, {0, 2, 0});
    parent->addChild(child);
    scene.addNode(parent);

    const auto batches = InstanceBatcher::batch(scene);
    ASSERT_EQ(batches.size(), 1u);
    ASSERT_EQ(batches[0].transforms.size(), 1u);
    const Vec3 origin = batches[0].transforms[0].transformPoint(Vec3(0, 0, 0));
    EXPECT_NEAR(origin.x, 10.0, 1e-12);
    EXPECT_NEAR(origin.y, 2.0, 1e-12);
    EXPECT_NEAR(origin.z, 0.0, 1e-12);
}

TEST(InstanceBatcherTest, PerInstanceMaterialsArePreserved) {
    SceneGraph scene;
    auto red = makeNode("red", 1.0f);
    Material redMat;
    redMat.color = Vec3(1, 0, 0);
    red->setMaterial(redMat);
    auto blue = makeNode("blue", 1.0f);
    Material blueMat;
    blueMat.color = Vec3(0, 0, 1);
    blue->setMaterial(blueMat);
    scene.addNode(red);
    scene.addNode(blue);

    const auto batches = InstanceBatcher::batch(scene);
    ASSERT_EQ(batches.size(), 1u);
    ASSERT_EQ(batches[0].materials.size(), 2u);
    EXPECT_NEAR(batches[0].materials[0].color.x, 1.0, 1e-12);
    EXPECT_NEAR(batches[0].materials[1].color.z, 1.0, 1e-12);
}

TEST(InstanceBatcherTest, InvisibleAndEmptyNodesAreExcluded) {
    SceneGraph scene;
    scene.addNode(makeNode("shown", 1.0f));
    auto hidden = makeNode("hidden", 1.0f);
    hidden->setVisible(false);
    scene.addNode(hidden);
    auto meshless = std::make_shared<SceneNode>("group");
    scene.addNode(meshless);
    auto empty = std::make_shared<SceneNode>("empty");
    empty->setMesh(std::make_unique<MeshData>());
    scene.addNode(empty);

    const auto batches = InstanceBatcher::batch(scene);
    ASSERT_EQ(batches.size(), 1u);
    EXPECT_EQ(batches[0].transforms.size(), 1u);
    EXPECT_EQ(batches[0].nodes[0]->name(), "shown");
}

TEST(InstanceBatcherTest, HidingAParentHidesItsSubtree) {
    SceneGraph scene;
    auto parent = std::make_shared<SceneNode>("assembly");
    parent->setVisible(false);
    parent->addChild(makeNode("child", 1.0f));
    scene.addNode(parent);

    EXPECT_TRUE(InstanceBatcher::batch(scene).empty());
}

TEST(InstanceBatcherTest, SameSizeDifferentGeometryDoesNotMerge) {
    // Two meshes with byte-identical buffer *sizes* but different vertex data
    // must never share a batch — otherwise a hash collision (or the old
    // size-only guard) would render one mesh with the other's geometry.
    SceneGraph scene;
    scene.addNode(makeNode("a", 1.0f));
    scene.addNode(makeNode("b", 2.0f));  // same 3-vertex layout, different coords

    const auto batches = InstanceBatcher::batch(scene);
    ASSERT_EQ(batches.size(), 2u);
    EXPECT_EQ(batches[0].transforms.size(), 1u);
    EXPECT_EQ(batches[1].transforms.size(), 1u);
    // Each batch's shared mesh is the geometry it actually represents.
    EXPECT_NE(batches[0].mesh->positions, batches[1].mesh->positions);
}

TEST(InstanceBatcherTest, ContentHashIsOrderStable) {
    const auto meshA = makeTriangle(1.0f);
    const auto meshB = makeTriangle(1.0f);
    const auto meshC = makeTriangle(3.0f);
    EXPECT_EQ(InstanceBatcher::meshContentHash(*meshA), InstanceBatcher::meshContentHash(*meshB));
    EXPECT_NE(InstanceBatcher::meshContentHash(*meshA), InstanceBatcher::meshContentHash(*meshC));
}

TEST(InstanceBatcherTest, ThousandInstancesOneBatch) {
    SceneGraph scene;
    for (int i = 0; i < 1000; ++i) {
        scene.addNode(makeNode("bolt" + std::to_string(i), 1.0f,
                               Vec3(static_cast<double>(i % 32), static_cast<double>(i / 32), 0)));
    }
    const auto batches = InstanceBatcher::batch(scene);
    ASSERT_EQ(batches.size(), 1u);
    EXPECT_EQ(batches[0].transforms.size(), 1000u);
    // Instance order follows traversal order — stable for GPU re-upload diffing.
    EXPECT_EQ(batches[0].nodes.front()->name(), "bolt0");
    EXPECT_EQ(batches[0].nodes.back()->name(), "bolt999");
}
