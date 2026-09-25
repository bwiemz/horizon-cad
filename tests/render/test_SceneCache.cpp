// The GL mesh cache lets go of what left the scene (Phase 113). A scene is
// rebuilt with new node IDs on every model change, and a cache that never
// dropped the old ones kept every mesh the model ever had.

#include <gtest/gtest.h>

#include <memory>
#include <unordered_map>

#include "horizon/render/SceneGraph.h"

using hz::render::SceneGraph;
using hz::render::SceneNode;

TEST(SceneCacheTest, NodeIdsReachEveryDepth) {
    SceneGraph scene;
    auto root = std::make_shared<SceneNode>("root");
    auto child = std::make_shared<SceneNode>("child");
    auto grandchild = std::make_shared<SceneNode>("grandchild");
    child->addChild(grandchild);
    root->addChild(child);
    scene.addNode(root);
    auto other = std::make_shared<SceneNode>("other");
    scene.addNode(other);

    const auto ids = scene.nodeIds();
    EXPECT_EQ(ids.size(), 4u);
    for (const auto& node : {root, child, grandchild, other}) {
        EXPECT_EQ(ids.count(node->id()), 1u) << node->name();
    }
}

TEST(SceneCacheTest, EntriesForNodesThatLeftTheSceneAreDropped) {
    SceneGraph scene;
    auto kept = std::make_shared<SceneNode>("kept");
    scene.addNode(kept);
    std::unordered_map<uint32_t, int> cache = {{kept->id(), 1}, {kept->id() + 1000, 2}};

    // A rebuild: everything but `kept` is gone.
    hz::render::eraseStaleEntries(cache, scene.nodeIds());
    ASSERT_EQ(cache.size(), 1u);
    EXPECT_EQ(cache.count(kept->id()), 1u);

    scene.clear();
    hz::render::eraseStaleEntries(cache, scene.nodeIds());
    EXPECT_TRUE(cache.empty());
}

// Nodes that share a mesh (an assembly's instances of one part, Phase 138)
// show it once: the scene lists it once, so the GPU holds it once. Each
// instance had a copy of its own, and the GPU one each.
TEST(SceneCacheTest, ASharedMeshIsListedOnce) {
    auto mesh = std::make_shared<const hz::render::MeshData>();
    SceneGraph scene;
    auto root = std::make_shared<SceneNode>("assembly");
    for (int i = 0; i < 3; ++i) {
        auto instance = std::make_shared<SceneNode>("bolt");
        instance->shareMesh(mesh);
        root->addChild(instance);
    }
    auto other = std::make_shared<SceneNode>("plate");
    other->setMesh(std::make_unique<hz::render::MeshData>());
    root->addChild(other);
    scene.addNode(root);

    const auto meshes = scene.meshes();
    EXPECT_EQ(meshes.size(), 2u) << "the bolt's, once, and the plate's";
    EXPECT_EQ(meshes.count(mesh.get()), 1u);
    EXPECT_EQ(mesh.use_count(), 4) << "the three nodes share it, uncopied";
}
