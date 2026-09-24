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
