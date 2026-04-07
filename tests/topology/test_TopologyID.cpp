#include "horizon/topology/TopologyID.h"

#include <gtest/gtest.h>

using hz::topo::TopologyID;

TEST(TopologyIDTest, DefaultIsInvalid) {
    TopologyID id;
    EXPECT_FALSE(id.isValid());
    EXPECT_TRUE(id.tag().empty());
}

TEST(TopologyIDTest, CreateFromComponents) {
    auto id = TopologyID::make("box", "top");
    EXPECT_TRUE(id.isValid());
    EXPECT_EQ(id.tag(), "box/top");
}

TEST(TopologyIDTest, SameInputsSameID) {
    auto a = TopologyID::make("box", "top");
    auto b = TopologyID::make("box", "top");
    EXPECT_EQ(a, b);
}

TEST(TopologyIDTest, DifferentInputsDifferentID) {
    auto a = TopologyID::make("box", "top");
    auto b = TopologyID::make("box", "bottom");
    EXPECT_NE(a, b);
}

TEST(TopologyIDTest, ChildInheritsParent) {
    auto parent = TopologyID::make("box", "top");
    auto child = parent.child("split", 0);
    EXPECT_TRUE(child.isValid());
    EXPECT_EQ(child.tag(), "box/top/split:0");
    EXPECT_TRUE(child.isDescendantOf(parent));
}

TEST(TopologyIDTest, GrandchildIsDescendant) {
    auto grandparent = TopologyID::make("box", "top");
    auto parent = grandparent.child("split", 0);
    auto grandchild = parent.child("fillet", 1);
    EXPECT_TRUE(grandchild.isDescendantOf(grandparent));
    EXPECT_TRUE(grandchild.isDescendantOf(parent));
    EXPECT_EQ(grandchild.tag(), "box/top/split:0/fillet:1");
}

TEST(TopologyIDTest, NonRelatedIsNotDescendant) {
    auto a = TopologyID::make("box", "top");
    auto b = TopologyID::make("cylinder", "side");
    EXPECT_FALSE(a.isDescendantOf(b));
    EXPECT_FALSE(b.isDescendantOf(a));
}

TEST(TopologyIDTest, SelfIsNotDescendant) {
    auto id = TopologyID::make("box", "top");
    EXPECT_FALSE(id.isDescendantOf(id));
}

TEST(TopologyIDTest, ResolveFindsBestMatch) {
    auto target = TopologyID::make("box", "top");
    auto exact = TopologyID::make("box", "top");
    auto child = target.child("split", 0);
    auto unrelated = TopologyID::make("cyl", "side");

    std::vector<TopologyID> candidates = {unrelated, child, exact};
    auto result = TopologyID::resolve(target, candidates);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(*result, exact);
}

TEST(TopologyIDTest, ResolveFallsBackToDescendant) {
    auto target = TopologyID::make("box", "top");
    auto child = target.child("split", 0);
    auto unrelated = TopologyID::make("cyl", "side");

    std::vector<TopologyID> candidates = {unrelated, child};
    auto result = TopologyID::resolve(target, candidates);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(*result, child);
}

TEST(TopologyIDTest, ResolveFailsWhenNoMatch) {
    auto target = TopologyID::make("box", "top");
    auto unrelated = TopologyID::make("cyl", "side");

    std::vector<TopologyID> candidates = {unrelated};
    auto result = TopologyID::resolve(target, candidates);
    EXPECT_FALSE(result.has_value());
}
