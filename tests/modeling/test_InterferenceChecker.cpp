#include <gtest/gtest.h>

#include <deque>
#include <memory>

#include "horizon/modeling/InterferenceChecker.h"
#include "horizon/modeling/PrimitiveFactory.h"
#include "horizon/topology/Solid.h"

using namespace hz::model;
using hz::math::Vec3;

namespace {

// Translate every vertex of a solid (moves it in world space).
void translate(hz::topo::Solid& solid, const Vec3& d) {
    for (auto& v : const_cast<std::deque<hz::topo::Vertex>&>(solid.vertices())) {
        v.point = v.point + d;
    }
}

}  // namespace

// ---------------------------------------------------------------------------
// Pairwise interference
// ---------------------------------------------------------------------------

TEST(InterferenceCheckerTest, OverlappingBoxesInterfere) {
    auto a = PrimitiveFactory::makeBox(4, 4, 4);  // [0,4]^3
    auto b = PrimitiveFactory::makeBox(4, 4, 4);
    translate(*b, Vec3(2, 2, 2));  // [2,6]^3 — overlaps a in [2,4]^3
    EXPECT_TRUE(InterferenceChecker::solidsInterfere(*a, *b));
}

TEST(InterferenceCheckerTest, SeparatedBoxesDoNotInterfere) {
    auto a = PrimitiveFactory::makeBox(4, 4, 4);  // [0,4]^3
    auto b = PrimitiveFactory::makeBox(4, 4, 4);
    translate(*b, Vec3(10, 0, 0));  // [10,14] in x — no overlap
    EXPECT_FALSE(InterferenceChecker::solidsInterfere(*a, *b));
}

TEST(InterferenceCheckerTest, PerpendicularBarsLiftedApartDoNotInterfere) {
    // Two thin bars that would cross when viewed down Z, but one is lifted above
    // the other so the geometry never touches.
    auto a = PrimitiveFactory::makeBox(10, 1, 1);  // long in X, z in [0,1]
    auto b = PrimitiveFactory::makeBox(1, 10, 1);  // long in Y
    translate(*b, Vec3(4, -4, 5));                 // lifted to z in [5,6]
    EXPECT_FALSE(InterferenceChecker::solidsInterfere(*a, *b));
}

TEST(InterferenceCheckerTest, InterlockingBarsInterfere) {
    // Same two bars, now sharing the same Z slab and crossing in XY: an edge of
    // each pierces a face of the other — a true interference with no vertex of
    // one inside the other.
    auto a = PrimitiveFactory::makeBox(10, 1, 1);  // [0,10]x[0,1]x[0,1]
    auto b = PrimitiveFactory::makeBox(1, 10, 1);  // [0,1]x[0,10]x[0,1]
    translate(*b, Vec3(4, -4, 0));                 // crosses a around x~4, same z
    EXPECT_TRUE(InterferenceChecker::solidsInterfere(*a, *b));
}

TEST(InterferenceCheckerTest, ContainedBoxInterferes) {
    auto big = PrimitiveFactory::makeBox(10, 10, 10);  // [0,10]^3
    auto small = PrimitiveFactory::makeBox(2, 2, 2);
    translate(*small, Vec3(4, 4, 4));  // [4,6]^3 fully inside big
    EXPECT_TRUE(InterferenceChecker::solidsInterfere(*big, *small));
    EXPECT_TRUE(InterferenceChecker::solidsInterfere(*small, *big));  // symmetric
}

// ---------------------------------------------------------------------------
// Multi-body broad + narrow phase
// ---------------------------------------------------------------------------

TEST(InterferenceCheckerTest, CheckFindsOnlyRealPairs) {
    auto a = PrimitiveFactory::makeBox(4, 4, 4);  // [0,4]^3
    auto b = PrimitiveFactory::makeBox(4, 4, 4);
    translate(*b, Vec3(2, 2, 2));  // overlaps a
    auto c = PrimitiveFactory::makeBox(4, 4, 4);
    translate(*c, Vec3(100, 100, 100));  // far away

    std::vector<const hz::topo::Solid*> solids = {a.get(), b.get(), c.get()};
    auto pairs = InterferenceChecker::check(solids);

    ASSERT_EQ(pairs.size(), 1u);
    EXPECT_EQ(pairs[0].indexA, 0u);
    EXPECT_EQ(pairs[0].indexB, 1u);
    EXPECT_TRUE(pairs[0].overlapBounds.isValid());
    // Overlap region is [2,4]^3.
    EXPECT_NEAR(pairs[0].overlapBounds.min().x, 2.0, 1e-6);
    EXPECT_NEAR(pairs[0].overlapBounds.max().x, 4.0, 1e-6);
}

TEST(InterferenceCheckerTest, EmptyAndSingleInputs) {
    EXPECT_TRUE(InterferenceChecker::check({}).empty());
    auto a = PrimitiveFactory::makeBox(1, 1, 1);
    std::vector<const hz::topo::Solid*> one = {a.get()};
    EXPECT_TRUE(InterferenceChecker::check(one).empty());
}

TEST(InterferenceCheckerTest, NullSolidsIgnored) {
    auto a = PrimitiveFactory::makeBox(4, 4, 4);
    auto b = PrimitiveFactory::makeBox(4, 4, 4);
    translate(*b, Vec3(2, 2, 2));
    std::vector<const hz::topo::Solid*> solids = {a.get(), nullptr, b.get()};
    auto pairs = InterferenceChecker::check(solids);
    ASSERT_EQ(pairs.size(), 1u);
    EXPECT_EQ(pairs[0].indexA, 0u);
    EXPECT_EQ(pairs[0].indexB, 2u);
}

TEST(InterferenceCheckerTest, SolidBoundsFitBox) {
    auto box = PrimitiveFactory::makeBox(3, 5, 7);
    auto b = InterferenceChecker::solidBounds(*box);
    ASSERT_TRUE(b.isValid());
    EXPECT_NEAR(b.min().x, 0.0, 1e-9);
    EXPECT_NEAR(b.max().x, 3.0, 1e-9);
    EXPECT_NEAR(b.max().y, 5.0, 1e-9);
    EXPECT_NEAR(b.max().z, 7.0, 1e-9);
}
