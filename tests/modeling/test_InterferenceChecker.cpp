#include <gtest/gtest.h>

#include <atomic>
#include <cmath>
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

// Cancelling an interference check stops it before the next pair. The check
// on a worker ran to its end whatever Cancel said.
TEST(InterferenceCheckerTest, ACancelledCheckStopsBeforeTheNextPair) {
    auto a = PrimitiveFactory::makeBox(4, 4, 4);
    auto b = PrimitiveFactory::makeBox(4, 4, 4);
    translate(*b, Vec3(2, 2, 2));
    const std::vector<const hz::topo::Solid*> solids = {a.get(), b.get()};
    std::atomic<bool> cancelled{true};
    EXPECT_TRUE(InterferenceChecker::check(solids, &cancelled).empty());
    cancelled = false;
    EXPECT_EQ(InterferenceChecker::check(solids, &cancelled).size(), 1u);
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

// ---------------------------------------------------------------------------
// Interference volume (Phase 96): how much two solids share, not just whether.
// ---------------------------------------------------------------------------

TEST(InterferenceCheckerTest, ReportsTheSharedVolume) {
    auto a = PrimitiveFactory::makeBox(4, 4, 4);
    auto b = PrimitiveFactory::makeBox(4, 4, 4);
    translate(*b, Vec3(2, 2, 2));  // shares [2,4]^3
    auto pairs = InterferenceChecker::check({a.get(), b.get()});
    ASSERT_EQ(pairs.size(), 1u);
    EXPECT_TRUE(pairs[0].volumeResolved);
    EXPECT_NEAR(pairs[0].volume, 8.0, 1e-9);
}

TEST(InterferenceCheckerTest, ContainedSolidSharesAllOfItself) {
    auto big = PrimitiveFactory::makeBox(10, 10, 10);
    auto small = PrimitiveFactory::makeBox(2, 3, 4);
    translate(*small, Vec3(1, 1, 1));
    auto pairs = InterferenceChecker::check({big.get(), small.get()});
    ASSERT_EQ(pairs.size(), 1u);
    EXPECT_NEAR(pairs[0].volume, 24.0, 1e-9);
}

TEST(InterferenceCheckerTest, PinThroughAPlateSharesItsCrossSection) {
    // A faceted pin through a 2-thick plate shares its polygon section x 2.
    auto plate = PrimitiveFactory::makeBox(20, 20, 2);
    auto pin = PrimitiveFactory::makeCylinder(1.5, 10, 32);
    translate(*pin, Vec3(10, 10, -4));
    auto pairs = InterferenceChecker::check({plate.get(), pin.get()});
    ASSERT_EQ(pairs.size(), 1u);
    const double section = 0.5 * 32 * 1.5 * 1.5 * std::sin(2.0 * 3.14159265358979323846 / 32);
    EXPECT_NEAR(pairs[0].volume, section * 2.0, 1e-9);
}

TEST(InterferenceCheckerTest, FaceContactIsNotInterference) {
    auto a = PrimitiveFactory::makeBox(4, 4, 4);
    auto b = PrimitiveFactory::makeBox(4, 4, 4);
    translate(*b, Vec3(4, 0, 0));
    EXPECT_TRUE(InterferenceChecker::check({a.get(), b.get()}).empty());
}

// Interference in parts of any size and anywhere (Phase 142 review): the
// check took its tolerances as absolute, where Booleans take them from the
// parts, and called an overlap under 1e-9 touching, which is all of an
// overlap in parts a ten-thousandth of these. Touching is still not
// interfering, small, far out, or both.
TEST(InterferenceCheckerTest, InterferenceIsFoundAtAnyScaleAndPlace) {
    const auto scaled = [](hz::topo::Solid& solid, double by, const Vec3& at) {
        for (auto& v : const_cast<std::deque<hz::topo::Vertex>&>(solid.vertices())) {
            v.point = v.point * by + at;
        }
    };
    for (const double by : {1e-4, 1.0}) {
        for (const Vec3& at : {Vec3(0, 0, 0), Vec3(1e6, -2e6, 1.5e6)}) {
            auto a = PrimitiveFactory::makeBox(4, 4, 4);
            auto overlapping = PrimitiveFactory::makeBox(4, 4, 4);
            auto touching = PrimitiveFactory::makeBox(4, 4, 4);
            translate(*overlapping, Vec3(2, 2, 2));
            translate(*touching, Vec3(4, 0, 0));
            scaled(*a, by, at);
            scaled(*overlapping, by, at);
            scaled(*touching, by, at);
            EXPECT_TRUE(InterferenceChecker::solidsInterfere(*a, *overlapping))
                << "scale " << by << ", at x " << at.x;
            EXPECT_FALSE(InterferenceChecker::solidsInterfere(*a, *touching))
                << "scale " << by << ", at x " << at.x;
        }
    }
}
