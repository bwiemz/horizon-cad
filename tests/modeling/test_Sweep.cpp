#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <vector>

#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/SketchPlane.h"
#include "horizon/modeling/Sweep.h"
#include "horizon/topology/Solid.h"
#include "horizon/topology/TopologyID.h"

using namespace hz::model;
using namespace hz::topo;
using namespace hz::draft;
using hz::math::Vec2;
using hz::math::Vec3;

namespace {

std::vector<std::shared_ptr<DraftEntity>> squareProfile(double s) {
    const double h = s * 0.5;
    std::vector<std::shared_ptr<DraftEntity>> p;
    p.push_back(std::make_shared<DraftLine>(Vec2(-h, -h), Vec2(h, -h)));
    p.push_back(std::make_shared<DraftLine>(Vec2(h, -h), Vec2(h, h)));
    p.push_back(std::make_shared<DraftLine>(Vec2(h, h), Vec2(-h, h)));
    p.push_back(std::make_shared<DraftLine>(Vec2(-h, h), Vec2(-h, -h)));
    return p;
}

// Profile drawn on the XY plane; sweep travels along +Z.
SketchPlane xyPlane() {
    return SketchPlane(Vec3(0, 0, 0), Vec3(0, 0, 1), Vec3(1, 0, 0));
}

}  // namespace

// ---------------------------------------------------------------------------
// StraightSweepEqualsExtrude
// ---------------------------------------------------------------------------

TEST(SweepTest, StraightSweepEqualsExtrude) {
    // Two path points → single level → box counts, height 10.
    std::vector<Vec3> path = {Vec3(0, 0, 0), Vec3(0, 0, 10)};
    auto solid = Sweep::execute(squareProfile(4.0), xyPlane(), path, "sweep_1");
    ASSERT_NE(solid, nullptr);

    EXPECT_EQ(solid->vertexCount(), 8u);
    EXPECT_EQ(solid->edgeCount(), 12u);
    EXPECT_EQ(solid->faceCount(), 6u);
    EXPECT_TRUE(solid->checkEulerFormula());
    EXPECT_TRUE(solid->checkManifold());

    // Verify the swept height: max Z minus min Z == 10.
    double zMin = 1e9, zMax = -1e9;
    for (const auto& v : solid->vertices()) {
        zMin = std::min(zMin, v.point.z);
        zMax = std::max(zMax, v.point.z);
    }
    EXPECT_NEAR(zMax - zMin, 10.0, 1e-9);
}

// ---------------------------------------------------------------------------
// LShapedPathIsValid
// ---------------------------------------------------------------------------

TEST(SweepTest, LShapedPathIsValid) {
    // Up then over: 3 path points → 2 levels.
    std::vector<Vec3> path = {Vec3(0, 0, 0), Vec3(0, 0, 10), Vec3(8, 0, 10)};
    auto solid = Sweep::execute(squareProfile(2.0), xyPlane(), path, "sweep_L");
    ASSERT_NE(solid, nullptr);

    // S=2 levels, N=4: V=12, E=20, F=10.
    EXPECT_EQ(solid->vertexCount(), 12u);
    EXPECT_EQ(solid->edgeCount(), 20u);
    EXPECT_EQ(solid->faceCount(), 10u);
    EXPECT_TRUE(solid->checkEulerFormula());
    EXPECT_TRUE(solid->checkManifold());
    EXPECT_TRUE(solid->isValid());

    for (const auto& f : solid->faces()) {
        EXPECT_NE(f.surface, nullptr);
        EXPECT_TRUE(f.topoId.isValid());
    }
}

// ---------------------------------------------------------------------------
// DegeneratePathsRejected
// ---------------------------------------------------------------------------

TEST(SweepTest, DegeneratePathsRejected) {
    // Fewer than 2 points.
    EXPECT_EQ(Sweep::execute(squareProfile(4.0), xyPlane(), {Vec3(0, 0, 0)}, "sweep_a"), nullptr);
    EXPECT_EQ(Sweep::execute(squareProfile(4.0), xyPlane(), {}, "sweep_b"), nullptr);

    // Two coincident points collapse to one distinct point → rejected.
    std::vector<Vec3> dup = {Vec3(1, 1, 1), Vec3(1, 1, 1)};
    EXPECT_EQ(Sweep::execute(squareProfile(4.0), xyPlane(), dup, "sweep_c"), nullptr);

    // Open profile rejected.
    std::vector<std::shared_ptr<DraftEntity>> open;
    open.push_back(std::make_shared<DraftLine>(Vec2(0, 0), Vec2(4, 0)));
    std::vector<Vec3> path = {Vec3(0, 0, 0), Vec3(0, 0, 5)};
    EXPECT_EQ(Sweep::execute(open, xyPlane(), path, "sweep_d"), nullptr);
}

// ---------------------------------------------------------------------------
// RepeatedPathPointsCollapse
// ---------------------------------------------------------------------------

TEST(SweepTest, RepeatedPathPointsCollapse) {
    // A duplicate interior point must not create a degenerate zero-height ring.
    std::vector<Vec3> path = {Vec3(0, 0, 0), Vec3(0, 0, 5), Vec3(0, 0, 5), Vec3(0, 0, 10)};
    auto solid = Sweep::execute(squareProfile(3.0), xyPlane(), path, "sweep_dup");
    ASSERT_NE(solid, nullptr);
    // Duplicate collapsed → 3 distinct points → 2 levels: V=12.
    EXPECT_EQ(solid->vertexCount(), 12u);
    EXPECT_TRUE(solid->checkManifold());
}
