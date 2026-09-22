#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <vector>

#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/SketchPlane.h"
#include "horizon/math/Constants.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/modeling/Sweep.h"
#include "horizon/topology/GeometryValidator.h"
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

double volumeOf(const Solid& solid) {
    return MassPropertiesCalculator::compute(solid).volume;
}

double pathLength(const std::vector<Vec3>& path) {
    double L = 0.0;
    for (size_t i = 1; i < path.size(); ++i) L += (path[i] - path[i - 1]).length();
    return L;
}

// Vertices of the solid lying on the plane through @p point with normal @p n.
std::vector<Vec3> verticesOnPlane(const Solid& solid, const Vec3& point, const Vec3& n) {
    std::vector<Vec3> out;
    for (const auto& v : solid.vertices()) {
        if (std::abs((v.point - point).dot(n)) < 1e-9) out.push_back(v.point);
    }
    return out;
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
    EXPECT_NEAR(volumeOf(*solid), 160.0, 1e-9);
    EXPECT_TRUE(GeometryValidator::isGeometricallyValid(*solid))
        << GeometryValidator::report(*solid);
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

    // The profile turns the corner with the path.  Translation transport
    // carried the XY-plane square along +X edge-on, so the second leg was a
    // zero-thickness sheet: 40 instead of 72, with two degenerate faces —
    // none of which the topology checks above could see.
    EXPECT_TRUE(GeometryValidator::isGeometricallyValid(*solid))
        << GeometryValidator::report(*solid);
    EXPECT_NEAR(volumeOf(*solid), 4.0 * 18.0, 1e-9);

    // The far cap is the profile turned onto the plane x = 8: a 2x2 square
    // centred on the path's end point.
    const auto cap = verticesOnPlane(*solid, Vec3(8, 0, 10), Vec3(1, 0, 0));
    ASSERT_EQ(cap.size(), 4u);
    for (const auto& p : cap) {
        EXPECT_NEAR(std::abs(p.y), 1.0, 1e-9);
        EXPECT_NEAR(std::abs(p.z - 10.0), 1.0, 1e-9);
    }
}

// ---------------------------------------------------------------------------
// Mitered transport
// ---------------------------------------------------------------------------

TEST(SweepTest, NonPlanarPathVolumeIsAreaTimesLength) {
    // A path that turns in three different planes.  With the profile centred
    // on the path every mitered segment is a prism cut obliquely at both ends,
    // whose volume is (normal section area) x (centre-line length).
    std::vector<Vec3> path = {Vec3(0, 0, 0), Vec3(0, 0, 10), Vec3(6, 2, 14), Vec3(9, 12, 15),
                              Vec3(3, 18, 22)};
    auto solid = Sweep::execute(squareProfile(2.0), xyPlane(), path, "sweep_3d");
    ASSERT_NE(solid, nullptr);
    EXPECT_TRUE(solid->isValid());
    EXPECT_TRUE(GeometryValidator::isGeometricallyValid(*solid))
        << GeometryValidator::report(*solid);
    EXPECT_NEAR(volumeOf(*solid), 4.0 * pathLength(path), 1e-8);

    // The far cap is perpendicular to the last segment and congruent to the
    // profile: transport is a rotation, so every side is still 2 long.
    const Vec3 endDir = (path[4] - path[3]).normalized();
    const auto cap = verticesOnPlane(*solid, path.back(), endDir);
    ASSERT_EQ(cap.size(), 4u);
    Vec3 centroid = Vec3::Zero;
    for (const auto& p : cap) centroid = centroid + p * 0.25;
    EXPECT_NEAR((centroid - path.back()).length(), 0.0, 1e-9);
    for (const auto& p : cap) EXPECT_NEAR((p - centroid).length(), std::sqrt(2.0), 1e-9);
}

TEST(SweepTest, ObliqueProfileSweepsItsNormalSection) {
    // A profile tilted 30 degrees off the path's normal plane sweeps an oblique
    // prism: base area x perpendicular height.
    const double a = hz::math::kPi / 6.0;
    SketchPlane tilted(Vec3(0, 0, 0), Vec3(0, -std::sin(a), std::cos(a)), Vec3(1, 0, 0));
    std::vector<Vec3> path = {Vec3(0, 0, 0), Vec3(0, 0, 10), Vec3(10, 0, 10)};
    auto solid = Sweep::execute(squareProfile(2.0), tilted, path, "sweep_oblique");
    ASSERT_NE(solid, nullptr);
    EXPECT_TRUE(GeometryValidator::isGeometricallyValid(*solid))
        << GeometryValidator::report(*solid);
    EXPECT_NEAR(volumeOf(*solid), 4.0 * std::cos(a) * 20.0, 1e-9);
}

TEST(SweepTest, WideUTurnIsValid) {
    std::vector<Vec3> path = {Vec3(0, 0, 0), Vec3(0, 0, 10), Vec3(10, 0, 10), Vec3(10, 0, 0)};
    auto solid = Sweep::execute(squareProfile(2.0), xyPlane(), path, "sweep_u");
    ASSERT_NE(solid, nullptr);
    EXPECT_TRUE(GeometryValidator::isGeometricallyValid(*solid))
        << GeometryValidator::report(*solid);
    EXPECT_NEAR(volumeOf(*solid), 4.0 * 30.0, 1e-9);
}

TEST(SweepTest, TurnTighterThanProfileRefused) {
    // A U-turn 1 wide with a profile 4 wide: the inside of the turn would have
    // to travel backwards, folding the band through itself.
    std::vector<Vec3> path = {Vec3(0, 0, 0), Vec3(0, 0, 10), Vec3(1, 0, 10), Vec3(1, 0, 0)};
    EXPECT_EQ(Sweep::execute(squareProfile(4.0), xyPlane(), path, "sweep_fold"), nullptr);
}

TEST(SweepTest, PathDoublingBackRefused) {
    std::vector<Vec3> path = {Vec3(0, 0, 0), Vec3(0, 0, 10), Vec3(0, 0, 5)};
    EXPECT_EQ(Sweep::execute(squareProfile(2.0), xyPlane(), path, "sweep_back"), nullptr);
}

TEST(SweepTest, ProfileContainingSweepDirectionRefused) {
    // An XY-plane profile swept along +X sweeps no volume.
    std::vector<Vec3> path = {Vec3(0, 0, 0), Vec3(10, 0, 0)};
    EXPECT_EQ(Sweep::execute(squareProfile(2.0), xyPlane(), path, "sweep_flat"), nullptr);
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
