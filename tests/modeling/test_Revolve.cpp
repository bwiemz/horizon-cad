#include "horizon/modeling/Revolve.h"

#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/SketchPlane.h"
#include "horizon/topology/Solid.h"
#include "horizon/topology/TopologyID.h"

#include <gtest/gtest.h>
#include <cmath>
#include <memory>
#include <vector>

using namespace hz::model;
using namespace hz::topo;
using namespace hz::draft;
using hz::math::Vec2;
using hz::math::Vec3;

static constexpr double kTwoPi = 2.0 * 3.14159265358979323846;

// Helper: build a rectangle profile offset from the Y axis.
static std::vector<std::shared_ptr<DraftEntity>> makeOffsetRectProfile(
    double xMin, double xMax, double yMin, double yMax) {
    std::vector<std::shared_ptr<DraftEntity>> profile;
    profile.push_back(std::make_shared<DraftLine>(Vec2(xMin, yMin), Vec2(xMax, yMin)));
    profile.push_back(std::make_shared<DraftLine>(Vec2(xMax, yMin), Vec2(xMax, yMax)));
    profile.push_back(std::make_shared<DraftLine>(Vec2(xMax, yMax), Vec2(xMin, yMax)));
    profile.push_back(std::make_shared<DraftLine>(Vec2(xMin, yMax), Vec2(xMin, yMin)));
    return profile;
}

// ---------------------------------------------------------------------------
// RevolveRectangleFullCircle
// ---------------------------------------------------------------------------

TEST(RevolveTest, RevolveRectangleFullCircle) {
    // Rectangle offset from Y axis, revolve 360 degrees around Y.
    auto profile = makeOffsetRectProfile(5.0, 10.0, 0.0, 5.0);
    SketchPlane plane;  // XY at origin

    auto solid = Revolve::execute(profile, plane, Vec3::Zero, Vec3::UnitY,
                                   kTwoPi, "revolve_1");

    ASSERT_NE(solid, nullptr);
    EXPECT_EQ(solid->vertexCount(), 8u);
    EXPECT_EQ(solid->edgeCount(), 12u);
    EXPECT_EQ(solid->faceCount(), 6u);
    EXPECT_EQ(solid->shellCount(), 1u);
    EXPECT_TRUE(solid->checkEulerFormula());
    EXPECT_TRUE(solid->checkManifold());
    EXPECT_TRUE(solid->isValid()) << solid->validationReport();
}

// ---------------------------------------------------------------------------
// RevolveHasTopologyIDs
// ---------------------------------------------------------------------------

TEST(RevolveTest, RevolveHasTopologyIDs) {
    auto profile = makeOffsetRectProfile(5.0, 10.0, 0.0, 5.0);
    SketchPlane plane;

    auto solid = Revolve::execute(profile, plane, Vec3::Zero, Vec3::UnitY,
                                   kTwoPi, "rev1");
    ASSERT_NE(solid, nullptr);

    for (const auto& face : solid->faces()) {
        EXPECT_TRUE(face.topoId.isValid())
            << "Face " << face.id << " has no TopologyID";
    }

    for (const auto& edge : solid->edges()) {
        EXPECT_TRUE(edge.topoId.isValid())
            << "Edge " << edge.id << " has no TopologyID";
    }
}

// ---------------------------------------------------------------------------
// RevolveHasNURBSGeometry
// ---------------------------------------------------------------------------

TEST(RevolveTest, RevolveHasNURBSGeometry) {
    auto profile = makeOffsetRectProfile(5.0, 10.0, 0.0, 5.0);
    SketchPlane plane;

    auto solid = Revolve::execute(profile, plane, Vec3::Zero, Vec3::UnitY,
                                   kTwoPi, "rev_geom");
    ASSERT_NE(solid, nullptr);

    for (const auto& face : solid->faces()) {
        EXPECT_NE(face.surface, nullptr)
            << "Face " << face.id << " (" << face.topoId.tag()
            << ") has no NURBS surface";
    }

    for (const auto& edge : solid->edges()) {
        EXPECT_NE(edge.curve, nullptr)
            << "Edge " << edge.id << " has no NURBS curve";
    }
}

// ---------------------------------------------------------------------------
// InvalidProfileReturnsNull
// ---------------------------------------------------------------------------

TEST(RevolveTest, InvalidProfileReturnsNull) {
    // Open chain (not closed).
    std::vector<std::shared_ptr<DraftEntity>> profile;
    profile.push_back(std::make_shared<DraftLine>(Vec2(0, 0), Vec2(10, 0)));

    SketchPlane plane;
    auto solid = Revolve::execute(profile, plane, Vec3::Zero, Vec3::UnitY,
                                   kTwoPi, "revolve_bad");
    EXPECT_EQ(solid, nullptr);
}

// ---------------------------------------------------------------------------
// EmptyProfileReturnsNull
// ---------------------------------------------------------------------------

TEST(RevolveTest, EmptyProfileReturnsNull) {
    std::vector<std::shared_ptr<DraftEntity>> profile;
    SketchPlane plane;

    auto solid = Revolve::execute(profile, plane, Vec3::Zero, Vec3::UnitY,
                                   kTwoPi, "revolve_empty");
    EXPECT_EQ(solid, nullptr);
}

// ---------------------------------------------------------------------------
// RevolveAroundZAxis
// ---------------------------------------------------------------------------

TEST(RevolveTest, RevolveAroundZAxis) {
    // Rectangle in XY plane, offset from Z axis, revolve around Z.
    auto profile = makeOffsetRectProfile(3.0, 7.0, -2.0, 2.0);
    SketchPlane plane;

    auto solid = Revolve::execute(profile, plane, Vec3::Zero, Vec3::UnitZ,
                                   kTwoPi, "revolve_z");
    ASSERT_NE(solid, nullptr);
    EXPECT_TRUE(solid->checkEulerFormula());
    EXPECT_TRUE(solid->isValid()) << solid->validationReport();
}
