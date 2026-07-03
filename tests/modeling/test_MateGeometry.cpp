#include <gtest/gtest.h>

#include <cmath>

#include "horizon/modeling/MateGeometry.h"
#include "horizon/modeling/PrimitiveFactory.h"
#include "horizon/topology/Solid.h"

using namespace hz::model;
using hz::math::Mat4;
using hz::math::Vec3;
using hz::topo::TopologyID;

// ---------------------------------------------------------------------------
// FindFaceByExactId
// ---------------------------------------------------------------------------

TEST(MateGeometryTest, FindFaceByExactId) {
    auto box = PrimitiveFactory::makeBox(10, 20, 30);
    ASSERT_NE(box, nullptr);

    const auto* top = MateGeometry::findFace(*box, TopologyID::make("box", "top"));
    ASSERT_NE(top, nullptr);
    EXPECT_EQ(top->topoId.tag(), "box/top");

    EXPECT_EQ(MateGeometry::findFace(*box, TopologyID::make("box", "nonexistent")), nullptr);
    EXPECT_EQ(MateGeometry::findFace(*box, TopologyID()), nullptr);
}

// ---------------------------------------------------------------------------
// PlanarFrameFromBoxFace
// ---------------------------------------------------------------------------

TEST(MateGeometryTest, PlanarFrameFromBoxFace) {
    auto box = PrimitiveFactory::makeBox(10, 20, 30);
    ASSERT_NE(box, nullptr);

    const auto* top = MateGeometry::findFace(*box, TopologyID::make("box", "top"));
    ASSERT_NE(top, nullptr);

    auto frame = MateGeometry::frameForFace(*top);
    ASSERT_TRUE(frame.has_value());
    EXPECT_EQ(frame->kind, MateFrameKind::Planar);
    // The top face normal is +Z (or -Z depending on construction); the plane
    // must be at z = 30 either way.
    EXPECT_NEAR(std::abs(frame->direction.z), 1.0, 1e-9);
    EXPECT_NEAR(frame->origin.z, 30.0, 1e-9);
}

// ---------------------------------------------------------------------------
// CylindricalFrameFromCylinderFace
// ---------------------------------------------------------------------------

TEST(MateGeometryTest, CylindricalFrameFromCylinderFace) {
    auto cyl = PrimitiveFactory::makeCylinder(5.0, 12.0);
    ASSERT_NE(cyl, nullptr);

    // The lateral surface is built as four quarter-cylinder patches.
    const auto* lateral = MateGeometry::findFace(*cyl, TopologyID::make("cylinder", "side0"));
    ASSERT_NE(lateral, nullptr);

    auto frame = MateGeometry::frameForFace(*lateral);
    ASSERT_TRUE(frame.has_value());
    EXPECT_EQ(frame->kind, MateFrameKind::Cylindrical);
    EXPECT_NEAR(std::abs(frame->direction.z), 1.0, 1e-6);
    EXPECT_NEAR(frame->radius, 5.0, 1e-6);
    EXPECT_NEAR(frame->origin.x, 0.0, 1e-6);
    EXPECT_NEAR(frame->origin.y, 0.0, 1e-6);
}

// ---------------------------------------------------------------------------
// TransformedFrame
// ---------------------------------------------------------------------------

TEST(MateGeometryTest, TransformedFrame) {
    MateFrame frame;
    frame.kind = MateFrameKind::Planar;
    frame.origin = Vec3(1, 0, 0);
    frame.direction = Vec3(0, 0, 1);

    Mat4 move = Mat4::translation(Vec3(0, 0, 5)) * Mat4::rotationX(M_PI / 2.0);
    MateFrame placed = frame.transformed(move);

    // Right-handed rotation about X maps +Z to -Y; the origin rotates then
    // translates.
    EXPECT_NEAR(placed.direction.y, -1.0, 1e-9);
    EXPECT_NEAR(placed.origin.x, 1.0, 1e-9);
    EXPECT_NEAR(placed.origin.z, 5.0, 1e-9);
}
