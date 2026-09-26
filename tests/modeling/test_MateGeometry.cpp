#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <numbers>
#include <optional>
#include <string>
#include <vector>

#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/SketchPlane.h"
#include "horizon/modeling/EdgeProjection.h"
#include "horizon/modeling/Extrude.h"
#include "horizon/modeling/MateGeometry.h"
#include "horizon/modeling/Naming.h"
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
    if (!frame) FAIL() << "no frame";
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
    if (!frame) FAIL() << "no frame";
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

    Mat4 move = Mat4::translation(Vec3(0, 0, 5)) * Mat4::rotationX(std::numbers::pi / 2.0);
    MateFrame placed = frame.transformed(move);

    // Right-handed rotation about X maps +Z to -Y; the origin rotates then
    // translates.
    EXPECT_NEAR(placed.direction.y, -1.0, 1e-9);
    EXPECT_NEAR(placed.origin.x, 1.0, 1e-9);
    EXPECT_NEAR(placed.origin.z, 5.0, 1e-9);
}

// ---------------------------------------------------------------------------
// Phase 160: spheres, cones and edges
// ---------------------------------------------------------------------------

namespace {

/// The frame of the first face of @p solid with an ideal surface.
std::optional<MateFrame> curvedFrame(const hz::topo::Solid& solid) {
    for (const auto& face : solid.faces()) {
        if (face.analyticSurface) return MateGeometry::frameForFace(face);
    }
    return std::nullopt;
}

}  // namespace

TEST(MateGeometryTest, ASphereIsItsCentreAndRadius) {
    const auto ball = PrimitiveFactory::makeSphere(4.0);
    ASSERT_NE(ball, nullptr);
    const auto frame = curvedFrame(*ball);
    if (!frame) FAIL() << "no frame";
    EXPECT_EQ(frame->kind, MateFrameKind::Spherical);
    EXPECT_NEAR(frame->origin.length(), 0.0, 1e-6);
    EXPECT_NEAR(frame->radius, 4.0, 1e-6);
}

TEST(MateGeometryTest, AConeIsItsApexAxisAndHalfAngle) {
    // 4 at the bottom, 2 at the top, 6 high: its apex 12 up, where it would
    // come to a point.
    const auto cone = PrimitiveFactory::makeCone(4.0, 2.0, 6.0);
    ASSERT_NE(cone, nullptr);
    const auto frame = curvedFrame(*cone);
    if (!frame) FAIL() << "no frame";
    EXPECT_EQ(frame->kind, MateFrameKind::Conical);
    EXPECT_NEAR(frame->origin.x, 0.0, 1e-6);
    EXPECT_NEAR(frame->origin.y, 0.0, 1e-6);
    EXPECT_NEAR(frame->origin.z, 12.0, 1e-6);
    EXPECT_NEAR(frame->direction.z, -1.0, 1e-6) << "from the apex into the cone";
    EXPECT_NEAR(frame->angle, std::atan(1.0 / 3.0), 1e-6);
}

TEST(MateGeometryTest, AStraightEdgeIsALineAndARoundOneACircle) {
    const auto box = PrimitiveFactory::makeBox(10, 20, 30);
    std::string upright;
    for (const auto& e : box->edges()) {
        const auto* he = e.halfEdge;
        if (he == nullptr || he->origin == nullptr || he->next == nullptr) continue;
        const Vec3 a = he->origin->point;
        const Vec3 b = he->next->origin->point;
        if (a.x == 10.0 && b.x == 10.0 && a.y == 20.0 && b.y == 20.0) {
            upright = wholeEdgeName(e.topoId.tag());
        }
    }
    ASSERT_FALSE(upright.empty());
    const auto line = MateGeometry::frameForEdge(*box, upright);
    if (!line) FAIL() << "no line";
    EXPECT_EQ(line->kind, MateFrameKind::Line);
    EXPECT_NEAR(std::abs(line->direction.z), 1.0, 1e-12);
    EXPECT_NEAR(line->origin.x, 10.0, 1e-12);
    EXPECT_NEAR(line->origin.y, 20.0, 1e-12);

    // A disc's rim, named as features name it: one curve in chords.
    std::vector<std::shared_ptr<hz::draft::DraftEntity>> profile{
        std::make_shared<hz::draft::DraftCircle>(hz::math::Vec2(1, 2), 5.0)};
    const auto disc =
        Extrude::execute(profile, hz::draft::SketchPlane(), Vec3::UnitZ, 12.0, "extrude_7",
                         Extrude::kDefaultSegments, 0.0, nullptr, NamingScheme::Stable);
    ASSERT_NE(disc, nullptr);
    std::string rim;
    for (const auto& e : disc->edges()) {
        const auto* he = e.halfEdge;
        if (e.analyticCurve && he != nullptr && he->origin != nullptr &&
            std::abs(he->origin->point.z - 12.0) < 1e-9) {
            rim = wholeEdgeName(e.topoId.tag());
        }
    }
    ASSERT_FALSE(rim.empty());
    const auto circle = MateGeometry::frameForEdge(*disc, rim);
    if (!circle) FAIL() << "no circle";
    EXPECT_EQ(circle->kind, MateFrameKind::Circle);
    EXPECT_NEAR(circle->origin.x, 1.0, 1e-6);
    EXPECT_NEAR(circle->origin.y, 2.0, 1e-6);
    EXPECT_NEAR(circle->origin.z, 12.0, 1e-6);
    EXPECT_NEAR(circle->radius, 5.0, 1e-6);
    EXPECT_NEAR(std::abs(circle->direction.z), 1.0, 1e-9);

    EXPECT_FALSE(MateGeometry::frameForEdge(*box, "box/nosuch").has_value());
}
