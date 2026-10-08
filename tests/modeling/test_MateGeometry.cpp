#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <numbers>
#include <optional>
#include <string>
#include <vector>

#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/SketchPlane.h"
#include "horizon/modeling/AssemblySolver.h"
#include "horizon/modeling/EdgeProjection.h"
#include "horizon/modeling/Extrude.h"
#include "horizon/modeling/MateGeometry.h"
#include "horizon/modeling/Naming.h"
#include "horizon/modeling/Pattern.h"
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

// ---------------------------------------------------------------------------
// A flat face's frame faces out of its part
// ---------------------------------------------------------------------------

namespace {

/// Each flat face of @p solid: its frame's direction leads out of the part,
/// away from @p inside.
void expectEveryFlatFaceFacesOut(const hz::topo::Solid& solid, const Vec3& inside) {
    for (const auto& face : solid.faces()) {
        const auto frame = MateGeometry::frameForFace(face);
        if (!frame || frame->kind != MateFrameKind::Planar) continue;
        EXPECT_GT(frame->direction.dot(frame->origin - inside), 0.0) << face.topoId.tag();
    }
}

MateFrame faceFrame(const hz::topo::Solid& solid, const std::string& tag) {
    const auto* face = MateGeometry::findFace(solid, TopologyID::fromTag(tag));
    EXPECT_NE(face, nullptr) << tag;
    const auto frame = face != nullptr ? MateGeometry::frameForFace(*face) : std::nullopt;
    EXPECT_TRUE(frame.has_value()) << tag;
    return frame.value_or(MateFrame{});
}

}  // namespace

// Its direction followed the way its surface was laid out, not the part: a
// box's bottom, back and left faces, an extrusion's bottom cap and the sides
// of a profile drawn clockwise faced into it.
TEST(MateGeometryTest, AFlatFaceFacesOutOfItsPart) {
    expectEveryFlatFaceFacesOut(*PrimitiveFactory::makeBox(10, 10, 10), Vec3(5, 5, 5));
    expectEveryFlatFaceFacesOut(*PrimitiveFactory::makeCylinder(3, 10), Vec3(0, 0, 5));

    std::vector<std::shared_ptr<hz::draft::DraftEntity>> clockwise;
    const std::vector<hz::math::Vec2> corners{{0, 0}, {0, 4}, {6, 4}, {6, 0}};
    for (size_t i = 0; i < corners.size(); ++i) {
        clockwise.push_back(
            std::make_shared<hz::draft::DraftLine>(corners[i], corners[(i + 1) % corners.size()]));
    }
    const auto prism =
        Extrude::execute(clockwise, hz::draft::SketchPlane(), Vec3::UnitZ, 2.0, "prism");
    ASSERT_NE(prism, nullptr);
    expectEveryFlatFaceFacesOut(*prism, Vec3(3, 2, 1));

    // A part far from the origin for its size, where measuring its winding
    // about the origin rounded the wrong way.
    const Mat4 far =
        Mat4::translation(Vec3(3e6, 7e6, 1e6)) * Mat4::rotationX(0.3) * Mat4::rotationZ(0.7);
    const auto moved = Pattern::transformed(*PrimitiveFactory::makeBox(1, 1, 1), far);
    expectEveryFlatFaceFacesOut(*moved, far.transformPoint(Vec3(0.5, 0.5, 0.5)));
}

// Mates on the faces of real parts, so on their frames as the assembly reads
// them: a cylinder tangent to a box's left face went inside the box, and a
// distance from it was measured into the box.
TEST(MateGeometryTest, MatesOnFlatFacesKeepThePartsOutsideEachOther) {
    const auto box = PrimitiveFactory::makeBox(10, 10, 10);
    const auto cylinder = PrimitiveFactory::makeCylinder(3, 10);
    SolverComponent base;
    base.id = 1;
    base.grounded = true;
    SolverComponent other;
    other.id = 2;
    AssemblySolver solver;

    SolverMate tangent;
    tangent.type = MateType::Tangent;
    tangent.componentA = 1;
    tangent.componentB = 2;
    tangent.frameA = faceFrame(*box, "box/left");
    tangent.frameB = faceFrame(*cylinder, "cylinder/side0");
    other.transform = Mat4::translation(Vec3(-20, 5, 0));
    auto result = solver.solve({base, other}, {tangent});
    ASSERT_EQ(result.status, AssemblySolveStatus::Success) << result.message;
    const MateFrame axis = tangent.frameB.transformed(result.transforms.at(2));
    EXPECT_NEAR(axis.origin.x, -3.0, 1e-6) << "the cylinder beside the box, not in it";

    // Another box 2 to the left of the first: its right face 2 out from the
    // first's left face.
    SolverMate apart;
    apart.type = MateType::Distance;
    apart.componentA = 1;
    apart.componentB = 2;
    apart.frameA = faceFrame(*box, "box/left");
    apart.frameB = faceFrame(*box, "box/right");
    apart.value = 2.0;
    other.transform = Mat4::translation(Vec3(-30, 0, 0));
    result = solver.solve({base, other}, {apart});
    ASSERT_EQ(result.status, AssemblySolveStatus::Success) << result.message;
    EXPECT_NEAR(apart.frameB.transformed(result.transforms.at(2)).origin.x, -2.0, 1e-6);

    // A bottom at 180 degrees to a top: the faces turned to each other, the
    // second box upright on the first, not upside down.
    SolverMate facing;
    facing.type = MateType::Angle;
    facing.componentA = 1;
    facing.componentB = 2;
    facing.frameA = faceFrame(*box, "box/top");
    facing.frameB = faceFrame(*box, "box/bottom");
    facing.value = std::numbers::pi;
    other.transform = Mat4::translation(Vec3(0, 0, 20)) * Mat4::rotationX(0.3);
    result = solver.solve({base, other}, {facing});
    ASSERT_EQ(result.status, AssemblySolveStatus::Success) << result.message;
    const MateFrame top = faceFrame(*box, "box/top").transformed(result.transforms.at(2));
    EXPECT_NEAR(top.direction.z, 1.0, 1e-6) << "upright";
}
