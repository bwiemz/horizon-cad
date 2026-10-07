#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include "horizon/math/Constants.h"
#include "horizon/modeling/Draft.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/modeling/MateGeometry.h"
#include "horizon/modeling/Pattern.h"
#include "horizon/modeling/PrimitiveFactory.h"
#include "horizon/modeling/Shell.h"
#include "horizon/topology/Solid.h"
#include "horizon/topology/TopologyID.h"

using namespace hz::model;
using hz::math::Vec3;

namespace {

// XY bounding box of the vertices at the given Z (within tol).
struct BBox2D {
    double minX = 1e9, maxX = -1e9, minY = 1e9, maxY = -1e9;
    double width() const { return maxX - minX; }
    double depth() const { return maxY - minY; }
};

BBox2D bboxAtZ(const hz::topo::Solid& solid, double z, double tol = 1e-6) {
    BBox2D b;
    for (const auto& v : solid.vertices()) {
        if (std::abs(v.point.z - z) > tol) continue;
        b.minX = std::min(b.minX, v.point.x);
        b.maxX = std::max(b.maxX, v.point.x);
        b.minY = std::min(b.minY, v.point.y);
        b.maxY = std::max(b.maxY, v.point.y);
    }
    return b;
}

}  // namespace

// ---------------------------------------------------------------------------
// BoxSideDraftTapersTop
// ---------------------------------------------------------------------------

TEST(DraftTest, BoxSideDraftTapersTop) {
    // 10x10 box, 5 tall (z in [0,5]). Draft the sides with the pull along +Z
    // and the neutral plane at the bottom (z=0), so the top face grows.
    auto box = PrimitiveFactory::makeBox(10.0, 10.0, 5.0);
    ASSERT_NE(box, nullptr);

    const double tanA = 0.1;
    auto drafted = Draft::execute(std::move(box), Vec3(0, 0, 1), Vec3(0, 0, 0), std::atan(tanA));
    ASSERT_NE(drafted, nullptr);
    EXPECT_TRUE(drafted->isValid());
    EXPECT_TRUE(drafted->checkManifold());

    // Bottom ring (z=0) unchanged: 10x10.
    BBox2D bottom = bboxAtZ(*drafted, 0.0);
    EXPECT_NEAR(bottom.width(), 10.0, 1e-6);
    EXPECT_NEAR(bottom.depth(), 10.0, 1e-6);

    // Top ring (z=5) grows by delta=5*0.1=0.5 per side → 11x11.
    BBox2D top = bboxAtZ(*drafted, 5.0);
    EXPECT_NEAR(top.width(), 11.0, 1e-6);
    EXPECT_NEAR(top.depth(), 11.0, 1e-6);
    EXPECT_NEAR(top.minX, -0.5, 1e-6);
    EXPECT_NEAR(top.maxX, 10.5, 1e-6);
}

// ---------------------------------------------------------------------------
// NegativeAngleTapersInward
// ---------------------------------------------------------------------------

TEST(DraftTest, NegativeAngleTapersInward) {
    auto box = PrimitiveFactory::makeBox(10.0, 10.0, 5.0);
    const double tanA = 0.1;
    auto drafted = Draft::execute(std::move(box), Vec3(0, 0, 1), Vec3(0, 0, 0), -std::atan(tanA));
    ASSERT_NE(drafted, nullptr);
    EXPECT_TRUE(drafted->isValid());

    BBox2D top = bboxAtZ(*drafted, 5.0);
    // Top shrinks to 9x9.
    EXPECT_NEAR(top.width(), 9.0, 1e-6);
    EXPECT_NEAR(top.minX, 0.5, 1e-6);
    EXPECT_NEAR(top.maxX, 9.5, 1e-6);
}

// ---------------------------------------------------------------------------
// ZeroAngleIsNoOp
// ---------------------------------------------------------------------------

TEST(DraftTest, ZeroAngleIsNoOp) {
    auto box = PrimitiveFactory::makeBox(10.0, 10.0, 5.0);
    // Snapshot vertex positions.
    std::vector<Vec3> before;
    for (const auto& v : box->vertices()) before.push_back(v.point);

    auto drafted = Draft::execute(std::move(box), Vec3(0, 0, 1), Vec3(0, 0, 0), 0.0);
    ASSERT_NE(drafted, nullptr);

    size_t i = 0;
    for (const auto& v : drafted->vertices()) {
        ASSERT_LT(i, before.size());
        EXPECT_NEAR((v.point - before[i]).length(), 0.0, 1e-9);
        ++i;
    }
    EXPECT_TRUE(drafted->isValid());
}

// ---------------------------------------------------------------------------
// NeutralAtTopKeepsTopFixed
// ---------------------------------------------------------------------------

TEST(DraftTest, NeutralAtTopKeepsTopFixed) {
    // Neutral plane at z=5 (the top): the top stays fixed, the bottom tapers.
    auto box = PrimitiveFactory::makeBox(10.0, 10.0, 5.0);
    const double tanA = 0.2;
    auto drafted = Draft::execute(std::move(box), Vec3(0, 0, 1), Vec3(0, 0, 5), std::atan(tanA));
    ASSERT_NE(drafted, nullptr);

    BBox2D top = bboxAtZ(*drafted, 5.0);
    EXPECT_NEAR(top.width(), 10.0, 1e-6);  // unchanged

    // Bottom is below the neutral plane (height -5) → moves inward by 5*0.2=1.
    BBox2D bottom = bboxAtZ(*drafted, 0.0);
    EXPECT_NEAR(bottom.width(), 8.0, 1e-6);
    EXPECT_NEAR(bottom.minX, 1.0, 1e-6);
}

// ---------------------------------------------------------------------------
// Several bodies: each drafts as it would alone
// ---------------------------------------------------------------------------

TEST(DraftTest, EachBodyOfAMultiBodyPartDraftsOutward) {
    // Two boxes 100 apart: the average of all their vertices lies between
    // them, outside both, and orienting faces against it drafted the inner
    // sides of each box the wrong way.
    auto left = PrimitiveFactory::makeBox(10.0, 10.0, 5.0);
    auto right = Pattern::transformed(*PrimitiveFactory::makeBox(10.0, 10.0, 5.0),
                                      hz::math::Mat4::translation(Vec3(100, 0, 0)));
    auto part = Pattern::collect(*left, *right);
    ASSERT_EQ(part->shells().size(), 2u);

    const double tanA = 0.1;
    auto drafted = Draft::execute(std::move(part), Vec3(0, 0, 1), Vec3(0, 0, 0), std::atan(tanA));
    ASSERT_NE(drafted, nullptr);

    // Each box's top grows by 0.5 on every side, including the sides that
    // face the other box.
    BBox2D top = bboxAtZ(*drafted, 5.0);
    EXPECT_NEAR(top.minX, -0.5, 1e-6);
    EXPECT_NEAR(top.maxX, 110.5, 1e-6);
    double innerLeft = -1e9;
    double innerRight = 1e9;
    for (const auto& v : drafted->vertices()) {
        if (std::abs(v.point.z - 5.0) > 1e-9) continue;
        if (v.point.x < 50.0) innerLeft = std::max(innerLeft, v.point.x);
        if (v.point.x > 50.0) innerRight = std::min(innerRight, v.point.x);
    }
    EXPECT_NEAR(innerLeft, 10.5, 1e-6) << "the left box's inner side drafts outward";
    EXPECT_NEAR(innerRight, 99.5, 1e-6) << "the right box's inner side drafts outward";
}

// ---------------------------------------------------------------------------
// A drafted cylinder is a cylinder no longer
// ---------------------------------------------------------------------------

// Its sides lean, so the cylinder they recorded is dropped, and the circle of
// the rim that moved: a shell's cavity, a mate and the ideal mass properties
// read them, and saw the cylinder as it was before the draft. Shelled, its
// wall was then thinner at the top than at the bottom.
TEST(DraftTest, ADraftedCylinderForgetsTheCylinderItWas) {
    const double angle = 5.0 * hz::math::kPi / 180.0;
    auto drafted = Draft::execute(PrimitiveFactory::makeCylinder(5.0, 10.0), Vec3(0, 0, 1),
                                  Vec3(0, 0, 0), angle);
    ASSERT_NE(drafted, nullptr);
    for (const auto& face : drafted->faces()) {
        EXPECT_EQ(face.analyticSurface, nullptr) << face.topoId.tag();
        const auto frame = MateGeometry::frameForFace(face);
        EXPECT_TRUE(!frame || frame->kind == MateFrameKind::Planar) << face.topoId.tag();
    }
    // The rim on the neutral plane has not moved, and keeps its circle.
    for (const auto& edge : drafted->edges()) {
        const Vec3& a = edge.halfEdge->origin->point;
        const Vec3& b = edge.halfEdge->twin->origin->point;
        const bool still = std::abs(a.z) < 1e-12 && std::abs(b.z) < 1e-12;
        EXPECT_EQ(edge.analyticCurve != nullptr, still) << edge.topoId.tag();
    }

    // Shelled 1 thick, open at the top: each side's cavity wall is the side
    // moved in by 1, so a 32-gon whose inradius is the side's less 1/cos(5°).
    const auto shelled =
        Shell::executeOffset(*drafted, 1.0, {hz::topo::TopologyID::make("cylinder", "top")}, "s");
    ASSERT_TRUE(shelled.ok) << shelled.message;
    const double n = 32.0;
    const double t = std::tan(angle);
    const auto frustum = [&](double inradius, double from, double to) {
        // A regular n-gon of inradius r has area n r^2 tan(pi/n).
        const double r0 = inradius + from * t;
        const double r1 = inradius + to * t;
        return n * std::tan(hz::math::kPi / n) * (r1 * r1 * r1 - r0 * r0 * r0) / (3.0 * t);
    };
    const double side = 5.0 * std::cos(hz::math::kPi / n);
    const double part = frustum(side, 0.0, 10.0);
    EXPECT_NEAR(MassPropertiesCalculator::compute(*drafted).volume, part, 1e-9);
    EXPECT_NEAR(MassPropertiesCalculator::compute(*shelled.solid).volume,
                part - frustum(side - 1.0 / std::cos(angle), 1.0, 10.0), 1e-9);
}
