#include <gtest/gtest.h>

#include <cmath>

#include "horizon/modeling/Draft.h"
#include "horizon/modeling/PrimitiveFactory.h"
#include "horizon/topology/Solid.h"

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
