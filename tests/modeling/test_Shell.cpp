#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/SketchPlane.h"
#include "horizon/modeling/BooleanOp.h"
#include "horizon/modeling/Extrude.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/modeling/Pattern.h"
#include "horizon/modeling/PrimitiveFactory.h"
#include "horizon/modeling/Shell.h"
#include "horizon/topology/GeometryValidator.h"
#include "horizon/topology/Queries.h"
#include "horizon/topology/Solid.h"
#include "horizon/topology/TopologyID.h"

using namespace hz::model;
using hz::math::Vec3;
using hz::topo::TopologyID;

namespace {

struct BBox {
    double minX = 1e9, maxX = -1e9, minY = 1e9, maxY = -1e9, minZ = 1e9, maxZ = -1e9;
};

BBox boundsOf(const hz::topo::Solid& solid) {
    BBox b;
    for (const auto& v : solid.vertices()) {
        b.minX = std::min(b.minX, v.point.x);
        b.maxX = std::max(b.maxX, v.point.x);
        b.minY = std::min(b.minY, v.point.y);
        b.maxY = std::max(b.maxY, v.point.y);
        b.minZ = std::min(b.minZ, v.point.z);
        b.maxZ = std::max(b.maxZ, v.point.z);
    }
    return b;
}

}  // namespace

// ---------------------------------------------------------------------------
// BoxTopRemovedProducesCup
// ---------------------------------------------------------------------------

TEST(ShellTest, BoxTopRemovedProducesCup) {
    // 10x10x5 box; remove the top ("box/top"), wall thickness 1 → cup.
    auto box = PrimitiveFactory::makeBox(10.0, 10.0, 5.0);
    ASSERT_NE(box, nullptr);

    ShellResult r = Shell::execute(std::move(box), 1.0, {TopologyID::make("box", "top")});
    ASSERT_TRUE(r.ok) << r.message;
    ASSERT_NE(r.solid, nullptr);
    EXPECT_TRUE(r.solid->checkEulerFormula());
    EXPECT_TRUE(r.solid->checkManifold());

    // The outer envelope is unchanged: 10x10x5.
    BBox b = boundsOf(*r.solid);
    EXPECT_NEAR(b.maxX - b.minX, 10.0, 1e-6);
    EXPECT_NEAR(b.maxY - b.minY, 10.0, 1e-6);
    EXPECT_NEAR(b.maxZ - b.minZ, 5.0, 1e-6);

    // Cup topology: 4-ring stack over an N=4 profile → 3N+2 = 14 faces,
    // 4N = 16 vertices, 7N = 28 edges.
    EXPECT_EQ(r.solid->faceCount(), 14u);
    EXPECT_EQ(r.solid->vertexCount(), 16u);
    EXPECT_EQ(r.solid->edgeCount(), 28u);

    // The cavity floor sits one wall-thickness above the base (z = 1); the
    // inner walls span x,y in [1, 9] (offset inward by 1 from [0, 10]).
    int cavityFloorVerts = 0, innerCornerVerts = 0;
    for (const auto& v : r.solid->vertices()) {
        if (std::abs(v.point.z - 1.0) < 1e-6) ++cavityFloorVerts;
        if (std::abs(v.point.x - 1.0) < 1e-6 || std::abs(v.point.x - 9.0) < 1e-6) {
            if (v.point.x > 0.5 && v.point.x < 9.5) ++innerCornerVerts;
        }
    }
    EXPECT_EQ(cavityFloorVerts, 4);  // inner base ring
    EXPECT_GT(innerCornerVerts, 0);
}

// ---------------------------------------------------------------------------
// TooThickRejected
// ---------------------------------------------------------------------------

TEST(ShellTest, TooThickRejected) {
    auto box = PrimitiveFactory::makeBox(10.0, 10.0, 5.0);
    // Inradius of the 10x10 top is 5; thickness 5 must be rejected.
    ShellResult r = Shell::execute(std::move(box), 5.0, {TopologyID::make("box", "top")});
    EXPECT_FALSE(r.ok);
    EXPECT_FALSE(r.message.empty());
    EXPECT_EQ(r.solid, nullptr);
}

// ---------------------------------------------------------------------------
// InvalidInputsRejected
// ---------------------------------------------------------------------------

TEST(ShellTest, InvalidInputsRejected) {
    // No removed face.
    {
        auto box = PrimitiveFactory::makeBox(10.0, 10.0, 5.0);
        ShellResult r = Shell::execute(std::move(box), 1.0, {});
        EXPECT_FALSE(r.ok);
    }
    // Non-positive thickness.
    {
        auto box = PrimitiveFactory::makeBox(10.0, 10.0, 5.0);
        ShellResult r = Shell::execute(std::move(box), 0.0, {TopologyID::make("box", "top")});
        EXPECT_FALSE(r.ok);
    }
    // Unknown removed face id.
    {
        auto box = PrimitiveFactory::makeBox(10.0, 10.0, 5.0);
        ShellResult r = Shell::execute(std::move(box), 1.0, {TopologyID::make("box", "nope")});
        EXPECT_FALSE(r.ok);
    }
    // Null solid.
    {
        ShellResult r = Shell::execute(nullptr, 1.0, {TopologyID::make("box", "top")});
        EXPECT_FALSE(r.ok);
    }
}

// ---------------------------------------------------------------------------
// RemoveBottomAlsoWorks
// ---------------------------------------------------------------------------

TEST(ShellTest, RemoveBottomAlsoWorks) {
    auto box = PrimitiveFactory::makeBox(8.0, 12.0, 6.0);
    ShellResult r = Shell::execute(std::move(box), 1.0, {TopologyID::make("box", "bottom")});
    ASSERT_TRUE(r.ok) << r.message;
    EXPECT_TRUE(r.solid->checkManifold());
    EXPECT_GT(r.solid->faceCount(), 6u);
}

TEST(ShellTest, APartWithSeveralBodiesIsRefusedNotTruncated) {
    // The cup is rebuilt from one body's caps; every other body used to be
    // dropped without a word.
    auto a = PrimitiveFactory::makeBox(10.0, 10.0, 10.0);
    auto b = Pattern::transformed(*PrimitiveFactory::makeBox(10.0, 10.0, 10.0),
                                  hz::math::Mat4::translation(Vec3(50, 0, 0)));
    auto part = Pattern::collect(*a, *b);
    ShellResult r = Shell::execute(std::move(part), 1.0, {TopologyID::make("box", "top")});
    EXPECT_FALSE(r.ok);
    EXPECT_EQ(r.solid, nullptr);
    EXPECT_NE(r.message.find("several bodies"), std::string::npos) << r.message;
}

// -- Refuse what cannot be shelled correctly (Phase 123) ----------------------

namespace {

// An extruded polygon, @p height tall, from @p corners (counter-clockwise).
std::unique_ptr<hz::topo::Solid> prismOf(const std::vector<hz::math::Vec2>& corners,
                                         double height) {
    std::vector<std::shared_ptr<hz::draft::DraftEntity>> profile;
    for (size_t i = 0; i < corners.size(); ++i) {
        profile.push_back(
            std::make_shared<hz::draft::DraftLine>(corners[i], corners[(i + 1) % corners.size()]));
    }
    return Extrude::execute(profile, hz::draft::SketchPlane{}, Vec3(0, 0, 1), height, "prism");
}

// The face whose corners are all at height @p z.
TopologyID faceAt(const hz::topo::Solid& solid, double z) {
    for (const auto& f : solid.faces()) {
        bool all = true;
        for (const auto* v : hz::topo::faceVertices(&f))
            all = all && std::abs(v->point.z - z) < 1e-9;
        if (all) return f.topoId;
    }
    return {};
}

double volumeOf(const hz::topo::Solid& solid) {
    return MassPropertiesCalculator::compute(solid).volume;
}

// An L whose arms are 10 long and 3 wide: its centroid is not where the old
// code assumed "inward" lay.
const std::vector<hz::math::Vec2> kL = {{0, 0}, {10, 0}, {10, 3}, {3, 3}, {3, 10}, {0, 10}};

}  // namespace

// Each edge of the cavity is the edge of the profile moved inward, to its
// left. Picking "inward" as the side facing the centroid moved the edges at
// the L's inner corner outward, and the cavity broke through the walls.
TEST(ShellTest, AnLShapedPrismIsHollowedInward) {
    auto l = prismOf(kL, 4.0);
    ASSERT_NE(l, nullptr);
    const TopologyID top = faceAt(*l, 4.0);
    ASSERT_TRUE(top.isValid());
    ShellResult r = Shell::execute(std::move(l), 1.0, {top});
    ASSERT_TRUE(r.ok) << r.message;
    // Outer L: 100 - 49 = 51. Cavity: (8 x 8 - 7 x 7) = 15, 3 deep.
    EXPECT_NEAR(volumeOf(*r.solid), 51.0 * 4.0 - 15.0 * 3.0, 1e-6);
    EXPECT_TRUE(hz::topo::GeometryValidator::isGeometricallyValid(*r.solid))
        << hz::topo::GeometryValidator::report(*r.solid);
}

// The limit is where an arm of the cavity closes up (half the arm's width,
// 1.5), not a distance from the centroid, which refused walls it could make.
TEST(ShellTest, TheWallCanBeAsThickAsTheProfileAllows) {
    {
        auto l = prismOf(kL, 4.0);
        const TopologyID top = faceAt(*l, 4.0);
        ShellResult r = Shell::execute(std::move(l), 1.4, {top});
        ASSERT_TRUE(r.ok) << r.message;
        const double cavity = (10.0 - 2.8) * (10.0 - 2.8) - 49.0;
        EXPECT_NEAR(volumeOf(*r.solid), 51.0 * 4.0 - cavity * (4.0 - 1.4), 1e-6);
    }
    {
        auto l = prismOf(kL, 4.0);
        const TopologyID top = faceAt(*l, 4.0);
        ShellResult r = Shell::execute(std::move(l), 1.5, {top});
        EXPECT_FALSE(r.ok);
        EXPECT_NE(r.message.find("too thick"), std::string::npos) << r.message;
    }
}

// The cup is built from the two caps. A plate with a hole through it would
// have come back as a plain cup, the hole gone: it is refused instead.
TEST(ShellTest, APartWithAHoleIsRefusedNotStripped) {
    auto plate = PrimitiveFactory::makeBox(20, 20, 5);
    std::vector<std::shared_ptr<hz::draft::DraftEntity>> circle = {
        std::make_shared<hz::draft::DraftCircle>(hz::math::Vec2(10, 10), 2.0)};
    auto rod = Extrude::execute(circle, hz::draft::SketchPlane{}, Vec3(0, 0, 1), 5.0, "rod");
    ASSERT_NE(rod, nullptr);
    auto drilled = BooleanOp::execute(*plate, *rod, BooleanType::Subtract);
    ASSERT_NE(drilled, nullptr);
    const TopologyID top = faceAt(*drilled, 5.0);
    ASSERT_TRUE(top.isValid());
    ShellResult r = Shell::execute(std::move(drilled), 1.0, {top});
    EXPECT_FALSE(r.ok);
    EXPECT_NE(r.message.find("plain prism"), std::string::npos) << r.message;
}

// Only the first face was ever opened; the rest were ignored.
TEST(ShellTest, OpeningTwoFacesIsRefused) {
    auto box = PrimitiveFactory::makeBox(10, 10, 10);
    ShellResult r = Shell::execute(
        std::move(box), 1.0, {TopologyID::make("box", "top"), TopologyID::make("box", "front")});
    EXPECT_FALSE(r.ok);
    EXPECT_NE(r.message.find("more than one face"), std::string::npos) << r.message;
}
