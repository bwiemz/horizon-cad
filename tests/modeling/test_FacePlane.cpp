// Phase 157: a flat face's plane, found by its name as a sketch keeps it.

#include <gtest/gtest.h>

#include <string>

#include "horizon/math/Mat4.h"
#include "horizon/modeling/BooleanOp.h"
#include "horizon/modeling/FacePlane.h"
#include "horizon/modeling/Naming.h"
#include "horizon/modeling/Pattern.h"
#include "horizon/modeling/PrimitiveFactory.h"
#include "horizon/topology/TopologyID.h"

using hz::math::Mat4;
using hz::math::Vec3;
using hz::model::planeOfFace;
using hz::model::PrimitiveFactory;
using hz::model::wholeFaceName;

TEST(FacePlaneTest, AFaceIsNamedWholeWithoutItsPieces) {
    EXPECT_EQ(wholeFaceName("extrude_1/cap_top"), "extrude_1/cap_top");
    EXPECT_EQ(wholeFaceName("extrude_1/cap_top/piece:0"), "extrude_1/cap_top");
    EXPECT_EQ(wholeFaceName("extrude_1/cap_top/piece:0/piece:3"), "extrude_1/cap_top");
    EXPECT_EQ(wholeFaceName("box/top/pattern_2:1/piece:0"), "box/top/pattern_2:1")
        << "a pattern's copy is a face of its own";
    EXPECT_EQ(wholeFaceName("box/top/piece:1/pattern_2:1"), "box/top/pattern_2:1");
    EXPECT_EQ(wholeFaceName("revolve_1/side:e4/facet:2"), "revolve_1/side:e4/facet:2");
}

TEST(FacePlaneTest, ABoxsFacesFaceOutOfIt) {
    const auto box = PrimitiveFactory::makeBox(10, 20, 30);
    const auto top = planeOfFace(*box, "box/top");
    if (!top) FAIL() << "no plane for the top";
    EXPECT_NEAR(top->normal.z, 1.0, 1e-12);
    EXPECT_NEAR(top->origin.x, 5.0, 1e-12);
    EXPECT_NEAR(top->origin.y, 10.0, 1e-12);
    EXPECT_NEAR(top->origin.z, 30.0, 1e-12);
    const auto bottom = planeOfFace(*box, "box/bottom");
    if (!bottom) FAIL() << "no plane for the bottom";
    EXPECT_NEAR(bottom->normal.z, -1.0, 1e-12) << "out of the part, not along the loop";
    EXPECT_NEAR(bottom->origin.z, 0.0, 1e-12);
}

// A part far from the origin for its size faces out as it does at the
// origin. Its winding was measured about the origin, where the rounding of
// each term outweighed the part's own volume and turned the sign.
TEST(FacePlaneTest, APartFarFromTheOriginFacesOut) {
    const auto near = PrimitiveFactory::makeBox(1, 1, 1);
    const Mat4 turn = Mat4::rotationX(0.3) * Mat4::rotationZ(0.7);
    for (const double far : {1e6, 1e7}) {
        const Mat4 place = Mat4::translation(Vec3(0.3, 0.7, 0.1) * far) * turn;
        const auto moved = hz::model::Pattern::transformed(*near, place);
        EXPECT_EQ(hz::model::outwardSign(*moved), hz::model::outwardSign(*near)) << far;
        const auto top = planeOfFace(*moved, "box/top");
        if (!top) FAIL() << "no plane for the top";
        EXPECT_NEAR(top->normal.dot(turn.transformDirection(Vec3::UnitZ)), 1.0, 1e-9) << far;
    }
}

TEST(FacePlaneTest, AFaceThatIsGoneOrCurvedIsSaidSo) {
    const auto box = PrimitiveFactory::makeBox(10, 20, 30);
    std::string why;
    EXPECT_FALSE(planeOfFace(*box, "box/nosuch", &why).has_value());
    EXPECT_EQ(why, "is not there");

    const auto cylinder = PrimitiveFactory::makeCylinder(5.0, 12.0);
    why.clear();
    EXPECT_FALSE(planeOfFace(*cylinder, "cylinder/side0", &why).has_value())
        << "a facet of its side is flat, and a facet of a curved face";
    EXPECT_EQ(why, "is no longer flat");
    EXPECT_TRUE(planeOfFace(*cylinder, "cylinder/top").has_value());
}

// A groove cut across a block's top splits it in two: the face named whole
// is both pieces, in one plane.
TEST(FacePlaneTest, ASplitFaceIsFoundWhole) {
    const auto block = PrimitiveFactory::makeBox(10, 10, 5);
    auto cutter = hz::model::Pattern::transformed(*PrimitiveFactory::makeBox(2, 12, 5),
                                                  Mat4::translation(Vec3(4, -1, 3)));
    int k = 0;
    for (auto& face : cutter->faces()) {
        face.topoId = hz::topo::TopologyID::fromTag("cutter/f" + std::to_string(k++));
    }
    const auto grooved =
        hz::model::BooleanOp::execute(*block, *cutter, hz::model::BooleanType::Subtract, nullptr,
                                      hz::model::NamingScheme::FromGeometry);
    ASSERT_NE(grooved, nullptr);
    int pieces = 0;
    for (const auto& face : grooved->faces()) {
        if (wholeFaceName(face.topoId.tag()) == "box/top") ++pieces;
    }
    ASSERT_EQ(pieces, 2) << "the top in two";
    const auto top = planeOfFace(*grooved, "box/top");
    if (!top) FAIL() << "no plane for the top in two";
    EXPECT_NEAR(top->normal.z, 1.0, 1e-9);
    EXPECT_NEAR(top->origin.z, 5.0, 1e-9);
}
