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
    ASSERT_TRUE(top.has_value());
    EXPECT_NEAR(top->normal.z, 1.0, 1e-12);
    EXPECT_NEAR(top->origin.x, 5.0, 1e-12);
    EXPECT_NEAR(top->origin.y, 10.0, 1e-12);
    EXPECT_NEAR(top->origin.z, 30.0, 1e-12);
    const auto bottom = planeOfFace(*box, "box/bottom");
    ASSERT_TRUE(bottom.has_value());
    EXPECT_NEAR(bottom->normal.z, -1.0, 1e-12) << "out of the part, not along the loop";
    EXPECT_NEAR(bottom->origin.z, 0.0, 1e-12);
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
    ASSERT_TRUE(top.has_value());
    EXPECT_NEAR(top->normal.z, 1.0, 1e-9);
    EXPECT_NEAR(top->origin.z, 5.0, 1e-9);
}
