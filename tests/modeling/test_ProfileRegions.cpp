// Profiles with holes and separate regions (Phase 131): the loops a sketch's
// curves make, nested by the even-odd rule, and extruded and revolved with
// their holes cut.

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "horizon/drafting/BlockDefinition.h"
#include "horizon/drafting/DraftBlockRef.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftLinearDimension.h"
#include "horizon/drafting/DraftRectangle.h"
#include "horizon/drafting/DraftText.h"
#include "horizon/drafting/SketchPlane.h"
#include "horizon/math/Constants.h"
#include "horizon/modeling/Extrude.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/modeling/ProfileValidator.h"
#include "horizon/modeling/Revolve.h"
#include "horizon/topology/Solid.h"

using hz::draft::DraftCircle;
using hz::draft::DraftEntity;
using hz::draft::DraftLine;
using hz::draft::DraftRectangle;
using hz::draft::SketchPlane;
using hz::math::kPi;
using hz::math::Vec2;
using hz::math::Vec3;
using hz::model::ProfileValidator;
using Profile = std::vector<std::shared_ptr<DraftEntity>>;

namespace {

std::shared_ptr<DraftEntity> rect(double x0, double y0, double x1, double y1) {
    return std::make_shared<DraftRectangle>(Vec2(x0, y0), Vec2(x1, y1));
}

std::shared_ptr<DraftEntity> circle(double x, double y, double r) {
    return std::make_shared<DraftCircle>(Vec2(x, y), r);
}

/// The area of a circle as the kernel facets it: a regular polygon of @p n
/// sides inscribed in it.
double facetedCircleArea(double r, int n) {
    return 0.5 * n * r * r * std::sin(2.0 * kPi / n);
}

double volumeOf(const hz::topo::Solid& solid) {
    return hz::model::MassPropertiesCalculator::compute(solid).volume;
}

}  // namespace

TEST(ProfileRegionsTest, LoopsInsideLoopsAreHolesAndIslands) {
    // A plate with a slot and two holes; an island inside the slot.
    const Profile plate = {rect(0, 0, 100, 60), circle(20, 30, 5), rect(40, 10, 90, 50),
                           circle(10, 50, 3), rect(55, 20, 75, 40)};
    const auto found = ProfileValidator::regions(plate);
    ASSERT_TRUE(found.ok()) << found.errorMessage;
    ASSERT_EQ(found.regions.size(), 2u) << "the plate, and the island in its slot";
    EXPECT_EQ(found.regions[0].holes.size(), 3u) << "two holes and the slot";
    EXPECT_TRUE(found.regions[1].holes.empty());
    EXPECT_FALSE(found.isSingleLoop());

    const auto single = ProfileValidator::regions({rect(0, 0, 1, 1)});
    EXPECT_TRUE(single.isSingleLoop());
}

TEST(ProfileRegionsTest, SeparateLoopsAreSeparateRegions) {
    const auto found = ProfileValidator::regions({rect(0, 0, 1, 1), circle(5, 5, 1)});
    ASSERT_TRUE(found.ok()) << found.errorMessage;
    EXPECT_EQ(found.regions.size(), 2u);
}

// Notes on a sketch are not its shape: they used to make any profile fail.
TEST(ProfileRegionsTest, NotesOnTheSketchArePassedOver) {
    const Profile noted = {rect(0, 0, 10, 10),
                           std::make_shared<hz::draft::DraftText>(Vec2(2, 2), "PLATE", 1.0),
                           std::make_shared<hz::draft::DraftLinearDimension>(
                               Vec2(0, 0), Vec2(10, 0), Vec2(5, -3),
                               hz::draft::DraftLinearDimension::Orientation::Horizontal)};
    const auto found = ProfileValidator::regions(noted);
    ASSERT_TRUE(found.ok()) << found.errorMessage;
    EXPECT_TRUE(found.isSingleLoop());
}

TEST(ProfileRegionsTest, WhatBoundsNoRegionIsRefusedWithWhere) {
    const auto crossing = ProfileValidator::regions({rect(0, 0, 10, 10), rect(5, 5, 15, 15)});
    EXPECT_FALSE(crossing.ok());
    EXPECT_NE(crossing.errorMessage.find("two loops of the profile meet at"), std::string::npos)
        << crossing.errorMessage;

    const auto open = ProfileValidator::regions(
        {rect(0, 0, 10, 10), std::make_shared<DraftLine>(Vec2(20, 0), Vec2(30, 0))});
    EXPECT_FALSE(open.ok());
    EXPECT_NE(open.errorMessage.find("(20, 0)"), std::string::npos) << open.errorMessage;

    auto block = std::make_shared<hz::draft::BlockDefinition>();
    block->name = "B";
    const auto ref = ProfileValidator::regions(
        {rect(0, 0, 1, 1), std::make_shared<hz::draft::DraftBlockRef>(block, Vec2(5, 5))});
    EXPECT_FALSE(ref.ok());
    EXPECT_NE(ref.errorMessage.find("block reference"), std::string::npos) << ref.errorMessage;

    EXPECT_FALSE(ProfileValidator::regions({}).ok());
    EXPECT_FALSE(
        ProfileValidator::regions({std::make_shared<hz::draft::DraftText>(Vec2(0, 0), "x", 1.0)})
            .ok())
        << "notes alone are no profile";
}

// A plate with two holes, extruded: the plate's volume less the holes', a
// closed solid of genus two, the hole walls named after their circles.
TEST(ProfileRegionsTest, APlateWithHolesExtrudes) {
    auto hole = circle(20, 30, 5);
    const Profile plate = {rect(0, 0, 100, 60), hole, circle(70, 30, 8)};
    std::string why;
    const auto solid =
        hz::model::Extrude::execute(plate, SketchPlane(), Vec3(0, 0, 1), 10.0, "extrude_1", 32, 0.0,
                                    &why, hz::model::NamingScheme::FromGeometry);
    ASSERT_NE(solid, nullptr) << why;
    EXPECT_TRUE(solid->checkManifold());
    EXPECT_TRUE(solid->checkEulerFormula());
    const double expected =
        (100.0 * 60.0 - facetedCircleArea(5, 32) - facetedCircleArea(8, 32)) * 10.0;
    EXPECT_NEAR(volumeOf(*solid), expected, expected * 1e-9);

    const std::string wall = "side:e" + std::to_string(hole->id());
    bool named = false;
    for (const auto& face : solid->faces()) {
        if (face.topoId.tag().find(wall) != std::string::npos) named = true;
    }
    EXPECT_TRUE(named) << "the hole's wall is named after its circle, so it can be picked again";

    // Downwards too, and with a negative distance.
    const auto down = hz::model::Extrude::execute(plate, SketchPlane(), Vec3(0, 0, 1), -10.0,
                                                  "extrude_1", 32, 0.0, &why);
    ASSERT_NE(down, nullptr) << why;
    EXPECT_NEAR(volumeOf(*down), expected, expected * 1e-9);
}

TEST(ProfileRegionsTest, SeparateRegionsExtrudeTogether) {
    std::string why;
    const auto solid =
        hz::model::Extrude::execute({rect(0, 0, 10, 10), rect(20, 0, 25, 4)}, SketchPlane(),
                                    Vec3(0, 0, 1), 2.0, "e", 32, 0.0, &why);
    ASSERT_NE(solid, nullptr) << why;
    EXPECT_NEAR(volumeOf(*solid), (100.0 + 20.0) * 2.0, 1e-9);
}

// A ring's cross-section with a window in it, revolved a quarter turn, and a
// whole turn: the window becomes a cut through the part, or a closed cavity.
TEST(ProfileRegionsTest, AProfileWithAHoleRevolves) {
    // In the XZ plane (x across, the sketch's y along world z), about z.
    const SketchPlane xz(Vec3(0, 0, 0), Vec3(0, -1, 0), Vec3(1, 0, 0));
    const Profile section = {rect(10, 0, 20, 10), rect(13, 3, 17, 7)};
    const double area = 100.0 - 16.0;  // both centroids at radius 15
    for (const double angle : {kPi / 2, 2.0 * kPi}) {
        SCOPED_TRACE(angle);
        std::string why;
        const auto solid = hz::model::Revolve::execute(section, xz, Vec3(0, 0, 0), Vec3(0, 0, 1),
                                                       angle, "revolve_1", 128, 0.0, &why);
        ASSERT_NE(solid, nullptr) << why;
        EXPECT_TRUE(solid->checkManifold());
        const double pappus = angle * 15.0 * area;
        EXPECT_NEAR(volumeOf(*solid), pappus, pappus * 0.002);
    }
}
