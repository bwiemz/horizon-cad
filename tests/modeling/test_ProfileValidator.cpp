#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftEllipse.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftPolyline.h"
#include "horizon/drafting/DraftRectangle.h"
#include "horizon/modeling/ProfileValidator.h"

using namespace hz::model;
using namespace hz::draft;
using hz::math::Vec2;

// ---------------------------------------------------------------------------
// Rectangle (4 lines) is closed
// ---------------------------------------------------------------------------

TEST(ProfileValidatorTest, RectangleIsClosed) {
    std::vector<std::shared_ptr<DraftEntity>> profile;
    profile.push_back(std::make_shared<DraftLine>(Vec2(0, 0), Vec2(2, 0)));
    profile.push_back(std::make_shared<DraftLine>(Vec2(2, 0), Vec2(2, 3)));
    profile.push_back(std::make_shared<DraftLine>(Vec2(2, 3), Vec2(0, 3)));
    profile.push_back(std::make_shared<DraftLine>(Vec2(0, 3), Vec2(0, 0)));

    auto result = ProfileValidator::validate(profile);
    EXPECT_TRUE(result.isClosed) << result.errorMessage;
    EXPECT_EQ(result.orderedEdges.size(), 4u);
    EXPECT_TRUE(result.errorMessage.empty());
}

// ---------------------------------------------------------------------------
// Single circle is closed
// ---------------------------------------------------------------------------

TEST(ProfileValidatorTest, SingleCircleIsClosed) {
    std::vector<std::shared_ptr<DraftEntity>> profile;
    profile.push_back(std::make_shared<DraftCircle>(Vec2(5, 5), 3.0));

    auto result = ProfileValidator::validate(profile);
    EXPECT_TRUE(result.isClosed);
    EXPECT_EQ(result.orderedEdges.size(), 1u);
}

// ---------------------------------------------------------------------------
// Open chain (2 lines) is not closed
// ---------------------------------------------------------------------------

TEST(ProfileValidatorTest, OpenChainIsNotClosed) {
    std::vector<std::shared_ptr<DraftEntity>> profile;
    profile.push_back(std::make_shared<DraftLine>(Vec2(0, 0), Vec2(1, 0)));
    profile.push_back(std::make_shared<DraftLine>(Vec2(1, 0), Vec2(1, 1)));

    auto result = ProfileValidator::validate(profile);
    EXPECT_FALSE(result.isClosed);
    EXPECT_FALSE(result.errorMessage.empty());
}

// ---------------------------------------------------------------------------
// Empty profile is not closed
// ---------------------------------------------------------------------------

TEST(ProfileValidatorTest, EmptyProfileIsNotClosed) {
    std::vector<std::shared_ptr<DraftEntity>> profile;

    auto result = ProfileValidator::validate(profile);
    EXPECT_FALSE(result.isClosed);
    EXPECT_FALSE(result.errorMessage.empty());
}

// ---------------------------------------------------------------------------
// Unordered rectangle (shuffled lines) still chains correctly
// ---------------------------------------------------------------------------

TEST(ProfileValidatorTest, UnorderedRectangleChains) {
    std::vector<std::shared_ptr<DraftEntity>> profile;
    // Deliberately out of order and some reversed.
    profile.push_back(std::make_shared<DraftLine>(Vec2(0, 0), Vec2(2, 0)));
    profile.push_back(std::make_shared<DraftLine>(Vec2(0, 3), Vec2(2, 3)));  // reversed
    profile.push_back(std::make_shared<DraftLine>(Vec2(2, 0), Vec2(2, 3)));
    profile.push_back(std::make_shared<DraftLine>(Vec2(0, 3), Vec2(0, 0)));

    auto result = ProfileValidator::validate(profile);
    EXPECT_TRUE(result.isClosed) << result.errorMessage;
    EXPECT_EQ(result.orderedEdges.size(), 4u);
}

// ---------------------------------------------------------------------------
// Triangle (3 lines) is closed
// ---------------------------------------------------------------------------

TEST(ProfileValidatorTest, TriangleIsClosed) {
    std::vector<std::shared_ptr<DraftEntity>> profile;
    profile.push_back(std::make_shared<DraftLine>(Vec2(0, 0), Vec2(3, 0)));
    profile.push_back(std::make_shared<DraftLine>(Vec2(3, 0), Vec2(1.5, 2.6)));
    profile.push_back(std::make_shared<DraftLine>(Vec2(1.5, 2.6), Vec2(0, 0)));

    auto result = ProfileValidator::validate(profile);
    EXPECT_TRUE(result.isClosed) << result.errorMessage;
    EXPECT_EQ(result.orderedEdges.size(), 3u);
}

// ---------------------------------------------------------------------------
// Shapes drawn with the Rectangle and Polyline tools are profiles too
// ---------------------------------------------------------------------------

TEST(ProfileValidatorTest, ARectangleIsAClosedProfileOfFourLines) {
    std::vector<std::shared_ptr<hz::draft::DraftEntity>> profile{
        std::make_shared<hz::draft::DraftRectangle>(hz::math::Vec2(0, 0), hz::math::Vec2(4, 3))};
    auto result = ProfileValidator::validate(profile);
    ASSERT_TRUE(result.isClosed) << result.errorMessage;
    EXPECT_EQ(result.orderedEdges.size(), 4u);
}

TEST(ProfileValidatorTest, AClosedPolylineIsAProfileAndAnOpenOneSaysWhere) {
    std::vector<hz::math::Vec2> pts{{0, 0}, {5, 0}, {5, 5}, {0, 5}};
    std::vector<std::shared_ptr<hz::draft::DraftEntity>> closed{
        std::make_shared<hz::draft::DraftPolyline>(pts, true)};
    auto ok = ProfileValidator::validate(closed);
    ASSERT_TRUE(ok.isClosed) << ok.errorMessage;
    EXPECT_EQ(ok.orderedEdges.size(), 4u);

    std::vector<std::shared_ptr<hz::draft::DraftEntity>> open{
        std::make_shared<hz::draft::DraftPolyline>(pts, false)};
    auto bad = ProfileValidator::validate(open);
    EXPECT_FALSE(bad.isClosed);
    EXPECT_NE(bad.errorMessage.find("(0, 0)"), std::string::npos) << bad.errorMessage;
    EXPECT_NE(bad.errorMessage.find("(0, 5)"), std::string::npos) << bad.errorMessage;
}

TEST(ProfileValidatorTest, UnsupportedEntitiesAreNamed) {
    std::vector<std::shared_ptr<hz::draft::DraftEntity>> ellipse{
        std::make_shared<hz::draft::DraftEllipse>(hz::math::Vec2(0, 0), 3.0, 1.0, 0.0)};
    auto result = ProfileValidator::validate(ellipse);
    EXPECT_FALSE(result.isClosed);
    EXPECT_NE(result.errorMessage.find("an ellipse"), std::string::npos) << result.errorMessage;
    EXPECT_EQ(result.errorMessage.find("circle cannot"), std::string::npos)
        << "an ellipse is not blamed on a circle: " << result.errorMessage;

    std::vector<std::shared_ptr<hz::draft::DraftEntity>> circleAndLine{
        std::make_shared<hz::draft::DraftLine>(hz::math::Vec2(0, 0), hz::math::Vec2(1, 0)),
        std::make_shared<hz::draft::DraftCircle>(hz::math::Vec2(5, 5), 1.0)};
    auto mixed = ProfileValidator::validate(circleAndLine);
    EXPECT_NE(mixed.errorMessage.find("a circle cannot be joined"), std::string::npos)
        << mixed.errorMessage;
}

// ---------------------------------------------------------------------------
// A profile that crosses itself bounds no single region (Phase 105)
// ---------------------------------------------------------------------------

namespace {

std::vector<std::shared_ptr<DraftEntity>> loop(const std::vector<Vec2>& points) {
    std::vector<std::shared_ptr<DraftEntity>> profile;
    for (size_t i = 0; i < points.size(); ++i) {
        profile.push_back(std::make_shared<DraftLine>(points[i], points[(i + 1) % points.size()]));
    }
    return profile;
}

}  // namespace

TEST(ProfileValidatorTest, ABowTieCrossesItself) {
    auto result = ProfileValidator::validate(loop({{0, 0}, {10, 10}, {10, 0}, {0, 10}}));
    EXPECT_FALSE(result.isClosed);
    EXPECT_EQ(result.errorMessage, "the profile crosses itself at (5, 5)");
}

TEST(ProfileValidatorTest, ALoopThatTouchesItselfIsRefused) {
    // Two squares meeting at the corner (1, 1), drawn as one loop through it.
    auto result = ProfileValidator::validate(
        loop({{0, 0}, {1, 0}, {1, 1}, {2, 1}, {2, 2}, {1, 2}, {1, 1}, {0, 1}}));
    EXPECT_FALSE(result.isClosed);
    EXPECT_NE(result.errorMessage.find("crosses itself at (1, 1)"), std::string::npos)
        << result.errorMessage;
}

TEST(ProfileValidatorTest, ConcaveShapesAndArcsAreNotCrossings) {
    // An L: concave, but simple.
    EXPECT_TRUE(ProfileValidator::validate(loop({{0, 0}, {4, 0}, {4, 1}, {1, 1}, {1, 3}, {0, 3}}))
                    .isClosed);

    // A slot: two lines joined by half-circles, listed out of order.
    const double pi = std::acos(-1.0);
    std::vector<std::shared_ptr<DraftEntity>> slot;
    slot.push_back(std::make_shared<DraftLine>(Vec2(0, 0), Vec2(10, 0)));
    slot.push_back(std::make_shared<DraftLine>(Vec2(10, 2), Vec2(0, 2)));
    slot.push_back(std::make_shared<DraftArc>(Vec2(10, 1), 1.0, -pi / 2, pi / 2));
    slot.push_back(std::make_shared<DraftArc>(Vec2(0, 1), 1.0, pi / 2, 3 * pi / 2));
    const auto result = ProfileValidator::validate(slot);
    EXPECT_TRUE(result.isClosed) << result.errorMessage;
}

TEST(ProfileValidatorTest, AnArcThatCutsAcrossTheProfileIsACrossing) {
    // A square whose top is an arc bulging down through its bottom edge.
    const double pi = std::acos(-1.0);
    std::vector<std::shared_ptr<DraftEntity>> profile;
    profile.push_back(std::make_shared<DraftLine>(Vec2(0, 0), Vec2(4, 0)));
    profile.push_back(std::make_shared<DraftLine>(Vec2(4, 0), Vec2(4, 1)));
    profile.push_back(std::make_shared<DraftLine>(Vec2(0, 1), Vec2(0, 0)));
    // From (4, 1) round below to (0, 1): centre (2, 1), radius 2, through (2, -1).
    profile.push_back(std::make_shared<DraftArc>(Vec2(2, 1), 2.0, pi, 2 * pi));
    const auto result = ProfileValidator::validate(profile);
    EXPECT_FALSE(result.isClosed);
    EXPECT_NE(result.errorMessage.find("crosses itself"), std::string::npos) << result.errorMessage;
}
