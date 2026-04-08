#include "horizon/modeling/ProfileValidator.h"

#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftLine.h"

#include <gtest/gtest.h>
#include <memory>
#include <vector>

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
