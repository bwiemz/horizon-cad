// What entities draw, and where a click or a line finds them.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#include "horizon/drafting/DraftHatch.h"

using hz::math::Vec2;
using Segments = std::vector<std::pair<Vec2, Vec2>>;

namespace {

bool near(const Vec2& a, const Vec2& b, double tol = 1e-9) {
    return (a - b).length() <= tol;
}

/// @p lines, each with its ends in order of x, sorted by y then x.
Segments sorted(Segments lines) {
    for (auto& [a, b] : lines) {
        if (b.x < a.x) std::swap(a, b);
    }
    std::sort(lines.begin(), lines.end(), [](const auto& l, const auto& r) {
        if (std::abs(l.first.y - r.first.y) > 1e-9) return l.first.y < r.first.y;
        return l.first.x < r.first.x;
    });
    return lines;
}

}  // namespace

// -- Hatching ------------------------------------------------------------------

// A hatch line through a corner of the outline counted the corner once for
// each side meeting there, and paired the crossings wrongly after it: through
// the corner of this notch, a line ran across the empty notch.
TEST(HatchTest, ALineThroughACornerStaysInside) {
    const hz::draft::DraftHatch hatch(
        {{0, 0}, {10, 0}, {10, 6}, {7, 6}, {7, 2}, {3, 2}, {2.5, 3}, {3, 6}, {0, 6}},
        hz::draft::HatchPattern::Lines, 0.0, 2.0);
    const Segments lines = sorted(hatch.generateHatchLines());
    // Lines at y = 1, 3 and 5; the one at 3 through the corner (2.5, 3).
    const Segments expected = {{{0, 1}, {10, 1}},
                               {{0, 3}, {2.5, 3}},
                               {{7, 3}, {10, 3}},
                               {{0, 5}, {2.5 + 0.5 * 2.0 / 3.0, 5}},
                               {{7, 5}, {10, 5}}};
    ASSERT_EQ(lines.size(), expected.size());
    for (size_t i = 0; i < lines.size(); ++i) {
        EXPECT_TRUE(near(lines[i].first, expected[i].first) &&
                    near(lines[i].second, expected[i].second))
            << i << ": (" << lines[i].first.x << ", " << lines[i].first.y << ") - ("
            << lines[i].second.x << ", " << lines[i].second.y << ")";
    }
}

// Through two corners on either side of a diamond, the line across its middle
// was lost: each corner counted twice made two empty lines of it.
TEST(HatchTest, ALineThroughTwoCornersIsDrawn) {
    const hz::draft::DraftHatch diamond({{0, -1}, {1, 0}, {0, 1}, {-1, 0}},
                                        hz::draft::HatchPattern::Lines, 0.0, 2.0);
    const Segments lines = sorted(diamond.generateHatchLines());
    ASSERT_EQ(lines.size(), 1u);
    EXPECT_TRUE(near(lines[0].first, Vec2(-1, 0)));
    EXPECT_TRUE(near(lines[0].second, Vec2(1, 0)));
}
