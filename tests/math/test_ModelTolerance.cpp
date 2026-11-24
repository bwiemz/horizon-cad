#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "horizon/math/ModelTolerance.h"

using hz::math::ModelTolerance;

TEST(ModelTolerance, ScalesWithThePartsSize) {
    EXPECT_DOUBLE_EQ(ModelTolerance::forExtent(100.0).linear, 1e-5);
    EXPECT_DOUBLE_EQ(ModelTolerance::forExtent(1e-3).linear, 1e-10);
    // Never finer than 64 units in the last place of the extent.
    const double huge = 1e300;
    EXPECT_GE(ModelTolerance::forExtent(huge).linear, 64 * (std::nextafter(huge, INFINITY) - huge));
}

TEST(ModelTolerance, AMeaninglessExtentIsTakenAsOne) {
    for (double e : {0.0, -5.0, std::nan(""), std::numeric_limits<double>::infinity()})
        EXPECT_DOUBLE_EQ(ModelTolerance::forExtent(e).linear, 1e-7) << e;
}

TEST(ModelTolerance, EntitiesCoincideWhenTheirTolerancesMeet) {
    const ModelTolerance t = ModelTolerance::forExtent(10.0);  // 1e-6
    EXPECT_DOUBLE_EQ(t.effective(0.0), 1e-6);
    EXPECT_DOUBLE_EQ(t.effective(1e-4), 1e-4);
    // The bounds are inclusive; probe just either side of them.
    EXPECT_TRUE(t.within(0.99e-6));
    EXPECT_FALSE(t.within(1.01e-6));
    EXPECT_FALSE(t.within(1.1e-6));
    EXPECT_TRUE(t.coincident(1.99e-6));            // 1e-6 + 1e-6
    EXPECT_TRUE(t.coincident(1.005e-4, 1e-4, 0));  // 1e-4 + 1e-6
    EXPECT_FALSE(t.coincident(1.5e-4, 1e-4, 0));
    EXPECT_FALSE(t.coincident(2.01e-6));
    EXPECT_FALSE(t.coincident(2.1e-6));
}
