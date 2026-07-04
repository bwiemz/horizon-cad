#include <gtest/gtest.h>

#include "horizon/modeling/Sheet.h"

using hz::model::Orientation;
using hz::model::PaperSize;
using hz::model::paperSizeName;
using hz::model::Sheet;

// A3 landscape is 420 x 297 mm; portrait swaps the two.
TEST(SheetTest, A3Dimensions) {
    Sheet s;
    s.size = PaperSize::A3;
    s.orientation = Orientation::Landscape;
    EXPECT_NEAR(s.widthMm(), 420.0, 1e-9);
    EXPECT_NEAR(s.heightMm(), 297.0, 1e-9);

    s.orientation = Orientation::Portrait;
    EXPECT_NEAR(s.widthMm(), 297.0, 1e-9);
    EXPECT_NEAR(s.heightMm(), 420.0, 1e-9);
}

// ISO A sizes halve the long edge each step (A0 area ~1 m^2).
TEST(SheetTest, IsoSizesHalveDown) {
    Sheet a4;
    a4.size = PaperSize::A4;
    EXPECT_NEAR(a4.widthMm(), 297.0, 1e-9);
    EXPECT_NEAR(a4.heightMm(), 210.0, 1e-9);

    Sheet a0;
    a0.size = PaperSize::A0;
    EXPECT_NEAR(a0.widthMm(), 1189.0, 1e-9);
    EXPECT_NEAR(a0.heightMm(), 841.0, 1e-9);
}

TEST(SheetTest, Names) {
    EXPECT_EQ(paperSizeName(PaperSize::A3), "A3");
    EXPECT_EQ(paperSizeName(PaperSize::AnsiA), "ANSI A");
    EXPECT_NE(paperSizeName(PaperSize::A0), paperSizeName(PaperSize::A1));
}
