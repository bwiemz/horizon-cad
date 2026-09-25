// A point typed for a drawing tool (Phase 128): absolute, relative, polar, or
// a length toward the cursor, refused whole when it is none of them.

#include <gtest/gtest.h>

#include <Qt>
#include <cmath>
#include <optional>
#include <string>

#include "horizon/ui/TypedPoint.h"

using hz::math::Vec2;
using hz::ui::TypedPoint;

namespace {

bool near(const std::optional<Vec2>& p, const Vec2& q) {
    return p && (*p - q).length() < 1e-9;
}

std::optional<Vec2> resolve(const std::string& text, std::optional<Vec2> base = Vec2(10, 20),
                            std::optional<Vec2> toward = Vec2(10, 30)) {
    return TypedPoint::resolve(text, base, toward);
}

}  // namespace

TEST(TypedPointTest, TheFourWaysToTypeAPoint) {
    EXPECT_TRUE(near(resolve("3,4"), Vec2(3, 4))) << "absolute";
    EXPECT_TRUE(near(resolve("-2.5,.5"), Vec2(-2.5, 0.5)));
    EXPECT_TRUE(near(resolve("@3,-4"), Vec2(13, 16))) << "relative to the last point";
    EXPECT_TRUE(near(resolve("@5<90"), Vec2(10, 25))) << "polar from the last point";
    EXPECT_TRUE(near(resolve("@2<180"), Vec2(8, 20)));
    EXPECT_TRUE(near(resolve("4<0"), Vec2(4, 0))) << "polar from the origin";
    EXPECT_TRUE(near(resolve("7"), Vec2(10, 27))) << "a length toward the cursor";
}

TEST(TypedPointTest, WhatIsNotAPointIsRefusedWhole) {
    std::string why;
    for (const char* text : {"", ",", "3,", ",4", "3,4,5", "1.2.3,4", "@", "@5", "5<", "<30",
                             "@5<30<1", "-", ".", "1e999,0"}) {
        EXPECT_FALSE(TypedPoint::resolve(text, Vec2(0, 0), Vec2(1, 0), &why)) << text;
        EXPECT_FALSE(why.empty()) << text;
    }
    EXPECT_FALSE(TypedPoint::resolve("@1,1", std::nullopt, std::nullopt, &why))
        << "relative with no last point";
    EXPECT_FALSE(TypedPoint::resolve("5", std::nullopt, Vec2(1, 0), &why))
        << "a length with no last point";
    EXPECT_FALSE(TypedPoint::resolve("5", Vec2(1, 1), Vec2(1, 1), &why))
        << "a length with the cursor on the last point: no direction";
}

TEST(TypedPointTest, KeysBuildTheTextAndEnterTakesOrRefusesIt) {
    TypedPoint typed;
    for (int key : {Qt::Key_At, Qt::Key_5, Qt::Key_Less, Qt::Key_9, Qt::Key_0}) {
        EXPECT_TRUE(typed.key(key));
    }
    EXPECT_EQ(typed.text(), "@5<90");
    EXPECT_EQ(typed.prompt(), "  Point (mm): @5<90");
    EXPECT_FALSE(TypedPoint().key(Qt::Key_A)) << "a letter first is the view's, a shortcut";
    EXPECT_TRUE(typed.key(Qt::Key_Backspace));
    EXPECT_EQ(typed.text(), "@5<9");
    EXPECT_TRUE(near(typed.take(Vec2(0, 0), std::nullopt),
                     Vec2(5 * std::cos(9 * 3.14159265358979323846 / 180),
                          5 * std::sin(9 * 3.14159265358979323846 / 180))));
    EXPECT_FALSE(typed.typing());
    EXPECT_EQ(typed.prompt(), "");

    for (int key : {Qt::Key_3, Qt::Key_Comma, Qt::Key_Comma, Qt::Key_4}) typed.key(key);
    EXPECT_FALSE(typed.take(Vec2(0, 0), std::nullopt));
    EXPECT_NE(typed.prompt().find("'3,,4' is not a point"), std::string::npos) << typed.prompt();
    EXPECT_TRUE(typed.key(Qt::Key_1)) << "typing again drops the refusal";
    EXPECT_EQ(typed.prompt(hz::math::LengthUnit::Inch), "  Point (in): 1");
    EXPECT_FALSE(typed.key(Qt::Key_Backspace) && typed.key(Qt::Key_Backspace))
        << "Backspace with nothing typed is not the point's";
}

// Phase 154: each length in the document's unit, or its own; each angle in
// degrees, or radians.
TEST(TypedPointTest, LengthsAndAnglesMayCarryTheirUnits) {
    using hz::math::LengthUnit;
    const auto inInches = [](const std::string& text) {
        return TypedPoint::resolve(text, Vec2(10, 20), Vec2(10, 30), nullptr, LengthUnit::Inch);
    };
    EXPECT_TRUE(near(inInches("1,2"), Vec2(25.4, 50.8))) << "bare: the document's unit";
    EXPECT_TRUE(near(inInches("25.4 mm,2"), Vec2(25.4, 50.8)));
    EXPECT_TRUE(near(inInches("@1' 6\",0"), Vec2(10 + 457.2, 20)));
    EXPECT_TRUE(near(inInches("@50.8 mm<90 deg"), Vec2(10, 70.8)));
    EXPECT_TRUE(near(inInches("@1<3.14159265358979323846 rad"), Vec2(10 - 25.4, 20)));
    EXPECT_TRUE(near(inInches("1 cm"), Vec2(10, 30))) << "a length toward the cursor";
    std::string why;
    EXPECT_FALSE(TypedPoint::resolve("2 furlongs,0", Vec2(0, 0), std::nullopt, &why));
    EXPECT_FALSE(TypedPoint::resolve("@5<30 in", Vec2(0, 0), std::nullopt, &why))
        << "an angle is not a length";

    // Typed as keys: a unit after a number, its degree sign taken back whole.
    TypedPoint typed;
    for (int key : {Qt::Key_2, Qt::Key_Space, Qt::Key_I, Qt::Key_N, Qt::Key_Comma, Qt::Key_1,
                    Qt::Key_Less, Qt::Key_9, Qt::Key_0, Qt::Key_degree}) {
        EXPECT_TRUE(typed.key(key)) << key;
    }
    EXPECT_EQ(typed.text(), "2 in,1<90\xC2\xB0");
    EXPECT_TRUE(typed.key(Qt::Key_Backspace));
    EXPECT_EQ(typed.text(), "2 in,1<90");
}
