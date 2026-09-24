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
    EXPECT_EQ(typed.prompt(), "  Point: @5<90");
    EXPECT_FALSE(typed.key(Qt::Key_A)) << "not a key of a point";
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
    EXPECT_EQ(typed.prompt(), "  Point: 1");
    EXPECT_FALSE(typed.key(Qt::Key_Backspace) && typed.key(Qt::Key_Backspace))
        << "Backspace with nothing typed is not the point's";
}
