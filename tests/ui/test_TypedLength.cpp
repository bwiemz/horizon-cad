// A length typed at a tool while it runs (Phase 125): the whole text is the
// number, or it is refused and the prompt says so.

#include <gtest/gtest.h>

#include <QKeyEvent>
#include <string>

#include "horizon/ui/ChamferTool.h"
#include "horizon/ui/FilletTool.h"
#include "horizon/ui/TypedLength.h"

using hz::ui::TypedLength;

namespace {

void type(TypedLength& input, const std::string& text) {
    for (char c : text) input.key(c == '.' ? Qt::Key_Period : Qt::Key_0 + (c - '0'));
}

void type(hz::ui::Tool& tool, const std::string& text, Qt::Key last = Qt::Key_Return) {
    for (char c : text) {
        QKeyEvent press(QEvent::KeyPress, c == '.' ? Qt::Key_Period : Qt::Key_0 + (c - '0'),
                        Qt::NoModifier);
        tool.keyPressEvent(&press);
    }
    QKeyEvent enter(QEvent::KeyPress, last, Qt::NoModifier);
    tool.keyPressEvent(&enter);
}

}  // namespace

TEST(TypedLengthTest, OnlyAWholePositiveNumberIsALength) {
    EXPECT_EQ(TypedLength::parse("2.5"), 2.5);
    EXPECT_EQ(TypedLength::parse(".5"), 0.5);
    EXPECT_EQ(TypedLength::parse("3."), 3.0);
    EXPECT_FALSE(TypedLength::parse(""));
    EXPECT_FALSE(TypedLength::parse("."));
    EXPECT_FALSE(TypedLength::parse("1.2.3")) << "not 1.2, the part of it that parses";
    EXPECT_FALSE(TypedLength::parse("0"));
    EXPECT_FALSE(TypedLength::parse("1e999")) << "out of range";
}

TEST(TypedLengthTest, EnterTakesTheLengthOrSaysWhyNot) {
    TypedLength radius(1.0);
    type(radius, "1.2.3");
    EXPECT_EQ(radius.prompt("radius"), "  Radius (mm): 1.2.3");
    EXPECT_TRUE(radius.key(Qt::Key_Return));
    EXPECT_DOUBLE_EQ(radius.value(), 1.0);
    EXPECT_EQ(radius.prompt("radius"), "  '1.2.3' is not a radius");

    type(radius, "25");
    EXPECT_TRUE(radius.key(Qt::Key_Backspace));
    type(radius, ".5");
    EXPECT_TRUE(radius.key(Qt::Key_Enter));
    EXPECT_DOUBLE_EQ(radius.value(), 2.5);
    EXPECT_EQ(radius.prompt("radius"), "  [radius=2.5 mm]");

    EXPECT_TRUE(radius.key(Qt::Key_Return)) << "Enter with nothing typed is still the input's";
    EXPECT_FALSE(radius.key(Qt::Key_Backspace)) << "nothing to take back";
    EXPECT_FALSE(radius.key(Qt::Key_A));
}

// The tools read "1.2.3" as 1.2, and threw away "." without a word.
TEST(TypedLengthTest, ChamferAndFilletRefuseWhatIsNotALength) {
    hz::ui::ChamferTool chamfer;
    type(chamfer, "1.2.3");
    EXPECT_NE(chamfer.promptText().find("'1.2.3' is not a distance"), std::string::npos)
        << chamfer.promptText();
    type(chamfer, "4");
    EXPECT_NE(chamfer.promptText().find("[distance=4 mm]"), std::string::npos);

    hz::ui::FilletTool fillet;
    type(fillet, ".");
    EXPECT_NE(fillet.promptText().find("'.' is not a radius"), std::string::npos)
        << fillet.promptText();
    type(fillet, "0.75", Qt::Key_Enter);
    EXPECT_NE(fillet.promptText().find("[radius=0.75 mm]"), std::string::npos);
}

// Phase 154: a radius in the document's unit, or typed with its own.
TEST(TypedLengthTest, ARadiusMayCarryItsUnit) {
    using hz::math::LengthUnit;
    EXPECT_EQ(TypedLength::parse("1", LengthUnit::Inch), 25.4);
    EXPECT_EQ(TypedLength::parse("3 mm", LengthUnit::Inch), 3.0);
    EXPECT_FALSE(TypedLength::parse("-1 in", LengthUnit::Inch)) << "a length is positive";

    TypedLength radius(1.0);
    for (int key : {Qt::Key_1, Qt::Key_Space, Qt::Key_C, Qt::Key_M}) {
        EXPECT_TRUE(radius.key(key, LengthUnit::Inch));
    }
    EXPECT_EQ(radius.prompt("radius", LengthUnit::Inch), "  Radius (in): 1 cm");
    EXPECT_TRUE(radius.key(Qt::Key_Return, LengthUnit::Inch));
    EXPECT_DOUBLE_EQ(radius.value(), 10.0);
    EXPECT_EQ(radius.prompt("radius", LengthUnit::Inch), "  [radius=0.393701 in]");
}
