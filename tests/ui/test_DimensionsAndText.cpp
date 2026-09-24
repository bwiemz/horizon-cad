// Dimensions and text through the window (Phase 129): the dimension style
// form, continued and baseline dimensions, typed dimension points, and text
// of several lines, written and edited.

#include <gtest/gtest.h>

#include <QAction>
#include <QApplication>
#include <QElapsedTimer>
#include <QFocusEvent>
#include <QInputDialog>
#include <QPlainTextEdit>
#include <QTimer>
#include <memory>
#include <string>
#include <vector>

#include "UiTestSupport.h"
#include "horizon/document/Document.h"
#include "horizon/document/UndoStack.h"
#include "horizon/drafting/DimensionStyle.h"
#include "horizon/drafting/DraftLinearDimension.h"
#include "horizon/drafting/DraftText.h"
#include "horizon/drafting/Layer.h"
#include "horizon/math/Constants.h"
#include "horizon/ui/MainWindow.h"

using hz::draft::DraftLinearDimension;
using hz::math::Vec2;
using hz::test::ToolDriver;
using hz::ui::MainWindow;
using Orientation = DraftLinearDimension::Orientation;

namespace {

void trigger(MainWindow& w, const char* name) {
    auto* action = w.findChild<QAction*>(QString::fromLatin1(name));
    ASSERT_NE(action, nullptr) << name;
    action->trigger();
}

template <typename T>
std::vector<const T*> all(MainWindow& w) {
    std::vector<const T*> out;
    for (const auto& e : w.activeDocument()->draftDocument().entities()) {
        if (const auto* t = dynamic_cast<const T*>(e.get())) out.push_back(t);
    }
    return out;
}

bool near(const Vec2& a, const Vec2& b) {
    return (a - b).length() < 1e-9;
}

/// Type @p text at the viewport, key by key, then Enter.
void type(ToolDriver& drive, const std::string& text) {
    for (char c : text) {
        Qt::Key key = Qt::Key_unknown;
        if (c >= '0' && c <= '9') key = static_cast<Qt::Key>(Qt::Key_0 + (c - '0'));
        if (c == ',') key = Qt::Key_Comma;
        if (c == '.') key = Qt::Key_Period;
        if (c == '-') key = Qt::Key_Minus;
        if (c == '@') key = Qt::Key_At;
        if (c == '<') key = Qt::Key_Less;
        drive.key(key);
    }
    drive.key(Qt::Key_Return);
}

/// A horizontal dimension of 0..10 along y = 0, its line at y = 5.
const DraftLinearDimension* addFirstDimension(MainWindow& w) {
    auto dim = std::make_shared<DraftLinearDimension>(Vec2(0, 0), Vec2(10, 0), Vec2(5, 5),
                                                      Orientation::Horizontal);
    w.activeDocument()->draftDocument().addEntity(dim);
    return dim.get();
}

/// Answers the next text dialog with @p text.
class TextAnswer {
public:
    explicit TextAnswer(QString text) : m_text(std::move(text)) {
        QObject::connect(&m_timer, &QTimer::timeout, [this] { poll(); });
        m_timer.start(5);
        m_clock.start();
    }
    bool seen() const { return m_seen; }

private:
    void poll() {
        auto* dialog = qobject_cast<QInputDialog*>(QApplication::activeModalWidget());
        if (dialog == nullptr) {
            if (m_clock.elapsed() > 5000) m_timer.stop();
            return;
        }
        m_timer.stop();
        m_seen = true;
        dialog->setTextValue(m_text);
        dialog->accept();
    }

    QString m_text;
    bool m_seen = false;
    QTimer m_timer;
    QElapsedTimer m_clock;
};

}  // namespace

// The style form shows the style as it is (the arrow's angle in degrees),
// changes all of it in one step, and a dimension shows its value in the unit.
TEST(DimensionsAndTextTest, TheStyleFormChangesTheStyleInOneStep) {
    MainWindow w;
    auto& doc = *w.activeDocument();
    addFirstDimension(w);
    hz::test::FormFiller filler(QStringLiteral("Dimension Style"),
                                hz::test::FormAnswers()
                                    .number(QStringLiteral("textHeight"), 3.5)
                                    .number(QStringLiteral("arrowAngle"), 30.0)
                                    .number(QStringLiteral("precision"), 3)
                                    .choose(QStringLiteral("unit"), QStringLiteral("in"))
                                    .choose(QStringLiteral("showUnits"), QStringLiteral("Yes")));
    trigger(w, "action_dim_style");
    ASSERT_TRUE(filler.seen());
    EXPECT_NEAR(filler.shown(QStringLiteral("arrowAngle")), 0.3 * hz::math::kRadToDeg, 0.05)
        << "degrees, not the radians it is kept in";

    const auto& style = doc.draftDocument().dimensionStyle();
    EXPECT_DOUBLE_EQ(style.textHeight, 3.5);
    EXPECT_NEAR(style.arrowAngle, 30.0 * hz::math::kDegToRad, 1e-12);
    EXPECT_EQ(style.precision, 3);
    EXPECT_EQ(style.unit, "in");
    EXPECT_TRUE(style.showUnits);
    EXPECT_EQ(all<DraftLinearDimension>(w).at(0)->displayText(style), "0.394\"");

    trigger(w, "action_undo");
    EXPECT_TRUE(doc.draftDocument().dimensionStyle() == hz::draft::DimensionStyle{})
        << "one step undoes it all";
}

// OK with nothing changed pushes nothing to undo, whatever the style holds:
// the most decimal places a file may have (12) too.
TEST(DimensionsAndTextTest, AnUnchangedStyleIsNotAStep) {
    MainWindow w;
    auto style = w.activeDocument()->draftDocument().dimensionStyle();
    style.precision = 12;
    w.activeDocument()->draftDocument().setDimensionStyle(style);
    hz::test::FormFiller filler(QStringLiteral("Dimension Style"), hz::test::FormAnswers());
    trigger(w, "action_dim_style");
    ASSERT_TRUE(filler.seen());
    EXPECT_FALSE(w.activeDocument()->undoStack().canUndo());
    EXPECT_EQ(w.activeDocument()->draftDocument().dimensionStyle().precision, 12);
}

// Continue takes the last dimension and goes on from its second point, each
// new one from where the one before ended, on the same line.
TEST(DimensionsAndTextTest, ContinueGoesOnFromTheLastDimension) {
    MainWindow w;
    ToolDriver drive(w);
    addFirstDimension(w);
    trigger(w, "action_dim_continue");
    drive.click(Vec2(25, 0));
    drive.click(Vec2(30, 0));

    const auto dims = all<DraftLinearDimension>(w);
    ASSERT_EQ(dims.size(), 3u);
    EXPECT_TRUE(near(dims[1]->defPoint1(), Vec2(10, 0)));
    EXPECT_TRUE(near(dims[1]->defPoint2(), Vec2(25, 0)));
    EXPECT_TRUE(near(dims[2]->defPoint1(), Vec2(25, 0)));
    EXPECT_TRUE(near(dims[2]->defPoint2(), Vec2(30, 0)));
    for (const auto* dim : dims) {
        EXPECT_EQ(dim->orientation(), Orientation::Horizontal);
        EXPECT_NEAR(dim->dimLinePoint().y, 5.0, 1e-9) << "one line";
    }

    trigger(w, "action_undo");
    EXPECT_EQ(all<DraftLinearDimension>(w).size(), 2u) << "each dimension is its own step";
}

// Baseline measures every dimension from the first one's first point, each a
// step further out so their lines and values do not overlap.
TEST(DimensionsAndTextTest, BaselineMeasuresFromOnePointSteppingOut) {
    MainWindow w;
    ToolDriver drive(w);
    addFirstDimension(w);
    trigger(w, "action_dim_baseline");
    drive.click(Vec2(25, 0));
    drive.click(Vec2(40, 0));

    const auto dims = all<DraftLinearDimension>(w);
    ASSERT_EQ(dims.size(), 3u);
    const double step = 1.5 * w.activeDocument()->draftDocument().dimensionStyle().textHeight;
    EXPECT_TRUE(near(dims[1]->defPoint1(), Vec2(0, 0)));
    EXPECT_TRUE(near(dims[1]->defPoint2(), Vec2(25, 0)));
    EXPECT_NEAR(dims[1]->dimLinePoint().y, 5.0 + step, 1e-9);
    EXPECT_TRUE(near(dims[2]->defPoint1(), Vec2(0, 0)));
    EXPECT_TRUE(near(dims[2]->defPoint2(), Vec2(40, 0)));
    EXPECT_NEAR(dims[2]->dimLinePoint().y, 5.0 + 2 * step, 1e-9);
}

// A vertical dimension whose line is to its left steps out to the left; Enter
// lets another dimension be picked to go on from.
TEST(DimensionsAndTextTest, ChainsFollowTheDimensionPicked) {
    MainWindow w;
    ToolDriver drive(w);
    addFirstDimension(w);
    auto vertical = std::make_shared<DraftLinearDimension>(Vec2(50, 0), Vec2(50, 10), Vec2(45, 5),
                                                           Orientation::Vertical);
    w.activeDocument()->draftDocument().addEntity(vertical);

    trigger(w, "action_dim_baseline");
    drive.click(Vec2(50, 20));
    auto dims = all<DraftLinearDimension>(w);
    ASSERT_EQ(dims.size(), 3u);
    EXPECT_EQ(dims[2]->orientation(), Orientation::Vertical) << "from the last one drawn";
    EXPECT_TRUE(near(dims[2]->defPoint1(), Vec2(50, 0)));
    EXPECT_LT(dims[2]->dimLinePoint().x, 45.0) << "out, on the side its line is";

    drive.key(Qt::Key_Return);
    drive.click(Vec2(5, 5));  // on the first dimension's line: picks it
    drive.click(Vec2(20, 0));
    dims = all<DraftLinearDimension>(w);
    ASSERT_EQ(dims.size(), 4u);
    EXPECT_EQ(dims[3]->orientation(), Orientation::Horizontal);
    EXPECT_TRUE(near(dims[3]->defPoint1(), Vec2(0, 0)));
    EXPECT_TRUE(near(dims[3]->defPoint2(), Vec2(20, 0)));
}

// With no dimension to go on from, a click on nothing adds nothing.
TEST(DimensionsAndTextTest, WithNoDimensionAChainWaitsForOne) {
    MainWindow w;
    ToolDriver drive(w);
    trigger(w, "action_dim_continue");
    drive.click(Vec2(5, 5));
    drive.click(Vec2(15, 5));
    EXPECT_TRUE(all<DraftLinearDimension>(w).empty());
}

// A linear dimension's points can be typed: the second relative to the
// first, the line's position relative to the second.
TEST(DimensionsAndTextTest, ALinearDimensionTakesTypedPoints) {
    MainWindow w;
    ToolDriver drive(w);
    trigger(w, "tool_dim-linear");
    type(drive, "2,3");
    type(drive, "@12,0");
    type(drive, "@-6,4");
    const auto dims = all<DraftLinearDimension>(w);
    ASSERT_EQ(dims.size(), 1u);
    EXPECT_TRUE(near(dims[0]->defPoint1(), Vec2(2, 3)));
    EXPECT_TRUE(near(dims[0]->defPoint2(), Vec2(14, 3)));
    EXPECT_EQ(dims[0]->orientation(), Orientation::Horizontal)
        << "from the typed point, not where the cursor last was";
    EXPECT_NEAR(dims[0]->computedValue(), 12.0, 1e-9);
}

// The text tool takes several lines; the panel edits them, one step when the
// field is left, none when nothing changed.
TEST(DimensionsAndTextTest, TextOfSeveralLinesIsWrittenAndEdited) {
    MainWindow w;
    ToolDriver drive(w);
    auto& doc = *w.activeDocument();
    trigger(w, "tool_text");
    {
        TextAnswer answer(QStringLiteral("first\r\nsecond\n\n"));
        drive.click(Vec2(0, 0));
        ASSERT_TRUE(answer.seen());
    }
    auto texts = all<hz::draft::DraftText>(w);
    ASSERT_EQ(texts.size(), 1u);
    EXPECT_EQ(texts[0]->text(), "first\nsecond") << "Windows line ends and trailing ones go";

    trigger(w, "tool_select");
    drive.click(texts[0]->lineBaseline(1) + Vec2(1.0, 0.5));  // the second line picks it
    auto* field = w.findChild<QPlainTextEdit*>(QStringLiteral("textContent"));
    ASSERT_NE(field, nullptr);
    EXPECT_EQ(field->toPlainText(), QStringLiteral("first\nsecond"));

    const auto steps = doc.undoStack().revision();
    QFocusEvent leave(QEvent::FocusOut);
    QCoreApplication::sendEvent(field, &leave);
    EXPECT_EQ(doc.undoStack().revision(), steps) << "leaving the field unchanged is no edit";

    field->setPlainText(QStringLiteral("first\nsecond\nthird"));
    QCoreApplication::sendEvent(field, &leave);
    texts = all<hz::draft::DraftText>(w);
    EXPECT_EQ(texts.at(0)->text(), "first\nsecond\nthird");
    trigger(w, "action_undo");
    EXPECT_EQ(all<hz::draft::DraftText>(w).at(0)->text(), "first\nsecond");
}
