// Phase 154: a document's lengths are shown in its unit, and typed in it or
// in any other.

#include <gtest/gtest.h>

#include <QAction>
#include <QCoreApplication>
#include <QDoubleSpinBox>
#include <QKeyEvent>
#include <QLineEdit>
#include <QTreeWidget>
#include <memory>

#include "UiTestSupport.h"
#include "horizon/document/Document.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/math/Units.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/ui/FeatureTreePanel.h"
#include "horizon/ui/MainWindow.h"
#include "horizon/ui/PropertyPanel.h"
#include "horizon/ui/QuantitySpinBox.h"
#include "horizon/ui/ViewportWidget.h"

using hz::math::LengthUnit;
using hz::math::Vec2;
using hz::test::FormAnswers;
using hz::test::FormFiller;
using hz::test::ToolDriver;
using hz::ui::MainWindow;
using hz::ui::QuantitySpinBox;

namespace {

/// @p text typed into @p field, and entered.
void type(QDoubleSpinBox& field, const QString& text) {
    auto* edit = field.findChild<QLineEdit*>();
    ASSERT_NE(edit, nullptr);
    edit->setText(text);
    field.interpretText();
}

void trigger(QObject& owner, const char* name) {
    auto* action = owner.findChild<QAction*>(QString::fromLatin1(name));
    ASSERT_NE(action, nullptr) << name;
    action->trigger();
}

/// The active document put in inches, by Edit ▸ Document Units.
void inInches(MainWindow& w) {
    FormFiller units(QStringLiteral("Document Units"),
                     FormAnswers().choose(QStringLiteral("unit"), QStringLiteral("Inches (in)")));
    trigger(w, "action_document_units");
    ASSERT_TRUE(units.seen());
    ASSERT_EQ(w.activeDocument()->lengthUnit(), LengthUnit::Inch);
}

double partVolume(const hz::doc::Document& doc) {
    return doc.solid() ? hz::model::MassPropertiesCalculator::compute(*doc.solid()).volume : 0.0;
}

}  // namespace

TEST(UnitsFieldTest, ALengthIsShownInItsUnitAndTakenInAny) {
    QuantitySpinBox field(QuantitySpinBox::Kind::Length, LengthUnit::Inch, 3);
    field.setRange(0.0, 1e6);
    field.setValue(50.8);
    EXPECT_EQ(field.text(), QStringLiteral("2.000 in"));
    EXPECT_DOUBLE_EQ(field.value(), 50.8) << "its value in millimetres";

    type(field, QStringLiteral("25.4 mm"));
    EXPECT_DOUBLE_EQ(field.value(), 25.4);
    type(field, QStringLiteral("1'"));
    EXPECT_DOUBLE_EQ(field.value(), 304.8);
    type(field, QStringLiteral("3/4"));
    EXPECT_DOUBLE_EQ(field.value(), 19.05) << "a bare number in the field's unit";
    type(field, QStringLiteral("0.001"));
    EXPECT_DOUBLE_EQ(field.value(), 0.0254) << "not rounded to the places shown";

    // What is not a length, or not in range, changes nothing.
    type(field, QStringLiteral("2 furlongs"));
    EXPECT_DOUBLE_EQ(field.value(), 0.0254);
    type(field, QStringLiteral("-2 in"));
    EXPECT_DOUBLE_EQ(field.value(), 0.0254);

    // Entered as shown, it keeps its value, not the places shown.
    field.setValue(10.0);
    EXPECT_EQ(field.text(), QStringLiteral("0.394 in"));
    field.interpretText();
    EXPECT_DOUBLE_EQ(field.value(), 10.0);

    field.setValue(19.05);
    field.setUnit(LengthUnit::Millimetre);
    EXPECT_EQ(field.text(), QStringLiteral("19.050 mm"));
    EXPECT_DOUBLE_EQ(field.value(), 19.05);
}

TEST(UnitsFieldTest, AnAngleIsInDegreesAndTakesRadians) {
    QuantitySpinBox field(QuantitySpinBox::Kind::Angle, LengthUnit::Inch, 2);
    field.setRange(-360.0, 360.0);
    field.setValue(30.0);
    EXPECT_EQ(field.text(), QStringLiteral("30.00\u00B0"));
    type(field, QStringLiteral("0.5 rad"));
    EXPECT_NEAR(field.value(), 28.64788975654116, 1e-9);
    type(field, QStringLiteral("45 deg"));
    EXPECT_DOUBLE_EQ(field.value(), 45.0);
    type(field, QStringLiteral("2 in"));
    EXPECT_DOUBLE_EQ(field.value(), 45.0) << "not an angle";
}

// A box in a part in inches, its sizes typed as a user types them.
TEST(UnitsFieldTest, ABoxIsTypedInTheDocumentsUnitOrAnother) {
    MainWindow w;
    inInches(w);
    FormFiller box(QStringLiteral("Box"),
                   FormAnswers()
                       .typed(QStringLiteral("size0"), QStringLiteral("1"))
                       .typed(QStringLiteral("size1"), QStringLiteral("2 in"))
                       .typed(QStringLiteral("size2"), QStringLiteral("50.8 mm")));
    trigger(w, "action_box");
    ASSERT_TRUE(box.seen());
    EXPECT_NEAR(partVolume(*w.activeDocument()), 25.4 * 50.8 * 50.8, 1e-6);
}

// A feature edited in inches: a bare number in the edit form is one.
TEST(UnitsFieldTest, AFeatureIsEditedInTheDocumentsUnit) {
    MainWindow w;
    inInches(w);
    hz::doc::Document& doc = *w.activeDocument();
    auto& drawing = doc.draftDocument();
    const Vec2 corners[] = {{0, 0}, {25.4, 0}, {25.4, 25.4}, {0, 25.4}};
    for (int k = 0; k < 4; ++k) {
        drawing.addEntity(std::make_shared<hz::draft::DraftLine>(corners[k], corners[(k + 1) % 4]));
    }
    {
        FormFiller extrude(QStringLiteral("Extrude"),
                           FormAnswers().typed(QStringLiteral("size"), QStringLiteral("1")));
        trigger(w, "action_extrude");
        ASSERT_TRUE(extrude.seen());
    }
    ASSERT_NEAR(partVolume(doc), 25.4 * 25.4 * 25.4, 1e-6);

    auto* panel = w.findChild<hz::ui::FeatureTreePanel*>();
    ASSERT_NE(panel, nullptr);
    auto* tree = panel->findChild<QTreeWidget*>();
    ASSERT_NE(tree, nullptr);
    tree->setCurrentItem(tree->topLevelItem(0));
    {
        FormFiller edit(QStringLiteral("Edit Extrude"),
                        FormAnswers().typed(QStringLiteral("distance"), QStringLiteral("2")));
        trigger(*panel, "editFeature");
        ASSERT_TRUE(edit.seen());
    }
    EXPECT_NEAR(partVolume(doc), 25.4 * 25.4 * 50.8, 1e-6);
}

// The property panel shows a line's length in the document's unit, and
// takes a new one in any.
TEST(UnitsFieldTest, ThePropertyPanelShowsAndTakesTheDocumentsUnit) {
    MainWindow w;
    inInches(w);
    auto line = std::make_shared<hz::draft::DraftLine>(Vec2(0, 0), Vec2(25.4, 0));
    w.activeDocument()->draftDocument().addEntity(line);
    auto* panel = w.findChild<hz::ui::PropertyPanel*>();
    ASSERT_NE(panel, nullptr);
    panel->updateForSelection({line->id()});
    auto* length = panel->findChild<QDoubleSpinBox*>(QStringLiteral("lineLength"));
    ASSERT_NE(length, nullptr);
    EXPECT_EQ(length->text(), QStringLiteral("1.0000 in"));
    type(*length, QStringLiteral("50.8 mm"));
    // An edit puts an edited copy in its place, with its id.
    const auto edited = std::dynamic_pointer_cast<hz::draft::DraftLine>(
        w.activeDocument()->draftDocument().sharedEntity(line->id()));
    ASSERT_NE(edited, nullptr);
    EXPECT_NEAR(edited->end().x, 50.8, 1e-9);
}

// Rotate's typed angle takes radians too (Phase 154).
TEST(UnitsFieldTest, ARotationIsTypedInRadians) {
    MainWindow w;
    ToolDriver drive(w);
    auto line = std::make_shared<hz::draft::DraftLine>(Vec2(10, 0), Vec2(20, 0));
    w.activeDocument()->draftDocument().addEntity(line);
    drive.viewport().selectionManager().select(line->id());
    trigger(w, "tool_rotate");
    drive.click(Vec2(0, 0));  // about the origin
    for (const Qt::Key key : {Qt::Key_1, Qt::Key_Period, Qt::Key_5, Qt::Key_7, Qt::Key_0, Qt::Key_7,
                              Qt::Key_9, Qt::Key_6, Qt::Key_3, Qt::Key_2, Qt::Key_6, Qt::Key_8,
                              Qt::Key_Space, Qt::Key_R, Qt::Key_A, Qt::Key_D, Qt::Key_Return}) {
        drive.key(key);
    }
    const auto selected = drive.viewport().selectionManager().selectedIds();
    ASSERT_EQ(selected.size(), 1u);
    const auto turned = std::dynamic_pointer_cast<hz::draft::DraftLine>(
        w.activeDocument()->draftDocument().sharedEntity(selected.front()));
    ASSERT_NE(turned, nullptr);
    EXPECT_NEAR(turned->start().x, 0.0, 1e-6);
    EXPECT_NEAR(turned->start().y, 10.0, 1e-6) << "a quarter turn";
}

// A value being typed keeps its letters and spaces: the window's one-key
// shortcuts (M for Move, R for Rectangle, Space for Select) do not take
// them. Qt first offers a key to the focused widget as a ShortcutOverride;
// the viewport claims it while something is typed, and only then.
TEST(UnitsFieldTest, ALetterTypedInAValueIsNotAShortcut) {
    MainWindow w;
    ToolDriver drive(w);
    hz::ui::ViewportWidget& viewport = drive.viewport();
    const auto claims = [&viewport](Qt::Key key, Qt::KeyboardModifiers modifiers = Qt::NoModifier) {
        QKeyEvent offered(QEvent::ShortcutOverride, key, modifiers);
        offered.ignore();
        QCoreApplication::sendEvent(&viewport, &offered);
        return offered.isAccepted();
    };

    trigger(w, "tool_line");
    EXPECT_FALSE(claims(Qt::Key_M)) << "nothing typed: M is Move's";
    drive.key(Qt::Key_5);
    EXPECT_TRUE(claims(Qt::Key_M)) << "\"5 m...\": a unit";
    EXPECT_TRUE(claims(Qt::Key_Space));
    EXPECT_TRUE(claims(Qt::Key_QuoteDbl, Qt::ShiftModifier));
    EXPECT_FALSE(claims(Qt::Key_S, Qt::ControlModifier)) << "Ctrl+S saves, typing or not";
    drive.key(Qt::Key_Escape);
    EXPECT_FALSE(claims(Qt::Key_M)) << "what was typed is dropped";

    // A tool that reads its own value: a rotation's angle.
    auto line = std::make_shared<hz::draft::DraftLine>(Vec2(10, 0), Vec2(20, 0));
    w.activeDocument()->draftDocument().addEntity(line);
    viewport.selectionManager().select(line->id());
    trigger(w, "tool_rotate");
    drive.click(Vec2(0, 0));
    EXPECT_FALSE(claims(Qt::Key_R));
    drive.key(Qt::Key_1);
    EXPECT_TRUE(claims(Qt::Key_R)) << "\"1 r...\": radians";
}
