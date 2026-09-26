// Phase 166b: every control a screen reader reaches has a name. Qt names
// most by their text (a button's, its action's) or by the label a form gives
// them; what it cannot name must be named in code.

#include <gtest/gtest.h>

#include <QAbstractButton>
#include <QAccessible>
#include <QAccessibleInterface>
#include <QApplication>
#include <QDir>
#include <QFile>
#include <QTabBar>
#include <QWidget>
#include <string>
#include <vector>

#include "horizon/ui/FeatureForm.h"
#include "horizon/ui/LocaleManager.h"
#include "horizon/ui/MainWindow.h"
#include "horizon/ui/ViewportWidget.h"

using hz::ui::MainWindow;

namespace {

/// What a screen reader would call @p widget; empty when nothing names it.
QString accessibleName(QWidget* widget) {
    QAccessibleInterface* face = QAccessible::queryAccessibleInterface(widget);
    return face != nullptr ? face->text(QAccessible::Name) : QString();
}

/// The controls under @p root that take focus or are clicked, with no name,
/// shown or not (a dock tabbed behind another is still reached): "Class
/// (objectName) in Parent (objectName) under the nearest named ancestor".
std::vector<std::string> unnamedControls(QWidget& root) {
    std::vector<std::string> out;
    for (QWidget* widget : root.findChildren<QWidget*>()) {
        // Qt's own parts of a control (a spin box's line edit, a toolbar's
        // overflow button, a combo box's popup list) are reached through it.
        if (widget->objectName().startsWith(QLatin1String("qt_")) ||
            widget->inherits("QComboBoxListView")) {
            continue;
        }
        const bool control = widget->focusPolicy() != Qt::NoFocus ||
                             qobject_cast<QAbstractButton*>(widget) != nullptr;
        if (!control || !accessibleName(widget).trimmed().isEmpty()) continue;
        std::string where = widget->metaObject()->className();
        where += " (" + widget->objectName().toStdString() + ")";
        if (QWidget* parent = widget->parentWidget()) {
            where += std::string(" in ") + parent->metaObject()->className() + " (" +
                     parent->objectName().toStdString() + ")";
        }
        for (QWidget* up = widget->parentWidget(); up != nullptr; up = up->parentWidget()) {
            if (!up->objectName().isEmpty() && up != widget->parentWidget()) {
                where += " under " + up->objectName().toStdString();
                break;
            }
        }

        out.push_back(where);
    }
    return out;
}

}  // namespace

TEST(AccessibilityTest, EveryControlOfTheWindowHasAName) {
    MainWindow w;
    w.show();
    QApplication::processEvents();
    const auto unnamed = unnamedControls(w);
    for (const auto& control : unnamed) ADD_FAILURE() << "no accessible name: " << control;

    // A tab's close button says which tab it closes.
    auto* tabs = w.findChild<QTabBar*>(QStringLiteral("documentTabs"));
    ASSERT_NE(tabs, nullptr);
    QWidget* close = tabs->tabButton(0, QTabBar::RightSide);
    if (close == nullptr) close = tabs->tabButton(0, QTabBar::LeftSide);
    ASSERT_NE(close, nullptr);
    EXPECT_EQ(accessibleName(close), QStringLiteral("Close ") + tabs->tabText(0));
}

// A form's fields are named by their labels, as Qt names a field that a form
// layout gives a label: a screen reader says "Depth:", not "spin box".
TEST(AccessibilityTest, AFormsFieldsAreNamedByTheirLabels) {
    hz::ui::FeatureForm form(nullptr, QStringLiteral("Extrude"));
    form.length(QStringLiteral("depth"), QStringLiteral("Depth:"), 10.0, 0.0, 100.0);
    form.angle(QStringLiteral("angle"), QStringLiteral("Angle:"), 30.0, 0.0, 90.0);
    form.count(QStringLiteral("count"), QStringLiteral("Instances:"), 3, 1, 10);
    form.choice(QStringLiteral("way"), QStringLiteral("Goes:"), {QStringLiteral("Up")});
    form.text(QStringLiteral("name"), QStringLiteral("Name:"));
    form.checklist(QStringLiteral("edges"), QStringLiteral("Edges:"), {{QStringLiteral("e1"), {}}});
    form.checklist(QStringLiteral("faces"), QStringLiteral("&Faces && loops &"),
                   {{QStringLiteral("f1"), {}}});
    form.dialog().show();
    QApplication::processEvents();
    const auto unnamed = unnamedControls(form.dialog());
    for (const auto& control : unnamed) ADD_FAILURE() << "no accessible name: " << control;
    const QString depth =
        accessibleName(form.dialog().findChild<QWidget*>(QStringLiteral("depth")));
    EXPECT_TRUE(depth.startsWith(QStringLiteral("Depth"))) << depth.toStdString();
    // Qt 6.9 does not name an item view by its label; the form names it.
    EXPECT_EQ(accessibleName(form.dialog().findChild<QWidget*>(QStringLiteral("edges"))),
              QStringLiteral("Edges:"));
    EXPECT_EQ(accessibleName(form.dialog().findChild<QWidget*>(QStringLiteral("faces"))),
              QStringLiteral("Faces & loops &"))
        << "a mnemonic's & dropped, && a literal &, and a last & kept";
    form.dialog().hide();
}

// A name given in code is translated with the rest of the window: a German
// screen reader says "Ansichtsfenster", not "Viewport".
TEST(AccessibilityTest, TheNamesAreTranslated) {
#ifndef HZ_SHIPPED_TRANSLATIONS_DIR
    GTEST_SKIP() << "no application target in this build";
#else
    const QString dir = QStringLiteral(HZ_SHIPPED_TRANSLATIONS_DIR);
    if (!QFile::exists(QDir(dir).filePath(QStringLiteral("horizon_de.qm")))) {
        GTEST_SKIP() << "the catalogs were not compiled (no lrelease)";
    }
    hz::ui::LocaleManager german;
    ASSERT_TRUE(german.apply(dir, QStringLiteral("de")));
    MainWindow w;
    auto* viewport = w.findChild<hz::ui::ViewportWidget*>();
    ASSERT_NE(viewport, nullptr);
    EXPECT_EQ(accessibleName(viewport), QStringLiteral("Ansichtsfenster"));

    auto* tabs = w.findChild<QTabBar*>(QStringLiteral("documentTabs"));
    ASSERT_NE(tabs, nullptr);
    QWidget* close = tabs->tabButton(0, QTabBar::RightSide);
    if (close == nullptr) close = tabs->tabButton(0, QTabBar::LeftSide);
    ASSERT_NE(close, nullptr);
    EXPECT_EQ(accessibleName(close), tabs->tabText(0) + QStringLiteral(" schließen"));
#endif
}
