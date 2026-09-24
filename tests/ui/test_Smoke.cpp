// Every command in the window runs without crashing (Phase 112): each menu and
// ribbon action is triggered on an empty drawing, part and assembly, and on a
// drawing with something selected, with every dialog it opens dismissed.

#include <gtest/gtest.h>

#include <QAction>
#include <QCoreApplication>
#include <QMenu>
#include <QMenuBar>
#include <QSet>
#include <memory>
#include <string>
#include <vector>

#include "UiTestSupport.h"
#include "horizon/document/Document.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/ui/MainWindow.h"
#include "horizon/ui/ViewportWidget.h"

using hz::test::ModalCloser;
using hz::ui::MainWindow;

namespace {

void collect(QMenu* menu, QList<QAction*>& out) {
    for (QAction* a : menu->actions()) {
        if (a->isSeparator()) continue;
        if (a->menu()) {
            emit a->menu()->aboutToShow();  // menus that fill themselves as they open
            collect(a->menu(), out);
        } else {
            out << a;
        }
    }
}

/// Every command: the menus' actions, then everything else the window holds
/// (the ribbon), each once. Exit is left out: it closes the window under test.
QList<QAction*> commands(MainWindow& w) {
    QList<QAction*> out;
    for (QAction* top : w.menuBar()->actions()) {
        if (top->menu()) collect(top->menu(), out);
    }
    for (QAction* a : w.findChildren<QAction*>()) {
        if (!a->isSeparator() && !a->menu() && !out.contains(a)) out << a;
    }
    QList<QAction*> kept;
    for (QAction* a : out) {
        const QString text = a->text().remove(QLatin1Char('&'));
        if (text == QStringLiteral("Exit")) continue;
        kept << a;
    }
    return kept;
}

/// Trigger each command, then Escape any tool it left waiting.
void runEveryCommand(MainWindow& w) {
    ModalCloser closer;
    auto* viewport = w.findChild<hz::ui::ViewportWidget*>();
    ASSERT_NE(viewport, nullptr);
    viewport->resize(1000, 700);
    for (QAction* action : commands(w)) {
        if (!action->isEnabled()) continue;
        SCOPED_TRACE(action->text().toStdString() + " [" + action->objectName().toStdString() +
                     "]");
        action->trigger();
        QCoreApplication::processEvents();
        ASSERT_NE(w.activeDocument(), nullptr);
    }
}

}  // namespace

TEST(SmokeTest, EveryCommandRunsOnAnEmptyDrawing) {
    MainWindow w;
    runEveryCommand(w);
}

TEST(SmokeTest, EveryCommandRunsOnAnEmptyPart) {
    MainWindow w;
    for (QAction* a : commands(w)) {
        if (a->text().remove(QLatin1Char('&')) == QStringLiteral("New Part")) a->trigger();
    }
    ASSERT_EQ(w.activeDocument()->type(), hz::doc::DocumentType::Part);
    runEveryCommand(w);
}

TEST(SmokeTest, EveryCommandRunsOnAnEmptyAssembly) {
    MainWindow w;
    for (QAction* a : commands(w)) {
        if (a->text().remove(QLatin1Char('&')) == QStringLiteral("New Assembly")) a->trigger();
    }
    ASSERT_NE(w.activeAssembly(), nullptr);
    runEveryCommand(w);
}

TEST(SmokeTest, EveryCommandRunsWithADrawingSelected) {
    MainWindow w;
    auto& drawing = w.activeDocument()->draftDocument();
    auto line = std::make_shared<hz::draft::DraftLine>(hz::math::Vec2(0, 0), hz::math::Vec2(5, 0));
    auto circle = std::make_shared<hz::draft::DraftCircle>(hz::math::Vec2(8, 0), 2.0);
    drawing.addEntity(line);
    drawing.addEntity(circle);
    auto* viewport = w.findChild<hz::ui::ViewportWidget*>();
    ASSERT_NE(viewport, nullptr);
    viewport->selectionManager().select(line->id());
    viewport->selectionManager().select(circle->id());
    runEveryCommand(w);
}
