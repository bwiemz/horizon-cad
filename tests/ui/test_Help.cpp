// Help (Phase 168): the user guide, compiled in and shown in its own window.

#include <gtest/gtest.h>

#include <QAction>
#include <QApplication>
#include <QFile>
#include <QKeyEvent>
#include <QLabel>
#include <QPushButton>
#include <QRegularExpression>
#include <QSet>
#include <QStatusBar>
#include <QTextBlock>
#include <QTextBrowser>
#include <QTextDocument>
#include <string>

#include "UiTestSupport.h"
#include "horizon/document/Document.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/ui/FeatureTreePanel.h"
#include "horizon/ui/GettingStartedTour.h"
#include "horizon/ui/HelpWindow.h"
#include "horizon/ui/MainWindow.h"

using hz::math::Vec2;
using hz::test::FormAnswers;
using hz::test::FormFiller;
using hz::test::ToolDriver;
using hz::ui::HelpWindow;
using hz::ui::MainWindow;

namespace {

void trigger(QObject& owner, const char* name) {
    auto* action = owner.findChild<QAction*>(QString::fromLatin1(name));
    ASSERT_NE(action, nullptr) << name;
    action->trigger();
}

/// The Markdown source of guide page @p page.
QString source(const QString& page) {
    QFile file(QStringLiteral(":/guide/") + page);
    return file.open(QIODevice::ReadOnly) ? QString::fromUtf8(file.readAll()) : QString();
}

/// The targets of @p page's links: "[text](target)".
QStringList links(const QString& page) {
    QStringList out;
    static const QRegularExpression link(QStringLiteral(R"(\]\(([^)\s]+)\))"));
    auto it = link.globalMatch(source(page));
    while (it.hasNext()) out << it.next().captured(1);
    return out;
}

QTextBrowser& browserOf(HelpWindow& help) {
    auto* browser = help.findChild<QTextBrowser*>(QStringLiteral("helpBrowser"));
    EXPECT_NE(browser, nullptr);
    return *browser;
}

}  // namespace

TEST(HelpTest, TheGuideIsCompiledIn) {
    const QStringList pages = HelpWindow::pages();
    for (const char* page :
         {"index.md", "getting-started.md", "drafting.md", "sketches.md", "parts.md",
          "assemblies.md", "drawings.md", "files.md", "settings.md", "shortcuts.md"}) {
        EXPECT_TRUE(pages.contains(QString::fromLatin1(page))) << page;
    }
    HelpWindow help;
    EXPECT_EQ(help.currentPage(), QStringLiteral("index.md"));
    EXPECT_EQ(help.windowTitle(), QStringLiteral("Horizon CAD User Guide")) << "not twice";
}

// Qt's Markdown reader shows a page: its title a heading, its tables tables,
// not the Markdown's own marks.
TEST(HelpTest, APageIsShownAsFormattedText) {
    HelpWindow help;
    help.showPage(QStringLiteral("shortcuts.md"));
    QTextBrowser& browser = browserOf(help);
    const QTextDocument& doc = *browser.document();
    EXPECT_EQ(doc.firstBlock().text(), QStringLiteral("Keyboard shortcuts"));
    EXPECT_EQ(doc.firstBlock().blockFormat().headingLevel(), 1);
    const QString text = browser.toPlainText();
    EXPECT_TRUE(text.contains(QStringLiteral("Command palette")));
    EXPECT_FALSE(text.contains(QStringLiteral("|---|"))) << "a table, not its Markdown";
    EXPECT_FALSE(text.contains(QStringLiteral("**"))) << "bold, not its Markdown";
}

// Every page has a title, every link goes to a page of the guide, and every
// page can be reached from the contents.
TEST(HelpTest, EveryLinkGoesToAPageAndEveryPageIsReached) {
    const QStringList pages = HelpWindow::pages();
    for (const QString& page : pages) {
        EXPECT_FALSE(HelpWindow::pageTitle(page).isEmpty()) << page.toStdString();
        for (const QString& target : links(page)) {
            EXPECT_TRUE(pages.contains(target))
                << page.toStdString() << " links to " << target.toStdString();
        }
    }
    QSet<QString> reached{QStringLiteral("index.md")};
    QStringList queue{QStringLiteral("index.md")};
    while (!queue.isEmpty()) {
        for (const QString& target : links(queue.takeFirst())) {
            if (pages.contains(target) && !reached.contains(target)) {
                reached.insert(target);
                queue << target;
            }
        }
    }
    for (const QString& page : pages) {
        EXPECT_TRUE(reached.contains(page)) << page.toStdString() << " is not linked to";
    }
}

// A link followed in the window opens its page there, and Back returns.
TEST(HelpTest, ALinkOpensItsPageAndBackReturns) {
    HelpWindow help;
    QTextBrowser& browser = browserOf(help);
    // As a click resolves it: against the page shown.
    browser.setSource(browser.source().resolved(QUrl(QStringLiteral("parts.md"))));
    EXPECT_EQ(help.currentPage(), QStringLiteral("parts.md"));
    EXPECT_EQ(help.windowTitle(), QStringLiteral("Parts - Horizon CAD User Guide"));
    EXPECT_TRUE(browser.toPlainText().contains(QStringLiteral("Feature Tree")));
    browser.backward();
    EXPECT_EQ(help.currentPage(), QStringLiteral("index.md"));
    EXPECT_EQ(help.windowTitle(), QStringLiteral("Horizon CAD User Guide"));

    help.showPage(QStringLiteral("no-such-page.md"));
    EXPECT_EQ(help.currentPage(), QStringLiteral("index.md")) << "the contents instead";
}

// F1 opens the guide at the page for what is being worked on.
TEST(HelpTest, F1OpensThePageForWhatIsBeingWorkedOn) {
    MainWindow w;
    auto* guide = w.findChild<QAction*>(QStringLiteral("action_user_guide"));
    ASSERT_NE(guide, nullptr);
    EXPECT_EQ(guide->shortcut(), QKeySequence(QKeySequence::HelpContents));

    guide->trigger();
    ASSERT_NE(w.helpWindow(), nullptr);
    EXPECT_TRUE(w.helpWindow()->isVisible());
    EXPECT_EQ(w.helpWindow()->currentPage(), QStringLiteral("drafting.md")) << "a drawing";

    trigger(w, "action_new_part");
    guide->trigger();
    EXPECT_EQ(w.helpWindow()->currentPage(), QStringLiteral("parts.md"));
    trigger(w, "action_sketch_xy");
    guide->trigger();
    EXPECT_EQ(w.helpWindow()->currentPage(), QStringLiteral("sketches.md")) << "a sketch edited";

    trigger(w, "action_new_assembly");
    guide->trigger();
    EXPECT_EQ(w.helpWindow()->currentPage(), QStringLiteral("assemblies.md"));
    w.helpWindow()->close();
}

// The guide's first part (getting-started.md), step by step as it says: a
// sketch on XY, a rectangle from a click and "@40,20", Finish Sketch, and
// an extrusion of the distance the form starts with, 10.
TEST(HelpTest, TheGuidesFirstPartIsMadeAsItSays) {
    MainWindow w;
    ToolDriver drive(w);
    trigger(w, "action_new_part");
    trigger(w, "action_sketch_xy");
    trigger(w, "tool_rectangle");
    drive.click(Vec2(0, 0));
    for (const Qt::Key key :
         {Qt::Key_At, Qt::Key_4, Qt::Key_0, Qt::Key_Comma, Qt::Key_2, Qt::Key_0, Qt::Key_Return}) {
        drive.key(key);
    }
    trigger(w, "action_sketch_finish");
    FormFiller filler(QStringLiteral("Extrude"), FormAnswers());
    trigger(w, "action_extrude");
    ASSERT_TRUE(filler.seen());

    const hz::doc::Document& doc = *w.activeDocument();
    ASSERT_EQ(doc.featureTree().featureCount(), 1u)
        << w.statusBar()->currentMessage().toStdString();
    ASSERT_NE(doc.solid(), nullptr);
    EXPECT_NEAR(hz::model::MassPropertiesCalculator::compute(*doc.solid()).volume, 8000.0, 1e-6);
}

// Getting Started (168c): each step outlines a part of the window and says
// what it is; Back and Next move through them, and the last ends the tour.
TEST(HelpTest, TheTourPointsAtEachPartOfTheWindowInTurn) {
    MainWindow w;
    w.resize(1400, 900);
    w.show();
    QApplication::processEvents();
    trigger(w, "action_getting_started");
    QPointer<hz::ui::GettingStartedTour> tour = w.tour();
    ASSERT_NE(tour, nullptr);
    ASSERT_GE(tour->stepCount(), 5);
    const auto* title = tour->findChild<QLabel*>(QStringLiteral("tourTitle"));
    ASSERT_NE(title, nullptr);

    QStringList titles;
    for (int i = 0; i < tour->stepCount(); ++i) {
        EXPECT_EQ(tour->step(), i);
        titles << title->text();
        EXPECT_FALSE(tour->findChild<QLabel*>(QStringLiteral("tourText"))->text().isEmpty());
        const QRect outlined = tour->outlined();
        EXPECT_FALSE(outlined.isEmpty()) << title->text().toStdString();
        EXPECT_TRUE(w.rect().contains(tour->geometry())) << "the panel is in the window";
        if (i + 1 < tour->stepCount()) tour->next();
    }
    EXPECT_EQ(titles.front(), QStringLiteral("The ribbon"));
    EXPECT_TRUE(titles.contains(QStringLiteral("The view")));

    // The view's step outlines the viewport.
    tour->back();
    while (title->text() != QStringLiteral("The view") && tour->step() > 0) tour->back();
    auto* viewport = w.findChild<hz::ui::ViewportWidget*>();
    ASSERT_NE(viewport, nullptr);
    const QPoint middle = viewport->mapTo(&w, viewport->rect().center());
    EXPECT_TRUE(tour->outlined().contains(middle));

    while (tour != nullptr && tour->step() + 1 < tour->stepCount()) tour->next();
    ASSERT_NE(tour, nullptr);
    tour->next();  // Done
    QApplication::processEvents();
    QCoreApplication::sendPostedEvents(nullptr, QEvent::DeferredDelete);
    EXPECT_EQ(tour, nullptr) << "ended, and gone";
    EXPECT_EQ(w.tour(), nullptr);
}

// The tour is offered the first time Horizon CAD starts, and not again.
TEST(HelpTest, TheTourIsOfferedOnceAtFirstStart) {
    {
        MainWindow w;
        w.offerTour();
        ASSERT_NE(w.tour(), nullptr) << "the first start";
    }
    MainWindow w;
    w.offerTour();
    EXPECT_EQ(w.tour(), nullptr) << "offered once";
    trigger(w, "action_getting_started");
    EXPECT_NE(w.tour(), nullptr) << "but always there in the Help menu";
}

// A panel floated as a window of its own is not in the window: its step
// shows the panel alone, in the middle, not an outline at a wrong place.
// And Escape closes the tour, from its buttons, which have the focus.
TEST(HelpTest, TheTourCopesWithAFloatedPanelAndClosesOnEscape) {
    MainWindow w;
    w.resize(1400, 900);
    w.show();
    QApplication::processEvents();
    auto* tree = w.findChild<hz::ui::FeatureTreePanel*>();
    ASSERT_NE(tree, nullptr);
    tree->setFloating(true);
    QApplication::processEvents();

    trigger(w, "action_getting_started");
    QPointer<hz::ui::GettingStartedTour> tour = w.tour();
    ASSERT_NE(tour, nullptr);
    const auto* title = tour->findChild<QLabel*>(QStringLiteral("tourTitle"));
    while (title->text() != QStringLiteral("The feature tree") &&
           tour->step() + 1 < tour->stepCount()) {
        tour->next();
    }
    ASSERT_EQ(title->text(), QStringLiteral("The feature tree"));
    EXPECT_TRUE(tour->outlined().isEmpty()) << "not outlined where it is not";
    EXPECT_TRUE(w.rect().contains(tour->geometry()));

    // The window's focus: which of its widgets keys go to when it is active
    // (the floated panel is the active window here).
    QWidget* focused = w.focusWidget();
    ASSERT_NE(focused, nullptr);
    EXPECT_TRUE(tour->isAncestorOf(focused)) << "the tour's buttons have the keys";
    QKeyEvent escape(QEvent::KeyPress, Qt::Key_Escape, Qt::NoModifier);
    QCoreApplication::sendEvent(focused, &escape);
    QCoreApplication::sendPostedEvents(nullptr, QEvent::DeferredDelete);
    EXPECT_EQ(tour, nullptr) << "closed by Escape";
    tree->setFloating(false);
}
