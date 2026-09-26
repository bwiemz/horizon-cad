// Help (Phase 168): the user guide, compiled in and shown in its own window.

#include <gtest/gtest.h>

#include <QAction>
#include <QApplication>
#include <QFile>
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
