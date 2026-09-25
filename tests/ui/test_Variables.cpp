// Phase 155: Edit ▸ Variables, a document's named values, as one undo step.

#include <gtest/gtest.h>

#include <QAction>
#include <QApplication>
#include <QElapsedTimer>
#include <QLabel>
#include <QPushButton>
#include <QTableWidget>
#include <QTimer>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "horizon/document/Document.h"
#include "horizon/document/UndoStack.h"
#include "horizon/ui/MainWindow.h"
#include "horizon/ui/VariablesDialog.h"

using hz::ui::MainWindow;
using hz::ui::VariablesDialog;

namespace {

/// Fills the next Variables dialog: each {name, expression} added as a row,
/// then OK. When OK is refused (the dialog says why and stays), what it
/// said is kept and the dialog is cancelled.
class VariablesAnswer {
public:
    explicit VariablesAnswer(std::vector<std::pair<QString, QString>> rows)
        : m_rows(std::move(rows)) {
        QObject::connect(&m_timer, &QTimer::timeout, [this] { poll(); });
        m_timer.start(5);
        m_clock.start();
    }
    bool seen() const { return m_seen; }
    /// Why OK was refused; empty when it was taken.
    const QString& refused() const { return m_refused; }
    /// The values the table showed, by name, when OK was pressed.
    const std::map<QString, QString>& shown() const { return m_shown; }

private:
    void poll() {
        auto* dialog = qobject_cast<VariablesDialog*>(QApplication::activeModalWidget());
        if (dialog == nullptr) {
            if (m_clock.elapsed() > 5000) m_timer.stop();
            return;
        }
        m_timer.stop();
        m_seen = true;
        auto* table = dialog->findChild<QTableWidget*>(QStringLiteral("variables"));
        auto* add = dialog->findChild<QPushButton*>(QStringLiteral("add"));
        if (table == nullptr || add == nullptr) {
            ADD_FAILURE() << "no table of variables";
            dialog->reject();
            return;
        }
        for (const auto& [name, expression] : m_rows) {
            add->click();
            const int row = table->rowCount() - 1;
            table->item(row, 0)->setText(name);
            table->item(row, 1)->setText(expression);
        }
        for (int row = 0; row < table->rowCount(); ++row) {
            m_shown[table->item(row, 0)->text()] = table->item(row, 2)->text();
        }
        dialog->accept();
        if (dialog->isVisible()) {  // refused
            m_refused = dialog->findChild<QLabel*>(QStringLiteral("problem"))->text();
            dialog->reject();
        }
    }

    std::vector<std::pair<QString, QString>> m_rows;
    bool m_seen = false;
    QString m_refused;
    std::map<QString, QString> m_shown;
    QTimer m_timer;
    QElapsedTimer m_clock;
};

void trigger(MainWindow& w, const char* name) {
    auto* action = w.findChild<QAction*>(QString::fromLatin1(name));
    ASSERT_NE(action, nullptr) << name;
    action->trigger();
}

}  // namespace

TEST(VariablesDialogTest, VariablesAreKeptAndUndoneAsOneStep) {
    MainWindow w;
    hz::doc::Document& document = *w.activeDocument();
    {
        VariablesAnswer answer({{QStringLiteral("wall"), QStringLiteral("3 mm")},
                                {QStringLiteral("width"), QStringLiteral("10 * wall")},
                                {QStringLiteral("count"), QStringLiteral("4")}});
        trigger(w, "action_variables");
        ASSERT_TRUE(answer.seen());
        EXPECT_TRUE(answer.refused().isEmpty()) << answer.refused().toStdString();
        EXPECT_EQ(answer.shown().at(QStringLiteral("width")), QStringLiteral("30.000 mm"))
            << "worked out as it is typed";
        EXPECT_EQ(answer.shown().at(QStringLiteral("count")), QStringLiteral("4"));
    }
    const std::map<std::string, std::string> kept{
        {"wall", "3 mm"}, {"width", "10 * wall"}, {"count", "4"}};
    EXPECT_EQ(document.parameterRegistry().definitions(), kept);
    EXPECT_TRUE(document.isDirty());
    document.undoStack().undo();
    EXPECT_TRUE(document.parameterRegistry().definitions().empty());
}

TEST(VariablesDialogTest, ALoopIsRefusedAndNothingChanges) {
    MainWindow w;
    hz::doc::Document& document = *w.activeDocument();
    VariablesAnswer answer({{QStringLiteral("a"), QStringLiteral("b + 1")},
                            {QStringLiteral("b"), QStringLiteral("a * 2")}});
    trigger(w, "action_variables");
    ASSERT_TRUE(answer.seen());
    EXPECT_TRUE(answer.refused().contains(QStringLiteral("loop")))
        << answer.refused().toStdString();
    EXPECT_TRUE(document.parameterRegistry().definitions().empty());
    EXPECT_FALSE(document.undoStack().canUndo());
}
