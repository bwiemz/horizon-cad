// Phase 155: Edit ▸ Variables, a document's named values, as one undo step.

#include <gtest/gtest.h>

#include <QAction>
#include <QApplication>
#include <QComboBox>
#include <QElapsedTimer>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QTableWidget>
#include <QTimer>
#include <QTreeWidget>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "UiTestSupport.h"
#include "horizon/document/Document.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/document/ModelCommands.h"
#include "horizon/document/UndoStack.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/ui/ConfigurationsDialog.h"
#include "horizon/ui/FeatureTreePanel.h"
#include "horizon/ui/MainWindow.h"
#include "horizon/ui/QuantitySpinBox.h"
#include "horizon/ui/VariablesDialog.h"

using hz::test::FormAnswers;
using hz::test::FormFiller;
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

namespace {

double partVolume(const hz::doc::Document& doc) {
    return doc.solid() ? hz::model::MassPropertiesCalculator::compute(*doc.solid()).volume : 0.0;
}

void editFirstFeature(MainWindow& w) {
    auto* panel = w.findChild<hz::ui::FeatureTreePanel*>();
    ASSERT_NE(panel, nullptr);
    auto* tree = panel->findChild<QTreeWidget*>();
    ASSERT_NE(tree, nullptr);
    tree->setCurrentItem(tree->topLevelItem(0));
    auto* edit = panel->findChild<QAction*>(QStringLiteral("editFeature"));
    ASSERT_NE(edit, nullptr);
    edit->trigger();
}

}  // namespace

// A field takes "=expression": it shows it, and holds its value; a number
// typed, or stepped to, is a number again.
TEST(VariablesDialogTest, AFieldTakesAnExpression) {
    using hz::ui::QuantitySpinBox;
    QuantitySpinBox field(QuantitySpinBox::Kind::Length, hz::math::LengthUnit::Millimetre, 3);
    field.setRange(0.0, 1e6);
    field.setResolver(
        [](const std::string& text, std::string* why) -> std::optional<QuantitySpinBox::Worked> {
            if (text == "wall * 2") return QuantitySpinBox::Worked{6.0, "(wall * 2)"};
            if (why != nullptr) *why = "no";
            return std::nullopt;
        });
    auto* edit = field.findChild<QLineEdit*>();
    ASSERT_NE(edit, nullptr);
    edit->setText(QStringLiteral("=wall * 2"));
    field.interpretText();
    EXPECT_DOUBLE_EQ(field.value(), 6.0);
    EXPECT_EQ(field.expression(), "(wall * 2)");
    EXPECT_EQ(field.text(), QStringLiteral("=wall * 2")) << "shown without its outer brackets";
    field.interpretText();
    EXPECT_EQ(field.expression(), "(wall * 2)") << "entered as shown, it stays";

    edit->setText(QStringLiteral("=nosuch"));
    field.interpretText();
    EXPECT_DOUBLE_EQ(field.value(), 6.0) << "what cannot be worked out changes nothing";

    field.stepBy(1);
    EXPECT_TRUE(field.expression().empty()) << "stepped to: a number";
    field.setExpression("(wall * 2)", 6.0);
    edit->setText(QStringLiteral("4"));
    field.interpretText();
    EXPECT_TRUE(field.expression().empty()) << "typed: a number";
    EXPECT_DOUBLE_EQ(field.value(), 4.0);
}

// A box's width given as an expression of a variable, in a part in inches:
// kept with its unit, followed when the variable changes, taken back by
// undo.
TEST(VariablesDialogTest, AFeaturesSizeIsAnExpressionOfAVariable) {
    MainWindow w;
    {
        FormFiller units(
            QStringLiteral("Document Units"),
            FormAnswers().choose(QStringLiteral("unit"), QStringLiteral("Inches (in)")));
        trigger(w, "action_document_units");
        ASSERT_TRUE(units.seen());
    }
    {
        FormFiller box(QStringLiteral("Box"),
                       FormAnswers()
                           .typed(QStringLiteral("size0"), QStringLiteral("1"))
                           .typed(QStringLiteral("size1"), QStringLiteral("1"))
                           .typed(QStringLiteral("size2"), QStringLiteral("1")));
        trigger(w, "action_box");
        ASSERT_TRUE(box.seen());
    }
    hz::doc::Document& doc = *w.activeDocument();
    {
        VariablesAnswer answer({{QStringLiteral("wall"), QStringLiteral("0.5 in")}});
        trigger(w, "action_variables");
        ASSERT_TRUE(answer.seen());
    }
    {
        FormFiller edit(
            QStringLiteral("Edit Box"),
            FormAnswers().typed(QStringLiteral("width"), QStringLiteral("=wall * 4 + 1")));
        editFirstFeature(w);
        ASSERT_TRUE(edit.seen());
    }
    const hz::doc::Feature* box = doc.featureTree().feature(0);
    ASSERT_NE(box, nullptr);
    EXPECT_EQ(box->parameterExpressions().at("width"), "((wall * 4) + (1 in))")
        << "a bare number kept in the unit it was typed in";
    EXPECT_NEAR(partVolume(doc), 3 * 25.4 * 25.4 * 25.4, 1e-6);

    doc.undoStack().push(std::make_unique<hz::doc::SetVariablesCommand>(
        doc, std::map<std::string, std::string>{{"wall", "1 in"}}));
    ASSERT_TRUE(doc.rebuildModel()) << doc.lastBuildMessage();
    EXPECT_NEAR(partVolume(doc), 5 * 25.4 * 25.4 * 25.4, 1e-6) << "it follows the variable";

    doc.undoStack().undo();  // the variable
    doc.undoStack().undo();  // the edit
    EXPECT_TRUE(box->parameterExpressions().empty());
}

namespace {

/// Fills the next Configurations dialog: each row a name and, by variable
/// name, its cells; then OK. When OK is refused, what it said is kept and
/// the dialog cancelled.
class ConfigurationsAnswer {
public:
    using Row = std::pair<QString, std::map<QString, QString>>;
    explicit ConfigurationsAnswer(std::vector<Row> rows) : m_rows(std::move(rows)) {
        QObject::connect(&m_timer, &QTimer::timeout, [this] { poll(); });
        m_timer.start(5);
        m_clock.start();
    }
    bool seen() const { return m_seen; }
    const QString& refused() const { return m_refused; }

private:
    void poll() {
        auto* dialog =
            qobject_cast<hz::ui::ConfigurationsDialog*>(QApplication::activeModalWidget());
        if (dialog == nullptr) {
            if (m_clock.elapsed() > 5000) m_timer.stop();
            return;
        }
        m_timer.stop();
        m_seen = true;
        auto* table = dialog->findChild<QTableWidget*>(QStringLiteral("configurations"));
        auto* add = dialog->findChild<QPushButton*>(QStringLiteral("add"));
        if (table == nullptr || add == nullptr) {
            ADD_FAILURE() << "no design table";
            dialog->reject();
            return;
        }
        for (const auto& [name, cells] : m_rows) {
            add->click();
            const int row = table->rowCount() - 1;
            table->item(row, 0)->setText(name);
            for (int column = 1; column < table->columnCount(); ++column) {
                // "wall (3 mm)": the variable's name before its own value.
                const QString header = table->horizontalHeaderItem(column)->text();
                const QString variable = header.section(QLatin1Char(' '), 0, 0);
                const auto cell = cells.find(variable);
                if (cell != cells.end()) table->item(row, column)->setText(cell->second);
            }
        }
        dialog->accept();
        if (dialog->isVisible()) {
            m_refused = dialog->findChild<QLabel*>(QStringLiteral("problem"))->text();
            dialog->reject();
        }
    }

    std::vector<Row> m_rows;
    bool m_seen = false;
    QString m_refused;
    QTimer m_timer;
    QElapsedTimer m_clock;
};

}  // namespace

// A design table: the part built in the configuration chosen in the feature
// tree, its own variables when none is; each choice one undo step.
TEST(VariablesDialogTest, TheConfigurationChosenBuildsThePart) {
    MainWindow w;
    {
        FormFiller box(QStringLiteral("Box"), FormAnswers()
                                                  .number(QStringLiteral("size0"), 10.0)
                                                  .number(QStringLiteral("size1"), 10.0)
                                                  .number(QStringLiteral("size2"), 10.0));
        trigger(w, "action_box");
        ASSERT_TRUE(box.seen());
    }
    hz::doc::Document& doc = *w.activeDocument();
    {
        VariablesAnswer answer({{QStringLiteral("wall"), QStringLiteral("3 mm")}});
        trigger(w, "action_variables");
        ASSERT_TRUE(answer.seen());
    }
    {
        FormFiller edit(QStringLiteral("Edit Box"),
                        FormAnswers().typed(QStringLiteral("width"), QStringLiteral("=wall * 2")));
        editFirstFeature(w);
        ASSERT_TRUE(edit.seen());
    }
    ASSERT_NEAR(partVolume(doc), 600.0, 1e-6);
    {
        ConfigurationsAnswer answer(
            {{QStringLiteral("Thin"), {{QStringLiteral("wall"), QStringLiteral("1 mm")}}},
             {QStringLiteral("Thick"), {{QStringLiteral("wall"), QStringLiteral("5 mm")}}}});
        trigger(w, "action_configurations");
        ASSERT_TRUE(answer.seen());
        EXPECT_TRUE(answer.refused().isEmpty()) << answer.refused().toStdString();
    }
    ASSERT_EQ(doc.configurations().size(), 2u);

    auto* chooser = w.findChild<QComboBox*>(QStringLiteral("configuration"));
    ASSERT_NE(chooser, nullptr);
    EXPECT_EQ(chooser->count(), 3) << "its own, Thin, Thick";
    const int thick = chooser->findData(QStringLiteral("Thick"));
    ASSERT_GE(thick, 0);
    chooser->setCurrentIndex(thick);
    emit chooser->activated(thick);
    EXPECT_EQ(doc.configurations().active(), "Thick");
    EXPECT_NEAR(partVolume(doc), 1000.0, 1e-6);

    trigger(w, "action_undo");
    EXPECT_EQ(doc.configurations().active(), "");
    EXPECT_NEAR(partVolume(doc), 600.0, 1e-6) << "undo builds it with its own again";
    EXPECT_EQ(chooser->currentIndex(), 0) << "the chooser follows";
}

TEST(VariablesDialogTest, TwoConfigurationsOfOneNameAreRefused) {
    MainWindow w;
    ConfigurationsAnswer answer({{QStringLiteral("A"), {}}, {QStringLiteral("A"), {}}});
    trigger(w, "action_configurations");
    ASSERT_TRUE(answer.seen());
    EXPECT_TRUE(answer.refused().contains(QStringLiteral("Two"))) << answer.refused().toStdString();
    EXPECT_EQ(w.activeDocument()->configurations().size(), 0u);
}

// Built on a worker, from a copy: the part's own feature still holds the
// value its expression works out to, and a save writes that beside it.
TEST(VariablesDialogTest, AnExpressionWorkedOutOnAWorkerIsKeptInThePart) {
    MainWindow w;
    w.setRebuildMode(MainWindow::RebuildMode::Always);
    {
        FormFiller box(QStringLiteral("Box"), FormAnswers()
                                                  .number(QStringLiteral("size0"), 10.0)
                                                  .number(QStringLiteral("size1"), 10.0)
                                                  .number(QStringLiteral("size2"), 10.0));
        trigger(w, "action_box");
        ASSERT_TRUE(box.seen());
    }
    const auto settle = [&w] {
        QElapsedTimer waited;
        waited.start();
        while (w.backgroundWorkRunning() && waited.elapsed() < 30'000) {
            QCoreApplication::processEvents(QEventLoop::AllEvents, 20);
        }
    };
    settle();
    hz::doc::Document& doc = *w.activeDocument();
    {
        VariablesAnswer answer({{QStringLiteral("wall"), QStringLiteral("3 mm")}});
        trigger(w, "action_variables");
        ASSERT_TRUE(answer.seen());
    }
    settle();
    {
        FormFiller edit(QStringLiteral("Edit Box"),
                        FormAnswers().typed(QStringLiteral("width"), QStringLiteral("=wall * 2")));
        editFirstFeature(w);
        ASSERT_TRUE(edit.seen());
    }
    settle();
    // wall to 5 mm; then undone and done again from the menu, which builds
    // the part on the worker each time.
    doc.undoStack().push(std::make_unique<hz::doc::SetVariablesCommand>(
        doc, std::map<std::string, std::string>{{"wall", "5 mm"}}));
    trigger(w, "action_undo");
    settle();
    trigger(w, "action_redo");
    settle();
    EXPECT_FALSE(doc.needsBuild());
    EXPECT_DOUBLE_EQ(doc.featureTree().feature(0)->parameters().at("width"), 10.0)
        << "the part's own feature, not only the worker's copy";
    EXPECT_NEAR(partVolume(doc), 1000.0, 1e-6);
}
