#pragma once

// Helpers for tests that drive real widgets under hz_ui_window_tests.

#include <gtest/gtest.h>

#include <QAbstractButton>
#include <QApplication>
#include <QComboBox>
#include <QDialog>
#include <QDoubleSpinBox>
#include <QElapsedTimer>
#include <QEventLoop>
#include <QFileDialog>
#include <QKeyEvent>
#include <QLineEdit>
#include <QListWidget>
#include <QMessageBox>
#include <QMouseEvent>
#include <QPushButton>
#include <QSpinBox>
#include <QString>
#include <QStringList>
#include <QTimer>
#include <QWidget>
#include <limits>
#include <map>
#include <optional>
#include <utility>

#include "horizon/document/FeatureTree.h"
#include "horizon/math/Vec2.h"
#include "horizon/ui/MainWindow.h"
#include "horizon/ui/ViewportWidget.h"

namespace hz::test {

/// Answers the next modal message box with `button`, and records whether one
/// appeared and what it said. With a `title`, only a box with that window
/// title is answered. Stops polling when destroyed or after `timeoutMs`, so a
/// dialog that never comes cannot leak into the next test.
class DialogResponder {
public:
    explicit DialogResponder(QMessageBox::StandardButton button, QString title = {},
                             int timeoutMs = 10'000)
        : m_button(button), m_title(std::move(title)), m_timeoutMs(timeoutMs) {
        m_clock.start();
        QObject::connect(&m_timer, &QTimer::timeout, [this] { poll(); });
        m_timer.start(5);
    }

    bool seen() const { return m_seen; }
    const QString& text() const { return m_text; }
    const QString& informativeText() const { return m_informativeText; }
    /// The button Enter would have pressed.
    QMessageBox::StandardButton defaultButton() const { return m_default; }

    /// Run the event loop until a dialog has been answered or `ms` elapse.
    void waitForDialog(int ms) {
        QElapsedTimer waited;
        waited.start();
        while (!m_seen && waited.elapsed() < ms) {
            QCoreApplication::processEvents(QEventLoop::AllEvents, 20);
        }
    }

private:
    void poll() {
        auto* box = qobject_cast<QMessageBox*>(QApplication::activeModalWidget());
        QAbstractButton* button = box ? box->button(m_button) : nullptr;
        if (button && (m_title.isEmpty() || box->windowTitle() == m_title)) {
            m_timer.stop();
            m_seen = true;
            m_text = box->text();
            m_informativeText = box->informativeText();
            m_default = box->standardButton(box->defaultButton());
            button->click();
            return;
        }
        if (m_clock.elapsed() > m_timeoutMs) m_timer.stop();
    }

    QMessageBox::StandardButton m_button;
    QString m_title;
    int m_timeoutMs;
    QTimer m_timer;
    QElapsedTimer m_clock;
    bool m_seen = false;
    QString m_text;
    QString m_informativeText;
    QMessageBox::StandardButton m_default = QMessageBox::NoButton;
};

/// What to put in a feature form before accepting it, built up by chaining:
/// `FormAnswers().number("size", 2).check("edges", {"(0, 0, 0)"})`.
struct FormAnswers {
    /// A number field (QDoubleSpinBox or QSpinBox) by object name; the empty
    /// name is the first number field, whatever it is called.
    FormAnswers& number(const QString& name, double value) {
        numbers[name] = value;
        return *this;
    }
    /// A text field (QLineEdit) by object name.
    FormAnswers& text(const QString& name, const QString& value) {
        texts[name] = value;
        return *this;
    }
    /// A choice (QComboBox) by object name: the item text to pick.
    FormAnswers& choose(const QString& name, const QString& text) {
        choices[name] = text;
        return *this;
    }
    /// A checklist (QListWidget) by object name: check every item whose text
    /// contains one of `parts`.
    FormAnswers& check(const QString& name, const QStringList& parts) {
        checks[name] = parts;
        return *this;
    }
    /// How the body combines, in the "bodyOperation" choice.
    FormAnswers& combine(std::optional<doc::BodyOperation> how) {
        operation = how;
        return *this;
    }
    /// Reject the form instead.
    FormAnswers& reject() {
        cancel = true;
        return *this;
    }

    std::map<QString, double> numbers;
    std::map<QString, QString> texts;
    std::map<QString, QString> choices;
    std::map<QString, QStringList> checks;
    std::optional<doc::BodyOperation> operation;
    bool cancel = false;
};

/// Answers the next modal dialog titled `title` with `answers`. Records
/// whether one came, what its "bodyOperation" choice proposed and what each
/// checklist offered. A field it was told to fill but cannot find is a test
/// failure, and the dialog is rejected.
class FormFiller {
public:
    FormFiller(QString title, FormAnswers answers)
        : m_title(std::move(title)), m_answers(std::move(answers)) {
        QObject::connect(&m_timer, &QTimer::timeout, [this] { poll(); });
        m_timer.start(5);
        m_clock.start();
    }
    bool seen() const { return m_seen; }
    std::optional<doc::BodyOperation> proposed() const { return m_proposed; }
    QStringList offered(const QString& list) const {
        const auto it = m_offered.find(list);
        return it == m_offered.end() ? QStringList{} : it->second;
    }
    /// What a number field showed when the form opened (NaN if none).
    double shown(const QString& field) const {
        const auto it = m_shown.find(field);
        return it == m_shown.end() ? std::numeric_limits<double>::quiet_NaN() : it->second;
    }

private:
    void poll() {
        auto* dialog = qobject_cast<QDialog*>(QApplication::activeModalWidget());
        if (dialog == nullptr || dialog->windowTitle() != m_title) {
            if (m_clock.elapsed() > 5000) m_timer.stop();
            return;
        }
        m_timer.stop();
        m_seen = true;
        if (auto* combo = dialog->findChild<QComboBox*>(QStringLiteral("bodyOperation"))) {
            m_proposed = static_cast<doc::BodyOperation>(combo->currentData().toInt());
        }
        for (auto* spin : dialog->findChildren<QDoubleSpinBox*>()) {
            m_shown[spin->objectName()] = spin->value();
        }
        for (auto* list : dialog->findChildren<QListWidget*>()) {
            QStringList items;
            for (int i = 0; i < list->count(); ++i) items << list->item(i)->text();
            m_offered[list->objectName()] = items;
        }
        if (m_answers.cancel) {
            dialog->reject();
            return;
        }
        if (!fill(*dialog)) {
            dialog->reject();
            return;
        }
        dialog->accept();
    }

    bool fill(QDialog& dialog) {
        for (const auto& [name, value] : m_answers.numbers) {
            if (auto* spin = dialog.findChild<QDoubleSpinBox*>(name)) {
                spin->setValue(value);
            } else if (auto* count = dialog.findChild<QSpinBox*>(name)) {
                count->setValue(static_cast<int>(value));
            } else {
                ADD_FAILURE() << "no number field " << name.toStdString();
                return false;
            }
        }
        for (const auto& [name, value] : m_answers.texts) {
            auto* edit = dialog.findChild<QLineEdit*>(name);
            if (edit == nullptr) {
                ADD_FAILURE() << "no text field " << name.toStdString();
                return false;
            }
            edit->setText(value);
        }
        for (const auto& [name, text] : m_answers.choices) {
            auto* combo = dialog.findChild<QComboBox*>(name);
            const int index = combo ? combo->findText(text) : -1;
            if (index < 0) {
                ADD_FAILURE() << "no choice " << text.toStdString() << " in " << name.toStdString();
                return false;
            }
            combo->setCurrentIndex(index);
        }
        for (const auto& [name, wanted] : m_answers.checks) {
            auto* list = dialog.findChild<QListWidget*>(name);
            if (list == nullptr) {
                ADD_FAILURE() << "no list " << name.toStdString();
                return false;
            }
            for (int i = 0; i < list->count(); ++i) {
                for (const QString& part : wanted) {
                    if (list->item(i)->text().contains(part)) {
                        list->item(i)->setCheckState(Qt::Checked);
                    }
                }
            }
        }
        if (m_answers.operation) {
            auto* combo = dialog.findChild<QComboBox*>(QStringLiteral("bodyOperation"));
            if (combo == nullptr) {
                ADD_FAILURE() << "no body operation choice";
                return false;
            }
            combo->setCurrentIndex(combo->findData(static_cast<int>(*m_answers.operation)));
        }
        return true;
    }

    QString m_title;
    FormAnswers m_answers;
    bool m_seen = false;
    std::optional<doc::BodyOperation> m_proposed;
    std::map<QString, QStringList> m_offered;
    std::map<QString, double> m_shown;
    QTimer m_timer;
    QElapsedTimer m_clock;
};

/// Picks `path` in the next file dialog by typing it into the file name box.
/// (QFileDialog::selectFile() leaves that box alone while it has focus, which
/// a shown dialog gives it — the accept then finds no file and does nothing.)
/// Gives up — rejecting the dialog — rather than hang the test.
class FilePicker {
public:
    explicit FilePicker(QString path) : m_path(std::move(path)) {
        QObject::connect(&m_timer, &QTimer::timeout, [this] { poll(); });
        m_timer.start(20);
        m_clock.start();
    }

private:
    void poll() {
        auto* dialog = qobject_cast<QFileDialog*>(QApplication::activeModalWidget());
        if (dialog == nullptr) {
            if (m_seen || m_clock.elapsed() > 10'000) m_timer.stop();
            return;
        }
        m_seen = true;
        if (m_clock.elapsed() > 10'000) {
            ADD_FAILURE() << "the file dialog would not take " << m_path.toStdString();
            dialog->reject();
            m_timer.stop();
            return;
        }
        auto* name = dialog->findChild<QLineEdit*>(QStringLiteral("fileNameEdit"));
        if (name == nullptr) {
            ADD_FAILURE() << "the file dialog has no file name box";
            dialog->reject();
            m_timer.stop();
            return;
        }
        name->setText(m_path);
        static_cast<QDialog*>(dialog)->accept();  // QFileDialog's own accept() is protected
    }

    QString m_path;
    bool m_seen = false;
    QTimer m_timer;
    QElapsedTimer m_clock;
};

/// Dismisses every modal dialog that opens while it lives: message boxes,
/// file dialogs, forms. For tests that run commands only to see they do not
/// crash, whatever they ask. Records the titles it dismissed.
class ModalCloser {
public:
    ModalCloser() {
        QObject::connect(&m_timer, &QTimer::timeout, [this] { poll(); });
        m_timer.start(5);
    }

    const QStringList& dismissed() const { return m_dismissed; }

private:
    void poll() {
        QWidget* modal = QApplication::activeModalWidget();
        if (modal == nullptr) return;
        m_dismissed << modal->windowTitle();
        if (auto* dialog = qobject_cast<QDialog*>(modal)) {
            dialog->reject();
        } else {
            modal->close();  // a modal that is not a dialog must not hang the test
        }
    }

    QTimer m_timer;
    QStringList m_dismissed;
};

/// Drives a window's viewport the way a user does: presses, moves and
/// releases at world points, through the viewport's own mouse handling, so the
/// active tool sees what it would see from a real click.
class ToolDriver {
public:
    explicit ToolDriver(hz::ui::MainWindow& window)
        : m_viewport(window.findChild<hz::ui::ViewportWidget*>()) {
        EXPECT_NE(m_viewport, nullptr);
        // A size to project with; the window is never shown, so no layout
        // gives it one.
        if (m_viewport != nullptr) m_viewport->resize(1000, 700);
    }

    hz::ui::ViewportWidget& viewport() { return *m_viewport; }

    void click(const hz::math::Vec2& world) {
        send(QEvent::MouseButtonPress, world, Qt::LeftButton, Qt::LeftButton);
        send(QEvent::MouseButtonRelease, world, Qt::LeftButton, Qt::NoButton);
    }

    void move(const hz::math::Vec2& world) {
        send(QEvent::MouseMove, world, Qt::NoButton, Qt::NoButton);
    }

    /// Press at @p from, move to @p to with the button held, release there:
    /// a box selection with the select tool.
    void drag(const hz::math::Vec2& from, const hz::math::Vec2& to) {
        send(QEvent::MouseButtonPress, from, Qt::LeftButton, Qt::LeftButton);
        const hz::math::Vec2 mid{(from.x + to.x) / 2.0, (from.y + to.y) / 2.0};
        send(QEvent::MouseMove, mid, Qt::NoButton, Qt::LeftButton);
        send(QEvent::MouseMove, to, Qt::NoButton, Qt::LeftButton);
        send(QEvent::MouseButtonRelease, to, Qt::LeftButton, Qt::NoButton);
    }

    void key(Qt::Key key) {
        QKeyEvent press(QEvent::KeyPress, key, Qt::NoModifier);
        QCoreApplication::sendEvent(m_viewport, &press);
    }

private:
    void send(QEvent::Type type, const hz::math::Vec2& world, Qt::MouseButton button,
              Qt::MouseButtons buttons) {
        const QPointF at = m_viewport->worldToScreen(world);
        QMouseEvent event(type, at, m_viewport->mapToGlobal(at), button, buttons, Qt::NoModifier);
        QCoreApplication::sendEvent(m_viewport, &event);
    }

    hz::ui::ViewportWidget* m_viewport;
};

}  // namespace hz::test
