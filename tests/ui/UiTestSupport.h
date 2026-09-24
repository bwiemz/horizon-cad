#pragma once

// Helpers for tests that drive real widgets under hz_ui_window_tests.

#include <QAbstractButton>
#include <QApplication>
#include <QElapsedTimer>
#include <QEventLoop>
#include <QMessageBox>
#include <QString>
#include <QTimer>
#include <utility>

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
};

}  // namespace hz::test
