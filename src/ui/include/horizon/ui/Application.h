#pragma once

#include <QApplication>
#include <QElapsedTimer>
#include <QString>
#include <QSurfaceFormat>

namespace hz::ui {

/// QApplication that contains exceptions escaping event handlers.
///
/// Qt does not let an exception propagate through its event loop: one that
/// escapes a slot or event handler terminates the process, taking every open
/// document with it. This catches it where the event is dispatched, logs it,
/// and tells the user, so the failing command is abandoned and the session —
/// and the chance to save — survives.
class Application : public QApplication {
    Q_OBJECT

public:
    Application(int& argc, char** argv);

    bool notify(QObject* receiver, QEvent* event) override;

    /// Log the reason before the process aborts through std::terminate (an
    /// exception thrown outside any event handler, or from a destructor).
    static void installTerminateHandler();

    /// The OpenGL the viewport asks for, set as the default before the
    /// application is made: desktop OpenGL 3.3 Core, depth, 4 samples.
    /// Desktop OpenGL is said so: left to the platform, Wayland's EGL chose
    /// OpenGL ES, which has no 3.3, and no context was made.
    static QSurfaceFormat surfaceFormat();

    /// Number of exceptions contained since start-up.
    int containedExceptionCount() const { return m_containedCount; }

private:
    void reportException(const QString& what);

    int m_containedCount = 0;
    bool m_reporting = false;
    QElapsedTimer m_lastDialog;
};

}  // namespace hz::ui
