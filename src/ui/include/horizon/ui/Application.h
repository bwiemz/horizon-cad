#pragma once

#include <QApplication>
#include <QElapsedTimer>
#include <QString>
#include <QStringList>
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

    /// Where the files shipped with the application are: the translation
    /// catalogs and the samples. Next to the executable, except in a macOS
    /// application bundle, where they are in Contents/Resources.
    static QString shippedFilesDirectory();
    /// The same, for an executable in `executableDir`.
    static QString shippedFilesDirectory(const QString& executableDir);

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

    /// The files the system asked to open while nothing listened for
    /// fileOpenRequested, oldest first, and no longer held. On macOS the
    /// document that launched the application can arrive before its window.
    QStringList takePendingFiles();

signals:
    /// The system asked for a file to be opened: on macOS, a document opened
    /// from the Finder or dropped on the Dock icon arrives as an event, not
    /// on the command line. With nothing connected, it waits in
    /// takePendingFiles() instead.
    void fileOpenRequested(const QString& file);

protected:
    bool event(QEvent* event) override;

private:
    void reportException(const QString& what);

    QStringList m_pendingFiles;
    int m_containedCount = 0;
    bool m_reporting = false;
    QElapsedTimer m_lastDialog;
};

}  // namespace hz::ui
