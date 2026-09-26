// hz::ui::Application contains exceptions that escape event handlers.

#include <gtest/gtest.h>

#include <QCoreApplication>
#include <QDir>
#include <QEvent>
#include <QFileOpenEvent>
#include <QMessageBox>
#include <QObject>
#include <QStringList>
#include <QTemporaryDir>
#include <stdexcept>

#include "UiTestSupport.h"
#include "horizon/ui/Application.h"

using hz::test::DialogResponder;

namespace {

/// Throws from its event handler, as a slot calling into a failing kernel op
/// would.
class ThrowingReceiver : public QObject {
public:
    static constexpr QEvent::Type kThrowEvent = QEvent::User;

    bool event(QEvent* e) override {
        if (e->type() == kThrowEvent) throw std::runtime_error("kernel op failed");
        return QObject::event(e);
    }
};

}  // namespace

TEST(ApplicationTest, ExceptionFromAnEventHandlerIsContainedAndReported) {
#if defined(Q_OS_MACOS)
    // Qt on macOS is built without exceptions: one cannot unwind through its
    // frames to Application::notify, and ends the process there (a crash
    // report, Phase 167). There is nothing to contain.
    GTEST_SKIP() << "an exception cannot pass through Qt's frames on macOS";
#endif
    auto* app = qobject_cast<hz::ui::Application*>(QCoreApplication::instance());
    ASSERT_NE(app, nullptr) << "the test binary must run under hz::ui::Application";
    const int before = app->containedExceptionCount();

    ThrowingReceiver receiver;
    QEvent event(ThrowingReceiver::kThrowEvent);
    DialogResponder responder(QMessageBox::Ok, QStringLiteral("Unexpected Error"), 2000);

    bool delivered = true;
    ASSERT_NO_THROW(delivered = QCoreApplication::sendEvent(&receiver, &event));
    EXPECT_FALSE(delivered);
    EXPECT_EQ(app->containedExceptionCount(), before + 1);
    EXPECT_TRUE(responder.seen()) << "the user is told the operation was abandoned";
    EXPECT_TRUE(responder.text().contains("kernel op failed")) << responder.text().toStdString();
}

TEST(ApplicationTest, RepeatedExceptionsDoNotRaiseADialogStorm) {
#if defined(Q_OS_MACOS)
    // Qt on macOS is built without exceptions: one cannot unwind through its
    // frames to Application::notify, and ends the process there (a crash
    // report, Phase 167). There is nothing to contain.
    GTEST_SKIP() << "an exception cannot pass through Qt's frames on macOS";
#endif
    auto* app = qobject_cast<hz::ui::Application*>(QCoreApplication::instance());
    ASSERT_NE(app, nullptr);
    const int before = app->containedExceptionCount();

    // The first failure is shown (or was, moments ago, by another test in
    // this process). The ones straight after it are logged and counted but not
    // shown: nothing answers a dialog for them, so one appearing would hang
    // the test rather than pass it.
    ThrowingReceiver receiver;
    DialogResponder responder(QMessageBox::Ok, QStringLiteral("Unexpected Error"), 2000);
    for (int i = 0; i < 4; ++i) {
        QEvent event(ThrowingReceiver::kThrowEvent);
        QCoreApplication::sendEvent(&receiver, &event);
    }
    EXPECT_EQ(app->containedExceptionCount(), before + 4);
}

// The viewport's OpenGL is desktop OpenGL 3.3 Core, said so. Left to the
// platform, Wayland's EGL chose OpenGL ES, which has no 3.3: no context,
// and a viewport that could not draw on a Wayland desktop.
TEST(ApplicationTest, TheViewportAsksForDesktopOpenGL) {
    const QSurfaceFormat format = hz::ui::Application::surfaceFormat();
    EXPECT_EQ(format.renderableType(), QSurfaceFormat::OpenGL);
    EXPECT_EQ(format.majorVersion(), 3);
    EXPECT_EQ(format.minorVersion(), 3);
    EXPECT_EQ(format.profile(), QSurfaceFormat::CoreProfile);
    EXPECT_EQ(format.depthBufferSize(), 24);
}

// The catalogs and samples are found where CMake puts them (HZ_SHIPPED_DIR):
// next to the executable, or in a macOS bundle's Contents/Resources.
TEST(ApplicationTest, ShippedFilesAreNextToTheExecutableOrInTheBundle) {
    using hz::ui::Application;
    QTemporaryDir dir;
    ASSERT_TRUE(dir.isValid());
    const QString bin = dir.filePath(QStringLiteral("bin"));
    const QString contents = dir.filePath(QStringLiteral("HorizonCAD.app/Contents"));
    const QString lone = dir.filePath(QStringLiteral("lone/MacOS"));
    for (const QString& made : {bin, contents + QStringLiteral("/MacOS"),
                                contents + QStringLiteral("/Resources"), lone}) {
        ASSERT_TRUE(QDir().mkpath(made));
    }

    EXPECT_EQ(Application::shippedFilesDirectory(bin), QDir(bin).absolutePath());
    const QString inBundle =
        Application::shippedFilesDirectory(contents + QStringLiteral("/MacOS"));
#if defined(Q_OS_MACOS)
    EXPECT_EQ(inBundle, QDir(contents + QStringLiteral("/Resources")).absolutePath());
#else
    EXPECT_EQ(inBundle, QDir(contents + QStringLiteral("/MacOS")).absolutePath())
        << "a bundle is macOS's alone";
#endif
    EXPECT_EQ(Application::shippedFilesDirectory(lone), QDir(lone).absolutePath())
        << "a folder named MacOS with no Resources beside it is no bundle";
    // The test binary is in no bundle.
    EXPECT_EQ(Application::shippedFilesDirectory(),
              QDir(QCoreApplication::applicationDirPath()).absolutePath());
}

// On macOS the document that launched the application can arrive before its
// window is made to hear it: it waits, and the window takes it.
TEST(ApplicationTest, AFileAskedForBeforeAnythingListensWaits) {
    auto* app = qobject_cast<hz::ui::Application*>(QCoreApplication::instance());
    ASSERT_NE(app, nullptr) << "the test binary must run under hz::ui::Application";
    app->takePendingFiles();
    const QString first = QDir::temp().filePath(QStringLiteral("first.hcad"));
    const QString second = QDir::temp().filePath(QStringLiteral("second.hzpart"));
    for (const QString& file : {first, second}) {
        QFileOpenEvent event(file);
        QCoreApplication::sendEvent(app, &event);
    }
    EXPECT_EQ(app->takePendingFiles(), (QStringList{first, second}));
    EXPECT_TRUE(app->takePendingFiles().isEmpty()) << "taken once";

    // Once something listens, it hears the file, and nothing waits.
    QStringList heard;
    const auto connection = QObject::connect(app, &hz::ui::Application::fileOpenRequested,
                                             [&heard](const QString& file) { heard << file; });
    QFileOpenEvent third(first);
    QCoreApplication::sendEvent(app, &third);
    QObject::disconnect(connection);
    EXPECT_EQ(heard, QStringList{first});
    EXPECT_TRUE(app->takePendingFiles().isEmpty());
}
