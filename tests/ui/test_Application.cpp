// hz::ui::Application contains exceptions that escape event handlers.

#include <gtest/gtest.h>

#include <QCoreApplication>
#include <QEvent>
#include <QMessageBox>
#include <QObject>
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
