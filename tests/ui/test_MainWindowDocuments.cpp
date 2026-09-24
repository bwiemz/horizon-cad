// Document-safety behaviour of the real MainWindow: modified markers, the
// quit prompt and the tab-close prompt. Runs on Qt's offscreen platform; the
// window is never shown, so no OpenGL context is created.

#include <gtest/gtest.h>

#include <QAbstractButton>
#include <QApplication>
#include <QMessageBox>
#include <QTabBar>
#include <QTimer>
#include <filesystem>
#include <memory>
#include <random>
#include <string>

#include "horizon/document/Commands.h"
#include "horizon/document/Document.h"
#include "horizon/document/UndoStack.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/ui/MainWindow.h"

// Qt keeps process-lifetime singletons that LeakSanitizer would report at exit.
// Disable leak detection only; every other AddressSanitizer check stays on.
#if defined(__SANITIZE_ADDRESS__)
#define HZ_ASAN_ACTIVE 1
#elif defined(__has_feature)
#if __has_feature(address_sanitizer)
#define HZ_ASAN_ACTIVE 1
#endif
#endif
#ifdef HZ_ASAN_ACTIVE
extern "C" const char* __lsan_default_options() {
    return "detect_leaks=0";
}
#endif

namespace fs = std::filesystem;
using hz::ui::MainWindow;

namespace {

/// Answers the next modal message box with `button`, and records whether one
/// appeared at all. Stops polling when destroyed, so a dialog that never
/// comes cannot leak into the next test.
class DialogResponder {
public:
    explicit DialogResponder(QMessageBox::StandardButton button) : m_button(button) {
        QObject::connect(&m_timer, &QTimer::timeout, [this] {
            auto* box = qobject_cast<QMessageBox*>(QApplication::activeModalWidget());
            if (!box) return;
            m_timer.stop();
            m_seen = true;
            m_text = box->text();
            box->button(m_button)->click();
        });
        m_timer.start(5);
    }
    bool seen() const { return m_seen; }
    const QString& text() const { return m_text; }

private:
    QMessageBox::StandardButton m_button;
    QTimer m_timer;
    bool m_seen = false;
    QString m_text;
};

void addLine(hz::doc::Document& doc) {
    auto line = std::make_shared<hz::draft::DraftLine>(hz::math::Vec2(0, 0), hz::math::Vec2(1, 0));
    doc.undoStack().push(std::make_unique<hz::doc::AddEntityCommand>(doc.draftDocument(), line));
}

QTabBar* tabBar(MainWindow& w) {
    auto* bar = w.findChild<QTabBar*>(QStringLiteral("documentTabs"));
    EXPECT_NE(bar, nullptr);
    return bar;
}

}  // namespace

TEST(MainWindowDocumentsTest, EditsMarkTheTabAndWindowModified) {
    MainWindow w;
    QTabBar* bar = tabBar(w);
    ASSERT_NE(bar, nullptr);
    const QString clean = bar->tabText(0);
    EXPECT_FALSE(clean.endsWith('*'));
    EXPECT_FALSE(w.isWindowModified());

    // Qt only draws the modified marker where the title has a "[*]"
    // placeholder, and warns on every setWindowModified() without one.
    EXPECT_TRUE(w.windowTitle().contains("[*]")) << w.windowTitle().toStdString();

    addLine(*w.activeDocument());
    EXPECT_EQ(bar->tabText(0), clean + " *");
    EXPECT_TRUE(w.isWindowModified());

    w.activeDocument()->undoStack().undo();
    EXPECT_EQ(bar->tabText(0), clean) << "undoing back to the saved state clears the marker";
    EXPECT_FALSE(w.isWindowModified());
}

TEST(MainWindowDocumentsTest, ClosingAnUnmodifiedWindowDoesNotAsk) {
    MainWindow w;
    DialogResponder responder(QMessageBox::Cancel);
    EXPECT_TRUE(w.close());
    EXPECT_FALSE(responder.seen());
}

TEST(MainWindowDocumentsTest, CancelKeepsTheWindowOpen) {
    MainWindow w;
    addLine(*w.activeDocument());

    DialogResponder responder(QMessageBox::Cancel);
    EXPECT_FALSE(w.close());
    EXPECT_TRUE(responder.seen()) << "quitting with unsaved work must ask";
    EXPECT_TRUE(w.activeDocument()->isDirty());
}

TEST(MainWindowDocumentsTest, DiscardClosesWithoutSaving) {
    MainWindow w;
    addLine(*w.activeDocument());

    DialogResponder responder(QMessageBox::Discard);
    EXPECT_TRUE(w.close());
    EXPECT_TRUE(responder.seen());
}

TEST(MainWindowDocumentsTest, SaveWritesTheFileThenCloses) {
    std::random_device rd;
    const fs::path path =
        fs::temp_directory_path() / ("hz_close_" + std::to_string(rd()) + ".hcad");

    MainWindow w;
    hz::doc::Document& doc = *w.activeDocument();
    doc.setFilePath(path.string());
    addLine(doc);

    DialogResponder responder(QMessageBox::Save);
    EXPECT_TRUE(w.close());
    EXPECT_TRUE(responder.seen());
    EXPECT_TRUE(fs::exists(path));
    EXPECT_FALSE(doc.isDirty());

    std::error_code ec;
    fs::remove(path, ec);
}

TEST(MainWindowDocumentsTest, TabCloseOffersSaveAndCancelKeepsTheTab) {
    MainWindow w;
    QTabBar* bar = tabBar(w);
    ASSERT_NE(bar, nullptr);
    addLine(*w.activeDocument());
    ASSERT_EQ(bar->count(), 1);

    {
        DialogResponder responder(QMessageBox::Cancel);
        emit bar->tabCloseRequested(0);
        EXPECT_TRUE(responder.seen());
        EXPECT_EQ(bar->count(), 1) << "cancel keeps the tab";
        EXPECT_TRUE(w.activeDocument()->isDirty());
    }
    {
        DialogResponder responder(QMessageBox::Discard);
        emit bar->tabCloseRequested(0);
        EXPECT_TRUE(responder.seen());
        // The window always keeps one document: a fresh, unmodified drawing
        // replaces the discarded one.
        ASSERT_EQ(bar->count(), 1);
        EXPECT_FALSE(w.activeDocument()->isDirty());
        EXPECT_FALSE(bar->tabText(0).endsWith('*'));
    }
}

int main(int argc, char** argv) {
    // Headless by default; an explicit QT_QPA_PLATFORM still wins.
    if (qEnvironmentVariableIsEmpty("QT_QPA_PLATFORM")) {
        qputenv("QT_QPA_PLATFORM", "offscreen");
    }
    QApplication app(argc, argv);
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
