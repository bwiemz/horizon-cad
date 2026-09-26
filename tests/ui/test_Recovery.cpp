// Autosave snapshots and crash recovery.

#include <gtest/gtest.h>

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QMessageBox>
#include <QTabBar>
#include <QTemporaryDir>
#include <memory>
#include <string>

#include "UiTestSupport.h"
#include "horizon/document/Commands.h"
#include "horizon/document/Document.h"
#include "horizon/document/UndoStack.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/fileio/NativeFormat.h"
#include "horizon/ui/MainWindow.h"
#include "horizon/ui/Preferences.h"
#include "horizon/ui/RecoveryManager.h"

using hz::test::DialogResponder;
using hz::ui::MainWindow;
using hz::ui::RecoveryManager;

namespace {

void addLine(hz::doc::Document& doc) {
    auto line = std::make_shared<hz::draft::DraftLine>(hz::math::Vec2(0, 0), hz::math::Vec2(1, 0));
    doc.undoStack().push(std::make_unique<hz::doc::AddEntityCommand>(doc.draftDocument(), line));
}

int filesMatching(const QString& dir, const QString& pattern) {
    return static_cast<int>(QDir(dir).entryList({pattern}, QDir::Files).size());
}

/// What a crashed session leaves: a directory with a snapshot and its sidecar
/// and no live owner. `contents` replaces the snapshot's JSON (a damaged
/// snapshot); by default it is a drawing with one line. Returns the directory.
QString leaveCrashedSession(const QString& root, const QString& title, const QString& originalPath,
                            const QString& type = "hcad", const QByteArray& contents = {}) {
    const QString dir = QDir(root).filePath(QStringLiteral("crashed-session"));
    QDir().mkpath(dir);
    QByteArray json = contents;
    if (json.isEmpty()) {
        hz::doc::Document doc;
        addLine(doc);
        json = QByteArray::fromStdString(hz::io::NativeFormat::documentToJson(doc, false));
    }
    QFile snapshot(QDir(dir).filePath(QStringLiteral("7.") + type));
    EXPECT_TRUE(snapshot.open(QIODevice::WriteOnly));
    snapshot.write(json);
    snapshot.close();
    QFile sidecar(QDir(dir).filePath(QStringLiteral("7.json")));
    EXPECT_TRUE(sidecar.open(QIODevice::WriteOnly));
    sidecar.write(QJsonDocument(QJsonObject{{"originalPath", originalPath},
                                            {"title", title},
                                            {"type", type},
                                            {"savedAt", "2026-09-23T12:00:00.000Z"}})
                      .toJson());
    return dir;
}

/// Rewrite the recovery count in the sidecar leaveCrashedSession() wrote.
void setRecoveries(const QString& dir, int recoveries) {
    QFile sidecar(QDir(dir).filePath(QStringLiteral("7.json")));
    ASSERT_TRUE(sidecar.open(QIODevice::ReadOnly));
    QJsonObject meta = QJsonDocument::fromJson(sidecar.readAll()).object();
    sidecar.close();
    meta.insert(QStringLiteral("recoveries"), recoveries);
    ASSERT_TRUE(sidecar.open(QIODevice::WriteOnly | QIODevice::Truncate));
    sidecar.write(QJsonDocument(meta).toJson());
}

/// The recovery count in the one sidecar in `dir`, or -1.
int recoveriesIn(const QString& dir) {
    const QStringList sidecars = QDir(dir).entryList({QStringLiteral("*.json")}, QDir::Files);
    if (sidecars.size() != 1) return -1;
    QFile sidecar(QDir(dir).filePath(sidecars.front()));
    if (!sidecar.open(QIODevice::ReadOnly)) return -1;
    return QJsonDocument::fromJson(sidecar.readAll()).object().value("recoveries").toInt();
}

}  // namespace

// ---------------------------------------------------------------------------
// RecoveryManager
// ---------------------------------------------------------------------------

TEST(RecoveryManagerTest, SnapshotWritesTheDocumentAndRemoveDeletesIt) {
    QTemporaryDir root;
    RecoveryManager recovery(root.path());
    ASSERT_TRUE(recovery.isActive());
    hz::doc::Document doc;
    addLine(doc);

    ASSERT_TRUE(recovery.snapshot(3, doc, QStringLiteral("Drawing 1"), QString()));
    EXPECT_TRUE(QFileInfo::exists(QDir(recovery.sessionDirectory()).filePath("3.hcad")));
    EXPECT_TRUE(QFileInfo::exists(QDir(recovery.sessionDirectory()).filePath("3.json")));

    hz::doc::Document reloaded;
    ASSERT_TRUE(hz::io::NativeFormat::load(
        QDir(recovery.sessionDirectory()).filePath("3.hcad").toStdString(), reloaded));
    EXPECT_EQ(reloaded.draftDocument().entities().size(), 1u);

    recovery.remove(3);
    EXPECT_EQ(filesMatching(recovery.sessionDirectory(), "3.*"), 0);
}

TEST(RecoveryManagerTest, ATypeChangeLeavesOneSnapshot) {
    QTemporaryDir root;
    RecoveryManager recovery(root.path());
    hz::doc::Document doc;
    ASSERT_TRUE(recovery.snapshot(1, doc, QStringLiteral("Doc"), QString()));
    doc.setType(hz::doc::DocumentType::Part);  // Save As a part
    ASSERT_TRUE(recovery.snapshot(1, doc, QStringLiteral("Doc"), QString()));
    EXPECT_FALSE(QFileInfo::exists(QDir(recovery.sessionDirectory()).filePath("1.hcad")));
    EXPECT_TRUE(QFileInfo::exists(QDir(recovery.sessionDirectory()).filePath("1.hzpart")));
}

TEST(RecoveryManagerTest, ACleanExitRemovesTheSession) {
    QTemporaryDir root;
    QString session;
    {
        RecoveryManager recovery(root.path());
        hz::doc::Document doc;
        ASSERT_TRUE(recovery.snapshot(1, doc, QStringLiteral("Doc"), QString()));
        session = recovery.sessionDirectory();
        ASSERT_TRUE(QFileInfo::exists(session));
    }
    EXPECT_FALSE(QFileInfo::exists(session));
}

TEST(RecoveryManagerTest, ACrashedSessionIsClaimedAndDiscarded) {
    QTemporaryDir root;
    const QString crashed =
        leaveCrashedSession(root.path(), QStringLiteral("Plan"), QStringLiteral("/tmp/plan.hcad"));

    RecoveryManager recovery(root.path());
    const auto orphans = recovery.claimOrphans();
    ASSERT_EQ(orphans.size(), 1u);
    EXPECT_EQ(orphans[0].title, QStringLiteral("Plan"));
    EXPECT_EQ(orphans[0].type, QStringLiteral("hcad"));
    EXPECT_EQ(orphans[0].originalPath, QStringLiteral("/tmp/plan.hcad"));
    EXPECT_TRUE(QFileInfo::exists(orphans[0].snapshotPath));

    recovery.discardClaimedOrphans();
    EXPECT_FALSE(QFileInfo::exists(crashed));
}

TEST(RecoveryManagerTest, ARunningSessionIsNotClaimed) {
    QTemporaryDir root;
    RecoveryManager running(root.path());
    hz::doc::Document doc;
    ASSERT_TRUE(running.snapshot(1, doc, QStringLiteral("Doc"), QString()));

    RecoveryManager starting(root.path());  // a second instance of the app
    EXPECT_TRUE(starting.claimOrphans().empty());
    EXPECT_TRUE(QFileInfo::exists(running.sessionDirectory()));
}

TEST(RecoveryManagerTest, ACrashedSessionWithNothingToRecoverIsCleanedUp) {
    QTemporaryDir root;
    const QString empty = QDir(root.path()).filePath(QStringLiteral("empty-session"));
    QDir().mkpath(empty);
    RecoveryManager recovery(root.path());
    EXPECT_TRUE(recovery.claimOrphans().empty());
    EXPECT_FALSE(QFileInfo::exists(empty));
}

// A recovery that does not finish (opening the document stops the
// application) is counted in the snapshot it was reading, for the next start.
TEST(RecoveryManagerTest, ARecoveryThatDidNotFinishIsCountedNextTime) {
    QTemporaryDir root;
    leaveCrashedSession(root.path(), QStringLiteral("Plan"), QString());
    {
        RecoveryManager first(root.path());
        const auto orphans = first.claimOrphans();
        ASSERT_EQ(orphans.size(), 1u);
        EXPECT_EQ(orphans[0].recoveries, 0);
        first.noteRecoveryAttempt();
        // Opening it stops the application here: nothing is discarded.
    }
    RecoveryManager second(root.path());
    const auto orphans = second.claimOrphans();
    ASSERT_EQ(orphans.size(), 1u);
    EXPECT_EQ(orphans[0].recoveries, 1);
}

TEST(RecoveryManagerTest, ASessionThatCannotStartSaysWhy) {
    QTemporaryDir dir;
    const QString notADirectory = QDir(dir.path()).filePath(QStringLiteral("file"));
    QFile file(notADirectory);
    ASSERT_TRUE(file.open(QIODevice::WriteOnly));
    file.close();
    RecoveryManager recovery(notADirectory);
    EXPECT_FALSE(recovery.isActive());
    EXPECT_TRUE(recovery.problem().contains(QStringLiteral("cannot create")))
        << recovery.problem().toStdString();
}

// ---------------------------------------------------------------------------
// MainWindow autosave and recovery
// ---------------------------------------------------------------------------

// An autosave that cannot be written is shown, until one can be again. It
// used to go to the log alone, and a crash then found nothing to recover.
TEST(RecoveryWindowTest, AFailedAutosaveIsShown) {
    MainWindow w;
    auto* warning = w.findChild<QLabel*>(QStringLiteral("autosaveWarning"));
    ASSERT_NE(warning, nullptr);
    EXPECT_TRUE(warning->isHidden());
    addLine(*w.activeDocument());

    const QString session = w.recovery().sessionDirectory();
    const QFileDevice::Permissions writable = QFile::permissions(session);
    QFile::setPermissions(session, QFileDevice::ReadOwner | QFileDevice::ExeOwner);
    QFile probe(QDir(session).filePath(QStringLiteral("probe")));
    if (probe.open(QIODevice::WriteOnly)) {
        probe.close();
        probe.remove();
        QFile::setPermissions(session, writable);
        GTEST_SKIP() << "a read-only directory is still writable here (root, or Windows)";
    }
    w.autosave();
    const bool shown = !warning->isHidden();
    const QString text = warning->text();
    // Applying the preferences (any change) is not a write: the failure stands.
    w.applyPreferences(hz::ui::Preferences::current());
    const bool stillShown = !warning->isHidden();
    QFile::setPermissions(session, writable);
    EXPECT_TRUE(shown);
    EXPECT_EQ(text, QStringLiteral("Autosave failed"));
    EXPECT_TRUE(stillShown) << "hidden by applying the preferences";

    w.autosave();
    EXPECT_TRUE(warning->isHidden()) << "gone once autosave works again";
}

// A document recovered before, whose session then stopped too, may be what
// stops Horizon CAD: Recover is no longer the default, so pressing Enter at
// every start does not repeat it.
TEST(RecoveryWindowTest, ADocumentThatStoppedTheLastRecoveryIsNotRecoveredByDefault) {
    QString crashed;
    {
        MainWindow w;
        const QString root = QFileInfo(w.recovery().sessionDirectory()).path();
        crashed = leaveCrashedSession(root, QStringLiteral("Plan"), QString());
        setRecoveries(crashed, 1);
        DialogResponder later(QMessageBox::Cancel, QStringLiteral("Recover Documents"), 5000);
        w.offerRecovery();
        ASSERT_TRUE(later.seen());
        EXPECT_EQ(later.defaultButton(), QMessageBox::Cancel);
        EXPECT_TRUE(later.informativeText().contains(QStringLiteral("Plan")))
            << later.informativeText().toStdString();
    }
    EXPECT_TRUE(QFileInfo::exists(crashed)) << "kept, for when the user chooses";
    QDir(crashed).removeRecursively();
}

// Recovered documents are in this session's snapshots before the crashed
// session's are deleted, and counted: a crash from here on still finds them.
TEST(RecoveryWindowTest, RecoveredDocumentsAreSnapshotAtOnceAndCounted) {
    MainWindow w;
    const QString root = QFileInfo(w.recovery().sessionDirectory()).path();
    const QString crashed = leaveCrashedSession(root, QStringLiteral("Plan"), QString());
    DialogResponder recover(QMessageBox::Yes, QStringLiteral("Recover Documents"), 5000);
    w.offerRecovery();
    ASSERT_TRUE(recover.seen());
    EXPECT_EQ(recover.defaultButton(), QMessageBox::Yes) << "never stopped one before";
    EXPECT_FALSE(QFileInfo::exists(crashed));
    EXPECT_EQ(recoveriesIn(w.recovery().sessionDirectory()), 1);
    DialogResponder discard(QMessageBox::Discard);
    w.close();
}

TEST(RecoveryWindowTest, AutosaveWritesOnlyModifiedDocuments) {
    MainWindow w;
    const QString session = w.recovery().sessionDirectory();
    w.autosave();
    EXPECT_EQ(filesMatching(session, "*.json"), 0) << "nothing modified, nothing written";

    addLine(*w.activeDocument());
    w.autosave();
    EXPECT_EQ(filesMatching(session, "*.json"), 1);

    w.activeDocument()->undoStack().undo();  // back to unmodified
    w.autosave();
    EXPECT_EQ(filesMatching(session, "*.json"), 0) << "the snapshot of a clean document goes";
}

TEST(RecoveryWindowTest, SavingDropsTheSnapshot) {
    QTemporaryDir dir;
    MainWindow w;
    const QString session = w.recovery().sessionDirectory();
    hz::doc::Document& doc = *w.activeDocument();
    doc.setFilePath(QDir(dir.path()).filePath("saved.hcad").toStdString());
    addLine(doc);
    w.autosave();
    ASSERT_EQ(filesMatching(session, "*.json"), 1);

    DialogResponder responder(QMessageBox::Save);
    ASSERT_TRUE(w.close());
    EXPECT_EQ(filesMatching(session, "*.json"), 0);
}

TEST(RecoveryWindowTest, RecoveredDocumentsReopenModified) {
    MainWindow w;
    const QString root = QFileInfo(w.recovery().sessionDirectory()).path();
    const QString crashed =
        leaveCrashedSession(root, QStringLiteral("Plan"), QStringLiteral("/tmp/plan.hcad"));

    DialogResponder responder(QMessageBox::Yes, QStringLiteral("Recover Documents"), 5000);
    w.offerRecovery();
    ASSERT_TRUE(responder.seen());
    EXPECT_TRUE(responder.text().contains("Plan"));

    auto* tabs = w.findChild<QTabBar*>(QStringLiteral("documentTabs"));
    ASSERT_NE(tabs, nullptr);
    ASSERT_EQ(tabs->count(), 2);
    // Named after the file it came from, marked until saved.
    EXPECT_EQ(tabs->tabText(1), QStringLiteral("plan.hcad (recovered) *"));
    hz::doc::Document& recovered = *w.activeDocument();
    EXPECT_TRUE(recovered.isDirty()) << "recovered work is not yet saved";
    EXPECT_EQ(recovered.filePath(), "/tmp/plan.hcad") << "Save writes back where it came from";
    EXPECT_EQ(recovered.draftDocument().entities().size(), 1u);
    EXPECT_FALSE(QFileInfo::exists(crashed)) << "recovered snapshots are not offered twice";

    // Nothing left to offer.
    DialogResponder none(QMessageBox::Yes, QStringLiteral("Recover Documents"), 500);
    w.offerRecovery();
    EXPECT_FALSE(none.seen());

    DialogResponder discard(QMessageBox::Discard);
    w.close();
}

TEST(RecoveryWindowTest, LaterKeepsTheSnapshotsForNextTime) {
    QString crashed;
    {
        MainWindow w;
        const QString root = QFileInfo(w.recovery().sessionDirectory()).path();
        crashed = leaveCrashedSession(root, QStringLiteral("Plan"), QString());
        DialogResponder later(QMessageBox::Cancel, QStringLiteral("Recover Documents"), 5000);
        w.offerRecovery();
        ASSERT_TRUE(later.seen());
    }
    EXPECT_TRUE(QFileInfo::exists(crashed));
    QDir(crashed).removeRecursively();
}

TEST(RecoveryWindowTest, ADamagedSnapshotIsReportedAndNotOpened) {
    for (const QString& type : {QStringLiteral("hcad"), QStringLiteral("hzasm")}) {
        MainWindow w;
        const QString root = QFileInfo(w.recovery().sessionDirectory()).path();
        const QString crashed = leaveCrashedSession(root, QStringLiteral("Broken"), QString(), type,
                                                    QByteArray("{ truncated"));

        DialogResponder recover(QMessageBox::Yes, QStringLiteral("Recover Documents"), 5000);
        // The question box has no OK button, so this one answers only the
        // failure report that follows it.
        DialogResponder report(QMessageBox::Ok, QStringLiteral("Recover Documents"), 5000);
        w.offerRecovery();
        ASSERT_TRUE(recover.seen()) << type.toStdString();
        EXPECT_TRUE(report.seen()) << "the user learns it could not be recovered";
        EXPECT_TRUE(report.text().contains("Broken")) << report.text().toStdString();

        auto* tabs = w.findChild<QTabBar*>(QStringLiteral("documentTabs"));
        ASSERT_NE(tabs, nullptr);
        EXPECT_EQ(tabs->count(), 1) << "nothing half-loaded is opened";
        EXPECT_FALSE(QFileInfo::exists(crashed));
    }
}

// ---------------------------------------------------------------------------
// Crash reports at the next start (Phase 167)
// ---------------------------------------------------------------------------

namespace {

/// crash::reportDirectory() is a directory of the test's own while this
/// lives, with a report in it that a crash left.
class LeftCrashReport {
public:
    LeftCrashReport() {
        qputenv("HZ_CRASH_DIR", QFile::encodeName(m_dir.path()));
        QFile report(path());
        EXPECT_TRUE(report.open(QIODevice::WriteOnly));
        report.write("Horizon CAD stopped: SIGSEGV, a bad memory access\n");
    }
    ~LeftCrashReport() { qunsetenv("HZ_CRASH_DIR"); }
    LeftCrashReport(const LeftCrashReport&) = delete;
    LeftCrashReport& operator=(const LeftCrashReport&) = delete;

    QString directory() const { return m_dir.path(); }
    QString path() const { return m_dir.filePath(QStringLiteral("crash-20260926-120000-7.txt")); }

private:
    QTemporaryDir m_dir;
};

}  // namespace

TEST(CrashReportWindowTest, AReportIsOfferedOnceAndKept) {
    LeftCrashReport left;
    MainWindow w;
    DialogResponder close(QMessageBox::Close, QStringLiteral("Horizon CAD Stopped"), 5000);
    w.offerCrashReports();
    ASSERT_TRUE(close.seen());
    EXPECT_TRUE(close.text().contains(QStringLiteral("nothing has been sent")))
        << close.text().toStdString();
    EXPECT_TRUE(close.detailedText().contains(QStringLiteral("SIGSEGV")))
        << close.detailedText().toStdString();
    EXPECT_FALSE(QFileInfo::exists(left.path()));
    EXPECT_TRUE(QFileInfo::exists(left.path() + QStringLiteral(".shown"))) << "kept";

    DialogResponder again(QMessageBox::Close, QStringLiteral("Horizon CAD Stopped"), 300);
    w.offerCrashReports();
    again.waitForDialog(300);
    EXPECT_FALSE(again.seen()) << "offered once";
}

// Deleting the report deletes it and its minidump, and nothing else: the
// documents the crash left are recovered after it, from their own files.
TEST(CrashReportWindowTest, DeletingAReportLeavesTheRecoveryFiles) {
    LeftCrashReport left;
    const QString dump = left.path().chopped(4) + QStringLiteral(".dmp");
    QFile dumpFile(dump);
    ASSERT_TRUE(dumpFile.open(QIODevice::WriteOnly));
    dumpFile.close();

    MainWindow w;
    const QString crashed = leaveCrashedSession(QFileInfo(w.recovery().sessionDirectory()).path(),
                                                QStringLiteral("Plan"), QString());
    DialogResponder remove(QStringLiteral("Delete Report"), QStringLiteral("Horizon CAD Stopped"),
                           5000);
    w.offerCrashReports();
    ASSERT_TRUE(remove.seen());
    EXPECT_FALSE(QFileInfo::exists(left.path()));
    EXPECT_FALSE(QFileInfo::exists(left.path() + QStringLiteral(".shown")));
    EXPECT_FALSE(QFileInfo::exists(dump));
    EXPECT_TRUE(QDir(left.directory()).entryList(QDir::Files).isEmpty());

    EXPECT_TRUE(QFileInfo::exists(QDir(crashed).filePath(QStringLiteral("7.hcad"))));
    DialogResponder later(QMessageBox::Cancel, QStringLiteral("Recover Documents"), 5000);
    w.offerRecovery();
    EXPECT_TRUE(later.seen()) << "still offered";
    QDir(crashed).removeRecursively();
}
