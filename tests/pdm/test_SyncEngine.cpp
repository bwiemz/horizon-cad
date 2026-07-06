#include <gtest/gtest.h>

#include <filesystem>
#include <string>

#include "horizon/pdm/RevisionArchive.h"
#include "horizon/pdm/SyncEngine.h"
#include "horizon/pdm/VaultManifest.h"

using namespace hz::pdm;
namespace fs = std::filesystem;

namespace {

/// Two fresh vault roots per test, cleaned up afterwards.
class SyncEngineTest : public ::testing::Test {
protected:
    void SetUp() override {
        const auto* info = ::testing::UnitTest::GetInstance()->current_test_info();
        const auto base = fs::temp_directory_path() / "hz_sync_tests" / info->name();
        m_local = (base / "local").string();
        m_remote = (base / "remote").string();
        fs::remove_all(base);
        fs::create_directories(m_local);
        fs::create_directories(m_remote);
    }

    void TearDown() override {
        std::error_code ec;
        fs::remove_all(fs::path(m_local).parent_path(), ec);
    }

    /// Commit @p content to a doc archive under @p root.
    static int commitTo(const std::string& root, const std::string& docId,
                        const std::string& content, const std::string& author = "test") {
        RevisionArchive archive((fs::path(root) / (docId + ".hzarchive")).string());
        archive.load();
        return archive.commit(content, author, "msg");
    }

    static int countOf(const std::string& root, const std::string& docId) {
        RevisionArchive archive((fs::path(root) / (docId + ".hzarchive")).string());
        archive.load();
        return archive.latestIndex() + 1;
    }

    static std::string contentOf(const std::string& root, const std::string& docId, int index) {
        RevisionArchive archive((fs::path(root) / (docId + ".hzarchive")).string());
        archive.load();
        std::string out;
        archive.contentAt(index, out);
        return out;
    }

    std::string m_local;
    std::string m_remote;
};

}  // namespace

TEST_F(SyncEngineTest, PushesNewDocumentToRemote) {
    commitTo(m_local, "bracket", "rev0");
    commitTo(m_local, "bracket", "rev1");

    FileSystemEndpoint remote(m_remote);
    SyncEngine engine(m_local, remote);
    const SyncReport report = engine.sync();

    EXPECT_TRUE(report.ok);
    EXPECT_EQ(report.pushed, 2);
    EXPECT_EQ(report.fetched, 0);
    EXPECT_TRUE(report.conflicts.empty());
    EXPECT_EQ(countOf(m_remote, "bracket"), 2);
    EXPECT_EQ(contentOf(m_remote, "bracket", 0), "rev0");
    EXPECT_EQ(contentOf(m_remote, "bracket", 1), "rev1");
}

TEST_F(SyncEngineTest, FetchesRemoteTail) {
    commitTo(m_local, "gear", "rev0");
    commitTo(m_remote, "gear", "rev0");
    commitTo(m_remote, "gear", "rev1");
    commitTo(m_remote, "gear", "rev2");

    FileSystemEndpoint remote(m_remote);
    SyncEngine engine(m_local, remote);
    const SyncReport report = engine.sync();

    EXPECT_TRUE(report.ok);
    EXPECT_EQ(report.fetched, 2);
    EXPECT_EQ(report.pushed, 0);
    EXPECT_EQ(countOf(m_local, "gear"), 3);
    EXPECT_EQ(contentOf(m_local, "gear", 2), "rev2");
}

TEST_F(SyncEngineTest, BidirectionalAcrossDocuments) {
    commitTo(m_local, "onlyLocal", "a");
    commitTo(m_remote, "onlyRemote", "b");

    FileSystemEndpoint remote(m_remote);
    SyncEngine engine(m_local, remote);
    const SyncReport report = engine.sync();

    EXPECT_TRUE(report.ok);
    EXPECT_EQ(report.pushed, 1);
    EXPECT_EQ(report.fetched, 1);
    EXPECT_EQ(countOf(m_remote, "onlyLocal"), 1);
    EXPECT_EQ(countOf(m_local, "onlyRemote"), 1);
}

TEST_F(SyncEngineTest, SecondSyncIsANoOp) {
    commitTo(m_local, "part", "rev0");
    FileSystemEndpoint remote(m_remote);
    SyncEngine engine(m_local, remote);
    engine.sync();

    const SyncReport second = engine.sync();
    EXPECT_TRUE(second.ok);
    EXPECT_EQ(second.pushed, 0);
    EXPECT_EQ(second.fetched, 0);
    EXPECT_TRUE(second.conflicts.empty());
}

TEST_F(SyncEngineTest, DivergentHistoriesConflictAndStayUntouched) {
    commitTo(m_local, "shaft", "base");
    commitTo(m_local, "shaft", "local-change");
    commitTo(m_remote, "shaft", "base");
    commitTo(m_remote, "shaft", "remote-change");

    FileSystemEndpoint remote(m_remote);
    SyncEngine engine(m_local, remote);
    const SyncReport report = engine.sync();

    ASSERT_EQ(report.conflicts.size(), 1u);
    EXPECT_EQ(report.conflicts[0], "shaft");
    EXPECT_EQ(report.pushed, 0);
    EXPECT_EQ(report.fetched, 0);
    // Neither side modified.
    EXPECT_EQ(contentOf(m_local, "shaft", 1), "local-change");
    EXPECT_EQ(contentOf(m_remote, "shaft", 1), "remote-change");
}

TEST_F(SyncEngineTest, RemoteLockByOtherUserBlocksPush) {
    commitTo(m_local, "hub", "rev0");

    const std::string manifestPath = (fs::path(m_remote) / "vault.manifest.json").string();
    VaultManifest locks(manifestPath);
    ASSERT_TRUE(locks.checkOut("hub", "alice"));

    FileSystemEndpoint remote(m_remote);
    SyncEngine bob(m_local, remote, &locks, "bob");
    const SyncReport report = bob.sync();

    ASSERT_EQ(report.conflicts.size(), 1u);
    EXPECT_EQ(report.conflicts[0], "locked:hub");
    EXPECT_EQ(countOf(m_remote, "hub"), 0);

    // The lock holder can push.
    SyncEngine alice(m_local, remote, &locks, "alice");
    const SyncReport ok = alice.sync();
    EXPECT_TRUE(ok.conflicts.empty());
    EXPECT_EQ(ok.pushed, 1);
    EXPECT_EQ(countOf(m_remote, "hub"), 1);
}

TEST_F(SyncEngineTest, SyncSurvivesOfflineEdits) {
    // Local-first: edits continue offline, next sync catches the remote up.
    FileSystemEndpoint remote(m_remote);
    SyncEngine engine(m_local, remote);

    commitTo(m_local, "arm", "rev0");
    engine.sync();
    commitTo(m_local, "arm", "rev1");  // "offline" edit
    commitTo(m_local, "arm", "rev2");

    const SyncReport report = engine.sync();
    EXPECT_EQ(report.pushed, 2);
    EXPECT_EQ(countOf(m_remote, "arm"), 3);
}
