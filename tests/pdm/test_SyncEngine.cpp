#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <iterator>
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

// -- Adversarial-review regressions ------------------------------------------

TEST_F(SyncEngineTest, InvalidDocIdsAreRejected) {
    FileSystemEndpoint remote(m_remote);
    SyncEngine engine(m_local, remote);

    for (const std::string bad : {"../evil", "a/b", "a\\b", "..", ""}) {
        const SyncReport report = engine.syncDocument(bad);
        ASSERT_EQ(report.conflicts.size(), 1u) << bad;
        EXPECT_EQ(report.conflicts[0], "invalid:" + bad);
    }
    // Nothing escaped the vault roots.
    EXPECT_FALSE(fs::exists(fs::path(m_remote).parent_path() / "evil.hzarchive"));
}

TEST_F(SyncEngineTest, CorruptRemoteBlobIsNotCommittedLocally) {
    commitTo(m_remote, "wheel", "rev0");
    commitTo(m_remote, "wheel", "rev1");

    // Tear the second blob after the manifest was written: the declared hash
    // no longer matches the bytes.
    const fs::path blob = fs::path(m_remote) / "wheel.hzarchive" / "rev_1.blob";
    ASSERT_TRUE(fs::exists(blob));
    {
        std::ofstream out(blob, std::ios::binary | std::ios::trunc);
        out << "GARBAGE";
    }

    FileSystemEndpoint remote(m_remote);
    SyncEngine engine(m_local, remote);
    const SyncReport report = engine.sync();

    ASSERT_EQ(report.conflicts.size(), 1u);
    EXPECT_EQ(report.conflicts[0], "corrupt:wheel");
    // rev0 replicated, the torn rev1 did NOT enter local history.
    EXPECT_EQ(countOf(m_local, "wheel"), 1);
    EXPECT_EQ(contentOf(m_local, "wheel", 0), "rev0");
}

TEST_F(SyncEngineTest, DuplicateConsecutiveRemoteRevisionsFailCleanly) {
    // A remote written by external tooling can hold consecutive identical
    // revisions; RevisionArchive::commit would dedupe the second and misalign
    // every later index. The engine must stop cleanly, not half-replicate.
    commitTo(m_remote, "plate", "A");
    commitTo(m_remote, "plate", "B");
    commitTo(m_remote, "plate", "C");
    // Forge the duplicate: overwrite rev_1's blob with rev_0's content and
    // patch its manifest hash to match (a "legitimate" external archive).
    const fs::path dir = fs::path(m_remote) / "plate.hzarchive";
    {
        std::ofstream out(dir / "rev_1.blob", std::ios::binary | std::ios::trunc);
        out << "A";
    }
    {
        std::ifstream in(dir / "manifest.json");
        std::string manifest((std::istreambuf_iterator<char>(in)),
                             std::istreambuf_iterator<char>());
        const std::string hashA = RevisionArchive::hashContent("A");
        const std::string hashB = RevisionArchive::hashContent("B");
        manifest.replace(manifest.find(hashB), hashB.size(), hashA);
        std::ofstream out(dir / "manifest.json", std::ios::trunc);
        out << manifest;
    }

    FileSystemEndpoint remote(m_remote);
    SyncEngine engine(m_local, remote);
    const SyncReport report = engine.sync();

    ASSERT_EQ(report.conflicts.size(), 1u);
    EXPECT_EQ(report.conflicts[0], "corrupt:plate");
    // The dedupe skip was detected: local holds only the aligned prefix.
    EXPECT_EQ(countOf(m_local, "plate"), 1);
}

namespace {

/// Endpoint wrapper that injects a concurrent remote push between the
/// engine's prefix check and its first append.
class RacingEndpoint : public SyncEndpoint {
public:
    RacingEndpoint(FileSystemEndpoint& inner, std::string docId, std::string sneak)
        : m_inner(inner), m_docId(std::move(docId)), m_sneak(std::move(sneak)) {}

    std::vector<std::string> listDocuments() override { return m_inner.listDocuments(); }
    int revisionCount(const std::string& docId) override {
        const int count = m_inner.revisionCount(docId);
        if (!m_raced && docId == m_docId && count > 0) {
            // Another machine appends right after the engine's initial count
            // (the prefix scan) — the engine's later pre-push recount then
            // observes the moved remote and must refuse to blind-append.
            RevisionInfo info;
            info.author = "other-machine";
            info.message = "raced";
            m_inner.pushRevision(docId, info, m_sneak);
            m_raced = true;
            return count;  // the engine's snapshot predates the other push
        }
        return m_inner.revisionCount(docId);
    }
    bool fetchRevision(const std::string& docId, int index, RevisionInfo& info,
                       std::string& content) override {
        return m_inner.fetchRevision(docId, index, info, content);
    }
    bool pushRevision(const std::string& docId, const RevisionInfo& info,
                      const std::string& content) override {
        return m_inner.pushRevision(docId, info, content);
    }

private:
    FileSystemEndpoint& m_inner;
    std::string m_docId;
    std::string m_sneak;
    bool m_raced = false;
};

}  // namespace

TEST_F(SyncEngineTest, ConcurrentRemotePushIsDetectedBeforeAppending) {
    commitTo(m_local, "axle", "base");
    commitTo(m_local, "axle", "mine");
    commitTo(m_remote, "axle", "base");

    FileSystemEndpoint inner(m_remote);
    RacingEndpoint racing(inner, "axle", "theirs");
    SyncEngine engine(m_local, racing);
    const SyncReport report = engine.sync();

    // The recount guard sees the remote moved and refuses to blind-append.
    bool flagged = false;
    for (const auto& c : report.conflicts) {
        if (c == "raced:axle") flagged = true;
    }
    EXPECT_TRUE(flagged) << "concurrent push was not detected";
    // The other machine's revision survived; ours was not blindly written
    // over it.
    EXPECT_EQ(contentOf(m_remote, "axle", 1), "theirs");
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
