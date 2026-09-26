#include <gtest/gtest.h>

#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include "horizon/pdm/RevisionArchive.h"

using hz::pdm::RevisionArchive;
using hz::pdm::RevisionInfo;

namespace {
// A unique temporary archive directory per test, cleaned up on destruction.
class TempDir {
public:
    explicit TempDir(const std::string& name)
        : m_path(std::filesystem::temp_directory_path() / name) {
        std::filesystem::remove_all(m_path);
    }
    ~TempDir() {
        std::error_code ec;
        std::filesystem::remove_all(m_path, ec);
    }
    std::string str() const { return m_path.string(); }
    std::filesystem::path path() const { return m_path; }

private:
    std::filesystem::path m_path;
};

void writeText(const std::filesystem::path& file, const std::string& text) {
    std::ofstream out(file, std::ios::binary | std::ios::trunc);
    out << text;
}

std::string readText(const std::filesystem::path& file) {
    std::ifstream in(file, std::ios::binary);
    return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

/// The 64-bit FNV-1a hash archives recorded before Phase 119.
std::string legacyHash(const std::string& content) {
    std::uint64_t h = 1469598103934665603ULL;
    for (unsigned char c : content) {
        h ^= c;
        h *= 1099511628211ULL;
    }
    char buf[17];
    std::snprintf(buf, sizeof(buf), "%016llx", static_cast<unsigned long long>(h));
    return buf;
}

/// An archive as a pre-Phase-119 build wrote it: FNV-1a hashes.
void writeLegacyArchive(const std::filesystem::path& dir, const std::vector<std::string>& revs) {
    std::filesystem::create_directories(dir);
    std::string manifest = "[";
    for (size_t i = 0; i < revs.size(); ++i) {
        writeText(dir / ("rev_" + std::to_string(i) + ".blob"), revs[i]);
        if (i > 0) manifest += ",";
        manifest += R"({"index":)" + std::to_string(i) +
                    R"(,"timestamp":"2026-07-04T12:00:00Z","author":"old","message":"m","hash":")" +
                    legacyHash(revs[i]) + R"("})";
    }
    writeText(dir / "manifest.json", manifest + "]");
}
}  // namespace

// Commits accumulate in order and their content reads back exactly.
TEST(RevisionArchiveTest, CommitsAndReadsBack) {
    TempDir dir("hz_pdm_commits");
    RevisionArchive archive(dir.str());

    const int r0 = archive.commit("v0 content", "alice", "initial");
    const int r1 = archive.commit("v1 content", "bob", "edit");
    EXPECT_EQ(r0, 0);
    EXPECT_EQ(r1, 1);
    EXPECT_EQ(archive.latestIndex(), 1);
    ASSERT_EQ(archive.history().size(), 2u);
    EXPECT_EQ(archive.history()[0].author, "alice");
    EXPECT_EQ(archive.history()[1].message, "edit");
    EXPECT_FALSE(archive.history()[0].timestamp.empty());

    std::string content;
    ASSERT_TRUE(archive.contentAt(0, content));
    EXPECT_EQ(content, "v0 content");
    ASSERT_TRUE(archive.contentAt(1, content));
    EXPECT_EQ(content, "v1 content");
    EXPECT_FALSE(archive.contentAt(2, content));  // out of range
}

// Committing unchanged content is a no-op that returns the current head.
TEST(RevisionArchiveTest, UnchangedContentIsNoOp) {
    TempDir dir("hz_pdm_noop");
    RevisionArchive archive(dir.str());

    const int r0 = archive.commit("same", "alice", "first");
    const int r1 = archive.commit("same", "alice", "again");  // identical content
    EXPECT_EQ(r0, 0);
    EXPECT_EQ(r1, 0);  // no new revision
    EXPECT_EQ(archive.history().size(), 1u);
}

// A reopened archive recovers its full history and content from disk.
TEST(RevisionArchiveTest, PersistsAcrossReopen) {
    TempDir dir("hz_pdm_persist");
    {
        RevisionArchive archive(dir.str());
        archive.commit("alpha", "alice", "a");
        archive.commit("beta", "bob", "b");
    }

    RevisionArchive reopened(dir.str());
    ASSERT_TRUE(reopened.load());
    ASSERT_EQ(reopened.history().size(), 2u);
    EXPECT_EQ(reopened.latestIndex(), 1);
    EXPECT_EQ(reopened.history()[1].author, "bob");

    std::string content;
    ASSERT_TRUE(reopened.contentAt(0, content));
    EXPECT_EQ(content, "alpha");

    // A follow-up commit continues numbering from the loaded head.
    EXPECT_EQ(reopened.commit("gamma", "carol", "c"), 2);
}

// load() on a directory with no manifest yields an empty archive.
TEST(RevisionArchiveTest, LoadEmptyArchive) {
    TempDir dir("hz_pdm_empty");
    RevisionArchive archive(dir.str());
    EXPECT_FALSE(archive.load());
    EXPECT_EQ(archive.latestIndex(), -1);
    EXPECT_TRUE(archive.history().empty());
}

// The content hash is SHA-256: stable, and different for different content.
TEST(RevisionArchiveTest, ContentHashIsSha256) {
    EXPECT_EQ(RevisionArchive::hashContent("abc"),
              "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
    EXPECT_NE(RevisionArchive::hashContent("hello"), RevisionArchive::hashContent("world"));
    EXPECT_TRUE(RevisionArchive::hashMatches("abc", RevisionArchive::hashContent("abc")));
    EXPECT_FALSE(RevisionArchive::hashMatches("abd", RevisionArchive::hashContent("abc")));
    // A hash of neither recorded form matches nothing.
    EXPECT_FALSE(RevisionArchive::hashMatches("abc", ""));
    EXPECT_FALSE(RevisionArchive::hashMatches("abc", "ba7816bf"));
}

// -- Failing closed (Phase 119) ----------------------------------------------

// A manifest that is not valid JSON used to read as an empty archive, and the
// next commit then wrote revision 0 over the history. Now the archive reads as
// corrupt and refuses to commit.
TEST(RevisionArchiveTest, CorruptManifestFailsClosed) {
    TempDir dir("hz_pdm_corrupt_manifest");
    {
        RevisionArchive archive(dir.str());
        ASSERT_EQ(archive.commit("first", "alice", "a"), 0);
        ASSERT_EQ(archive.commit("second", "alice", "b"), 1);
    }
    writeText(dir.path() / "manifest.json", "{ this is not json");

    RevisionArchive archive(dir.str());
    EXPECT_FALSE(archive.load());
    EXPECT_TRUE(archive.isCorrupt());
    EXPECT_TRUE(archive.history().empty());
    EXPECT_EQ(archive.commit("overwrite", "mallory", "x"), -1);
    EXPECT_EQ(readText(dir.path() / "rev_0.blob"), "first") << "the history was overwritten";
    EXPECT_EQ(readText(dir.path() / "manifest.json"), "{ this is not json");
}

// A manifest that parses but does not describe revisions 0, 1, 2... in order
// is as untrustworthy as one that does not parse.
TEST(RevisionArchiveTest, InconsistentManifestsFailClosed) {
    const std::string good = RevisionArchive::hashContent("x");
    const std::vector<std::string> manifests = {
        R"({"index":0})",                             // not an array
        R"([{"index":1,"hash":")" + good + R"("}])",  // does not start at 0
        R"([{"index":0,"hash":")" + good + R"("},{"index":0,"hash":")" + good + R"("}])",
        R"([{"index":0}])",                                      // no hash
        R"([{"index":0,"hash":"not-a-hash"}])",                  // not a hash
        R"([{"index":0,"hash":")" + good + R"(","author":7}])",  // wrong type
        R"([7])",                                                // not a revision
        // Not a whole number: nlohmann cast it to int, undefined behaviour.
        R"([{"index":1e300,"hash":")" + good + R"("}])",
        R"([{"index":-1,"hash":")" + good + R"("}])",
    };
    for (const std::string& manifest : manifests) {
        TempDir dir("hz_pdm_inconsistent");
        std::filesystem::create_directories(dir.path());
        writeText(dir.path() / "rev_0.blob", "x");
        writeText(dir.path() / "manifest.json", manifest);

        RevisionArchive archive(dir.str());
        EXPECT_FALSE(archive.load()) << manifest;
        EXPECT_TRUE(archive.isCorrupt()) << manifest;
        EXPECT_EQ(archive.commit("y", "alice", "m"), -1) << manifest;
    }
}

// Revisions with no manifest are a damaged archive, not a new one: committing
// would write revision 0 over the first of them.
TEST(RevisionArchiveTest, BlobsWithoutAManifestFailClosed) {
    TempDir dir("hz_pdm_orphan_blobs");
    {
        RevisionArchive archive(dir.str());
        ASSERT_EQ(archive.commit("precious", "alice", "a"), 0);
    }
    std::filesystem::remove(dir.path() / "manifest.json");

    RevisionArchive archive(dir.str());
    EXPECT_FALSE(archive.load());
    EXPECT_TRUE(archive.isCorrupt());
    EXPECT_EQ(archive.commit("new", "bob", "b"), -1);
    EXPECT_EQ(readText(dir.path() / "rev_0.blob"), "precious");
}

// Content is verified against its recorded hash on every read.
TEST(RevisionArchiveTest, TamperedContentReadsAsCorrupt) {
    TempDir dir("hz_pdm_tampered");
    RevisionArchive archive(dir.str());
    ASSERT_EQ(archive.commit("original", "alice", "a"), 0);
    writeText(dir.path() / "rev_0.blob", "tampered");

    std::string out;
    EXPECT_EQ(archive.read(0, out), RevisionArchive::ReadStatus::Corrupt);
    EXPECT_EQ(out, "tampered");  // the bytes, for diagnosis
    EXPECT_FALSE(archive.contentAt(0, out));
    EXPECT_EQ(archive.read(1, out), RevisionArchive::ReadStatus::Missing);
    EXPECT_TRUE(out.empty());

    // A head that fails verification cannot be compared against, so nothing
    // is appended after it.
    EXPECT_EQ(archive.commit("next", "alice", "b"), -1);
}

// Two handles on one archive: the one that loaded first appends after the
// other's commit instead of writing over it.
TEST(RevisionArchiveTest, StaleHandleAppendsInsteadOfOverwriting) {
    TempDir dir("hz_pdm_stale_handle");
    RevisionArchive first(dir.str());
    RevisionArchive second(dir.str());
    first.load();
    second.load();

    EXPECT_EQ(first.commit("from first", "alice", "a"), 0);
    EXPECT_EQ(second.commit("from second", "bob", "b"), 1);

    RevisionArchive reopened(dir.str());
    ASSERT_TRUE(reopened.load());
    ASSERT_EQ(reopened.history().size(), 2u);
    std::string content;
    ASSERT_TRUE(reopened.contentAt(0, content));
    EXPECT_EQ(content, "from first");
    ASSERT_TRUE(reopened.contentAt(1, content));
    EXPECT_EQ(content, "from second");
}

// An archive written before Phase 119 has FNV-1a hashes. It still loads and
// verifies, and grows with SHA-256 revisions.
TEST(RevisionArchiveTest, LegacyArchiveStillVerifies) {
    TempDir dir("hz_pdm_legacy");
    writeLegacyArchive(dir.path(), {"old zero", "old one"});

    RevisionArchive archive(dir.str());
    ASSERT_TRUE(archive.load());
    EXPECT_FALSE(archive.isCorrupt());
    std::string content;
    ASSERT_TRUE(archive.contentAt(1, content));
    EXPECT_EQ(content, "old one");

    EXPECT_EQ(archive.commit("new two", "alice", "c"), 2);
    EXPECT_EQ(archive.history()[1].contentHash.size(), 16u);
    EXPECT_EQ(archive.history()[2].contentHash.size(), 64u);

    // A tampered legacy revision is caught by its FNV-1a hash.
    writeText(dir.path() / "rev_0.blob", "changed");
    EXPECT_EQ(archive.read(0, content), RevisionArchive::ReadStatus::Corrupt);
}
