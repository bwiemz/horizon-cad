#include <gtest/gtest.h>

#include <filesystem>
#include <string>

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

private:
    std::filesystem::path m_path;
};
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

// The content hash is stable and distinguishes different content.
TEST(RevisionArchiveTest, ContentHashIsStableAndDistinct) {
    EXPECT_EQ(RevisionArchive::hashContent("hello"), RevisionArchive::hashContent("hello"));
    EXPECT_NE(RevisionArchive::hashContent("hello"), RevisionArchive::hashContent("world"));
    EXPECT_EQ(RevisionArchive::hashContent("hello").size(), 16u);
}
