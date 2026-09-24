#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <memory>
#include <random>
#include <sstream>
#include <string>

#include "horizon/document/Document.h"
#include "horizon/drafting/DraftText.h"
#include "horizon/fileio/AtomicFile.h"
#include "horizon/fileio/NativeFormat.h"

#ifndef _WIN32
#include <unistd.h>
#endif

namespace fs = std::filesystem;
using hz::io::createFileExclusively;
using hz::io::ExclusiveCreate;
using hz::io::pathFromUtf8;
using hz::io::writeFileAtomically;

namespace {

/// A fresh directory under the system temp dir, removed with its contents.
class ScratchDir {
public:
    ScratchDir() {
        std::random_device rd;
        m_path = fs::temp_directory_path() / ("hz_atomic_" + std::to_string(rd()));
        fs::create_directories(m_path);
    }
    ~ScratchDir() {
        std::error_code ec;
        fs::permissions(m_path, fs::perms::owner_all, fs::perm_options::add, ec);
        fs::remove_all(m_path, ec);
    }
    const fs::path& path() const { return m_path; }

private:
    fs::path m_path;
};

std::string readAll(const fs::path& p) {
    std::ifstream in(p, std::ios::binary);
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

std::size_t entryCount(const fs::path& dir) {
    std::size_t n = 0;
    for ([[maybe_unused]] const auto& e : fs::directory_iterator(dir)) ++n;
    return n;
}

bool runningAsRoot() {
#ifdef _WIN32
    return false;
#else
    return ::geteuid() == 0;
#endif
}

}  // namespace

TEST(AtomicFileTest, WritesANewFile) {
    ScratchDir dir;
    const fs::path target = dir.path() / "new.txt";
    std::string error;
    ASSERT_TRUE(writeFileAtomically(target, "hello", &error)) << error;
    EXPECT_EQ(readAll(target), "hello");
    EXPECT_EQ(entryCount(dir.path()), 1u) << "no temporary file left behind";
}

TEST(AtomicFileTest, ReplacesAnExistingFileCompletely) {
    ScratchDir dir;
    const fs::path target = dir.path() / "doc.hcad";
    ASSERT_TRUE(writeFileAtomically(target, std::string(4096, 'x')));
    ASSERT_TRUE(writeFileAtomically(target, "short"));
    EXPECT_EQ(readAll(target), "short") << "no tail of the longer old content survives";
    EXPECT_EQ(entryCount(dir.path()), 1u);
}

TEST(AtomicFileTest, WritesBinaryDataUnchanged) {
    ScratchDir dir;
    const fs::path target = dir.path() / "bytes.bin";
    const std::string bytes("\x00\r\n\xff\x1a\n", 6);
    ASSERT_TRUE(writeFileAtomically(target, bytes));
    EXPECT_EQ(readAll(target), bytes);
}

TEST(AtomicFileTest, EmptyContentMakesAnEmptyFile) {
    ScratchDir dir;
    const fs::path target = dir.path() / "empty";
    ASSERT_TRUE(writeFileAtomically(target, ""));
    EXPECT_TRUE(fs::exists(target));
    EXPECT_EQ(fs::file_size(target), 0u);
}

TEST(AtomicFileTest, MissingDirectoryFailsWithAReason) {
    ScratchDir dir;
    const fs::path target = dir.path() / "no" / "such" / "dir" / "file.txt";
    std::string error;
    EXPECT_FALSE(writeFileAtomically(target, "data", &error));
    EXPECT_FALSE(error.empty());
    EXPECT_FALSE(fs::exists(target));
}

TEST(AtomicFileTest, DirectoryTargetFailsAndLeavesNothingBehind) {
    ScratchDir dir;
    const fs::path target = dir.path() / "subdir";
    fs::create_directory(target);
    std::string error;
    EXPECT_FALSE(writeFileAtomically(target, "data", &error));
    EXPECT_FALSE(error.empty());
    EXPECT_TRUE(fs::is_directory(target));
    EXPECT_EQ(entryCount(dir.path()), 1u) << "the temporary file is removed on failure";
}

TEST(AtomicFileTest, FailedWriteLeavesTheOriginalIntact) {
    if (runningAsRoot()) GTEST_SKIP() << "root ignores directory permissions";
#ifdef _WIN32
    GTEST_SKIP() << "POSIX directory permissions";
#endif
    ScratchDir dir;
    const fs::path target = dir.path() / "precious.hcad";
    ASSERT_TRUE(writeFileAtomically(target, "original"));

    // A directory we cannot create files in: the temporary cannot be made.
    fs::permissions(dir.path(), fs::perms::owner_read | fs::perms::owner_exec,
                    fs::perm_options::replace);
    std::string error;
    EXPECT_FALSE(writeFileAtomically(target, "replacement", &error));
    fs::permissions(dir.path(), fs::perms::owner_all, fs::perm_options::replace);

    EXPECT_FALSE(error.empty());
    EXPECT_EQ(readAll(target), "original");
    EXPECT_EQ(entryCount(dir.path()), 1u);
}

TEST(AtomicFileTest, KeepsThePermissionsOfTheReplacedFile) {
#ifdef _WIN32
    GTEST_SKIP() << "POSIX permission bits";
#endif
    ScratchDir dir;
    const fs::path target = dir.path() / "mode.txt";
    ASSERT_TRUE(writeFileAtomically(target, "v1"));
    const auto mode = fs::perms::owner_read | fs::perms::owner_write | fs::perms::group_read;
    fs::permissions(target, mode, fs::perm_options::replace);

    ASSERT_TRUE(writeFileAtomically(target, "v2"));
    EXPECT_EQ(fs::status(target).permissions() & fs::perms::all, mode);
}

TEST(AtomicFileTest, WritesThroughASymlink) {
#ifdef _WIN32
    GTEST_SKIP() << "symlinks need privileges on Windows";
#endif
    ScratchDir dir;
    const fs::path real = dir.path() / "real.txt";
    const fs::path link = dir.path() / "link.txt";
    ASSERT_TRUE(writeFileAtomically(real, "old"));
    fs::create_symlink(real, link);

    ASSERT_TRUE(writeFileAtomically(link, "new"));
    EXPECT_TRUE(fs::is_symlink(link)) << "the link itself is kept";
    EXPECT_EQ(readAll(real), "new");
}

TEST(AtomicFileTest, NonAsciiUtf8PathsRoundTrip) {
    ScratchDir dir;
    // What QString::toStdString() hands over for a user folder like "Jürgen".
    const std::string utf8Name = "J\xC3\xBCrgen-\xE6\xB8\xAC\xE8\xA9\xA6.hcad";
    const fs::path target = dir.path() / pathFromUtf8(utf8Name);
    ASSERT_TRUE(writeFileAtomically(target, "data"));
    EXPECT_EQ(readAll(target), "data");
    // Compared as bytes: gtest prints a std::u8string through a function that
    // exists only when gtest itself was built as C++20.
    const std::u8string name = target.filename().u8string();
    EXPECT_EQ(std::string(name.begin(), name.end()), utf8Name);
}

// ---------------------------------------------------------------------------
// Native format
// ---------------------------------------------------------------------------

TEST(AtomicFileTest, NativeSaveSurvivesInvalidUtf8Text) {
    // DXF import stores text bytes in whatever code page the file used; a
    // Latin-1 "Ø" (0xD8) followed by ASCII is not valid UTF-8. Saving used to
    // throw from json::dump() after truncating the target.
    ScratchDir dir;
    const std::string path = (dir.path() / "latin1.hcad").string();

    hz::doc::Document doc;
    doc.draftDocument().addEntity(
        std::make_shared<hz::draft::DraftText>(hz::math::Vec2(0, 0), "\xD8 25 mm"));
    ASSERT_TRUE(hz::io::NativeFormat::save(path, doc));

    hz::doc::Document loaded;
    ASSERT_TRUE(hz::io::NativeFormat::load(path, loaded));
    ASSERT_EQ(loaded.draftDocument().entities().size(), 1u);
    const auto* text =
        dynamic_cast<const hz::draft::DraftText*>(loaded.draftDocument().entities()[0].get());
    ASSERT_NE(text, nullptr);
    EXPECT_EQ(text->text(), "\xEF\xBF\xBD 25 mm") << "the invalid byte becomes U+FFFD";
}

TEST(AtomicFileTest, FailedNativeSaveKeepsThePreviousFile) {
    if (runningAsRoot()) GTEST_SKIP() << "root ignores directory permissions";
#ifdef _WIN32
    GTEST_SKIP() << "POSIX directory permissions";
#endif
    ScratchDir dir;
    const std::string path = (dir.path() / "part.hcad").string();
    hz::doc::Document doc;
    doc.draftDocument().addEntity(
        std::make_shared<hz::draft::DraftText>(hz::math::Vec2(0, 0), "first version"));
    ASSERT_TRUE(hz::io::NativeFormat::save(path, doc));
    const std::string before = readAll(path);

    doc.draftDocument().addEntity(
        std::make_shared<hz::draft::DraftText>(hz::math::Vec2(0, 5), "second version"));
    fs::permissions(dir.path(), fs::perms::owner_read | fs::perms::owner_exec,
                    fs::perm_options::replace);
    const bool saved = hz::io::NativeFormat::save(path, doc);
    fs::permissions(dir.path(), fs::perms::owner_all, fs::perm_options::replace);

    EXPECT_FALSE(saved);
    EXPECT_EQ(readAll(path), before) << "a failed save must not touch the saved file";
}

// The first create claims the path; later ones find it taken and leave it
// alone. The vault's check-out locks rest on this.
TEST(AtomicFileTest, ExclusiveCreateClaimsThePathOnce) {
    ScratchDir dir;
    const fs::path lock = dir.path() / "part.lock";
    std::string error;
    ASSERT_EQ(createFileExclusively(lock, "alice", &error), ExclusiveCreate::Created) << error;
    EXPECT_EQ(readAll(lock), "alice");
    EXPECT_EQ(createFileExclusively(lock, "bob", &error), ExclusiveCreate::Exists);
    EXPECT_EQ(readAll(lock), "alice") << "the second create changed the file";

    // Anything at the path counts as taken, a directory included.
    fs::create_directories(dir.path() / "folder");
    EXPECT_EQ(createFileExclusively(dir.path() / "folder", "x"), ExclusiveCreate::Exists);
}

TEST(AtomicFileTest, ExclusiveCreateInAMissingDirectoryFails) {
    ScratchDir dir;
    std::string error;
    EXPECT_EQ(createFileExclusively(dir.path() / "missing" / "part.lock", "x", &error),
              ExclusiveCreate::Failed);
    EXPECT_NE(error.find("cannot create"), std::string::npos) << error;
    EXPECT_EQ(entryCount(dir.path()), 0u);
}
