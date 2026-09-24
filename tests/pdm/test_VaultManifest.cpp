#include <gtest/gtest.h>

#include <atomic>
#include <filesystem>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

#include "horizon/pdm/VaultManifest.h"

using hz::pdm::LockState;
using hz::pdm::VaultManifest;

namespace {
/// A vault lock directory under the temp dir, removed with its contents.
class TempManifest {
public:
    explicit TempManifest(const std::string& name)
        : m_path(std::filesystem::temp_directory_path() / name) {
        std::filesystem::remove_all(m_path);
    }
    ~TempManifest() {
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
}  // namespace

// A free document can be checked out; the owner is then recorded.
TEST(VaultManifestTest, CheckOutAcquiresLock) {
    TempManifest m("hz_vault_acquire.json");
    VaultManifest vault(m.str());

    EXPECT_EQ(vault.lockOwner("part.hzpart"), "");
    EXPECT_TRUE(vault.checkOut("part.hzpart", "alice"));

    const LockState s = vault.status("part.hzpart");
    EXPECT_TRUE(s.locked);
    EXPECT_EQ(s.owner, "alice");
    EXPECT_FALSE(s.timestamp.empty());
}

// A second user cannot check out a document already held by another.
TEST(VaultManifestTest, OtherUserCannotCheckOut) {
    TempManifest m("hz_vault_contended.json");
    VaultManifest vault(m.str());

    ASSERT_TRUE(vault.checkOut("part.hzpart", "alice"));
    EXPECT_FALSE(vault.checkOut("part.hzpart", "bob"));
    EXPECT_TRUE(vault.isLockedByOther("part.hzpart", "bob"));
    EXPECT_FALSE(vault.isLockedByOther("part.hzpart", "alice"));

    // The holder re-checking-out their own document is idempotent (succeeds).
    EXPECT_TRUE(vault.checkOut("part.hzpart", "alice"));
}

// The owner can check in, after which another user may take the lock.
TEST(VaultManifestTest, CheckInReleasesForNextUser) {
    TempManifest m("hz_vault_release.json");
    VaultManifest vault(m.str());

    ASSERT_TRUE(vault.checkOut("part.hzpart", "alice"));
    EXPECT_FALSE(vault.checkIn("part.hzpart", "bob"));  // bob is not the holder
    EXPECT_TRUE(vault.checkIn("part.hzpart", "alice"));
    EXPECT_EQ(vault.lockOwner("part.hzpart"), "");

    EXPECT_TRUE(vault.checkOut("part.hzpart", "bob"));
    EXPECT_EQ(vault.lockOwner("part.hzpart"), "bob");
}

// Independent documents lock independently.
TEST(VaultManifestTest, LocksArePerDocument) {
    TempManifest m("hz_vault_perdoc.json");
    VaultManifest vault(m.str());

    ASSERT_TRUE(vault.checkOut("a.hzpart", "alice"));
    EXPECT_TRUE(vault.checkOut("b.hzpart", "bob"));  // different doc, free
    EXPECT_EQ(vault.lockOwner("a.hzpart"), "alice");
    EXPECT_EQ(vault.lockOwner("b.hzpart"), "bob");
}

// breakLock force-releases regardless of owner.
TEST(VaultManifestTest, BreakLockOverrides) {
    TempManifest m("hz_vault_break.json");
    VaultManifest vault(m.str());

    ASSERT_TRUE(vault.checkOut("part.hzpart", "alice"));
    vault.breakLock("part.hzpart");
    EXPECT_EQ(vault.lockOwner("part.hzpart"), "");
    EXPECT_TRUE(vault.checkOut("part.hzpart", "bob"));  // now free
}

// A second handle onto the same manifest file sees locks taken by the first —
// the shared-folder concurrency model.
TEST(VaultManifestTest, SharedManifestIsVisibleAcrossHandles) {
    TempManifest m("hz_vault_shared.json");
    VaultManifest alice(m.str());
    VaultManifest bob(m.str());

    ASSERT_TRUE(alice.checkOut("part.hzpart", "alice"));
    // Bob's independent handle reads the same file and sees the lock.
    EXPECT_TRUE(bob.isLockedByOther("part.hzpart", "bob"));
    EXPECT_FALSE(bob.checkOut("part.hzpart", "bob"));
}

// -- Atomic, fail-closed locks (Phase 119) ------------------------------------

// Many users racing for one document: exactly one gets it. The lock record
// used to be read, changed and rewritten, so several could win.
TEST(VaultManifestTest, RacingCheckOutsHaveExactlyOneWinner) {
    TempManifest m("hz_vault_race");
    constexpr int kUsers = 8;
    for (int round = 0; round < 20; ++round) {
        const std::string doc = "part" + std::to_string(round) + ".hzpart";
        std::atomic<int> winners{0};
        std::atomic<bool> go{false};
        std::vector<std::thread> users;
        users.reserve(kUsers);
        for (int u = 0; u < kUsers; ++u) {
            users.emplace_back([&, u] {
                VaultManifest vault(m.str());  // each user has their own handle
                while (!go.load()) std::this_thread::yield();
                if (vault.checkOut(doc, "user" + std::to_string(u))) ++winners;
            });
        }
        go = true;
        for (auto& t : users) t.join();
        EXPECT_EQ(winners.load(), 1) << doc;

        VaultManifest vault(m.str());
        const LockState s = vault.status(doc);
        EXPECT_TRUE(s.locked);
        EXPECT_FALSE(s.unreadable);
    }
}

// A lock whose owner cannot be read counts as held by someone else, for every
// user, until an administrator breaks it. It used to read as free.
TEST(VaultManifestTest, UnreadableLockFailsClosed) {
    for (const std::string text : {"{ not json", "", R"({"owner":""})", R"({"owner":7})", "[]"}) {
        TempManifest m("hz_vault_unreadable");
        VaultManifest vault(m.str());
        ASSERT_TRUE(vault.checkOut("part.hzpart", "alice"));
        writeText(m.path() / "part.hzpart.lock", text);

        const LockState s = vault.status("part.hzpart");
        EXPECT_TRUE(s.locked) << text;
        EXPECT_TRUE(s.unreadable) << text;
        EXPECT_EQ(vault.lockOwner("part.hzpart"), "") << text;
        EXPECT_TRUE(vault.isLockedByOther("part.hzpart", "alice")) << text;
        EXPECT_FALSE(vault.checkOut("part.hzpart", "alice")) << text;
        EXPECT_FALSE(vault.checkOut("part.hzpart", "bob")) << text;
        EXPECT_FALSE(vault.checkIn("part.hzpart", "alice")) << text;

        EXPECT_TRUE(vault.breakLock("part.hzpart")) << text;
        EXPECT_TRUE(vault.checkOut("part.hzpart", "bob")) << text;
    }
}

// Before Phase 119 the locks were one JSON file at this path. Such a vault
// reads as all-locked, so no document is taken twice across the upgrade.
TEST(VaultManifestTest, LegacyManifestFileFailsClosed) {
    TempManifest m("hz_vault_legacy.json");
    writeText(m.path(), R"({"part.hzpart":{"owner":"alice","timestamp":"2026-07-04T12:00:00Z"}})");

    VaultManifest vault(m.str());
    EXPECT_TRUE(vault.status("part.hzpart").unreadable);
    EXPECT_TRUE(vault.status("other.hzpart").unreadable);
    EXPECT_TRUE(vault.isLockedByOther("other.hzpart", "bob"));
    EXPECT_FALSE(vault.checkOut("other.hzpart", "bob"));
    EXPECT_TRUE(std::filesystem::is_regular_file(m.path())) << "the old manifest was replaced";
}

// Document ids name lock files, so they cannot leave the lock directory; a
// lock needs a named owner.
TEST(VaultManifestTest, RefusesUnusableIdsAndOwners) {
    TempManifest m("hz_vault_ids");
    VaultManifest vault(m.str());
    for (const std::string bad : {"../escape", "a/b", "a\\b", "..", ".", ""}) {
        EXPECT_FALSE(vault.checkOut(bad, "alice")) << bad;
        EXPECT_TRUE(vault.isLockedByOther(bad, "alice")) << bad;
        EXPECT_FALSE(vault.breakLock(bad)) << bad;
    }
    EXPECT_FALSE(std::filesystem::exists(m.path().parent_path() / "escape.lock"));
    EXPECT_FALSE(vault.checkOut("part.hzpart", ""));
    EXPECT_FALSE(vault.status("part.hzpart").locked);
}

// Breaking a lock that is not held is harmless and reports the document free.
TEST(VaultManifestTest, BreakingAFreeLockSucceeds) {
    TempManifest m("hz_vault_break_free");
    VaultManifest vault(m.str());
    EXPECT_TRUE(vault.breakLock("part.hzpart"));
    EXPECT_FALSE(vault.status("part.hzpart").locked);
}
