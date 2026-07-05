#include <gtest/gtest.h>

#include <filesystem>
#include <string>

#include "horizon/pdm/VaultManifest.h"

using hz::pdm::LockState;
using hz::pdm::VaultManifest;

namespace {
class TempManifest {
public:
    explicit TempManifest(const std::string& name)
        : m_path(std::filesystem::temp_directory_path() / name) {
        std::filesystem::remove(m_path);
    }
    ~TempManifest() {
        std::error_code ec;
        std::filesystem::remove(m_path, ec);
    }
    std::string str() const { return m_path.string(); }

private:
    std::filesystem::path m_path;
};
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
