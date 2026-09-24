// Fuzz target: what the PDM reads back from a shared folder anyone can write
// to: a revision archive's manifest, and a check-out lock. Any input may be
// refused (the archive then reads as corrupt, the lock as held by someone
// unknown); none may crash, hang or throw.

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>

#include "FuzzTempDir.h"
#include "horizon/pdm/RevisionArchive.h"
#include "horizon/pdm/VaultManifest.h"

extern "C" int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size) {
    static const hz::fuzz::TempDir dir("hz_fuzz_pdm");
    static const bool made = [] {
        std::filesystem::create_directories(dir.path() / "archive");
        std::filesystem::create_directories(dir.path() / "locks");
        return true;
    }();
    (void)made;

    // The input as an archive's manifest, over no blobs: every revision it
    // lists must read as missing, never as content.
    dir.write("archive/manifest.json", data, size);
    hz::pdm::RevisionArchive archive((dir.path() / "archive").string());
    if (archive.load()) {
        for (int i = 0; i <= archive.latestIndex() && i < 64; ++i) {
            std::string content;
            (void)archive.read(i, content);
        }
    }

    // The input as a document's lock file.
    dir.write("locks/part.lock", data, size);
    const hz::pdm::VaultManifest vault((dir.path() / "locks").string());
    (void)vault.status("part");
    (void)vault.isLockedByOther("part", "someone");
    return 0;
}
