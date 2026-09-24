#pragma once

#include <filesystem>
#include <string>
#include <string_view>

namespace hz::io {

/// Convert a UTF-8 path string — what `QString::toStdString()` produces — to a
/// filesystem path.
///
/// `std::filesystem::path` built from a narrow `std::string` reads it in the
/// process code page on Windows, so any non-ASCII character (`C:\Users\José`)
/// is mangled and the file cannot be opened. This reads it as UTF-8 on every
/// platform.
std::filesystem::path pathFromUtf8(std::string_view utf8);

/// Empty when `path` names an existing file; otherwise why it cannot be read
/// as one ("the file does not exist", "it is a folder, not a file"). Checked
/// before opening, because std::ifstream happily "opens" a directory on POSIX
/// and only fails on the first read, with a message about basic_filebuf.
std::string whyUnreadable(const std::filesystem::path& path);

/// Replace the file at `path` with `data` atomically: a crash, a full disk or a
/// concurrent reader sees either the previous file or the complete new one,
/// never a truncated mix.
///
/// The data is written to a temporary file in the target's directory, flushed
/// to stable storage, and renamed over the target. An existing target keeps its
/// permissions; a symlink is followed and the file it points to replaced.
///
/// Returns false on failure, with the reason in `error` when given. On failure
/// the original file is untouched and no temporary file is left behind.
bool writeFileAtomically(const std::filesystem::path& path, std::string_view data,
                         std::string* error = nullptr);

/// What createFileExclusively() did.
enum class ExclusiveCreate {
    Created,  ///< nothing was at the path; a file holding the data is now
    Exists,   ///< something already exists at the path, and is untouched
    Failed,   ///< the file could not be created or written (see `error`)
};

/// Create the file at `path` holding `data`, only if nothing exists there.
///
/// Checking that the path is free and claiming it are one step (O_EXCL,
/// CREATE_NEW), so of several processes racing to create the same file, one
/// gets Created and the others Exists. That holds on a network file system
/// that honours exclusive creation (NFS v3 and later, SMB), which makes the
/// file usable as a lock.
///
/// The data is written after the file is claimed, so another process can
/// briefly see it empty. A write that fails removes the file again.
ExclusiveCreate createFileExclusively(const std::filesystem::path& path, std::string_view data,
                                      std::string* error = nullptr);

}  // namespace hz::io
