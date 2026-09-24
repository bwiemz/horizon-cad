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

}  // namespace hz::io
