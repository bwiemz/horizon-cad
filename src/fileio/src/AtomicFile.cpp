#include "horizon/fileio/AtomicFile.h"

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <random>
#include <system_error>

#ifdef _WIN32
#include <fcntl.h>
#include <io.h>
#include <share.h>
#include <sys/stat.h>
#else
#include <fcntl.h>
#include <unistd.h>
#endif

namespace hz::io {

namespace fs = std::filesystem;

std::filesystem::path pathFromUtf8(std::string_view utf8) {
    return fs::path(std::u8string(utf8.begin(), utf8.end()));
}

std::string whyUnreadable(const std::filesystem::path& path) {
    std::error_code ec;
    const fs::file_status status = fs::status(path, ec);
    if (ec || !fs::exists(status)) return "the file does not exist";
    if (fs::is_directory(status)) return "it is a folder, not a file";
    return {};
}

namespace {

void setError(std::string* error, const std::string& message) {
    if (error) *error = message;
}

/// The path as UTF-8 for messages. `path::string()` converts through the
/// Windows code page and throws on characters it cannot represent.
std::string displayPath(const fs::path& path) {
    const std::u8string u8 = path.u8string();
    return std::string(u8.begin(), u8.end());
}

std::string describe(const char* what, const fs::path& path, int err) {
    return std::string(what) + " '" + displayPath(path) +
           "': " + std::generic_category().message(err);
}

/// A hidden sibling of `target` with a random suffix, so concurrent saves of
/// the same file never share a temporary.
fs::path temporarySibling(const fs::path& target) {
    static std::atomic<std::uint64_t> counter{0};
    std::random_device rd;
    const std::uint64_t r = (static_cast<std::uint64_t>(rd()) << 32) ^ rd() ^
                            counter.fetch_add(1, std::memory_order_relaxed);
    char suffix[17];
    std::snprintf(suffix, sizeof(suffix), "%016llx", static_cast<unsigned long long>(r));

    fs::path name(".");
    name += target.filename();
    name += ".tmp-";
    name += suffix;
    return target.parent_path() / name;
}

#ifdef _WIN32

int openExclusive(const fs::path& p) {
    int fd = -1;
    const errno_t rc = _wsopen_s(&fd, p.c_str(), _O_WRONLY | _O_CREAT | _O_EXCL | _O_BINARY,
                                 _SH_DENYRW, _S_IREAD | _S_IWRITE);
    return rc == 0 ? fd : -1;
}

bool writeAll(int fd, std::string_view data) {
    constexpr std::size_t kChunk = 1u << 30;
    std::size_t done = 0;
    while (done < data.size()) {
        const auto chunk = static_cast<unsigned int>(std::min(kChunk, data.size() - done));
        const int n = _write(fd, data.data() + done, chunk);
        if (n <= 0) return false;
        done += static_cast<std::size_t>(n);
    }
    return true;
}

bool syncFile(int fd) {
    return _commit(fd) == 0;
}

bool closeFile(int fd) {
    return _close(fd) == 0;
}

void syncDirectory(const fs::path& /*dir*/) {
    // NTFS metadata journaling covers the rename; there is no directory fsync.
}

#else

int openExclusive(const fs::path& p) {
    return ::open(p.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0666);
}

bool writeAll(int fd, std::string_view data) {
    std::size_t done = 0;
    while (done < data.size()) {
        const ssize_t n = ::write(fd, data.data() + done, data.size() - done);
        if (n < 0) {
            if (errno == EINTR) continue;
            return false;
        }
        if (n == 0) return false;
        done += static_cast<std::size_t>(n);
    }
    return true;
}

bool syncFile(int fd) {
    return ::fsync(fd) == 0;
}

bool closeFile(int fd) {
    return ::close(fd) == 0;
}

/// Make the rename itself durable. Best effort: some filesystems refuse to
/// open or sync a directory, and the file contents are already on disk.
void syncDirectory(const fs::path& dir) {
    const int dfd = ::open(dir.empty() ? "." : dir.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC);
    if (dfd < 0) return;
    (void)::fsync(dfd);
    (void)::close(dfd);
}

#endif

}  // namespace

bool writeFileAtomically(const fs::path& path, std::string_view data, std::string* error) {
    std::error_code ec;

    // Replace what a symlink points to, not the link itself.
    fs::path target = path;
    if (fs::is_symlink(path, ec)) {
        const fs::path resolved = fs::canonical(path, ec);
        if (!ec) target = resolved;
    }
    if (target.filename().empty()) {
        setError(error, "not a file path: '" + displayPath(path) + "'");
        return false;
    }

    // O_EXCL on a random name: a collision only means another writer picked
    // the same suffix, so try a few more.
    fs::path tmp;
    int fd = -1;
    for (int attempt = 0; attempt < 8 && fd < 0; ++attempt) {
        tmp = temporarySibling(target);
        fd = openExclusive(tmp);
        if (fd < 0 && errno != EEXIST) break;
    }
    if (fd < 0) {
        setError(error, describe("cannot create a file next to", target, errno));
        return false;
    }

    const auto fail = [&](const std::string& message) {
        setError(error, message);
        std::error_code ignored;
        fs::remove(tmp, ignored);
        return false;
    };

    if (!writeAll(fd, data)) {
        const int err = errno;
        (void)closeFile(fd);
        return fail(describe("cannot write", target, err));
    }
    if (!syncFile(fd)) {
        const int err = errno;
        (void)closeFile(fd);
        return fail(describe("cannot flush", target, err));
    }
    if (!closeFile(fd)) return fail(describe("cannot finish writing", target, errno));

    // Keep the permissions of the file being replaced.
    const fs::file_status existing = fs::status(target, ec);
    if (!ec && fs::is_regular_file(existing)) {
        fs::permissions(tmp, existing.permissions(), fs::perm_options::replace, ec);
    }

    fs::rename(tmp, target, ec);
    if (ec) return fail("cannot replace '" + displayPath(target) + "': " + ec.message());

    syncDirectory(target.parent_path());
    return true;
}

}  // namespace hz::io
