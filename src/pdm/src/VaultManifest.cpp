#include "horizon/pdm/VaultManifest.h"

#include <ctime>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <sstream>
#include <utility>

#include "horizon/fileio/AtomicFile.h"

namespace hz::pdm {

namespace {

namespace fs = std::filesystem;

std::string utcNow() {
    const std::time_t t = std::time(nullptr);
    std::tm tm{};
#if defined(_WIN32)
    gmtime_s(&tm, &t);
#else
    gmtime_r(&t, &tm);
#endif
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &tm);
    return buf;
}

LockState unreadableLock() {
    LockState s;
    s.locked = true;
    s.unreadable = true;
    return s;
}

/// The lock of @p docId in @p dir, read from disk. Anything that is present
/// but cannot be read as a lock is an unreadable lock, never a free document.
LockState readLock(const fs::path& dir, const std::string& docId) {
    std::error_code ec;
    const fs::file_type dirType = fs::status(dir, ec).type();
    if (dirType == fs::file_type::not_found) return {};  // no lock ever taken
    if (dirType != fs::file_type::directory) return unreadableLock();

    const fs::path file = dir / (docId + ".lock");
    const fs::file_type type = fs::status(file, ec).type();
    if (type == fs::file_type::not_found) return {};  // free
    if (type != fs::file_type::regular) return unreadableLock();

    std::ifstream in(file, std::ios::binary);
    if (!in) return unreadableLock();
    std::ostringstream ss;
    ss << in.rdbuf();

    LockState s;
    s.locked = true;
    try {
        const nlohmann::json j = nlohmann::json::parse(ss.str());
        s.owner = j.at("owner").get<std::string>();
        s.timestamp = j.value("timestamp", "");
    } catch (const nlohmann::json::exception&) {
        // Empty (a lock being written this instant) or damaged.
        return unreadableLock();
    }
    if (s.owner.empty()) return unreadableLock();
    return s;
}

}  // namespace

bool isValidDocId(const std::string& docId) {
    if (docId.empty() || docId == "." || docId == "..") return false;
    for (const char c : docId) {
        const bool ok = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
                        (c >= '0' && c <= '9') || c == '.' || c == '_' || c == '-';
        if (!ok) return false;
    }
    return true;
}

VaultManifest::VaultManifest(std::string lockDir) : m_dir(std::move(lockDir)) {}

std::string VaultManifest::lockPath(const std::string& docId) const {
    return (fs::path(m_dir) / (docId + ".lock")).string();
}

LockState VaultManifest::status(const std::string& docId) const {
    if (!isValidDocId(docId)) return unreadableLock();
    return readLock(fs::path(m_dir), docId);
}

std::string VaultManifest::lockOwner(const std::string& docId) const {
    return status(docId).owner;
}

bool VaultManifest::isLockedByOther(const std::string& docId, const std::string& user) const {
    const LockState s = status(docId);
    return s.locked && (s.unreadable || s.owner != user);
}

bool VaultManifest::checkOut(const std::string& docId, const std::string& user) {
    if (!isValidDocId(docId) || user.empty()) return false;

    std::error_code ec;
    fs::create_directories(m_dir, ec);  // a failure shows up as Failed below

    const nlohmann::json lock = {{"owner", user}, {"timestamp", utcNow()}};
    switch (io::createFileExclusively(fs::path(lockPath(docId)), lock.dump(2))) {
        case io::ExclusiveCreate::Created:
            return true;
        case io::ExclusiveCreate::Exists: {
            // Checking out a document one already holds is idempotent.
            const LockState s = status(docId);
            return s.locked && !s.unreadable && s.owner == user;
        }
        case io::ExclusiveCreate::Failed:
            break;
    }
    return false;
}

bool VaultManifest::checkIn(const std::string& docId, const std::string& user) {
    if (!isValidDocId(docId) || user.empty()) return false;
    const LockState s = status(docId);
    if (!s.locked || s.unreadable || s.owner != user) return false;  // not the holder

    // Reading the owner and removing the file are two steps. Only a lock that
    // is broken and taken by someone else in between is lost: an
    // administrator's override racing the holder's own release.
    std::error_code ec;
    return fs::remove(fs::path(lockPath(docId)), ec) && !ec;
}

bool VaultManifest::breakLock(const std::string& docId) {
    if (!isValidDocId(docId)) return false;
    std::error_code ec;
    fs::remove(fs::path(lockPath(docId)), ec);
    return !ec && !status(docId).locked;
}

}  // namespace hz::pdm
