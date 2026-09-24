#include "horizon/pdm/RevisionArchive.h"

#include <cstdint>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <sstream>
#include <utility>

#include "horizon/fileio/AtomicFile.h"
#include "horizon/pdm/Sha256.h"

namespace hz::pdm {

namespace {

namespace fs = std::filesystem;

/// Current UTC time as an ISO-8601 string (e.g. "2026-07-04T12:34:56Z").
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

bool readFile(const std::string& path, std::string& out) {
    std::ifstream in(path, std::ios::binary);
    if (!in) return false;
    std::ostringstream ss;
    ss << in.rdbuf();
    out = ss.str();
    return true;
}

/// The 64-bit FNV-1a hash archives recorded before Phase 119, as 16 hex
/// digits. Still computed to verify their revisions.
std::string fnv1aHex(const std::string& content) {
    std::uint64_t h = 1469598103934665603ULL;
    for (unsigned char c : content) {
        h ^= c;
        h *= 1099511628211ULL;
    }
    char buf[17];
    std::snprintf(buf, sizeof(buf), "%016llx", static_cast<unsigned long long>(h));
    return buf;
}

bool isLowerHex(const std::string& s) {
    for (const char c : s) {
        if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'))) return false;
    }
    return true;
}

/// Whether @p hash has the form of a hash an archive records.
bool isRecordedHash(const std::string& hash) {
    return (hash.size() == 64 || hash.size() == 16) && isLowerHex(hash);
}

/// Whether @p dir holds revision blobs (`rev_<n>.blob`). An unreadable
/// directory counts as holding some: the caller fails closed on it.
bool hasBlobs(const std::string& dir) {
    std::error_code ec;
    if (fs::status(dir, ec).type() == fs::file_type::not_found) return false;
    fs::directory_iterator it(dir, ec);
    if (ec) return true;
    for (const fs::directory_iterator end; it != end; it.increment(ec)) {
        if (ec) return true;
        const std::string name = it->path().filename().string();
        if (name.rfind("rev_", 0) == 0 && name.size() > 9 &&
            name.compare(name.size() - 5, 5, ".blob") == 0) {
            return true;
        }
    }
    return static_cast<bool>(ec);
}

}  // namespace

RevisionArchive::RevisionArchive(std::string archiveDir) : m_dir(std::move(archiveDir)) {}

std::string RevisionArchive::manifestPath() const {
    return (fs::path(m_dir) / "manifest.json").string();
}

std::string RevisionArchive::blobPath(int index) const {
    return (fs::path(m_dir) / ("rev_" + std::to_string(index) + ".blob")).string();
}

std::string RevisionArchive::hashContent(const std::string& content) {
    return sha256Hex(content);
}

bool RevisionArchive::hashMatches(const std::string& content, const std::string& recordedHash) {
    if (recordedHash.size() == 64) return sha256Hex(content) == recordedHash;
    if (recordedHash.size() == 16) return fnv1aHex(content) == recordedHash;
    return false;
}

bool RevisionArchive::load() {
    m_revisions.clear();
    m_corrupt = false;

    std::error_code ec;
    const fs::file_type type = fs::status(manifestPath(), ec).type();
    if (type == fs::file_type::not_found) {
        // No manifest: a new archive, unless it already holds revisions.
        m_corrupt = hasBlobs(m_dir);
        return false;
    }
    std::string text;
    if (type != fs::file_type::regular || !readFile(manifestPath(), text)) {
        m_corrupt = true;
        return false;
    }

    std::vector<RevisionInfo> revisions;
    try {
        const nlohmann::json j = nlohmann::json::parse(text);
        if (!j.is_array()) {
            m_corrupt = true;
            return false;
        }
        for (const auto& e : j) {
            RevisionInfo r;
            r.index = e.at("index").get<int>();
            r.timestamp = e.value("timestamp", "");
            r.author = e.value("author", "");
            r.message = e.value("message", "");
            r.contentHash = e.at("hash").get<std::string>();
            // Revisions are numbered 0, 1, 2... in order; the blob of each is
            // found by its number, so any other sequence cannot be trusted.
            if (r.index != static_cast<int>(revisions.size()) || !isRecordedHash(r.contentHash)) {
                m_corrupt = true;
                return false;
            }
            revisions.push_back(std::move(r));
        }
    } catch (const nlohmann::json::exception&) {
        m_corrupt = true;
        return false;
    }
    m_revisions = std::move(revisions);
    return true;
}

bool RevisionArchive::writeManifest() const {
    nlohmann::json j = nlohmann::json::array();
    for (const RevisionInfo& r : m_revisions) {
        j.push_back({{"index", r.index},
                     {"timestamp", r.timestamp},
                     {"author", r.author},
                     {"message", r.message},
                     {"hash", r.contentHash}});
    }
    // Atomic replacement: a crash mid-write must not leave a truncated
    // manifest (which load() would report as a corrupt archive).
    return io::writeFileAtomically(fs::path(manifestPath()), j.dump(2));
}

int RevisionArchive::commit(const std::string& content, const std::string& author,
                            const std::string& message) {
    // Append to the history on disk, not to what this handle last saw.
    load();
    if (m_corrupt) return -1;

    // No-op if the content is byte-identical to the current head. When the
    // head cannot be read or does not match its hash (partially synced shared
    // folder, corrupt store) REFUSE the commit instead of blindly appending —
    // appending without the dedupe check can create duplicate consecutive
    // revisions, which replication treats as corruption.
    if (!m_revisions.empty()) {
        std::string head;
        if (read(latestIndex(), head) != ReadStatus::Ok) return -1;
        if (head == content) return latestIndex();
    }

    std::error_code ec;
    fs::create_directories(m_dir, ec);
    if (ec) return -1;

    // The blob first, then the manifest that makes it a revision: a crash in
    // between leaves an unlisted blob, which the next commit replaces.
    const int index = static_cast<int>(m_revisions.size());
    if (!io::writeFileAtomically(fs::path(blobPath(index)), content)) return -1;

    RevisionInfo r;
    r.index = index;
    r.timestamp = utcNow();
    r.author = author;
    r.message = message;
    r.contentHash = hashContent(content);
    m_revisions.push_back(std::move(r));

    if (!writeManifest()) {
        m_revisions.pop_back();  // keep memory consistent with disk on failure
        return -1;
    }
    return index;
}

RevisionArchive::ReadStatus RevisionArchive::read(int index, std::string& out) const {
    out.clear();
    if (index < 0 || index >= static_cast<int>(m_revisions.size())) return ReadStatus::Missing;
    if (!readFile(blobPath(index), out)) {
        out.clear();
        return ReadStatus::Missing;
    }
    return hashMatches(out, m_revisions[static_cast<size_t>(index)].contentHash)
               ? ReadStatus::Ok
               : ReadStatus::Corrupt;
}

}  // namespace hz::pdm
