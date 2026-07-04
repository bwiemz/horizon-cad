#include "horizon/pdm/RevisionArchive.h"

#include <cstdint>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <sstream>
#include <utility>

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

bool writeFile(const std::string& path, const std::string& content) {
    std::ofstream out(path, std::ios::binary);
    if (!out) return false;
    out << content;
    return static_cast<bool>(out);
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
    // 64-bit FNV-1a — stable across runs and platforms.
    std::uint64_t h = 1469598103934665603ULL;
    for (unsigned char c : content) {
        h ^= c;
        h *= 1099511628211ULL;
    }
    char buf[17];
    std::snprintf(buf, sizeof(buf), "%016llx", static_cast<unsigned long long>(h));
    return buf;
}

bool RevisionArchive::load() {
    m_revisions.clear();
    std::string text;
    if (!readFile(manifestPath(), text)) return false;

    try {
        const nlohmann::json j = nlohmann::json::parse(text);
        if (!j.is_array()) return false;
        for (const auto& e : j) {
            RevisionInfo r;
            r.index = e.value("index", 0);
            r.timestamp = e.value("timestamp", "");
            r.author = e.value("author", "");
            r.message = e.value("message", "");
            r.contentHash = e.value("hash", "");
            m_revisions.push_back(std::move(r));
        }
    } catch (const nlohmann::json::exception&) {
        m_revisions.clear();  // malformed manifest -> treat as empty
        return false;
    }
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
    return writeFile(manifestPath(), j.dump(2));
}

int RevisionArchive::commit(const std::string& content, const std::string& author,
                            const std::string& message) {
    // No-op if the content is byte-identical to the current head.
    if (!m_revisions.empty()) {
        std::string head;
        if (contentAt(latestIndex(), head) && head == content) {
            return latestIndex();
        }
    }

    std::error_code ec;
    fs::create_directories(m_dir, ec);
    if (ec) return -1;

    const int index = static_cast<int>(m_revisions.size());
    if (!writeFile(blobPath(index), content)) return -1;

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

bool RevisionArchive::contentAt(int index, std::string& out) const {
    if (index < 0 || index >= static_cast<int>(m_revisions.size())) return false;
    return readFile(blobPath(index), out);
}

}  // namespace hz::pdm
