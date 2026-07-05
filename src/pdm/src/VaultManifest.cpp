#include "horizon/pdm/VaultManifest.h"

#include <ctime>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <sstream>
#include <utility>

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

/// Read the whole manifest as a JSON object (empty object if absent/malformed).
nlohmann::json readManifest(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) return nlohmann::json::object();
    std::ostringstream ss;
    ss << in.rdbuf();
    try {
        nlohmann::json j = nlohmann::json::parse(ss.str());
        if (j.is_object()) return j;
    } catch (const nlohmann::json::exception&) {
    }
    return nlohmann::json::object();
}

bool writeManifest(const std::string& path, const nlohmann::json& j) {
    const fs::path p(path);
    if (p.has_parent_path()) {
        std::error_code ec;
        fs::create_directories(p.parent_path(), ec);
    }
    std::ofstream out(path, std::ios::binary);
    if (!out) return false;
    out << j.dump(2);
    return static_cast<bool>(out);
}

LockState stateFromEntry(const nlohmann::json& manifest, const std::string& docId) {
    LockState s;
    auto it = manifest.find(docId);
    if (it != manifest.end() && it->is_object()) {
        s.owner = it->value("owner", "");
        s.timestamp = it->value("timestamp", "");
        s.locked = !s.owner.empty();
    }
    return s;
}

}  // namespace

VaultManifest::VaultManifest(std::string manifestPath) : m_path(std::move(manifestPath)) {}

LockState VaultManifest::status(const std::string& docId) const {
    return stateFromEntry(readManifest(m_path), docId);
}

std::string VaultManifest::lockOwner(const std::string& docId) const {
    return status(docId).owner;
}

bool VaultManifest::isLockedByOther(const std::string& docId, const std::string& user) const {
    const LockState s = status(docId);
    return s.locked && s.owner != user;
}

bool VaultManifest::checkOut(const std::string& docId, const std::string& user) {
    nlohmann::json manifest = readManifest(m_path);
    const LockState s = stateFromEntry(manifest, docId);
    if (s.locked && s.owner != user) return false;  // held by someone else

    manifest[docId] = {{"owner", user}, {"timestamp", utcNow()}};
    return writeManifest(m_path, manifest);
}

bool VaultManifest::checkIn(const std::string& docId, const std::string& user) {
    nlohmann::json manifest = readManifest(m_path);
    const LockState s = stateFromEntry(manifest, docId);
    if (!s.locked || s.owner != user) return false;  // not the holder

    manifest.erase(docId);
    return writeManifest(m_path, manifest);
}

void VaultManifest::breakLock(const std::string& docId) {
    nlohmann::json manifest = readManifest(m_path);
    if (manifest.contains(docId)) {
        manifest.erase(docId);
        writeManifest(m_path, manifest);
    }
}

}  // namespace hz::pdm
