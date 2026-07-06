#include "horizon/document/CollaborationSession.h"

#include <algorithm>
#include <chrono>
#include <ctime>
#include <nlohmann/json.hpp>

namespace hz::doc {

using nlohmann::json;

namespace {

std::string nowIso8601() {
    const auto now = std::chrono::system_clock::now();
    const std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm utc{};
#if defined(_WIN32)
    gmtime_s(&utc, &t);
#else
    gmtime_r(&t, &utc);
#endif
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &utc);
    return buf;
}

}  // namespace

CollaborationSession::CollaborationSession(std::string sessionId)
    : m_sessionId(std::move(sessionId)) {}

// -- Membership ---------------------------------------------------------------

bool CollaborationSession::hasParticipant(const std::string& userId) const {
    return std::any_of(m_participants.begin(), m_participants.end(),
                       [&](const Participant& p) { return p.userId == userId; });
}

bool CollaborationSession::join(const Participant& participant) {
    if (participant.userId.empty() || hasParticipant(participant.userId)) return false;
    m_participants.push_back(participant);
    return true;
}

bool CollaborationSession::leave(const std::string& userId) {
    const auto it = std::remove_if(m_participants.begin(), m_participants.end(),
                                   [&](const Participant& p) { return p.userId == userId; });
    if (it == m_participants.end()) return false;
    m_participants.erase(it, m_participants.end());

    m_tokens.erase(std::remove_if(m_tokens.begin(), m_tokens.end(),
                                  [&](const EditToken& t) { return t.owner == userId; }),
                   m_tokens.end());
    m_presence.erase(userId);
    return true;
}

// -- Feature edit tokens ---------------------------------------------------------

bool CollaborationSession::acquireToken(const std::string& userId, uint64_t featureId,
                                        const std::vector<uint64_t>& downstreamIds) {
    if (!hasParticipant(userId) || featureId == 0) return false;

    std::vector<uint64_t> covered;
    covered.push_back(featureId);
    for (uint64_t id : downstreamIds) {
        if (id != 0 && std::find(covered.begin(), covered.end(), id) == covered.end()) {
            covered.push_back(id);
        }
    }

    // Every requested feature must be free or already owned by the requester.
    for (uint64_t id : covered) {
        const std::string owner = coveringOwner(id);
        if (!owner.empty() && owner != userId) return false;
    }

    // Replace any existing token of this user on the same primary feature.
    m_tokens.erase(std::remove_if(m_tokens.begin(), m_tokens.end(),
                                  [&](const EditToken& t) {
                                      return t.owner == userId && t.featureId == featureId;
                                  }),
                   m_tokens.end());

    EditToken token;
    token.featureId = featureId;
    token.covered = std::move(covered);
    token.owner = userId;
    token.acquiredAt = nowIso8601();
    m_tokens.push_back(std::move(token));
    return true;
}

bool CollaborationSession::releaseToken(const std::string& userId, uint64_t featureId) {
    const auto it = std::find_if(m_tokens.begin(), m_tokens.end(), [&](const EditToken& t) {
        return t.featureId == featureId && t.owner == userId;
    });
    if (it == m_tokens.end()) return false;
    m_tokens.erase(it);
    return true;
}

std::string CollaborationSession::coveringOwner(uint64_t featureId) const {
    for (const EditToken& token : m_tokens) {
        if (std::find(token.covered.begin(), token.covered.end(), featureId) !=
            token.covered.end()) {
            return token.owner;
        }
    }
    return {};
}

bool CollaborationSession::canEdit(const std::string& userId, uint64_t featureId) const {
    if (!hasParticipant(userId)) return false;
    const std::string owner = coveringOwner(featureId);
    return owner.empty() || owner == userId;
}

// -- Presence ---------------------------------------------------------------------

void CollaborationSession::updatePresence(const std::string& userId, const Presence& presence) {
    if (!hasParticipant(userId)) return;
    m_presence[userId] = presence;
}

// -- Serialization -------------------------------------------------------------------

std::string CollaborationSession::toJson() const {
    json root;
    root["session"] = m_sessionId;

    json members = json::array();
    for (const Participant& p : m_participants) {
        members.push_back(
            {{"userId", p.userId}, {"displayName", p.displayName}, {"colorRgb", p.colorRgb}});
    }
    root["participants"] = members;

    json tokens = json::array();
    for (const EditToken& t : m_tokens) {
        tokens.push_back({{"featureId", t.featureId},
                          {"covered", t.covered},
                          {"owner", t.owner},
                          {"acquiredAt", t.acquiredAt}});
    }
    root["tokens"] = tokens;

    json presence = json::object();
    for (const auto& [userId, p] : m_presence) {
        presence[userId] = {{"x", p.x}, {"y", p.y}, {"selectedFeature", p.selectedFeature}};
    }
    root["presence"] = presence;

    return root.dump();
}

bool CollaborationSession::fromJson(const std::string& text, CollaborationSession& out) {
    json root;
    try {
        root = json::parse(text);
    } catch (...) {
        return false;
    }
    if (!root.is_object() || !root.contains("session") || !root["session"].is_string()) {
        return false;
    }

    CollaborationSession session(root["session"].get<std::string>());
    try {
        for (const auto& m : root.value("participants", json::array())) {
            Participant p;
            p.userId = m.value("userId", "");
            p.displayName = m.value("displayName", "");
            p.colorRgb = m.value("colorRgb", 0u);
            if (!p.userId.empty()) session.m_participants.push_back(std::move(p));
        }
        for (const auto& t : root.value("tokens", json::array())) {
            EditToken token;
            token.featureId = t.value("featureId", uint64_t{0});
            token.covered = t.value("covered", std::vector<uint64_t>{});
            token.owner = t.value("owner", "");
            token.acquiredAt = t.value("acquiredAt", "");
            if (token.featureId != 0 && !token.owner.empty()) {
                session.m_tokens.push_back(std::move(token));
            }
        }
        if (root.contains("presence") && root["presence"].is_object()) {
            for (const auto& [userId, p] : root["presence"].items()) {
                Presence presence;
                presence.x = p.value("x", 0.0);
                presence.y = p.value("y", 0.0);
                presence.selectedFeature = p.value("selectedFeature", uint64_t{0});
                session.m_presence[userId] = presence;
            }
        }
    } catch (...) {
        return false;
    }

    out = std::move(session);
    return true;
}

}  // namespace hz::doc
