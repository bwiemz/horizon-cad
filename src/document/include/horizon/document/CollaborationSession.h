#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace hz::doc {

/// One participant in a live session.
struct Participant {
    std::string userId;
    std::string displayName;
    uint32_t colorRgb = 0;  ///< presence/selection tint (0xRRGGBB)
};

/// A held edit token: one feature plus the downstream features it covers.
struct EditToken {
    uint64_t featureId = 0;         ///< the feature being edited
    std::vector<uint64_t> covered;  ///< featureId + downstream ids
    std::string owner;              ///< holding participant
    std::string acquiredAt;         ///< ISO-8601 (diagnostics only)
};

/// A participant's live cursor/selection, for presence display.
struct Presence {
    double x = 0.0;
    double y = 0.0;
    uint64_t selectedFeature = 0;  ///< 0 = nothing selected
};

/// Transport-agnostic live-collaboration session state (Phase 70, roadmap
/// §7.6).
///
/// Feature-level pessimistic token locking — deliberately NOT OT/CRDT on the
/// feature tree: when a participant opens a feature for editing, that feature
/// and everything downstream of it (which a rebuild would regenerate) is
/// covered by their token; others observe in real time but cannot modify
/// covered features. Tokens release on confirm + rebuild. All state
/// serializes to JSON so a future WebSocket transport can broadcast snapshots
/// and deltas; every mutation here is deterministic and fully testable
/// offline.
class CollaborationSession {
public:
    CollaborationSession() = default;
    explicit CollaborationSession(std::string sessionId);

    const std::string& sessionId() const { return m_sessionId; }

    // -- Membership -----------------------------------------------------------

    /// Add a participant. Fails when the userId is already present.
    bool join(const Participant& participant);

    /// Remove a participant, releasing every token they hold.
    bool leave(const std::string& userId);

    const std::vector<Participant>& participants() const { return m_participants; }
    bool hasParticipant(const std::string& userId) const;

    // -- Feature edit tokens ----------------------------------------------------

    /// Acquire the edit token for @p featureId on behalf of @p userId.
    /// @p downstreamIds are the features after it in tree order (a rebuild
    /// regenerates them, so they are covered too). Fails when the requester
    /// is not a participant, or any covered feature is already covered by
    /// another participant's token. Re-acquiring one's own token succeeds.
    bool acquireToken(const std::string& userId, uint64_t featureId,
                      const std::vector<uint64_t>& downstreamIds = {});

    /// Release @p featureId's token — only its owner may (confirm + rebuild).
    bool releaseToken(const std::string& userId, uint64_t featureId);

    /// Owner of the token covering @p featureId, or "" when it is free.
    std::string coveringOwner(uint64_t featureId) const;

    /// True when @p userId may modify @p featureId: either it is uncovered,
    /// or the covering token belongs to @p userId.
    bool canEdit(const std::string& userId, uint64_t featureId) const;

    const std::vector<EditToken>& tokens() const { return m_tokens; }

    // -- Presence ---------------------------------------------------------------

    /// Update a participant's live cursor/selection (no-op for non-members).
    void updatePresence(const std::string& userId, const Presence& presence);

    const std::map<std::string, Presence>& presence() const { return m_presence; }

    // -- Serialization (transport payloads) ---------------------------------------

    std::string toJson() const;

    /// Parse a snapshot produced by toJson(). Returns false on malformed
    /// input, leaving @p out untouched.
    static bool fromJson(const std::string& text, CollaborationSession& out);

private:
    std::string m_sessionId;
    std::vector<Participant> m_participants;
    std::vector<EditToken> m_tokens;
    std::map<std::string, Presence> m_presence;
};

}  // namespace hz::doc
