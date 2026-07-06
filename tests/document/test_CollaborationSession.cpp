#include <gtest/gtest.h>

#include "horizon/document/CollaborationSession.h"

using namespace hz::doc;

namespace {

Participant user(const std::string& id, uint32_t color = 0xFF0000) {
    Participant p;
    p.userId = id;
    p.displayName = id + " display";
    p.colorRgb = color;
    return p;
}

}  // namespace

TEST(CollaborationSessionTest, MembershipRules) {
    CollaborationSession session("s1");
    EXPECT_TRUE(session.join(user("alice")));
    EXPECT_FALSE(session.join(user("alice"))) << "duplicate userId must be refused";
    EXPECT_FALSE(session.join(Participant{})) << "empty userId must be refused";
    EXPECT_TRUE(session.join(user("bob", 0x00FF00)));
    EXPECT_EQ(session.participants().size(), 2u);

    EXPECT_TRUE(session.leave("alice"));
    EXPECT_FALSE(session.leave("alice"));
    EXPECT_FALSE(session.hasParticipant("alice"));
}

TEST(CollaborationSessionTest, TokenCoversDownstreamFeatures) {
    CollaborationSession session("s1");
    session.join(user("alice"));
    session.join(user("bob"));

    // Alice edits feature 10; features 11 and 12 are downstream.
    ASSERT_TRUE(session.acquireToken("alice", 10, {11, 12}));

    EXPECT_EQ(session.coveringOwner(10), "alice");
    EXPECT_EQ(session.coveringOwner(12), "alice");
    EXPECT_TRUE(session.canEdit("alice", 11));
    EXPECT_FALSE(session.canEdit("bob", 11)) << "downstream features are locked for others";
    EXPECT_TRUE(session.canEdit("bob", 99)) << "uncovered features stay editable";

    // Bob cannot take any covered feature, even as the primary.
    EXPECT_FALSE(session.acquireToken("bob", 12));
    EXPECT_FALSE(session.acquireToken("bob", 5, {10}));

    // Release on confirm + rebuild frees everything.
    EXPECT_TRUE(session.releaseToken("alice", 10));
    EXPECT_TRUE(session.acquireToken("bob", 12));
}

TEST(CollaborationSessionTest, OnlyOwnerReleases) {
    CollaborationSession session("s1");
    session.join(user("alice"));
    session.join(user("bob"));
    ASSERT_TRUE(session.acquireToken("alice", 7));

    EXPECT_FALSE(session.releaseToken("bob", 7));
    EXPECT_EQ(session.coveringOwner(7), "alice");
    EXPECT_TRUE(session.releaseToken("alice", 7));
    EXPECT_TRUE(session.coveringOwner(7).empty());
}

TEST(CollaborationSessionTest, NonParticipantsCanNeitherAcquireNorEdit) {
    CollaborationSession session("s1");
    session.join(user("alice"));
    EXPECT_FALSE(session.acquireToken("mallory", 1));
    EXPECT_FALSE(session.canEdit("mallory", 1));
}

TEST(CollaborationSessionTest, LeavingReleasesAllTokens) {
    CollaborationSession session("s1");
    session.join(user("alice"));
    session.join(user("bob"));
    ASSERT_TRUE(session.acquireToken("alice", 1, {2}));
    ASSERT_TRUE(session.acquireToken("alice", 5));

    session.leave("alice");
    EXPECT_TRUE(session.coveringOwner(1).empty());
    EXPECT_TRUE(session.coveringOwner(5).empty());
    EXPECT_TRUE(session.acquireToken("bob", 2));
}

TEST(CollaborationSessionTest, ReacquiringOwnTokenUpdatesCoverage) {
    CollaborationSession session("s1");
    session.join(user("alice"));
    ASSERT_TRUE(session.acquireToken("alice", 3, {4}));
    // The sketch confirm added a feature: re-acquire with wider coverage.
    ASSERT_TRUE(session.acquireToken("alice", 3, {4, 5}));
    EXPECT_EQ(session.tokens().size(), 1u);
    EXPECT_EQ(session.coveringOwner(5), "alice");
}

TEST(CollaborationSessionTest, PresenceTracksMembersOnly) {
    CollaborationSession session("s1");
    session.join(user("alice"));

    Presence p;
    p.x = 12.5;
    p.y = -3.0;
    p.selectedFeature = 42;
    session.updatePresence("alice", p);
    session.updatePresence("ghost", p);  // ignored

    ASSERT_EQ(session.presence().size(), 1u);
    EXPECT_DOUBLE_EQ(session.presence().at("alice").x, 12.5);
    EXPECT_EQ(session.presence().at("alice").selectedFeature, 42u);
}

TEST(CollaborationSessionTest, JsonRoundTrip) {
    CollaborationSession session("design-review");
    session.join(user("alice", 0x336699));
    session.join(user("bob", 0x996633));
    session.acquireToken("alice", 10, {11});
    Presence p;
    p.x = 1.0;
    p.y = 2.0;
    p.selectedFeature = 10;
    session.updatePresence("bob", p);

    const std::string payload = session.toJson();

    CollaborationSession restored;
    ASSERT_TRUE(CollaborationSession::fromJson(payload, restored));
    EXPECT_EQ(restored.sessionId(), "design-review");
    EXPECT_EQ(restored.participants().size(), 2u);
    EXPECT_EQ(restored.coveringOwner(11), "alice");
    EXPECT_FALSE(restored.canEdit("bob", 10));
    ASSERT_EQ(restored.presence().count("bob"), 1u);
    EXPECT_DOUBLE_EQ(restored.presence().at("bob").y, 2.0);

    // Restored state stays live: bob still cannot take alice's features.
    EXPECT_FALSE(restored.acquireToken("bob", 11));
}

TEST(CollaborationSessionTest, FromJsonRejectsGarbage) {
    CollaborationSession out;
    EXPECT_FALSE(CollaborationSession::fromJson("not json", out));
    EXPECT_FALSE(CollaborationSession::fromJson("{}", out));
    EXPECT_FALSE(CollaborationSession::fromJson("[1,2,3]", out));
}
