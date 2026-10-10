// A half-edge structure that is not yet sound, or no longer sound, is the
// normal state of a kernel in the middle of building or undoing a feature.
// Every walk of a loop here followed `next` to the end of the list without
// looking, so a loop with a break in it read past a null pointer and took the
// application down; a loop whose chain never came back to where it started
// spun forever. Both were reached from ordinary inputs: a face whose loop was
// only partly built, and a solid after a kill had left a half-edge with no
// `next`.
#include <gtest/gtest.h>

#include <vector>

#include "horizon/topology/HalfEdge.h"
#include "horizon/topology/Queries.h"

using namespace hz::topo;

namespace {

// A ring of `n` half-edges whose `next` pointers make one cycle, the way a
// closed face's outer loop does. Each one's origin is its own vertex.
struct Ring {
    std::vector<HalfEdge> halfEdges;
    std::vector<Vertex> vertices;
    Wire wire{};

    explicit Ring(int n) : halfEdges(static_cast<size_t>(n)), vertices(static_cast<size_t>(n)) {
        for (int i = 0; i < n; ++i) {
            halfEdges[static_cast<size_t>(i)].origin = &vertices[static_cast<size_t>(i)];
            halfEdges[static_cast<size_t>(i)].next = &halfEdges[static_cast<size_t>((i + 1) % n)];
        }
        wire.halfEdge = &halfEdges.front();
    }
};

}  // namespace

// -- A sound ring answers as it always did ------------------------------------

TEST(QueriesTest, AClosedLoopCountsItsOwnHalfEdges) {
    const Ring ring(5);
    EXPECT_EQ(loopSize(&ring.wire), 5);
}

TEST(QueriesTest, AClosedLoopListsItsOwnVerticesInOrder) {
    Ring ring(4);
    Face f{};
    f.outerLoop = &ring.wire;

    const std::vector<Vertex*> v = faceVertices(&f);
    ASSERT_EQ(v.size(), 4u);
    for (int i = 0; i < 4; ++i) {
        EXPECT_EQ(v[static_cast<size_t>(i)], &ring.vertices[static_cast<size_t>(i)]) << i;
    }
}

// -- A ring with a break in it is answered, not walked off the end ------------

// The chain stops part way round. Reading it as a loop read through the null,
// which is a crash rather than a wrong answer, so the whole application went
// with it.
TEST(QueriesTest, ALoopBrokenPartWayRoundIsCountedUpToTheBreak) {
    Ring ring(6);
    ring.halfEdges[2].next = nullptr;  // the loop stops after three

    EXPECT_EQ(loopSize(&ring.wire), 3);
}

TEST(QueriesTest, ALoopBrokenPartWayRoundListsTheVerticesItHas) {
    Ring ring(6);
    ring.halfEdges[3].next = nullptr;

    Face f{};
    f.outerLoop = &ring.wire;

    const std::vector<Vertex*> v = faceVertices(&f);
    ASSERT_EQ(v.size(), 4u);
    EXPECT_EQ(v[0], &ring.vertices[0]);
    EXPECT_EQ(v[3], &ring.vertices[3]);
}

TEST(QueriesTest, ALoopThatNeverClosesIsCountedNotSpunOn) {
    // A chain that runs off into itself and never returns to its start: a
    // `while (cur != start)` walk of it never ends.
    HalfEdge chain[8];
    for (int i = 0; i < 7; ++i) {
        chain[i].next = &chain[i + 1];
    }
    chain[7].next = &chain[2];  // back into the middle, never to the start

    Wire w{};
    w.halfEdge = &chain[0];
    EXPECT_LE(loopSize(&w), 8) << "bounded by the ring it was given";
}

// The same chain, through each walk that reads it. They all have to answer:
// one of them answering by never answering at all is the worst of it, since
// the application stops answering too.
TEST(QueriesTest, ALoopThatNeverClosesIsListedNotSpunOn) {
    HalfEdge chain[8];
    for (int i = 0; i < 7; ++i) {
        chain[i].next = &chain[i + 1];
    }
    chain[7].next = &chain[2];

    Wire w{};
    w.halfEdge = &chain[0];
    Face f{};
    f.outerLoop = &w;

    EXPECT_LE(faceVertices(&f).size(), 8u) << "bounded by the ring it was given";
    EXPECT_LE(adjacentFaces(&f).size(), 8u) << "bounded by the ring it was given";
}

// A vertex whose half-edges lead round a ring that never closes.
TEST(QueriesTest, TheEdgesAtAVertexOfARingThatNeverClosesAreFound) {
    HalfEdge chain[8];
    for (int i = 0; i < 7; ++i) {
        chain[i].next = &chain[i + 1];
    }
    chain[7].next = &chain[2];

    Vertex v{};
    v.halfEdge = &chain[0];

    EXPECT_LE(incidentEdges(&v).size(), 8u) << "bounded by the ring it was given";
}

TEST(QueriesTest, ALoopOfOneHalfEdgeIsOneHalfEdge) {
    HalfEdge only{};
    only.next = &only;
    Wire w{};
    w.halfEdge = &only;
    EXPECT_EQ(loopSize(&w), 1);

    Face f{};
    f.outerLoop = &w;
    EXPECT_EQ(faceVertices(&f).size(), 1u);
}

// -- Nothing at all is nothing, not a crash -----------------------------------

TEST(QueriesTest, MissingPiecesAreAnsweredAsNothing) {
    EXPECT_EQ(loopSize(nullptr), 0);

    Wire empty{};
    EXPECT_EQ(loopSize(&empty), 0);

    EXPECT_TRUE(faceVertices(nullptr).empty());

    Face noLoop{};
    EXPECT_TRUE(faceVertices(&noLoop).empty());

    EXPECT_TRUE(adjacentFaces(nullptr).empty());
    EXPECT_TRUE(incidentEdges(nullptr).empty());
    EXPECT_EQ(leftFace(nullptr), nullptr);
    EXPECT_EQ(rightFace(nullptr), nullptr);
}

// -- The neighbours of a face whose loop is broken ----------------------------

TEST(QueriesTest, TheNeighboursOfALoopBrokenPartWayRoundAreFound) {
    Ring ring(6);
    Face neighbour{};
    Edge edge{};
    HalfEdge twin{};

    twin.face = &neighbour;
    edge.halfEdge = &ring.halfEdges[0];
    ring.halfEdges[0].edge = &edge;
    ring.halfEdges[0].twin = &twin;

    ring.halfEdges[3].next = nullptr;  // the rest of the loop is unreadable

    Face f{};
    f.outerLoop = &ring.wire;

    const std::vector<Face*> adjacent = adjacentFaces(&f);
    ASSERT_EQ(adjacent.size(), 1u);
    EXPECT_EQ(adjacent[0], &neighbour);
}

TEST(QueriesTest, TheEdgesAtAVertexOfABrokenRingAreFound) {
    Ring ring(6);
    Edge edge{};
    ring.halfEdges[0].edge = &edge;
    // A vertex whose half-edge leads off the ring: no twin to walk back from.
    HalfEdge stray{};
    stray.origin = &ring.vertices[0];
    ring.vertices[0].halfEdge = &stray;

    const std::vector<Edge*> edges = incidentEdges(&ring.vertices[0]);
    for (const Edge* e : edges) {
        EXPECT_NE(e, nullptr);
    }
}