#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <random>
#include <set>
#include <utility>
#include <vector>

#include "../PortableRandom.h"
#include "horizon/geometry/Triangulate2D.h"
#include "horizon/math/RobustPredicates.h"

using hz::geo::RegionTriangulation;
using hz::geo::triangulateRegion;
using hz::geo::Triangulation2D;
using hz::math::Containment;
using hz::math::Vec2;

namespace {

using Loop = std::vector<Vec2>;
using Edge = std::pair<std::uint32_t, std::uint32_t>;

Edge undirected(std::uint32_t a, std::uint32_t b) {
    return a < b ? Edge{a, b} : Edge{b, a};
}

/// A coordinate as the triangulation keeps it: a nonzero magnitude below 1e-60 is +0 (Ruling 4).
double flushed(double c) {
    return c != 0.0 && std::fabs(c) < 1e-60 ? 0.0 : c;
}

Vec2 flushed(const Vec2& p) {
    return {flushed(p.x), flushed(p.y)};
}

double loopArea(const Loop& loop) {
    double twice = 0.0;
    for (std::size_t i = 0; i < loop.size(); ++i) {
        const Vec2& a = loop[i];
        const Vec2& b = loop[(i + 1) % loop.size()];
        twice += a.x * b.y - b.x * a.y;
    }
    return std::abs(twice) / 2.0;
}

bool insideRegion(const Loop& outer, const std::vector<Loop>& holes, const Vec2& p) {
    if (hz::math::classifyPoint(outer, p) != Containment::Inside) {
        return false;
    }
    for (const Loop& hole : holes) {
        if (hz::math::classifyPoint(hole, p) != Containment::Outside) {
            return false;
        }
    }
    return true;
}

/// The five properties of the brief, plus the index contract, of @p t: the triangulation of the
/// region with @p interior the points offered after the boundary (to the constructor, or to
/// RegionTriangulation::insert).
void checkTriangulation(const Triangulation2D& t, const Loop& outer, const std::vector<Loop>& holes,
                        const Loop& interior, double merge) {
    EXPECT_TRUE(t.problem.empty()) << t.problem;

    // Index contract: points[i] is the i-th boundary point, as flushed.
    std::vector<Vec2> boundary;
    for (const Vec2& p : outer) {
        boundary.push_back(flushed(p));
    }
    for (const Loop& h : holes) {
        for (const Vec2& p : h) {
            boundary.push_back(flushed(p));
        }
    }
    EXPECT_GE(t.points.size(), boundary.size());
    for (std::size_t i = 0; i < boundary.size() && i < t.points.size(); ++i) {
        EXPECT_EQ(t.points[i].x, boundary[i].x);
        EXPECT_EQ(t.points[i].y, boundary[i].y);
    }

    // The boundary point each boundary index stands for: the first within merge of it.
    std::vector<std::uint32_t> rep(boundary.size());
    for (std::size_t i = 0; i < boundary.size(); ++i) {
        rep[i] = static_cast<std::uint32_t>(i);
        for (std::size_t j = 0; j < i; ++j) {
            if (rep[j] == j && (boundary[i] - boundary[j]).length() <= merge) {
                rep[i] = static_cast<std::uint32_t>(j);
                break;
            }
        }
    }

    // Positive area for every triangle; the total area; edge incidence.
    double total = 0.0;
    std::map<Edge, std::vector<std::pair<std::size_t, std::uint32_t>>> edges;
    std::set<std::uint32_t> used;
    for (std::size_t k = 0; k < t.triangles.size(); ++k) {
        const auto& tri = t.triangles[k];
        const Vec2& a = t.points[tri[0]];
        const Vec2& b = t.points[tri[1]];
        const Vec2& c = t.points[tri[2]];
        EXPECT_GT(hz::math::orient2d(a, b, c), 0) << "triangle " << k;
        total += 0.5 * ((b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y));
        for (int e = 0; e < 3; ++e) {
            edges[undirected(tri[e], tri[(e + 1) % 3])].emplace_back(k, tri[(e + 2) % 3]);
            used.insert(tri[e]);
        }
    }
    double expected = loopArea(outer);
    for (const Loop& h : holes) {
        expected -= loopArea(h);
    }
    EXPECT_NEAR(total, expected, 1e-12 * expected);

    // Each boundary segment is an edge: one triangle's when the loops run along it once, both
    // sides' when twice (a slit or bridge). Every other edge has a triangle on both sides.
    std::map<Edge, int> segments;
    std::size_t base = 0;
    auto addLoop = [&](const Loop& loop) {
        for (std::size_t i = 0; i < loop.size(); ++i) {
            const std::uint32_t a = rep[base + i];
            const std::uint32_t b = rep[base + (i + 1) % loop.size()];
            if (a != b) {
                ++segments[undirected(a, b)];
            }
        }
        base += loop.size();
    };
    addLoop(outer);
    for (const Loop& h : holes) {
        addLoop(h);
    }
    for (const auto& [s, uses] : segments) {
        const auto it = edges.find(s);
        if (it == edges.end()) {
            ADD_FAILURE() << "segment " << s.first << "-" << s.second << " missing";
            continue;
        }
        EXPECT_EQ(it->second.size(), uses % 2 == 1 ? 1U : 2U)
            << "segment " << s.first << "-" << s.second;
    }
    for (const auto& [edge, users] : edges) {
        if (segments.count(edge) == 0) {
            EXPECT_EQ(users.size(), 2U) << "edge " << edge.first << "-" << edge.second;
        }
    }

    // boundaryVertex is the earliest index of the merge group.
    EXPECT_EQ(t.boundaryVertex, rep);

    // Merged-away boundary points are referenced by no triangle.
    for (std::size_t i = 0; i < boundary.size(); ++i) {
        if (rep[i] != i) {
            EXPECT_EQ(used.count(static_cast<std::uint32_t>(i)), 0U);
        }
    }

    // Every kept interior point is used.
    for (std::size_t i = boundary.size(); i < t.points.size(); ++i) {
        EXPECT_EQ(used.count(static_cast<std::uint32_t>(i)), 1U) << "interior point " << i;
    }

    // Interior points inside the region are vertices.
    for (const Vec2& p : interior) {
        if (!insideRegion(outer, holes, p)) {
            continue;
        }
        bool found = false;
        for (std::size_t i = boundary.size(); i < t.points.size(); ++i) {
            if (t.points[i].x == flushed(p.x) && t.points[i].y == flushed(p.y) &&
                used.count(static_cast<std::uint32_t>(i)) == 1U) {
                found = true;
            }
        }
        // Within merge of another point is allowed to drop it.
        if (merge == 0.0) {
            EXPECT_TRUE(found) << "interior point " << p.x << "," << p.y;
        }
    }

    // Delaunay except across boundary segments: across every other shared edge, the far
    // vertex of one triangle is not strictly inside the other's circumcircle.
    for (const auto& [edge, users] : edges) {
        if (users.size() != 2 || segments.count(edge) != 0) {
            continue;
        }
        for (int s = 0; s < 2; ++s) {
            const auto& tri = t.triangles[users[s].first];
            const Vec2& far = t.points[users[1 - s].second];
            EXPECT_LE(hz::math::incircle(t.points[tri[0]], t.points[tri[1]], t.points[tri[2]], far),
                      0)
                << "edge " << edge.first << "-" << edge.second;
        }
    }
}

/// checkTriangulation of triangulateRegion's result. Returns the result.
Triangulation2D checkRegion(const Loop& outer, const std::vector<Loop>& holes,
                            const Loop& interior = {}, double merge = 0.0) {
    Triangulation2D t = triangulateRegion(outer, holes, interior, merge);
    checkTriangulation(t, outer, holes, interior, merge);
    return t;
}

}  // namespace

namespace {

double triangleArea(const Triangulation2D& t) {
    double total = 0.0;
    for (const auto& tri : t.triangles) {
        const Vec2& a = t.points[tri[0]];
        const Vec2& b = t.points[tri[1]];
        const Vec2& c = t.points[tri[2]];
        total += 0.5 * ((b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y));
    }
    return total;
}

}  // namespace

TEST(Triangulate2D, Square) {
    const Loop outer{{0, 0}, {4, 0}, {4, 4}, {0, 4}};
    const Triangulation2D t = checkRegion(outer, {}, {{1, 1}, {2, 3}, {3, 1.5}});
    EXPECT_EQ(t.points.size(), 7U);
}

TEST(Triangulate2D, LShape) {
    const Loop outer{{0, 0}, {4, 0}, {4, 2}, {2, 2}, {2, 4}, {0, 4}};
    // (3,3) is in the notch: not in the region, not a vertex.
    const Triangulation2D t = checkRegion(outer, {}, {{1, 1}, {1, 3}, {3, 1}, {3, 3}});
    EXPECT_EQ(t.points.size(), 9U);
}

TEST(Triangulate2D, SquareWithSquareHole) {
    const Loop outer{{0, 0}, {10, 0}, {10, 10}, {0, 10}};
    const Loop hole{{3, 3}, {3, 7}, {7, 7}, {7, 3}};
    checkRegion(outer, {hole}, {{1, 1}, {9, 9}, {5, 5}, {5, 1.5}});
}

TEST(Triangulate2D, HoleCloseToTheOuterEdge) {
    const Loop outer{{0, 0}, {10, 0}, {10, 10}, {0, 10}};
    const Loop hole{{1e-6, 3}, {1e-6, 7}, {5, 7}, {5, 3}};
    checkRegion(outer, {hole}, {{8, 5}, {5e-7, 5}, {2, 5}});
}

TEST(Triangulate2D, Sliver) {
    const Loop outer{{0, 0}, {100, 0}, {100, 0.01}, {0, 0.01}};
    checkRegion(outer, {}, {{50, 0.005}, {25, 0.001}, {75, 0.009}});
}

TEST(Triangulate2D, CollinearAndDuplicatedBoundaryPoints) {
    // (0,0), (2,0), (4,0) are collinear; (4,3) is given twice.
    const Loop outer{{0, 0}, {2, 0}, {4, 0}, {4, 3}, {4, 3}, {0, 3}};
    const Triangulation2D t = checkRegion(outer, {}, {{2, 1.5}});
    EXPECT_EQ(t.points.size(), 7U);
    for (const auto& tri : t.triangles) {
        for (const std::uint32_t i : tri) {
            EXPECT_NE(i, 4U);  // the duplicate
        }
    }
}

TEST(Triangulate2D, MergeDistanceMergesNearbyBoundaryPoints) {
    const Loop outer{{0, 0}, {4, 0}, {4, 4}, {4 + 1e-9, 4}, {0, 4}};
    const Triangulation2D t = checkRegion(outer, {}, {}, 1e-6);
    for (const auto& tri : t.triangles) {
        for (const std::uint32_t i : tri) {
            EXPECT_NE(i, 3U);
        }
    }
}

TEST(Triangulate2D, KeepsBoundaryIndicesAcrossLoops) {
    const Loop outer{{0, 0}, {10, 0}, {10, 10}, {0, 10}};
    const Loop hole{{3, 3}, {3, 7}, {7, 7}, {7, 3}};
    const Triangulation2D t = triangulateRegion(outer, {hole}, {{1, 1}, {9, 9}});
    EXPECT_TRUE(t.problem.empty());
    ASSERT_EQ(t.points.size(), 10U);
    EXPECT_EQ(t.points[4].x, 3.0);
    EXPECT_EQ(t.points[7].y, 3.0);
    EXPECT_EQ(t.points[8].x, 1.0);  // interior points follow, in input order
    EXPECT_EQ(t.points[9].x, 9.0);
}

TEST(Triangulate2D, InteriorPointOnABoundarySegmentIsDropped) {
    const Loop outer{{0, 0}, {4, 0}, {4, 4}, {0, 4}};
    const Loop hole{{1, 1}, {1, 3}, {3, 3}, {3, 1}};
    // On the outer edge, on a hole edge, on a corner, and inside the hole.
    const Triangulation2D t =
        triangulateRegion(outer, {hole}, {{2, 0}, {1, 2}, {4, 4}, {2, 2}, {0.5, 0.5}}, 0.0);
    EXPECT_TRUE(t.problem.empty());
    ASSERT_EQ(t.points.size(), 9U);
    EXPECT_EQ(t.points[8].x, 0.5);
    checkRegion(outer, {hole}, {{2, 0}, {1, 2}, {4, 4}, {2, 2}, {0.5, 0.5}});
}

TEST(Triangulate2D, InteriorPointWithinMergeOfABoundaryPointIsDropped) {
    const Loop outer{{0, 0}, {4, 0}, {4, 4}, {0, 4}};
    const Triangulation2D t = triangulateRegion(outer, {}, {{1e-4, 1e-4}, {2, 2}}, 1e-3);
    EXPECT_TRUE(t.problem.empty());
    ASSERT_EQ(t.points.size(), 5U);
    EXPECT_EQ(t.points[4].x, 2.0);
}

TEST(Triangulate2D, ClockwiseAndCounterClockwiseOuterLoopsGiveTheSameRegion) {
    const Loop ccw{{0, 0}, {4, 0}, {4, 2}, {2, 2}, {2, 4}, {0, 4}};
    const Loop cw(ccw.rbegin(), ccw.rend());
    const Triangulation2D a = checkRegion(ccw, {}, {{1, 1}});
    const Triangulation2D b = checkRegion(cw, {}, {{1, 1}});
    EXPECT_EQ(a.triangles.size(), b.triangles.size());
}

TEST(Triangulate2D, RandomInteriorPointsInACircleishPolygon) {
    Loop outer;
    for (int i = 0; i < 40; ++i) {
        const double angle = 2.0 * 3.14159265358979323846 * i / 40.0;
        outer.emplace_back(10.0 * std::cos(angle), 10.0 * std::sin(angle));
    }
    std::mt19937 rng(20261124);
    const hz::test::Uniform coord(-10.0, 10.0);
    Loop interior;
    for (int i = 0; i < 400; ++i) {
        const double x = coord(rng);
        const double y = coord(rng);
        interior.emplace_back(x, y);
    }
    const Triangulation2D t = checkRegion(outer, {}, interior);
    EXPECT_GT(t.points.size(), 200U);
}

TEST(Triangulate2D, RandomInteriorPointsAroundAHole) {
    const Loop outer{{-10, -10}, {10, -10}, {10, 10}, {-10, 10}};
    const Loop hole{{-3, -3}, {-3, 3}, {3, 3}, {3, -3}};
    std::mt19937 rng(7);
    const hz::test::Uniform coord(-10.0, 10.0);
    Loop interior;
    for (int i = 0; i < 300; ++i) {
        const double x = coord(rng);
        const double y = coord(rng);
        interior.emplace_back(x, y);
    }
    checkRegion(outer, {hole}, interior);
}

TEST(Triangulate2D, ReentrantStarWithHoleAndManyPoints) {
    std::mt19937 rng(99);
    const hz::test::Uniform radius(4.0, 10.0);
    const hz::test::Uniform coord(-10.0, 10.0);
    Loop outer;
    for (int i = 0; i < 60; ++i) {
        const double angle = 2.0 * 3.14159265358979323846 * i / 60.0;
        const double r = radius(rng);
        outer.emplace_back(r * std::cos(angle), r * std::sin(angle));
    }
    Loop hole;
    for (int i = 0; i < 12; ++i) {
        const double angle = -2.0 * 3.14159265358979323846 * i / 12.0;  // clockwise
        hole.emplace_back(2.0 * std::cos(angle), 2.0 * std::sin(angle));
    }
    Loop interior;
    for (int i = 0; i < 500; ++i) {
        const double x = coord(rng);
        const double y = coord(rng);
        interior.emplace_back(x, y);
    }
    checkRegion(outer, {hole}, interior);
}

TEST(Triangulate2D, HoleCrossingTheOuterLoopIsReported) {
    const Loop outer{{0, 0}, {10, 0}, {10, 10}, {0, 10}};
    const Loop hole{{5, 5}, {5, 15}, {8, 15}, {8, 5}};  // sticks out through the top edge
    const Triangulation2D t = triangulateRegion(outer, {hole});
    EXPECT_FALSE(t.problem.empty());
    // Segment 2 of the outer loop (10,10)-(0,10), and the hole's segment 0 or 2, cross.
    EXPECT_NE(t.problem.find("outer segment 2"), std::string::npos) << t.problem;
    EXPECT_NE(t.problem.find("hole 0 segment"), std::string::npos) << t.problem;
}

TEST(Triangulate2D, FigureEightOuterLoopIsReported) {
    const Loop outer{{0, 0}, {4, 4}, {4, 0}, {0, 4}};  // segments 0 and 2 cross at (2,2)
    const Triangulation2D t = triangulateRegion(outer, {});
    EXPECT_FALSE(t.problem.empty());
    EXPECT_NE(t.problem.find("outer segment 0"), std::string::npos) << t.problem;
    EXPECT_NE(t.problem.find("outer segment 2"), std::string::npos) << t.problem;
}

TEST(Triangulate2D, HoleTouchingTheOuterLoopAtAVertexIsReported) {
    const Loop outer{{0, 0}, {10, 0}, {10, 10}, {0, 10}};
    const Loop hole{{10, 10}, {7, 8}, {8, 6}};
    const Triangulation2D t = triangulateRegion(outer, {hole});
    EXPECT_FALSE(t.problem.empty());
    EXPECT_NE(t.problem.find("hole 0 point 0"), std::string::npos) << t.problem;
    EXPECT_NE(t.problem.find("outer point 2"), std::string::npos) << t.problem;
}

TEST(Triangulate2D, HoleVertexOnAnOuterSegmentIsReported) {
    const Loop outer{{0, 0}, {10, 0}, {10, 10}, {0, 10}};
    const Loop hole{{5, 0}, {7, 3}, {3, 3}};  // (5,0) lies on the outer's segment 0
    const Triangulation2D t = triangulateRegion(outer, {hole});
    EXPECT_FALSE(t.problem.empty());
    EXPECT_NE(t.problem.find("outer segment 0"), std::string::npos) << t.problem;
}

TEST(Triangulate2D, SlitInTheOuterLoopIsPartOfTheRegion) {
    // Dense points left of the slit, sparse right: a fill that reaches one side first
    // by a long way labels the other side from it.
    const Loop outer{{0, 0}, {10, 0}, {10, 10}, {5, 10}, {5, 0.5}, {5, 10}, {0, 10}};
    for (std::uint32_t seed = 0; seed < 10; ++seed) {
        std::mt19937 rng(seed);
        const hz::test::Uniform left(0.01, 4.99);
        const hz::test::Uniform right(5.01, 9.99);
        const hz::test::Uniform up(0.01, 9.99);
        Loop interior;
        for (int i = 0; i < 400; ++i) {
            const double x = left(rng);
            interior.emplace_back(x, up(rng));
        }
        for (int i = 0; i < 5; ++i) {
            const double x = right(rng);
            interior.emplace_back(x, up(rng));
        }
        const Triangulation2D t = checkRegion(outer, {}, interior);
        EXPECT_NEAR(triangleArea(t), 100.0, 1e-9) << "seed " << seed;
    }
}

TEST(Triangulate2D, BridgeToAnInnerLoopIsPartOfTheRegion) {
    const Loop outer{{0, 0}, {10, 0}, {10, 10}, {5, 10}, {5, 6},  {4, 6},
                     {4, 4}, {6, 4},  {6, 6},   {5, 6},  {5, 10}, {0, 10}};
    for (std::uint32_t seed = 0; seed < 10; ++seed) {
        std::mt19937 rng(seed);
        const hz::test::Uniform u(0.01, 9.99);
        Loop interior;
        for (int i = 0; i < 400; ++i) {
            const double x = u(rng);
            const double y = u(rng);
            if (x < 5 || i % 50 == 0) {
                interior.emplace_back(x, y);
            }
        }
        const Triangulation2D t = triangulateRegion(outer, {}, interior);
        EXPECT_TRUE(t.problem.empty()) << t.problem;
        EXPECT_NEAR(triangleArea(t), 96.0, 1e-9) << "seed " << seed;
    }
}

TEST(Triangulate2D, HoleOutsideTheOuterLoopIsReported) {
    const Loop outer{{0, 0}, {4, 0}, {4, 4}, {0, 4}};
    const Loop hole{{10, 10}, {10, 12}, {12, 12}, {12, 10}};
    const Triangulation2D t = triangulateRegion(outer, {hole});
    EXPECT_NE(t.problem.find("hole 0 is not inside the outer loop"), std::string::npos)
        << t.problem;
}

TEST(Triangulate2D, HoleEnclosingTheOuterLoopIsReported) {
    const Loop outer{{0, 0}, {4, 0}, {4, 4}, {0, 4}};
    const Loop hole{{-1, -1}, {-1, 5}, {5, 5}, {5, -1}};
    const Triangulation2D t = triangulateRegion(outer, {hole});
    EXPECT_NE(t.problem.find("hole 0 is not inside the outer loop"), std::string::npos)
        << t.problem;
}

TEST(Triangulate2D, HoleInsideAnotherHoleIsReported) {
    const Loop outer{{0, 0}, {10, 0}, {10, 10}, {0, 10}};
    const Loop big{{2, 2}, {2, 8}, {8, 8}, {8, 2}};
    const Loop small{{4, 4}, {4, 5}, {5, 5}, {5, 4}};
    const Triangulation2D t = triangulateRegion(outer, {big, small});
    EXPECT_NE(t.problem.find("hole 1 is inside hole 0"), std::string::npos) << t.problem;
}

TEST(Triangulate2D, CocircularLatticeAtManyScales) {
    for (const double scale : {1e-6, 1.0, 1e6}) {
        Loop outer;
        for (const auto& [x, y] : {std::pair{0, 0}, {8, 0}, {8, 8}, {0, 8}}) {
            outer.emplace_back(x * scale, y * scale);
        }
        Loop interior;
        for (int i = 0; i <= 8; ++i) {
            for (int j = 0; j <= 8; ++j) {
                interior.emplace_back(i * scale, j * scale);
            }
        }
        const Triangulation2D t = checkRegion(outer, {}, interior);
        EXPECT_EQ(t.points.size(), 4U + 49U) << "scale " << scale;
    }
}

TEST(Triangulate2D, BoundariesAtSmallAndLargeScales) {
    for (const double scale : {1e-6, 1e6}) {
        const Loop outer{{0, 0}, {10 * scale, 0}, {10 * scale, 10 * scale}, {0, 10 * scale}};
        const Loop hole{{3 * scale, 3 * scale},
                        {3 * scale, 7 * scale},
                        {7 * scale, 7 * scale},
                        {7 * scale, 3 * scale}};
        checkRegion(outer, {hole},
                    {{scale, scale}, {9 * scale, 5 * scale}, {5 * scale, 9 * scale}});
    }
}

TEST(Triangulate2D, BoundaryVertexIsTheEarliestIndexOfAMergeGroup) {
    const Loop outer{{0, 0}, {4, 0}, {4, 4}, {4 + 1e-9, 4}, {4, 4 + 1e-9}, {0, 4}};
    const Triangulation2D t = triangulateRegion(outer, {}, {}, 1e-6);
    EXPECT_TRUE(t.problem.empty()) << t.problem;
    EXPECT_EQ(t.boundaryVertex, (std::vector<std::uint32_t>{0, 1, 2, 2, 2, 5}));
    // Exact duplicates merge with merge = 0, and -0.0 equals 0.0.
    const Loop dup{{0, 0}, {4, 0}, {4, 3}, {4, 3}, {-0.0, 3}, {0, 3}};
    const Triangulation2D u = triangulateRegion(dup, {});
    EXPECT_EQ(u.boundaryVertex, (std::vector<std::uint32_t>{0, 1, 2, 2, 4, 4}));
}

TEST(Triangulate2D, InteriorPointsAreClassifiedAgainstTheMergedLoops) {
    // (1e-3,-1e-4) merges into (0,0); the segment is then (0,0)-(10,0), so (1,0) is on it.
    const Loop outer{{0, 0}, {1e-3, -1e-4}, {10, 0}, {10, 10}, {0, 10}};
    const Triangulation2D t = triangulateRegion(outer, {}, {{1, 0}}, 2e-3);
    EXPECT_TRUE(t.problem.empty()) << t.problem;
    EXPECT_EQ(t.points.size(), 5U);
}

TEST(Triangulate2D, DegenerateOuterLoopIsReported) {
    EXPECT_FALSE(triangulateRegion({{0, 0}, {0, 0}, {1, 0}}, {}).problem.empty());
    EXPECT_FALSE(triangulateRegion({}, {}).problem.empty());
}

TEST(Triangulate2D, AnOutwardSpikeIsReported) {
    // The spike (4,2)-(6,2) is run along both ways, so it divides nothing, and it lies outside
    // the square: no triangle has it as an edge.
    const Loop outer{{0, 0}, {4, 0}, {4, 2}, {6, 2}, {4, 2}, {4, 4}, {0, 4}};
    const Triangulation2D t = triangulateRegion(outer, {});
    EXPECT_FALSE(t.problem.empty());
}

TEST(Triangulate2D, AZeroAreaOuterLoopIsReported) {
    const Triangulation2D t = triangulateRegion({{0, 0}, {1, 0}, {2, 0}, {1, 0}}, {});
    EXPECT_FALSE(t.problem.empty());
}

TEST(Triangulate2D, ALoopGivenTwiceInTheSameDirectionIsReported) {
    // Every segment is run along twice, so none is a wall and nothing is inside.
    const Loop twice{{0, 0}, {4, 0}, {4, 4}, {0, 4}, {0, 0}, {4, 0}, {4, 4}, {0, 4}};
    const Triangulation2D t = triangulateRegion(twice, {});
    EXPECT_FALSE(t.problem.empty());
}

TEST(Triangulate2D, APointThatIsNotFiniteIsReported) {
    const double nan = std::nan("");
    const double inf = std::numeric_limits<double>::infinity();
    const Loop square{{0, 0}, {4, 0}, {4, 4}, {0, 4}};
    const Loop hole{{1, 1}, {1, 2}, {2, 2}, {2, 1}};
    const std::vector<Triangulation2D> results = {
        triangulateRegion({{0, 0}, {4, 0}, {nan, 4}, {0, 4}}, {}),
        triangulateRegion(square, {{{1, 1}, {1, inf}, {2, 2}, {2, 1}}}),
        triangulateRegion(square, {hole}, {{3, 3}, {-inf, 3}}),
        triangulateRegion({{0, 0}, {4, 0}, {4, 4}, {0, nan}}, {}, {{1, 1}})};
    for (const Triangulation2D& t : results) {
        EXPECT_NE(t.problem.find("not finite"), std::string::npos) << t.problem;
        EXPECT_TRUE(t.triangles.empty());
    }
}

TEST(Triangulate2D, PointsSpreadBeyondWhatThePredicatesHoldAreReported) {
    // From -1e308 to 1e308: the extent itself overflows, and so would the predicates' products.
    const Triangulation2D t =
        triangulateRegion({{-1e308, -1e308}, {1e308, -1e308}, {1e308, 1e308}, {-1e308, 1e308}}, {});
    EXPECT_NE(t.problem.find("too far"), std::string::npos) << t.problem;
    EXPECT_TRUE(t.triangles.empty());
    // Far out but close together, and wide but within the predicates' reach, are fine.
    const double far = 1e30;
    const Triangulation2D u = triangulateRegion(
        {{far, far}, {far + 4e20, far}, {far + 4e20, far + 4e20}, {far, far + 4e20}}, {});
    EXPECT_TRUE(u.problem.empty()) << u.problem;
    EXPECT_EQ(u.triangles.size(), 2u);
    checkRegion({{-1e60, -1e60}, {1e60, -1e60}, {1e60, 1e60}, {-1e60, 1e60}}, {});
}

TEST(Triangulate2D, TwentyThousandPointsOnTheUnitSquare) {
    const Loop outer{{0, 0}, {1, 0}, {1, 1}, {0, 1}};
    std::mt19937 rng(5);
    const hz::test::Uniform u(0.0, 1.0);
    Loop interior;
    for (int i = 0; i < 20000; ++i) {
        const double x = u(rng);
        interior.emplace_back(x, u(rng));
    }
    const Triangulation2D t = triangulateRegion(outer, {}, interior);
    EXPECT_TRUE(t.problem.empty()) << t.problem;
    EXPECT_GT(t.points.size(), 19000U);
    EXPECT_NEAR(triangleArea(t), 1.0, 1e-9);
}

TEST(Triangulate2D, PointsSpreadTooLittleAreSaidToBe) {
    const auto t = triangulateRegion({{0, 0}, {1e-62, 0}, {1e-62, 1e-62}, {0, 1e-62}}, {});
    EXPECT_NE(t.problem.find("spread too little"), std::string::npos) << t.problem;
}

TEST(Triangulate2D, TinyCoordinatesAreFlushedToZero) {
    // The exactly cocircular rectangle whose incircle sign came out wrong (stage 2, Task 2's
    // review): its differences are near 1, but its coordinates of 9e-90 and 7e-90 lie below
    // incircle's floor, where the exact expansion's smallest terms underflow. Flushed to 0,
    // they are exact and every sign is right.
    const double x = 1.528379950790292;
    const double y = 3.5558643899770326;
    const double tinyX = 9.0470469239970971e-90;
    const double tinyY = 7.0334986959029244e-90;
    const Loop outer{{x, y}, {tinyX, y}, {tinyX, tinyY}, {x, tinyY}};
    const Triangulation2D t =
        checkRegion(outer, {}, {{0.4, 0.9}, {1.1, 2.6}, {0.3, 3.1}, {1.4, 0.2}, {0.76, 1.78}});
    ASSERT_EQ(t.points.size(), 9U);
    EXPECT_EQ(t.points[0].x, x);
    EXPECT_EQ(t.points[1].x, 0.0);
    EXPECT_EQ(t.points[2].x, 0.0);
    EXPECT_EQ(t.points[2].y, 0.0);
    EXPECT_EQ(t.points[3].y, 0.0);
}

TEST(Triangulate2D, OnlyCoordinatesBelowTheFloorAreFlushed) {
    // Either sign, either coordinate, boundary and interior points alike; 1e-60 itself is kept,
    // and so is a zero's sign.
    const double below = std::nextafter(1e-60, 0.0);
    const Loop outer{{-4, -4}, {-below, -4}, {4, -4}, {4, 1e-60}, {4, 4}, {-4, 4}};
    const Loop interior{{2e-61, 2}, {-2, -below}, {1e-60, -1e-60}, {2, 2}, {-0.0, 1}};
    const Triangulation2D t = checkRegion(outer, {}, interior);
    ASSERT_EQ(t.points.size(), 11U);
    EXPECT_EQ(t.points[1].x, 0.0);
    EXPECT_FALSE(std::signbit(t.points[1].x));  // flushed to +0
    EXPECT_TRUE(std::signbit(t.points[10].x));  // -0.0, as given
    EXPECT_EQ(t.points[3].y, 1e-60);
    EXPECT_EQ(t.points[6].x, 0.0);
    EXPECT_EQ(t.points[6].y, 2.0);
    EXPECT_EQ(t.points[7].y, 0.0);
    EXPECT_EQ(t.points[8].x, 1e-60);
    EXPECT_EQ(t.points[8].y, -1e-60);
    // A point given later is flushed alike.
    RegionTriangulation r(outer, {}, interior);
    ASSERT_TRUE(r.insert({-3e-61, -3}));
    EXPECT_EQ(r.result().points.back().x, 0.0);
    EXPECT_EQ(r.result().points.back().y, -3.0);
}

TEST(RegionTriangulation, InsertingKeepsTheContract) {
    // 500 points offered one at a time to a re-entrant star round a clockwise hole, a square with
    // a slit, and a square with a bridge to an inner loop: each is taken exactly when it lies
    // strictly inside the region (at random, none falls on a segment or on another point), and
    // the contract holds after every 50.
    struct Shape {
        Loop outer;
        std::vector<Loop> holes;
    };
    std::vector<Shape> shapes;
    {
        std::mt19937 rng(99);
        const hz::test::Uniform radius(4.0, 10.0);
        Shape star;
        for (int i = 0; i < 60; ++i) {
            const double angle = 2.0 * 3.14159265358979323846 * i / 60.0;
            const double r = radius(rng);
            star.outer.emplace_back(r * std::cos(angle), r * std::sin(angle));
        }
        star.holes.emplace_back();
        for (int i = 0; i < 12; ++i) {
            const double angle = -2.0 * 3.14159265358979323846 * i / 12.0;
            star.holes.back().emplace_back(2.0 * std::cos(angle), 2.0 * std::sin(angle));
        }
        shapes.push_back(star);
    }
    shapes.push_back({{{0, 0}, {10, 0}, {10, 10}, {5, 10}, {5, 0.5}, {5, 10}, {0, 10}}, {}});
    // The bridge's inner loop runs clockwise, so that the winding number insideRegion takes and
    // the triangulation's walls agree on it.
    shapes.push_back({{{0, 0},
                       {10, 0},
                       {10, 10},
                       {5, 10},
                       {5, 6},
                       {6, 6},
                       {6, 4},
                       {4, 4},
                       {4, 6},
                       {5, 6},
                       {5, 10},
                       {0, 10}},
                      {}});
    for (std::size_t s = 0; s < shapes.size(); ++s) {
        const Shape& shape = shapes[s];
        Vec2 lo = shape.outer.front();
        Vec2 hi = lo;
        for (const Vec2& p : shape.outer) {
            lo = {std::min(lo.x, p.x), std::min(lo.y, p.y)};
            hi = {std::max(hi.x, p.x), std::max(hi.y, p.y)};
        }
        std::mt19937 rng(static_cast<std::uint32_t>(20261124 + s));
        const hz::test::Uniform ux(lo.x, hi.x);
        const hz::test::Uniform uy(lo.y, hi.y);
        Loop offered;
        for (int i = 0; i < 20; ++i) {
            const double x = ux(rng);
            offered.emplace_back(x, uy(rng));
        }
        RegionTriangulation r(shape.outer, shape.holes, offered);
        checkTriangulation(r.result(), shape.outer, shape.holes, offered, 0.0);
        for (int i = 1; i <= 500; ++i) {
            const double x = ux(rng);
            const Vec2 p{x, uy(rng)};
            const std::size_t before = r.result().points.size();
            const bool inside = insideRegion(shape.outer, shape.holes, p);
            EXPECT_EQ(r.insert(p), inside) << "shape " << s << " point " << p.x << "," << p.y;
            EXPECT_EQ(r.result().points.size(), before + (inside ? 1U : 0U));
            offered.push_back(p);
            if (i % 50 == 0) {
                checkTriangulation(r.result(), shape.outer, shape.holes, offered, 0.0);
            }
        }
    }
}

TEST(RegionTriangulation, RefusesPointsOutsideOnASegmentOrNearAPoint) {
    const Loop outer{{0, 0}, {4, 0}, {4, 4}, {0, 4}};
    const Loop hole{{1, 1}, {1, 3}, {3, 3}, {3, 1}};
    RegionTriangulation r(outer, {hole}, {{0.5, 3.5}}, 0.1);
    const Triangulation2D before = r.result();
    const double inf = std::numeric_limits<double>::infinity();
    // Outside, far outside, in the hole, on the outer loop, on the hole, on a vertex, within
    // merge of a vertex and of the interior point, and not finite.
    const std::vector<Vec2> refused = {{5, 5},   {-1e300, 2}, {2, 2},       {2, 0},
                                       {1, 2},   {4, 4},      {3.95, 3.95}, {0.55, 3.45},
                                       {0.5, 0}, {inf, 1},    {1, -inf},    {std::nan(""), 1}};
    for (const Vec2& p : refused) {
        EXPECT_FALSE(r.insert(p)) << p.x << "," << p.y;
    }
    EXPECT_EQ(r.result().points.size(), before.points.size());
    EXPECT_EQ(r.result().triangles, before.triangles);
    EXPECT_TRUE(r.insert({0.5, 0.5}));
    EXPECT_FALSE(r.insert({0.55, 0.5}));  // within merge of the point just taken
    checkTriangulation(r.result(), outer, {hole}, {{0.5, 3.5}, {0.5, 0.5}}, 0.1);
    // A region that could not be triangulated takes nothing.
    RegionTriangulation crossed({{0, 0}, {4, 4}, {4, 0}, {0, 4}}, {});
    EXPECT_FALSE(crossed.result().problem.empty());
    EXPECT_FALSE(crossed.insert({3.5, 2}));
}

TEST(RegionTriangulation, PointsOnEdgesAndCocircularPointsKeepTheContract) {
    // A point on the diagonal of a square splits both its triangles.
    const Loop square{{0, 0}, {2, 0}, {2, 2}, {0, 2}};
    RegionTriangulation one(square, {});
    ASSERT_EQ(one.result().triangles.size(), 2U);
    ASSERT_TRUE(one.insert({1, 1}));
    ASSERT_EQ(one.result().triangles.size(), 4U);
    for (const auto& tri : one.result().triangles) {
        EXPECT_TRUE(tri[0] == 4U || tri[1] == 4U || tri[2] == 4U);
    }
    checkTriangulation(one.result(), square, {}, {{1, 1}}, 0.0);
    // An 8 x 8 square: its lattice points, then the midpoints of the lattice's sides, many on an
    // edge of the triangulation so far and all cocircular with others.
    const Loop outer{{0, 0}, {8, 0}, {8, 8}, {0, 8}};
    RegionTriangulation r(outer, {});
    Loop offered;
    for (int i = 1; i < 8; ++i) {
        for (int j = 1; j < 8; ++j) {
            offered.emplace_back(i, j);
        }
    }
    for (int i = 1; i < 8; ++i) {
        for (int j = 0; j < 8; ++j) {
            offered.emplace_back(i, j + 0.5);
            offered.emplace_back(j + 0.5, i);
        }
    }
    for (const Vec2& p : offered) {
        EXPECT_TRUE(r.insert(p)) << p.x << "," << p.y;
    }
    EXPECT_EQ(r.result().points.size(), 4U + offered.size());
    checkTriangulation(r.result(), outer, {}, offered, 0.0);
}

TEST(RegionTriangulation, PointsBesideASlitsTipKeepItAnEdge) {
    // Points go in on both sides of a slit's free end, where triangles whose circumcircles hold
    // the point reach round the tip to the slit's other side: the slit stays an edge.
    const Loop outer{{0, 0}, {10, 0}, {10, 10}, {5, 10}, {5, 0.5}, {5, 10}, {0, 10}};
    Loop offered{{1, 1}, {2, 8}, {8, 9}};
    RegionTriangulation r(outer, {}, offered);
    for (int k = 0; k < 40; ++k) {
        const double angle = 2.0 * 3.14159265358979323846 * k / 13.0 + 0.1;
        const double radius = 0.05 + 0.02 * k;
        const Vec2 p{5.0 + radius * std::cos(angle), 0.5 + radius * std::sin(angle)};
        EXPECT_EQ(r.insert(p), insideRegion(outer, {}, p)) << p.x << "," << p.y;
        offered.push_back(p);
        checkTriangulation(r.result(), outer, {}, offered, 0.0);
    }
}
