#include <gtest/gtest.h>

#include <cmath>
#include <map>
#include <random>
#include <set>
#include <utility>

#include "../PortableRandom.h"
#include "horizon/geometry/Triangulate2D.h"
#include "horizon/math/RobustPredicates.h"

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

/// The five properties of the brief, plus the index contract. Returns the result.
Triangulation2D checkRegion(const Loop& outer, const std::vector<Loop>& holes,
                            const Loop& interior = {}, double merge = 0.0) {
    const Triangulation2D t = triangulateRegion(outer, holes, interior, merge);

    // Index contract: points[i] is the i-th boundary point.
    std::vector<Vec2> boundary = outer;
    for (const Loop& h : holes) {
        boundary.insert(boundary.end(), h.begin(), h.end());
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

    // Each boundary segment is exactly one triangle's edge.
    std::set<Edge> segments;
    std::size_t base = 0;
    auto addLoop = [&](const Loop& loop) {
        for (std::size_t i = 0; i < loop.size(); ++i) {
            const std::uint32_t a = rep[base + i];
            const std::uint32_t b = rep[base + (i + 1) % loop.size()];
            if (a != b) {
                segments.insert(undirected(a, b));
            }
        }
        base += loop.size();
    };
    addLoop(outer);
    for (const Loop& h : holes) {
        addLoop(h);
    }
    for (const Edge& s : segments) {
        const auto it = edges.find(s);
        if (it == edges.end()) {
            ADD_FAILURE() << "segment " << s.first << "-" << s.second << " missing";
            continue;
        }
        EXPECT_EQ(it->second.size(), 1U) << "segment " << s.first << "-" << s.second;
    }

    // Merged-away boundary points are referenced by no triangle.
    for (std::size_t i = 0; i < boundary.size(); ++i) {
        if (rep[i] != i) {
            EXPECT_EQ(used.count(static_cast<std::uint32_t>(i)), 0U);
        }
    }

    // Interior points inside the region are vertices.
    for (const Vec2& p : interior) {
        if (!insideRegion(outer, holes, p)) {
            continue;
        }
        bool found = false;
        for (std::size_t i = boundary.size(); i < t.points.size(); ++i) {
            if (t.points[i].x == p.x && t.points[i].y == p.y &&
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
    return t;
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
    ASSERT_EQ(t.points.size(), 9U);
    EXPECT_EQ(t.points[8].x, 0.5);
    checkRegion(outer, {hole}, {{2, 0}, {1, 2}, {4, 4}, {2, 2}, {0.5, 0.5}});
}

TEST(Triangulate2D, InteriorPointWithinMergeOfABoundaryPointIsDropped) {
    const Loop outer{{0, 0}, {4, 0}, {4, 4}, {0, 4}};
    const Triangulation2D t = triangulateRegion(outer, {}, {{1e-4, 1e-4}, {2, 2}}, 1e-3);
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
