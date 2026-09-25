#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>

#include "../TimeLimits.h"
#include "horizon/math/Vec3.h"
#include "horizon/modeling/DrawingProjection.h"
#include "horizon/modeling/FilletOp.h"
#include "horizon/modeling/PrimitiveFactory.h"
#include "horizon/topology/Solid.h"

using hz::math::Vec2;
using hz::math::Vec3;
using hz::model::DrawingProjection;
using hz::model::PrimitiveFactory;
using hz::model::ProjectedEdge;
using hz::model::StandardView;
using hz::model::ViewProjection;

namespace {

int countVisible(const std::vector<ProjectedEdge>& edges) {
    int n = 0;
    for (const auto& e : edges) {
        if (e.visibility == ProjectedEdge::Visibility::Visible) ++n;
    }
    return n;
}

double segLength(const ProjectedEdge& e) {
    const double dx = e.a.x - e.b.x;
    const double dy = e.a.y - e.b.y;
    return std::sqrt(dx * dx + dy * dy);
}

}  // namespace

// ---------------------------------------------------------------------------
// The classic hidden-line result: an isometric cube shows 9 visible edges and
// 3 hidden edges (the three meeting at the occluded far corner).
// ---------------------------------------------------------------------------

TEST(DrawingProjectionTest, IsometricCubeNineVisibleThreeHidden) {
    auto box = PrimitiveFactory::makeBox(2.0, 2.0, 2.0);
    ASSERT_EQ(box->edgeCount(), 12u);

    auto edges =
        DrawingProjection::project(*box, DrawingProjection::standardView(StandardView::Isometric));
    ASSERT_EQ(edges.size(), 12u);  // straight edges → one segment each

    const int visible = countVisible(edges);
    const int hidden = static_cast<int>(edges.size()) - visible;
    EXPECT_EQ(visible, 9);
    EXPECT_EQ(hidden, 3);
}

// ---------------------------------------------------------------------------
// Orthographic projection: edges parallel to the view direction collapse to
// (near) points, while edges across the near face keep their length.
// ---------------------------------------------------------------------------

TEST(DrawingProjectionTest, FrontViewDropsViewParallelEdges) {
    auto box = PrimitiveFactory::makeBox(2.0, 2.0, 2.0);
    // Front view looks along -Y: the four Y-parallel "depth" edges project to
    // points and are dropped, leaving the eight edges in the XZ view plane —
    // four on the near face (visible) and four on the far face (hidden).
    auto edges =
        DrawingProjection::project(*box, DrawingProjection::standardView(StandardView::Front));
    ASSERT_EQ(edges.size(), 8u);
    for (const auto& e : edges) {
        EXPECT_NEAR(segLength(e), 2.0, 1e-6);  // no collapsed segments remain
    }
    EXPECT_EQ(countVisible(edges), 4);  // near face visible, far face hidden
}

// ---------------------------------------------------------------------------
// Model association: every projected edge carries the TopologyID of the model
// edge it came from, so downstream dimensioning can reference the 3D geometry.
// ---------------------------------------------------------------------------

TEST(DrawingProjectionTest, ProjectedEdgesCarrySourceTopologyId) {
    auto box = PrimitiveFactory::makeBox(4.0, 3.0, 2.0);
    auto edges =
        DrawingProjection::project(*box, DrawingProjection::standardView(StandardView::Isometric));
    ASSERT_FALSE(edges.empty());
    for (const auto& e : edges) {
        EXPECT_TRUE(e.sourceEdge.isValid());
    }
}

// ---------------------------------------------------------------------------
// A cylinder has curved edges (its two circular rims), which sample into
// several visible/hidden segments rather than collapsing away.
// ---------------------------------------------------------------------------

TEST(DrawingProjectionTest, CylinderRimsSplitIntoVisibleAndHidden) {
    auto cyl = PrimitiveFactory::makeCylinder(3.0, 6.0);
    auto edges =
        DrawingProjection::project(*cyl, DrawingProjection::standardView(StandardView::Isometric));
    ASSERT_GT(edges.size(), 0u);
    const int visible = countVisible(edges);
    const int hidden = static_cast<int>(edges.size()) - visible;
    // A cylinder rim's near arc is visible and its far arc is occluded by the
    // body, so partial-visibility splitting yields both visible and hidden runs.
    EXPECT_GT(visible, 0);
    EXPECT_GT(hidden, 0);
}

// ---------------------------------------------------------------------------
// Standard views are distinct orthographic directions.
// ---------------------------------------------------------------------------

TEST(DrawingProjectionTest, StandardViewsHaveDistinctDirections) {
    const auto front = DrawingProjection::standardView(StandardView::Front);
    const auto top = DrawingProjection::standardView(StandardView::Top);
    const auto right = DrawingProjection::standardView(StandardView::Right);
    EXPECT_GT((front.dir - top.dir).length(), 1e-6);
    EXPECT_GT((front.dir - right.dir).length(), 1e-6);
    EXPECT_GT((top.dir - right.dir).length(), 1e-6);
}

// ---------------------------------------------------------------------------
// Clean projection (Phase 147): each edge by what it is, curves drawn as
// curves, and a Front view seen from the front.
// ---------------------------------------------------------------------------

namespace {

bool vertical(const ProjectedEdge& e) {
    return std::abs(e.a.x - e.b.x) < 1e-9 && segLength(e) > 1e-9;
}

}  // namespace

// A cylinder seen side-on is its two outlines and its two rims: the seams
// between its side's facets are not drawn, except where the side turns away,
// where they are its silhouette. Every seam was drawn: 32 lines down it.
TEST(DrawingProjectionTest, ACylinderSeenSideOnIsItsOutlinesAndRims) {
    constexpr double r = 3.0;
    auto cyl = PrimitiveFactory::makeCylinder(r, 6.0, 32);
    auto edges =
        DrawingProjection::project(*cyl, DrawingProjection::standardView(StandardView::Front));
    int outlines = 0;
    for (const auto& e : edges) {
        if (!vertical(e)) continue;
        EXPECT_EQ(e.kind, ProjectedEdge::Kind::Silhouette) << "a seam drawn at x = " << e.a.x;
        EXPECT_EQ(e.visibility, ProjectedEdge::Visibility::Visible);
        // Where the facets turn away: on the circle, within a facet's sag.
        EXPECT_NEAR(std::abs(e.a.x), r, r * (1.0 - std::cos(3.14159265358979 / 32)) + 1e-9);
        ++outlines;
    }
    EXPECT_EQ(outlines, 2);
    for (const auto& e : edges) {
        if (!vertical(e)) {
            EXPECT_NEAR(e.a.y, e.b.y, 1e-9) << "a rim, seen edge-on, is level";
        }
    }
}

// Seen from above, a rim is drawn on its circle, as the arc each chord
// records: it was a 32-gon.
TEST(DrawingProjectionTest, ARimSeenFromAboveIsDrawnOnItsCircle) {
    constexpr double r = 3.0;
    auto cyl = PrimitiveFactory::makeCylinder(r, 6.0, 32);
    auto edges =
        DrawingProjection::project(*cyl, DrawingProjection::standardView(StandardView::Top));
    ASSERT_FALSE(edges.empty());
    double worst = 0.0;
    for (const auto& e : edges) {
        EXPECT_NEAR(std::hypot(e.a.x, e.a.y), r, 1e-9);
        EXPECT_NEAR(std::hypot(e.b.x, e.b.y), r, 1e-9);
        const double mid = std::hypot((e.a.x + e.b.x) / 2, (e.a.y + e.b.y) / 2);
        worst = std::max(worst, r - mid);
    }
    EXPECT_LT(worst, 1e-3) << "drawn along the arc, not across it by the chord";
}

// A fillet meets the faces it joins smoothly: those edges are Tangent, for
// the drawing to show or leave out; the seams inside the fillet are not
// edges at all.
TEST(DrawingProjectionTest, AFilletsEdgesWithItsFacesAreTangent) {
    auto box = PrimitiveFactory::makeBox(10, 10, 10);
    ASSERT_FALSE(box->edges().empty());
    auto rounded = hz::model::FilletOp::execute(*box, {box->edges().front().topoId}, 2.0, "f");
    ASSERT_NE(rounded.solid, nullptr) << rounded.errorMessage;
    auto edges = DrawingProjection::project(
        *rounded.solid, DrawingProjection::standardView(StandardView::Isometric));
    int tangent = 0;
    for (const auto& e : edges) tangent += e.kind == ProjectedEdge::Kind::Tangent ? 1 : 0;
    EXPECT_GT(tangent, 0) << "the fillet's two edges with the box";
    // Its facets' seams, where not a silhouette, are not drawn: a box with
    // one rounded edge has 13 edges and the fillet's silhouette, not 13 plus
    // a seam per facet.
    int sharp = 0;
    for (const auto& e : edges) sharp += e.kind == ProjectedEdge::Kind::Edge ? 1 : 0;
    EXPECT_LE(sharp, 30) << "sharp runs, the box's own edges split by visibility";
}

// The Front view looks from the front, as the viewport's Front does, with
// +X to the right. It looked from behind, mirrored.
TEST(DrawingProjectionTest, TheFrontViewHasXToTheRight) {
    auto box = PrimitiveFactory::makeBox(10, 4, 2);  // x in [0, 10]
    auto edges =
        DrawingProjection::project(*box, DrawingProjection::standardView(StandardView::Front));
    ASSERT_FALSE(edges.empty());
    double lo = 1e9;
    double hi = -1e9;
    for (const auto& e : edges) {
        lo = std::min({lo, e.a.x, e.b.x});
        hi = std::max({hi, e.a.x, e.b.x});
    }
    EXPECT_NEAR(lo, 0.0, 1e-9);
    EXPECT_NEAR(hi, 10.0, 1e-9);
}

// A finely faceted part projects in a moment: every edge was cut into 24
// pieces, however short, and each piece's ray tried every triangle.
TEST(DrawingProjectionTest, AFinelyFacetedCylinderProjectsQuickly) {
    auto cyl = PrimitiveFactory::makeCylinder(10.0, 10.0, 2048);
    const auto start = std::chrono::steady_clock::now();
    auto edges =
        DrawingProjection::project(*cyl, DrawingProjection::standardView(StandardView::Isometric));
    const double seconds =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    EXPECT_FALSE(edges.empty());
    std::printf("[   INFO   ] 2048-facet cylinder projected in %.3f s\n", seconds);
#if HZ_TIME_LIMITS
#ifdef NDEBUG
    EXPECT_LT(seconds, 0.2);
#else
    EXPECT_LT(seconds, 2.0) << "unoptimized: a generous bound";
#endif
#endif
}
