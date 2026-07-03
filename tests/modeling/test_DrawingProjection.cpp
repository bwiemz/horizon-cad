#include <gtest/gtest.h>

#include <cmath>

#include "horizon/math/Vec3.h"
#include "horizon/modeling/DrawingProjection.h"
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

TEST(DrawingProjectionTest, FrontViewCollapsesDepthEdges) {
    auto box = PrimitiveFactory::makeBox(2.0, 2.0, 2.0);
    // Front view looks along -Y, so the four Y-parallel "depth" edges project to
    // points and the other eight (in the XZ plane) keep unit length.
    auto edges =
        DrawingProjection::project(*box, DrawingProjection::standardView(StandardView::Front));
    ASSERT_EQ(edges.size(), 12u);

    int collapsed = 0;
    int fullLength = 0;
    for (const auto& e : edges) {
        const double len = segLength(e);
        if (len < 1e-6) {
            ++collapsed;
        } else {
            EXPECT_NEAR(len, 2.0, 1e-6);  // box edge length
            ++fullLength;
        }
    }
    EXPECT_EQ(collapsed, 4);   // the four Y-parallel depth edges
    EXPECT_EQ(fullLength, 8);  // near- and far-face edges in the view plane
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

TEST(DrawingProjectionTest, CylinderProducesSegments) {
    auto cyl = PrimitiveFactory::makeCylinder(3.0, 6.0);
    auto edges =
        DrawingProjection::project(*cyl, DrawingProjection::standardView(StandardView::Front));
    EXPECT_GT(edges.size(), 0u);
    // At least some edge segment should be visible (the near silhouette).
    EXPECT_GT(countVisible(edges), 0);
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
