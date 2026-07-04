#include <gtest/gtest.h>

#include <cmath>

#include "horizon/math/Vec2.h"
#include "horizon/math/Vec3.h"
#include "horizon/modeling/DrawingView.h"
#include "horizon/modeling/PrimitiveFactory.h"
#include "horizon/topology/Solid.h"

using hz::math::Vec2;
using hz::math::Vec3;
using hz::model::Drawing;
using hz::model::DrawingGenerator;
using hz::model::DrawingView;
using hz::model::PrimitiveFactory;
using hz::model::StandardView;
using hz::model::ViewProjection;

namespace {

// Placed axis-aligned bounds of a view: [placement, placement + size].
struct Box2 {
    double x0, y0, x1, y1;
};

Box2 placedBox(const DrawingView& v) {
    return {v.placement.x, v.placement.y, v.placement.x + v.width(), v.placement.y + v.height()};
}

bool overlaps(const Box2& a, const Box2& b) {
    return a.x0 < b.x1 && b.x0 < a.x1 && a.y0 < b.y1 && b.y0 < a.y1;
}

}  // namespace

TEST(DrawingViewTest, MakeViewComputesBounds) {
    auto box = PrimitiveFactory::makeBox(4.0, 3.0, 2.0);
    DrawingView v = DrawingGenerator::makeView(*box, StandardView::Front);
    ASSERT_FALSE(v.edges.empty());
    // Front view (look -Y) shows the XZ face: width = X extent, height = Z extent.
    EXPECT_NEAR(v.width(), 4.0, 1e-6);
    EXPECT_NEAR(v.height(), 2.0, 1e-6);
}

TEST(DrawingViewTest, StandardViewsProducesFourNonEmptyViews) {
    auto box = PrimitiveFactory::makeBox(2.0, 2.0, 2.0);
    Drawing d = DrawingGenerator::standardViews(*box);
    ASSERT_EQ(d.views.size(), 4u);

    EXPECT_EQ(d.views[0].kind, StandardView::Front);
    EXPECT_EQ(d.views[1].kind, StandardView::Top);
    EXPECT_EQ(d.views[2].kind, StandardView::Right);
    EXPECT_EQ(d.views[3].kind, StandardView::Isometric);

    for (const auto& v : d.views) {
        EXPECT_FALSE(v.edges.empty());
        EXPECT_GT(v.width(), 0.0);
        EXPECT_GT(v.height(), 0.0);
    }
}

TEST(DrawingViewTest, StandardViewsAreLaidOutWithoutOverlap) {
    auto box = PrimitiveFactory::makeBox(3.0, 2.0, 4.0);
    Drawing d = DrawingGenerator::standardViews(*box, 10.0);
    ASSERT_EQ(d.views.size(), 4u);

    // Every pair of placed views must be disjoint on the sheet.
    for (size_t i = 0; i < d.views.size(); ++i) {
        for (size_t j = i + 1; j < d.views.size(); ++j) {
            EXPECT_FALSE(overlaps(placedBox(d.views[i]), placedBox(d.views[j])))
                << "views " << i << " and " << j << " overlap";
        }
    }
}

TEST(DrawingViewTest, MakeViewFromArbitraryProjection) {
    auto box = PrimitiveFactory::makeBox(4.0, 3.0, 2.0);
    ViewProjection proj;
    proj.dir = Vec3(-1.0, -1.0, -1.0);  // an isometric-ish direction
    proj.up = Vec3(0.0, 0.0, 1.0);

    DrawingView v = DrawingGenerator::makeView(*box, proj);
    EXPECT_FALSE(v.edges.empty());
    EXPECT_GT(v.width(), 0.0);
    EXPECT_GT(v.height(), 0.0);
    // The view records the camera it was projected through.
    EXPECT_DOUBLE_EQ(v.projection.dir.x, -1.0);
    EXPECT_DOUBLE_EQ(v.projection.dir.z, -1.0);
}

TEST(DrawingViewTest, AuxiliaryViewMatchesEquivalentStandardView) {
    // Looking square at the +X face (outward normal +X) means viewing along -X,
    // which is exactly the Right standard view — so the two agree edge-for-edge.
    auto box = PrimitiveFactory::makeBox(4.0, 3.0, 2.0);

    DrawingView aux = DrawingGenerator::auxiliaryView(*box, Vec3(1.0, 0.0, 0.0));
    DrawingView right = DrawingGenerator::makeView(*box, StandardView::Right);

    EXPECT_DOUBLE_EQ(aux.projection.dir.x, -1.0);  // opposite the outward normal
    EXPECT_EQ(aux.edges.size(), right.edges.size());
    EXPECT_NEAR(aux.width(), right.width(), 1e-9);
    EXPECT_NEAR(aux.height(), right.height(), 1e-9);
}

TEST(DrawingViewTest, DetailViewCropsToCircleAndEnlarges) {
    auto box = PrimitiveFactory::makeBox(4.0, 3.0, 2.0);
    DrawingView front = DrawingGenerator::makeView(*box, StandardView::Front);
    ASSERT_FALSE(front.edges.empty());

    // Detail around a corner of the front view.
    const Vec2 corner = front.boundsMin;
    const double radius = 1.0;
    const double scale = 3.0;
    DrawingView detail = DrawingGenerator::detailView(front, corner, radius, scale);

    ASSERT_FALSE(detail.edges.empty());  // the corner has geometry within the circle
    // Cropped geometry lies within `radius` of the corner, then is enlarged
    // `scale`x about it — so every kept point is within radius*scale of the
    // corner.
    const double maxDist = radius * scale + 1e-6;
    for (const auto& e : detail.edges) {
        for (const Vec2& p : {e.a, e.b}) {
            const double dx = p.x - corner.x;
            const double dy = p.y - corner.y;
            EXPECT_LE(std::sqrt(dx * dx + dy * dy), maxDist);
        }
    }
}

TEST(DrawingViewTest, DetailViewEmptyWhenCircleMissesGeometry) {
    auto box = PrimitiveFactory::makeBox(4.0, 3.0, 2.0);
    DrawingView front = DrawingGenerator::makeView(*box, StandardView::Front);
    DrawingView detail = DrawingGenerator::detailView(front, Vec2(1000.0, 1000.0), 1.0, 2.0);
    EXPECT_TRUE(detail.edges.empty());
}
