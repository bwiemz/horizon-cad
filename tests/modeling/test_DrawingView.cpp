#include <gtest/gtest.h>

#include "horizon/modeling/DrawingView.h"
#include "horizon/modeling/PrimitiveFactory.h"
#include "horizon/topology/Solid.h"

using hz::model::Drawing;
using hz::model::DrawingGenerator;
using hz::model::DrawingView;
using hz::model::PrimitiveFactory;
using hz::model::StandardView;

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
