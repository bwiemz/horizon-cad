#include <gtest/gtest.h>

#include <cmath>
#include <memory>

#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftText.h"
#include "horizon/fileio/BalloonRenderer.h"
#include "horizon/modeling/DrawingBalloon.h"
#include "horizon/modeling/DrawingView.h"
#include "horizon/modeling/PrimitiveFactory.h"
#include "horizon/topology/Solid.h"
#include "horizon/topology/TopologyID.h"

using hz::io::BalloonRenderer;
using hz::model::DrawingBalloon;
using hz::model::DrawingGenerator;
using hz::model::DrawingView;
using hz::model::PrimitiveFactory;
using hz::model::StandardView;
using hz::topo::TopologyID;

// A balloon anchored to an edge in the view renders a circle plus the item
// number text, and the circle sits at the configured offset from the edge.
TEST(BalloonRendererTest, RendersCircleAndNumber) {
    auto box = PrimitiveFactory::makeBox(4.0, 3.0, 2.0);
    DrawingView front = DrawingGenerator::makeView(*box, StandardView::Front);
    ASSERT_FALSE(front.edges.empty());

    DrawingBalloon balloon;
    balloon.item = 7;
    balloon.feature = front.edges.front().sourceEdge;
    balloon.offset = {15.0, 15.0};
    balloon.radius = 4.0;

    auto entities = BalloonRenderer::render(front, balloon);
    ASSERT_FALSE(entities.empty());

    const hz::draft::DraftCircle* circle = nullptr;
    const hz::draft::DraftText* text = nullptr;
    for (const auto& e : entities) {
        if (auto* c = dynamic_cast<const hz::draft::DraftCircle*>(e.get())) circle = c;
        if (auto* t = dynamic_cast<const hz::draft::DraftText*>(e.get())) text = t;
    }
    ASSERT_NE(circle, nullptr);
    ASSERT_NE(text, nullptr);
    EXPECT_EQ(text->text(), "7");  // the BOM item number
    EXPECT_NEAR(circle->radius(), 4.0, 1e-9);

    // The circle center is the edge midpoint (on the sheet) plus the offset, so
    // it is at least the offset distance from the edge.
    const auto& pe = front.edges.front();
    const hz::math::Vec2 mid((pe.a.x + pe.b.x) * 0.5 - front.boundsMin.x + front.placement.x,
                             (pe.a.y + pe.b.y) * 0.5 - front.boundsMin.y + front.placement.y);
    EXPECT_NEAR(circle->center().x, mid.x + 15.0, 1e-9);
    EXPECT_NEAR(circle->center().y, mid.y + 15.0, 1e-9);
}

// A balloon whose feature is not in the view renders to nothing.
TEST(BalloonRendererTest, ReturnsEmptyWhenFeatureNotInView) {
    auto box = PrimitiveFactory::makeBox(4.0, 3.0, 2.0);
    DrawingView front = DrawingGenerator::makeView(*box, StandardView::Front);

    DrawingBalloon balloon;
    balloon.item = 1;
    balloon.feature = TopologyID::make("box", "edge999");  // not in the model/view
    EXPECT_TRUE(BalloonRenderer::render(front, balloon).empty());
}
