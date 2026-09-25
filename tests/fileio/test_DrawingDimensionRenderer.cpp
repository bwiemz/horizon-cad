#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <tuple>

#include "horizon/drafting/DimensionStyle.h"
#include "horizon/drafting/DraftLinearDimension.h"
#include "horizon/fileio/DrawingDimensionRenderer.h"
#include "horizon/modeling/DrawingDimension.h"
#include "horizon/modeling/DrawingView.h"
#include "horizon/modeling/PrimitiveFactory.h"
#include "horizon/topology/Solid.h"
#include "horizon/topology/TopologyID.h"

using hz::io::DrawingDimensionRenderer;
using hz::model::DrawingDimensioner;
using hz::model::DrawingGenerator;
using hz::model::DrawingView;
using hz::model::LinearDimension;
using hz::model::PrimitiveFactory;
using hz::model::StandardView;
using hz::topo::TopologyID;

// A model-driven dimension renders onto its view as a drafted linear dimension
// spanning the edge's projection.
TEST(DrawingDimensionRendererTest, RendersEdgeDimensionInView) {
    auto box = PrimitiveFactory::makeBox(4.0, 3.0, 2.0);
    DrawingView front = DrawingGenerator::makeView(*box, StandardView::Front);
    ASSERT_FALSE(front.edges.empty());

    // Anchor a dimension to an edge that appears in the front view.
    const TopologyID edgeId = front.edges.front().sourceEdge;
    LinearDimension dim;
    ASSERT_TRUE(DrawingDimensioner::dimensionEdge(*box, edgeId, dim));

    auto drafted = DrawingDimensionRenderer::render(front, dim, 5.0);
    ASSERT_NE(drafted, nullptr);

    // The drafted length equals the edge's in-plane projected length (which, for
    // a front-view edge, is the true model length).
    const auto& pe = front.edges.front();
    const double projLen = std::hypot(pe.a.x - pe.b.x, pe.a.y - pe.b.y);
    EXPECT_NEAR(drafted->computedValue(), projLen, 1e-6);
    EXPECT_NEAR(drafted->computedValue(), dim.value, 1e-6);
}

// A dimension whose edge is not in the view renders to nothing.
TEST(DrawingDimensionRendererTest, ReturnsNullWhenEdgeNotInView) {
    auto box = PrimitiveFactory::makeBox(4.0, 3.0, 2.0);
    DrawingView front = DrawingGenerator::makeView(*box, StandardView::Front);

    LinearDimension dim;
    dim.edge = TopologyID::make("box", "edge999");  // not present in the model/view
    dim.value = 1.0;
    EXPECT_EQ(DrawingDimensionRenderer::render(front, dim, 5.0), nullptr);
}

// A partly hidden edge is drawn in runs: its dimension spans them all, end
// to end. It went on the first run only, stating the whole length there.
TEST(DrawingDimensionRendererTest, APartlyHiddenEdgeIsDimensionedEndToEnd) {
    const auto edge = hz::topo::TopologyID::fromTag("box/edge/e1");
    hz::model::DrawingView view;
    for (const auto& [a, b, hidden] :
         {std::tuple{hz::math::Vec2(4, 0), hz::math::Vec2(6, 0), true},
          std::tuple{hz::math::Vec2(0, 0), hz::math::Vec2(4, 0), false},
          std::tuple{hz::math::Vec2(6, 0), hz::math::Vec2(10, 0), false}}) {
        hz::model::ProjectedEdge e;
        e.a = a;
        e.b = b;
        e.sourceEdge = edge;
        e.visibility = hidden ? hz::model::ProjectedEdge::Visibility::Hidden
                              : hz::model::ProjectedEdge::Visibility::Visible;
        view.edges.push_back(e);
    }
    view.boundsMax = {10.0, 0.0};
    view.scale = 2.0;
    view.placement = {100.0, 50.0};
    hz::model::LinearDimension dim;
    dim.edge = edge;
    dim.value = 10.0;
    const auto drafted = hz::io::DrawingDimensionRenderer::render(view, dim, 5.0);
    ASSERT_NE(drafted, nullptr);
    const double from = std::min(drafted->defPoint1().x, drafted->defPoint2().x);
    const double to = std::max(drafted->defPoint1().x, drafted->defPoint2().x);
    EXPECT_NEAR(from, 100.0, 1e-9);
    EXPECT_NEAR(to, 120.0, 1e-9) << "the whole edge, at 2:1";
    EXPECT_EQ(drafted->displayText(hz::draft::DimensionStyle{}),
              hz::draft::DimensionStyle{}.formatLength(10.0))
        << "stated at 1:1";
}
