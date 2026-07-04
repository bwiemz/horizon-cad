#include <gtest/gtest.h>

#include <cmath>

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
