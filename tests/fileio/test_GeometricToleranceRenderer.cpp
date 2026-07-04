#include <gtest/gtest.h>

#include "horizon/drafting/DraftText.h"
#include "horizon/fileio/GeometricToleranceRenderer.h"
#include "horizon/modeling/DrawingView.h"
#include "horizon/modeling/GeometricTolerance.h"
#include "horizon/modeling/PrimitiveFactory.h"
#include "horizon/topology/Solid.h"
#include "horizon/topology/TopologyID.h"

using hz::io::GeometricToleranceRenderer;
using hz::model::DatumFeature;
using hz::model::DrawingGenerator;
using hz::model::DrawingView;
using hz::model::FeatureControlFrame;
using hz::model::Gdt;
using hz::model::GeometricCharacteristic;
using hz::model::MaterialCondition;
using hz::model::PrimitiveFactory;
using hz::model::StandardView;
using hz::topo::TopologyID;

// A feature control frame anchored to an edge in the view renders as text whose
// content is the formatted frame.
TEST(GeometricToleranceRendererTest, RendersFrameInView) {
    auto box = PrimitiveFactory::makeBox(4.0, 3.0, 2.0);
    DrawingView front = DrawingGenerator::makeView(*box, StandardView::Front);
    ASSERT_FALSE(front.edges.empty());

    FeatureControlFrame frame;
    frame.characteristic = GeometricCharacteristic::Perpendicularity;
    frame.tolerance = 0.05;
    frame.modifier = MaterialCondition::MMC;
    frame.datumRefs = {"A"};
    frame.feature = front.edges.front().sourceEdge;

    auto text = GeometricToleranceRenderer::render(front, frame, 8.0);
    ASSERT_NE(text, nullptr);
    EXPECT_EQ(text->text(), Gdt::format(frame));
}

// A frame whose feature is not in the view renders to nothing.
TEST(GeometricToleranceRendererTest, ReturnsNullWhenFeatureNotInView) {
    auto box = PrimitiveFactory::makeBox(4.0, 3.0, 2.0);
    DrawingView front = DrawingGenerator::makeView(*box, StandardView::Front);

    FeatureControlFrame frame;
    frame.characteristic = GeometricCharacteristic::Flatness;
    frame.tolerance = 0.1;
    frame.feature = TopologyID::make("box", "edge999");  // not present in model/view
    EXPECT_EQ(GeometricToleranceRenderer::render(front, frame, 8.0), nullptr);
}

// A datum feature anchored to an edge renders as its boxed reference letter.
TEST(GeometricToleranceRendererTest, RendersDatumFeature) {
    auto box = PrimitiveFactory::makeBox(4.0, 3.0, 2.0);
    DrawingView front = DrawingGenerator::makeView(*box, StandardView::Front);
    ASSERT_FALSE(front.edges.empty());

    DatumFeature datum;
    datum.label = "A";
    datum.feature = front.edges.front().sourceEdge;

    auto text = GeometricToleranceRenderer::render(front, datum, -8.0);
    ASSERT_NE(text, nullptr);
    EXPECT_EQ(text->text(), Gdt::datumSymbol(datum));
}
