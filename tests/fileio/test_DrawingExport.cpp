#include <gtest/gtest.h>

#include <cstdio>
#include <filesystem>
#include <string>

#include "horizon/document/Document.h"
#include "horizon/drafting/DraftDocument.h"
#include "horizon/fileio/DrawingExport.h"
#include "horizon/fileio/DxfFormat.h"
#include "horizon/modeling/DrawingBalloon.h"
#include "horizon/modeling/DrawingDimension.h"
#include "horizon/modeling/DrawingView.h"
#include "horizon/modeling/GeometricTolerance.h"
#include "horizon/modeling/PrimitiveFactory.h"
#include "horizon/topology/Solid.h"

using hz::doc::Document;
using hz::io::DrawingExport;
using hz::io::DxfFormat;
using hz::model::Drawing;
using hz::model::DrawingDimensioner;
using hz::model::DrawingGenerator;
using hz::model::FeatureControlFrame;
using hz::model::GeometricCharacteristic;
using hz::model::LinearDimension;
using hz::model::PrimitiveFactory;

namespace {
std::string tempPath(const std::string& name) {
    return (std::filesystem::temp_directory_path() / name).string();
}
}  // namespace

// A projected drawing exports to DXF and round-trips: visible edges land on a
// "Visible" layer, hidden edges on a "Hidden" layer.
TEST(DrawingExportTest, StandardViewsRoundTripThroughDxf) {
    auto box = PrimitiveFactory::makeBox(4.0, 3.0, 2.0);
    const std::string path = tempPath("hz_test_drawing_export.dxf");

    ASSERT_TRUE(DrawingExport::standardViewsToDxf(path, *box));

    Document loaded;
    ASSERT_TRUE(DxfFormat::load(path, loaded));

    const auto& entities = loaded.draftDocument().entities();
    ASSERT_FALSE(entities.empty());

    int visibleLines = 0;
    int hiddenLines = 0;
    for (const auto& e : entities) {
        if (e->layer() == "Visible") {
            ++visibleLines;
        } else if (e->layer() == "Hidden") {
            ++hiddenLines;
        }
    }
    // A box always has both visible near-face edges and hidden far-face edges.
    EXPECT_GT(visibleLines, 0);
    EXPECT_GT(hiddenLines, 0);

    std::remove(path.c_str());
}

// The multi-view layout offsets each view to its placement, so the exported
// geometry spans well beyond a single view's extent (views do not overlap).
TEST(DrawingExportTest, MultiViewLayoutIsPlacedOnSheet) {
    auto box = PrimitiveFactory::makeBox(2.0, 2.0, 2.0);
    auto drawing = DrawingGenerator::standardViews(*box, 10.0);
    const std::string path = tempPath("hz_test_drawing_export_layout.dxf");

    ASSERT_TRUE(DrawingExport::toDxf(path, drawing));

    Document loaded;
    ASSERT_TRUE(DxfFormat::load(path, loaded));
    const auto& entities = loaded.draftDocument().entities();
    ASSERT_FALSE(entities.empty());

    // Some geometry must sit in the right column / upper row (placed views),
    // proving the placement offsets were applied rather than all views stacking
    // at the origin. Box edge is 2 and the gap is 10, so placed views start
    // near x=12 or y=12.
    bool sawPlaced = false;
    for (const auto& e : entities) {
        const hz::math::BoundingBox bb = e->boundingBox();
        if (bb.max().x > 11.0 || bb.max().y > 11.0) {
            sawPlaced = true;
            break;
        }
    }
    EXPECT_TRUE(sawPlaced);

    std::remove(path.c_str());
}

// A drawing carrying dimensions exports them: after round-trip, entities land on
// the "Dimensions" layer (the DXF writer decomposes each dimension to lines/text).
TEST(DrawingExportTest, ExportsDimensionsOnDimensionLayer) {
    auto box = PrimitiveFactory::makeBox(4.0, 3.0, 2.0);
    Drawing drawing = DrawingGenerator::standardViews(*box);
    ASSERT_FALSE(drawing.views.empty());

    // Anchor a dimension to an edge of the front view.
    auto& front = drawing.views.front();
    ASSERT_FALSE(front.edges.empty());
    LinearDimension dim;
    ASSERT_TRUE(DrawingDimensioner::dimensionEdge(*box, front.edges.front().sourceEdge, dim));
    front.dimensions.push_back(dim);

    const std::string path = tempPath("hz_test_drawing_export_dims.dxf");
    ASSERT_TRUE(DrawingExport::toDxf(path, drawing));

    Document loaded;
    ASSERT_TRUE(DxfFormat::load(path, loaded));

    int dimensionEntities = 0;
    for (const auto& e : loaded.draftDocument().entities()) {
        if (e->layer() == "Dimensions") ++dimensionEntities;
    }
    EXPECT_GT(dimensionEntities, 0);

    std::remove(path.c_str());
}

// A drawing carrying GD&T frames/datums exports them as text on the "Tolerances"
// layer, anchored to the toleranced feature's projected edge.
TEST(DrawingExportTest, ExportsGdtOnToleranceLayer) {
    auto box = PrimitiveFactory::makeBox(4.0, 3.0, 2.0);
    Drawing drawing = DrawingGenerator::standardViews(*box);
    ASSERT_FALSE(drawing.views.empty());

    auto& front = drawing.views.front();
    ASSERT_FALSE(front.edges.empty());
    const auto edgeId = front.edges.front().sourceEdge;

    FeatureControlFrame frame;
    frame.characteristic = GeometricCharacteristic::Perpendicularity;
    frame.tolerance = 0.05;
    frame.datumRefs = {"A"};
    frame.feature = edgeId;
    front.tolerances.push_back(frame);

    hz::model::DatumFeature datum;
    datum.label = "A";
    datum.feature = edgeId;
    front.datums.push_back(datum);

    const std::string path = tempPath("hz_test_drawing_export_gdt.dxf");
    ASSERT_TRUE(DrawingExport::toDxf(path, drawing));

    Document loaded;
    ASSERT_TRUE(DxfFormat::load(path, loaded));

    int toleranceEntities = 0;
    for (const auto& e : loaded.draftDocument().entities()) {
        if (e->layer() == "Tolerances") ++toleranceEntities;
    }
    EXPECT_GE(toleranceEntities, 2);  // one frame + one datum symbol

    std::remove(path.c_str());
}

// A drawing carrying BOM balloons exports them on the "Balloons" layer (a circle
// and a number, at minimum, per balloon).
TEST(DrawingExportTest, ExportsBalloonsOnBalloonLayer) {
    auto box = PrimitiveFactory::makeBox(4.0, 3.0, 2.0);
    Drawing drawing = DrawingGenerator::standardViews(*box);
    ASSERT_FALSE(drawing.views.empty());

    auto& front = drawing.views.front();
    ASSERT_FALSE(front.edges.empty());

    hz::model::DrawingBalloon balloon;
    balloon.item = 3;
    balloon.feature = front.edges.front().sourceEdge;
    front.balloons.push_back(balloon);

    const std::string path = tempPath("hz_test_drawing_export_balloons.dxf");
    ASSERT_TRUE(DrawingExport::toDxf(path, drawing));

    Document loaded;
    ASSERT_TRUE(DxfFormat::load(path, loaded));

    int balloonEntities = 0;
    for (const auto& e : loaded.draftDocument().entities()) {
        if (e->layer() == "Balloons") ++balloonEntities;
    }
    EXPECT_GE(balloonEntities, 2);  // at least the circle and the number text

    std::remove(path.c_str());
}
