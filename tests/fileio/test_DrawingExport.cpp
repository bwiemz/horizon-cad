#include <gtest/gtest.h>

#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <string>

#include "horizon/document/Document.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftDocument.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftText.h"
#include "horizon/drafting/LineType.h"
#include "horizon/fileio/DrawingExport.h"
#include "horizon/fileio/DxfFormat.h"
#include "horizon/math/BoundingBox.h"
#include "horizon/modeling/DrawingBalloon.h"
#include "horizon/modeling/DrawingDimension.h"
#include "horizon/modeling/DrawingView.h"
#include "horizon/modeling/GeometricTolerance.h"
#include "horizon/modeling/PrimitiveFactory.h"
#include "horizon/modeling/SectionView.h"
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

// A section is captioned "SECTION A-A" and its cut drawn across its source
// as a chain line with arrows the way it looks; a detail is captioned with
// its scale and circled on its source. All on the ViewLabels layer.
TEST(DrawingExportTest, SectionsAndDetailsAreCaptionedAndMarked) {
    auto box = PrimitiveFactory::makeBox(100.0, 50.0, 20.0);
    hz::model::Sheet sheet;
    hz::model::TitleBlock tb;
    Drawing drawing = DrawingGenerator::sheetLayout(*box, sheet, tb);
    const hz::model::DrawingView front = drawing.views[0];  // a copy: views are added below
    const double frontScale = front.scale;

    hz::math::BoundingBox bounds;
    for (const auto& v : box->vertices()) bounds.expand(v.point);
    auto section = hz::model::SectionGenerator::sectionView(*box, bounds.center(),
                                                            hz::math::Vec3(1.0, 0.0, 0.0));
    section.role = hz::model::ViewRole::Section;
    section.label = "A";
    section.source = 0;
    section.scale = frontScale;
    section.placement = {30.0, 60.0};

    const hz::math::Vec2 centre{(front.boundsMin.x + front.boundsMax.x) / 2.0, front.boundsMin.y};
    auto detail = DrawingGenerator::detailView(front, centre, 10.0, 1.0);
    detail.role = hz::model::ViewRole::Detail;
    detail.label = "B";
    detail.source = 0;
    detail.detailCenter = centre;
    detail.detailRadius = 10.0;
    detail.scale = 2.0 * frontScale;
    detail.placement = {30.0, 150.0};
    drawing.views.push_back(section);
    drawing.views.push_back(detail);

    Document doc;
    DrawingExport::populate(doc, drawing, &sheet, &tb);
    ASSERT_NE(doc.layerManager().getLayer("ViewLabels"), nullptr);
    const auto& own = DrawingExport::layers();
    EXPECT_NE(std::find(own.begin(), own.end(), "ViewLabels"), own.end());

    std::vector<std::string> texts;
    int chains = 0;
    const hz::draft::DraftCircle* circle = nullptr;
    const hz::math::Vec2 frontLow = front.toSheet(front.boundsMin);
    const hz::math::Vec2 frontHigh = front.toSheet(front.boundsMax);
    for (const auto& e : doc.draftDocument().entities()) {
        if (e->layer() != "ViewLabels") continue;
        if (const auto* t = dynamic_cast<const hz::draft::DraftText*>(e.get())) {
            texts.push_back(t->text());
        } else if (const auto* c = dynamic_cast<const hz::draft::DraftCircle*>(e.get())) {
            circle = c;
        } else if (const auto* l = dynamic_cast<const hz::draft::DraftLine*>(e.get())) {
            if (l->lineType() != static_cast<int>(hz::draft::LineType::Center)) continue;
            ++chains;
            // Vertical, through the middle of Front, and past its top and bottom.
            EXPECT_NEAR(l->start().x, (frontLow.x + frontHigh.x) / 2.0, 1e-6);
            EXPECT_NEAR(l->end().x, l->start().x, 1e-9);
            EXPECT_LT(std::min(l->start().y, l->end().y), frontLow.y);
            EXPECT_GT(std::max(l->start().y, l->end().y), frontHigh.y);
        }
    }
    const auto has = [&](const std::string& s) {
        return std::find(texts.begin(), texts.end(), s) != texts.end();
    };
    EXPECT_TRUE(has("SECTION A-A")) << "at its source's scale, none stated";
    EXPECT_TRUE(has("DETAIL B (" + DrawingGenerator::scaleName(2.0 * frontScale) + ")"));
    EXPECT_EQ(std::count(texts.begin(), texts.end(), "A"), 2) << "a letter at each end of the cut";
    EXPECT_TRUE(has("B"));
    EXPECT_EQ(chains, 1);
    ASSERT_NE(circle, nullptr);
    const hz::math::Vec2 onSheet = front.toSheet(centre);
    EXPECT_NEAR(circle->center().x, onSheet.x, 1e-9);
    EXPECT_NEAR(circle->center().y, onSheet.y, 1e-9);
    EXPECT_NEAR(circle->radius(), 10.0 * frontScale, 1e-9);
}
