#include <gtest/gtest.h>

#include <cstdio>
#include <filesystem>
#include <string>

#include "horizon/document/Document.h"
#include "horizon/drafting/DraftDocument.h"
#include "horizon/fileio/DrawingExport.h"
#include "horizon/fileio/DxfFormat.h"
#include "horizon/modeling/DrawingView.h"
#include "horizon/modeling/PrimitiveFactory.h"
#include "horizon/topology/Solid.h"

using hz::doc::Document;
using hz::io::DrawingExport;
using hz::io::DxfFormat;
using hz::model::DrawingGenerator;
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
