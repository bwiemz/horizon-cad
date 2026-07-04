#include <gtest/gtest.h>

#include <cstdio>
#include <filesystem>
#include <string>

#include "horizon/document/Document.h"
#include "horizon/drafting/DraftDocument.h"
#include "horizon/drafting/DraftText.h"
#include "horizon/fileio/DrawingExport.h"
#include "horizon/fileio/DxfFormat.h"
#include "horizon/fileio/TitleBlockRenderer.h"
#include "horizon/modeling/DrawingView.h"
#include "horizon/modeling/PrimitiveFactory.h"
#include "horizon/modeling/Sheet.h"
#include "horizon/modeling/TitleBlock.h"
#include "horizon/topology/Solid.h"

using hz::doc::Document;
using hz::io::DrawingExport;
using hz::io::DxfFormat;
using hz::io::TitleBlockRenderer;
using hz::model::DrawingGenerator;
using hz::model::PaperSize;
using hz::model::PrimitiveFactory;
using hz::model::Sheet;
using hz::model::TitleBlock;

namespace {
std::string tempPath(const std::string& name) {
    return (std::filesystem::temp_directory_path() / name).string();
}
}  // namespace

// The border is a closed rectangle inset by the sheet margin.
TEST(TitleBlockRendererTest, BorderMatchesSheetInset) {
    Sheet sheet;
    sheet.size = PaperSize::A3;  // 420 x 297
    sheet.margin = 10.0;

    auto entities = TitleBlockRenderer::renderBorder(sheet);
    ASSERT_EQ(entities.size(), 4u);  // four sides

    hz::math::BoundingBox bb;
    for (const auto& e : entities) {
        EXPECT_EQ(e->layer(), TitleBlockRenderer::kBorderLayer);
        bb.expand(e->boundingBox());
    }
    EXPECT_NEAR(bb.min().x, 10.0, 1e-6);
    EXPECT_NEAR(bb.min().y, 10.0, 1e-6);
    EXPECT_NEAR(bb.max().x, 410.0, 1e-6);  // 420 - 10
    EXPECT_NEAR(bb.max().y, 287.0, 1e-6);  // 297 - 10
}

// The title block carries its title text and only the populated fields.
TEST(TitleBlockRendererTest, RendersTitleAndPopulatedFields) {
    Sheet sheet;
    sheet.size = PaperSize::A3;

    TitleBlock block;
    block.title = "BRACKET";
    block.partNumber = "PN-1234";
    block.revision = "B";
    block.drawnBy = "AB";
    // material/company deliberately left empty

    auto entities = TitleBlockRenderer::renderTitleBlock(sheet, block);
    ASSERT_FALSE(entities.empty());

    bool sawTitle = false;
    int textCount = 0;
    for (const auto& e : entities) {
        EXPECT_EQ(e->layer(), TitleBlockRenderer::kTitleBlockLayer);
        if (auto* t = dynamic_cast<const hz::draft::DraftText*>(e.get())) {
            ++textCount;
            if (t->text() == "BRACKET") sawTitle = true;
        }
    }
    EXPECT_TRUE(sawTitle);
    // Title + 4 populated fields (part, rev, drawn, scale, sheet) — at least 4.
    EXPECT_GE(textCount, 4);
}

// A framed drawing exports border + title-block entities onto their layers.
TEST(TitleBlockRendererTest, ExportsFramedDrawing) {
    auto box = PrimitiveFactory::makeBox(4.0, 3.0, 2.0);
    auto drawing = DrawingGenerator::standardViews(*box);

    Sheet sheet;
    sheet.size = PaperSize::A4;
    TitleBlock block;
    block.title = "PLATE";
    block.partNumber = "P-1";

    const std::string path = tempPath("hz_test_titleblock.dxf");
    ASSERT_TRUE(DrawingExport::toDxf(path, drawing, sheet, block));

    Document loaded;
    ASSERT_TRUE(DxfFormat::load(path, loaded));

    int borderEntities = 0;
    int titleBlockEntities = 0;
    for (const auto& e : loaded.draftDocument().entities()) {
        if (e->layer() == TitleBlockRenderer::kBorderLayer) ++borderEntities;
        if (e->layer() == TitleBlockRenderer::kTitleBlockLayer) ++titleBlockEntities;
    }
    EXPECT_EQ(borderEntities, 4);      // rectangle sides
    EXPECT_GT(titleBlockEntities, 0);  // box + dividers + text

    std::remove(path.c_str());
}
