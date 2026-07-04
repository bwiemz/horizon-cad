#include <gtest/gtest.h>

#include <cstdio>
#include <filesystem>
#include <string>

#include "horizon/document/Document.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/fileio/DrawingDocumentIO.h"
#include "horizon/fileio/NativeFormat.h"

using hz::doc::Document;
using hz::doc::DocumentType;
using hz::doc::PrimitiveFeature;
using hz::io::DrawingDocumentIO;
using hz::io::DrawingDocumentSpec;
using hz::io::NativeFormat;
using hz::model::Drawing;

namespace {
std::string tempPath(const std::string& name) {
    return (std::filesystem::temp_directory_path() / name).string();
}
}  // namespace

// A .hzdwg references a part and regenerates its views on load, so the drawing
// tracks the model rather than storing a stale snapshot.
TEST(DrawingDocumentIOTest, ReferencesPartAndRegeneratesViews) {
    // 1. Author and save a part.
    const std::string partPath = tempPath("hz_test_dwg_part.hzpart");
    {
        Document part;
        part.setType(DocumentType::Part);
        part.featureTree().addFeature(PrimitiveFeature::makeBox(4.0, 3.0, 2.0));
        ASSERT_TRUE(part.rebuildModel());
        ASSERT_TRUE(NativeFormat::save(partPath, part));
    }

    // 2. Save a drawing document referencing the part.
    const std::string dwgPath = tempPath("hz_test_drawing.hzdwg");
    DrawingDocumentSpec spec;
    spec.partPath = partPath;
    spec.gap = 12.0;
    ASSERT_TRUE(DrawingDocumentIO::save(dwgPath, spec));

    // 3. Load the drawing: it re-opens the part and regenerates the views.
    DrawingDocumentSpec loadedSpec;
    Drawing drawing;
    ASSERT_TRUE(DrawingDocumentIO::load(dwgPath, loadedSpec, drawing));

    EXPECT_EQ(loadedSpec.partPath, partPath);
    EXPECT_DOUBLE_EQ(loadedSpec.gap, 12.0);
    ASSERT_EQ(drawing.views.size(), 4u);  // front, top, right, isometric
    for (const auto& v : drawing.views) {
        EXPECT_FALSE(v.edges.empty());
    }

    std::remove(dwgPath.c_str());
    std::remove(partPath.c_str());
}

// A drawing document whose part reference is missing fails to load rather than
// producing an empty drawing.
TEST(DrawingDocumentIOTest, MissingPartReferenceFailsToLoad) {
    const std::string dwgPath = tempPath("hz_test_drawing_missing.hzdwg");
    DrawingDocumentSpec spec;
    spec.partPath = tempPath("hz_test_nonexistent_part.hzpart");
    ASSERT_TRUE(DrawingDocumentIO::save(dwgPath, spec));

    DrawingDocumentSpec loadedSpec;
    Drawing drawing;
    EXPECT_FALSE(DrawingDocumentIO::load(dwgPath, loadedSpec, drawing));

    std::remove(dwgPath.c_str());
}
