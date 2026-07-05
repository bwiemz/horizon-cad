// Era 3 stabilization: end-to-end integration across the drawings, simulation,
// and PDM modules built in Phases 53-63, proving they compose on one model.

#include <gtest/gtest.h>

#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

#include "horizon/document/Document.h"
#include "horizon/drafting/DraftDocument.h"
#include "horizon/fileio/DrawingExport.h"
#include "horizon/fileio/DxfFormat.h"
#include "horizon/fileio/TitleBlockRenderer.h"
#include "horizon/modeling/DrawingBalloon.h"
#include "horizon/modeling/DrawingDimension.h"
#include "horizon/modeling/DrawingView.h"
#include "horizon/modeling/GeometricTolerance.h"
#include "horizon/modeling/PrimitiveFactory.h"
#include "horizon/modeling/Sheet.h"
#include "horizon/modeling/TitleBlock.h"
#include "horizon/pdm/RevisionArchive.h"
#include "horizon/pdm/SemanticDiff.h"
#include "horizon/simulation/LinearStaticSolver.h"
#include "horizon/simulation/Material.h"
#include "horizon/simulation/SolidMesher.h"
#include "horizon/topology/Solid.h"

namespace {
std::string tempPath(const std::string& name) {
    return (std::filesystem::temp_directory_path() / name).string();
}
}  // namespace

// A single model drives a fully-annotated, framed drawing that round-trips
// through DXF with every annotation kind landing on its own layer.
TEST(Era3PipelineTest, AnnotatedFramedDrawingRoundTrips) {
    using namespace hz;

    auto box = model::PrimitiveFactory::makeBox(40.0, 30.0, 20.0);
    model::Drawing drawing = model::DrawingGenerator::standardViews(*box);
    ASSERT_FALSE(drawing.views.empty());

    auto& front = drawing.views.front();
    ASSERT_FALSE(front.edges.empty());
    const auto edgeId = front.edges.front().sourceEdge;

    // Dimension, GD&T frame + datum, and a BOM balloon, all anchored to one edge.
    model::LinearDimension dim;
    ASSERT_TRUE(model::DrawingDimensioner::dimensionEdge(*box, edgeId, dim));
    front.dimensions.push_back(dim);

    model::FeatureControlFrame fcf;
    fcf.characteristic = model::GeometricCharacteristic::Perpendicularity;
    fcf.tolerance = 0.05;
    fcf.modifier = model::MaterialCondition::MMC;
    fcf.datumRefs = {"A"};
    fcf.feature = edgeId;
    front.tolerances.push_back(fcf);

    model::DatumFeature datum;
    datum.label = "A";
    datum.feature = edgeId;
    front.datums.push_back(datum);

    model::DrawingBalloon balloon;
    balloon.item = 1;
    balloon.feature = edgeId;
    front.balloons.push_back(balloon);

    model::Sheet sheet;
    sheet.size = model::PaperSize::A3;
    model::TitleBlock block;
    block.title = "BRACKET";
    block.partNumber = "PN-100";

    const std::string path = tempPath("hz_it_era3_drawing.dxf");
    ASSERT_TRUE(io::DrawingExport::toDxf(path, drawing, sheet, block));

    doc::Document loaded;
    ASSERT_TRUE(io::DxfFormat::load(path, loaded));

    int visible = 0, dimension = 0, tolerance = 0, balloonEnt = 0, border = 0, titleBlock = 0;
    for (const auto& e : loaded.draftDocument().entities()) {
        const std::string& layer = e->layer();
        if (layer == "Visible")
            ++visible;
        else if (layer == "Dimensions")
            ++dimension;
        else if (layer == "Tolerances")
            ++tolerance;
        else if (layer == "Balloons")
            ++balloonEnt;
        else if (layer == io::TitleBlockRenderer::kBorderLayer)
            ++border;
        else if (layer == io::TitleBlockRenderer::kTitleBlockLayer)
            ++titleBlock;
    }
    EXPECT_GT(visible, 0);
    EXPECT_GT(dimension, 0);
    EXPECT_GE(tolerance, 2);   // frame + datum symbol
    EXPECT_GE(balloonEnt, 2);  // circle + number
    EXPECT_EQ(border, 4);      // sheet border rectangle
    EXPECT_GT(titleBlock, 0);

    std::remove(path.c_str());
}

// The same model feeds a finite-element analysis: mesh -> fix a face -> load the
// opposite face -> recover the analytical bar elongation.
TEST(Era3PipelineTest, ModelFeedsFiniteElementAnalysis) {
    using namespace hz;

    const double L = 20.0, a = 2.0, A = a * a;
    auto bar = model::PrimitiveFactory::makeBox(L, a, a);
    sim::TetMesh mesh = sim::meshSolidBoundingBox(*bar, 10, 2, 2);
    ASSERT_FALSE(mesh.elements.empty());

    const auto fixed = sim::nodesOnPlane(mesh, 0, 0.0);
    const auto loaded = sim::nodesOnPlane(mesh, 0, L);
    ASSERT_FALSE(fixed.empty());
    ASSERT_FALSE(loaded.empty());

    const auto mat = sim::materials::aluminum();
    const double F = 2.0e6;
    std::vector<sim::NodalLoad> loads;
    for (int n : loaded) {
        loads.push_back(sim::NodalLoad{n, math::Vec3(F / loaded.size(), 0.0, 0.0)});
    }

    const sim::StaticResult r = sim::LinearStaticSolver::solve(mesh, mat, fixed, loads);
    ASSERT_TRUE(r.converged);

    // The clean analytical quantity is the axial (x) elongation of the loaded
    // face; the max displacement *magnitude* also folds in the free-end lateral
    // Poisson contraction, so compare the axial component directly.
    const double expected = F * L / (A * mat.youngsModulus);
    const double axialTip = r.displacements[loaded.front()].x;
    EXPECT_NEAR(axialTip, expected, 0.05 * expected);
    EXPECT_GT(r.maxVonMises, 0.0);
}

// Saving successive states into the PDM archive and diffing them yields a
// human-readable change set.
TEST(Era3PipelineTest, PdmRevisionsDiffSemantically) {
    using namespace hz;

    const auto dir = std::filesystem::temp_directory_path() / "hz_it_era3_archive";
    std::filesystem::remove_all(dir);

    pdm::RevisionArchive archive(dir.string());
    const std::string v0 = R"({"features":[{"type":"box","w":40}],"params":{"len":100}})";
    const std::string v1 = R"({"features":[{"type":"box","w":42},{"type":"fillet","r":2}],
                               "params":{"len":100}})";
    EXPECT_EQ(archive.commit(v0, "alice", "initial"), 0);
    EXPECT_EQ(archive.commit(v1, "bob", "widen + fillet"), 1);

    std::string a, b;
    ASSERT_TRUE(archive.contentAt(0, a));
    ASSERT_TRUE(archive.contentAt(1, b));

    const auto changes = pdm::diffJson(a, b);
    // The box width changed and a fillet feature was added.
    bool sawWidthChange = false, sawFilletAdd = false;
    for (const auto& c : changes) {
        if (c.path == "/features/0/w" && c.kind == pdm::ChangeKind::Modified) sawWidthChange = true;
        if (c.path == "/features/1" && c.kind == pdm::ChangeKind::Added) sawFilletAdd = true;
    }
    EXPECT_TRUE(sawWidthChange);
    EXPECT_TRUE(sawFilletAdd);

    std::filesystem::remove_all(dir);
}
