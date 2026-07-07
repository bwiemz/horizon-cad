#include <gtest/gtest.h>

#include <filesystem>
#include <string>
#include <vector>

#include "horizon/fileio/StepFormat.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/modeling/PrimitiveFactory.h"
#include "horizon/topology/Solid.h"

using hz::io::StepFormat;
using hz::model::MassPropertiesCalculator;
using hz::model::PrimitiveFactory;

namespace fs = std::filesystem;

// HZ_STEP_FIXTURE_DIR is provided by tests/fileio/CMakeLists.txt and points
// at tests/fileio/fixtures/step in the source tree.
#ifndef HZ_STEP_FIXTURE_DIR
#error "HZ_STEP_FIXTURE_DIR must be defined by the build"
#endif

namespace {

const fs::path kFixtureRoot{HZ_STEP_FIXTURE_DIR};

std::vector<fs::path> stepFilesIn(const std::string& subdir) {
    std::vector<fs::path> files;
    const fs::path dir = kFixtureRoot / subdir;
    for (const auto& entry : fs::directory_iterator(dir)) {
        if (entry.path().extension() == ".step" || entry.path().extension() == ".STEP") {
            files.push_back(entry.path());
        }
    }
    return files;
}

double volumeOf(const hz::topo::Solid& solid) {
    return MassPropertiesCalculator::compute(solid).volume;
}

constexpr double kTetraVolume = 1000.0 / 6.0;

}  // namespace

// ===========================================================================
// Fixture directory scans — every checked-in (or user-dropped) file must
// behave per its directory's contract.  Real exports from FreeCAD, Onshape,
// SolidWorks, Fusion etc. can be dropped into import_ok/ and are validated
// automatically (see fixtures/step/README.md).
// ===========================================================================

TEST(StepFixtures, EveryImportOkFixtureLoadsAsManifoldGeometry) {
    const auto files = stepFilesIn("import_ok");
    ASSERT_FALSE(files.empty()) << "no fixtures found under " << kFixtureRoot;

    for (const auto& file : files) {
        auto solids = StepFormat::load(file.string());
        ASSERT_FALSE(solids.empty())
            << file.filename() << " failed to import: " << StepFormat::lastError();
        for (size_t i = 0; i < solids.size(); ++i) {
            EXPECT_TRUE(solids[i]->isValid())
                << file.filename() << " solid " << i << ": " << solids[i]->validationReport();
            EXPECT_GT(volumeOf(*solids[i]), 0.0)
                << file.filename() << " solid " << i << " has no enclosed volume";
        }
    }
}

TEST(StepFixtures, EveryRejectFixtureFailsWithClearError) {
    const auto files = stepFilesIn("reject");
    ASSERT_FALSE(files.empty()) << "no fixtures found under " << kFixtureRoot;

    for (const auto& file : files) {
        auto solids = StepFormat::load(file.string());
        EXPECT_TRUE(solids.empty())
            << file.filename() << " imported but is expected to be rejected";
        EXPECT_FALSE(StepFormat::lastError().empty())
            << file.filename() << " was rejected without a diagnostic";
    }
}

// ===========================================================================
// Specific exporter-style fixtures.
// ===========================================================================

TEST(StepFixtures, FreeCadStyleAnalyticGeometryImportsExactly) {
    auto solids =
        StepFormat::load((kFixtureRoot / "import_ok" / "freecad_style_tetrahedron.step").string());
    ASSERT_EQ(solids.size(), 1u) << StepFormat::lastError();
    EXPECT_EQ(solids[0]->faceCount(), 4u);
    EXPECT_TRUE(solids[0]->isValid()) << solids[0]->validationReport();
    EXPECT_NEAR(volumeOf(*solids[0]), kTetraVolume, 1e-6);
}

TEST(StepFixtures, SolidWorksStyleScrambledOrderImportsIdentically) {
    // Same geometry as the FreeCAD-style file, but entities in reverse order
    // with forward references, wrapped argument lists, comments in DATA, and
    // an AP203 schema string — the importer must be insensitive to all of it.
    auto sw = StepFormat::load(
        (kFixtureRoot / "import_ok" / "solidworks_style_tetrahedron.step").string());
    ASSERT_EQ(sw.size(), 1u) << StepFormat::lastError();
    EXPECT_NEAR(volumeOf(*sw[0]), kTetraVolume, 1e-6);

    const auto swProps = MassPropertiesCalculator::compute(*sw[0]);
    auto fc =
        StepFormat::load((kFixtureRoot / "import_ok" / "freecad_style_tetrahedron.step").string());
    ASSERT_EQ(fc.size(), 1u);
    const auto fcProps = MassPropertiesCalculator::compute(*fc[0]);
    EXPECT_NEAR(swProps.volume, fcProps.volume, 1e-9);
    EXPECT_NEAR(swProps.surfaceArea, fcProps.surfaceArea, 1e-9);
}

TEST(StepFixtures, AssemblyProductStructureIsFlattenedToParts) {
    // Documented limitation: NEXT_ASSEMBLY_USAGE_OCCURRENCE product structure
    // is not mapped.  The parts import as independent solids at their
    // authored coordinates; assembly transforms are ignored.  If assembly
    // support lands, this test should flip to assert the structure.
    auto solids =
        StepFormat::load((kFixtureRoot / "import_ok" / "assembly_two_parts_nauo.step").string());
    ASSERT_EQ(solids.size(), 2u) << StepFormat::lastError();
    EXPECT_NEAR(volumeOf(*solids[0]), kTetraVolume, 1e-6);
    EXPECT_NEAR(volumeOf(*solids[1]), kTetraVolume, 1e-6);
}

TEST(StepFixtures, BrepWithVoidsIsRejectedWithDocumentedError) {
    // Documented limitation: BREP_WITH_VOIDS (internal cavities) is not
    // mapped.  The file must be rejected with a diagnostic naming what was
    // missing — never silently imported without its cavity.
    auto solids =
        StepFormat::load((kFixtureRoot / "reject" / "brep_with_voids_minimal.step").string());
    EXPECT_TRUE(solids.empty());
    EXPECT_NE(StepFormat::lastError().find("MANIFOLD_SOLID_BREP"), std::string::npos)
        << StepFormat::lastError();
}

// ===========================================================================
// Formatting-robustness round trips: Horizon's own export restyled the way
// third-party tools format their files.
// ===========================================================================

namespace {

/// Re-import restyled text and compare mass properties against the original.
void expectRestyledReimportMatches(const hz::topo::Solid& original, const std::string& restyled) {
    auto solids = StepFormat::fromString(restyled);
    ASSERT_EQ(solids.size(), 1u) << StepFormat::lastError();
    const auto a = MassPropertiesCalculator::compute(original);
    const auto b = MassPropertiesCalculator::compute(*solids[0]);
    EXPECT_NEAR(a.volume, b.volume, std::abs(a.volume) * 1e-6);
    EXPECT_NEAR(a.surfaceArea, b.surfaceArea, std::abs(a.surfaceArea) * 1e-6);
}

}  // namespace

TEST(StepFixtures, ReimportSurvivesCommentsAndWhitespaceReflow) {
    auto box = PrimitiveFactory::makeBox(7.0, 11.0, 13.0);
    ASSERT_NE(box, nullptr);
    std::string text = StepFormat::toString({box.get()});

    // Inject a comment after every statement and reflow argument lists onto
    // separate lines (no string literal in the export contains a comma).
    std::string restyled;
    restyled.reserve(text.size() * 2);
    for (char c : text) {
        if (c == ',') {
            restyled += ",\n    ";
        } else if (c == ';') {
            restyled += "; /* restyled */";
        } else {
            restyled += c;
        }
    }
    expectRestyledReimportMatches(*box, restyled);
}

TEST(StepFixtures, ReimportSurvivesEntityReordering) {
    auto box = PrimitiveFactory::makeBox(3.0, 5.0, 9.0);
    ASSERT_NE(box, nullptr);
    const std::string text = StepFormat::toString({box.get()});

    // Reverse the DATA-section statements so every reference points forward.
    const size_t dataPos = text.find("DATA;");
    const size_t endPos = text.find("ENDSEC;", dataPos);
    ASSERT_NE(dataPos, std::string::npos);
    ASSERT_NE(endPos, std::string::npos);

    std::vector<std::string> statements;
    size_t pos = dataPos + 5;
    while (pos < endPos) {
        const size_t semi = text.find(';', pos);
        if (semi == std::string::npos || semi >= endPos) break;
        std::string stmt = text.substr(pos, semi - pos + 1);
        if (stmt.find('#') != std::string::npos) statements.push_back(std::move(stmt));
        pos = semi + 1;
    }
    ASSERT_GT(statements.size(), 10u);

    std::string restyled = text.substr(0, dataPos + 5) + "\n";
    for (auto it = statements.rbegin(); it != statements.rend(); ++it) {
        restyled += *it;
        restyled += '\n';
    }
    restyled += text.substr(endPos);
    expectRestyledReimportMatches(*box, restyled);
}
