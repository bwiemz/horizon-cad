#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <regex>
#include <sstream>
#include <string>
#include <vector>

#include "horizon/fileio/ImportReport.h"
#include "horizon/fileio/StepFormat.h"
#include "horizon/math/Mat4.h"
#include "horizon/modeling/BooleanOp.h"
#include "horizon/modeling/Faceting.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/modeling/Pattern.h"
#include "horizon/modeling/PrimitiveFactory.h"
#include "horizon/topology/GeometryValidator.h"
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

/// These files name no length unit, as hand-written ones often do: read in
/// millimetres, as the user reading one is asked to choose.
hz::io::StepReadOptions inMillimetres() {
    hz::io::StepReadOptions options;
    options.unknownLengthUnit = hz::math::LengthUnit::Millimetre;
    return options;
}

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
        auto solids = StepFormat::load(file.string(), nullptr, nullptr, inMillimetres());
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
        (kFixtureRoot / "import_ok" / "solidworks_style_tetrahedron.step").string(), nullptr,
        nullptr, inMillimetres());
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
    // This file's products name no shapes (no SHAPE_DEFINITION_REPRESENTATION)
    // and its uses no placements, so its structure places nothing: the parts
    // import as independent solids at their authored coordinates. A file
    // whose structure does place its parts is read placed (Phase 153,
    // test_StepAssembly.cpp).
    auto solids =
        StepFormat::load((kFixtureRoot / "import_ok" / "assembly_two_parts_nauo.step").string(),
                         nullptr, nullptr, inMillimetres());
    ASSERT_EQ(solids.size(), 2u) << StepFormat::lastError();
    EXPECT_NEAR(volumeOf(*solids[0]), kTetraVolume, 1e-6);
    EXPECT_NEAR(volumeOf(*solids[1]), kTetraVolume, 1e-6);
}

TEST(StepFixtures, AMalformedSolidWithVoidsIsRejectedWithWhy) {
    // A BREP_WITH_VOIDS whose faces have empty loops: rejected, saying why,
    // never imported without its cavity.
    auto solids =
        StepFormat::load((kFixtureRoot / "reject" / "brep_with_voids_minimal.step").string(),
                         nullptr, nullptr, inMillimetres());
    EXPECT_TRUE(solids.empty());
    EXPECT_NE(StepFormat::lastError().find("empty edge loop"), std::string::npos)
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

namespace {

/// A 10 mm cube with a 4 mm cube cut out of its middle: a body with a
/// cavity, the second shell a Boolean leaves.
std::unique_ptr<hz::topo::Solid> hollowCube() {
    auto outer = PrimitiveFactory::makeBox(10, 10, 10);
    auto inner = hz::model::Pattern::transformed(*PrimitiveFactory::makeBox(4, 4, 4),
                                                 hz::math::Mat4::translation({3, 3, 3}));
    return hz::model::BooleanOp::execute(*outer, *inner, hz::model::BooleanType::Subtract);
}

}  // namespace

// A body with a cavity goes out as one BREP_WITH_VOIDS (its void a closed
// shell turned round, as the standard has it), not two sibling solids that
// another reader takes for two bodies; and comes back as it went.
TEST(StepFixtures, ASolidWithAVoidGoesOutAndComesBackWithIt) {
    const auto hollow = hollowCube();
    ASSERT_NE(hollow, nullptr);
    ASSERT_EQ(hollow->shells().size(), 2u);
    ASSERT_EQ(hz::model::Pattern::bodyShells(*hollow).size(), 1u) << "one body, with a cavity";
    ASSERT_NEAR(volumeOf(*hollow), 1000.0 - 64.0, 1e-6);

    const std::string text = StepFormat::toString({hollow.get()});
    EXPECT_NE(text.find("BREP_WITH_VOIDS("), std::string::npos);
    EXPECT_NE(text.find("ORIENTED_CLOSED_SHELL('',*,"), std::string::npos);
    EXPECT_EQ(text.find("MANIFOLD_SOLID_BREP("), std::string::npos) << "not a solid per shell";

    hz::io::ImportReport report;
    const auto again = StepFormat::fromString(text, &report);
    ASSERT_EQ(again.size(), 1u) << StepFormat::lastError();
    EXPECT_TRUE(again[0]->isValid()) << again[0]->validationReport();
    EXPECT_EQ(again[0]->shells().size(), 2u);
    EXPECT_EQ(hz::model::Pattern::bodyShells(*again[0]).size(), 1u);
    EXPECT_NEAR(volumeOf(*again[0]), 1000.0 - 64.0, 1e-6);
    EXPECT_TRUE(report.skipped.empty());
}

// Other systems write a solid with voids as a complex instance, each part
// with its own attributes: read the same.
TEST(StepFixtures, ASolidWithVoidsAsAComplexInstanceIsRead) {
    const auto hollow = hollowCube();
    ASSERT_NE(hollow, nullptr);
    std::string text = StepFormat::toString({hollow.get()});
    const std::regex simple(R"(BREP_WITH_VOIDS\('([^']*)',(#\d+),\((#\d+)\)\))");
    std::smatch m;
    ASSERT_TRUE(std::regex_search(text, m, simple));
    text.replace(m.position(0), m.length(0),
                 "(BREP_WITH_VOIDS((" + m[3].str() + "))MANIFOLD_SOLID_BREP(" + m[2].str() +
                     ")REPRESENTATION_ITEM('" + m[1].str() + "')SOLID_MODEL())");

    const auto again = StepFormat::fromString(text);
    ASSERT_EQ(again.size(), 1u) << StepFormat::lastError();
    EXPECT_EQ(again[0]->shells().size(), 2u);
    EXPECT_NEAR(volumeOf(*again[0]), 1000.0 - 64.0, 1e-6);
}

// SolidWorks writes a void's closed shell facing into the void already, and
// marks it turned round as well: taken at its word, the void would be more
// material. It is turned round by what it measures.
TEST(StepFixtures, AVoidMarkedTheWrongWayIsTurnedRound) {
    const auto hollow = hollowCube();
    ASSERT_NE(hollow, nullptr);
    std::string text = StepFormat::toString({hollow.get()});
    const std::string turned = ",.F.)";
    const auto at = text.find(turned, text.find("ORIENTED_CLOSED_SHELL("));
    ASSERT_NE(at, std::string::npos);
    text.replace(at, turned.size(), ",.T.)");

    const auto again = StepFormat::fromString(text);
    ASSERT_EQ(again.size(), 1u) << StepFormat::lastError();
    EXPECT_EQ(hz::model::Pattern::bodyShells(*again[0]).size(), 1u);
    EXPECT_NEAR(volumeOf(*again[0]), 1000.0 - 64.0, 1e-6);
}

// A solid with voids whose shells have no faces: not read, and said why;
// the solid beside it is read.
TEST(StepFixtures, AnEmptySolidWithVoidsIsNotReadAndSaysWhy) {
    std::ifstream in(kFixtureRoot / "import_ok" / "freecad_style_tetrahedron.step",
                     std::ios::binary);
    std::stringstream text;
    text << in.rdbuf();
    std::string file = text.str();
    const auto data = file.rfind("ENDSEC;");
    ASSERT_NE(data, std::string::npos);
    file.insert(data,
                "#9001=CLOSED_SHELL('',());\n#9002=CLOSED_SHELL('',());\n"
                "#9003=ORIENTED_CLOSED_SHELL('',*,#9002,.F.);\n"
                "#9004=BREP_WITH_VOIDS('hollow part',#9001,(#9003));\n");
    hz::io::ImportReport report;
    const auto solids = StepFormat::fromString(file, &report, nullptr, inMillimetres());
    ASSERT_EQ(solids.size(), 1u) << StepFormat::lastError();
    ASSERT_EQ(report.skipped.size(), 1u);
    EXPECT_NE(report.skipped[0].find("(#9004): it has no faces"), std::string::npos)
        << report.skipped[0];
}

namespace {

/// The FreeCAD-style tetrahedron (wound outward, as other systems' files
/// are: this kernel's own primitives face in) rewritten as a surface model,
/// as Creo writes a solid: its shell an OPEN_SHELL of a
/// SHELL_BASED_SURFACE_MODEL in a MANIFOLD_SURFACE_SHAPE_REPRESENTATION.
std::string tetrahedronAsSurfaceModel() {
    std::ifstream in(kFixtureRoot / "import_ok" / "freecad_style_tetrahedron.step",
                     std::ios::binary);
    std::stringstream read;
    read << in.rdbuf();
    std::string text = read.str();
    text = std::regex_replace(text, std::regex(R"(MANIFOLD_SOLID_BREP\('[^']*',(#\d+)\))"),
                              "SHELL_BASED_SURFACE_MODEL('',($1))");
    text = std::regex_replace(text, std::regex("CLOSED_SHELL"), "OPEN_SHELL");
    text = std::regex_replace(text, std::regex("ADVANCED_BREP_SHAPE_REPRESENTATION"),
                              "MANIFOLD_SURFACE_SHAPE_REPRESENTATION");
    return text;
}

}  // namespace

// A surface model's shell that closes is a solid (Creo writes solids so);
// one wound inside out is turned round, to face out as the file's others.
TEST(StepFixtures, ASurfaceModelsClosedShellIsASolid) {
    const std::string text = tetrahedronAsSurfaceModel();
    ASSERT_EQ(text.find("MANIFOLD_SOLID_BREP"), std::string::npos);
    const auto solids = StepFormat::fromString(text);
    ASSERT_EQ(solids.size(), 1u) << StepFormat::lastError();
    EXPECT_TRUE(solids[0]->isValid()) << solids[0]->validationReport();
    ASSERT_EQ(hz::model::measureShells(*solids[0]).size(), 1u);
    EXPECT_NEAR(hz::model::measureShells(*solids[0])[0].volume, kTetraVolume, 1e-6);

    // Every face turned round, its bound and its sense: inside out, which
    // the reader turns back (a solid's volume is positive either way; its
    // shell's signed volume is not).
    std::string inward = std::regex_replace(
        text, std::regex(R"(FACE_OUTER_BOUND\('',(#\d+),\.T\.\))"), "FACE_OUTER_BOUND('',$1,.F.)");
    inward = std::regex_replace(inward, std::regex(R"((ADVANCED_FACE\('',\(#\d+\),#\d+,)\.T\.\))"),
                                "$1.F.)");
    ASSERT_NE(inward, text);
    const auto turned = StepFormat::fromString(inward);
    ASSERT_EQ(turned.size(), 1u) << StepFormat::lastError();
    const auto shells = hz::model::measureShells(*turned[0]);
    ASSERT_EQ(shells.size(), 1u);
    EXPECT_NEAR(shells[0].volume, kTetraVolume, 1e-6) << "turned round, not inside out";
}

// A surface model's shell that does not close is an open surface, not a
// solid: not read, and said so.
TEST(StepFixtures, ASurfaceModelsOpenShellIsSaidToBeASurface) {
    std::string text = tetrahedronAsSurfaceModel();
    std::smatch shell;
    ASSERT_TRUE(std::regex_search(text, shell, std::regex(R"(OPEN_SHELL\('',\((#\d+),)")));
    text.replace(static_cast<std::size_t>(shell.position(0)),
                 static_cast<std::size_t>(shell.length(0)), "OPEN_SHELL('',(");
    EXPECT_TRUE(StepFormat::fromString(text).empty());
    EXPECT_NE(StepFormat::lastError().find("open surface, not a solid"), std::string::npos)
        << StepFormat::lastError();
}

// A flat face whose corner lies off its plane (Creo's, by 0.75 mm) is cut
// into triangles, each flat, the corners its edges' own: the part is valid.
TEST(StepFixtures, AFlatFaceOffItsPlaneIsCutIntoFlatTriangles) {
    auto box = PrimitiveFactory::makeBox(7.0, 11.0, 13.0);
    std::string text = StepFormat::toString({box.get()});
    // The corner at (7, 11, 13) moved 0.2 up, off the planes of its faces.
    const std::string corner = "(7.,11.,13.)";
    const auto at = text.find(corner);
    ASSERT_NE(at, std::string::npos);
    text.replace(at, corner.size(), "(7.,11.,13.2)");
    const auto solids = StepFormat::fromString(text);
    ASSERT_EQ(solids.size(), 1u) << StepFormat::lastError();
    const auto faceted = hz::model::facetCurved(*solids[0]);
    ASSERT_NE(faceted.solid, nullptr) << faceted.error;
    EXPECT_TRUE(faceted.solid->isValid()) << faceted.solid->validationReport();
    const auto issues = hz::topo::GeometryValidator::check(*faceted.solid);
    EXPECT_EQ(issues.nonPlanarLoops, 0);
    EXPECT_NEAR(volumeOf(*faceted.solid), 7.0 * 11 * 13, 20.0) << "about the box";
}
