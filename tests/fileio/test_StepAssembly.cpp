// Phase 153: STEP assemblies. A file's product structure places its parts:
// each read once, and placed as many times, and where, its assemblies say;
// an assembly written the same way.

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <numbers>
#include <regex>
#include <set>
#include <string>
#include <vector>

#include "horizon/document/AssemblyDocument.h"
#include "horizon/document/Document.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/fileio/ImportReport.h"
#include "horizon/fileio/NativeFormat.h"
#include "horizon/fileio/StepAssemblyFiles.h"
#include "horizon/fileio/StepFormat.h"
#include "horizon/math/Mat4.h"
#include "horizon/modeling/Faceting.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/modeling/PrimitiveFactory.h"
#include "horizon/topology/Solid.h"

namespace fs = std::filesystem;
using hz::io::ImportReport;
using hz::io::StepAssembly;
using hz::io::StepFormat;
using hz::io::StepOccurrence;
using hz::io::StepWritePart;
using hz::math::Mat4;
using hz::math::Vec3;
using hz::model::MassPropertiesCalculator;
using hz::model::PrimitiveFactory;
using hz::topo::Solid;

namespace {

constexpr double kPi = std::numbers::pi;
constexpr double kTetraVolume = 1000.0 / 6.0;

fs::path fixture(const char* name) {
    return fs::path(HZ_STEP_FIXTURE_DIR) / "import_ok" / name;
}

/// @p solid measured as a part measures it: its curved faces in facets on
/// their surfaces, as designed where it can be. (A cylinder read from STEP
/// has one vertex on each circle, which its corners alone mismeasure.)
hz::model::MassProperties measured(const Solid& solid) {
    const auto faceted = hz::model::facetCurved(solid);
    const Solid& facets = faceted.solid ? *faceted.solid : solid;
    const auto ideal = MassPropertiesCalculator::computeIdeal(facets);
    return ideal.exact ? ideal.properties : MassPropertiesCalculator::compute(facets);
}

Vec3 centroidOf(const Solid& solid) {
    return measured(solid).centerOfMass;
}

double volumeOf(const Solid& solid) {
    return measured(solid).volume;
}

void expectNear(const Vec3& actual, const Vec3& expected, double tolerance,
                const std::string& what) {
    EXPECT_NEAR(actual.x, expected.x, tolerance) << what;
    EXPECT_NEAR(actual.y, expected.y, tolerance) << what;
    EXPECT_NEAR(actual.z, expected.z, tolerance) << what;
}

void expectSameTransform(const Mat4& actual, const Mat4& expected, const std::string& what) {
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            EXPECT_NEAR(actual.at(r, c), expected.at(r, c), 1e-9)
                << what << " (" << r << ", " << c << ")";
        }
    }
}

std::set<std::string> faceNames(const Solid& solid) {
    std::set<std::string> names;
    for (const auto& face : solid.faces()) names.insert(face.topoId.tag());
    return names;
}

/// A rig: a bracket placed twice, once turned, and a pin once, tilted.
struct Rig {
    std::unique_ptr<Solid> bracket = PrimitiveFactory::makeBox(10, 20, 30);
    std::unique_ptr<Solid> pin = PrimitiveFactory::makeCylinder(5, 10);

    std::vector<StepWritePart> parts() const {
        return {{"Bracket", {bracket.get()}}, {"Pin", {pin.get()}}};
    }
    static std::vector<StepOccurrence> occurrences() {
        return {{0, "Bracket:1", Mat4::translation({100, 0, 0}) * Mat4::rotationZ(kPi / 2)},
                {0, "Bracket:2", Mat4::identity()},
                {1, "Pin:1", Mat4::translation({0, 50, 0}) * Mat4::rotationX(kPi / 6)}};
    }
    std::string text() const { return StepFormat::assemblyToString("Rig", parts(), occurrences()); }
};

/// The ids of @p text's PRODUCT_DEFINITIONs, in the order written.
std::vector<std::string> definitionIds(const std::string& text) {
    std::vector<std::string> ids;
    const std::regex definition(R"(#(\d+) = PRODUCT_DEFINITION\('design')");
    for (auto it = std::sregex_iterator(text.begin(), text.end(), definition);
         it != std::sregex_iterator(); ++it) {
        ids.push_back((*it)[1].str());
    }
    return ids;
}

/// @p text with @p entities added at the end of its data.
std::string withEntities(std::string text, const std::string& entities) {
    const auto end = text.rfind("ENDSEC;");
    return text.insert(end, entities);
}

}  // namespace

// Written and read back: each part once, and each placement of it, by name.
TEST(StepAssemblyTest, AnAssemblyIsWrittenAndReadBackPartByPart) {
    const Rig rig;
    const StepAssembly read = StepFormat::assemblyFromString(rig.text());
    ASSERT_EQ(read.parts.size(), 2u) << StepFormat::lastError();
    EXPECT_TRUE(read.structured);
    EXPECT_EQ(read.parts[0].name, "Bracket");
    EXPECT_EQ(read.parts[1].name, "Pin");
    ASSERT_EQ(read.parts[0].bodies.size(), 1u);
    ASSERT_EQ(read.parts[1].bodies.size(), 1u);
    EXPECT_NEAR(volumeOf(*read.parts[0].bodies[0]), volumeOf(*rig.bracket), 1e-6);
    expectNear(centroidOf(*read.parts[1].bodies[0]), centroidOf(*rig.pin), 1e-6,
               "the pin, in its own place");

    const auto written = Rig::occurrences();
    ASSERT_EQ(read.occurrences.size(), written.size());
    for (std::size_t k = 0; k < written.size(); ++k) {
        EXPECT_EQ(read.occurrences[k].part, written[k].part);
        EXPECT_EQ(read.occurrences[k].name, written[k].name);
        expectSameTransform(read.occurrences[k].transform, written[k].transform, written[k].name);
    }
}

// Read as solids, as an import into a part is: each placement of each part
// where the assembly puts it, and each its own names, so a face of one
// bracket is not taken for the other's.
TEST(StepAssemblyTest, AnAssemblyReadAsSolidsHasEachPartWhereItIsPlaced) {
    const Rig rig;
    ImportReport report;
    const auto solids = StepFormat::fromString(rig.text(), &report);
    ASSERT_EQ(solids.size(), 3u) << StepFormat::lastError();
    const auto occurrences = Rig::occurrences();
    const Solid* originals[] = {rig.bracket.get(), rig.bracket.get(), rig.pin.get()};
    for (std::size_t k = 0; k < solids.size(); ++k) {
        expectNear(centroidOf(*solids[k]),
                   occurrences[k].transform.transformPoint(centroidOf(*originals[k])), 1e-6,
                   occurrences[k].name);
    }
    EXPECT_NEAR(volumeOf(*solids[0]), volumeOf(*rig.bracket), 1e-6);
    EXPECT_NEAR(volumeOf(*solids[1]), volumeOf(*rig.bracket), 1e-6);

    const auto first = faceNames(*solids[0]);
    for (const auto& name : faceNames(*solids[1])) {
        EXPECT_EQ(first.count(name), 0u) << name << " names a face of both brackets";
    }
    ASSERT_EQ(report.converted.size(), 1u);
    EXPECT_NE(report.converted[0].find("2 parts placed"), std::string::npos) << report.converted[0];
}

// Another system's nesting: an assembly within the assembly, in
// centimetres, and a placement written the other way round. Each part is
// where the assemblies above it, together, put it.
TEST(StepAssemblyTest, ANestedAssemblyCompoundsItsPlacementsAndUnits) {
    ImportReport report;
    const auto solids = StepFormat::load(fixture("assembly_nested_placed.step").string(), &report);
    ASSERT_EQ(solids.size(), 3u) << StepFormat::lastError();
    expectNear(centroidOf(*solids[0]), {107.5, 2.5, 2.5}, 1e-9, "part A, in Sub, in Top");
    expectNear(centroidOf(*solids[1]), {2.5, 2.5, 52.5}, 1e-9, "part A, in Top");
    expectNear(centroidOf(*solids[2]), {22.5, 32.5, 2.5}, 1e-9, "part B, placed the other way");
    for (const auto& solid : solids) EXPECT_NEAR(volumeOf(*solid), kTetraVolume, 1e-9);
    EXPECT_TRUE(report.skipped.empty());
    EXPECT_TRUE(report.approximated.empty()) << report.approximated.front();
}

TEST(StepAssemblyTest, ANestedAssemblysPartsAreReadOnceEach) {
    const StepAssembly read =
        StepFormat::loadAssembly(fixture("assembly_nested_placed.step").string());
    ASSERT_EQ(read.parts.size(), 2u) << StepFormat::lastError();
    EXPECT_TRUE(read.structured);
    EXPECT_EQ(read.parts[0].name, "Part A");
    EXPECT_EQ(read.parts[1].name, "Gr\xC3\xBCn") << "its name, its escape undone";
    ASSERT_EQ(read.occurrences.size(), 3u);
    EXPECT_EQ(read.occurrences[0].part, 0u);
    EXPECT_EQ(read.occurrences[1].part, 0u);
    EXPECT_EQ(read.occurrences[2].part, 1u);
    EXPECT_EQ(read.occurrences[0].name, "Sub:1/A in Sub") << "named by the uses down to it";
    EXPECT_EQ(read.occurrences[1].name, "A in Top");
    // Sub at 100 mm, and in it part A at 1 cm, turned a quarter about z.
    expectNear(read.occurrences[0].transform.transformPoint({0, 0, 0}), {110, 0, 0}, 1e-12,
               "part A's origin");
    expectNear(read.occurrences[0].transform.transformDirection({1, 0, 0}), {0, 1, 0}, 1e-12,
               "part A's x");
}

// A file of parts alone (its products name no shapes) reads as it did
// before assemblies were read: each solid once, where drawn, named by its
// place in the file.
TEST(StepAssemblyTest, AFileOfPartsAloneReadsAsItDidBefore) {
    const auto solids = StepFormat::load(fixture("assembly_two_parts_nauo.step").string());
    ASSERT_EQ(solids.size(), 2u) << StepFormat::lastError();
    EXPECT_EQ(faceNames(*solids[0]).begin()->rfind("step/solid:0/", 0), 0u);
    EXPECT_EQ(faceNames(*solids[1]).begin()->rfind("step/solid:1/", 0), 0u);

    const StepAssembly read =
        StepFormat::loadAssembly(fixture("assembly_two_parts_nauo.step").string());
    EXPECT_FALSE(read.structured);
    ASSERT_EQ(read.parts.size(), 2u);
    EXPECT_EQ(read.parts[0].name, "part_a");
    ASSERT_EQ(read.occurrences.size(), 2u);
    expectSameTransform(read.occurrences[1].transform, Mat4::identity(), "where drawn");

    // Horizon's own part file: one product, one part, placed once.
    const auto box = PrimitiveFactory::makeBox(1, 2, 3);
    const StepAssembly part = StepFormat::assemblyFromString(StepFormat::toString({box.get()}));
    EXPECT_FALSE(part.structured);
    ASSERT_EQ(part.parts.size(), 1u);
    EXPECT_EQ(part.parts[0].name, "part_0");
    EXPECT_EQ(part.occurrences.size(), 1u);
}

// Names travel as written: quotes, backslashes, accents, beyond the BMP.
TEST(StepAssemblyTest, NamesTravelWithQuotesAndAccents) {
    const Rig rig;
    const std::string bracket =
        "Bracket 'A' \\ \xC3\x98"
        "10 \xF0\x9F\x94\xA9";  // Ø10, a bolt
    const std::string use =
        "O'Brien's \xE2\x80\x9C"
        "first\xE2\x80\x9D";  // “first”
    const std::string text = StepFormat::assemblyToString(
        "Rig \xE2\x80\x94 1", {{bracket, {rig.bracket.get()}}}, {{0, use, Mat4::identity()}});
    const StepAssembly read = StepFormat::assemblyFromString(text);
    ASSERT_EQ(read.parts.size(), 1u) << StepFormat::lastError();
    EXPECT_EQ(read.parts[0].name, bracket);
    ASSERT_EQ(read.occurrences.size(), 1u);
    EXPECT_EQ(read.occurrences[0].name, use);
}

// What STEP cannot place is left out, and said; the rest is written.
TEST(StepAssemblyTest, WhatCannotBePlacedIsLeftOutAndSaid) {
    const Rig rig;
    hz::io::StepWriteReport report;
    const std::string text = StepFormat::assemblyToString("Rig", rig.parts(),
                                                          {{0, "doubled", Mat4::scale(2.0)},
                                                           {7, "of no part", Mat4::identity()},
                                                           {1, "pin", Mat4::identity()}},
                                                          {}, &report);
    ASSERT_EQ(report.leftOut.size(), 2u);
    EXPECT_NE(report.leftOut[0].find("doubled"), std::string::npos);
    EXPECT_NE(report.leftOut[1].find("of no part"), std::string::npos);
    const StepAssembly read = StepFormat::assemblyFromString(text);
    ASSERT_EQ(read.parts.size(), 1u) << "the bracket, placed nowhere, is not written";
    EXPECT_EQ(read.parts[0].name, "Pin");
    EXPECT_EQ(read.occurrences.size(), 1u);

    const fs::path path = fs::temp_directory_path() / "hz_step_assembly_nothing.step";
    EXPECT_FALSE(StepFormat::saveAssembly(path.string(), "Rig", rig.parts(),
                                          {{0, "doubled", Mat4::scale(2.0)}}));
    EXPECT_EQ(StepFormat::lastError(), "no component to place");
    EXPECT_FALSE(fs::exists(path));
}

// An assembly that contains itself is followed once, not forever.
TEST(StepAssemblyTest, AnAssemblyThatContainsItselfIsFollowedOnce) {
    const Rig rig;
    const std::string text = rig.text();
    const auto ids = definitionIds(text);
    ASSERT_EQ(ids.size(), 3u);  // the bracket, the pin, the rig
    ImportReport report;
    const auto solids = StepFormat::fromString(
        withEntities(text, "#900000 = NEXT_ASSEMBLY_USAGE_OCCURRENCE('x','loop','',#" + ids[0] +
                               ",#" + ids[0] + ",$);\n"),
        &report);
    EXPECT_EQ(solids.size(), 3u) << StepFormat::lastError();
    ASSERT_EQ(report.approximated.size(), 1u);
    EXPECT_NE(report.approximated[0].find("contains itself"), std::string::npos);
}

// Assemblies that multiply (each using the one below twice, 24 deep) are
// cut short, and said, instead of placing a part 16 million times.
TEST(StepAssemblyTest, AssembliesThatMultiplyAreCutShort) {
    const auto box = PrimitiveFactory::makeBox(1, 1, 1);
    const std::string text = StepFormat::toString({box.get()});
    const auto ids = definitionIds(text);
    ASSERT_EQ(ids.size(), 1u);
    std::string entities;
    int next = 900000;
    std::string below = ids[0];
    for (int level = 0; level < 24; ++level) {
        const std::string product = std::to_string(next++);
        const std::string formation = std::to_string(next++);
        const std::string definition = std::to_string(next++);
        entities += "#" + product + " = PRODUCT('L','L','',());\n";
        entities += "#" + formation + " = PRODUCT_DEFINITION_FORMATION('',$,#" + product + ");\n";
        entities += "#" + definition + " = PRODUCT_DEFINITION('design',$,#" + formation + ",$);\n";
        for (int twice = 0; twice < 2; ++twice) {
            entities += "#" + std::to_string(next++) +
                        " = NEXT_ASSEMBLY_USAGE_OCCURRENCE('u','u',''," + "#" + definition + ",#" +
                        below + ",$);\n";
        }
        below = definition;
    }
    ImportReport report;
    const StepAssembly read = StepFormat::assemblyFromString(withEntities(text, entities), &report);
    ASSERT_EQ(read.parts.size(), 1u) << StepFormat::lastError();
    EXPECT_LE(read.occurrences.size(), 20000u);
    EXPECT_GT(read.occurrences.size(), 1000u);
    ASSERT_EQ(report.skipped.size(), 1u);
    EXPECT_NE(report.skipped[0].find("more times than can be read"), std::string::npos);
    ASSERT_EQ(report.approximated.size(), 1u) << "one line for every use without a placement";
    EXPECT_NE(report.approximated[0].find("48 uses of parts"), std::string::npos)
        << report.approximated[0];
}

// Kept as Horizon files: a part file for each part, its bodies imported, and
// an assembly placing them. A part file already there is not written over,
// and a name no file system takes is made one.
TEST(StepAssemblyTest, AnAssemblyIsKeptAsPartFilesAndAnAssemblyFile) {
    const fs::path dir =
        fs::temp_directory_path() /
        ("hz_step_assembly_" +
         std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    const fs::path partsDir = dir / "Rig parts";
    fs::create_directories(partsDir);
    std::ofstream(partsDir / "Pin.hzpart") << "someone else's";

    const Rig rig;
    std::vector<StepWritePart> parts = rig.parts();
    parts[0].name = "Bracket: left/right";
    StepAssembly read = StepFormat::assemblyFromString(
        StepFormat::assemblyToString("Rig", parts, Rig::occurrences()));
    ASSERT_EQ(read.parts.size(), 2u) << StepFormat::lastError();
    hz::io::StepAssemblyFiles files;
    std::string error;
    const fs::path assemblyPath = dir / "Rig.hzasm";
    ASSERT_TRUE(hz::io::saveStepAssembly(read, assemblyPath.string(), partsDir.string(), "rig.step",
                                         &files, &error))
        << error;
    ASSERT_EQ(files.parts.size(), 2u);
    EXPECT_EQ(fs::path(files.parts[0]).filename(), "Bracket_ left_right.hzpart");
    EXPECT_EQ(fs::path(files.parts[1]).filename(), "Pin 2.hzpart") << "Pin.hzpart was there";
    std::ifstream kept(partsDir / "Pin.hzpart");
    EXPECT_EQ(std::string(std::istreambuf_iterator<char>(kept), {}), "someone else's");

    hz::doc::AssemblyDocument assembly;
    ASSERT_TRUE(hz::io::NativeFormat::loadAssembly(assemblyPath.string(), assembly, &error))
        << error;
    const auto occurrences = Rig::occurrences();
    ASSERT_EQ(assembly.components().size(), occurrences.size());
    for (std::size_t k = 0; k < occurrences.size(); ++k) {
        const auto& component = assembly.components()[k];
        EXPECT_EQ(component.name, occurrences[k].name);
        EXPECT_EQ(fs::path(component.partPath), fs::path(files.parts[occurrences[k].part]));
        expectSameTransform(component.transform, occurrences[k].transform, component.name);
    }

    hz::doc::Document bracket;
    ASSERT_TRUE(hz::io::NativeFormat::load(files.parts[0], bracket, &error)) << error;
    ASSERT_TRUE(bracket.rebuildModel());
    ASSERT_EQ(bracket.featureTree().featureCount(), 1u);
    EXPECT_NE(dynamic_cast<const hz::doc::ImportedBodyFeature*>(bracket.featureTree().feature(0)),
              nullptr);
    ASSERT_NE(bracket.solid(), nullptr);
    EXPECT_NEAR(volumeOf(*bracket.solid()), volumeOf(*rig.bracket), 1e-6);
    fs::remove_all(dir);
}

// Half a UTF-16 pair, or one on its own, is the replacement character, not
// dropped, and not written as a surrogate UTF-8 cannot hold.
TEST(StepAssemblyTest, ABrokenEscapeInANameIsTheReplacementCharacter) {
    const Rig rig;
    std::string text = rig.text();
    const std::string was = "PRODUCT('Pin','Pin'";
    const auto at = text.find(was);
    ASSERT_NE(at, std::string::npos);
    text.replace(at, was.size(),
                 "PRODUCT('Pin','A\\X2\\D83D\\X0\\B\\X2\\DE00\\X0\\C\\X2\\D83DDE00\\X0\\'");
    const StepAssembly read = StepFormat::assemblyFromString(text);
    ASSERT_EQ(read.parts.size(), 2u) << StepFormat::lastError();
    EXPECT_EQ(read.parts[1].name,
              "A\xEF\xBF\xBD"
              "B\xEF\xBF\xBD"
              "C\xF0\x9F\x98\x80");
}

// A part of many solids placed many times is cut short by the solids placed,
// not only by the placements.
TEST(StepAssemblyTest, APartOfManySolidsPlacedManyTimesIsCutShort) {
    std::vector<std::unique_ptr<Solid>> boxes;
    StepWritePart part{"Stack", {}};
    boxes.reserve(30);
    part.bodies.reserve(30);
    for (int k = 0; k < 30; ++k) {
        boxes.push_back(PrimitiveFactory::makeBox(1, 1, 1));
        part.bodies.push_back(boxes.back().get());
    }
    std::vector<StepOccurrence> occurrences;
    occurrences.reserve(1000);
    for (int k = 0; k < 1000; ++k) {
        occurrences.push_back({0, "Stack", Mat4::translation({2.0 * k, 0, 0})});
    }
    ImportReport report;
    const StepAssembly read = StepFormat::assemblyFromString(
        StepFormat::assemblyToString("Stacks", {part}, occurrences), &report);
    ASSERT_EQ(read.parts.size(), 1u) << StepFormat::lastError();
    EXPECT_EQ(read.parts[0].bodies.size(), 30u);
    EXPECT_LE(read.occurrences.size() * 30, 20000u + 30u);
    EXPECT_GT(read.occurrences.size(), 600u);
    ASSERT_EQ(report.skipped.size(), 1u);
    EXPECT_NE(report.skipped[0].find("more times than can be read"), std::string::npos);
}
