// Part files saved before the exact kernel (stage 2, Ruling 10): each holds its imported bodies as
// STEP the writer of its day wrote, which no exact form may hold. Such a part opens exact where its
// exact form holds; a body whose exact form fails is kept as the file has it, untouched by the
// exact pass, and builds as it built before.

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <nlohmann/json.hpp>
#include <sstream>
#include <string>
#include <utility>

#include "../TimeLimits.h"
#include "horizon/document/Document.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/fileio/ImportReport.h"
#include "horizon/fileio/NativeFormat.h"
#include "horizon/fileio/StepFormat.h"
#include "horizon/math/Mat4.h"
#include "horizon/modeling/Faceting.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/modeling/Pattern.h"
#include "horizon/modeling/PrimitiveFactory.h"
#include "horizon/topology/Solid.h"

using hz::doc::Document;
using hz::doc::DocumentType;
using hz::doc::ImportedBodyFeature;
using hz::io::NativeFormat;
using json = nlohmann::json;
namespace fs = std::filesystem;

namespace {

/// The old part bodies' folder (a function, not a static path: constructing one can throw, and
/// nothing would catch it before main).
fs::path oldPartDir() {
    return fs::path(HZ_OLD_PART_DIR);
}

std::string readText(const fs::path& path) {
    std::ifstream in(path, std::ios::binary);
    std::ostringstream text;
    text << in.rdbuf();
    return text.str();
}

/// A part file holding one imported body from @p source whose STEP is @p step, as a part saved
/// before stage 2 holds it: a part saved today, its body's text replaced by the old one.
std::string oldPartFile(const std::string& step, const std::string& source) {
    Document part;
    part.setType(DocumentType::Part);
    part.featureTree().addFeature(std::make_unique<ImportedBodyFeature>(
        std::shared_ptr<const hz::topo::Solid>(hz::model::PrimitiveFactory::makeBox(1, 1, 1)),
        source));
    json root = json::parse(NativeFormat::documentToJson(part, false));
    for (json& feature : root.at("featureTree")) {
        if (feature.value("type", "") == "imported") feature["step"] = step;
    }
    return root.dump();
}

/// The Moon buggy body's faceted copy as master built it (54f72335): 12750 faces enclosing
/// 2799280.89787. A platform's last bits may put a band's grid a column either way where its width
/// over its count is a whole number but for rounding (11952 faces, 2799004.59, are seen when it is
/// moved by them): within the facets' own chordal error, and far from the hundreds of thousands of
/// faces a facet count that follows rounding gave (CI fix 2).
constexpr std::size_t kFewestFaces = 11500;
constexpr std::size_t kMostFaces = 13500;
constexpr double kVolume = 2799280.89787;
/// Relative: twice what a grid's column moves it by (9.9e-5); a face lost, or one too many, moves
/// it by far more.
constexpr double kVolumeWithin = 2e-4;

/// Seconds since @p start.
double secondsSince(std::chrono::steady_clock::time_point start) {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
}

/// Expects @p seconds under @p limit, a bound for an unoptimized GCC or Clang build, times
/// HZ_UNOPTIMIZED_SLOWDOWN (MSVC's Debug build); nothing under a sanitizer, where a limit means
/// nothing (tests/TimeLimits.h). @p what says what took that long, and what it takes in Debug.
void expectWithin(double seconds, double limit, const std::string& what) {
#if HZ_TIME_LIMITS
    EXPECT_LT(seconds, limit * HZ_UNOPTIMIZED_SLOWDOWN) << what;
#else
    (void)seconds;
    (void)limit;
    (void)what;
#endif
}

/// @p text with @p from replaced by @p to, which it must hold once.
std::string replaced(std::string text, const std::string& from, const std::string& to) {
    const auto at = text.find(from);
    EXPECT_NE(at, std::string::npos) << from;
    if (at != std::string::npos) text.replace(at, from.size(), to);
    return text;
}

/// @p step opened as a part file holding it as its one imported body from @p source; null where
/// it does not open.
const ImportedBodyFeature* openedText(Document& part, const std::string& step,
                                      const std::string& source) {
    std::string error;
    hz::io::ImportReport report;
    EXPECT_TRUE(NativeFormat::documentFromJson(oldPartFile(step, source), part, &error, &report))
        << error;
    if (part.featureTree().featureCount() != 1) return nullptr;
    return dynamic_cast<const ImportedBodyFeature*>(part.featureTree().feature(0));
}

/// The old part body @p name opened as openedText opens its text.
const ImportedBodyFeature* opened(Document& part, const std::string& name,
                                  const std::string& source) {
    const std::string step = readText(oldPartDir() / name);
    EXPECT_FALSE(step.empty()) << name;
    return openedText(part, step, source);
}

}  // namespace

// The corpus's Moon buggy (tests/interop/step/edm_ug_moon_buggy_asm.stp), its body 15 as a part
// file saved before stage 2 holds it: written by master's writer (54f72335), each face a B-spline
// patch, a ring of two bands (a torus's and a cylinder's) and two flats. Its exact pass refuses
// its B-spline torus band (face #149: its seam would cross where its surface closes). With its
// shell listing the cylinder band first, as a writer may, the pass lays that band's seam before
// it refuses the torus's. Either way the body is kept as the file has it, not as the failed pass
// left it: its 4 edges, none a seam or a piece of a split rim, and it builds as on master, 12750
// faces enclosing 2799280.9, in moments (CI fix 2: what a platform's last bits may move is
// bounded, not pinned). The corpus's Solid Edge body was kept so too, until its rims read the
// wrong way round were turned (below).
TEST(OldPartFile, ABodyKeptAsItWasIsTheFilesUntouchedByTheExactPass) {
    const std::string step = readText(oldPartDir() / "edm_ug_moon_buggy_asm.body15.step");
    ASSERT_FALSE(step.empty());
    const std::pair<const char*, std::string> texts[] = {
        {"as master wrote it", step},
        {"its cylinder band first", replaced(step, "#200 = CLOSED_SHELL('',(#149,#161,#187,#199));",
                                             "#200 = CLOSED_SHELL('',(#187,#149,#161,#199));")},
    };
    for (const auto& text : texts) {
        SCOPED_TRACE(text.first);
        Document part;
        const ImportedBodyFeature* imported =
            openedText(part, text.second, "edm_ug_moon_buggy_asm.stp");
        ASSERT_NE(imported, nullptr);
        ASSERT_NE(imported->solid(), nullptr);
        EXPECT_NE(imported->note().find("its seam would cross where its surface closes"),
                  std::string::npos)
            << imported->note();
        EXPECT_NE(imported->note().find("; kept as it was"), std::string::npos) << imported->note();

        const hz::topo::Solid& body = *imported->solid();
        EXPECT_EQ(body.edgeCount(), 4u) << "the file's edges, as it has them";
        for (const auto& e : body.edges()) {
            const std::string name = e.topoId.tag();
            EXPECT_EQ(name.find("/seam"), std::string::npos) << name;
            EXPECT_EQ(name.find("/piece:"), std::string::npos) << name;
        }

        const auto start = std::chrono::steady_clock::now();
        ASSERT_TRUE(part.rebuildModel()) << part.lastBuildMessage();
        // 0.42 s in Debug; the faceting's own seam gone wrong took 13.5 s, and 90 s on macOS.
        expectWithin(secondsSince(start), 5.0, "built in moments (0.42 s, Debug)");
        EXPECT_EQ(part.failedFeatureIndex(), -1) << part.lastBuildMessage();
        ASSERT_NE(part.solid(), nullptr);
        EXPECT_GE(part.solid()->faceCount(), kFewestFaces);
        EXPECT_LE(part.solid()->faceCount(), kMostFaces);
        EXPECT_NEAR(hz::model::MassPropertiesCalculator::compute(*part.solid()).volume, kVolume,
                    kVolumeWithin * kVolume);
        EXPECT_EQ(part.exactSolid(), nullptr) << "no exact form holds it";
    }
}

// The corpus's Solid Edge and Onshape parts, as part files saved before stage 2 hold them. Each
// was kept as it was ("its two loops wind round it the same way"): master's writer gave a closed
// rim's EDGE_CURVE same_sense .T. whichever way its carrier ran, and where the source's ran
// against the rim (Solid Edge's #288, #289 and #314, Onshape's #101, same_sense .F.), the rim
// reads back the wrong way round on both its faces. Those rims are turned before any band is
// seamed, and each body opens exact. Solid Edge's carriers are its own circles and lines, so its
// exact volume is its source's (OpenCASCADE's, in the manifest); Onshape's is the body's as master
// wrote it, 1.6e-6 from its source's. Each builds in moments, its faceted copy the size it is
// wherever its last bits fall.
TEST(OldPartFile, ABodyWhoseRimsReadTheWrongWayRoundOpensExact) {
    struct Case {
        const char* file;
        const char* source;
        double volume;
        std::size_t faces;  // its faceted copy's
        /// Its build's bound in Debug: about seven times what it takes (Solid Edge 0.17 s, Onshape
        /// 2.8 s, which meshes and integrates its counterpart), so a gcov build two or three times
        /// slower passes, and a build that runs away (an exact face's mesh or mass, or a faceted
        /// copy's grid, ten times over and more) does not.
        double seconds;
    };
    const Case cases[] = {
        {"solidedge_ercf_knife_holder.body.step", "solidedge_ercf_knife_holder.step", 684.683543,
         1652, 1.5},
        {"onshape_rocketsmith_upper_airframe.body.step", "onshape_rocketsmith_upper_airframe.step",
         712990.962654, 10229, 20.0},
    };
    for (const Case& c : cases) {
        SCOPED_TRACE(c.file);
        Document part;
        const ImportedBodyFeature* imported = opened(part, c.file, c.source);
        ASSERT_NE(imported, nullptr);
        EXPECT_EQ(imported->note(), "") << "exact, nothing to say";
        const auto start = std::chrono::steady_clock::now();
        ASSERT_TRUE(part.rebuildModel()) << part.lastBuildMessage();
        expectWithin(secondsSince(start), c.seconds, "built in moments");
        EXPECT_EQ(part.failedFeatureIndex(), -1) << part.lastBuildMessage();
        EXPECT_EQ(part.exactNote(), "");
        ASSERT_NE(part.exactSolid(), nullptr);
        EXPECT_NEAR(hz::model::MassPropertiesCalculator::compute(*part.exactSolid()).volume,
                    c.volume, 1e-9 * c.volume);
        ASSERT_NE(part.solid(), nullptr);
        // A tenth either way: a grid's column moved by a platform's last bits, not a blow-up.
        EXPECT_GE(part.solid()->faceCount(), c.faces - c.faces / 10);
        EXPECT_LE(part.solid()->faceCount(), c.faces + c.faces / 10);
    }
}

// CI fix 2 (macOS): a kept body's faceted copy is the same size wherever its last bits fall. The
// corpus's Solid Edge body, with one corner (#45) moved 0.05 mm off its edges so that no exact
// form holds it (its rims read the wrong way round are now turned, and it would open exact), is
// kept as the file has it, its band that no seam was laid on (face #1084) whole. That band is cut
// into facets along a seam of the faceting's own, straight up the band from one rim's point to
// the other's, at one u but for rounding. How many pieces that seam is cut into was measured by
// how far the surface turns along u across that span, at 256 points a hundredth of a last bit
// apart: their chords' directions were rounding, they turned hundreds of radians, and the seam
// was cut into hundreds of pieces, the band's facets spaced as finely. Moved by a millionth of a
// micrometre, as another platform's library rounds, the part built 287,864 faces in 90 s on
// macOS, and up to 254,044 here. Each faceted copy, moved by any of these, is the size its
// unmoved copy is (3869 faces, or 3797 where a band's grid falls a column the other way: 1.7e-5
// of its volume), and is cut in moments.
TEST(OldPartFile, ItsFacetedCopyIsTheSameSizeWhereverItsLastBitsFall) {
    const std::string step = replaced(
        readText(oldPartDir() / "solidedge_ercf_knife_holder.body.step"),
        "#45 = CARTESIAN_POINT('',(3.5,15.9,2.65));", "#45 = CARTESIAN_POINT('',(3.5,15.9,2.7));");
    hz::io::StepReadOptions asAPartFile;
    asAPartFile.unknownLengthUnit = hz::math::LengthUnit::Millimetre;
    asAPartFile.keepWhenNotExact = true;
    hz::io::ImportReport report;
    const auto read = hz::io::StepFormat::fromString(step, &report, nullptr, asAPartFile);
    ASSERT_EQ(read.size(), 1u) << hz::io::StepFormat::lastError();
    ASSERT_EQ(report.converted.size(), 1u);
    EXPECT_NE(report.converted.front().find("its curve ends 0.05 from its vertex"),
              std::string::npos)
        << report.converted.front();
    EXPECT_NE(report.converted.front().find("; kept as it was"), std::string::npos)
        << report.converted.front();
    // Its faceted copy's size: as the original test bounded master's 3868 faces, a column of a
    // band's grid either way, and twice what one moves its volume by.
    constexpr std::size_t kFewest = 3700;
    constexpr std::size_t kMost = 4000;
    constexpr double kKeptVolume = 684.504274;
    constexpr double kKeptWithin = 4e-5;
    for (int k = 0; k < 20; ++k) {
        // 2^-40 to 2^-49 mm, either way: from about four hundred last bits of its coordinates
        // down to one.
        const double d = std::ldexp(k < 10 ? 1.0 : -1.0, -40 - k % 10);
        const auto moved = hz::model::Pattern::transformed(
            *read.front(), hz::math::Mat4::translation(hz::math::Vec3(3.0 * d, -5.0 * d, 7.0 * d)));
        const auto start = std::chrono::steady_clock::now();
        const auto faceted = hz::model::facetCurved(*moved);
        const double seconds = secondsSince(start);
        ASSERT_NE(faceted.solid, nullptr) << "moved by " << d << ": " << faceted.error;
        EXPECT_GE(faceted.solid->faceCount(), kFewest) << "moved by " << d;
        EXPECT_LE(faceted.solid->faceCount(), kMost) << "moved by " << d;
        // 0.16 s in Debug; the regression this guards took up to 13.5 s here, 90 s on macOS.
        expectWithin(seconds, 5.0, "moved by " + std::to_string(d) + " (0.16 s, Debug)");
        EXPECT_NEAR(hz::model::MassPropertiesCalculator::compute(*faceted.solid).volume,
                    kKeptVolume, kKeptWithin * kKeptVolume)
            << "moved by " << d;
    }
}
