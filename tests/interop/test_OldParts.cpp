// Part files saved before the exact kernel (stage 2, Ruling 10): each holds its imported bodies as
// STEP the writer of its day wrote, which no exact form may hold. Such a part opens as it did: a
// body whose exact form fails is kept as the file has it, untouched by the exact pass, and builds
// as it built before.

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <nlohmann/json.hpp>
#include <sstream>
#include <string>

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

/// The Solid Edge body's faceted copy as master built it (54f72335): 3868 faces enclosing
/// 684.443891. A platform's last bits may put a band's grid a column either way where its width
/// over its count is a whole number but for rounding (3796 faces, 684.4555, were seen): within the
/// facets' own chordal error, and far from the hundreds of thousands of faces a facet count that
/// follows rounding gave.
constexpr std::size_t kFewestFaces = 3700;
constexpr std::size_t kMostFaces = 4000;
constexpr double kVolume = 684.443891;
/// Relative: twice what a grid's column moved it by (1.7e-5); a face lost, or one too many, moves
/// it by far more.
constexpr double kVolumeWithin = 4e-5;

/// Seconds since @p start.
double secondsSince(std::chrono::steady_clock::time_point start) {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
}

}  // namespace

// The corpus's Solid Edge part (tests/interop/step/solidedge_ercf_knife_holder.step), as a part
// file saved before stage 2 holds it: written by master's writer (54f72335), each face a B-spline
// patch. Its exact pass inserts a seam on one band and splits two rims, then refuses the next band
// (its two loops wind round it the same way: counted, not measured, so on every platform); the
// body is kept as the file has it, not as the failed pass left it: its 90 edges, none a seam or a
// piece of a split rim, and it builds as on master, 3868 faces enclosing 684.444, in moments (CI
// fix 2: what a platform's last bits may move is bounded, not pinned).
TEST(OldPartFile, ABodyKeptAsItWasIsTheFilesUntouchedByTheExactPass) {
    const std::string step = readText(oldPartDir() / "solidedge_ercf_knife_holder.body.step");
    ASSERT_FALSE(step.empty());

    Document opened;
    std::string error;
    hz::io::ImportReport report;
    ASSERT_TRUE(NativeFormat::documentFromJson(
        oldPartFile(step, "solidedge_ercf_knife_holder.step"), opened, &error, &report))
        << error;
    ASSERT_EQ(opened.featureTree().featureCount(), 1u);
    const auto* imported =
        dynamic_cast<const ImportedBodyFeature*>(opened.featureTree().feature(0));
    ASSERT_NE(imported, nullptr);
    ASSERT_NE(imported->solid(), nullptr);
    EXPECT_NE(imported->note().find("; kept as it was"), std::string::npos) << imported->note();

    const hz::topo::Solid& body = *imported->solid();
    EXPECT_EQ(body.edgeCount(), 90u) << "the file's edges, as it has them";
    for (const auto& e : body.edges()) {
        const std::string name = e.topoId.tag();
        EXPECT_EQ(name.find("/seam"), std::string::npos) << name;
        EXPECT_EQ(name.find("/piece:"), std::string::npos) << name;
    }

    const auto start = std::chrono::steady_clock::now();
    ASSERT_TRUE(opened.rebuildModel()) << opened.lastBuildMessage();
    EXPECT_LT(secondsSince(start), 10.0) << "built in moments (0.2 s, Debug)";
    EXPECT_EQ(opened.failedFeatureIndex(), -1) << opened.lastBuildMessage();
    ASSERT_NE(opened.solid(), nullptr);
    EXPECT_GE(opened.solid()->faceCount(), kFewestFaces);
    EXPECT_LE(opened.solid()->faceCount(), kMostFaces);
    EXPECT_NEAR(hz::model::MassPropertiesCalculator::compute(*opened.solid()).volume, kVolume,
                kVolumeWithin * kVolume);
    EXPECT_EQ(opened.exactSolid(), nullptr) << "no exact form holds it";
}

// CI fix 2 (macOS): the body's faceted copy is the same size wherever its last bits fall. Its band
// that no seam could be laid on (face #1084) is cut into facets along a seam of the faceting's own,
// straight up the band from one rim's point to the other's, at one u but for rounding. How many
// pieces that seam is cut into was measured by how far the surface turns along u across that span,
// at 256 points a hundredth of a last bit apart: their chords' directions were rounding, they
// turned hundreds of radians, and the seam was cut into hundreds of pieces, the band's facets
// spaced as finely. Moved by a millionth of a micrometre, as another platform's library rounds,
// the part built 287,864 faces in 90 s on macOS, and up to 254,044 here. Each faceted copy, moved
// by any of these, is the size master's is, and is cut in moments.
TEST(OldPartFile, ItsFacetedCopyIsTheSameSizeWhereverItsLastBitsFall) {
    const std::string step = readText(oldPartDir() / "solidedge_ercf_knife_holder.body.step");
    ASSERT_FALSE(step.empty());
    hz::io::StepReadOptions asAPartFile;
    asAPartFile.unknownLengthUnit = hz::math::LengthUnit::Millimetre;
    asAPartFile.keepWhenNotExact = true;
    const auto read = hz::io::StepFormat::fromString(step, nullptr, nullptr, asAPartFile);
    ASSERT_EQ(read.size(), 1u) << hz::io::StepFormat::lastError();
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
        EXPECT_GE(faceted.solid->faceCount(), kFewestFaces) << "moved by " << d;
        EXPECT_LE(faceted.solid->faceCount(), kMostFaces) << "moved by " << d;
        EXPECT_LT(seconds, 5.0) << "moved by " << d << " (0.15 s, Debug)";
        EXPECT_NEAR(hz::model::MassPropertiesCalculator::compute(*faceted.solid).volume, kVolume,
                    kVolumeWithin * kVolume)
            << "moved by " << d;
    }
}
