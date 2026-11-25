// The external-file acceptance suite (tests/interop/step/README.md): every
// STEP file in the corpus, each exported by another CAD system, is read as
// its manifest says, and comes through Horizon CAD unchanged: kept as a part
// and read back, kept as an assembly of part files and read back, and sent
// out as STEP again and read back.
//
// The manifest's expected bodies, volumes and bounds are OpenCASCADE's
// reading of the same file (tools/interop/occt_check.py), not Horizon CAD's
// own: a reader agreeing with itself proves nothing.

#include <gtest/gtest.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <ostream>
#include <random>
#include <set>
#include <sstream>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include "StepAcceptance.h"
#include "horizon/fileio/StepFormat.h"
#include "horizon/modeling/PrimitiveFactory.h"

using hz::interop::CorpusEntry;
using hz::interop::SolidReading;
using hz::interop::StepSummary;
using json = nlohmann::json;
namespace fs = std::filesystem;

namespace {

/// The corpus's folder (a function, not a static path: constructing one
/// can throw, and nothing would catch it before main).
fs::path corpusDir() {
    return fs::path(HZ_STEP_CORPUS_DIR);
}

const std::vector<CorpusEntry>& corpus() {
    static const std::vector<CorpusEntry> entries =
        hz::interop::readManifest(corpusDir() / "manifest.json");
    return entries;
}

/// A folder of the test's own, removed after it. CTest runs each case in a
/// process of its own, in parallel: the name is random, not a count or a
/// time, which two processes can share.
class WorkDir {
public:
    WorkDir() {
        std::random_device random;
        m_path = fs::temp_directory_path() /
                 ("hz_interop_" + std::to_string(random()) + "_" + std::to_string(random()));
        fs::create_directories(m_path);
    }
    ~WorkDir() {
        std::error_code ec;
        fs::remove_all(m_path, ec);
    }
    WorkDir(const WorkDir&) = delete;
    WorkDir& operator=(const WorkDir&) = delete;
    const fs::path& path() const { return m_path; }

private:
    fs::path m_path;
};

std::string testName(const ::testing::TestParamInfo<CorpusEntry>& info) {
    std::string name = fs::path(info.param.file).stem().string();
    std::replace_if(
        name.begin(), name.end(),
        [](char c) { return std::isalnum(static_cast<unsigned char>(c)) == 0; }, '_');
    return name;
}

/// Whether the manifest lists @p check among what the file still fails.
bool knownGap(const json& manifest, const char* check) {
    return manifest.contains("known_gaps") && manifest["known_gaps"].contains(check);
}

bool near(double value, double expected, double relative) {
    return std::abs(value - expected) <= relative * std::max(std::abs(expected), 1e-9);
}

}  // namespace

namespace hz::interop {
/// A test's name for gtest's messages: its file's.
void PrintTo(const CorpusEntry& entry, std::ostream* out) {
    *out << entry.file;
}
}  // namespace hz::interop

class StepCorpus : public ::testing::TestWithParam<CorpusEntry> {};

TEST_P(StepCorpus, IsReadAsItsManifestSaysAndComesThroughUnchanged) {
    const CorpusEntry& entry = GetParam();
    const json& manifest = entry.manifest;
    WorkDir work;
    const StepSummary s =
        hz::interop::summarize(corpusDir() / entry.file, work.path(), entry.unitIfUnnamed);

    // Every solid read exact, or on the regression list (exact_failures) with why; and each
    // exact solid's read-point mesh measuring its exact volume, within the mesh's allowance.
    for (const std::string& problem : hz::interop::checkExactFailures(s, manifest)) {
        ADD_FAILURE() << problem;
    }
    for (const std::string& problem : hz::interop::checkMeshAgainstMass(s)) {
        ADD_FAILURE() << problem;
    }

    // A known gap: it is still not read, and says why. Once it is, the
    // manifest says what it gives instead.
    if (manifest.contains("not_read")) {
        EXPECT_FALSE(s.error.empty()) << "it is read now: replace not_read with what it gives";
        EXPECT_NE(s.error.find(manifest["not_read"].get<std::string>()), std::string::npos)
            << "it fails for another reason: " << s.error;
        return;
    }
    // Not read because no solid of it is exact: the regression list, above, says whether that
    // is known. There is nothing more to check until it reads.
    if (!s.error.empty() && !s.solids.empty()) return;
    ASSERT_TRUE(s.error.empty()) << "not read: " << s.error;

    // As OpenCASCADE reads it. A check the manifest lists as a known gap
    // must still fail: once it passes, the manifest says so.
    const json& expect = manifest.at("expect");
    const auto check = [&](const char* name, bool passes, const std::string& found) {
        if (knownGap(manifest, name)) {
            EXPECT_FALSE(passes) << name << " is as OpenCASCADE reads it now: take it out of "
                                 << "known_gaps (" << found << ")";
        } else {
            EXPECT_TRUE(passes) << name << ": " << found;
        }
    };
    for (const char* count : {"bodies", "parts", "occurrences"}) {
        if (!expect.contains(count)) continue;
        const std::size_t got = std::string(count) == "bodies"  ? s.bodies
                                : std::string(count) == "parts" ? s.parts
                                                                : s.occurrences;
        const auto want = expect[count].get<std::size_t>();
        check(count, got == want, std::to_string(got) + ", not " + std::to_string(want));
    }
    // Its exact volume, integrated on its exact faces, OpenCASCADE's to a millionth unless the
    // manifest says how near.
    if (expect.contains("volume")) {
        const double volume = expect["volume"].get<double>();
        const double tolerance = expect.value("volume_tolerance", 1e-6);
        const double exact = hz::interop::exactVolume(s);
        std::ostringstream found;
        found.precision(12);
        found << exact << " mm³ exact, not " << volume << " within " << tolerance << " (relative "
              << (exact - volume) / volume << ")";
        check("volume", near(exact, volume, tolerance), found.str());
    }
    if (expect.contains("bounds")) {
        ASSERT_FALSE(s.bounds.empty);
        const json& bounds = expect["bounds"];
        const double diagonal = std::hypot(bounds[1][0].get<double>() - bounds[0][0].get<double>(),
                                           bounds[1][1].get<double>() - bounds[0][1].get<double>(),
                                           bounds[1][2].get<double>() - bounds[0][2].get<double>());
        const double tolerance = expect.value("bounds_tolerance", 0.005) * diagonal;
        const hz::math::Vec3 got[2] = {s.bounds.min, s.bounds.max};
        double worst = 0.0;
        for (std::size_t corner = 0; corner < 2; ++corner) {
            const double axes[3] = {got[corner].x, got[corner].y, got[corner].z};
            for (std::size_t axis = 0; axis < 3; ++axis) {
                worst = std::max(worst, std::abs(axes[axis] - bounds[corner][axis].get<double>()));
            }
        }
        check("bounds", worst <= tolerance,
              "a corner " + std::to_string(worst) + " mm off, over " + std::to_string(tolerance));
    }
    // What it reports leaving out or approximating, as many as the manifest
    // says: a new item, or one gone, is a change to look at.
    if (expect.contains("skipped")) {
        EXPECT_EQ(s.skipped.size(), expect["skipped"].get<std::size_t>())
            << "skipped: " << json(s.skipped).dump(1);
    }
    if (expect.contains("approximated")) {
        EXPECT_EQ(s.approximated.size(), expect["approximated"].get<std::size_t>())
            << "approximated: " << json(s.approximated).dump(1);
    }
    // Its curved faces the export keeps in facets, as many as the manifest
    // says: one more is a face that went out whole before.
    if (expect.contains("export_faceted")) {
        EXPECT_EQ(s.exportFaceted.size(), expect["export_faceted"].get<std::size_t>())
            << "kept in facets when sent out: " << json(s.exportFaceted).dump(1);
    }
    if (expect.value("valid", true)) {
        EXPECT_TRUE(s.allValid);
    }

    // Through Horizon CAD, whatever the file. A known gap must still fail:
    // once it passes, the manifest says so.
    if (knownGap(manifest, "import-build")) {
        EXPECT_FALSE(s.importBuildError.empty())
            << "it builds now: take import-build out of known_gaps";
    } else {
        EXPECT_TRUE(s.importBuildError.empty())
            << "imported, it does not build: " << s.importBuildError;
    }
    if (knownGap(manifest, "reopen")) {
        EXPECT_FALSE(s.reopenedVolume.has_value())
            << "it reopens now: take reopen out of known_gaps";
    } else {
        // Measured as the part is shown, measured and exported: its exact counterpart. A part
        // file keeps an imported body as STEP, so its exact faces are read again from the text
        // written, as an export's are, and to the same millionth.
        ASSERT_TRUE(s.reopenedVolume.has_value()) << "kept as a part: " << s.reopenError;
        const double reopened = s.reopenedVolume.value_or(0.0);
        EXPECT_TRUE(near(reopened, s.facetedVolume, 1e-6))
            << "kept as a part and read back: " << reopened << " mm³, not " << s.facetedVolume;
    }
    if (s.parts > 0) {
        ASSERT_TRUE(s.keptComponents.has_value()) << "kept as an assembly: " << s.keepError;
        EXPECT_EQ(s.keptComponents.value_or(0), s.occurrences)
            << "kept as an assembly and read back";
    }
    // Sent out exact (its exact counterpart, its faces as their kinds), it reads back as the
    // bodies read, to a millionth.
    ASSERT_TRUE(s.reimportedBodies.has_value()) << "sent out as STEP: " << s.exportError;
    EXPECT_EQ(s.reimportedBodies.value_or(0), s.bodies) << "sent out as STEP and read back";
    EXPECT_TRUE(near(s.reimportedVolume, s.facetedVolume, 1e-6))
        << "sent out as STEP and read back: " << s.reimportedVolume << " mm³, not "
        << s.facetedVolume;
}

INSTANTIATE_TEST_SUITE_P(Corpus, StepCorpus, ::testing::ValuesIn(corpus()), testName);

// Every file in the corpus is in its manifest, with where it came from and
// under what licence: a file whose provenance is not recorded cannot stay.
TEST(StepCorpusManifest, EveryFileIsListedWithItsProvenance) {
    std::set<std::string> listed;
    for (const CorpusEntry& entry : corpus()) {
        SCOPED_TRACE(entry.file);
        EXPECT_TRUE(listed.insert(entry.file).second) << "listed twice";
        EXPECT_TRUE(fs::exists(corpusDir() / entry.file)) << "not in the corpus";
        for (const char* field : {"source", "license", "exporter", "schema", "content"}) {
            EXPECT_TRUE(entry.manifest.contains(field) &&
                        !entry.manifest[field].get<std::string>().empty())
                << "no " << field;
        }
        EXPECT_TRUE(entry.manifest.contains("expect") != entry.manifest.contains("not_read"))
            << "not one of: what it gives, or why it is not read";
        if (entry.manifest.contains("known_gaps")) {
            for (const auto& [check, why] : entry.manifest["known_gaps"].items()) {
                EXPECT_FALSE(why.get<std::string>().empty()) << check << ": no why";
            }
        }
        // The regression list: each solid with why, as the read names and says them.
        if (entry.manifest.contains("exact_failures")) {
            ASSERT_TRUE(entry.manifest["exact_failures"].is_array());
            for (const json& failure : entry.manifest["exact_failures"]) {
                for (const char* field : {"solid", "why"}) {
                    EXPECT_TRUE(failure.contains(field) && failure[field].is_string() &&
                                !failure[field].get<std::string>().empty())
                        << "an exact failure with no " << field << ": " << failure.dump();
                }
            }
        }
    }
    for (const auto& file : fs::directory_iterator(corpusDir())) {
        const std::string ext = file.path().extension().string();
        if (ext == ".step" || ext == ".stp" || ext == ".STEP" || ext == ".STP") {
            EXPECT_EQ(listed.count(file.path().filename().string()), 1u)
                << file.path().filename() << " is not in the manifest";
        }
    }
}

// --- The regression list and the read point's mesh, on summaries made here ---

namespace {

/// A solid read exact: @p volume mm³, its read-point mesh @p mesh mm³, allowed @p allowance.
SolidReading exactSolid(const std::string& name, double volume, double mesh, double allowance) {
    SolidReading solid;
    solid.solid = name;
    solid.exact = true;
    solid.exactVolume = volume;
    solid.meshVolume = mesh;
    solid.meshAllowance = allowance;
    return solid;
}

/// A solid the read left out for want of its exact form, with why.
SolidReading leftOut(const std::string& name, const std::string& why) {
    SolidReading solid;
    solid.solid = name;
    solid.exactProblem = why;
    return solid;
}

/// A manifest entry whose regression list names each of @p failures (solid, why).
json listing(const std::vector<std::pair<std::string, std::string>>& failures) {
    json list = json::array();
    for (const auto& [solid, why] : failures) list.push_back({{"solid", solid}, {"why", why}});
    return {{"exact_failures", list}};
}

bool says(const std::string& text, const std::string& part) {
    return text.find(part) != std::string::npos;
}

const std::string kTorus =
    "its curve stands 3.09 off its face's surface, beyond the limit of 0.0508 (a thousandth of "
    "the solid's size) (face #709, edge #426)";

}  // namespace

// A solid the list names that the read makes exact now is an error, as a known gap that closes
// is: the list is to say so.
TEST(StepCorpusRegressionList, AListedFailureThatReadsExactNowIsAnError) {
    StepSummary s;
    s.solids.push_back(exactSolid("solid 1", 3788.27, 3788.27, 1.0));
    const auto problems =
        hz::interop::checkExactFailures(s, listing({{"solid #12", "its curve stands 3.09 off"}}));
    ASSERT_EQ(problems.size(), 1u);
    EXPECT_TRUE(says(problems[0], "solid #12")) << problems[0];
    EXPECT_TRUE(says(problems[0], "take it out of exact_failures")) << problems[0];
}

TEST(StepCorpusRegressionList, AListedFailureThatStillFailsIsNoError) {
    StepSummary s;
    s.solids.push_back(leftOut("solid #12", kTorus));
    EXPECT_TRUE(
        hz::interop::checkExactFailures(s, listing({{"solid #12", "its curve stands 3.09 off"}}))
            .empty());
}

// A solid left out that the list does not name is a regression, with why.
TEST(StepCorpusRegressionList, AnUnlistedFailureIsAnError) {
    StepSummary s;
    s.solids.push_back(exactSolid("solid 1", 10.0, 10.0, 1.0));
    s.solids.push_back(leftOut("solid 2", kTorus));
    const auto problems = hz::interop::checkExactFailures(s, json::object());
    ASSERT_EQ(problems.size(), 1u);
    EXPECT_TRUE(says(problems[0], "solid 2")) << problems[0];
    EXPECT_TRUE(says(problems[0], kTorus)) << problems[0];
}

// The list holds why as well as which: the same solid failing another way is a change to look
// at, the new why and the listed one both said.
TEST(StepCorpusRegressionList, AListedFailureThatFailsForAnotherReasonIsAnError) {
    StepSummary s;
    s.solids.push_back(leftOut("solid #12", "its curve ends 0.374 from its vertex"));
    const auto problems =
        hz::interop::checkExactFailures(s, listing({{"solid #12", "its curve stands 3.09 off"}}));
    ASSERT_EQ(problems.size(), 1u);
    EXPECT_TRUE(says(problems[0], "its curve ends 0.374 from its vertex")) << problems[0];
    EXPECT_TRUE(says(problems[0], "its curve stands 3.09 off")) << problems[0];
}

// A solid read exact whose exact volume cannot be integrated is a failure of the exact path
// too: on the list, or an error.
TEST(StepCorpusRegressionList, AnExactSolidWhoseVolumeIsNotIntegratedIsAFailure) {
    StepSummary s;
    SolidReading solid = exactSolid("solid 3", 0.0, 0.0, 0.0);
    solid.exactProblem = "its exact volume could not be integrated: face 7: no pcurve";
    s.solids.push_back(solid);
    EXPECT_EQ(hz::interop::checkExactFailures(s, json::object()).size(), 1u);
    EXPECT_TRUE(
        hz::interop::checkExactFailures(s, listing({{"solid 3", "face 7: no pcurve"}})).empty());
}

// Each exact solid's read-point mesh measures its exact volume to within its area times the
// mesh's chord tolerance; nearer is not required, further is an error.
TEST(StepCorpusMeshAgainstMass, AMeshWithinItsAllowanceAgrees) {
    StepSummary s;
    s.solids.push_back(exactSolid("solid 1", 1000.0, 1000.9, 1.0));
    s.solids.push_back(exactSolid("solid 2", 1000.0, 999.1, 1.0));
    EXPECT_TRUE(hz::interop::checkMeshAgainstMass(s).empty());
}

TEST(StepCorpusMeshAgainstMass, AMeshBeyondItsAllowanceIsAnError) {
    StepSummary s;
    s.solids.push_back(exactSolid("solid 1", 1000.0, 1000.5, 1.0));
    s.solids.push_back(exactSolid("solid 2", 1000.0, 998.5, 1.0));
    const auto problems = hz::interop::checkMeshAgainstMass(s);
    ASSERT_EQ(problems.size(), 1u);
    EXPECT_TRUE(says(problems[0], "solid 2")) << problems[0];
}

TEST(StepCorpusMeshAgainstMass, AMeshThatCouldNotBeMadeIsAnError) {
    StepSummary s;
    SolidReading solid = exactSolid("solid 1", 1000.0, 0.0, 1.0);
    solid.meshProblem = "step/solid:0/f:32040: a loop jumps in (u, v)";
    s.solids.push_back(solid);
    const auto problems = hz::interop::checkMeshAgainstMass(s);
    ASSERT_EQ(problems.size(), 1u);
    EXPECT_TRUE(says(problems[0], solid.meshProblem)) << problems[0];
}

// A solid left out has no mesh to measure: the regression list speaks for it.
TEST(StepCorpusMeshAgainstMass, ASolidLeftOutHasNoMeshToCompare) {
    StepSummary s;
    s.solids.push_back(leftOut("solid #12", kTorus));
    EXPECT_TRUE(hz::interop::checkMeshAgainstMass(s).empty());
}

namespace {

/// @p text written to @p dir as @p name: its path.
fs::path written(const fs::path& dir, const std::string& name, const std::string& text) {
    const fs::path path = dir / name;
    std::ofstream(path, std::ios::binary) << text;
    return path;
}

/// A 7 x 11 x 13 box written to STEP with its corner at (7, 11, 13) raised 0.2, off the planes
/// of its faces: a solid the read cannot make exact (it says "beyond the limit"). With
/// @p alsoGood, a 2 x 3 x 5 box beside it, written after it, which it can.
std::string stepWithACornerOff(bool alsoGood) {
    const auto off = hz::model::PrimitiveFactory::makeBox(7.0, 11.0, 13.0);
    const auto good = hz::model::PrimitiveFactory::makeBox(2.0, 3.0, 5.0);
    std::vector<const hz::topo::Solid*> solids{off.get()};
    if (alsoGood) solids.push_back(good.get());
    std::string text = hz::io::StepFormat::toString(solids);
    const std::string corner = "(7.,11.,13.)";
    const auto at = text.find(corner);
    if (at != std::string::npos) text.replace(at, corner.size(), "(7.,11.,13.2)");
    return text;
}

}  // namespace

// A file some of whose solids are left out for want of their exact form: the summary names
// each solid as the read does, the one left out with why, the one read with its exact volume and
// its read-point mesh, which agrees with it.
TEST(StepCorpusSummary, ASolidLeftOutForItsExactFormIsNamedWithWhy) {
    WorkDir work;
    const fs::path step = written(work.path(), "two.step", stepWithACornerOff(true));
    const StepSummary s = hz::interop::summarize(step, work.path() / "work");
    ASSERT_TRUE(s.error.empty()) << s.error;
    ASSERT_EQ(s.solids.size(), 2u);
    const SolidReading& read = s.solids[0];
    EXPECT_EQ(read.solid, "solid 2");
    EXPECT_TRUE(read.exact);
    EXPECT_TRUE(read.exactProblem.empty()) << read.exactProblem;
    EXPECT_NEAR(read.exactVolume, 30.0, 1e-9);
    EXPECT_NEAR(hz::interop::exactVolume(s), 30.0, 1e-9);
    EXPECT_TRUE(read.meshProblem.empty()) << read.meshProblem;
    EXPECT_GT(read.meshAllowance, 0.0);
    EXPECT_NEAR(read.meshVolume, read.exactVolume, read.meshAllowance);
    EXPECT_TRUE(hz::interop::checkMeshAgainstMass(s).empty());
    const SolidReading& out = s.solids[1];
    EXPECT_EQ(out.solid, "solid 1");
    EXPECT_FALSE(out.exact);
    EXPECT_TRUE(says(out.exactProblem, "beyond the limit")) << out.exactProblem;
    // Kept as a part and read back, measured as the part is, and its faceted copy.
    ASSERT_TRUE(s.reopenedVolume.has_value()) << s.reopenError;
    EXPECT_NEAR(s.reopenedVolume.value_or(0.0), 30.0, 1e-9);
    ASSERT_TRUE(s.reopenedFacetedVolume.has_value());
    EXPECT_NEAR(s.reopenedFacetedVolume.value_or(0.0), 30.0, 1e-9);
    // The regression list names it, or the summary disagrees with it.
    EXPECT_EQ(hz::interop::checkExactFailures(s, json::object()).size(), 1u);
    EXPECT_TRUE(
        hz::interop::checkExactFailures(s, listing({{"solid 1", "beyond the limit"}})).empty());
}

// A file none of whose solids is read: the solid its error names, left out with why.
TEST(StepCorpusSummary, AFileNoSolidOfWhichIsExactNamesTheOneItsErrorNames) {
    WorkDir work;
    const fs::path step = written(work.path(), "one.step", stepWithACornerOff(false));
    const StepSummary s = hz::interop::summarize(step, work.path() / "work");
    ASSERT_FALSE(s.error.empty());
    ASSERT_EQ(s.solids.size(), 1u);
    const SolidReading& out = s.solids[0];
    EXPECT_EQ(out.solid.rfind("solid #", 0), 0u) << out.solid;
    EXPECT_FALSE(out.exact);
    EXPECT_EQ(s.error, out.solid + " could not be made exact: " + out.exactProblem);
    const json json = hz::interop::toJson(s);
    ASSERT_EQ(json.at("solids").size(), 1u);
    EXPECT_EQ(json["solids"][0].at("solid"), out.solid);
    EXPECT_EQ(json["solids"][0].at("exact"), false);
    EXPECT_EQ(json["solids"][0].at("exactProblem"), out.exactProblem);
}
