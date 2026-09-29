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
#include <ostream>
#include <random>
#include <set>
#include <sstream>
#include <string>
#include <system_error>
#include <vector>

#include "StepAcceptance.h"

using hz::interop::CorpusEntry;
using hz::interop::StepSummary;
using json = nlohmann::json;
namespace fs = std::filesystem;

namespace {

const fs::path kCorpus{HZ_STEP_CORPUS_DIR};

const std::vector<CorpusEntry>& corpus() {
    static const std::vector<CorpusEntry> entries =
        hz::interop::readManifest(kCorpus / "manifest.json");
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
        hz::interop::summarize(kCorpus / entry.file, work.path(), entry.unitIfUnnamed);

    // A known gap: it is still not read, and says why. Once it is, the
    // manifest says what it gives instead.
    if (manifest.contains("not_read")) {
        EXPECT_FALSE(s.error.empty()) << "it is read now: replace not_read with what it gives";
        EXPECT_NE(s.error.find(manifest["not_read"].get<std::string>()), std::string::npos)
            << "it fails for another reason: " << s.error;
        return;
    }
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
    if (expect.contains("volume")) {
        const double volume = expect["volume"].get<double>();
        const double tolerance = expect.value("volume_tolerance", 1e-3);
        std::ostringstream found;
        found << s.idealVolume << " mm³ (faceted " << s.facetedVolume << "), not " << volume
              << " within " << tolerance * 100 << "%";
        check("volume", near(s.idealVolume, volume, tolerance), found.str());
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
        for (int corner = 0; corner < 2; ++corner) {
            const double axes[3] = {got[corner].x, got[corner].y, got[corner].z};
            for (int axis = 0; axis < 3; ++axis) {
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
        ASSERT_TRUE(s.reopenedVolume.has_value()) << "kept as a part: " << s.reopenError;
        EXPECT_TRUE(near(*s.reopenedVolume, s.facetedVolume, 1e-9))
            << "kept as a part and read back: " << *s.reopenedVolume << " mm³, not "
            << s.facetedVolume;
    }
    if (s.parts > 0) {
        ASSERT_TRUE(s.keptComponents.has_value()) << "kept as an assembly: " << s.keepError;
        EXPECT_EQ(*s.keptComponents, s.occurrences) << "kept as an assembly and read back";
    }
    ASSERT_TRUE(s.reimportedBodies.has_value()) << "sent out as STEP: " << s.exportError;
    EXPECT_EQ(*s.reimportedBodies, s.bodies) << "sent out as STEP and read back";
    EXPECT_TRUE(near(s.reimportedVolume, s.facetedVolume, 2e-3))
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
        EXPECT_TRUE(fs::exists(kCorpus / entry.file)) << "not in the corpus";
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
    }
    for (const auto& file : fs::directory_iterator(kCorpus)) {
        const std::string ext = file.path().extension().string();
        if (ext == ".step" || ext == ".stp" || ext == ".STEP" || ext == ".STP") {
            EXPECT_EQ(listed.count(file.path().filename().string()), 1u)
                << file.path().filename() << " is not in the manifest";
        }
    }
}
