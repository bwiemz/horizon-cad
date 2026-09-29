// Reads every file of the external STEP corpus as Horizon CAD would, and
// writes what it found for tools/interop/occt_check.py to compare with
// OpenCASCADE (see tests/interop/step/README.md):
//
//   hz_step_acceptance <corpus dir> <work dir>
//
// It writes <work dir>/summary.json, and under it what it kept and exported.
// It judges nothing: the acceptance test and the check do.

#include <cstdio>
#include <exception>
#include <filesystem>
#include <fstream>

#include "StepAcceptance.h"

namespace fs = std::filesystem;

int main(int argc, char* argv[]) {
    if (argc != 3) {
        std::fprintf(stderr, "usage: hz_step_acceptance <corpus dir> <work dir>\n");
        return 2;
    }
    try {
        const fs::path corpus = fs::absolute(argv[1]);
        const fs::path work = fs::absolute(argv[2]);
        fs::create_directories(work);
        nlohmann::json files = nlohmann::json::array();
        for (const auto& entry : hz::interop::readManifest(corpus / "manifest.json")) {
            const auto summary = hz::interop::summarize(
                corpus / entry.file, work / fs::path(entry.file).stem(), entry.unitIfUnnamed);
            std::printf("%-44s %s\n", entry.file.c_str(),
                        summary.error.empty() ? (std::to_string(summary.bodies) + " bodies, " +
                                                 std::to_string(summary.idealVolume) + " mm3")
                                                    .c_str()
                                              : ("not read: " + summary.error).c_str());
            files.push_back(hz::interop::toJson(summary));
        }
        std::ofstream(work / "summary.json") << nlohmann::json{{"files", files}}.dump(2) << "\n";
        std::printf("wrote %s\n", (work / "summary.json").string().c_str());
        return 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "hz_step_acceptance: %s\n", e.what());
        return 1;
    }
}
