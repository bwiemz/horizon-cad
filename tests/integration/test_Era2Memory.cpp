// Era-2 stabilization guard (roadmap §5.12): a 100-part assembly resolved in
// lightweight mode must stay far under the 2 GB memory bound, and lightweight
// resolution must never fall back to full part loads.
//
// Parts are stored in the Phase-51 binary format: its typed tessellation
// cache is the mechanism that makes lightweight loads cheap (no JSON parse),
// so this guard doubles as the integration test binding BinaryFormat to the
// Phase-41 lightweight-assembly architecture.

#include <gtest/gtest.h>

#include <cstddef>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "horizon/document/Document.h"
#include "horizon/document/DocumentManager.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/document/Sketch.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/fileio/BinaryFormat.h"

#if defined(_WIN32)
#define NOMINMAX
#include <windows.h>
// clang-format off
#include <psapi.h>
// clang-format on
#else
#include <fstream>
#endif

using namespace hz::doc;
using hz::io::BinaryFormat;
using hz::math::Vec2;
using hz::math::Vec3;

namespace fs = std::filesystem;

namespace {

/// Resident set size of this process, or 0 when unavailable.
size_t currentRssBytes() {
#if defined(_WIN32)
    PROCESS_MEMORY_COUNTERS pmc{};
    if (GetProcessMemoryInfo(GetCurrentProcess(), &pmc, sizeof(pmc)) == 0) return 0;
    return pmc.WorkingSetSize;
#else
    std::ifstream status("/proc/self/status");
    std::string line;
    while (std::getline(status, line)) {
        if (line.rfind("VmRSS:", 0) == 0) {
            return std::stoull(line.substr(6)) * 1024;
        }
    }
    return 0;
#endif
}

std::shared_ptr<Sketch> makeRectSketch(double w, double h) {
    auto sketch = std::make_shared<Sketch>();
    sketch->addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(0, 0), Vec2(w, 0)));
    sketch->addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(w, 0), Vec2(w, h)));
    sketch->addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(w, h), Vec2(0, h)));
    sketch->addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(0, h), Vec2(0, 0)));
    return sketch;
}

}  // namespace

TEST(Era2MemoryTest, HundredPartLightweightAssemblyStaysUnderMemoryBound) {
    // 100 distinct part files (distinct canonical paths — no registry dedup),
    // each carrying a tessellation cache. Build one part and byte-copy it:
    // per-component memory cost is identical, but the setup stays seconds
    // instead of minutes (rebuild + tessellation + save per part is slow in
    // debug builds).
    const fs::path dir = fs::temp_directory_path() / "hz_era2_memory_parts";
    fs::create_directories(dir);

    Document part;
    part.setType(DocumentType::Part);
    auto sketch = makeRectSketch(10.0, 5.0);
    part.addSketch(sketch);
    part.featureTree().addFeature(std::make_unique<ExtrudeFeature>(sketch, Vec3(0, 0, 1), 3.0));
    part.rebuildModel();
    ASSERT_NE(part.solid(), nullptr);
    const std::string templatePath = (dir / "part_0.hzpart").string();
    ASSERT_TRUE(BinaryFormat::save(templatePath, part));

    std::vector<std::string> partPaths;
    partPaths.reserve(100);
    partPaths.push_back(templatePath);
    for (int i = 1; i < 100; ++i) {
        const std::string path = (dir / ("part_" + std::to_string(i) + ".hzpart")).string();
        fs::copy_file(templatePath, path, fs::copy_options::overwrite_existing);
        partPaths.push_back(path);
    }

    // Lightweight resolution must use ONLY the mesh loader — a fallback to
    // full part loads is the memory regression this guard exists to catch.
    int fullLoads = 0;
    DocumentManager mgr;
    mgr.setPartLoader([&fullLoads](const std::string& path, Document& doc) {
        ++fullLoads;
        return BinaryFormat::load(path, doc);
    });
    mgr.setMeshLoader([](const std::string& path) { return BinaryFormat::loadPartMesh(path); });

    const size_t rssBefore = currentRssBytes();

    std::vector<ComponentInstance> components(100);
    for (int i = 0; i < 100; ++i) {
        components[i].id = static_cast<uint64_t>(i + 1);
        components[i].partPath = partPaths[static_cast<size_t>(i)];
        ASSERT_TRUE(mgr.resolveComponent(components[i], ComponentState::Lightweight))
            << "component " << i;
        ASSERT_NE(components[i].cachedMesh, nullptr) << "component " << i;
    }

    const size_t rssAfter = currentRssBytes();

    EXPECT_EQ(fullLoads, 0) << "lightweight resolution must not load feature trees";

    if (rssBefore > 0 && rssAfter > rssBefore) {
        const size_t deltaBytes = rssAfter - rssBefore;
        RecordProperty("rss_delta_mb", static_cast<int>(deltaBytes / (1024 * 1024)));
        // Roadmap bar: 100-part assembly < 2 GB. 100 cached box meshes should
        // be a few MB; the generous bound keeps CI noise out while still
        // catching per-component document/feature-tree leaks.
        EXPECT_LT(deltaBytes, size_t{2} * 1024 * 1024 * 1024)
            << "100 lightweight components consumed " << deltaBytes / (1024 * 1024) << " MB";
    }

    std::error_code ec;
    fs::remove_all(dir, ec);
}
