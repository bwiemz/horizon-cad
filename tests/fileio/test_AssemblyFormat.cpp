#include <gtest/gtest.h>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "horizon/document/AssemblyDocument.h"
#include "horizon/fileio/NativeFormat.h"

using namespace hz::doc;
using hz::io::NativeFormat;
using hz::math::Mat4;
using hz::math::Vec3;

namespace fs = std::filesystem;

namespace {

std::string tempPath(const std::string& name) {
    return (fs::temp_directory_path() / name).string();
}

}  // namespace

// ---------------------------------------------------------------------------
// AssemblyRoundTrip
// ---------------------------------------------------------------------------

TEST(AssemblyFormatTest, AssemblyRoundTrip) {
    AssemblyDocument original;

    ComponentInstance a;
    a.name = "base";
    a.partPath = "base.hzpart";
    a.transform = Mat4::translation(Vec3(1, 2, 3));
    original.addComponent(a);

    ComponentInstance b;
    b.name = "lid";
    b.partPath = "lid.hzpart";
    b.transform = Mat4::rotationZ(1.5) * Mat4::translation(Vec3(0, 0, 10));
    b.suppressed = true;
    original.addComponent(b);

    std::string path = tempPath("hz_test_asm_roundtrip.hzasm");
    ASSERT_TRUE(NativeFormat::saveAssembly(path, original));

    AssemblyDocument loaded;
    ASSERT_TRUE(NativeFormat::loadAssembly(path, loaded));

    ASSERT_EQ(loaded.components().size(), 2u);

    const auto& la = loaded.components()[0];
    EXPECT_EQ(la.name, "base");
    // Relative references resolve to absolute (against the assembly file)
    // on load.
    EXPECT_EQ(fs::path(la.partPath).lexically_normal(),
              (fs::temp_directory_path() / "base.hzpart").lexically_normal());
    EXPECT_FALSE(la.suppressed);
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            EXPECT_DOUBLE_EQ(la.transform.at(row, col), a.transform.at(row, col))
                << "row " << row << " col " << col;
        }
    }

    const auto& lb = loaded.components()[1];
    EXPECT_EQ(lb.name, "lid");
    EXPECT_TRUE(lb.suppressed);
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            EXPECT_NEAR(lb.transform.at(row, col), b.transform.at(row, col), 1e-12);
        }
    }

    // Loading resets dirty.
    EXPECT_FALSE(loaded.isDirty());

    std::remove(path.c_str());
}

// ---------------------------------------------------------------------------
// AbsolutePartPathsAreStoredRelative
// ---------------------------------------------------------------------------

TEST(AssemblyFormatTest, AbsolutePartPathsAreStoredRelative) {
    fs::path dir = fs::temp_directory_path() / "hz_test_asm_rel";
    fs::create_directories(dir / "parts");

    AssemblyDocument original;
    ComponentInstance a;
    a.name = "bolt";
    a.partPath = (dir / "parts" / "bolt.hzpart").string();  // absolute
    original.addComponent(a);

    std::string path = (dir / "main.hzasm").string();
    ASSERT_TRUE(NativeFormat::saveAssembly(path, original));

    // The stored path is relative to the assembly file. Scope the stream so
    // the file handle is closed before remove_all (Windows refuses to delete
    // open files).
    {
        std::ifstream in(path);
        std::string content((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
        EXPECT_NE(content.find("parts/bolt.hzpart"), std::string::npos);
        EXPECT_EQ(content.find(dir.generic_string() + "/parts"), std::string::npos);
    }

    // In memory the path is held absolute (resolved against the assembly
    // file) so a later Save As under a different directory re-relativizes
    // correctly.
    AssemblyDocument loaded;
    ASSERT_TRUE(NativeFormat::loadAssembly(path, loaded));
    ASSERT_EQ(loaded.components().size(), 1u);
    EXPECT_TRUE(fs::path(loaded.components()[0].partPath).is_absolute());
    EXPECT_EQ(fs::path(loaded.components()[0].partPath).lexically_normal(),
              (dir / "parts" / "bolt.hzpart").lexically_normal());

    // Save As into a sibling directory: the relative reference must be
    // recomputed against the new location, not copied through verbatim.
    fs::create_directories(dir / "sub");
    std::string movedPath = (dir / "sub" / "moved.hzasm").string();
    ASSERT_TRUE(NativeFormat::saveAssembly(movedPath, loaded));

    AssemblyDocument reloaded;
    ASSERT_TRUE(NativeFormat::loadAssembly(movedPath, reloaded));
    ASSERT_EQ(reloaded.components().size(), 1u);
    EXPECT_EQ(fs::path(reloaded.components()[0].partPath).lexically_normal(),
              (dir / "parts" / "bolt.hzpart").lexically_normal());

    fs::remove_all(dir);
}

// ---------------------------------------------------------------------------
// LoadRejectsNonAssemblyFiles
// ---------------------------------------------------------------------------

TEST(AssemblyFormatTest, LoadRejectsNonAssemblyFiles) {
    std::string path = tempPath("hz_test_not_asm.hzasm");
    {
        std::ofstream out(path);
        out << R"({"version": 16, "type": "hcad", "entities": []})";
    }

    AssemblyDocument loaded;
    EXPECT_FALSE(NativeFormat::loadAssembly(path, loaded));
    EXPECT_FALSE(NativeFormat::loadAssembly("/nonexistent/file.hzasm", loaded));

    std::remove(path.c_str());
}

// ---------------------------------------------------------------------------
// LoadRejectsMalformedTypeField
// ---------------------------------------------------------------------------

TEST(AssemblyFormatTest, LoadRejectsMalformedTypeField) {
    // A non-string "type" must return false, not throw.
    std::string path = tempPath("hz_test_bad_type.hzasm");
    {
        std::ofstream out(path);
        out << R"({"version": 16, "type": 42, "components": []})";
    }

    AssemblyDocument loaded;
    EXPECT_FALSE(NativeFormat::loadAssembly(path, loaded));

    std::remove(path.c_str());
}

// ---------------------------------------------------------------------------
// MatesRoundTrip
// ---------------------------------------------------------------------------

TEST(AssemblyFormatTest, MatesRoundTrip) {
    AssemblyDocument original;

    ComponentInstance a;
    a.partPath = "a.hzpart";
    uint64_t compA = original.addComponent(a);
    ComponentInstance b;
    b.partPath = "b.hzpart";
    uint64_t compB = original.addComponent(b);

    Mate coincident;
    coincident.type = MateType::Coincident;
    coincident.a = {compA, hz::topo::TopologyID::make("extrude_1", "cap_top")};
    coincident.b = {compB, hz::topo::TopologyID::make("extrude_2", "cap_bottom")};
    original.addMate(coincident);

    Mate distance;
    distance.type = MateType::Distance;
    distance.a = {compA, hz::topo::TopologyID::make("extrude_1", "lateral:0")};
    distance.b = {compB, hz::topo::TopologyID::make("extrude_2", "lateral:2")};
    distance.value = 12.5;
    original.addMate(distance);

    Mate fixed;
    fixed.type = MateType::Fixed;
    fixed.a = {compA, hz::topo::TopologyID()};
    original.addMate(fixed);

    std::string path = tempPath("hz_test_mates_roundtrip.hzasm");
    ASSERT_TRUE(NativeFormat::saveAssembly(path, original));

    AssemblyDocument loaded;
    ASSERT_TRUE(NativeFormat::loadAssembly(path, loaded));
    ASSERT_EQ(loaded.mates().size(), 3u);

    const auto& lc = loaded.mates()[0];
    EXPECT_EQ(lc.type, MateType::Coincident);
    EXPECT_EQ(lc.a.componentId, compA);
    EXPECT_EQ(lc.a.faceId.tag(), "extrude_1/cap_top");
    EXPECT_EQ(lc.b.faceId.tag(), "extrude_2/cap_bottom");

    const auto& ld = loaded.mates()[1];
    EXPECT_EQ(ld.type, MateType::Distance);
    EXPECT_DOUBLE_EQ(ld.value, 12.5);

    const auto& lf = loaded.mates()[2];
    EXPECT_EQ(lf.type, MateType::Fixed);
    EXPECT_EQ(lf.a.componentId, compA);
    EXPECT_FALSE(lf.a.faceId.isValid());

    std::remove(path.c_str());
}

// Phase 160: a mate on an edge or a datum keeps what it refers to, and a
// limited one its limits. A mate of a type this build does not know is
// left out and said so, not read as Coincident.
TEST(AssemblyFormatTest, EdgeAndDatumReferencesAndLimitsRoundTrip) {
    AssemblyDocument original;
    ComponentInstance a;
    a.partPath = "a.hzpart";
    const uint64_t compA = original.addComponent(a);
    ComponentInstance b;
    b.partPath = "b.hzpart";
    const uint64_t compB = original.addComponent(b);
    Mate mate;
    mate.type = MateType::Distance;
    mate.a = {compA, hz::topo::TopologyID::fromTag("extrude_1/edge:cap_top|side:e2"),
              hz::doc::ReferenceKind::Edge};
    mate.b = {compB, hz::topo::TopologyID::fromTag("datum_3"), hz::doc::ReferenceKind::Datum};
    mate.value = 4.0;
    mate.minimum = 2.0;
    mate.maximum = 9.0;
    original.addMate(mate);

    std::string text = NativeFormat::assemblyToJson(original, "");
    AssemblyDocument loaded;
    ASSERT_TRUE(NativeFormat::assemblyFromJson(text, loaded, ""));
    ASSERT_EQ(loaded.mates().size(), 1u);
    const Mate& kept = loaded.mates().front();
    EXPECT_EQ(kept.a.kind, hz::doc::ReferenceKind::Edge);
    EXPECT_EQ(kept.a.faceId.tag(), "extrude_1/edge:cap_top|side:e2");
    EXPECT_EQ(kept.b.kind, hz::doc::ReferenceKind::Datum);
    EXPECT_EQ(kept.b.faceId.tag(), "datum_3");
    ASSERT_TRUE(kept.minimum.has_value() && kept.maximum.has_value());
    EXPECT_DOUBLE_EQ(kept.minimum.value_or(0.0), 2.0);
    EXPECT_DOUBLE_EQ(kept.maximum.value_or(0.0), 9.0);

    // A type from a later build: left out, and said.
    const auto at = text.find("\"distance\"");
    ASSERT_NE(at, std::string::npos);
    text.replace(at, 10, "\"gearing\"");
    AssemblyDocument later;
    hz::io::ImportReport report;
    ASSERT_TRUE(NativeFormat::assemblyFromJson(text, later, "", nullptr, &report));
    EXPECT_TRUE(later.mates().empty());
    ASSERT_EQ(report.skipped.size(), 1u);
    EXPECT_NE(report.skipped.front().find("gearing"), std::string::npos) << report.skipped.front();
}

// Phase 161: exploded views round-trip, steps and all; which is shown is
// not kept. A step's component that is gone is left out of it, and a view
// with a step that is no direction is left out and said.
TEST(AssemblyFormatTest, ExplodedViewsRoundTrip) {
    AssemblyDocument original;
    ComponentInstance a;
    a.partPath = "a.hzpart";
    const uint64_t compA = original.addComponent(a);
    ExplodedView view;
    view.name = "Apart";
    view.steps.push_back({{compA, 99}, Vec3(0, -1, 0), 12.5});
    const uint64_t viewId = original.addExplodedView(view);
    ExplodedView second;
    second.name = "Other";
    original.addExplodedView(second);
    ASSERT_TRUE(original.setShownView(viewId));

    std::string text = NativeFormat::assemblyToJson(original, "");
    AssemblyDocument loaded;
    ASSERT_TRUE(NativeFormat::assemblyFromJson(text, loaded, ""));
    ASSERT_EQ(loaded.explodedViews().size(), 2u);
    const ExplodedView* kept = loaded.explodedView(viewId);
    ASSERT_NE(kept, nullptr);
    EXPECT_EQ(kept->name, "Apart");
    ASSERT_EQ(kept->steps.size(), 1u);
    EXPECT_EQ(kept->steps[0].components, std::vector<uint64_t>{compA}) << "99 is no component";
    EXPECT_NEAR(kept->steps[0].direction.y, -1.0, 1e-12);
    EXPECT_DOUBLE_EQ(kept->steps[0].distance, 12.5);
    EXPECT_EQ(loaded.shownView(), 0u) << "opened unexploded";
    EXPECT_FALSE(loaded.isDirty());

    // A direction of nothing: that view is left out, and said.
    const auto at = text.find("[0.0,-1.0,0.0]");
    ASSERT_NE(at, std::string::npos) << text;
    text.replace(at, 14, "[0.0,0.0,0.0]");
    AssemblyDocument broken;
    hz::io::ImportReport report;
    ASSERT_TRUE(NativeFormat::assemblyFromJson(text, broken, "", nullptr, &report));
    EXPECT_EQ(broken.explodedViews().size(), 1u);
    ASSERT_EQ(report.skipped.size(), 1u);
    EXPECT_NE(report.skipped.front().find("exploded view 1"), std::string::npos)
        << report.skipped.front();
}

// Phase 161: component patterns round-trip, and their instances keep their
// ids (an exploded view's step names one). A pattern this build cannot read
// is left out and said, and its instances go with it.
TEST(AssemblyFormatTest, ComponentPatternsRoundTripWithTheirInstances) {
    AssemblyDocument original;
    ComponentInstance pin;
    pin.name = "pin";
    pin.partPath = "pin.hzpart";
    const uint64_t seed = original.addComponent(pin);
    ComponentPattern ring;
    ring.name = "Ring";
    ring.kind = ComponentPattern::Kind::Circular;
    ring.seeds = {seed};
    ring.direction = Vec3::UnitZ;
    ring.axisPoint = Vec3(1, 2, 0);
    ring.spacing = 0.5;
    ring.count = 4;
    ring.skipped = {2};
    const uint64_t id = original.addPattern(ring);
    original.updatePatterns();
    ASSERT_EQ(original.components().size(), 3u) << "the seed and two instances";
    const uint64_t last = original.components().back().id;
    ExplodedView view;
    view.steps.push_back({{last}, Vec3::UnitZ, 5.0});
    original.addExplodedView(view);

    std::string text = NativeFormat::assemblyToJson(original, "");
    AssemblyDocument loaded;
    ASSERT_TRUE(NativeFormat::assemblyFromJson(text, loaded, ""));
    ASSERT_EQ(loaded.patterns().size(), 1u);
    const ComponentPattern& kept = loaded.patterns().front();
    EXPECT_EQ(kept.id, id);
    EXPECT_EQ(kept.name, "Ring");
    EXPECT_EQ(kept.kind, ComponentPattern::Kind::Circular);
    EXPECT_NEAR(kept.axisPoint.y, 2.0, 1e-12);
    EXPECT_DOUBLE_EQ(kept.spacing, 0.5);
    EXPECT_EQ(kept.count, 4);
    EXPECT_EQ(kept.skipped, std::vector<int>{2});
    ASSERT_EQ(loaded.components().size(), 3u);
    const ComponentInstance* instance = loaded.component(last);
    ASSERT_NE(instance, nullptr) << "the same id";
    EXPECT_EQ(instance->patternId, id);
    EXPECT_EQ(instance->patternIndex, 3);
    EXPECT_EQ(loaded.explodedViews().front().steps[0].components, std::vector<uint64_t>{last});
    EXPECT_FALSE(loaded.isDirty());

    // A kind from a later build: the pattern left out, and said; its
    // instances not kept as components of their own.
    const auto at = text.find("\"circular\"");
    ASSERT_NE(at, std::string::npos);
    text.replace(at, 10, "\"helical\"");
    AssemblyDocument later;
    hz::io::ImportReport report;
    ASSERT_TRUE(NativeFormat::assemblyFromJson(text, later, "", nullptr, &report));
    EXPECT_TRUE(later.patterns().empty());
    EXPECT_EQ(later.components().size(), 1u) << "the seed alone";
    ASSERT_EQ(report.skipped.size(), 1u);
    EXPECT_NE(report.skipped.front().find("helical"), std::string::npos) << report.skipped.front();
}
