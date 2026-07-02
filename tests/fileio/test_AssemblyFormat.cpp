#include <gtest/gtest.h>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>

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

    // The stored path is relative to the assembly file.
    std::ifstream in(path);
    std::string content((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    EXPECT_NE(content.find("parts/bolt.hzpart"), std::string::npos);
    EXPECT_EQ(content.find(dir.generic_string() + "/parts"), std::string::npos);

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
