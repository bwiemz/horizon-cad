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
    EXPECT_EQ(la.partPath, "base.hzpart");
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

    AssemblyDocument loaded;
    ASSERT_TRUE(NativeFormat::loadAssembly(path, loaded));
    ASSERT_EQ(loaded.components().size(), 1u);
    EXPECT_EQ(loaded.components()[0].partPath, "parts/bolt.hzpart");

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
