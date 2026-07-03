#include <gtest/gtest.h>

#include <cstdio>
#include <filesystem>
#include <string>

#include "horizon/document/Document.h"
#include "horizon/scripting/ScriptContext.h"
#include "horizon/scripting/ScriptEngine.h"

using hz::script::ScriptContext;
using hz::script::ScriptEngine;

// ---------------------------------------------------------------------------
// Interpreter basics
// ---------------------------------------------------------------------------

TEST(ScriptEngineTest, RunsPythonAndCapturesStdout) {
    ScriptEngine engine;
    auto res = engine.run("print(6 * 7)");
    ASSERT_TRUE(res.ok) << res.error;
    EXPECT_EQ(res.output, "42\n");
    EXPECT_TRUE(res.error.empty());
}

TEST(ScriptEngineTest, SyntaxErrorReportedNotThrown) {
    ScriptEngine engine;
    auto res = engine.run("this is not python");
    EXPECT_FALSE(res.ok);
    EXPECT_FALSE(res.error.empty());
}

TEST(ScriptEngineTest, RuntimeExceptionReported) {
    ScriptEngine engine;
    auto res = engine.run("raise ValueError('boom')");
    EXPECT_FALSE(res.ok);
    EXPECT_NE(res.error.find("boom"), std::string::npos);
}

TEST(ScriptEngineTest, GlobalsPersistAcrossRuns) {
    ScriptEngine engine;
    ASSERT_TRUE(engine.run("counter = 10").ok);
    auto res = engine.run("counter += 5\nprint(counter)");
    ASSERT_TRUE(res.ok) << res.error;
    EXPECT_EQ(res.output, "15\n");
}

TEST(ScriptEngineTest, EvalReturnsRepr) {
    ScriptEngine engine;
    auto res = engine.eval("2 ** 8");
    ASSERT_TRUE(res.ok) << res.error;
    EXPECT_EQ(res.value, "256");
}

// ---------------------------------------------------------------------------
// horizon API module — math + reference geometry
// ---------------------------------------------------------------------------

TEST(ScriptEngineTest, Vec3Binding) {
    ScriptEngine engine;
    auto res = engine.run(R"(
import horizon as hz
a = hz.Vec3(1, 2, 3)
b = hz.Vec3(4, 5, 6)
c = a + b
print(c.x, c.y, c.z)
print(round(hz.Vec3(3, 4, 0).length(), 3))
)");
    ASSERT_TRUE(res.ok) << res.error;
    EXPECT_EQ(res.output, "5.0 7.0 9.0\n5.0\n");
}

TEST(ScriptEngineTest, ReferenceGeometryBinding) {
    ScriptEngine engine;
    auto res = engine.run(R"(
import horizon as hz
p = hz.plane_through_points(hz.Vec3(0, 0, 0), hz.Vec3(1, 0, 0), hz.Vec3(0, 1, 0))
print(round(p.normal.z, 3))
bad = hz.plane_through_points(hz.Vec3(0, 0, 0), hz.Vec3(1, 0, 0), hz.Vec3(2, 0, 0))
print(bad is None)
)");
    ASSERT_TRUE(res.ok) << res.error;
    EXPECT_EQ(res.output, "1.0\nTrue\n");
}

// ---------------------------------------------------------------------------
// horizon API module — document authoring
// ---------------------------------------------------------------------------

TEST(ScriptEngineTest, ScriptBuildsBox) {
    hz::doc::Document doc;
    doc.setType(hz::doc::DocumentType::Part);
    ScriptContext ctx(doc);

    ScriptEngine engine;
    auto res = engine.run(R"(
import horizon as hz
i = doc.add_rectangle_sketch(4.0, 3.0)
doc.add_extrude(i, hz.Vec3(0, 0, 1), 2.0)
doc.rebuild()
print(doc.solid_face_count())
)",
                          &ctx);
    ASSERT_TRUE(res.ok) << res.error;
    EXPECT_EQ(res.output, "6\n");
    EXPECT_EQ(doc.featureTree().featureCount(), 1u);
    ASSERT_NE(doc.solid(), nullptr);
    EXPECT_EQ(doc.solid()->faceCount(), 6u);
}

TEST(ScriptEngineTest, ScriptBuildsLinearPattern) {
    hz::doc::Document doc;
    doc.setType(hz::doc::DocumentType::Part);
    ScriptContext ctx(doc);

    ScriptEngine engine;
    auto res = engine.run(R"(
import horizon as hz
i = doc.add_rectangle_sketch(2.0, 2.0)
doc.add_extrude(i, hz.Vec3(0, 0, 1), 2.0)
doc.add_linear_pattern(hz.Vec3(1, 0, 0), 5.0, 3)
doc.rebuild()
print(doc.solid_shell_count())
)",
                          &ctx);
    ASSERT_TRUE(res.ok) << res.error;
    EXPECT_EQ(res.output, "3\n");
    ASSERT_NE(doc.solid(), nullptr);
    EXPECT_EQ(doc.solid()->shellCount(), 3u);
}

TEST(ScriptEngineTest, ScriptBuildsPrimitive) {
    hz::doc::Document doc;
    doc.setType(hz::doc::DocumentType::Part);
    ScriptContext ctx(doc);

    ScriptEngine engine;
    auto res = engine.run(R"(
doc.add_box(4.0, 4.0, 4.0)
doc.rebuild()
print(doc.solid_face_count(), round(doc.mass_properties().volume, 1))
)",
                          &ctx);
    ASSERT_TRUE(res.ok) << res.error;
    EXPECT_EQ(res.output, "6 64.0\n");  // box 4^3 = volume 64
}

TEST(ScriptEngineTest, ScriptQueriesMassProperties) {
    hz::doc::Document doc;
    doc.setType(hz::doc::DocumentType::Part);
    ScriptContext ctx(doc);

    ScriptEngine engine;
    auto res = engine.run(R"(
import horizon as hz
i = doc.add_rectangle_sketch(2.0, 2.0)
doc.add_extrude(i, hz.Vec3(0, 0, 1), 3.0)
doc.rebuild()
mp = doc.mass_properties()          # unit density
mp2 = doc.mass_properties(2.0)      # density 2
print(round(mp.volume, 3), round(mp.mass, 3), round(mp.center_of_mass.z, 3))
print(round(mp2.mass, 3), mp.valid)
)",
                          &ctx);
    ASSERT_TRUE(res.ok) << res.error;
    // Box 2x2x3: volume 12, unit mass 12, centroid z = 1.5; density 2 -> mass 24.
    EXPECT_EQ(res.output, "12.0 12.0 1.5\n24.0 True\n");
}

TEST(ScriptEngineTest, ScriptAddsTransparentDatum) {
    hz::doc::Document doc;
    doc.setType(hz::doc::DocumentType::Part);
    ScriptContext ctx(doc);

    ScriptEngine engine;
    auto res = engine.run(R"(
import horizon as hz
doc.add_datum_plane(hz.Vec3(0, 0, 5), hz.Vec3(0, 0, 1), hz.Vec3(1, 0, 0))
i = doc.add_rectangle_sketch(3.0, 3.0)
doc.add_extrude(i, hz.Vec3(0, 0, 1), 1.0)
doc.rebuild()
print(doc.feature_count(), doc.solid_face_count())
)",
                          &ctx);
    ASSERT_TRUE(res.ok) << res.error;
    // 2 features (datum + extrude); the datum does not add faces.
    EXPECT_EQ(res.output, "2 6\n");
}

TEST(ScriptEngineTest, ScriptExportsDrawingDxf) {
    hz::doc::Document doc;
    doc.setType(hz::doc::DocumentType::Part);
    ScriptContext ctx(doc);

    // generic_string() uses forward slashes, so the path embeds safely in the
    // Python string literal on every platform.
    const std::string path =
        (std::filesystem::temp_directory_path() / "hz_test_script_drawing.dxf").generic_string();
    std::remove(path.c_str());

    ScriptEngine engine;
    const std::string script =
        "doc.add_box(6.0, 4.0, 2.0)\n"
        "doc.rebuild()\n"
        "print(doc.export_drawing_dxf('" +
        path + "'))\n";
    auto res = engine.run(script.c_str(), &ctx);
    ASSERT_TRUE(res.ok) << res.error;
    EXPECT_EQ(res.output, "True\n");
    EXPECT_TRUE(std::filesystem::exists(path));
    EXPECT_GT(std::filesystem::file_size(path), 0u);

    std::remove(path.c_str());
}
