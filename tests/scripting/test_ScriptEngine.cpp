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

// ---------------------------------------------------------------------------
// horizon API module — sheet metal
// ---------------------------------------------------------------------------

TEST(ScriptEngineTest, SheetMetalBendAllowance) {
    ScriptEngine engine;
    // Compare against the analytical value in Python to avoid float-format
    // fragility, printing a single boolean.
    const std::string script =
        "import math\n"
        "p = horizon.SheetMetalParams(thickness=2.0, bend_radius=3.0, k_factor=0.44)\n"
        "ba = horizon.bend_allowance(math.pi / 2, p)\n"
        "expected = (math.pi / 2) * (3.0 + 0.44 * 2.0)\n"
        "print(abs(ba - expected) < 1e-9 and p.is_valid())\n";
    auto res = engine.run(script);
    ASSERT_TRUE(res.ok) << res.error;
    EXPECT_EQ(res.output, "True\n");
}

TEST(ScriptEngineTest, SheetMetalDevelopedLength) {
    ScriptEngine engine;
    const std::string script =
        "import math\n"
        "p = horizon.SheetMetalParams(1.0, 1.0, 0.5)\n"
        "strip = horizon.SheetMetalStrip(segments=[10.0, 10.0, 10.0],\n"
        "                                bend_angles=[math.pi/2, math.pi/2])\n"
        "length = horizon.developed_length(strip, p)\n"
        "expected = 30.0 + 2.0 * (math.pi / 2) * 1.5\n"
        "print(abs(length - expected) < 1e-9)\n";
    auto res = engine.run(script);
    ASSERT_TRUE(res.ok) << res.error;
    EXPECT_EQ(res.output, "True\n");
}

// ---------------------------------------------------------------------------
// horizon API — CAM (toolpaths, G-code, feeds & speeds)
// ---------------------------------------------------------------------------

TEST(ScriptEngineTest, CamToolpathsAndGcode) {
    ScriptEngine engine;
    const std::string script =
        "import math\n"
        "sq = [horizon.Vec2(0, 0), horizon.Vec2(10, 0), horizon.Vec2(10, 10), horizon.Vec2(0, "
        "10)]\n"
        "c = horizon.cam_contour(sq, cut_depth=-2.0, safe_z=5.0, feed=100.0, closed=True)\n"
        "print(abs(c.cutting_length() - 47.0) < 1e-9)\n"  // 40 perimeter + 7 plunge
        "g = horizon.cam_gcode(c)\n"
        "print('G21' in g and 'M2' in g)\n"
        "pk = horizon.cam_pocket_rect(horizon.Vec2(0, 0), horizon.Vec2(100, 100),\n"
        "                             tool_radius=5.0, stepover=10.0, cut_depth=-2.0,\n"
        "                             safe_z=5.0, feed=100.0)\n"
        "print(pk.move_count() == 22)\n"
        "rpm = horizon.spindle_rpm(100.0, 10.0)\n"
        "print(abs(rpm - 1000.0 * 100.0 / (math.pi * 10.0)) < 1e-6)\n"
        "print(abs(horizon.feed_rate(rpm, 2, 0.05) - rpm * 2 * 0.05) < 1e-9)\n";
    auto res = engine.run(script);
    ASSERT_TRUE(res.ok) << res.error;
    EXPECT_EQ(res.output, "True\nTrue\nTrue\nTrue\nTrue\n");
}

// ---------------------------------------------------------------------------
// horizon API — stress-life fatigue
// ---------------------------------------------------------------------------

TEST(ScriptEngineTest, FatigueSafetyFactor) {
    ScriptEngine engine;
    // Steel S-N curve from Su = 500 MPa (Se = 250 MPa). A fully-reversed load at
    // half the endurance limit has a safety factor of 2; a tensile mean lowers it.
    const std::string script =
        "import math\n"
        "sn = horizon.SNCurve.steel(500e6)\n"
        "print(sn.is_valid())\n"
        "print(abs(horizon.fatigue_safety_factor(sn, sn.endurance_limit, 0.0) - 1.0) < 1e-9)\n"
        "print(abs(horizon.fatigue_safety_factor(sn, 0.5 * sn.endurance_limit, 0.0) - 2.0) < "
        "1e-9)\n"
        "print(math.isinf(horizon.cycles_to_failure(sn, 0.4 * 500e6)))\n";
    auto res = engine.run(script);
    ASSERT_TRUE(res.ok) << res.error;
    EXPECT_EQ(res.output, "True\nTrue\nTrue\nTrue\n");
}

// ---------------------------------------------------------------------------
// horizon API — FEA static analysis on the document's solid
// ---------------------------------------------------------------------------

TEST(ScriptEngineTest, StaticAnalysisOnSolid) {
    hz::doc::Document doc;
    doc.setType(hz::doc::DocumentType::Part);
    ScriptContext ctx(doc);

    // A steel bar (10 x 1 x 1) pulled axially: delta = FL/(A E) is recovered as
    // the peak displacement, and the stress is non-zero.
    ScriptEngine engine;
    const std::string script =
        "doc.add_box(10.0, 1.0, 1.0)\n"
        "doc.rebuild()\n"
        "E = 200e9\n"
        "F = 1.0e6\n"
        "r = doc.static_analysis(force=F, youngs_modulus=E, poisson_ratio=0.3,\n"
        "                        axis=0, resolution=6)\n"
        "expected = F * 10.0 / (1.0 * E)\n"
        "print(r.converged)\n"
        "print(abs(r.max_displacement - expected) < 0.1 * expected)\n"
        "print(r.max_von_mises > 0.0)\n";
    auto res = engine.run(script, &ctx);
    ASSERT_TRUE(res.ok) << res.error;
    EXPECT_EQ(res.output, "True\nTrue\nTrue\n");
}

TEST(ScriptEngineTest, StaticAnalysisWithoutSolidDoesNotConverge) {
    hz::doc::Document doc;
    doc.setType(hz::doc::DocumentType::Part);
    ScriptContext ctx(doc);

    ScriptEngine engine;
    auto res = engine.run(
        "r = doc.static_analysis(force=1.0, youngs_modulus=200e9)\nprint(r.converged)\n", &ctx);
    ASSERT_TRUE(res.ok) << res.error;
    EXPECT_EQ(res.output, "False\n");
}

TEST(ScriptEngineTest, ModalAnalysisOnSolid) {
    hz::doc::Document doc;
    doc.setType(hz::doc::DocumentType::Part);
    ScriptContext ctx(doc);

    // A steel bar fixed at one end has strictly positive, ascending natural
    // frequencies. The list is returned as a plain Python list of floats.
    ScriptEngine engine;
    const std::string script =
        "doc.add_box(10.0, 1.0, 1.0)\n"
        "doc.rebuild()\n"
        "r = doc.modal_analysis(youngs_modulus=200e9, poisson_ratio=0.3, density=7850.0,\n"
        "                       axis=0, num_modes=4, resolution=4)\n"
        "f = r.natural_frequencies\n"
        "print(r.converged)\n"
        "print(len(f) == 4)\n"
        "print(f[0] > 0.0)\n"
        "print(all(f[i] >= f[i - 1] - 1e-6 for i in range(1, len(f))))\n";
    auto res = engine.run(script, &ctx);
    ASSERT_TRUE(res.ok) << res.error;
    EXPECT_EQ(res.output, "True\nTrue\nTrue\nTrue\n");
}

TEST(ScriptEngineTest, ModalAnalysisWithoutSolidDoesNotConverge) {
    hz::doc::Document doc;
    doc.setType(hz::doc::DocumentType::Part);
    ScriptContext ctx(doc);

    ScriptEngine engine;
    auto res = engine.run(
        "r = doc.modal_analysis(youngs_modulus=200e9)\n"
        "print(r.converged)\nprint(len(r.natural_frequencies))\n",
        &ctx);
    ASSERT_TRUE(res.ok) << res.error;
    EXPECT_EQ(res.output, "False\n0\n");
}

TEST(ScriptEngineTest, ThermalAnalysisOnSolid) {
    hz::doc::Document doc;
    doc.setType(hz::doc::DocumentType::Part);
    ScriptContext ctx(doc);

    // Hold the x=0 face at 0 and the x=L face at 100; steady 1-D conduction gives
    // a linear profile, so min=0, max=100, and |q| = k*dT/L is uniform.
    ScriptEngine engine;
    const std::string script =
        "doc.add_box(10.0, 2.0, 2.0)\n"
        "doc.rebuild()\n"
        "r = doc.thermal_analysis(conductivity=50.0, hot_temperature=100.0,\n"
        "                         cold_temperature=0.0, axis=0, resolution=4)\n"
        "print(r.converged)\n"
        "print(abs(r.max_temperature - 100.0) < 1e-6)\n"
        "print(abs(r.min_temperature - 0.0) < 1e-6)\n"
        "print(abs(r.max_flux - 50.0 * 100.0 / 10.0) < 1e-3)\n";
    auto res = engine.run(script, &ctx);
    ASSERT_TRUE(res.ok) << res.error;
    EXPECT_EQ(res.output, "True\nTrue\nTrue\nTrue\n");
}

TEST(ScriptEngineTest, ThermalAnalysisWithoutSolidDoesNotConverge) {
    hz::doc::Document doc;
    doc.setType(hz::doc::DocumentType::Part);
    ScriptContext ctx(doc);

    ScriptEngine engine;
    auto res = engine.run(
        "r = doc.thermal_analysis(conductivity=50.0, hot_temperature=100.0)\n"
        "print(r.converged)\n",
        &ctx);
    ASSERT_TRUE(res.ok) << res.error;
    EXPECT_EQ(res.output, "False\n");
}
