#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include "horizon/ai/CadTools.h"
#include "horizon/document/Document.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/document/UndoStack.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/Layer.h"

using namespace hz;
using ai::ToolRegistry;
using ai::ToolResult;
using nlohmann::json;

namespace {

constexpr double kPi = 3.14159265358979323846;

class CadToolsTest : public ::testing::Test {
protected:
    doc::Document document;
    ToolRegistry tools;
    int modelChanges = 0;
    int drawingChanges = 0;

    void SetUp() override {
        ai::registerCadTools(tools, document,
                             {[this] { ++modelChanges; }, [this] { ++drawingChanges; }});
    }

    /// Run a tool that must succeed; its result as JSON.
    json ok(const std::string& name, const json& input = json::object()) {
        const ToolResult result = tools.run(name, input);
        EXPECT_FALSE(result.isError) << name << ": " << result.content;
        if (result.isError) return json::object();
        const json value = json::parse(result.content, nullptr, false);
        EXPECT_FALSE(value.is_discarded()) << result.content;
        return value;
    }

    /// Run a tool that must fail; its message.
    std::string fails(const std::string& name, const json& input) {
        const ToolResult result = tools.run(name, input);
        EXPECT_TRUE(result.isError) << name << " " << input.dump() << " -> " << result.content;
        return result.content;
    }

    double volume() { return ok("get_document_info")["solid"].value("volume_mm3", 0.0); }

    std::uint64_t rectangleSketch(double w, double h, json extra = json::object()) {
        json input = {
            {"entities",
             json::array(
                 {{{"type", "rectangle"}, {"corner", {0, 0}}, {"width", w}, {"height", h}}})}};
        for (auto it = extra.begin(); it != extra.end(); ++it) input[it.key()] = it.value();
        return ok("create_sketch", input)["sketch_id"].get<std::uint64_t>();
    }

    void box(double x, double y, double z, json position = json::array({0, 0, 0})) {
        ok("add_primitive",
           {{"shape", "box"}, {"size_x", x}, {"size_y", y}, {"size_z", z}, {"position", position}});
    }

    /// The id of the first flat face facing @p dir.
    json face(const json& dir) {
        const json faces = ok("list_faces", {{"facing", dir}})["faces"];
        EXPECT_FALSE(faces.empty());
        return faces.empty() ? json::object() : faces[0];
    }
};

}  // namespace

TEST_F(CadToolsTest, RegistersTheToolsWithSchemas) {
    const auto specs = tools.specs();
    EXPECT_GE(specs.size(), 32u);
    for (const auto& spec : specs) {
        EXPECT_FALSE(spec.description.empty()) << spec.name;
        EXPECT_EQ(spec.inputSchema["type"], "object") << spec.name;
        // Only the schema words every provider takes.
        const std::string text = spec.inputSchema.dump();
        for (const char* banned :
             {"additionalProperties", "oneOf", "anyOf", "$ref", "\"default\""}) {
            EXPECT_EQ(text.find(banned), std::string::npos) << spec.name << " uses " << banned;
        }
    }
    EXPECT_EQ(tools.access("list_edges"), ai::ToolAccess::Read);
    EXPECT_EQ(tools.access("extrude"), ai::ToolAccess::Modify);
}

TEST_F(CadToolsTest, ANewDocumentBecomesAPartUndoably) {
    ASSERT_EQ(document.type(), doc::DocumentType::Drawing);
    box(10, 10, 10);
    EXPECT_EQ(document.type(), doc::DocumentType::Part);
    EXPECT_EQ(document.undoStack().undoCount(), 1u);  // one step, the type with it
    document.undoStack().undo();
    EXPECT_EQ(document.type(), doc::DocumentType::Drawing);
    EXPECT_EQ(document.featureTree().featureCount(), 0u);

    // A first feature that fails leaves the type as it was.
    const std::uint64_t line =
        ok("create_sketch",
           {{"entities",
             json::array({{{"type", "line"}, {"from", {0, 0}}, {"to", {5, 0}}}})}})["sketch_id"];
    document.undoStack().undo();  // the sketch, and the type
    EXPECT_EQ(document.type(), doc::DocumentType::Drawing);
    document.undoStack().redo();
    EXPECT_EQ(document.type(), doc::DocumentType::Part);
    fails("extrude", {{"sketch_id", line}, {"distance", 5}});  // an open profile
    EXPECT_EQ(document.featureTree().featureCount(), 0u);
}

TEST_F(CadToolsTest, SketchAndExtrudeMakeABox) {
    const auto sketch = rectangleSketch(60, 40);
    const json result = ok("extrude", {{"sketch_id", sketch}, {"distance", 5}});
    EXPECT_EQ(result["added"], "Extrude");
    EXPECT_EQ(result["feature_index"], 0);
    EXPECT_NEAR(result["solid"]["volume_mm3"].get<double>(), 12000.0, 1e-6);
    EXPECT_EQ(result["solid"]["bounding_box"]["size"], json::array({60.0, 40.0, 5.0}));
    EXPECT_EQ(document.type(), doc::DocumentType::Part);
    EXPECT_GE(modelChanges, 2);
}

TEST_F(CadToolsTest, AProfileWithAHole) {
    const auto sketch = ok(
        "create_sketch",
        {{"entities",
          json::array({{{"type", "rectangle"}, {"corner", {0, 0}}, {"width", 60}, {"height", 40}},
                       {{"type", "circle"}, {"center", {30, 20}}, {"radius", 5}}})}})["sketch_id"];
    ok("extrude", {{"sketch_id", sketch}, {"distance", 5}});
    // Faceted, so a little more material than the true circle leaves.
    EXPECT_NEAR(volume(), 12000.0 - kPi * 25.0 * 5.0, 5.0);
    EXPECT_LT(volume(), 12000.0 - 380.0);
}

TEST_F(CadToolsTest, CutThroughAll) {
    box(20, 20, 10);
    const auto sketch = ok(
        "create_sketch",
        {{"entities",
          json::array({{{"type", "circle"}, {"center", {10, 10}}, {"radius", 5}}})}})["sketch_id"];
    const json result = ok(
        "extrude", {{"sketch_id", sketch}, {"extent", "through_all_both"}, {"operation", "cut"}});
    EXPECT_EQ(result["operation"], "cut");
    EXPECT_NEAR(volume(), 4000.0 - kPi * 25.0 * 10.0, 10.0);
}

TEST_F(CadToolsTest, ExtrudeOnXZPlane) {
    const auto sketch = rectangleSketch(10, 20, {{"plane", "XZ"}});
    const json result = ok("extrude", {{"sketch_id", sketch}, {"distance", 5}});
    const json& bbox = result["solid"]["bounding_box"];
    // On XZ the sketch's x is world x and its y world z; it faces -Y.
    EXPECT_EQ(bbox["min"], json::array({0.0, -5.0, 0.0}));
    EXPECT_EQ(bbox["max"], json::array({10.0, 0.0, 20.0}));
}

TEST_F(CadToolsTest, SketchOnATopFaceAndCutDown) {
    box(20, 20, 10);
    const json top = face(json::array({0, 0, 1}));
    EXPECT_EQ(top["normal"], json::array({0.0, 0.0, 1.0}));
    const json sketch =
        ok("create_sketch",
           {{"face", top["name"]},
            {"entities",
             json::array(
                 {{{"type", "rectangle"}, {"center", {0, 0}}, {"width", 4}, {"height", 4}}})}});
    EXPECT_EQ(sketch["on_face"], top["name"]);
    EXPECT_EQ(sketch["plane"]["origin"], json::array({10.0, 10.0, 10.0}));
    // A cut from a face goes into the part.
    const json cut =
        ok("extrude", {{"sketch_id", sketch["sketch_id"]}, {"distance", 3}, {"operation", "cut"}});
    EXPECT_EQ(cut["direction"], json::array({0.0, 0.0, -1.0}));
    EXPECT_NEAR(volume(), 4000.0 - 48.0, 1e-6);
}

TEST_F(CadToolsTest, RevolveMakesACylinder) {
    const auto sketch = rectangleSketch(10, 20);
    ok("revolve", {{"sketch_id", sketch}});
    // About the sketch's y axis: radius 10, height 20 along y. Faceted.
    EXPECT_NEAR(volume(), kPi * 100.0 * 20.0, 0.01 * kPi * 100.0 * 20.0);
    const json features = ok("list_features")["features"];
    ASSERT_EQ(features.size(), 1u);
    EXPECT_EQ(features[0]["type"], "Revolve");
    EXPECT_NEAR(features[0]["parameters"]["angle"]["value"].get<double>(), 360.0, 1e-6);
    EXPECT_EQ(features[0]["parameters"]["angle"]["kind"], "angle");
}

TEST_F(CadToolsTest, PrimitiveWithPositionAndJoin) {
    box(10, 10, 10);
    box(10, 10, 10, json::array({5, 0, 0}));
    EXPECT_NEAR(volume(), 1500.0, 1e-6);
    const json features = ok("list_features")["features"];
    EXPECT_EQ(features[0]["operation"], "new");
    EXPECT_EQ(features[1]["operation"], "join");  // there was a part to join
}

TEST_F(CadToolsTest, CentredBoxAndCylinderPlacement) {
    const json result =
        ok("add_primitive",
           {{"shape", "box"}, {"size_x", 10}, {"size_y", 10}, {"size_z", 10}, {"centered", true}});
    EXPECT_EQ(result["solid"]["bounding_box"]["min"], json::array({-5.0, -5.0, -5.0}));
    EXPECT_NE(fails("add_primitive", {{"shape", "box"},
                                      {"size_x", 1},
                                      {"size_y", 1},
                                      {"size_z", 1},
                                      {"centered", true},
                                      {"axis", {1, 0, 0}}})
                  .find("centered"),
              std::string::npos);
}

TEST_F(CadToolsTest, FilletEdgesFromListEdges) {
    box(20, 20, 10);
    const json top = face(json::array({0, 0, 1}));
    const json edges = ok("list_edges", {{"on_face", top["id"]}})["edges"];
    ASSERT_EQ(edges.size(), 4u);
    json ids = json::array();
    for (const auto& edge : edges) {
        EXPECT_EQ(edge["kind"], "line");
        EXPECT_NEAR(edge["length"].get<double>(), 20.0, 1e-9);
        ids.push_back(edge["id"]);
    }
    const json result = ok("fillet", {{"edges", ids}, {"radius", 2}});
    EXPECT_EQ(result["edges"], 4);
    const double v = result["solid"]["volume_mm3"].get<double>();
    // Each edge loses (1 - pi/4) r^2 along its length, less at the corners.
    EXPECT_LT(v, 4000.0);
    EXPECT_GT(v, 4000.0 - 4.0 * (1.0 - kPi / 4.0) * 4.0 * 20.0 - 1.0);
}

TEST_F(CadToolsTest, ChamferEdges) {
    box(20, 20, 10);
    const json edges = ok("list_edges", {{"direction", {1, 0, 0}}})["edges"];
    ASSERT_EQ(edges.size(), 4u);  // the four edges along x
    const json result = ok("chamfer", {{"edges", json::array({edges[0]["id"]})}, {"distance", 2}});
    EXPECT_NEAR(result["solid"]["volume_mm3"].get<double>(), 4000.0 - 0.5 * 2 * 2 * 20, 1e-6);
}

TEST_F(CadToolsTest, ShellRemovesTopFace) {
    box(20, 20, 10);
    const json top = face(json::array({0, 0, 1}));
    ok("shell", {{"thickness", 2}, {"remove_faces", json::array({top["id"]})}});
    EXPECT_NEAR(volume(), 4000.0 - 16.0 * 16.0 * 8.0, 1e-6);
}

TEST_F(CadToolsTest, HoleThroughAFace) {
    box(20, 20, 10);
    const json top = face(json::array({0, 0, 1}));
    const json result = ok("add_hole", {{"face", top["name"]},
                                        {"position", {10, 10, 10}},
                                        {"diameter", 5},
                                        {"through_all", true}});
    EXPECT_EQ(result["face_normal"], json::array({0.0, 0.0, 1.0}));
    EXPECT_NEAR(volume(), 4000.0 - kPi * 6.25 * 10.0, 3.0);

    // A counterbored one, blind.
    ok("add_hole", {{"face", top["name"]},
                    {"position", {4, 4, 10}},
                    {"diameter", 3},
                    {"depth", 5},
                    {"type", "counterbore"}});
    const json features = ok("list_features")["features"];
    EXPECT_EQ(features.back()["parameters"]["type"]["choice"], "Counterbore");
}

TEST_F(CadToolsTest, LinearAndCircularPattern) {
    box(10, 10, 10);
    ok("linear_pattern", {{"direction", {1, 0, 0}}, {"spacing", 20}, {"count", 3}});
    EXPECT_NEAR(volume(), 3000.0, 1e-6);

    doc::Document other;
    ToolRegistry round;
    ai::registerCadTools(round, other);
    round.run(
        "add_primitive",
        {{"shape", "box"}, {"size_x", 4}, {"size_y", 4}, {"size_z", 4}, {"position", {20, -2, 0}}});
    const ToolResult result = round.run("circular_pattern", {{"count", 6}});
    ASSERT_FALSE(result.isError) << result.content;
    EXPECT_NEAR(json::parse(result.content)["solid"]["volume_mm3"].get<double>(), 6 * 64.0, 1e-6);
    EXPECT_NEAR(json::parse(result.content)["angle_step_deg"].get<double>(), 60.0, 1e-9);
}

TEST_F(CadToolsTest, MirrorAcrossPlane) {
    box(10, 10, 10, json::array({5, 0, 0}));
    const json result = ok("mirror", json::object());  // YZ by default
    EXPECT_NEAR(result["solid"]["volume_mm3"].get<double>(), 2000.0, 1e-6);
    EXPECT_EQ(result["solid"]["bounding_box"]["min"][0], -15.0);
}

TEST_F(CadToolsTest, PatternOfChosenFeatures) {
    box(60, 20, 5);
    const auto sketch = ok(
        "create_sketch",
        {{"entities",
          json::array({{{"type", "circle"}, {"center", {5, 10}}, {"radius", 2}}})}})["sketch_id"];
    ok("extrude", {{"sketch_id", sketch}, {"extent", "through_all_both"}, {"operation", "cut"}});
    const double one = 60 * 20 * 5 - volume();
    ok("linear_pattern",
       {{"direction", {1, 0, 0}}, {"spacing", 10}, {"count", 5}, {"features", json::array({1})}});
    EXPECT_NEAR(60 * 20 * 5 - volume(), 5 * one, 1e-6);
}

TEST_F(CadToolsTest, EditFeatureChangesDistanceAndAngle) {
    const auto sketch = rectangleSketch(60, 40);
    ok("extrude", {{"sketch_id", sketch}, {"distance", 5}});
    const json edited = ok("edit_feature", {{"index", 0}, {"parameters", {{"distance", 10}}}});
    EXPECT_EQ(edited["edited_feature"], 0);
    EXPECT_NEAR(volume(), 24000.0, 1e-6);
    // An extent by its name.
    ok("edit_feature", {{"index", 0}, {"parameters", {{"extent", "symmetric"}}}});
    EXPECT_EQ(ok("list_features")["features"][0]["parameters"]["extent"]["choice"],
              "Both ways, half each");
    EXPECT_NE(fails("edit_feature", {{"index", 0}, {"parameters", {{"extent", "sideways"}}}})
                  .find("Through all"),
              std::string::npos);
    EXPECT_EQ(ok("get_document_info")["solid"]["bounding_box"]["min"][2], -5.0);

    doc::Document other;
    ToolRegistry turn;
    ai::registerCadTools(turn, other);
    const json made =
        json::parse(turn.run("create_sketch", {{"entities", json::array({{{"type", "rectangle"},
                                                                          {"corner", {0, 0}},
                                                                          {"width", 10},
                                                                          {"height", 20}}})}})
                        .content);
    turn.run("revolve", {{"sketch_id", made["sketch_id"]}});
    const double full =
        json::parse(turn.run("get_document_info", json::object()).content)["solid"]["volume_mm3"];
    const ToolResult half =
        turn.run("edit_feature", {{"index", 0}, {"parameters", {{"angle", 180}}}});
    ASSERT_FALSE(half.isError) << half.content;
    EXPECT_NEAR(json::parse(half.content)["solid"]["volume_mm3"].get<double>(), full / 2.0, 1.0);
}

TEST_F(CadToolsTest, VariablesDriveAParameter) {
    const auto sketch = rectangleSketch(60, 40);
    ok("extrude", {{"sketch_id", sketch}, {"distance", 5}});
    const json set =
        ok("set_variables", {{"variables", {{"depth", "8 mm"}, {"half", "depth / 2"}}}});
    EXPECT_NEAR(set["variables"]["depth"]["value"].get<double>(), 8.0, 1e-12);
    EXPECT_EQ(set["variables"]["depth"]["measure"], "a length");
    ok("edit_feature", {{"index", 0}, {"expressions", {{"distance", "depth"}}}});
    EXPECT_NEAR(volume(), 60.0 * 40.0 * 8.0, 1e-6);
    // Changing the variable rebuilds the part.
    ok("set_variables", {{"variables", {{"depth", 12}}}});  // a bare number: mm
    EXPECT_NEAR(volume(), 60.0 * 40.0 * 12.0, 1e-6);
    const json features = ok("list_features")["features"];
    EXPECT_EQ(features[0]["expressions"]["distance"], "depth");

    EXPECT_NE(fails("set_variables", {{"variables", {{"bad name", "1 mm"}}}}).find("cannot name"),
              std::string::npos);
    fails("set_variables", {{"variables", {{"loop", "loop + 1 mm"}}}});
    fails("edit_feature", {{"index", 0}, {"expressions", {{"distance", "nosuchvariable"}}}});
    EXPECT_NEAR(volume(), 60.0 * 40.0 * 12.0, 1e-6);
}

TEST_F(CadToolsTest, ANumberReplacesAnExpression) {
    const auto sketch = rectangleSketch(10, 10);
    ok("extrude", {{"sketch_id", sketch}, {"distance", 5}});
    ok("set_variables", {{"variables", {{"depth", "8 mm"}}}});
    ok("edit_feature", {{"index", 0}, {"expressions", {{"distance", "depth"}}}});
    EXPECT_NEAR(volume(), 800.0, 1e-6);
    // A number sticks: the expression goes.
    ok("edit_feature", {{"index", 0}, {"parameters", {{"distance", 3}}}});
    EXPECT_NEAR(volume(), 300.0, 1e-6);
    EXPECT_FALSE(ok("list_features")["features"][0].contains("expressions"));
    ok("set_variables", {{"variables", {{"depth", "9 mm"}}}});
    EXPECT_NEAR(volume(), 300.0, 1e-6);
    // "" drops an expression and keeps the value it came to.
    ok("edit_feature", {{"index", 0}, {"expressions", {{"distance", "depth"}}}});
    EXPECT_NEAR(volume(), 900.0, 1e-6);
    ok("edit_feature", {{"index", 0}, {"expressions", {{"distance", ""}}}});
    ok("set_variables", {{"variables", {{"depth", "2 mm"}}}});
    EXPECT_NEAR(volume(), 900.0, 1e-6);
}

TEST_F(CadToolsTest, LoftBetweenTwoSquares) {
    const auto square = [this](double side, double offset) {
        return ok("create_sketch", {{"offset", offset},
                                    {"entities", json::array({{{"type", "rectangle"},
                                                               {"center", {0, 0}},
                                                               {"width", side},
                                                               {"height", side}}})}})["sketch_id"];
    };
    const json bottom = square(20, 0);
    const json top = square(10, 10);
    ok("loft", {{"sketch_ids", {bottom, top}}});
    // A frustum of a pyramid: h/3 (A1 + A2 + sqrt(A1 A2)).
    EXPECT_NEAR(volume(), 10.0 / 3.0 * (400.0 + 100.0 + 200.0), 1e-6);
    fails("loft", {{"sketch_ids", {bottom}}});
}

TEST_F(CadToolsTest, SweepAlongAPath) {
    const json path =
        ok("create_sketch",
           {{"plane", "XZ"},
            {"entities",
             json::array({{{"type", "line"}, {"from", {0, 0}}, {"to", {30, 0}}}})}})["sketch_id"];
    const json profile =
        ok("create_sketch",
           {{"plane", "YZ"},
            {"entities",
             json::array({{{"type", "circle"}, {"center", {0, 0}}, {"radius", 2}}})}})["sketch_id"];
    const json swept = ok("sweep", {{"profile_sketch_id", profile}, {"path_sketch_id", path}});
    EXPECT_NEAR(swept["solid"]["bounding_box"]["size"][0].get<double>(), 30.0, 1e-6);
    EXPECT_NEAR(volume(), kPi * 4.0 * 30.0, 0.02 * kPi * 4.0 * 30.0);
    fails("sweep", {{"profile_sketch_id", profile}, {"path_sketch_id", profile}});
}

TEST_F(CadToolsTest, DraftTapersTheSides) {
    box(20, 20, 10);
    ok("draft", {{"angle", 5}});
    EXPECT_GT(std::abs(volume() - 4000.0), 1.0);
    fails("draft", {{"angle", 0}});
    fails("draft", {{"angle", 60}});
}

TEST_F(CadToolsTest, CombineSeparateBodies) {
    box(10, 10, 10);
    ok("add_primitive", {{"shape", "box"},
                         {"size_x", 10},
                         {"size_y", 10},
                         {"size_z", 10},
                         {"position", {5, 0, 0}},
                         {"operation", "new"}});
    const json two = ok("get_document_info")["solid"];
    EXPECT_EQ(two["shells"], 2);
    const json combined = ok("combine_bodies", {{"operation", "union"}});
    EXPECT_EQ(combined["solid"]["shells"], 1);
    EXPECT_NEAR(combined["solid"]["volume_mm3"].get<double>(), 1500.0, 1e-6);
}

TEST_F(CadToolsTest, SketchOnADatumPlane) {
    const json datum =
        ok("add_datum", {{"kind", "plane"}, {"origin", {0, 0, 5}}, {"normal", {0, 0, 1}}});
    EXPECT_EQ(datum["feature_index"], 0);
    const json sketch =
        ok("create_sketch",
           {{"datum", 0},
            {"entities", json::array({{{"type", "circle"}, {"center", {0, 0}}, {"radius", 5}}})}});
    EXPECT_EQ(sketch["plane"]["origin"], json::array({0.0, 0.0, 5.0}));
    const json made = ok("extrude", {{"sketch_id", sketch["sketch_id"]}, {"distance", 5}});
    EXPECT_NEAR(made["solid"]["bounding_box"]["min"][2].get<double>(), 5.0, 1e-9);
    ok("add_datum", {{"kind", "axis"}, {"direction", {0, 0, 1}}});
    ok("add_datum", {{"kind", "point"}, {"position", {1, 2, 3}}});
    // A datum that is not a plane cannot be sketched on.
    fails("create_sketch",
          {{"datum", 2},
           {"entities", json::array({{{"type", "circle"}, {"center", {0, 0}}, {"radius", 1}}})}});
    fails("add_datum", {{"kind", "plane"}, {"normal", {0, 0, 0}}});
}

TEST_F(CadToolsTest, MeasureWhatWasMade) {
    box(20, 20, 10);
    const json top = face(json::array({0, 0, 1}));
    const json bottom = face(json::array({0, 0, -1}));
    const json side = face(json::array({1, 0, 0}));
    const json thickness =
        ok("measure", {{"from", {{"face", top["id"]}}}, {"to", {{"face", bottom["name"]}}}});
    EXPECT_NEAR(thickness["distance"].get<double>(), 10.0, 1e-9);
    EXPECT_NEAR(thickness["angle_between_normals_deg"].get<double>(), 180.0, 1e-9);
    const json corner =
        ok("measure", {{"from", {{"face", top["id"]}}}, {"to", {{"face", side["id"]}}}});
    EXPECT_NEAR(corner["angle_between_normals_deg"].get<double>(), 90.0, 1e-9);
    EXPECT_FALSE(corner.contains("distance"));
    const json height =
        ok("measure", {{"from", {{"point", {3, 3, 25}}}}, {"to", {{"face", top["id"]}}}});
    EXPECT_NEAR(height["distance"].get<double>(), 15.0, 1e-9);
    const json points =
        ok("measure", {{"from", {{"point", {0, 0, 0}}}}, {"to", {{"point", {3, 4, 0}}}}});
    EXPECT_NEAR(points["distance"].get<double>(), 5.0, 1e-12);
    const json edges = ok("list_edges", {{"direction", {1, 0, 0}}})["edges"];
    ASSERT_GE(edges.size(), 2u);
    const json gap =
        ok("measure", {{"from", {{"edge", edges[0]["id"]}}}, {"to", {{"edge", edges[1]["id"]}}}});
    EXPECT_GT(gap["distance"].get<double>(), 0.0);
    EXPECT_NEAR(gap["angle_deg"].get<double>(), 0.0, 1e-9);
    fails("measure", {{"from", {{"face", "nope"}}}, {"to", {{"point", {0, 0, 0}}}}});
    fails("measure", {{"from", 5}, {"to", {{"point", {0, 0, 0}}}}});
}

TEST_F(CadToolsTest, DeleteAndSuppressFeature) {
    box(10, 10, 10);
    box(10, 10, 10, json::array({20, 0, 0}));
    EXPECT_NEAR(volume(), 2000.0, 1e-6);
    ok("suppress_feature", {{"index", 1}});
    EXPECT_NEAR(volume(), 1000.0, 1e-6);
    EXPECT_EQ(ok("list_features")["features"][1]["suppressed"], true);
    ok("suppress_feature", {{"index", 1}, {"suppressed", false}});
    EXPECT_NEAR(volume(), 2000.0, 1e-6);
    ok("delete_feature", {{"index", 0}});
    EXPECT_EQ(document.featureTree().featureCount(), 1u);
    EXPECT_NEAR(volume(), 1000.0, 1e-6);
}

TEST_F(CadToolsTest, FailingFeatureIsWithdrawn) {
    box(20, 20, 10);
    const std::size_t features = document.featureTree().featureCount();
    const std::size_t steps = document.undoStack().undoCount();
    const json edges = ok("list_edges", {{"direction", {1, 0, 0}}})["edges"];
    const std::string message =
        fails("fillet", {{"edges", json::array({edges[0]["id"]})}, {"radius", 50}});
    EXPECT_NE(message.find("not added"), std::string::npos);
    EXPECT_EQ(document.featureTree().featureCount(), features);
    EXPECT_EQ(document.undoStack().undoCount(), steps);
    EXPECT_NEAR(volume(), 4000.0, 1e-6);
    EXPECT_LT(document.failedFeatureIndex(), 0);
}

TEST_F(CadToolsTest, EachMutationIsOneUndoStep) {
    const auto steps = [this] { return document.undoStack().undoCount(); };
    const auto sketch = rectangleSketch(10, 10);
    EXPECT_EQ(steps(), 1u);
    ok("extrude", {{"sketch_id", sketch}, {"distance", 5}});
    EXPECT_EQ(steps(), 2u);
    ok("set_variables", {{"variables", {{"t", "1 mm"}}}});
    EXPECT_EQ(steps(), 3u);
    ok("draw_2d",
       {{"entities", json::array({{{"type", "line"}, {"from", {0, 0}}, {"to", {5, 0}}},
                                  {{"type", "circle"}, {"center", {0, 0}}, {"radius", 2}}})},
        {"layer", "NEW"}});
    EXPECT_EQ(steps(), 4u);  // two entities and a new layer: one step
    ok("get_document_info");
    ok("list_edges");
    EXPECT_EQ(steps(), 4u);  // queries change nothing
    document.undoStack().undo();
    EXPECT_EQ(document.layerManager().getLayer("NEW"), nullptr);
    EXPECT_TRUE(document.draftDocument().entities().empty());
}

TEST_F(CadToolsTest, Draw2dOnLayer) {
    ok("create_layer", {{"name", "NOTES"}, {"color", "#FF8000"}, {"make_current", true}});
    ASSERT_NE(document.layerManager().getLayer("NOTES"), nullptr);
    EXPECT_EQ(document.layerManager().getLayer("NOTES")->color, 0xFFFF8000u);
    EXPECT_EQ(document.layerManager().currentLayer(), "NOTES");
    const json drawn =
        ok("draw_2d",
           {{"entities", json::array({{{"type", "line"}, {"from", {0, 0}}, {"to", {10, 0}}},
                                      {{"type", "text"}, {"position", {0, 5}}, {"text", "Hi"}}})}});
    EXPECT_EQ(drawn["layer"], "NOTES");
    ASSERT_EQ(drawn["ids"].size(), 2u);
    const json listed = ok("list_entities", {{"layer", "NOTES"}});
    EXPECT_EQ(listed["total"], 2);
    EXPECT_EQ(listed["entities"][0]["type"], "line");
    EXPECT_EQ(listed["entities"][1]["text"], "Hi");
    EXPECT_GE(drawingChanges, 2);
    EXPECT_NE(fails("create_layer", {{"name", "NOTES"}}).find("exists"), std::string::npos);
}

TEST_F(CadToolsTest, TransformAndDeleteEntities) {
    const json drawn =
        ok("draw_2d",
           {{"entities", json::array({{{"type", "line"}, {"from", {0, 0}}, {"to", {10, 0}}}})}});
    const auto id = drawn["ids"][0];
    ok("transform_entities", {{"ids", {id}}, {"operation", "move"}, {"dx", 5}, {"dy", 1}});
    auto* line = dynamic_cast<draft::DraftLine*>(
        document.draftDocument().findEntity(id.get<std::uint64_t>()));
    ASSERT_NE(line, nullptr);
    EXPECT_NEAR(line->start().x, 5.0, 1e-12);
    EXPECT_NEAR(line->start().y, 1.0, 1e-12);

    const json copied =
        ok("transform_entities", {{"ids", {id}}, {"operation", "copy"}, {"dx", 0}, {"dy", 10}});
    ASSERT_EQ(copied["new_ids"].size(), 1u);
    const json turned =
        ok("transform_entities",
           {{"ids", {id}}, {"operation", "rotate"}, {"angle", 90}, {"center", {5, 1}}});
    // The turned line replaces the original, under a new id.
    ASSERT_EQ(turned["new_ids"].size(), 1u);
    EXPECT_EQ(document.draftDocument().findEntity(id.get<std::uint64_t>()), nullptr);
    const json newId = turned["new_ids"][0];
    line = dynamic_cast<draft::DraftLine*>(
        document.draftDocument().findEntity(newId.get<std::uint64_t>()));
    ASSERT_NE(line, nullptr);
    EXPECT_NEAR(line->end().x, 5.0, 1e-9);
    EXPECT_NEAR(line->end().y, 11.0, 1e-9);
    EXPECT_EQ(document.draftDocument().entities().size(), 2u);  // with the copy made before
    // Mirror keeps the original by default.
    const json mirrored = ok(
        "transform_entities",
        {{"ids", {newId}}, {"operation", "mirror"}, {"axis_start", {0, 0}}, {"axis_end", {0, 1}}});
    EXPECT_EQ(mirrored["new_ids"].size(), 1u);
    EXPECT_EQ(document.draftDocument().entities().size(), 3u);
    document.undoStack().undo();  // the mirror
    document.undoStack().undo();  // the turn, as one step: the original is back
    EXPECT_NE(document.draftDocument().findEntity(id.get<std::uint64_t>()), nullptr);
    document.undoStack().redo();
    line = dynamic_cast<draft::DraftLine*>(
        document.draftDocument().findEntity(newId.get<std::uint64_t>()));
    ASSERT_NE(line, nullptr);

    // A locked layer is left alone.
    document.layerManager().getLayer(line->layer())->locked = true;
    const std::string refused = fails("delete_entities", {{"ids", {newId}}});
    EXPECT_NE(refused.find("locked"), std::string::npos);
    document.layerManager().getLayer(line->layer())->locked = false;
    const json deleted = ok("delete_entities", {{"ids", {newId, 999999999}}});
    EXPECT_EQ(deleted["deleted"], 1);
    EXPECT_EQ(deleted["skipped"].size(), 1u);
    EXPECT_EQ(document.draftDocument().entities().size(), 1u);
}

TEST_F(CadToolsTest, ListEdgesIsCappedAndCurvesAreOne) {
    box(20, 20, 10);
    const json capped = ok("list_edges", {{"max", 5}});
    EXPECT_EQ(capped["edges"].size(), 5u);
    EXPECT_EQ(capped["total"], 12);
    EXPECT_EQ(capped["omitted"], 7);

    doc::Document other;
    ToolRegistry round;
    ai::registerCadTools(round, other);
    round.run("add_primitive", {{"shape", "cylinder"}, {"radius", 5}, {"height", 10}});
    const json edges = json::parse(round.run("list_edges", json::object()).content);
    // Two circular rims, each one edge, whatever the facets.
    EXPECT_EQ(edges["total"], 2);
    EXPECT_EQ(edges["edges"][0]["kind"], "curve");
    EXPECT_NEAR(edges["edges"][0]["length"].get<double>(), 2 * kPi * 5, 0.2);
    const json faces = json::parse(round.run("list_faces", json::object()).content);
    EXPECT_EQ(faces["total"], 3);  // top, bottom, one curved side
}

TEST_F(CadToolsTest, NearPointSortsEdges) {
    box(20, 20, 10);
    const json edges = ok("list_edges", {{"near_point", {20, 20, 10}}, {"max", 3}})["edges"];
    ASSERT_EQ(edges.size(), 3u);
    for (const auto& edge : edges) {
        const json& mid = edge["mid"];
        EXPECT_TRUE(mid[0] == 20.0 || mid[1] == 20.0) << edge.dump();
    }
}

TEST_F(CadToolsTest, MassPropertiesWithMaterial) {
    box(10, 10, 10);
    const json aluminium = ok("get_mass_properties", {{"material", "aluminium"}});
    EXPECT_NEAR(aluminium["mass_g"].get<double>(), 2.7, 1e-9);
    EXPECT_EQ(aluminium["center_of_mass"], json::array({5.0, 5.0, 5.0}));
    const json custom = ok("get_mass_properties", {{"density_kg_m3", 1000}});
    EXPECT_NEAR(custom["mass_g"].get<double>(), 1.0, 1e-9);
    fails("get_mass_properties", {{"material", "unobtainium"}});
}

TEST_F(CadToolsTest, DocumentInfoAndALaterFailure) {
    const json empty = ok("get_document_info");
    EXPECT_TRUE(empty["solid"].is_null());
    EXPECT_EQ(empty["features"], 0);
    box(20, 20, 20);
    const json top = face(json::array({0, 0, 1}));
    const json edges = ok("list_edges", {{"on_face", top["id"]}})["edges"];
    json ids = json::array();
    for (const auto& edge : edges) ids.push_back(edge["id"]);
    ok("fillet", {{"edges", ids}, {"radius", 6}});
    // Make the box too thin for the fillet: the edit stands (the box does
    // not fail), and the fillet's failure is reported.
    const json edited = ok("edit_feature", {{"index", 0}, {"parameters", {{"depth", 4}}}});
    ASSERT_TRUE(edited.contains("build_error")) << edited.dump();
    EXPECT_EQ(edited["build_error"]["feature_index"], 1);
    const json info = ok("get_document_info");
    EXPECT_EQ(info["build_error"]["feature_index"], 1);
    EXPECT_EQ(ok("list_features")["features"][1].contains("failed"), true);
}

TEST_F(CadToolsTest, BadArgumentsAreErrorsNotCrashes) {
    box(20, 20, 10);
    const std::vector<std::pair<std::string, json>> bad = {
        {"extrude", {{"sketch_id", 999999}, {"distance", 5}}},
        {"extrude", {{"sketch_id", "abc"}, {"distance", 5}}},
        {"extrude", {{"sketch_id", -1}, {"distance", 5}}},
        {"add_primitive", {{"shape", "box"}, {"size_x", -1}, {"size_y", 1}, {"size_z", 1}}},
        {"add_primitive", {{"shape", "box"}, {"size_x", "ten"}, {"size_y", 1}, {"size_z", 1}}},
        {"add_primitive", {{"shape", "pyramid"}}},
        {"add_primitive", {{"shape", "torus"}, {"major_radius", 2}, {"minor_radius", 3}}},
        {"add_primitive", {{"shape", "sphere"}, {"radius", 1}, {"axis", {0, 0, 0}}}},
        {"fillet", {{"edges", json::array({"no/such/edge"})}, {"radius", 1}}},
        {"fillet", {{"edges", json::array()}, {"radius", 1}}},
        {"fillet", {{"edges", 5}, {"radius", 1}}},
        {"shell", {{"thickness", 0}}},
        {"shell", {{"thickness", 1}, {"remove_faces", json::array({"nope"})}}},
        {"add_hole", {{"face", "no face"}, {"position", {0, 0, 0}}, {"diameter", 3}, {"depth", 2}}},
        {"add_hole", {{"face", "x"}, {"position", {0, 0}}, {"diameter", 3}}},
        {"linear_pattern", {{"direction", {1, 0, 0}}, {"spacing", 5}, {"count", 1}}},
        {"linear_pattern", {{"direction", {1, 0, 0}}, {"spacing", 5}, {"count", 2.5}}},
        {"linear_pattern", {{"direction", {1, 0, 0}}, {"spacing", 5}, {"count", 1e12}}},
        {"circular_pattern", {{"count", 4}, {"total_angle", 400}}},
        {"linear_pattern",
         {{"direction", {1, 0, 0}}, {"spacing", 5}, {"count", 2}, {"features", {7}}}},
        {"edit_feature", {{"index", 5}, {"parameters", {{"width", 3}}}}},
        {"edit_feature", {{"index", 0}, {"parameters", {{"nonsense", 3}}}}},
        {"edit_feature", {{"index", 0}, {"parameters", {{"width", 0}}}}},
        {"edit_feature", {{"index", 0}}},
        {"delete_feature", {{"index", 3}}},
        {"create_sketch", {{"entities", json::array({{{"type", "circle"}, {"radius", 1}}})}}},
        {"create_sketch",
         {{"face", "missing"},
          {"entities", json::array({{{"type", "circle"}, {"center", {0, 0}}, {"radius", 1}}})}}},
        {"create_sketch",
         {{"plane", "AB"},
          {"entities", json::array({{{"type", "circle"}, {"center", {0, 0}}, {"radius", 1}}})}}},
        {"create_sketch",
         {{"normal", {0, 0, 1}},
          {"x_axis", {0, 0, 2}},
          {"entities", json::array({{{"type", "circle"}, {"center", {0, 0}}, {"radius", 1}}})}}},
        {"revolve", {{"sketch_id", 1}, {"angle", 0}}},
        {"draw_2d", {{"entities", "a line"}}},
        {"transform_entities", {{"ids", json::array({1})}, {"operation", "explode"}}},
        {"transform_entities", {{"ids", json::array()}, {"operation", "move"}}},
        {"create_layer", {{"name", "L"}, {"color", "chartreuse-ish"}}},
        {"list_edges", {{"max", 0}}},
        {"list_edges", {{"direction", {0, 0, 0}}}},
        {"get_mass_properties", {{"density_kg_m3", -5}}},
        {"set_variables", json::object()},
        {"set_variables", {{"remove", json::array({"never_was"})}}},
    };
    const std::size_t features = document.featureTree().featureCount();
    const std::size_t steps = document.undoStack().undoCount();
    for (const auto& [tool, input] : bad) {
        const ToolResult result = tools.run(tool, input);
        EXPECT_TRUE(result.isError) << tool << " " << input.dump() << " -> " << result.content;
        EXPECT_FALSE(result.content.empty()) << tool;
    }
    EXPECT_EQ(document.featureTree().featureCount(), features);
    EXPECT_EQ(document.undoStack().undoCount(), steps);
    EXPECT_NEAR(volume(), 4000.0, 1e-6);
}

TEST_F(CadToolsTest, QueriesOnAnEmptyPartSayThereIsNoSolid) {
    EXPECT_NE(fails("list_edges", json::object()).find("no solid"), std::string::npos);
    EXPECT_NE(fails("list_faces", json::object()).find("no solid"), std::string::npos);
    fails("get_mass_properties", json::object());
    fails("fillet", {{"edges", json::array({"a"})}, {"radius", 1}});
    EXPECT_EQ(ok("list_features")["features"].size(), 0u);
    EXPECT_EQ(ok("list_sketches")["sketches"].size(), 0u);
}

TEST_F(CadToolsTest, ListSketchesShowsUse) {
    const auto sketch = rectangleSketch(10, 10, {{"name", "Base"}});
    ok("extrude", {{"sketch_id", sketch}, {"distance", 5}});
    const json sketches = ok("list_sketches", {{"include_entities", true}})["sketches"];
    ASSERT_EQ(sketches.size(), 1u);
    EXPECT_EQ(sketches[0]["name"], "Base");
    EXPECT_EQ(sketches[0]["used_by_features"], json::array({0}));
    EXPECT_EQ(sketches[0]["entity_list"][0]["type"], "rectangle");
}
