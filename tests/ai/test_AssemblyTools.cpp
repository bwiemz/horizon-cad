// The assistant's assembly tools against real part files: a plate with a
// hole and a pin, made with the part tools, saved, and put together.

#include <gtest/gtest.h>

#include <atomic>
#include <cmath>
#include <filesystem>
#include <memory>
#include <string>

#include "horizon/ai/CadTools.h"
#include "horizon/document/AssemblyDocument.h"
#include "horizon/document/Document.h"
#include "horizon/document/DocumentManager.h"
#include "horizon/document/UndoStack.h"
#include "horizon/fileio/NativeFormat.h"

using namespace hz;
using ai::ToolRegistry;
using ai::ToolResult;
using nlohmann::json;

namespace fs = std::filesystem;

namespace {

/// A folder of its own for each test, removed after it.
fs::path freshFolder() {
    static std::atomic<int> serial{0};
    const fs::path dir =
        fs::temp_directory_path() /
        ("hz_ai_assembly_" +
         std::to_string(::testing::UnitTest::GetInstance()->current_test_info()->line()) + "_" +
         std::to_string(serial++));
    fs::remove_all(dir);
    fs::create_directories(dir);
    return dir;
}

json parsed(const ToolResult& result) {
    const json value = json::parse(result.content, nullptr, false);
    EXPECT_FALSE(value.is_discarded()) << result.content;
    return value;
}

class AssemblyToolsTest : public ::testing::Test {
protected:
    fs::path dir;
    doc::DocumentManager documents;
    doc::AssemblyDocument assembly;
    doc::Document backing;
    ToolRegistry tools;
    int changes = 0;

    void SetUp() override {
        dir = freshFolder();
        documents.setPartLoader([](const std::string& path, doc::Document& part) {
            return io::NativeFormat::load(path, part);
        });
        documents.setMeshLoader(
            [](const std::string& path) { return io::NativeFormat::loadPartMesh(path); });
        documents.setAssemblyLoader([](const std::string& path, doc::AssemblyDocument& a) {
            return io::NativeFormat::loadAssembly(path, a);
        });
        // A 40 x 40 x 10 plate with a 10 mm hole through its middle, and an
        // 8 mm pin 20 long, each made with the part tools.
        part("plate.hzpart", [](ToolRegistry& t) {
            run(t, "add_primitive",
                {{"shape", "box"}, {"size_x", 40}, {"size_y", 40}, {"size_z", 10}});
            run(t, "add_hole",
                {{"face", "top"},
                 {"position", {20, 20, 10}},
                 {"diameter", 10},
                 {"through_all", true}});
        });
        part("pin.hzpart", [](ToolRegistry& t) {
            run(t, "add_primitive", {{"shape", "cylinder"}, {"radius", 4}, {"height", 20}});
        });
        assembly.setFilePath((dir / "rig.hzasm").string());
        ai::registerAssemblyTools(tools, assembly, backing, documents, {[this] { ++changes; }});
    }

    void TearDown() override {
        std::error_code ec;
        fs::remove_all(dir, ec);
    }

    static json run(ToolRegistry& t, const std::string& name, const json& input) {
        const ToolResult result = t.run(name, input);
        EXPECT_FALSE(result.isError) << name << ": " << result.content;
        return result.isError ? json::object() : parsed(result);
    }

    template <typename Build>
    void part(const std::string& file, Build build) {
        doc::Document part;
        ToolRegistry t;
        ai::registerCadTools(t, part);
        build(t);
        ASSERT_NE(part.solid(), nullptr) << file;
        ASSERT_TRUE(io::NativeFormat::save((dir / file).string(), part));
    }

    json ok(const std::string& name, const json& input = json::object()) {
        return run(tools, name, input);
    }

    std::string fails(const std::string& name, const json& input) {
        const ToolResult result = tools.run(name, input);
        EXPECT_TRUE(result.isError) << name << " " << input.dump() << " -> " << result.content;
        return result.content;
    }

    std::uint64_t insert(const std::string& file, json extra = json::object()) {
        json input = {{"file", file}};
        for (auto it = extra.begin(); it != extra.end(); ++it) input[it.key()] = it.value();
        return ok("insert_component", input)["inserted"]["id"].get<std::uint64_t>();
    }

    json component(std::uint64_t id) {
        const json info = ok("get_assembly_info");  // held: the loop is over a part of it
        for (const auto& c : info["components"]) {
            if (c["id"] == id) return c;
        }
        ADD_FAILURE() << "no component " << id;
        return json::object();
    }

    /// The name of @p id's cylinder of @p radius, as list_component_geometry
    /// gives it.
    std::string cylinder(std::uint64_t id, double radius) {
        const json listed = ok("list_component_geometry", {{"component", id}});
        for (const auto& g : listed["geometry"]) {
            if (g["shape"] == "cylinder" && std::abs(g.value("radius", 0.0) - radius) < 1e-6) {
                return g["geometry"].get<std::string>();
            }
        }
        ADD_FAILURE() << "no cylinder of radius " << radius << " on " << id;
        return {};
    }

    std::size_t steps() const { return backing.undoStack().undoCount(); }
};

}  // namespace

TEST_F(AssemblyToolsTest, InfoListsThePartFilesInItsFolder) {
    const json info = ok("get_assembly_info");
    EXPECT_EQ(info["type"], "assembly");
    EXPECT_TRUE(info["components"].empty());
    EXPECT_EQ(info["files_in_folder"], json::array({"pin.hzpart", "plate.hzpart"}));
}

TEST_F(AssemblyToolsTest, InsertsAPartByItsNameAsOneUndoStep) {
    const auto plate = insert("plate.hzpart");
    EXPECT_EQ(steps(), 1u);
    EXPECT_EQ(changes, 1);
    const json info = component(plate);
    EXPECT_EQ(info["name"], "plate");
    EXPECT_EQ(info["bounding_box"]["size"], json::array({40.0, 40.0, 10.0}));

    // The next goes beside it, not into it.
    const auto pin = insert("pin.hzpart", {{"name", "Pin A"}});
    EXPECT_EQ(component(pin)["name"], "Pin A");
    EXPECT_GT(component(pin)["bounding_box"]["min"][0].get<double>(), 40.0);
    EXPECT_TRUE(ok("check_interference")["clear"].get<bool>());

    backing.undoStack().undo();
    EXPECT_EQ(assembly.components().size(), 1u);
}

TEST_F(AssemblyToolsTest, InsertSaysWhatIsWrongWithAFile) {
    const std::string missing = fails("insert_component", {{"file", "bracket.hzpart"}});
    EXPECT_NE(missing.find("pin.hzpart, plate.hzpart"), std::string::npos) << missing;
    EXPECT_NE(fails("insert_component", {{"file", "notes.txt"}}).find(".hzpart"),
              std::string::npos);
    EXPECT_EQ(steps(), 0u);

    // Unsaved, a name has no folder; a full path still works.
    assembly.setFilePath({});
    EXPECT_NE(fails("insert_component", {{"file", "plate.hzpart"}}).find("full path"),
              std::string::npos);
    insert((dir / "plate.hzpart").string());
}

TEST_F(AssemblyToolsTest, MatesAPinIntoTheHole) {
    const auto plate = insert("plate.hzpart");
    const auto pin = insert("pin.hzpart", {{"position", {100, -50, 30}}});
    ok("add_mate", {{"type", "fixed"}, {"a", {{"component", plate}, {"geometry", "bottom"}}}});
    ok("add_mate", {{"type", "concentric"},
                    {"a", {{"component", plate}, {"geometry", cylinder(plate, 5.0)}}},
                    {"b", {{"component", "pin"}, {"geometry", cylinder(pin, 4.0)}}}});
    const json made = ok("add_mate", {{"type", "coincident"},
                                      {"a", {{"component", plate}, {"geometry", "top"}}},
                                      {"b", {{"component", pin}, {"geometry", "bottom"}}}});
    EXPECT_TRUE(made["mates"]["solved"].get<bool>());
    EXPECT_EQ(made["mate"]["type"], "coincident");

    const json box = component(pin)["bounding_box"];
    EXPECT_NEAR(box["min"][2].get<double>(), 10.0, 1e-6);  // standing on the plate
    EXPECT_NEAR(box["max"][2].get<double>(), 30.0, 1e-6);
    EXPECT_NEAR((box["min"][0].get<double>() + box["max"][0].get<double>()) / 2.0, 20.0, 1e-3);
    EXPECT_NEAR((box["min"][1].get<double>() + box["max"][1].get<double>()) / 2.0, 20.0, 1e-3);
    EXPECT_EQ(ok("get_assembly_info")["mates"].size(), 3u);

    // Moved, a mated pin goes back where its mates hold it.
    const json moved = ok("place_component", {{"component", pin}, {"move_by", {0, 0, 50}}});
    EXPECT_TRUE(moved.contains("note"));
    EXPECT_NEAR(component(pin)["bounding_box"]["min"][2].get<double>(), 10.0, 1e-6);

    // In the hole, narrower than it, the pin is clear of the plate.
    EXPECT_TRUE(ok("check_interference")["clear"].get<bool>());
}

TEST_F(AssemblyToolsTest, AMateThatCannotHoldIsNotAdded) {
    const auto plate = insert("plate.hzpart");
    const auto pin = insert("pin.hzpart");
    ok("add_mate", {{"type", "fixed"}, {"a", {{"component", plate}, {"geometry", "bottom"}}}});
    ok("add_mate", {{"type", "fixed"}, {"a", {{"component", pin}, {"geometry", "bottom"}}}});
    const std::size_t before = steps();
    const std::string why =
        fails("add_mate", {{"type", "coincident"},
                           {"a", {{"component", plate}, {"geometry", "top"}}},
                           {"b", {{"component", pin}, {"geometry", "bottom"}}}});
    EXPECT_NE(why.find("cannot hold"), std::string::npos) << why;
    EXPECT_EQ(steps(), before);
    EXPECT_EQ(assembly.mates().size(), 2u);
}

TEST_F(AssemblyToolsTest, GeometryANameDoesNotFindIsSaid) {
    const auto plate = insert("plate.hzpart");
    const auto pin = insert("pin.hzpart");
    const std::string why =
        fails("add_mate", {{"type", "coincident"},
                           {"a", {{"component", plate}, {"geometry", "face:nothing"}}},
                           {"b", {{"component", pin}, {"geometry", "bottom"}}}});
    EXPECT_NE(why.find("list_component_geometry"), std::string::npos) << why;
    EXPECT_NE(fails("add_mate", {{"type", "coincident"},
                                 {"a", {{"component", plate}, {"geometry", "top"}}},
                                 {"b", {{"component", plate}, {"geometry", "bottom"}}}})
                  .find("two components"),
              std::string::npos);
    EXPECT_NE(fails("place_component", {{"component", 99}, {"move_by", {1, 0, 0}}})
                  .find("1 (plate), 2 (pin)"),
              std::string::npos);
}

TEST_F(AssemblyToolsTest, PlacesAndTurnsAFreeComponent) {
    const auto pin = insert("pin.hzpart");
    ok("place_component", {{"component", pin},
                           {"position", {5, 6, 7}},
                           {"rotation", {{"axis", {1, 0, 0}}, {"angle", 180}}}});
    const json placed = component(pin);
    EXPECT_EQ(placed["position"], json::array({5.0, 6.0, 7.0}));
    EXPECT_NEAR(placed["rotation"]["angle"].get<double>(), 180.0, 1e-6);
    EXPECT_NEAR(placed["rotation"]["axis"][0].get<double>(), 1.0, 1e-6);
    // Upside down: the pin hangs below its origin.
    EXPECT_NEAR(placed["bounding_box"]["max"][2].get<double>(), 7.0, 1e-6);
    EXPECT_NEAR(placed["bounding_box"]["min"][2].get<double>(), -13.0, 1e-6);

    ok("place_component", {{"component", pin}, {"move_by", {0, 0, 13}}});
    EXPECT_NEAR(component(pin)["bounding_box"]["min"][2].get<double>(), 0.0, 1e-6);
    ok("place_component",
       {{"component", pin}, {"rotate_by", {{"axis", {0, 0, 1}}, {"angle", 90}}}});
    EXPECT_NEAR(component(pin)["rotation"]["angle"].get<double>(), 180.0,
                1e-6);       // still upside down
    EXPECT_EQ(steps(), 4u);  // the insert and three moves
}

TEST_F(AssemblyToolsTest, InterferenceBillOfMaterialsAndPatterns) {
    const auto plate = insert("plate.hzpart");
    const auto pin = insert("pin.hzpart", {{"position", {0, 0, 0}}});
    const json clash = ok("check_interference");
    ASSERT_EQ(clash["interfering"].size(), 1u);
    EXPECT_GT(clash["interfering"][0]["shared_volume_mm3"].get<double>(), 100.0);

    const json pattern = ok("pattern_components", {{"components", json::array({"pin"})},
                                                   {"count", 3},
                                                   {"direction", {0, 1, 0}},
                                                   {"spacing", 60}});
    EXPECT_EQ(pattern["instances_made"], 2);
    const json bom = ok("get_bill_of_materials");
    ASSERT_EQ(bom["lines"].size(), 2u);
    EXPECT_EQ(bom["lines"][0]["part"], "plate");
    EXPECT_EQ(bom["lines"][1]["quantity"], 3);
    EXPECT_EQ(bom["total_quantity"], 4);

    ok("suppress_component", {{"component", plate}});
    EXPECT_EQ(ok("get_bill_of_materials")["total_quantity"], 3);
    EXPECT_TRUE(ok("check_interference")["clear"].get<bool>());

    // Two called "pin" now: a name is no longer enough.
    ok("rename_component", {{"component", pin}, {"name", "Pin"}});
    ok("insert_component", {{"file", "pin.hzpart"}, {"name", "pin"}});
    EXPECT_NE(fails("delete_component", {{"component", "PIN"}}).find("more than one"),
              std::string::npos);
    ok("delete_component", {{"component", pin}});
    EXPECT_EQ(assembly.patterns().size(), 0u);  // its pattern went with its seed
}
