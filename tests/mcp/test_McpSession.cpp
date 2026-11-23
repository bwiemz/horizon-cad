#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <iterator>
#include <random>
#include <string>

#include "horizon/document/AssemblyDocument.h"
#include "horizon/document/Document.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/mcp/McpSession.h"

namespace fs = std::filesystem;
using hz::ai::ToolResult;
using hz::mcp::McpSession;
using nlohmann::json;

namespace {

struct Fixture : ::testing::Test {
    fs::path dir;

    void SetUp() override {
        std::random_device seed;
        dir = fs::temp_directory_path() / ("hz_mcp_" + std::to_string(seed()));
        fs::create_directories(dir);
    }
    void TearDown() override {
        std::error_code ec;
        fs::remove_all(dir, ec);
    }

    static json ok(McpSession& session, const std::string& tool, const json& in = json::object()) {
        const ToolResult result = session.tools().run(tool, in);
        EXPECT_FALSE(result.isError) << tool << ": " << result.content;
        return json::parse(result.content, nullptr, false);
    }
    static std::string fails(McpSession& session, const std::string& tool, const json& in) {
        const ToolResult result = session.tools().run(tool, in);
        EXPECT_TRUE(result.isError) << tool << ": " << result.content;
        return result.content;
    }
    static void box(McpSession& session) {
        ok(session, "add_primitive",
           {{"shape", "box"}, {"size_x", 10}, {"size_y", 10}, {"size_z", 10}});
    }
};

}  // namespace

using McpSessionTest = Fixture;

TEST_F(McpSessionTest, HasTheCadAndDocumentTools) {
    McpSession session(true, dir);
    for (const char* name : {"create_sketch", "extrude", "list_edges", "get_session_info",
                             "new_document", "open_document", "save_document", "export_document"}) {
        EXPECT_TRUE(session.tools().has(name)) << name;
    }
    EXPECT_EQ(session.tools().access("save_document"), hz::ai::ToolAccess::External);
    const json info = ok(session, "get_session_info");
    EXPECT_EQ(info["type"], "part");
    EXPECT_TRUE(info["path"].is_null());
}

TEST_F(McpSessionTest, SavesAndOpensAPart) {
    const std::string path = (dir / "cube.hzpart").string();
    {
        McpSession session(true, dir);
        box(session);
        EXPECT_NE(fails(session, "save_document", json::object()).find("path"), std::string::npos);
        EXPECT_EQ(ok(session, "save_document", {{"path", path}})["saved"], path);
        EXPECT_EQ(ok(session, "get_session_info")["modified"], false);
        // Saved again where it was.
        box(session);
        ok(session, "save_document", json::object());
    }
    McpSession reopened(true, dir);
    std::string error;
    ASSERT_TRUE(reopened.open(path, &error)) << error;
    EXPECT_EQ(reopened.document().featureTree().featureCount(), 2u);
    ASSERT_NE(reopened.document().solid(), nullptr);
    EXPECT_EQ(reopened.path(), path);
}

TEST_F(McpSessionTest, OpenAndNewTakeOverAfterTheCall) {
    const std::string path = (dir / "one.hzpart").string();
    {
        McpSession writer(true, dir);
        box(writer);
        ok(writer, "save_document", {{"path", path}});
    }
    McpSession session(true, dir);
    const json opened = ok(session, "open_document", {{"path", path}});
    EXPECT_EQ(opened["features"], 1);
    // Not yet: the tool that asked is still the one being run.
    EXPECT_EQ(session.document().featureTree().featureCount(), 0u);
    session.applyPending();
    EXPECT_EQ(session.document().featureTree().featureCount(), 1u);
    EXPECT_EQ(session.path(), path);
    // The tools work on the new document.
    box(session);
    EXPECT_EQ(session.document().featureTree().featureCount(), 2u);

    ok(session, "new_document", {{"type", "drawing"}});
    session.applyPending();
    EXPECT_EQ(session.document().type(), hz::doc::DocumentType::Drawing);
    EXPECT_TRUE(session.path().empty());
    session.applyPending();  // nothing pending: nothing changes
    EXPECT_EQ(session.document().type(), hz::doc::DocumentType::Drawing);
}

TEST_F(McpSessionTest, BadFilesAreErrors) {
    McpSession session(true, dir);
    fails(session, "open_document", {{"path", (dir / "missing.hzpart").string()}});
    fails(session, "open_document", {{"path", (dir / "model.step").string()}});
    {
        std::ofstream junk(dir / "junk.hzpart");
        junk << "not json";
    }
    fails(session, "open_document", {{"path", (dir / "junk.hzpart").string()}});
    fails(session, "save_document", {{"path", (dir / "x.step").string()}});
    fails(session, "export_document", {{"path", (dir / "x.step").string()}});  // no solid yet
    fails(session, "new_document", {{"type", "sheet"}});
    session.applyPending();
    EXPECT_EQ(session.document().type(), hz::doc::DocumentType::Part);
}

TEST_F(McpSessionTest, ExportsTheSolidAndTheDrawing) {
    McpSession session(true, dir);
    box(session);
    for (const char* name : {"part.step", "part.stl", "part.glb"}) {
        const std::string path = (dir / name).string();
        ok(session, "export_document", {{"path", path}});
        EXPECT_GT(fs::file_size(path), 100u) << name;
    }
    fails(session, "export_document", {{"path", (dir / "part.obj").string()}});

    McpSession drawing(false, dir);
    ok(drawing, "draw_2d",
       {{"entities", json::array({{{"type", "circle"}, {"center", {0, 0}}, {"radius", 5}}})}});
    const std::string dxf = (dir / "sketch.dxf").string();
    ok(drawing, "export_document", {{"path", dxf}});
    std::ifstream in(dxf);
    const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    EXPECT_NE(text.find("CIRCLE"), std::string::npos);
}

TEST_F(McpSessionTest, FilesStayInTheFolder) {
    McpSession session(true, dir);
    box(session);
    // Relative paths are in the folder.
    ok(session, "save_document", {{"path", "inside.hzpart"}});
    EXPECT_TRUE(fs::exists(dir / "inside.hzpart"));
    // Out of it, by "..", by an absolute path, or through a link: refused.
    const fs::path outside = dir.parent_path() / (dir.filename().string() + "-outside.stl");
    EXPECT_NE(fails(session, "export_document", {{"path", "../escape.stl"}}).find("outside"),
              std::string::npos);
    fails(session, "export_document", {{"path", outside.string()}});
    fails(session, "open_document", {{"path", "../../etc/passwd.hcad"}});
    std::error_code ec;
    fs::create_directory_symlink(dir.parent_path(), dir / "up", ec);
    if (!ec) fails(session, "export_document", {{"path", "up/through-link.stl"}});
    // A link to nothing as the last part: writing through it would make its
    // target, outside.
    const fs::path target = dir.parent_path() / (dir.filename().string() + "-dangling.stl");
    fs::create_symlink(target, dir / "dangling.stl", ec);
    if (!ec) {
        EXPECT_NE(fails(session, "export_document", {{"path", "dangling.stl"}, {"overwrite", true}})
                      .find("symbolic link"),
                  std::string::npos);
        EXPECT_FALSE(fs::exists(target));
    }
#ifdef _WIN32
    EXPECT_NE(fails(session, "export_document", {{"path", "\\\\host\\share\\x.stl"}})
                  .find("network share"),
              std::string::npos);
#endif
    EXPECT_FALSE(fs::exists(outside));
    EXPECT_EQ(ok(session, "get_session_info")["folder"], fs::weakly_canonical(dir).string());
}

TEST_F(McpSessionTest, AFileIsReplacedOnlyWhenAsked) {
    McpSession session(true, dir);
    box(session);
    ok(session, "export_document", {{"path", "part.stl"}});
    EXPECT_NE(fails(session, "export_document", {{"path", "part.stl"}}).find("overwrite"),
              std::string::npos);
    ok(session, "export_document", {{"path", "part.stl"}, {"overwrite", true}});
    {
        std::ofstream other(dir / "someone-elses.hzpart");
        other << "{}";
    }
    fails(session, "save_document", {{"path", "someone-elses.hzpart"}});
    // Its own file is saved over as a matter of course.
    ok(session, "save_document", {{"path", "mine.hzpart"}});
    box(session);
    ok(session, "save_document", {{"path", "mine.hzpart"}});
    ok(session, "save_document", json::object());
}

TEST_F(McpSessionTest, AFailedSaveLeavesTheTypeAsItWas) {
    McpSession session(true, dir);
    box(session);
    // A folder that is not there: the save fails.
    fails(session, "save_document", {{"path", "no-such-folder/part.hcad"}});
    EXPECT_EQ(session.document().type(), hz::doc::DocumentType::Part);
}

TEST_F(McpSessionTest, AFileReadInPartIsNotSavedOverUnasked) {
    const fs::path file = dir / "future.hcad";
    {
        std::ofstream out(file);
        out << R"({"version": 16, "type": "hcad",
            "entities": [
                {"type": "line", "id": 1, "start": {"x": 0, "y": 0}, "end": {"x": 1, "y": 0}},
                {"type": "hologram", "id": 3}
            ]})";
    }
    McpSession session(false, dir);
    const json opened = ok(session, "open_document", {{"path", "future.hcad"}});
    ASSERT_TRUE(opened.contains("left_out")) << opened.dump();
    session.applyPending();
    EXPECT_EQ(ok(session, "get_session_info")["left_out"].size(), 1u);
    EXPECT_NE(fails(session, "save_document", json::object()).find("in part"), std::string::npos);
    std::ifstream before(file);
    const std::string kept((std::istreambuf_iterator<char>(before)),
                           std::istreambuf_iterator<char>());
    EXPECT_NE(kept.find("hologram"), std::string::npos);  // untouched
    ok(session, "save_document", {{"path", "copy.hcad"}});
    EXPECT_FALSE(ok(session, "get_session_info").contains("left_out"));
}

// The whole of an assembly from outside: parts made and saved, a new
// assembly that takes them from the folder, mated, saved, exported as STEP
// with its parts and placements, and opened again.
TEST_F(McpSessionTest, AssemblesPartsSavesAndExportsThem) {
    McpSession session(true, dir);
    box(session);
    ok(session, "save_document", {{"path", "block.hzpart"}});
    ok(session, "new_document", {{"type", "part"}});
    session.applyPending();
    ok(session, "add_primitive", {{"shape", "cylinder"}, {"radius", 2}, {"height", 8}});
    ok(session, "save_document", {{"path", "pin.hzpart"}});

    ok(session, "new_document", {{"type", "assembly"}});
    session.applyPending();
    ASSERT_NE(session.assembly(), nullptr);
    EXPECT_TRUE(session.tools().has("add_mate"));
    EXPECT_FALSE(session.tools().has("extrude"));
    EXPECT_EQ(ok(session, "get_session_info")["type"], "assembly");
    // Unsaved, it takes its parts from the server's folder.
    const json info = ok(session, "get_assembly_info");
    EXPECT_EQ(info["files_in_folder"], json::array({"block.hzpart", "pin.hzpart"}));
    const auto block =
        ok(session, "insert_component", {{"file", "block.hzpart"}})["inserted"]["id"];
    const auto pin = ok(session, "insert_component",
                        {{"file", "pin.hzpart"}, {"position", {30, 0, 0}}})["inserted"]["id"];
    ok(session, "add_mate", {{"type", "fixed"}, {"a", {{"component", block}}}});
    ok(session, "add_mate",
       {{"type", "coincident"},
        {"a", {{"component", block}, {"geometry", "top"}}},
        {"b", {{"component", pin}, {"geometry", "bottom"}}}});
    EXPECT_TRUE(ok(session, "get_session_info")["modified"].get<bool>());

    EXPECT_NE(fails(session, "save_document", {{"path", "rig.hzpart"}}).find(".hzasm"),
              std::string::npos);
    ok(session, "save_document", {{"path", "rig.hzasm"}});
    EXPECT_FALSE(ok(session, "get_session_info")["modified"].get<bool>());

    const json step = ok(session, "export_document", {{"path", "rig.step"}});
    EXPECT_EQ(step["parts"], 2);
    EXPECT_EQ(step["components"], 2);
    std::ifstream written(dir / "rig.step");
    const std::string text((std::istreambuf_iterator<char>(written)),
                           std::istreambuf_iterator<char>());
    EXPECT_NE(text.find("NEXT_ASSEMBLY_USAGE_OCCURRENCE"), std::string::npos);
    ok(session, "export_document", {{"path", "rig.stl"}});
    EXPECT_GT(fs::file_size(dir / "rig.stl"), 0u);
    EXPECT_NE(fails(session, "export_document", {{"path", "rig.dxf"}}).find(".step"),
              std::string::npos);

    ok(session, "new_document", {{"type", "part"}});
    session.applyPending();
    EXPECT_EQ(session.assembly(), nullptr);
    EXPECT_TRUE(session.tools().has("extrude"));
    const json opened = ok(session, "open_document", {{"path", "rig.hzasm"}});
    EXPECT_EQ(opened["components"], 2);
    session.applyPending();
    ASSERT_NE(session.assembly(), nullptr);
    EXPECT_EQ(session.assembly()->mates().size(), 2u);
    EXPECT_FALSE(ok(session, "get_session_info")["modified"].get<bool>());
}

// An assembly in the folder may place parts from anywhere: opened by a tool,
// what it places is kept to the folder too, checked before any is read.
TEST_F(McpSessionTest, AnAssemblyReachingOutOfTheFolderIsNotOpened) {
    const fs::path outside = dir.parent_path() / (dir.filename().string() + "_outside");
    fs::create_directories(outside);
    {
        McpSession maker(true, outside);
        box(maker);
        ok(maker, "save_document", {{"path", "secret.hzpart"}});
    }
    McpSession session(true, dir);
    ok(session, "new_document", {{"type", "assembly"}});
    session.applyPending();
    {
        // Written as a hand-made file would be: its component outside.
        hz::doc::ComponentInstance far;
        far.partPath = (outside / "secret.hzpart").string();
        session.assembly()->addComponent(far);
    }
    ok(session, "save_document", {{"path", "reach.hzasm"}});
    const std::string why = fails(session, "open_document", {{"path", "reach.hzasm"}});
    EXPECT_NE(why.find("outside the server's folder"), std::string::npos) << why;
    // On the command line, the user's own choice, it opens.
    std::string error;
    EXPECT_TRUE(session.open((dir / "reach.hzasm").string(), &error)) << error;
    fs::remove_all(outside);
}
