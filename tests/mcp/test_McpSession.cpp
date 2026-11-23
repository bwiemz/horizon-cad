#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <random>
#include <string>

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
        // As the session names its files: macOS's temporary folder is under
        // /var, a link to /private/var, and Windows's can be given in short
        // names (RUNNER~1), and the paths it reported differed from these.
        dir = fs::canonical(dir);
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
    fails(session, "new_document", {{"type", "assembly"}});
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
