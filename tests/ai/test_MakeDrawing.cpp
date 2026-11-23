// make_drawing: the request a model makes read and checked, then handed to
// the host, which makes the sheet (the window: tests/ui/test_AiAssistant.cpp).

#include <gtest/gtest.h>

#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "horizon/ai/CadTools.h"
#include "horizon/document/AssemblyDocument.h"
#include "horizon/document/Document.h"
#include "horizon/document/DocumentManager.h"
#include "horizon/document/UndoStack.h"

using namespace hz;
using hz::ai::DrawingMade;
using hz::ai::DrawingRequest;
using hz::ai::ToolRegistry;
using hz::ai::ToolResult;
using nlohmann::json;

namespace {

struct MakeDrawingTest : ::testing::Test {
    doc::Document document;
    ToolRegistry tools;
    std::vector<DrawingRequest> asked;
    DrawingMade answer;

    void SetUp() override {
        answer.tab = "Drawing of plate";
        answer.scale = "1:2";
        answer.views = {"front", "top", "right", "isometric"};
        ai::CadToolHooks hooks;
        hooks.makeDrawing = [this](const DrawingRequest& request) {
            asked.push_back(request);
            return answer;
        };
        ai::registerCadTools(tools, document, std::move(hooks));
    }

    /// A part with a solid, saved as plate.hzpart.
    void savedPart() {
        const ToolResult box = tools.run(
            "add_primitive", {{"shape", "box"}, {"size_x", 80}, {"size_y", 40}, {"size_z", 6}});
        ASSERT_FALSE(box.isError) << box.content;
        document.setFilePath("/work/plate.hzpart");
        document.setDirty(false);
    }

    json made(const json& input = json::object()) {
        const ToolResult result = tools.run("make_drawing", input);
        EXPECT_FALSE(result.isError) << result.content;
        return json::parse(result.content, nullptr, false);
    }

    std::string refused(const json& input) {
        const ToolResult result = tools.run("make_drawing", input);
        EXPECT_TRUE(result.isError) << result.content;
        return result.content;
    }
};

}  // namespace

TEST(MakeDrawingToolTest, OnlyWhereTheHostMakesSheets) {
    doc::Document document;
    ToolRegistry tools;
    ai::registerCadTools(tools, document);
    EXPECT_FALSE(tools.has("make_drawing"));  // horizon-mcp, a test: no window to open it in
}

TEST_F(MakeDrawingTest, TheHostIsAskedForTheSheetTheModelDescribed) {
    savedPart();
    const json result = made({{"paper", "Letter"},
                              {"orientation", "portrait"},
                              {"scale", "1:2"},
                              {"title", "Mounting plate"},
                              {"part_number", "MP-100"},
                              {"revision", "B"},
                              {"drawn_by", "AI"},
                              {"date", "2026-11-23"},
                              {"material", "6061-T6"},
                              {"company", "Acme"}});
    ASSERT_EQ(asked.size(), 1u);
    const DrawingRequest& request = asked[0];
    EXPECT_EQ(request.source, "/work/plate.hzpart");
    EXPECT_EQ(request.paper, "ansi_a");  // Letter
    EXPECT_TRUE(request.portrait);
    EXPECT_DOUBLE_EQ(request.scale, 0.5);
    EXPECT_EQ(request.title, "Mounting plate");
    EXPECT_EQ(request.partNumber, "MP-100");
    EXPECT_EQ(request.revision, "B");
    EXPECT_EQ(request.drawnBy, "AI");
    EXPECT_EQ(request.date, "2026-11-23");
    EXPECT_EQ(request.material, "6061-T6");
    EXPECT_EQ(request.company, "Acme");
    EXPECT_EQ(result["tab"], "Drawing of plate");
    EXPECT_EQ(result["paper"], "ansi_a");
    EXPECT_EQ(result["orientation"], "portrait");
    EXPECT_EQ(result["scale"], "1:2");
    EXPECT_EQ(result["views"].size(), 4u);
    EXPECT_NE(result["note"].get<std::string>().find("not saved"), std::string::npos);
    // Nothing was added to the part.
    EXPECT_EQ(document.undoStack().undoCount(), 1u);
}

TEST_F(MakeDrawingTest, ByDefaultA3LandscapeAtTheScaleThatFits) {
    savedPart();
    made();
    ASSERT_EQ(asked.size(), 1u);
    EXPECT_EQ(asked[0].paper, "a3");
    EXPECT_FALSE(asked[0].portrait);
    EXPECT_EQ(asked[0].scale, 0.0);
    EXPECT_TRUE(asked[0].title.empty());  // the host names it after the file
}

TEST_F(MakeDrawingTest, ScalesAsADrawingWritesThem) {
    savedPart();
    made({{"scale", "2:1"}});
    made({{"scale", 0.2}});
    made({{"scale", "1"}});
    ASSERT_EQ(asked.size(), 3u);
    EXPECT_DOUBLE_EQ(asked[0].scale, 2.0);
    EXPECT_DOUBLE_EQ(asked[1].scale, 0.2);
    EXPECT_DOUBLE_EQ(asked[2].scale, 1.0);
    for (const json& bad : {json("two"), json("1:0"), json("0"), json("1:2:3"), json(-1),
                            json("5000:1"), json(true)}) {
        EXPECT_NE(refused({{"scale", bad}}).find("\"scale\""), std::string::npos) << bad;
    }
    EXPECT_EQ(asked.size(), 3u);
}

TEST_F(MakeDrawingTest, AnUnsavedPartIsRefusedAndAChangedOneSaidToNeedSaving) {
    EXPECT_NE(refused({}).find("nothing to draw"), std::string::npos);  // no solid yet
    savedPart();
    document.setFilePath("");
    EXPECT_NE(refused({}).find("has not been saved"), std::string::npos);
    EXPECT_TRUE(asked.empty());

    // Changed since it was saved: drawn from its tab, which is what the user
    // sees, and the user told to save it too.
    document.setFilePath("/work/plate.hzpart");
    document.setDirty(true);
    const json result = made();
    EXPECT_NE(result["note"].get<std::string>().find("save it too"), std::string::npos);
}

TEST_F(MakeDrawingTest, ArgumentsOutOfTheWayAreRefused) {
    savedPart();
    EXPECT_NE(refused({{"paper", "b5"}}).find("\"paper\" must be one of"), std::string::npos);
    EXPECT_NE(refused({{"orientation", "sideways"}}).find("\"orientation\""), std::string::npos);
    EXPECT_NE(refused({{"title", std::string(500, 'x')}}).find("longer than a title block"),
              std::string::npos);
    EXPECT_TRUE(asked.empty());

    // What the host could not do is the model's error, as it said it.
    answer = {};
    answer.error = "The drawing could not be made: its part could not be read";
    EXPECT_NE(refused({}).find("its part could not be read"), std::string::npos);
}

TEST(MakeDrawingToolTest, AnAssemblyIsDrawnFromItsFile) {
    doc::AssemblyDocument assembly;
    doc::Document backing;
    doc::DocumentManager documents;
    ToolRegistry tools;
    std::optional<DrawingRequest> asked;
    ai::AssemblyToolHooks hooks;
    hooks.makeDrawing = [&asked](const DrawingRequest& request) {
        asked = request;
        DrawingMade made;
        made.tab = "Drawing of rig";
        return made;
    };
    ai::registerAssemblyTools(tools, assembly, backing, documents, std::move(hooks));
    ASSERT_TRUE(tools.has("make_drawing"));

    const ToolResult empty = tools.run("make_drawing", json::object());
    EXPECT_TRUE(empty.isError);
    EXPECT_NE(empty.content.find("The assembly has nothing to draw"), std::string::npos);

    doc::ComponentInstance component;
    component.name = "base";
    component.partPath = "base.hzpart";
    assembly.components().push_back(component);
    const ToolResult unsaved = tools.run("make_drawing", json::object());
    EXPECT_NE(unsaved.content.find("The assembly has not been saved"), std::string::npos);

    assembly.setFilePath("/work/rig.hzasm");
    assembly.setDirty(false);
    const ToolResult result = tools.run("make_drawing", {{"paper", "a2"}});
    ASSERT_FALSE(result.isError) << result.content;
    ASSERT_TRUE(asked.has_value());
    EXPECT_EQ(asked->source, "/work/rig.hzasm");
    EXPECT_EQ(asked->paper, "a2");
}
