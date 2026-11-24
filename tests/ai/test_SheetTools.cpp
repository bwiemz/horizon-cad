// The drawing sheet tools: a model's requests read and checked, and handed
// to the host as SheetChanges (the window's side: tests/ui/test_AiAssistant.cpp).

#include <gtest/gtest.h>

#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "horizon/ai/CadTools.h"

using hz::ai::SheetChange;
using hz::ai::SheetChanged;
using hz::ai::SheetToolHooks;
using hz::ai::ToolRegistry;
using hz::ai::ToolResult;
using nlohmann::json;

namespace {

struct SheetToolsTest : ::testing::Test {
    ToolRegistry tools;
    std::vector<SheetChange> asked;
    SheetChanged answer;
    std::vector<int> edgesAskedFor;

    void SetUp() override {
        answer.step = true;
        SheetToolHooks hooks;
        hooks.info = [] { return json{{"paper", "a3"}, {"views", json::array()}}; };
        hooks.edges = [this](int view, std::string* error) -> json {
            edgesAskedFor.push_back(view);
            if (view > 3) {
                *error = "the sheet has no view " + std::to_string(view);
                return nullptr;
            }
            return {{"view", view}, {"edges", json::array({{{"edge", "e1"}}})}};
        };
        hooks.change = [this](const SheetChange& change) {
            asked.push_back(change);
            return answer;
        };
        hz::ai::registerSheetTools(tools, std::move(hooks));
    }

    json made(const std::string& tool, const json& input) {
        const ToolResult result = tools.run(tool, input);
        EXPECT_FALSE(result.isError) << tool << ": " << result.content;
        return json::parse(result.content, nullptr, false);
    }

    std::string refused(const std::string& tool, const json& input) {
        const ToolResult result = tools.run(tool, input);
        EXPECT_TRUE(result.isError) << tool << ": " << result.content;
        return result.content;
    }
};

}  // namespace

TEST(SheetToolsRegistration, OnlyWithEveryHook) {
    ToolRegistry tools;
    SheetToolHooks hooks;
    hooks.info = [] { return json::object(); };
    hz::ai::registerSheetTools(tools, hooks);
    EXPECT_FALSE(tools.has("get_sheet_info"));
}

TEST_F(SheetToolsTest, ReadingIsFreeAndChangingIsOneStep) {
    EXPECT_EQ(tools.access("get_sheet_info"), hz::ai::ToolAccess::Read);
    EXPECT_EQ(tools.access("list_view_edges"), hz::ai::ToolAccess::Read);
    for (const char* tool : {"set_title_block", "set_sheet_scale", "move_view", "remove_view",
                             "set_view_options", "add_dimension", "remove_dimension"}) {
        EXPECT_EQ(tools.access(tool), hz::ai::ToolAccess::Modify) << tool;
    }
    EXPECT_EQ(made("get_sheet_info", json::object())["paper"], "a3");
    EXPECT_EQ(made("list_view_edges", {{"view", 2}})["edges"][0]["edge"], "e1");
    EXPECT_NE(refused("list_view_edges", {{"view", 9}}).find("no view 9"), std::string::npos);

    const ToolResult changed = tools.run("set_sheet_scale", {{"scale", "1:2"}});
    EXPECT_TRUE(changed.changedModel);
    EXPECT_EQ(changed.undoSteps, 1);
    // A change to nothing (the host took no step) is said to be so, and
    // counts no step: an undo of the response must not take one too many.
    answer.step = false;
    const ToolResult same = tools.run("set_sheet_scale", {{"scale", "1:2"}});
    EXPECT_FALSE(same.isError);
    EXPECT_EQ(same.undoSteps, 0);
    EXPECT_NE(same.content.find("nothing changed"), std::string::npos);
}

TEST_F(SheetToolsTest, TheTitleBlockAndPaper) {
    made("set_title_block", {{"title", "Bracket"},
                             {"material", "6061-T6"},
                             {"paper", "Letter"},
                             {"orientation", "portrait"}});
    ASSERT_EQ(asked.size(), 1u);
    const SheetChange& c = asked[0];
    EXPECT_EQ(c.kind, SheetChange::Kind::TitleBlock);
    EXPECT_EQ(c.paper, "ansi_a");
    EXPECT_EQ(c.orientation, "portrait");
    ASSERT_EQ(c.fields.size(), 2u);  // only those given
    EXPECT_EQ(c.fields[0], (std::pair<std::string, std::string>{"title", "Bracket"}));
    EXPECT_EQ(c.fields[1], (std::pair<std::string, std::string>{"material", "6061-T6"}));

    EXPECT_NE(refused("set_title_block", json::object()).find("give a title block field"),
              std::string::npos);
    EXPECT_NE(refused("set_title_block", {{"paper", "b5"}}).find("\"paper\" must be one of"),
              std::string::npos);
    EXPECT_NE(refused("set_title_block", {{"title", std::string(400, 'x')}}).find("longer"),
              std::string::npos);
    EXPECT_EQ(asked.size(), 1u);
}

TEST_F(SheetToolsTest, ScalesAndViews) {
    made("set_sheet_scale", {{"scale", "fit"}});
    made("set_sheet_scale", {{"scale", "2:1"}});
    ASSERT_EQ(asked.size(), 2u);
    EXPECT_EQ(asked[0].scale, 0.0);
    EXPECT_DOUBLE_EQ(asked[1].scale, 2.0);
    EXPECT_NE(refused("set_sheet_scale", {{"scale", "huge"}}).find("\"scale\""), std::string::npos);

    made("move_view", {{"view", 1}, {"by", {10, -5}}});
    made("move_view", {{"view", 2}, {"to", {150, 80}}});
    ASSERT_EQ(asked.size(), 4u);
    EXPECT_EQ(asked[2].kind, SheetChange::Kind::MoveView);
    EXPECT_EQ(asked[2].view, 1);
    EXPECT_FALSE(asked[2].to);
    EXPECT_DOUBLE_EQ(asked[2].x, 10.0);
    EXPECT_DOUBLE_EQ(asked[2].y, -5.0);
    EXPECT_TRUE(asked[3].to);
    EXPECT_DOUBLE_EQ(asked[3].x, 150.0);
    EXPECT_NE(refused("move_view", {{"view", 1}}).find("\"by\""), std::string::npos);
    EXPECT_NE(refused("move_view", {{"view", 1}, {"by", {1, 2}}, {"to", {3, 4}}}).find("\"to\""),
              std::string::npos);
    EXPECT_NE(refused("move_view", {{"view", 1}, {"by", {1e9, 0}}}).find("off any sheet"),
              std::string::npos);
    EXPECT_NE(refused("move_view", {{"view", -1}, {"by", {1, 0}}}).find("\"view\""),
              std::string::npos);

    made("set_view_options", {{"view", 0}, {"hidden_edges", false}});
    EXPECT_EQ(asked.back().hidden, 0);
    EXPECT_EQ(asked.back().tangent, -1);  // as it is
    EXPECT_EQ(asked.back().centreLines, -1);
    EXPECT_NE(refused("set_view_options", {{"view", 0}}).find("hidden_edges"), std::string::npos);

    made("remove_view", {{"view", 3}});
    EXPECT_EQ(asked.back().kind, SheetChange::Kind::RemoveView);
    EXPECT_EQ(asked.back().view, 3);
}

TEST_F(SheetToolsTest, DimensionsAndWhatTheHostSays) {
    made("add_dimension", {{"view", 0}, {"edge", "Extrude1:Edge4"}});
    made("remove_dimension", {{"view", 0}, {"edge", "Extrude1:Edge4"}});
    ASSERT_EQ(asked.size(), 2u);
    EXPECT_EQ(asked[0].kind, SheetChange::Kind::AddDimension);
    EXPECT_EQ(asked[0].edge, "Extrude1:Edge4");
    EXPECT_EQ(asked[1].kind, SheetChange::Kind::RemoveDimension);
    EXPECT_NE(refused("add_dimension", {{"view", 0}, {"edge", ""}}).find("\"edge\""),
              std::string::npos);

    answer = {};
    answer.error = "that edge is dimensioned already";
    EXPECT_NE(refused("add_dimension", {{"view", 0}, {"edge", "e"}}).find("dimensioned already"),
              std::string::npos);
    answer = {};
    answer.step = true;
    answer.note = "not every view has room on the sheet";
    EXPECT_EQ(made("set_sheet_scale", {{"scale", "5:1"}})["warning"],
              "not every view has room on the sheet");
}

TEST_F(SheetToolsTest, AHoleIsCalledOutByAnEdgeOfIt) {
    EXPECT_EQ(tools.access("add_hole_callout"), hz::ai::ToolAccess::Modify);
    made("add_hole_callout", {{"view", 1}, {"edge", "edge:hole_1/bore|primitive_1/top"}});
    ASSERT_EQ(asked.size(), 1u);
    EXPECT_EQ(asked[0].kind, SheetChange::Kind::HoleCallout);
    EXPECT_EQ(asked[0].view, 1);
    EXPECT_EQ(asked[0].edge, "edge:hole_1/bore|primitive_1/top");
    EXPECT_NE(refused("add_hole_callout", {{"view", 1}, {"edge", ""}}).find("an edge of the hole"),
              std::string::npos);
}
