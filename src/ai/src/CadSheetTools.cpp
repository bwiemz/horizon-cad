// The drawing sheet tools: what a sheet shows, and the Drawing menu's
// changes to it, named rather than clicked. The host (the window) makes
// each change, one undo step, as the menu's command does.

#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "CadToolsCommon.h"
#include "horizon/ai/CadTools.h"

namespace hz::ai {

namespace {

using detail::ArgError;
using detail::boolean;
using detail::choice;
using detail::has;
using detail::integer;
using detail::number;
using detail::object;
using detail::text;
using nlohmann::json;

/// No view number past this: a sheet has a handful.
constexpr long long kMaxView = 999;
/// No move larger than this, in sheet millimetres: an A0 sheet is 1189 long.
constexpr double kMaxMove = 5000.0;

const std::vector<std::string> kPapers = {
    "a0", "a1", "a2", "a3", "a4", "ansi_a", "ansi_b", "ansi_c", "ansi_d", "letter", "tabloid"};
const std::vector<std::string> kFields = {"title", "part_number", "revision", "drawn_by",
                                          "date",  "material",    "company",  "sheet_number"};

int viewOf(const json& in) {
    return static_cast<int>(integer(in, "view", 0, kMaxView));
}

/// A switch given as true or false: 1 or 0; -1 when not given.
int switchOf(const json& in, const char* key) {
    if (!has(in, key)) return -1;
    return boolean(in, key, false) ? 1 : 0;
}

/// A point given as [x, y] in sheet millimetres.
std::pair<double, double> pointOf(const json& in, const char* key) {
    const json& given = in[key];
    if (!given.is_array() || given.size() != 2 || !given[0].is_number() || !given[1].is_number()) {
        throw ArgError(std::string("\"") + key + "\" must be [x, y] in sheet millimetres");
    }
    const double x = given[0].get<double>();
    const double y = given[1].get<double>();
    if (!(std::abs(x) <= kMaxMove) || !(std::abs(y) <= kMaxMove)) {
        throw ArgError(std::string("\"") + key + "\" is off any sheet");
    }
    return {x, y};
}

ToolResult apply(const SheetToolHooks& hooks, const SheetChange& change, json done) {
    const SheetChanged changed = hooks.change(change);
    if (!changed.error.empty()) return ToolResult::error(changed.error);
    if (!changed.step) done["note"] = "nothing changed: the sheet was already so";
    if (!changed.note.empty()) done["warning"] = changed.note;
    return ToolResult::ok(done, changed.step);
}

ToolResult setTitleBlock(const SheetToolHooks& hooks, const json& in) {
    SheetChange change;
    change.kind = SheetChange::Kind::TitleBlock;
    for (const std::string& key : kFields) {
        if (has(in, key.c_str())) {
            change.fields.emplace_back(key, detail::titleBlockField(in, key.c_str()));
        }
    }
    if (has(in, "paper")) {
        change.paper = choice(in, "paper", kPapers, std::nullopt);
        if (change.paper == "letter") change.paper = "ansi_a";
        if (change.paper == "tabloid") change.paper = "ansi_b";
    }
    if (has(in, "orientation")) {
        change.orientation = choice(in, "orientation", {"landscape", "portrait"}, std::nullopt);
    }
    if (change.fields.empty() && change.paper.empty() && change.orientation.empty()) {
        throw ArgError("give a title block field, \"paper\" or \"orientation\" to change");
    }
    return apply(hooks, change, {{"changed", "title block"}});
}

ToolResult setScale(const SheetToolHooks& hooks, const json& in) {
    if (!has(in, "scale")) throw ArgError("\"scale\" is required");
    SheetChange change;
    change.kind = SheetChange::Kind::Scale;
    const json& given = in["scale"];
    if (given.is_string() &&
        (given.get<std::string>() == "fit" || given.get<std::string>() == "")) {
        change.scale = 0.0;
    } else {
        change.scale = detail::drawingScale(in, "scale");
    }
    return apply(hooks, change, {{"changed", "scale"}});
}

ToolResult moveView(const SheetToolHooks& hooks, const json& in) {
    SheetChange change;
    change.kind = SheetChange::Kind::MoveView;
    change.view = viewOf(in);
    if (has(in, "to") == has(in, "by")) {
        throw ArgError(
            "give \"by\" ([dx, dy], sheet mm) or \"to\" ([x, y], the view's lower-left corner)");
    }
    change.to = has(in, "to");
    const auto [x, y] = pointOf(in, change.to ? "to" : "by");
    change.x = x;
    change.y = y;
    return apply(hooks, change, {{"changed", "view " + std::to_string(change.view) + " moved"}});
}

ToolResult removeView(const SheetToolHooks& hooks, const json& in) {
    SheetChange change;
    change.kind = SheetChange::Kind::RemoveView;
    change.view = viewOf(in);
    return apply(hooks, change,
                 {{"changed", "view " + std::to_string(change.view) +
                                  " removed, with any view taken from it; the views after "
                                  "it are numbered one less"}});
}

ToolResult setViewOptions(const SheetToolHooks& hooks, const json& in) {
    SheetChange change;
    change.kind = SheetChange::Kind::ViewSwitches;
    change.view = viewOf(in);
    change.hidden = switchOf(in, "hidden_edges");
    change.tangent = switchOf(in, "tangent_edges");
    change.centreLines = switchOf(in, "centre_lines");
    if (change.hidden < 0 && change.tangent < 0 && change.centreLines < 0) {
        throw ArgError("give \"hidden_edges\", \"tangent_edges\" or \"centre_lines\"");
    }
    return apply(hooks, change, {{"changed", "view " + std::to_string(change.view) + " options"}});
}

ToolResult dimension(const SheetToolHooks& hooks, const json& in, bool add) {
    SheetChange change;
    change.kind = add ? SheetChange::Kind::AddDimension : SheetChange::Kind::RemoveDimension;
    change.view = viewOf(in);
    change.edge = text(in, "edge");
    if (change.edge.empty()) throw ArgError("\"edge\" is required (list_view_edges names them)");
    return apply(
        hooks, change,
        {{"changed",
          std::string(add ? "dimension added to " : "dimensions removed from ") + change.edge}});
}

ToolResult holeCallout(const SheetToolHooks& hooks, const json& in) {
    SheetChange change;
    change.kind = SheetChange::Kind::HoleCallout;
    change.view = viewOf(in);
    change.edge = text(in, "edge");
    if (change.edge.empty()) {
        throw ArgError("\"edge\" is required: an edge of the hole, as list_view_edges names it");
    }
    return apply(hooks, change,
                 {{"changed", "hole called out in view " + std::to_string(change.view)}});
}

}  // namespace

void registerSheetTools(ToolRegistry& registry, SheetToolHooks hooks) {
    if (!hooks.info || !hooks.edges || !hooks.change) return;
    const auto shared = std::make_shared<SheetToolHooks>(std::move(hooks));
    const auto add = [&registry, &shared](
                         ToolSpec spec, ToolAccess access,
                         ToolResult (*handler)(const SheetToolHooks&, const json&)) {
        registry.add(std::move(spec), access,
                     [shared, handler](const json& in) { return handler(*shared, in); });
    };
    const json viewSchema =
        detail::integerSchema("the view's number, as get_sheet_info numbers them (from 0)");

    registry.add({"get_sheet_info",
                  "What the drawing sheet shows: its paper and size in mm, its scale, its title "
                  "block, and its views, numbered from 0: each one's kind (front, top, right, "
                  "isometric; a section or a detail), label, scale, lower-left corner and size "
                  "on the sheet in mm, what it shows (hidden edges, tangent edges, centre "
                  "lines) and its dimensions. Sheet coordinates: mm from the lower-left corner "
                  "of the paper.",
                  object(json::object())},
                 ToolAccess::Read,
                 [shared](const json&) { return ToolResult::ok(shared->info()); });

    registry.add(
        {"list_view_edges",
         "The part's edges one view shows, by name, for add_dimension: each one's name, whether "
         "it is seen or hidden, what it is (straight, circle, arc or curve), what a dimension "
         "of it would say (a length, a circle's diameter, an arc's radius, in mm) and where it "
         "is on the sheet (its midpoint).",
         object({{"view", viewSchema}}, {"view"})},
        ToolAccess::Read, [shared](const json& in) {
            std::string error;
            json edges = shared->edges(viewOf(in), &error);
            if (!error.empty()) return ToolResult::error(error);
            return ToolResult::ok(edges);
        });

    json fieldSchemas = {
        {"paper", detail::enumSchema(kPapers,
                                     "paper size; on new paper the views are laid "
                                     "out again")},
        {"orientation", detail::enumSchema({"landscape", "portrait"}, "paper orientation")}};
    for (const std::string& key : kFields) {
        fieldSchemas[key] = detail::stringSchema("the title block's " + key);
    }
    add({"set_title_block",
         "Change the sheet's title block fields (those given; the others stay), and its paper "
         "or orientation. One undo step.",
         object(std::move(fieldSchemas))},
        ToolAccess::Modify, &setTitleBlock);
    add({"set_sheet_scale",
         "Lay the sheet's views out again at a scale: \"1:2\", \"2:1\" (sheet:model), or "
         "\"fit\" for the largest standard scale that fits. Sections and details are kept. "
         "One undo step.",
         object({{"scale", detail::stringSchema("\"1:2\", \"1:1\", \"2:1\"... or \"fit\"")}},
                {"scale"})},
        ToolAccess::Modify, &setScale);
    add({"move_view",
         "Move a view on the sheet, by [dx, dy] or to [x, y] (its lower-left corner), in sheet "
         "mm. Notes drawn in it move with it. One undo step.",
         object({{"view", viewSchema},
                 {"by", detail::arraySchema(detail::numberSchema("mm"), "[dx, dy], sheet mm")},
                 {"to", detail::arraySchema(detail::numberSchema("mm"),
                                            "[x, y], the view's new lower-left corner")}},
                {"view"})},
        ToolAccess::Modify, &moveView);
    add({"remove_view",
         "Remove a view from the sheet, and any section or detail taken from it. The views "
         "after it are numbered one less. A sheet keeps at least one view. One undo step.",
         object({{"view", viewSchema}}, {"view"})},
        ToolAccess::Modify, &removeView);
    add({"set_view_options",
         "Show or leave out a view's hidden edges, tangent edges (where a fillet meets a face) "
         "and centre lines. Those not given stay as they are. One undo step.",
         object({{"view", viewSchema},
                 {"hidden_edges", detail::booleanSchema("show hidden edges (dashed)")},
                 {"tangent_edges", detail::booleanSchema("show tangent edges")},
                 {"centre_lines", detail::booleanSchema("show centre lines")}},
                {"view"})},
        ToolAccess::Modify, &setViewOptions);
    add({"add_dimension",
         "Dimension an edge in a view, measured from the part and kept up to date with it: a "
         "straight edge's length, a circle's diameter, an arc's radius. Name the edge as "
         "list_view_edges does. One undo step.",
         object({{"view", viewSchema},
                 {"edge", detail::stringSchema("the edge's name from list_view_edges")}},
                {"view", "edge"})},
        ToolAccess::Modify,
        [](const SheetToolHooks& h, const json& in) { return dimension(h, in, true); });
    add({"add_hole_callout",
         "Call out a hole in a view: its size and depth as its Hole feature says, with a "
         "counterbore's or countersink's, and how many of it the part has (\"3× ⌀6.6 THRU\"), "
         "on a leader from the hole. Name an edge of the hole as list_view_edges does (one "
         "marked with its \"hole\"); call it out in the view that looks along the hole, where "
         "it is a circle. One undo step.",
         object({{"view", viewSchema},
                 {"edge", detail::stringSchema("an edge of the hole, from list_view_edges")}},
                {"view", "edge"})},
        ToolAccess::Modify, &holeCallout);
    add({"remove_dimension",
         "Remove the dimensions of an edge in a view, and its hole's callout. One undo step.",
         object({{"view", viewSchema},
                 {"edge", detail::stringSchema("the edge's name, as get_sheet_info lists it")}},
                {"view", "edge"})},
        ToolAccess::Modify,
        [](const SheetToolHooks& h, const json& in) { return dimension(h, in, false); });
}

}  // namespace hz::ai
