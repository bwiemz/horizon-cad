// make_drawing: a drawing sheet of the part or the assembly, made by the
// host (the window) in a tab of its own. The tool reads and checks the
// request; the sheet is the host's, as the menu's New Drawing from Part is.

#include <cmath>
#include <cstddef>
#include <functional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "CadToolsCommon.h"
#include "horizon/ai/CadTools.h"

namespace hz::ai::detail {

namespace {

/// The longest a title block field may be: a line of the block, not a page.
constexpr std::size_t kMaxField = 120;
/// No drawing is made at a scale beyond these (sheet mm per model mm).
constexpr double kMinScale = 0.001;
constexpr double kMaxScale = 1000.0;

/// A scale as a model gives it: "1:2", "2:1", "1", or a number. 0 when none.
double scaleOf(const json& in) {
    if (!has(in, "scale")) return 0.0;
    double scale = 0.0;
    const json& given = in["scale"];
    if (given.is_number()) {
        scale = given.get<double>();
    } else if (given.is_string()) {
        const std::string text = given.get<std::string>();
        const std::size_t colon = text.find(':');
        try {
            std::size_t used = 0;
            if (colon == std::string::npos) {
                scale = std::stod(text, &used);
                if (used != text.size()) scale = 0.0;
            } else {
                const std::string sheet = text.substr(0, colon);
                const std::string model = text.substr(colon + 1);
                std::size_t usedModel = 0;
                const double s = std::stod(sheet, &used);
                const double m = std::stod(model, &usedModel);
                if (used == sheet.size() && usedModel == model.size() && m > 0.0) scale = s / m;
            }
        } catch (const std::exception&) {
            scale = 0.0;
        }
    }
    if (!std::isfinite(scale) || scale < kMinScale || scale > kMaxScale) {
        throw ArgError(
            "\"scale\" must be a drawing scale such as \"1:2\", \"1:1\" or \"2:1\" (sheet:model), "
            "or leave it out for the largest standard scale that fits");
    }
    return scale;
}

std::string field(const json& in, const char* key) {
    std::string value = text(in, key, "");
    if (value.size() > kMaxField) {
        throw ArgError(std::string("\"") + key + "\" is longer than a title block holds (" +
                       std::to_string(kMaxField) + " characters)");
    }
    return value;
}

ToolResult makeDrawing(const MakeDrawing& make, const DrawingSource& source, const json& in) {
    const std::string& what = source.what;
    if (source.empty) {
        return ToolResult::error("The " + what + " has nothing to draw yet. Make it first.");
    }
    if (source.path.empty()) {
        return ToolResult::error(
            "The " + what +
            " has not been saved. A drawing refers to its file. Ask the user to save it "
            "(File > Save), then make the drawing.");
    }

    DrawingRequest request;
    request.source = source.path;
    std::string paper = choice(
        in, "paper",
        {"a0", "a1", "a2", "a3", "a4", "ansi_a", "ansi_b", "ansi_c", "ansi_d", "letter", "tabloid"},
        std::string("a3"));
    if (paper == "letter") paper = "ansi_a";
    if (paper == "tabloid") paper = "ansi_b";
    request.paper = paper;
    request.portrait = choice(in, "orientation", {"landscape", "portrait"},
                              std::string("landscape")) == "portrait";
    request.scale = scaleOf(in);
    request.title = field(in, "title");
    request.partNumber = field(in, "part_number");
    request.revision = field(in, "revision");
    request.drawnBy = field(in, "drawn_by");
    request.date = field(in, "date");
    request.material = field(in, "material");
    request.company = field(in, "company");

    const DrawingMade made = make(request);
    if (!made.error.empty()) return ToolResult::error(made.error);
    std::string note =
        "The drawing is open in a tab of its own, behind this one, and is not saved. Tell the "
        "user they can switch to it to look at it, and save it (File > Save). It is drawn from "
        "the " +
        what + " as this tab has it, and follows its changes.";
    if (source.unsaved) {
        note += " The " + what +
                " has changes that are not saved: the user should save it too, or the saved "
                "drawing will show it as its file has it when it is opened again.";
    }
    return ToolResult::ok({{"tab", made.tab},
                           {"paper", paper},
                           {"orientation", request.portrait ? "portrait" : "landscape"},
                           {"scale", made.scale},
                           {"views", made.views},
                           {"note", note}});
}

}  // namespace

void registerDrawingTool(ToolRegistry& registry, MakeDrawing make,
                         std::function<DrawingSource()> source) {
    if (!make || !source) return;
    registry.add(
        {"make_drawing",
         "Make an engineering drawing sheet of this part or assembly: its front, top, right and "
         "isometric views (third-angle projection) laid out over a title block, at the largest "
         "standard scale that fits unless a scale is given. It opens in a tab of its own, "
         "unsaved, for the user to look at and save. The part or assembly must have been saved "
         "once: the sheet refers to its file.",
         object({{"paper", enumSchema({"a0", "a1", "a2", "a3", "a4", "ansi_a", "ansi_b", "ansi_c",
                                       "ansi_d", "letter", "tabloid"},
                                      "paper size (default a3; letter is ansi_a, tabloid ansi_b)")},
                 {"orientation", enumSchema({"landscape", "portrait"}, "default landscape")},
                 {"scale", stringSchema("sheet:model, such as \"1:2\" or \"2:1\"; leave it out "
                                        "for the largest standard scale that fits")},
                 {"title", stringSchema("the title block's title (default: the file's name)")},
                 {"part_number", stringSchema("part or drawing number")},
                 {"revision", stringSchema("revision (default A)")},
                 {"drawn_by", stringSchema("who drew it")},
                 {"date", stringSchema("date drawn")},
                 {"material", stringSchema("material")},
                 {"company", stringSchema("company or organisation")}})},
        ToolAccess::Read, [make = std::move(make), source = std::move(source)](const json& in) {
            return makeDrawing(make, source(), in);
        });
}

}  // namespace hz::ai::detail
