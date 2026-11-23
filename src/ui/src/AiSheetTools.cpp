#include "horizon/ui/AiSheetTools.h"

#include <QFileInfo>
#include <QPointer>
#include <QString>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <map>
#include <nlohmann/json.hpp>
#include <utility>
#include <vector>

#include "horizon/document/Document.h"
#include "horizon/document/UndoStack.h"
#include "horizon/fileio/DrawingDocumentIO.h"
#include "horizon/modeling/DrawingDimension.h"
#include "horizon/modeling/DrawingProjection.h"
#include "horizon/modeling/DrawingView.h"
#include "horizon/modeling/Naming.h"
#include "horizon/modeling/TitleBlock.h"
#include "horizon/topology/Solid.h"
#include "horizon/ui/DrawingWorkbench.h"

namespace hz::ui {

namespace {

using nlohmann::json;

/// No more edges listed for one view than this: a model reads them all.
constexpr std::size_t kMaxEdges = 300;

const std::vector<std::pair<std::string, model::PaperSize>>& papers() {
    static const std::vector<std::pair<std::string, model::PaperSize>> list = {
        {"a0", model::PaperSize::A0},        {"a1", model::PaperSize::A1},
        {"a2", model::PaperSize::A2},        {"a3", model::PaperSize::A3},
        {"a4", model::PaperSize::A4},        {"ansi_a", model::PaperSize::AnsiA},
        {"ansi_b", model::PaperSize::AnsiB}, {"ansi_c", model::PaperSize::AnsiC},
        {"ansi_d", model::PaperSize::AnsiD}};
    return list;
}

double round1(double value) {
    return std::round(value * 10.0) / 10.0;
}

const char* kindName(model::StandardView kind) {
    switch (kind) {
        case model::StandardView::Front:
            return "front";
        case model::StandardView::Top:
            return "top";
        case model::StandardView::Right:
            return "right";
        case model::StandardView::Isometric:
            return "isometric";
    }
    return "view";
}

const char* roleName(model::ViewRole role) {
    switch (role) {
        case model::ViewRole::Section:
            return "section";
        case model::ViewRole::Detail:
            return "detail";
        case model::ViewRole::Projection:
            return "projection";
    }
    return "projection";
}

const char* dimensionName(io::DrawingDimensionSpec::Kind kind) {
    switch (kind) {
        case io::DrawingDimensionSpec::Kind::Radius:
            return "radius";
        case io::DrawingDimensionSpec::Kind::Diameter:
            return "diameter";
        case io::DrawingDimensionSpec::Kind::Length:
            return "length";
    }
    return "length";
}

json describe(const DrawingWorkbench& drawings, const doc::Document& sheet) {
    const io::DrawingDocumentSpec* spec = drawings.specOf(&sheet);
    const model::Drawing* drawn = drawings.drawnViews(&sheet);
    if (spec == nullptr || drawn == nullptr) return {{"error", "the tab shows no drawing sheet"}};
    const model::TitleBlock& tb = spec->titleBlock;
    json out = {
        {"part", QFileInfo(QString::fromStdString(spec->partPath)).fileName().toStdString()},
        {"paper", paperNameOf(spec->sheet.size)},
        {"orientation",
         spec->sheet.orientation == model::Orientation::Portrait ? "portrait" : "landscape"},
        {"size_mm", json::array({round1(spec->sheet.widthMm()), round1(spec->sheet.heightMm())})},
        {"title_block",
         {{"title", tb.title},
          {"part_number", tb.partNumber},
          {"revision", tb.revision},
          {"drawn_by", tb.drawnBy},
          {"date", tb.date},
          {"material", tb.material},
          {"company", tb.company},
          {"sheet_number", tb.sheetNumber},
          {"scale", tb.scale}}}};
    out["scale"] = spec->scale > 0.0
                       ? model::DrawingGenerator::scaleName(spec->scale)
                       : "fit (" + tb.scale + ", the largest standard scale that fits)";
    json views = json::array();
    for (std::size_t i = 0; i < drawn->views.size(); ++i) {
        const model::DrawingView& v = drawn->views[i];
        json view = {
            {"view", i},
            {"kind", v.role == model::ViewRole::Projection ? kindName(v.kind) : roleName(v.role)},
            {"scale", model::DrawingGenerator::scaleName(v.scale)},
            {"lower_left", json::array({round1(v.placement.x), round1(v.placement.y)})},
            {"size", json::array({round1(v.sheetWidth()), round1(v.sheetHeight())})},
            {"hidden_edges", v.showHidden},
            {"tangent_edges", v.showTangentEdges},
            {"centre_lines", v.showCentreLines}};
        if (!v.label.empty()) view["label"] = v.label;
        if (v.source >= 0) view["taken_from_view"] = v.source;
        json dimensions = json::array();
        if (i < spec->views.size()) {
            for (const io::DrawingDimensionSpec& d : spec->views[i].dimensions) {
                dimensions.push_back({{"edge", d.edge}, {"kind", dimensionName(d.kind)}});
            }
        }
        view["dimensions"] = std::move(dimensions);
        if (i < spec->views.size() && !spec->views[i].holeCallouts.empty()) {
            view["hole_callouts"] = spec->views[i].holeCallouts;
        }
        views.push_back(std::move(view));
    }
    out["views"] = std::move(views);
    return out;
}

/// Whether the curve named @p name closes on itself: every end of its chords
/// shared (a whole circle, not an arc).
bool closedCurve(const topo::Solid& solid, const std::string& name) {
    std::map<const topo::Vertex*, int> ends;
    for (const topo::Edge& e : solid.edges()) {
        if (model::logicalEdge(e.topoId.tag()) != name || e.halfEdge == nullptr ||
            e.halfEdge->twin == nullptr) {
            continue;
        }
        ++ends[e.halfEdge->origin];
        ++ends[e.halfEdge->twin->origin];
    }
    return !ends.empty() && std::all_of(ends.begin(), ends.end(),
                                        [](const auto& end) { return end.second % 2 == 0; });
}

json edgesOf(DrawingWorkbench& drawings, doc::Document& sheet, int index, std::string* error) {
    const model::Drawing* drawn = drawings.drawnViews(&sheet);
    if (drawn == nullptr) {
        *error = "the tab shows no drawing sheet";
        return nullptr;
    }
    if (index < 0 || static_cast<std::size_t>(index) >= drawn->views.size()) {
        *error = "the sheet has no view " + std::to_string(index) + " (get_sheet_info lists them)";
        return nullptr;
    }
    const topo::Solid* solid = drawings.solidOf(sheet, error);
    if (solid == nullptr) return nullptr;
    const std::vector<io::HoleNote>* holes = drawings.holesOf(sheet, error);
    if (holes == nullptr) return nullptr;
    const model::DrawingView& view = drawn->views[static_cast<std::size_t>(index)];

    // Each model edge once, by its name: a curve is many chords of one name.
    struct Seen {
        bool visible = false;
        math::Vec2 at;
        topo::TopologyID id;
    };
    std::vector<std::string> order;
    std::map<std::string, Seen> seen;
    for (const model::ProjectedEdge& e : view.edges) {
        const std::string name = model::logicalEdge(e.sourceEdge.tag());
        if (name.empty()) continue;
        auto [it, added] = seen.try_emplace(name);
        if (added) {
            order.push_back(name);
            const math::Vec2 a = view.toSheet(e.a);
            const math::Vec2 b = view.toSheet(e.b);
            it->second.at = {(a.x + b.x) / 2.0, (a.y + b.y) / 2.0};
            it->second.id = e.sourceEdge;
        }
        if (e.visibility == model::ProjectedEdge::Visibility::Visible) it->second.visible = true;
    }
    // Seen edges first: those are what a drawing dimensions.
    std::stable_sort(order.begin(), order.end(), [&seen](const auto& a, const auto& b) {
        return seen[a].visible && !seen[b].visible;
    });
    json edges = json::array();
    for (const std::string& name : order) {
        if (edges.size() >= kMaxEdges) break;
        const Seen& s = seen[name];
        json edge = {{"edge", name},
                     {"seen", s.visible},
                     {"at", json::array({round1(s.at.x), round1(s.at.y)})}};
        // An edge a Hole feature made: add_hole_callout calls the hole out.
        if (const io::HoleNote* hole = io::DrawingDocumentIO::holeOfEdge(*holes, name)) {
            edge["hole"] = hole->feature;
        }
        double measured = 0.0;
        if (model::DrawingDimensioner::measureRadius(*solid, s.id, measured)) {
            if (closedCurve(*solid, name)) {
                edge["is"] = "circle";
                edge["diameter_mm"] = round1(2.0 * measured);
            } else {
                edge["is"] = "arc";
                edge["radius_mm"] = round1(measured);
            }
        } else if (model::DrawingDimensioner::isStraight(*solid, name) &&
                   model::DrawingDimensioner::measureEdge(*solid, s.id, measured)) {
            edge["is"] = "straight";
            edge["length_mm"] = round1(measured);
        } else {
            edge["is"] = "curve";  // no one number says how large it is
        }
        edges.push_back(std::move(edge));
    }
    json out = {{"view", index}, {"edges", std::move(edges)}};
    if (order.size() > kMaxEdges) out["more"] = order.size() - kMaxEdges;
    return out;
}

ai::SheetChanged change(DrawingWorkbench& drawings, doc::Document& sheet,
                        const ai::SheetChange& c) {
    ai::SheetChanged out;
    const io::DrawingDocumentSpec* spec = drawings.specOf(&sheet);
    const model::Drawing* drawn = drawings.drawnViews(&sheet);
    if (spec == nullptr || drawn == nullptr) {
        out.error = "the tab shows no drawing sheet";
        return out;
    }
    // A step is one the undo stack took: a change to nothing takes none.
    const auto revision = sheet.undoStack().revision();
    const auto view = static_cast<std::size_t>(c.view);  // -1: no view, which is refused
    const bool known = c.view >= 0 && view < drawn->views.size();
    std::string error;
    bool fits = true;
    bool made = false;
    switch (c.kind) {
        case ai::SheetChange::Kind::TitleBlock: {
            model::TitleBlock block = spec->titleBlock;
            for (const auto& [key, value] : c.fields) {
                if (key == "title") block.title = value;
                if (key == "part_number") block.partNumber = value;
                if (key == "revision") block.revision = value;
                if (key == "drawn_by") block.drawnBy = value;
                if (key == "date") block.date = value;
                if (key == "material") block.material = value;
                if (key == "company") block.company = value;
                if (key == "sheet_number") block.sheetNumber = value;
            }
            model::Sheet paper = spec->sheet;
            if (!c.paper.empty()) {
                const auto size = paperSizeNamed(c.paper);
                if (!size) {
                    out.error = "no paper is called \"" + c.paper + "\"";
                    return out;
                }
                paper.size = *size;
            }
            if (!c.orientation.empty()) {
                paper.orientation = c.orientation == "portrait" ? model::Orientation::Portrait
                                                                : model::Orientation::Landscape;
            }
            made = drawings.setTitleBlock(sheet, block, paper, &fits, &error);
            break;
        }
        case ai::SheetChange::Kind::Scale:
            made = drawings.setScale(sheet, c.scale, &fits, &error);
            break;
        case ai::SheetChange::Kind::MoveView: {
            math::Vec2 by{c.x, c.y};
            if (c.to && known) {
                const math::Vec2& at = drawn->views[view].placement;
                by = {c.x - at.x, c.y - at.y};
            }
            made = drawings.moveView(sheet, view, by, &error);
            break;
        }
        case ai::SheetChange::Kind::RemoveView: {
            int taken = 0;
            made = drawings.removeView(sheet, view, &taken, &error);
            if (made && taken > 0) {
                out.note = std::to_string(taken) +
                           " view(s) taken from it went with it; the views are numbered anew";
            }
            break;
        }
        case ai::SheetChange::Kind::ViewSwitches: {
            if (!known) {
                made = drawings.setViewSwitches(sheet, view, true, true, true, &error);
                break;
            }
            const model::DrawingView& v = drawn->views[view];
            const auto pick = [](int given, bool now) { return given < 0 ? now : given == 1; };
            made = drawings.setViewSwitches(sheet, view, pick(c.hidden, v.showHidden),
                                            pick(c.tangent, v.showTangentEdges),
                                            pick(c.centreLines, v.showCentreLines), &error);
            break;
        }
        case ai::SheetChange::Kind::AddDimension:
            made = drawings.addEdgeDimension(sheet, view, c.edge, &error);
            break;
        case ai::SheetChange::Kind::RemoveDimension:
            made = drawings.removeEdgeDimensions(sheet, view, c.edge, &error);
            break;
        case ai::SheetChange::Kind::HoleCallout:
            made = drawings.addHoleCallout(sheet, view, c.edge, &error);
            break;
    }
    if (!made) {
        out.error = error.empty() ? std::string("the change could not be made") : error;
        return out;
    }
    out.step = sheet.undoStack().revision() != revision;
    if (!fits) {
        out.note = "not every view has room on the sheet: those without are placed beside it";
    }
    return out;
}

}  // namespace

std::optional<model::PaperSize> paperSizeNamed(const std::string& name) {
    for (const auto& [known, size] : papers()) {
        if (known == name) return size;
    }
    return std::nullopt;
}

std::string paperNameOf(model::PaperSize size) {
    for (const auto& [name, known] : papers()) {
        if (known == size) return name;
    }
    return model::paperSizeName(size);
}

ai::SheetToolHooks sheetToolHooks(DrawingWorkbench& drawings, std::weak_ptr<doc::Document> sheet) {
    const QPointer<DrawingWorkbench> workbench(&drawings);
    ai::SheetToolHooks hooks;
    hooks.info = [workbench, sheet]() -> json {
        const auto shown = sheet.lock();
        if (!workbench || !shown) return {{"error", "the sheet is closed"}};
        return describe(*workbench, *shown);
    };
    hooks.edges = [workbench, sheet](int view, std::string* error) -> json {
        const auto shown = sheet.lock();
        if (!workbench || !shown) {
            *error = "the sheet is closed";
            return nullptr;
        }
        return edgesOf(*workbench, *shown, view, error);
    };
    hooks.change = [workbench, sheet](const ai::SheetChange& c) {
        const auto shown = sheet.lock();
        if (!workbench || !shown) {
            ai::SheetChanged closed;
            closed.error = "the sheet is closed";
            return closed;
        }
        return change(*workbench, *shown, c);
    };
    return hooks;
}

}  // namespace hz::ui
