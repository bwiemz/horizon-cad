// Tools for the 2D drawing: shapes and text on layers, and moving, copying,
// turning, scaling, mirroring and deleting what is there. Like every tool
// that touches entities, they leave those on hidden or locked layers alone.

#include <algorithm>
#include <cctype>
#include <cmath>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "CadToolsCommon.h"
#include "horizon/ai/ProfileBuilder.h"
#include "horizon/document/Commands.h"
#include "horizon/document/Document.h"
#include "horizon/document/UndoStack.h"
#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftDocument.h"
#include "horizon/drafting/DraftEntity.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftPolyline.h"
#include "horizon/drafting/DraftRectangle.h"
#include "horizon/drafting/DraftText.h"
#include "horizon/drafting/Layer.h"
#include "horizon/math/BoundingBox.h"

namespace hz::ai::detail {

json describeEntity(const draft::DraftEntity& entity) {
    json out = {{"id", entity.id()}, {"layer", entity.layer()}};
    if (entity.construction()) out["construction"] = true;
    if (const auto* line = dynamic_cast<const draft::DraftLine*>(&entity)) {
        out["type"] = "line";
        out["from"] = toJson(line->start());
        out["to"] = toJson(line->end());
    } else if (const auto* circle = dynamic_cast<const draft::DraftCircle*>(&entity)) {
        out["type"] = "circle";
        out["center"] = toJson(circle->center());
        out["radius"] = round6(circle->radius());
    } else if (const auto* arc = dynamic_cast<const draft::DraftArc*>(&entity)) {
        out["type"] = "arc";
        out["center"] = toJson(arc->center());
        out["radius"] = round6(arc->radius());
        out["start_angle"] = round6(arc->startAngle() / kDegree);
        out["end_angle"] = round6(arc->endAngle() / kDegree);
    } else if (const auto* polyline = dynamic_cast<const draft::DraftPolyline*>(&entity)) {
        out["type"] = "polyline";
        json points = json::array();
        for (const auto& p : polyline->points()) points.push_back(toJson(p));
        out["points"] = std::move(points);
        out["closed"] = polyline->closed();
    } else if (const auto* rectangle = dynamic_cast<const draft::DraftRectangle*>(&entity)) {
        out["type"] = "rectangle";
        out["corner1"] = toJson(rectangle->corner1());
        out["corner2"] = toJson(rectangle->corner2());
    } else if (const auto* label = dynamic_cast<const draft::DraftText*>(&entity)) {
        out["type"] = "text";
        out["position"] = toJson(label->position());
        out["text"] = label->text();
        out["height"] = round6(label->textHeight());
    } else {
        out["type"] = "other";
        const math::BoundingBox box = entity.boundingBox();
        if (box.isValid()) {
            out["bounds"] = {{"min", toJson(math::Vec2(box.min().x, box.min().y))},
                             {"max", toJson(math::Vec2(box.max().x, box.max().y))}};
        }
    }
    return out;
}

namespace {

using math::Vec2;

/// "#RRGGBB" or a few colour names, as ARGB.
std::uint32_t colorOf(const json& in, const char* key) {
    std::string value = text(in, key);
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    static const std::map<std::string, std::uint32_t> names = {
        {"white", 0xFFFFFF}, {"black", 0x000000},  {"red", 0xFF0000},   {"green", 0x00C000},
        {"blue", 0x0000FF},  {"yellow", 0xFFFF00}, {"cyan", 0x00FFFF},  {"magenta", 0xFF00FF},
        {"gray", 0x808080},  {"grey", 0x808080},   {"orange", 0xFF8000}};
    if (const auto it = names.find(value); it != names.end()) return 0xFF000000u | it->second;
    if (value.size() == 7 && value[0] == '#' &&
        std::all_of(value.begin() + 1, value.end(),
                    [](char c) { return std::isxdigit(static_cast<unsigned char>(c)); })) {
        return 0xFF000000u | static_cast<std::uint32_t>(std::stoul(value.substr(1), nullptr, 16));
    }
    throw ArgError(std::string("\"") + key + "\" must be \"#RRGGBB\" or a colour name");
}

const draft::LayerProperties* layerOf(const doc::Document& document, const std::string& name) {
    return document.layerManager().getLayer(name);
}

/// The ids in @p key that are entities of the drawing on a visible, unlocked
/// layer; the others are listed in @p skipped with why.
std::vector<std::uint64_t> editableIds(const doc::Document& document, const json& in,
                                       const char* key, json& skipped) {
    if (!has(in, key) || !in[key].is_array() || in[key].empty()) {
        throw ArgError(std::string("\"") + key + "\" must be a list of entity ids");
    }
    std::vector<std::uint64_t> ids;
    for (const auto& item : in[key]) {
        const json wrapped = {{"id", item}};
        const std::uint64_t entityId = id(wrapped, "id");
        const draft::DraftEntity* entity = document.draftDocument().findEntity(entityId);
        if (entity == nullptr) {
            skipped.push_back({{"id", entityId}, {"why", "no such entity"}});
            continue;
        }
        const auto* layer = layerOf(document, entity->layer());
        if (layer == nullptr || !layer->visible || layer->locked) {
            skipped.push_back({{"id", entityId}, {"why", "its layer is hidden or locked"}});
            continue;
        }
        if (std::find(ids.begin(), ids.end(), entityId) == ids.end()) ids.push_back(entityId);
    }
    return ids;
}

// --- draw_2d ------------------------------------------------------------------

ToolResult draw2d(const CadContext& context, const json& in) {
    doc::Document& document = context.document;
    if (!has(in, "entities")) throw ArgError("\"entities\" is required");
    auto entities = buildEntities(in["entities"], true);
    const std::string layer = text(in, "layer", document.layerManager().currentLayer());
    if (layer.empty()) throw ArgError("\"layer\" is empty");

    auto composite = std::make_unique<doc::CompositeCommand>("Draw");
    bool createdLayer = false;
    if (const auto* existing = layerOf(document, layer)) {
        if (existing->locked) throw ArgError("layer \"" + layer + "\" is locked");
        if (!existing->visible) throw ArgError("layer \"" + layer + "\" is hidden");
    } else {
        draft::LayerProperties props;
        props.name = layer;
        composite->addCommand(
            std::make_unique<doc::AddLayerCommand>(document.layerManager(), props));
        createdLayer = true;
    }
    json ids = json::array();
    for (auto& entity : entities) {
        entity->setLayer(layer);
        ids.push_back(entity->id());
        composite->addCommand(
            std::make_unique<doc::AddEntityCommand>(document.draftDocument(), entity));
    }
    document.undoStack().push(std::move(composite));
    context.drawingChanged();
    json out = {{"ids", std::move(ids)}, {"layer", layer}};
    if (createdLayer) out["created_layer"] = true;
    return ToolResult::ok(out, true);
}

// --- create_layer -------------------------------------------------------------

ToolResult createLayer(const CadContext& context, const json& in) {
    doc::Document& document = context.document;
    const std::string name = text(in, "name");
    if (name.empty()) throw ArgError("\"name\" is empty");
    if (layerOf(document, name) != nullptr) throw ArgError("layer \"" + name + "\" already exists");
    draft::LayerProperties props;
    props.name = name;
    if (has(in, "color")) props.color = colorOf(in, "color");
    props.lineWidth = positive(in, "line_width", props.lineWidth);
    auto composite = std::make_unique<doc::CompositeCommand>("New Layer");
    composite->addCommand(std::make_unique<doc::AddLayerCommand>(document.layerManager(), props));
    const bool current = boolean(in, "make_current", false);
    if (current) {
        composite->addCommand(
            std::make_unique<doc::SetCurrentLayerCommand>(document.layerManager(), name));
    }
    document.undoStack().push(std::move(composite));
    context.drawingChanged();
    return ToolResult::ok(json{{"layer", name}, {"current", current}}, true);
}

// --- list_entities ------------------------------------------------------------

ToolResult listEntities(const CadContext& context, const json& in) {
    const doc::Document& document = context.document;
    const std::string layer = text(in, "layer", "");
    const auto offset = static_cast<std::size_t>(integer(in, "offset", 0, 1000000000, 0));
    const auto max = static_cast<std::size_t>(integer(in, "max", 1, 2000, 100));
    json entities = json::array();
    std::size_t matched = 0;
    for (const auto& entity : document.draftDocument().entities()) {
        if (!layer.empty() && entity->layer() != layer) continue;
        if (matched++ < offset) continue;
        if (entities.size() < max) entities.push_back(describeEntity(*entity));
    }
    json out = {{"entities", std::move(entities)}, {"total", matched}};
    if (matched > offset + max) out["more"] = matched - offset - max;
    return ToolResult::ok(out);
}

// --- delete_entities / transform_entities ------------------------------------

ToolResult deleteEntities(const CadContext& context, const json& in) {
    doc::Document& document = context.document;
    json skipped = json::array();
    const auto ids = editableIds(document, in, "ids", skipped);
    if (ids.empty()) return ToolResult::error("Nothing was deleted: " + skipped.dump());
    document.undoStack().push(
        std::make_unique<doc::RemoveEntitiesCommand>(document.draftDocument(), ids));
    context.drawingChanged();
    json out = {{"deleted", ids.size()}};
    if (!skipped.empty()) out["skipped"] = std::move(skipped);
    return ToolResult::ok(out, true);
}

ToolResult transformEntities(const CadContext& context, const json& in) {
    doc::Document& document = context.document;
    draft::DraftDocument& drawing = document.draftDocument();
    const std::string operation =
        choice(in, "operation", {"move", "copy", "rotate", "scale", "mirror"});
    json skipped = json::array();
    const auto ids = editableIds(document, in, "ids", skipped);
    if (ids.empty()) return ToolResult::error("Nothing to transform: " + skipped.dump());
    std::set<std::uint64_t> existed;
    for (const auto& entity : drawing.entities()) existed.insert(entity->id());
    if (operation == "move") {
        document.undoStack().push(std::make_unique<doc::MoveEntityCommand>(
            drawing, ids, Vec2(number(in, "dx", 0.0), number(in, "dy", 0.0)),
            document.constraintSystem(), document.variableResolver()));
    } else if (operation == "copy") {
        document.undoStack().push(std::make_unique<doc::DuplicateEntityCommand>(
            drawing, ids, Vec2(number(in, "dx", 0.0), number(in, "dy", 0.0))));
    } else {
        // The drawing's rotate, scale and mirror add transformed copies. A
        // turned or scaled entity replaces the original unless it is kept; a
        // mirror keeps it unless told not to.
        const Vec2 center = has(in, "center") ? vec2(in, "center") : Vec2(0.0, 0.0);
        auto composite = std::make_unique<doc::CompositeCommand>("Transform");
        bool keep = boolean(in, "keep_original", false);
        if (operation == "rotate") {
            composite->addCommand(std::make_unique<doc::RotateEntityCommand>(
                drawing, ids, center, number(in, "angle") * kDegree));
        } else if (operation == "scale") {
            composite->addCommand(std::make_unique<doc::ScaleEntityCommand>(
                drawing, ids, center, positive(in, "factor")));
        } else {
            const Vec2 a = vec2(in, "axis_start");
            const Vec2 b = vec2(in, "axis_end");
            if ((b - a).length() < 1e-12) throw ArgError("the mirror axis has no length");
            composite->addCommand(std::make_unique<doc::MirrorEntityCommand>(drawing, ids, a, b));
            keep = boolean(in, "keep_original", true);
        }
        if (!keep) {
            composite->addCommand(std::make_unique<doc::RemoveEntitiesCommand>(drawing, ids));
        }
        document.undoStack().push(std::move(composite));
    }
    context.drawingChanged();
    json out = {{"operation", operation}, {"entities", ids.size()}};
    // What was added: the copies, or the entities that replaced the originals.
    json added = json::array();
    for (const auto& entity : drawing.entities()) {
        if (!existed.count(entity->id())) added.push_back(entity->id());
    }
    if (!added.empty()) out["new_ids"] = std::move(added);
    if (!skipped.empty()) out["skipped"] = std::move(skipped);
    return ToolResult::ok(out, true);
}

json idsSchema() {
    return arraySchema(json{{"type", "integer"}}, "entity ids from list_entities or draw_2d");
}

}  // namespace

void registerDraftTools(ToolRegistry& registry, const Context& context) {
    registry.add({"draw_2d",
                  "Draw 2D shapes and text into the drawing (not a sketch), on a layer (made if "
                  "it does not exist; default the current layer). Coordinates in mm. Returns the "
                  "new entity ids.",
                  object({{"entities", arraySchema(entitySchema(true), "the shapes")},
                          {"layer", stringSchema("layer name")}},
                         {"entities"})},
                 ToolAccess::Modify, [context](const json& in) { return draw2d(*context, in); });

    registry.add({"create_layer",
                  "Make a drawing layer with a colour (\"#RRGGBB\" or a name) and line width.",
                  object({{"name", stringSchema("")},
                          {"color", stringSchema("\"#RRGGBB\" or red, green, blue, yellow, cyan, "
                                                 "magenta, white, black, gray, orange")},
                          {"line_width", numberSchema("default 1")},
                          {"make_current", booleanSchema("draw on it from now on")}},
                         {"name"})},
                 ToolAccess::Modify,
                 [context](const json& in) { return createLayer(*context, in); });

    registry.add({"list_entities",
                  "List the drawing's entities with their geometry (mm, degrees), optionally on "
                  "one layer, a page at a time.",
                  object({{"layer", stringSchema("only this layer")},
                          {"offset", integerSchema("skip this many (default 0)")},
                          {"max", integerSchema("at most this many (default 100)")}})},
                 ToolAccess::Read,
                 [context](const json& in) { return listEntities(*context, in); });

    registry.add({"delete_entities",
                  "Delete drawing entities by id (those on hidden or locked layers are left).",
                  object({{"ids", idsSchema()}}, {"ids"})},
                 ToolAccess::Modify,
                 [context](const json& in) { return deleteEntities(*context, in); });

    registry.add(
        {"transform_entities",
         "Move, copy, rotate, scale or mirror drawing entities. move/copy: dx, dy. rotate: angle "
         "(degrees, counterclockwise) about center. scale: factor about center. mirror: in the "
         "line axis_start to axis_end. Rotated and scaled entities replace the originals (new "
         "ids, returned) unless keep_original; mirror keeps the originals unless keep_original "
         "is false.",
         object({{"ids", idsSchema()},
                 {"operation", enumSchema({"move", "copy", "rotate", "scale", "mirror"}, "")},
                 {"dx", numberSchema("mm")},
                 {"dy", numberSchema("mm")},
                 {"angle", numberSchema("degrees")},
                 {"factor", numberSchema("scale factor")},
                 {"center", pointSchema("[x, y], default origin")},
                 {"axis_start", pointSchema("[x, y]")},
                 {"axis_end", pointSchema("[x, y]")},
                 {"keep_original", booleanSchema("rotate, scale, mirror: keep the originals")}},
                {"ids", "operation"})},
        ToolAccess::Modify, [context](const json& in) { return transformEntities(*context, in); });
}

}  // namespace hz::ai::detail
