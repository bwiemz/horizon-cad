// The rest of the part modeller for a model: loft, sweep, draft, combining
// bodies, datum geometry, and measuring what is there.

#include <algorithm>
#include <cmath>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "CadToolsCommon.h"
#include "horizon/document/Document.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/document/Sketch.h"
#include "horizon/modeling/BooleanOp.h"
#include "horizon/modeling/FacePlane.h"
#include "horizon/modeling/Measure.h"
#include "horizon/modeling/Naming.h"
#include "horizon/modeling/ReferenceGeometry.h"
#include "horizon/topology/HalfEdge.h"
#include "horizon/topology/Solid.h"

namespace hz::ai::detail {

namespace {

using math::Vec3;

const topo::Solid& solidOf(const CadContext& context) {
    const topo::Solid* solid = context.document.solid();
    if (solid == nullptr) throw ArgError("the part has no solid yet");
    return *solid;
}

doc::BodyOperation operationOf(const CadContext& context, const json& in) {
    static const std::vector<std::string> names = {"new", "join", "cut", "intersect"};
    const std::string fallback = context.document.solid() != nullptr ? "join" : "new";
    return doc::bodyOperationFromName(choice(in, "operation", names, fallback))
        .value_or(doc::BodyOperation::NewBody);
}

json operationSchema() {
    return enumSchema({"new", "join", "cut", "intersect"},
                      "how the new body combines with the part (default join if there is a "
                      "part, else new)");
}

// --- loft / sweep -------------------------------------------------------------

ToolResult loft(const CadContext& context, const json& in) {
    if (!has(in, "sketch_ids") || !in["sketch_ids"].is_array()) {
        throw ArgError("\"sketch_ids\" must be a list of sketch ids, first to last");
    }
    std::vector<std::shared_ptr<doc::Sketch>> sections;
    for (std::size_t i = 0; i < in["sketch_ids"].size(); ++i) {
        const json item = {{"id", in["sketch_ids"][i]}};
        sections.push_back(sketchOf(context.document, item, "id"));
    }
    if (sections.size() < 2) throw ArgError("a loft needs at least two sections");
    auto feature = std::make_unique<doc::LoftFeature>(std::move(sections));
    feature->setOperation(operationOf(context, in));
    return addFeature(context, std::move(feature), "Loft");
}

ToolResult sweep(const CadContext& context, const json& in) {
    auto profile = sketchOf(context.document, in, "profile_sketch_id");
    auto path = sketchOf(context.document, in, "path_sketch_id");
    if (profile == path) throw ArgError("the profile and the path must be different sketches");
    auto feature = std::make_unique<doc::SweepFeature>(profile, path);
    feature->setOperation(operationOf(context, in));
    return addFeature(context, std::move(feature), "Sweep");
}

// --- draft / combine ----------------------------------------------------------

ToolResult draft(const CadContext& context, const json& in) {
    solidOf(context);
    const double angle = number(in, "angle");
    if (!(std::abs(angle) > 0.0) || std::abs(angle) >= 45.0) {
        throw ArgError("\"angle\" must be between -45 and 45 degrees, not 0");
    }
    const Vec3 pull = direction(in, "pull_direction", Vec3::UnitZ);
    const Vec3 neutral = vec3(in, "neutral_point", Vec3::Zero);
    return addFeature(context, std::make_unique<doc::DraftFeature>(pull, neutral, angle * kDegree),
                      "Draft");
}

ToolResult combineBodies(const CadContext& context, const json& in) {
    solidOf(context);
    const std::string operation = choice(in, "operation", {"union", "subtract", "intersect"});
    const model::BooleanType type = operation == "union"      ? model::BooleanType::Union
                                    : operation == "subtract" ? model::BooleanType::Subtract
                                                              : model::BooleanType::Intersect;
    return addFeature(context, std::make_unique<doc::BooleanFeature>(type), "Combine bodies");
}

// --- datum geometry -----------------------------------------------------------

ToolResult addDatum(const CadContext& context, const json& in) {
    const std::string kind = choice(in, "kind", {"plane", "axis", "point"});
    std::unique_ptr<doc::DatumFeature> feature;
    if (kind == "plane") {
        const Vec3 normal = direction(in, "normal");
        Vec3 xAxis = has(in, "x_axis")                         ? direction(in, "x_axis")
                     : std::abs(normal.dot(Vec3::UnitX)) < 0.9 ? Vec3::UnitX
                                                               : Vec3::UnitY;
        xAxis = xAxis - normal * xAxis.dot(normal);
        if (!(xAxis.length() > 1e-9)) throw ArgError("\"x_axis\" must not be along \"normal\"");
        feature = doc::DatumFeature::makePlane(
            model::DatumPlane{vec3(in, "origin", Vec3::Zero), normal, xAxis.normalized()});
    } else if (kind == "axis") {
        feature = doc::DatumFeature::makeAxis(
            model::DatumAxis{vec3(in, "origin", Vec3::Zero), direction(in, "direction")});
    } else {
        feature = doc::DatumFeature::makePoint(model::DatumPoint{vec3(in, "position")});
    }
    return addFeature(context, std::move(feature), "Datum " + kind);
}

// --- measure ------------------------------------------------------------------

/// What a measurement is taken from: a point, a straight edge, or a flat
/// face.
struct Target {
    enum class Kind { Point, Edge, Face } kind = Kind::Point;
    Vec3 a;       ///< the point; an edge's start; a point on the face
    Vec3 b;       ///< an edge's end
    Vec3 normal;  ///< a face's, out of the part
};

Target targetOf(const CadContext& context, const json& in, const char* key) {
    if (!has(in, key) || !in[key].is_object()) {
        throw ArgError(std::string("\"") + key +
                       "\" must be {\"point\": [x,y,z]}, {\"edge\": id} or {\"face\": id or name}");
    }
    const json& spec = in[key];
    Target target;
    if (has(spec, "point")) {
        target.a = vec3(spec, "point");
        return target;
    }
    const topo::Solid& solid = solidOf(context);
    if (has(spec, "face")) {
        const std::string name = text(spec, "face");
        const double outward = model::outwardSign(solid);
        for (const auto& face : solid.faces()) {
            if (!face.alive || !face.topoId.isValid()) continue;
            const std::string logical = model::logicalFace(face.topoId.tag());
            if (logical != name && model::wholeFaceName(logical) != name) continue;
            const auto plane = model::planeOf(face, outward);
            if (!plane) throw ArgError("face \"" + name + "\" is not flat");
            target.kind = Target::Kind::Face;
            target.a = plane->origin;
            target.normal = plane->normal;
            return target;
        }
        throw ArgError("there is no face \"" + name + "\"; use list_faces");
    }
    if (has(spec, "edge")) {
        const std::string name = text(spec, "edge");
        int pieces = 0;
        for (const auto& edge : solid.edges()) {
            if (!edge.alive || !edge.topoId.isValid() || !edge.halfEdge) continue;
            if (model::logicalEdge(edge.topoId.tag()) != name) continue;
            const topo::HalfEdge* he = edge.halfEdge;
            if (!he->origin || !he->next || !he->next->origin) continue;
            ++pieces;
            target.a = he->origin->point;
            target.b = he->next->origin->point;
        }
        if (pieces == 0) throw ArgError("there is no edge \"" + name + "\"; use list_edges");
        if (pieces > 1) throw ArgError("edge \"" + name + "\" is curved: measure its points");
        target.kind = Target::Kind::Edge;
        return target;
    }
    throw ArgError(std::string("\"") + key + "\" names no point, edge or face");
}

ToolResult measure(const CadContext& context, const json& in) {
    const Target from = targetOf(context, in, "from");
    const Target to = targetOf(context, in, "to");
    using K = Target::Kind;
    json out = json::object();
    const auto planeDistance = [](const Vec3& point, const Target& face) {
        return std::abs((point - face.a).dot(face.normal));
    };
    if (from.kind == K::Point && to.kind == K::Point) {
        out["distance"] = round6(model::measure::distance(from.a, to.a));
        out["delta"] = toJson(to.a - from.a);
    } else if (from.kind == K::Face && to.kind == K::Face) {
        const double angle = model::measure::angleBetween(from.normal, to.normal) / kDegree;
        out["angle_between_normals_deg"] = round6(angle);
        if (std::abs(std::abs(from.normal.dot(to.normal)) - 1.0) < 1e-9) {
            out["distance"] = round6(planeDistance(to.a, from));  // parallel: a thickness
        }
    } else if ((from.kind == K::Point && to.kind == K::Face) ||
               (from.kind == K::Face && to.kind == K::Point)) {
        const Target& face = from.kind == K::Face ? from : to;
        const Target& point = from.kind == K::Point ? from : to;
        out["distance"] = round6(planeDistance(point.a, face));
    } else if (from.kind == K::Edge && to.kind == K::Edge) {
        out["distance"] = round6(model::measure::segmentToSegment(from.a, from.b, to.a, to.b));
        // Between lines, not directions: an edge's way round is arbitrary.
        const double raw = model::measure::angleBetween(from.b - from.a, to.b - to.a) / kDegree;
        out["angle_deg"] = round6(std::min(raw, 180.0 - raw));
    } else if (from.kind == K::Edge || to.kind == K::Edge) {
        const Target& edge = from.kind == K::Edge ? from : to;
        const Target& other = from.kind == K::Edge ? to : from;
        if (other.kind == K::Point) {
            out["distance"] = round6(model::measure::pointToSegment(other.a, edge.a, edge.b));
        } else {
            // An edge and a face: the distances of its ends, and its angle to
            // the face's plane.
            out["distance_from_start"] = round6(planeDistance(edge.a, other));
            out["distance_from_end"] = round6(planeDistance(edge.b, other));
            const double toNormal =
                model::measure::angleBetween(edge.b - edge.a, other.normal) / kDegree;
            out["angle_to_plane_deg"] = round6(std::abs(90.0 - toNormal));
        }
    }
    out["units"] = "mm, degrees";
    return ToolResult::ok(out);
}

json targetSchema(const std::string& description) {
    json schema = object({{"point", pointSchema("[x,y,z]")},
                          {"edge", stringSchema("a straight edge's id from list_edges")},
                          {"face", stringSchema("a flat face's id or name from list_faces")}});
    schema["description"] = description;
    return schema;
}

}  // namespace

void registerAdvancedTools(ToolRegistry& registry, const Context& context) {
    const auto add = [&registry, &context](ToolSpec spec, ToolAccess access,
                                           ToolResult (*handler)(const CadContext&, const json&)) {
        registry.add(std::move(spec), access,
                     [context, handler](const json& in) { return handler(*context, in); });
    };

    add({"loft",
         "Loft a solid through two or more sketches' closed profiles, first to last (for example "
         "a rectangle on XY and a smaller one on XY with offset 20). The profiles should have the "
         "same number of corners.",
         object({{"sketch_ids", arraySchema(json{{"type", "integer"}}, "the sections, in order")},
                 {"operation", operationSchema()}},
                {"sketch_ids"})},
        ToolAccess::Modify, &loft);

    add({"sweep",
         "Sweep a sketch's closed profile along a path: another sketch's open chain of lines and "
         "arcs. Draw the profile on a plane square to the path where it starts (a path on XZ "
         "starting along +X wants its profile on YZ).",
         object({{"profile_sketch_id", integerSchema("the closed profile")},
                 {"path_sketch_id", integerSchema("the open path")},
                 {"operation", operationSchema()}},
                {"profile_sketch_id", "path_sketch_id"})},
        ToolAccess::Modify, &sweep);

    add({"draft",
         "Taper the part's side faces by an angle, as for a moulded part: faces along the pull "
         "direction lean in or out about the plane through neutral_point.",
         object({{"angle", numberSchema("degrees, -45 to 45 (negative leans in)")},
                 {"pull_direction", pointSchema("[x,y,z], default [0,0,1]")},
                 {"neutral_point", pointSchema("[x,y,z], default origin")}},
                {"angle"})},
        ToolAccess::Modify, &draft);

    add({"combine_bodies",
         "Combine the part's separate bodies (made with operation \"new\") into one: union, "
         "subtract (the later bodies from the first) or intersect.",
         object({{"operation", enumSchema({"union", "subtract", "intersect"}, "")}},
                {"operation"})},
        ToolAccess::Modify, &combineBodies);

    add({"add_datum",
         "Add reference geometry: a datum plane (origin, normal, x_axis) to sketch on "
         "(create_sketch "
         "datum), an axis (origin, direction), or a point (position).",
         object({{"kind", enumSchema({"plane", "axis", "point"}, "")},
                 {"origin", pointSchema("plane, axis: [x,y,z], default origin")},
                 {"normal", pointSchema("plane: [x,y,z]")},
                 {"x_axis", pointSchema("plane: its x direction [x,y,z]")},
                 {"direction", pointSchema("axis: [x,y,z]")},
                 {"position", pointSchema("point: [x,y,z]")}},
                {"kind"})},
        ToolAccess::Modify, &addDatum);

    add({"measure",
         "Measure between two things: points, straight edges (ids from list_edges) or flat faces "
         "(from list_faces). Gives the distance (a wall's thickness between parallel faces, a "
         "point's height above a face, the gap between edges) and angles. Use it to check what "
         "was made.",
         object({{"from", targetSchema("{\"point\": [x,y,z]} | {\"edge\": id} | {\"face\": id}")},
                 {"to", targetSchema("the same kinds")}},
                {"from", "to"})},
        ToolAccess::Read, &measure);
}

}  // namespace hz::ai::detail
