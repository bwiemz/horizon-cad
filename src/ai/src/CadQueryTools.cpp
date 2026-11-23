// Tools that look: the document, its features and sketches, the solid's faces
// and edges with their geometry (so a model can pick "the top edges" by
// numbers rather than guess), and mass properties.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <string>
#include <vector>

#include "CadToolsCommon.h"
#include "horizon/document/Document.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/document/Sketch.h"
#include "horizon/drafting/Layer.h"
#include "horizon/math/Units.h"
#include "horizon/modeling/FacePlane.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/modeling/Naming.h"
#include "horizon/topology/HalfEdge.h"
#include "horizon/topology/Solid.h"

namespace hz::ai::detail {

namespace {

using math::Vec3;

constexpr int kDefaultMax = 60;
constexpr int kLimit = 2000;

const char* kindName(doc::Feature::ParameterKind kind) {
    switch (kind) {
        case doc::Feature::ParameterKind::Length:
            return "length";
        case doc::Feature::ParameterKind::Angle:
            return "angle";
        case doc::Feature::ParameterKind::Count:
            return "count";
        case doc::Feature::ParameterKind::Choice:
            return "choice";
    }
    return "length";
}

std::string hexColor(std::uint32_t argb) {
    char buffer[8];
    std::snprintf(buffer, sizeof buffer, "#%06X", static_cast<unsigned>(argb & 0xFFFFFFu));
    return buffer;
}

/// Newell's vector of a loop, from a point near it (never absolute
/// coordinates: far from the origin they cancel badly).
Vec3 newell(const topo::Wire* loop, const Vec3& reference, Vec3* vertexSum, int* vertexCount) {
    Vec3 n;
    if (loop == nullptr || loop->halfEdge == nullptr) return n;
    const topo::HalfEdge* start = loop->halfEdge;
    const topo::HalfEdge* he = start;
    int guard = 0;
    do {
        if (!he->origin || !he->next || !he->next->origin) break;
        const Vec3 a = he->origin->point - reference;
        const Vec3 b = he->next->origin->point - reference;
        n.x += (a.y - b.y) * (a.z + b.z);
        n.y += (a.z - b.z) * (a.x + b.x);
        n.z += (a.x - b.x) * (a.y + b.y);
        if (vertexSum) *vertexSum = *vertexSum + he->origin->point;
        if (vertexCount) ++*vertexCount;
        he = he->next;
    } while (he && he != start && ++guard < 100000);
    return n;
}

struct FaceInfo {
    bool ok = false;
    Vec3 normal;  ///< out of the part, unit
    Vec3 centre;  ///< vertex average of the outer loop
    double area = 0.0;
};

FaceInfo faceInfo(const topo::Face& face, double outward) {
    FaceInfo info;
    if (!face.alive || !face.outerLoop || !face.outerLoop->halfEdge ||
        !face.outerLoop->halfEdge->origin) {
        return info;
    }
    const Vec3 reference = face.outerLoop->halfEdge->origin->point;
    Vec3 sum;
    int count = 0;
    Vec3 n = newell(face.outerLoop, reference, &sum, &count);
    for (const topo::Wire* inner : face.innerLoops)
        n = n + newell(inner, reference, nullptr, nullptr);
    const double length = n.length();
    if (count == 0 || !(length > 1e-15)) return info;
    info.ok = true;
    info.area = length / 2.0;
    info.normal = n * (outward / length);
    info.centre = sum / static_cast<double>(count);
    return info;
}

int maxOf(const json& in) {
    return static_cast<int>(integer(in, "max", 1, kLimit, kDefaultMax));
}

// --- get_document_info --------------------------------------------------------

ToolResult documentInfo(const CadContext& context) {
    const doc::Document& document = context.document;
    json layers = json::array();
    for (const auto& name : document.layerManager().layerNames()) {
        const auto* layer = document.layerManager().getLayer(name);
        if (!layer) continue;
        layers.push_back({{"name", name},
                          {"color", hexColor(layer->color)},
                          {"visible", layer->visible},
                          {"locked", layer->locked}});
    }
    json out = {
        {"type", document.type() == doc::DocumentType::Part ? "part" : "drawing"},
        {"display_unit", std::string(math::symbolOf(document.lengthUnit()))},
        {"tool_units", "mm and degrees"},
        {"features", document.featureTree().featureCount()},
        {"sketches", document.sketches().size()},
        {"drawing_entities", document.draftDocument().entities().size()},
        {"layers", std::move(layers)},
        {"current_layer", document.layerManager().currentLayer()},
        {"variables", document.parameterRegistry().definitions()},
        {"solid", solidSummary(document)},
    };
    if (document.failedFeatureIndex() >= 0) {
        out["build_error"] = {{"feature_index", document.failedFeatureIndex()},
                              {"message", document.lastBuildMessage()}};
    }
    if (document.featureTree().rollbackIndex() >= 0) {
        out["rolled_back_to_feature"] = document.featureTree().rollbackIndex();
    }
    if (!document.configurations().active().empty()) {
        out["active_configuration"] = document.configurations().active();
    }
    return ToolResult::ok(out);
}

// --- list_features ------------------------------------------------------------

json describeFeature(const doc::Document& document, const doc::Feature& feature,
                     std::size_t index) {
    json parameters = json::object();
    for (const auto& [name, value] : feature.parameters()) {
        const auto kind = feature.parameterKind(name);
        json entry = {{"kind", kindName(kind)}};
        switch (kind) {
            case doc::Feature::ParameterKind::Angle:
                entry["value"] = round6(value / kDegree);
                break;
            case doc::Feature::ParameterKind::Count:
                entry["value"] = static_cast<long long>(std::llround(value));
                break;
            case doc::Feature::ParameterKind::Choice: {
                const auto choices = feature.parameterChoices(name);
                const auto code = static_cast<long long>(std::llround(value));
                entry["value"] = code;
                if (code >= 0 && static_cast<std::size_t>(code) < choices.size()) {
                    entry["choice"] = choices[static_cast<std::size_t>(code)];
                }
                entry["choices"] = choices;
                break;
            }
            case doc::Feature::ParameterKind::Length:
                entry["value"] = round6(value);
                break;
        }
        parameters[name] = std::move(entry);
    }
    json out = {{"index", index},
                {"id", feature.featureID()},
                {"type", feature.name()},
                {"parameters", std::move(parameters)}};
    if (feature.createsNewBody()) out["operation"] = doc::bodyOperationName(feature.operation());
    if (feature.isSuppressed()) out["suppressed"] = true;
    json vectors = json::object();
    for (const auto& [name, value] : feature.vectors()) vectors[name] = toJson(value);
    if (!vectors.empty()) out["vectors"] = std::move(vectors);
    json references = json::object();
    for (const auto& [name, value] : feature.references()) {
        if (!value.empty()) references[name] = value;
    }
    if (!references.empty()) out["references"] = std::move(references);
    if (!feature.parameterExpressions().empty())
        out["expressions"] = feature.parameterExpressions();
    json sketches = json::array();
    for (const auto& sketch : feature.sketches()) {
        if (sketch) sketches.push_back(sketch->id());
    }
    if (!sketches.empty()) out["sketches"] = std::move(sketches);
    if (document.failedFeatureIndex() == static_cast<int>(index)) {
        out["failed"] = document.lastBuildMessage();
    }
    if (!feature.expressionError().empty()) out["expression_error"] = feature.expressionError();
    return out;
}

ToolResult listFeatures(const CadContext& context) {
    const doc::Document& document = context.document;
    const auto& tree = document.featureTree();
    json features = json::array();
    for (std::size_t i = 0; i < tree.featureCount(); ++i) {
        if (const auto* feature = tree.feature(i)) {
            features.push_back(describeFeature(document, *feature, i));
        }
    }
    json out = {{"features", std::move(features)},
                {"note", "lengths in mm, angles in degrees; edit with edit_feature by index"}};
    if (tree.rollbackIndex() >= 0) out["rolled_back_to_feature"] = tree.rollbackIndex();
    return ToolResult::ok(out);
}

// --- list_sketches ------------------------------------------------------------

ToolResult listSketches(const CadContext& context, const json& in) {
    const doc::Document& document = context.document;
    const bool withEntities = boolean(in, "include_entities", false);
    std::map<std::uint64_t, std::vector<std::size_t>> usedBy;
    const auto& tree = document.featureTree();
    for (std::size_t i = 0; i < tree.featureCount(); ++i) {
        for (const auto& sketch : tree.feature(i)->sketches()) {
            if (sketch) usedBy[sketch->id()].push_back(i);
        }
    }
    json sketches = json::array();
    for (const auto& sketch : document.sketches()) {
        const auto& plane = sketch->plane();
        json entry = {{"id", sketch->id()},
                      {"name", sketch->name()},
                      {"plane",
                       {{"origin", toJson(plane.origin())},
                        {"normal", toJson(plane.normal())},
                        {"x_axis", toJson(plane.xAxis())}}},
                      {"entities", sketch->entities().size()},
                      {"used_by_features", usedBy[sketch->id()]}};
        if (!sketch->face().empty()) entry["on_face"] = sketch->face();
        if (withEntities) {
            json entities = json::array();
            for (const auto& entity : sketch->entities())
                entities.push_back(describeEntity(*entity));
            entry["entity_list"] = std::move(entities);
        }
        sketches.push_back(std::move(entry));
    }
    return ToolResult::ok(json{{"sketches", std::move(sketches)}});
}

// --- list_faces ---------------------------------------------------------------

ToolResult listFaces(const CadContext& context, const json& in) {
    const topo::Solid* solid = context.document.solid();
    if (solid == nullptr) return ToolResult::error("The part has no solid yet.");
    const int max = maxOf(in);
    const bool flatOnly = boolean(in, "flat_only", false);
    const bool filterFacing = has(in, "facing");
    const Vec3 facing = filterFacing ? direction(in, "facing") : Vec3();

    struct Group {
        std::string id;
        std::string name;
        int facets = 0;
        double area = 0.0;
        Vec3 weightedCentre;
        Vec3 normal;
        bool flat = false;
    };
    const double outward = model::outwardSign(*solid);
    std::vector<Group> groups;
    std::map<std::string, std::size_t> rowOf;
    for (const auto& face : solid->faces()) {
        if (!face.alive || !face.topoId.isValid()) continue;
        const FaceInfo info = faceInfo(face, outward);
        if (!info.ok) continue;
        const std::string logical = model::logicalFace(face.topoId.tag());
        auto [it, fresh] = rowOf.emplace(logical, groups.size());
        if (fresh) {
            Group group;
            group.id = logical;
            group.name = model::wholeFaceName(logical);
            group.normal = info.normal;
            group.flat = model::planeOf(face, outward).has_value();
            groups.push_back(group);
        }
        Group& group = groups[it->second];
        ++group.facets;
        group.area += info.area;
        group.weightedCentre = group.weightedCentre + info.centre * info.area;
        if (group.facets > 1) group.flat = false;  // a curved face in facets
    }

    std::vector<const Group*> chosen;
    for (const auto& group : groups) {
        if (flatOnly && !group.flat) continue;
        if (filterFacing && !(group.flat && group.normal.dot(facing) > std::cos(2.0 * kDegree))) {
            continue;
        }
        chosen.push_back(&group);
    }
    if (filterFacing) {
        // The outermost first: "the top face" is the first facing up.
        std::stable_sort(chosen.begin(), chosen.end(), [&](const Group* a, const Group* b) {
            const Vec3 ca = a->weightedCentre / a->area;
            const Vec3 cb = b->weightedCentre / b->area;
            return ca.dot(facing) > cb.dot(facing);
        });
    }
    json faces = json::array();
    for (const Group* group : chosen) {
        if (static_cast<int>(faces.size()) >= max) break;
        json entry = {{"id", group->id},
                      {"name", group->name},
                      {"kind", group->flat ? "flat" : "curved"},
                      {"center", toJson(group->area > 0 ? group->weightedCentre / group->area
                                                        : group->weightedCentre)},
                      {"area_mm2", round6(group->area)}};
        if (group->flat) entry["normal"] = toJson(group->normal);
        faces.push_back(std::move(entry));
    }
    json out = {{"faces", std::move(faces)}, {"total", chosen.size()}};
    if (chosen.size() > static_cast<std::size_t>(max)) {
        out["omitted"] = chosen.size() - static_cast<std::size_t>(max);
        out["note"] = "narrow with facing or flat_only, or raise max";
    }
    out["usage"] =
        "id: for shell's remove_faces; name: for create_sketch's face, add_hole's face, mirror's "
        "face";
    return ToolResult::ok(out);
}

// --- list_edges ---------------------------------------------------------------

ToolResult listEdges(const CadContext& context, const json& in) {
    const topo::Solid* solid = context.document.solid();
    if (solid == nullptr) return ToolResult::error("The part has no solid yet.");
    const int max = maxOf(in);
    const bool filterDirection = has(in, "direction");
    const Vec3 along = filterDirection ? direction(in, "direction") : Vec3();
    const std::string onFace = text(in, "on_face", "");
    const bool nearSort = has(in, "near_point");
    const Vec3 near = nearSort ? vec3(in, "near_point") : Vec3();
    const std::string kindFilter = choice(in, "kind", {"any", "line", "curve"}, std::string("any"));

    struct Group {
        std::string id;
        Vec3 from;
        Vec3 to;
        Vec3 midSum;
        double length = 0.0;
        int pieces = 0;
        std::vector<std::string> faceIds;
        std::vector<Vec3> faceNormals;
    };
    const double outward = model::outwardSign(*solid);
    std::vector<Group> groups;
    std::map<std::string, std::size_t> rowOf;
    for (const auto& edge : solid->edges()) {
        const topo::HalfEdge* he = edge.halfEdge;
        if (!edge.alive || !edge.topoId.isValid() || !he || !he->origin || !he->next ||
            !he->next->origin) {
            continue;
        }
        // The seams between the facets of one curved face are no edge of the part.
        if (edge.topoId.tag().find("/seam:") != std::string::npos) continue;
        const std::string logical = model::logicalEdge(edge.topoId.tag());
        auto [it, fresh] = rowOf.emplace(logical, groups.size());
        const Vec3 a = he->origin->point;
        const Vec3 b = he->next->origin->point;
        if (fresh) {
            Group group;
            group.id = logical;
            group.from = a;
            group.to = b;
            const topo::HalfEdge* sides[] = {he, he->twin};
            for (const topo::HalfEdge* side : sides) {
                if (!side || !side->face) continue;
                const FaceInfo info = faceInfo(*side->face, outward);
                group.faceIds.push_back(model::logicalFace(side->face->topoId.tag()));
                group.faceNormals.push_back(info.normal);
            }
            groups.push_back(group);
        }
        Group& group = groups[it->second];
        ++group.pieces;
        group.length += (b - a).length();
        group.midSum = group.midSum + (a + b) * 0.5;
        if (group.pieces > 1) group.to = b;
    }

    std::vector<const Group*> chosen;
    for (const auto& group : groups) {
        const bool line = group.pieces == 1;
        if ((kindFilter == "line" && !line) || (kindFilter == "curve" && line)) continue;
        if (filterDirection) {
            const Vec3 d = group.to - group.from;
            const double length = d.length();
            if (!line || !(length > 0) ||
                std::abs(d.dot(along)) / length < std::cos(1.0 * kDegree)) {
                continue;
            }
        }
        if (!onFace.empty() &&
            std::none_of(group.faceIds.begin(), group.faceIds.end(), [&](const std::string& f) {
                return f == onFace || model::wholeFaceName(f) == onFace;
            })) {
            continue;
        }
        chosen.push_back(&group);
    }
    if (nearSort) {
        std::stable_sort(chosen.begin(), chosen.end(), [&](const Group* a, const Group* b) {
            return (a->midSum / a->pieces - near).length() <
                   (b->midSum / b->pieces - near).length();
        });
    }
    json edges = json::array();
    for (const Group* group : chosen) {
        if (static_cast<int>(edges.size()) >= max) break;
        json faces = json::array();
        for (std::size_t i = 0; i < group->faceIds.size(); ++i) {
            faces.push_back({{"id", group->faceIds[i]}, {"normal", toJson(group->faceNormals[i])}});
        }
        json entry = {{"id", group->id},
                      {"kind", group->pieces == 1 ? "line" : "curve"},
                      {"from", toJson(group->from)},
                      {"to", toJson(group->to)},
                      {"mid", toJson(group->midSum / group->pieces)},
                      {"length", round6(group->length)},
                      {"faces", std::move(faces)}};
        edges.push_back(std::move(entry));
    }
    json out = {{"edges", std::move(edges)}, {"total", chosen.size()}};
    if (chosen.size() > static_cast<std::size_t>(max)) {
        out["omitted"] = chosen.size() - static_cast<std::size_t>(max);
        out["note"] = "narrow with direction, on_face, kind or near_point, or raise max";
    }
    out["usage"] = "pass ids to fillet or chamfer";
    return ToolResult::ok(out);
}

// --- get_mass_properties ------------------------------------------------------

const std::map<std::string, double>& densities() {
    static const std::map<std::string, double> table = {
        {"steel", 7850.0},     {"stainless_steel", 8000.0},
        {"aluminium", 2700.0}, {"aluminum", 2700.0},
        {"titanium", 4500.0},  {"brass", 8500.0},
        {"copper", 8960.0},    {"abs", 1050.0},
        {"pla", 1240.0},       {"petg", 1270.0},
        {"nylon", 1150.0},     {"acrylic", 1180.0},
        {"wood", 750.0},       {"water", 1000.0}};
    return table;
}

ToolResult massProperties(const CadContext& context, const json& in) {
    const topo::Solid* solid = context.document.solid();
    if (solid == nullptr) return ToolResult::error("The part has no solid yet.");
    std::vector<std::string> names;
    for (const auto& [name, density] : densities()) names.push_back(name);
    double density = 1000.0;
    std::string material = "density 1000 kg/m3 (water)";
    if (has(in, "material")) {
        material = choice(in, "material", names);
        density = densities().at(material);
    }
    if (has(in, "density_kg_m3")) {
        density = positive(in, "density_kg_m3");
        material = "density " + std::to_string(density) + " kg/m3";
    }
    const auto props = model::MassPropertiesCalculator::compute(*solid);
    if (!props.valid) return ToolResult::error("The solid could not be measured.");
    const double grams = props.volume * density * 1e-6;  // mm3 -> m3 is 1e-9; kg -> g 1e3
    return ToolResult::ok(json{{"volume_mm3", round6(props.volume)},
                               {"surface_area_mm2", round6(props.surfaceArea)},
                               {"center_of_mass", toJson(props.centerOfMass)},
                               {"density_kg_m3", density},
                               {"material", material},
                               {"mass_g", round6(grams)},
                               {"mass_kg", round6(grams / 1000.0)},
                               {"note",
                                "measured on the faceted solid; curved faces are slightly "
                                "under their true volume"}});
}

}  // namespace

void registerQueryTools(ToolRegistry& registry, const Context& context) {
    registry.add({"get_document_info",
                  "Summarise the open document: part or drawing, counts of features, sketches and "
                  "drawing entities, layers, variables, the solid (bounding box in mm, volume, "
                  "faces) and any build error.",
                  object(json::object())},
                 ToolAccess::Read, [context](const json&) { return documentInfo(*context); });

    registry.add({"list_features",
                  "List the part's features in history order, with each one's index, type, "
                  "parameters (value and kind: lengths in mm, angles in degrees), vectors, "
                  "references, expressions, body operation, and failures.",
                  object(json::object())},
                 ToolAccess::Read, [context](const json&) { return listFeatures(*context); });

    registry.add({"list_sketches",
                  "List the part's sketches: id, name, plane (origin, normal, x axis), the face it "
                  "follows, how many entities, and which features use it.",
                  object({{"include_entities",
                           booleanSchema("also list each sketch's entities (default false)")}})},
                 ToolAccess::Read,
                 [context](const json& in) { return listSketches(*context, in); });

    registry.add(
        {"list_faces",
         "List the solid's faces: id (for shell), name (for sketching on a face, holes and "
         "mirrors), flat or curved, outward normal (flat faces), centre and area. Use facing "
         "[x,y,z] to find flat faces facing a way, outermost first (facing [0,0,1]: the top).",
         object({{"facing", pointSchema("only flat faces whose outward normal is this direction")},
                 {"flat_only", booleanSchema("only flat faces")},
                 {"max", integerSchema("at most this many (default 60)")}})},
        ToolAccess::Read, [context](const json& in) { return listFaces(*context, in); });

    registry.add(
        {"list_edges",
         "List the solid's edges for fillet and chamfer: id, line or curve, end points, midpoint, "
         "length, and the two faces it joins (id and outward normal). An edge between a face "
         "facing +Z and a side face is a top edge. Filter by direction (lines parallel to it), "
         "on_face (a face id or name), kind, or sort by near_point.",
         object({{"direction", pointSchema("only straight edges parallel to this [x,y,z]")},
                 {"on_face", stringSchema("only edges of this face (id or name from list_faces)")},
                 {"kind", enumSchema({"any", "line", "curve"}, "default any")},
                 {"near_point", pointSchema("sort nearest first to this point [x,y,z]")},
                 {"max", integerSchema("at most this many (default 60)")}})},
        ToolAccess::Read, [context](const json& in) { return listEdges(*context, in); });

    std::vector<std::string> materials;
    for (const auto& [name, density] : densities()) materials.push_back(name);
    registry.add({"get_mass_properties",
                  "Volume (mm3), surface area (mm2), centre of mass, and mass (g, kg) of the solid "
                  "in a material or at a density.",
                  object({{"material", enumSchema(materials, "a common material")},
                          {"density_kg_m3", numberSchema("a density instead of a material")}})},
                 ToolAccess::Read,
                 [context](const json& in) { return massProperties(*context, in); });
}

}  // namespace hz::ai::detail
