// Tools that build and change the part: sketches, features, variables. Each
// change is one step on the document's undo stack, made through the same
// commands the window uses.

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
#include "horizon/document/Document.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/document/ModelCommands.h"
#include "horizon/document/ParameterRegistry.h"
#include "horizon/document/Sketch.h"
#include "horizon/document/UndoStack.h"
#include "horizon/drafting/DraftEntity.h"
#include "horizon/drafting/SketchPlane.h"
#include "horizon/math/Expression.h"
#include "horizon/math/Quantity.h"
#include "horizon/math/Units.h"
#include "horizon/modeling/FacePlane.h"
#include "horizon/modeling/Naming.h"
#include "horizon/topology/HalfEdge.h"
#include "horizon/topology/Solid.h"
#include "horizon/topology/TopologyID.h"

namespace hz::ai::detail {

namespace {

using math::Vec3;

const std::vector<std::string> kOperations = {"new", "join", "cut", "intersect"};

/// The body operation asked for; by default join when there is a part to
/// join, else a new body.
doc::BodyOperation operationOf(const CadContext& context, const json& in) {
    const std::string fallback = context.document.solid() != nullptr ? "join" : "new";
    const std::string name = choice(in, "operation", kOperations, fallback);
    return doc::bodyOperationFromName(name).value_or(doc::BodyOperation::NewBody);
}

json operationSchema() {
    return enumSchema(kOperations,
                      "how the new body combines with the part: join (add), cut (remove), "
                      "intersect, new (separate body). Default: join if there is a part, else new");
}

const topo::Solid& solidOf(const CadContext& context) {
    const topo::Solid* solid = context.document.solid();
    if (solid == nullptr) throw ArgError("the part has no solid yet");
    return *solid;
}

/// The plane named by "plane" (XY, XZ, YZ) and "offset", as the window
/// makes them: XY seen from above, XZ from the front, YZ from the right.
draft::SketchPlane standardPlane(const json& in) {
    const std::string which = choice(in, "plane", {"xy", "xz", "yz"}, std::string("xy"));
    const double offset = number(in, "offset", 0.0);
    Vec3 normal = Vec3::UnitZ;
    Vec3 xAxis = Vec3::UnitX;
    if (which == "xz") {
        normal = Vec3(0, -1, 0);
    } else if (which == "yz") {
        normal = Vec3::UnitX;
        xAxis = Vec3::UnitY;
    }
    return draft::SketchPlane(normal * offset, normal, xAxis);
}

Vec3 acrossFor(const Vec3& normal) {
    return std::abs(normal.dot(Vec3::UnitX)) < 0.9 ? Vec3::UnitX : Vec3::UnitY;
}

// --- create_sketch ------------------------------------------------------------

ToolResult createSketch(const CadContext& context, const json& in) {
    doc::Document& document = context.document;
    draft::SketchPlane plane;
    std::string face;
    if (has(in, "face")) {
        face = model::wholeFaceName(text(in, "face"));
        std::string why;
        const auto found = model::planeOfFace(solidOf(context), face, &why);
        if (!found) {
            throw ArgError("face \"" + face + "\" " + (why.empty() ? "is not a flat face" : why) +
                           "; use a name from list_faces");
        }
        plane = draft::SketchPlane(found->origin, found->normal, acrossFor(found->normal));
    } else if (has(in, "origin") || has(in, "normal")) {
        const Vec3 normal = direction(in, "normal");
        Vec3 xAxis = has(in, "x_axis") ? direction(in, "x_axis") : acrossFor(normal);
        xAxis = xAxis - normal * xAxis.dot(normal);
        if (!(xAxis.length() > 1e-9)) throw ArgError("\"x_axis\" must not be along \"normal\"");
        plane = draft::SketchPlane(vec3(in, "origin", Vec3::Zero), normal, xAxis.normalized());
    } else {
        plane = standardPlane(in);
    }
    if (!has(in, "entities")) throw ArgError("\"entities\" is required");
    auto entities = buildEntities(in["entities"], false);

    auto sketch = std::make_shared<doc::Sketch>(plane);
    sketch->setName(text(in, "name", "Sketch " + std::to_string(document.sketches().size() + 1)));
    sketch->setFace(face);
    for (auto& entity : entities) sketch->addEntity(entity);
    if (document.type() == doc::DocumentType::Drawing &&
        document.draftDocument().entities().empty()) {
        document.setType(doc::DocumentType::Part);
    }
    document.undoStack().push(std::make_unique<doc::AddSketchCommand>(document, sketch));
    context.modelChanged();
    json out = {{"sketch_id", sketch->id()},
                {"name", sketch->name()},
                {"entities", entities.size()},
                {"plane",
                 {{"origin", toJson(plane.origin())},
                  {"normal", toJson(plane.normal())},
                  {"x_axis", toJson(plane.xAxis())},
                  {"y_axis", toJson(plane.yAxis())}}},
                {"next", "extrude or revolve it by sketch_id"}};
    if (!face.empty()) out["on_face"] = face;
    return ToolResult::ok(out, true);
}

// --- extrude / revolve --------------------------------------------------------

ToolResult extrude(const CadContext& context, const json& in) {
    auto sketch = sketchOf(context.document, in, "sketch_id");
    const std::string extent =
        choice(in, "extent", {"blind", "symmetric", "through_all", "through_all_both"},
               std::string("blind"));
    const bool through = extent == "through_all" || extent == "through_all_both";
    const double distance = through ? positive(in, "distance", 1.0) : positive(in, "distance");
    const doc::BodyOperation operation = operationOf(context, in);
    // A cut from a sketch on a face goes into the part by default.
    const std::string defaultWay =
        operation == doc::BodyOperation::Cut && !sketch->face().empty() ? "reverse" : "normal";
    const std::string way = choice(in, "direction", {"normal", "reverse"}, defaultWay);
    const Vec3 dir = sketch->drawnPlane().normal() * (way == "reverse" ? -1.0 : 1.0);

    auto feature = std::make_unique<doc::ExtrudeFeature>(sketch, dir, distance);
    using Extent = doc::ExtrudeFeature::Extent;
    feature->setExtent(extent == "symmetric"          ? Extent::Symmetric
                       : extent == "through_all"      ? Extent::ThroughAll
                       : extent == "through_all_both" ? Extent::ThroughAllBoth
                                                      : Extent::Blind);
    feature->setOperation(operation);
    return addFeature(
        context, std::move(feature), "Extrude",
        {{"direction", toJson(dir)}, {"operation", doc::bodyOperationName(operation)}});
}

ToolResult revolve(const CadContext& context, const json& in) {
    auto sketch = sketchOf(context.document, in, "sketch_id");
    const double angle = number(in, "angle", 360.0);
    if (!(angle > 0.0) || angle > 360.0)
        throw ArgError("\"angle\" must be more than 0 and at most 360");
    const draft::SketchPlane& plane = sketch->drawnPlane();
    Vec3 point = plane.origin();
    Vec3 axis = plane.yAxis();
    if (has(in, "axis_direction")) {
        axis = direction(in, "axis_direction");
        point = vec3(in, "axis_point", plane.origin());
    } else {
        const std::string which =
            choice(in, "axis", {"sketch_x", "sketch_y"}, std::string("sketch_y"));
        if (which == "sketch_x") axis = plane.xAxis();
    }
    auto feature = std::make_unique<doc::RevolveFeature>(sketch, point, axis, angle * kDegree);
    const doc::BodyOperation operation = operationOf(context, in);
    feature->setOperation(operation);
    return addFeature(context, std::move(feature), "Revolve",
                      {{"axis_point", toJson(point)}, {"axis_direction", toJson(axis)}});
}

// --- add_primitive ------------------------------------------------------------

ToolResult addPrimitive(const CadContext& context, const json& in) {
    const std::string shape = choice(in, "shape", {"box", "cylinder", "sphere", "cone", "torus"});
    std::unique_ptr<doc::PrimitiveFeature> feature;
    Vec3 position = vec3(in, "position", Vec3::Zero);
    const Vec3 axis = direction(in, "axis", Vec3::UnitZ);
    if (shape == "box") {
        const double sx = positive(in, "size_x");
        const double sy = positive(in, "size_y");
        const double sz = positive(in, "size_z");
        if (boolean(in, "centered", false)) {
            if ((axis - Vec3::UnitZ).length() > 1e-9) {
                throw ArgError("\"centered\" works only with the default axis [0, 0, 1]");
            }
            position = position - Vec3(sx / 2.0, sy / 2.0, sz / 2.0);
        }
        feature = doc::PrimitiveFeature::makeBox(sx, sy, sz);
    } else if (shape == "cylinder") {
        feature =
            doc::PrimitiveFeature::makeCylinder(positive(in, "radius"), positive(in, "height"));
    } else if (shape == "sphere") {
        feature = doc::PrimitiveFeature::makeSphere(positive(in, "radius"));
    } else if (shape == "cone") {
        const double bottom = number(in, "bottom_radius");
        const double top = number(in, "top_radius", 0.0);
        if (bottom < 0.0 || top < 0.0 || (bottom == 0.0 && top == 0.0)) {
            throw ArgError("the radii must not be negative, and one must be more than 0");
        }
        feature = doc::PrimitiveFeature::makeCone(bottom, top, positive(in, "height"));
    } else {
        const double major = positive(in, "major_radius");
        const double minor = positive(in, "minor_radius");
        if (minor >= major) throw ArgError("\"minor_radius\" must be less than \"major_radius\"");
        feature = doc::PrimitiveFeature::makeTorus(major, minor);
    }
    if (!feature->setVector("basePoint", position)) throw ArgError("\"position\" was refused");
    if (!feature->setVector("axisDirection", axis)) throw ArgError("\"axis\" was refused");
    feature->setOperation(operationOf(context, in));
    return addFeature(context, std::move(feature), "Primitive (" + shape + ")");
}

// --- fillet / chamfer / shell ------------------------------------------------

std::vector<topo::TopologyID> edgeIds(const CadContext& context, const json& in) {
    const topo::Solid& solid = solidOf(context);
    std::set<std::string> known;
    for (const auto& edge : solid.edges()) {
        if (edge.alive && edge.topoId.isValid())
            known.insert(model::logicalEdge(edge.topoId.tag()));
    }
    std::vector<topo::TopologyID> ids;
    for (const auto& name : strings(in, "edges")) {
        if (!known.count(name)) {
            throw ArgError("there is no edge \"" + name + "\"; use ids from list_edges");
        }
        ids.push_back(topo::TopologyID::fromTag(name));
    }
    return ids;
}

ToolResult fillet(const CadContext& context, const json& in) {
    const auto ids = edgeIds(context, in);
    const double radius = positive(in, "radius");
    return addFeature(context, std::make_unique<doc::FilletFeature>(ids, radius), "Fillet",
                      {{"edges", ids.size()}});
}

ToolResult chamfer(const CadContext& context, const json& in) {
    const auto ids = edgeIds(context, in);
    const double distance = positive(in, "distance");
    return addFeature(context, std::make_unique<doc::ChamferFeature>(ids, distance), "Chamfer",
                      {{"edges", ids.size()}});
}

ToolResult shell(const CadContext& context, const json& in) {
    const topo::Solid& solid = solidOf(context);
    const double thickness = positive(in, "thickness");
    std::vector<topo::TopologyID> ids;
    if (has(in, "remove_faces")) {
        std::set<std::string> known;
        for (const auto& face : solid.faces()) {
            if (face.alive && face.topoId.isValid())
                known.insert(model::logicalFace(face.topoId.tag()));
        }
        for (const auto& name : strings(in, "remove_faces")) {
            if (!known.count(name)) {
                throw ArgError("there is no face \"" + name + "\"; use ids from list_faces");
            }
            ids.push_back(topo::TopologyID::fromTag(name));
        }
    }
    return addFeature(context, std::make_unique<doc::ShellFeature>(thickness, ids), "Shell",
                      {{"open_faces", ids.size()}});
}

// --- add_hole -----------------------------------------------------------------

ToolResult addHole(const CadContext& context, const json& in) {
    const std::string face = model::wholeFaceName(text(in, "face"));
    std::string why;
    const auto plane = model::planeOfFace(solidOf(context), face, &why);
    if (!plane) {
        throw ArgError("face \"" + face + "\" " + (why.empty() ? "is not a flat face" : why) +
                       "; use a name from list_faces");
    }
    const Vec3 position = vec3(in, "position");
    const double diameter = positive(in, "diameter");
    const bool through = boolean(in, "through_all", false);
    const double depth = through ? positive(in, "depth", 1.0) : positive(in, "depth");
    auto feature = doc::HoleFeature::make(face, position, diameter, depth);
    const std::string type =
        choice(in, "type", {"simple", "counterbore", "countersink"}, std::string("simple"));

    std::map<std::string, double> values;
    values["type"] = type == "counterbore" ? 1.0 : type == "countersink" ? 2.0 : 0.0;
    values["extent"] = through ? 1.0 : 0.0;
    if (has(in, "bore_diameter")) values["boreDiameter"] = positive(in, "bore_diameter");
    if (has(in, "bore_depth")) values["boreDepth"] = positive(in, "bore_depth");
    if (has(in, "sink_diameter")) values["sinkDiameter"] = positive(in, "sink_diameter");
    if (has(in, "sink_angle")) values["sinkAngle"] = positive(in, "sink_angle") * kDegree;
    if (type == "counterbore" && !has(in, "bore_diameter")) values["boreDiameter"] = diameter * 1.8;
    if (type == "countersink" && !has(in, "sink_diameter")) values["sinkDiameter"] = diameter * 2.0;
    const auto refused = doc::setParameters(*feature, values);
    if (!refused.empty()) {
        std::string list;
        for (const auto& name : refused) list += (list.empty() ? "" : ", ") + name;
        throw ArgError("the hole refused: " + list +
                       " (a counterbore or countersink must be wider than the hole)");
    }
    return addFeature(context, std::move(feature), "Hole",
                      {{"face", face}, {"face_normal", toJson(plane->normal)}});
}

// --- patterns and mirror ------------------------------------------------------

ToolResult linearPattern(const CadContext& context, const json& in) {
    solidOf(context);
    const Vec3 dir = direction(in, "direction");
    const double spacing = positive(in, "spacing");
    const int count = static_cast<int>(integer(in, "count", 2, doc::kMaxPatternCount));
    auto feature = doc::PatternFeature::makeLinear(dir, spacing, count);
    feature->setTargets(featureTargets(context.document, in, "features"));
    return addFeature(context, std::move(feature), "Linear pattern");
}

ToolResult circularPattern(const CadContext& context, const json& in) {
    solidOf(context);
    const Vec3 point = vec3(in, "axis_point", Vec3::Zero);
    const Vec3 axis = direction(in, "axis_direction", Vec3::UnitZ);
    const int count = static_cast<int>(integer(in, "count", 2, doc::kMaxPatternCount));
    const double total = number(in, "total_angle", 360.0);
    if (!(total > 0.0) || total > 360.0) {
        throw ArgError("\"total_angle\" must be more than 0 and at most 360");
    }
    // A full turn spaces the instances evenly round it; a part turn puts the
    // last at its end.
    const double step = std::abs(total - 360.0) < 1e-9 ? total / count : total / (count - 1);
    auto feature = doc::PatternFeature::makeCircular(point, axis, step * kDegree, count);
    feature->setTargets(featureTargets(context.document, in, "features"));
    return addFeature(context, std::move(feature), "Circular pattern",
                      {{"angle_step_deg", round6(step)}});
}

ToolResult mirror(const CadContext& context, const json& in) {
    const topo::Solid& solid = solidOf(context);
    Vec3 point = Vec3::Zero;
    Vec3 normal = Vec3::UnitX;
    std::string face;
    if (has(in, "face")) {
        face = model::wholeFaceName(text(in, "face"));
        std::string why;
        const auto plane = model::planeOfFace(solid, face, &why);
        if (!plane) throw ArgError("face \"" + face + "\" " + (why.empty() ? "is not flat" : why));
        point = plane->origin;
        normal = plane->normal;
    } else if (has(in, "plane_normal")) {
        normal = direction(in, "plane_normal");
        point = vec3(in, "plane_point", Vec3::Zero);
    } else {
        const std::string which = choice(in, "plane", {"xy", "xz", "yz"}, std::string("yz"));
        normal = which == "xy" ? Vec3::UnitZ : which == "xz" ? Vec3::UnitY : Vec3::UnitX;
        point = normal * number(in, "offset", 0.0);
    }
    auto feature = doc::MirrorFeature::make(point, normal);
    if (!face.empty() && !feature->setReference("planeFace", face)) {
        throw ArgError("the face was refused as a mirror plane");
    }
    feature->setTargets(featureTargets(context.document, in, "features"));
    return addFeature(context, std::move(feature), "Mirror",
                      {{"plane_point", toJson(point)}, {"plane_normal", toJson(normal)}});
}

// --- edit / delete / suppress -------------------------------------------------

/// Letters and digits only, lower case: "Through all, both ways" is
/// "throughallbothways".
std::string squeezed(const std::string& text) {
    std::string out;
    for (const char c : text) {
        if (std::isalnum(static_cast<unsigned char>(c))) {
            out += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        }
    }
    return out;
}

/// The code of the choice @p given names among @p choices: the name itself
/// (in any case or punctuation), the start of only one, or the words the
/// tools use for an extrusion's extent ("blind", "symmetric").
std::size_t choiceCode(const std::vector<std::string>& choices, const std::string& parameter,
                       const std::string& given) {
    static const std::map<std::string, std::string> synonyms = {{"blind", "tothedistance"},
                                                                {"symmetric", "bothwayshalfeach"},
                                                                {"midplane", "bothwayshalfeach"},
                                                                {"uptoface", "uptoaface"}};
    std::string wanted = squeezed(given);
    if (const auto it = synonyms.find(wanted); it != synonyms.end()) wanted = it->second;
    std::vector<std::size_t> starts;
    for (std::size_t i = 0; i < choices.size(); ++i) {
        const std::string name = squeezed(choices[i]);
        if (name == wanted) return i;
        if (!wanted.empty() && name.rfind(wanted, 0) == 0) starts.push_back(i);
    }
    if (starts.size() == 1) return starts.front();
    std::string list;
    for (const auto& c : choices) list += (list.empty() ? "\"" : ", \"") + c + "\"";
    throw ArgError("\"" + parameter + "\" must be one of " + list);
}

ToolResult editFeature(const CadContext& context, const json& in) {
    doc::Document& document = context.document;
    doc::Feature& feature = featureAt(document, in, "index");
    const auto index = static_cast<int>(*document.featureTree().indexOf(&feature));
    const auto current = feature.parameters();
    const auto currentVectors = feature.vectors();
    const auto names = [](const auto& map) {
        std::string list;
        for (const auto& [name, value] : map) list += (list.empty() ? "" : ", ") + name;
        return list.empty() ? std::string("none") : list;
    };

    std::map<std::string, double> parameters;
    if (has(in, "parameters")) {
        if (!in["parameters"].is_object()) throw ArgError("\"parameters\" must be an object");
        for (auto it = in["parameters"].begin(); it != in["parameters"].end(); ++it) {
            const std::string& name = it.key();
            if (!current.count(name)) {
                throw ArgError("feature " + std::to_string(index) + " (" + feature.name() +
                               ") has no parameter \"" + name + "\"; it has: " + names(current));
            }
            const auto kind = feature.parameterKind(name);
            double value = 0.0;
            if (kind == doc::Feature::ParameterKind::Choice && it.value().is_string()) {
                value = static_cast<double>(choiceCode(feature.parameterChoices(name), name,
                                                       it.value().get<std::string>()));
            } else {
                value = number(in["parameters"], name.c_str());
            }
            if (kind == doc::Feature::ParameterKind::Angle) value *= kDegree;
            if ((kind == doc::Feature::ParameterKind::Count ||
                 kind == doc::Feature::ParameterKind::Choice) &&
                value != std::floor(value)) {
                throw ArgError("\"" + name + "\" must be a whole number");
            }
            parameters[name] = value;
        }
    }
    std::map<std::string, Vec3> vectors;
    if (has(in, "vectors")) {
        if (!in["vectors"].is_object()) throw ArgError("\"vectors\" must be an object");
        for (auto it = in["vectors"].begin(); it != in["vectors"].end(); ++it) {
            if (!currentVectors.count(it.key())) {
                throw ArgError("feature " + std::to_string(index) + " has no vector \"" + it.key() +
                               "\"; it has: " + names(currentVectors));
            }
            vectors[it.key()] = vec3(in["vectors"], it.key().c_str());
        }
    }
    std::map<std::string, std::string> expressions;
    if (has(in, "expressions")) {
        if (!in["expressions"].is_object()) throw ArgError("\"expressions\" must be an object");
        for (auto it = in["expressions"].begin(); it != in["expressions"].end(); ++it) {
            if (!current.count(it.key())) {
                throw ArgError("feature " + std::to_string(index) + " has no parameter \"" +
                               it.key() + "\"; it has: " + names(current));
            }
            // Kept as the window keeps it: its units written, a bare number
            // in a length taken as millimetres (the tools' unit).
            const std::string written = text(in["expressions"], it.key().c_str());
            const auto parsed = math::Expression::parse(written);
            if (!parsed) throw ArgError("\"" + written + "\" is not an expression");
            const auto kind = feature.parameterKind(it.key());
            const math::QuantityKind measure =
                kind == doc::Feature::ParameterKind::Angle    ? math::QuantityKind::Angle
                : kind == doc::Feature::ParameterKind::Length ? math::QuantityKind::Length
                                                              : math::QuantityKind::Number;
            std::string why;
            const auto kept = math::normalized(*parsed, document.variables(), measure,
                                               math::LengthUnit::Millimetre, &why);
            if (!kept) throw ArgError("expression for \"" + it.key() + "\": " + why);
            expressions[it.key()] = kept->toString();
        }
    }
    std::optional<doc::BodyOperation> operation;
    if (has(in, "operation")) {
        if (!feature.createsNewBody()) throw ArgError("this feature has no body operation");
        operation = doc::bodyOperationFromName(choice(in, "operation", kOperations));
    }
    if (parameters.empty() && vectors.empty() && expressions.empty() && !operation) {
        throw ArgError("nothing to change: give parameters, expressions, vectors or operation");
    }
    if (const auto refused = doc::refusedParameters(feature, parameters); !refused.empty()) {
        throw ArgError("the feature refused \"" + refused.front() +
                       "\" (out of range for it: a zero size, too few segments)");
    }
    if (const auto refused = doc::refusedVectors(feature, vectors); !refused.empty()) {
        throw ArgError("the feature refused the vector \"" + refused.front() + "\"");
    }

    auto command = std::make_unique<doc::EditFeatureCommand>(document, &feature, parameters,
                                                             operation, vectors, expressions);
    const doc::Command* step = command.get();
    document.undoStack().push(std::move(command));
    json report = rebuildAndReport(context);
    const std::string expressionError = feature.expressionError();
    if (document.failedFeatureIndex() == index || !expressionError.empty()) {
        const std::string reason =
            !expressionError.empty() ? expressionError : document.lastBuildMessage();
        document.undoStack().withdraw(step);
        document.rebuildModel();
        context.modelChanged();
        return ToolResult::error("The edit makes feature " + std::to_string(index) +
                                 " fail, so it was not made: " + reason);
    }
    context.modelChanged();
    report["edited_feature"] = index;
    return ToolResult::ok(report, true);
}

ToolResult deleteFeature(const CadContext& context, const json& in) {
    doc::Document& document = context.document;
    doc::Feature& feature = featureAt(document, in, "index");
    const std::string name = feature.name();
    document.undoStack().push(std::make_unique<doc::RemoveFeatureCommand>(document, &feature));
    json report = rebuildAndReport(context);
    context.modelChanged();
    report["deleted"] = name;
    report["note"] = "later features moved up one index";
    return ToolResult::ok(report, true);
}

ToolResult suppressFeature(const CadContext& context, const json& in) {
    doc::Document& document = context.document;
    doc::Feature& feature = featureAt(document, in, "index");
    const bool suppress = boolean(in, "suppressed", true);
    if (feature.isSuppressed() == suppress) {
        return ToolResult::ok(json{{"unchanged", true}, {"suppressed", suppress}});
    }
    document.undoStack().push(
        std::make_unique<doc::SetFeatureSuppressedCommand>(document, &feature, suppress));
    json report = rebuildAndReport(context);
    context.modelChanged();
    report["suppressed"] = suppress;
    return ToolResult::ok(report, true);
}

// --- set_variables ------------------------------------------------------------

ToolResult setVariables(const CadContext& context, const json& in) {
    doc::Document& document = context.document;
    std::map<std::string, std::string> after = document.parameterRegistry().definitions();
    std::vector<std::string> changed;
    if (has(in, "variables")) {
        if (!in["variables"].is_object()) throw ArgError("\"variables\" must be an object");
        for (auto it = in["variables"].begin(); it != in["variables"].end(); ++it) {
            const std::string& name = it.key();
            if (!doc::ParameterRegistry::isName(name)) {
                throw ArgError("\"" + name +
                               "\" cannot name a variable (a letter or _, then "
                               "letters, digits and _; not a unit or a function)");
            }
            std::string definition;
            if (it.value().is_string()) {
                definition = it.value().get<std::string>();
            } else if (it.value().is_number()) {
                // A bare number is a length in millimetres, as everywhere in the tools.
                const double value = it.value().get<double>();
                if (!std::isfinite(value)) throw ArgError("\"" + name + "\" must be finite");
                definition = json(value).dump() + " mm";
            } else {
                throw ArgError("\"" + name + "\" must be an expression such as \"12 mm\"");
            }
            if (definition.empty()) throw ArgError("\"" + name + "\" is empty");
            after[name] = definition;
            changed.push_back(name);
        }
    }
    if (has(in, "remove")) {
        for (const auto& name : strings(in, "remove")) {
            if (after.erase(name) == 0) throw ArgError("there is no variable \"" + name + "\"");
            changed.push_back(name);
        }
    }
    if (changed.empty()) throw ArgError("give variables to set or names to remove");
    std::string why;
    if (!doc::ParameterRegistry::check(after, &why)) throw ArgError(why);

    document.undoStack().push(std::make_unique<doc::SetVariablesCommand>(document, after));
    json report = rebuildAndReport(context);
    context.modelChanged();
    std::map<std::string, std::string> errors;
    json values = json::object();
    for (const auto& [name, quantity] : document.variables(&errors)) {
        const bool angle = quantity.angle == 1 && quantity.length == 0;
        values[name] = {{"value", round6(angle ? quantity.value / kDegree : quantity.value)},
                        {"measure", math::measureName(quantity)}};
    }
    report["variables"] = std::move(values);
    if (!errors.empty()) report["variable_errors"] = errors;
    return ToolResult::ok(report, true);
}

json indexSchema() {
    return integerSchema("the feature's index, from list_features");
}

json targetsSchema() {
    return arraySchema(json{{"type", "integer"}},
                       "indices of the features to repeat (from list_features); omit to repeat "
                       "the whole part");
}

}  // namespace

void registerModelTools(ToolRegistry& registry, const Context& context) {
    const auto add = [&registry, &context](ToolSpec spec,
                                           ToolResult (*handler)(const CadContext&, const json&)) {
        registry.add(std::move(spec), ToolAccess::Modify,
                     [context, handler](const json& in) { return handler(*context, in); });
    };

    add({"create_sketch",
         "Make a sketch: 2D shapes on a plane, to extrude or revolve. The plane is XY (default), "
         "XZ or YZ moved by offset along its normal; or a flat face of the part by name (the "
         "sketch follows the face); or origin, normal and x_axis. Coordinates are the plane's own "
         "2D x and y in mm (on XY: world x and y; on XZ: world x and z; on YZ: world y and z). "
         "Several closed shapes may be one profile: a shape inside another is a hole. Returns "
         "sketch_id.",
         object({{"entities", arraySchema(entitySchema(false), "the shapes")},
                 {"plane", enumSchema({"XY", "XZ", "YZ"}, "a standard plane (default XY)")},
                 {"offset", numberSchema("move the standard plane along its normal, mm")},
                 {"face", stringSchema("sketch on this flat face (a name from list_faces)")},
                 {"origin", pointSchema("custom plane: a point on it [x,y,z]")},
                 {"normal", pointSchema("custom plane: its normal [x,y,z]")},
                 {"x_axis", pointSchema("custom plane: its x direction [x,y,z]")},
                 {"name", stringSchema("a name for the sketch")}},
                {"entities"})},
        &createSketch);

    add({"extrude",
         "Extrude a sketch's closed profile into a solid, along its plane's normal (or reverse). "
         "extent: blind (distance), symmetric (half each way), through_all or through_all_both "
         "(for cuts through the part). A cut from a sketch on a face goes into the part by "
         "default.",
         object({{"sketch_id", integerSchema("from create_sketch or list_sketches")},
                 {"distance", numberSchema("mm (not needed for through_all)")},
                 {"direction",
                  enumSchema({"normal", "reverse"}, "along the sketch normal or against it")},
                 {"extent", enumSchema({"blind", "symmetric", "through_all", "through_all_both"},
                                       "default blind")},
                 {"operation", operationSchema()}},
                {"sketch_id"})},
        &extrude);

    add({"revolve",
         "Revolve a sketch's closed profile about an axis. By default the axis is the sketch's own "
         "y axis through its origin (sketch_x for its x axis); or give axis_point and "
         "axis_direction in world coordinates. The profile must lie on one side of the axis.",
         object({{"sketch_id", integerSchema("from create_sketch or list_sketches")},
                 {"angle", numberSchema("degrees, default 360")},
                 {"axis", enumSchema({"sketch_x", "sketch_y"}, "default sketch_y")},
                 {"axis_point", pointSchema("world point on the axis")},
                 {"axis_direction", pointSchema("world direction of the axis")},
                 {"operation", operationSchema()}},
                {"sketch_id"})},
        &revolve);

    add({"add_primitive",
         "Add a box, cylinder, sphere, cone or torus. position is the box's minimum corner (or its "
         "centre with centered), the centre of a cylinder's or cone's base, a sphere's or torus's "
         "centre; axis is the shape's own z direction (default [0,0,1]).",
         object({{"shape", enumSchema({"box", "cylinder", "sphere", "cone", "torus"}, "")},
                 {"size_x", numberSchema("box, mm")},
                 {"size_y", numberSchema("box, mm")},
                 {"size_z", numberSchema("box, mm")},
                 {"centered", booleanSchema("box: position is its centre")},
                 {"radius", numberSchema("cylinder, sphere, mm")},
                 {"height", numberSchema("cylinder, cone, mm")},
                 {"bottom_radius", numberSchema("cone, mm")},
                 {"top_radius", numberSchema("cone, mm (0 for a point)")},
                 {"major_radius", numberSchema("torus, mm")},
                 {"minor_radius", numberSchema("torus, mm")},
                 {"position", pointSchema("[x,y,z], default origin")},
                 {"axis", pointSchema("[x,y,z], default [0,0,1]")},
                 {"operation", operationSchema()}},
                {"shape"})},
        &addPrimitive);

    add({"fillet", "Round edges of the part with a radius. Edge ids come from list_edges.",
         object({{"edges", arraySchema(json{{"type", "string"}}, "edge ids")},
                 {"radius", numberSchema("mm")}},
                {"edges", "radius"})},
        &fillet);

    add({"chamfer", "Bevel edges of the part by a distance. Edge ids come from list_edges.",
         object({{"edges", arraySchema(json{{"type", "string"}}, "edge ids")},
                 {"distance", numberSchema("mm")}},
                {"edges", "distance"})},
        &chamfer);

    add({"shell",
         "Hollow the part to a wall thickness, leaving the chosen faces open (ids from "
         "list_faces; none: a closed hollow).",
         object({{"thickness", numberSchema("wall thickness, mm")},
                 {"remove_faces", arraySchema(json{{"type", "string"}}, "face ids to leave open")}},
                {"thickness"})},
        &shell);

    add({"add_hole",
         "Drill a hole into a flat face (name from list_faces) at a point on it: simple, "
         "counterbore or countersink, to a depth or through all. Point the hole at the face's "
         "centre or offset from it along the face.",
         object({{"face", stringSchema("a flat face's name from list_faces")},
                 {"position", pointSchema("world point on the face [x,y,z]")},
                 {"diameter", numberSchema("mm")},
                 {"depth", numberSchema("mm (not needed with through_all)")},
                 {"through_all", booleanSchema("through the whole part")},
                 {"type", enumSchema({"simple", "counterbore", "countersink"}, "default simple")},
                 {"bore_diameter", numberSchema("counterbore, mm")},
                 {"bore_depth", numberSchema("counterbore, mm")},
                 {"sink_diameter", numberSchema("countersink, mm")},
                 {"sink_angle", numberSchema("countersink, degrees (default 90)")}},
                {"face", "position", "diameter"})},
        &addHole);

    add({"linear_pattern",
         "Repeat the part, or chosen features, in a line: count copies (including the original) "
         "spacing mm apart along direction.",
         object({{"direction", pointSchema("[x,y,z]")},
                 {"spacing", numberSchema("mm between copies")},
                 {"count", integerSchema("copies including the original, at least 2")},
                 {"features", targetsSchema()}},
                {"direction", "spacing", "count"})},
        &linearPattern);

    add({"circular_pattern",
         "Repeat the part, or chosen features, around an axis: count copies (including the "
         "original) evenly round total_angle (default 360 degrees).",
         object({{"axis_point", pointSchema("a point on the axis, default origin")},
                 {"axis_direction", pointSchema("default [0,0,1]")},
                 {"count", integerSchema("copies including the original, at least 2")},
                 {"total_angle", numberSchema("degrees, default 360")},
                 {"features", targetsSchema()}},
                {"count"})},
        &circularPattern);

    add({"mirror",
         "Mirror the part, or chosen features, in a plane and join the image: a standard plane "
         "(YZ default, moved by offset), a plane through plane_point with plane_normal, or a flat "
         "face by name.",
         object({{"plane", enumSchema({"XY", "XZ", "YZ"}, "default YZ")},
                 {"offset", numberSchema("mm along the standard plane's normal")},
                 {"plane_point", pointSchema("[x,y,z]")},
                 {"plane_normal", pointSchema("[x,y,z]")},
                 {"face", stringSchema("a flat face's name from list_faces")},
                 {"features", targetsSchema()}})},
        &mirror);

    add({"edit_feature",
         "Change a feature: parameters by name (lengths mm, angles degrees, choices by name or "
         "code), expressions of variables (\"depth\", \"wall * 2\"), vectors by name [x,y,z], or "
         "the body operation. Names come from list_features. An edit that makes the feature fail "
         "is not made.",
         object({{"index", indexSchema()},
                 {"parameters", object(json::object())},
                 {"expressions", object(json::object())},
                 {"vectors", object(json::object())},
                 {"operation", operationSchema()}},
                {"index"})},
        &editFeature);

    add({"delete_feature", "Delete a feature from the history (its sketch stays).",
         object({{"index", indexSchema()}}, {"index"})},
        &deleteFeature);

    add({"suppress_feature",
         "Suppress a feature (keep it in the history but leave it out of the part), or bring it "
         "back with suppressed false.",
         object({{"index", indexSchema()}, {"suppressed", booleanSchema("default true")}},
                {"index"})},
        &suppressFeature);

    add({"set_variables",
         "Set design variables, merged with those the part has: name to expression with units "
         "(\"12 mm\", \"30 deg\", \"wall * 3\", \"4\" for a count; a bare JSON number is mm). "
         "Drive feature parameters from them with edit_feature's expressions. remove deletes "
         "variables.",
         object({{"variables", object(json::object())},
                 {"remove", arraySchema(json{{"type", "string"}}, "variable names to delete")}})},
        &setVariables);
}

}  // namespace hz::ai::detail
