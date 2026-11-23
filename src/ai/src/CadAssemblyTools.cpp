// The assembly tools: what an assembly holds, inserting and placing its
// components, mating them, and checking it. Every change is one undo step on
// the assembly's tab, made as the Assembly menu makes it: the assembly
// changed, its mates solved, and left as it was when they cannot hold it.

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include "CadToolsCommon.h"
#include "horizon/document/AssemblyDocument.h"
#include "horizon/document/AssemblyMates.h"
#include "horizon/document/BillOfMaterials.h"
#include "horizon/document/Document.h"
#include "horizon/document/DocumentManager.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/document/ModelCommands.h"
#include "horizon/document/UndoStack.h"
#include "horizon/math/BoundingBox.h"
#include "horizon/math/Mat4.h"
#include "horizon/math/Quaternion.h"
#include "horizon/modeling/EdgeProjection.h"
#include "horizon/modeling/FacePlane.h"
#include "horizon/modeling/MateGeometry.h"
#include "horizon/modeling/Naming.h"
#include "horizon/topology/Solid.h"
#include "horizon/topology/TopologyID.h"

namespace hz::ai {

namespace detail {
namespace {

namespace fs = std::filesystem;
using math::Mat4;
using math::Vec3;

struct AssemblyContext {
    doc::AssemblyDocument& assembly;
    doc::Document& backing;
    doc::DocumentManager& documents;
    AssemblyToolHooks hooks;

    void changed() const {
        if (hooks.assemblyChanged) hooks.assemblyChanged();
    }
};
using Assembly = std::shared_ptr<AssemblyContext>;

/// No more of a component's faces, edges or datums listed than this, nor of
/// the files in the assembly's folder: a model reads every one.
constexpr std::size_t kMaxListed = 80;

std::string lower(std::string text) {
    std::transform(text.begin(), text.end(), text.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return text;
}

/// The folder a relative part path is taken from: the assembly's file's;
/// none while it is unsaved.
std::string folderOf(const doc::AssemblyDocument& assembly) {
    return assembly.filePath().empty() ? std::string()
                                       : fs::path(assembly.filePath()).parent_path().string();
}

// --- Placements -------------------------------------------------------------

Vec3 positionOf(const Mat4& t) {
    return {t.at(0, 3), t.at(1, 3), t.at(2, 3)};
}

/// The rotation of rigid @p t as an axis and an angle in degrees.
json rotationJson(const Mat4& t) {
    const double trace = t.at(0, 0) + t.at(1, 1) + t.at(2, 2);
    const double angle = std::acos(std::clamp((trace - 1.0) / 2.0, -1.0, 1.0));
    Vec3 axis(0.0, 0.0, 1.0);
    if (angle > 1e-9) {
        const double s = std::sin(angle);
        if (s > 1e-6) {
            axis = Vec3(t.at(2, 1) - t.at(1, 2), t.at(0, 2) - t.at(2, 0), t.at(1, 0) - t.at(0, 1)) *
                   (1.0 / (2.0 * s));
        } else {
            // A half turn: the axis from the diagonal, R = 2aa^T - I.
            const double xx = std::sqrt(std::max(0.0, (t.at(0, 0) + 1.0) / 2.0));
            const double yy = std::sqrt(std::max(0.0, (t.at(1, 1) + 1.0) / 2.0));
            const double zz = std::sqrt(std::max(0.0, (t.at(2, 2) + 1.0) / 2.0));
            if (xx >= yy && xx >= zz) {
                axis = {xx, t.at(0, 1) / (2.0 * xx), t.at(0, 2) / (2.0 * xx)};
            } else if (yy >= zz) {
                axis = {t.at(0, 1) / (2.0 * yy), yy, t.at(1, 2) / (2.0 * yy)};
            } else {
                axis = {t.at(0, 2) / (2.0 * zz), t.at(1, 2) / (2.0 * zz), zz};
            }
        }
        axis = axis.normalized();
    }
    return {{"axis", toJson(axis)}, {"angle", round6(angle / kDegree)}};
}

/// A rotation a model gives: {"axis": [x, y, z], "angle": degrees}, about Z
/// when it names no axis.
Mat4 rotationFrom(const json& in, const char* key) {
    const json& r = in[key];
    if (!r.is_object()) {
        throw ArgError(std::string("\"") + key +
                       "\" must be {\"axis\": [x, y, z], \"angle\": degrees}");
    }
    const Vec3 axis = direction(r, "axis", Vec3::UnitZ);
    return Mat4::rotation(math::Quaternion::fromAxisAngle(axis, number(r, "angle") * kDegree));
}

/// Its rotation alone (no position).
Mat4 rotationPart(const Mat4& t) {
    Mat4 r = t;
    r.at(0, 3) = r.at(1, 3) = r.at(2, 3) = 0.0;
    return r;
}

/// Where @p comp is, in the assembly: its mesh placed by its transform.
math::BoundingBox placedBounds(const doc::ComponentInstance& comp) {
    math::BoundingBox box;
    const auto mesh = comp.mesh();  // mirrored, for a mirrored one
    if (!mesh) return box;
    const auto& p = mesh->positions;
    for (std::size_t i = 0; i + 2 < p.size(); i += 3) {
        box.expand(comp.transform.transformPoint(Vec3(p[i], p[i + 1], p[i + 2])));
    }
    return box;
}

json boundsJson(const math::BoundingBox& box) {
    return {{"min", toJson(box.min())}, {"max", toJson(box.max())}, {"size", toJson(box.size())}};
}

// --- Components and mates as a model reads them ------------------------------

std::string labelOf(const doc::ComponentInstance& comp) {
    return std::to_string(comp.id) + " (" + (comp.name.empty() ? "component" : comp.name) + ")";
}

json componentJson(const doc::ComponentInstance& comp) {
    json out = {{"id", comp.id},
                {"name", comp.name},
                {"file", comp.partPath},
                {"position", toJson(positionOf(comp.transform))},
                {"rotation", rotationJson(comp.transform)}};
    if (comp.suppressed) out["suppressed"] = true;
    if (comp.mirrored) out["mirrored"] = true;
    if (comp.isAssembly()) out["subassembly"] = true;
    if (comp.isPatternInstance()) {
        out["pattern"] = {{"pattern", comp.patternId},
                          {"seed", comp.seedId},
                          {"index", comp.patternIndex},
                          {"note", "placed by its pattern, from its seed: not mated or moved"}};
    }
    const math::BoundingBox box = placedBounds(comp);
    if (box.isValid()) {
        out["bounding_box"] = boundsJson(box);
    } else if (!comp.mesh()) {
        out["loaded"] = false;  // its file could not be read
    }
    return out;
}

const std::map<doc::MateType, std::string>& mateNames() {
    static const std::map<doc::MateType, std::string> names = {
        {doc::MateType::Coincident, "coincident"}, {doc::MateType::Concentric, "concentric"},
        {doc::MateType::Distance, "distance"},     {doc::MateType::Angle, "angle"},
        {doc::MateType::Parallel, "parallel"},     {doc::MateType::Perpendicular, "perpendicular"},
        {doc::MateType::Tangent, "tangent"},       {doc::MateType::Fixed, "fixed"}};
    return names;
}

std::string referenceText(const doc::MateReference& ref) {
    const char* kind = ref.kind == doc::ReferenceKind::Edge    ? "edge:"
                       : ref.kind == doc::ReferenceKind::Datum ? "datum:"
                                                               : "face:";
    return kind + ref.faceId.tag();
}

json mateJson(const doc::Mate& mate) {
    json out = {{"id", mate.id},
                {"type", mateNames().at(mate.type)},
                {"a", {{"component", mate.a.componentId}, {"geometry", referenceText(mate.a)}}}};
    if (mate.type != doc::MateType::Fixed) {
        out["b"] = {{"component", mate.b.componentId}, {"geometry", referenceText(mate.b)}};
    }
    const bool angular = mate.type == doc::MateType::Angle;
    const double unit = angular ? kDegree : 1.0;
    const char* valueKey = angular ? "angle" : "distance";
    if (mate.type == doc::MateType::Distance || angular) {
        out[valueKey] = round6(mate.value / unit);
        if (mate.minimum) out["minimum"] = round6(*mate.minimum / unit);
        if (mate.maximum) out["maximum"] = round6(*mate.maximum / unit);
    }
    return out;
}

json patternJson(const doc::ComponentPattern& pattern) {
    const bool linear = pattern.kind == doc::ComponentPattern::Kind::Linear;
    json out = {{"id", pattern.id},
                {"name", pattern.name},
                {"kind", linear ? "linear" : "circular"},
                {"seeds", pattern.seeds},
                {"count", pattern.count},
                {linear ? "direction" : "axis", toJson(pattern.direction)}};
    if (linear) {
        out["spacing"] = round6(pattern.spacing);
    } else {
        out["axis_point"] = toJson(pattern.axisPoint);
        out["angle"] = round6(pattern.spacing / kDegree);
    }
    if (!pattern.skipped.empty()) out["skipped"] = pattern.skipped;
    return out;
}

// --- The geometry a mate can take ---------------------------------------------

const char* shapeName(model::MateFrameKind kind) {
    switch (kind) {
        case model::MateFrameKind::Planar:
            return "plane";
        case model::MateFrameKind::Cylindrical:
            return "cylinder";
        case model::MateFrameKind::Line:
            return "line";
        case model::MateFrameKind::Point:
            return "point";
        case model::MateFrameKind::Circle:
            return "circle";
        case model::MateFrameKind::Spherical:
            return "sphere";
        case model::MateFrameKind::Conical:
            return "cone";
    }
    return "other";
}

/// The world axis @p direction runs along, "+Z" or "-X"; empty for none.
std::string facing(const Vec3& direction) {
    static const std::pair<Vec3, const char*> axes[] = {{{1, 0, 0}, "+X"}, {{-1, 0, 0}, "-X"},
                                                        {{0, 1, 0}, "+Y"}, {{0, -1, 0}, "-Y"},
                                                        {{0, 0, 1}, "+Z"}, {{0, 0, -1}, "-Z"}};
    for (const auto& [axis, name] : axes) {
        if (direction.dot(axis) > std::cos(2.0 * kDegree)) return name;
    }
    return {};
}

/// One thing on a component a mate can take, where it is in the assembly.
struct Geometry {
    doc::ReferenceKind kind = doc::ReferenceKind::Face;
    std::string name;   ///< its name, as a mate keeps it
    std::string label;  ///< a datum's own name
    model::MateFrame frame;

    std::string reference() const {
        const char* prefix = kind == doc::ReferenceKind::Edge    ? "edge:"
                             : kind == doc::ReferenceKind::Datum ? "datum:"
                                                                 : "face:";
        return prefix + name;
    }
};

/// What a mate can take on @p comp, as the Add Mate form offers it: faces (a
/// curved face's facets together), straight and round edges, and its part's
/// datums; their frames placed in the assembly.
std::vector<Geometry> geometryOf(const doc::ComponentInstance& comp, bool edges) {
    std::vector<Geometry> out;
    if (const topo::Solid* solid = comp.solid()) {
        std::set<std::string> seen;
        for (const auto& face : solid->faces()) {
            if (!face.alive || !face.topoId.isValid()) continue;
            const std::string logical = model::logicalFace(face.topoId.tag());
            if (!seen.insert(logical).second) continue;
            if (const auto frame = model::MateGeometry::frameForFace(face)) {
                out.push_back(
                    {doc::ReferenceKind::Face, logical, {}, frame->transformed(comp.transform)});
            }
        }
        if (edges) {
            seen.clear();
            for (const auto& edge : solid->edges()) {
                if (!edge.topoId.isValid()) continue;
                if (edge.topoId.tag().find("/seam:") != std::string::npos) continue;
                const std::string whole = model::wholeEdgeName(edge.topoId.tag());
                if (!seen.insert(whole).second) continue;
                if (const auto frame = model::MateGeometry::frameForEdge(*solid, whole)) {
                    out.push_back(
                        {doc::ReferenceKind::Edge, whole, {}, frame->transformed(comp.transform)});
                }
            }
        }
    }
    if (comp.resolvedPart) {
        const doc::FeatureTree& tree = comp.resolvedPart->featureTree();
        for (std::size_t i = 0; i < tree.featureCount(); ++i) {
            const auto* datum = dynamic_cast<const doc::DatumFeature*>(tree.feature(i));
            if (datum == nullptr) continue;
            out.push_back({doc::ReferenceKind::Datum, datum->featureID(), datum->name(),
                           doc::datumFrame(*datum, comp.mirrored).transformed(comp.transform)});
        }
    }
    return out;
}

json geometryJson(const Geometry& g) {
    json out = {{"geometry", g.reference()},
                {"shape", shapeName(g.frame.kind)},
                {"origin", toJson(g.frame.origin)}};
    if (!g.label.empty()) out["label"] = g.label;
    const bool directed = g.frame.kind != model::MateFrameKind::Point &&
                          g.frame.kind != model::MateFrameKind::Spherical;
    if (directed) {
        out[g.frame.kind == model::MateFrameKind::Planar ? "normal" : "axis"] =
            toJson(g.frame.direction);
        const std::string way = facing(g.frame.direction);
        if (!way.empty() && g.frame.kind == model::MateFrameKind::Planar) out["facing"] = way;
    }
    if (g.frame.radius > 0.0) out["radius"] = round6(g.frame.radius);
    if (g.frame.kind == model::MateFrameKind::Conical) {
        out["half_angle"] = round6(g.frame.angle / kDegree);
    }
    return out;
}

// --- Finding what a model names ------------------------------------------------

/// The part of @p comp read whole, so its faces and datums are there.
void resolve(const AssemblyContext& c, doc::ComponentInstance& comp) {
    if (comp.solid() == nullptr || !comp.resolvedPart) {
        c.documents.resolveComponent(comp, doc::ComponentState::Resolved, folderOf(c.assembly));
    }
}

std::string componentList(const doc::AssemblyDocument& assembly) {
    std::string list;
    for (const auto& comp : assembly.components()) {
        list += (list.empty() ? "" : ", ") + labelOf(comp);
    }
    return list.empty() ? "none yet" : list;
}

/// The component @p key names: by its id, or by its name when no other
/// component has that name.
doc::ComponentInstance& componentOf(const AssemblyContext& c, const json& in, const char* key) {
    if (!has(in, key)) throw ArgError(std::string("\"") + key + "\" is required");
    const json& v = in[key];
    doc::ComponentInstance* found = nullptr;
    if (v.is_string() && !v.get<std::string>().empty() &&
        !std::all_of(v.get<std::string>().begin(), v.get<std::string>().end(),
                     [](char ch) { return ch >= '0' && ch <= '9'; })) {
        const std::string name = lower(v.get<std::string>());
        for (auto& comp : c.assembly.components()) {
            if (lower(comp.name) != name) continue;
            if (found != nullptr) {
                throw ArgError("more than one component is called \"" + v.get<std::string>() +
                               "\"; name it by its id: " + componentList(c.assembly));
            }
            found = &comp;
        }
    } else {
        found = c.assembly.component(id(in, key));
    }
    if (found == nullptr) {
        throw ArgError("no such component; the components are: " + componentList(c.assembly));
    }
    return *found;
}

/// The geometry a model names on @p comp: "face:...", "edge:..." or
/// "datum:..." as list_component_geometry gives it, a bare name of one, or
/// top, bottom, front, back, left or right (the outermost flat face facing
/// that way in the assembly).
doc::MateReference referenceOn(const AssemblyContext& c, doc::ComponentInstance& comp,
                               const std::string& given) {
    resolve(c, comp);
    const std::vector<Geometry> all = geometryOf(comp, true);
    doc::MateReference ref;
    ref.componentId = comp.id;
    const auto take = [&ref](const Geometry& g) {
        ref.kind = g.kind;
        ref.faceId = topo::TopologyID::fromTag(g.name);
        return ref;
    };

    static const std::map<std::string, Vec3> sides = {{"top", {0, 0, 1}},    {"bottom", {0, 0, -1}},
                                                      {"front", {0, -1, 0}}, {"back", {0, 1, 0}},
                                                      {"right", {1, 0, 0}},  {"left", {-1, 0, 0}}};
    const auto side = sides.find(lower(given));
    if (side != sides.end()) {
        const Geometry* best = nullptr;
        double reach = -std::numeric_limits<double>::max();
        for (const auto& g : all) {
            if (g.kind != doc::ReferenceKind::Face ||
                g.frame.kind != model::MateFrameKind::Planar) {
                continue;
            }
            if (g.frame.direction.dot(side->second) < std::cos(2.0 * kDegree)) continue;
            if (g.frame.origin.dot(side->second) > reach) {
                reach = g.frame.origin.dot(side->second);
                best = &g;
            }
        }
        if (best == nullptr) {
            throw ArgError(labelOf(comp) + " has no flat face facing " + side->first +
                           "; use a name from list_component_geometry");
        }
        return take(*best);
    }

    const auto colon = given.find(':');
    const std::string kind = colon == std::string::npos ? std::string() : given.substr(0, colon);
    const bool prefixed = kind == "face" || kind == "edge" || kind == "datum";
    const std::string name = prefixed ? given.substr(colon + 1) : given;
    for (const auto& g : all) {
        if (prefixed && g.reference() != given) continue;
        if (g.name == name || (g.kind == doc::ReferenceKind::Face &&
                               g.name == model::logicalFace(model::wholeFaceName(name)))) {
            return take(g);
        }
        // A datum by its own name ("Plane 1").
        if (g.kind == doc::ReferenceKind::Datum && lower(g.label) == lower(name)) return take(g);
    }
    if (all.empty()) {
        throw ArgError(labelOf(comp) + " has no geometry to mate: its part could not be read");
    }
    throw ArgError("\"" + given + "\" is not on " + labelOf(comp) +
                   "; list_component_geometry names what is, or say top, bottom, front, back, "
                   "left or right");
}

doc::Mate& mateOf(const AssemblyContext& c, const json& in, const char* key) {
    doc::Mate* mate = c.assembly.mate(id(in, key));
    if (mate == nullptr) {
        std::string ids;
        for (const auto& m : c.assembly.mates())
            ids += (ids.empty() ? "" : ", ") + std::to_string(m.id);
        throw ArgError("no such mate; the mates are: " + (ids.empty() ? "none yet" : ids));
    }
    return *mate;
}

/// What the solve of a change said, for the model.
json solveNote(const AssemblyContext& c, const doc::MatePlacement& placement) {
    if (c.assembly.mates().empty()) return "no mates: each component stays where it is put";
    const model::AssemblySolveResult& result = placement.result;
    json out = {{"solved", true}, {"freedom_left", result.remainingDOF}};
    if (result.redundantCount > 0) out["redundant_constraints"] = result.redundantCount;
    if (!result.ungroundedComponents.empty()) {
        json free = json::array();
        for (const uint64_t id : result.ungroundedComponents) {
            const auto* comp = c.assembly.component(id);
            free.push_back(comp != nullptr ? labelOf(*comp) : std::to_string(id));
        }
        out["not_connected_to_ground"] = free;
    }
    return out;
}

/// Change the assembly by @p edit as the Assembly menu does: one undo step
/// named @p verb, its patterns and mates brought up to date after it. When
/// the mates cannot hold it, or @p edit throws, the assembly is left as it
/// was and the reason is the error.
ToolResult editAssembly(const AssemblyContext& c, const std::string& verb,
                        const std::function<json()>& edit) {
    const bool wasDirty = c.assembly.isDirty();
    doc::AssemblyState before = c.assembly.snapshot();
    const auto putBack = [&] {
        c.assembly.restore(before);
        c.assembly.setDirty(wasDirty);
    };
    json report;
    try {
        report = edit();
    } catch (...) {
        putBack();
        throw;
    }
    c.assembly.updatePatterns();  // instances follow their seeds (Phase 161)
    const doc::MatePlacement placement = doc::placeByMates(c.assembly, c.documents);
    if (!placement.placed) {
        putBack();
        return ToolResult::error(verb + " was not made: the mates cannot hold it (" +
                                 placement.why + "). Nothing changed.");
    }
    // From here the undo stack carries the change: undone back to the saved
    // state, the tab is unmodified again, which the assembly's own flag would
    // not say.
    c.assembly.setDirty(wasDirty);
    c.backing.undoStack().push(std::make_unique<doc::AssemblyEditCommand>(
        c.assembly, std::move(before), c.assembly.snapshot(), verb));
    c.changed();
    if (!report.is_object()) report = json::object();
    report["mates"] = solveNote(c, placement);
    return ToolResult::ok(report, true);
}

/// Where component @p id is now, for a report after a change.
json placementOf(const AssemblyContext& c, uint64_t id) {
    const doc::ComponentInstance* comp = c.assembly.component(id);
    return comp != nullptr ? componentJson(*comp) : json(nullptr);
}

// --- The queries ---------------------------------------------------------------

std::vector<std::string> filesNear(const std::string& folder) {
    std::vector<std::string> files;
    if (folder.empty()) return files;
    std::error_code ec;
    fs::directory_iterator it(folder, ec);
    for (; !ec && it != fs::directory_iterator(); it.increment(ec)) {
        const std::string ext = lower(it->path().extension().string());
        if (ext == ".hzpart" || ext == ".hzasm") files.push_back(it->path().filename().string());
    }
    std::sort(files.begin(), files.end());
    return files;
}

ToolResult getAssemblyInfo(const AssemblyContext& c, const json& /*in*/) {
    const doc::AssemblyDocument& a = c.assembly;
    json components = json::array();
    for (const auto& comp : a.components()) components.push_back(componentJson(comp));
    json mates = json::array();
    for (const auto& mate : a.mates()) mates.push_back(mateJson(mate));
    json out = {{"type", "assembly"},
                {"file", a.filePath().empty() ? json(nullptr) : json(a.filePath())},
                {"units", "mm, degrees"},
                {"components", components},
                {"mates", mates}};
    if (!a.patterns().empty()) {
        json patterns = json::array();
        for (const auto& pattern : a.patterns()) patterns.push_back(patternJson(pattern));
        out["patterns"] = patterns;
    }
    if (!a.explodedViews().empty()) {
        json views = json::array();
        for (const auto& view : a.explodedViews()) {
            views.push_back({{"id", view.id}, {"name", view.name}, {"steps", view.steps.size()}});
        }
        out["exploded_views"] = views;
    }
    const std::string folder = folderOf(a);
    if (folder.empty()) {
        out["folder"] = nullptr;
        out["note"] = "The assembly is not saved yet: insert parts by their full paths.";
    } else {
        out["folder"] = folder;
        std::vector<std::string> files = filesNear(folder);
        if (files.size() > kMaxListed) {
            out["files_not_listed"] = files.size() - kMaxListed;
            files.resize(kMaxListed);
        }
        out["files_in_folder"] = files;
    }
    return ToolResult::ok(out);
}

ToolResult listComponentGeometry(const AssemblyContext& c, const json& in) {
    doc::ComponentInstance& comp = componentOf(c, in, "component");
    resolve(c, comp);
    const bool edges = boolean(in, "edges", false);
    const std::vector<Geometry> all = geometryOf(comp, edges);
    json items = json::array();
    std::size_t left = 0;
    for (const auto& g : all) {
        if (items.size() >= kMaxListed) {
            ++left;
            continue;
        }
        items.push_back(geometryJson(g));
    }
    json out = {{"component", labelOf(comp)}, {"geometry", items}};
    if (left > 0) out["not_listed"] = left;
    if (all.empty()) out["note"] = "Its part could not be read, so it has nothing to mate.";
    if (!edges) out["edges"] = "not listed: ask with edges true for straight and round edges";
    return ToolResult::ok(out);
}

ToolResult checkInterference(const AssemblyContext& c, const json& /*in*/) {
    for (auto& comp : c.assembly.components()) {
        if (!comp.suppressed) resolve(c, comp);
    }
    const doc::InterferenceReport report = c.assembly.findInterference();
    json pairs = json::array();
    const auto label = [&c](uint64_t id) {
        const auto* comp = c.assembly.component(id);
        return comp != nullptr ? labelOf(*comp) : std::to_string(id);
    };
    for (const auto& pair : report.pairs) {
        json item = {{"a", label(pair.componentA)}, {"b", label(pair.componentB)}};
        if (pair.volumeResolved) {
            item["shared_volume_mm3"] = round6(pair.volume);
        } else {
            item["shared_volume_mm3"] = "could not be measured";
        }
        pairs.push_back(item);
    }
    json out = {{"interfering", pairs}, {"clear", report.pairs.empty()}};
    if (!report.unchecked.empty()) {
        json unchecked = json::array();
        for (const uint64_t id : report.unchecked) unchecked.push_back(label(id));
        out["not_checked"] = unchecked;
        out["note"] = "Their parts could not be read: nothing is known of them.";
    }
    return ToolResult::ok(out);
}

ToolResult billOfMaterials(const AssemblyContext& c, const json& in) {
    const std::string kind =
        choice(in, "kind", {"top_level", "indented", "parts_only"}, std::string("top_level"));
    const doc::BomKind bomKind = kind == "indented"     ? doc::BomKind::Indented
                                 : kind == "parts_only" ? doc::BomKind::PartsOnly
                                                        : doc::BomKind::TopLevel;
    const doc::BillOfMaterials bom = doc::BomGenerator::generate(c.assembly, bomKind);
    json lines = json::array();
    for (const auto& line : bom.lines) {
        json item = {{"item", line.index.empty() ? std::to_string(line.item) : line.index},
                     {"part", line.partName},
                     {"file", line.partPath},
                     {"quantity", line.quantity}};
        if (line.assembly) item["subassembly"] = true;
        lines.push_back(item);
    }
    return ToolResult::ok({{"lines", lines}, {"total_quantity", bom.totalQuantity()}});
}

// --- The changes ---------------------------------------------------------------

ToolResult insertComponent(const AssemblyContext& c, const json& in) {
    const std::string given = text(in, "file");
    if (given.empty()) throw ArgError("\"file\" must name a part (.hzpart) or assembly (.hzasm)");
    const std::string folder = folderOf(c.assembly);
    fs::path path(given);
    if (!path.is_absolute()) {
        if (folder.empty()) {
            throw ArgError("the assembly is not saved, so \"" + given +
                           "\" has no folder to be in: give its full path");
        }
        path = fs::path(folder) / path;
    }
    path = path.lexically_normal();
    const std::string ext = lower(path.extension().string());
    if (ext != ".hzpart" && ext != ".hzasm") {
        throw ArgError("\"" + given + "\" is not a part (.hzpart) or an assembly (.hzasm)");
    }
    std::error_code ec;
    if (!fs::is_regular_file(path, ec)) {
        std::string near;
        for (const auto& file : filesNear(folder)) near += (near.empty() ? "" : ", ") + file;
        throw ArgError("there is no file " + path.string() +
                       (near.empty() ? std::string() : "; the folder has: " + near));
    }

    doc::ComponentInstance comp;
    comp.partPath = path.string();
    comp.name = has(in, "name") ? text(in, "name") : path.stem().string();
    // Not this assembly, nor one that places it: an assembly inside itself.
    std::vector<std::string> within;
    if (!c.assembly.filePath().empty()) {
        within.push_back(doc::DocumentManager::canonicalPath(c.assembly.filePath()));
    }
    std::string why;
    if (!c.documents.resolveComponent(comp, doc::ComponentState::Lightweight, folder, &why,
                                      within)) {
        return ToolResult::error(comp.name + " is not inserted: " +
                                 (why.empty() ? "no geometry could be made from it" : why));
    }

    if (has(in, "position") || has(in, "rotation")) {
        const Mat4 turn = has(in, "rotation") ? rotationFrom(in, "rotation") : Mat4::identity();
        comp.transform = Mat4::translation(vec3(in, "position", Vec3())) * turn;
    } else {
        // Beside the others, along +X, not on top of them at the origin, as
        // the Assembly menu places it.
        math::BoundingBox others;
        for (const auto& placed : c.assembly.components()) {
            if (!placed.suppressed) others.expand(placedBounds(placed));
        }
        const math::BoundingBox own = placedBounds(comp);
        if (others.isValid() && own.isValid()) {
            const double gap = 0.1 * std::max(others.size().x, own.size().x);
            comp.transform = Mat4::translation(Vec3(others.max().x + gap - own.min().x, 0.0, 0.0));
        }
    }

    uint64_t added = 0;
    ToolResult result = editAssembly(c, "Insert Component", [&] {
        added = c.assembly.addComponent(std::move(comp));
        return json::object();
    });
    if (!result.isError) {
        json report = json::parse(result.content);
        report["inserted"] = placementOf(c, added);
        report["next"] =
            "Place it with mates (add_mate; list_component_geometry names its faces) or move it "
            "(place_component).";
        result.content = report.dump();
    }
    return result;
}

ToolResult placeComponent(const AssemblyContext& c, const json& in) {
    doc::ComponentInstance& target = componentOf(c, in, "component");
    if (target.isPatternInstance()) {
        throw ArgError(labelOf(target) + " is placed by its pattern: move its seed instead");
    }
    const bool any =
        has(in, "position") || has(in, "rotation") || has(in, "move_by") || has(in, "rotate_by");
    if (!any) throw ArgError("give position, rotation, move_by or rotate_by");
    const uint64_t id = target.id;
    Mat4 placed = target.transform;
    if (has(in, "position") || has(in, "rotation")) {
        const Mat4 turn = has(in, "rotation") ? rotationFrom(in, "rotation") : rotationPart(placed);
        placed = Mat4::translation(vec3(in, "position", positionOf(placed))) * turn;
    }
    if (has(in, "rotate_by")) {
        // About its middle, unless the model says about where.
        const json& r = in["rotate_by"];
        if (!r.is_object())
            throw ArgError("\"rotate_by\" must be {\"axis\", \"angle\", \"about\"}");
        doc::ComponentInstance probe = target;
        probe.transform = placed;
        const math::BoundingBox box = placedBounds(probe);
        const Vec3 about = vec3(r, "about", box.isValid() ? box.center() : positionOf(placed));
        placed = Mat4::translation(about) * rotationFrom(in, "rotate_by") *
                 Mat4::translation(about * -1.0) * placed;
    }
    if (has(in, "move_by")) placed = Mat4::translation(vec3(in, "move_by")) * placed;

    const Mat4 asked = placed;
    ToolResult result = editAssembly(c, "Place Component", [&] {
        c.assembly.component(id)->transform = placed;
        return json::object();
    });
    if (!result.isError) {
        json report = json::parse(result.content);
        report["component"] = placementOf(c, id);
        const doc::ComponentInstance* now = c.assembly.component(id);
        if (now != nullptr && positionOf(now->transform).distanceTo(positionOf(asked)) > 1e-6) {
            report["note"] =
                "Its mates moved it from where it was asked to go: change or delete them to "
                "place it freely.";
        }
        result.content = report.dump();
    }
    return result;
}

ToolResult addMate(const AssemblyContext& c, const json& in) {
    const std::string type = choice(in, "type",
                                    {"coincident", "concentric", "distance", "angle", "parallel",
                                     "perpendicular", "tangent", "fixed"});
    doc::Mate mate;
    for (const auto& [value, name] : mateNames()) {
        if (name == type) mate.type = value;
    }
    const bool fixed = mate.type == doc::MateType::Fixed;
    const auto side = [&c, &in, fixed](const char* key) {
        if (!has(in, key) || !in[key].is_object()) {
            throw ArgError(std::string("\"") + key +
                           "\" must be {\"component\": id, \"geometry\": name}");
        }
        doc::ComponentInstance& comp = componentOf(c, in[key], "component");
        if (comp.isPatternInstance()) {
            throw ArgError(labelOf(comp) + " is placed by its pattern: mate its seed instead");
        }
        // A fixed mate grounds the component where it is: what it names on
        // it says nothing more.
        if (fixed && !has(in[key], "geometry")) {
            doc::MateReference whole;
            whole.componentId = comp.id;
            return whole;
        }
        return referenceOn(c, comp, text(in[key], "geometry"));
    };
    mate.a = side("a");
    // A fixed mate has no B, as the Assembly menu makes it.
    if (!fixed) {
        mate.b = side("b");
        if (mate.b.componentId == mate.a.componentId) {
            throw ArgError("a mate joins two components: \"a\" and \"b\" are both " +
                           labelOf(*c.assembly.component(mate.a.componentId)));
        }
    }
    const bool angular = mate.type == doc::MateType::Angle;
    if (mate.type == doc::MateType::Distance) mate.value = number(in, "distance");
    if (angular) mate.value = number(in, "angle") * kDegree;
    if (mate.type == doc::MateType::Distance || angular) {
        const double unit = angular ? kDegree : 1.0;
        if (has(in, "minimum")) mate.minimum = number(in, "minimum") * unit;
        if (has(in, "maximum")) mate.maximum = number(in, "maximum") * unit;
        if (mate.minimum && mate.maximum && *mate.minimum > *mate.maximum) {
            throw ArgError("\"minimum\" is more than \"maximum\"");
        }
    }
    uint64_t added = 0;
    ToolResult result = editAssembly(c, "Add Mate", [&] {
        added = c.assembly.addMate(mate);
        return json::object();
    });
    if (!result.isError) {
        json report = json::parse(result.content);
        if (const doc::Mate* made = c.assembly.mate(added)) report["mate"] = mateJson(*made);
        report["moved"] = placementOf(c, fixed ? mate.a.componentId : mate.b.componentId);
        result.content = report.dump();
    }
    return result;
}

ToolResult editMate(const AssemblyContext& c, const json& in) {
    doc::Mate& mate = mateOf(c, in, "mate");
    const bool angular = mate.type == doc::MateType::Angle;
    if (mate.type != doc::MateType::Distance && !angular) {
        throw ArgError(
            "only a distance or an angle mate has a value to change; delete and add "
            "a mate to change another");
    }
    const double unit = angular ? kDegree : 1.0;
    doc::Mate changed = mate;
    const char* valueKey = angular ? "angle" : "distance";
    if (has(in, valueKey)) changed.value = number(in, valueKey) * unit;
    if (has(in, "minimum")) {
        if (in["minimum"].is_null()) {
            changed.minimum.reset();
        } else {
            changed.minimum = number(in, "minimum") * unit;
        }
    }
    if (has(in, "maximum")) {
        if (in["maximum"].is_null()) {
            changed.maximum.reset();
        } else {
            changed.maximum = number(in, "maximum") * unit;
        }
    }
    if (changed.minimum && changed.maximum && *changed.minimum > *changed.maximum) {
        throw ArgError("\"minimum\" is more than \"maximum\"");
    }
    const uint64_t id = changed.id;
    ToolResult result = editAssembly(c, "Edit Mate", [&] {
        *c.assembly.mate(id) = changed;
        return json::object();
    });
    if (!result.isError) {
        json report = json::parse(result.content);
        if (const doc::Mate* now = c.assembly.mate(id)) report["mate"] = mateJson(*now);
        result.content = report.dump();
    }
    return result;
}

ToolResult deleteMate(const AssemblyContext& c, const json& in) {
    const uint64_t id = mateOf(c, in, "mate").id;
    return editAssembly(c, "Remove Mate", [&] {
        c.assembly.removeMate(id);
        return json{{"deleted_mate", id}};
    });
}

ToolResult deleteComponent(const AssemblyContext& c, const json& in) {
    const doc::ComponentInstance& comp = componentOf(c, in, "component");
    const uint64_t id = comp.id;
    const std::string label = labelOf(comp);
    return editAssembly(c, "Remove Component", [&] {
        const std::size_t mates = c.assembly.mates().size();
        c.assembly.removeComponent(id);
        json out = {{"deleted", label}};
        if (c.assembly.mates().size() < mates) {
            out["mates_deleted_with_it"] = mates - c.assembly.mates().size();
        }
        return out;
    });
}

ToolResult suppressComponent(const AssemblyContext& c, const json& in) {
    doc::ComponentInstance& comp = componentOf(c, in, "component");
    const bool suppress = boolean(in, "suppressed", true);
    if (comp.suppressed == suppress) {
        return ToolResult::ok(json{{"unchanged", true}, {"suppressed", suppress}});
    }
    const uint64_t id = comp.id;
    return editAssembly(c, suppress ? "Suppress Component" : "Unsuppress Component", [&] {
        c.assembly.component(id)->suppressed = suppress;
        return json{{"component", labelOf(*c.assembly.component(id))}, {"suppressed", suppress}};
    });
}

ToolResult renameComponent(const AssemblyContext& c, const json& in) {
    doc::ComponentInstance& comp = componentOf(c, in, "component");
    const std::string name = text(in, "name");
    if (name.empty()) throw ArgError("\"name\" must not be empty");
    const uint64_t id = comp.id;
    return editAssembly(c, "Rename Component", [&] {
        c.assembly.component(id)->name = name;
        return json{{"component", labelOf(*c.assembly.component(id))}};
    });
}

ToolResult patternComponents(const AssemblyContext& c, const json& in) {
    if (!has(in, "components") || !in["components"].is_array() || in["components"].empty()) {
        throw ArgError("\"components\" must list the components to repeat");
    }
    doc::ComponentPattern pattern;
    for (const auto& item : in["components"]) {
        const json holder = {{"component", item}};
        const doc::ComponentInstance& comp = componentOf(c, holder, "component");
        if (comp.isPatternInstance()) {
            throw ArgError(labelOf(comp) + " is a pattern's instance: repeat its seed instead");
        }
        if (std::find(pattern.seeds.begin(), pattern.seeds.end(), comp.id) == pattern.seeds.end()) {
            pattern.seeds.push_back(comp.id);
        }
    }
    const std::string kind = choice(in, "kind", {"linear", "circular"}, std::string("linear"));
    pattern.count = static_cast<int>(integer(in, "count", 2, doc::ComponentPattern::kMaxCount));
    if (kind == "linear") {
        pattern.kind = doc::ComponentPattern::Kind::Linear;
        pattern.direction = direction(in, "direction");
        pattern.spacing = number(in, "spacing");
        if (pattern.spacing == 0.0) throw ArgError("\"spacing\" must not be 0");
    } else {
        pattern.kind = doc::ComponentPattern::Kind::Circular;
        pattern.direction = direction(in, "axis", Vec3::UnitZ);
        pattern.axisPoint = vec3(in, "axis_point", Vec3());
        // An angle between instances, or the whole turn shared out.
        pattern.spacing = has(in, "angle") ? number(in, "angle") * kDegree
                                           : 2.0 * kPi / static_cast<double>(pattern.count);
        if (pattern.spacing == 0.0) throw ArgError("\"angle\" must not be 0");
    }
    pattern.name = has(in, "name") ? text(in, "name")
                                   : "Pattern " + std::to_string(c.assembly.patterns().size() + 1);
    uint64_t added = 0;
    ToolResult result = editAssembly(c, "Pattern Components", [&] {
        added = c.assembly.addPattern(pattern);
        return json::object();
    });
    if (!result.isError) {
        json report = json::parse(result.content);
        if (const doc::ComponentPattern* made = c.assembly.pattern(added)) {
            report["pattern"] = patternJson(*made);
        }
        std::size_t instances = 0;
        for (const auto& comp : c.assembly.components()) {
            if (comp.patternId == added) ++instances;
        }
        report["instances_made"] = instances;
        result.content = report.dump();
    }
    return result;
}

ToolResult deletePattern(const AssemblyContext& c, const json& in) {
    const uint64_t id = detail::id(in, "pattern");
    if (c.assembly.pattern(id) == nullptr) throw ArgError("no such pattern");
    return editAssembly(c, "Remove Component Pattern", [&] {
        c.assembly.removePattern(id);
        return json{{"deleted_pattern", id}, {"note", "its seeds stay; its instances are gone"}};
    });
}

// --- Schemas ---------------------------------------------------------------------

json componentSchema(const std::string& description) {
    return {{"type", {"integer", "string"}},
            {"description", description + " (its id, or its name)"}};
}

json rotationSchema(const std::string& description) {
    return {{"type", "object"},
            {"description", description},
            {"properties",
             {{"axis", pointSchema("the axis turned about (default Z)")},
              {"angle", numberSchema("degrees, right-handed about the axis")}}},
            {"required", {"angle"}}};
}

json sideSchema(const std::string& description) {
    json schema = object(
        {{"component", componentSchema("the component")},
         {"geometry",
          stringSchema("on it: a name from list_component_geometry (\"face:...\", "
                       "\"edge:...\", \"datum:...\"), or top, bottom, front, back, "
                       "left or right: its outermost flat face facing that way (any for fixed)")}},
        {"component"});
    schema["description"] = description;
    return schema;
}

}  // namespace
}  // namespace detail

void registerAssemblyTools(ToolRegistry& registry, doc::AssemblyDocument& assembly,
                           doc::Document& backing, doc::DocumentManager& documents,
                           AssemblyToolHooks hooks) {
    using namespace detail;
    const auto context = std::make_shared<AssemblyContext>(
        AssemblyContext{assembly, backing, documents, std::move(hooks)});
    const auto add = [&registry, &context](
                         ToolSpec spec, ToolAccess access,
                         ToolResult (*handler)(const AssemblyContext&, const json&)) {
        registry.add(std::move(spec), access,
                     [context, handler](const json& in) { return handler(*context, in); });
    };

    add({"get_assembly_info",
         "The assembly: its components (id, name, file, position, rotation, extent), its mates, "
         "its component patterns, and the part files in its folder. Call it first.",
         object(json::object())},
        ToolAccess::Read, &getAssemblyInfo);
    add({"list_component_geometry",
         "What a mate can take on one component: its faces (planes, cylinders, cones, "
         "spheres), its part's datums, and with edges true its straight and round edges; each "
         "with its name for add_mate and where it is in the assembly (origin, normal or axis, "
         "radius, which way a flat face faces).",
         object({{"component", componentSchema("the component")},
                 {"edges", booleanSchema("list edges too (default false)")}},
                {"component"})},
        ToolAccess::Read, &listComponentGeometry);
    add({"check_interference",
         "Which components share material, and how much (touching faces do not count). Run it "
         "after mating.",
         object(json::object())},
        ToolAccess::Read, &checkInterference);
    add({"get_bill_of_materials",
         "The assembly's parts and how many of each (suppressed components left out).",
         object({{"kind", enumSchema({"top_level", "indented", "parts_only"},
                                     "top_level: a subassembly is one line; indented: its parts "
                                     "under it; parts_only: every part at any depth")}})},
        ToolAccess::Read, &billOfMaterials);

    add({"insert_component",
         "Place a part (.hzpart) or a subassembly (.hzasm) in the assembly, from its file: a "
         "name in the assembly's folder (get_assembly_info lists them) or a full path. Without "
         "a position it goes beside the others. Then mate it.",
         object({{"file", stringSchema("the part or assembly file")},
                 {"name", stringSchema("what to call it (default: the file's name)")},
                 {"position", pointSchema("where its part's origin goes")},
                 {"rotation", rotationSchema("how it is turned")}},
                {"file"})},
        ToolAccess::Modify, &insertComponent);
    add({"place_component",
         "Move or turn a component: to a position and rotation, or by an offset (move_by) or a "
         "turn (rotate_by, about its middle unless about is given). Its mates are solved after: "
         "a mated component goes where they hold it.",
         object({{"component", componentSchema("the component")},
                 {"position", pointSchema("where its part's origin goes")},
                 {"rotation", rotationSchema("its rotation, from its part's own axes")},
                 {"move_by", pointSchema("an offset to move it by")},
                 {"rotate_by",
                  {{"type", "object"},
                   {"description", "a turn to give it"},
                   {"properties",
                    {{"axis", pointSchema("the axis (default Z)")},
                     {"angle", numberSchema("degrees")},
                     {"about", pointSchema("a point on the axis (default its middle)")}}},
                   {"required", {"angle"}}}}},
                {"component"})},
        ToolAccess::Modify, &placeComponent);
    add({"add_mate",
         "Constrain two components (b moves to a): coincident (two flat faces against each "
         "other, facing opposite ways, or a point on a plane), concentric (two cylinders, "
         "axes or round edges on one axis: a pin in a hole), distance (two parallel flat faces "
         "apart by distance), angle (two flat faces at an angle), parallel, perpendicular, "
         "tangent (a cylinder against a plane), fixed (grounds a; no b). The mates are solved at "
         "once; one that cannot hold is not added.",
         object({{"type", enumSchema({"coincident", "concentric", "distance", "angle", "parallel",
                                      "perpendicular", "tangent", "fixed"},
                                     "the kind of mate")},
                 {"a", sideSchema("the first side (the one that stays, usually)")},
                 {"b", sideSchema("the second side (not for fixed)")},
                 {"distance", numberSchema("distance: the gap, mm (may be negative)")},
                 {"angle", numberSchema("angle: degrees")},
                 {"minimum", numberSchema("distance or angle: free from here...")},
                 {"maximum", numberSchema("...to here (a limit mate)")}},
                {"type", "a"})},
        ToolAccess::Modify, &addMate);
    add({"edit_mate", "Change a distance or angle mate's value or limits (null clears a limit).",
         object({{"mate", integerSchema("the mate's id")},
                 {"distance", numberSchema("mm")},
                 {"angle", numberSchema("degrees")},
                 {"minimum", numberSchema("the lower limit, or null")},
                 {"maximum", numberSchema("the upper limit, or null")}},
                {"mate"})},
        ToolAccess::Modify, &editMate);
    add({"delete_mate", "Remove a mate.",
         object({{"mate", integerSchema("the mate's id")}}, {"mate"})},
        ToolAccess::Modify, &deleteMate);
    add({"delete_component", "Remove a component, and its mates.",
         object({{"component", componentSchema("the component")}}, {"component"})},
        ToolAccess::Modify, &deleteComponent);
    add({"suppress_component",
         "Leave a component out (suppressed true) or bring it back (false): out of the view, the "
         "interference check and the bill of materials.",
         object({{"component", componentSchema("the component")},
                 {"suppressed", booleanSchema("default true")}},
                {"component"})},
        ToolAccess::Modify, &suppressComponent);
    add({"rename_component", "Give a component a new name.",
         object({{"component", componentSchema("the component")},
                 {"name", stringSchema("its new name")}},
                {"component", "name"})},
        ToolAccess::Modify, &renameComponent);
    add({"pattern_components",
         "Repeat components along a direction (linear: spacing mm, count) or about an axis "
         "(circular: axis, axis_point, count, and angle between them, default the whole turn "
         "shared out). The instances follow their seeds.",
         object({{"components", arraySchema(json{{"type", {"integer", "string"}}},
                                            "the components to repeat (ids or names)")},
                 {"kind", enumSchema({"linear", "circular"}, "default linear")},
                 {"count", integerSchema("how many, the seeds the first")},
                 {"direction", pointSchema("linear: the direction")},
                 {"spacing", numberSchema("linear: mm between instances")},
                 {"axis", pointSchema("circular: the axis (default Z)")},
                 {"axis_point", pointSchema("circular: a point on the axis (default the origin)")},
                 {"angle", numberSchema("circular: degrees between instances")},
                 {"name", stringSchema("its name")}},
                {"components", "count"})},
        ToolAccess::Modify, &patternComponents);
    add({"delete_component_pattern",
         "Remove a component pattern and its instances; its seeds stay.",
         object({{"pattern", integerSchema("the pattern's id")}}, {"pattern"})},
        ToolAccess::Modify, &deletePattern);
}

}  // namespace hz::ai
