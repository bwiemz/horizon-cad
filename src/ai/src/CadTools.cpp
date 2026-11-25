#include "horizon/ai/CadTools.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <utility>

#include "CadToolsCommon.h"
#include "horizon/document/Commands.h"
#include "horizon/document/Document.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/document/ModelCommands.h"
#include "horizon/document/Sketch.h"
#include "horizon/document/UndoStack.h"
#include "horizon/modeling/FacePlane.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/topology/Solid.h"

namespace hz::ai {

void registerCadTools(ToolRegistry& registry, doc::Document& document, CadToolHooks hooks) {
    const auto context =
        std::make_shared<detail::CadContext>(detail::CadContext{document, std::move(hooks)});
    detail::registerQueryTools(registry, context);
    detail::registerModelTools(registry, context);
    detail::registerDraftTools(registry, context);
    detail::registerAdvancedTools(registry, context);
    detail::registerDrawingTool(registry, context->hooks.makeDrawing, [&document] {
        return detail::DrawingSource{"part", document.filePath(), document.isDirty(),
                                     document.solid() == nullptr};
    });
}

namespace detail {

namespace {

std::string quoted(const char* key) {
    return std::string("\"") + key + "\"";
}

}  // namespace

// --- Arguments ----------------------------------------------------------------

bool has(const json& in, const char* key) {
    return in.is_object() && in.contains(key) && !in[key].is_null();
}

double number(const json& in, const char* key) {
    if (!has(in, key)) throw ArgError(quoted(key) + " is required");
    const json& v = in[key];
    double value = 0.0;
    if (v.is_number()) {
        value = v.get<double>();
    } else if (v.is_string()) {
        // Some models send numbers as strings; take a plain one.
        const std::string s = v.get<std::string>();
        std::size_t used = 0;
        try {
            value = std::stod(s, &used);
        } catch (const std::exception&) {
            used = 0;
        }
        if (used == 0 || used != s.size()) throw ArgError(quoted(key) + " must be a number");
    } else {
        throw ArgError(quoted(key) + " must be a number");
    }
    if (!std::isfinite(value)) throw ArgError(quoted(key) + " must be finite");
    if (std::abs(value) > kMaxMagnitude) {
        throw ArgError(quoted(key) + " is " + json(value).dump() + ", beyond " +
                       json(kMaxMagnitude).dump() +
                       ": lengths are in mm (a part is not a kilometre "
                       "across)");
    }
    return value;
}

double number(const json& in, const char* key, double otherwise) {
    return has(in, key) ? number(in, key) : otherwise;
}

double positive(const json& in, const char* key) {
    const double value = number(in, key);
    if (!(value > 0.0)) throw ArgError(quoted(key) + " must be greater than 0");
    return value;
}

double positive(const json& in, const char* key, double otherwise) {
    return has(in, key) ? positive(in, key) : otherwise;
}

long long integer(const json& in, const char* key, long long lo, long long hi) {
    const double value = number(in, key);
    if (value != std::floor(value)) throw ArgError(quoted(key) + " must be a whole number");
    if (value < static_cast<double>(lo) || value > static_cast<double>(hi)) {
        throw ArgError(quoted(key) + " must be from " + std::to_string(lo) + " to " +
                       std::to_string(hi));
    }
    return static_cast<long long>(value);
}

long long integer(const json& in, const char* key, long long lo, long long hi,
                  long long otherwise) {
    return has(in, key) ? integer(in, key, lo, hi) : otherwise;
}

std::uint64_t id(const json& in, const char* key) {
    if (!has(in, key)) throw ArgError(quoted(key) + " is required");
    const json& v = in[key];
    if (v.is_number_unsigned()) return v.get<std::uint64_t>();
    if (v.is_number_integer() && v.get<long long>() >= 0) {
        return static_cast<std::uint64_t>(v.get<long long>());
    }
    if (v.is_string()) {
        const std::string s = v.get<std::string>();
        if (!s.empty() &&
            std::all_of(s.begin(), s.end(), [](char c) { return c >= '0' && c <= '9'; }) &&
            s.size() < 20) {
            return std::stoull(s);
        }
    }
    throw ArgError(quoted(key) + " must be an id (a whole number)");
}

bool boolean(const json& in, const char* key, bool otherwise) {
    if (!has(in, key)) return otherwise;
    const json& v = in[key];
    if (v.is_boolean()) return v.get<bool>();
    if (v.is_string()) {
        const std::string s = v.get<std::string>();
        if (s == "true") return true;
        if (s == "false") return false;
    }
    throw ArgError(quoted(key) + " must be true or false");
}

std::string text(const json& in, const char* key) {
    if (!has(in, key)) throw ArgError(quoted(key) + " is required");
    if (!in[key].is_string()) throw ArgError(quoted(key) + " must be a string");
    return in[key].get<std::string>();
}

std::string text(const json& in, const char* key, const std::string& otherwise) {
    return has(in, key) ? text(in, key) : otherwise;
}

std::string choice(const json& in, const char* key, const std::vector<std::string>& choices,
                   std::optional<std::string> otherwise) {
    if (!has(in, key)) {
        if (otherwise) return *otherwise;
        throw ArgError(quoted(key) + " is required");
    }
    std::string value = text(in, key);
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (std::find(choices.begin(), choices.end(), value) != choices.end()) return value;
    std::string list;
    for (const auto& c : choices) list += (list.empty() ? "" : ", ") + c;
    throw ArgError(quoted(key) + " must be one of: " + list);
}

namespace {

std::vector<double> numbersOf(const json& v, const char* key, std::size_t count) {
    std::vector<double> out;
    if (v.is_array() && v.size() == count) {
        for (const auto& item : v) {
            if (!item.is_number()) break;
            out.push_back(item.get<double>());
        }
    } else if (v.is_object()) {
        static const char* names[] = {"x", "y", "z"};
        for (std::size_t i = 0; i < count; ++i) {
            if (!v.contains(names[i]) || !v[names[i]].is_number()) break;
            out.push_back(v[names[i]].get<double>());
        }
    }
    if (out.size() != count) {
        throw ArgError(quoted(key) +
                       (count == 2 ? " must be a point [x, y]" : " must be [x, y, z]"));
    }
    for (const double d : out) {
        if (!std::isfinite(d)) throw ArgError(quoted(key) + " must be finite");
    }
    return out;
}

}  // namespace

math::Vec2 vec2(const json& in, const char* key) {
    if (!has(in, key)) throw ArgError(quoted(key) + " is required");
    const auto v = numbersOf(in[key], key, 2);
    return {v[0], v[1]};
}

math::Vec3 vec3(const json& in, const char* key) {
    if (!has(in, key)) throw ArgError(quoted(key) + " is required");
    const auto v = numbersOf(in[key], key, 3);
    return {v[0], v[1], v[2]};
}

math::Vec3 vec3(const json& in, const char* key, const math::Vec3& otherwise) {
    return has(in, key) ? vec3(in, key) : otherwise;
}

math::Vec3 direction(const json& in, const char* key) {
    const math::Vec3 v = vec3(in, key);
    const double length = v.length();
    if (!(length > 1e-12)) throw ArgError(quoted(key) + " must not be zero");
    return v / length;
}

math::Vec3 direction(const json& in, const char* key, const math::Vec3& otherwise) {
    return has(in, key) ? direction(in, key) : otherwise;
}

std::vector<std::string> strings(const json& in, const char* key) {
    if (!has(in, key)) throw ArgError(quoted(key) + " is required");
    const json& v = in[key];
    std::vector<std::string> out;
    if (v.is_string()) {
        out.push_back(v.get<std::string>());
    } else if (v.is_array()) {
        for (const auto& item : v) {
            if (!item.is_string()) throw ArgError(quoted(key) + " must be a list of strings");
            out.push_back(item.get<std::string>());
        }
    } else {
        throw ArgError(quoted(key) + " must be a list of strings");
    }
    if (out.empty()) throw ArgError(quoted(key) + " is empty");
    return out;
}

// --- Values -------------------------------------------------------------------

double round6(double value) {
    if (!std::isfinite(value)) return value;
    const double r = std::round(value * 1e6) / 1e6;
    return r == 0.0 ? 0.0 : r;  // no "-0"
}

json toJson(const math::Vec2& v) {
    return json::array({round6(v.x), round6(v.y)});
}

json toJson(const math::Vec3& v) {
    return json::array({round6(v.x), round6(v.y), round6(v.z)});
}

// --- Schemas ------------------------------------------------------------------

json object(json properties, const std::vector<std::string>& required) {
    json schema = {{"type", "object"}, {"properties", std::move(properties)}};
    if (!required.empty()) schema["required"] = required;
    return schema;
}

json numberSchema(const std::string& description) {
    return {{"type", "number"}, {"description", description}};
}

json integerSchema(const std::string& description) {
    return {{"type", "integer"}, {"description", description}};
}

json stringSchema(const std::string& description) {
    return {{"type", "string"}, {"description", description}};
}

json booleanSchema(const std::string& description) {
    return {{"type", "boolean"}, {"description", description}};
}

json enumSchema(const std::vector<std::string>& values, const std::string& description) {
    return {{"type", "string"}, {"enum", values}, {"description", description}};
}

json pointSchema(const std::string& description) {
    return {{"type", "array"}, {"items", {{"type", "number"}}}, {"description", description}};
}

json arraySchema(json items, const std::string& description) {
    return {{"type", "array"}, {"items", std::move(items)}, {"description", description}};
}

// --- The document -------------------------------------------------------------

void CadContext::modelChanged() const {
    if (hooks.modelChanged) hooks.modelChanged();
}

void CadContext::drawingChanged() const {
    if (hooks.drawingChanged) hooks.drawingChanged();
}

std::string faceNameOf(const CadContext& context, const std::string& given) {
    std::string word = given;
    std::transform(word.begin(), word.end(), word.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    static const std::map<std::string, math::Vec3> sides = {
        {"top", {0, 0, 1}},  {"bottom", {0, 0, -1}}, {"front", {0, -1, 0}},
        {"back", {0, 1, 0}}, {"right", {1, 0, 0}},   {"left", {-1, 0, 0}}};
    const auto side = sides.find(word);
    const topo::Solid* solid = context.document.solid();
    if (side == sides.end() || solid == nullptr) return model::wholeFaceName(given);
    const double outward = model::outwardSign(*solid);
    std::string best;
    double bestReach = -std::numeric_limits<double>::max();
    for (const auto& face : solid->faces()) {
        if (!face.alive || !face.topoId.isValid()) continue;
        const auto plane = model::planeOf(face, outward);
        if (!plane || plane->normal.dot(side->second) < std::cos(2.0 * kDegree)) continue;
        const double reach = plane->origin.dot(side->second);
        if (reach > bestReach) {
            bestReach = reach;
            best = model::wholeFaceName(face.topoId.tag());
        }
    }
    if (best.empty()) {
        throw ArgError("the part has no flat face facing " + word + "; use a name from list_faces");
    }
    return best;
}

json solidSummary(const doc::Document& document) {
    const topo::Solid* solid = document.solid();
    if (solid == nullptr) return nullptr;
    math::Vec3 lo(std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),
                  std::numeric_limits<double>::max());
    math::Vec3 hi = lo * -1.0;
    bool any = false;
    for (const auto& vertex : solid->vertices()) {
        if (!vertex.alive) continue;
        any = true;
        lo = {std::min(lo.x, vertex.point.x), std::min(lo.y, vertex.point.y),
              std::min(lo.z, vertex.point.z)};
        hi = {std::max(hi.x, vertex.point.x), std::max(hi.y, vertex.point.y),
              std::max(hi.z, vertex.point.z)};
    }
    json out = {{"shells", solid->shellCount()}, {"faces", solid->faceCount()}};
    if (any) {
        out["bounding_box"] = {{"min", toJson(lo)}, {"max", toJson(hi)}, {"size", toJson(hi - lo)}};
    }
    // As the part is measured: on its exact counterpart, where it has one.
    const auto mass = model::MassPropertiesCalculator::compute(*document.exactOrFacetedSolid());
    if (mass.valid) {
        out["volume_mm3"] = round6(mass.volume);
    } else {
        out["volume_problem"] = mass.problem.empty()
                                    ? std::string("it encloses nothing that can be measured")
                                    : mass.problem;
    }
    return out;
}

doc::Feature& featureAt(doc::Document& document, const json& in, const char* key) {
    const auto count = static_cast<long long>(document.featureTree().featureCount());
    if (count == 0) throw ArgError("the part has no features");
    const long long index = integer(in, key, 0, count - 1);
    return *document.featureTree().feature(static_cast<std::size_t>(index));
}

std::shared_ptr<doc::Sketch> sketchOf(doc::Document& document, const json& in, const char* key) {
    const std::uint64_t wanted = id(in, key);
    auto sketch = document.findSketch(wanted);
    if (!sketch) {
        std::string ids;
        for (const auto& s : document.sketches()) {
            ids += (ids.empty() ? "" : ", ") + std::to_string(s->id());
        }
        throw ArgError("there is no sketch " + std::to_string(wanted) +
                       (ids.empty() ? " (the part has no sketches)" : " (sketches: " + ids + ")"));
    }
    return sketch;
}

std::vector<std::string> featureTargets(doc::Document& document, const json& in, const char* key) {
    std::vector<std::string> out;
    if (!has(in, key)) return out;
    const json& list = in[key];
    if (!list.is_array()) throw ArgError(quoted(key) + " must be a list of feature indices");
    const auto count = static_cast<long long>(document.featureTree().featureCount());
    for (const auto& item : list) {
        if (!item.is_number_integer()) throw ArgError(quoted(key) + " must be feature indices");
        const auto index = item.get<long long>();
        if (index < 0 || index >= count) {
            throw ArgError("there is no feature " + std::to_string(index));
        }
        out.push_back(document.featureTree().feature(static_cast<std::size_t>(index))->featureID());
    }
    return out;
}

json rebuildAndReport(const CadContext& context) {
    doc::Document& document = context.document;
    document.rebuildModel();
    json out = {{"solid", solidSummary(document)}};
    if (document.failedFeatureIndex() >= 0) {
        out["build_error"] = {{"feature_index", document.failedFeatureIndex()},
                              {"message", document.lastBuildMessage()}};
    }
    return out;
}

namespace {

/// The document's type, changed as a step of the undo stack.
class SetDocumentTypeCommand : public doc::Command {
public:
    SetDocumentTypeCommand(doc::Document& document, doc::DocumentType type)
        : m_document(document), m_after(type), m_before(document.type()) {}
    void execute() override { m_document.setType(m_after); }
    void undo() override { m_document.setType(m_before); }
    std::string description() const override { return "Make a Part"; }

private:
    doc::Document& m_document;
    doc::DocumentType m_after;
    doc::DocumentType m_before;
};

}  // namespace

std::unique_ptr<doc::Command> asPartStep(doc::Document& document,
                                         std::unique_ptr<doc::Command> command) {
    if (document.type() != doc::DocumentType::Drawing ||
        !document.draftDocument().entities().empty()) {
        return command;
    }
    auto step = std::make_unique<doc::CompositeCommand>(command->description());
    step->addCommand(std::make_unique<SetDocumentTypeCommand>(document, doc::DocumentType::Part));
    step->addCommand(std::move(command));
    return step;
}

ToolResult addFeature(const CadContext& context, std::unique_ptr<doc::Feature> feature,
                      const std::string& verb, json extra) {
    doc::Document& document = context.document;
    auto command = std::make_unique<doc::AddFeatureCommand>(document, std::move(feature));
    const doc::Feature* added = command->feature();
    // A new document's first feature makes it a part, in the same step: a
    // withdrawn feature or an undo puts the type back too.
    auto step = asPartStep(document, std::move(command));
    const doc::Command* pushed = step.get();
    document.undoStack().push(std::move(step));
    document.rebuildModel();

    const auto index = document.featureTree().indexOf(added);
    if (index && document.failedFeatureIndex() == static_cast<int>(*index)) {
        // It fails itself: withdrawn, the part and its history as they were.
        const std::string reason = document.lastBuildMessage();
        document.undoStack().withdraw(pushed);
        document.rebuildModel();
        context.modelChanged();
        return ToolResult::error(
            verb + " failed and was not added: " + (reason.empty() ? "the build failed" : reason));
    }
    context.modelChanged();
    json out = {{"added", verb}, {"solid", solidSummary(document)}};
    if (index) out["feature_index"] = *index;
    if (document.failedFeatureIndex() >= 0) {
        out["build_error"] = {{"feature_index", document.failedFeatureIndex()},
                              {"message", document.lastBuildMessage()}};
    }
    for (auto it = extra.begin(); it != extra.end(); ++it) out[it.key()] = it.value();
    return ToolResult::ok(out, true);
}

}  // namespace detail

}  // namespace hz::ai
