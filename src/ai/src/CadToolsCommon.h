#pragma once

// What the CAD tools share: reading a model's arguments (each complaint names
// the argument), writing schemas, and adding a feature the way the window
// does. Internal to hz_ai.

#include <cstdint>
#include <memory>
#include <nlohmann/json.hpp>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "horizon/ai/CadTools.h"
#include "horizon/ai/ToolRegistry.h"
#include "horizon/math/Vec2.h"
#include "horizon/math/Vec3.h"

namespace hz::doc {
class Command;
class Document;
class Feature;
class Sketch;
}  // namespace hz::doc

namespace hz::topo {
class Solid;
}

namespace hz::draft {
class DraftEntity;
}

namespace hz::ai::detail {

using nlohmann::json;

/// A model's argument is wrong. The registry turns it into an error result.
struct ArgError : std::invalid_argument {
    using std::invalid_argument::invalid_argument;
};

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegree = kPi / 180.0;
/// No length, coordinate or angle a model gives is beyond this: 1 km in mm.
/// Past it is a garbled number, not a part (and past the kernel's tested
/// range).
constexpr double kMaxMagnitude = 1e6;

// --- Arguments ----------------------------------------------------------------

bool has(const json& in, const char* key);
double number(const json& in, const char* key);
double number(const json& in, const char* key, double otherwise);
double positive(const json& in, const char* key);
double positive(const json& in, const char* key, double otherwise);
long long integer(const json& in, const char* key, long long lo, long long hi);
long long integer(const json& in, const char* key, long long lo, long long hi, long long otherwise);
std::uint64_t id(const json& in, const char* key);
bool boolean(const json& in, const char* key, bool otherwise);
std::string text(const json& in, const char* key);
std::string text(const json& in, const char* key, const std::string& otherwise);
/// One of @p choices; the first when absent and @p optional.
std::string choice(const json& in, const char* key, const std::vector<std::string>& choices,
                   std::optional<std::string> otherwise = std::nullopt);
math::Vec2 vec2(const json& in, const char* key);
math::Vec3 vec3(const json& in, const char* key);
math::Vec3 vec3(const json& in, const char* key, const math::Vec3& otherwise);
/// A direction: a vec3 of some length, made unit.
math::Vec3 direction(const json& in, const char* key);
math::Vec3 direction(const json& in, const char* key, const math::Vec3& otherwise);
std::vector<std::string> strings(const json& in, const char* key);

// --- Values for the model -----------------------------------------------------

/// @p value rounded to 6 decimals (a model reads 12.000000000001 as noise).
double round6(double value);
json toJson(const math::Vec2& v);
json toJson(const math::Vec3& v);

// --- Schemas ------------------------------------------------------------------

json object(json properties, const std::vector<std::string>& required = {});
json numberSchema(const std::string& description);
json integerSchema(const std::string& description);
json stringSchema(const std::string& description);
json booleanSchema(const std::string& description);
json enumSchema(const std::vector<std::string>& values, const std::string& description);
json pointSchema(const std::string& description);  ///< [x, y, z]
json arraySchema(json items, const std::string& description);

// --- The document -------------------------------------------------------------

struct CadContext {
    doc::Document& document;
    CadToolHooks hooks;

    void modelChanged() const;
    void drawingChanged() const;
};
using Context = std::shared_ptr<CadContext>;

/// The whole name of the face @p given names: a name or id from list_faces,
/// or one of top, bottom, front, back, left, right (the outermost flat face
/// facing +Z, -Z, -Y, +Y, -X, +X), which a model reaches for first.
std::string faceNameOf(const CadContext& context, const std::string& given);

/// The solid as a model needs to see it: faces, bodies, volume, extent.
json solidSummary(const doc::Document& document);
/// The feature at @p index (as list_features numbers them), or an ArgError.
doc::Feature& featureAt(doc::Document& document, const json& in, const char* key);
/// The sketch with id @p key, or an ArgError.
std::shared_ptr<doc::Sketch> sketchOf(doc::Document& document, const json& in, const char* key);
/// The features named by index in @p key, as feature ids (for a pattern's or
/// mirror's targets). Empty when absent.
std::vector<std::string> featureTargets(doc::Document& document, const json& in, const char* key);

/// @p command, made to turn an empty drawing into a part as it runs, and
/// back when undone: the first sketch or feature of a new document makes it a
/// part, as one undo step with what made it so.
std::unique_ptr<doc::Command> asPartStep(doc::Document& document,
                                         std::unique_ptr<doc::Command> command);

/// Add @p feature as the window does: one undo step, built at once; one that
/// fails itself is withdrawn, the history as it was, and the reason is the
/// error. @p extra goes into the result.
ToolResult addFeature(const CadContext& context, std::unique_ptr<doc::Feature> feature,
                      const std::string& verb, json extra = json::object());

/// Rebuild after an edit, and say how it went (with the solid).
json rebuildAndReport(const CadContext& context);

/// One drawing entity as a model reads it: id, layer, type, geometry.
json describeEntity(const draft::DraftEntity& entity);

/// A drawing scale as a model gives it ("1:2", "2:1", "1", or a number):
/// sheet millimetres per model millimetre; 0 when @p key is absent. An
/// ArgError for anything else.
double drawingScale(const json& in, const char* key);
/// A title block field, @p key: text no longer than a block holds ("" when
/// absent).
std::string titleBlockField(const json& in, const char* key);

/// What make_drawing draws: the part or the assembly, as its tab has it.
struct DrawingSource {
    std::string what;      ///< "part" or "assembly"
    std::string path;      ///< its file; empty when it was never saved
    bool unsaved = false;  ///< changed since it was saved: said, so it is saved too
    bool empty = false;    ///< nothing to draw: no solid, no component
};

/// Add make_drawing, which asks @p make for a sheet of what @p source says
/// the tab shows. Nothing is added when either is unset.
void registerDrawingTool(ToolRegistry& registry, MakeDrawing make,
                         std::function<DrawingSource()> source);

void registerQueryTools(ToolRegistry& registry, const Context& context);
void registerModelTools(ToolRegistry& registry, const Context& context);
void registerDraftTools(ToolRegistry& registry, const Context& context);
void registerAdvancedTools(ToolRegistry& registry, const Context& context);

}  // namespace hz::ai::detail
