#include "horizon/fileio/NativeFormat.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <set>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>

#include "horizon/constraint/Constraint.h"
#include "horizon/constraint/ConstraintSystem.h"
#include "horizon/constraint/GeometryRef.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/document/Sketch.h"
#include "horizon/drafting/BlockTable.h"
#include "horizon/drafting/DraftAngularDimension.h"
#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftBlockRef.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftEllipse.h"
#include "horizon/drafting/DraftHatch.h"
#include "horizon/drafting/DraftLeader.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftLinearDimension.h"
#include "horizon/drafting/DraftPolyline.h"
#include "horizon/drafting/DraftRadialDimension.h"
#include "horizon/drafting/DraftRectangle.h"
#include "horizon/drafting/DraftSpline.h"
#include "horizon/drafting/DraftText.h"
#include "horizon/drafting/LineType.h"
#include "horizon/drafting/SketchPlane.h"
#include "horizon/fileio/AtomicFile.h"
#include "horizon/fileio/StepFormat.h"
#include "horizon/math/Units.h"
#include "horizon/modeling/SolidTessellator.h"

using json = nlohmann::json;

namespace hz::io {

/// Serialize without throwing. Text imported from a DXF carries whatever code
/// page its source used, and json::dump() throws on bytes that are not valid
/// UTF-8 — which used to abort a save half-way through a truncated file.
/// Invalid sequences become U+FFFD instead.
static std::string dumpJson(const json& root, int indent) {
    return root.dump(indent, ' ', false, json::error_handler_t::replace);
}

/// The envelope version this build writes, and the newest it can read. A file
/// from a newer build may hold content this one does not know; loading it
/// would drop that content silently, and saving would then destroy it.
///
/// 17: features carry "bodyOperation" (Phase 102) and "featureSuppressed"
/// (Phase 104). An older build would ignore both and build a different part
/// — every body separate, suppressed features back in — without a word.
/// 18: a feature may carry "naming": 2 (Phase 106). An older build would name
/// its faces and edges, and its Booleans' results, by position, and every
/// reference made against the new names — a fillet's edges, a mate's faces —
/// would miss.
/// 19: a feature may carry "naming": 3 (Phase 139), whose names are scoped to
/// their feature and kept across fillets. An older build, which knows no 3,
/// refuses the file rather than build it under other names.
/// 20: a sketch may carry "face", the face it follows, and "placed", where a
/// build last put it on that face (Phase 157). An older build would leave
/// the sketch on the plane it was drawn on, wherever the face has gone, and
/// build a different part.
static constexpr int kFormatVersion = 20;

/// A sketch's plane: its origin, normal and x axis.
static json planeToJson(const draft::SketchPlane& plane) {
    return {{"origin", {plane.origin().x, plane.origin().y, plane.origin().z}},
            {"normal", {plane.normal().x, plane.normal().y, plane.normal().z}},
            {"xAxis", {plane.xAxis().x, plane.xAxis().y, plane.xAxis().z}}};
}

/// A plane planeToJson() wrote. Throws, as json does, on a malformed one.
static draft::SketchPlane planeFromJson(const json& pObj) {
    const auto vec = [&pObj](const char* key) {
        const auto& v = pObj.at(key);
        return math::Vec3(v.at(0).get<double>(), v.at(1).get<double>(), v.at(2).get<double>());
    };
    return draft::SketchPlane(vec("origin"), vec("normal"), vec("xAxis"));
}

/// Store `message` in `error` (when given) and report failure.
static bool fail(std::string* error, std::string message) {
    if (error) *error = std::move(message);
    return false;
}

/// Open `filePath` for reading, or say why it cannot be.
static bool openForReading(const std::string& filePath, std::ifstream& file, std::string* error) {
    const std::filesystem::path p = pathFromUtf8(filePath);
    if (std::string why = whyUnreadable(p); !why.empty()) return fail(error, std::move(why));
    file.open(p);
    if (!file.is_open()) return fail(error, "the file could not be read (check its permissions)");
    return true;
}

/// nlohmann's messages start with an internal tag
/// ("[json.exception.parse_error.101] "); the rest — "parse error at line 3,
/// column 5: ..." — is what a user can act on.
static std::string jsonMessage(const std::exception& e) {
    std::string what = e.what();
    if (what.rfind("[json.exception.", 0) == 0) {
        const auto end = what.find("] ");
        if (end != std::string::npos) what.erase(0, end + 2);
    }
    return what;
}

/// An integer field in [lo, hi], or `fallback` when absent. Throws a
/// std:: exception, not a json one — every per-item loop catches
/// std::exception, so a bad field skips its item rather than the document.
///
/// nlohmann converts
/// a float to an integer with a plain static_cast — undefined behaviour out of
/// range — and an enum, count or index from a file can hold anything. A value
/// that is not a whole number in range throws, and the load reports the file
/// as damaged. An enum that indexes a table (a line type's dash pattern) is
/// otherwise an out-of-bounds read waiting for its first draw.
static long long intField(const json& obj, const char* key, long long fallback, long long lo,
                          long long hi) {
    if (!obj.contains(key)) return fallback;
    const json& j = obj.at(key);
    long long v = 0;
    if (j.is_number_integer() && !j.is_number_unsigned()) {
        v = j.get<long long>();
    } else if (j.is_number_unsigned()) {
        const auto u = j.get<unsigned long long>();
        if (u > static_cast<unsigned long long>(hi)) {
            throw std::out_of_range(std::string("\"") + key + "\" is out of range");
        }
        v = static_cast<long long>(u);
    } else if (j.is_number_float()) {
        const double d = j.get<double>();
        if (!std::isfinite(d) || d != std::floor(d) || d < static_cast<double>(lo) ||
            d > static_cast<double>(hi)) {
            throw std::out_of_range(std::string("\"") + key + "\" is out of range");
        }
        v = static_cast<long long>(d);
    } else {
        throw std::invalid_argument(std::string("\"") + key + "\" is not a number");
    }
    if (v < lo || v > hi) throw std::out_of_range(std::string("\"") + key + "\" is out of range");
    return v;
}

/// A count that is clamped rather than refused: 0 and huge values are the
/// same mistake as a slightly wrong one, and clamping keeps the rest of the
/// file.
static int clampedCount(const json& obj, const char* key, int fallback, int lo, int hi) {
    if (!obj.contains(key)) return fallback;
    const json& j = obj.at(key);
    if (!j.is_number()) throw std::invalid_argument(std::string("\"") + key + "\" is not a number");
    const double d = j.get<double>();
    if (!std::isfinite(d)) throw std::out_of_range(std::string("\"") + key + "\" is not finite");
    return static_cast<int>(std::clamp(d, static_cast<double>(lo), static_cast<double>(hi)));
}

constexpr long long kLastLineType = static_cast<long long>(draft::LineType::Phantom);
constexpr long long kLastHatchPattern = static_cast<long long>(draft::HatchPattern::CrossHatch);
constexpr long long kLastAlignment = static_cast<long long>(draft::TextAlignment::Right);
constexpr long long kLastOrientation =
    static_cast<long long>(draft::DraftLinearDimension::Orientation::Aligned);

// ---------------------------------------------------------------------------
// Constraint serialization helpers
// ---------------------------------------------------------------------------

static std::string featureTypeToString(cstr::FeatureType ft) {
    switch (ft) {
        case cstr::FeatureType::Point:
            return "point";
        case cstr::FeatureType::Line:
            return "line";
        case cstr::FeatureType::Circle:
            return "circle";
    }
    return "point";
}

static cstr::FeatureType featureTypeFromString(const std::string& s) {
    if (s == "line") return cstr::FeatureType::Line;
    if (s == "circle") return cstr::FeatureType::Circle;
    return cstr::FeatureType::Point;
}

static json serializeRef(const cstr::GeometryRef& ref) {
    return {{"entityId", ref.entityId},
            {"featureType", featureTypeToString(ref.featureType)},
            {"featureIndex", ref.featureIndex}};
}

static cstr::GeometryRef deserializeRef(const json& obj) {
    cstr::GeometryRef ref;
    ref.entityId = obj.value("entityId", uint64_t(0));
    ref.featureType = featureTypeFromString(obj.value("featureType", "point"));
    ref.featureIndex = obj.value("featureIndex", 0);
    return ref;
}

static std::string constraintTypeToString(cstr::ConstraintType ct) {
    switch (ct) {
        case cstr::ConstraintType::Coincident:
            return "coincident";
        case cstr::ConstraintType::Horizontal:
            return "horizontal";
        case cstr::ConstraintType::Vertical:
            return "vertical";
        case cstr::ConstraintType::Perpendicular:
            return "perpendicular";
        case cstr::ConstraintType::Parallel:
            return "parallel";
        case cstr::ConstraintType::Tangent:
            return "tangent";
        case cstr::ConstraintType::Equal:
            return "equal";
        case cstr::ConstraintType::Fixed:
            return "fixed";
        case cstr::ConstraintType::Distance:
            return "distance";
        case cstr::ConstraintType::Angle:
            return "angle";
    }
    return "unknown";
}

// ---------------------------------------------------------------------------
// Entity serialization helper (shared by top-level and per-sketch writes)
// ---------------------------------------------------------------------------

static json serializeEntity(const draft::DraftEntity& entity) {
    json obj;
    obj["id"] = entity.id();
    obj["layer"] = entity.layer();
    obj["color"] = entity.color();
    obj["lineWidth"] = entity.lineWidth();
    obj["lineType"] = entity.lineType();
    if (entity.groupId() != 0) {
        obj["groupId"] = entity.groupId();
    }

    if (auto* line = dynamic_cast<const draft::DraftLine*>(&entity)) {
        obj["type"] = "line";
        obj["start"] = {{"x", line->start().x}, {"y", line->start().y}};
        obj["end"] = {{"x", line->end().x}, {"y", line->end().y}};
    } else if (auto* circle = dynamic_cast<const draft::DraftCircle*>(&entity)) {
        obj["type"] = "circle";
        obj["center"] = {{"x", circle->center().x}, {"y", circle->center().y}};
        obj["radius"] = circle->radius();
    } else if (auto* arc = dynamic_cast<const draft::DraftArc*>(&entity)) {
        obj["type"] = "arc";
        obj["center"] = {{"x", arc->center().x}, {"y", arc->center().y}};
        obj["radius"] = arc->radius();
        obj["startAngle"] = arc->startAngle();
        obj["endAngle"] = arc->endAngle();
    } else if (auto* rect = dynamic_cast<const draft::DraftRectangle*>(&entity)) {
        obj["type"] = "rectangle";
        obj["corner1"] = {{"x", rect->corner1().x}, {"y", rect->corner1().y}};
        obj["corner2"] = {{"x", rect->corner2().x}, {"y", rect->corner2().y}};
    } else if (auto* polyline = dynamic_cast<const draft::DraftPolyline*>(&entity)) {
        obj["type"] = "polyline";
        obj["closed"] = polyline->closed();
        json pointsArray = json::array();
        for (const auto& pt : polyline->points()) {
            pointsArray.push_back({{"x", pt.x}, {"y", pt.y}});
        }
        obj["points"] = pointsArray;
    } else if (auto* ld = dynamic_cast<const draft::DraftLinearDimension*>(&entity)) {
        obj["type"] = "linearDimension";
        obj["defPoint1"] = {{"x", ld->defPoint1().x}, {"y", ld->defPoint1().y}};
        obj["defPoint2"] = {{"x", ld->defPoint2().x}, {"y", ld->defPoint2().y}};
        obj["dimLinePoint"] = {{"x", ld->dimLinePoint().x}, {"y", ld->dimLinePoint().y}};
        obj["orientation"] = static_cast<int>(ld->orientation());
        if (ld->hasTextOverride()) obj["textOverride"] = ld->textOverride();
    } else if (auto* rd = dynamic_cast<const draft::DraftRadialDimension*>(&entity)) {
        obj["type"] = "radialDimension";
        obj["center"] = {{"x", rd->center().x}, {"y", rd->center().y}};
        obj["radius"] = rd->radius();
        obj["textPoint"] = {{"x", rd->textPoint().x}, {"y", rd->textPoint().y}};
        obj["isDiameter"] = rd->isDiameter();
        if (rd->hasTextOverride()) obj["textOverride"] = rd->textOverride();
    } else if (auto* ad = dynamic_cast<const draft::DraftAngularDimension*>(&entity)) {
        obj["type"] = "angularDimension";
        obj["vertex"] = {{"x", ad->vertex().x}, {"y", ad->vertex().y}};
        obj["line1Point"] = {{"x", ad->line1Point().x}, {"y", ad->line1Point().y}};
        obj["line2Point"] = {{"x", ad->line2Point().x}, {"y", ad->line2Point().y}};
        obj["arcRadius"] = ad->arcRadius();
        if (ad->hasTextOverride()) obj["textOverride"] = ad->textOverride();
    } else if (auto* leader = dynamic_cast<const draft::DraftLeader*>(&entity)) {
        obj["type"] = "leader";
        obj["text"] = leader->text();
        json ptsArray = json::array();
        for (const auto& pt : leader->points()) {
            ptsArray.push_back({{"x", pt.x}, {"y", pt.y}});
        }
        obj["points"] = ptsArray;
        if (leader->hasTextOverride()) obj["textOverride"] = leader->textOverride();
    } else if (auto* bref = dynamic_cast<const draft::DraftBlockRef*>(&entity)) {
        obj["type"] = "blockRef";
        obj["blockName"] = bref->blockName();
        obj["insertPos"] = {{"x", bref->insertPos().x}, {"y", bref->insertPos().y}};
        obj["rotation"] = bref->rotation();
        obj["scale"] = bref->uniformScale();
        if (bref->mirrored()) obj["mirrored"] = true;
    } else if (auto* txt = dynamic_cast<const draft::DraftText*>(&entity)) {
        obj["type"] = "text";
        obj["position"] = {{"x", txt->position().x}, {"y", txt->position().y}};
        obj["text"] = txt->text();
        obj["textHeight"] = txt->textHeight();
        obj["rotation"] = txt->rotation();
        obj["alignment"] = static_cast<int>(txt->alignment());
    } else if (auto* spline = dynamic_cast<const draft::DraftSpline*>(&entity)) {
        obj["type"] = "spline";
        obj["closed"] = spline->closed();
        json cpArray = json::array();
        for (const auto& cp : spline->controlPoints()) {
            cpArray.push_back({{"x", cp.x}, {"y", cp.y}});
        }
        obj["controlPoints"] = cpArray;
        // Only write weights when at least one differs from 1.0 (backward compat).
        if (spline->hasNonUniformWeights()) {
            json wArray = json::array();
            for (double w : spline->weights()) wArray.push_back(w);
            obj["weights"] = wArray;
        }
    } else if (auto* hatch = dynamic_cast<const draft::DraftHatch*>(&entity)) {
        obj["type"] = "hatch";
        obj["pattern"] = static_cast<int>(hatch->pattern());
        obj["angle"] = hatch->angle();
        obj["spacing"] = hatch->spacing();
        json bndArray = json::array();
        for (const auto& pt : hatch->boundary()) {
            bndArray.push_back({{"x", pt.x}, {"y", pt.y}});
        }
        obj["boundary"] = bndArray;
    } else if (auto* ellipse = dynamic_cast<const draft::DraftEllipse*>(&entity)) {
        obj["type"] = "ellipse";
        obj["center"] = {{"x", ellipse->center().x}, {"y", ellipse->center().y}};
        obj["semiMajor"] = ellipse->semiMajor();
        obj["semiMinor"] = ellipse->semiMinor();
        obj["rotation"] = ellipse->rotation();
    }

    return obj;
}

/// Build the complete document JSON envelope. Shared by save() and by
/// BinaryFormat, which stores the same envelope inside a FlatBuffers container
/// (without the tessellation cache — that lives in typed binary vectors).
/// The constraints of @p system, as the file holds them.
static json constraintsToJson(const cstr::ConstraintSystem& system) {
    json constraintsArray = json::array();
    for (const auto& c : system.constraints()) {
        json cObj;
        cObj["id"] = c->id();
        cObj["type"] = constraintTypeToString(c->type());

        switch (c->type()) {
            case cstr::ConstraintType::Coincident: {
                auto* cc = dynamic_cast<const cstr::CoincidentConstraint*>(c.get());
                cObj["refA"] = serializeRef(cc->pointA());
                cObj["refB"] = serializeRef(cc->pointB());
                break;
            }
            case cstr::ConstraintType::Horizontal: {
                auto* hc = dynamic_cast<const cstr::HorizontalConstraint*>(c.get());
                cObj["refA"] = serializeRef(hc->refA());
                cObj["refB"] = serializeRef(hc->refB());
                break;
            }
            case cstr::ConstraintType::Vertical: {
                auto* vc = dynamic_cast<const cstr::VerticalConstraint*>(c.get());
                cObj["refA"] = serializeRef(vc->refA());
                cObj["refB"] = serializeRef(vc->refB());
                break;
            }
            case cstr::ConstraintType::Perpendicular: {
                auto* pc = dynamic_cast<const cstr::PerpendicularConstraint*>(c.get());
                cObj["refA"] = serializeRef(pc->lineA());
                cObj["refB"] = serializeRef(pc->lineB());
                break;
            }
            case cstr::ConstraintType::Parallel: {
                auto* pc = dynamic_cast<const cstr::ParallelConstraint*>(c.get());
                cObj["refA"] = serializeRef(pc->lineA());
                cObj["refB"] = serializeRef(pc->lineB());
                break;
            }
            case cstr::ConstraintType::Tangent: {
                auto* tc = dynamic_cast<const cstr::TangentConstraint*>(c.get());
                cObj["refA"] = serializeRef(tc->lineRef());
                cObj["refB"] = serializeRef(tc->circleRef());
                break;
            }
            case cstr::ConstraintType::Equal: {
                auto* ec = dynamic_cast<const cstr::EqualConstraint*>(c.get());
                cObj["refA"] = serializeRef(ec->refA());
                cObj["refB"] = serializeRef(ec->refB());
                break;
            }
            case cstr::ConstraintType::Fixed: {
                auto* fc = dynamic_cast<const cstr::FixedConstraint*>(c.get());
                cObj["ref"] = serializeRef(fc->pointRef());
                cObj["position"] = {{"x", fc->position().x}, {"y", fc->position().y}};
                break;
            }
            case cstr::ConstraintType::Distance: {
                auto* dc = dynamic_cast<const cstr::DistanceConstraint*>(c.get());
                cObj["refA"] = serializeRef(dc->refA());
                cObj["refB"] = serializeRef(dc->refB());
                cObj["value"] = dc->dimensionalValue();
                break;
            }
            case cstr::ConstraintType::Angle: {
                auto* ac = dynamic_cast<const cstr::AngleConstraint*>(c.get());
                cObj["refA"] = serializeRef(ac->lineA());
                cObj["refB"] = serializeRef(ac->lineB());
                cObj["value"] = ac->dimensionalValue();
                break;
            }
        }

        // Variable reference (v13+)
        if (c->hasVariableReference()) {
            cObj["variableName"] = c->variableReference();
        }

        constraintsArray.push_back(cObj);
    }
    return constraintsArray;
}

/// Read @p array's constraints into @p system, dropping any that name an
/// entity @p drawing does not have (a corrupted or hand-edited file).
static void constraintsFromJson(const json& array, const draft::DraftDocument& drawing,
                                cstr::ConstraintSystem& system) {
    for (const auto& cObj : array) {
        std::string ctype = cObj.value("type", "");
        std::shared_ptr<cstr::Constraint> constraint;

        if (ctype == "coincident") {
            constraint = std::make_shared<cstr::CoincidentConstraint>(
                deserializeRef(cObj.at("refA")), deserializeRef(cObj.at("refB")));
        } else if (ctype == "horizontal") {
            constraint = std::make_shared<cstr::HorizontalConstraint>(
                deserializeRef(cObj.at("refA")), deserializeRef(cObj.at("refB")));
        } else if (ctype == "vertical") {
            constraint = std::make_shared<cstr::VerticalConstraint>(
                deserializeRef(cObj.at("refA")), deserializeRef(cObj.at("refB")));
        } else if (ctype == "perpendicular") {
            constraint = std::make_shared<cstr::PerpendicularConstraint>(
                deserializeRef(cObj.at("refA")), deserializeRef(cObj.at("refB")));
        } else if (ctype == "parallel") {
            constraint = std::make_shared<cstr::ParallelConstraint>(
                deserializeRef(cObj.at("refA")), deserializeRef(cObj.at("refB")));
        } else if (ctype == "tangent") {
            constraint = std::make_shared<cstr::TangentConstraint>(deserializeRef(cObj.at("refA")),
                                                                   deserializeRef(cObj.at("refB")));
        } else if (ctype == "equal") {
            constraint = std::make_shared<cstr::EqualConstraint>(deserializeRef(cObj.at("refA")),
                                                                 deserializeRef(cObj.at("refB")));
        } else if (ctype == "fixed") {
            auto pos = math::Vec2(cObj.at("position").at("x").get<double>(),
                                  cObj.at("position").at("y").get<double>());
            constraint =
                std::make_shared<cstr::FixedConstraint>(deserializeRef(cObj.at("ref")), pos);
        } else if (ctype == "distance") {
            double val = cObj.value("value", 0.0);
            constraint = std::make_shared<cstr::DistanceConstraint>(
                deserializeRef(cObj.at("refA")), deserializeRef(cObj.at("refB")), val);
        } else if (ctype == "angle") {
            double val = cObj.value("value", 0.0);
            constraint = std::make_shared<cstr::AngleConstraint>(
                deserializeRef(cObj.at("refA")), deserializeRef(cObj.at("refB")), val);
        }

        if (constraint) {
            // Restore the original constraint ID from the file.
            if (cObj.contains("id")) {
                uint64_t savedId = cObj.at("id").get<uint64_t>();
                constraint->setId(savedId);
                cstr::Constraint::advanceIdCounter(savedId);
            }
            // Variable reference (v13+)
            if (cObj.contains("variableName")) {
                constraint->setVariableReference(cObj.at("variableName").get<std::string>());
            }
            system.addConstraint(constraint);
        }
    }

    // Validate constraint entity references — remove any that reference
    // non-existent entities (corrupted or manually-edited files).
    std::set<uint64_t> entityIds;
    for (const auto& e : drawing.entities()) {
        entityIds.insert(e->id());
    }
    std::vector<uint64_t> invalidConstraints;
    for (const auto& c : system.constraints()) {
        for (uint64_t eid : c->referencedEntityIds()) {
            if (entityIds.find(eid) == entityIds.end()) {
                invalidConstraints.push_back(c->id());
                break;
            }
        }
    }
    for (uint64_t cid : invalidConstraints) {
        system.removeConstraint(cid);
    }
}

/// The unit a document shows and takes lengths in (Phase 154). An older
/// build ignores it and reads the model as it is, in millimetres: no
/// version bump.
static json unitsJson(math::LengthUnit unit) {
    return {{"length", std::string(math::symbolOf(unit))}};
}

/// A file's unit; millimetres when it names none (every file before Phase
/// 154) or one that is not known.
static math::LengthUnit unitsFrom(const json& root) {
    const auto units = root.find("units");
    if (units == root.end() || !units->is_object()) return math::LengthUnit::Millimetre;
    const auto length = units->find("length");
    if (length == units->end() || !length->is_string()) return math::LengthUnit::Millimetre;
    return math::lengthUnitFrom(length->get<std::string>()).value_or(math::LengthUnit::Millimetre);
}

static json buildDocumentRoot(const doc::Document& doc, bool includeTessellation) {
    json root;
    root["version"] = kFormatVersion;
    root["type"] = doc.type() == doc::DocumentType::Part ? "hzpart" : "hcad";
    root["units"] = unitsJson(doc.lengthUnit());

    // --- Dimension style ---
    const auto& ds = doc.draftDocument().dimensionStyle();
    root["dimensionStyle"] = {{"textHeight", ds.textHeight},
                              {"arrowSize", ds.arrowSize},
                              {"arrowAngle", ds.arrowAngle},
                              {"extensionGap", ds.extensionGap},
                              {"extensionOvershoot", ds.extensionOvershoot},
                              {"precision", ds.precision},
                              {"showUnits", ds.showUnits},
                              {"unit", ds.unit}};

    // --- Layer table ---
    json layersArray = json::array();
    for (const auto& name : doc.layerManager().layerNames()) {
        const auto* lp = doc.layerManager().getLayer(name);
        if (!lp) continue;
        json layerObj;
        layerObj["name"] = lp->name;
        layerObj["color"] = lp->color;
        layerObj["lineWidth"] = lp->lineWidth;
        layerObj["visible"] = lp->visible;
        layerObj["locked"] = lp->locked;
        layerObj["lineType"] = lp->lineType;
        layersArray.push_back(layerObj);
    }
    root["layers"] = layersArray;
    root["currentLayer"] = doc.layerManager().currentLayer();

    // --- Block definitions ---
    json blocksArray = json::array();
    for (const auto& name : doc.draftDocument().blockTable().blockNames()) {
        auto def = doc.draftDocument().blockTable().findBlock(name);
        if (!def) continue;
        json blockObj;
        blockObj["name"] = def->name;
        blockObj["basePoint"] = {{"x", def->basePoint.x}, {"y", def->basePoint.y}};
        json defEnts = json::array();
        for (const auto& subEnt : def->entities) {
            json se;
            se["layer"] = subEnt->layer();
            se["color"] = subEnt->color();
            se["lineWidth"] = subEnt->lineWidth();
            se["lineType"] = subEnt->lineType();
            if (auto* ln = dynamic_cast<const draft::DraftLine*>(subEnt.get())) {
                se["type"] = "line";
                se["start"] = {{"x", ln->start().x}, {"y", ln->start().y}};
                se["end"] = {{"x", ln->end().x}, {"y", ln->end().y}};
            } else if (auto* ci = dynamic_cast<const draft::DraftCircle*>(subEnt.get())) {
                se["type"] = "circle";
                se["center"] = {{"x", ci->center().x}, {"y", ci->center().y}};
                se["radius"] = ci->radius();
            } else if (auto* ar = dynamic_cast<const draft::DraftArc*>(subEnt.get())) {
                se["type"] = "arc";
                se["center"] = {{"x", ar->center().x}, {"y", ar->center().y}};
                se["radius"] = ar->radius();
                se["startAngle"] = ar->startAngle();
                se["endAngle"] = ar->endAngle();
            } else if (auto* re = dynamic_cast<const draft::DraftRectangle*>(subEnt.get())) {
                se["type"] = "rectangle";
                se["corner1"] = {{"x", re->corner1().x}, {"y", re->corner1().y}};
                se["corner2"] = {{"x", re->corner2().x}, {"y", re->corner2().y}};
            } else if (auto* pl = dynamic_cast<const draft::DraftPolyline*>(subEnt.get())) {
                se["type"] = "polyline";
                se["closed"] = pl->closed();
                json pts = json::array();
                for (const auto& pt : pl->points()) pts.push_back({{"x", pt.x}, {"y", pt.y}});
                se["points"] = pts;
            } else if (auto* sp = dynamic_cast<const draft::DraftSpline*>(subEnt.get())) {
                se["type"] = "spline";
                se["closed"] = sp->closed();
                json cps = json::array();
                for (const auto& cp : sp->controlPoints())
                    cps.push_back({{"x", cp.x}, {"y", cp.y}});
                se["controlPoints"] = cps;
                if (sp->hasNonUniformWeights()) {
                    json wArr = json::array();
                    for (double w : sp->weights()) wArr.push_back(w);
                    se["weights"] = wArr;
                }
            } else if (auto* txt = dynamic_cast<const draft::DraftText*>(subEnt.get())) {
                se["type"] = "text";
                se["position"] = {{"x", txt->position().x}, {"y", txt->position().y}};
                se["text"] = txt->text();
                se["textHeight"] = txt->textHeight();
                se["rotation"] = txt->rotation();
                se["alignment"] = static_cast<int>(txt->alignment());
            } else if (auto* hatch = dynamic_cast<const draft::DraftHatch*>(subEnt.get())) {
                se["type"] = "hatch";
                se["pattern"] = static_cast<int>(hatch->pattern());
                se["angle"] = hatch->angle();
                se["spacing"] = hatch->spacing();
                json bnd = json::array();
                for (const auto& pt : hatch->boundary()) bnd.push_back({{"x", pt.x}, {"y", pt.y}});
                se["boundary"] = bnd;
            } else if (auto* el = dynamic_cast<const draft::DraftEllipse*>(subEnt.get())) {
                se["type"] = "ellipse";
                se["center"] = {{"x", el->center().x}, {"y", el->center().y}};
                se["semiMajor"] = el->semiMajor();
                se["semiMinor"] = el->semiMinor();
                se["rotation"] = el->rotation();
            }
            defEnts.push_back(se);
        }
        blockObj["entities"] = defEnts;
        blocksArray.push_back(blockObj);
    }
    root["blocks"] = blocksArray;

    // --- Entities (top-level for backward compatibility) ---
    json entitiesArray = json::array();
    for (const auto& entity : doc.draftDocument().entities()) {
        entitiesArray.push_back(serializeEntity(*entity));
    }
    root["entities"] = entitiesArray;

    // --- Constraints ---
    root["constraints"] = constraintsToJson(doc.constraintSystem());

    // --- Design variables (v14: nested objects with optional expressions) ---
    json designVars = json::object();
    const auto& paramReg = doc.parameterRegistry();
    for (const auto& [name, value] : paramReg.all()) {
        json varObj;
        varObj["value"] = value;
        if (paramReg.isExpression(name)) {
            varObj["expression"] = paramReg.getExpression(name);
        }
        designVars[name] = varObj;
    }
    root["designVariables"] = designVars;

    // --- Configurations (Phase 156): absent when there are none; an older
    // build reads the variables as the document has them. ---
    const auto& configurations = doc.configurations();
    if (configurations.size() > 0) {
        json table = json::array();
        for (const std::string& name : configurations.configurationNames()) {
            json overrides = json::object();
            for (const auto& [variable, expression] : configurations.overrides(name)) {
                overrides[variable] = expression;
            }
            table.push_back({{"name", name}, {"values", overrides}});
        }
        root["configurations"] = {{"active", configurations.active()}, {"table", table}};
    }

    // --- Sketches ---
    json sketchesArray = json::array();
    for (const auto& sketch : doc.sketches()) {
        json skObj;
        skObj["id"] = sketch->id();
        skObj["name"] = sketch->name();

        // The plane it was drawn on; for one that follows a face, the face,
        // and where a build last placed it there (Phase 157).
        skObj["plane"] = planeToJson(sketch->drawnPlane());
        if (!sketch->face().empty()) skObj["face"] = sketch->face();
        if (sketch->placed()) skObj["placed"] = planeToJson(*sketch->placed());

        json skEntities = json::array();
        for (const auto& entity : sketch->entities()) {
            skEntities.push_back(serializeEntity(*entity));
        }
        skObj["entities"] = skEntities;
        skObj["constraints"] = constraintsToJson(sketch->constraintSystem());

        sketchesArray.push_back(skObj);
    }
    root["sketches"] = sketchesArray;

    // --- Feature tree (v15+; v16 stores real sketch IDs and full inputs) ---
    json featureTreeArray = json::array();
    const auto& ftree = doc.featureTree();
    for (size_t fi = 0; fi < ftree.featureCount(); ++fi) {
        const auto* feat = ftree.feature(fi);
        json fObj;
        fObj["featureID"] = feat->featureID();

        if (const auto* ext = dynamic_cast<const doc::ExtrudeFeature*>(feat)) {
            fObj["type"] = "extrude";
            fObj["distance"] = ext->distance();
            fObj["direction"] = {ext->direction().x, ext->direction().y, ext->direction().z};
            if (ext->sketch()) fObj["sketchId"] = ext->sketch()->id();
            if (ext->hasCurvedProfile()) fObj["segments"] = ext->segments();
            if (ext->chordTolerance() > 0.0) fObj["chordTolerance"] = ext->chordTolerance();
            // How far it goes, when not simply its distance (Phase 134).
            if (ext->extent() != doc::ExtrudeFeature::Extent::Blind) {
                fObj["extent"] = static_cast<int>(ext->extent());
            }
        } else if (const auto* rev = dynamic_cast<const doc::RevolveFeature*>(feat)) {
            fObj["type"] = "revolve";
            fObj["angle"] = rev->angle();
            fObj["axisPoint"] = {rev->axisPoint().x, rev->axisPoint().y, rev->axisPoint().z};
            fObj["axisDir"] = {rev->axisDir().x, rev->axisDir().y, rev->axisDir().z};
            fObj["segments"] = rev->segments();
            if (rev->chordTolerance() > 0.0) fObj["chordTolerance"] = rev->chordTolerance();
            if (rev->sketch()) fObj["sketchId"] = rev->sketch()->id();
        } else if (const auto* loft = dynamic_cast<const doc::LoftFeature*>(feat)) {
            fObj["type"] = "loft";
            json sectionIds = json::array();
            for (const auto& sk : loft->sections()) {
                if (sk) sectionIds.push_back(sk->id());
            }
            fObj["sketchIds"] = sectionIds;
        } else if (const auto* sweep = dynamic_cast<const doc::SweepFeature*>(feat)) {
            fObj["type"] = "sweep";
            if (sweep->profile()) fObj["sketchId"] = sweep->profile()->id();
            if (sweep->path()) fObj["pathSketchId"] = sweep->path()->id();
            fObj["segments"] = sweep->segments();
            if (sweep->chordTolerance() > 0.0) fObj["chordTolerance"] = sweep->chordTolerance();
        } else if (const auto* draft = dynamic_cast<const doc::DraftFeature*>(feat)) {
            fObj["type"] = "draft";
            fObj["pullDir"] = {draft->pullDir().x, draft->pullDir().y, draft->pullDir().z};
            fObj["neutralPoint"] = {draft->neutralPoint().x, draft->neutralPoint().y,
                                    draft->neutralPoint().z};
            fObj["angle"] = draft->angle();
        } else if (const auto* shell = dynamic_cast<const doc::ShellFeature*>(feat)) {
            fObj["type"] = "shell";
            fObj["thickness"] = shell->thickness();
            json removed = json::array();
            for (const auto& id : shell->removedFaceIds()) removed.push_back(id.tag());
            fObj["removedFaces"] = removed;
        } else if (const auto* fillet = dynamic_cast<const doc::FilletFeature*>(feat)) {
            fObj["type"] = "fillet";
            fObj["radius"] = fillet->radius();
            fObj["arcSegments"] = fillet->arcSegments();
            if (fillet->chordTolerance() > 0.0) fObj["chordTolerance"] = fillet->chordTolerance();
            json edges = json::array();
            for (const auto& id : fillet->edgeIds()) edges.push_back(id.tag());
            fObj["edges"] = edges;
        } else if (const auto* chamfer = dynamic_cast<const doc::ChamferFeature*>(feat)) {
            fObj["type"] = "chamfer";
            fObj["distance"] = chamfer->distance();
            json edges = json::array();
            for (const auto& id : chamfer->edgeIds()) edges.push_back(id.tag());
            fObj["edges"] = edges;
        } else if (const auto* boolean = dynamic_cast<const doc::BooleanFeature*>(feat)) {
            fObj["type"] = "boolean";
            switch (boolean->booleanType()) {
                case model::BooleanType::Union:
                    fObj["operation"] = "union";
                    break;
                case model::BooleanType::Subtract:
                    fObj["operation"] = "subtract";
                    break;
                case model::BooleanType::Intersect:
                    fObj["operation"] = "intersect";
                    break;
            }
        } else if (const auto* pat = dynamic_cast<const doc::PatternFeature*>(feat)) {
            fObj["type"] = "pattern";
            fObj["kind"] = pat->kind() == doc::PatternFeature::Kind::Linear ? "linear" : "circular";
            fObj["vecA"] = {pat->vecA().x, pat->vecA().y, pat->vecA().z};
            fObj["vecB"] = {pat->vecB().x, pat->vecB().y, pat->vecB().z};
            fObj["scalar"] = pat->scalar();
            fObj["count"] = pat->count();
            fObj["suppressed"] = pat->suppressedInstances();
            // The features it repeats (Phase 134); none, the whole part.
            if (!pat->targets().empty()) fObj["features"] = pat->targets();
        } else if (const auto* imported = dynamic_cast<const doc::ImportedBodyFeature*>(feat)) {
            // The body itself travels with the part, as STEP text, so the part
            // does not depend on the file it was imported from.
            fObj["type"] = "imported";
            fObj["source"] = imported->source();
            if (imported->solid()) fObj["step"] = StepFormat::toString({imported->solid().get()});
        } else if (const auto* datum = dynamic_cast<const doc::DatumFeature*>(feat)) {
            fObj["type"] = "datum";
            switch (datum->datumKind()) {
                case doc::DatumFeature::DatumKind::Plane:
                    fObj["datumKind"] = "plane";
                    break;
                case doc::DatumFeature::DatumKind::Axis:
                    fObj["datumKind"] = "axis";
                    break;
                case doc::DatumFeature::DatumKind::Point:
                    fObj["datumKind"] = "point";
                    break;
            }
            fObj["origin"] = {datum->origin().x, datum->origin().y, datum->origin().z};
            fObj["dirA"] = {datum->dirA().x, datum->dirA().y, datum->dirA().z};
            fObj["dirB"] = {datum->dirB().x, datum->dirB().y, datum->dirB().z};
        } else if (const auto* prim = dynamic_cast<const doc::PrimitiveFeature*>(feat)) {
            fObj["type"] = "primitive";
            switch (prim->kind()) {
                case doc::PrimitiveFeature::Kind::Box:
                    fObj["primitiveKind"] = "box";
                    break;
                case doc::PrimitiveFeature::Kind::Cylinder:
                    fObj["primitiveKind"] = "cylinder";
                    break;
                case doc::PrimitiveFeature::Kind::Sphere:
                    fObj["primitiveKind"] = "sphere";
                    break;
                case doc::PrimitiveFeature::Kind::Cone:
                    fObj["primitiveKind"] = "cone";
                    break;
                case doc::PrimitiveFeature::Kind::Torus:
                    fObj["primitiveKind"] = "torus";
                    break;
            }
            fObj["p0"] = prim->p0();
            fObj["p1"] = prim->p1();
            fObj["p2"] = prim->p2();
            if (prim->isFaceted()) fObj["segments"] = prim->segments();
            if (prim->chordTolerance() > 0.0) fObj["chordTolerance"] = prim->chordTolerance();
            // Where it stands (Phase 134); absent, at the origin along +Z.
            if (prim->isPlaced()) {
                const auto& b = prim->basePoint();
                const auto& a = prim->axisDirection();
                fObj["basePoint"] = {b.x, b.y, b.z};
                fObj["axisDirection"] = {a.x, a.y, a.z};
            }
        }

        // How a created body combines with the part (Phase 102).
        // ("operation" is taken: the Boolean feature's own type.)
        if (feat->createsNewBody()) {
            fObj["bodyOperation"] = doc::bodyOperationName(feat->operation());
        }
        // Phase 104. ("suppressed" is taken: a pattern's skipped instances.)
        if (feat->isSuppressed()) fObj["featureSuppressed"] = true;
        // Phase 106. Absent: positional names, as every file before it.
        if (feat->naming() != model::NamingScheme::Positional) {
            fObj["naming"] = static_cast<int>(feat->naming());
        }
        // Phase 155: parameters given as expressions of the variables. The
        // parameters beside them hold the values they worked out to, which
        // a build that reads no expressions uses.
        if (!feat->parameterExpressions().empty()) {
            json expressions = json::object();
            for (const auto& [name, text] : feat->parameterExpressions()) {
                expressions[name] = text;
            }
            fObj["expressions"] = expressions;
        }

        featureTreeArray.push_back(fObj);
    }
    root["featureTree"] = featureTreeArray;
    // Where the part is rolled back to, if it is (Phase 133; before, a save
    // rolled it forward).
    if (doc.featureTree().rollbackIndex() >= 0) {
        root["rollbackIndex"] = doc.featureTree().rollbackIndex();
    }

    // --- Tessellation cache (v16+, parts only) ---
    // Enables lightweight assembly loading: readers can display the part
    // without replaying the feature tree.
    if (includeTessellation && doc.solid()) {
        geo::MeshData mesh = model::SolidTessellator::tessellate(*doc.solid());
        json cache;
        cache["positions"] = mesh.positions;
        cache["normals"] = mesh.normals;
        cache["indices"] = mesh.indices;
        // Its faces and edges by name (Phase 143), so a component drawn from
        // the cache can be clicked: a mate is made on a clicked face. Keys
        // an older build does not read.
        if (mesh.hasFaces()) {
            cache["faceTags"] = mesh.faceTags;
            cache["triangleFaces"] = mesh.triangleFaces;
            json edges = json::array();
            for (const auto& edge : mesh.edges) {
                edges.push_back({{"tag", edge.tag}, {"points", edge.points}});
            }
            cache["edges"] = std::move(edges);
        }
        root["tessellationCache"] = cache;
    }

    return root;
}

bool NativeFormat::save(const std::string& filePath, const doc::Document& doc, std::string* error) {
    const json root = buildDocumentRoot(doc, /*includeTessellation=*/true);

    // Pretty-print drawings for diff-friendliness, but write compactly when a
    // tessellation cache is embedded — indented output puts one mesh number
    // per line and inflates part files by orders of magnitude.
    const int indent = root.contains("tessellationCache") ? -1 : 2;
    return writeFileAtomically(pathFromUtf8(filePath), dumpJson(root, indent), error);
}

std::string NativeFormat::documentToJson(const doc::Document& doc, bool includeTessellation) {
    return dumpJson(buildDocumentRoot(doc, includeTessellation), -1);
}

// ---------------------------------------------------------------------------
// Entity deserialization helper
// ---------------------------------------------------------------------------

static std::shared_ptr<draft::DraftEntity> deserializeEntity(const json& obj,
                                                             const draft::BlockTable* blockTable) {
    std::string type = obj.value("type", "");
    std::string layer = obj.value("layer", "0");
    uint32_t color = obj.value("color", 0xFFFFFFFFu);
    double lineWidth = obj.value("lineWidth", 0.0);

    std::shared_ptr<draft::DraftEntity> entity;

    if (type == "line") {
        double sx = obj.at("start").at("x").get<double>();
        double sy = obj.at("start").at("y").get<double>();
        double ex = obj.at("end").at("x").get<double>();
        double ey = obj.at("end").at("y").get<double>();
        entity = std::make_shared<draft::DraftLine>(math::Vec2(sx, sy), math::Vec2(ex, ey));
    } else if (type == "circle") {
        double cx = obj.at("center").at("x").get<double>();
        double cy = obj.at("center").at("y").get<double>();
        double r = obj.at("radius").get<double>();
        entity = std::make_shared<draft::DraftCircle>(math::Vec2(cx, cy), r);
    } else if (type == "arc") {
        double cx = obj.at("center").at("x").get<double>();
        double cy = obj.at("center").at("y").get<double>();
        double r = obj.at("radius").get<double>();
        double sa = obj.at("startAngle").get<double>();
        double ea = obj.at("endAngle").get<double>();
        entity = std::make_shared<draft::DraftArc>(math::Vec2(cx, cy), r, sa, ea);
    } else if (type == "rectangle") {
        double c1x = obj.at("corner1").at("x").get<double>();
        double c1y = obj.at("corner1").at("y").get<double>();
        double c2x = obj.at("corner2").at("x").get<double>();
        double c2y = obj.at("corner2").at("y").get<double>();
        entity =
            std::make_shared<draft::DraftRectangle>(math::Vec2(c1x, c1y), math::Vec2(c2x, c2y));
    } else if (type == "polyline") {
        bool closed = obj.value("closed", false);
        std::vector<math::Vec2> points;
        for (const auto& pt : obj.at("points")) {
            points.emplace_back(pt.at("x").get<double>(), pt.at("y").get<double>());
        }
        entity = std::make_shared<draft::DraftPolyline>(points, closed);
    } else if (type == "linearDimension") {
        auto p1 = math::Vec2(obj.at("defPoint1").at("x").get<double>(),
                             obj.at("defPoint1").at("y").get<double>());
        auto p2 = math::Vec2(obj.at("defPoint2").at("x").get<double>(),
                             obj.at("defPoint2").at("y").get<double>());
        auto dp = math::Vec2(obj.at("dimLinePoint").at("x").get<double>(),
                             obj.at("dimLinePoint").at("y").get<double>());
        auto orient = static_cast<draft::DraftLinearDimension::Orientation>(
            intField(obj, "orientation", 0, 0, kLastOrientation));
        auto dim = std::make_shared<draft::DraftLinearDimension>(p1, p2, dp, orient);
        if (obj.contains("textOverride"))
            dim->setTextOverride(obj.at("textOverride").get<std::string>());
        entity = dim;
    } else if (type == "radialDimension") {
        auto center = math::Vec2(obj.at("center").at("x").get<double>(),
                                 obj.at("center").at("y").get<double>());
        double radius = obj.at("radius").get<double>();
        auto textPt = math::Vec2(obj.at("textPoint").at("x").get<double>(),
                                 obj.at("textPoint").at("y").get<double>());
        bool isDiam = obj.value("isDiameter", false);
        auto dim = std::make_shared<draft::DraftRadialDimension>(center, radius, textPt, isDiam);
        if (obj.contains("textOverride"))
            dim->setTextOverride(obj.at("textOverride").get<std::string>());
        entity = dim;
    } else if (type == "angularDimension") {
        auto vertex = math::Vec2(obj.at("vertex").at("x").get<double>(),
                                 obj.at("vertex").at("y").get<double>());
        auto l1 = math::Vec2(obj.at("line1Point").at("x").get<double>(),
                             obj.at("line1Point").at("y").get<double>());
        auto l2 = math::Vec2(obj.at("line2Point").at("x").get<double>(),
                             obj.at("line2Point").at("y").get<double>());
        double arcR = obj.at("arcRadius").get<double>();
        auto dim = std::make_shared<draft::DraftAngularDimension>(vertex, l1, l2, arcR);
        if (obj.contains("textOverride"))
            dim->setTextOverride(obj.at("textOverride").get<std::string>());
        entity = dim;
    } else if (type == "leader") {
        std::vector<math::Vec2> points;
        for (const auto& pt : obj.at("points")) {
            points.emplace_back(pt.at("x").get<double>(), pt.at("y").get<double>());
        }
        std::string text = obj.value("text", "");
        auto ldr = std::make_shared<draft::DraftLeader>(points, text);
        if (obj.contains("textOverride"))
            ldr->setTextOverride(obj.at("textOverride").get<std::string>());
        entity = ldr;
    } else if (type == "blockRef") {
        if (blockTable) {
            std::string blockName = obj.value("blockName", "");
            auto def = blockTable->findBlock(blockName);
            if (def) {
                auto pos = math::Vec2(obj.at("insertPos").at("x").get<double>(),
                                      obj.at("insertPos").at("y").get<double>());
                double rot = obj.value("rotation", 0.0);
                double scl = obj.value("scale", 1.0);
                auto ref = std::make_shared<draft::DraftBlockRef>(def, pos, rot, scl);
                // Absent before a reference could be mirrored.
                const auto mirrored = obj.find("mirrored");
                if (mirrored != obj.end() && mirrored->is_boolean()) {
                    ref->setMirrored(mirrored->get<bool>());
                }
                entity = ref;
            }
        }
    } else if (type == "text") {
        auto pos = math::Vec2(obj.at("position").at("x").get<double>(),
                              obj.at("position").at("y").get<double>());
        std::string text = obj.value("text", "");
        double textHeight = obj.value("textHeight", 2.5);
        auto txt = std::make_shared<draft::DraftText>(pos, text, textHeight);
        if (obj.contains("rotation")) txt->setRotation(obj.at("rotation").get<double>());
        if (obj.contains("alignment"))
            txt->setAlignment(static_cast<draft::TextAlignment>(
                intField(obj, "alignment", 0, 0, kLastAlignment)));
        entity = txt;
    } else if (type == "spline") {
        bool closed = obj.value("closed", false);
        std::vector<math::Vec2> controlPoints;
        for (const auto& cp : obj.at("controlPoints")) {
            controlPoints.emplace_back(cp.at("x").get<double>(), cp.at("y").get<double>());
        }
        auto splineEnt = std::make_shared<draft::DraftSpline>(controlPoints, closed);
        if (obj.contains("weights")) {
            std::vector<double> wts;
            wts.reserve(obj.at("weights").size());
            for (const auto& w : obj.at("weights")) wts.push_back(w.get<double>());
            splineEnt->setWeights(wts);
        }
        entity = splineEnt;
    } else if (type == "hatch") {
        std::vector<math::Vec2> boundary;
        for (const auto& pt : obj.at("boundary")) {
            boundary.emplace_back(pt.at("x").get<double>(), pt.at("y").get<double>());
        }
        auto hatchPattern =
            static_cast<draft::HatchPattern>(intField(obj, "pattern", 1, 0, kLastHatchPattern));
        double hatchAngle = obj.value("angle", 0.0);
        double hatchSpacing = obj.value("spacing", 1.0);
        entity =
            std::make_shared<draft::DraftHatch>(boundary, hatchPattern, hatchAngle, hatchSpacing);
    } else if (type == "ellipse") {
        auto center = math::Vec2(obj.at("center").at("x").get<double>(),
                                 obj.at("center").at("y").get<double>());
        double semiMajor = obj.value("semiMajor", 1.0);
        double semiMinor = obj.value("semiMinor", 1.0);
        double rot = obj.value("rotation", 0.0);
        entity = std::make_shared<draft::DraftEllipse>(center, semiMajor, semiMinor, rot);
    }

    if (entity) {
        if (obj.contains("id")) {
            uint64_t savedId = obj.at("id").get<uint64_t>();
            entity->setId(savedId);
            draft::DraftEntity::advanceIdCounter(savedId);
        }
        entity->setLayer(layer);
        entity->setColor(color);
        entity->setLineWidth(lineWidth);
        entity->setLineType(static_cast<int>(intField(obj, "lineType", 0, 0, kLastLineType)));
        uint64_t gid = obj.value("groupId", uint64_t(0));
        entity->setGroupId(gid);
    }

    return entity;
}

/// Record an item a load had to leave out: "<kind> <n> (<type>): <why>".
static void noteSkipped(ImportReport* report, const std::string& kind, size_t index,
                        const json& obj, const std::string& why) {
    if (!report) return;
    std::string line = kind + " " + std::to_string(index + 1);
    if (obj.is_object() && obj.contains("type") && obj.at("type").is_string()) {
        line += " (" + obj.at("type").get<std::string>() + ")";
    }
    report->skipped.push_back(line + ": " + why);
}

/// Populate a Document from a parsed envelope. Shared by load() and
/// BinaryFormat.
static bool loadDocumentRoot(const json& root, doc::Document& doc, ImportReport* report) {
    if (!root.contains("version") || !root.contains("entities")) return false;

    doc.draftDocument().clear();
    doc.layerManager().clear();
    doc.constraintSystem().clear();
    doc.parameterRegistry().clear();

    // --- Document type (v16+; earlier files are all drawings) ---
    // Defensive read: a malformed (non-string) "type" must not throw.
    std::string typeTag = "hcad";
    if (root.contains("type") && root.at("type").is_string()) {
        typeTag = root.at("type").get<std::string>();
    }
    doc.setType(typeTag == "hzpart" ? doc::DocumentType::Part : doc::DocumentType::Drawing);
    doc.setLengthUnit(unitsFrom(root));

    // --- Load dimension style (v4+) ---
    if (root.contains("dimensionStyle")) {
        const auto& dsObj = root.at("dimensionStyle");
        draft::DimensionStyle ds;
        ds.textHeight = dsObj.value("textHeight", 2.5);
        ds.arrowSize = dsObj.value("arrowSize", 1.5);
        ds.arrowAngle = dsObj.value("arrowAngle", 0.3);
        ds.extensionGap = dsObj.value("extensionGap", 0.5);
        ds.extensionOvershoot = dsObj.value("extensionOvershoot", 1.0);
        ds.precision = static_cast<int>(intField(dsObj, "precision", 2, 0, 12));
        ds.showUnits = dsObj.value("showUnits", false);
        // Absent before Phase 129 (millimetres); an unknown one reads as them.
        const auto unit = dsObj.find("unit");
        if (unit != dsObj.end() && unit->is_string() &&
            draft::isDimensionUnit(unit->get<std::string>())) {
            ds.unit = unit->get<std::string>();
        }
        doc.draftDocument().setDimensionStyle(ds);
    }

    // --- Load layer table (v3+) ---
    if (root.contains("layers")) {
        for (const auto& layerObj : root.at("layers")) {
            draft::LayerProperties props;
            props.name = layerObj.value("name", "0");
            props.color = layerObj.value("color", 0xFFFFFFFFu);
            props.lineWidth = layerObj.value("lineWidth", 1.0);
            props.visible = layerObj.value("visible", true);
            props.locked = layerObj.value("locked", false);
            props.lineType = static_cast<int>(intField(layerObj, "lineType", 1, 0, kLastLineType));
            if (props.name == "0") {
                // Update default layer properties instead of adding duplicate.
                auto* defaultLayer = doc.layerManager().getLayer("0");
                if (defaultLayer) *defaultLayer = props;
            } else {
                doc.layerManager().addLayer(props);
            }
        }
        if (root.contains("currentLayer")) {
            doc.layerManager().setCurrentLayer(root.at("currentLayer").get<std::string>());
        }
    }

    // --- Load block definitions (v6+) ---
    if (root.contains("blocks")) {
        size_t blockIndex = 0;
        for (const auto& blockObj : root.at("blocks")) {
            const size_t thisBlock = blockIndex++;
            try {
                auto def = std::make_shared<draft::BlockDefinition>();
                def->name = blockObj.value("name", "");
                def->basePoint = math::Vec2(blockObj.at("basePoint").at("x").get<double>(),
                                            blockObj.at("basePoint").at("y").get<double>());
                if (blockObj.contains("entities")) {
                    for (const auto& se : blockObj.at("entities")) {
                        std::string stype = se.value("type", "");
                        std::shared_ptr<draft::DraftEntity> subEnt;
                        if (stype == "line") {
                            subEnt = std::make_shared<draft::DraftLine>(
                                math::Vec2(se.at("start").at("x").get<double>(),
                                           se.at("start").at("y").get<double>()),
                                math::Vec2(se.at("end").at("x").get<double>(),
                                           se.at("end").at("y").get<double>()));
                        } else if (stype == "circle") {
                            subEnt = std::make_shared<draft::DraftCircle>(
                                math::Vec2(se.at("center").at("x").get<double>(),
                                           se.at("center").at("y").get<double>()),
                                se.at("radius").get<double>());
                        } else if (stype == "arc") {
                            subEnt = std::make_shared<draft::DraftArc>(
                                math::Vec2(se.at("center").at("x").get<double>(),
                                           se.at("center").at("y").get<double>()),
                                se.at("radius").get<double>(), se.at("startAngle").get<double>(),
                                se.at("endAngle").get<double>());
                        } else if (stype == "rectangle") {
                            subEnt = std::make_shared<draft::DraftRectangle>(
                                math::Vec2(se.at("corner1").at("x").get<double>(),
                                           se.at("corner1").at("y").get<double>()),
                                math::Vec2(se.at("corner2").at("x").get<double>(),
                                           se.at("corner2").at("y").get<double>()));
                        } else if (stype == "polyline") {
                            std::vector<math::Vec2> pts;
                            for (const auto& pt : se.at("points"))
                                pts.emplace_back(pt.at("x").get<double>(),
                                                 pt.at("y").get<double>());
                            subEnt = std::make_shared<draft::DraftPolyline>(
                                pts, se.value("closed", false));
                        } else if (stype == "spline") {
                            std::vector<math::Vec2> cps;
                            for (const auto& cp : se.at("controlPoints"))
                                cps.emplace_back(cp.at("x").get<double>(),
                                                 cp.at("y").get<double>());
                            auto blkSp = std::make_shared<draft::DraftSpline>(
                                cps, se.value("closed", false));
                            if (se.contains("weights")) {
                                std::vector<double> wts;
                                wts.reserve(se.at("weights").size());
                                for (const auto& w : se.at("weights"))
                                    wts.push_back(w.get<double>());
                                blkSp->setWeights(wts);
                            }
                            subEnt = blkSp;
                        } else if (stype == "text") {
                            auto pos = math::Vec2(se.at("position").at("x").get<double>(),
                                                  se.at("position").at("y").get<double>());
                            auto txt = std::make_shared<draft::DraftText>(
                                pos, se.value("text", ""), se.value("textHeight", 2.5));
                            if (se.contains("rotation"))
                                txt->setRotation(se.at("rotation").get<double>());
                            if (se.contains("alignment"))
                                txt->setAlignment(static_cast<draft::TextAlignment>(
                                    intField(se, "alignment", 0, 0, kLastAlignment)));
                            subEnt = txt;
                        } else if (stype == "hatch") {
                            std::vector<math::Vec2> boundary;
                            for (const auto& pt : se.at("boundary"))
                                boundary.emplace_back(pt.at("x").get<double>(),
                                                      pt.at("y").get<double>());
                            subEnt = std::make_shared<draft::DraftHatch>(
                                boundary,
                                static_cast<draft::HatchPattern>(
                                    intField(se, "pattern", 1, 0, kLastHatchPattern)),
                                se.value("angle", 0.0), se.value("spacing", 1.0));
                        } else if (stype == "ellipse") {
                            auto ctr = math::Vec2(se.at("center").at("x").get<double>(),
                                                  se.at("center").at("y").get<double>());
                            subEnt = std::make_shared<draft::DraftEllipse>(
                                ctr, se.value("semiMajor", 1.0), se.value("semiMinor", 1.0),
                                se.value("rotation", 0.0));
                        }
                        if (subEnt) {
                            subEnt->setLayer(se.value("layer", "0"));
                            subEnt->setColor(se.value("color", 0u));
                            subEnt->setLineWidth(se.value("lineWidth", 0.0));
                            subEnt->setLineType(
                                static_cast<int>(intField(se, "lineType", 0, 0, kLastLineType)));
                            def->entities.push_back(subEnt);
                        }
                    }
                }
                doc.draftDocument().blockTable().addBlock(def);
            } catch (const std::exception& e) {
                noteSkipped(report, "block", thisBlock, blockObj, jsonMessage(e));
                continue;  // Skip malformed block definitions.
            }
        }
    }

    // --- Load entities ---
    // Every ID the file gives its entities first, so that an ID handed to a
    // duplicate below is not one a later entity holds in the file.
    for (const auto& obj : root.at("entities")) {
        if (!obj.is_object()) continue;
        const auto id = obj.find("id");
        if (id != obj.end() && id->is_number_unsigned()) {
            draft::DraftEntity::advanceIdCounter(id->get<uint64_t>());
        }
    }
    const auto* blockTablePtr = &doc.draftDocument().blockTable();
    size_t entityIndex = 0;
    for (const auto& obj : root.at("entities")) {
        const size_t thisEntity = entityIndex++;
        try {
            auto entity = deserializeEntity(obj, blockTablePtr);
            if (entity) {
                uint64_t gid = entity->groupId();
                if (gid != 0) {
                    doc.draftDocument().advanceGroupIdCounter(gid);
                }
                // Two entities under one ID (a damaged or hand-edited file):
                // the later one would hide the earlier from every lookup,
                // selection and deletion. It keeps its place, under a new ID;
                // constraints naming the ID stay with the earlier one.
                if (doc.draftDocument().findEntity(entity->id()) != nullptr) {
                    const uint64_t taken = entity->id();
                    entity->setId(draft::DraftEntity::newId());
                    if (report) {
                        report->approximated.push_back(
                            "entity " + std::to_string(thisEntity + 1) + ": its ID " +
                            std::to_string(taken) +
                            " belongs to an earlier entity; it was given a new one");
                    }
                }
                doc.draftDocument().addEntity(entity);
            } else {
                noteSkipped(report, "entity", thisEntity, obj,
                            "not a kind of entity this version reads");
            }
        } catch (const std::exception& e) {
            noteSkipped(report, "entity", thisEntity, obj, jsonMessage(e));
            continue;  // Skip malformed entities.
        }
    }

    // --- Load constraints (v5+) ---
    if (root.contains("constraints")) {
        constraintsFromJson(root.at("constraints"), doc.draftDocument(), doc.constraintSystem());
    }

    // --- Load configurations (Phase 156). What is not a configuration is
    // left out; an active one that is not there leaves none active. ---
    doc.configurations() = {};
    if (const auto configurations = root.find("configurations");
        configurations != root.end() && configurations->is_object()) {
        if (const auto table = configurations->find("table");
            table != configurations->end() && table->is_array()) {
            for (const json& row : *table) {
                if (!row.is_object() || !row.contains("name") || !row.at("name").is_string()) {
                    continue;
                }
                doc::ConfigurationTable::Overrides overrides;
                if (const auto values = row.find("values");
                    values != row.end() && values->is_object()) {
                    for (const auto& [variable, expression] : values->items()) {
                        if (expression.is_string())
                            overrides[variable] = expression.get<std::string>();
                    }
                }
                doc.configurations().setConfiguration(row.at("name").get<std::string>(), overrides);
            }
        }
        if (const auto active = configurations->find("active");
            active != configurations->end() && active->is_string()) {
            doc.configurations().setActive(active->get<std::string>());
        }
    }

    // --- Load design variables (v13+, v14 nested format) ---
    if (root.contains("designVariables")) {
        auto& pReg = doc.parameterRegistry();
        for (const auto& [name, value] : root.at("designVariables").items()) {
            if (value.is_number()) {
                // v13 flat format: "width": 50.0
                pReg.set(name, value.get<double>());
            } else if (value.is_object()) {
                // v14 nested format: "width": {"value": 50.0, "expression": "..."}
                if (value.contains("expression") && value.at("expression").is_string()) {
                    pReg.setExpression(name, value.at("expression").get<std::string>());
                } else if (value.contains("value") && value.at("value").is_number()) {
                    pReg.set(name, value.at("value").get<double>());
                }
            }
        }
    }

    // Rebuild spatial index after loading all entities.
    doc.draftDocument().rebuildSpatialIndex();

    // --- Load sketches ---
    if (root.contains("sketches")) {
        doc.sketches().clear();

        size_t sketchIndex = 0;
        for (const auto& skObj : root.at("sketches")) {
            const size_t thisSketch = sketchIndex++;
            try {
                // Reconstruct SketchPlane
                draft::SketchPlane plane;  // default XY
                if (skObj.contains("plane")) plane = planeFromJson(skObj.at("plane"));

                auto sketch = std::make_shared<doc::Sketch>(plane);
                // Phase 157: the face it follows, and where it was placed.
                if (skObj.contains("face") && skObj.at("face").is_string()) {
                    sketch->setFace(skObj.at("face").get<std::string>());
                }
                if (skObj.contains("placed")) sketch->setPlaced(planeFromJson(skObj.at("placed")));
                if (skObj.contains("id")) {
                    sketch->setId(skObj.at("id").get<uint64_t>());
                }
                if (skObj.contains("name")) {
                    sketch->setName(skObj.at("name").get<std::string>());
                }

                // Load per-sketch entities
                if (skObj.contains("entities")) {
                    const std::string where =
                        "sketch " + std::to_string(thisSketch + 1) + " entity";
                    size_t sketchEntity = 0;
                    for (const auto& eObj : skObj.at("entities")) {
                        const size_t thisOne = sketchEntity++;
                        try {
                            auto entity = deserializeEntity(eObj, blockTablePtr);
                            if (entity) {
                                sketch->addEntity(entity);
                            } else {
                                noteSkipped(report, where, thisOne, eObj,
                                            "not a kind of entity this version reads");
                            }
                        } catch (const std::exception& e) {
                            noteSkipped(report, where, thisOne, eObj, jsonMessage(e));
                            continue;
                        }
                    }
                }

                // Its constraints (saved since Phase 131; before, they were lost).
                if (skObj.contains("constraints")) {
                    constraintsFromJson(skObj.at("constraints"), sketch->drawing(),
                                        sketch->constraintSystem());
                }

                doc.sketches().push_back(sketch);
            } catch (const std::exception& e) {
                noteSkipped(report, "sketch", thisSketch, skObj, jsonMessage(e));
                continue;  // Skip malformed sketches.
            }
        }
    }

    // --- Load feature tree (v15+; v16 stores real sketch IDs) ---
    if (root.contains("featureTree")) {
        doc.featureTree().clear();
        size_t featureIndex = 0;
        for (const auto& fObj : root.at("featureTree")) {
            const size_t thisFeature = featureIndex++;
            const size_t before = doc.featureTree().featureCount();
            try {
                std::string ftype = fObj.value("type", "");
                std::string persistedId = fObj.value("featureID", "");

                // How a created body combines with the part. Files written
                // before operations existed rebuilt every created body on its
                // own, so they load as separate bodies.
                doc::BodyOperation operation = doc::BodyOperation::NewBody;
                if (fObj.contains("bodyOperation")) {
                    const auto parsed =
                        doc::bodyOperationFromName(fObj.at("bodyOperation").get<std::string>());
                    if (!parsed) throw std::invalid_argument("unknown feature operation");
                    operation = *parsed;
                }
                const bool suppressed =
                    fObj.contains("featureSuppressed") && fObj.at("featureSuppressed").get<bool>();
                // A file from before persistent naming made its fillets,
                // chamfers and mates against positional names: keep them.
                const int namingCode = fObj.contains("naming") ? fObj.at("naming").get<int>() : 1;
                if (namingCode != static_cast<int>(model::NamingScheme::Positional) &&
                    namingCode != static_cast<int>(model::NamingScheme::FromGeometry) &&
                    namingCode != static_cast<int>(model::NamingScheme::Stable)) {
                    throw std::invalid_argument("unknown naming scheme");
                }
                const auto naming = static_cast<model::NamingScheme>(namingCode);
                auto addLoaded = [&](std::unique_ptr<doc::Feature> feature) {
                    if (feature->createsNewBody()) feature->setOperation(operation);
                    feature->setSuppressed(suppressed);
                    feature->setNaming(naming);
                    // Phase 155. What is not text is left out: the value
                    // beside it stands.
                    if (const auto expressions = fObj.find("expressions");
                        expressions != fObj.end() && expressions->is_object()) {
                        for (const auto& [name, text] : expressions->items()) {
                            if (text.is_string()) {
                                feature->setParameterExpression(name, text.get<std::string>());
                            }
                        }
                    }
                    doc.featureTree().addFeature(std::move(feature));
                };

                auto findSketch = [&](uint64_t id) -> std::shared_ptr<doc::Sketch> {
                    for (const auto& sk : doc.sketches()) {
                        if (sk->id() == id) return sk;
                    }
                    return nullptr;
                };

                // Multi-sketch features resolve their own references.
                if (ftype == "loft") {
                    std::vector<std::shared_ptr<doc::Sketch>> sections;
                    if (fObj.contains("sketchIds")) {
                        for (const auto& idJson : fObj.at("sketchIds")) {
                            auto sk = findSketch(idJson.get<uint64_t>());
                            if (sk) sections.push_back(sk);
                        }
                    }
                    if (sections.size() < 2) {
                        throw std::invalid_argument("it needs two or more section sketches, and " +
                                                    std::to_string(sections.size()) +
                                                    " could be found");
                    }
                    auto feat = std::make_unique<doc::LoftFeature>(std::move(sections));
                    feat->restoreFeatureID(persistedId);
                    addLoaded(std::move(feat));
                    continue;
                }
                if (ftype == "sweep") {
                    auto profile = fObj.contains("sketchId")
                                       ? findSketch(fObj.at("sketchId").get<uint64_t>())
                                       : nullptr;
                    auto path = fObj.contains("pathSketchId")
                                    ? findSketch(fObj.at("pathSketchId").get<uint64_t>())
                                    : nullptr;
                    if (!profile) throw std::invalid_argument("its profile sketch is missing");
                    if (!path) throw std::invalid_argument("its path sketch is missing");
                    {
                        auto feat = std::make_unique<doc::SweepFeature>(profile, path);
                        if (fObj.contains("segments")) {
                            feat->setParameter("segments", fObj.at("segments").get<double>());
                        }
                        // After the count: setting the count clears the tolerance.
                        if (fObj.contains("chordTolerance")) {
                            feat->setParameter("chordTolerance",
                                               fObj.at("chordTolerance").get<double>());
                        }
                        feat->restoreFeatureID(persistedId);
                        addLoaded(std::move(feat));
                    }
                    continue;
                }

                // Input-consuming features (no sketch reference).
                if (ftype == "draft") {
                    math::Vec3 pullDir(0, 0, 1);
                    math::Vec3 neutralPoint = math::Vec3::Zero;
                    if (fObj.contains("pullDir")) {
                        pullDir = math::Vec3(fObj.at("pullDir").at(0).get<double>(),
                                             fObj.at("pullDir").at(1).get<double>(),
                                             fObj.at("pullDir").at(2).get<double>());
                    }
                    if (fObj.contains("neutralPoint")) {
                        neutralPoint = math::Vec3(fObj.at("neutralPoint").at(0).get<double>(),
                                                  fObj.at("neutralPoint").at(1).get<double>(),
                                                  fObj.at("neutralPoint").at(2).get<double>());
                    }
                    double angle = fObj.value("angle", 0.0);
                    auto feat = std::make_unique<doc::DraftFeature>(pullDir, neutralPoint, angle);
                    feat->restoreFeatureID(persistedId);
                    addLoaded(std::move(feat));
                    continue;
                }
                if (ftype == "shell") {
                    double thickness = fObj.value("thickness", 1.0);
                    std::vector<topo::TopologyID> removed;
                    if (fObj.contains("removedFaces")) {
                        for (const auto& tagJson : fObj.at("removedFaces")) {
                            removed.push_back(
                                topo::TopologyID::fromTag(tagJson.get<std::string>()));
                        }
                    }
                    auto feat = std::make_unique<doc::ShellFeature>(thickness, std::move(removed));
                    feat->restoreFeatureID(persistedId);
                    addLoaded(std::move(feat));
                    continue;
                }
                if (ftype == "fillet") {
                    double radius = fObj.value("radius", 1.0);
                    std::vector<topo::TopologyID> edges;
                    if (fObj.contains("edges")) {
                        for (const auto& tagJson : fObj.at("edges")) {
                            edges.push_back(topo::TopologyID::fromTag(tagJson.get<std::string>()));
                        }
                    }
                    auto feat = std::make_unique<doc::FilletFeature>(std::move(edges), radius);
                    // Absent in files written before the resolution was a
                    // feature property; the feature's own default stands in.
                    if (fObj.contains("arcSegments")) {
                        feat->setParameter("arcSegments", fObj.at("arcSegments").get<double>());
                    }
                    // After the count: setting the count clears the tolerance.
                    if (fObj.contains("chordTolerance")) {
                        feat->setParameter("chordTolerance",
                                           fObj.at("chordTolerance").get<double>());
                    }
                    feat->restoreFeatureID(persistedId);
                    addLoaded(std::move(feat));
                    continue;
                }
                if (ftype == "chamfer") {
                    double distance = fObj.value("distance", 1.0);
                    std::vector<topo::TopologyID> edges;
                    if (fObj.contains("edges")) {
                        for (const auto& tagJson : fObj.at("edges")) {
                            edges.push_back(topo::TopologyID::fromTag(tagJson.get<std::string>()));
                        }
                    }
                    auto feat = std::make_unique<doc::ChamferFeature>(std::move(edges), distance);
                    feat->restoreFeatureID(persistedId);
                    addLoaded(std::move(feat));
                    continue;
                }
                if (ftype == "boolean") {
                    const std::string op = fObj.value("operation", "union");
                    model::BooleanType type = model::BooleanType::Union;
                    if (op == "subtract") {
                        type = model::BooleanType::Subtract;
                    } else if (op == "intersect") {
                        type = model::BooleanType::Intersect;
                    }
                    auto feat = std::make_unique<doc::BooleanFeature>(type);
                    feat->restoreFeatureID(persistedId);
                    addLoaded(std::move(feat));
                    continue;
                }
                if (ftype == "pattern") {
                    auto readVec = [&](const char* key) {
                        math::Vec3 v;
                        if (fObj.contains(key)) {
                            v = math::Vec3(fObj.at(key).at(0).get<double>(),
                                           fObj.at(key).at(1).get<double>(),
                                           fObj.at(key).at(2).get<double>());
                        }
                        return v;
                    };
                    math::Vec3 vecA = readVec("vecA");
                    math::Vec3 vecB = readVec("vecB");
                    double scalar = fObj.value("scalar", 0.0);
                    const int count = clampedCount(fObj, "count", 1, 1, doc::kMaxPatternCount);
                    std::vector<int> suppressed;
                    if (fObj.contains("suppressed")) {
                        for (const json& index : fObj.at("suppressed")) {
                            // An instance index; anything outside the pattern
                            // suppresses nothing, so it is dropped.
                            if (!index.is_number()) continue;
                            const double d = index.get<double>();
                            if (std::isfinite(d) && d >= 0.0 && d < count) {
                                suppressed.push_back(static_cast<int>(d));
                            }
                        }
                    }
                    std::unique_ptr<doc::PatternFeature> feat;
                    if (fObj.value("kind", "linear") == "circular") {
                        feat = doc::PatternFeature::makeCircular(vecA, vecB, scalar, count,
                                                                 std::move(suppressed));
                    } else {
                        feat = doc::PatternFeature::makeLinear(vecA, scalar, count,
                                                               std::move(suppressed));
                    }
                    if (const auto targets = fObj.find("features");
                        targets != fObj.end() && targets->is_array()) {
                        std::vector<std::string> ids;
                        for (const json& id : *targets) {
                            if (id.is_string()) ids.push_back(id.get<std::string>());
                        }
                        feat->setTargets(std::move(ids));
                    }
                    feat->restoreFeatureID(persistedId);
                    addLoaded(std::move(feat));
                    continue;
                }
                if (ftype == "datum") {
                    auto readVec = [&](const char* key) {
                        math::Vec3 v;
                        if (fObj.contains(key)) {
                            v = math::Vec3(fObj.at(key).at(0).get<double>(),
                                           fObj.at(key).at(1).get<double>(),
                                           fObj.at(key).at(2).get<double>());
                        }
                        return v;
                    };
                    math::Vec3 origin = readVec("origin");
                    math::Vec3 dirA = readVec("dirA");
                    math::Vec3 dirB = readVec("dirB");
                    std::string kind = fObj.value("datumKind", "plane");
                    std::unique_ptr<doc::DatumFeature> feat;
                    if (kind == "axis") {
                        feat = doc::DatumFeature::makeAxis(model::DatumAxis{origin, dirA});
                    } else if (kind == "point") {
                        feat = doc::DatumFeature::makePoint(model::DatumPoint{origin});
                    } else {
                        feat = doc::DatumFeature::makePlane(model::DatumPlane{origin, dirA, dirB});
                    }
                    feat->restoreFeatureID(persistedId);
                    addLoaded(std::move(feat));
                    continue;
                }
                if (ftype == "imported") {
                    auto solids = StepFormat::fromString(fObj.at("step").get<std::string>());
                    if (solids.empty()) {
                        throw std::invalid_argument("its body could not be read: " +
                                                    StepFormat::lastError());
                    }
                    auto feat = std::make_unique<doc::ImportedBodyFeature>(
                        std::shared_ptr<const topo::Solid>(std::move(solids.front())),
                        fObj.value("source", ""));
                    feat->restoreFeatureID(persistedId);
                    addLoaded(std::move(feat));
                    continue;
                }
                if (ftype == "primitive") {
                    const double p0 = fObj.value("p0", 1.0);
                    const double p1 = fObj.value("p1", 1.0);
                    const double p2 = fObj.value("p2", 1.0);
                    const std::string kind = fObj.value("primitiveKind", "box");
                    std::unique_ptr<doc::PrimitiveFeature> feat;
                    if (kind == "cylinder") {
                        feat = doc::PrimitiveFeature::makeCylinder(p0, p1);
                    } else if (kind == "sphere") {
                        feat = doc::PrimitiveFeature::makeSphere(p0);
                    } else if (kind == "cone") {
                        feat = doc::PrimitiveFeature::makeCone(p0, p1, p2);
                    } else if (kind == "torus") {
                        feat = doc::PrimitiveFeature::makeTorus(p0, p1);
                    } else {
                        feat = doc::PrimitiveFeature::makeBox(p0, p1, p2);
                    }
                    if (fObj.contains("segments")) {
                        feat->setParameter("segments", fObj.at("segments").get<double>());
                    }
                    // After the count: setting the count clears the tolerance.
                    if (fObj.contains("chordTolerance")) {
                        feat->setParameter("chordTolerance",
                                           fObj.at("chordTolerance").get<double>());
                    }
                    for (const char* key : {"basePoint", "axisDirection"}) {
                        const auto v = fObj.find(key);
                        if (v == fObj.end() || !v->is_array() || v->size() != 3) continue;
                        if (!(*v)[0].is_number() || !(*v)[1].is_number() || !(*v)[2].is_number()) {
                            continue;
                        }
                        feat->setVector(key,
                                        math::Vec3((*v)[0].get<double>(), (*v)[1].get<double>(),
                                                   (*v)[2].get<double>()));
                    }
                    feat->restoreFeatureID(persistedId);
                    addLoaded(std::move(feat));
                    continue;
                }

                // Single-sketch features (extrude/revolve): v16 files reference
                // the sketch by ID; v15 files stored a (buggy) index — fall
                // back to it so old files keep loading.
                std::shared_ptr<doc::Sketch> sketch;
                if (ftype != "extrude" && ftype != "revolve") {
                    throw std::invalid_argument("not a kind of feature this version reads");
                }
                if (fObj.contains("sketchId")) {
                    sketch = findSketch(fObj.at("sketchId").get<uint64_t>());
                } else {
                    const int sketchIndex =
                        static_cast<int>(intField(fObj, "sketchIndex", -1, -1, 1'000'000));
                    if (sketchIndex >= 0 && sketchIndex < static_cast<int>(doc.sketches().size())) {
                        sketch = doc.sketches()[static_cast<size_t>(sketchIndex)];
                    }
                }
                if (!sketch) throw std::invalid_argument("its sketch is missing");

                if (ftype == "extrude") {
                    double distance = fObj.value("distance", 1.0);
                    math::Vec3 direction(0, 0, 1);
                    if (fObj.contains("direction")) {
                        direction = math::Vec3(fObj.at("direction").at(0).get<double>(),
                                               fObj.at("direction").at(1).get<double>(),
                                               fObj.at("direction").at(2).get<double>());
                    }
                    auto feat = std::make_unique<doc::ExtrudeFeature>(sketch, direction, distance);
                    if (fObj.contains("segments")) {
                        feat->setParameter("segments", fObj.at("segments").get<double>());
                    }
                    if (fObj.contains("chordTolerance")) {
                        feat->setParameter("chordTolerance",
                                           fObj.at("chordTolerance").get<double>());
                    }
                    if (const auto extent = fObj.find("extent");
                        extent != fObj.end() && extent->is_number()) {
                        feat->setParameter("extent", extent->get<double>());  // 0-3, else ignored
                    }
                    feat->restoreFeatureID(persistedId);
                    addLoaded(std::move(feat));
                } else if (ftype == "revolve") {
                    double angle = fObj.value("angle", 6.283185307179586);
                    math::Vec3 axisPoint = hz::math::Vec3::Zero;
                    math::Vec3 axisDir = hz::math::Vec3::UnitY;
                    if (fObj.contains("axisPoint")) {
                        axisPoint = math::Vec3(fObj.at("axisPoint").at(0).get<double>(),
                                               fObj.at("axisPoint").at(1).get<double>(),
                                               fObj.at("axisPoint").at(2).get<double>());
                    }
                    if (fObj.contains("axisDir")) {
                        axisDir = math::Vec3(fObj.at("axisDir").at(0).get<double>(),
                                             fObj.at("axisDir").at(1).get<double>(),
                                             fObj.at("axisDir").at(2).get<double>());
                    }
                    auto feat =
                        std::make_unique<doc::RevolveFeature>(sketch, axisPoint, axisDir, angle);
                    if (fObj.contains("segments")) {
                        feat->setParameter("segments", fObj.at("segments").get<double>());
                    }
                    // After the count: setting the count clears the tolerance.
                    if (fObj.contains("chordTolerance")) {
                        feat->setParameter("chordTolerance",
                                           fObj.at("chordTolerance").get<double>());
                    }
                    feat->restoreFeatureID(persistedId);
                    addLoaded(std::move(feat));
                }
            } catch (const std::exception& e) {
                noteSkipped(report, "feature", thisFeature, fObj, jsonMessage(e));
                continue;  // Skip malformed features.
            }
            if (doc.featureTree().featureCount() == before) {
                noteSkipped(report, "feature", thisFeature, fObj,
                            "not a kind of feature this version reads");
            }
        }
    }

    // The rollback point, if it is one of the features read.
    if (const auto rollback = root.find("rollbackIndex");
        rollback != root.end() && rollback->is_number_integer()) {
        const auto index = rollback->get<long long>();
        if (index >= 0 && index < static_cast<long long>(doc.featureTree().featureCount()) - 1) {
            doc.featureTree().setRollbackIndex(static_cast<int>(index));
        }
    }

    // Files saved before Phase 131 carry an empty "Default Sketch" that every
    // document used to have. One nothing uses (the document's list is its
    // only owner: no feature holds it) is dropped, so it does not stand in
    // the feature tree as a sketch nobody made.
    auto& sketches = doc.sketches();
    sketches.erase(std::remove_if(sketches.begin(), sketches.end(),
                                  [](const std::shared_ptr<doc::Sketch>& sketch) {
                                      return sketch->name() == "Default Sketch" &&
                                             sketch->entities().empty() && sketch.use_count() == 1;
                                  }),
                   sketches.end());

    return true;
}

/// loadDocumentRoot, with every failure turned into a reason. A malformed file
/// throws json type and range errors from deep inside the reader; nothing
/// above this may see them, or they reach the Qt event loop and end the
/// session.
/// Refuse a file written by a newer format than this build reads.
static bool checkVersion(const json& root, std::string* error) {
    const json& version = root.at("version");
    if (!version.is_number_integer())
        return fail(error, "the file's format version is not a number");
    const auto v = version.get<long long>();
    if (v > kFormatVersion) {
        return fail(error, "it was written by a newer version of Horizon CAD (file format " +
                               std::to_string(v) + "; this version reads up to " +
                               std::to_string(kFormatVersion) + ")");
    }
    if (v < 1) return fail(error, "the file's format version " + std::to_string(v) + " is invalid");
    return true;
}

static bool loadDocumentChecked(const json& root, doc::Document& doc, std::string* error,
                                ImportReport* report) {
    if (!root.is_object() || !root.contains("version") || !root.contains("entities")) {
        return fail(error, "this is not a Horizon document (it has no version or entity list)");
    }
    if (!checkVersion(root, error)) return false;
    try {
        if (loadDocumentRoot(root, doc, report)) return true;
        return fail(error, "the document could not be read");
    } catch (const std::exception& e) {
        return fail(error, "the file is damaged: " + jsonMessage(e));
    }
}

bool NativeFormat::load(const std::string& filePath, doc::Document& doc, std::string* error,
                        ImportReport* report) {
    std::ifstream file;
    if (!openForReading(filePath, file, error)) return false;

    json root;
    try {
        file >> root;
    } catch (const std::exception& e) {
        return fail(error, "the file is not a readable Horizon document: " + jsonMessage(e));
    }

    return loadDocumentChecked(root, doc, error, report);
}

bool NativeFormat::documentFromJson(const std::string& text, doc::Document& doc, std::string* error,
                                    ImportReport* report) {
    json root;
    try {
        root = json::parse(text);
    } catch (const std::exception& e) {
        return fail(error, "the document is not valid JSON: " + jsonMessage(e));
    }
    return loadDocumentChecked(root, doc, error, report);
}

// ---------------------------------------------------------------------------
// Assembly serialization (.hzasm)
// ---------------------------------------------------------------------------

/// Build the assembly JSON envelope. @p filePath anchors relative component
/// paths (empty → paths stored as-is). Shared by saveAssembly() and
/// BinaryFormat.
static json buildAssemblyRoot(const doc::AssemblyDocument& asmDoc, const std::string& filePath) {
    json root;
    root["version"] = kFormatVersion;
    root["type"] = "hzasm";
    root["units"] = unitsJson(asmDoc.lengthUnit());

    const std::filesystem::path asmDir = std::filesystem::path(filePath).parent_path();

    json componentsArray = json::array();
    for (const auto& comp : asmDoc.components()) {
        json cObj;
        cObj["id"] = comp.id;
        cObj["name"] = comp.name;

        // Store part paths relative to the assembly file when possible so
        // the pair stays valid if the containing directory moves.
        std::filesystem::path partPath(comp.partPath);
        if (partPath.is_absolute() && !asmDir.empty()) {
            std::error_code ec;
            auto rel = std::filesystem::relative(partPath, asmDir, ec);
            if (!ec && !rel.empty()) partPath = rel;
        }
        cObj["partPath"] = partPath.generic_string();

        json transformArray = json::array();
        for (int row = 0; row < 4; ++row) {
            for (int col = 0; col < 4; ++col) {
                transformArray.push_back(comp.transform.at(row, col));
            }
        }
        cObj["transform"] = transformArray;
        cObj["suppressed"] = comp.suppressed;

        componentsArray.push_back(cObj);
    }
    root["components"] = componentsArray;

    // --- Mates (Phase 42) ---
    auto mateTypeToString = [](doc::MateType t) -> const char* {
        switch (t) {
            case doc::MateType::Coincident:
                return "coincident";
            case doc::MateType::Concentric:
                return "concentric";
            case doc::MateType::Distance:
                return "distance";
            case doc::MateType::Angle:
                return "angle";
            case doc::MateType::Parallel:
                return "parallel";
            case doc::MateType::Perpendicular:
                return "perpendicular";
            case doc::MateType::Tangent:
                return "tangent";
            case doc::MateType::Fixed:
                return "fixed";
        }
        return "coincident";
    };

    json matesArray = json::array();
    for (const auto& mate : asmDoc.mates()) {
        json mObj;
        mObj["id"] = mate.id;
        mObj["type"] = mateTypeToString(mate.type);
        mObj["a"] = {{"componentId", mate.a.componentId}, {"faceTag", mate.a.faceId.tag()}};
        mObj["b"] = {{"componentId", mate.b.componentId}, {"faceTag", mate.b.faceId.tag()}};
        mObj["value"] = mate.value;
        matesArray.push_back(mObj);
    }
    root["mates"] = matesArray;

    return root;
}

bool NativeFormat::saveAssembly(const std::string& filePath, const doc::AssemblyDocument& asmDoc,
                                std::string* error) {
    const json root = buildAssemblyRoot(asmDoc, filePath);
    return writeFileAtomically(pathFromUtf8(filePath), dumpJson(root, 2), error);
}

std::string NativeFormat::assemblyToJson(const doc::AssemblyDocument& asmDoc,
                                         const std::string& filePath) {
    return dumpJson(buildAssemblyRoot(asmDoc, filePath), -1);
}

/// Populate an AssemblyDocument from a parsed envelope. @p filePath anchors
/// relative component paths. Shared by loadAssembly() and BinaryFormat.
static bool loadAssemblyRoot(const json& root, doc::AssemblyDocument& asmDoc,
                             const std::string& filePath, ImportReport* report) {
    if (!root.contains("type") || !root.at("type").is_string() ||
        root.at("type").get<std::string>() != "hzasm") {
        return false;
    }

    asmDoc.clear();
    asmDoc.setLengthUnit(unitsFrom(root));

    if (root.contains("components")) {
        size_t componentIndex = 0;
        for (const auto& cObj : root.at("components")) {
            const size_t thisComponent = componentIndex++;
            try {
                doc::ComponentInstance comp;
                comp.id = cObj.value("id", uint64_t{0});
                comp.name = cObj.value("name", "");
                comp.partPath = cObj.value("partPath", "");
                comp.suppressed = cObj.value("suppressed", false);

                // Component paths are stored relative to the assembly file;
                // hold them absolute in memory so the reference stays valid
                // if the assembly is later saved elsewhere (Save As).
                std::filesystem::path p(comp.partPath);
                if (p.is_relative() && !comp.partPath.empty()) {
                    std::filesystem::path asmDir = std::filesystem::path(filePath).parent_path();
                    if (!asmDir.empty()) {
                        comp.partPath = (asmDir / p).lexically_normal().string();
                    }
                }

                if (cObj.contains("transform") && cObj.at("transform").size() == 16) {
                    const auto& t = cObj.at("transform");
                    for (int row = 0; row < 4; ++row) {
                        for (int col = 0; col < 4; ++col) {
                            comp.transform.at(row, col) =
                                t[static_cast<size_t>(row * 4 + col)].get<double>();
                        }
                    }
                }

                asmDoc.addComponent(std::move(comp));
            } catch (const std::exception& e) {
                noteSkipped(report, "component", thisComponent, cObj, jsonMessage(e));
                continue;  // Skip malformed components.
            }
        }
    }

    // --- Mates (Phase 42) ---
    auto mateTypeFromString = [](const std::string& t) {
        if (t == "concentric") return doc::MateType::Concentric;
        if (t == "distance") return doc::MateType::Distance;
        if (t == "angle") return doc::MateType::Angle;
        if (t == "parallel") return doc::MateType::Parallel;
        if (t == "perpendicular") return doc::MateType::Perpendicular;
        if (t == "tangent") return doc::MateType::Tangent;
        if (t == "fixed") return doc::MateType::Fixed;
        return doc::MateType::Coincident;
    };

    if (root.contains("mates")) {
        size_t mateIndex = 0;
        for (const auto& mObj : root.at("mates")) {
            const size_t thisMate = mateIndex++;
            try {
                doc::Mate mate;
                mate.id = mObj.value("id", uint64_t{0});
                mate.type = mateTypeFromString(mObj.value("type", "coincident"));
                mate.value = mObj.value("value", 0.0);
                if (mObj.contains("a")) {
                    mate.a.componentId = mObj.at("a").value("componentId", uint64_t{0});
                    mate.a.faceId = topo::TopologyID::fromTag(mObj.at("a").value("faceTag", ""));
                }
                if (mObj.contains("b")) {
                    mate.b.componentId = mObj.at("b").value("componentId", uint64_t{0});
                    mate.b.faceId = topo::TopologyID::fromTag(mObj.at("b").value("faceTag", ""));
                }
                asmDoc.addMate(std::move(mate));
            } catch (const std::exception& e) {
                noteSkipped(report, "mate", thisMate, mObj, jsonMessage(e));
                continue;  // Skip malformed mates.
            }
        }
    }

    asmDoc.setDirty(false);
    return true;
}

/// loadAssemblyRoot with every failure turned into a reason; see
/// loadDocumentChecked.
static bool loadAssemblyChecked(const json& root, doc::AssemblyDocument& asmDoc,
                                const std::string& filePath, std::string* error,
                                ImportReport* report) {
    if (!root.is_object()) return fail(error, "this is not a Horizon assembly");
    if (root.contains("version") && !checkVersion(root, error)) return false;
    try {
        if (loadAssemblyRoot(root, asmDoc, filePath, report)) return true;
        return fail(error, "this is not a Horizon assembly (its type is not \"hzasm\")");
    } catch (const std::exception& e) {
        return fail(error, "the file is damaged: " + jsonMessage(e));
    }
}

bool NativeFormat::loadAssembly(const std::string& filePath, doc::AssemblyDocument& asmDoc,
                                std::string* error, ImportReport* report) {
    std::ifstream file;
    if (!openForReading(filePath, file, error)) return false;

    json root;
    try {
        file >> root;
    } catch (const std::exception& e) {
        return fail(error, "the file is not a readable Horizon assembly: " + jsonMessage(e));
    }

    return loadAssemblyChecked(root, asmDoc, filePath, error, report);
}

bool NativeFormat::assemblyFromJson(const std::string& text, doc::AssemblyDocument& asmDoc,
                                    const std::string& filePath, std::string* error,
                                    ImportReport* report) {
    json root;
    try {
        root = json::parse(text);
    } catch (const std::exception& e) {
        return fail(error, "the assembly is not valid JSON: " + jsonMessage(e));
    }
    return loadAssemblyChecked(root, asmDoc, filePath, error, report);
}

// ---------------------------------------------------------------------------
// Lightweight tessellation-cache read
// ---------------------------------------------------------------------------

namespace {

/// The cache's faces and edges by name, when it has them and they hold
/// together; a mesh without them otherwise (drawn, but not clickable).
void readCachedFaces(const json& cache, geo::MeshData& mesh) {
    if (!cache.contains("faceTags") || !cache.contains("triangleFaces")) return;
    const auto& tags = cache.at("faceTags");
    const auto& faces = cache.at("triangleFaces");
    if (!tags.is_array() || !faces.is_array() || faces.size() != mesh.indices.size() / 3) return;
    std::vector<std::string> faceTags;
    faceTags.reserve(tags.size());
    for (const auto& tag : tags) {
        if (!tag.is_string()) return;
        faceTags.push_back(tag.get<std::string>());
    }
    std::vector<uint32_t> triangleFaces;
    triangleFaces.reserve(faces.size());
    for (const auto& face : faces) {
        if (!face.is_number_unsigned() || face.get<uint64_t>() >= faceTags.size()) return;
        triangleFaces.push_back(face.get<uint32_t>());
    }
    std::vector<geo::MeshData::Edge> edges;
    if (cache.contains("edges") && cache.at("edges").is_array()) {
        for (const auto& edge : cache.at("edges")) {
            if (!edge.is_object() || !edge.contains("tag") || !edge.at("tag").is_string() ||
                !edge.contains("points") || !edge.at("points").is_array()) {
                continue;
            }
            geo::MeshData::Edge e;
            e.tag = edge.at("tag").get<std::string>();
            for (const auto& v : edge.at("points")) {
                if (!v.is_number()) break;
                e.points.push_back(v.get<float>());
            }
            if (e.points.size() >= 6 && e.points.size() % 3 == 0) edges.push_back(std::move(e));
        }
    }
    mesh.faceTags = std::move(faceTags);
    mesh.triangleFaces = std::move(triangleFaces);
    mesh.edges = std::move(edges);
}

}  // namespace

std::shared_ptr<geo::MeshData> NativeFormat::loadPartMesh(const std::string& filePath) {
    std::ifstream file(pathFromUtf8(filePath));
    if (!file.is_open()) return nullptr;

    json root;
    try {
        file >> root;
    } catch (...) {
        return nullptr;
    }

    if (!root.contains("tessellationCache")) return nullptr;

    try {
        const auto& cache = root.at("tessellationCache");
        auto mesh = std::make_shared<geo::MeshData>();
        mesh->positions = cache.value("positions", std::vector<float>{});
        mesh->normals = cache.value("normals", std::vector<float>{});

        // Read indices as signed 64-bit first so negative or oversized
        // values are caught instead of silently wrapping.
        std::vector<int64_t> rawIndices = cache.value("indices", std::vector<int64_t>{});
        if (mesh->positions.empty() || rawIndices.empty()) return nullptr;
        if (mesh->positions.size() % 3 != 0 || rawIndices.size() % 3 != 0) return nullptr;
        if (!mesh->normals.empty() && mesh->normals.size() != mesh->positions.size()) {
            return nullptr;
        }

        const int64_t vertexCount = static_cast<int64_t>(mesh->positions.size() / 3);
        mesh->indices.reserve(rawIndices.size());
        for (int64_t index : rawIndices) {
            if (index < 0 || index >= vertexCount) return nullptr;
            mesh->indices.push_back(static_cast<uint32_t>(index));
        }
        readCachedFaces(cache, *mesh);
        return mesh;
    } catch (const std::exception&) {
        return nullptr;
    }
}

}  // namespace hz::io
