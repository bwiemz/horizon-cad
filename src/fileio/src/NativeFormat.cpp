#include "horizon/fileio/NativeFormat.h"

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <set>
#include <system_error>

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
#include "horizon/drafting/SketchPlane.h"
#include "horizon/modeling/SolidTessellator.h"

using json = nlohmann::json;

namespace hz::io {

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
static json buildDocumentRoot(const doc::Document& doc, bool includeTessellation) {
    json root;
    root["version"] = 16;
    root["type"] = doc.type() == doc::DocumentType::Part ? "hzpart" : "hcad";

    // --- Dimension style ---
    const auto& ds = doc.draftDocument().dimensionStyle();
    root["dimensionStyle"] = {{"textHeight", ds.textHeight},
                              {"arrowSize", ds.arrowSize},
                              {"arrowAngle", ds.arrowAngle},
                              {"extensionGap", ds.extensionGap},
                              {"extensionOvershoot", ds.extensionOvershoot},
                              {"precision", ds.precision},
                              {"showUnits", ds.showUnits}};

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
    json constraintsArray = json::array();
    for (const auto& c : doc.constraintSystem().constraints()) {
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
    root["constraints"] = constraintsArray;

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

    // --- Sketches ---
    json sketchesArray = json::array();
    for (const auto& sketch : doc.sketches()) {
        json skObj;
        skObj["id"] = sketch->id();
        skObj["name"] = sketch->name();

        const auto& plane = sketch->plane();
        skObj["plane"] = {{"origin", {plane.origin().x, plane.origin().y, plane.origin().z}},
                          {"normal", {plane.normal().x, plane.normal().y, plane.normal().z}},
                          {"xAxis", {plane.xAxis().x, plane.xAxis().y, plane.xAxis().z}}};

        json skEntities = json::array();
        for (const auto& entity : sketch->entities()) {
            skEntities.push_back(serializeEntity(*entity));
        }
        skObj["entities"] = skEntities;

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
        } else if (const auto* rev = dynamic_cast<const doc::RevolveFeature*>(feat)) {
            fObj["type"] = "revolve";
            fObj["angle"] = rev->angle();
            fObj["axisPoint"] = {rev->axisPoint().x, rev->axisPoint().y, rev->axisPoint().z};
            fObj["axisDir"] = {rev->axisDir().x, rev->axisDir().y, rev->axisDir().z};
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
            fObj["suppressed"] = pat->suppressed();
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
        }

        featureTreeArray.push_back(fObj);
    }
    root["featureTree"] = featureTreeArray;

    // --- Tessellation cache (v16+, parts only) ---
    // Enables lightweight assembly loading: readers can display the part
    // without replaying the feature tree.
    if (includeTessellation && doc.solid()) {
        render::MeshData mesh = model::SolidTessellator::tessellate(*doc.solid());
        json cache;
        cache["positions"] = mesh.positions;
        cache["normals"] = mesh.normals;
        cache["indices"] = mesh.indices;
        root["tessellationCache"] = cache;
    }

    return root;
}

bool NativeFormat::save(const std::string& filePath, const doc::Document& doc) {
    const json root = buildDocumentRoot(doc, /*includeTessellation=*/true);

    std::ofstream file(filePath);
    if (!file.is_open()) return false;
    // Pretty-print drawings for diff-friendliness, but write compactly when a
    // tessellation cache is embedded — indented output puts one mesh number
    // per line and inflates part files by orders of magnitude.
    if (root.contains("tessellationCache")) {
        file << root.dump();
    } else {
        file << root.dump(2);
    }
    return file.good();
}

std::string NativeFormat::documentToJson(const doc::Document& doc, bool includeTessellation) {
    return buildDocumentRoot(doc, includeTessellation).dump();
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
        double sx = obj["start"]["x"].get<double>();
        double sy = obj["start"]["y"].get<double>();
        double ex = obj["end"]["x"].get<double>();
        double ey = obj["end"]["y"].get<double>();
        entity = std::make_shared<draft::DraftLine>(math::Vec2(sx, sy), math::Vec2(ex, ey));
    } else if (type == "circle") {
        double cx = obj["center"]["x"].get<double>();
        double cy = obj["center"]["y"].get<double>();
        double r = obj["radius"].get<double>();
        entity = std::make_shared<draft::DraftCircle>(math::Vec2(cx, cy), r);
    } else if (type == "arc") {
        double cx = obj["center"]["x"].get<double>();
        double cy = obj["center"]["y"].get<double>();
        double r = obj["radius"].get<double>();
        double sa = obj["startAngle"].get<double>();
        double ea = obj["endAngle"].get<double>();
        entity = std::make_shared<draft::DraftArc>(math::Vec2(cx, cy), r, sa, ea);
    } else if (type == "rectangle") {
        double c1x = obj["corner1"]["x"].get<double>();
        double c1y = obj["corner1"]["y"].get<double>();
        double c2x = obj["corner2"]["x"].get<double>();
        double c2y = obj["corner2"]["y"].get<double>();
        entity =
            std::make_shared<draft::DraftRectangle>(math::Vec2(c1x, c1y), math::Vec2(c2x, c2y));
    } else if (type == "polyline") {
        bool closed = obj.value("closed", false);
        std::vector<math::Vec2> points;
        for (const auto& pt : obj["points"]) {
            points.emplace_back(pt["x"].get<double>(), pt["y"].get<double>());
        }
        entity = std::make_shared<draft::DraftPolyline>(points, closed);
    } else if (type == "linearDimension") {
        auto p1 =
            math::Vec2(obj["defPoint1"]["x"].get<double>(), obj["defPoint1"]["y"].get<double>());
        auto p2 =
            math::Vec2(obj["defPoint2"]["x"].get<double>(), obj["defPoint2"]["y"].get<double>());
        auto dp = math::Vec2(obj["dimLinePoint"]["x"].get<double>(),
                             obj["dimLinePoint"]["y"].get<double>());
        auto orient =
            static_cast<draft::DraftLinearDimension::Orientation>(obj.value("orientation", 0));
        auto dim = std::make_shared<draft::DraftLinearDimension>(p1, p2, dp, orient);
        if (obj.contains("textOverride"))
            dim->setTextOverride(obj["textOverride"].get<std::string>());
        entity = dim;
    } else if (type == "radialDimension") {
        auto center =
            math::Vec2(obj["center"]["x"].get<double>(), obj["center"]["y"].get<double>());
        double radius = obj["radius"].get<double>();
        auto textPt =
            math::Vec2(obj["textPoint"]["x"].get<double>(), obj["textPoint"]["y"].get<double>());
        bool isDiam = obj.value("isDiameter", false);
        auto dim = std::make_shared<draft::DraftRadialDimension>(center, radius, textPt, isDiam);
        if (obj.contains("textOverride"))
            dim->setTextOverride(obj["textOverride"].get<std::string>());
        entity = dim;
    } else if (type == "angularDimension") {
        auto vertex =
            math::Vec2(obj["vertex"]["x"].get<double>(), obj["vertex"]["y"].get<double>());
        auto l1 =
            math::Vec2(obj["line1Point"]["x"].get<double>(), obj["line1Point"]["y"].get<double>());
        auto l2 =
            math::Vec2(obj["line2Point"]["x"].get<double>(), obj["line2Point"]["y"].get<double>());
        double arcR = obj["arcRadius"].get<double>();
        auto dim = std::make_shared<draft::DraftAngularDimension>(vertex, l1, l2, arcR);
        if (obj.contains("textOverride"))
            dim->setTextOverride(obj["textOverride"].get<std::string>());
        entity = dim;
    } else if (type == "leader") {
        std::vector<math::Vec2> points;
        for (const auto& pt : obj["points"]) {
            points.emplace_back(pt["x"].get<double>(), pt["y"].get<double>());
        }
        std::string text = obj.value("text", "");
        auto ldr = std::make_shared<draft::DraftLeader>(points, text);
        if (obj.contains("textOverride"))
            ldr->setTextOverride(obj["textOverride"].get<std::string>());
        entity = ldr;
    } else if (type == "blockRef") {
        if (blockTable) {
            std::string blockName = obj.value("blockName", "");
            auto def = blockTable->findBlock(blockName);
            if (def) {
                auto pos = math::Vec2(obj["insertPos"]["x"].get<double>(),
                                      obj["insertPos"]["y"].get<double>());
                double rot = obj.value("rotation", 0.0);
                double scl = obj.value("scale", 1.0);
                entity = std::make_shared<draft::DraftBlockRef>(def, pos, rot, scl);
            }
        }
    } else if (type == "text") {
        auto pos =
            math::Vec2(obj["position"]["x"].get<double>(), obj["position"]["y"].get<double>());
        std::string text = obj.value("text", "");
        double textHeight = obj.value("textHeight", 2.5);
        auto txt = std::make_shared<draft::DraftText>(pos, text, textHeight);
        if (obj.contains("rotation")) txt->setRotation(obj["rotation"].get<double>());
        if (obj.contains("alignment"))
            txt->setAlignment(static_cast<draft::TextAlignment>(obj["alignment"].get<int>()));
        entity = txt;
    } else if (type == "spline") {
        bool closed = obj.value("closed", false);
        std::vector<math::Vec2> controlPoints;
        for (const auto& cp : obj["controlPoints"]) {
            controlPoints.emplace_back(cp["x"].get<double>(), cp["y"].get<double>());
        }
        auto splineEnt = std::make_shared<draft::DraftSpline>(controlPoints, closed);
        if (obj.contains("weights")) {
            std::vector<double> wts;
            wts.reserve(obj["weights"].size());
            for (const auto& w : obj["weights"]) wts.push_back(w.get<double>());
            splineEnt->setWeights(wts);
        }
        entity = splineEnt;
    } else if (type == "hatch") {
        std::vector<math::Vec2> boundary;
        for (const auto& pt : obj["boundary"]) {
            boundary.emplace_back(pt["x"].get<double>(), pt["y"].get<double>());
        }
        auto hatchPattern = static_cast<draft::HatchPattern>(obj.value("pattern", 1));
        double hatchAngle = obj.value("angle", 0.0);
        double hatchSpacing = obj.value("spacing", 1.0);
        entity =
            std::make_shared<draft::DraftHatch>(boundary, hatchPattern, hatchAngle, hatchSpacing);
    } else if (type == "ellipse") {
        auto center =
            math::Vec2(obj["center"]["x"].get<double>(), obj["center"]["y"].get<double>());
        double semiMajor = obj.value("semiMajor", 1.0);
        double semiMinor = obj.value("semiMinor", 1.0);
        double rot = obj.value("rotation", 0.0);
        entity = std::make_shared<draft::DraftEllipse>(center, semiMajor, semiMinor, rot);
    }

    if (entity) {
        if (obj.contains("id")) {
            uint64_t savedId = obj["id"].get<uint64_t>();
            entity->setId(savedId);
            draft::DraftEntity::advanceIdCounter(savedId);
        }
        entity->setLayer(layer);
        entity->setColor(color);
        entity->setLineWidth(lineWidth);
        entity->setLineType(obj.value("lineType", 0));
        uint64_t gid = obj.value("groupId", uint64_t(0));
        entity->setGroupId(gid);
    }

    return entity;
}

/// Populate a Document from a parsed envelope. Shared by load() and
/// BinaryFormat.
static bool loadDocumentRoot(const json& root, doc::Document& doc) {
    if (!root.contains("version") || !root.contains("entities")) return false;

    doc.draftDocument().clear();
    doc.layerManager().clear();
    doc.constraintSystem().clear();
    doc.parameterRegistry().clear();

    // --- Document type (v16+; earlier files are all drawings) ---
    // Defensive read: a malformed (non-string) "type" must not throw.
    std::string typeTag = "hcad";
    if (root.contains("type") && root["type"].is_string()) {
        typeTag = root["type"].get<std::string>();
    }
    doc.setType(typeTag == "hzpart" ? doc::DocumentType::Part : doc::DocumentType::Drawing);

    // --- Load dimension style (v4+) ---
    if (root.contains("dimensionStyle")) {
        const auto& dsObj = root["dimensionStyle"];
        draft::DimensionStyle ds;
        ds.textHeight = dsObj.value("textHeight", 2.5);
        ds.arrowSize = dsObj.value("arrowSize", 1.5);
        ds.arrowAngle = dsObj.value("arrowAngle", 0.3);
        ds.extensionGap = dsObj.value("extensionGap", 0.5);
        ds.extensionOvershoot = dsObj.value("extensionOvershoot", 1.0);
        ds.precision = dsObj.value("precision", 2);
        ds.showUnits = dsObj.value("showUnits", false);
        doc.draftDocument().setDimensionStyle(ds);
    }

    // --- Load layer table (v3+) ---
    if (root.contains("layers")) {
        for (const auto& layerObj : root["layers"]) {
            draft::LayerProperties props;
            props.name = layerObj.value("name", "0");
            props.color = layerObj.value("color", 0xFFFFFFFFu);
            props.lineWidth = layerObj.value("lineWidth", 1.0);
            props.visible = layerObj.value("visible", true);
            props.locked = layerObj.value("locked", false);
            props.lineType = layerObj.value("lineType", 1);
            if (props.name == "0") {
                // Update default layer properties instead of adding duplicate.
                auto* defaultLayer = doc.layerManager().getLayer("0");
                if (defaultLayer) *defaultLayer = props;
            } else {
                doc.layerManager().addLayer(props);
            }
        }
        if (root.contains("currentLayer")) {
            doc.layerManager().setCurrentLayer(root["currentLayer"].get<std::string>());
        }
    }

    // --- Load block definitions (v6+) ---
    if (root.contains("blocks")) {
        for (const auto& blockObj : root["blocks"]) {
            try {
                auto def = std::make_shared<draft::BlockDefinition>();
                def->name = blockObj.value("name", "");
                def->basePoint = math::Vec2(blockObj["basePoint"]["x"].get<double>(),
                                            blockObj["basePoint"]["y"].get<double>());
                if (blockObj.contains("entities")) {
                    for (const auto& se : blockObj["entities"]) {
                        std::string stype = se.value("type", "");
                        std::shared_ptr<draft::DraftEntity> subEnt;
                        if (stype == "line") {
                            subEnt = std::make_shared<draft::DraftLine>(
                                math::Vec2(se["start"]["x"].get<double>(),
                                           se["start"]["y"].get<double>()),
                                math::Vec2(se["end"]["x"].get<double>(),
                                           se["end"]["y"].get<double>()));
                        } else if (stype == "circle") {
                            subEnt = std::make_shared<draft::DraftCircle>(
                                math::Vec2(se["center"]["x"].get<double>(),
                                           se["center"]["y"].get<double>()),
                                se["radius"].get<double>());
                        } else if (stype == "arc") {
                            subEnt = std::make_shared<draft::DraftArc>(
                                math::Vec2(se["center"]["x"].get<double>(),
                                           se["center"]["y"].get<double>()),
                                se["radius"].get<double>(), se["startAngle"].get<double>(),
                                se["endAngle"].get<double>());
                        } else if (stype == "rectangle") {
                            subEnt = std::make_shared<draft::DraftRectangle>(
                                math::Vec2(se["corner1"]["x"].get<double>(),
                                           se["corner1"]["y"].get<double>()),
                                math::Vec2(se["corner2"]["x"].get<double>(),
                                           se["corner2"]["y"].get<double>()));
                        } else if (stype == "polyline") {
                            std::vector<math::Vec2> pts;
                            for (const auto& pt : se["points"])
                                pts.emplace_back(pt["x"].get<double>(), pt["y"].get<double>());
                            subEnt = std::make_shared<draft::DraftPolyline>(
                                pts, se.value("closed", false));
                        } else if (stype == "spline") {
                            std::vector<math::Vec2> cps;
                            for (const auto& cp : se["controlPoints"])
                                cps.emplace_back(cp["x"].get<double>(), cp["y"].get<double>());
                            auto blkSp = std::make_shared<draft::DraftSpline>(
                                cps, se.value("closed", false));
                            if (se.contains("weights")) {
                                std::vector<double> wts;
                                wts.reserve(se["weights"].size());
                                for (const auto& w : se["weights"]) wts.push_back(w.get<double>());
                                blkSp->setWeights(wts);
                            }
                            subEnt = blkSp;
                        } else if (stype == "text") {
                            auto pos = math::Vec2(se["position"]["x"].get<double>(),
                                                  se["position"]["y"].get<double>());
                            auto txt = std::make_shared<draft::DraftText>(
                                pos, se.value("text", ""), se.value("textHeight", 2.5));
                            if (se.contains("rotation"))
                                txt->setRotation(se["rotation"].get<double>());
                            if (se.contains("alignment"))
                                txt->setAlignment(
                                    static_cast<draft::TextAlignment>(se["alignment"].get<int>()));
                            subEnt = txt;
                        } else if (stype == "hatch") {
                            std::vector<math::Vec2> boundary;
                            for (const auto& pt : se["boundary"])
                                boundary.emplace_back(pt["x"].get<double>(), pt["y"].get<double>());
                            subEnt = std::make_shared<draft::DraftHatch>(
                                boundary, static_cast<draft::HatchPattern>(se.value("pattern", 1)),
                                se.value("angle", 0.0), se.value("spacing", 1.0));
                        } else if (stype == "ellipse") {
                            auto ctr = math::Vec2(se["center"]["x"].get<double>(),
                                                  se["center"]["y"].get<double>());
                            subEnt = std::make_shared<draft::DraftEllipse>(
                                ctr, se.value("semiMajor", 1.0), se.value("semiMinor", 1.0),
                                se.value("rotation", 0.0));
                        }
                        if (subEnt) {
                            subEnt->setLayer(se.value("layer", "0"));
                            subEnt->setColor(se.value("color", 0u));
                            subEnt->setLineWidth(se.value("lineWidth", 0.0));
                            subEnt->setLineType(se.value("lineType", 0));
                            def->entities.push_back(subEnt);
                        }
                    }
                }
                doc.draftDocument().blockTable().addBlock(def);
            } catch (const nlohmann::json::exception&) {
                continue;  // Skip malformed block definitions.
            }
        }
    }

    // --- Load entities ---
    const auto* blockTablePtr = &doc.draftDocument().blockTable();
    for (const auto& obj : root["entities"]) {
        try {
            auto entity = deserializeEntity(obj, blockTablePtr);
            if (entity) {
                uint64_t gid = entity->groupId();
                if (gid != 0) {
                    doc.draftDocument().advanceGroupIdCounter(gid);
                }
                doc.draftDocument().addEntity(entity);
            }
        } catch (const nlohmann::json::exception&) {
            continue;  // Skip malformed entities.
        }
    }

    // --- Load constraints (v5+) ---
    if (root.contains("constraints")) {
        for (const auto& cObj : root["constraints"]) {
            std::string ctype = cObj.value("type", "");
            std::shared_ptr<cstr::Constraint> constraint;

            if (ctype == "coincident") {
                constraint = std::make_shared<cstr::CoincidentConstraint>(
                    deserializeRef(cObj["refA"]), deserializeRef(cObj["refB"]));
            } else if (ctype == "horizontal") {
                constraint = std::make_shared<cstr::HorizontalConstraint>(
                    deserializeRef(cObj["refA"]), deserializeRef(cObj["refB"]));
            } else if (ctype == "vertical") {
                constraint = std::make_shared<cstr::VerticalConstraint>(
                    deserializeRef(cObj["refA"]), deserializeRef(cObj["refB"]));
            } else if (ctype == "perpendicular") {
                constraint = std::make_shared<cstr::PerpendicularConstraint>(
                    deserializeRef(cObj["refA"]), deserializeRef(cObj["refB"]));
            } else if (ctype == "parallel") {
                constraint = std::make_shared<cstr::ParallelConstraint>(
                    deserializeRef(cObj["refA"]), deserializeRef(cObj["refB"]));
            } else if (ctype == "tangent") {
                constraint = std::make_shared<cstr::TangentConstraint>(
                    deserializeRef(cObj["refA"]), deserializeRef(cObj["refB"]));
            } else if (ctype == "equal") {
                constraint = std::make_shared<cstr::EqualConstraint>(deserializeRef(cObj["refA"]),
                                                                     deserializeRef(cObj["refB"]));
            } else if (ctype == "fixed") {
                auto pos = math::Vec2(cObj["position"]["x"].get<double>(),
                                      cObj["position"]["y"].get<double>());
                constraint =
                    std::make_shared<cstr::FixedConstraint>(deserializeRef(cObj["ref"]), pos);
            } else if (ctype == "distance") {
                double val = cObj.value("value", 0.0);
                constraint = std::make_shared<cstr::DistanceConstraint>(
                    deserializeRef(cObj["refA"]), deserializeRef(cObj["refB"]), val);
            } else if (ctype == "angle") {
                double val = cObj.value("value", 0.0);
                constraint = std::make_shared<cstr::AngleConstraint>(
                    deserializeRef(cObj["refA"]), deserializeRef(cObj["refB"]), val);
            }

            if (constraint) {
                // Restore the original constraint ID from the file.
                if (cObj.contains("id")) {
                    uint64_t savedId = cObj["id"].get<uint64_t>();
                    constraint->setId(savedId);
                    cstr::Constraint::advanceIdCounter(savedId);
                }
                // Variable reference (v13+)
                if (cObj.contains("variableName")) {
                    constraint->setVariableReference(cObj["variableName"].get<std::string>());
                }
                doc.constraintSystem().addConstraint(constraint);
            }
        }

        // Validate constraint entity references — remove any that reference
        // non-existent entities (corrupted or manually-edited files).
        std::set<uint64_t> entityIds;
        for (const auto& e : doc.draftDocument().entities()) {
            entityIds.insert(e->id());
        }
        std::vector<uint64_t> invalidConstraints;
        for (const auto& c : doc.constraintSystem().constraints()) {
            for (uint64_t eid : c->referencedEntityIds()) {
                if (entityIds.find(eid) == entityIds.end()) {
                    invalidConstraints.push_back(c->id());
                    break;
                }
            }
        }
        for (uint64_t cid : invalidConstraints) {
            doc.constraintSystem().removeConstraint(cid);
        }
    }

    // --- Load design variables (v13+, v14 nested format) ---
    if (root.contains("designVariables")) {
        auto& pReg = doc.parameterRegistry();
        for (const auto& [name, value] : root["designVariables"].items()) {
            if (value.is_number()) {
                // v13 flat format: "width": 50.0
                pReg.set(name, value.get<double>());
            } else if (value.is_object()) {
                // v14 nested format: "width": {"value": 50.0, "expression": "..."}
                if (value.contains("expression") && value["expression"].is_string()) {
                    pReg.setExpression(name, value["expression"].get<std::string>());
                } else if (value.contains("value") && value["value"].is_number()) {
                    pReg.set(name, value["value"].get<double>());
                }
            }
        }
    }

    // Rebuild spatial index after loading all entities.
    doc.draftDocument().rebuildSpatialIndex();

    // --- Load sketches ---
    if (root.contains("sketches")) {
        // Clear the default sketch collection — we'll rebuild from file data.
        doc.sketches().clear();

        for (const auto& skObj : root["sketches"]) {
            try {
                // Reconstruct SketchPlane
                draft::SketchPlane plane;  // default XY
                if (skObj.contains("plane")) {
                    const auto& pObj = skObj["plane"];
                    math::Vec3 origin(pObj["origin"][0].get<double>(),
                                      pObj["origin"][1].get<double>(),
                                      pObj["origin"][2].get<double>());
                    math::Vec3 normal(pObj["normal"][0].get<double>(),
                                      pObj["normal"][1].get<double>(),
                                      pObj["normal"][2].get<double>());
                    math::Vec3 xAxis(pObj["xAxis"][0].get<double>(), pObj["xAxis"][1].get<double>(),
                                     pObj["xAxis"][2].get<double>());
                    plane = draft::SketchPlane(origin, normal, xAxis);
                }

                auto sketch = std::make_shared<doc::Sketch>(plane);
                if (skObj.contains("id")) {
                    sketch->setId(skObj["id"].get<uint64_t>());
                }
                if (skObj.contains("name")) {
                    sketch->setName(skObj["name"].get<std::string>());
                }

                // Load per-sketch entities
                if (skObj.contains("entities")) {
                    for (const auto& eObj : skObj["entities"]) {
                        try {
                            auto entity = deserializeEntity(eObj, blockTablePtr);
                            if (entity) {
                                sketch->addEntity(entity);
                            }
                        } catch (const nlohmann::json::exception&) {
                            continue;
                        }
                    }
                }

                doc.sketches().push_back(sketch);
            } catch (const nlohmann::json::exception&) {
                continue;  // Skip malformed sketches.
            }
        }

        // Ensure a default sketch pointer is valid: pick the first sketch if named
        // "Default Sketch", otherwise create one.
        bool hasDefault = false;
        for (const auto& sk : doc.sketches()) {
            if (sk->name() == "Default Sketch") {
                hasDefault = true;
                break;
            }
        }
        if (!hasDefault) {
            auto defSk = std::make_shared<doc::Sketch>();
            defSk->setName("Default Sketch");
            doc.sketches().insert(doc.sketches().begin(), defSk);
        }
    } else {
        // Pre-sketch file (v14 and earlier): load top-level entities into the
        // default sketch as well so that defaultSketch() contains them.
        for (const auto& entity : doc.draftDocument().entities()) {
            doc.defaultSketch().addEntity(entity);
        }
    }

    // --- Load feature tree (v15+; v16 stores real sketch IDs) ---
    if (root.contains("featureTree")) {
        doc.featureTree().clear();
        for (const auto& fObj : root["featureTree"]) {
            try {
                std::string ftype = fObj.value("type", "");
                std::string persistedId = fObj.value("featureID", "");

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
                        for (const auto& idJson : fObj["sketchIds"]) {
                            auto sk = findSketch(idJson.get<uint64_t>());
                            if (sk) sections.push_back(sk);
                        }
                    }
                    if (sections.size() >= 2) {
                        auto feat = std::make_unique<doc::LoftFeature>(std::move(sections));
                        feat->restoreFeatureID(persistedId);
                        doc.featureTree().addFeature(std::move(feat));
                    }
                    continue;
                }
                if (ftype == "sweep") {
                    auto profile = fObj.contains("sketchId")
                                       ? findSketch(fObj["sketchId"].get<uint64_t>())
                                       : nullptr;
                    auto path = fObj.contains("pathSketchId")
                                    ? findSketch(fObj["pathSketchId"].get<uint64_t>())
                                    : nullptr;
                    if (profile && path) {
                        auto feat = std::make_unique<doc::SweepFeature>(profile, path);
                        feat->restoreFeatureID(persistedId);
                        doc.featureTree().addFeature(std::move(feat));
                    }
                    continue;
                }

                // Input-consuming features (no sketch reference).
                if (ftype == "draft") {
                    math::Vec3 pullDir(0, 0, 1);
                    math::Vec3 neutralPoint = math::Vec3::Zero;
                    if (fObj.contains("pullDir")) {
                        pullDir = math::Vec3(fObj["pullDir"][0].get<double>(),
                                             fObj["pullDir"][1].get<double>(),
                                             fObj["pullDir"][2].get<double>());
                    }
                    if (fObj.contains("neutralPoint")) {
                        neutralPoint = math::Vec3(fObj["neutralPoint"][0].get<double>(),
                                                  fObj["neutralPoint"][1].get<double>(),
                                                  fObj["neutralPoint"][2].get<double>());
                    }
                    double angle = fObj.value("angle", 0.0);
                    auto feat = std::make_unique<doc::DraftFeature>(pullDir, neutralPoint, angle);
                    feat->restoreFeatureID(persistedId);
                    doc.featureTree().addFeature(std::move(feat));
                    continue;
                }
                if (ftype == "shell") {
                    double thickness = fObj.value("thickness", 1.0);
                    std::vector<topo::TopologyID> removed;
                    if (fObj.contains("removedFaces")) {
                        for (const auto& tagJson : fObj["removedFaces"]) {
                            removed.push_back(
                                topo::TopologyID::fromTag(tagJson.get<std::string>()));
                        }
                    }
                    auto feat = std::make_unique<doc::ShellFeature>(thickness, std::move(removed));
                    feat->restoreFeatureID(persistedId);
                    doc.featureTree().addFeature(std::move(feat));
                    continue;
                }
                if (ftype == "fillet") {
                    double radius = fObj.value("radius", 1.0);
                    std::vector<topo::TopologyID> edges;
                    if (fObj.contains("edges")) {
                        for (const auto& tagJson : fObj["edges"]) {
                            edges.push_back(topo::TopologyID::fromTag(tagJson.get<std::string>()));
                        }
                    }
                    auto feat = std::make_unique<doc::FilletFeature>(std::move(edges), radius);
                    feat->restoreFeatureID(persistedId);
                    doc.featureTree().addFeature(std::move(feat));
                    continue;
                }
                if (ftype == "chamfer") {
                    double distance = fObj.value("distance", 1.0);
                    std::vector<topo::TopologyID> edges;
                    if (fObj.contains("edges")) {
                        for (const auto& tagJson : fObj["edges"]) {
                            edges.push_back(topo::TopologyID::fromTag(tagJson.get<std::string>()));
                        }
                    }
                    auto feat = std::make_unique<doc::ChamferFeature>(std::move(edges), distance);
                    feat->restoreFeatureID(persistedId);
                    doc.featureTree().addFeature(std::move(feat));
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
                    doc.featureTree().addFeature(std::move(feat));
                    continue;
                }
                if (ftype == "pattern") {
                    auto readVec = [&](const char* key) {
                        math::Vec3 v;
                        if (fObj.contains(key)) {
                            v = math::Vec3(fObj[key][0].get<double>(), fObj[key][1].get<double>(),
                                           fObj[key][2].get<double>());
                        }
                        return v;
                    };
                    math::Vec3 vecA = readVec("vecA");
                    math::Vec3 vecB = readVec("vecB");
                    double scalar = fObj.value("scalar", 0.0);
                    int count = fObj.value("count", 1);
                    std::vector<int> suppressed = fObj.value("suppressed", std::vector<int>{});
                    std::unique_ptr<doc::PatternFeature> feat;
                    if (fObj.value("kind", "linear") == "circular") {
                        feat = doc::PatternFeature::makeCircular(vecA, vecB, scalar, count,
                                                                 std::move(suppressed));
                    } else {
                        feat = doc::PatternFeature::makeLinear(vecA, scalar, count,
                                                               std::move(suppressed));
                    }
                    feat->restoreFeatureID(persistedId);
                    doc.featureTree().addFeature(std::move(feat));
                    continue;
                }
                if (ftype == "datum") {
                    auto readVec = [&](const char* key) {
                        math::Vec3 v;
                        if (fObj.contains(key)) {
                            v = math::Vec3(fObj[key][0].get<double>(), fObj[key][1].get<double>(),
                                           fObj[key][2].get<double>());
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
                    doc.featureTree().addFeature(std::move(feat));
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
                    feat->restoreFeatureID(persistedId);
                    doc.featureTree().addFeature(std::move(feat));
                    continue;
                }

                // Single-sketch features (extrude/revolve): v16 files reference
                // the sketch by ID; v15 files stored a (buggy) index — fall
                // back to it so old files keep loading.
                std::shared_ptr<doc::Sketch> sketch;
                if (fObj.contains("sketchId")) {
                    sketch = findSketch(fObj["sketchId"].get<uint64_t>());
                } else {
                    int sketchIndex = fObj.value("sketchIndex", -1);
                    if (sketchIndex >= 0 && sketchIndex < static_cast<int>(doc.sketches().size())) {
                        sketch = doc.sketches()[static_cast<size_t>(sketchIndex)];
                    }
                }
                if (!sketch) continue;

                if (ftype == "extrude") {
                    double distance = fObj.value("distance", 1.0);
                    math::Vec3 direction(0, 0, 1);
                    if (fObj.contains("direction")) {
                        direction = math::Vec3(fObj["direction"][0].get<double>(),
                                               fObj["direction"][1].get<double>(),
                                               fObj["direction"][2].get<double>());
                    }
                    auto feat = std::make_unique<doc::ExtrudeFeature>(sketch, direction, distance);
                    feat->restoreFeatureID(persistedId);
                    doc.featureTree().addFeature(std::move(feat));
                } else if (ftype == "revolve") {
                    double angle = fObj.value("angle", 6.283185307179586);
                    math::Vec3 axisPoint = hz::math::Vec3::Zero;
                    math::Vec3 axisDir = hz::math::Vec3::UnitY;
                    if (fObj.contains("axisPoint")) {
                        axisPoint = math::Vec3(fObj["axisPoint"][0].get<double>(),
                                               fObj["axisPoint"][1].get<double>(),
                                               fObj["axisPoint"][2].get<double>());
                    }
                    if (fObj.contains("axisDir")) {
                        axisDir = math::Vec3(fObj["axisDir"][0].get<double>(),
                                             fObj["axisDir"][1].get<double>(),
                                             fObj["axisDir"][2].get<double>());
                    }
                    auto feat =
                        std::make_unique<doc::RevolveFeature>(sketch, axisPoint, axisDir, angle);
                    feat->restoreFeatureID(persistedId);
                    doc.featureTree().addFeature(std::move(feat));
                }
            } catch (const nlohmann::json::exception&) {
                continue;  // Skip malformed features.
            }
        }
    }

    return true;
}

bool NativeFormat::load(const std::string& filePath, doc::Document& doc) {
    std::ifstream file(filePath);
    if (!file.is_open()) return false;

    json root;
    try {
        file >> root;
    } catch (...) {
        return false;
    }

    return loadDocumentRoot(root, doc);
}

bool NativeFormat::documentFromJson(const std::string& text, doc::Document& doc) {
    json root;
    try {
        root = json::parse(text);
    } catch (...) {
        return false;
    }
    return loadDocumentRoot(root, doc);
}

// ---------------------------------------------------------------------------
// Assembly serialization (.hzasm)
// ---------------------------------------------------------------------------

/// Build the assembly JSON envelope. @p filePath anchors relative component
/// paths (empty → paths stored as-is). Shared by saveAssembly() and
/// BinaryFormat.
static json buildAssemblyRoot(const doc::AssemblyDocument& asmDoc, const std::string& filePath) {
    json root;
    root["version"] = 16;
    root["type"] = "hzasm";

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

bool NativeFormat::saveAssembly(const std::string& filePath, const doc::AssemblyDocument& asmDoc) {
    const json root = buildAssemblyRoot(asmDoc, filePath);

    std::ofstream file(filePath);
    if (!file.is_open()) return false;
    file << root.dump(2);
    return file.good();
}

std::string NativeFormat::assemblyToJson(const doc::AssemblyDocument& asmDoc,
                                         const std::string& filePath) {
    return buildAssemblyRoot(asmDoc, filePath).dump();
}

/// Populate an AssemblyDocument from a parsed envelope. @p filePath anchors
/// relative component paths. Shared by loadAssembly() and BinaryFormat.
static bool loadAssemblyRoot(const json& root, doc::AssemblyDocument& asmDoc,
                             const std::string& filePath) {
    if (!root.contains("type") || !root["type"].is_string() ||
        root["type"].get<std::string>() != "hzasm") {
        return false;
    }

    asmDoc.clear();

    if (root.contains("components")) {
        for (const auto& cObj : root["components"]) {
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

                if (cObj.contains("transform") && cObj["transform"].size() == 16) {
                    const auto& t = cObj["transform"];
                    for (int row = 0; row < 4; ++row) {
                        for (int col = 0; col < 4; ++col) {
                            comp.transform.at(row, col) =
                                t[static_cast<size_t>(row * 4 + col)].get<double>();
                        }
                    }
                }

                asmDoc.addComponent(std::move(comp));
            } catch (const nlohmann::json::exception&) {
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
        for (const auto& mObj : root["mates"]) {
            try {
                doc::Mate mate;
                mate.id = mObj.value("id", uint64_t{0});
                mate.type = mateTypeFromString(mObj.value("type", "coincident"));
                mate.value = mObj.value("value", 0.0);
                if (mObj.contains("a")) {
                    mate.a.componentId = mObj["a"].value("componentId", uint64_t{0});
                    mate.a.faceId = topo::TopologyID::fromTag(mObj["a"].value("faceTag", ""));
                }
                if (mObj.contains("b")) {
                    mate.b.componentId = mObj["b"].value("componentId", uint64_t{0});
                    mate.b.faceId = topo::TopologyID::fromTag(mObj["b"].value("faceTag", ""));
                }
                asmDoc.addMate(std::move(mate));
            } catch (const nlohmann::json::exception&) {
                continue;  // Skip malformed mates.
            }
        }
    }

    asmDoc.setDirty(false);
    return true;
}

bool NativeFormat::loadAssembly(const std::string& filePath, doc::AssemblyDocument& asmDoc) {
    std::ifstream file(filePath);
    if (!file.is_open()) return false;

    json root;
    try {
        file >> root;
    } catch (...) {
        return false;
    }

    return loadAssemblyRoot(root, asmDoc, filePath);
}

bool NativeFormat::assemblyFromJson(const std::string& text, doc::AssemblyDocument& asmDoc,
                                    const std::string& filePath) {
    json root;
    try {
        root = json::parse(text);
    } catch (...) {
        return false;
    }
    return loadAssemblyRoot(root, asmDoc, filePath);
}

// ---------------------------------------------------------------------------
// Lightweight tessellation-cache read
// ---------------------------------------------------------------------------

std::shared_ptr<render::MeshData> NativeFormat::loadPartMesh(const std::string& filePath) {
    std::ifstream file(filePath);
    if (!file.is_open()) return nullptr;

    json root;
    try {
        file >> root;
    } catch (...) {
        return nullptr;
    }

    if (!root.contains("tessellationCache")) return nullptr;

    try {
        const auto& cache = root["tessellationCache"];
        auto mesh = std::make_shared<render::MeshData>();
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
        return mesh;
    } catch (const nlohmann::json::exception&) {
        return nullptr;
    }
}

}  // namespace hz::io
