#include "horizon/fileio/NativeFormat.h"
#include "horizon/constraint/Constraint.h"
#include "horizon/constraint/ConstraintSystem.h"
#include "horizon/constraint/GeometryRef.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftRectangle.h"
#include "horizon/drafting/DraftPolyline.h"
#include "horizon/drafting/DraftLinearDimension.h"
#include "horizon/drafting/DraftRadialDimension.h"
#include "horizon/drafting/DraftAngularDimension.h"
#include "horizon/drafting/DraftLeader.h"
#include "horizon/drafting/DraftBlockRef.h"
#include "horizon/drafting/DraftText.h"
#include "horizon/drafting/DraftSpline.h"
#include "horizon/drafting/DraftHatch.h"
#include "horizon/drafting/DraftEllipse.h"
#include "horizon/drafting/BlockTable.h"

#include <nlohmann/json.hpp>
#include <fstream>
#include <set>

using json = nlohmann::json;

namespace hz::io {

// ---------------------------------------------------------------------------
// Constraint serialization helpers
// ---------------------------------------------------------------------------

static std::string featureTypeToString(cstr::FeatureType ft) {
    switch (ft) {
        case cstr::FeatureType::Point:  return "point";
        case cstr::FeatureType::Line:   return "line";
        case cstr::FeatureType::Circle: return "circle";
    }
    return "point";
}

static cstr::FeatureType featureTypeFromString(const std::string& s) {
    if (s == "line")   return cstr::FeatureType::Line;
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
        case cstr::ConstraintType::Coincident:     return "coincident";
        case cstr::ConstraintType::Horizontal:     return "horizontal";
        case cstr::ConstraintType::Vertical:       return "vertical";
        case cstr::ConstraintType::Perpendicular:  return "perpendicular";
        case cstr::ConstraintType::Parallel:       return "parallel";
        case cstr::ConstraintType::Tangent:        return "tangent";
        case cstr::ConstraintType::Equal:          return "equal";
        case cstr::ConstraintType::Fixed:          return "fixed";
        case cstr::ConstraintType::Distance:       return "distance";
        case cstr::ConstraintType::Angle:          return "angle";
    }
    return "unknown";
}

bool NativeFormat::save(const std::string& filePath,
                        const doc::Document& doc) {
    json root;
    root["version"] = 11;
    root["type"] = "hcad";

    // --- Dimension style ---
    const auto& ds = doc.draftDocument().dimensionStyle();
    root["dimensionStyle"] = {
        {"textHeight",         ds.textHeight},
        {"arrowSize",          ds.arrowSize},
        {"arrowAngle",         ds.arrowAngle},
        {"extensionGap",       ds.extensionGap},
        {"extensionOvershoot", ds.extensionOvershoot},
        {"precision",          ds.precision},
        {"showUnits",          ds.showUnits}
    };

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
                se["end"]   = {{"x", ln->end().x},   {"y", ln->end().y}};
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
                for (const auto& cp : sp->controlPoints()) cps.push_back({{"x", cp.x}, {"y", cp.y}});
                se["controlPoints"] = cps;
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

    // --- Entities ---
    json entitiesArray = json::array();
    for (const auto& entity : doc.draftDocument().entities()) {
        json obj;
        obj["id"] = entity->id();
        obj["layer"] = entity->layer();
        obj["color"] = entity->color();
        obj["lineWidth"] = entity->lineWidth();
        obj["lineType"] = entity->lineType();

        if (auto* line = dynamic_cast<const draft::DraftLine*>(entity.get())) {
            obj["type"] = "line";
            obj["start"] = {{"x", line->start().x}, {"y", line->start().y}};
            obj["end"]   = {{"x", line->end().x},   {"y", line->end().y}};
        } else if (auto* circle = dynamic_cast<const draft::DraftCircle*>(entity.get())) {
            obj["type"] = "circle";
            obj["center"] = {{"x", circle->center().x}, {"y", circle->center().y}};
            obj["radius"] = circle->radius();
        } else if (auto* arc = dynamic_cast<const draft::DraftArc*>(entity.get())) {
            obj["type"] = "arc";
            obj["center"] = {{"x", arc->center().x}, {"y", arc->center().y}};
            obj["radius"] = arc->radius();
            obj["startAngle"] = arc->startAngle();
            obj["endAngle"] = arc->endAngle();
        } else if (auto* rect = dynamic_cast<const draft::DraftRectangle*>(entity.get())) {
            obj["type"] = "rectangle";
            obj["corner1"] = {{"x", rect->corner1().x}, {"y", rect->corner1().y}};
            obj["corner2"] = {{"x", rect->corner2().x}, {"y", rect->corner2().y}};
        } else if (auto* polyline = dynamic_cast<const draft::DraftPolyline*>(entity.get())) {
            obj["type"] = "polyline";
            obj["closed"] = polyline->closed();
            json pointsArray = json::array();
            for (const auto& pt : polyline->points()) {
                pointsArray.push_back({{"x", pt.x}, {"y", pt.y}});
            }
            obj["points"] = pointsArray;
        } else if (auto* ld = dynamic_cast<const draft::DraftLinearDimension*>(entity.get())) {
            obj["type"] = "linearDimension";
            obj["defPoint1"] = {{"x", ld->defPoint1().x}, {"y", ld->defPoint1().y}};
            obj["defPoint2"] = {{"x", ld->defPoint2().x}, {"y", ld->defPoint2().y}};
            obj["dimLinePoint"] = {{"x", ld->dimLinePoint().x}, {"y", ld->dimLinePoint().y}};
            obj["orientation"] = static_cast<int>(ld->orientation());
            if (ld->hasTextOverride()) obj["textOverride"] = ld->textOverride();
        } else if (auto* rd = dynamic_cast<const draft::DraftRadialDimension*>(entity.get())) {
            obj["type"] = "radialDimension";
            obj["center"] = {{"x", rd->center().x}, {"y", rd->center().y}};
            obj["radius"] = rd->radius();
            obj["textPoint"] = {{"x", rd->textPoint().x}, {"y", rd->textPoint().y}};
            obj["isDiameter"] = rd->isDiameter();
            if (rd->hasTextOverride()) obj["textOverride"] = rd->textOverride();
        } else if (auto* ad = dynamic_cast<const draft::DraftAngularDimension*>(entity.get())) {
            obj["type"] = "angularDimension";
            obj["vertex"] = {{"x", ad->vertex().x}, {"y", ad->vertex().y}};
            obj["line1Point"] = {{"x", ad->line1Point().x}, {"y", ad->line1Point().y}};
            obj["line2Point"] = {{"x", ad->line2Point().x}, {"y", ad->line2Point().y}};
            obj["arcRadius"] = ad->arcRadius();
            if (ad->hasTextOverride()) obj["textOverride"] = ad->textOverride();
        } else if (auto* leader = dynamic_cast<const draft::DraftLeader*>(entity.get())) {
            obj["type"] = "leader";
            obj["text"] = leader->text();
            json ptsArray = json::array();
            for (const auto& pt : leader->points()) {
                ptsArray.push_back({{"x", pt.x}, {"y", pt.y}});
            }
            obj["points"] = ptsArray;
            if (leader->hasTextOverride()) obj["textOverride"] = leader->textOverride();
        } else if (auto* bref = dynamic_cast<const draft::DraftBlockRef*>(entity.get())) {
            obj["type"] = "blockRef";
            obj["blockName"] = bref->blockName();
            obj["insertPos"] = {{"x", bref->insertPos().x}, {"y", bref->insertPos().y}};
            obj["rotation"] = bref->rotation();
            obj["scale"] = bref->uniformScale();
        } else if (auto* txt = dynamic_cast<const draft::DraftText*>(entity.get())) {
            obj["type"] = "text";
            obj["position"] = {{"x", txt->position().x}, {"y", txt->position().y}};
            obj["text"] = txt->text();
            obj["textHeight"] = txt->textHeight();
            obj["rotation"] = txt->rotation();
            obj["alignment"] = static_cast<int>(txt->alignment());
        } else if (auto* spline = dynamic_cast<const draft::DraftSpline*>(entity.get())) {
            obj["type"] = "spline";
            obj["closed"] = spline->closed();
            json cpArray = json::array();
            for (const auto& cp : spline->controlPoints()) {
                cpArray.push_back({{"x", cp.x}, {"y", cp.y}});
            }
            obj["controlPoints"] = cpArray;
        } else if (auto* hatch = dynamic_cast<const draft::DraftHatch*>(entity.get())) {
            obj["type"] = "hatch";
            obj["pattern"] = static_cast<int>(hatch->pattern());
            obj["angle"] = hatch->angle();
            obj["spacing"] = hatch->spacing();
            json bndArray = json::array();
            for (const auto& pt : hatch->boundary()) {
                bndArray.push_back({{"x", pt.x}, {"y", pt.y}});
            }
            obj["boundary"] = bndArray;
        } else if (auto* ellipse = dynamic_cast<const draft::DraftEllipse*>(entity.get())) {
            obj["type"] = "ellipse";
            obj["center"] = {{"x", ellipse->center().x}, {"y", ellipse->center().y}};
            obj["semiMajor"] = ellipse->semiMajor();
            obj["semiMinor"] = ellipse->semiMinor();
            obj["rotation"] = ellipse->rotation();
        }

        entitiesArray.push_back(obj);
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

        constraintsArray.push_back(cObj);
    }
    root["constraints"] = constraintsArray;

    std::ofstream file(filePath);
    if (!file.is_open()) return false;
    file << root.dump(2);
    return file.good();
}

bool NativeFormat::load(const std::string& filePath,
                        doc::Document& doc) {
    std::ifstream file(filePath);
    if (!file.is_open()) return false;

    json root;
    try {
        file >> root;
    } catch (...) {
        return false;
    }

    if (!root.contains("version") || !root.contains("entities")) return false;

    doc.draftDocument().clear();
    doc.layerManager().clear();
    doc.constraintSystem().clear();

    // --- Load dimension style (v4+) ---
    if (root.contains("dimensionStyle")) {
        const auto& dsObj = root["dimensionStyle"];
        draft::DimensionStyle ds;
        ds.textHeight         = dsObj.value("textHeight", 2.5);
        ds.arrowSize          = dsObj.value("arrowSize", 1.5);
        ds.arrowAngle         = dsObj.value("arrowAngle", 0.3);
        ds.extensionGap       = dsObj.value("extensionGap", 0.5);
        ds.extensionOvershoot = dsObj.value("extensionOvershoot", 1.0);
        ds.precision          = dsObj.value("precision", 2);
        ds.showUnits          = dsObj.value("showUnits", false);
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
            def->basePoint = math::Vec2(
                blockObj["basePoint"]["x"].get<double>(),
                blockObj["basePoint"]["y"].get<double>());
            if (blockObj.contains("entities")) {
                for (const auto& se : blockObj["entities"]) {
                    std::string stype = se.value("type", "");
                    std::shared_ptr<draft::DraftEntity> subEnt;
                    if (stype == "line") {
                        subEnt = std::make_shared<draft::DraftLine>(
                            math::Vec2(se["start"]["x"].get<double>(), se["start"]["y"].get<double>()),
                            math::Vec2(se["end"]["x"].get<double>(), se["end"]["y"].get<double>()));
                    } else if (stype == "circle") {
                        subEnt = std::make_shared<draft::DraftCircle>(
                            math::Vec2(se["center"]["x"].get<double>(), se["center"]["y"].get<double>()),
                            se["radius"].get<double>());
                    } else if (stype == "arc") {
                        subEnt = std::make_shared<draft::DraftArc>(
                            math::Vec2(se["center"]["x"].get<double>(), se["center"]["y"].get<double>()),
                            se["radius"].get<double>(),
                            se["startAngle"].get<double>(),
                            se["endAngle"].get<double>());
                    } else if (stype == "rectangle") {
                        subEnt = std::make_shared<draft::DraftRectangle>(
                            math::Vec2(se["corner1"]["x"].get<double>(), se["corner1"]["y"].get<double>()),
                            math::Vec2(se["corner2"]["x"].get<double>(), se["corner2"]["y"].get<double>()));
                    } else if (stype == "polyline") {
                        std::vector<math::Vec2> pts;
                        for (const auto& pt : se["points"])
                            pts.emplace_back(pt["x"].get<double>(), pt["y"].get<double>());
                        subEnt = std::make_shared<draft::DraftPolyline>(pts, se.value("closed", false));
                    } else if (stype == "spline") {
                        std::vector<math::Vec2> cps;
                        for (const auto& cp : se["controlPoints"])
                            cps.emplace_back(cp["x"].get<double>(), cp["y"].get<double>());
                        subEnt = std::make_shared<draft::DraftSpline>(cps, se.value("closed", false));
                    } else if (stype == "text") {
                        auto pos = math::Vec2(se["position"]["x"].get<double>(),
                                               se["position"]["y"].get<double>());
                        auto txt = std::make_shared<draft::DraftText>(
                            pos, se.value("text", ""), se.value("textHeight", 2.5));
                        if (se.contains("rotation"))
                            txt->setRotation(se["rotation"].get<double>());
                        if (se.contains("alignment"))
                            txt->setAlignment(static_cast<draft::TextAlignment>(se["alignment"].get<int>()));
                        subEnt = txt;
                    } else if (stype == "hatch") {
                        std::vector<math::Vec2> boundary;
                        for (const auto& pt : se["boundary"])
                            boundary.emplace_back(pt["x"].get<double>(), pt["y"].get<double>());
                        subEnt = std::make_shared<draft::DraftHatch>(
                            boundary,
                            static_cast<draft::HatchPattern>(se.value("pattern", 1)),
                            se.value("angle", 0.0), se.value("spacing", 1.0));
                    } else if (stype == "ellipse") {
                        auto ctr = math::Vec2(se["center"]["x"].get<double>(),
                                               se["center"]["y"].get<double>());
                        subEnt = std::make_shared<draft::DraftEllipse>(
                            ctr, se.value("semiMajor", 1.0),
                            se.value("semiMinor", 1.0), se.value("rotation", 0.0));
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
    for (const auto& obj : root["entities"]) {
        try {
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
            entity = std::make_shared<draft::DraftLine>(
                math::Vec2(sx, sy), math::Vec2(ex, ey));
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
            entity = std::make_shared<draft::DraftRectangle>(
                math::Vec2(c1x, c1y), math::Vec2(c2x, c2y));
        } else if (type == "polyline") {
            bool closed = obj.value("closed", false);
            std::vector<math::Vec2> points;
            for (const auto& pt : obj["points"]) {
                points.emplace_back(pt["x"].get<double>(), pt["y"].get<double>());
            }
            entity = std::make_shared<draft::DraftPolyline>(points, closed);
        } else if (type == "linearDimension") {
            auto p1 = math::Vec2(obj["defPoint1"]["x"].get<double>(),
                                  obj["defPoint1"]["y"].get<double>());
            auto p2 = math::Vec2(obj["defPoint2"]["x"].get<double>(),
                                  obj["defPoint2"]["y"].get<double>());
            auto dp = math::Vec2(obj["dimLinePoint"]["x"].get<double>(),
                                  obj["dimLinePoint"]["y"].get<double>());
            auto orient = static_cast<draft::DraftLinearDimension::Orientation>(
                obj.value("orientation", 0));
            auto dim = std::make_shared<draft::DraftLinearDimension>(p1, p2, dp, orient);
            if (obj.contains("textOverride"))
                dim->setTextOverride(obj["textOverride"].get<std::string>());
            entity = dim;
        } else if (type == "radialDimension") {
            auto center = math::Vec2(obj["center"]["x"].get<double>(),
                                      obj["center"]["y"].get<double>());
            double radius = obj["radius"].get<double>();
            auto textPt = math::Vec2(obj["textPoint"]["x"].get<double>(),
                                      obj["textPoint"]["y"].get<double>());
            bool isDiam = obj.value("isDiameter", false);
            auto dim = std::make_shared<draft::DraftRadialDimension>(
                center, radius, textPt, isDiam);
            if (obj.contains("textOverride"))
                dim->setTextOverride(obj["textOverride"].get<std::string>());
            entity = dim;
        } else if (type == "angularDimension") {
            auto vertex = math::Vec2(obj["vertex"]["x"].get<double>(),
                                      obj["vertex"]["y"].get<double>());
            auto l1 = math::Vec2(obj["line1Point"]["x"].get<double>(),
                                  obj["line1Point"]["y"].get<double>());
            auto l2 = math::Vec2(obj["line2Point"]["x"].get<double>(),
                                  obj["line2Point"]["y"].get<double>());
            double arcR = obj["arcRadius"].get<double>();
            auto dim = std::make_shared<draft::DraftAngularDimension>(
                vertex, l1, l2, arcR);
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
            std::string blockName = obj.value("blockName", "");
            auto def = doc.draftDocument().blockTable().findBlock(blockName);
            if (def) {
                auto pos = math::Vec2(obj["insertPos"]["x"].get<double>(),
                                       obj["insertPos"]["y"].get<double>());
                double rot = obj.value("rotation", 0.0);
                double scl = obj.value("scale", 1.0);
                entity = std::make_shared<draft::DraftBlockRef>(def, pos, rot, scl);
            }
        } else if (type == "text") {
            auto pos = math::Vec2(obj["position"]["x"].get<double>(),
                                   obj["position"]["y"].get<double>());
            std::string text = obj.value("text", "");
            double textHeight = obj.value("textHeight", 2.5);
            auto txt = std::make_shared<draft::DraftText>(pos, text, textHeight);
            if (obj.contains("rotation"))
                txt->setRotation(obj["rotation"].get<double>());
            if (obj.contains("alignment"))
                txt->setAlignment(static_cast<draft::TextAlignment>(obj["alignment"].get<int>()));
            entity = txt;
        } else if (type == "spline") {
            bool closed = obj.value("closed", false);
            std::vector<math::Vec2> controlPoints;
            for (const auto& cp : obj["controlPoints"]) {
                controlPoints.emplace_back(cp["x"].get<double>(), cp["y"].get<double>());
            }
            entity = std::make_shared<draft::DraftSpline>(controlPoints, closed);
        } else if (type == "hatch") {
            std::vector<math::Vec2> boundary;
            for (const auto& pt : obj["boundary"]) {
                boundary.emplace_back(pt["x"].get<double>(), pt["y"].get<double>());
            }
            auto hatchPattern = static_cast<draft::HatchPattern>(obj.value("pattern", 1));
            double hatchAngle = obj.value("angle", 0.0);
            double hatchSpacing = obj.value("spacing", 1.0);
            entity = std::make_shared<draft::DraftHatch>(boundary, hatchPattern,
                                                          hatchAngle, hatchSpacing);
        } else if (type == "ellipse") {
            auto center = math::Vec2(obj["center"]["x"].get<double>(),
                                      obj["center"]["y"].get<double>());
            double semiMajor = obj.value("semiMajor", 1.0);
            double semiMinor = obj.value("semiMinor", 1.0);
            double rot = obj.value("rotation", 0.0);
            entity = std::make_shared<draft::DraftEllipse>(center, semiMajor, semiMinor, rot);
        }

        if (entity) {
            // Restore the original entity ID from the file.
            if (obj.contains("id")) {
                uint64_t savedId = obj["id"].get<uint64_t>();
                entity->setId(savedId);
                draft::DraftEntity::advanceIdCounter(savedId);
            }
            entity->setLayer(layer);
            entity->setColor(color);
            entity->setLineWidth(lineWidth);
            entity->setLineType(obj.value("lineType", 0));
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
                constraint = std::make_shared<cstr::EqualConstraint>(
                    deserializeRef(cObj["refA"]), deserializeRef(cObj["refB"]));
            } else if (ctype == "fixed") {
                auto pos = math::Vec2(cObj["position"]["x"].get<double>(),
                                       cObj["position"]["y"].get<double>());
                constraint = std::make_shared<cstr::FixedConstraint>(
                    deserializeRef(cObj["ref"]), pos);
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

    return true;
}

}  // namespace hz::io
