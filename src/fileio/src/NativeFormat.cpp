#include "horizon/fileio/NativeFormat.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftRectangle.h"
#include "horizon/drafting/DraftPolyline.h"

#include <nlohmann/json.hpp>
#include <fstream>

using json = nlohmann::json;

namespace hz::io {

bool NativeFormat::save(const std::string& filePath,
                        const doc::Document& doc) {
    json root;
    root["version"] = 3;
    root["type"] = "hcad";

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
        layersArray.push_back(layerObj);
    }
    root["layers"] = layersArray;
    root["currentLayer"] = doc.layerManager().currentLayer();

    // --- Entities ---
    json entitiesArray = json::array();
    for (const auto& entity : doc.draftDocument().entities()) {
        json obj;
        obj["id"] = entity->id();
        obj["layer"] = entity->layer();
        obj["color"] = entity->color();
        obj["lineWidth"] = entity->lineWidth();

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
        }

        entitiesArray.push_back(obj);
    }
    root["entities"] = entitiesArray;

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

    // --- Load layer table (v3+) ---
    if (root.contains("layers")) {
        for (const auto& layerObj : root["layers"]) {
            draft::LayerProperties props;
            props.name = layerObj.value("name", "0");
            props.color = layerObj.value("color", 0xFFFFFFFFu);
            props.lineWidth = layerObj.value("lineWidth", 1.0);
            props.visible = layerObj.value("visible", true);
            props.locked = layerObj.value("locked", false);
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

    // --- Load entities ---
    for (const auto& obj : root["entities"]) {
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
        }

        if (entity) {
            entity->setLayer(layer);
            entity->setColor(color);
            entity->setLineWidth(lineWidth);
            doc.draftDocument().addEntity(entity);
        }
    }

    return true;
}

}  // namespace hz::io
