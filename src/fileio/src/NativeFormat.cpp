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
                        const draft::DraftDocument& doc) {
    json root;
    root["version"] = 2;
    root["type"] = "hcad";

    json entitiesArray = json::array();
    for (const auto& entity : doc.entities()) {
        json obj;
        obj["id"] = entity->id();
        obj["layer"] = entity->layer();
        obj["color"] = entity->color();

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
                        draft::DraftDocument& doc) {
    std::ifstream file(filePath);
    if (!file.is_open()) return false;

    json root;
    try {
        file >> root;
    } catch (...) {
        return false;
    }

    if (!root.contains("version") || !root.contains("entities")) return false;

    doc.clear();

    for (const auto& obj : root["entities"]) {
        std::string type = obj.value("type", "");
        std::string layer = obj.value("layer", "0");
        uint32_t color = obj.value("color", 0xFFFFFFFFu);

        if (type == "line") {
            double sx = obj["start"]["x"].get<double>();
            double sy = obj["start"]["y"].get<double>();
            double ex = obj["end"]["x"].get<double>();
            double ey = obj["end"]["y"].get<double>();

            auto line = std::make_shared<draft::DraftLine>(
                math::Vec2(sx, sy), math::Vec2(ex, ey));
            line->setLayer(layer);
            line->setColor(color);
            doc.addEntity(line);
        } else if (type == "circle") {
            double cx = obj["center"]["x"].get<double>();
            double cy = obj["center"]["y"].get<double>();
            double r = obj["radius"].get<double>();

            auto circle = std::make_shared<draft::DraftCircle>(
                math::Vec2(cx, cy), r);
            circle->setLayer(layer);
            circle->setColor(color);
            doc.addEntity(circle);
        } else if (type == "arc") {
            double cx = obj["center"]["x"].get<double>();
            double cy = obj["center"]["y"].get<double>();
            double r = obj["radius"].get<double>();
            double sa = obj["startAngle"].get<double>();
            double ea = obj["endAngle"].get<double>();

            auto arc = std::make_shared<draft::DraftArc>(
                math::Vec2(cx, cy), r, sa, ea);
            arc->setLayer(layer);
            arc->setColor(color);
            doc.addEntity(arc);
        } else if (type == "rectangle") {
            double c1x = obj["corner1"]["x"].get<double>();
            double c1y = obj["corner1"]["y"].get<double>();
            double c2x = obj["corner2"]["x"].get<double>();
            double c2y = obj["corner2"]["y"].get<double>();

            auto rect = std::make_shared<draft::DraftRectangle>(
                math::Vec2(c1x, c1y), math::Vec2(c2x, c2y));
            rect->setLayer(layer);
            rect->setColor(color);
            doc.addEntity(rect);
        } else if (type == "polyline") {
            bool closed = obj.value("closed", false);
            std::vector<math::Vec2> points;
            for (const auto& pt : obj["points"]) {
                points.emplace_back(pt["x"].get<double>(), pt["y"].get<double>());
            }
            auto polyline = std::make_shared<draft::DraftPolyline>(points, closed);
            polyline->setLayer(layer);
            polyline->setColor(color);
            doc.addEntity(polyline);
        }
    }

    return true;
}

}  // namespace hz::io
