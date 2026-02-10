#include "horizon/fileio/NativeFormat.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftCircle.h"

#include <nlohmann/json.hpp>
#include <fstream>

using json = nlohmann::json;

namespace hz::io {

bool NativeFormat::save(const std::string& filePath,
                        const draft::DraftDocument& doc) {
    json root;
    root["version"] = 1;
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
        }
    }

    return true;
}

}  // namespace hz::io
