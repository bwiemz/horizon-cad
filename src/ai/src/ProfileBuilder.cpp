#include "horizon/ai/ProfileBuilder.h"

#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

#include "CadToolsCommon.h"
#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftPolyline.h"
#include "horizon/drafting/DraftRectangle.h"
#include "horizon/drafting/DraftText.h"
#include "horizon/math/Vec2.h"

namespace hz::ai {

namespace {

using detail::ArgError;
using math::Vec2;
using nlohmann::json;

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegree = kPi / 180.0;
constexpr int kMaxPolygonSides = 1024;
constexpr std::size_t kMaxPoints = 100000;

/// Reads one entity's arguments, naming the entity in every complaint.
struct Reader {
    const json& in;
    std::string who;  ///< "entity 3 (circle)"

    [[noreturn]] void fail(const std::string& what) const { throw ArgError(who + ": " + what); }

    bool has(const char* key) const { return in.contains(key) && !in[key].is_null(); }

    double number(const char* key) const {
        if (!has(key)) fail(std::string("\"") + key + "\" is required");
        const json& v = in[key];
        if (!v.is_number()) fail(std::string("\"") + key + "\" must be a number");
        const double d = v.get<double>();
        if (!std::isfinite(d)) fail(std::string("\"") + key + "\" must be finite");
        return d;
    }
    double number(const char* key, double otherwise) const {
        return has(key) ? number(key) : otherwise;
    }
    double positive(const char* key) const {
        const double d = number(key);
        if (!(d > 0.0)) fail(std::string("\"") + key + "\" must be greater than 0");
        return d;
    }
    Vec2 point(const char* key) const {
        if (!has(key)) fail(std::string("\"") + key + "\" is required");
        return pointOf(in[key], key);
    }
    Vec2 pointOf(const json& v, const std::string& name) const {
        double x = 0.0;
        double y = 0.0;
        if (v.is_array() && v.size() == 2 && v[0].is_number() && v[1].is_number()) {
            x = v[0].get<double>();
            y = v[1].get<double>();
        } else if (v.is_object() && v.contains("x") && v.contains("y") && v["x"].is_number() &&
                   v["y"].is_number()) {
            x = v["x"].get<double>();
            y = v["y"].get<double>();
        } else {
            fail("\"" + name + "\" must be a point [x, y]");
        }
        if (!std::isfinite(x) || !std::isfinite(y)) fail("\"" + name + "\" must be finite");
        return {x, y};
    }
};

Vec2 rotated(const Vec2& v, double angle) {
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    return {v.x * c - v.y * s, v.x * s + v.y * c};
}

using Entities = std::vector<std::shared_ptr<draft::DraftEntity>>;

void addRectangle(const Reader& r, Entities& out) {
    const double w = r.positive("width");
    const double h = r.positive("height");
    const double angle = r.number("angle", 0.0) * kDegree;
    Vec2 corner;
    if (r.has("center")) {
        corner = r.point("center") - rotated(Vec2(w / 2.0, h / 2.0), angle);
    } else {
        corner = r.point("corner");
    }
    if (angle == 0.0) {
        out.push_back(std::make_shared<draft::DraftRectangle>(corner, corner + Vec2(w, h)));
        return;
    }
    const std::vector<Vec2> points = {corner, corner + rotated(Vec2(w, 0.0), angle),
                                      corner + rotated(Vec2(w, h), angle),
                                      corner + rotated(Vec2(0.0, h), angle)};
    out.push_back(std::make_shared<draft::DraftPolyline>(points, true));
}

void addSlot(const Reader& r, Entities& out) {
    const Vec2 c = r.point("center");
    const double length = r.number("length");
    if (length < 0.0) r.fail("\"length\" must not be negative");
    const double radius = r.positive("width") / 2.0;
    const double angle = r.number("angle", 0.0) * kDegree;
    if (length == 0.0) {
        out.push_back(std::make_shared<draft::DraftCircle>(c, radius));
        return;
    }
    const Vec2 u(std::cos(angle), std::sin(angle));
    const Vec2 n(-u.y, u.x);
    const Vec2 a = c - u * (length / 2.0);
    const Vec2 b = c + u * (length / 2.0);
    out.push_back(std::make_shared<draft::DraftLine>(a + n * radius, b + n * radius));
    out.push_back(
        std::make_shared<draft::DraftArc>(b, radius, angle - kPi / 2.0, angle + kPi / 2.0));
    out.push_back(std::make_shared<draft::DraftLine>(b - n * radius, a - n * radius));
    out.push_back(
        std::make_shared<draft::DraftArc>(a, radius, angle + kPi / 2.0, angle + 3.0 * kPi / 2.0));
}

void addPolygon(const Reader& r, Entities& out) {
    const Vec2 c = r.point("center");
    const double radius = r.positive("radius");
    if (!r.has("sides") || !r.in["sides"].is_number_integer()) {
        r.fail("\"sides\" must be a whole number");
    }
    const auto sides = r.in["sides"].get<long long>();
    if (sides < 3 || sides > kMaxPolygonSides) {
        r.fail("\"sides\" must be from 3 to " + std::to_string(kMaxPolygonSides));
    }
    const double start = r.number("angle", 90.0) * kDegree;  // a vertex at the top by default
    std::vector<Vec2> points;
    points.reserve(static_cast<std::size_t>(sides));
    for (long long i = 0; i < sides; ++i) {
        const double a = start + 2.0 * kPi * static_cast<double>(i) / static_cast<double>(sides);
        points.emplace_back(c.x + radius * std::cos(a), c.y + radius * std::sin(a));
    }
    out.push_back(std::make_shared<draft::DraftPolyline>(points, true));
}

void addPolyline(const Reader& r, Entities& out) {
    if (!r.has("points") || !r.in["points"].is_array())
        r.fail("\"points\" must be a list of points");
    const json& list = r.in["points"];
    if (list.size() > kMaxPoints) r.fail("too many points");
    std::vector<Vec2> points;
    points.reserve(list.size());
    for (std::size_t i = 0; i < list.size(); ++i) {
        points.push_back(r.pointOf(list[i], "points[" + std::to_string(i) + "]"));
    }
    bool closed = false;
    if (r.has("closed")) {
        if (!r.in["closed"].is_boolean()) r.fail("\"closed\" must be true or false");
        closed = r.in["closed"].get<bool>();
    }
    // A closed polyline given with its first point repeated at the end.
    if (closed && points.size() > 3 && (points.front() - points.back()).length() < 1e-9) {
        points.pop_back();
    }
    if (points.size() < (closed ? 3u : 2u)) {
        r.fail(closed ? "a closed polyline needs at least 3 points"
                      : "a polyline needs at least 2 points");
    }
    out.push_back(std::make_shared<draft::DraftPolyline>(points, closed));
}

void addOne(const Reader& r, bool allowText, Entities& out) {
    if (!r.in.is_object()) r.fail("must be an object");
    if (!r.has("type") || !r.in["type"].is_string()) r.fail("\"type\" is required");
    const std::string type = r.in["type"].get<std::string>();
    const std::size_t first = out.size();
    if (type == "line") {
        const Vec2 from = r.point("from");
        const Vec2 to = r.point("to");
        if ((to - from).length() < 1e-9) r.fail("its two ends are the same point");
        out.push_back(std::make_shared<draft::DraftLine>(from, to));
    } else if (type == "polyline") {
        addPolyline(r, out);
    } else if (type == "rectangle") {
        addRectangle(r, out);
    } else if (type == "circle") {
        const double radius =
            r.has("diameter") ? r.positive("diameter") / 2.0 : r.positive("radius");
        out.push_back(std::make_shared<draft::DraftCircle>(r.point("center"), radius));
    } else if (type == "arc") {
        const double start = r.number("start_angle") * kDegree;
        const double end = r.number("end_angle") * kDegree;
        if (std::abs(end - start) < 1e-9) r.fail("its start and end angles are the same");
        out.push_back(
            std::make_shared<draft::DraftArc>(r.point("center"), r.positive("radius"), start, end));
    } else if (type == "slot") {
        addSlot(r, out);
    } else if (type == "polygon") {
        addPolygon(r, out);
    } else if (type == "text" && allowText) {
        if (!r.has("text") || !r.in["text"].is_string()) r.fail("\"text\" is required");
        const double height = r.has("height") ? r.positive("height") : 2.5;
        out.push_back(std::make_shared<draft::DraftText>(r.point("position"),
                                                         r.in["text"].get<std::string>(), height));
    } else {
        r.fail("unknown type \"" + type + "\"" +
               (allowText ? ""
                          : " (a sketch takes line, polyline, rectangle, circle, arc, slot "
                            "and polygon)"));
    }
    if (r.has("construction")) {
        if (!r.in["construction"].is_boolean()) r.fail("\"construction\" must be true or false");
        if (r.in["construction"].get<bool>()) {
            for (std::size_t i = first; i < out.size(); ++i) out[i]->setConstruction(true);
        }
    }
}

}  // namespace

Entities buildEntities(const json& entities, bool allowText) {
    if (!entities.is_array()) throw ArgError("\"entities\" must be a list");
    if (entities.empty()) throw ArgError("\"entities\" is empty");
    Entities out;
    for (std::size_t i = 0; i < entities.size(); ++i) {
        const json& entity = entities[i];
        std::string who = "entity " + std::to_string(i);
        if (entity.is_object() && entity.contains("type") && entity["type"].is_string()) {
            who += " (" + entity["type"].get<std::string>() + ")";
        }
        addOne(Reader{entity, std::move(who)}, allowText, out);
    }
    return out;
}

json entitySchema(bool allowText) {
    json types = {"line", "polyline", "rectangle", "circle", "arc", "slot", "polygon"};
    if (allowText) types.push_back("text");
    const json point = {
        {"type", "array"}, {"items", {{"type", "number"}}}, {"description", "[x, y] in mm"}};
    json properties = {
        {"type", {{"type", "string"}, {"enum", types}}},
        {"from", point},
        {"to", point},
        {"points",
         {{"type", "array"}, {"items", point}, {"description", "polyline: its points in order"}}},
        {"closed",
         {{"type", "boolean"}, {"description", "polyline: join the last point to the first"}}},
        {"corner", point},
        {"center", point},
        {"width",
         {{"type", "number"}, {"description", "rectangle width; slot width (its diameter)"}}},
        {"height", {{"type", "number"}, {"description", "rectangle height; text height"}}},
        {"radius",
         {{"type", "number"}, {"description", "circle, arc; polygon: centre to a vertex"}}},
        {"diameter", {{"type", "number"}, {"description", "circle, instead of radius"}}},
        {"start_angle",
         {{"type", "number"}, {"description", "arc, degrees from +x, counterclockwise"}}},
        {"end_angle", {{"type", "number"}, {"description", "arc, degrees"}}},
        {"length",
         {{"type", "number"}, {"description", "slot: centre to centre of its round ends"}}},
        {"sides", {{"type", "integer"}, {"description", "polygon: number of sides"}}},
        {"angle",
         {{"type", "number"}, {"description", "rectangle, slot, polygon: rotation in degrees"}}},
        {"construction",
         {{"type", "boolean"}, {"description", "reference geometry, not part of the profile"}}}};
    if (allowText) {
        properties["position"] = point;
        properties["text"] = {{"type", "string"}};
    }
    return {
        {"type", "object"},
        {"description",
         "A shape. line: from, to. polyline: points, closed. rectangle: corner (lower left) or "
         "center, width, height, angle. circle: center, radius or diameter. arc: center, radius, "
         "start_angle, end_angle. slot: center, length, width, angle. polygon: center, sides, "
         "radius, angle." +
             std::string(allowText ? " text: position, text, height." : "")},
        {"properties", std::move(properties)},
        {"required", {"type"}}};
}

}  // namespace hz::ai
