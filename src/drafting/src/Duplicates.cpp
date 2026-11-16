#include "horizon/drafting/Duplicates.h"

#include <algorithm>
#include <cmath>
#include <optional>
#include <string>
#include <typeinfo>
#include <unordered_map>
#include <utility>

#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftEllipse.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftPolyline.h"
#include "horizon/drafting/DraftRectangle.h"
#include "horizon/drafting/DraftSpline.h"
#include "horizon/drafting/DraftText.h"

namespace hz::draft {

namespace {

using math::Vec2;

/// What two entities must share exactly (their kind and style, and a
/// text's words), and their shape as lengths, compared to a tolerance: a
/// line's ends in the same order whichever way it was drawn, an arc's and
/// an ellipse's by points on them, not angles, so one tolerance serves all.
struct Key {
    std::string exact;
    std::vector<double> shape;
};

bool before(const Vec2& a, const Vec2& b) {
    return a.x < b.x || (a.x == b.x && a.y < b.y);
}

void put(std::vector<double>& out, const Vec2& p) {
    out.push_back(p.x);
    out.push_back(p.y);
}

/// @p points as they run, or backward when that reads first: an open path
/// drawn from either end is the same path.
std::vector<Vec2> eitherWay(const std::vector<Vec2>& points, bool closed) {
    if (closed || points.size() < 2) return points;
    std::vector<Vec2> back(points.rbegin(), points.rend());
    return std::lexicographical_compare(back.begin(), back.end(), points.begin(), points.end(),
                                        before)
               ? back
               : points;
}

std::optional<Key> keyOf(const DraftEntity& entity) {
    Key key;
    key.exact = std::string(typeid(entity).name()) + '\n' + entity.layer() + '\n' +
                std::to_string(entity.color()) + '\n' + std::to_string(entity.lineWidth()) + '\n' +
                std::to_string(entity.lineType()) + '\n' + (entity.construction() ? "c" : "-");
    auto& shape = key.shape;
    if (const auto* line = dynamic_cast<const DraftLine*>(&entity)) {
        const bool swap = before(line->end(), line->start());
        put(shape, swap ? line->end() : line->start());
        put(shape, swap ? line->start() : line->end());
    } else if (const auto* circle = dynamic_cast<const DraftCircle*>(&entity)) {
        put(shape, circle->center());
        shape.push_back(circle->radius());
    } else if (const auto* arc = dynamic_cast<const DraftArc*>(&entity)) {
        put(shape, arc->center());
        shape.push_back(arc->radius());
        // Its ends, and so its sweep, from the angles: an angle and the
        // same plus a turn are one arc.
        for (const double angle : {arc->startAngle(), arc->endAngle()}) {
            put(shape, arc->center() + Vec2(std::cos(angle), std::sin(angle)) * arc->radius());
        }
    } else if (const auto* poly = dynamic_cast<const DraftPolyline*>(&entity)) {
        key.exact += poly->closed() ? "\nclosed" : "\nopen";
        for (const Vec2& p : eitherWay(poly->points(), poly->closed())) put(shape, p);
    } else if (const auto* rect = dynamic_cast<const DraftRectangle*>(&entity)) {
        for (const Vec2& p : rect->corners()) put(shape, p);
    } else if (const auto* ellipse = dynamic_cast<const DraftEllipse*>(&entity)) {
        put(shape, ellipse->center());
        // The ends of its axes: either end of each, the same ellipse.
        const Vec2 major(std::cos(ellipse->rotation()), std::sin(ellipse->rotation()));
        const Vec2 minor(-major.y, major.x);
        for (const auto& [axis, length] :
             {std::pair{major, ellipse->semiMajor()}, std::pair{minor, ellipse->semiMinor()}}) {
            const Vec2 a = ellipse->center() + axis * length;
            const Vec2 b = ellipse->center() - axis * length;
            put(shape, before(b, a) ? b : a);
        }
    } else if (const auto* spline = dynamic_cast<const DraftSpline*>(&entity)) {
        key.exact += spline->closed() ? "\nclosed" : "\nopen";
        for (const Vec2& p : spline->controlPoints()) put(shape, p);
        for (const double w : spline->weights()) shape.push_back(w);
    } else if (const auto* text = dynamic_cast<const DraftText*>(&entity)) {
        key.exact +=
            '\n' + text->text() + '\n' + std::to_string(static_cast<int>(text->alignment()));
        put(shape, text->position());
        shape.push_back(text->textHeight());
        // Its turn, as a point a height along its baseline.
        put(shape, text->position() + Vec2(std::cos(text->rotation()), std::sin(text->rotation())) *
                                          text->textHeight());
    } else {
        return std::nullopt;
    }
    for (const double v : shape) {
        if (!std::isfinite(v)) return std::nullopt;  // nothing to compare
    }
    return key;
}

}  // namespace

std::vector<uint64_t> duplicateEntities(const std::vector<std::shared_ptr<DraftEntity>>& entities,
                                        double tolerance) {
    const double tol = tolerance > 0.0 && std::isfinite(tolerance) ? tolerance : 1e-12;
    // Each entity filed by its exact part and its shape on a grid of the
    // tolerance: a copy made as a copy is, its numbers the same, is filed
    // with the first. Within a file, each is checked against those kept.
    struct Kept {
        std::vector<double> shape;
    };
    std::unordered_map<std::string, std::vector<Kept>> files;
    std::vector<uint64_t> copies;
    for (const auto& entity : entities) {
        if (!entity) continue;
        const auto key = keyOf(*entity);
        if (!key) continue;
        std::string file = key->exact;
        for (const double v : key->shape) {
            file += ' ';
            file += std::to_string(std::llround(v / (4.0 * tol)));
        }
        auto& kept = files[file];
        const bool copy = std::any_of(kept.begin(), kept.end(), [&](const Kept& k) {
            if (k.shape.size() != key->shape.size()) return false;
            for (size_t i = 0; i < k.shape.size(); ++i) {
                if (!(std::abs(k.shape[i] - key->shape[i]) <= tol)) return false;
            }
            return true;
        });
        if (copy) {
            copies.push_back(entity->id());
        } else {
            kept.push_back({key->shape});
        }
    }
    return copies;
}

}  // namespace hz::draft
