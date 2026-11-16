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
/// drawn from either end is the same path. A closed one is read from its
/// lowest corner, either way round: the same outline drawn from another
/// corner, or clockwise, is the same outline.
std::vector<Vec2> eitherWay(const std::vector<Vec2>& points, bool closed) {
    const size_t n = points.size();
    if (n < 2) return points;
    if (!closed) {
        std::vector<Vec2> back(points.rbegin(), points.rend());
        return std::lexicographical_compare(back.begin(), back.end(), points.begin(), points.end(),
                                            before)
                   ? back
                   : points;
    }
    const size_t lowest = static_cast<size_t>(
        std::min_element(points.begin(), points.end(), before) - points.begin());
    std::vector<Vec2> forward;
    std::vector<Vec2> backward;
    forward.reserve(n);
    backward.reserve(n);
    for (size_t i = 0; i < n; ++i) {
        forward.push_back(points[(lowest + i) % n]);
        backward.push_back(points[(lowest + n - i) % n]);
    }
    return std::lexicographical_compare(backward.begin(), backward.end(), forward.begin(),
                                        forward.end(), before)
               ? backward
               : forward;
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
        const auto corners = rect->corners();
        for (const Vec2& p : eitherWay({corners.begin(), corners.end()}, true)) put(shape, p);
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
    // Each entity kept is filed by its exact part and by where its shape
    // starts (its first two numbers) on a grid of the tolerance; each is
    // looked for in its own cell and the eight around it, so one within the
    // tolerance across a cell's edge is found too, and checked number by
    // number against those filed there.
    struct Kept {
        std::vector<double> shape;
    };
    const double cell = 4.0 * tol;
    const auto cellOf = [cell](double v) { return static_cast<long long>(std::floor(v / cell)); };
    const auto fileOf = [](const std::string& exact, long long cx, long long cy) {
        return exact + ' ' + std::to_string(cx) + ' ' + std::to_string(cy);
    };
    std::unordered_map<std::string, std::vector<Kept>> files;
    std::vector<uint64_t> copies;
    for (const auto& entity : entities) {
        if (!entity) continue;
        const auto key = keyOf(*entity);
        if (!key || key->shape.size() < 2) continue;
        const auto& shape = key->shape;
        const long long cx = cellOf(shape[0]);
        const long long cy = cellOf(shape[1]);
        const auto same = [&](const Kept& k) {
            if (k.shape.size() != shape.size()) return false;
            for (size_t i = 0; i < shape.size(); ++i) {
                if (!(std::abs(k.shape[i] - shape[i]) <= tol)) return false;
            }
            return true;
        };
        bool copy = false;
        for (long long dx = -1; dx <= 1 && !copy; ++dx) {
            for (long long dy = -1; dy <= 1 && !copy; ++dy) {
                const auto found = files.find(fileOf(key->exact, cx + dx, cy + dy));
                copy = found != files.end() &&
                       std::any_of(found->second.begin(), found->second.end(), same);
            }
        }
        if (copy) {
            copies.push_back(entity->id());
        } else {
            files[fileOf(key->exact, cx, cy)].push_back({shape});
        }
    }
    return copies;
}

}  // namespace hz::draft
