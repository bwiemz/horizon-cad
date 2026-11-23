#include "horizon/fileio/GeometricToleranceRenderer.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftText.h"
#include "horizon/math/Constants.h"
#include "horizon/modeling/DrawingView.h"
#include "horizon/modeling/GeometricTolerance.h"
#include "horizon/modeling/Naming.h"

namespace hz::io {

namespace {

using Entities = std::vector<std::shared_ptr<draft::DraftEntity>>;
using math::Vec2;

constexpr double kH = GeometricToleranceRenderer::kTextHeight;
constexpr double kCell = GeometricToleranceRenderer::kFrameHeight;
constexpr double kPad = 1.0;  ///< between a cell's side and its text
/// A character's width, in text heights: a text's height is its capitals',
/// and a sans-serif digit or capital is about 0.8 of it wide. Cells are
/// sized by it, so it errs wide.
constexpr double kCharWidth = 0.85;
constexpr double kModifierRadius = 0.5 * kH;  ///< the circle of Ⓜ or Ⓛ
constexpr double kModifierGap = 0.6;          ///< between the tolerance and its modifier
constexpr double kLeader = 10.0;              ///< from the edge to a frame's shoulder
constexpr double kShoulder = 3.0;
constexpr double kArrowLength = 2.5;
constexpr double kArrowHalfAngle = 0.35;  ///< radians: about 20°
constexpr double kTriangleHalfBase = 1.5;
constexpr double kTriangleHeight = 2.6;
constexpr double kDatumLeader = 4.0;
static_assert(kTriangleHeight + kDatumLeader + kCell == GeometricToleranceRenderer::kDatumDepth,
              "a datum symbol reaches as far as the header says");
/// Where on a circle frames leave it (up and left: a callout and a diameter
/// leave up and right), and datums (down and left), in degrees.
constexpr double kFrameAngle = 135.0;
constexpr double kDatumAngle = 225.0;

void line(Entities& out, const Vec2& a, const Vec2& b) {
    out.push_back(std::make_shared<draft::DraftLine>(a, b));
}

void circle(Entities& out, const Vec2& centre, double radius) {
    out.push_back(std::make_shared<draft::DraftCircle>(centre, radius));
}

void box(Entities& out, const Vec2& low, double width, double height) {
    const Vec2 a = low;
    const Vec2 b{low.x + width, low.y};
    const Vec2 c{low.x + width, low.y + height};
    const Vec2 d{low.x, low.y + height};
    line(out, a, b);
    line(out, b, c);
    line(out, c, d);
    line(out, d, a);
}

void text(Entities& out, const Vec2& at, const std::string& s, double height,
          draft::TextAlignment alignment) {
    auto t = std::make_shared<draft::DraftText>(at, s, height);
    t->setAlignment(alignment);
    out.push_back(std::move(t));
}

/// Characters, not bytes: "⌀" is three bytes of UTF-8 and one character wide.
std::size_t characters(const std::string& s) {
    return static_cast<std::size_t>(std::count_if(s.begin(), s.end(), [](char c) {
        return (static_cast<unsigned>(static_cast<unsigned char>(c)) & 0xC0U) != 0x80U;
    }));
}

double widthOf(const std::string& s, double height) {
    return static_cast<double>(characters(s)) * kCharWidth * height;
}

/// The baseline that centres capitals of @p height in a cell from @p bottom.
double baselineIn(double bottom, double height) {
    return bottom + (kCell - height) / 2.0;  // a text's height is its capitals'
}

Vec2 turned(const Vec2& v, double angle) {
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    return {v.x * c - v.y * s, v.x * s + v.y * c};
}

/// The characteristic's symbol, drawn in lines, circles and arcs about
/// @p o, @p a its half size.
void drawSymbol(Entities& out, model::GeometricCharacteristic c, const Vec2& o, double a) {
    using model::GeometricCharacteristic;
    const auto at = [&o, a](double x, double y) { return Vec2{o.x + x * a, o.y + y * a}; };
    switch (c) {
        case GeometricCharacteristic::Flatness:  // a parallelogram, leaning right
            line(out, at(-1.0, -0.4), at(0.6, -0.4));
            line(out, at(0.6, -0.4), at(1.0, 0.4));
            line(out, at(1.0, 0.4), at(-0.6, 0.4));
            line(out, at(-0.6, 0.4), at(-1.0, -0.4));
            break;
        case GeometricCharacteristic::Straightness:
            line(out, at(-1.0, 0.0), at(1.0, 0.0));
            break;
        case GeometricCharacteristic::Circularity:
            circle(out, o, 0.8 * a);
            break;
        case GeometricCharacteristic::Cylindricity: {
            // A circle between two parallel lines at 60°, touching it.
            circle(out, o, 0.6 * a);
            const Vec2 along{0.5, 0.8660254037844386};
            const Vec2 across{along.y, -along.x};
            for (const double side : {-1.0, 1.0}) {
                const Vec2 p = o + across * (0.6 * a * side);
                line(out, p - along * a, p + along * a);
            }
            break;
        }
        case GeometricCharacteristic::Perpendicularity:
            line(out, at(-1.0, -0.8), at(1.0, -0.8));
            line(out, at(0.0, -0.8), at(0.0, 1.0));
            break;
        case GeometricCharacteristic::Parallelism:
            line(out, at(-0.75, -0.8), at(0.05, 0.8));
            line(out, at(-0.05, -0.8), at(0.75, 0.8));
            break;
        case GeometricCharacteristic::Angularity:  // a base, and a line 30° from it
            line(out, at(-1.0, -0.7), at(1.0, -0.7));
            line(out, at(-1.0, -0.7), at(0.645, 0.25));
            break;
        case GeometricCharacteristic::Position:  // a circle, crossed past it
            circle(out, o, 0.6 * a);
            line(out, at(-1.0, 0.0), at(1.0, 0.0));
            line(out, at(0.0, -1.0), at(0.0, 1.0));
            break;
        case GeometricCharacteristic::Concentricity:
            circle(out, o, 0.9 * a);
            circle(out, o, 0.45 * a);
            break;
        case GeometricCharacteristic::Symmetry:  // three lines, the middle one longest
            line(out, at(-0.6, 0.55), at(0.6, 0.55));
            line(out, at(-1.0, 0.0), at(1.0, 0.0));
            line(out, at(-0.6, -0.55), at(0.6, -0.55));
            break;
        case GeometricCharacteristic::Runout: {  // an arrow, up and right
            const Vec2 tip = at(0.8, 0.8);
            line(out, at(-0.8, -0.8), tip);
            const Vec2 back{-0.7071067811865476, -0.7071067811865476};
            line(out, tip, tip + turned(back, 0.4) * (0.7 * a));
            line(out, tip, tip + turned(back, -0.4) * (0.7 * a));
            break;
        }
        case GeometricCharacteristic::Profile:  // of a surface: a closed half circle
            out.push_back(
                std::make_shared<draft::DraftArc>(at(0.0, -0.45), 0.9 * a, 0.0, math::kPi));
            line(out, at(-0.9, -0.45), at(0.9, -0.45));
            break;
    }
}

/// What a frame's tolerance cell says: its ⌀, and the tolerance as the
/// sheet states lengths.
std::string toleranceText(const model::FeatureControlFrame& f,
                          const std::function<std::string(double)>& value) {
    return (f.diameterZone ? "\xE2\x8C\x80" : "") + value(f.tolerance);
}

/// The widths of a frame's cells: its symbol's, its tolerance's, each datum's.
std::vector<double> cellsOf(const model::FeatureControlFrame& f, const std::string& tolerance) {
    std::vector<double> cells{kCell};
    double width = kPad + widthOf(tolerance, kH) + kPad;
    if (f.modifier != model::MaterialCondition::None) {
        width += kModifierGap + 2.0 * kModifierRadius;
    }
    cells.push_back(std::max(kCell, width));
    for (const std::string& label : f.datumRefs) {
        cells.push_back(std::max(kCell, 2.0 * kPad + widthOf(label, kH)));
    }
    return cells;
}

double widthOf(const std::vector<double>& cells) {
    double width = 0.0;
    for (const double c : cells) width += c;
    return width;
}

/// A frame, its lower-left corner at @p low.
void drawFrame(Entities& out, const model::FeatureControlFrame& f, const std::string& tolerance,
               const std::vector<double>& cells, const Vec2& low) {
    box(out, low, widthOf(cells), kCell);
    double x = low.x;
    for (std::size_t i = 0; i + 1 < cells.size(); ++i) {
        x += cells[i];
        line(out, {x, low.y}, {x, low.y + kCell});
    }
    drawSymbol(out, f.characteristic, {low.x + kCell / 2.0, low.y + kCell / 2.0}, 0.3 * kCell);

    const double left = low.x + cells[0] + kPad;
    text(out, {left, baselineIn(low.y, kH)}, tolerance, kH, draft::TextAlignment::Left);
    if (f.modifier != model::MaterialCondition::None) {
        const Vec2 centre{left + widthOf(tolerance, kH) + kModifierGap + kModifierRadius,
                          low.y + kCell / 2.0};
        circle(out, centre, kModifierRadius);
        constexpr double kLetter = 1.2 * kModifierRadius;
        text(out, {centre.x, centre.y - kLetter / 2.0},
             f.modifier == model::MaterialCondition::MMC ? "M" : "L", kLetter,
             draft::TextAlignment::Center);
    }
    x = low.x + cells[0] + cells[1];
    for (std::size_t i = 0; i < f.datumRefs.size(); ++i) {
        const double width = cells[i + 2];
        text(out, {x + width / 2.0, baselineIn(low.y, kH)}, f.datumRefs[i], kH,
             draft::TextAlignment::Center);
        x += width;
    }
}

/// An edge as the view shows it on the sheet: a circle seen end-on, or a
/// line from @p a to @p b (a straight edge, or the outermost piece of a
/// curve).
struct Shape {
    bool circle = false;
    Vec2 centre;
    double radius = 0.0;
    Vec2 a;
    Vec2 b;
};

/// A point on an edge and the way out of the view from it.
struct Anchor {
    Vec2 at;
    Vec2 out;
};

std::string keyOf(const std::string& tag) {
    return model::withoutPiece(model::logicalEdge(tag));
}

std::optional<Shape> shapeOf(const model::DrawingView& view, const std::string& key,
                             const Vec2& viewCentre) {
    std::vector<std::pair<Vec2, Vec2>> pieces;
    for (const model::ProjectedEdge& e : view.edges) {
        if (keyOf(e.sourceEdge.tag()) == key)
            pieces.emplace_back(view.toSheet(e.a), view.toSheet(e.b));
    }
    if (pieces.empty()) return std::nullopt;
    std::vector<Vec2> points;
    for (const auto& [a, b] : pieces) {
        points.push_back(a);
        points.push_back(b);
    }
    Vec2 centre{0.0, 0.0};
    for (const Vec2& p : points) centre = centre + p;
    centre = centre / static_cast<double>(points.size());

    // Its principal direction: along it, how far it runs; across it, how far
    // it strays from a line.
    double sxx = 0.0;
    double syy = 0.0;
    double sxy = 0.0;
    for (const Vec2& p : points) {
        const Vec2 d = p - centre;
        sxx += d.x * d.x;
        syy += d.y * d.y;
        sxy += d.x * d.y;
    }
    const double angle = 0.5 * std::atan2(2.0 * sxy, sxx - syy);
    const Vec2 along{std::cos(angle), std::sin(angle)};
    const Vec2 across{-along.y, along.x};
    double low = std::numeric_limits<double>::max();
    double high = std::numeric_limits<double>::lowest();
    double stray = 0.0;
    for (const Vec2& p : points) {
        const double t = (p - centre).dot(along);
        low = std::min(low, t);
        high = std::max(high, t);
        stray = std::max(stray, std::abs((p - centre).dot(across)));
    }
    Shape shape;
    if (stray <= std::max(1e-6, 1e-3 * (high - low))) {
        shape.a = centre + along * low;
        shape.b = centre + along * high;
        return shape;
    }
    // A circle: its points as far from their centre as each other.
    double radius = 0.0;
    for (const Vec2& p : points) radius += p.distanceTo(centre);
    radius /= static_cast<double>(points.size());
    double off = 0.0;
    for (const Vec2& p : points) off = std::max(off, std::abs(p.distanceTo(centre) - radius));
    if (pieces.size() >= 3 && radius > 1e-6 && off <= 0.05 * radius) {
        shape.circle = true;
        shape.centre = centre;
        shape.radius = radius;
        return shape;
    }
    // Another curve (an arc, a spline): its piece furthest out of the view.
    const auto outermost =
        std::max_element(pieces.begin(), pieces.end(), [&](const auto& l, const auto& r) {
            return ((l.first + l.second) * 0.5).distanceTo(viewCentre) <
                   ((r.first + r.second) * 0.5).distanceTo(viewCentre);
        });
    shape.a = outermost->first;
    shape.b = outermost->second;
    return shape;
}

/// The point @p t of the way along a line shape, and out of the view from it.
Anchor along(const Shape& shape, double t, const Vec2& viewCentre) {
    const Vec2 at = shape.a + (shape.b - shape.a) * t;
    const Vec2 run = shape.b - shape.a;
    Vec2 out;
    if (run.length() < 1e-9) {
        // An edge seen end-on, as a point: out from the view's middle.
        out = at - viewCentre;
        if (out.length() < 1e-9) out = {0.0, 1.0};
    } else {
        out = {-run.y, run.x};
        if (out.dot(at - viewCentre) < 0.0) out = -out;
    }
    return {at, out.normalized()};
}

/// The point at @p degrees round a circle shape, and out from it.
Anchor around(const Shape& shape, double degrees) {
    const Vec2 out{std::cos(degrees * math::kDegToRad), std::sin(degrees * math::kDegToRad)};
    return {shape.centre + out * shape.radius, out};
}

/// A datum feature symbol: a triangle on the edge at @p anchor, a leader out
/// from it, and the letter in a box at its end.
void drawDatum(Entities& out, const std::string& label, const Anchor& anchor) {
    const Vec2 n = anchor.out;
    const Vec2 t{-n.y, n.x};
    const Vec2 b1 = anchor.at + t * kTriangleHalfBase;
    const Vec2 b2 = anchor.at - t * kTriangleHalfBase;
    const Vec2 apex = anchor.at + n * kTriangleHeight;
    line(out, b1, b2);
    line(out, b2, apex);
    line(out, apex, b1);
    const Vec2 end = apex + n * kDatumLeader;
    line(out, apex, end);
    // The box's middle, so that the leader meets the middle of its side.
    const double reach = (kCell / 2.0) / std::max(std::abs(n.x), std::abs(n.y));
    const Vec2 middle = end + n * reach;
    box(out, {middle.x - kCell / 2.0, middle.y - kCell / 2.0}, kCell, kCell);
    text(out, {middle.x, middle.y - kH / 2.0}, label, kH, draft::TextAlignment::Center);
}

struct Framed {
    const model::FeatureControlFrame* frame;
    std::string tolerance;
    std::vector<double> cells;
};

/// Frames on one leader from @p anchor: out to an elbow, a shoulder, and the
/// frames beside it, one under another.
void drawOnLeader(Entities& out, const std::vector<Framed>& frames, const Anchor& anchor) {
    const Vec2 elbow = anchor.at + anchor.out * kLeader;
    const double side = anchor.out.x < -1e-9 ? -1.0 : 1.0;
    const Vec2 shoulder{elbow.x + side * kShoulder, elbow.y};
    line(out, anchor.at, elbow);
    line(out, elbow, shoulder);
    // Its arrow, at the edge.
    line(out, anchor.at, anchor.at + turned(anchor.out, kArrowHalfAngle) * kArrowLength);
    line(out, anchor.at, anchor.at + turned(anchor.out, -kArrowHalfAngle) * kArrowLength);
    double bottom = shoulder.y - kCell / 2.0;
    for (const Framed& f : frames) {
        const double width = widthOf(f.cells);
        drawFrame(out, *f.frame, f.tolerance, f.cells,
                  {side > 0.0 ? shoulder.x : shoulder.x - width, bottom});
        bottom -= kCell;
    }
}

}  // namespace

std::vector<std::shared_ptr<draft::DraftEntity>> GeometricToleranceRenderer::render(
    const model::DrawingView& view, const std::function<std::string(double)>& value,
    const Hangers& hangers) {
    Entities out;
    const Vec2 viewCentre = view.toSheet((view.boundsMin + view.boundsMax) * 0.5);

    // Each edge's frames and datums, in the order they were given.
    std::vector<std::string> keys;
    const auto keyAt = [&keys](const std::string& key) {
        const auto it = std::find(keys.begin(), keys.end(), key);
        if (it != keys.end()) return static_cast<std::size_t>(it - keys.begin());
        keys.push_back(key);
        return keys.size() - 1;
    };
    std::vector<std::vector<Framed>> frames;
    std::vector<std::vector<const model::DatumFeature*>> datums;
    const auto grow = [&] {
        frames.resize(keys.size());
        datums.resize(keys.size());
    };
    for (const model::FeatureControlFrame& f : view.tolerances) {
        const std::size_t k = keyAt(keyOf(f.feature.tag()));
        grow();
        std::string tolerance = toleranceText(f, value);
        std::vector<double> cells = cellsOf(f, tolerance);
        frames[k].push_back({&f, std::move(tolerance), std::move(cells)});
    }
    for (const model::DatumFeature& d : view.datums) {
        const std::size_t k = keyAt(keyOf(d.feature.tag()));
        grow();
        datums[k].push_back(&d);
    }

    for (std::size_t k = 0; k < keys.size(); ++k) {
        const std::optional<Shape> shape = shapeOf(view, keys[k], viewCentre);
        if (!shape) continue;
        std::size_t firstDatum = 0;
        const auto hanger = hangers.find(keys[k]);
        if (!frames[k].empty() && hanger != hangers.end()) {
            // Under the hole's callout, one under another; its datum under them.
            double bottom = hanger->second.y - kCell;
            for (const Framed& f : frames[k]) {
                drawFrame(out, *f.frame, f.tolerance, f.cells, {hanger->second.x, bottom});
                bottom -= kCell;
            }
            if (!datums[k].empty()) {
                drawDatum(out, datums[k].front()->label,
                          {{hanger->second.x + kCell / 2.0, bottom + kCell}, {0.0, -1.0}});
                firstDatum = 1;
            }
        } else if (!frames[k].empty()) {
            drawOnLeader(
                out, frames[k],
                shape->circle ? around(*shape, kFrameAngle) : along(*shape, 0.5, viewCentre));
        }
        // Datums along the edge, clear of its frames' leader at its middle.
        for (std::size_t i = firstDatum; i < datums[k].size(); ++i) {
            const auto n = static_cast<double>(i - firstDatum);
            const Anchor anchor =
                shape->circle
                    ? around(*shape, kDatumAngle + 30.0 * n)
                    : along(*shape, std::max(0.05, (frames[k].empty() ? 0.5 : 0.25) - 0.15 * n),
                            viewCentre);
            drawDatum(out, datums[k][i]->label, anchor);
        }
    }
    return out;
}

}  // namespace hz::io
