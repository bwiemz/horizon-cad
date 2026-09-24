#include "horizon/drafting/PlotScene.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "horizon/drafting/BlockDefinition.h"
#include "horizon/drafting/DimensionStyle.h"
#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftBlockRef.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftDimension.h"
#include "horizon/drafting/DraftDocument.h"
#include "horizon/drafting/DraftEllipse.h"
#include "horizon/drafting/DraftHatch.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftPolyline.h"
#include "horizon/drafting/DraftRectangle.h"
#include "horizon/drafting/DraftSpline.h"
#include "horizon/drafting/DraftText.h"
#include "horizon/drafting/Layer.h"
#include "horizon/math/Constants.h"

namespace hz::draft {

namespace {

constexpr int kMaxDepth = 16;  // blocks nested deeper than this are a cycle

/// A stroke's resolved look.
struct Style {
    uint32_t color = 0xFFFFFFFF;
    double width = 1.0;
    int lineType = 1;
};

/// Points along an arc from @p start through @p sweep radians, a degree or
/// less apart.
std::vector<math::Vec2> arcPoints(const math::Vec2& centre, double radius, double start,
                                  double sweep, bool includeEnd) {
    const int steps = std::max(2, static_cast<int>(std::ceil(std::abs(sweep) / math::kDegToRad)));
    std::vector<math::Vec2> points;
    points.reserve(static_cast<size_t>(steps) + 1);
    const int last = includeEnd ? steps : steps - 1;
    for (int i = 0; i <= last; ++i) {
        const double a = start + sweep * static_cast<double>(i) / static_cast<double>(steps);
        points.emplace_back(centre.x + radius * std::cos(a), centre.y + radius * std::sin(a));
    }
    return points;
}

class Builder {
public:
    Builder(const LayerManager& layers, const DimensionStyle& dims, PlotScene& out)
        : m_layers(layers), m_dims(dims), m_out(out) {}

    /// Plot @p e: at the top level (@p byBlock null), or inside a block
    /// reference whose resolved style @p byBlock gives what is ByBlock.
    void entity(const DraftEntity& e, const Style* byBlock, int depth) {
        const LayerProperties* layer = m_layers.getLayer(e.layer());
        if (layer != nullptr && !layer->visible) return;
        const Style s = resolve(e, layer, byBlock);

        if (const auto* ref = dynamic_cast<const DraftBlockRef*>(&e)) {
            if (depth >= kMaxDepth || !ref->definition()) return;
            const BlockDefinition& def = *ref->definition();
            for (const auto& child : def.entities) {
                if (!child) continue;
                // Placed as the reference places it: scaled and turned about
                // the base point, then moved to the insertion point.
                const auto placed = child->clone();
                placed->scale(def.basePoint, ref->uniformScale());
                placed->rotate(def.basePoint, ref->rotation());
                placed->translate(ref->insertPos() - def.basePoint);
                entity(*placed, &s, depth + 1);
            }
        } else if (const auto* line = dynamic_cast<const DraftLine*>(&e)) {
            stroke({line->start(), line->end()}, false, s);
        } else if (const auto* circle = dynamic_cast<const DraftCircle*>(&e)) {
            stroke(arcPoints(circle->center(), circle->radius(), 0.0, math::kTwoPi, false), true, s);
        } else if (const auto* arc = dynamic_cast<const DraftArc*>(&e)) {
            stroke(arcPoints(arc->center(), arc->radius(), arc->startAngle(), arc->sweepAngle(), true),
                   false, s);
        } else if (const auto* ellipse = dynamic_cast<const DraftEllipse*>(&e)) {
            stroke(ellipse->evaluate(360), false, s);
        } else if (const auto* spline = dynamic_cast<const DraftSpline*>(&e)) {
            stroke(spline->evaluate(32), false, s);
        } else if (const auto* polyline = dynamic_cast<const DraftPolyline*>(&e)) {
            stroke(polyline->points(), polyline->closed(), s);
        } else if (const auto* rect = dynamic_cast<const DraftRectangle*>(&e)) {
            const auto c = rect->corners();
            stroke({c[0], c[1], c[2], c[3]}, true, s);
        } else if (const auto* hatch = dynamic_cast<const DraftHatch*>(&e)) {
            stroke(hatch->boundary(), true, s);
            segments(hatch->generateHatchLines(), s);
        } else if (const auto* text = dynamic_cast<const DraftText*>(&e)) {
            addText(text->position(), text->text(), text->textHeight(), text->rotation(),
                    text->alignment(), s);
        } else if (const auto* dim = dynamic_cast<const DraftDimension*>(&e)) {
            segments(dim->extensionLines(m_dims), s);
            segments(dim->dimensionLines(m_dims), s);
            segments(dim->arrowheadLines(m_dims), s);
            addText(dim->textPosition(), dim->displayText(m_dims), m_dims.textHeight, 0.0,
                    TextAlignment::Center, s);
        }
    }

private:
    /// ByLayer from the layer, ByBlock from the reference; a value of its own
    /// otherwise. Inside a block, what the entity leaves unset is ByBlock.
    static Style resolve(const DraftEntity& e, const LayerProperties* layer,
                         const Style* byBlock) {
        Style inherited;
        if (byBlock != nullptr) {
            inherited = *byBlock;
        } else if (layer != nullptr) {
            inherited = {layer->color, layer->lineWidth, layer->lineType};
        }
        Style s;
        s.color = e.color() != 0 ? e.color() : inherited.color;
        s.width = e.lineWidth() > 0.0 ? e.lineWidth() : inherited.width;
        s.lineType = e.lineType() > 0 ? e.lineType() : inherited.lineType;
        return s;
    }

    void stroke(std::vector<math::Vec2> points, bool closed, const Style& s) {
        if (points.size() < 2) return;
        for (const auto& p : points) m_out.bounds.expand(math::Vec3(p.x, p.y, 0.0));
        m_out.strokes.push_back({std::move(points), closed, s.color, s.width, s.lineType});
    }

    void segments(const std::vector<std::pair<math::Vec2, math::Vec2>>& lines, const Style& s) {
        for (const auto& [a, b] : lines) stroke({a, b}, false, s);
    }

    void addText(const math::Vec2& at, const std::string& text, double height, double rotation,
                 TextAlignment alignment, const Style& s) {
        if (text.empty() || !(height > 0.0)) return;
        // Its extent, near enough for the bounds: characters about 0.6 of the
        // height wide, placed by the alignment, turned by the rotation.
        const double width = 0.6 * height * static_cast<double>(text.size());
        const double left = alignment == TextAlignment::Left     ? 0.0
                            : alignment == TextAlignment::Center ? -width / 2.0
                                                                 : -width;
        const double c = std::cos(rotation);
        const double sn = std::sin(rotation);
        for (const auto& [x, y] : {std::pair{left, 0.0}, std::pair{left + width, 0.0},
                                   std::pair{left, height}, std::pair{left + width, height}}) {
            m_out.bounds.expand(math::Vec3(at.x + x * c - y * sn, at.y + x * sn + y * c, 0.0));
        }
        m_out.texts.push_back({at, text, height, rotation, alignment, s.color});
    }

    const LayerManager& m_layers;
    const DimensionStyle& m_dims;
    PlotScene& m_out;
};

}  // namespace

PlotScene buildPlotScene(const DraftDocument& drawing, const LayerManager& layers,
                         const DimensionStyle& style) {
    PlotScene scene;
    Builder builder(layers, style, scene);
    for (const auto& entity : drawing.entities()) {
        if (entity) builder.entity(*entity, nullptr, 0);
    }
    return scene;
}

double plotWeightMm(double width) {
    if (!(width > 0.0) || std::abs(width - 1.0) < 1e-9) return 0.25;
    return std::clamp(width, 0.05, 2.11);
}

std::vector<double> plotDashMm(int lineType) {
    // The viewport's patterns (GLRenderer's line shader), in centimetres.
    switch (lineType) {
        case 2:  // Dashed
            return {5.0, 3.0};
        case 3:  // Dotted
            return {0.5, 2.5};
        case 4:  // DashDot
            return {5.0, 1.5, 0.5, 1.5};
        case 5:  // Center
            return {10.0, 1.5, 2.0, 1.5};
        case 6:  // Hidden
            return {2.5, 1.5};
        case 7:  // Phantom
            return {10.0, 1.5, 2.0, 1.5, 2.0, 1.5};
        default:
            return {};
    }
}

PlotTransform plotTransform(const PlotScene& scene, const PlotLayout& layout, bool* fits) {
    PlotTransform t;
    const double printableW = std::max(layout.paperWidthMm - 2.0 * layout.marginMm, 1.0);
    const double printableH = std::max(layout.paperHeightMm - 2.0 * layout.marginMm, 1.0);
    t.paperCentre = math::Vec2(layout.paperWidthMm / 2.0, layout.paperHeightMm / 2.0);
    const math::BoundingBox& b = scene.bounds;
    double spanW = 0.0;
    double spanH = 0.0;
    if (b.isValid()) {
        t.worldCentre = math::Vec2(b.center().x, b.center().y);
        spanW = b.max().x - b.min().x;
        spanH = b.max().y - b.min().y;
    }
    if (layout.scale > 0.0) {
        t.scale = layout.scale;
    } else {
        const double inf = std::numeric_limits<double>::infinity();
        const double sx = spanW > 0.0 ? printableW / spanW : inf;
        const double sy = spanH > 0.0 ? printableH / spanH : inf;
        t.scale = std::isfinite(std::min(sx, sy)) ? std::min(sx, sy) : 1.0;
    }
    if (fits) {
        *fits = spanW * t.scale <= printableW * (1.0 + 1e-9) &&
                spanH * t.scale <= printableH * (1.0 + 1e-9);
    }
    return t;
}

uint32_t plotColor(uint32_t argb, bool monochrome) {
    if (monochrome) return 0xFF000000u;
    const double r = static_cast<double>((argb >> 16) & 0xFFu);
    const double g = static_cast<double>((argb >> 8) & 0xFFu);
    const double b = static_cast<double>(argb & 0xFFu);
    // White, or near it, is drawn on the dark viewport; on white paper it
    // would vanish, so it plots black, as CAD plotters do.
    if (0.2126 * r + 0.7152 * g + 0.0722 * b > 230.0) return 0xFF000000u;
    return 0xFF000000u | (argb & 0xFFFFFFu);
}

}  // namespace hz::draft
