#include "horizon/ui/DrawingCache.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include "horizon/constraint/SketchSolver.h"
#include "horizon/document/Document.h"
#include "horizon/document/UndoStack.h"
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
#include "horizon/drafting/PlotScene.h"
#include "horizon/math/Constants.h"
#include "horizon/math/Vec4.h"
#include "horizon/render/SelectionManager.h"

namespace hz::ui {

namespace {

/// Segments in a whole circle; an arc has its share, and at least four.
constexpr int kCircleSegments = 64;
/// About this many entities to a chunk, and at most this many chunks a side.
constexpr double kEntitiesPerChunk = 256.0;
constexpr int kMaxGrid = 32;

constexpr uint32_t kSelectedColor = 0xFFFF9900;          // orange
constexpr uint32_t kUnderConstrainedColor = 0xFF00CC00;  // green
constexpr uint32_t kOverConstrainedColor = 0xFFFF0000;   // red

void vertex(std::vector<float>& v, double x, double y, double along) {
    v.push_back(static_cast<float>(x));
    v.push_back(static_cast<float>(y));
    v.push_back(0.0f);
    v.push_back(static_cast<float>(along));
}

/// A line through @p points, its dash pattern running on from one segment
/// to the next.
void pointLine(std::vector<float>& v, const std::vector<math::Vec2>& points, bool closed) {
    double along = 0.0;
    const auto segment = [&v, &along](const math::Vec2& a, const math::Vec2& b) {
        vertex(v, a.x, a.y, along);
        along += a.distanceTo(b);
        vertex(v, b.x, b.y, along);
    };
    for (size_t i = 0; i + 1 < points.size(); ++i) segment(points[i], points[i + 1]);
    if (closed && points.size() >= 2) segment(points.back(), points.front());
}

/// Separate segments, each starting its dash pattern afresh.
void separateSegments(std::vector<float>& v,
                      const std::vector<std::pair<math::Vec2, math::Vec2>>& segments) {
    for (const auto& segment : segments) {
        vertex(v, segment.first.x, segment.first.y, 0.0);
        vertex(v, segment.second.x, segment.second.y, segment.first.distanceTo(segment.second));
    }
}

/// @p count segments of the circle about @p centre, from @p start through
/// @p sweep radians.
void arcSegments(std::vector<float>& v, const math::Vec2& centre, double radius, double start,
                 double sweep, int count) {
    const double step = sweep / static_cast<double>(count);
    for (int i = 0; i < count; ++i) {
        const double a0 = start + step * static_cast<double>(i);
        const double a1 = start + step * static_cast<double>(i + 1);
        vertex(v, centre.x + radius * std::cos(a0), centre.y + radius * std::sin(a0),
               radius * step * static_cast<double>(i));
        vertex(v, centre.x + radius * std::cos(a1), centre.y + radius * std::sin(a1),
               radius * step * static_cast<double>(i + 1));
    }
}

/// The lines being built: one list of vertices for each pen and chunk.
class Buckets {
public:
    explicit Buckets(size_t chunks) : m_chunks(chunks) {}

    std::vector<float>& of(const DrawingCache::Pen& pen, size_t chunk) {
        for (size_t i = 0; i < m_pens.size(); ++i) {
            if (m_pens[i] == pen) return m_lists[i][chunk];
        }
        m_pens.push_back(pen);
        m_lists.emplace_back(m_chunks);
        return m_lists.back()[chunk];
    }

    /// Each pen's lists end to end, chunk by chunk, into a batch; the
    /// chunks' extents grown by what is in them.
    std::vector<DrawingCache::Batch> batches(std::vector<math::BoundingBox>& bounds) {
        std::vector<DrawingCache::Batch> out;
        out.reserve(m_pens.size());
        for (size_t p = 0; p < m_pens.size(); ++p) {
            DrawingCache::Batch batch;
            batch.pen = m_pens[p];
            size_t floats = 0;
            for (const auto& list : m_lists[p]) floats += list.size();
            batch.vertices.reserve(floats);
            for (size_t c = 0; c < m_chunks; ++c) {
                const auto& list = m_lists[p][c];
                if (list.empty()) continue;
                for (size_t i = 0; i + 1 < list.size(); i += 4) {
                    bounds[c].expand(math::Vec3(list[i], list[i + 1], 0.0));
                }
                const auto first = static_cast<std::uint32_t>(batch.vertices.size() / 4);
                const auto count = static_cast<std::uint32_t>(list.size() / 4);
                batch.runs.push_back({static_cast<std::uint32_t>(c), {first, count}});
                batch.vertices.insert(batch.vertices.end(), list.begin(), list.end());
            }
            if (!batch.vertices.empty()) out.push_back(std::move(batch));
        }
        return out;
    }

private:
    size_t m_chunks;
    std::vector<DrawingCache::Pen> m_pens;
    std::vector<std::vector<std::vector<float>>> m_lists;  // [pen][chunk]
};

}  // namespace

DrawingCache::Stamp DrawingCache::stampOf(const doc::Document& doc,
                                          const render::SelectionManager& selection,
                                          std::uint64_t dofRevision) const {
    Stamp stamp;
    stamp.document = &doc;
    stamp.drawing = &doc.activeDrawing();
    stamp.drawingRevision = doc.activeDrawing().revision();
    stamp.history = doc.undoStack().revision();
    stamp.selection = selection.revision();
    stamp.dof = dofRevision;
    return stamp;
}

bool DrawingCache::update(const doc::Document& doc, const render::SelectionManager& selection,
                          const cstr::DOFAnalysis& dof, std::uint64_t dofRevision) {
    const Stamp stamp = stampOf(doc, selection, dofRevision);
    if (m_built && stamp == m_stamp && doc.layerManager() == m_layers &&
        doc.activeDrawing().dimensionStyle() == m_style) {
        return false;
    }
    build(doc, selection, dof);
    m_stamp = stamp;
    return true;
}

void DrawingCache::clear() {
    m_batches.clear();
    m_texts.clear();
    m_chunkBounds.clear();
    m_built = false;
}

void DrawingCache::build(const doc::Document& doc, const render::SelectionManager& selection,
                         const cstr::DOFAnalysis& dof) {
    const draft::DraftDocument& drawing = doc.activeDrawing();
    const draft::LayerManager& layers = doc.layerManager();
    const draft::DimensionStyle& style = drawing.dimensionStyle();
    m_layers = layers;
    m_style = style;
    m_built = true;
    ++m_builds;
    m_batches.clear();
    m_texts.clear();

    // What is shown, and where: the grid is over the middles of the shown
    // entities, each entity in the chunk of its middle.
    struct Shown {
        const draft::DraftEntity* entity;
        const draft::LayerProperties* layer;
        math::Vec2 middle;
        bool placed;
    };
    std::vector<Shown> shown;
    shown.reserve(drawing.entities().size());
    math::BoundingBox middles;
    for (const auto& entity : drawing.entities()) {
        const draft::LayerProperties* layer = layers.getLayer(entity->layer());
        if (layer && !layer->visible) continue;
        const math::BoundingBox box = entity->boundingBox();
        const bool placed = box.isValid();
        const math::Vec3 middle = placed ? box.center() : math::Vec3();
        if (placed) middles.expand(middle);
        shown.push_back({entity.get(), layer, math::Vec2(middle.x, middle.y), placed});
    }
    const int grid =
        std::clamp(static_cast<int>(
                       std::ceil(std::sqrt(static_cast<double>(shown.size()) / kEntitiesPerChunk))),
                   1, kMaxGrid);
    const auto chunkOf = [&middles, grid](const Shown& s) -> size_t {
        if (!s.placed || grid == 1) return 0;
        const math::Vec3 size = middles.size();
        const auto cell = [grid](double at, double lo, double extent) {
            if (extent <= 0.0) return 0;
            return std::clamp(static_cast<int>((at - lo) / extent * grid), 0, grid - 1);
        };
        const int col = cell(s.middle.x, middles.min().x, size.x);
        const int row = cell(s.middle.y, middles.min().y, size.y);
        return static_cast<size_t>(row * grid + col);
    };

    const size_t chunks = static_cast<size_t>(grid) * static_cast<size_t>(grid);
    Buckets buckets(chunks);
    for (const Shown& s : shown) {
        const draft::DraftEntity& entity = *s.entity;
        const size_t chunk = chunkOf(s);
        const bool selected = selection.isSelected(entity.id());

        uint32_t color =
            entity.color() == 0x00000000 ? (s.layer ? s.layer->color : 0xFFFFFFFF) : entity.color();
        if (selected) {
            color = kSelectedColor;
        } else if (const auto it = dof.entityStatus.find(entity.id());
                   it != dof.entityStatus.end()) {
            if (it->second == cstr::EntityDOFStatus::Free) color = kUnderConstrainedColor;
            if (it->second == cstr::EntityDOFStatus::OverConstrained) {
                color = kOverConstrainedColor;
            }
        }
        const float width = entity.lineWidth() == 0.0
                                ? (s.layer ? static_cast<float>(s.layer->lineWidth) : 1.0f)
                                : static_cast<float>(entity.lineWidth());
        const int lineType =
            entity.lineType() == 0 ? (s.layer ? s.layer->lineType : 1) : entity.lineType();
        const Pen pen{color, width, lineType};
        const auto lines = [&]() -> std::vector<float>& { return buckets.of(pen, chunk); };

        if (const auto* line = dynamic_cast<const draft::DraftLine*>(&entity)) {
            pointLine(lines(), {line->start(), line->end()}, false);
        } else if (const auto* circle = dynamic_cast<const draft::DraftCircle*>(&entity)) {
            arcSegments(lines(), circle->center(), circle->radius(), 0.0, math::kTwoPi,
                        kCircleSegments);
        } else if (const auto* arc = dynamic_cast<const draft::DraftArc*>(&entity)) {
            double sweep = arc->endAngle() - arc->startAngle();
            if (sweep <= 0.0) sweep += math::kTwoPi;
            const int count = std::max(4, static_cast<int>(kCircleSegments * sweep / math::kTwoPi));
            arcSegments(lines(), arc->center(), arc->radius(), arc->startAngle(), sweep, count);
        } else if (const auto* rect = dynamic_cast<const draft::DraftRectangle*>(&entity)) {
            const auto c = rect->corners();
            pointLine(lines(), {c[0], c[1], c[2], c[3]}, true);
        } else if (const auto* poly = dynamic_cast<const draft::DraftPolyline*>(&entity)) {
            pointLine(lines(), poly->points(), poly->closed());
        } else if (const auto* spline = dynamic_cast<const draft::DraftSpline*>(&entity)) {
            pointLine(lines(), spline->evaluate(), false);
        } else if (const auto* hatch = dynamic_cast<const draft::DraftHatch*>(&entity)) {
            pointLine(lines(), hatch->boundary(), true);
            separateSegments(lines(), hatch->generateHatchLines());
        } else if (const auto* ellipse = dynamic_cast<const draft::DraftEllipse*>(&entity)) {
            pointLine(lines(), ellipse->evaluate(), false);
        } else if (const auto* dim = dynamic_cast<const draft::DraftDimension*>(&entity)) {
            separateSegments(lines(), dim->extensionLines(style));
            separateSegments(lines(), dim->dimensionLines(style));
            separateSegments(lines(), dim->arrowheadLines(style));
            m_texts.push_back({dim->textPosition(), dim->displayText(style), color});
        } else if (const auto* ref = dynamic_cast<const draft::DraftBlockRef*>(&entity)) {
            // The block's contents placed, as a plot draws them; what they
            // leave ByBlock is the reference's, the selection's colour too.
            const draft::PlotScene contents =
                draft::plotBlockReference(*ref, layers, style, color, width, lineType);
            for (const auto& stroke : contents.strokes) {
                pointLine(
                    buckets.of(Pen{stroke.color, static_cast<float>(stroke.width), stroke.lineType},
                               chunk),
                    stroke.points, stroke.closed);
            }
            for (const auto& text : contents.texts) {
                m_texts.push_back({text.position, text.text, text.color, text.height, text.rotation,
                                   static_cast<int>(text.alignment)});
            }
        } else if (const auto* text = dynamic_cast<const draft::DraftText*>(&entity)) {
            const auto textLines = text->lines();
            for (size_t i = 0; i < textLines.size(); ++i) {
                m_texts.push_back({text->lineBaseline(i), textLines[i], color, text->textHeight(),
                                   text->rotation(), static_cast<int>(text->alignment())});
            }
        }
    }

    m_chunkBounds.assign(chunks, math::BoundingBox());
    m_batches = buckets.batches(m_chunkBounds);
}

std::vector<bool> DrawingCache::visibleChunks(const math::Mat4& viewProjection) const {
    std::vector<bool> visible(m_chunkBounds.size(), false);
    for (size_t c = 0; c < m_chunkBounds.size(); ++c) {
        const math::BoundingBox& box = m_chunkBounds[c];
        if (!box.isValid()) continue;  // nothing in it
        // Outside when every corner is beyond one and the same side of the
        // view: -w <= x, y, z <= w inside. A chunk partly seen is drawn.
        int beyond[6] = {0, 0, 0, 0, 0, 0};
        for (int corner = 0; corner < 4; ++corner) {
            const double x = (corner & 1) ? box.max().x : box.min().x;
            const double y = (corner & 2) ? box.max().y : box.min().y;
            const math::Vec4 clip = viewProjection * math::Vec4(x, y, 0.0, 1.0);
            beyond[0] += clip.x < -clip.w ? 1 : 0;
            beyond[1] += clip.x > clip.w ? 1 : 0;
            beyond[2] += clip.y < -clip.w ? 1 : 0;
            beyond[3] += clip.y > clip.w ? 1 : 0;
            beyond[4] += clip.z < -clip.w ? 1 : 0;
            beyond[5] += clip.z > clip.w ? 1 : 0;
        }
        visible[c] = std::none_of(std::begin(beyond), std::end(beyond),
                                  [](int corners) { return corners == 4; });
    }
    return visible;
}

std::vector<DrawingCache::Range> DrawingCache::visibleRanges(const Batch& batch,
                                                             const std::vector<bool>& visible) {
    std::vector<Range> ranges;
    for (const auto& [chunk, run] : batch.runs) {
        if (chunk >= visible.size() || !visible[chunk]) continue;
        if (!ranges.empty() && ranges.back().first + ranges.back().count == run.first) {
            ranges.back().count += run.count;
        } else {
            ranges.push_back(run);
        }
    }
    return ranges;
}

}  // namespace hz::ui
