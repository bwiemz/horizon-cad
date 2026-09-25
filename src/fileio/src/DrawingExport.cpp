#include "horizon/fileio/DrawingExport.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "horizon/document/Document.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftLinearDimension.h"
#include "horizon/drafting/DraftText.h"
#include "horizon/drafting/Layer.h"
#include "horizon/drafting/LineType.h"
#include "horizon/fileio/BalloonRenderer.h"
#include "horizon/fileio/DrawingDimensionRenderer.h"
#include "horizon/fileio/DxfFormat.h"
#include "horizon/fileio/GeometricToleranceRenderer.h"
#include "horizon/fileio/TitleBlockRenderer.h"
#include "horizon/modeling/DrawingProjection.h"
#include "horizon/modeling/DrawingView.h"
#include "horizon/modeling/Naming.h"
#include "horizon/modeling/Sheet.h"
#include "horizon/modeling/TitleBlock.h"

namespace hz::io {

namespace {

constexpr char kVisibleLayer[] = "Visible";
constexpr char kHiddenLayer[] = "Hidden";
constexpr char kDimensionLayer[] = "Dimensions";
constexpr char kToleranceLayer[] = "Tolerances";
constexpr char kBalloonLayer[] = "Balloons";
constexpr char kSectionLayer[] = "Section";  ///< cut-profile boundaries of section views
constexpr char kHatchLayer[] = "Hatch";      ///< cross-hatching inside cut profiles
/// Captions under sections and details, and their marks on the views they
/// were taken from (Phase 149).
constexpr char kViewLabelLayer[] = "ViewLabels";
/// Centre marks on holes and bosses, and their axes (Phase 149).
constexpr char kCentreLineLayer[] = "CentreLines";
constexpr double kCentreOverrun = 2.0;    ///< how far a centre line runs past the outline, on paper
constexpr double kCaptionHeight = 3.5;    ///< ISO 3098 for A3 and A4
constexpr double kArrowLength = 7.0;      ///< a section's viewing arrows, on the sheet
constexpr double kMarkOverrun = 5.0;      ///< how far a cut's line runs past its view
constexpr double kDimensionOffset = 5.0;  ///< sheet distance from the edge to the dimension line
constexpr double kToleranceOffset = 8.0;  ///< sheet distance from the edge to a GD&T frame
constexpr double kDatumOffset = -8.0;     ///< opposite side, so datums clear tolerances
constexpr double kRadialTextHeight = 3.0;

void addDrawingLayers(doc::Document& doc) {
    draft::LayerProperties visible;
    visible.name = kVisibleLayer;
    doc.layerManager().addLayer(visible);

    draft::LayerProperties hidden;
    hidden.name = kHiddenLayer;
    doc.layerManager().addLayer(hidden);

    draft::LayerProperties dimensions;
    dimensions.name = kDimensionLayer;
    doc.layerManager().addLayer(dimensions);

    draft::LayerProperties tolerances;
    tolerances.name = kToleranceLayer;
    doc.layerManager().addLayer(tolerances);

    draft::LayerProperties balloons;
    balloons.name = kBalloonLayer;
    doc.layerManager().addLayer(balloons);

    draft::LayerProperties section;
    section.name = kSectionLayer;
    doc.layerManager().addLayer(section);

    draft::LayerProperties hatch;
    hatch.name = kHatchLayer;
    doc.layerManager().addLayer(hatch);

    draft::LayerProperties border;
    border.name = TitleBlockRenderer::kBorderLayer;
    doc.layerManager().addLayer(border);

    draft::LayerProperties titleBlock;
    titleBlock.name = TitleBlockRenderer::kTitleBlockLayer;
    doc.layerManager().addLayer(titleBlock);

    draft::LayerProperties viewLabels;
    viewLabels.name = kViewLabelLayer;
    doc.layerManager().addLayer(viewLabels);

    draft::LayerProperties centreLines;
    centreLines.name = kCentreLineLayer;
    doc.layerManager().addLayer(centreLines);

    draft::LayerProperties partsList;
    partsList.name = TitleBlockRenderer::kPartsListLayer;
    doc.layerManager().addLayer(partsList);
}

void addLabelLine(doc::Document& doc, const math::Vec2& a, const math::Vec2& b,
                  draft::LineType type = draft::LineType::Continuous) {
    auto line = std::make_shared<draft::DraftLine>(a, b);
    line->setLayer(kViewLabelLayer);
    line->setLineType(static_cast<int>(type));
    doc.addEntity(std::move(line));
}

void addLabelText(doc::Document& doc, const math::Vec2& at, const std::string& text,
                  double height) {
    auto t = std::make_shared<draft::DraftText>(at, text, height);
    t->setLayer(kViewLabelLayer);
    t->setAlignment(draft::TextAlignment::Center);
    doc.addEntity(std::move(t));
}

/// An arrow on the sheet whose head is at @p head, pointing along @p dir
/// (a unit vector).
void addArrow(doc::Document& doc, const math::Vec2& head, const math::Vec2& dir) {
    const math::Vec2 tail{head.x - dir.x * kArrowLength, head.y - dir.y * kArrowLength};
    addLabelLine(doc, tail, head);
    const double c = std::cos(0.35);
    const double s = std::sin(0.35);
    for (const double side : {1.0, -1.0}) {
        const math::Vec2 back{-(dir.x * c - side * dir.y * s), -(side * dir.x * s + dir.y * c)};
        addLabelLine(doc, head, {head.x + back.x * 3.0, head.y + back.y * 3.0});
    }
}

/// The caption under @p view, and its mark on its source: a section's cut,
/// with arrows the way it looks, or a detail's circle.
void addViewLabels(doc::Document& doc, const model::Drawing& drawing, size_t index) {
    const model::DrawingView& view = drawing.views[index];
    if (view.label.empty() || view.role == model::ViewRole::Projection) return;
    const bool section = view.role == model::ViewRole::Section;
    const bool hasSource = view.source >= 0 &&
                           static_cast<size_t>(view.source) < drawing.views.size() &&
                           static_cast<size_t>(view.source) != index;

    std::string caption =
        section ? "SECTION " + view.label + "-" + view.label : "DETAIL " + view.label;
    if (!section || (hasSource && std::abs(drawing.views[static_cast<size_t>(view.source)].scale -
                                           view.scale) > 1e-12)) {
        caption += " (" + model::DrawingGenerator::scaleName(view.scale) + ")";
    }
    const auto [low, high] = view.sheetFootprint();
    addLabelText(doc, {(low.x + high.x) / 2.0, low.y + 1.5}, caption, kCaptionHeight);
    if (!hasSource) return;

    const model::DrawingView& from = drawing.views[static_cast<size_t>(view.source)];
    if (!section) {
        const math::Vec2 centre = from.toSheet(view.detailCenter);
        const double radius = view.detailRadius * from.scale;
        auto circle = std::make_shared<draft::DraftCircle>(centre, radius);
        circle->setLayer(kViewLabelLayer);
        doc.addEntity(std::move(circle));
        const double r = radius + 2.0;
        addLabelText(doc, {centre.x + r * 0.7071, centre.y + r * 0.7071}, view.label,
                     kCaptionHeight);
        return;
    }

    // The cut, seen on the source: the plane meets its view plane in a line.
    const math::Vec3 point = view.projection.origin;
    const math::Vec3 sight = view.projection.dir.normalized();  // the section looks this way
    const math::Vec3 along = sight.cross(from.projection.dir);  // in both planes
    const math::Vec2 p = model::DrawingProjection::toView(from.projection, point);
    const math::Vec2 q = model::DrawingProjection::toView(from.projection, point + along);
    const math::Vec2 s = model::DrawingProjection::toView(from.projection, point + sight);
    math::Vec2 d{q.x - p.x, q.y - p.y};
    math::Vec2 look{s.x - p.x, s.y - p.y};
    const double dl = std::hypot(d.x, d.y);
    const double ll = std::hypot(look.x, look.y);
    if (dl < 1e-9 || ll < 1e-9) return;  // the cut is not seen edge-on there
    d = {d.x / dl, d.y / dl};
    look = {look.x / ll, look.y / ll};

    // Across the source's bounds, and a little past them.
    const double over = kMarkOverrun / from.scale;
    double tLo = -std::numeric_limits<double>::infinity();
    double tHi = std::numeric_limits<double>::infinity();
    const double lo[2] = {from.boundsMin.x - over, from.boundsMin.y - over};
    const double hi[2] = {from.boundsMax.x + over, from.boundsMax.y + over};
    const double origin[2] = {p.x, p.y};
    const double step[2] = {d.x, d.y};
    for (int axis = 0; axis < 2; ++axis) {
        if (std::abs(step[axis]) < 1e-12) {
            if (origin[axis] < lo[axis] || origin[axis] > hi[axis]) return;  // misses the view
            continue;
        }
        const double t0 = (lo[axis] - origin[axis]) / step[axis];
        const double t1 = (hi[axis] - origin[axis]) / step[axis];
        tLo = std::max(tLo, std::min(t0, t1));
        tHi = std::min(tHi, std::max(t0, t1));
    }
    if (!(tLo < tHi)) return;
    const math::Vec2 a = from.toSheet({p.x + d.x * tLo, p.y + d.y * tLo});
    const math::Vec2 b = from.toSheet({p.x + d.x * tHi, p.y + d.y * tHi});
    addLabelLine(doc, a, b, draft::LineType::Center);
    for (const math::Vec2& end : {a, b}) {
        addArrow(doc, end, look);
        addLabelText(doc,
                     {end.x - look.x * (kArrowLength + 3.0),
                      end.y - look.y * (kArrowLength + 3.0) - kCaptionHeight / 2.0},
                     view.label, kCaptionHeight);
    }
}

// Fit a 2D circle through the view's projected segments of @p edgeId.
// Returns false when the view has fewer than three distinct points for it.
bool fitProjectedCircle(const model::DrawingView& view, const topo::TopologyID& edgeId,
                        math::Vec2& outCenter, double& outRadius) {
    // Every piece of the curve: a circle is one edge of many chords, each
    // named as part of it.
    const std::string logical = model::logicalEdge(edgeId.tag());
    std::vector<math::Vec2> pts;
    for (const model::ProjectedEdge& e : view.edges) {
        if (!(e.sourceEdge == edgeId) && model::logicalEdge(e.sourceEdge.tag()) != logical)
            continue;
        pts.push_back(e.a);
        pts.push_back(e.b);
    }
    if (pts.size() < 3) return false;

    // Three spread samples → circumcenter.
    const math::Vec2 a = pts[0];
    const math::Vec2 b = pts[pts.size() / 3];
    const math::Vec2 c = pts[(2 * pts.size()) / 3];
    const double d1x = b.x - a.x;
    const double d1y = b.y - a.y;
    const double d2x = c.x - a.x;
    const double d2y = c.y - a.y;
    const double d11 = d1x * d1x + d1y * d1y;
    const double d22 = d2x * d2x + d2y * d2y;
    const double cross = d1x * d2y - d1y * d2x;
    if (std::abs(cross) < 1e-12) return false;
    const double cx = a.x + (d2y * d11 - d1y * d22) / (2.0 * cross);
    const double cy = a.y + (d1x * d22 - d2x * d11) / (2.0 * cross);
    outCenter = math::Vec2{cx, cy};
    outRadius = std::hypot(a.x - cx, a.y - cy);
    return outRadius > 1e-12;
}

}  // namespace

const std::vector<std::string>& DrawingExport::layers() {
    static const std::vector<std::string> names{kVisibleLayer,
                                                kHiddenLayer,
                                                kDimensionLayer,
                                                kToleranceLayer,
                                                kBalloonLayer,
                                                kSectionLayer,
                                                kHatchLayer,
                                                TitleBlockRenderer::kBorderLayer,
                                                TitleBlockRenderer::kTitleBlockLayer,
                                                kViewLabelLayer,
                                                kCentreLineLayer,
                                                TitleBlockRenderer::kPartsListLayer};
    return names;
}

void DrawingExport::populate(doc::Document& doc, const model::Drawing& drawing,
                             const model::Sheet* sheet, const model::TitleBlock* titleBlock,
                             const model::PartsList* partsList) {
    addDrawingLayers(doc);
    // Sheet frame and title block first, then the drawing content.
    if (sheet != nullptr) {
        for (auto& e : TitleBlockRenderer::renderBorder(*sheet)) doc.addEntity(std::move(e));
        if (titleBlock != nullptr) {
            for (auto& e : TitleBlockRenderer::renderTitleBlock(*sheet, *titleBlock)) {
                doc.addEntity(std::move(e));
            }
            if (partsList != nullptr) {
                for (auto& e :
                     TitleBlockRenderer::renderPartsList(*sheet, *titleBlock, *partsList)) {
                    doc.addEntity(std::move(e));
                }
            }
        }
    }
    for (const model::DrawingView& view : drawing.views) {
        // Onto the sheet: each view's lower-left corner at its placement, at
        // its scale (DrawingView::toSheet), so views never overlap.
        const auto toSheet = [&view](const math::Vec2& p) { return view.toSheet(p); };

        for (const model::ProjectedEdge& e : view.edges) {
            const bool visible = e.visibility == model::ProjectedEdge::Visibility::Visible;
            if (!visible && !view.showHidden) continue;
            if (e.kind == model::ProjectedEdge::Kind::Tangent && !view.showTangentEdges) continue;
            const math::Vec2 a = toSheet(e.a);
            const math::Vec2 b = toSheet(e.b);

            auto line = std::make_shared<draft::DraftLine>(a, b);
            line->setLayer(visible ? kVisibleLayer : kHiddenLayer);
            line->setLineType(
                static_cast<int>(visible ? draft::LineType::Continuous : draft::LineType::Hidden));
            doc.addEntity(std::move(line));
        }

        // Centre lines, a little past the outline on paper, whatever the scale.
        if (view.showCentreLines) {
            for (const auto& segment : view.centreLines) {
                const math::Vec2 a = toSheet(segment.first);
                const math::Vec2 b = toSheet(segment.second);
                const double length = std::hypot(b.x - a.x, b.y - a.y);
                if (length < 1e-9) continue;
                const math::Vec2 run{(b.x - a.x) / length * kCentreOverrun,
                                     (b.y - a.y) / length * kCentreOverrun};
                auto line = std::make_shared<draft::DraftLine>(
                    math::Vec2{a.x - run.x, a.y - run.y}, math::Vec2{b.x + run.x, b.y + run.y});
                line->setLayer(kCentreLineLayer);
                line->setLineType(static_cast<int>(draft::LineType::Center));
                doc.addEntity(std::move(line));
            }
        }

        // Section views: cut-profile loops and hatch lines (Phase 61).
        for (const auto& loop : view.sectionLoops) {
            for (size_t i = 0; i < loop.size(); ++i) {
                auto line = std::make_shared<draft::DraftLine>(
                    toSheet(loop[i]), toSheet(loop[(i + 1) % loop.size()]));
                line->setLayer(kSectionLayer);
                line->setLineType(static_cast<int>(draft::LineType::Continuous));
                doc.addEntity(std::move(line));
            }
        }
        for (const auto& seg : view.sectionHatch) {
            auto line = std::make_shared<draft::DraftLine>(toSheet(seg.first), toSheet(seg.second));
            line->setLayer(kHatchLayer);
            line->setLineType(static_cast<int>(draft::LineType::Continuous));
            doc.addEntity(std::move(line));
        }

        // Radial dimensions (Phase 61): leader from the projected circle at
        // 45°, annotated "R…" or "⌀…".
        for (const model::RadialDimension& dim : view.radialDimensions) {
            math::Vec2 center;
            double radius = 0.0;
            if (!fitProjectedCircle(view, dim.edge, center, radius)) continue;

            const double kInvSqrt2 = 0.7071067811865476;
            const math::Vec2 dir{kInvSqrt2, kInvSqrt2};
            // The leader's length is the sheet's, whatever the view's scale.
            const math::Vec2 onCircle =
                toSheet({center.x + dir.x * radius, center.y + dir.y * radius});
            const math::Vec2 textPos{onCircle.x + dir.x * kDimensionOffset,
                                     onCircle.y + dir.y * kDimensionOffset};

            auto leader = std::make_shared<draft::DraftLine>(onCircle, textPos);
            leader->setLayer(kDimensionLayer);
            doc.addEntity(std::move(leader));

            char buf[64];
            if (dim.diameter) {
                std::snprintf(buf, sizeof(buf), "\xE2\x8C\x80%.2f", dim.value * 2.0);
            } else {
                std::snprintf(buf, sizeof(buf), "R%.2f", dim.value);
            }
            auto text = std::make_shared<draft::DraftText>(textPos, buf, kRadialTextHeight);
            text->setLayer(kDimensionLayer);
            doc.addEntity(std::move(text));
        }

        // Render this view's dimensions (the DXF writer decomposes them to
        // lines + text). Dimensions whose edge isn't in the view are skipped.
        for (const model::LinearDimension& dim : view.dimensions) {
            auto drafted = DrawingDimensionRenderer::render(view, dim, kDimensionOffset,
                                                            doc.draftDocument().dimensionStyle());
            if (drafted) {
                drafted->setLayer(kDimensionLayer);
                doc.addEntity(std::move(drafted));
            }
        }

        // Render this view's GD&T annotations as text near the toleranced
        // feature. Frames/datums whose feature isn't in the view are skipped.
        for (const model::FeatureControlFrame& frame : view.tolerances) {
            auto drafted = GeometricToleranceRenderer::render(view, frame, kToleranceOffset);
            if (drafted) {
                drafted->setLayer(kToleranceLayer);
                doc.addEntity(std::move(drafted));
            }
        }
        for (const model::DatumFeature& datum : view.datums) {
            auto drafted = GeometricToleranceRenderer::render(view, datum, kDatumOffset);
            if (drafted) {
                drafted->setLayer(kToleranceLayer);
                doc.addEntity(std::move(drafted));
            }
        }

        // Render this view's BOM balloons: each is a leader + circle + number.
        // Balloons whose feature isn't in the view render to nothing.
        for (const model::DrawingBalloon& balloon : view.balloons) {
            for (auto& entity : BalloonRenderer::render(view, balloon)) {
                entity->setLayer(kBalloonLayer);
                doc.addEntity(std::move(entity));
            }
        }
    }
    // Sections' and details' captions, and their marks on their sources.
    for (size_t i = 0; i < drawing.views.size(); ++i) addViewLabels(doc, drawing, i);
}

bool DrawingExport::toDxf(const std::string& path, const model::Drawing& drawing) {
    doc::Document doc;  // defaults to DocumentType::Drawing
    populate(doc, drawing);
    return DxfFormat::save(path, doc);
}

bool DrawingExport::toDxf(const std::string& path, const model::Drawing& drawing,
                          const model::Sheet& sheet, const model::TitleBlock& titleBlock) {
    doc::Document doc;  // defaults to DocumentType::Drawing
    populate(doc, drawing, &sheet, &titleBlock);
    return DxfFormat::save(path, doc);
}

bool DrawingExport::standardViewsToDxf(const std::string& path, const topo::Solid& solid) {
    return toDxf(path, model::DrawingGenerator::standardViews(solid));
}

}  // namespace hz::io
