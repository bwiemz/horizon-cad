#include "horizon/fileio/DrawingExport.h"

#include <cmath>
#include <cstdio>
#include <memory>
#include <vector>

#include "horizon/document/Document.h"
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
#include "horizon/modeling/DrawingView.h"
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
constexpr double kDimensionOffset = 5.0;     ///< sheet distance from the edge to the dimension line
constexpr double kToleranceOffset = 8.0;     ///< sheet distance from the edge to a GD&T frame
constexpr double kDatumOffset = -8.0;        ///< opposite side, so datums clear tolerances
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
}

// Fit a 2D circle through the view's projected segments of @p edgeId.
// Returns false when the view has fewer than three distinct points for it.
bool fitProjectedCircle(const model::DrawingView& view, const topo::TopologyID& edgeId,
                        math::Vec2& outCenter, double& outRadius) {
    std::vector<math::Vec2> pts;
    for (const model::ProjectedEdge& e : view.edges) {
        if (!(e.sourceEdge == edgeId)) continue;
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

// Populate @p doc with the drawing's views, dimensions, GD&T and balloons.
void populateDrawing(doc::Document& doc, const model::Drawing& drawing) {
    for (const model::DrawingView& view : drawing.views) {
        // Map view-space coordinates onto the sheet: shift the view's lower-left
        // corner (boundsMin) to its placement, so views never overlap.
        const auto toSheet = [&view](const math::Vec2& p) {
            return math::Vec2{(p.x - view.boundsMin.x) + view.placement.x,
                              (p.y - view.boundsMin.y) + view.placement.y};
        };

        for (const model::ProjectedEdge& e : view.edges) {
            const math::Vec2 a = toSheet(e.a);
            const math::Vec2 b = toSheet(e.b);

            auto line = std::make_shared<draft::DraftLine>(a, b);
            const bool visible = e.visibility == model::ProjectedEdge::Visibility::Visible;
            line->setLayer(visible ? kVisibleLayer : kHiddenLayer);
            line->setLineType(
                static_cast<int>(visible ? draft::LineType::Continuous : draft::LineType::Hidden));
            doc.addEntity(std::move(line));
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
            const math::Vec2 onCircle{center.x + dir.x * radius, center.y + dir.y * radius};
            const math::Vec2 textPos{center.x + dir.x * (radius + kDimensionOffset),
                                     center.y + dir.y * (radius + kDimensionOffset)};

            auto leader = std::make_shared<draft::DraftLine>(toSheet(onCircle), toSheet(textPos));
            leader->setLayer(kDimensionLayer);
            doc.addEntity(std::move(leader));

            char buf[64];
            if (dim.diameter) {
                std::snprintf(buf, sizeof(buf), "\xE2\x8C\x80%.2f", dim.value * 2.0);
            } else {
                std::snprintf(buf, sizeof(buf), "R%.2f", dim.value);
            }
            auto text =
                std::make_shared<draft::DraftText>(toSheet(textPos), buf, kRadialTextHeight);
            text->setLayer(kDimensionLayer);
            doc.addEntity(std::move(text));
        }

        // Render this view's dimensions (the DXF writer decomposes them to
        // lines + text). Dimensions whose edge isn't in the view are skipped.
        for (const model::LinearDimension& dim : view.dimensions) {
            auto drafted = DrawingDimensionRenderer::render(view, dim, kDimensionOffset);
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
}

}  // namespace

bool DrawingExport::toDxf(const std::string& path, const model::Drawing& drawing) {
    doc::Document doc;  // defaults to DocumentType::Drawing
    addDrawingLayers(doc);
    populateDrawing(doc, drawing);
    return DxfFormat::save(path, doc);
}

bool DrawingExport::toDxf(const std::string& path, const model::Drawing& drawing,
                          const model::Sheet& sheet, const model::TitleBlock& titleBlock) {
    doc::Document doc;  // defaults to DocumentType::Drawing
    addDrawingLayers(doc);

    // Sheet frame + title block first, then the drawing content.
    for (auto& e : TitleBlockRenderer::renderBorder(sheet)) doc.addEntity(std::move(e));
    for (auto& e : TitleBlockRenderer::renderTitleBlock(sheet, titleBlock)) {
        doc.addEntity(std::move(e));
    }
    populateDrawing(doc, drawing);

    return DxfFormat::save(path, doc);
}

bool DrawingExport::standardViewsToDxf(const std::string& path, const topo::Solid& solid) {
    return toDxf(path, model::DrawingGenerator::standardViews(solid));
}

}  // namespace hz::io
