#include "horizon/fileio/DrawingExport.h"

#include <memory>

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
constexpr double kDimensionOffset = 5.0;  ///< sheet distance from the edge to the dimension line
constexpr double kToleranceOffset = 8.0;  ///< sheet distance from the edge to a GD&T frame
constexpr double kDatumOffset = -8.0;     ///< opposite side, so datums clear tolerances

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

    draft::LayerProperties border;
    border.name = TitleBlockRenderer::kBorderLayer;
    doc.layerManager().addLayer(border);

    draft::LayerProperties titleBlock;
    titleBlock.name = TitleBlockRenderer::kTitleBlockLayer;
    doc.layerManager().addLayer(titleBlock);
}

// Populate @p doc with the drawing's views, dimensions, GD&T and balloons.
void populateDrawing(doc::Document& doc, const model::Drawing& drawing) {
    for (const model::DrawingView& view : drawing.views) {
        // Map view-space coordinates onto the sheet: shift the view's lower-left
        // corner (boundsMin) to its placement, so views never overlap.
        for (const model::ProjectedEdge& e : view.edges) {
            const math::Vec2 a{(e.a.x - view.boundsMin.x) + view.placement.x,
                               (e.a.y - view.boundsMin.y) + view.placement.y};
            const math::Vec2 b{(e.b.x - view.boundsMin.x) + view.placement.x,
                               (e.b.y - view.boundsMin.y) + view.placement.y};

            auto line = std::make_shared<draft::DraftLine>(a, b);
            const bool visible = e.visibility == model::ProjectedEdge::Visibility::Visible;
            line->setLayer(visible ? kVisibleLayer : kHiddenLayer);
            line->setLineType(
                static_cast<int>(visible ? draft::LineType::Continuous : draft::LineType::Hidden));
            doc.addEntity(std::move(line));
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
