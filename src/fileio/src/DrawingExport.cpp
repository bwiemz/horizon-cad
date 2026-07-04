#include "horizon/fileio/DrawingExport.h"

#include <memory>

#include "horizon/document/Document.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftLinearDimension.h"
#include "horizon/drafting/DraftText.h"
#include "horizon/drafting/Layer.h"
#include "horizon/drafting/LineType.h"
#include "horizon/fileio/DrawingDimensionRenderer.h"
#include "horizon/fileio/DxfFormat.h"
#include "horizon/fileio/GeometricToleranceRenderer.h"
#include "horizon/modeling/DrawingView.h"

namespace hz::io {

namespace {

constexpr char kVisibleLayer[] = "Visible";
constexpr char kHiddenLayer[] = "Hidden";
constexpr char kDimensionLayer[] = "Dimensions";
constexpr char kToleranceLayer[] = "Tolerances";
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
}

}  // namespace

bool DrawingExport::toDxf(const std::string& path, const model::Drawing& drawing) {
    doc::Document doc;  // defaults to DocumentType::Drawing
    addDrawingLayers(doc);

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
    }

    return DxfFormat::save(path, doc);
}

bool DrawingExport::standardViewsToDxf(const std::string& path, const topo::Solid& solid) {
    return toDxf(path, model::DrawingGenerator::standardViews(solid));
}

}  // namespace hz::io
