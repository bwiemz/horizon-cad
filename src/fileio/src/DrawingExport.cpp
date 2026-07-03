#include "horizon/fileio/DrawingExport.h"

#include <memory>

#include "horizon/document/Document.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/Layer.h"
#include "horizon/drafting/LineType.h"
#include "horizon/fileio/DxfFormat.h"
#include "horizon/modeling/DrawingView.h"

namespace hz::io {

namespace {

constexpr char kVisibleLayer[] = "Visible";
constexpr char kHiddenLayer[] = "Hidden";

void addDrawingLayers(doc::Document& doc) {
    draft::LayerProperties visible;
    visible.name = kVisibleLayer;
    doc.layerManager().addLayer(visible);

    draft::LayerProperties hidden;
    hidden.name = kHiddenLayer;
    doc.layerManager().addLayer(hidden);
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
    }

    return DxfFormat::save(path, doc);
}

bool DrawingExport::standardViewsToDxf(const std::string& path, const topo::Solid& solid) {
    return toDxf(path, model::DrawingGenerator::standardViews(solid));
}

}  // namespace hz::io
