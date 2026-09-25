#pragma once

#include <string>

namespace hz::topo {
class Solid;
}  // namespace hz::topo

namespace hz::model {
struct Drawing;
struct Sheet;
struct TitleBlock;
}  // namespace hz::model

namespace hz::doc {
class Document;
}  // namespace hz::doc

namespace hz::io {

/// Exports generated 2D drawings (hidden-line projections of a solid) to DXF so
/// they open in any CAD tool.
///
/// Visible edges are written as continuous lines on a "Visible" layer; hidden
/// edges as dashed lines on a "Hidden" layer. Each view's geometry is offset to
/// its sheet placement so a multi-view drawing lays out without overlap.
class DrawingExport {
public:
    /// Draw @p drawing into @p doc, on its layers (Visible, Hidden, Section,
    /// Dimensions, ...): framed by @p sheet's border, and @p titleBlock, when
    /// given. Each view is placed and scaled by DrawingView::toSheet, and
    /// leaves out its hidden or tangent edges when it says so. What a drawing
    /// document shows, prints and exports is this.
    static void populate(doc::Document& doc, const model::Drawing& drawing,
                         const model::Sheet* sheet = nullptr,
                         const model::TitleBlock* titleBlock = nullptr);

    /// Write a laid-out multi-view drawing to a DXF file. Returns false on I/O
    /// failure.
    static bool toDxf(const std::string& path, const model::Drawing& drawing);

    /// Write a drawing framed by a sheet border and title block to a DXF file.
    /// The border and title block are drawn on their own layers ("Border",
    /// "TitleBlock"), sized for @p sheet, with @p titleBlock's fields populated.
    static bool toDxf(const std::string& path, const model::Drawing& drawing,
                      const model::Sheet& sheet, const model::TitleBlock& titleBlock);

    /// Convenience: generate the standard four-view drawing of @p solid and write
    /// it to a DXF file.
    static bool standardViewsToDxf(const std::string& path, const topo::Solid& solid);
};

}  // namespace hz::io
