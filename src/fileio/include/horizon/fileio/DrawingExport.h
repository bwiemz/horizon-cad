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

namespace hz::io {

/// Exports generated 2D drawings (hidden-line projections of a solid) to DXF so
/// they open in any CAD tool.
///
/// Visible edges are written as continuous lines on a "Visible" layer; hidden
/// edges as dashed lines on a "Hidden" layer. Each view's geometry is offset to
/// its sheet placement so a multi-view drawing lays out without overlap.
class DrawingExport {
public:
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
