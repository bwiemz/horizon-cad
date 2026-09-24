#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "horizon/drafting/DraftText.h"
#include "horizon/math/BoundingBox.h"
#include "horizon/math/Vec2.h"

namespace hz::draft {

class DraftBlockRef;
class DraftDocument;
class LayerManager;
struct DimensionStyle;

/// A line to plot, in world coordinates, its style resolved (ByLayer and
/// ByBlock taken from the layer and the block reference).
struct PlotStroke {
    std::vector<math::Vec2> points;
    bool closed = false;
    uint32_t color = 0xFF000000;  ///< ARGB
    double width = 1.0;           ///< the line width, as the entity has it (plotWeightMm)
    int lineType = 1;             ///< 1 Continuous ... 7 Phantom (LineType)
};

/// Text to plot: its baseline point, height (cap height) and rotation in
/// world units and radians.
struct PlotText {
    math::Vec2 position;
    std::string text;
    double height = 2.5;
    double rotation = 0.0;
    TextAlignment alignment = TextAlignment::Left;
    uint32_t color = 0xFF000000;
};

/// A drawing as a plot draws it: strokes and text, with every block expanded.
/// It is built without Qt, so each output (PDF, SVG) and each test reads the
/// same geometry.
struct PlotScene {
    std::vector<PlotStroke> strokes;
    std::vector<PlotText> texts;
    math::BoundingBox bounds;  ///< of all of it, in world coordinates

    bool empty() const { return strokes.empty() && texts.empty(); }
};

/// What a drawing plots: the entities on visible layers (locked ones too),
/// blocks expanded to 16 levels deep, circles and arcs in steps of at most a
/// degree. Entities inside a block are hidden with their own layer, and take
/// ByBlock values from the reference.
PlotScene buildPlotScene(const DraftDocument& drawing, const LayerManager& layers,
                         const DimensionStyle& style);

/// What one block reference draws: its block's contents placed, blocks
/// within it too, with what they leave ByBlock taken from @p color, @p width
/// and @p lineType (the reference's own, as resolved by the caller). The
/// viewport draws block references with it.
PlotScene plotBlockReference(const DraftBlockRef& ref, const LayerManager& layers,
                             const DimensionStyle& style, uint32_t color, double width,
                             int lineType);

/// A line width as millimetres on paper. Widths are millimetres, as DXF has
/// them, except the default width 1.0, which plots at the usual default of
/// 0.25 mm (DXF writes it as "default" too). Clamped to DXF's 0.05–2.11 mm.
double plotWeightMm(double width);

/// A line type's dash pattern as on and off lengths in millimetres on paper
/// (the viewport's pattern, ten times over); empty for a continuous line.
std::vector<double> plotDashMm(int lineType);

/// A standard paper size, portrait.
struct PaperSize {
    const char* name;
    double widthMm;
    double heightMm;
};

/// ISO A0–A4 and ANSI Letter, Legal and Tabloid.
const std::vector<PaperSize>& standardPaperSizes();

/// The paper and how the drawing sits on it.
struct PlotLayout {
    double paperWidthMm = 297.0;  ///< A4 landscape by default
    double paperHeightMm = 210.0;
    double marginMm = 10.0;
    /// Millimetres on paper per drawing unit (the drawing is in millimetres,
    /// so 1:50 is 0.02); 0 fits the drawing to the paper inside the margins.
    double scale = 0.0;
    bool monochrome = false;  ///< every stroke and text black
};

/// Where a world point lands on the paper: millimetres from the top left, the
/// drawing's centre at the centre of the printable area.
struct PlotTransform {
    double scale = 1.0;  ///< paper mm per world unit
    math::Vec2 worldCentre;
    math::Vec2 paperCentre;

    math::Vec2 toPaper(const math::Vec2& world) const {
        return {paperCentre.x + (world.x - worldCentre.x) * scale,
                paperCentre.y - (world.y - worldCentre.y) * scale};
    }
};

/// The transform for @p scene on @p layout. @p fits is set to whether the
/// drawing's bounds, at that scale, are inside the printable area; a fitted
/// scale always fits.
PlotTransform plotTransform(const PlotScene& scene, const PlotLayout& layout, bool* fits = nullptr);

/// The colour a stroke or text is plotted in: black when monochrome, and
/// white (or near it, drawn on the dark viewport) as black on white paper.
uint32_t plotColor(uint32_t argb, bool monochrome);

}  // namespace hz::draft
