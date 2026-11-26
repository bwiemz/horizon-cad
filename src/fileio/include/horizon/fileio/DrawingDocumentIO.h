#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "horizon/math/Units.h"
#include "horizon/modeling/DrawingView.h"
#include "horizon/modeling/GeometricTolerance.h"
#include "horizon/modeling/HoleCallout.h"
#include "horizon/modeling/Sheet.h"
#include "horizon/modeling/TitleBlock.h"

namespace hz::doc {
class Document;
}  // namespace hz::doc

namespace hz::topo {
class Solid;
}  // namespace hz::topo

namespace hz::io {

/// A dimension on a view (Phase 149): the model edge it measures, by its
/// stable name, measured again from the part whenever the sheet is drawn.
struct DrawingDimensionSpec {
    enum class Kind { Length, Radius, Diameter };
    std::string edge;  ///< the edge's name: a curve's logical name, not one of its chords'
    Kind kind = Kind::Length;
};

/// A geometric tolerance on a view: a feature control frame, on the model
/// edge it names by its stable name. The frame's own feature is not saved:
/// it is the edge, found again in the view whenever the sheet is drawn.
struct DrawingToleranceSpec {
    std::string edge;  ///< its logical name, without the piece of a circle
    model::FeatureControlFrame frame;
};

/// A datum feature symbol on a view: the model edge that establishes the
/// datum, by its stable name, and the datum's letter, one on the sheet.
struct DrawingDatumSpec {
    std::string edge;  ///< its logical name, without the piece of a circle
    std::string label;
};

/// Where a view was on the sheet when it was saved (Phase 150): what it is,
/// and its rectangle. What was drawn in it is carried from there to where
/// the view is drawn when the sheet is opened: the part may have changed
/// since, and its views moved.
struct DrawingViewFrame {
    model::ViewRole role = model::ViewRole::Projection;
    model::StandardView kind = model::StandardView::Front;
    std::string label;
    math::Vec2 low;
    math::Vec2 high;
};

/// One view of a drawing, as saved: how it looks at the part, and where and
/// how large it sits on the sheet. Its geometry is not saved: it is projected
/// again from the part when the drawing is read (Principle 9).
struct DrawingViewSpec {
    model::StandardView kind = model::StandardView::Front;  ///< its label
    model::ViewProjection projection;
    double scale = 1.0;    ///< sheet mm per model mm
    math::Vec2 placement;  ///< the sheet position of its lower-left corner
    bool showHidden = true;
    bool showTangentEdges = true;
    /// A section (its plane: through `projection.origin`, looking along
    /// `projection.dir`) or a detail of an earlier view (Phase 149).
    model::ViewRole role = model::ViewRole::Projection;
    std::string label;        ///< "A": section A-A, detail A
    int source = -1;          ///< the projection before it that it was taken from
    math::Vec2 detailCenter;  ///< a detail's circle, in its source's view space
    double detailRadius = 0.0;
    bool showCentreLines = true;
    std::vector<DrawingDimensionSpec> dimensions;
    /// The holes called out in it, each by an edge of the hole: its logical
    /// name, without the piece of the circle (model::withoutPiece).
    std::vector<std::string> holeCallouts;
    /// Its geometric tolerances and datum feature symbols (format 5).
    std::vector<DrawingToleranceSpec> tolerances;
    std::vector<DrawingDatumSpec> datums;
};

/// What a hole feature of the part says of its holes, for their callouts.
struct HoleNote {
    std::string feature;  ///< its id ("hole_1"): every edge it makes names it
    model::HoleCallout::Kind kind = model::HoleCallout::Kind::Simple;
    model::HoleCallout::Depth depth = model::HoleCallout::Depth::Blind;
    double diameter = 0.0;
    double holeDepth = 0.0;
    double boreDiameter = 0.0;
    double boreDepth = 0.0;
    double sinkDiameter = 0.0;
    double sinkAngleDegrees = 90.0;
};

/// A drawing document's persisted specification: the part it draws, its
/// sheet and title block, and its views.
struct DrawingDocumentSpec {
    std::string partPath;  ///< the source .hzpart; saved relative to the drawing
    double gap = 10.0;     ///< spacing between views when they are laid out
    model::Sheet sheet;
    model::TitleBlock titleBlock;
    /// The views. None: the standard sheet layout (DrawingGenerator::sheetLayout),
    /// at @p scale when it is positive, else at the largest that fits.
    std::vector<DrawingViewSpec> views;
    double scale = 0.0;
    /// What was drawn on the sheet by hand (Phase 150): its entities, their
    /// layers and blocks, and the dimension style, as a document of their
    /// own. Null: nothing.
    std::shared_ptr<doc::Document> annotations;
    /// Where the views were when the annotations were saved.
    std::vector<DrawingViewFrame> frames;
    /// The unit the sheet shows and takes lengths in (Phase 154);
    /// millimetres when the file names none.
    math::LengthUnit lengthUnit = math::LengthUnit::Millimetre;
    /// The format version it was read from: 1 lays out the standard views
    /// by the gap at 1:1, as version 1 did; 2 is the sheet layout; 3 adds
    /// sections and details.
    int version = 3;
    /// What of its file readSpec left out, each with why ("view 2: ..."): a
    /// view that is not one, a dimension it cannot read, an annotation.
    /// Saved over that file, the sheet would lose them (Document::leftOut).
    std::vector<std::string> leftOut;
};

/// Reads/writes `.hzdwg` drawing documents.
///
/// A `.hzdwg` is a small JSON file that references a part file. Loading it opens
/// and rebuilds that part, then projects its views again, so the drawing shows
/// the model as it is now rather than a stale snapshot.
///
/// Version 2 (Phase 148) keeps the sheet, the title block and every view's
/// direction, scale and placement. Version 3 (Phase 149) adds each view's
/// role, label and source: sections and details. A version 1 file (part and
/// gap only) still reads, as the four standard views laid out by its gap at
/// 1:1.
class DrawingDocumentIO {
public:
    /// Write a `.hzdwg` describing @p spec. The part's path is written relative
    /// to the drawing's folder when it can be. Returns false on I/O failure.
    static bool save(const std::string& path, const DrawingDocumentSpec& spec);
    /// What save() writes for @p spec at @p path: two specs that serialise
    /// alike are the same drawing.
    static std::string serialise(const std::string& path, const DrawingDocumentSpec& spec);

    /// Load a `.hzdwg`: read the spec, open and rebuild the referenced part, and
    /// project its views into @p outDrawing. @p outSpec receives the spec, its
    /// part path as found (relative paths are the drawing folder's). Returns
    /// false, with the reason in @p error when given, on I/O failure, a file
    /// that is not a drawing, a missing or broken part, or a rebuild that
    /// makes no solid.
    static bool load(const std::string& path, DrawingDocumentSpec& outSpec,
                     model::Drawing& outDrawing, std::string* error = nullptr);

    /// Read a `.hzdwg`'s spec alone, its part path resolved as load() resolves
    /// it; the part is not read. False, with the reason, as load().
    static bool readSpec(const std::string& path, DrawingDocumentSpec& outSpec,
                         std::string* error = nullptr);

    /// The drawing @p spec describes, projected from @p solid, its dimensions
    /// measured from it. A dimension, callout, tolerance or datum whose edge
    /// the view no longer shows is left out, and said in @p lost ("view 2: a
    /// length").
    static model::Drawing build(const topo::Solid& solid, const DrawingDocumentSpec& spec,
                                std::vector<std::string>* lost = nullptr,
                                const std::vector<HoleNote>* holes = nullptr);

    /// The most a chord of a view of an exact part sags on paper, in millimetres (stage 3).
    static constexpr double kChordOnPaper = 0.01;

    /// The largest scale (paper ÷ model) a sheet of @p spec draws its views at, of a part whose
    /// faceted solid is @p faceted, which its copy is cut for: its views' largest but its details'
    /// (Ruling 35); with none, its scale where it is set, else the one its standard layout
    /// chooses for @p faceted above @p room, as before (DrawingGenerator::sheetLayout); a version
    /// 1 sheet's views are at 1:1. A detail is cut from its view as drawn, so its chords sag up
    /// to 0.01 × its scale ÷ its view's mm on paper (0.1 mm for a 10:1 detail of a 1:1 view).
    static double sheetScale(const DrawingDocumentSpec& spec, const topo::Solid& faceted,
                             const model::TitleBlock& room);

    /// The step of the tolerance a sheet whose views are at most @p scale is cut at (Ruling 34):
    /// its quarter octaves above 1:1, rounded up, 4 log2(@p scale), so that the tolerance,
    /// kChordOnPaper halved every four steps, is no coarser than kChordOnPaper / @p scale, and
    /// every scale of one step is cut alike. A scale a sheet cannot be drawn at is 1:1, step 0.
    static int toleranceStep(double scale);

    /// What a sheet whose views are at most @p scale draws a part from, where it has an exact
    /// part @p exact (Document::exactSolid; stage 3): its exact faces cut into facets, 32 a turn
    /// and so finely that no chord sags more than kChordOnPaper on paper (model::facetExact at
    /// kChordOnPaper / 2^(k / 4) mm, k its toleranceStep; an import's faces as its own bridge
    /// cuts them, Ruling 17, its rims into at most 400 chords, Ruling 32, so that an imported rim
    /// over about 325 mm in radius sags a little more than 0.01 mm on paper at 1:1).
    /// Its edges are named so that dimensions, callouts, tolerances and datums find theirs by
    /// model::logicalEdge as on the faceted part: a curve's chords logically
    /// (model::nameEdgesLogically), a chord of an exact edge inside one face (a seam) as that
    /// edge. Every other line inside a face is `<face>/line:<n>/at:<k>` (Rulings 31 and 34), a
    /// name of this copy alone: the lines depend on the tolerance, and so their names carry its
    /// step k. A dimension placed on one resolves on a copy cut at that step, and on no other: at
    /// another scale it is lost, and said to be, never measured on a line elsewhere; and the
    /// faceted part's lines (`<face>/seam:<n>`) are none of them. A part with no exact part is
    /// drawn from its faceted solid (Document::solid), as before. Null, with why, where it cannot
    /// be cut.
    static std::unique_ptr<topo::Solid> cutForSheet(const topo::Solid& exact, double scale,
                                                    std::string* error = nullptr);

    /// The hole features of @p part that take part in its build.
    static std::vector<HoleNote> holesOf(const doc::Document& part);
    /// The hole of @p holes that made the edge named @p edge; null for an
    /// edge no hole made.
    static const HoleNote* holeOfEdge(const std::vector<HoleNote>& holes, const std::string& edge);

    /// The spec that saves @p drawing as it is laid out.
    static std::vector<DrawingViewSpec> viewsOf(const model::Drawing& drawing);

    /// The most characters a view's label has: a caption shows a few.
    static constexpr std::size_t kMaxLabel = 8;
    /// The most dimensions, hole callouts, tolerances or datums a view's
    /// entry is read with (each kind): a sheet with more is refused, so no
    /// change makes more.
    static constexpr std::size_t kMaxPerView = 256;

    /// @p label, typed or read from a file, as a view's label: no line
    /// breaks, and at most kMaxLabel characters, counted by code point and
    /// never cut inside one. @p label is UTF-8.
    static std::string usableLabel(std::string label);
};

}  // namespace hz::io
