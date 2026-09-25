#pragma once

#include <string>
#include <vector>

#include "horizon/modeling/DrawingView.h"
#include "horizon/modeling/Sheet.h"
#include "horizon/modeling/TitleBlock.h"

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
    /// The format version it was read from: 1 lays out the standard views
    /// by the gap at 1:1, as version 1 did; 2 is the sheet layout; 3 adds
    /// sections and details.
    int version = 3;
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
    /// measured from it. A dimension whose edge the part no longer has is
    /// left out, and said in @p lost ("view 2: a length").
    static model::Drawing build(const topo::Solid& solid, const DrawingDocumentSpec& spec,
                                std::vector<std::string>* lost = nullptr);

    /// The spec that saves @p drawing as it is laid out.
    static std::vector<DrawingViewSpec> viewsOf(const model::Drawing& drawing);
};

}  // namespace hz::io
