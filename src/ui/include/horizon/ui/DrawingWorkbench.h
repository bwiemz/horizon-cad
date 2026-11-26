#pragma once

#include <QObject>
#include <QString>
#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "horizon/fileio/DrawingDocumentIO.h"
#include "horizon/modeling/PartsList.h"

namespace hz::doc {
class Document;
}  // namespace hz::doc

namespace hz::topo {
class Solid;
}  // namespace hz::topo

namespace hz::ui {

class WorkbenchHost;

/// Drawing sheets made from a part (Phase 148): a sheet's tab shows the
/// part's standard views, laid out at a standard scale with a border and
/// title block, and is saved as a `.hzdwg` that names the part.
///
/// A sheet is drawn from its part as it is now: from the part's tab when it
/// is open, else from its file. It is drawn again when the part changes
/// there. The sheet's own layers are locked, since it holds nothing of its
/// own; what is drawn on it by hand, on other layers, is saved with it and
/// moves with the view it was drawn in (Phase 150).
class DrawingWorkbench : public QObject {
    Q_OBJECT

public:
    explicit DrawingWorkbench(WorkbenchHost& host, QObject* parent = nullptr);

    // --- The Drawing menu ---
    /// A sheet of the active part (saved), or of a part file chosen.
    void onNewDrawingFromPart();
    void onTitleBlock();
    void onScale();
    /// Draw the active sheet again from its part.
    void onUpdateFromPart();
    /// A section through a view of the active sheet, set in a form (Phase 149).
    void onAddSectionView();
    /// A detail of a view: its centre and radius clicked, then a form.
    void onAddDetailView();
    /// A view clicked, then the place it goes to.
    void onMoveView();
    /// A view chosen in a form, and the views taken from it, removed.
    void onRemoveView();
    /// A view's switches: its hidden edges, tangent edges and centre lines.
    void onViewProperties();
    /// Edges clicked, one after another, each dimensioned: its length, or
    /// its radius or diameter, measured from the part and kept by its name.
    void onAddDimension();
    /// An edge clicked: its dimensions removed.
    void onRemoveDimension();
    /// A hole's edge clicked: the hole called out ("3× ⌀6.6 THRU"), its size
    /// as its feature says, and how many of it the part has.
    void onAddHoleCallout();
    /// A frame asked for in a form (its characteristic, tolerance, zone,
    /// material condition and datums), then edges clicked, each toleranced
    /// with it.
    void onAddTolerance();
    /// A datum letter asked for, then edges clicked, each made the datum of
    /// it: the first the letter asked for, the next the letter after it.
    void onAddDatum();

    // --- For the window ---
    /// Whether @p document is a sheet this workbench draws.
    bool isSheet(const doc::Document* document) const;
    /// The paper @p document is drawn on, when it is a sheet; else null.
    const model::Sheet* paperOf(const doc::Document* document) const;
    /// Open the `.hzdwg` at @p fileName in a tab of its own. False, said to
    /// the user, when it or its part cannot be read.
    bool open(const QString& fileName);
    /// A new sheet drawn by @p spec, of the part or assembly at its
    /// partPath (from its tab when it is open, else its file), in a tab
    /// "Drawing of <name>", unsaved. Made the active tab unless @p behind.
    /// An empty title is the file's name. The tab's title in @p tab and what
    /// was drawn in @p drawn, when given. False, with why in @p error, when
    /// the part or assembly cannot be read.
    bool newDrawing(io::DrawingDocumentSpec spec, bool behind, QString* tab, model::Drawing* drawn,
                    std::string* error);
    /// Save the sheet @p document to @p path. False, with the reason.
    bool save(doc::Document& document, const std::string& path, std::string* error);
    /// The sheet @p document's file, changed by another program: read again,
    /// and drawn from it. Said to the user when it cannot be.
    void readAgain(doc::Document& document);
    /// A sheet shown in its tab: seen from above, its paper in view.
    void shown(doc::Document& document);
    /// Sheets of the part at @p path: drawn again from it (it changed).
    void refreshDrawingsOf(const std::string& path);

    // --- Changes from values rather than forms and clicks (the AI assistant's) ---
    // Each makes the change its Drawing menu command makes to the sheet
    // @p document shows, as one undo step (none when it changes nothing).
    // False, with why in @p error and nothing changed, when it cannot be made.

    /// The sheet @p document shows, as its spec has it; null for a document
    /// that is not a sheet.
    const io::DrawingDocumentSpec* specOf(const doc::Document* document) const;
    /// Its views as last drawn; null for a document that is not a sheet.
    const model::Drawing* drawnViews(const doc::Document* document) const;
    /// What it is drawn from: its part's solid, or its assembly's gathered,
    /// for naming and measuring edges. Null, with why.
    const topo::Solid* solidOf(doc::Document& document, std::string* error);
    /// Its part's hole features, for their callouts: none for an assembly's.
    /// Null, with why, when the part cannot be read.
    const std::vector<io::HoleNote>* holesOf(doc::Document& document, std::string* error);
    /// Its title block and paper. On new paper its views are laid out again;
    /// @p fits false when some found no room.
    bool setTitleBlock(doc::Document& document, const model::TitleBlock& block,
                       const model::Sheet& paper, bool* fits, std::string* error);
    /// Its scale, its views laid out again at it (0: the largest that fits).
    bool setScale(doc::Document& document, double scale, bool* fits, std::string* error);
    /// View @p view moved by @p by, in sheet millimetres.
    bool moveView(doc::Document& document, std::size_t view, const math::Vec2& by,
                  std::string* error);
    /// View @p view removed, and the views taken from it, @p taken of them.
    bool removeView(doc::Document& document, std::size_t view, int* taken, std::string* error);
    /// Whether view @p view shows its hidden edges, tangent edges and centre
    /// lines.
    bool setViewSwitches(doc::Document& document, std::size_t view, bool hidden, bool tangent,
                         bool centreLines, std::string* error);
    /// The model edge named @p edge dimensioned in view @p view, measured
    /// from the part: a circle's diameter, an arc's radius, else its length.
    bool addEdgeDimension(doc::Document& document, std::size_t view, const std::string& edge,
                          std::string* error);
    /// The hole that made the edge named @p edge called out in view @p view:
    /// its size, as its feature says, and how many of it the part has. Only
    /// a part's sheet: an assembly's holes are its parts'.
    bool addHoleCallout(doc::Document& document, std::size_t view, const std::string& edge,
                        std::string* error);
    /// @p frame, a geometric tolerance, on the model edge named @p edge in
    /// view @p view: one the standard allows (model::Gdt::check), referring
    /// to datums the sheet has. Its feature is the edge.
    bool addTolerance(doc::Document& document, std::size_t view, const std::string& edge,
                      const model::FeatureControlFrame& frame, std::string* error);
    /// The model edge named @p edge in view @p view made datum @p label: a
    /// letter no other datum on the sheet has; empty, the first free one,
    /// said in @p given.
    bool addDatum(doc::Document& document, std::size_t view, const std::string& edge,
                  const std::string& label, std::string* error, std::string* given = nullptr);
    /// The dimensions of the edge named @p edge in view @p view removed, and
    /// its hole's callout, and its frames and datum (or its hole's). A
    /// frame still referring to a datum removed is said in @p note.
    bool removeEdgeDimensions(doc::Document& document, std::size_t view, const std::string& edge,
                              std::string* error, std::string* note = nullptr);

private:
    friend class SheetEditCommand;

    struct Source;
    /// What was drawn by hand that a change of the sheet carried with its
    /// views: each entity, and how far it went.
    using Carried = std::vector<std::pair<uint64_t, math::Vec2>>;
    struct Sheet {
        std::weak_ptr<doc::Document> document;
        io::DrawingDocumentSpec spec;
        /// The spec as last drawn: where a change's undo goes back to.
        io::DrawingDocumentSpec drawn;
        /// The views as last drawn: what a click on the sheet finds.
        model::Drawing drawing;
        /// The part files it was drawn from: its part's, or its assembly's
        /// parts'. It is drawn again when one changes.
        std::vector<std::string> files;
        /// The room its parts list takes above the title block (an assembly's).
        double partsListHeight = 0.0;
        /// What it was drawn from, kept until one of its files changes;
        /// null: read again at the next drawing.
        std::shared_ptr<const Source> source;
        /// Its views are not drawn where its spec has them (an undo or redo
        /// when its part could not be read): the next drawing does not carry
        /// notes from where they are drawn.
        bool viewsBehind = false;
    };
    /// What a sheet is drawn from (Phase 150): a part's solid, or an
    /// assembly's components gathered into one, with its parts list and a
    /// balloon for each part. Its own copy of them: a sheet keeps it until
    /// one of its files changes, not re-reading its parts on every edit, and
    /// a part's tab may rebuild meanwhile.
    struct Source {
        /// What the views are drawn from: a part's faceted solid; or, where it has an exact
        /// part, that cut for a sheet whose largest scale is `scale`
        /// (io::DrawingDocumentIO::cutForSheet; stage 3); or an assembly's components
        /// gathered, each its faceted part (an assembly's sheet stays faceted in stage 3).
        std::unique_ptr<topo::Solid> solid;
        /// Where the part has an exact part: its own copies of it and of its faceted solid.
        /// `solid` is cut again from `exact` for a sheet at another scale; `faceted` chooses the
        /// scale of a sheet laid out anew, as before. Null otherwise.
        std::shared_ptr<const topo::Solid> exact;
        std::shared_ptr<const topo::Solid> faceted;
        double scale = 0.0;  ///< the largest view scale `solid` was cut for; 0 when not cut
        /// The scale the standard layout of `faceted` chose, and for which paper, title block
        /// and gap (its width, height and margin, the block's height, the gap): a sheet laid out
        /// anew asks it again at every edit.
        mutable std::optional<std::pair<std::array<double, 5>, double>> laidOut;
        model::PartsList partsList;
        /// A component's name prefix, and its item number.
        std::vector<std::pair<std::string, int>> balloons;
        std::vector<std::string> files;  ///< every file it was read from
        std::vector<uint64_t> missing;   ///< components with no part to draw
        /// A part's hole features, for its holes' callouts; none for an
        /// assembly.
        std::vector<io::HoleNote> holes;
    };
    /// An edge clicked on a sheet: the view it is in, and its name.
    struct PickedEdge {
        int view = -1;
        topo::TopologyID edge;
    };

    Sheet* sheetOf(const doc::Document* document);
    const Sheet* sheetOf(const doc::Document* document) const;
    /// The active tab's sheet, or null with a word in the status bar.
    Sheet* activeSheet(const QString& verb);
    /// Draw @p spec's sheet into @p document again, from @p source: on the
    /// sheet's own layers, locked, leaving any other. A version 1 spec is
    /// given the views it was drawn with. The views drawn go to @p drawn.
    /// What is not drawn for an edge the part no longer has goes to
    /// @p leftOut, as a sheet read says what it left out
    /// (io::DrawingDocumentIO::leftOutOf), when it is given (a sheet read);
    /// else it is said in the status bar (a sheet drawn again as it is
    /// changed).
    void draw(doc::Document& document, io::DrawingDocumentSpec& spec, const Source& source,
              model::Drawing* drawn = nullptr, std::vector<std::string>* leftOut = nullptr);
    /// What the sheet @p spec describes (of a part, or an assembly, at spec.partPath) is
    /// drawn from, as it is now: an exact part cut for the sheet's largest scale. Null, with the
    /// reason, when it cannot be read or cut.
    std::shared_ptr<const Source> sourceOf(const io::DrawingDocumentSpec& spec, std::string* error);
    /// @p sheet's source: the one it keeps, cut again from its exact part where the sheet's
    /// largest scale has changed, or read now and kept.
    std::shared_ptr<const Source> sourceFor(Sheet& sheet, std::string* error);
    /// The largest scale a sheet of @p spec draws @p source's part at
    /// (io::DrawingDocumentIO::sheetScale): for a sheet laid out anew, the scale its faceted part's
    /// layout chooses, asked once for each paper, title block and gap. Only for a source with an
    /// exact part.
    static double scaleOf(const io::DrawingDocumentSpec& spec, const Source& source);
    /// @p from with its exact part cut for a sheet whose largest scale is @p scale: a new source,
    /// all else @p from's. Null, with the reason, when it cannot be cut.
    static std::shared_ptr<const Source> cutFor(const Source& from, double scale,
                                                std::string* error);
    /// @p sheet's title block with the room its parts list takes: what its
    /// views are laid out and placed around.
    static model::TitleBlock roomFor(const Sheet& sheet);
    /// The part at @p path as it is now: its tab's document when it is open and
    /// built, else its file's, built in @p holder: its solid, and its exact part where it has
    /// one. Null, with the reason.
    const doc::Document* partDocument(const std::string& path, doc::Document& holder,
                                      std::string* error);
    /// Lay the sheet's standard views out again at @p scale (0: the largest
    /// that fits), with its sections and details kept, scaled with them, and
    /// placed again where there is room; @p fits false when some found none.
    /// False, with the reason, when the part cannot be read.
    bool layOutAgain(Sheet& sheet, double scale, bool* fits, std::string* error);
    /// The view of @p sheet under the sheet point @p at; -1 for none. Only
    /// projections, when @p projectionOnly.
    static int viewAt(const Sheet& sheet, const math::Vec2& at, bool projectionOnly);
    /// The first letter no view of @p sheet is labelled with.
    static QString nextLabel(const Sheet& sheet);
    /// The edge drawn nearest the sheet point @p at, within reach of it.
    static std::optional<PickedEdge> edgeAt(const Sheet& sheet, const math::Vec2& at);
    /// A PickTool for edges on the sheet showing @p document, handing each
    /// one clicked to @p use, the tool staying for the next.
    void pickEdges(const QString& verb, const std::weak_ptr<doc::Document>& document,
                   void (DrawingWorkbench::*use)(const std::weak_ptr<doc::Document>&,
                                                 const PickedEdge&));
    void addDimension(const std::weak_ptr<doc::Document>& document, const PickedEdge& picked);
    void removeDimensions(const std::weak_ptr<doc::Document>& document, const PickedEdge& picked);
    void addHoleCalloutAt(const std::weak_ptr<doc::Document>& document, const PickedEdge& picked);
    void addToleranceAt(const std::weak_ptr<doc::Document>& document, const PickedEdge& picked);
    void addDatumAt(const std::weak_ptr<doc::Document>& document, const PickedEdge& picked);
    /// A detail of view @p source of the sheet showing @p document, clicked
    /// at @p centre with @p radius on the sheet: its scale and label asked.
    void addDetail(const std::weak_ptr<doc::Document>& document, int source,
                   const math::Vec2& centre, const math::Vec2& onCircle);
    /// Add @p view (built, sized) to @p sheet as @p spec says, where there is
    /// room, and draw the sheet again.
    void addView(Sheet& sheet, const model::DrawingView& view, io::DrawingViewSpec spec,
                 const QString& verb);
    /// Draw @p sheet again from its spec and its part, carrying what was
    /// drawn by hand inside a view with the view (said in @p carried, when
    /// given). False, with the reason.
    bool drawAgain(Sheet& sheet, std::string* error, Carried* carried = nullptr);
    /// Draw the active sheet again after its spec changed, as one step on its
    /// undo stack. False, said in the status bar and in @p error, when it
    /// cannot be drawn: the spec is put back as it was drawn.
    bool redraw(Sheet& sheet, const QString& verb, std::string* error = nullptr);
    /// @p sheet's views listed in its spec: the automatic layout's, as drawn,
    /// when it lists none.
    static void listViews(Sheet& sheet);
    /// Draw the sheet showing @p document as @p spec has it (an undo or a
    /// redo of a change), moving what the change carried by @p sign times
    /// how far it went: just that, not what lies where the views are now.
    void replay(doc::Document& document, const io::DrawingDocumentSpec& spec,
                const Carried& carried, double sign);

    WorkbenchHost& m_host;
    /// The frame Add Geometric Tolerance puts on each edge clicked, and the
    /// letter Add Datum Feature makes the next one clicked.
    model::FeatureControlFrame m_frame;
    std::string m_datum;
    /// Each sheet where it was made: a form holds one while it is open, and
    /// the list may lose a closed sheet meanwhile (refreshDrawingsOf).
    std::vector<std::unique_ptr<Sheet>> m_sheets;
};

}  // namespace hz::ui
