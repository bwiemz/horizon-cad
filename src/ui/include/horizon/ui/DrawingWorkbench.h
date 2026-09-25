#pragma once

#include <QObject>
#include <QString>
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

    // --- For the window ---
    /// Whether @p document is a sheet this workbench draws.
    bool isSheet(const doc::Document* document) const;
    /// The paper @p document is drawn on, when it is a sheet; else null.
    const model::Sheet* paperOf(const doc::Document* document) const;
    /// Open the `.hzdwg` at @p fileName in a tab of its own. False, said to
    /// the user, when it or its part cannot be read.
    bool open(const QString& fileName);
    /// Save the sheet @p document to @p path. False, with the reason.
    bool save(doc::Document& document, const std::string& path, std::string* error);
    /// The sheet @p document's file, changed by another program: read again,
    /// and drawn from it. Said to the user when it cannot be.
    void readAgain(doc::Document& document);
    /// A sheet shown in its tab: seen from above, its paper in view.
    void shown(doc::Document& document);
    /// Sheets of the part at @p path: drawn again from it (it changed).
    void refreshDrawingsOf(const std::string& path);

private:
    struct Source;
    struct Sheet {
        std::weak_ptr<doc::Document> document;
        io::DrawingDocumentSpec spec;
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
    };
    /// What a sheet is drawn from (Phase 150): a part's solid, or an
    /// assembly's components gathered into one, with its parts list and a
    /// balloon for each part. Its own copy of them: a sheet keeps it until
    /// one of its files changes, not re-reading its parts on every edit, and
    /// a part's tab may rebuild meanwhile.
    struct Source {
        std::unique_ptr<topo::Solid> solid;
        model::PartsList partsList;
        /// A component's name prefix, and its item number.
        std::vector<std::pair<std::string, int>> balloons;
        std::vector<std::string> files;  ///< every file it was read from
        std::vector<uint64_t> missing;   ///< components with no part to draw
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
    void draw(doc::Document& document, io::DrawingDocumentSpec& spec, const Source& source,
              model::Drawing* drawn = nullptr);
    /// What the sheet of @p path (a part, or an assembly) is drawn from, as
    /// it is now. Null, with the reason, when it cannot be read.
    std::shared_ptr<const Source> sourceOf(const std::string& path, std::string* error);
    /// @p sheet's source: the one it keeps, or read now and kept.
    std::shared_ptr<const Source> sourceFor(Sheet& sheet, std::string* error);
    /// @p sheet's title block with the room its parts list takes: what its
    /// views are laid out and placed around.
    static model::TitleBlock roomFor(const Sheet& sheet);
    /// The part at @p path as it is now: its tab's solid when it is open and
    /// built, else its file's, built in @p holder. Null, with the reason.
    const topo::Solid* partSolid(const std::string& path, doc::Document& holder,
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
    /// A detail of view @p source of the sheet showing @p document, clicked
    /// at @p centre with @p radius on the sheet: its scale and label asked.
    void addDetail(const std::weak_ptr<doc::Document>& document, int source,
                   const math::Vec2& centre, const math::Vec2& onCircle);
    /// Add @p view (built, sized) to @p sheet as @p spec says, where there is
    /// room, and draw the sheet again.
    void addView(Sheet& sheet, const model::DrawingView& view, io::DrawingViewSpec spec,
                 const QString& verb);
    /// Draw @p sheet again from its spec and its part, carrying what was
    /// drawn by hand inside a view with the view. False, with the reason.
    bool drawAgain(Sheet& sheet, std::string* error);
    /// Draw the active sheet again after its spec changed, and mark it
    /// modified.
    void redraw(Sheet& sheet, const QString& verb);

    WorkbenchHost& m_host;
    /// Each sheet where it was made: a form holds one while it is open, and
    /// the list may lose a closed sheet meanwhile (refreshDrawingsOf).
    std::vector<std::unique_ptr<Sheet>> m_sheets;
};

}  // namespace hz::ui
