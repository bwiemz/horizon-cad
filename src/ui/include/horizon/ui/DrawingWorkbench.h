#pragma once

#include <QObject>
#include <QString>
#include <memory>
#include <string>
#include <vector>

#include "horizon/fileio/DrawingDocumentIO.h"

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
/// own; what is drawn on it by hand, on other layers, stays for the session
/// but is not yet saved.
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
    struct Sheet {
        std::weak_ptr<doc::Document> document;
        io::DrawingDocumentSpec spec;
        /// The views as last drawn: what a click on the sheet finds.
        model::Drawing drawing;
    };

    Sheet* sheetOf(const doc::Document* document);
    const Sheet* sheetOf(const doc::Document* document) const;
    /// The active tab's sheet, or null with a word in the status bar.
    Sheet* activeSheet(const QString& verb);
    /// Draw @p spec's sheet into @p document again, from its part as it is
    /// now: on the sheet's own layers, locked, leaving any other. False, with
    /// the reason, when the part cannot be read or built. A version 1 spec
    /// is given the views it was drawn with. The views drawn go to @p drawn.
    bool draw(doc::Document& document, io::DrawingDocumentSpec& spec, std::string* error,
              model::Drawing* drawn = nullptr);
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
    /// A detail of view @p source of the sheet showing @p document, clicked
    /// at @p centre with @p radius on the sheet: its scale and label asked.
    void addDetail(const std::weak_ptr<doc::Document>& document, int source,
                   const math::Vec2& centre, const math::Vec2& onCircle);
    /// Add @p view (built, sized) to @p sheet as @p spec says, where there is
    /// room, and draw the sheet again.
    void addView(Sheet& sheet, const model::DrawingView& view, io::DrawingViewSpec spec,
                 const QString& verb);
    /// Draw the active sheet again after its spec changed, and mark it
    /// modified.
    void redraw(Sheet& sheet, const QString& verb);

    WorkbenchHost& m_host;
    /// Each sheet where it was made: a form holds one while it is open, and
    /// the list may lose a closed sheet meanwhile (refreshDrawingsOf).
    std::vector<std::unique_ptr<Sheet>> m_sheets;
};

}  // namespace hz::ui
