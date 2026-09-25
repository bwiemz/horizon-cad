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
    };

    Sheet* sheetOf(const doc::Document* document);
    const Sheet* sheetOf(const doc::Document* document) const;
    /// The active tab's sheet, or null with a word in the status bar.
    Sheet* activeSheet(const QString& verb);
    /// Draw @p spec's sheet into @p document again, from its part as it is
    /// now: on the sheet's own layers, locked, leaving any other. False, with
    /// the reason, when the part cannot be read or built. A version 1 spec
    /// is given the views it was drawn with.
    bool draw(doc::Document& document, io::DrawingDocumentSpec& spec, std::string* error);
    /// Draw the active sheet again after its spec changed, and mark it
    /// modified.
    void redraw(Sheet& sheet, const QString& verb);

    WorkbenchHost& m_host;
    std::vector<Sheet> m_sheets;
};

}  // namespace hz::ui
