#pragma once

#include <QString>
#include <memory>
#include <string>
#include <vector>

class QWidget;

namespace hz::doc {
class AssemblyDocument;
class Document;
class DocumentManager;
class Feature;
class Sketch;
}  // namespace hz::doc

namespace hz::io {
struct ImportReport;
}  // namespace hz::io

namespace hz::ui {

class Tool;
class ViewportWidget;

/// What a workbench asks of the window it works in (Phase 146).
///
/// A workbench holds one kind of document's commands and their state; the
/// window holds the tabs, menus, docks and files. Each command reaches the
/// window only through this: the documents open and shown, the view, and the
/// status bar, files and background work. It is kept narrow on purpose. A
/// command that needs more asks for it here, where it shows, instead of
/// reaching into a 5,000-line window.
class WorkbenchHost {
public:
    virtual ~WorkbenchHost() = default;

    /// The parent for a workbench's dialogs.
    virtual QWidget* dialogParent() = 0;

    /// The active tab's document. For an assembly's tab, its backing
    /// document: its undo stack records the assembly's edits.
    virtual doc::Document* currentDocument() = 0;
    /// The active tab's assembly; null when the tab shows no assembly.
    virtual std::shared_ptr<doc::AssemblyDocument> currentAssembly() = 0;
    /// Every assembly open in a tab.
    virtual std::vector<std::shared_ptr<doc::AssemblyDocument>> openAssemblies() = 0;
    virtual doc::DocumentManager& documents() = 0;
    virtual ViewportWidget& viewport() = 0;

    /// A message in the status bar; @p timeoutMs 0 keeps it until the next.
    virtual void showStatus(const QString& message, int timeoutMs = 0) = 0;
    /// The status bar's message now.
    virtual QString currentStatus() = 0;
    /// The prompt beside it: what the window is doing ("Ready").
    virtual void setPrompt(const QString& text) = 0;

    /// Build the view's scene again from the active tab's document.
    virtual void rebuildScene() = 0;
    /// Show which tabs have unsaved changes.
    virtual void refreshModifiedIndicators() = 0;
    /// A command changed the view's selection: what shows it (the
    /// properties, the status bar) follows.
    virtual void selectionChanged() = 0;

    /// Open @p fileName in a tab, or show the tab it has; false, said to the
    /// user, when it cannot be read.
    virtual bool openPath(const QString& fileName) = 0;
    /// Show @p document, made by the workbench, in a new tab titled @p title,
    /// and make it the active one.
    virtual void addTab(std::shared_ptr<doc::Document> document, const QString& title) = 0;
    /// Tell the user a file could not be read or written, and why.
    virtual void reportFileError(const QString& summary, const std::string& path,
                                 const std::string& reason) = 0;
    /// Tell the user what of @p fileName was left out when it was read
    /// (@p items, each with why), and that its document is not saved over it.
    virtual void reportLeftOut(const QString& fileName, const std::vector<std::string>& items) = 0;
    /// Tell the user what reading @p fileName left out or changed; with
    /// @p notSavedOver, that its document is not saved over it.
    virtual void showImportReport(const QString& fileName, const io::ImportReport& report,
                                  bool notSavedOver = false) = 0;
    /// Whether @p fileName may be written: asked when it is the file the
    /// active document was read from in part, which it would lose.
    virtual bool mayReplaceSource(const QString& fileName) = 0;

    /// The active tab's title.
    virtual QString currentTitle() = 0;
    /// The panels (layers, properties, the trees) and the view shown again
    /// for the active document, changed by more than a command's step.
    virtual void refreshPanels() = 0;
    /// Build the active part again, on a worker when it is slow, and show it.
    virtual void rebuildModel() = 0;

    /// Add @p feature at the end of the active part's history, as one
    /// undoable step (with @p wrapperSketch, a profile sketch made for it),
    /// and build the part, on a worker when builds are slow. A feature that
    /// fails itself is withdrawn when its build is shown, and why is said
    /// with @p verb, its command. False when it was refused at once.
    virtual bool addFeature(std::unique_ptr<doc::Feature> feature, const QString& verb,
                            const std::shared_ptr<doc::Sketch>& wrapperSketch = nullptr) = 0;
    /// The sketch chosen in the sketch list, or the one last made or
    /// finished; null when there is none.
    virtual std::shared_ptr<doc::Sketch> chosenSketch() = 0;
    /// Stop editing the sketch edited, as Finish Sketch does, making it the
    /// chosen one.
    virtual void finishSketch() = 0;

    /// Whether work of this size goes to a worker thread: the window's
    /// rebuild mode decides (Always; Auto for @p large work; Never).
    virtual bool onWorker(bool large) = 0;
    /// Background work started or ended: the busy indicator follows.
    virtual void backgroundWorkChanged() = 0;

    /// Make @p tool, made by a workbench, the view's active tool: the window
    /// keeps it until another of its name replaces it.
    virtual void runTool(std::unique_ptr<Tool> tool) = 0;
    /// Back to the Select tool, as when a command's clicks are done. Not from
    /// inside the tool's own mouse event, which would end it mid-event.
    virtual void endTool() = 0;
};

}  // namespace hz::ui
