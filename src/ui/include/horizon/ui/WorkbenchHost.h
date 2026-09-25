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
}  // namespace hz::doc

namespace hz::ui {

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

    /// Open @p fileName in a tab, or show the tab it has; false, said to the
    /// user, when it cannot be read.
    virtual bool openPath(const QString& fileName) = 0;
    /// Tell the user a file could not be read or written, and why.
    virtual void reportFileError(const QString& summary, const std::string& path,
                                 const std::string& reason) = 0;

    /// Whether work of this size goes to a worker thread: the window's
    /// rebuild mode decides (Always; Auto for @p large work; Never).
    virtual bool onWorker(bool large) = 0;
    /// Background work started or ended: the busy indicator follows.
    virtual void backgroundWorkChanged() = 0;
};

}  // namespace hz::ui
