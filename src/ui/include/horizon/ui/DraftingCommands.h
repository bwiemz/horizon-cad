#pragma once

#include <QObject>
#include <cstdint>
#include <vector>

#include "horizon/ui/Clipboard.h"

namespace hz::ui {

class WorkbenchHost;

/// The 2D drafting workbench's commands on what is selected: copy, cut,
/// paste and duplicate; rectangular and polar arrays; blocks (create,
/// insert, explode); groups; and the drawing's dimension style. Each works
/// through its WorkbenchHost on the active tab's drawing and the view's
/// selection. The window holds the menus and the ribbon, and the drawing
/// tools, which it switches between itself.
class DraftingCommands : public QObject {
    Q_OBJECT
public:
    explicit DraftingCommands(WorkbenchHost& host, QObject* parent = nullptr);

    /// What Copy and Cut hold, for the Paste tool the window registers.
    Clipboard& clipboard() { return m_clipboard; }

    // --- Edit ---
    /// The selection copied, moved over by one unit each way.
    void onDuplicate();
    void onCopy();
    void onCut();
    /// The Paste tool, when something is copied.
    void onPaste();
    void onGroupEntities();
    void onUngroupEntities();

    // --- Tools ---
    void onRectangularArray();
    void onPolarArray();

    // --- Dimensions and blocks ---
    void onDimensionStyle();
    void onCreateBlock();
    /// The Insert Block tool, for a block chosen in a form.
    void onInsertBlock();
    void onExplode();

private:
    /// The ids of the selected entities on layers that are shown and
    /// unlocked, in the drawing's order: what a command may change.
    std::vector<uint64_t> editableSelection();
    /// Select @p ids alone, and show the view and the properties again.
    void selectOnly(const std::vector<uint64_t>& ids);

    WorkbenchHost& m_host;
    Clipboard m_clipboard;
};

}  // namespace hz::ui
