#pragma once

#include <QDockWidget>
#include <QString>
#include <cstdint>

class QAction;
class QTreeWidget;
class QTreeWidgetItem;

namespace hz::doc {
class AssemblyDocument;
}

namespace hz::ui {

/// Dock panel listing an assembly's components and mates (Phase 143), where
/// a part shows its feature tree. From its context menu and the Delete key:
/// open a component's part (Phase 144), remove, suppress or unsuppress and
/// rename a component; edit a mate's value, or remove it. Like the feature
/// tree, the panel only asks: the owner makes each change, as an undoable
/// edit, and refreshes the panel.
class AssemblyTreePanel : public QDockWidget {
    Q_OBJECT

public:
    explicit AssemblyTreePanel(QWidget* parent = nullptr);

    /// Rebuild the rows from @p assembly; empty when it is null. The current
    /// row stays current if it is still there, and @p assembly is the one
    /// the rows were of: ids start at 1 in each assembly.
    void refresh(const doc::AssemblyDocument* assembly);

    /// The component (or mate) of the current row; 0 when the row is not one.
    uint64_t currentComponent() const;
    uint64_t currentMate() const;
    /// The component pattern of the current row (Phase 161); 0 when none.
    uint64_t currentPattern() const;

    /// Make @p id's row current, without asking the owner to select it.
    void showComponent(uint64_t id);

    QTreeWidget* tree() const { return m_tree; }

protected:
    bool eventFilter(QObject* watched, QEvent* event) override;

signals:
    /// A component's row made current: select it in the view.
    void componentSelected(uint64_t id);
    void removeComponentRequested(uint64_t id);
    void suppressRequested(uint64_t id, bool suppress);
    void renameRequested(uint64_t id);
    /// Open the part a component places, in its tab (Phase 144).
    void openPartRequested(uint64_t id);
    void editMateRequested(uint64_t id);
    void removeMateRequested(uint64_t id);
    /// A component pattern's row double-clicked, or removed (Phase 161).
    void editPatternRequested(uint64_t id);
    void removePatternRequested(uint64_t id);

private:
    /// Enable the actions for the current row, and name the suppress one.
    void updateActions();

    QTreeWidget* m_tree = nullptr;
    QAction* m_removeAction = nullptr;
    QAction* m_suppressAction = nullptr;
    QAction* m_renameAction = nullptr;
    QAction* m_editMateAction = nullptr;
    QAction* m_openPartAction = nullptr;
    bool m_currentSuppressed = false;
    /// The assembly the rows are of (identity only; never dereferenced).
    const doc::AssemblyDocument* m_shown = nullptr;
};

}  // namespace hz::ui
