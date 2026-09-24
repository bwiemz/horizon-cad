#pragma once

#include <QDockWidget>
#include <string>

class QAction;
class QTreeWidget;
class QTreeWidgetItem;
class QStackedWidget;

namespace hz::doc {
class FeatureTree;
}

namespace hz::ui {

/// Dock panel that displays the ordered list of features in the FeatureTree.
///
/// Supports rollback (leaving out the features beyond a given index), failure
/// highlighting, drag-reorder, and — from the context menu, double-click and
/// the Delete key — editing, suppressing and deleting a feature. The panel
/// only asks; the owner makes each change (as an undoable command).
class FeatureTreePanel : public QDockWidget {
    Q_OBJECT

public:
    explicit FeatureTreePanel(QWidget* parent = nullptr);

    /// Rebuild the tree widget contents from the given FeatureTree.
    void refresh(const doc::FeatureTree& tree);

    /// Mark a feature row as failed with a red background and error tooltip.
    void markFailed(int featureIndex, const std::string& errorMessage);

    /// Clear all failure markings.
    void clearFailures();

    /// Current rollback index (-1 = no rollback, all features active).
    int rollbackIndex() const { return m_rollbackIndex; }

    /// Set the rollback index.  Features after this index are grayed out.
    void setRollbackIndex(int index);

    /// Where a feature dragged from row `from` ends up (its index after the
    /// move) when dropped over row `overRow` — on that row's lower half when
    /// `belowMiddle` — or below every row when `overRow` is -1, in a list of
    /// `count` rows.
    static int dropDestination(int from, int overRow, bool belowMiddle, int count);

signals:
    /// Edit the feature (double-click, or Edit… in the menu).
    void featureDoubleClicked(int featureIndex);
    void featureReordered(int fromIndex, int toIndex);
    void featureDeleteRequested(int featureIndex);
    void featureSuppressRequested(int featureIndex, bool suppress);
    void rollbackChanged(int newIndex);

    /// Emitted by the empty-state action buttons so the owner can start work.
    void createBoxRequested();
    void openFileRequested();

private slots:
    void onItemDoubleClicked(QTreeWidgetItem* item, int column);

private:
    /// Row of the current feature, or -1.
    int currentRow() const;
    /// Enable the actions for the current feature, and name the suppress one.
    void updateActions();

    QStackedWidget* m_stack = nullptr;  ///< page 0 = tree, page 1 = empty state
    QTreeWidget* m_treeWidget = nullptr;
    QAction* m_editAction = nullptr;
    QAction* m_suppressAction = nullptr;
    QAction* m_deleteAction = nullptr;
    int m_rollbackIndex = -1;
};

}  // namespace hz::ui
