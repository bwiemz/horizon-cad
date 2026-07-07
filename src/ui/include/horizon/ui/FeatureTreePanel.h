#pragma once

#include <QDockWidget>
#include <string>

class QTreeWidget;
class QTreeWidgetItem;
class QStackedWidget;

namespace hz::doc {
class FeatureTree;
}

namespace hz::ui {

/// Dock panel that displays the ordered list of features in the FeatureTree.
///
/// Supports rollback (suppressing features beyond a given index), failure
/// highlighting, double-click editing, and drag-reorder.
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

signals:
    void featureDoubleClicked(int featureIndex);
    void featureReordered(int fromIndex, int toIndex);
    void rollbackChanged(int newIndex);

    /// Emitted by the empty-state action buttons so the owner can start work.
    void createBoxRequested();
    void openFileRequested();

private slots:
    void onItemDoubleClicked(QTreeWidgetItem* item, int column);

private:
    QStackedWidget* m_stack = nullptr;  ///< page 0 = tree, page 1 = empty state
    QTreeWidget* m_treeWidget = nullptr;
    int m_rollbackIndex = -1;
};

}  // namespace hz::ui
