#pragma once

#include <QDockWidget>
#include <cstdint>
#include <string>
#include <vector>

class QAction;
class QComboBox;
class QLabel;
class QListWidget;
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

    /// A sketch as the panel lists it.
    struct SketchRow {
        uint64_t id = 0;
        std::string name;
        std::string usedBy;  ///< the feature that uses it, or empty
        std::string face;    ///< the face it follows (Phase 157), or empty
        bool editing = false;
    };
    /// List the document's sketches, above the features (hidden when there
    /// are none). @p selected is the sketch Extrude and Revolve would use.
    void refreshSketches(const std::vector<SketchRow>& rows, uint64_t selected);

    /// Mark a feature row as failed with a red background and error tooltip.
    void markFailed(int featureIndex, const std::string& errorMessage);

    /// Clear all failure markings.
    void clearFailures();

    /// Current rollback index (-1 = no rollback, all features active).
    int rollbackIndex() const { return m_rollbackIndex; }

    /// Set the rollback index.  Features after this index are grayed out.
    void setRollbackIndex(int index);

    /// The part's configurations and the active one, "" for none (Phase
    /// 156): chosen above the features. Hidden when it has none.
    void setConfigurations(const std::vector<std::string>& names, const std::string& active);

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
    /// A configuration chosen above the features; "" for none (Phase 156).
    void configurationChosen(const QString& name);

    /// Edit the sketch (double-click, or Edit Sketch in its menu).
    void sketchEditRequested(uint64_t sketchId);
    /// The sketch chosen in the list, for Extrude and Revolve to use.
    void sketchSelected(uint64_t sketchId);
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

    QLabel* m_sketchTitle = nullptr;
    QListWidget* m_sketchList = nullptr;
    QStackedWidget* m_stack = nullptr;  ///< page 0 = tree, page 1 = empty state
    QWidget* m_configurationRow = nullptr;
    QComboBox* m_configuration = nullptr;
    QTreeWidget* m_treeWidget = nullptr;
    QAction* m_editAction = nullptr;
    QAction* m_suppressAction = nullptr;
    QAction* m_deleteAction = nullptr;
    QAction* m_rollbackHereAction = nullptr;
    QAction* m_rollForwardAction = nullptr;
    int m_rollbackIndex = -1;
};

}  // namespace hz::ui
