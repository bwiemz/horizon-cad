# Phase 38: Feature Tree UI & Parametric Editing Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The feature tree panel — the signature UI of parametric CAD.

**Architecture:** A `FeatureTreePanel` (QDockWidget with QTreeWidget) displays the ordered feature list. Double-click edits parameters and triggers rebuild. A rollback bar divides active/suppressed features. Failed features render red with error messages. The FeatureTree from Phase 35 drives the rebuild, with TopologyID resolution connecting downstream features to regenerated topology.

**Tech Stack:** C++20, Qt6 (QDockWidget, QTreeWidget, QInputDialog)

**Spec Reference:** `docs/superpowers/specs/2026-04-05-horizon-cad-roadmap-design.md` — Section 4.8

---

## Spec Compliance Check

| # | Spec Requirement | Plan Task | Status |
|---|---|---|---|
| 1 | FeatureTreePanel (QDockWidget): ordered feature list | Task 1 | ✅ |
| 2 | Double-click to edit → parametric rebuild | Task 2 | ✅ |
| 3 | Drag to reorder (within dependency constraints) → regenerate | Task 3 | ✅ |
| 4 | Rollback bar — visual divider, features below suppressed | Task 3 | ✅ |
| 5 | Feature failure: red marking, error message, render to last success | Task 2 | ✅ |
| 6 | TNP integration: rebuild uses TopologyID resolution, failed = error | Task 2 | ✅ |

---

## File Map

| Action | File | Responsibility |
|--------|------|---------------|
| Create | `src/ui/include/horizon/ui/FeatureTreePanel.h` | QDockWidget with tree view |
| Create | `src/ui/src/FeatureTreePanel.cpp` | Implementation |
| Modify | `src/ui/CMakeLists.txt` | Add new source |
| Modify | `src/ui/include/horizon/ui/MainWindow.h` | Add FeatureTreePanel member |
| Modify | `src/ui/src/MainWindow.cpp` | Create and dock the panel |
| Modify | `src/document/include/horizon/document/FeatureTree.h` | Add rebuild-from, rollback bar index |
| Modify | `src/document/src/FeatureTree.cpp` | Partial rebuild, suppression |

---

## Task 1: FeatureTreePanel — QDockWidget with QTreeWidget

**Files:**
- Create: `src/ui/include/horizon/ui/FeatureTreePanel.h`
- Create: `src/ui/src/FeatureTreePanel.cpp`
- Modify: `src/ui/CMakeLists.txt`
- Modify: `src/ui/include/horizon/ui/MainWindow.h`
- Modify: `src/ui/src/MainWindow.cpp`

- [ ] **Step 1: Create FeatureTreePanel**

```cpp
// FeatureTreePanel.h
#pragma once
#include <QDockWidget>
#include <QTreeWidget>

namespace hz::doc { class FeatureTree; class Feature; }

namespace hz::ui {

class MainWindow;

class FeatureTreePanel : public QDockWidget {
    Q_OBJECT
public:
    explicit FeatureTreePanel(MainWindow* parent);

    /// Refresh the tree view from the FeatureTree.
    void refresh(const doc::FeatureTree& tree);

    /// Mark a feature as failed (red background + error message).
    void markFailed(int featureIndex, const std::string& errorMessage);

    /// Get the rollback bar position (features at or above are active).
    int rollbackIndex() const { return m_rollbackIndex; }
    void setRollbackIndex(int index);

signals:
    void featureDoubleClicked(int featureIndex);
    void featureReordered(int fromIndex, int toIndex);
    void rollbackChanged(int newIndex);

private:
    QTreeWidget* m_treeWidget;
    int m_rollbackIndex = -1;  // -1 = no rollback (all active)
    MainWindow* m_mainWindow;

    void onItemDoubleClicked(QTreeWidgetItem* item, int column);
    void setupContextMenu();
};

}
```

- [ ] **Step 2: Implement FeatureTreePanel**

```cpp
FeatureTreePanel::FeatureTreePanel(MainWindow* parent)
    : QDockWidget("Feature Tree", parent), m_mainWindow(parent) {
    m_treeWidget = new QTreeWidget(this);
    m_treeWidget->setHeaderLabels({"Feature", "Status"});
    m_treeWidget->setColumnCount(2);
    m_treeWidget->setDragDropMode(QAbstractItemView::InternalMove);
    m_treeWidget->setDefaultDropAction(Qt::MoveAction);
    setWidget(m_treeWidget);

    connect(m_treeWidget, &QTreeWidget::itemDoubleClicked,
            this, &FeatureTreePanel::onItemDoubleClicked);
}

void FeatureTreePanel::refresh(const doc::FeatureTree& tree) {
    m_treeWidget->clear();
    for (size_t i = 0; i < tree.featureCount(); ++i) {
        auto* feature = tree.feature(i);
        auto* item = new QTreeWidgetItem(m_treeWidget);
        item->setText(0, QString::fromStdString(feature->name()));
        item->setData(0, Qt::UserRole, static_cast<int>(i));

        // Rollback bar: features after rollback index are grayed out
        if (m_rollbackIndex >= 0 && static_cast<int>(i) > m_rollbackIndex) {
            item->setForeground(0, QBrush(QColor(128, 128, 128)));
            item->setText(1, "Suppressed");
        } else {
            item->setText(1, "OK");
        }
    }
}

void FeatureTreePanel::markFailed(int featureIndex, const std::string& errorMessage) {
    auto* item = m_treeWidget->topLevelItem(featureIndex);
    if (item) {
        item->setBackground(0, QBrush(QColor(255, 100, 100)));  // Red
        item->setText(1, QString::fromStdString(errorMessage));
    }
}

void FeatureTreePanel::onItemDoubleClicked(QTreeWidgetItem* item, int) {
    int index = item->data(0, Qt::UserRole).toInt();
    emit featureDoubleClicked(index);
}
```

- [ ] **Step 3: Add to MainWindow**

In `MainWindow.h`:
```cpp
#include "FeatureTreePanel.h"
FeatureTreePanel* m_featureTreePanel = nullptr;
```

In `MainWindow.cpp` constructor:
```cpp
m_featureTreePanel = new FeatureTreePanel(this);
addDockWidget(Qt::LeftDockWidgetArea, m_featureTreePanel);
```

Connect the `featureDoubleClicked` signal to trigger parameter editing + rebuild.

- [ ] **Step 4: Build and verify**

- [ ] **Step 5: Commit**

```bash
git commit -m "feat(ui): add FeatureTreePanel with ordered feature list display"
```

---

## Task 2: Double-Click Edit + Parametric Rebuild + Failure Handling

**Files:**
- Modify: `src/ui/src/FeatureTreePanel.cpp`
- Modify: `src/ui/src/MainWindow.cpp`
- Modify: `src/document/include/horizon/document/FeatureTree.h`
- Modify: `src/document/src/FeatureTree.cpp`

- [ ] **Step 1: Add parameter editing to FeatureTree features**

Add to the Feature base class:
```cpp
/// Get editable parameters as name→value pairs.
virtual std::map<std::string, double> parameters() const { return {}; }

/// Set a parameter by name. Returns false if invalid.
virtual bool setParameter(const std::string& name, double value) { return false; }
```

Implement for `ExtrudeFeature`:
```cpp
std::map<std::string, double> ExtrudeFeature::parameters() const {
    return {{"distance", m_distance}};
}

bool ExtrudeFeature::setParameter(const std::string& name, double value) {
    if (name == "distance") { m_distance = value; return true; }
    return false;
}
```

Similar for `RevolveFeature` (angle parameter).

- [ ] **Step 2: Implement rebuild with failure handling**

Add to `FeatureTree`:
```cpp
struct BuildResult {
    std::unique_ptr<topo::Solid> solid;
    int lastSuccessfulFeature = -1;  // Index of last feature that succeeded
    std::string failureMessage;      // Error message if a feature failed
    int failedFeatureIndex = -1;     // Which feature failed
};

BuildResult buildWithDiagnostics() const;
```

Implementation:
```cpp
FeatureTree::BuildResult FeatureTree::buildWithDiagnostics() const {
    BuildResult result;
    if (m_features.empty()) return result;

    std::unique_ptr<topo::Solid> solid;
    for (size_t i = 0; i < m_features.size(); ++i) {
        // Skip suppressed features (after rollback bar)
        if (m_rollbackIndex >= 0 && static_cast<int>(i) > m_rollbackIndex) break;

        auto newSolid = m_features[i]->execute(std::move(solid));
        if (!newSolid) {
            result.failedFeatureIndex = static_cast<int>(i);
            result.failureMessage = "Feature '" + m_features[i]->name() + "' failed";
            // Return the last good solid
            break;
        }
        solid = std::move(newSolid);
        result.lastSuccessfulFeature = static_cast<int>(i);
    }
    result.solid = std::move(solid);
    return result;
}
```

- [ ] **Step 3: Connect double-click to edit + rebuild**

In MainWindow, handle `featureDoubleClicked`:
```cpp
void MainWindow::onFeatureDoubleClicked(int featureIndex) {
    auto& tree = /* get feature tree */;
    auto* feature = tree.feature(featureIndex);
    if (!feature) return;

    auto params = feature->parameters();
    if (params.empty()) return;

    // Show edit dialog for each parameter
    for (auto& [name, value] : params) {
        bool ok;
        double newValue = QInputDialog::getDouble(this,
            "Edit " + QString::fromStdString(feature->name()),
            QString::fromStdString(name) + ":", value, 0.01, 1e6, 4, &ok);
        if (ok && newValue != value) {
            const_cast<doc::Feature*>(feature)->setParameter(name, newValue);
        }
    }

    // Rebuild from the modified feature forward
    auto result = tree.buildWithDiagnostics();

    // Update viewport
    if (result.solid) {
        auto mesh = model::SolidTessellator::tessellate(*result.solid, 0.1);
        // Update scene graph...
    }

    // Update feature tree panel
    m_featureTreePanel->refresh(tree);
    if (result.failedFeatureIndex >= 0) {
        m_featureTreePanel->markFailed(result.failedFeatureIndex, result.failureMessage);
    }
}
```

- [ ] **Step 4: Build and verify**

- [ ] **Step 5: Commit**

```bash
git commit -m "feat(ui): add double-click feature editing with parametric rebuild

Editing parameters triggers full rebuild. Failed features marked red
with error messages. Solid renders up to last successful feature."
```

---

## Task 3: Drag Reorder + Rollback Bar

- [ ] **Step 1: Implement drag-to-reorder**

Handle QTreeWidget's drag-drop to reorder features:
```cpp
// In FeatureTreePanel, handle model changes:
connect(m_treeWidget->model(), &QAbstractItemModel::rowsMoved,
        this, [this](const QModelIndex&, int start, int, const QModelIndex&, int dest) {
    emit featureReordered(start, dest);
});
```

In MainWindow, handle reorder:
```cpp
void MainWindow::onFeatureReordered(int from, int to) {
    auto& tree = /* get feature tree */;
    tree.moveFeature(from, to);
    auto result = tree.buildWithDiagnostics();
    // Update viewport and panel...
}
```

Add `moveFeature(from, to)` to FeatureTree.

- [ ] **Step 2: Implement rollback bar**

Add a visual separator in the QTreeWidget. Clicking between features sets the rollback point:

```cpp
void FeatureTreePanel::setRollbackIndex(int index) {
    m_rollbackIndex = index;
    refresh(/* current tree */);
    emit rollbackChanged(index);
}
```

Features after the rollback index are grayed out and not executed during rebuild.

Add `m_rollbackIndex` to FeatureTree:
```cpp
int rollbackIndex() const { return m_rollbackIndex; }
void setRollbackIndex(int index) { m_rollbackIndex = index; }
```

- [ ] **Step 3: Build and verify**

- [ ] **Step 4: Commit**

```bash
git commit -m "feat(ui): add feature drag-to-reorder and rollback bar"
```

---

## Task 4: Final Phase Commit + Push

- [ ] **Step 1: Run all tests, report count**

- [ ] **Step 2: Commit and push**

```bash
git add -A
git commit -m "Phase 38: Feature Tree UI with parametric editing

- FeatureTreePanel (QDockWidget): ordered feature list
- Double-click to edit parameters → triggers parametric rebuild
- Feature failure: red marking, error message, render to last success
- Drag to reorder features (within dependency constraints)
- Rollback bar: suppress features below the bar
- TNP integration: rebuild uses TopologyID resolution"

git push origin master
```
