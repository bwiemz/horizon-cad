#include "horizon/ui/FeatureTreePanel.h"

#include "horizon/document/FeatureTree.h"

#include <QHeaderView>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QVBoxLayout>

namespace hz::ui {

FeatureTreePanel::FeatureTreePanel(QWidget* parent)
    : QDockWidget(tr("Feature Tree"), parent) {
    setObjectName("FeatureTreePanel");

    auto* container = new QWidget(this);
    auto* layout = new QVBoxLayout(container);
    layout->setContentsMargins(0, 0, 0, 0);

    m_treeWidget = new QTreeWidget(container);
    m_treeWidget->setHeaderLabels({tr("Feature"), tr("Status")});
    m_treeWidget->setRootIsDecorated(false);
    m_treeWidget->setSelectionMode(QAbstractItemView::SingleSelection);
    m_treeWidget->setDragDropMode(QAbstractItemView::InternalMove);
    m_treeWidget->header()->setStretchLastSection(true);

    layout->addWidget(m_treeWidget);
    setWidget(container);

    connect(m_treeWidget, &QTreeWidget::itemDoubleClicked,
            this, &FeatureTreePanel::onItemDoubleClicked);

    // Detect drag-drop reorder via the model's rowsMoved signal.
    connect(m_treeWidget->model(), &QAbstractItemModel::rowsMoved, this,
            [this](const QModelIndex& /*parent*/, int start, int /*end*/,
                   const QModelIndex& /*dest*/, int row) {
                int toIndex = (row > start) ? row - 1 : row;
                emit featureReordered(start, toIndex);
            });
}

void FeatureTreePanel::refresh(const doc::FeatureTree& tree) {
    m_treeWidget->clear();

    const int count = static_cast<int>(tree.featureCount());
    for (int i = 0; i < count; ++i) {
        const auto* feat = tree.feature(static_cast<size_t>(i));
        auto* item = new QTreeWidgetItem(m_treeWidget);
        item->setData(0, Qt::UserRole, i);
        item->setText(0, QString::fromStdString(feat->name()));

        const bool suppressed =
            (m_rollbackIndex >= 0 && i > m_rollbackIndex);
        if (suppressed) {
            item->setText(1, tr("Suppressed"));
            item->setForeground(0, QColor(160, 160, 160));
            item->setForeground(1, QColor(160, 160, 160));
        } else {
            item->setText(1, tr("OK"));
        }
    }
}

void FeatureTreePanel::markFailed(int featureIndex,
                                   const std::string& errorMessage) {
    if (featureIndex < 0 || featureIndex >= m_treeWidget->topLevelItemCount())
        return;

    auto* item = m_treeWidget->topLevelItem(featureIndex);
    item->setBackground(0, QColor(255, 180, 180));
    item->setBackground(1, QColor(255, 180, 180));
    item->setText(1, tr("FAILED"));
    item->setToolTip(0, QString::fromStdString(errorMessage));
    item->setToolTip(1, QString::fromStdString(errorMessage));
}

void FeatureTreePanel::clearFailures() {
    for (int i = 0; i < m_treeWidget->topLevelItemCount(); ++i) {
        auto* item = m_treeWidget->topLevelItem(i);
        item->setBackground(0, QBrush());
        item->setBackground(1, QBrush());
        item->setToolTip(0, {});
        item->setToolTip(1, {});
    }
}

void FeatureTreePanel::setRollbackIndex(int index) {
    if (m_rollbackIndex == index) return;
    m_rollbackIndex = index;
    emit rollbackChanged(index);
}

void FeatureTreePanel::onItemDoubleClicked(QTreeWidgetItem* item, int /*column*/) {
    if (!item) return;
    bool ok = false;
    int index = item->data(0, Qt::UserRole).toInt(&ok);
    if (ok) {
        emit featureDoubleClicked(index);
    }
}

}  // namespace hz::ui
