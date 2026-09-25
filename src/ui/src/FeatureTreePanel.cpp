#include "horizon/ui/FeatureTreePanel.h"

#include <QAction>
#include <QComboBox>
#include <QDropEvent>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QKeySequence>
#include <QLabel>
#include <QListWidget>
#include <QMenu>
#include <QPushButton>
#include <QSignalBlocker>
#include <QStackedWidget>
#include <QTimer>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QVBoxLayout>
#include <algorithm>
#include <functional>

#include "horizon/document/FeatureTree.h"

namespace hz::ui {

namespace {

/// The feature list. A drop does not move its rows: it reports which feature
/// goes where, and the owner — which makes the move, as a command on the
/// feature tree — rebuilds the rows from the tree, so the panel cannot show an
/// order the tree does not have. (The panel used to let QTreeWidget move the
/// row and listen for the model's rowsMoved; QTreeWidget moves a row by taking
/// the item out and inserting it again, which is not reported as a move.)
class FeatureList : public QTreeWidget {
public:
    using QTreeWidget::QTreeWidget;

    std::function<void(int from, int to)> onMove;

protected:
    void dropEvent(QDropEvent* event) override {
        QTreeWidgetItem* dragged = currentItem();
        if (event->source() != this || dragged == nullptr) {
            event->ignore();  // only this list's own rows move
            return;
        }
        const int from = indexOfTopLevelItem(dragged);
        const QPoint pos = event->position().toPoint();
        QTreeWidgetItem* over = itemAt(pos);
        const int overRow = over ? indexOfTopLevelItem(over) : -1;
        const bool below = over != nullptr && pos.y() >= visualItemRect(over).center().y();
        const int to = FeatureTreePanel::dropDestination(from, overRow, below, topLevelItemCount());

        // Not a move as far as the drag is concerned: on a move, the view
        // deletes the dragged row once the drag returns.
        event->setDropAction(Qt::CopyAction);
        event->accept();
        stopAutoScroll();
        setState(NoState);
        viewport()->update();
        if (to != from && onMove) {
            // Once the drag has unwound: the move rebuilds these rows.
            QTimer::singleShot(0, this, [this, from, to] {
                if (onMove) onMove(from, to);
            });
        }
    }
};

}  // namespace

int FeatureTreePanel::dropDestination(int from, int overRow, bool belowMiddle, int count) {
    if (count <= 0 || from < 0 || from >= count) return from;
    // Where among the current rows the feature goes in: before the row it is
    // over, or after it when dropped on that row's lower half; after the last
    // when dropped below every row.
    const int slot = overRow < 0 ? count : overRow + (belowMiddle ? 1 : 0);
    // Its index once it has left its own place.
    const int to = slot > from ? slot - 1 : slot;
    return std::clamp(to, 0, count - 1);
}

FeatureTreePanel::FeatureTreePanel(QWidget* parent) : QDockWidget(tr("Feature Tree"), parent) {
    setObjectName("FeatureTreePanel");

    m_stack = new QStackedWidget(this);

    // -- Page 0: the feature tree --------------------------------------------
    auto* list = new FeatureList(m_stack);
    list->onMove = [this](int from, int to) { emit featureReordered(from, to); };
    m_treeWidget = list;
    m_treeWidget->setHeaderLabels({tr("Feature"), tr("Status")});
    m_treeWidget->setRootIsDecorated(false);
    m_treeWidget->setSelectionMode(QAbstractItemView::SingleSelection);
    m_treeWidget->setDragDropMode(QAbstractItemView::InternalMove);
    m_treeWidget->header()->setStretchLastSection(true);
    m_stack->addWidget(m_treeWidget);

    // -- Page 1: empty state -------------------------------------------------
    auto* empty = new QWidget(m_stack);
    auto* emptyLayout = new QVBoxLayout(empty);
    emptyLayout->setContentsMargins(20, 28, 20, 20);
    emptyLayout->setSpacing(8);
    emptyLayout->addStretch(1);

    auto* title = new QLabel(tr("No features yet"), empty);
    title->setObjectName("emptyStateTitle");
    title->setAlignment(Qt::AlignHCenter);
    emptyLayout->addWidget(title);

    auto* hint =
        new QLabel(tr("Start a sketch or create a primitive to\nbegin building your part."), empty);
    hint->setObjectName("emptyStateHint");
    hint->setAlignment(Qt::AlignHCenter);
    hint->setWordWrap(true);
    emptyLayout->addWidget(hint);
    emptyLayout->addSpacing(8);

    auto* boxBtn = new QPushButton(tr("Create Box"), empty);
    connect(boxBtn, &QPushButton::clicked, this, &FeatureTreePanel::createBoxRequested);
    emptyLayout->addWidget(boxBtn);

    auto* openBtn = new QPushButton(tr("Open File…"), empty);
    connect(openBtn, &QPushButton::clicked, this, &FeatureTreePanel::openFileRequested);
    emptyLayout->addWidget(openBtn);

    emptyLayout->addStretch(2);
    m_stack->addWidget(empty);

    m_stack->setCurrentWidget(empty);  // start empty until features exist

    // The sketches, above the features: a sketch is edited from here, and the
    // one chosen here is what Extrude and Revolve take.
    auto* page = new QWidget(this);
    auto* pageLayout = new QVBoxLayout(page);
    pageLayout->setContentsMargins(0, 0, 0, 0);
    pageLayout->setSpacing(2);
    // The configuration the part is built in, when it has any (Phase 156).
    m_configurationRow = new QWidget(page);
    auto* configurationLayout = new QHBoxLayout(m_configurationRow);
    configurationLayout->setContentsMargins(4, 4, 4, 0);
    configurationLayout->addWidget(new QLabel(tr("Configuration:"), m_configurationRow));
    m_configuration = new QComboBox(m_configurationRow);
    m_configuration->setObjectName(QStringLiteral("configuration"));
    configurationLayout->addWidget(m_configuration, 1);
    pageLayout->addWidget(m_configurationRow);
    m_configurationRow->hide();
    connect(m_configuration, &QComboBox::activated, this, [this](int index) {
        emit configurationChosen(m_configuration->itemData(index).toString());
    });
    m_sketchTitle = new QLabel(tr("Sketches"), page);
    m_sketchTitle->setContentsMargins(4, 4, 4, 0);
    pageLayout->addWidget(m_sketchTitle);
    m_sketchList = new QListWidget(page);
    m_sketchList->setObjectName(QStringLiteral("sketchList"));
    m_sketchList->setMaximumHeight(120);
    pageLayout->addWidget(m_sketchList);
    pageLayout->addWidget(m_stack, 1);
    m_sketchTitle->hide();
    m_sketchList->hide();
    setWidget(page);
    connect(m_sketchList, &QListWidget::itemDoubleClicked, this, [this](QListWidgetItem* item) {
        emit sketchEditRequested(item->data(Qt::UserRole).toULongLong());
    });
    connect(m_sketchList, &QListWidget::currentItemChanged, this,
            [this](QListWidgetItem* item, QListWidgetItem* /*previous*/) {
                if (item) emit sketchSelected(item->data(Qt::UserRole).toULongLong());
            });

    connect(m_treeWidget, &QTreeWidget::itemDoubleClicked, this,
            &FeatureTreePanel::onItemDoubleClicked);

    // Edit / Suppress / Delete act on the current feature, from the context
    // menu or (Delete) the keyboard while the tree has focus.
    m_editAction = new QAction(tr("Edit…"), this);
    m_editAction->setObjectName(QStringLiteral("editFeature"));
    connect(m_editAction, &QAction::triggered, this, [this] {
        if (currentRow() >= 0) emit featureDoubleClicked(currentRow());
    });
    m_suppressAction = new QAction(tr("Suppress"), this);
    m_suppressAction->setObjectName(QStringLiteral("suppressFeature"));
    connect(m_suppressAction, &QAction::triggered, this, [this] {
        const int row = currentRow();
        if (row < 0) return;
        const bool suppressed = m_treeWidget->topLevelItem(row)->data(0, Qt::UserRole + 1).toBool();
        emit featureSuppressRequested(row, !suppressed);
    });
    m_deleteAction = new QAction(tr("Delete"), this);
    m_deleteAction->setObjectName(QStringLiteral("deleteFeature"));
    m_deleteAction->setShortcut(QKeySequence::Delete);
    m_deleteAction->setShortcutContext(Qt::WidgetWithChildrenShortcut);
    connect(m_deleteAction, &QAction::triggered, this, [this] {
        if (currentRow() >= 0) emit featureDeleteRequested(currentRow());
    });
    m_treeWidget->addAction(m_deleteAction);
    // The part as it stood after a feature: the ones after it are left out of
    // the build, until it is rolled forward again. Each is one undo step.
    m_rollbackHereAction = new QAction(tr("Roll Back to Here"), this);
    m_rollbackHereAction->setObjectName(QStringLiteral("rollbackHere"));
    connect(m_rollbackHereAction, &QAction::triggered, this, [this] {
        if (currentRow() >= 0) emit rollbackChanged(currentRow());
    });
    m_rollForwardAction = new QAction(tr("Roll Forward to the End"), this);
    m_rollForwardAction->setObjectName(QStringLiteral("rollForward"));
    connect(m_rollForwardAction, &QAction::triggered, this, [this] { emit rollbackChanged(-1); });
    m_treeWidget->setContextMenuPolicy(Qt::CustomContextMenu);
    connect(m_treeWidget, &QWidget::customContextMenuRequested, this, [this](const QPoint& pos) {
        QTreeWidgetItem* item = m_treeWidget->itemAt(pos);
        if (!item) return;
        m_treeWidget->setCurrentItem(item);
        QMenu menu(this);
        menu.addAction(m_editAction);
        menu.addAction(m_suppressAction);
        menu.addSeparator();
        menu.addAction(m_rollbackHereAction);
        menu.addAction(m_rollForwardAction);
        menu.addSeparator();
        menu.addAction(m_deleteAction);
        menu.exec(m_treeWidget->viewport()->mapToGlobal(pos));
    });
    connect(m_treeWidget, &QTreeWidget::currentItemChanged, this, [this] { updateActions(); });
    updateActions();
}

void FeatureTreePanel::refresh(const doc::FeatureTree& tree) {
    const int current = currentRow();
    m_treeWidget->clear();
    m_rollbackIndex = tree.rollbackIndex();

    const int count = static_cast<int>(tree.featureCount());
    m_stack->setCurrentIndex(count > 0 ? 0 : 1);  // tree vs empty state
    for (int i = 0; i < count; ++i) {
        const auto* feat = tree.feature(static_cast<size_t>(i));
        auto* item = new QTreeWidgetItem(m_treeWidget);
        item->setData(0, Qt::UserRole, i);
        item->setData(0, Qt::UserRole + 1, feat->isSuppressed());
        item->setText(0, QString::fromStdString(feat->name()));
        // Features are dropped between rows, never onto one another: a drop
        // onto an item would nest it in the panel without changing the tree.
        item->setFlags(item->flags() & ~Qt::ItemIsDropEnabled);

        const bool rolledBack = (m_rollbackIndex >= 0 && i > m_rollbackIndex);
        if (feat->isSuppressed() || rolledBack) {
            item->setText(1, feat->isSuppressed() ? tr("Suppressed") : tr("Rolled back"));
            item->setForeground(0, QColor(160, 160, 160));
            item->setForeground(1, QColor(160, 160, 160));
        } else {
            item->setText(1, tr("OK"));
        }
    }
    if (current >= 0 && current < count) {
        m_treeWidget->setCurrentItem(m_treeWidget->topLevelItem(current));
    }
    updateActions();
}

void FeatureTreePanel::refreshSketches(const std::vector<SketchRow>& rows, uint64_t selected) {
    const QSignalBlocker quiet(m_sketchList);  // listing is not choosing
    m_sketchList->clear();
    for (const SketchRow& row : rows) {
        QStringList notes;
        if (!row.face.empty()) notes << tr("on a face");
        if (row.editing) {
            notes << tr("editing");
        } else if (!row.usedBy.empty()) {
            notes << tr("used by %1").arg(QString::fromStdString(row.usedBy));
        }
        QString text = QString::fromStdString(row.name);
        if (!notes.isEmpty()) text += QStringLiteral(" (%1)").arg(notes.join(QStringLiteral(", ")));
        auto* item = new QListWidgetItem(text, m_sketchList);
        if (!row.face.empty()) {
            item->setToolTip(tr("Follows the part's face %1: when the part changes, it moves "
                                "with the face")
                                 .arg(QString::fromStdString(row.face)));
        }
        item->setData(Qt::UserRole, QVariant::fromValue<qulonglong>(row.id));
        if (row.id == selected) m_sketchList->setCurrentItem(item);
    }
    m_sketchTitle->setVisible(!rows.empty());
    m_sketchList->setVisible(!rows.empty());
}

int FeatureTreePanel::currentRow() const {
    QTreeWidgetItem* item = m_treeWidget->currentItem();
    return item ? m_treeWidget->indexOfTopLevelItem(item) : -1;
}

void FeatureTreePanel::updateActions() {
    const int row = currentRow();
    const bool has = row >= 0;
    m_editAction->setEnabled(has);
    m_suppressAction->setEnabled(has);
    m_deleteAction->setEnabled(has);
    const int count = m_treeWidget->topLevelItemCount();
    m_rollbackHereAction->setEnabled(has && row != m_rollbackIndex && row < count - 1);
    m_rollForwardAction->setEnabled(m_rollbackIndex >= 0);
    const bool suppressed =
        has && m_treeWidget->topLevelItem(row)->data(0, Qt::UserRole + 1).toBool();
    m_suppressAction->setText(suppressed ? tr("Unsuppress") : tr("Suppress"));
}

void FeatureTreePanel::markFailed(int featureIndex, const std::string& errorMessage) {
    if (featureIndex < 0 || featureIndex >= m_treeWidget->topLevelItemCount()) return;

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

namespace hz::ui {

void FeatureTreePanel::setConfigurations(const std::vector<std::string>& names,
                                         const std::string& active) {
    const QSignalBlocker quiet(m_configuration);
    m_configuration->clear();
    m_configuration->addItem(tr("(Its own variables)"), QString());
    for (const std::string& name : names) {
        const QString text = QString::fromStdString(name);
        m_configuration->addItem(text, text);
    }
    m_configuration->setCurrentIndex(
        std::max(0, m_configuration->findData(QString::fromStdString(active))));
    m_configurationRow->setVisible(!names.empty());
}

}  // namespace hz::ui
