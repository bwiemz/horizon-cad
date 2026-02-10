#include "horizon/ui/LayerPanel.h"
#include "horizon/ui/MainWindow.h"
#include "horizon/ui/ViewportWidget.h"
#include "horizon/document/Document.h"
#include "horizon/document/Commands.h"
#include "horizon/document/UndoStack.h"

#include <QColorDialog>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QInputDialog>
#include <QMessageBox>
#include <QPushButton>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QVBoxLayout>
#include <QWidget>

namespace hz::ui {

LayerPanel::LayerPanel(MainWindow* mainWindow, QWidget* parent)
    : QDockWidget(tr("Layers"), parent)
    , m_mainWindow(mainWindow) {
    setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    createWidgets();
}

void LayerPanel::createWidgets() {
    auto* container = new QWidget(this);
    auto* layout = new QVBoxLayout(container);

    m_tree = new QTreeWidget(this);
    m_tree->setHeaderLabels({tr("Name"), tr("V"), tr("L"), tr("Color"), tr("Width")});
    m_tree->setRootIsDecorated(false);
    m_tree->setSelectionMode(QAbstractItemView::SingleSelection);
    m_tree->header()->setStretchLastSection(false);
    m_tree->header()->setSectionResizeMode(0, QHeaderView::Stretch);
    m_tree->header()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
    m_tree->header()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
    m_tree->header()->setSectionResizeMode(3, QHeaderView::ResizeToContents);
    m_tree->header()->setSectionResizeMode(4, QHeaderView::ResizeToContents);

    connect(m_tree, &QTreeWidget::itemDoubleClicked,
            this, &LayerPanel::onItemDoubleClicked);
    connect(m_tree, &QTreeWidget::itemChanged,
            this, &LayerPanel::onItemChanged);
    layout->addWidget(m_tree);

    auto* btnLayout = new QHBoxLayout();
    m_addBtn = new QPushButton(tr("Add"), this);
    connect(m_addBtn, &QPushButton::clicked, this, &LayerPanel::onAddLayer);
    btnLayout->addWidget(m_addBtn);

    m_deleteBtn = new QPushButton(tr("Delete"), this);
    connect(m_deleteBtn, &QPushButton::clicked, this, &LayerPanel::onDeleteLayer);
    btnLayout->addWidget(m_deleteBtn);

    auto* colorBtn = new QPushButton(tr("Color..."), this);
    connect(colorBtn, &QPushButton::clicked, this, &LayerPanel::onColorClicked);
    btnLayout->addWidget(colorBtn);

    layout->addLayout(btnLayout);
    setWidget(container);
}

void LayerPanel::refresh() {
    auto* viewport = m_mainWindow->findChild<ViewportWidget*>();
    if (!viewport || !viewport->document()) return;

    m_refreshing = true;
    m_tree->clear();

    const auto& layerMgr = viewport->document()->layerManager();
    std::string current = layerMgr.currentLayer();

    for (const auto& name : layerMgr.layerNames()) {
        const auto* lp = layerMgr.getLayer(name);
        if (!lp) continue;

        auto* item = new QTreeWidgetItem(m_tree);
        item->setText(0, QString::fromStdString(name));
        item->setData(0, Qt::UserRole, QString::fromStdString(name));

        // Bold for current layer.
        if (name == current) {
            QFont f = item->font(0);
            f.setBold(true);
            item->setFont(0, f);
            item->setText(0, QString::fromStdString(name) + " *");
        }

        // Visible checkbox.
        item->setCheckState(1, lp->visible ? Qt::Checked : Qt::Unchecked);

        // Locked checkbox.
        item->setCheckState(2, lp->locked ? Qt::Checked : Qt::Unchecked);

        // Color swatch.
        int r = (lp->color >> 16) & 0xFF;
        int g = (lp->color >> 8) & 0xFF;
        int b = lp->color & 0xFF;
        item->setBackground(3, QColor(r, g, b));
        item->setText(3, "");

        // Line width.
        item->setText(4, QString::number(lp->lineWidth, 'f', 1));
    }

    m_refreshing = false;
}

void LayerPanel::onAddLayer() {
    auto* viewport = m_mainWindow->findChild<ViewportWidget*>();
    if (!viewport || !viewport->document()) return;

    bool ok = false;
    QString name = QInputDialog::getText(this, tr("New Layer"),
                                         tr("Layer name:"), QLineEdit::Normal,
                                         QString(), &ok);
    if (!ok || name.isEmpty()) return;

    // Check if layer already exists.
    if (viewport->document()->layerManager().getLayer(name.toStdString())) {
        QMessageBox::warning(this, tr("Error"),
                             tr("Layer '%1' already exists.").arg(name));
        return;
    }

    draft::LayerProperties props;
    props.name = name.toStdString();
    props.color = 0xFFFFFFFF;
    props.lineWidth = 1.0;
    props.visible = true;
    props.locked = false;

    auto cmd = std::make_unique<doc::AddLayerCommand>(
        viewport->document()->layerManager(), props);
    viewport->document()->undoStack().push(std::move(cmd));
    refresh();
}

void LayerPanel::onDeleteLayer() {
    auto* viewport = m_mainWindow->findChild<ViewportWidget*>();
    if (!viewport || !viewport->document()) return;

    auto* item = m_tree->currentItem();
    if (!item) return;

    std::string name = item->data(0, Qt::UserRole).toString().toStdString();
    if (name == "0") {
        QMessageBox::warning(this, tr("Error"),
                             tr("Cannot delete the default layer."));
        return;
    }

    auto cmd = std::make_unique<doc::RemoveLayerCommand>(
        viewport->document()->layerManager(),
        viewport->document()->draftDocument(), name);
    viewport->document()->undoStack().push(std::move(cmd));
    refresh();
    viewport->update();
}

void LayerPanel::onItemDoubleClicked(QTreeWidgetItem* item, int column) {
    if (!item || column != 0) return;

    auto* viewport = m_mainWindow->findChild<ViewportWidget*>();
    if (!viewport || !viewport->document()) return;

    std::string name = item->data(0, Qt::UserRole).toString().toStdString();
    auto cmd = std::make_unique<doc::SetCurrentLayerCommand>(
        viewport->document()->layerManager(), name);
    viewport->document()->undoStack().push(std::move(cmd));
    refresh();
}

void LayerPanel::onItemChanged(QTreeWidgetItem* item, int column) {
    if (m_refreshing || !item) return;
    if (column != 1 && column != 2) return;

    auto* viewport = m_mainWindow->findChild<ViewportWidget*>();
    if (!viewport || !viewport->document()) return;

    std::string name = item->data(0, Qt::UserRole).toString().toStdString();
    const auto* lp = viewport->document()->layerManager().getLayer(name);
    if (!lp) return;

    draft::LayerProperties newProps = *lp;
    if (column == 1) {
        newProps.visible = (item->checkState(1) == Qt::Checked);
    } else if (column == 2) {
        newProps.locked = (item->checkState(2) == Qt::Checked);
    }

    auto cmd = std::make_unique<doc::ModifyLayerCommand>(
        viewport->document()->layerManager(), name, newProps);
    viewport->document()->undoStack().push(std::move(cmd));
    viewport->update();
}

void LayerPanel::onColorClicked() {
    auto* viewport = m_mainWindow->findChild<ViewportWidget*>();
    if (!viewport || !viewport->document()) return;

    auto* item = m_tree->currentItem();
    if (!item) return;

    std::string name = item->data(0, Qt::UserRole).toString().toStdString();
    const auto* lp = viewport->document()->layerManager().getLayer(name);
    if (!lp) return;

    int r = (lp->color >> 16) & 0xFF;
    int g = (lp->color >> 8) & 0xFF;
    int b = lp->color & 0xFF;
    QColor initial(r, g, b);

    QColor color = QColorDialog::getColor(initial, this, tr("Layer Color"));
    if (!color.isValid()) return;

    draft::LayerProperties newProps = *lp;
    newProps.color = 0xFF000000u
        | (static_cast<uint32_t>(color.red()) << 16)
        | (static_cast<uint32_t>(color.green()) << 8)
        | static_cast<uint32_t>(color.blue());

    auto cmd = std::make_unique<doc::ModifyLayerCommand>(
        viewport->document()->layerManager(), name, newProps);
    viewport->document()->undoStack().push(std::move(cmd));
    refresh();
    viewport->update();
}

}  // namespace hz::ui
