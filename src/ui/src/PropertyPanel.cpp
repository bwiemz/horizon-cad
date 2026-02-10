#include "horizon/ui/PropertyPanel.h"
#include "horizon/ui/MainWindow.h"
#include "horizon/ui/ViewportWidget.h"
#include "horizon/document/Document.h"
#include "horizon/document/Commands.h"
#include "horizon/document/UndoStack.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftRectangle.h"
#include "horizon/drafting/DraftPolyline.h"

#include <QColorDialog>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>

namespace hz::ui {

PropertyPanel::PropertyPanel(MainWindow* mainWindow, QWidget* parent)
    : QDockWidget(tr("Properties"), parent)
    , m_mainWindow(mainWindow) {
    setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    createWidgets();
}

void PropertyPanel::createWidgets() {
    auto* container = new QWidget(this);
    auto* layout = new QVBoxLayout(container);
    auto* form = new QFormLayout();

    m_typeLabel = new QLabel(tr("No selection"), this);
    form->addRow(tr("Type:"), m_typeLabel);

    m_layerCombo = new QComboBox(this);
    m_layerCombo->setEnabled(false);
    connect(m_layerCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &PropertyPanel::onLayerChanged);
    form->addRow(tr("Layer:"), m_layerCombo);

    auto* colorLayout = new QHBoxLayout();
    m_colorButton = new QPushButton(this);
    m_colorButton->setFixedSize(40, 24);
    m_colorButton->setEnabled(false);
    connect(m_colorButton, &QPushButton::clicked, this, &PropertyPanel::onColorClicked);
    colorLayout->addWidget(m_colorButton);

    m_byLayerButton = new QPushButton(tr("ByLayer"), this);
    m_byLayerButton->setFixedHeight(24);
    m_byLayerButton->setEnabled(false);
    connect(m_byLayerButton, &QPushButton::clicked, this, &PropertyPanel::onByLayerColorClicked);
    colorLayout->addWidget(m_byLayerButton);
    form->addRow(tr("Color:"), colorLayout);

    m_lineWidthSpin = new QDoubleSpinBox(this);
    m_lineWidthSpin->setRange(0.0, 10.0);
    m_lineWidthSpin->setSingleStep(0.5);
    m_lineWidthSpin->setDecimals(1);
    m_lineWidthSpin->setSpecialValueText(tr("ByLayer"));
    m_lineWidthSpin->setEnabled(false);
    connect(m_lineWidthSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &PropertyPanel::onLineWidthChanged);
    form->addRow(tr("Width:"), m_lineWidthSpin);

    layout->addLayout(form);
    layout->addStretch();
    setWidget(container);
}

void PropertyPanel::refreshLayerList() {
    if (!m_mainWindow) return;
    auto* viewport = m_mainWindow->findChild<ViewportWidget*>();
    if (!viewport || !viewport->document()) return;

    m_updatingUI = true;
    QString currentText = m_layerCombo->currentText();
    m_layerCombo->clear();
    for (const auto& name : viewport->document()->layerManager().layerNames()) {
        m_layerCombo->addItem(QString::fromStdString(name));
    }
    int idx = m_layerCombo->findText(currentText);
    if (idx >= 0) m_layerCombo->setCurrentIndex(idx);
    m_updatingUI = false;
}

void PropertyPanel::updateForSelection(const std::vector<uint64_t>& selectedIds) {
    m_currentIds = selectedIds;

    if (selectedIds.empty()) {
        m_typeLabel->setText(tr("No selection"));
        m_layerCombo->setEnabled(false);
        m_colorButton->setEnabled(false);
        m_byLayerButton->setEnabled(false);
        m_lineWidthSpin->setEnabled(false);
        m_colorButton->setStyleSheet("");
        return;
    }

    auto* viewport = m_mainWindow->findChild<ViewportWidget*>();
    if (!viewport || !viewport->document()) return;

    auto& doc = viewport->document()->draftDocument();

    // Find first selected entity.
    const draft::DraftEntity* first = nullptr;
    for (const auto& e : doc.entities()) {
        if (e->id() == selectedIds.front()) {
            first = e.get();
            break;
        }
    }
    if (!first) {
        m_typeLabel->setText(tr("No selection"));
        m_layerCombo->setEnabled(false);
        m_colorButton->setEnabled(false);
        m_byLayerButton->setEnabled(false);
        m_lineWidthSpin->setEnabled(false);
        m_colorButton->setStyleSheet("");
        m_currentIds.clear();
        return;
    }

    m_updatingUI = true;

    // Type label.
    if (selectedIds.size() == 1) {
        QString typeName = "Entity";
        if (dynamic_cast<const draft::DraftLine*>(first)) typeName = "Line";
        else if (dynamic_cast<const draft::DraftCircle*>(first)) typeName = "Circle";
        else if (dynamic_cast<const draft::DraftArc*>(first)) typeName = "Arc";
        else if (dynamic_cast<const draft::DraftRectangle*>(first)) typeName = "Rectangle";
        else if (dynamic_cast<const draft::DraftPolyline*>(first)) typeName = "Polyline";
        m_typeLabel->setText(typeName);
    } else {
        m_typeLabel->setText(tr("%1 entities").arg(selectedIds.size()));
    }

    // Layer combo.
    refreshLayerList();
    int layerIdx = m_layerCombo->findText(QString::fromStdString(first->layer()));
    if (layerIdx >= 0) m_layerCombo->setCurrentIndex(layerIdx);
    m_layerCombo->setEnabled(true);

    // Color button.
    uint32_t c = first->color();
    if (c == 0x00000000) {
        m_colorButton->setStyleSheet("background-color: #808080; border: 1px solid #555;");
    } else {
        int r = (c >> 16) & 0xFF;
        int g = (c >> 8) & 0xFF;
        int b = c & 0xFF;
        m_colorButton->setStyleSheet(
            QString("background-color: rgb(%1,%2,%3); border: 1px solid #555;")
                .arg(r).arg(g).arg(b));
    }
    m_colorButton->setEnabled(true);
    m_byLayerButton->setEnabled(true);

    // Line width.
    m_lineWidthSpin->setValue(first->lineWidth());
    m_lineWidthSpin->setEnabled(true);

    m_updatingUI = false;
}

void PropertyPanel::onLayerChanged(int index) {
    if (m_updatingUI || m_currentIds.empty() || index < 0) return;

    auto* viewport = m_mainWindow->findChild<ViewportWidget*>();
    if (!viewport || !viewport->document()) return;

    std::string newLayer = m_layerCombo->itemText(index).toStdString();
    auto cmd = std::make_unique<doc::ChangeEntityLayerCommand>(
        viewport->document()->draftDocument(), m_currentIds, newLayer);
    viewport->document()->undoStack().push(std::move(cmd));
    viewport->update();
}

void PropertyPanel::onColorClicked() {
    if (m_currentIds.empty()) return;

    auto* viewport = m_mainWindow->findChild<ViewportWidget*>();
    if (!viewport || !viewport->document()) return;

    QColor initial(Qt::white);
    // Find current color of first entity.
    for (const auto& e : viewport->document()->draftDocument().entities()) {
        if (e->id() == m_currentIds.front()) {
            uint32_t c = e->color();
            if (c != 0x00000000) {
                initial = QColor((c >> 16) & 0xFF, (c >> 8) & 0xFF, c & 0xFF);
            }
            break;
        }
    }

    QColor color = QColorDialog::getColor(initial, this, tr("Entity Color"));
    if (!color.isValid()) return;

    uint32_t argb = 0xFF000000u
        | (static_cast<uint32_t>(color.red()) << 16)
        | (static_cast<uint32_t>(color.green()) << 8)
        | static_cast<uint32_t>(color.blue());

    auto cmd = std::make_unique<doc::ChangeEntityColorCommand>(
        viewport->document()->draftDocument(), m_currentIds, argb);
    viewport->document()->undoStack().push(std::move(cmd));

    m_colorButton->setStyleSheet(
        QString("background-color: rgb(%1,%2,%3); border: 1px solid #555;")
            .arg(color.red()).arg(color.green()).arg(color.blue()));
    viewport->update();
}

void PropertyPanel::onByLayerColorClicked() {
    if (m_currentIds.empty()) return;

    auto* viewport = m_mainWindow->findChild<ViewportWidget*>();
    if (!viewport || !viewport->document()) return;

    auto cmd = std::make_unique<doc::ChangeEntityColorCommand>(
        viewport->document()->draftDocument(), m_currentIds, 0x00000000u);
    viewport->document()->undoStack().push(std::move(cmd));

    m_colorButton->setStyleSheet("background-color: #808080; border: 1px solid #555;");
    viewport->update();
}

void PropertyPanel::onLineWidthChanged(double value) {
    if (m_updatingUI || m_currentIds.empty()) return;

    auto* viewport = m_mainWindow->findChild<ViewportWidget*>();
    if (!viewport || !viewport->document()) return;

    auto cmd = std::make_unique<doc::ChangeEntityLineWidthCommand>(
        viewport->document()->draftDocument(), m_currentIds, value);
    viewport->document()->undoStack().push(std::move(cmd));
    viewport->update();
}

}  // namespace hz::ui
