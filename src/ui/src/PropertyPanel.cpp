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
#include "horizon/drafting/DraftDimension.h"
#include "horizon/drafting/DraftLinearDimension.h"
#include "horizon/drafting/DraftRadialDimension.h"
#include "horizon/drafting/DraftAngularDimension.h"
#include "horizon/drafting/DraftLeader.h"
#include "horizon/drafting/DraftBlockRef.h"
#include "horizon/drafting/DraftText.h"
#include "horizon/drafting/DraftSpline.h"
#include "horizon/drafting/DraftHatch.h"
#include "horizon/drafting/DraftEllipse.h"
#include "horizon/drafting/LineType.h"
#include "horizon/math/Constants.h"
#include "horizon/constraint/ConstraintSystem.h"
#include "horizon/document/ConstraintCommands.h"

#include <QColorDialog>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
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

    m_lineTypeCombo = new QComboBox(this);
    m_lineTypeCombo->addItem(tr("ByLayer"));       // index 0
    m_lineTypeCombo->addItem(tr("Continuous"));     // index 1
    m_lineTypeCombo->addItem(tr("Dashed"));         // index 2
    m_lineTypeCombo->addItem(tr("Dotted"));         // index 3
    m_lineTypeCombo->addItem(tr("DashDot"));        // index 4
    m_lineTypeCombo->addItem(tr("Center"));         // index 5
    m_lineTypeCombo->addItem(tr("Hidden"));         // index 6
    m_lineTypeCombo->addItem(tr("Phantom"));        // index 7
    m_lineTypeCombo->setEnabled(false);
    connect(m_lineTypeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &PropertyPanel::onLineTypeChanged);
    form->addRow(tr("Line Type:"), m_lineTypeCombo);

    // Dimension-specific properties (hidden by default).
    m_dimPropsWidget = new QWidget(this);
    auto* dimForm = new QFormLayout(m_dimPropsWidget);
    dimForm->setContentsMargins(0, 0, 0, 0);

    m_textOverrideEdit = new QLineEdit(m_dimPropsWidget);
    m_textOverrideEdit->setPlaceholderText(tr("Auto"));
    connect(m_textOverrideEdit, &QLineEdit::editingFinished,
            this, &PropertyPanel::onTextOverrideChanged);
    dimForm->addRow(tr("Text:"), m_textOverrideEdit);
    m_dimPropsWidget->hide();

    // Block-ref specific properties (hidden by default).
    m_blockPropsWidget = new QWidget(this);
    auto* blockForm = new QFormLayout(m_blockPropsWidget);
    blockForm->setContentsMargins(0, 0, 0, 0);

    m_blockNameLabel = new QLabel(m_blockPropsWidget);
    blockForm->addRow(tr("Block:"), m_blockNameLabel);

    m_blockRotationSpin = new QDoubleSpinBox(m_blockPropsWidget);
    m_blockRotationSpin->setRange(-360.0, 360.0);
    m_blockRotationSpin->setDecimals(2);
    m_blockRotationSpin->setSuffix(QString::fromUtf8("\xC2\xB0"));
    connect(m_blockRotationSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &PropertyPanel::onBlockRotationChanged);
    blockForm->addRow(tr("Rotation:"), m_blockRotationSpin);

    m_blockScaleSpin = new QDoubleSpinBox(m_blockPropsWidget);
    m_blockScaleSpin->setRange(-1000.0, 1000.0);
    m_blockScaleSpin->setDecimals(3);
    connect(m_blockScaleSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &PropertyPanel::onBlockScaleChanged);
    blockForm->addRow(tr("Scale:"), m_blockScaleSpin);

    m_blockPropsWidget->hide();

    // Text entity specific properties (hidden by default).
    m_textPropsWidget = new QWidget(this);
    auto* textForm = new QFormLayout(m_textPropsWidget);
    textForm->setContentsMargins(0, 0, 0, 0);

    m_textContentEdit = new QLineEdit(m_textPropsWidget);
    connect(m_textContentEdit, &QLineEdit::editingFinished,
            this, &PropertyPanel::onTextContentChanged);
    textForm->addRow(tr("Content:"), m_textContentEdit);

    m_textHeightSpin = new QDoubleSpinBox(m_textPropsWidget);
    m_textHeightSpin->setRange(0.1, 1000.0);
    m_textHeightSpin->setDecimals(2);
    m_textHeightSpin->setSingleStep(0.5);
    connect(m_textHeightSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &PropertyPanel::onTextHeightChanged);
    textForm->addRow(tr("Height:"), m_textHeightSpin);

    m_textRotationSpin = new QDoubleSpinBox(m_textPropsWidget);
    m_textRotationSpin->setRange(-360.0, 360.0);
    m_textRotationSpin->setDecimals(2);
    m_textRotationSpin->setSuffix(QString::fromUtf8("\xC2\xB0"));
    connect(m_textRotationSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &PropertyPanel::onTextRotationChanged);
    textForm->addRow(tr("Rotation:"), m_textRotationSpin);

    m_textAlignCombo = new QComboBox(m_textPropsWidget);
    m_textAlignCombo->addItem(tr("Left"));
    m_textAlignCombo->addItem(tr("Center"));
    m_textAlignCombo->addItem(tr("Right"));
    connect(m_textAlignCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &PropertyPanel::onTextAlignmentChanged);
    textForm->addRow(tr("Align:"), m_textAlignCombo);

    m_textPropsWidget->hide();

    // Spline-specific properties (hidden by default).
    m_splinePropsWidget = new QWidget(this);
    auto* splineForm = new QFormLayout(m_splinePropsWidget);
    splineForm->setContentsMargins(0, 0, 0, 0);

    m_splinePointCountLabel = new QLabel(m_splinePropsWidget);
    splineForm->addRow(tr("Points:"), m_splinePointCountLabel);

    m_splineClosedCombo = new QComboBox(m_splinePropsWidget);
    m_splineClosedCombo->addItem(tr("Open"));
    m_splineClosedCombo->addItem(tr("Closed"));
    connect(m_splineClosedCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &PropertyPanel::onSplineClosedChanged);
    splineForm->addRow(tr("Type:"), m_splineClosedCombo);

    m_splinePropsWidget->hide();

    // Hatch-specific properties (hidden by default).
    m_hatchPropsWidget = new QWidget(this);
    auto* hatchForm = new QFormLayout(m_hatchPropsWidget);
    hatchForm->setContentsMargins(0, 0, 0, 0);

    m_hatchPatternCombo = new QComboBox(m_hatchPropsWidget);
    m_hatchPatternCombo->addItem(tr("Solid"));
    m_hatchPatternCombo->addItem(tr("Lines"));
    m_hatchPatternCombo->addItem(tr("CrossHatch"));
    connect(m_hatchPatternCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &PropertyPanel::onHatchPatternChanged);
    hatchForm->addRow(tr("Pattern:"), m_hatchPatternCombo);

    m_hatchAngleSpin = new QDoubleSpinBox(m_hatchPropsWidget);
    m_hatchAngleSpin->setRange(-360.0, 360.0);
    m_hatchAngleSpin->setDecimals(2);
    m_hatchAngleSpin->setSuffix(QString::fromUtf8("\xC2\xB0"));
    connect(m_hatchAngleSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &PropertyPanel::onHatchAngleChanged);
    hatchForm->addRow(tr("Angle:"), m_hatchAngleSpin);

    m_hatchSpacingSpin = new QDoubleSpinBox(m_hatchPropsWidget);
    m_hatchSpacingSpin->setRange(0.01, 100.0);
    m_hatchSpacingSpin->setDecimals(3);
    m_hatchSpacingSpin->setSingleStep(0.1);
    connect(m_hatchSpacingSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &PropertyPanel::onHatchSpacingChanged);
    hatchForm->addRow(tr("Spacing:"), m_hatchSpacingSpin);

    m_hatchPropsWidget->hide();

    // Ellipse-specific properties.
    m_ellipsePropsWidget = new QWidget(this);
    auto* ellipseForm = new QFormLayout(m_ellipsePropsWidget);
    ellipseForm->setContentsMargins(0, 0, 0, 0);

    m_ellipseSemiMajorSpin = new QDoubleSpinBox(m_ellipsePropsWidget);
    m_ellipseSemiMajorSpin->setRange(0.001, 1e6);
    m_ellipseSemiMajorSpin->setDecimals(4);
    m_ellipseSemiMajorSpin->setSingleStep(0.1);
    connect(m_ellipseSemiMajorSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &PropertyPanel::onEllipseSemiMajorChanged);
    ellipseForm->addRow(tr("Semi-Major:"), m_ellipseSemiMajorSpin);

    m_ellipseSemiMinorSpin = new QDoubleSpinBox(m_ellipsePropsWidget);
    m_ellipseSemiMinorSpin->setRange(0.001, 1e6);
    m_ellipseSemiMinorSpin->setDecimals(4);
    m_ellipseSemiMinorSpin->setSingleStep(0.1);
    connect(m_ellipseSemiMinorSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &PropertyPanel::onEllipseSemiMinorChanged);
    ellipseForm->addRow(tr("Semi-Minor:"), m_ellipseSemiMinorSpin);

    m_ellipseRotationSpin = new QDoubleSpinBox(m_ellipsePropsWidget);
    m_ellipseRotationSpin->setRange(-360.0, 360.0);
    m_ellipseRotationSpin->setDecimals(2);
    m_ellipseRotationSpin->setSuffix(QString::fromUtf8("\xC2\xB0"));
    connect(m_ellipseRotationSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &PropertyPanel::onEllipseRotationChanged);
    ellipseForm->addRow(tr("Rotation:"), m_ellipseRotationSpin);

    m_ellipsePropsWidget->hide();

    // Constraint info (hidden by default).
    m_constraintWidget = new QWidget(this);
    auto* cstrLayout = new QVBoxLayout(m_constraintWidget);
    cstrLayout->setContentsMargins(0, 0, 0, 0);
    cstrLayout->addWidget(new QLabel(tr("Constraints:"), m_constraintWidget));
    m_constraintList = new QListWidget(m_constraintWidget);
    m_constraintList->setMaximumHeight(120);
    cstrLayout->addWidget(m_constraintList);
    m_deleteConstraintBtn = new QPushButton(tr("Delete Constraint"), m_constraintWidget);
    m_deleteConstraintBtn->setEnabled(false);
    connect(m_deleteConstraintBtn, &QPushButton::clicked, this, &PropertyPanel::onDeleteConstraint);
    connect(m_constraintList, &QListWidget::currentRowChanged,
            this, [this](int row) { m_deleteConstraintBtn->setEnabled(row >= 0); });
    cstrLayout->addWidget(m_deleteConstraintBtn);
    m_constraintWidget->hide();

    layout->addLayout(form);
    layout->addWidget(m_dimPropsWidget);
    layout->addWidget(m_blockPropsWidget);
    layout->addWidget(m_textPropsWidget);
    layout->addWidget(m_splinePropsWidget);
    layout->addWidget(m_hatchPropsWidget);
    layout->addWidget(m_ellipsePropsWidget);
    layout->addWidget(m_constraintWidget);
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
        m_lineTypeCombo->setEnabled(false);
        m_colorButton->setStyleSheet("");
        m_dimPropsWidget->hide();
        m_blockPropsWidget->hide();
        m_textPropsWidget->hide();
        m_splinePropsWidget->hide();
        m_hatchPropsWidget->hide();
        m_ellipsePropsWidget->hide();
        m_constraintWidget->hide();
        return;
    }

    auto* viewport = m_mainWindow->findChild<ViewportWidget*>();
    if (!viewport || !viewport->document()) return;

    auto& doc = viewport->document()->draftDocument();

    // Find first selected entity.
    const draft::DraftEntity* first = nullptr;
    if (selectedIds.empty()) return;  // Defensive guard.
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
        m_lineTypeCombo->setEnabled(false);
        m_colorButton->setStyleSheet("");
        m_dimPropsWidget->hide();
        m_blockPropsWidget->hide();
        m_textPropsWidget->hide();
        m_splinePropsWidget->hide();
        m_hatchPropsWidget->hide();
        m_ellipsePropsWidget->hide();
        m_constraintWidget->hide();
        m_currentIds.clear();
        return;
    }

    m_updatingUI = true;

    // Type label.
    const draft::DraftDimension* dimEntity = nullptr;
    if (selectedIds.size() == 1) {
        QString typeName = "Entity";
        if (dynamic_cast<const draft::DraftLine*>(first)) typeName = "Line";
        else if (dynamic_cast<const draft::DraftCircle*>(first)) typeName = "Circle";
        else if (dynamic_cast<const draft::DraftArc*>(first)) typeName = "Arc";
        else if (dynamic_cast<const draft::DraftRectangle*>(first)) typeName = "Rectangle";
        else if (dynamic_cast<const draft::DraftPolyline*>(first)) typeName = "Polyline";
        else if (dynamic_cast<const draft::DraftLinearDimension*>(first)) typeName = "Linear Dim";
        else if (dynamic_cast<const draft::DraftRadialDimension*>(first)) typeName = "Radial Dim";
        else if (dynamic_cast<const draft::DraftAngularDimension*>(first)) typeName = "Angular Dim";
        else if (dynamic_cast<const draft::DraftLeader*>(first)) typeName = "Leader";
        else if (dynamic_cast<const draft::DraftBlockRef*>(first)) typeName = "Block Ref";
        else if (dynamic_cast<const draft::DraftText*>(first)) typeName = "Text";
        else if (dynamic_cast<const draft::DraftSpline*>(first)) typeName = "Spline";
        else if (dynamic_cast<const draft::DraftHatch*>(first)) typeName = "Hatch";
        else if (dynamic_cast<const draft::DraftEllipse*>(first)) typeName = "Ellipse";
        m_typeLabel->setText(typeName);
        dimEntity = dynamic_cast<const draft::DraftDimension*>(first);
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

    // Line type.
    int lt = first->lineType();
    if (lt >= 0 && lt <= 7) {
        m_lineTypeCombo->setCurrentIndex(lt);
    } else {
        m_lineTypeCombo->setCurrentIndex(0);
    }
    m_lineTypeCombo->setEnabled(true);

    // Dimension text override (single dimension selection only).
    if (dimEntity) {
        m_textOverrideEdit->setText(QString::fromStdString(dimEntity->textOverride()));
        m_dimPropsWidget->show();
    } else {
        m_dimPropsWidget->hide();
    }

    // Block ref properties (single block ref selection only).
    if (selectedIds.size() == 1) {
        auto* bref = dynamic_cast<const draft::DraftBlockRef*>(first);
        if (bref) {
            m_blockNameLabel->setText(QString::fromStdString(bref->blockName()));
            m_blockRotationSpin->setValue(bref->rotation() * math::kRadToDeg);
            m_blockScaleSpin->setValue(bref->uniformScale());
            m_blockPropsWidget->show();
        } else {
            m_blockPropsWidget->hide();
        }
    } else {
        m_blockPropsWidget->hide();
    }

    // Text entity properties (single text selection only).
    if (selectedIds.size() == 1) {
        auto* textEnt = dynamic_cast<const draft::DraftText*>(first);
        if (textEnt) {
            m_textContentEdit->setText(QString::fromStdString(textEnt->text()));
            m_textHeightSpin->setValue(textEnt->textHeight());
            m_textRotationSpin->setValue(textEnt->rotation() * math::kRadToDeg);
            m_textAlignCombo->setCurrentIndex(static_cast<int>(textEnt->alignment()));
            m_textPropsWidget->show();
        } else {
            m_textPropsWidget->hide();
        }
    } else {
        m_textPropsWidget->hide();
    }

    // Spline entity properties (single spline selection only).
    if (selectedIds.size() == 1) {
        auto* splineEnt = dynamic_cast<const draft::DraftSpline*>(first);
        if (splineEnt) {
            m_splinePointCountLabel->setText(QString::number(splineEnt->controlPointCount()));
            m_splineClosedCombo->setCurrentIndex(splineEnt->closed() ? 1 : 0);
            m_splinePropsWidget->show();
        } else {
            m_splinePropsWidget->hide();
        }
    } else {
        m_splinePropsWidget->hide();
    }

    // Hatch entity properties (single hatch selection only).
    if (selectedIds.size() == 1) {
        auto* hatchEnt = dynamic_cast<const draft::DraftHatch*>(first);
        if (hatchEnt) {
            m_hatchPatternCombo->setCurrentIndex(static_cast<int>(hatchEnt->pattern()));
            m_hatchAngleSpin->setValue(hatchEnt->angle() * math::kRadToDeg);
            m_hatchSpacingSpin->setValue(hatchEnt->spacing());
            m_hatchPropsWidget->show();
        } else {
            m_hatchPropsWidget->hide();
        }
    } else {
        m_hatchPropsWidget->hide();
    }

    // Ellipse entity properties (single ellipse selection only).
    if (selectedIds.size() == 1) {
        auto* ellipseEnt = dynamic_cast<const draft::DraftEllipse*>(first);
        if (ellipseEnt) {
            m_ellipseSemiMajorSpin->setValue(ellipseEnt->semiMajor());
            m_ellipseSemiMinorSpin->setValue(ellipseEnt->semiMinor());
            m_ellipseRotationSpin->setValue(ellipseEnt->rotation() * math::kRadToDeg);
            m_ellipsePropsWidget->show();
        } else {
            m_ellipsePropsWidget->hide();
        }
    } else {
        m_ellipsePropsWidget->hide();
    }

    // Constraint info.
    updateConstraintList();

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

void PropertyPanel::onLineTypeChanged(int index) {
    if (m_updatingUI || m_currentIds.empty()) return;
    if (index < 0 || index > 7) return;

    auto* viewport = m_mainWindow->findChild<ViewportWidget*>();
    if (!viewport || !viewport->document()) return;

    auto cmd = std::make_unique<doc::ChangeEntityLineTypeCommand>(
        viewport->document()->draftDocument(), m_currentIds, index);
    viewport->document()->undoStack().push(std::move(cmd));
    viewport->update();
}

void PropertyPanel::onTextOverrideChanged() {
    if (m_updatingUI || m_currentIds.size() != 1) return;

    auto* viewport = m_mainWindow->findChild<ViewportWidget*>();
    if (!viewport || !viewport->document()) return;

    std::string newText = m_textOverrideEdit->text().toStdString();
    auto cmd = std::make_unique<doc::ChangeTextOverrideCommand>(
        viewport->document()->draftDocument(), m_currentIds.front(), newText);
    viewport->document()->undoStack().push(std::move(cmd));
    viewport->update();
}

void PropertyPanel::updateConstraintList() {
    m_constraintList->clear();
    m_deleteConstraintBtn->setEnabled(false);

    auto* viewport = m_mainWindow->findChild<ViewportWidget*>();
    if (!viewport || !viewport->document() || m_currentIds.empty()) {
        m_constraintWidget->hide();
        return;
    }

    auto& cstrSys = viewport->document()->constraintSystem();
    bool hasConstraints = false;

    for (uint64_t entityId : m_currentIds) {
        auto constrs = cstrSys.constraintsForEntity(entityId);
        for (const auto* c : constrs) {
            QString text = QString("[%1] %2")
                .arg(c->id())
                .arg(QString::fromStdString(c->typeName()));
            if (c->hasDimensionalValue()) {
                text += QString(" = %1").arg(c->dimensionalValue(), 0, 'f', 4);
            }
            // Avoid duplicates if constraint references multiple selected entities.
            bool found = false;
            for (int i = 0; i < m_constraintList->count(); ++i) {
                if (m_constraintList->item(i)->data(Qt::UserRole).toULongLong() == c->id()) {
                    found = true;
                    break;
                }
            }
            if (!found) {
                auto* item = new QListWidgetItem(text);
                item->setData(Qt::UserRole, QVariant::fromValue<quint64>(c->id()));
                m_constraintList->addItem(item);
                hasConstraints = true;
            }
        }
    }

    if (hasConstraints) {
        m_constraintWidget->show();
    } else {
        m_constraintWidget->hide();
    }
}

void PropertyPanel::onDeleteConstraint() {
    auto* viewport = m_mainWindow->findChild<ViewportWidget*>();
    if (!viewport || !viewport->document()) return;

    auto* item = m_constraintList->currentItem();
    if (!item) return;

    uint64_t constraintId = item->data(Qt::UserRole).toULongLong();
    auto cmd = std::make_unique<doc::RemoveConstraintCommand>(
        viewport->document()->constraintSystem(), constraintId);
    viewport->document()->undoStack().push(std::move(cmd));
    viewport->update();

    // Refresh the list.
    updateConstraintList();
}

void PropertyPanel::onBlockRotationChanged(double value) {
    if (m_updatingUI || m_currentIds.size() != 1) return;

    auto* viewport = m_mainWindow->findChild<ViewportWidget*>();
    if (!viewport || !viewport->document()) return;

    double radians = value * math::kDegToRad;
    auto cmd = std::make_unique<doc::ChangeBlockRefRotationCommand>(
        viewport->document()->draftDocument(), m_currentIds.front(), radians);
    viewport->document()->undoStack().push(std::move(cmd));
    viewport->update();
}

void PropertyPanel::onBlockScaleChanged(double value) {
    if (m_updatingUI || m_currentIds.size() != 1) return;
    if (std::abs(value) < 1e-6) return;  // Prevent zero scale.

    auto* viewport = m_mainWindow->findChild<ViewportWidget*>();
    if (!viewport || !viewport->document()) return;

    auto cmd = std::make_unique<doc::ChangeBlockRefScaleCommand>(
        viewport->document()->draftDocument(), m_currentIds.front(), value);
    viewport->document()->undoStack().push(std::move(cmd));
    viewport->update();
}

void PropertyPanel::onTextContentChanged() {
    if (m_updatingUI || m_currentIds.size() != 1) return;

    auto* viewport = m_mainWindow->findChild<ViewportWidget*>();
    if (!viewport || !viewport->document()) return;

    std::string newText = m_textContentEdit->text().toStdString();
    auto cmd = std::make_unique<doc::ChangeTextContentCommand>(
        viewport->document()->draftDocument(), m_currentIds.front(), newText);
    viewport->document()->undoStack().push(std::move(cmd));
    viewport->update();
}

void PropertyPanel::onTextHeightChanged(double value) {
    if (m_updatingUI || m_currentIds.size() != 1) return;

    auto* viewport = m_mainWindow->findChild<ViewportWidget*>();
    if (!viewport || !viewport->document()) return;

    auto cmd = std::make_unique<doc::ChangeTextHeightCommand>(
        viewport->document()->draftDocument(), m_currentIds.front(), value);
    viewport->document()->undoStack().push(std::move(cmd));
    viewport->update();
}

void PropertyPanel::onTextRotationChanged(double value) {
    if (m_updatingUI || m_currentIds.size() != 1) return;

    auto* viewport = m_mainWindow->findChild<ViewportWidget*>();
    if (!viewport || !viewport->document()) return;

    double radians = value * math::kDegToRad;
    auto cmd = std::make_unique<doc::ChangeTextRotationCommand>(
        viewport->document()->draftDocument(), m_currentIds.front(), radians);
    viewport->document()->undoStack().push(std::move(cmd));
    viewport->update();
}

void PropertyPanel::onTextAlignmentChanged(int index) {
    if (m_updatingUI || m_currentIds.size() != 1) return;
    if (index < 0 || index > 2) return;

    auto* viewport = m_mainWindow->findChild<ViewportWidget*>();
    if (!viewport || !viewport->document()) return;

    auto cmd = std::make_unique<doc::ChangeTextAlignmentCommand>(
        viewport->document()->draftDocument(), m_currentIds.front(), index);
    viewport->document()->undoStack().push(std::move(cmd));
    viewport->update();
}

void PropertyPanel::onSplineClosedChanged(int index) {
    if (m_updatingUI || m_currentIds.size() != 1) return;
    if (index < 0 || index > 1) return;

    auto* viewport = m_mainWindow->findChild<ViewportWidget*>();
    if (!viewport || !viewport->document()) return;

    bool closed = (index == 1);
    auto cmd = std::make_unique<doc::ChangeSplineClosedCommand>(
        viewport->document()->draftDocument(), m_currentIds.front(), closed);
    viewport->document()->undoStack().push(std::move(cmd));
    viewport->update();
}

void PropertyPanel::onHatchPatternChanged(int index) {
    if (m_updatingUI || m_currentIds.size() != 1) return;
    if (index < 0 || index > 2) return;

    auto* viewport = m_mainWindow->findChild<ViewportWidget*>();
    if (!viewport || !viewport->document()) return;

    auto cmd = std::make_unique<doc::ChangeHatchPatternCommand>(
        viewport->document()->draftDocument(), m_currentIds.front(), index);
    viewport->document()->undoStack().push(std::move(cmd));
    viewport->update();
}

void PropertyPanel::onHatchAngleChanged(double value) {
    if (m_updatingUI || m_currentIds.size() != 1) return;

    auto* viewport = m_mainWindow->findChild<ViewportWidget*>();
    if (!viewport || !viewport->document()) return;

    double radians = value * math::kDegToRad;
    auto cmd = std::make_unique<doc::ChangeHatchAngleCommand>(
        viewport->document()->draftDocument(), m_currentIds.front(), radians);
    viewport->document()->undoStack().push(std::move(cmd));
    viewport->update();
}

void PropertyPanel::onHatchSpacingChanged(double value) {
    if (m_updatingUI || m_currentIds.size() != 1) return;
    if (value < 0.01) return;

    auto* viewport = m_mainWindow->findChild<ViewportWidget*>();
    if (!viewport || !viewport->document()) return;

    auto cmd = std::make_unique<doc::ChangeHatchSpacingCommand>(
        viewport->document()->draftDocument(), m_currentIds.front(), value);
    viewport->document()->undoStack().push(std::move(cmd));
    viewport->update();
}

void PropertyPanel::onEllipseSemiMajorChanged(double value) {
    if (m_updatingUI || m_currentIds.size() != 1) return;
    if (value < 0.001) return;

    auto* viewport = m_mainWindow->findChild<ViewportWidget*>();
    if (!viewport || !viewport->document()) return;

    auto cmd = std::make_unique<doc::ChangeEllipseSemiMajorCommand>(
        viewport->document()->draftDocument(), m_currentIds.front(), value);
    viewport->document()->undoStack().push(std::move(cmd));
    viewport->update();
}

void PropertyPanel::onEllipseSemiMinorChanged(double value) {
    if (m_updatingUI || m_currentIds.size() != 1) return;
    if (value < 0.001) return;

    auto* viewport = m_mainWindow->findChild<ViewportWidget*>();
    if (!viewport || !viewport->document()) return;

    auto cmd = std::make_unique<doc::ChangeEllipseSemiMinorCommand>(
        viewport->document()->draftDocument(), m_currentIds.front(), value);
    viewport->document()->undoStack().push(std::move(cmd));
    viewport->update();
}

void PropertyPanel::onEllipseRotationChanged(double value) {
    if (m_updatingUI || m_currentIds.size() != 1) return;

    auto* viewport = m_mainWindow->findChild<ViewportWidget*>();
    if (!viewport || !viewport->document()) return;

    double radians = value * math::kDegToRad;
    auto cmd = std::make_unique<doc::ChangeEllipseRotationCommand>(
        viewport->document()->draftDocument(), m_currentIds.front(), radians);
    viewport->document()->undoStack().push(std::move(cmd));
    viewport->update();
}

}  // namespace hz::ui
