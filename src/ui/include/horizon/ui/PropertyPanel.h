#pragma once

#include <QDockWidget>
#include <cstdint>
#include <vector>

class QLabel;
class QComboBox;
class QLineEdit;
class QPushButton;
class QDoubleSpinBox;
class QListWidget;

namespace hz::ui {

class MainWindow;

class PropertyPanel : public QDockWidget {
    Q_OBJECT

public:
    explicit PropertyPanel(MainWindow* mainWindow, QWidget* parent = nullptr);

    void updateForSelection(const std::vector<uint64_t>& selectedIds);
    void refreshLayerList();

private slots:
    void onLayerChanged(int index);
    void onColorClicked();
    void onByLayerColorClicked();
    void onLineWidthChanged(double value);
    void onTextOverrideChanged();
    void onDeleteConstraint();
    void onBlockRotationChanged(double value);
    void onBlockScaleChanged(double value);
    void onTextContentChanged();
    void onTextHeightChanged(double value);
    void onTextRotationChanged(double value);
    void onTextAlignmentChanged(int index);
    void onSplineClosedChanged(int index);
    void onHatchPatternChanged(int index);
    void onHatchAngleChanged(double value);
    void onHatchSpacingChanged(double value);
    void onEllipseSemiMajorChanged(double value);
    void onEllipseSemiMinorChanged(double value);
    void onEllipseRotationChanged(double value);

private:
    void createWidgets();
    void updateConstraintList();

    MainWindow* m_mainWindow;
    std::vector<uint64_t> m_currentIds;
    bool m_updatingUI = false;

    QLabel* m_typeLabel = nullptr;
    QComboBox* m_layerCombo = nullptr;
    QPushButton* m_colorButton = nullptr;
    QPushButton* m_byLayerButton = nullptr;
    QDoubleSpinBox* m_lineWidthSpin = nullptr;

    // Dimension-specific
    QWidget* m_dimPropsWidget = nullptr;
    QLineEdit* m_textOverrideEdit = nullptr;

    // Block-ref specific
    QWidget* m_blockPropsWidget = nullptr;
    QLabel* m_blockNameLabel = nullptr;
    QDoubleSpinBox* m_blockRotationSpin = nullptr;
    QDoubleSpinBox* m_blockScaleSpin = nullptr;

    // Text entity specific
    QWidget* m_textPropsWidget = nullptr;
    QLineEdit* m_textContentEdit = nullptr;
    QDoubleSpinBox* m_textHeightSpin = nullptr;
    QDoubleSpinBox* m_textRotationSpin = nullptr;
    QComboBox* m_textAlignCombo = nullptr;

    // Spline-specific
    QWidget* m_splinePropsWidget = nullptr;
    QLabel* m_splinePointCountLabel = nullptr;
    QComboBox* m_splineClosedCombo = nullptr;

    // Hatch-specific
    QWidget* m_hatchPropsWidget = nullptr;
    QComboBox* m_hatchPatternCombo = nullptr;
    QDoubleSpinBox* m_hatchAngleSpin = nullptr;
    QDoubleSpinBox* m_hatchSpacingSpin = nullptr;

    // Ellipse-specific
    QWidget* m_ellipsePropsWidget = nullptr;
    QDoubleSpinBox* m_ellipseSemiMajorSpin = nullptr;
    QDoubleSpinBox* m_ellipseSemiMinorSpin = nullptr;
    QDoubleSpinBox* m_ellipseRotationSpin = nullptr;

    // Constraint info
    QWidget* m_constraintWidget = nullptr;
    QListWidget* m_constraintList = nullptr;
    QPushButton* m_deleteConstraintBtn = nullptr;
};

}  // namespace hz::ui
