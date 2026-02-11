#pragma once

#include <QDockWidget>
#include <cstdint>
#include <vector>

class QLabel;
class QComboBox;
class QLineEdit;
class QPushButton;
class QDoubleSpinBox;

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

private:
    void createWidgets();

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
};

}  // namespace hz::ui
