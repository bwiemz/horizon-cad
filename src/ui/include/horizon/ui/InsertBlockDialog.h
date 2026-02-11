#pragma once

#include <QDialog>
#include <string>
#include <vector>

class QComboBox;
class QDoubleSpinBox;

namespace hz::ui {

class InsertBlockDialog : public QDialog {
    Q_OBJECT

public:
    explicit InsertBlockDialog(const std::vector<std::string>& blockNames,
                               QWidget* parent = nullptr);

    std::string selectedBlock() const;
    double rotation() const;
    double scale() const;

private:
    QComboBox* m_blockCombo;
    QDoubleSpinBox* m_rotation;
    QDoubleSpinBox* m_scale;
};

}  // namespace hz::ui
