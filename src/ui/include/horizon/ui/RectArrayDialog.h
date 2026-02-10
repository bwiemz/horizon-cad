#pragma once

#include <QDialog>

class QSpinBox;
class QDoubleSpinBox;

namespace hz::ui {

class RectArrayDialog : public QDialog {
    Q_OBJECT

public:
    explicit RectArrayDialog(QWidget* parent = nullptr);

    int columns() const;
    int rows() const;
    double spacingX() const;
    double spacingY() const;

private:
    QSpinBox* m_columns;
    QSpinBox* m_rows;
    QDoubleSpinBox* m_spacingX;
    QDoubleSpinBox* m_spacingY;
};

}  // namespace hz::ui
