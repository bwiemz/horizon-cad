#pragma once

#include <QDialog>

class QSpinBox;
class QDoubleSpinBox;
class QCheckBox;

namespace hz::ui {

class PolarArrayDialog : public QDialog {
    Q_OBJECT

public:
    explicit PolarArrayDialog(QWidget* parent = nullptr);

    int count() const;
    double totalAngle() const;
    double centerX() const;
    double centerY() const;

private slots:
    void onFillFullCircleChanged(int state);

private:
    QSpinBox* m_count;
    QDoubleSpinBox* m_totalAngle;
    QDoubleSpinBox* m_centerX;
    QDoubleSpinBox* m_centerY;
    QCheckBox* m_fillFullCircle;
};

}  // namespace hz::ui
