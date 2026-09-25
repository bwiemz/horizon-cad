#pragma once

#include <QDialog>

#include "horizon/math/Units.h"

class QSpinBox;
class QDoubleSpinBox;
class QCheckBox;

namespace hz::ui {

class PolarArrayDialog : public QDialog {
    Q_OBJECT

public:
    /// Its centre shown and typed in @p unit, the document's (Phase 154).
    explicit PolarArrayDialog(QWidget* parent = nullptr,
                              math::LengthUnit unit = math::LengthUnit::Millimetre);

    int count() const;
    double totalAngle() const;
    double centerX() const;
    double centerY() const;

private slots:
    void onFillFullCircleChanged(Qt::CheckState state);

private:
    QSpinBox* m_count;
    QDoubleSpinBox* m_totalAngle;
    QDoubleSpinBox* m_centerX;
    QDoubleSpinBox* m_centerY;
    QCheckBox* m_fillFullCircle;
};

}  // namespace hz::ui
