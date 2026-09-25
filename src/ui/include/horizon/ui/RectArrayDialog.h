#pragma once

#include <QDialog>

#include "horizon/math/Units.h"

class QSpinBox;
class QDoubleSpinBox;

namespace hz::ui {

class RectArrayDialog : public QDialog {
    Q_OBJECT

public:
    /// Its spacings shown and typed in @p unit, the document's (Phase 154).
    explicit RectArrayDialog(QWidget* parent = nullptr,
                             math::LengthUnit unit = math::LengthUnit::Millimetre);

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
