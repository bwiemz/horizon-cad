#pragma once

#include <QDoubleSpinBox>
#include <optional>

#include "horizon/math/Units.h"

namespace hz::ui {

/// A number field for a length or an angle (Phase 154).
///
/// Its value() is what the model keeps and what callers read: millimetres
/// for a length, degrees for an angle. It is shown in the document's unit
/// with its symbol ("1.000 in", "30.000°"), and takes a value typed in any
/// unit ("2 in", "50.8 mm", "1' 6\"", "0.5 rad"); a bare number is in the
/// document's unit (degrees, for an angle). Arrows step by one unit.
class QuantitySpinBox : public QDoubleSpinBox {
    Q_OBJECT

public:
    enum class Kind { Length, Angle };

    /// Shown to @p decimals places; a length in @p unit.
    QuantitySpinBox(Kind kind, math::LengthUnit unit, int decimals, QWidget* parent = nullptr);

    Kind kind() const { return m_kind; }
    math::LengthUnit unit() const { return m_unit; }
    /// Show a length in @p unit from now on (the document shown changed);
    /// its value stays.
    void setUnit(math::LengthUnit unit);

protected:
    QString textFromValue(double value) const override;
    double valueFromText(const QString& text) const override;
    QValidator::State validate(QString& input, int& pos) const override;

private:
    /// @p text read, in millimetres or degrees; nullopt when it is not a
    /// value (or is one only in part, while it is being typed).
    std::optional<double> read(const QString& text) const;

    Kind m_kind;
    math::LengthUnit m_unit;
    int m_shown;
};

}  // namespace hz::ui
