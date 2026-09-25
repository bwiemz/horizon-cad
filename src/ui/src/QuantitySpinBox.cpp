#include "horizon/ui/QuantitySpinBox.h"

#include <QLineEdit>
#include <algorithm>
#include <cmath>
#include <string_view>

#include "horizon/math/Constants.h"

namespace hz::ui {

namespace {

/// Enough places that a value typed in any unit (0.001 in is 0.0254 mm)
/// is kept, not rounded to the places shown.
constexpr int kKeptDecimals = 10;

}  // namespace

QuantitySpinBox::QuantitySpinBox(Kind kind, math::LengthUnit unit, int decimals, QWidget* parent)
    : QDoubleSpinBox(parent), m_kind(kind), m_unit(unit), m_shown(std::max(decimals, 0)) {
    setDecimals(kKeptDecimals);
    setSingleStep(kind == Kind::Length ? math::millimetresPer(unit) : 1.0);
    setKeyboardTracking(false);
    setCorrectionMode(QAbstractSpinBox::CorrectToPreviousValue);
}

void QuantitySpinBox::setUnit(math::LengthUnit unit) {
    if (unit == m_unit) return;
    m_unit = unit;
    if (m_kind == Kind::Length) setSingleStep(math::millimetresPer(unit));
    lineEdit()->setText(textFromValue(value()));
}

QString QuantitySpinBox::textFromValue(double value) const {
    if (m_kind == Kind::Angle) {
        if (std::abs(value) < 0.5 * std::pow(10.0, -m_shown)) value = 0.0;
        return QString::number(value, 'f', m_shown) + QChar(0x00B0);
    }
    return QString::fromStdString(math::formatLength(value, m_unit, m_shown));
}

std::optional<double> QuantitySpinBox::read(const QString& text) const {
    const std::string typed = text.trimmed().toStdString();
    if (m_kind == Kind::Angle) {
        const auto radians = math::parseAngle(typed);
        if (!radians) return std::nullopt;
        return *radians * math::kRadToDeg;
    }
    return math::parseLength(typed, m_unit);
}

double QuantitySpinBox::valueFromText(const QString& text) const {
    return read(text).value_or(value());
}

QValidator::State QuantitySpinBox::validate(QString& input, int& /*pos*/) const {
    const auto value = read(input);
    // Anything may be on its way to a value ("2 i" before "2 in"): only a
    // value in range is taken; the rest waits, and an edit left there goes
    // back to the value before it.
    if (!value) return QValidator::Intermediate;
    if (*value < minimum() || *value > maximum()) return QValidator::Intermediate;
    return QValidator::Acceptable;
}

}  // namespace hz::ui
