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

bool QuantitySpinBox::fromExpression(double value) const {
    // As the field holds it: Qt rounds a value to its places, and not
    // always to the nearest double to them.
    return !m_expression.empty() &&
           std::abs(value - m_expressionValue) <= 1e-9 * std::max(1.0, std::abs(value));
}

std::string QuantitySpinBox::expression() const {
    return fromExpression(value()) ? m_expression : std::string();
}

void QuantitySpinBox::setExpression(const std::string& kept, double value) {
    m_expression = kept;
    m_expressionValue = value;
    setValue(value);
    lineEdit()->setText(textFromValue(this->value()));
    m_expressionValue = this->value();  // as the field rounds it
}

void QuantitySpinBox::stepBy(int steps) {
    m_expression.clear();  // a number, stepped to
    QDoubleSpinBox::stepBy(steps);
}

QString QuantitySpinBox::textFromValue(double value) const {
    if (fromExpression(value)) {
        // Its outermost brackets, as kept, left off: "=(wall * 2)" shown
        // "=wall * 2".
        std::string shown = m_expression;
        while (shown.size() >= 2 && shown.front() == '(' && shown.back() == ')') {
            int depth = 0;
            bool whole = true;
            for (std::size_t k = 0; k + 1 < shown.size(); ++k) {
                depth += shown[k] == '(' ? 1 : shown[k] == ')' ? -1 : 0;
                if (depth == 0) whole = false;
            }
            if (!whole) break;
            shown = shown.substr(1, shown.size() - 2);
        }
        return QStringLiteral("=") + QString::fromStdString(shown);
    }
    if (m_kind == Kind::Angle) {
        if (std::abs(value) < 0.5 * std::pow(10.0, -m_shown)) value = 0.0;
        return QString::number(value, 'f', m_shown) + QChar(0x00B0);
    }
    return QString::fromStdString(math::formatLength(value, m_unit, m_shown));
}

std::optional<double> QuantitySpinBox::read(const QString& text) const {
    const std::string typed = text.trimmed().toStdString();
    if (!typed.empty() && typed.front() == '=') {
        if (!m_resolver) return std::nullopt;
        std::string why;
        const auto worked = m_resolver(typed.substr(1), &why);
        if (!worked) return std::nullopt;
        return worked->value;
    }
    if (m_kind == Kind::Angle) {
        const auto radians = math::parseAngle(typed);
        if (!radians) return std::nullopt;
        return *radians * math::kRadToDeg;
    }
    return math::parseLength(typed, m_unit);
}

double QuantitySpinBox::valueFromText(const QString& text) const {
    // The value as shown is the value: Enter, or leaving the field, without
    // an edit must not round it to the places shown (10 mm is "0.394 in").
    if (text == textFromValue(value())) return value();
    const std::string typed = text.trimmed().toStdString();
    if (!typed.empty() && typed.front() == '=' && m_resolver) {
        std::string why;
        if (const auto worked = m_resolver(typed.substr(1), &why)) {
            m_expression = worked->kept;
            m_expressionValue = worked->value;
            return worked->value;
        }
        return value();
    }
    m_expression.clear();  // a number
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
