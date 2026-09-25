#include "horizon/ui/TypedLength.h"

#include <Qt>
#include <cctype>
#include <charconv>
#include <cmath>
#include <locale>
#include <sstream>

#include "horizon/ui/TypedUnits.h"

namespace hz::ui {

std::optional<double> TypedLength::parse(std::string_view text, math::LengthUnit unit) {
    const auto value = math::parseLength(text, unit);
    if (!value || !(*value > 0.0)) return std::nullopt;
    return value;
}

bool TypedLength::key(int key, math::LengthUnit unit) {
    if (key >= Qt::Key_0 && key <= Qt::Key_9) {
        m_refused.clear();
        m_text += static_cast<char>('0' + (key - Qt::Key_0));
        return true;
    }
    if (key == Qt::Key_Period) {
        m_refused.clear();
        m_text += '.';
        return true;
    }
    if (key == Qt::Key_Backspace && !m_text.empty()) {
        takeBack(m_text);
        return true;
    }
    if (typeUnitKey(key, m_text)) {
        m_refused.clear();
        return true;
    }
    if (key == Qt::Key_Return || key == Qt::Key_Enter) {
        if (m_text.empty()) return true;
        if (const auto value = parse(m_text, unit)) {
            m_value = *value;
        } else {
            m_refused = m_text;
        }
        m_text.clear();
        return true;
    }
    return false;
}

void TypedLength::clear() {
    m_text.clear();
    m_refused.clear();
}

std::string TypedLength::prompt(const std::string& name, math::LengthUnit unit) const {
    const std::string symbol(math::symbolOf(unit));
    if (!m_text.empty()) {
        std::string label = name;
        if (!label.empty()) label[0] = static_cast<char>(std::toupper(label[0]));
        return "  " + label + " (" + symbol + "): " + m_text;
    }
    if (!m_refused.empty()) return "  '" + m_refused + "' is not a " + name;
    std::ostringstream value;
    value.imbue(std::locale::classic());
    value << m_value / math::millimetresPer(unit);
    return "  [" + name + "=" + value.str() + " " + symbol + "]";
}

}  // namespace hz::ui
