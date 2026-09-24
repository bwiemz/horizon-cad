#include "horizon/ui/TypedLength.h"

#include <Qt>
#include <cctype>
#include <charconv>
#include <cmath>
#include <sstream>

namespace hz::ui {

std::optional<double> TypedLength::parse(std::string_view text) {
    double value = 0.0;
    const char* end = text.data() + text.size();
    const auto [stop, error] = std::from_chars(text.data(), end, value);
    if (error != std::errc() || stop != end) return std::nullopt;
    if (!std::isfinite(value) || value <= 0.0) return std::nullopt;
    return value;
}

bool TypedLength::key(int key) {
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
        m_text.pop_back();
        return true;
    }
    if (key == Qt::Key_Return || key == Qt::Key_Enter) {
        if (m_text.empty()) return true;
        if (const auto value = parse(m_text)) {
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

std::string TypedLength::prompt(const std::string& name) const {
    if (!m_text.empty()) {
        std::string label = name;
        if (!label.empty()) label[0] = static_cast<char>(std::toupper(label[0]));
        return "  " + label + ": " + m_text;
    }
    if (!m_refused.empty()) return "  '" + m_refused + "' is not a " + name;
    std::ostringstream value;
    value << m_value;
    return "  [" + name + "=" + value.str() + "]";
}

}  // namespace hz::ui
