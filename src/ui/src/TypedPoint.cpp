#include "horizon/ui/TypedPoint.h"

#include <Qt>
#include <charconv>
#include <cmath>

#include "horizon/math/Constants.h"
#include "horizon/ui/TypedUnits.h"

namespace hz::ui {

namespace {

std::optional<math::Vec2> fail(std::string* why, const char* reason) {
    if (why) *why = reason;
    return std::nullopt;
}

}  // namespace

bool TypedPoint::key(int key) {
    char c = 0;
    if (key >= Qt::Key_0 && key <= Qt::Key_9) {
        c = static_cast<char>('0' + (key - Qt::Key_0));
    } else if (key == Qt::Key_Period) {
        c = '.';
    } else if (key == Qt::Key_Minus) {
        c = '-';
    } else if (key == Qt::Key_Comma) {
        c = ',';
    } else if (key == Qt::Key_At) {
        c = '@';
    } else if (key == Qt::Key_Less) {
        c = '<';
    } else if (key == Qt::Key_Backspace && !m_text.empty()) {
        takeBack(m_text);
        return true;
    } else if (typeUnitKey(key, m_text)) {
        m_refused.clear();
        m_reason.clear();
        return true;
    } else {
        return false;
    }
    m_refused.clear();
    m_reason.clear();
    m_text += c;
    return true;
}

void TypedPoint::clear() {
    m_text.clear();
    m_refused.clear();
    m_reason.clear();
}

std::optional<math::Vec2> TypedPoint::take(const std::optional<math::Vec2>& base,
                                           const std::optional<math::Vec2>& toward,
                                           math::LengthUnit unit) {
    std::string why;
    const auto point = resolve(m_text, base, toward, &why, unit);
    if (point) {
        clear();
    } else {
        m_refused = m_text;
        m_reason = why;
        m_text.clear();
    }
    return point;
}

std::string TypedPoint::prompt(math::LengthUnit unit) const {
    if (!m_text.empty()) return "  Point (" + std::string(math::symbolOf(unit)) + "): " + m_text;
    if (!m_refused.empty()) return "  '" + m_refused + "' is not a point: " + m_reason;
    return "";
}

std::optional<math::Vec2> TypedPoint::resolve(std::string_view text,
                                              const std::optional<math::Vec2>& base,
                                              const std::optional<math::Vec2>& toward,
                                              std::string* why, math::LengthUnit unit) {
    const auto length = [unit](std::string_view part) { return math::parseLength(part, unit); };
    const bool relative = !text.empty() && text.front() == '@';
    if (relative) text.remove_prefix(1);
    if (relative && !base) return fail(why, "there is no last point to be relative to");
    const math::Vec2 origin = relative ? *base : math::Vec2(0, 0);

    if (const size_t at = text.find('<'); at != std::string_view::npos) {
        const auto along = length(text.substr(0, at));
        const auto radians = math::parseAngle(text.substr(at + 1));
        if (!along || !radians) {
            return fail(why, "polar input is length<angle, the angle in degrees or rad");
        }
        return origin + math::Vec2(std::cos(*radians), std::sin(*radians)) * *along;
    }
    if (const size_t comma = text.find(','); comma != std::string_view::npos) {
        const auto x = length(text.substr(0, comma));
        const auto y = length(text.substr(comma + 1));
        if (!x || !y) return fail(why, "a point is x,y");
        return origin + math::Vec2(*x, *y);
    }
    if (relative) return fail(why, "a relative point is @dx,dy or @length<angle");

    // A length alone: that far from the last point, toward the cursor.
    const auto far = length(text);
    if (!far) return fail(why, "type x,y, @dx,dy, @length<angle or a length");
    if (!base) return fail(why, "a length alone needs a last point to measure from");
    if (!toward) return fail(why, "a length alone goes toward the cursor");
    const math::Vec2 direction = *toward - *base;
    const double reach = direction.length();
    if (!(reach > 1e-12)) return fail(why, "move the cursor away from the last point first");
    return *base + direction * (*far / reach);
}

}  // namespace hz::ui
