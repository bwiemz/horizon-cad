#include "horizon/drafting/DraftDimension.h"
#include "horizon/math/Constants.h"

#include <cmath>
#include <iomanip>
#include <sstream>

namespace hz::draft {

DraftDimension::DraftDimension() = default;

std::string DraftDimension::displayText(const DimensionStyle& style) const {
    if (hasTextOverride()) return m_textOverride;

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(style.precision) << computedValue();
    return oss.str();
}

std::vector<std::pair<math::Vec2, math::Vec2>>
DraftDimension::makeArrowhead(const math::Vec2& tip, const math::Vec2& direction,
                               double size, double halfAngle) {
    // direction should point away from the measured area (into the arrow).
    double dirAngle = std::atan2(direction.y, direction.x);

    double a1 = dirAngle + math::kPi - halfAngle;
    double a2 = dirAngle + math::kPi + halfAngle;

    math::Vec2 wing1{tip.x + size * std::cos(a1), tip.y + size * std::sin(a1)};
    math::Vec2 wing2{tip.x + size * std::cos(a2), tip.y + size * std::sin(a2)};

    return {{tip, wing1}, {tip, wing2}};
}

}  // namespace hz::draft
