#include "horizon/drafting/DimensionStyle.h"

#include <iomanip>
#include <locale>
#include <sstream>

#include "horizon/math/Units.h"

namespace hz::draft {

double millimetresPerUnit(const std::string& unit) {
    const auto known = math::lengthUnitFrom(unit);
    return known && isDimensionUnit(unit) ? math::millimetresPer(*known) : 1.0;
}

bool isDimensionUnit(const std::string& unit) {
    // By symbol only, as the file stores it: "inch" is not a style's unit.
    for (const math::LengthUnit known : math::kLengthUnits) {
        if (unit == math::symbolOf(known)) return true;
    }
    return false;
}

std::string DimensionStyle::formatLength(double mm) const {
    std::ostringstream out;
    out.imbue(std::locale::classic());  // a point, whatever the user's locale
    out << std::fixed << std::setprecision(precision) << mm / millimetresPerUnit(unit);
    if (showUnits) {
        // Inches and feet by their marks, as drawings write them.
        if (unit == "in") {
            out << '"';
        } else if (unit == "ft") {
            out << '\'';
        } else {
            out << ' ' << (isDimensionUnit(unit) ? unit : std::string("mm"));
        }
    }
    return out.str();
}

}  // namespace hz::draft
