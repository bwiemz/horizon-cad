#include "horizon/drafting/DimensionStyle.h"

#include <iomanip>
#include <locale>
#include <sstream>

namespace hz::draft {

double millimetresPerUnit(const std::string& unit) {
    if (unit == "cm") return 10.0;
    if (unit == "m") return 1000.0;
    if (unit == "in") return 25.4;
    if (unit == "ft") return 304.8;
    return 1.0;
}

bool isDimensionUnit(const std::string& unit) {
    return unit == "mm" || unit == "cm" || unit == "m" || unit == "in" || unit == "ft";
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
