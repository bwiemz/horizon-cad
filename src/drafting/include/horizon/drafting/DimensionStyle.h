#pragma once

#include <string>

namespace hz::draft {

/// Global settings controlling the appearance of dimension entities.
struct DimensionStyle {
    double textHeight = 2.5;          // world units
    double arrowSize = 1.5;           // arrowhead line length (world units)
    double arrowAngle = 0.3;          // half-angle in radians (~17 degrees)
    double extensionGap = 0.5;        // gap between definition point and extension line start
    double extensionOvershoot = 1.0;  // extension line past the dimension line
    int precision = 2;                // decimal places for displayed value
    bool showUnits = false;           // append the unit to lengths ("25.40 mm")
    std::string unit = "mm";          // lengths are shown in: mm, cm, m, in or ft

    /// A length, in millimetres as the model has it, as this style shows it:
    /// in its unit, to its precision, with the unit after it when showUnits.
    std::string formatLength(double mm) const;

    bool operator==(const DimensionStyle&) const = default;
};

/// Millimetres in one @p unit (mm, cm, m, in, ft); 1 for anything else.
double millimetresPerUnit(const std::string& unit);

/// Whether @p unit is one a dimension style can show.
bool isDimensionUnit(const std::string& unit);

}  // namespace hz::draft
