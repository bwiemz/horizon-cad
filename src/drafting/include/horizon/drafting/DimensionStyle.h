#pragma once

namespace hz::draft {

/// Global settings controlling the appearance of dimension entities.
struct DimensionStyle {
    double textHeight        = 2.5;   // world units
    double arrowSize         = 1.5;   // arrowhead line length (world units)
    double arrowAngle        = 0.3;   // half-angle in radians (~17 degrees)
    double extensionGap      = 0.5;   // gap between definition point and extension line start
    double extensionOvershoot = 1.0;  // extension line past the dimension line
    int    precision         = 2;     // decimal places for displayed value
    bool   showUnits         = false; // append unit suffix (e.g. "mm")
};

}  // namespace hz::draft
