#include "horizon/modeling/SheetMetal.h"

#include <cmath>

namespace hz::model {

double bendAllowance(double angleRad, const SheetMetalParams& p) {
    if (angleRad <= 0.0 || !p.isValid()) return 0.0;
    return angleRad * (p.bendRadius + p.kFactor * p.thickness);
}

double bendDeduction(double angleRad, const SheetMetalParams& p) {
    if (angleRad <= 0.0 || !p.isValid()) return 0.0;
    const double setback = (p.bendRadius + p.thickness) * std::tan(angleRad / 2.0);
    return 2.0 * setback - bendAllowance(angleRad, p);
}

double developedLength(const SheetMetalStrip& strip, const SheetMetalParams& p) {
    if (!p.isValid()) return 0.0;
    if (strip.segments.empty()) return 0.0;
    if (strip.bendAngles.size() + 1 != strip.segments.size()) return 0.0;

    double total = 0.0;
    for (double s : strip.segments) total += s;
    for (double angle : strip.bendAngles) total += bendAllowance(angle, p);
    return total;
}

}  // namespace hz::model
