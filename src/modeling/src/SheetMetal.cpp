#include "horizon/modeling/SheetMetal.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

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
    // A bend takes as much sheet whichever way it turns.
    for (double angle : strip.bendAngles) total += bendAllowance(std::abs(angle), p);
    return total;
}

namespace {

bool refuse(std::string* why, std::string message) {
    if (why != nullptr) *why = std::move(message);
    return false;
}

std::string mm(double value) {
    std::ostringstream out;
    out << std::setprecision(3) << value;
    return out.str();
}

/// Shorter than this, a line is no line (model mm).
constexpr double kTiny = 1e-9;
/// Less than this, a turn is no bend (radians): a bend that small would be
/// chords far shorter than the kernel tells apart.
constexpr double kNoTurn = 1e-6;
/// Less than this, a flat between bends is none (model mm).
constexpr double kNoFlat = 1e-6;
/// A turn within this of a half turn folds the sheet back on itself.
constexpr double kHalfTurn = 3.14159265358979323846 - 1e-6;

}  // namespace

std::optional<SheetMetalStrip> stripFromPolyline(const std::vector<math::Vec2>& points,
                                                 const SheetMetalParams& params, bool materialRight,
                                                 std::string* why) {
    if (!(params.thickness > 0.0) || !std::isfinite(params.thickness)) {
        refuse(why, "the thickness must be more than 0");
        return std::nullopt;
    }
    if (!params.isValid() || !std::isfinite(params.bendRadius)) {
        refuse(why, "the bend radius must be 0 or more, and the K-factor between 0 and 1");
        return std::nullopt;
    }
    if (points.size() < 2) {
        refuse(why, "a base flange needs at least one line");
        return std::nullopt;
    }
    for (std::size_t i = 0; i < points.size(); ++i) {
        if (!std::isfinite(points[i].x) || !std::isfinite(points[i].y)) {
            refuse(why, "point " + std::to_string(i + 1) + " of the sketch is not a number");
            return std::nullopt;
        }
    }
    const std::size_t lines = points.size() - 1;
    std::vector<double> lengths(lines);
    std::vector<math::Vec2> along(lines);
    for (std::size_t i = 0; i < lines; ++i) {
        const math::Vec2 d = points[i + 1] - points[i];
        lengths[i] = std::hypot(d.x, d.y);
        if (!(lengths[i] > kTiny)) {
            refuse(why, "line " + std::to_string(i + 1) + " has no length");
            return std::nullopt;
        }
        along[i] = {d.x / lengths[i], d.y / lengths[i]};
    }
    SheetMetalStrip strip;
    std::vector<double> setback(lines > 1 ? lines - 1 : 0, 0.0);
    for (std::size_t j = 0; j + 1 < lines; ++j) {
        const math::Vec2& a = along[j];
        const math::Vec2& b = along[j + 1];
        double turn = std::atan2(a.x * b.y - a.y * b.x, a.x * b.x + a.y * b.y);
        if (std::abs(turn) < kNoTurn) turn = 0.0;
        if (std::abs(turn) >= kHalfTurn) {
            refuse(why, "at the end of line " + std::to_string(j + 1) +
                            " the sketch turns back on itself: a bend can be at most nearly "
                            "half a turn");
            return std::nullopt;
        }
        // Positive toward the material: to the left of travel, or the right.
        const double angle = materialRight ? -turn : turn;
        if (angle != 0.0 && !(params.bendRadius > kTiny)) {
            refuse(why, "a bend needs an inside radius above 0 (at the end of line " +
                            std::to_string(j + 1) + ")");
            return std::nullopt;
        }
        strip.bendAngles.push_back(angle);
        const double radius =
            angle > 0.0 ? params.bendRadius + params.thickness : params.bendRadius;
        setback[j] = radius * std::tan(std::abs(angle) / 2.0);
    }
    for (std::size_t i = 0; i < lines; ++i) {
        const double before = i > 0 ? setback[i - 1] : 0.0;
        const double after = i + 1 < lines ? setback[i] : 0.0;
        double flat = lengths[i] - before - after;
        if (flat < -kTiny) {
            refuse(why, "line " + std::to_string(i + 1) + " is too short for the bends at its " +
                            (before > 0.0 && after > 0.0 ? "ends" : "end") + ": it is " +
                            mm(lengths[i]) + " mm, and they take " + mm(before + after) +
                            " mm of it (a smaller bend radius takes less)");
            return std::nullopt;
        }
        // Some flat between bends, and at each end: a sheet that starts in a
        // bend, or goes from one bend straight into the next, is not folded.
        if (flat < kNoFlat) {
            refuse(why, "line " + std::to_string(i + 1) +
                            " is no longer than its bends take: leave it some flat, or use a "
                            "smaller bend radius");
            return std::nullopt;
        }
        strip.segments.push_back(flat);
    }
    return strip;
}

std::vector<double> bendLinePositions(const SheetMetalStrip& strip, const SheetMetalParams& p) {
    std::vector<double> lines;
    double at = 0.0;
    for (std::size_t i = 0; i < strip.segments.size(); ++i) {
        at += strip.segments[i];
        if (i >= strip.bendAngles.size()) break;
        const double allowance = bendAllowance(std::abs(strip.bendAngles[i]), p);
        lines.push_back(at + allowance / 2.0);
        at += allowance;
    }
    return lines;
}

}  // namespace hz::model
