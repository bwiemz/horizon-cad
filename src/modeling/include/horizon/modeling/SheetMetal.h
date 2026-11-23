#pragma once

#include <optional>
#include <string>
#include <vector>

#include "horizon/math/Vec2.h"

namespace hz::model {

/// Sheet-metal bend parameters (consistent length units).
///
/// The K-factor locates the neutral bending axis as a fraction of the material
/// thickness from the inside surface (0 = inside face, 0.5 = mid-thickness);
/// typical values are ~0.33–0.45 for common metals.
struct SheetMetalParams {
    double thickness = 1.0;   ///< material thickness t
    double bendRadius = 1.0;  ///< inside bend radius r
    double kFactor = 0.44;    ///< neutral-axis position, 0 <= K <= 1

    /// Usable when thickness > 0, radius >= 0, and 0 <= K <= 1.
    bool isValid() const {
        return thickness > 0.0 && bendRadius >= 0.0 && kFactor >= 0.0 && kFactor <= 1.0;
    }
};

/// A sheet-metal strip to unfold: `segments` flat lengths (measured between bend
/// tangent lines) joined by `bendAngles` bends. There is exactly one fewer bend
/// than segment: `bendAngles.size() + 1 == segments.size()`.
struct SheetMetalStrip {
    std::vector<double> segments;    ///< flat segment lengths
    std::vector<double> bendAngles;  ///< bend angles (radians) between consecutive segments
};

/// Bend allowance: the developed (flat) length of the neutral axis through a
/// bend of @p angleRad radians — BA = angle * (r + K*t). Zero for a non-positive
/// angle or invalid params.
double bendAllowance(double angleRad, const SheetMetalParams& p);

/// Bend deduction for the setback method: BD = 2*(r + t)*tan(angle/2) - BA.
/// The amount to subtract from the sum of outside dimensions to get the flat
/// length. Zero for a non-positive angle or invalid params.
double bendDeduction(double angleRad, const SheetMetalParams& p);

/// Developed (flat-pattern) length of @p strip: the sum of its flat segment
/// lengths plus the bend allowance of each bend, whichever way it turns.
/// Returns 0 if the params are invalid or the strip's segment/bend counts are
/// inconsistent.
double developedLength(const SheetMetalStrip& strip, const SheetMetalParams& p);

/// A base flange's strip from its sketch: @p points, end to end, are the
/// virtual sharps of the sheet's bottom surface (the one its thickness grows
/// from), the material to the left of travel, or to the right with
/// @p materialRight. Each corner is a bend of the turn's angle, positive
/// toward the material. A flat segment is its line less the setback of each
/// bend at its ends: R tan(angle/2), R being the bottom surface's radius
/// there (r + t on a bend toward the material, r away from it). nullopt,
/// with why in @p why when given, for fewer than two points, a line of no
/// length, a corner that turns back on itself, a line too short for its
/// bends, or parameters that are not usable.
std::optional<SheetMetalStrip> stripFromPolyline(const std::vector<math::Vec2>& points,
                                                 const SheetMetalParams& params, bool materialRight,
                                                 std::string* why);

/// Where each bend's centre line runs across the flat pattern: its distance
/// from the strip's start along the developed length (the middle of the
/// bend's allowance, whichever way it turns).
std::vector<double> bendLinePositions(const SheetMetalStrip& strip, const SheetMetalParams& p);

}  // namespace hz::model
