#pragma once

namespace hz::cam {

/// A cutting tool's geometry relevant to feeds and speeds.
struct Tool {
    double diameter = 6.0;  ///< cutter diameter, mm
    int flutes = 2;         ///< number of cutting edges
};

/// Spindle speed (RPM) that achieves the cutting surface speed
/// @p surfaceSpeedMPerMin (m/min) for a tool of @p diameterMm.
/// N = 1000 * v / (pi * D). Returns 0 for a non-positive diameter.
double spindleRpm(double surfaceSpeedMPerMin, double diameterMm);

/// Table feed rate (mm/min) from spindle speed @p rpm, flute count @p flutes,
/// and the per-tooth chip load @p chipLoadMm (mm/tooth): f = N * flutes * chip.
double feedRate(double rpm, int flutes, double chipLoadMm);

/// Recommended cutting surface speeds (m/min) for HSS-class tooling — starting
/// points a caller can scale for coatings/carbide.
namespace surfaceSpeed {
inline constexpr double kAluminum = 300.0;  ///< m/min
inline constexpr double kBrass = 150.0;
inline constexpr double kMildSteel = 30.0;
inline constexpr double kStainless = 20.0;
}  // namespace surfaceSpeed

}  // namespace hz::cam
