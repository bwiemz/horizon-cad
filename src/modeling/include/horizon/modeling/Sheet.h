#pragma once

#include <string>

namespace hz::model {

/// Standard drawing paper sizes (dimensions in millimetres, ISO 216 and ANSI).
enum class PaperSize {
    A0,
    A1,
    A2,
    A3,
    A4,
    AnsiA,  // Letter, 8.5 x 11 in
    AnsiB,  // Tabloid, 11 x 17 in
    AnsiC,  // 17 x 22 in
    AnsiD,  // 22 x 34 in
};

enum class Orientation { Landscape, Portrait };

/// A drawing sheet: a paper size and orientation. Dimensions are reported in
/// millimetres; the long edge is horizontal in landscape.
struct Sheet {
    PaperSize size = PaperSize::A3;
    Orientation orientation = Orientation::Landscape;
    double margin = 10.0;  ///< border inset from the paper edge, in mm

    /// Sheet width (mm), accounting for orientation.
    double widthMm() const;
    /// Sheet height (mm), accounting for orientation.
    double heightMm() const;
};

/// Human-readable name of a paper size (e.g. "A3", "ANSI A").
std::string paperSizeName(PaperSize size);

/// ISO 128 standard line weights (mm) for technical drawings.
namespace iso128 {
inline constexpr double kThin = 0.25;       ///< hidden/dimension lines
inline constexpr double kMedium = 0.35;     ///< general
inline constexpr double kThick = 0.5;       ///< visible outlines
inline constexpr double kExtraThick = 0.7;  ///< borders, section lines
}  // namespace iso128

}  // namespace hz::model
