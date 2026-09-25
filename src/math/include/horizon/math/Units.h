#pragma once

#include <array>
#include <optional>
#include <string>
#include <string_view>

namespace hz::math {

/// A unit lengths are shown and typed in (Phase 154). The model is in
/// millimetres whatever the unit: the unit changes only what is shown, and
/// how a number typed without one is read.
enum class LengthUnit { Millimetre, Centimetre, Metre, Inch, Foot };

/// Every length unit, in the order they are offered.
inline constexpr std::array<LengthUnit, 5> kLengthUnits{LengthUnit::Millimetre,
                                                        LengthUnit::Centimetre, LengthUnit::Metre,
                                                        LengthUnit::Inch, LengthUnit::Foot};

/// Millimetres in one @p unit.
double millimetresPer(LengthUnit unit);

/// @p unit's symbol: "mm", "cm", "m", "in" or "ft".
std::string_view symbolOf(LengthUnit unit);

/// The unit @p text names: a symbol, or a name ("inch", "feet",
/// "millimetre", "meters"), in any case, or a mark (' for feet, " for
/// inches). Nullopt for anything else.
std::optional<LengthUnit> lengthUnitFrom(std::string_view text);

/// @p mm in @p unit to @p decimals places, with a point whatever the
/// locale, and its symbol after it when @p withSymbol ("25.400 mm",
/// "1.000 in"). A value that rounds to zero is written without a sign.
std::string formatLength(double mm, LengthUnit unit, int decimals, bool withSymbol = true);

/// A length typed as text, in millimetres (Phase 154); nullopt when it is
/// not one.
///
/// One or more terms under one leading sign, each a number with its unit
/// after it: "25.4", "2 in", "2in", "1 ft 6 in", "1' 6\"", "3/4 in",
/// "1 1/2 in", "1.5e3 mm". A number with no unit is in @p unit; after a
/// term in feet, one is in inches ("1' 6"). The decimal mark is a point
/// only: a comma separates a typed point's coordinates.
std::optional<double> parseLength(std::string_view text, LengthUnit unit);

/// An angle typed as text, in radians (Phase 154); nullopt when it is not
/// one. A number, with a sign, and a unit after it: degrees when it has
/// none, or "°", "deg", "degree(s)"; "rad", "radian(s)".
std::optional<double> parseAngle(std::string_view text);

}  // namespace hz::math
