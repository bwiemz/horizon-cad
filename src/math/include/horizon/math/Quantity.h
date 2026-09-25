#pragma once

#include <map>
#include <memory>
#include <optional>
#include <string>

#include "horizon/math/Expression.h"
#include "horizon/math/Units.h"

namespace hz::math {

/// A value and what it measures (Phase 155): millimetres to the power
/// `length`, times radians to the power `angle`. A pure number has both 0;
/// a length has length 1; an area length 2; an angle angle 1.
struct Quantity {
    double value = 0.0;
    int length = 0;
    int angle = 0;

    bool pure() const { return length == 0 && angle == 0; }
    bool sameMeasure(const Quantity& other) const {
        return length == other.length && angle == other.angle;
    }
};

/// What an expression is asked to give (a feature's parameter): a length,
/// an angle, or a plain number (a count, a factor).
enum class QuantityKind { Length, Angle, Number };

/// @p expression evaluated with what each value measures (Phase 155):
/// lengths added only to lengths, a unit only on a plain number, a power of
/// a length only a whole one. @p variables by name. Nullopt, and why in
/// @p why, when it measures nothing (a length plus a number), names a
/// variable not given or a function not known, or is not finite.
std::optional<Quantity> evaluateQuantity(const Expression& expression,
                                         const std::map<std::string, Quantity>& variables,
                                         std::string* why = nullptr);

/// @p expression as it is kept (Phase 155): every plain number added to, or
/// taken from, a length written in @p unit ("wall + 1" in inches is kept as
/// "wall + 1 in"), and one added to an angle in degrees; and, for a @p kind
/// of Length or Angle, a result that is a plain number written in @p unit
/// or degrees ("2 * 3" kept as "(2 * 3) in"). What is kept never depends on
/// the document's unit, so a change of unit changes nothing modelled.
/// Nullptr, and why in @p why, when it cannot measure @p kind (an area for
/// a length) or is not a quantity at all.
std::unique_ptr<Expression> normalized(const Expression& expression,
                                       const std::map<std::string, Quantity>& variables,
                                       QuantityKind kind, LengthUnit unit,
                                       std::string* why = nullptr);

/// What @p quantity measures, in words: "a number", "a length", "an angle",
/// "an area" (length 2), "length^3".
std::string measureName(const Quantity& quantity);

}  // namespace hz::math
