#pragma once

// Internal to the geometry library: an angle brought into one turn, the same way for curves and
// surfaces.

#include "horizon/math/Constants.h"

namespace hz::geo::detail {

/// An angle within one turn of [0, 2 pi) normalised into it; a tiny negative angle that rounds
/// up to 2 pi gives 0.
inline double wrapAngle(double a) {
    if (a < 0.0) {
        a += math::kTwoPi;
    }
    if (a >= math::kTwoPi) {
        a -= math::kTwoPi;  // an angle past a full turn, e.g. longitude + pi
    }
    return a >= math::kTwoPi ? 0.0 : a;
}

}  // namespace hz::geo::detail
