#pragma once

#include "horizon/math/Vec2.h"

namespace hz::draft {

/// What a snap landed on. Object snaps (every type but Grid and None) come
/// from entities; the overlay draws a different marker for each.
enum class SnapType { None, Grid, Endpoint, Midpoint, Center, Quadrant, Intersection };

/// A point an entity offers to snap to, and what kind of point it is.
struct SnapPoint {
    math::Vec2 point;
    SnapType type = SnapType::Endpoint;
};

}  // namespace hz::draft
