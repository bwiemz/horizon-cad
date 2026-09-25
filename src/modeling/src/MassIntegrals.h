#pragma once

#include <array>

#include "horizon/math/Vec3.h"
#include "horizon/modeling/MassProperties.h"

namespace hz::model::detail {

/// What mass properties are made from, over a closed triangulated boundary:
/// the ten volume integrals {1, x, y, z, x², y², z², xy, yz, zx} before their
/// constant factors (Eberly, "Polyhedral Mass Properties (Revisited)"), and
/// the surface area. Linear in the boundary, so two measures of one solid at
/// different refinements extrapolate component by component.
struct Measures {
    std::array<double, 10> sums{};
    double area = 0.0;

    /// Add a boundary triangle's volume terms (its winding gives their sign).
    void addVolume(const math::Vec3& p0, const math::Vec3& p1, const math::Vec3& p2);

    /// Add a boundary triangle: its volume terms and its area.
    void add(const math::Vec3& p0, const math::Vec3& p1, const math::Vec3& p2);
};

/// The mass properties the measures give, at @p density; the sums' sign is
/// normalised so the volume is positive.
MassProperties finish(const Measures& measures, double density);

}  // namespace hz::model::detail
