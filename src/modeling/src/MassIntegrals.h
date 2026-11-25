#pragma once

#include <array>
#include <vector>

#include "horizon/math/Vec3.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/topology/HalfEdge.h"

namespace hz::model::detail {

/// What mass properties are made from, over a closed triangulated boundary:
/// the ten volume integrals {1, x, y, z, x², y², z², xy, yz, zx} before their
/// constant factors (Eberly, "Polyhedral Mass Properties (Revisited)"), and
/// the surface area. Linear in the boundary, so two measures of one solid at
/// different refinements extrapolate component by component.
struct Measures {
    std::array<double, 10> sums{};
    double area = 0.0;
    /// The point the sums' coordinates are taken from: 0 unless a measure is made about a point
    /// near its solid, where the moments of a part far from the origin keep their precision.
    math::Vec3 origin;

    /// Add a boundary triangle's volume terms (its winding gives their sign).
    void addVolume(const math::Vec3& p0, const math::Vec3& p1, const math::Vec3& p2);

    /// Add a boundary triangle: its volume terms and its area.
    void add(const math::Vec3& p0, const math::Vec3& p1, const math::Vec3& p2);
};

/// The mass properties the measures give, at @p density; the sums' sign is
/// normalised so the volume is positive. The centre of mass is the measures' origin plus the
/// centre they give.
MassProperties finish(const Measures& measures, double density);

/// The triangles a face's loops make, as the polygon formulas sum them: its outer loop
/// triangulated (its holes bridged in), wound as the loop runs. None for a face with no loop.
std::vector<std::array<math::Vec3, 3>> faceTriangles(const topo::Face& face);

/// The mass properties of @p solid's face loops, triangulated as they are:
/// compute() without cutting a solid described by curves into facets.
MassProperties measureLoops(const topo::Solid& solid, const Material* material);

}  // namespace hz::model::detail
