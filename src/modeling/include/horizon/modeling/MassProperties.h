#pragma once

#include <string>

#include "horizon/math/Mat3.h"
#include "horizon/math/Vec3.h"
#include "horizon/topology/Solid.h"

namespace hz::model {

/// A material with a mass density (kg/m³). Presets cover common engineering
/// materials; callers may also supply a custom one.
struct Material {
    std::string name;
    double density = 1000.0;  ///< kg/m³

    static Material steel() { return {"Steel", 7850.0}; }
    static Material aluminum() { return {"Aluminum", 2700.0}; }
    static Material titanium() { return {"Titanium", 4500.0}; }
    static Material absPlastic() { return {"ABS Plastic", 1050.0}; }
};

/// Volume, surface area, and inertial properties of a solid.
///
/// With no material the density is taken as 1, so @c mass equals @c volume and
/// @c inertia is the unit-density (geometric) tensor — scale by a density to get
/// physical values. Lengths are in model units; interpret the derived units
/// accordingly (e.g. mm → volume in mm³).
struct MassProperties {
    double volume = 0.0;       ///< signed-tetrahedra volume (always reported positive)
    double surfaceArea = 0.0;  ///< sum of boundary-triangle areas
    math::Vec3 centerOfMass;   ///< volume-weighted centroid
    math::Mat3 inertia;        ///< tensor about the center of mass (× density)
    double mass = 0.0;         ///< density × volume
    double density = 1.0;      ///< density used (1 when no material)
    bool valid = false;        ///< false if the solid produced no triangles
};

class MassPropertiesCalculator {
public:
    /// Compute mass properties from the solid's triangulated boundary. When
    /// @p material is null a unit density is used.
    static MassProperties compute(const topo::Solid& solid, const Material* material = nullptr);
};

}  // namespace hz::model
