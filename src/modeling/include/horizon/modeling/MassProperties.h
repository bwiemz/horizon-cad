#pragma once

#include <string>
#include <vector>

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

/// Mass properties of what a solid is meant to be (Phase 141): each face that
/// approximates a curved surface measured on that surface (its
/// topo::Face::analyticSurface), and each edge on the curve it approximates
/// (topo::Edge::analyticCurve, or its own curve). The faces are refined, their
/// points put on those surfaces, at rising resolution until the result stops
/// changing, and the sequence extrapolated to its limit. compute() measures
/// the facets, which is what Booleans, export and the display use.
struct IdealMassProperties {
    MassProperties properties;  ///< of the ideal geometry
    /// Every face that approximates a curved surface records it, and the
    /// ideals of neighbouring faces meet. Where not, the result is between
    /// the facets and the design: a face without its ideal is measured as
    /// modelled, and an edge where two ideals part is closed with a sliver.
    bool exact = false;
    /// The faces measured as modelled though they are facets of a curved
    /// surface (they bend by under 30° from a neighbour that has no ideal
    /// either), by logical name (model::logicalFace), each once.
    std::vector<std::string> withoutIdeal;
    /// The edges where the ideals of the faces on either side do not meet.
    int partedEdges = 0;
    /// The last correction the extrapolation made to the volume or the
    /// area, relative: an estimate of the error, found within a factor of 3
    /// of it for a torus, a sphere and a cone.
    double estimatedError = 0.0;
    /// The finest refinement measured: each edge in this many pieces.
    int refinement = 1;
};

class MassPropertiesCalculator {
public:
    /// Compute mass properties from the solid's triangulated boundary. When
    /// @p material is null a unit density is used.
    static MassProperties compute(const topo::Solid& solid, const Material* material = nullptr);

    /// The ideal mass properties (IdealMassProperties): refined until the
    /// estimated error of the volume and the area is under @p tolerance,
    /// relative, or the refinement reaches a budget of triangles; where
    /// ideals part, at eight pieces an edge. The default gives the closed
    /// forms of the primitives to 1e-9. Far slower than compute(): for a
    /// request, not for every rebuild.
    static IdealMassProperties computeIdeal(const topo::Solid& solid,
                                            const Material* material = nullptr,
                                            double tolerance = 1e-10);
};

}  // namespace hz::model
