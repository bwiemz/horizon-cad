#pragma once

#include <atomic>
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
    double volume = 0.0;       ///< enclosed volume (always reported positive)
    double surfaceArea = 0.0;  ///< boundary area (triangles, or exact faces integrated)
    math::Vec3 centerOfMass;   ///< volume-weighted centroid
    math::Mat3 inertia;        ///< tensor about the center of mass (× density)
    double mass = 0.0;         ///< density × volume
    double density = 1.0;      ///< density used (1 when no material)
    bool valid = false;        ///< false if the solid encloses nothing that could be measured
    /// Why the properties are not valid, where that is known: the face that could not be
    /// measured and why ("<face>: <why>"). Empty when they are valid.
    std::string problem;
};

/// Mass properties of what a solid is meant to be (Phase 141): each face that
/// approximates a curved surface measured on that surface (its
/// topo::Face::analyticSurface), and each edge on the curve it approximates
/// (topo::Edge::analyticCurve, or its own curve). The faces are refined, their
/// points put on those surfaces, at rising resolution until the result stops
/// changing, and the sequence extrapolated to its limit. compute() measures
/// the facets, which is what Booleans, export and the display use.
///
/// It is a measurement, not a closed form: what it is measured on, and how
/// near its measure came, are said apart. onIdealSurfaces says the first,
/// withinTolerance and estimatedError the second.
struct IdealMassProperties {
    MassProperties properties;  ///< of the ideal geometry
    /// Every face that approximates a curved surface records it, and the
    /// ideals of neighbouring faces meet: it is measured on the design's
    /// surfaces. Where not, the result is between the facets and the
    /// design: a face without its ideal is measured as modelled, and an
    /// edge where two ideals part is closed with a sliver.
    bool onIdealSurfaces = false;
    /// The faces measured as modelled though they are facets of a curved
    /// surface (they bend by under 30° from a neighbour that has no ideal
    /// either), by logical name (model::logicalFace), each once.
    std::vector<std::string> withoutIdeal;
    /// The edges where the ideals of the faces on either side do not meet.
    int partedEdges = 0;
    /// The last correction the extrapolation made to the volume or the
    /// area, relative: an estimate of the error, found within a factor of 3
    /// of it for a torus, a sphere and a cone. 0 when nothing curved was
    /// refined (every face flat); meaningless when measured once alone
    /// (refinement 1 with curved faces): withinTolerance is false then.
    double estimatedError = 0.0;
    /// The relative tolerance the estimated error was to come under.
    double tolerance = 0.0;
    /// Whether it did: the refinement settled (four measures or more) with
    /// estimatedError at or under tolerance. False when it stopped first,
    /// at the budget of triangles, the finest refinement, or edges where
    /// ideals part; and for a solid measured by its corners alone. True for
    /// a solid of flat faces, measured as it is.
    bool withinTolerance = false;
    /// The finest refinement measured: each edge in this many pieces. For a solid with exact
    /// faces, integrated not refined, the most pieces one stretch of a face's boundary was
    /// integrated in, and never under 2: each piece is measured by two rules (7-point Gauss,
    /// 15-point Kronrod), whose difference is the estimate.
    int refinement = 1;
};

class MassPropertiesCalculator {
public:
    /// Compute mass properties from the solid's boundary. When @p material is
    /// null a unit density is used. A solid with exact faces (as read from a
    /// STEP file) is integrated on them, exactly: each face over its own
    /// region of its own surface, to 1e-11 of the face, its other faces by
    /// their polygons. A face that cannot be integrated leaves the result not
    /// valid, with its problem; it is never meshed or faceted instead. Any
    /// other solid is measured on its triangulated loops; one described by
    /// curves without exact faces in facets (describedByCurves, facetCurved).
    /// Nothing throws: a failure is a result that is not valid.
    static MassProperties compute(const topo::Solid& solid, const Material* material = nullptr);

    /// The ideal mass properties (IdealMassProperties): refined until the
    /// estimated error of the volume and the area is under @p tolerance,
    /// relative, or the refinement reaches a budget of triangles; where
    /// ideals part, at eight pieces an edge. Which it was is said in the
    /// result (withinTolerance). The default gives the closed forms of the
    /// primitives to 1e-9. Far slower than compute(): for a
    /// request, not for every rebuild. Set @p cancelled to stop it, and its
    /// result is not valid. A solid described by curves is measured in
    /// facets that record its surfaces; one that cannot be cut into them is
    /// measured by its corners, and is not exact. A solid with exact faces is
    /// integrated on them, as compute() integrates it: the same numbers at the
    /// default tolerance, a thin wall's faces taken tighter where a finer one
    /// is asked for. estimatedError is the integration's estimate, with what
    /// edges standing off their surfaces (tolerances beyond the resolution)
    /// leave uncertain; it is on its ideal surfaces when every face is exact.
    ///
    /// @p triangleBudget: no refinement is measured that would make more
    /// triangles than this. Less trades precision for time: the result says
    /// when it stopped short (withinTolerance).
    static IdealMassProperties computeIdeal(const topo::Solid& solid,
                                            const Material* material = nullptr,
                                            double tolerance = 1e-10,
                                            const std::atomic<bool>* cancelled = nullptr,
                                            double triangleBudget = kIdealTriangleBudget);
    /// The budget of triangles computeIdeal refines to by default.
    static constexpr double kIdealTriangleBudget = 6e6;

    /// The volume each of @p solid's shells encloses, in the solid's shell order, signed by which
    /// way its faces face: positive for a shell whose faces face out of what it encloses,
    /// negative for one whose faces face into it (a void's, or a shell written inside out).
    /// compute() normalises that sign away; a reader that must tell such a shell needs it. Each
    /// exact face is integrated on its surface as compute() integrates it, but to @p accuracy of
    /// the face's size (a sign needs far less than compute()'s 1e-11); each other face by its
    /// polygons. Empty, with why in @p problem, where a face cannot be integrated, a volume is
    /// not finite, or @p cancelled is set. Nothing throws.
    static std::vector<double> shellVolumes(const topo::Solid& solid,
                                            std::string* problem = nullptr, double accuracy = 1e-6,
                                            const std::atomic<bool>* cancelled = nullptr);
};

}  // namespace hz::model
