#pragma once

#include <string>
#include <vector>

#include "horizon/math/Vec3.h"

namespace hz::topo {

struct Face;
class Solid;

/// Geometric validation of a half-edge B-Rep.
///
/// `Solid::isValid()` (Euler + manifold) is *combinatorial only*: it inspects
/// twin/next/prev linkage and entity counts and never looks at a coordinate.
/// A solid can therefore pass every structural check while carrying loops
/// whose geometry is nonsense — self-intersecting boundaries, vertices that
/// wander off the face's own surface, twin half-edges whose endpoints sit at
/// different points in space.  Those defects are invisible to the structural
/// validators but corrupt every downstream consumer (mass properties,
/// tessellation, Booleans, export).
///
/// This validator closes that gap.  It is deliberately *separate* from
/// `Solid::isValid()` so existing structural call sites keep their meaning;
/// operations that construct geometry should gate on both.
///
/// Checks performed (see `Issues` for the per-check counters):
///
///  - **vertex chain** — `he->next->origin == he->twin->origin`.  A closed
///    half-edge loop can be structurally sound while the vertex a half-edge
///    ends at (via its twin) is not the vertex the next half-edge starts at.
///  - **twin coincidence** — the two half-edges of an edge must span the same
///    two *positions*, so an edge is a single segment in space.
///  - **degenerate edges** — zero-length edges.
///  - **degenerate faces** — loops with fewer than three vertices or
///    vanishing area.
///  - **loop planarity** — every vertex of a planar face lies on that face's
///    plane.  A face is treated as planar when it carries no surface, or when
///    its bound surface's normal field is constant.  Curved carriers
///    (cylinder, sphere, torus, ruled blends) are skipped rather than guessed
///    at.
///  - **loop self-intersection** — non-adjacent boundary segments of a planar
///    loop must not cross.
///  - **shell closure** — the face area vectors of a closed shell must sum to
///    zero.  A non-zero sum means the skin is either open or inconsistently
///    oriented.
///  - **edge curve agreement** *(reported, not failing)* — an edge's bound
///    NURBS curve should start and end at the edge's vertices.
///  - **coincident vertices** *(reported, not failing)* — two distinct vertex
///    records at the same position, which is legal (cone apex, sewn seams)
///    but is usually a missed weld.
class GeometryValidator {
public:
    struct Issues {
        int vertexChainErrors = 0;
        int twinCoincidenceErrors = 0;
        int degenerateEdges = 0;
        int degenerateFaces = 0;
        int nonPlanarLoops = 0;
        int selfIntersectingLoops = 0;
        int openShells = 0;
        int edgeCurveMismatches = 0;  ///< Reported only; does not fail ok().
        int coincidentVertices = 0;   ///< Reported only; does not fail ok().

        /// True when no failing check fired.  The two report-only counters
        /// are excluded: both have legitimate occurrences in valid models.
        bool ok() const {
            return vertexChainErrors == 0 && twinCoincidenceErrors == 0 && degenerateEdges == 0 &&
                   degenerateFaces == 0 && nonPlanarLoops == 0 && selfIntersectingLoops == 0 &&
                   openShells == 0;
        }
    };

    /// Default absolute tolerance for coincidence and planarity, in model units.
    static constexpr double kDefaultTol = 1e-7;

    /// Run every check.  @p tol is an absolute distance tolerance; area and
    /// self-intersection tests derive their thresholds from it.
    static Issues check(const Solid& solid, double tol = kDefaultTol);

    /// Convenience: `check(solid, tol).ok()`.
    static bool isGeometricallyValid(const Solid& solid, double tol = kDefaultTol);

    /// Human-readable listing of what `check()` found, in the style of
    /// `Solid::validationReport()`.
    static std::string report(const Solid& solid, double tol = kDefaultTol);

    // -- Building blocks (exposed for targeted use by modeling ops) ----------

    /// Newell area vector of a face's outer loop.  Length is the loop area;
    /// direction is the outward normal for an outward-wound loop.
    static math::Vec3 loopAreaVector(const Face& face);

    /// True when the face's carrier is planar (absent, or constant-normal).
    /// @p normalOut receives the unit plane normal when the result is true.
    static bool facePlane(const Face& face, math::Vec3& originOut, math::Vec3& normalOut,
                          double tol = kDefaultTol);
};

}  // namespace hz::topo
