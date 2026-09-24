#pragma once

// Internal topology helper shared by Loft and Sweep.
//
// A "ring stack" is S+1 parallel rings of N vertices each, sewn into a closed
// solid with two cap faces and S levels of N lateral faces. It generalizes the
// prism construction used by Extrude to an arbitrary number of intermediate
// sections, which is exactly what lofting (interpolate between profiles) and
// sweeping (transport a profile along a path) both need.
//
//   V = (S+1)N,  E = (2S+1)N,  F = S*N + 2   →  Euler V-E+F = 2 for all S.

#include <memory>
#include <vector>

#include "horizon/drafting/DraftEntity.h"
#include "horizon/geometry/surfaces/NurbsSurface.h"
#include "horizon/math/Vec2.h"
#include "horizon/math/Vec3.h"
#include "horizon/modeling/SolidSewer.h"
#include "horizon/topology/Solid.h"

namespace hz::model::ringstack {

/// Result of building a ring-stack solid: the vertex grid plus the faces,
/// so callers can bind geometry (surfaces) and TopologyIDs.
struct RingStackBuild {
    std::vector<std::vector<topo::Vertex*>> rings;       ///< rings[level][i]
    topo::Face* bottomFace = nullptr;                    ///< cap on ring 0
    topo::Face* topFace = nullptr;                       ///< cap on ring S
    std::vector<std::vector<topo::Face*>> lateralFaces;  ///< [level 0..S-1][i 0..N-1]
};

/// Build a closed solid from @p rings (each ring is a list of N>=3 points in
/// consistent winding order). Requires rings.size() >= 2 and every ring to
/// have the same vertex count. Returns an empty build (null faces) on bad
/// input.
RingStackBuild build(topo::Solid& solid, const std::vector<std::vector<math::Vec3>>& rings);

/// Normalize a closed polygon soup to outward winding, SolidSewer's input
/// contract, by the sign of the volume it encloses.
void orientOutward(std::vector<SolidSewer::InputFace>& faces);

/// Assign a degree-1 line curve to every edge of the solid.
void assignEdgeCurves(topo::Solid& solid);

/// A bilinear (degree 1x1) NURBS patch through four corners.
/// U runs p00->p10, V runs p00->p01.
std::shared_ptr<geo::NurbsSurface> makeBilinearPatch(const math::Vec3& p00, const math::Vec3& p10,
                                                     const math::Vec3& p01, const math::Vec3& p11);

/// A planar cap surface covering @p ring, oriented by @p normal.
std::shared_ptr<geo::NurbsSurface> makeCapSurface(const std::vector<math::Vec3>& ring,
                                                  const math::Vec3& normal);

/// How finely the arcs and circles of a profile are faceted.
struct ProfileResolution {
    int segmentsPerTurn = 32;  ///< chords per full turn of any arc
    /// When positive, each arc's count is instead derived from its own radius
    /// so that no chord sags more than this.
    double chordTolerance = 0.0;

    /// Chords across an arc of @p radius sweeping @p sweep radians (>= 1).
    int stepsFor(double radius, double sweep) const;
};

/// A circle some of a sampled profile's chords approximate.
struct ProfileArc {
    math::Vec2 center;
    double radius = 0.0;
};

/// A closed profile as a polygon, with each chord's provenance.
struct SampledProfile {
    std::vector<math::Vec2> vertices;  ///< chain order; closing vertex not repeated
    /// edgeArc[i] indexes `arcs` for the chord vertices[i] -> vertices[i+1]
    /// when it approximates an arc or circle, and is -1 for a straight edge.
    std::vector<int> edgeArc;
    std::vector<ProfileArc> arcs;
    /// edgeSource[i] indexes the ordered edge the chord vertices[i] ->
    /// vertices[i+1] was cut from (-1 for the implicit closing chord of an
    /// open chain), and edgeFacet[i] says which of that edge's chords it is,
    /// counted along the edge's own direction — so the numbering does not
    /// depend on which way round the loop happens to be chained.
    std::vector<int> edgeSource;
    std::vector<int> edgeFacet;
    /// Chords cut from each ordered edge.
    std::vector<int> sourceFacets;
};

/// Facet an ordered closed profile of lines, arcs, or a single circle.  Arcs
/// are followed along their curve, not across their chord.  Returns an empty
/// profile on unsupported geometry.
SampledProfile sampleProfile(const std::vector<std::shared_ptr<draft::DraftEntity>>& orderedEdges,
                             double tolerance, const ProfileResolution& resolution = {});

/// The vertices of sampleProfile() at the default resolution.
std::vector<math::Vec2> extractProfileVertices(
    const std::vector<std::shared_ptr<draft::DraftEntity>>& orderedEdges, double tolerance);

}  // namespace hz::model::ringstack
