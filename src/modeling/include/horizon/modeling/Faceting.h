#pragma once

#include <memory>
#include <string>
#include <vector>

#include "horizon/math/BoundingBox.h"
#include "horizon/math/Constants.h"
#include "horizon/modeling/FacetResolution.h"

namespace hz::topo {
class Solid;
}  // namespace hz::topo

namespace hz::model {

/// A solid with faces on curved surfaces, made the kernel's way (Phase 141).
struct FacetedSolid {
    std::unique_ptr<topo::Solid> solid;  ///< null when it could not be made
    /// The curved faces not bounded by a rectangle of their surface's (u, v):
    /// each is one facet, its outline, and only its ideal is the surface.
    std::vector<std::string> outlined;
    std::string error;  ///< why there is no solid
};

/// @p exact, whose faces lie on curved surfaces and whose edges run along
/// curves (as a STEP file describes a part: a cylinder is a face bounded by
/// two circles and a seam), made as the kernel makes its own curved parts:
/// planar facets that record the surface they approximate
/// (topo::Face::analyticSurface), bounded by chords that record their curve
/// (topo::Edge::analyticCurve). Booleans, mass properties and the display
/// read loops of points, which a face bounded by one circle does not have.
///
/// - Each edge is cut into chords of equal length, as many as keep each
///   within @p maxAngle of turning.
/// - A flat face keeps its holes, now polygons.
/// - A curved face bounded by a rectangle of its surface's (u, v), a seam
///   and a pole or an apex allowed, is a grid of facets, `<face>/facet:<k>`:
///   the whole of a cylinder, a cone, a sphere or a torus, and a fillet.
/// - A curved face with any other outline, or with holes, is cut into
///   facets within its trim (Phase 152): its region in (u, v) triangulated,
///   its outline points the edges' own, points added inside as close as
///   the surface turns by @p maxAngle.
/// - One that cannot be (a pole on a trimmed outline, a hole across a seam)
///   is one facet, its outline, and is listed.
///
/// A face with no NURBS carrier is faceted as flat, its outline: where it is
/// not (its exact surface is curved, or with no surface its outline leaves
/// any plane) there is no solid, and the error names it: `face <tag> has no
/// surface to facet`, or for an exact surface `face <tag> has no NURBS
/// carrier to facet on` (facetExact facets it).
FacetedSolid facetCurved(const topo::Solid& exact, double maxAngle = math::kTwoPi / 32.0);

/// The bridge (stage 3): @p exact, whose faces carry exact geometry
/// (topo::Face::exactSurface, pcurves, topo::Edge::exactCurve) and no NURBS
/// carrier, as planar facets at a feature's own @p resolution, so that a
/// part built exactly has the faces, edges and vertices master's faceted
/// construction gave it. Every face is cut from its exact fields; a NURBS
/// carrier (topo::Face::surface) is not read, and an import's faceted copy
/// stays facetCurved's.
///
/// - Each edge is cut into chords along its exact curve: a circle or an arc
///   into `stepsFor(radius, sweep)` at equal angles, a line into one, an
///   ellipse or a NURBS curve at equal steps of its parameter into as many
///   as keep each within 2π/segmentsPerTurn of turning and, with a chord
///   tolerance, sagging no more than it. A curve in a loop of two corners or
///   fewer is two chords or more, and a closed one three or more. An edge
///   with no exact curve is one chord, straight.
/// - A plane is one face, with its holes; so is a face with no exact surface
///   whose carrier is flat, or with neither whose outline lies in a plane.
///   Its loops run as the face's do: round its outward normal.
/// - A curved face is a grid uniform in its own (u, v), over the rectangle
///   its outer loop bounds there (a side of it may be a pole or an apex). Its
///   columns are the most chords of the edges along u, its rows of those
///   along v, and the edges of a side alone on it are cut to match (so a
///   periodic face's columns are the largest count of its edges); a
///   direction with no edges along it (a sphere's u) is cut as its widest
///   iso-curve is. Its facets are `<face>/facet:<k>`, row by row from its
///   first corner; a face of one facet keeps its tag. Each records the face's
///   surface as NURBS over that rectangle (topo::Face::analyticSurface), one
///   shared by all of them.
/// - Each chord of an edge is `<edge>/chord:<k>`, counted along the edge from
///   its half-edge's origin; an edge of one chord keeps its tag; an edge with
///   no name names no chord. A chord of a curve records it as NURBS over the
///   edge's range (topo::Edge::analyticCurve), one shared by all its chords;
///   a line is its own chord, and records nothing. The edges inside a face
///   are named as SolidSewer names them: a caller that names edges its own
///   way (model::nameEdgesLogically) does so after, as for facetCurved.
///
/// No solid, and the error says why, where a face has no boundary, a curved
/// face has no exact surface (`face <tag> has no surface to facet`, as for
/// facetCurved, or `face <tag> has no exact surface to facet` where it has a
/// carrier), a curved face is not such a rectangle or has holes, its
/// opposite sides cannot be cut alike, or the facets do not close.
FacetedSolid facetExact(const topo::Solid& exact, const FacetResolution& resolution);

/// Whether @p solid is described by curves, as a STEP file describes one: an
/// edge runs along a curve (of degree above 1) between its vertices. Its
/// faces are then no loops of points to measure, cut or draw until
/// facetCurved makes them so. The kernel's own solids never are: their edges
/// are chords, and the curve a chord stands in for is its analyticCurve.
bool describedByCurves(const topo::Solid& solid);

/// What one shell of a solid encloses: its signed volume (negative for a
/// shell facing in, as a cavity's does) and the box round it.
struct ShellMeasure {
    double volume = 0.0;
    math::BoundingBox box;
};

/// Each shell of @p solid measured, in shell order (an empty shell measures
/// nothing). A solid described by curves is measured on its facets: its
/// faces' corners do not span them (a sphere's are one point), and a cavity
/// of curved faces measures nothing by them.
std::vector<ShellMeasure> measureShells(const topo::Solid& solid);

}  // namespace hz::model
