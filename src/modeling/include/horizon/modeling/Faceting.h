#pragma once

#include <memory>
#include <string>
#include <vector>

#include "horizon/math/Constants.h"

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
/// - Another curved face is one facet, its outline, and is listed.
FacetedSolid facetCurved(const topo::Solid& exact, double maxAngle = math::kTwoPi / 32.0);

}  // namespace hz::model
