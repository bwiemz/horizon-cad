#pragma once

#include <memory>

#include "horizon/topology/Solid.h"

namespace hz::model {

/// Factory for creating primitive solid bodies with correct B-Rep topology
/// and NURBS surface/curve geometry bindings.
///
/// Every primitive satisfies Euler's formula (V - E + F = 2) and has
/// TopologyIDs assigned to all faces and edges for stable naming.
/// Builds primitive solids as half-edge B-Reps.
///
/// **Curved primitives are faceted.**  The kernel's Boolean, classification
/// and mass-property paths all evaluate a solid from its face loops, so a
/// curved surface bound to a coarse loop is decoration, not geometry: it makes
/// display disagree with every computation.  Cylinders, cones, spheres and
/// tori are therefore tessellated into planar facets at construction, and the
/// facets carry planar patches that match them.  The B-Rep is then exactly
/// what the rest of the kernel treats it as, and volumes converge to the
/// analytic value as the facet count rises.
///
/// Facet counts default to `kDefaultSegments`; pass an explicit count, or use
/// `segmentsForTolerance()` to derive one from a chord-sag budget.
class PrimitiveFactory {
public:
    /// Default facet count around a full revolution for curved primitives.
    /// At radius 5 this holds the chord sag under 0.025 model units and puts
    /// a cylinder's volume within 0.1% of pi*r^2*h.
    static constexpr int kDefaultSegments = 32;

    /// Facets around a full revolution that hold the chord sag of a circle of
    /// @p radius within @p tolerance.  A facet chord subtends 2*pi/n, so its
    /// sag is r*(1 - cos(pi/n)); this inverts that.  Clamped to [3, 4096].
    static int segmentsForTolerance(double radius, double tolerance);

    /// Axis-aligned box from (0,0,0) to (width, height, depth).
    /// 8V, 12E, 6F.  Each face is a bilinear planar NURBS surface.
    static std::unique_ptr<topo::Solid> makeBox(double width, double height, double depth);

    /// Cylinder centered at origin, axis along +Z, given radius and height.
    ///
    /// A @p segments -sided prism: two n-gon caps and n lateral quads
    /// (2n V, 3n E, n+2 F).  Volume is the inscribed prism's, converging to
    /// pi*r^2*h from below.  Returns nullptr for a non-positive radius or
    /// height.
    static std::unique_ptr<topo::Solid> makeCylinder(double radius, double height,
                                                     int segments = kDefaultSegments);

    /// Sphere centered at origin with given radius.
    ///
    /// A UV-sphere: @p segments facets around the axis and @p stacks bands
    /// from pole to pole, with triangle fans at the two poles.  Volume
    /// converges to (4/3)*pi*r^3 from below.  Returns nullptr for a
    /// non-positive radius.  @p stacks defaults to half @p segments, the
    /// aspect that keeps facets roughly square.
    static std::unique_ptr<topo::Solid> makeSphere(double radius, int segments = kDefaultSegments,
                                                   int stacks = 0);

    /// Cone (frustum) centered at origin, axis along +Z.
    /// @param bottomRadius  Radius at z=0.
    /// @param topRadius     Radius at z=height (0 for a sharp cone).
    /// @param height        Height along Z.
    /// @param segments      Facets around the axis.
    ///
    /// A frustum (both radii non-zero) is a prism with two n-gon caps.  When
    /// one radius is zero that ring collapses to a point, so the solid is
    /// built with apex topology instead — one n-gon cap plus n triangles —
    /// rather than a ring of zero-length edges around a zero-area cap.
    ///
    /// Returns nullptr when both radii, or the height, are zero.
    static std::unique_ptr<topo::Solid> makeCone(double bottomRadius, double topRadius,
                                                 double height, int segments = kDefaultSegments);

    /// Torus centered at origin, axis along +Z.
    /// @param majorRadius  Distance from center to tube center.
    /// @param minorRadius  Tube radius.
    /// @param segments     Facets around the major circle.
    /// @param tubeSegments Facets around the tube; defaults to half @p segments.
    ///
    /// A genus-1 quad grid (n*m V, 2n*m E, n*m F).  Note that
    /// `Solid::checkEulerFormula()` has no genus term, so it reports false for
    /// a torus even though `checkManifold()` and the geometric validator both
    /// pass — V - E + F is 0 here, not 2.  Returns nullptr unless
    /// 0 < minorRadius < majorRadius.
    static std::unique_ptr<topo::Solid> makeTorus(double majorRadius, double minorRadius,
                                                  int segments = kDefaultSegments,
                                                  int tubeSegments = 0);
};

}  // namespace hz::model
