#pragma once

#include <memory>

#include "horizon/geometry/surfaces/NurbsSurface.h"
#include "horizon/geometry/surfaces/Surface.h"

namespace hz::geo {

/// The typed surface @p surface exactly is, or null. A NURBS is a given kind when every
/// point of a dense sample of its domain (a 33 x 33 grid, its boundary included) lies on
/// the candidate within @p tolerance, its normals agree with the candidate's up to a sign
/// that is the same everywhere (so the domain maps onto the typed surface without
/// folding), and the angles it sweeps about a revolved kind's axis do not double back or
/// wrap more than once. The parametrisations need not be equal: the typed surface is
/// given the frame whose seam sits at the NURBS's:
///   - Plane: origin S(uMin, vMin), x along the u direction there, z along the normal.
///   - Cylinder: z along the axis towards the larger v, origin on the axis at the foot of
///     S(uMin, vMin), x towards S(uMin, vMid).
///   - Cone: origin at the apex (radius 0 at v = 0), z along the axis towards the
///     surface, x towards S(uMin, vMid).
///   - Sphere: origin at the centre, z towards the pole at the larger v when a v edge
///     collapses to one, x towards S(uMin, vMid).
///   - Torus: origin at the centre, z along the axis (counter-clockwise in u), x towards
///     S(uMin, vMid).
/// Kinds are tried simplest first (plane, sphere, cylinder, cone, torus). For surfaces
/// Horizon makes (NurbsSurface::makePlane ... makeCone); imported B-splines are not
/// passed through it (spec, goal 7). Never throws: degenerate input gives null.
std::unique_ptr<Surface> recognize(const NurbsSurface& surface, double tolerance);

}  // namespace hz::geo
