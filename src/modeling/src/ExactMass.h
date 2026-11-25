#pragma once

// Mass properties of a solid with exact faces (stage 2 of the exact kernel), integrated on each
// face's own surface over its own region in (u, v): no mesh is made for them. Private to the
// modeling library; MassPropertiesCalculator::compute and computeIdeal measure such a solid
// through it.

#include <atomic>
#include <string>
#include <vector>

#include "horizon/modeling/MassProperties.h"
#include "horizon/topology/Solid.h"

namespace hz::model::detail {

/// What measureExact gives.
struct ExactMeasure {
    /// Not valid, with its problem said, when a face could not be integrated, the faces enclose
    /// no volume or one that is not finite, or the request was stopped.
    MassProperties properties;
    /// The error the integration estimates, relative: the larger of the volume's and the area's.
    double estimatedError = 0.0;
    /// The live faces with no exact surface, by logical name, each once: summed by the polygon
    /// formulas, as modelled.
    std::vector<std::string> plainFaces;
    /// The most pieces one stretch of a face's boundary was integrated in.
    int pieces = 0;
};

/// The mass properties of @p solid, which has exact faces (topo::hasExactFaces).
///
/// By the divergence theorem each of the ten volume integrals {1, x, y, z, x², y², z², xy, yz,
/// zx} is a sum over the faces of an integral over the face's trimmed region R, in its unwrapped
/// parameters (topo::unwrapLoop), of g(S) · (S_u × S_v)_k, signed by the face's `reversed`; the
/// area is the same integral of |S_u × S_v|. By Green's theorem each reduces to a line integral
/// along the face's boundary of G(u, v) = ∫ g from a reference parameter to u, taken by
/// Gauss–Legendre between the surface's knots (in u, or in v where that is cheaper: a cylinder's
/// or a cone's axis); the line integral is adaptive Gauss–Kronrod (15 points), to 1e-11 of the
/// face. Where the estimate summed over the faces is still above @p tolerance relative to the
/// volume or the area (a thin wall), every face is taken again as much tighter as that needs.
///
/// The boundary is each half-edge's pcurve where that is a closed form (a line, a circle, any
/// B-spline on a plane); a fitted pcurve is replaced by its edge's exact curve, integrated in its
/// own parameter, each point put on the surface by Newton from a march along the edge started
/// from the pcurve, so a face bounded by closed forms is exact even where its pcurves are not.
/// The jump from one stretch's end to the next one's start is a straight stretch of the
/// boundary: rounding on most surfaces, a run along the pole or the apex where a pcurve meets one.
/// An edge whose tolerance is beyond the resolution (it stands off its surface) bounds a region
/// known only to that tolerance: the estimate adds that width times the edge's length times the
/// integrands' size along it. Faces with no exact surface are summed by the polygon formulas.
/// Each face is signed by its own sense, so a void subtracts and a mirror image measures as its
/// original; a solid whose every face faces in measures as the solid they bound, its sign
/// normalised as the polygon formulas' is (detail::finish).
///
/// A face that cannot be integrated (no loop, a half-edge with no pcurve, a loop that winds round
/// its surface or runs against the face's sense, a value that is not finite, an integral that
/// does not settle within its budget of evaluations, stretches or subdivisions) leaves the result
/// not valid, its problem "<face>: <why>"; so do sums that are not finite. Nothing throws.
/// @p cancelled, when set, stops it between faces and within one, and the result is not valid.
ExactMeasure measureExact(const topo::Solid& solid, const Material* material,
                          const std::atomic<bool>* cancelled, double tolerance = 1e-10);

/// What measureShellVolumes gives: each shell's signed volume, in the solid's shell order, or why
/// there is none.
struct ShellVolumes {
    std::vector<double> volumes;
    std::string problem;
};

/// The volume each of @p solid's shells encloses, signed by which way its faces face, as
/// measureExact sums it but shell by shell and not normalised: each exact face integrated to
/// @p accuracy of its own size, each other face by its polygons. A shell whose faces face out
/// of what it encloses is positive; one facing into it (a void's) is negative. Empty volumes,
/// with the problem said, where a face cannot be integrated, a sum is not finite, or
/// @p cancelled is set. Nothing throws.
ShellVolumes measureShellVolumes(const topo::Solid& solid, double accuracy,
                                 const std::atomic<bool>* cancelled);

}  // namespace hz::model::detail
