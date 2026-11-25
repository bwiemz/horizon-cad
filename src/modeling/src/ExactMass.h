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
};

/// The mass properties of @p solid, which has exact faces (topo::hasExactFaces).
///
/// By the divergence theorem each of the ten volume integrals {1, x, y, z, x², y², z², xy, yz,
/// zx} is a sum over the faces of an integral over the face's trimmed region R, in its unwrapped
/// parameters (topo::unwrapLoop), of g(S) · (S_u × S_v)_k, signed by the face's `reversed`; the
/// area is the same integral of |S_u × S_v|. By Green's theorem each reduces to a line integral
/// along the face's pcurves of G(u, v) = ∫ g from a reference parameter to u, taken by
/// Gauss–Legendre between the surface's knots (in u, or in v where that is cheaper: a cylinder's
/// or a cone's axis); the line integral is adaptive Gauss–Kronrod (15 points) in each pcurve's
/// parameter, to 1e-11 of the face. The jump from one pcurve's end to the next one's start is a
/// straight stretch of the boundary: rounding on most surfaces, a run along the pole or the apex
/// where a pcurve meets one. Faces with no exact surface are summed by the polygon formulas.
/// Each face is signed by its own sense, so a void subtracts and a mirror image measures as its
/// original; a solid whose every face faces in measures as the solid they bound, its sign
/// normalised as the polygon formulas' is (detail::finish).
///
/// A face that cannot be integrated (no loop, a half-edge with no pcurve, a loop that winds round
/// its surface or runs against the face's sense, a value that is not finite, an integral that
/// does not settle within its budget of evaluations, stretches or subdivisions) leaves the result
/// not valid, its problem "<face>: <why>". Nothing throws. @p cancelled, when set, stops it
/// between faces and within one, and the result is not valid.
ExactMeasure measureExact(const topo::Solid& solid, const Material* material,
                          const std::atomic<bool>* cancelled);

}  // namespace hz::model::detail
