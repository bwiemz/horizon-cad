#pragma once

#include <optional>

#include "horizon/geometry/curves/NurbsCurve.h"
#include "horizon/geometry/surfaces/NurbsSurface.h"

namespace hz::geo {

/// Freeform surface construction (Phase 75, roadmap §7.11 surfacing
/// workbench). First slice: bilinearly blended Coons boundary patches.
class SurfaceBuilder {
public:
    /// Build a Coons patch bounded by four curves:
    ///
    ///        c2 (u: 0→1 at v=1)
    ///      ┌───────────────┐
    ///  c1  │               │  c3   (v: 0→1)
    ///      └───────────────┘
    ///        c0 (u: 0→1 at v=0)
    ///
    /// Requirements (first slice): c0/c2 share degree and knot vector, as do
    /// c1/c3; all four are non-rational (unit weights); and the endpoints
    /// meet at the four corners within @p cornerTol. Returns std::nullopt
    /// when the inputs are incompatible.
    ///
    /// The resulting surface interpolates all four boundary curves exactly
    /// (its boundary control rows/columns are the input control points; the
    /// interior comes from the discrete Coons formula).
    static std::optional<NurbsSurface> coonsPatch(const NurbsCurve& c0, const NurbsCurve& c1,
                                                  const NurbsCurve& c2, const NurbsCurve& c3,
                                                  double cornerTol = 1e-9);
};

}  // namespace hz::geo
