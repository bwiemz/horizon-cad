#pragma once

#include "horizon/math/Vec3.h"
#include "horizon/modeling/DrawingView.h"

namespace hz::topo {
class Solid;
}  // namespace hz::topo

namespace hz::model {

/// Section views for 2D drawings (Phase 61).
///
/// The solid is cut by a plane; the intersected material becomes closed cut
/// profiles with 45° cross-hatching, and the retained half (the material on
/// the opposite side of the plane normal) projects as visible outline
/// geometry. Hidden lines are omitted, per drafting convention for sections.
///
/// The cut is computed against the solid's tessellation — robust for any
/// valid B-Rep, including multi-shell solids (each shell contributes its own
/// cut loops).
class SectionGenerator {
public:
    /// Build a section view of @p solid cut by the plane through
    /// @p planePoint with @p planeNormal. The view looks at the cut (view
    /// direction is the negated normal), so the retained material lies behind
    /// the cut plane. @p hatchSpacing is the distance between hatch lines in
    /// model units.
    static DrawingView sectionView(const topo::Solid& solid, const math::Vec3& planePoint,
                                   const math::Vec3& planeNormal, double hatchSpacing = 2.0);
};

}  // namespace hz::model
