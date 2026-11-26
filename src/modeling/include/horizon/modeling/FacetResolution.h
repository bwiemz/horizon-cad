#pragma once

#include "horizon/math/Vec3.h"

namespace hz::model {

/// How finely a feature's curves are cut into chords: master's rule for the arcs and circles of a
/// profile (ringstack::ProfileResolution is this), and the rule the bridge from exact faces
/// (facetExact) cuts every edge and curved face by, so that a part built exactly has the facet,
/// edge and vertex counts master's faceted construction gave it.
struct FacetResolution {
    int segmentsPerTurn = 32;  ///< chords per full turn of any arc
    /// When positive, each arc's count is instead derived from its own radius so that no chord
    /// sags more than this (PrimitiveFactory::segmentsForTolerance).
    double chordTolerance = 0.0;
    /// When positive, the count master's faceted primitives give the second direction of a
    /// sphere or a torus, in place of stepsFor's: a sphere's face is cut from pole to pole into
    /// this many rows (in proportion over less than the whole half turn), a torus's round its
    /// tube into this many per whole turn; never fewer than master's least, 2 from pole to pole
    /// and 3 round a whole tube. 0 is none: those directions are cut by stepsFor, as every other
    /// is. (PrimitiveFactory::makeSphere's stacks and makeTorus's tubeSegments, each half the
    /// segments by default, rounded down.)
    int minorSegments = 0;
    /// When positive, with an axis (axisDirection not zero): a revolve's count per whole turn round
    /// its axis (stage 3, Ruling 24), master's one count for every point its profile turned
    /// through (a revolve's segments(), from its widest radius under a chord tolerance; a hole's
    /// 32). Every circle about that axis (centred on it, in a plane square to it) is cut into it in
    /// proportion to the circle's sweep, rounded up as master's steps were, and so is every curved
    /// face turned about the axis round it, whatever chordTolerance says; such a face's points
    /// inside are its first meridian's turned about the axis to master's angles, exactly on the
    /// surface, whatever its u (a NURBS surface's is no angle between its knots). Curves along the
    /// profile keep stepsFor, as master's sampled profile did. 0 is none.
    int aroundAxis = 0;
    math::Vec3 axisPoint{};      ///< a point on that axis
    math::Vec3 axisDirection{};  ///< its direction (any length); zero is no axis

    /// Chords across an arc of @p radius sweeping @p sweep radians (>= 1): the per-turn count
    /// (segmentsPerTurn, or the radius's own under chordTolerance; at least 3) in proportion to
    /// the sweep, rounded up.
    int stepsFor(double radius, double sweep) const;

    /// Steps round the axis for a turn of @p sweep radians (>= 1): aroundAxis (at least 3) in
    /// proportion, rounded up. Only where turnsAboutAnAxis().
    int stepsAround(double sweep) const;

    /// Whether aroundAxis applies: it is positive and the axis has a direction.
    bool turnsAboutAnAxis() const;
};

}  // namespace hz::model
