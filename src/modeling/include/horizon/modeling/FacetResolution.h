#pragma once

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

    /// Chords across an arc of @p radius sweeping @p sweep radians (>= 1): the per-turn count
    /// (segmentsPerTurn, or the radius's own under chordTolerance; at least 3) in proportion to
    /// the sweep, rounded up.
    int stepsFor(double radius, double sweep) const;
};

}  // namespace hz::model
