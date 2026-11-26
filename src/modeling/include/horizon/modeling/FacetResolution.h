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

    /// Chords across an arc of @p radius sweeping @p sweep radians (>= 1): the per-turn count
    /// (segmentsPerTurn, or the radius's own under chordTolerance; at least 3) in proportion to
    /// the sweep, rounded up.
    int stepsFor(double radius, double sweep) const;
};

}  // namespace hz::model
