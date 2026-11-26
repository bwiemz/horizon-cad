#pragma once

#include <string>
#include <vector>

#include "horizon/math/ModelTolerance.h"
#include "horizon/topology/Solid.h"

namespace hz::topo {

enum class ExactIssueKind {
    MissingGeometry,  ///< A face has no exact surface, an edge no curve, a half-edge no pcurve.
    VertexOffCurve,   ///< An edge's curve misses one of its vertices at its end parameter.
    PcurveOffCurve,   ///< A pcurve, mapped onto its surface, strays from the edge's curve.
    PcurveOffVertex,  ///< A pcurve's end, mapped onto its surface, misses its vertex.
    LoopNotClosed,    ///< The half-edge cycle breaks, or consecutive pcurves do not join.
    LoopWrongOrientation,  ///< A loop's signed area in (u, v) has the wrong sign for its face.
    LoopSelfIntersects,    ///< A loop crosses itself.
    LoopsCross,            ///< Two loops of one face cross.
    HoleOutsideOuterLoop,  ///< An inner loop lies outside the face's outer loop.
    HoleInsideHole,        ///< An inner loop lies inside another inner loop of the face.
    RangeTooLong  ///< An edge's curve range, or a loop's unwrapped extent, exceeds one period.
};

struct ExactIssue {
    ExactIssueKind kind = ExactIssueKind::MissingGeometry;
    const Face* face = nullptr;
    const Edge* edge = nullptr;
    std::string detail;
    /// The half-edge whose pcurve is at issue (PcurveOffCurve and PcurveOffVertex): a seam's two
    /// share their face and edge. Null for the other kinds.
    const HalfEdge* halfEdge = nullptr;
};

/// Every way @p solid's exact geometry disagrees with its topology; empty when it agrees.
/// The solid must be combinatorially valid (`Solid::isValid`): twins, next pointers and origins
/// are followed without null checks. Each pcurve is sampled at 17 parameters (its two ends and
/// 15 between), and how far it strays from its edge's curve is measured there and at the peaks
/// between them (geo::pcurveGap, the measure pcurveOn takes of a closed form). Closeness tests use
/// @p tolerance together with the edge's and vertices' own tolerances. Pass
/// topo::exactTolerance(solid) (horizon/topology/ExactTolerance.h): the validator then lays the
/// loops out in parameter space exactly as the mesher does. That is not a promise that the mesher
/// succeeds: it has a triangle cap and a refinement limit, and loops touching at one point pass
/// here but are reported by the mesher.
std::vector<ExactIssue> validateExact(const Solid& solid, const math::ModelTolerance& tolerance);

/// The distances validateExact measures on a half-edge's pcurve and compares with its edge's
/// tolerance.
struct PcurveGaps {
    double start = 0.0;  ///< the pcurve's start, on its face's surface, from its origin vertex
    double end = 0.0;    ///< its end from its next half-edge's origin vertex
    /// The farthest of its samples, on the surface, from the edge's curve, and of the peaks between
    /// them that could pass the edge's tolerance (geo::pcurveGap).
    double strays = 0.0;
};

/// What validateExact measures on @p h's pcurve with @p tolerance, by the same code on the same
/// geometry: what a PcurveOffCurve or PcurveOffVertex issue on @p h compared with its tolerances,
/// so that a caller can tell how far past them it is (another measure of the same gap, at other
/// points or by another route, can differ in the last bit). On a straight edge whose pcurve the
/// validator accepts without searching the curve (it lies well within the edge's tolerance as it
/// stands), @c strays is 0; a larger tolerance accepts it too. With no exact curve on its edge,
/// @c strays is 0, as the validator measures no stray then; with no pcurve, or no face with an
/// exact surface, all three are 0 (the validator reports those as missing geometry). As for
/// validateExact, @p h's solid must be combinatorially valid: @p h's origin, its next and its
/// next's origin are followed without null checks.
PcurveGaps pcurveGaps(const HalfEdge& h, const math::ModelTolerance& tolerance);

/// An edge's or a vertex's tolerance that a check of validateExact's found short of what it
/// measured by rounding alone, and that measure.
struct RoundingShortfall {
    /// The tolerance the check was bound by: the edge's, or for a pcurve's end its vertex's where
    /// that is the larger. It points into the solid the issue is of.
    double* tolerance = nullptr;
    double measured = 0.0;              ///< what the validator measured (pcurveGaps)
    const ExactIssue* issue = nullptr;  ///< the issue it explains
};

/// @p issues, validateExact's of @p solid with @p tolerance, read as rounding shortfalls: all of
/// them, or none.
struct RoundingTakeUp {
    /// Each issue's shortfalls, where every issue is explained by rounding alone; else empty.
    std::vector<RoundingShortfall> shortfalls;
    /// The first issue that is not, where one is not; else null.
    const ExactIssue* beyond = nullptr;
};

/// How far two measures of one gap on @p h's pcurve can part by rounding alone (16 units in the
/// last place of every magnitude they are computed from): how far from the origin @p solid's
/// points lie (its farthest vertex's distance and its size), how far the surface's and the
/// curve's own defining terms reach (a frame's origin and radii, or the farthest control point),
/// and how far a rounding of each parameter moves its point (|u| |dS/du| + |v| |dS/dv| along the
/// pcurve, |t| |dC/dt| at the curve's ends). Zero where that is not finite. @p h must have a
/// pcurve and a face with an exact surface, as validateExact's pcurve issues do.
double pcurveRounding(const Solid& solid, const HalfEdge& h);

/// @p issues as rounding shortfalls (Ruling 66): each a PcurveOffCurve or PcurveOffVertex issue on
/// the half-edge it names, whose measures (pcurveGaps with @p tolerance) exceed what its check
/// allowed by pcurveRounding at most, every excess a shortfall of the tolerance the check was bound
/// by. All or nothing: any other issue, a pcurve past its tolerance by more (or by a measure that
/// is not a number), or one that runs against its curve, is @c beyond, and nothing is taken up, so
/// a shortfall's growth cannot cover a stray beside it. @p solid must be the issues' solid and
/// combinatorially valid.
RoundingTakeUp roundingShortfalls(const Solid& solid, const std::vector<ExactIssue>& issues,
                                  const math::ModelTolerance& tolerance);

/// validateExact of @p solid again, once the tolerances @p takeUp names have grown (Ruling 66's
/// second validation): what validateExact finds, in its order, by making again only the checks
/// those tolerances enter. A grown edge's tolerance enters every check of the edge (its curve's
/// ends and range, its pcurves' ends, how far they stray and whether they run with it); a grown
/// vertex's, the checks of each edge's ends at it (its curve's end there, its pcurves' ends); and
/// either, the joins of each loop through them, whose faces are checked again whole. Every other
/// check reads what it read when validateExact found @p takeUp's issues, and found nothing there.
/// @p takeUp must be roundingShortfalls' of that validation's issues, all of them shortfalls
/// (beyond null), and @p solid changed since only by those tolerances growing; a take-up with
/// an issue beyond rounding, or a shortfall naming a tolerance not of its issue's edge or that
/// edge's ends, is validated whole.
std::vector<ExactIssue> validateTakenUp(const Solid& solid, const math::ModelTolerance& tolerance,
                                        const RoundingTakeUp& takeUp);

}  // namespace hz::topo
