#pragma once

// A face's loops in (u, v), continuous across a periodic surface's seam. The exact validator
// reads loops through these functions and the mesher builds its boundaries from them, so the
// two agree on every face: a face the validator calls clean unwraps the same way for both.

#include <cstddef>
#include <vector>

#include "horizon/geometry/curves/Curve2D.h"
#include "horizon/geometry/surfaces/Surface.h"
#include "horizon/math/ModelTolerance.h"
#include "horizon/math/Vec2.h"
#include "horizon/topology/HalfEdge.h"

namespace hz::topo {

/// How many parameters each pcurve is sampled at when its loop is unwrapped: its two ends and
/// 15 between. The unwrapping and alignment decisions depend on it, so callers that must agree
/// pass the same count.
inline constexpr int kLoopSamples = 17;

/// The parameter of sample @p i of @p samples along @p h's pcurve: pcurveStart at i = 0,
/// pcurveEnd at i = samples - 1 (either way round), evenly between. Every caller that must agree
/// with the unwrapping (the validator's checks, the mesher's layout) samples through this one
/// formula, so their samples are the same to the last bit.
double loopSampleParameter(const HalfEdge& h, int i, int samples);

/// One chord of a sampled pcurve: the curve between parameters t0 and t1, shifted by whole
/// periods so the loop it belongs to is continuous; a and b are its (shifted) ends.
struct LoopSegment {
    const geo::Curve2D* curve = nullptr;
    double t0 = 0.0;
    double t1 = 0.0;
    math::Vec2 shift;
    math::Vec2 a;
    math::Vec2 b;
};

/// A loop as a closed run of chords, continuous in unwrapped (u, v).
using LoopPolygon = std::vector<LoopSegment>;

/// The period of @p surface along u: the width of its u range when u wraps, else 0.
double uPeriod(const geo::Surface& surface);
/// The period of @p surface along v: the width of its v range when v wraps, else 0.
double vPeriod(const geo::Surface& surface);

/// The half-edges of loop @p wire of @p face, from the wire's first, following next. False when
/// the cycle leaves the face or does not return to its start within @p limit half-edges; @p ring
/// then holds what was walked.
bool loopHalfEdges(const Face& face, const Wire& wire, std::size_t limit,
                   std::vector<const HalfEdge*>& ring);

/// How far apart (a distance on the surface) the end of @p h's pcurve and the start of the next
/// half-edge's may be and still join: each end may be off their shared vertex by the larger of
/// its edge's and the vertex's effective tolerance, so the two together by the sum.
double joinAllowance(const HalfEdge& h, const math::ModelTolerance& tolerance);

/// joinAllowance(h, tolerance) for @p h followed by @p next, which need not be linked to it yet
/// (a loop being put together): the same sum, at @p next's origin.
double joinAllowance(const HalfEdge& h, const HalfEdge& next,
                     const math::ModelTolerance& tolerance);

/// Which way a loop must run in (u, v): counter-clockwise, clockwise, or either where the caller
/// does not know (or asks the loop which).
enum class LoopSense { Either, CounterClockwise, Clockwise };

/// The way a loop of @p face must run: an outer loop counter-clockwise unless the face is
/// reversed, an inner loop the other way.
LoopSense loopSense(const Face& face, bool outer);

/// Whether @p surface comes to a point along the straight (u, v) path from @p from to @p to, as
/// written (no period taken off): its two ends, and points evenly along it (kLoopSamples in all),
/// each within @p allowance (a distance on the surface) of both ends. A pole's or an apex's run
/// in (u, v) is one point all along it, whichever window its ends are written in and however far
/// apart they are; a jump round a seam of a surface that does not come to a point there is not.
/// It measures along the way, not by a derivative at one end: a pcurve that ends a hair short of
/// a pole, where the derivative along the pole is small but not zero, still joins one that starts
/// on it at another longitude.
bool runsAlongPoint(const geo::Surface& surface, const math::Vec2& from, const math::Vec2& to,
                    double allowance);

/// Samples each half-edge's pcurve at @p samples parameters (both ends included, either way
/// round) and chains them, shifting each by whole periods (periodU, periodV; 0 where the surface
/// is not periodic in that direction) so it continues the previous one. @p out then holds
/// samples - 1 chords for each half-edge of @p ring, in ring order, each carrying its half-edge's
/// shift. False when the loop closes only after winding a whole number of periods round the
/// surface; @p out then holds the chain as unwrapped, its end a whole number of periods from its
/// start. @p joinAllowance[k] is how far apart (a distance on
/// the surface) half-edge k's end and the next one's start may be and still join; where the raw
/// jump is within it (a rounding), or runs along a point of the surface (runsAlongPoint: a pole
/// or an apex, whether the pcurves end on it or a hair off it), nothing is shifted: that jump is
/// a move along the singular side, not a wrap. Which way round the singular side it runs is the
/// loop's to say: where the loop would otherwise close only after winding, the whole periods it
/// winds are taken at the last join where its point is the same with them added (a pole's or
/// apex's), and every chord after that join is moved by them.
/// A loop that wraps at a cone's rim wraps at its apex when it runs the other way (a mirrored
/// copy's loops are reversed).
///
/// A loop that runs round a pole between two pcurves without a seam (a lune between two
/// meridians, a ball split by them) does not fix which way round the pole it runs: the two
/// meridians bound two regions, and its face's sense picks one. With @p sense given, where the
/// loop as unwrapped runs the other way (its pole runs laid as the windows its pcurves are
/// written in happen to put them), each run along a pole, or along an apex, is laid round the
/// pole the way that keeps the face on the side its sense says (no more than a period), provided
/// the runs' changes cancel, so the loop still closes; a loop they cannot turn is left as it was.
/// Every half-edge must carry a pcurve.
bool unwrapLoop(const std::vector<const HalfEdge*>& ring, int samples, const geo::Surface& surface,
                double periodU, double periodV, const std::vector<double>& joinAllowance,
                LoopPolygon& out, LoopSense sense = LoopSense::Either);

/// Shifts @p loop by whole periods so it lies in the window of @p reference (nearest by the
/// middle of its extent), for loops of one face that were each unwrapped from their own start.
void alignLoop(LoopPolygon& loop, const LoopPolygon& reference, double periodU, double periodV);

/// The signed area (positive counter-clockwise) of an unwrapped loop's chords, with the jump
/// from each pcurve's end to the next one's start included: the area the exact validator judges
/// a loop's orientation by. A jump along a pole or an apex is closed along the run unwrapLoop
/// laid for it round the pole (the arc the face spans there, where unwrapLoop was given the
/// face's sense), not across the window its pcurves happen to be written in. Everyone who must
/// agree with it (the STEP reader's face sense, the seams model::insertSeams lays) measures
/// through this one function.
double signedArea(const LoopPolygon& loop);

}  // namespace hz::topo
