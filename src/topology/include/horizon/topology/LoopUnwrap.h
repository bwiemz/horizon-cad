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

/// Samples each half-edge's pcurve at @p samples parameters (both ends included, either way
/// round) and chains them, shifting each by whole periods (periodU, periodV; 0 where the surface
/// is not periodic in that direction) so it continues the previous one. @p out then holds
/// samples - 1 chords for each half-edge of @p ring, in ring order, each carrying its half-edge's
/// shift. False when the loop closes only after winding a whole number of periods round the
/// surface; @p out is then not meaningful. @p joinAllowance[k] is how far apart (a distance on
/// the surface) half-edge k's end and the next one's start may be and still join; on an axis
/// where the raw jump is already within it (a pole or an apex, where the derivative vanishes)
/// nothing is shifted: that jump is a move along the singular side, not a wrap. Every half-edge
/// must carry a pcurve.
bool unwrapLoop(const std::vector<const HalfEdge*>& ring, int samples, const geo::Surface& surface,
                double periodU, double periodV, const std::vector<double>& joinAllowance,
                LoopPolygon& out);

/// Shifts @p loop by whole periods so it lies in the window of @p reference (nearest by the
/// middle of its extent), for loops of one face that were each unwrapped from their own start.
void alignLoop(LoopPolygon& loop, const LoopPolygon& reference, double periodU, double periodV);

}  // namespace hz::topo
