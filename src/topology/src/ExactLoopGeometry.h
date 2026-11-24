#pragma once

// Internal to the exact validator: the 2D geometry of a face's loops in (u, v), once they are
// unwrapped (horizon/topology/LoopUnwrap.h).

#include <functional>
#include <vector>

#include "horizon/geometry/curves/Curve2D.h"
#include "horizon/geometry/surfaces/Surface.h"
#include "horizon/math/RobustPredicates.h"
#include "horizon/math/Vec2.h"
#include "horizon/topology/HalfEdge.h"
#include "horizon/topology/LoopUnwrap.h"

namespace hz::topo::detail {

/// How far apart two (u, v) points may be and still be the same point of the surface, at a
/// given point.
using UvTolerance = std::function<double(const math::Vec2&)>;

/// Signed area (positive counter-clockwise) of the chords' polygon.
double signedArea(const LoopPolygon& loop);

/// True when two non-adjacent chords of the loop cross for real: a crossing of chords is
/// confirmed by bisecting both curves until their deviation from the chords is below the
/// tolerance, so the sag of a coarse sampling does not count.
bool selfIntersects(const LoopPolygon& loop, const UvTolerance& tolerance);

/// True when a chord of @p p and one of @p q cross for real (confirmed as above).
bool loopsCross(const LoopPolygon& p, const LoopPolygon& q, const UvTolerance& tolerance);

/// True when @p point lies outside @p outer by more than the chords' greatest sag, so the
/// chord approximation cannot be what puts it there.
bool clearlyOutside(const LoopPolygon& outer, const math::Vec2& point);

/// True when @p point lies inside @p loop by more than the chords' greatest sag (each chord's
/// own, as for clearlyOutside).
bool clearlyInside(const LoopPolygon& loop, const math::Vec2& point);

/// One loop, indexed for the questions the validator asks of it many times: its chords and its
/// polygon's edges (from chord start to chord start, as clearlyInside's polygon) bucketed by
/// height, and each chord's sag, once. Every answer is the one the plain functions above give:
/// a chord pair whose boxes do not meet cannot cross properly, a polygon edge that does not
/// span a point's height adds nothing to its winding, and a point outside a chord's box grown
/// by its sag (and a margin far above rounding) is outside its band; only those are skipped.
/// A loop with a coordinate that is not finite is asked plainly. It refers to @p loop, which
/// must outlive it unchanged.
class LoopIndex {
public:
    explicit LoopIndex(const LoopPolygon& loop);

    /// selfIntersects(loop, tolerance).
    bool selfIntersects(const UvTolerance& tolerance) const;
    /// loopsCross(loop, other's loop, tolerance).
    bool crosses(const LoopIndex& other, const UvTolerance& tolerance) const;
    /// clearlyInside(loop, point) for Inside, clearlyOutside(loop, point) for Outside.
    bool clearly(math::Containment side, const math::Vec2& point);

private:
    /// Indices of boxes, bucketed by the heights they span.
    struct Buckets {
        double lo = 0.0;
        double scale = 0.0;
        int count = 1;
        std::vector<std::vector<int>> members;
        /// The bucket of height @p y: monotone in y, so a box spanning y is in its bucket.
        int of(double y) const;
        void build(double ylo, double yhi, std::size_t items);
        void add(int item, double ylo, double yhi);
    };
    struct Box {
        double x0, y0, x1, y1;
    };

    math::Containment classify(const math::Vec2& point) const;
    bool inBand(const math::Vec2& point) const;
    void prepareBands();

    const LoopPolygon& loop_;
    bool finite_ = true;
    Box extent_{};  ///< of the polygon's vertices (the chords' starts)
    std::vector<Box> chordBox_;
    Buckets chords_;
    Buckets edges_;
    bool bandsReady_ = false;
    std::vector<double> sag_;
    std::vector<Box> bandBox_;
    Buckets bands_;
};

}  // namespace hz::topo::detail
