#pragma once

// A sketch profile's loops as exact curves (stage 3): what the exact extrude builds its faces on.
// Private to the modeling library.

#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "horizon/drafting/SketchPlane.h"
#include "horizon/geometry/Frame.h"
#include "horizon/geometry/curves/Curve2D.h"
#include "horizon/geometry/curves/Curve3.h"
#include "horizon/math/Vec2.h"
#include "horizon/math/Vec3.h"
#include "horizon/modeling/ProfileValidator.h"

namespace hz::model::exact {

/// A curve from one point to another along it: in the sketch's (x, y) (a pcurve on a plane whose
/// (u, v) are the sketch's), or in 3D.
template <typename Curve>
struct Course {
    std::shared_ptr<const Curve> curve;
    double start = 0.0;  ///< its parameter at the course's start
    double end = 0.0;    ///< and at its end (below the start where it runs backwards)
};

/// One curve of a profile loop, as the loop runs it: from the corner `start` to the corner `end`.
struct LoopCurve {
    enum class Kind { Line, Arc, Circle };
    Kind kind = Kind::Line;
    /// What its face is named after: its entity's source (ProfileValidationResult::edgeSources),
    /// "e<id>", or "e<id>.<k>" for side k of a rectangle or polyline.
    std::string source;
    math::Vec2 start;  ///< the corner it leaves, in the sketch's (x, y)
    math::Vec2 end;    ///< the corner it reaches; a closed curve's is its start
    /// An arc's or a circle's own circle, which it follows counter-clockwise from the angle
    /// `ownStart` by `sweep` radians (2 pi for a circle, whose own start is angle 0: on the
    /// sketch's x axis, where master's ring starts).
    math::Vec2 center;
    double radius = 0.0;
    double ownStart = 0.0;
    double sweep = 0.0;
    /// Whether the loop runs an arc or a circle its own way, counter-clockwise; a line runs from
    /// `start` to `end` and is always forward.
    bool forward = true;

    bool curved() const { return kind != Kind::Line; }
    /// Its own start and end, in the sketch's (x, y): on its circle for an arc, where the corners
    /// the loop gives it may miss them by as much as the sketch's ends missed each other.
    math::Vec2 ownStartPoint() const;
    math::Vec2 ownEndPoint() const;

    /// Its course in the sketch's (x, y), from `start` to `end` (@p along) or back: a Line2D, or
    /// a Circle2D over its angles.
    Course<geo::Curve2D> pcurve(bool along) const;
    /// The frame of its own circle on @p plane moved by @p offset: centred on it, x toward its own
    /// start, z the plane's normal (so that the angle from x runs its own way).
    geo::Frame ownFrame(const draft::SketchPlane& plane, const math::Vec3& offset) const;
    /// Its course in 3D on @p plane moved by @p offset, from `start` to `end` (@p along) or back:
    /// a LineCurve between the corners (parameter the length along it), or a CircleCurve in
    /// ownFrame() (parameter the angle from its own start, 0 to sweep).
    Course<geo::Curve3> curve(const draft::SketchPlane& plane, const math::Vec3& offset,
                              bool along) const;
};

/// A closed loop of a profile, its material on its left: an outer loop runs counter-clockwise in
/// the sketch's (x, y), a hole clockwise. Curve i runs from corner i to corner i + 1.
struct Loop {
    std::vector<LoopCurve> curves;
    /// Whether it runs against the chain the profile validator made of it (and master's faceted
    /// extrude followed): then its curve i is the chain's curve n - 1 - i, and its corner i the
    /// chain's corner (n - i) mod n, n the number of curves.
    bool againstChain = false;
};

/// A region of a profile as exact loops: its outer loop and its holes.
struct Region {
    Loop outer;
    std::vector<Loop> holes;
};

/// @p loop run the other way round: its curves in the other order, each from its end to its start
/// (an arc's or a circle's `forward` flipped), and `againstChain` flipped.
void reverse(Loop& loop);

/// How far the curves of @p loop that meet at its corner @p j miss it: an arc's or a circle's own
/// end may, by as much as the sketch's ends missed each other (a line runs corner to corner).
double cornerGap(const Loop& loop, std::size_t j);

/// @p region's loops as exact curves, each oriented (Loop). The corners are where master's
/// faceted extrude put its vertices (ringstack::sampleProfile): the first curve's start, then
/// where each curve ends as the loop chains them. Nullopt, with @p why, where a curve is not a
/// line, an arc or a circle, a line has no length or an arc no radius, or a loop encloses no
/// area.
std::optional<Region> regionOf(const ProfileRegion& region, std::string* why);

}  // namespace hz::model::exact
