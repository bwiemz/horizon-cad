#pragma once

#include <vector>

#include "horizon/math/Vec2.h"
#include "horizon/math/Vec3.h"

namespace hz::cam {

/// How the tool moves to a target position.
enum class MoveType {
    Rapid,  ///< non-cutting positioning move (G0)
    Feed,   ///< cutting move at the feed rate (G1)
};

/// A single tool move to an absolute position.
struct Move {
    MoveType type = MoveType::Rapid;
    math::Vec3 target{0.0, 0.0, 0.0};
    double feed = 0.0;  ///< feed rate for a Feed move (ignored for Rapid)
};

/// An ordered list of tool moves — a machining toolpath. Distances are measured
/// between consecutive move targets (the tool starts at the first move's target).
struct Toolpath {
    std::vector<Move> moves;

    /// Summed length of the Feed (cutting) moves.
    double cuttingLength() const;
    /// Summed length of the Rapid (positioning) moves.
    double rapidLength() const;
};

/// Generates 2.5-axis toolpaths (constant cutting depth, positioning at a safe
/// Z plane). Profiles and hole locations are given in the XY plane.
class CamGenerator {
public:
    /// A contour (profile) toolpath: rapid to the first point at @p safeZ, plunge
    /// to @p cutDepth, feed along the profile (closing back to the start when
    /// @p closed), then rapid retract to @p safeZ. Returns an empty path for an
    /// empty profile.
    static Toolpath contour(const std::vector<math::Vec2>& profile, double cutDepth, double safeZ,
                            double feed, bool closed);

    /// A drilling toolpath: for each hole, rapid to (x, y, safeZ), feed-plunge to
    /// @p cutDepth, then rapid retract to @p safeZ.
    static Toolpath drill(const std::vector<math::Vec2>& holes, double cutDepth, double safeZ,
                          double feed);
};

}  // namespace hz::cam
