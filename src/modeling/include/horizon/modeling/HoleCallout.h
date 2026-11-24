#pragma once

// A hole's callout on a drawing: its size, as the hole feature that made it
// says, and how many such holes the part has, on a leader from the hole.

#include <functional>
#include <string>
#include <vector>

namespace hz::model {

struct HoleCallout {
    enum class Kind { Simple, Counterbore, Countersink };
    enum class Depth { Blind, Through, UpToFace };

    /// The edge it points at, by its logical name without the piece it is
    /// cut into ("edge:hole_1/wall|primitive_1/top"): a circle cut into
    /// pieces is one edge to it.
    std::string edge;
    int count = 1;  ///< the holes of this feature the part has, its copies too
    Kind kind = Kind::Simple;
    Depth depth = Depth::Blind;
    double diameter = 0.0;
    double holeDepth = 0.0;  ///< a blind hole's
    double boreDiameter = 0.0;
    double boreDepth = 0.0;
    double sinkDiameter = 0.0;
    double sinkAngleDegrees = 90.0;
};

/// The callout's lines, lengths written by @p length (in the drawing's unit
/// and precision): "3× ⌀6.6 THRU", then "CBORE ⌀11 × 6.4 DEEP" or
/// "CSK ⌀12 × 90°". A blind hole is "⌀6.6 × 10 DEEP"; one up to a face
/// states no depth.
std::vector<std::string> holeCalloutLines(const HoleCallout& callout,
                                          const std::function<std::string(double)>& length);

/// @p edgeName without the piece of a circle it names ("…/piece:1"): the
/// name a hole callout keeps.
std::string withoutPiece(const std::string& edgeName);

}  // namespace hz::model
