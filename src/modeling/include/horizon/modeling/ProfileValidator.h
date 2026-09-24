#pragma once

#include <memory>
#include <string>
#include <vector>

#include "horizon/drafting/DraftEntity.h"

namespace hz::model {

/// Result of validating a 2D profile for extrusion or revolution.
struct ProfileValidationResult {
    bool isClosed = false;
    std::vector<std::shared_ptr<draft::DraftEntity>> orderedEdges;
    /// Where each ordered edge came from, for naming what it builds:
    /// "e<id>" for a sketch entity, "e<id>.<k>" for side k of a rectangle or
    /// polyline. Entity ids are saved with the document, so these survive
    /// reopening it, and an edit elsewhere in the sketch leaves them alone.
    std::vector<std::string> edgeSources;
    std::string errorMessage;
};

/// A region of a profile: its outer loop and the loops of the holes in it,
/// each a closed loop as ProfileValidator::validate() gives one.
struct ProfileRegion {
    ProfileValidationResult outer;
    std::vector<ProfileValidationResult> holes;
};

/// The regions a sketch's curves bound, or why they bound none.
struct ProfileRegions {
    std::vector<ProfileRegion> regions;
    std::string errorMessage;

    bool ok() const { return errorMessage.empty() && !regions.empty(); }
    /// One region with no holes: a profile as validate() accepts it.
    bool isSingleLoop() const { return regions.size() == 1 && regions.front().holes.empty(); }
};

/// Validates that a set of 2D draft entities forms a single closed loop
/// suitable for extrusion or revolution operations.
class ProfileValidator {
public:
    /// Validate that the given entities form a closed loop.
    ///
    /// Supported entity types:
    ///   - DraftLine: uses start() / end()
    ///   - DraftArc: uses startPoint() / endPoint()
    ///   - DraftRectangle, DraftPolyline: taken as their line segments (a
    ///     closed polyline includes its closing segment), so a shape drawn
    ///     with the Rectangle or Polyline tool is a profile like any other;
    ///     `orderedEdges` then holds those segments as DraftLines
    ///   - DraftCircle: always forms a closed loop by itself
    /// Anything else (an ellipse, a spline, text) is reported by name.
    ///
    /// @param entities  The profile entities to validate.
    /// @param tolerance Maximum gap between consecutive endpoints.
    /// @return Validation result with ordered edges if closed.
    static ProfileValidationResult validate(
        const std::vector<std::shared_ptr<draft::DraftEntity>>& entities, double tolerance = 1e-6);

    /// The regions @p entities bound: every closed loop they make, loops
    /// nested in one another read by the even-odd rule (a loop inside one
    /// region's outer loop is a hole in it; a loop inside that hole starts a
    /// region of its own). Text, dimensions, leaders and hatches are notes on
    /// the sketch, not its shape, and are passed over. Everything else must
    /// close into loops that neither cross themselves nor each other.
    static ProfileRegions regions(const std::vector<std::shared_ptr<draft::DraftEntity>>& entities,
                                  double tolerance = 1e-6);
};

}  // namespace hz::model
