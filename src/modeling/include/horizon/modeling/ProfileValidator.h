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
};

}  // namespace hz::model
