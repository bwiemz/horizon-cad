#pragma once

#include "horizon/drafting/DraftEntity.h"

#include <memory>
#include <string>
#include <vector>

namespace hz::model {

/// Result of validating a 2D profile for extrusion or revolution.
struct ProfileValidationResult {
    bool isClosed = false;
    std::vector<std::shared_ptr<draft::DraftEntity>> orderedEdges;
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
    ///   - DraftCircle: always forms a closed loop by itself
    ///
    /// @param entities  The profile entities to validate.
    /// @param tolerance Maximum gap between consecutive endpoints.
    /// @return Validation result with ordered edges if closed.
    static ProfileValidationResult validate(
        const std::vector<std::shared_ptr<draft::DraftEntity>>& entities,
        double tolerance = 1e-6);
};

}  // namespace hz::model
