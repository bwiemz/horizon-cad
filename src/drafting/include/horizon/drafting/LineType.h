#pragma once

#include <string>

namespace hz::draft {

/// Standard line types for 2D CAD drawing entities.
enum class LineType : int {
    ByLayer     = 0,  ///< Inherit line type from layer (default for entities).
    Continuous  = 1,  ///< Solid line (default for layers).
    Dashed      = 2,  ///< Long dashes.
    Dotted      = 3,  ///< Dots.
    DashDot     = 4,  ///< Alternating dash and dot.
    Center      = 5,  ///< Long dash, short dash (for center lines).
    Hidden      = 6,  ///< Short dashes (for hidden edges).
    Phantom     = 7   ///< Long dash, two short dashes.
};

/// Total number of non-ByLayer line types (Continuous through Phantom).
inline constexpr int kLineTypeCount = 7;

/// Returns display name, e.g. "Continuous", "Dashed".  Returns "ByLayer" for ByLayer.
const char* lineTypeName(LineType lt);

/// Returns display name for an int line type value.
const char* lineTypeName(int lt);

/// Parse a display name to a LineType.  Returns ByLayer for unrecognized names.
LineType lineTypeFromName(const std::string& name);

/// Returns DXF linetype name, e.g. "CONTINUOUS", "DASHED".
const char* lineTypeDxfName(LineType lt);

/// Parse a DXF linetype name to a LineType.  Returns Continuous for unrecognized names.
LineType lineTypeFromDxfName(const std::string& dxfName);

}  // namespace hz::draft
