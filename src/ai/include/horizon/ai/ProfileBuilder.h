#pragma once

#include <memory>
#include <nlohmann/json.hpp>
#include <vector>

namespace hz::draft {
class DraftEntity;
}

namespace hz::ai {

/// Drafting entities from the shapes a model describes, in a plane's own 2D
/// coordinates (millimetres; angles in degrees, counterclockwise from +x):
///
///   {"type": "line",      "from": [x, y], "to": [x, y]}
///   {"type": "polyline",  "points": [[x, y], ...], "closed": true}
///   {"type": "rectangle", "corner": [x, y] | "center": [x, y], "width", "height", "angle"?}
///   {"type": "circle",    "center": [x, y], "radius" | "diameter"}
///   {"type": "arc",       "center": [x, y], "radius", "start_angle", "end_angle"}
///   {"type": "slot",      "center": [x, y], "length", "width", "angle"?}
///   {"type": "polygon",   "center": [x, y], "sides", "radius", "angle"?}
///   {"type": "text",      "position": [x, y], "text", "height"?}    (when @p allowText)
///
/// Any of them may say "construction": true. A point is [x, y] or {"x", "y"}.
/// Throws std::invalid_argument saying which entity and which argument is
/// wrong; nothing is half-built.
std::vector<std::shared_ptr<draft::DraftEntity>> buildEntities(const nlohmann::json& entities,
                                                               bool allowText);

/// The JSON schema of one entity above, for a tool's input schema.
nlohmann::json entitySchema(bool allowText);

}  // namespace hz::ai
