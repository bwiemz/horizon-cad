#pragma once

#include <memory>
#include <string>
#include <vector>

#include "horizon/topology/Solid.h"
#include "horizon/topology/TopologyID.h"

namespace hz::model {

/// Result of a shell operation.
struct ShellResult {
    std::unique_ptr<topo::Solid> solid;  ///< The hollowed solid (null on failure).
    bool ok = false;
    std::string message;  ///< Diagnostic on failure (empty on success).
};

/// Hollows a solid to a thin wall by removing the given face(s) and offsetting
/// the remaining walls inward by @p thickness.
///
/// Era-2 scope: prism-family solids (box / extrude / loft output — planar
/// lateral faces with a removable planar cap). The hollow is produced by
/// subtracting an inner cutter built from the removed cap's profile offset
/// inward, reusing the Boolean engine. Refuses (with a message) when the wall
/// thickness meets or exceeds the profile inradius (the inner offset would
/// self-intersect) or when the solid is not a supported prism.
class Shell {
public:
    /// @param solid           The solid to hollow (ownership consumed).
    /// @param thickness       Wall thickness (> 0).
    /// @param removedFaceIds  TopologyIDs of faces to open (>= 1; the first
    ///                        defines the shell axis).
    static ShellResult execute(std::unique_ptr<topo::Solid> solid, double thickness,
                               const std::vector<topo::TopologyID>& removedFaceIds);
};

}  // namespace hz::model
