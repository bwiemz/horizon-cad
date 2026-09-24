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

/// Hollows a solid to a thin wall by opening one face and offsetting the
/// remaining walls inward by @p thickness.
///
/// Scope: a right prism only — a planar cap, the opposite cap straight below
/// it, and one planar side per edge (a box, or an extruded profile with no
/// other features). The cup is built directly from the two caps: the removed
/// cap's profile offset inward by the thickness is the cavity. Refuses, with
/// a message, anything else, which the cup would silently drop or get wrong:
/// holes, bosses, split faces, tapered sides, more than one open face, several
/// bodies; and a thickness the profile cannot take (an edge of the cavity
/// would collapse, or the cavity cross itself or leave the part).
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
