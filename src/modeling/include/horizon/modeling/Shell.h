#pragma once

#include <memory>
#include <string>
#include <vector>

#include "horizon/modeling/Naming.h"
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

    /// Phase 163: hollow a body by offsetting each face, not the profile.
    ///
    /// The cavity is @p solid with every face moved inward by @p thickness,
    /// and each face to open (@p openFaceIds, or their pieces and facets)
    /// moved outward, so the cavity comes out through it. Each corner of the
    /// cavity is where its faces' offset surfaces meet: a flat face's plane
    /// moved along its normal; a facet of a cylinder, cone or sphere on that
    /// surface grown or shrunk. The shell is @p solid less the cavity, so the
    /// part keeps its faces, their names and their true surfaces; the
    /// cavity's faces are named `<featureID>/inner:<face>`, on their offset
    /// ideals.
    ///
    /// Takes any number of open faces, holes, bosses and curved walls, on a
    /// single body. Refuses, with a message: a corner where four or more
    /// faces meet whose offsets do not meet in a point; a face on another
    /// kind of surface; a face opened in part; and a wall too thick for the
    /// part (a face or hole of the cavity collapses, an edge turns over, or
    /// a face's loop crosses itself). Two faces of the cavity crossing each
    /// other past those local checks is not detected: no check in the
    /// kernel sees faces crossing yet.
    static ShellResult executeOffset(const topo::Solid& solid, double thickness,
                                     const std::vector<topo::TopologyID>& openFaceIds,
                                     const std::string& featureID,
                                     NamingScheme naming = NamingScheme::Stable);
};

}  // namespace hz::model
