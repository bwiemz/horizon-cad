#pragma once

#include <optional>
#include <string>

#include "horizon/math/Vec3.h"
#include "horizon/topology/Solid.h"

namespace hz::model {

/// A flat face's plane (Phase 157): the middle of its outline, and the way
/// it faces, out of the part (unit length).
struct FacePlane {
    math::Vec3 origin;
    math::Vec3 normal;
};

/// +1 when @p solid's face loops wind counterclockwise seen from outside,
/// -1 when they wind the other way: its volume by the divergence theorem,
/// over each loop fanned from its first vertex, has that sign. A solid's
/// loops all wind one way, but which way depends on how it was built.
double outwardSign(const topo::Solid& solid);

/// @p face's plane, facing out of a solid whose outwardSign() is
/// @p outward: through the middle of its outline, normal by Newell's
/// method. Nullopt when it is not flat (its corners out of one plane, or a
/// curved surface under them: a facet of a cylinder's side is flat, but its
/// ideal is not), or has fewer than three corners.
std::optional<FacePlane> planeOf(const topo::Face& face, double outward);

/// A face's name as what refers to it keeps it: without the `/piece:<n>` a
/// Boolean gives each piece of a face it splits (or an edge it keeps: see
/// wholeEdgeName()), so it names the face whole, split or not, and the face
/// again once nothing splits it.
std::string wholeFaceName(const std::string& tag);

/// The plane of the flat face named @p name in @p solid, a name as
/// wholeFaceName() gives it: every piece of the face, which must share one
/// plane. Nullopt, and why in @p why ("is not there", "is no longer flat"),
/// otherwise.
std::optional<FacePlane> planeOfFace(const topo::Solid& solid, const std::string& name,
                                     std::string* why = nullptr);

}  // namespace hz::model
