#pragma once

#include <optional>

#include "horizon/math/Mat4.h"
#include "horizon/math/Vec3.h"
#include "horizon/topology/Solid.h"
#include "horizon/topology/TopologyID.h"

namespace hz::model {

/// Geometric mate constraint types between component faces.
///
/// Defined in the modeling layer (the solver consumes them); the document
/// layer aliases this enum for storage on assemblies.
enum class MateType {
    Coincident,     ///< Planar faces lie in the same plane (anti-parallel normals).
    Concentric,     ///< Cylindrical faces share an axis.
    Distance,       ///< Planar faces parallel at a signed offset.
    Angle,          ///< Angle between planar face normals equals value (radians).
    Parallel,       ///< Directions parallel.
    Perpendicular,  ///< Directions perpendicular.
    Tangent,        ///< Cylinder tangent to a plane.
    Fixed,          ///< Grounds a component (uses reference `a` only).
};

/// Kind of geometry a mate frame describes.
enum class MateFrameKind {
    Planar,       ///< origin = point on plane, direction = plane normal.
    Cylindrical,  ///< origin = point on axis, direction = axis, radius set.
};

/// The geometric abstraction a mate constrains: a plane or an axis.
struct MateFrame {
    MateFrameKind kind = MateFrameKind::Planar;
    math::Vec3 origin;
    math::Vec3 direction;  ///< Unit length.
    double radius = 0.0;   ///< Cylindrical frames only.

    /// The frame placed by a component transform.
    [[nodiscard]] MateFrame transformed(const math::Mat4& m) const;
};

/// Extracts mate frames from B-Rep faces.
class MateGeometry {
public:
    /// Find a face by TopologyID: exact tag match preferred, otherwise the
    /// first face whose ID descends from @p id (genealogy resolution).
    static const topo::Face* findFace(const topo::Solid& solid, const topo::TopologyID& id);

    /// Extract a mate frame from a face (in the part's local coordinates).
    /// Planar faces yield Planar frames; cylindrical faces yield axis
    /// frames. Returns nullopt for unsupported geometry.
    static std::optional<MateFrame> frameForFace(const topo::Face& face);
};

}  // namespace hz::model
