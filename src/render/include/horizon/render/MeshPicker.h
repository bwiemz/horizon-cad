#pragma once

#include <optional>

#include "horizon/math/Mat4.h"
#include "horizon/math/Vec3.h"
#include "horizon/render/SceneGraph.h"

namespace hz::render {

class Camera;

/// What a ray or a click reached on a mesh.
struct MeshHit {
    double distance = 0.0;  ///< along the ray, from its origin
    math::Vec3 point;       ///< where, in world coordinates
    int face = -1;          ///< index into the mesh's faceTags, or -1
    int edge = -1;          ///< index into the mesh's edges, or -1
};

/// Picking on the CPU: a face by the ray through the cursor, an edge by its
/// distance on screen. It needs no GL, so the window's tests pick as a user
/// does. A mesh is placed in the world by @p model (its node's world
/// transform).
class MeshPicker {
public:
    /// The nearest triangle of @p mesh that the ray from @p origin along
    /// @p direction (world coordinates) hits in front of it, with the face it
    /// belongs to (-1 when the mesh does not name its faces).
    static std::optional<MeshHit> pickFace(const MeshData& mesh, const math::Mat4& model,
                                           const math::Vec3& origin, const math::Vec3& direction);

    /// The edge of @p mesh drawn nearest the screen point (@p x, @p y), in
    /// Qt's coordinates (0 at the top) on a @p width by @p height view, when
    /// it is within @p tolerancePx pixels and not behind what the ray through
    /// that point meets first, @p hiddenBeyond along it.
    static std::optional<MeshHit> pickEdge(const MeshData& mesh, const math::Mat4& model,
                                           const Camera& camera, double x, double y, int width,
                                           int height, double tolerancePx, double hiddenBeyond);
};

}  // namespace hz::render
