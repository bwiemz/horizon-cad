#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "horizon/drafting/DraftDocument.h"
#include "horizon/drafting/DraftEntity.h"
#include "horizon/drafting/SketchPlane.h"
#include "horizon/drafting/SpatialIndex.h"
#include "horizon/math/IdCounter.h"
#include "horizon/math/Mat4.h"
#include "horizon/math/Vec3.h"

// Forward-declare ConstraintSystem to allow Sketch.h to be included in contexts
// that don't necessarily link Horizon::Constraint. The full header is in Sketch.cpp.
namespace hz::cstr {
class ConstraintSystem;
}

namespace hz::doc {

/// A Sketch owns a SketchPlane, a drawing in the plane's local 2D coordinates
/// (its entities, their spatial index, blocks and dimension style), and a
/// ConstraintSystem. While a sketch is edited, the window's drawing tools work
/// on its drawing (Document::activeDrawing()).
class Sketch {
public:
    Sketch();  // Default XY plane
    explicit Sketch(const draft::SketchPlane& plane);
    ~Sketch();

    // Non-copyable, movable
    Sketch(const Sketch&) = delete;
    Sketch& operator=(const Sketch&) = delete;
    Sketch(Sketch&&) noexcept;
    Sketch& operator=(Sketch&&) noexcept;

    uint64_t id() const;
    void setId(uint64_t id);

    const std::string& name() const;
    void setName(const std::string& name);

    /// Where it is: on the face it follows, where the last build placed it
    /// (placed()); else on the plane it was drawn on.
    const draft::SketchPlane& plane() const;
    /// The plane it was drawn on, and the file keeps (Phase 157).
    const draft::SketchPlane& drawnPlane() const { return m_plane; }
    /// Draw it on @p plane: where it is from now on, placed nowhere else.
    void setPlane(const draft::SketchPlane& plane);

    /// The face it follows (Phase 157), by its whole name
    /// (model::wholeFaceName()); "" for a sketch on a fixed plane. Each build
    /// places it on that face as the part stands before the first feature
    /// made from it.
    const std::string& face() const { return m_face; }
    void setFace(std::string face) { m_face = std::move(face); }

    /// Where a build last placed it on its face; nullopt until one has, or
    /// once it is drawn on a plane again.
    const std::optional<draft::SketchPlane>& placed() const { return m_placed; }
    void setPlaced(const std::optional<draft::SketchPlane>& placed) { m_placed = placed; }

    /// Place it on the plane through @p point facing @p normal: its drawn
    /// plane's origin and x axis taken straight onto that plane. A face that
    /// moves along its normal carries it along unturned, one that grows
    /// leaves it where it is, and the plane it was drawn on places it there.
    void placeOn(const math::Vec3& point, const math::Vec3& normal);

    /// The move from its drawn plane to where it is placed; none when it is
    /// not placed. What takes a direction or a point given with it (an
    /// extrusion's, a revolve's axis) to where it now is.
    math::Mat4 placement() const;

    /// The sketch's drawing, in the plane's local 2D coordinates.
    draft::DraftDocument& drawing() { return m_drawing; }
    const draft::DraftDocument& drawing() const { return m_drawing; }

    // Entity management — entities store local 2D coordinates. These are the
    // drawing's own (drawing()).
    void addEntity(std::shared_ptr<draft::DraftEntity> entity);
    void removeEntity(uint64_t entityId);
    const std::vector<std::shared_ptr<draft::DraftEntity>>& entities() const;
    std::vector<std::shared_ptr<draft::DraftEntity>>& entities();

    // Constraint system
    cstr::ConstraintSystem& constraintSystem();
    const cstr::ConstraintSystem& constraintSystem() const;

    // Spatial index
    const draft::SpatialIndex& spatialIndex() const;
    draft::SpatialIndex& spatialIndex();
    void rebuildSpatialIndex();

    /// Clear all entities, constraints, and the spatial index.
    void clear();

private:
    uint64_t m_id;
    std::string m_name;
    draft::SketchPlane m_plane;  ///< drawn on
    std::string m_face;
    std::optional<draft::SketchPlane> m_placed;
    draft::DraftDocument m_drawing;
    std::unique_ptr<cstr::ConstraintSystem> m_constraints;

    static math::IdCounter<uint64_t> s_nextId;
};

}  // namespace hz::doc
