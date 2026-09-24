#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "horizon/drafting/DraftDocument.h"
#include "horizon/drafting/DraftEntity.h"
#include "horizon/drafting/SketchPlane.h"
#include "horizon/drafting/SpatialIndex.h"
#include "horizon/math/IdCounter.h"

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

    const draft::SketchPlane& plane() const;
    void setPlane(const draft::SketchPlane& plane);

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
    draft::SketchPlane m_plane;
    draft::DraftDocument m_drawing;
    std::unique_ptr<cstr::ConstraintSystem> m_constraints;

    static math::IdCounter<uint64_t> s_nextId;
};

}  // namespace hz::doc
