#include "horizon/document/Sketch.h"

#include <utility>

#include "horizon/constraint/ConstraintSystem.h"

namespace hz::doc {

math::IdCounter<uint64_t> Sketch::s_nextId{1};

Sketch::Sketch()
    : m_id(s_nextId.next()),
      m_name(),
      m_plane(),
      m_constraints(std::make_unique<cstr::ConstraintSystem>()) {}

Sketch::Sketch(const draft::SketchPlane& plane)
    : m_id(s_nextId.next()),
      m_name(),
      m_plane(plane),
      m_constraints(std::make_unique<cstr::ConstraintSystem>()) {}

Sketch::~Sketch() = default;

Sketch::Sketch(Sketch&&) noexcept = default;
Sketch& Sketch::operator=(Sketch&&) noexcept = default;

uint64_t Sketch::id() const {
    return m_id;
}

void Sketch::setId(uint64_t id) {
    m_id = id;
    // Keep the global counter ahead of loaded IDs so sketches created later
    // (possibly in another open document) never collide.
    s_nextId.reserveThrough(id);
}

const std::string& Sketch::name() const {
    return m_name;
}

void Sketch::setName(const std::string& name) {
    m_name = name;
}

const draft::SketchPlane& Sketch::plane() const {
    return m_plane;
}

void Sketch::setPlane(const draft::SketchPlane& plane) {
    m_plane = plane;
}

void Sketch::addEntity(std::shared_ptr<draft::DraftEntity> entity) {
    if (!entity) return;
    m_drawing.addEntity(std::move(entity));
}

void Sketch::removeEntity(uint64_t entityId) {
    m_drawing.removeEntity(entityId);
}

const std::vector<std::shared_ptr<draft::DraftEntity>>& Sketch::entities() const {
    return m_drawing.entities();
}

std::vector<std::shared_ptr<draft::DraftEntity>>& Sketch::entities() {
    return m_drawing.entities();
}

cstr::ConstraintSystem& Sketch::constraintSystem() {
    return *m_constraints;
}

const cstr::ConstraintSystem& Sketch::constraintSystem() const {
    return *m_constraints;
}

const draft::SpatialIndex& Sketch::spatialIndex() const {
    return m_drawing.spatialIndex();
}

draft::SpatialIndex& Sketch::spatialIndex() {
    return m_drawing.spatialIndex();
}

void Sketch::rebuildSpatialIndex() {
    m_drawing.rebuildSpatialIndex();
}

void Sketch::clear() {
    m_drawing.clear();
    m_constraints->clear();
}

}  // namespace hz::doc
