#include "horizon/document/Sketch.h"
#include "horizon/constraint/ConstraintSystem.h"

#include <algorithm>
#include <stdexcept>

namespace hz::doc {

uint64_t Sketch::s_nextId = 1;

Sketch::Sketch()
    : m_id(s_nextId++), m_name(), m_plane(), m_constraints(std::make_unique<cstr::ConstraintSystem>()) {}

Sketch::Sketch(const draft::SketchPlane& plane)
    : m_id(s_nextId++), m_name(), m_plane(plane), m_constraints(std::make_unique<cstr::ConstraintSystem>()) {}

Sketch::~Sketch() = default;

Sketch::Sketch(Sketch&&) noexcept = default;
Sketch& Sketch::operator=(Sketch&&) noexcept = default;

uint64_t Sketch::id() const {
    return m_id;
}

void Sketch::setId(uint64_t id) {
    m_id = id;
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
    m_entities.push_back(entity);
    m_spatialIndex.insert(entity);
}

void Sketch::removeEntity(uint64_t entityId) {
    m_spatialIndex.remove(entityId);
    m_entities.erase(
        std::remove_if(m_entities.begin(), m_entities.end(),
                       [entityId](const std::shared_ptr<draft::DraftEntity>& e) {
                           return e && e->id() == entityId;
                       }),
        m_entities.end());
}

const std::vector<std::shared_ptr<draft::DraftEntity>>& Sketch::entities() const {
    return m_entities;
}

std::vector<std::shared_ptr<draft::DraftEntity>>& Sketch::entities() {
    return m_entities;
}

cstr::ConstraintSystem& Sketch::constraintSystem() {
    return *m_constraints;
}

const cstr::ConstraintSystem& Sketch::constraintSystem() const {
    return *m_constraints;
}

const draft::SpatialIndex& Sketch::spatialIndex() const {
    return m_spatialIndex;
}

draft::SpatialIndex& Sketch::spatialIndex() {
    return m_spatialIndex;
}

void Sketch::rebuildSpatialIndex() {
    m_spatialIndex.rebuild(m_entities);
}

void Sketch::clear() {
    m_entities.clear();
    m_constraints->clear();
    m_spatialIndex.clear();
}

}  // namespace hz::doc
