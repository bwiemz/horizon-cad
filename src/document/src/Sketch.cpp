#include "horizon/document/Sketch.h"

#include <cmath>
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
    return m_placed ? *m_placed : m_plane;
}

void Sketch::setPlane(const draft::SketchPlane& plane) {
    m_plane = plane;
    m_placed.reset();
}

void Sketch::placeOn(const math::Vec3& point, const math::Vec3& normal) {
    const math::Vec3 n = normal.normalized();
    const math::Vec3 from = m_plane.origin() - point;
    // On the plane it was drawn on (as a build finds it again, the part
    // unchanged): there, exactly, not a rounding away.
    if ((n - m_plane.normal()).length() < 1e-12 && std::abs(from.dot(n)) < 1e-12) {
        m_placed = m_plane;
        return;
    }
    const math::Vec3 origin = m_plane.origin() - n * from.dot(n);
    math::Vec3 across = m_plane.xAxis() - n * m_plane.xAxis().dot(n);
    if (across.length() < 1e-6) {
        // Its x axis now along the normal: x is y x normal in a right-handed
        // frame, so the y axis, taken onto the plane, gives it.
        across = (m_plane.yAxis() - n * m_plane.yAxis().dot(n)).cross(n);
    }
    m_placed = draft::SketchPlane(origin, n, across);
}

math::Mat4 Sketch::placement() const {
    if (!m_placed) return math::Mat4::identity();
    return m_placed->localToWorldMatrix() * m_plane.worldToLocalMatrix();
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
