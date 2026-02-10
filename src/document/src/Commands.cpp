#include "horizon/document/Commands.h"

namespace hz::doc {

// --- AddEntityCommand ---

AddEntityCommand::AddEntityCommand(draft::DraftDocument& doc,
                                   std::shared_ptr<draft::DraftEntity> entity)
    : m_doc(doc), m_entity(std::move(entity)), m_entityId(0) {
    if (m_entity) m_entityId = m_entity->id();
}

void AddEntityCommand::execute() {
    if (m_entity) {
        m_doc.addEntity(m_entity);
    }
}

void AddEntityCommand::undo() {
    m_doc.removeEntity(m_entityId);
}

std::string AddEntityCommand::description() const {
    return "Add Entity";
}

// --- RemoveEntityCommand ---

RemoveEntityCommand::RemoveEntityCommand(draft::DraftDocument& doc, uint64_t entityId)
    : m_doc(doc), m_entityId(entityId) {
    for (const auto& e : m_doc.entities()) {
        if (e->id() == entityId) {
            m_entity = e;
            break;
        }
    }
}

void RemoveEntityCommand::execute() {
    m_doc.removeEntity(m_entityId);
}

void RemoveEntityCommand::undo() {
    if (m_entity) {
        m_doc.addEntity(m_entity);
    }
}

std::string RemoveEntityCommand::description() const {
    return "Remove Entity";
}

// --- MoveEntityCommand ---

MoveEntityCommand::MoveEntityCommand(draft::DraftDocument& doc,
                                     const std::vector<uint64_t>& entityIds,
                                     const math::Vec2& delta)
    : m_doc(doc), m_entityIds(entityIds), m_delta(delta) {}

void MoveEntityCommand::execute() {
    for (uint64_t id : m_entityIds) {
        for (const auto& e : m_doc.entities()) {
            if (e->id() == id) {
                e->translate(m_delta);
                break;
            }
        }
    }
}

void MoveEntityCommand::undo() {
    math::Vec2 neg{-m_delta.x, -m_delta.y};
    for (uint64_t id : m_entityIds) {
        for (const auto& e : m_doc.entities()) {
            if (e->id() == id) {
                e->translate(neg);
                break;
            }
        }
    }
}

std::string MoveEntityCommand::description() const {
    return "Move Entity";
}

}  // namespace hz::doc
