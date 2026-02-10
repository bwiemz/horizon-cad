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

}  // namespace hz::doc
