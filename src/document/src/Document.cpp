#include "horizon/document/Document.h"
#include "horizon/document/UndoStack.h"

namespace hz::doc {

Document::Document()
    : m_undoStack(std::make_unique<UndoStack>()) {}

Document::~Document() = default;

uint64_t Document::addEntity(std::shared_ptr<draft::DraftEntity> entity) {
    if (!entity) return 0;
    uint64_t id = entity->id();
    m_draftDoc.addEntity(std::move(entity));
    m_dirty = true;
    return id;
}

std::shared_ptr<draft::DraftEntity> Document::removeEntity(uint64_t id) {
    const auto& entities = m_draftDoc.entities();
    std::shared_ptr<draft::DraftEntity> found;
    for (const auto& e : entities) {
        if (e->id() == id) {
            found = e;
            break;
        }
    }
    m_draftDoc.removeEntity(id);
    m_dirty = true;
    return found;
}

void Document::clear() {
    m_draftDoc.clear();
    m_layerManager.clear();
    m_undoStack->clear();
    m_dirty = false;
    m_filePath.clear();
}

UndoStack& Document::undoStack() { return *m_undoStack; }
const UndoStack& Document::undoStack() const { return *m_undoStack; }

}  // namespace hz::doc
