#include "horizon/document/Document.h"
#include "horizon/document/UndoStack.h"

#include <algorithm>

namespace hz::doc {

Document::Document()
    : m_undoStack(std::make_unique<UndoStack>()) {
    m_defaultSketch = std::make_shared<Sketch>();
    m_defaultSketch->setName("Default Sketch");
    m_sketches.push_back(m_defaultSketch);
}

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
    m_constraintSystem.clear();
    m_parameterRegistry.clear();
    m_undoStack->clear();
    m_dirty = false;
    m_filePath.clear();

    m_sketches.clear();
    m_defaultSketch = std::make_shared<Sketch>();
    m_defaultSketch->setName("Default Sketch");
    m_sketches.push_back(m_defaultSketch);
}

void Document::addSketch(std::shared_ptr<Sketch> sketch) {
    if (sketch) m_sketches.push_back(std::move(sketch));
}

std::shared_ptr<Sketch> Document::removeSketch(uint64_t sketchId) {
    auto it = std::find_if(m_sketches.begin(), m_sketches.end(),
                           [sketchId](const auto& s) { return s->id() == sketchId; });
    if (it != m_sketches.end()) {
        auto sketch = *it;
        m_sketches.erase(it);
        return sketch;
    }
    return nullptr;
}

UndoStack& Document::undoStack() { return *m_undoStack; }
const UndoStack& Document::undoStack() const { return *m_undoStack; }

}  // namespace hz::doc
