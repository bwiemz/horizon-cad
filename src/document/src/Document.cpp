#include "horizon/document/Document.h"

#include <algorithm>

#include "horizon/document/UndoStack.h"

namespace hz::doc {

Document::Document() : m_undoStack(std::make_unique<UndoStack>()) {}

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

bool Document::isDirty() const {
    return m_dirty || !m_undoStack->isClean();
}

void Document::setDirty(bool dirty) {
    m_dirty = dirty;
    if (!dirty)
        m_undoStack->setClean();  // notifies through the undo stack
    else if (m_onChange)
        m_onChange();
}

void Document::setChangeCallback(std::function<void()> callback) {
    m_onChange = std::move(callback);
    if (m_onChange) {
        m_undoStack->setChangeCallback([this] { m_onChange(); });
    } else {
        m_undoStack->setChangeCallback(nullptr);
    }
}

void Document::clear() {
    m_draftDoc.clear();
    m_layerManager.clear();
    m_constraintSystem.clear();
    m_parameterRegistry.clear();
    m_undoStack->clear();
    m_dirty = false;
    m_filePath.clear();

    m_editedSketch.reset();
    m_sketches.clear();

    m_featureTree.clear();
    m_solid.reset();
    m_lastBuildMessage.clear();
    m_failedFeatureIndex = -1;
    m_built = false;
}

bool Document::rebuildModel() {
    return applyBuild(m_featureTree.buildWithDiagnostics());
}

bool Document::applyBuild(BuildResult result) {
    if (result.cancelled) return m_failedFeatureIndex < 0;
    m_solid = std::move(result.solid);
    m_lastBuildMessage = result.failureMessage;
    m_failedFeatureIndex = result.failedFeatureIndex;
    m_built = true;
    m_builtRevision = m_featureTree.revision();
    return m_failedFeatureIndex < 0;
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
        if (sketch == m_editedSketch) m_editedSketch.reset();
        return sketch;
    }
    return nullptr;
}

std::shared_ptr<Sketch> Document::findSketch(uint64_t sketchId) const {
    for (const auto& sketch : m_sketches) {
        if (sketch->id() == sketchId) return sketch;
    }
    return nullptr;
}

std::vector<draft::DraftDocument*> Document::drawings() {
    std::vector<draft::DraftDocument*> all{&m_draftDoc};
    for (const auto& sketch : m_sketches) all.push_back(&sketch->drawing());
    return all;
}

void Document::editSketch(std::shared_ptr<Sketch> sketch) {
    // Only a sketch of this document: another's would outlive nothing here.
    if (sketch && std::find(m_sketches.begin(), m_sketches.end(), sketch) == m_sketches.end()) {
        sketch.reset();
    }
    m_editedSketch = std::move(sketch);
}

UndoStack& Document::undoStack() {
    return *m_undoStack;
}
const UndoStack& Document::undoStack() const {
    return *m_undoStack;
}

}  // namespace hz::doc
