#pragma once

#include "horizon/drafting/DraftDocument.h"
#include "horizon/drafting/DraftEntity.h"
#include <cstdint>
#include <memory>
#include <string>

namespace hz::doc {

class UndoStack;

/// Central document model for Horizon CAD.
/// Owns the DraftDocument (entity storage) and UndoStack.
class Document {
public:
    Document();
    ~Document();

    // --- Entity operations ---

    uint64_t addEntity(std::shared_ptr<draft::DraftEntity> entity);
    std::shared_ptr<draft::DraftEntity> removeEntity(uint64_t id);
    void clear();

    // --- Accessors ---

    const draft::DraftDocument& draftDocument() const { return m_draftDoc; }
    draft::DraftDocument& draftDocument() { return m_draftDoc; }

    UndoStack& undoStack();
    const UndoStack& undoStack() const;

    // --- Dirty tracking ---

    bool isDirty() const { return m_dirty; }
    void setDirty(bool dirty) { m_dirty = dirty; }

    // --- File path ---

    const std::string& filePath() const { return m_filePath; }
    void setFilePath(const std::string& path) { m_filePath = path; }

private:
    draft::DraftDocument m_draftDoc;
    std::unique_ptr<UndoStack> m_undoStack;
    bool m_dirty = false;
    std::string m_filePath;
};

}  // namespace hz::doc
