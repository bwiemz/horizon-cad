#pragma once

#include "horizon/document/UndoStack.h"
#include "horizon/drafting/DraftDocument.h"
#include "horizon/drafting/DraftEntity.h"
#include "horizon/math/Vec2.h"
#include <memory>
#include <vector>

namespace hz::doc {

/// Command to add a DraftEntity to a DraftDocument.
class AddEntityCommand : public Command {
public:
    AddEntityCommand(draft::DraftDocument& doc,
                     std::shared_ptr<draft::DraftEntity> entity);

    void execute() override;
    void undo() override;
    std::string description() const override;

private:
    draft::DraftDocument& m_doc;
    std::shared_ptr<draft::DraftEntity> m_entity;
    uint64_t m_entityId;
};

/// Command to remove a DraftEntity from a DraftDocument.
class RemoveEntityCommand : public Command {
public:
    RemoveEntityCommand(draft::DraftDocument& doc, uint64_t entityId);

    void execute() override;
    void undo() override;
    std::string description() const override;

private:
    draft::DraftDocument& m_doc;
    std::shared_ptr<draft::DraftEntity> m_entity;
    uint64_t m_entityId;
};

/// Command to move (translate) one or more DraftEntities.
class MoveEntityCommand : public Command {
public:
    MoveEntityCommand(draft::DraftDocument& doc,
                      const std::vector<uint64_t>& entityIds,
                      const math::Vec2& delta);

    void execute() override;
    void undo() override;
    std::string description() const override;

private:
    draft::DraftDocument& m_doc;
    std::vector<uint64_t> m_entityIds;
    math::Vec2 m_delta;
};

}  // namespace hz::doc
