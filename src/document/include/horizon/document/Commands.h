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

/// Composite command that bundles multiple sub-commands into one undo step.
class CompositeCommand : public Command {
public:
    explicit CompositeCommand(const std::string& desc);

    void addCommand(std::unique_ptr<Command> cmd);
    void execute() override;
    void undo() override;
    std::string description() const override;

private:
    std::string m_description;
    std::vector<std::unique_ptr<Command>> m_commands;
};

/// Command to duplicate (clone) one or more entities with an offset.
class DuplicateEntityCommand : public Command {
public:
    DuplicateEntityCommand(draft::DraftDocument& doc,
                           const std::vector<uint64_t>& sourceIds,
                           const math::Vec2& offset);

    void execute() override;
    void undo() override;
    std::string description() const override;

    /// IDs of the cloned entities (valid after execute).
    std::vector<uint64_t> clonedIds() const;

private:
    draft::DraftDocument& m_doc;
    std::vector<uint64_t> m_sourceIds;
    math::Vec2 m_offset;
    std::vector<std::shared_ptr<draft::DraftEntity>> m_clones;
};

/// Command to mirror one or more entities across an axis, creating copies.
class MirrorEntityCommand : public Command {
public:
    MirrorEntityCommand(draft::DraftDocument& doc,
                        const std::vector<uint64_t>& entityIds,
                        const math::Vec2& axisP1,
                        const math::Vec2& axisP2);

    void execute() override;
    void undo() override;
    std::string description() const override;

    std::vector<uint64_t> mirroredIds() const;

private:
    draft::DraftDocument& m_doc;
    std::vector<uint64_t> m_sourceIds;
    math::Vec2 m_axisP1, m_axisP2;
    std::vector<std::shared_ptr<draft::DraftEntity>> m_mirroredEntities;
};

}  // namespace hz::doc
