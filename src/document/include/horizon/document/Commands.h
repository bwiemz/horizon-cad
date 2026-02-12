#pragma once

#include "horizon/document/UndoStack.h"
#include "horizon/drafting/BlockDefinition.h"
#include "horizon/drafting/DraftDocument.h"
#include "horizon/drafting/DraftEntity.h"
#include "horizon/drafting/Layer.h"
#include "horizon/math/Vec2.h"
#include <memory>
#include <string>
#include <utility>
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
    bool empty() const { return m_commands.empty(); }
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

/// Command to rotate-copy one or more entities around a center point.
class RotateEntityCommand : public Command {
public:
    RotateEntityCommand(draft::DraftDocument& doc,
                        const std::vector<uint64_t>& entityIds,
                        const math::Vec2& center,
                        double angle);

    void execute() override;
    void undo() override;
    std::string description() const override;

    std::vector<uint64_t> rotatedIds() const;

private:
    draft::DraftDocument& m_doc;
    std::vector<uint64_t> m_sourceIds;
    math::Vec2 m_center;
    double m_angle;
    std::vector<std::shared_ptr<draft::DraftEntity>> m_rotatedEntities;
};

/// Command to scale-copy one or more entities from a base point.
class ScaleEntityCommand : public Command {
public:
    ScaleEntityCommand(draft::DraftDocument& doc,
                       const std::vector<uint64_t>& entityIds,
                       const math::Vec2& basePoint,
                       double factor);

    void execute() override;
    void undo() override;
    std::string description() const override;

    std::vector<uint64_t> scaledIds() const;

private:
    draft::DraftDocument& m_doc;
    std::vector<uint64_t> m_sourceIds;
    math::Vec2 m_basePoint;
    double m_factor;
    std::vector<std::shared_ptr<draft::DraftEntity>> m_scaledEntities;
};

// ---------------------------------------------------------------------------
// Property commands
// ---------------------------------------------------------------------------

/// Command to change the layer of one or more entities.
class ChangeEntityLayerCommand : public Command {
public:
    ChangeEntityLayerCommand(draft::DraftDocument& doc,
                             const std::vector<uint64_t>& entityIds,
                             const std::string& newLayer);
    void execute() override;
    void undo() override;
    std::string description() const override;

private:
    draft::DraftDocument& m_doc;
    std::vector<uint64_t> m_entityIds;
    std::string m_newLayer;
    std::vector<std::pair<uint64_t, std::string>> m_oldLayers;
};

/// Command to change the color of one or more entities.
class ChangeEntityColorCommand : public Command {
public:
    ChangeEntityColorCommand(draft::DraftDocument& doc,
                             const std::vector<uint64_t>& entityIds,
                             uint32_t newColor);
    void execute() override;
    void undo() override;
    std::string description() const override;

private:
    draft::DraftDocument& m_doc;
    std::vector<uint64_t> m_entityIds;
    uint32_t m_newColor;
    std::vector<std::pair<uint64_t, uint32_t>> m_oldColors;
};

/// Command to change the line width of one or more entities.
class ChangeEntityLineWidthCommand : public Command {
public:
    ChangeEntityLineWidthCommand(draft::DraftDocument& doc,
                                 const std::vector<uint64_t>& entityIds,
                                 double newWidth);
    void execute() override;
    void undo() override;
    std::string description() const override;

private:
    draft::DraftDocument& m_doc;
    std::vector<uint64_t> m_entityIds;
    double m_newWidth;
    std::vector<std::pair<uint64_t, double>> m_oldWidths;
};

/// Command to change the text override of a dimension entity.
class ChangeTextOverrideCommand : public Command {
public:
    ChangeTextOverrideCommand(draft::DraftDocument& doc,
                              uint64_t entityId,
                              const std::string& newText);
    void execute() override;
    void undo() override;
    std::string description() const override;

private:
    draft::DraftDocument& m_doc;
    uint64_t m_entityId;
    std::string m_newText;
    std::string m_oldText;
};

// ---------------------------------------------------------------------------
// Layer commands
// ---------------------------------------------------------------------------

/// Command to add a new layer.
class AddLayerCommand : public Command {
public:
    AddLayerCommand(draft::LayerManager& mgr, const draft::LayerProperties& props);
    void execute() override;
    void undo() override;
    std::string description() const override;

private:
    draft::LayerManager& m_mgr;
    draft::LayerProperties m_props;
};

/// Command to remove a layer (moves entities on it to layer "0").
class RemoveLayerCommand : public Command {
public:
    RemoveLayerCommand(draft::LayerManager& mgr, draft::DraftDocument& doc,
                       const std::string& layerName);
    void execute() override;
    void undo() override;
    std::string description() const override;

private:
    draft::LayerManager& m_mgr;
    draft::DraftDocument& m_doc;
    std::string m_name;
    draft::LayerProperties m_savedProps;
    std::vector<std::pair<uint64_t, std::string>> m_movedEntities;
    bool m_wasCurrentLayer = false;
};

/// Command to modify layer properties.
class ModifyLayerCommand : public Command {
public:
    ModifyLayerCommand(draft::LayerManager& mgr, const std::string& layerName,
                       const draft::LayerProperties& newProps);
    void execute() override;
    void undo() override;
    std::string description() const override;

private:
    draft::LayerManager& m_mgr;
    std::string m_name;
    draft::LayerProperties m_newProps;
    draft::LayerProperties m_oldProps;
};

/// Command to set the current drawing layer.
class SetCurrentLayerCommand : public Command {
public:
    SetCurrentLayerCommand(draft::LayerManager& mgr, const std::string& layerName);
    void execute() override;
    void undo() override;
    std::string description() const override;

private:
    draft::LayerManager& m_mgr;
    std::string m_newLayer;
    std::string m_oldLayer;
};

// ---------------------------------------------------------------------------
// Block commands
// ---------------------------------------------------------------------------

/// Command to create a block definition from selected entities.
/// Removes originals and inserts a block reference at the centroid.
class CreateBlockCommand : public Command {
public:
    CreateBlockCommand(draft::DraftDocument& doc,
                       const std::string& blockName,
                       const std::vector<uint64_t>& entityIds);
    void execute() override;
    void undo() override;
    std::string description() const override;

    uint64_t blockRefId() const;

private:
    draft::DraftDocument& m_doc;
    std::string m_blockName;
    std::vector<uint64_t> m_entityIds;
    // Saved for undo.
    std::vector<std::shared_ptr<draft::DraftEntity>> m_savedEntities;
    std::shared_ptr<draft::DraftEntity> m_blockRef;
    std::shared_ptr<draft::BlockDefinition> m_definition;
};

/// Command to explode a block reference into individual entities.
class ExplodeBlockCommand : public Command {
public:
    ExplodeBlockCommand(draft::DraftDocument& doc, uint64_t blockRefId);
    void execute() override;
    void undo() override;
    std::string description() const override;

    std::vector<uint64_t> explodedIds() const;

private:
    draft::DraftDocument& m_doc;
    uint64_t m_blockRefId;
    std::shared_ptr<draft::DraftEntity> m_savedBlockRef;
    std::vector<std::shared_ptr<draft::DraftEntity>> m_explodedEntities;
};

/// Command to change a block reference's rotation.
class ChangeBlockRefRotationCommand : public Command {
public:
    ChangeBlockRefRotationCommand(draft::DraftDocument& doc,
                                  uint64_t entityId,
                                  double newRotation);
    void execute() override;
    void undo() override;
    std::string description() const override;

private:
    draft::DraftDocument& m_doc;
    uint64_t m_entityId;
    double m_newRotation;
    double m_oldRotation = 0.0;
};

/// Command to change a block reference's uniform scale.
class ChangeBlockRefScaleCommand : public Command {
public:
    ChangeBlockRefScaleCommand(draft::DraftDocument& doc,
                               uint64_t entityId,
                               double newScale);
    void execute() override;
    void undo() override;
    std::string description() const override;

private:
    draft::DraftDocument& m_doc;
    uint64_t m_entityId;
    double m_newScale;
    double m_oldScale = 1.0;
};

// ---------------------------------------------------------------------------
// Text entity commands
// ---------------------------------------------------------------------------

/// Command to change a text entity's content string.
class ChangeTextContentCommand : public Command {
public:
    ChangeTextContentCommand(draft::DraftDocument& doc,
                             uint64_t entityId,
                             const std::string& newText);
    void execute() override;
    void undo() override;
    std::string description() const override;

private:
    draft::DraftDocument& m_doc;
    uint64_t m_entityId;
    std::string m_newText;
    std::string m_oldText;
};

/// Command to change a text entity's height.
class ChangeTextHeightCommand : public Command {
public:
    ChangeTextHeightCommand(draft::DraftDocument& doc,
                            uint64_t entityId,
                            double newHeight);
    void execute() override;
    void undo() override;
    std::string description() const override;

private:
    draft::DraftDocument& m_doc;
    uint64_t m_entityId;
    double m_newHeight;
    double m_oldHeight = 2.5;
};

/// Command to change a text entity's rotation.
class ChangeTextRotationCommand : public Command {
public:
    ChangeTextRotationCommand(draft::DraftDocument& doc,
                              uint64_t entityId,
                              double newRotation);
    void execute() override;
    void undo() override;
    std::string description() const override;

private:
    draft::DraftDocument& m_doc;
    uint64_t m_entityId;
    double m_newRotation;
    double m_oldRotation = 0.0;
};

/// Command to change a text entity's alignment.
class ChangeTextAlignmentCommand : public Command {
public:
    ChangeTextAlignmentCommand(draft::DraftDocument& doc,
                               uint64_t entityId,
                               int newAlignment);
    void execute() override;
    void undo() override;
    std::string description() const override;

private:
    draft::DraftDocument& m_doc;
    uint64_t m_entityId;
    int m_newAlignment;
    int m_oldAlignment = 1;  // Center
};

// ---------------------------------------------------------------------------
// Spline entity commands
// ---------------------------------------------------------------------------

/// Command to toggle a spline's closed/open state.
class ChangeSplineClosedCommand : public Command {
public:
    ChangeSplineClosedCommand(draft::DraftDocument& doc,
                              uint64_t entityId,
                              bool newClosed);
    void execute() override;
    void undo() override;
    std::string description() const override;

private:
    draft::DraftDocument& m_doc;
    uint64_t m_entityId;
    bool m_newClosed;
    bool m_oldClosed = false;
};

// ---------------------------------------------------------------------------
// Hatch entity commands
// ---------------------------------------------------------------------------

/// Command to change a hatch entity's pattern type.
class ChangeHatchPatternCommand : public Command {
public:
    ChangeHatchPatternCommand(draft::DraftDocument& doc,
                              uint64_t entityId,
                              int newPattern);
    void execute() override;
    void undo() override;
    std::string description() const override;

private:
    draft::DraftDocument& m_doc;
    uint64_t m_entityId;
    int m_newPattern;
    int m_oldPattern = 1;
};

/// Command to change a hatch entity's angle.
class ChangeHatchAngleCommand : public Command {
public:
    ChangeHatchAngleCommand(draft::DraftDocument& doc,
                            uint64_t entityId,
                            double newAngle);
    void execute() override;
    void undo() override;
    std::string description() const override;

private:
    draft::DraftDocument& m_doc;
    uint64_t m_entityId;
    double m_newAngle;
    double m_oldAngle = 0.0;
};

/// Command to change a hatch entity's spacing.
class ChangeHatchSpacingCommand : public Command {
public:
    ChangeHatchSpacingCommand(draft::DraftDocument& doc,
                              uint64_t entityId,
                              double newSpacing);
    void execute() override;
    void undo() override;
    std::string description() const override;

private:
    draft::DraftDocument& m_doc;
    uint64_t m_entityId;
    double m_newSpacing;
    double m_oldSpacing = 1.0;
};

// ---------------------------------------------------------------------------
// Ellipse entity commands
// ---------------------------------------------------------------------------

/// Command to change an ellipse's semi-major radius.
class ChangeEllipseSemiMajorCommand : public Command {
public:
    ChangeEllipseSemiMajorCommand(draft::DraftDocument& doc,
                                  uint64_t entityId, double newValue);
    void execute() override;
    void undo() override;
    std::string description() const override;

private:
    draft::DraftDocument& m_doc;
    uint64_t m_entityId;
    double m_newValue;
    double m_oldValue = 1.0;
};

/// Command to change an ellipse's semi-minor radius.
class ChangeEllipseSemiMinorCommand : public Command {
public:
    ChangeEllipseSemiMinorCommand(draft::DraftDocument& doc,
                                  uint64_t entityId, double newValue);
    void execute() override;
    void undo() override;
    std::string description() const override;

private:
    draft::DraftDocument& m_doc;
    uint64_t m_entityId;
    double m_newValue;
    double m_oldValue = 1.0;
};

/// Command to change an ellipse's rotation.
class ChangeEllipseRotationCommand : public Command {
public:
    ChangeEllipseRotationCommand(draft::DraftDocument& doc,
                                 uint64_t entityId, double newRotation);
    void execute() override;
    void undo() override;
    std::string description() const override;

private:
    draft::DraftDocument& m_doc;
    uint64_t m_entityId;
    double m_newRotation;
    double m_oldRotation = 0.0;
};

// ---------------------------------------------------------------------------
// Grip editing command
// ---------------------------------------------------------------------------

/// Snapshot-based command for grip editing.
/// Stores the entity state before and after a grip move.
/// The caller applies the grip move BEFORE pushing this command;
/// execute() is a no-op on first call.  Undo/redo swaps the snapshots.
class GripMoveCommand : public Command {
public:
    /// \param doc        The document
    /// \param entityId   ID of the entity being grip-edited
    /// \param beforeState  Clone of the entity BEFORE the grip move
    /// \param afterState   Clone of the entity AFTER the grip move
    GripMoveCommand(draft::DraftDocument& doc, uint64_t entityId,
                    std::shared_ptr<draft::DraftEntity> beforeState,
                    std::shared_ptr<draft::DraftEntity> afterState);
    void execute() override;
    void undo() override;
    std::string description() const override;

private:
    void applyState(const draft::DraftEntity& state);

    draft::DraftDocument& m_doc;
    uint64_t m_entityId;
    std::shared_ptr<draft::DraftEntity> m_beforeState;
    std::shared_ptr<draft::DraftEntity> m_afterState;
    bool m_firstExec = true;
};

}  // namespace hz::doc
