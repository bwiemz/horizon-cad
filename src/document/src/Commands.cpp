#include "horizon/document/Commands.h"
#include "horizon/drafting/DraftBlockRef.h"
#include "horizon/drafting/DraftDimension.h"
#include "horizon/drafting/DraftText.h"
#include "horizon/drafting/DraftSpline.h"
#include "horizon/drafting/DraftHatch.h"
#include "horizon/drafting/DraftEllipse.h"
#include <unordered_map>

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

// --- CompositeCommand ---

CompositeCommand::CompositeCommand(const std::string& desc)
    : m_description(desc) {}

void CompositeCommand::addCommand(std::unique_ptr<Command> cmd) {
    m_commands.push_back(std::move(cmd));
}

void CompositeCommand::execute() {
    for (auto& cmd : m_commands) {
        cmd->execute();
    }
}

void CompositeCommand::undo() {
    for (auto it = m_commands.rbegin(); it != m_commands.rend(); ++it) {
        (*it)->undo();
    }
}

std::string CompositeCommand::description() const {
    return m_description;
}

// --- DuplicateEntityCommand ---

DuplicateEntityCommand::DuplicateEntityCommand(draft::DraftDocument& doc,
                                               const std::vector<uint64_t>& sourceIds,
                                               const math::Vec2& offset)
    : m_doc(doc), m_sourceIds(sourceIds), m_offset(offset) {}

void DuplicateEntityCommand::execute() {
    if (m_clones.empty()) {
        for (uint64_t id : m_sourceIds) {
            for (const auto& e : m_doc.entities()) {
                if (e->id() == id) {
                    auto clone = e->clone();
                    clone->translate(m_offset);
                    m_clones.push_back(clone);
                    break;
                }
            }
        }
        remapCloneGroupIds(m_doc, m_clones);
    }
    for (const auto& clone : m_clones) {
        m_doc.addEntity(clone);
    }
}

void DuplicateEntityCommand::undo() {
    for (const auto& clone : m_clones) {
        m_doc.removeEntity(clone->id());
    }
}

std::string DuplicateEntityCommand::description() const {
    return "Duplicate";
}

std::vector<uint64_t> DuplicateEntityCommand::clonedIds() const {
    std::vector<uint64_t> ids;
    ids.reserve(m_clones.size());
    for (const auto& clone : m_clones) {
        ids.push_back(clone->id());
    }
    return ids;
}

// --- MirrorEntityCommand ---

MirrorEntityCommand::MirrorEntityCommand(draft::DraftDocument& doc,
                                         const std::vector<uint64_t>& entityIds,
                                         const math::Vec2& axisP1,
                                         const math::Vec2& axisP2)
    : m_doc(doc), m_sourceIds(entityIds), m_axisP1(axisP1), m_axisP2(axisP2) {}

void MirrorEntityCommand::execute() {
    if (m_mirroredEntities.empty()) {
        for (uint64_t id : m_sourceIds) {
            for (const auto& e : m_doc.entities()) {
                if (e->id() == id) {
                    auto mirrored = e->clone();
                    mirrored->mirror(m_axisP1, m_axisP2);
                    m_mirroredEntities.push_back(mirrored);
                    break;
                }
            }
        }
        remapCloneGroupIds(m_doc, m_mirroredEntities);
    }
    for (const auto& e : m_mirroredEntities) {
        m_doc.addEntity(e);
    }
}

void MirrorEntityCommand::undo() {
    for (const auto& e : m_mirroredEntities) {
        m_doc.removeEntity(e->id());
    }
}

std::string MirrorEntityCommand::description() const {
    return "Mirror";
}

std::vector<uint64_t> MirrorEntityCommand::mirroredIds() const {
    std::vector<uint64_t> ids;
    ids.reserve(m_mirroredEntities.size());
    for (const auto& e : m_mirroredEntities) {
        ids.push_back(e->id());
    }
    return ids;
}

// --- RotateEntityCommand ---

RotateEntityCommand::RotateEntityCommand(draft::DraftDocument& doc,
                                         const std::vector<uint64_t>& entityIds,
                                         const math::Vec2& center,
                                         double angle)
    : m_doc(doc), m_sourceIds(entityIds), m_center(center), m_angle(angle) {}

void RotateEntityCommand::execute() {
    if (m_rotatedEntities.empty()) {
        for (uint64_t id : m_sourceIds) {
            for (const auto& e : m_doc.entities()) {
                if (e->id() == id) {
                    auto rotated = e->clone();
                    rotated->rotate(m_center, m_angle);
                    m_rotatedEntities.push_back(rotated);
                    break;
                }
            }
        }
        remapCloneGroupIds(m_doc, m_rotatedEntities);
    }
    for (const auto& e : m_rotatedEntities) {
        m_doc.addEntity(e);
    }
}

void RotateEntityCommand::undo() {
    for (const auto& e : m_rotatedEntities) {
        m_doc.removeEntity(e->id());
    }
}

std::string RotateEntityCommand::description() const {
    return "Rotate";
}

std::vector<uint64_t> RotateEntityCommand::rotatedIds() const {
    std::vector<uint64_t> ids;
    ids.reserve(m_rotatedEntities.size());
    for (const auto& e : m_rotatedEntities) {
        ids.push_back(e->id());
    }
    return ids;
}

// --- ScaleEntityCommand ---

ScaleEntityCommand::ScaleEntityCommand(draft::DraftDocument& doc,
                                       const std::vector<uint64_t>& entityIds,
                                       const math::Vec2& basePoint,
                                       double factor)
    : m_doc(doc), m_sourceIds(entityIds), m_basePoint(basePoint), m_factor(factor) {}

void ScaleEntityCommand::execute() {
    if (m_scaledEntities.empty()) {
        for (uint64_t id : m_sourceIds) {
            for (const auto& e : m_doc.entities()) {
                if (e->id() == id) {
                    auto scaled = e->clone();
                    scaled->scale(m_basePoint, m_factor);
                    m_scaledEntities.push_back(scaled);
                    break;
                }
            }
        }
        remapCloneGroupIds(m_doc, m_scaledEntities);
    }
    for (const auto& e : m_scaledEntities) {
        m_doc.addEntity(e);
    }
}

void ScaleEntityCommand::undo() {
    for (const auto& e : m_scaledEntities) {
        m_doc.removeEntity(e->id());
    }
}

std::string ScaleEntityCommand::description() const {
    return "Scale";
}

std::vector<uint64_t> ScaleEntityCommand::scaledIds() const {
    std::vector<uint64_t> ids;
    ids.reserve(m_scaledEntities.size());
    for (const auto& e : m_scaledEntities) {
        ids.push_back(e->id());
    }
    return ids;
}

// --- ChangeEntityLayerCommand ---

ChangeEntityLayerCommand::ChangeEntityLayerCommand(draft::DraftDocument& doc,
                                                   const std::vector<uint64_t>& entityIds,
                                                   const std::string& newLayer)
    : m_doc(doc), m_entityIds(entityIds), m_newLayer(newLayer) {}

void ChangeEntityLayerCommand::execute() {
    m_oldLayers.clear();
    for (uint64_t id : m_entityIds) {
        for (const auto& e : m_doc.entities()) {
            if (e->id() == id) {
                m_oldLayers.emplace_back(id, e->layer());
                e->setLayer(m_newLayer);
                break;
            }
        }
    }
}

void ChangeEntityLayerCommand::undo() {
    for (const auto& [id, oldLayer] : m_oldLayers) {
        for (const auto& e : m_doc.entities()) {
            if (e->id() == id) {
                e->setLayer(oldLayer);
                break;
            }
        }
    }
}

std::string ChangeEntityLayerCommand::description() const {
    return "Change Layer";
}

// --- ChangeEntityColorCommand ---

ChangeEntityColorCommand::ChangeEntityColorCommand(draft::DraftDocument& doc,
                                                   const std::vector<uint64_t>& entityIds,
                                                   uint32_t newColor)
    : m_doc(doc), m_entityIds(entityIds), m_newColor(newColor) {}

void ChangeEntityColorCommand::execute() {
    m_oldColors.clear();
    for (uint64_t id : m_entityIds) {
        for (const auto& e : m_doc.entities()) {
            if (e->id() == id) {
                m_oldColors.emplace_back(id, e->color());
                e->setColor(m_newColor);
                break;
            }
        }
    }
}

void ChangeEntityColorCommand::undo() {
    for (const auto& [id, oldColor] : m_oldColors) {
        for (const auto& e : m_doc.entities()) {
            if (e->id() == id) {
                e->setColor(oldColor);
                break;
            }
        }
    }
}

std::string ChangeEntityColorCommand::description() const {
    return "Change Color";
}

// --- ChangeEntityLineWidthCommand ---

ChangeEntityLineWidthCommand::ChangeEntityLineWidthCommand(draft::DraftDocument& doc,
                                                           const std::vector<uint64_t>& entityIds,
                                                           double newWidth)
    : m_doc(doc), m_entityIds(entityIds), m_newWidth(newWidth) {}

void ChangeEntityLineWidthCommand::execute() {
    m_oldWidths.clear();
    for (uint64_t id : m_entityIds) {
        for (const auto& e : m_doc.entities()) {
            if (e->id() == id) {
                m_oldWidths.emplace_back(id, e->lineWidth());
                e->setLineWidth(m_newWidth);
                break;
            }
        }
    }
}

void ChangeEntityLineWidthCommand::undo() {
    for (const auto& [id, oldWidth] : m_oldWidths) {
        for (const auto& e : m_doc.entities()) {
            if (e->id() == id) {
                e->setLineWidth(oldWidth);
                break;
            }
        }
    }
}

std::string ChangeEntityLineWidthCommand::description() const {
    return "Change Line Width";
}

// --- ChangeEntityLineTypeCommand ---

ChangeEntityLineTypeCommand::ChangeEntityLineTypeCommand(draft::DraftDocument& doc,
                                                         const std::vector<uint64_t>& entityIds,
                                                         int newLineType)
    : m_doc(doc), m_entityIds(entityIds), m_newLineType(newLineType) {}

void ChangeEntityLineTypeCommand::execute() {
    m_oldLineTypes.clear();
    for (uint64_t id : m_entityIds) {
        for (const auto& e : m_doc.entities()) {
            if (e->id() == id) {
                m_oldLineTypes.emplace_back(id, e->lineType());
                e->setLineType(m_newLineType);
                break;
            }
        }
    }
}

void ChangeEntityLineTypeCommand::undo() {
    for (const auto& [id, oldLt] : m_oldLineTypes) {
        for (const auto& e : m_doc.entities()) {
            if (e->id() == id) {
                e->setLineType(oldLt);
                break;
            }
        }
    }
}

std::string ChangeEntityLineTypeCommand::description() const {
    return "Change Line Type";
}

// --- ChangeTextOverrideCommand ---

ChangeTextOverrideCommand::ChangeTextOverrideCommand(draft::DraftDocument& doc,
                                                     uint64_t entityId,
                                                     const std::string& newText)
    : m_doc(doc), m_entityId(entityId), m_newText(newText) {}

void ChangeTextOverrideCommand::execute() {
    for (const auto& e : m_doc.entities()) {
        if (e->id() == m_entityId) {
            if (auto* dim = dynamic_cast<draft::DraftDimension*>(e.get())) {
                m_oldText = dim->textOverride();
                dim->setTextOverride(m_newText);
            }
            break;
        }
    }
}

void ChangeTextOverrideCommand::undo() {
    for (const auto& e : m_doc.entities()) {
        if (e->id() == m_entityId) {
            if (auto* dim = dynamic_cast<draft::DraftDimension*>(e.get())) {
                dim->setTextOverride(m_oldText);
            }
            break;
        }
    }
}

std::string ChangeTextOverrideCommand::description() const {
    return "Change Text Override";
}

// --- AddLayerCommand ---

AddLayerCommand::AddLayerCommand(draft::LayerManager& mgr,
                                 const draft::LayerProperties& props)
    : m_mgr(mgr), m_props(props) {}

void AddLayerCommand::execute() {
    m_mgr.addLayer(m_props);
}

void AddLayerCommand::undo() {
    m_mgr.removeLayer(m_props.name);
}

std::string AddLayerCommand::description() const {
    return "Add Layer";
}

// --- RemoveLayerCommand ---

RemoveLayerCommand::RemoveLayerCommand(draft::LayerManager& mgr,
                                       draft::DraftDocument& doc,
                                       const std::string& layerName)
    : m_mgr(mgr), m_doc(doc), m_name(layerName) {}

void RemoveLayerCommand::execute() {
    if (m_name == "0") return;  // Never remove the default layer.

    // Save layer properties.
    const auto* lp = m_mgr.getLayer(m_name);
    if (lp) m_savedProps = *lp;

    // Move entities on this layer to "0".
    m_movedEntities.clear();
    for (const auto& e : m_doc.entities()) {
        if (e->layer() == m_name) {
            m_movedEntities.emplace_back(e->id(), e->layer());
            e->setLayer("0");
        }
    }

    // If this is the current layer, switch to "0" first.
    m_wasCurrentLayer = (m_mgr.currentLayer() == m_name);
    if (m_wasCurrentLayer) {
        m_mgr.setCurrentLayer("0");
    }
    m_mgr.removeLayer(m_name);
}

void RemoveLayerCommand::undo() {
    if (m_name == "0") return;

    m_mgr.addLayer(m_savedProps);

    // Restore current layer if it was current before removal.
    if (m_wasCurrentLayer) {
        m_mgr.setCurrentLayer(m_name);
    }

    // Restore entity layers.
    for (const auto& [id, oldLayer] : m_movedEntities) {
        for (const auto& e : m_doc.entities()) {
            if (e->id() == id) {
                e->setLayer(oldLayer);
                break;
            }
        }
    }
}

std::string RemoveLayerCommand::description() const {
    return "Remove Layer";
}

// --- ModifyLayerCommand ---

ModifyLayerCommand::ModifyLayerCommand(draft::LayerManager& mgr,
                                       const std::string& layerName,
                                       const draft::LayerProperties& newProps)
    : m_mgr(mgr), m_name(layerName), m_newProps(newProps) {}

void ModifyLayerCommand::execute() {
    auto* lp = m_mgr.getLayer(m_name);
    if (lp) {
        m_oldProps = *lp;
        *lp = m_newProps;
    }
}

void ModifyLayerCommand::undo() {
    auto* lp = m_mgr.getLayer(m_name);
    if (lp) {
        *lp = m_oldProps;
    }
}

std::string ModifyLayerCommand::description() const {
    return "Modify Layer";
}

// --- SetCurrentLayerCommand ---

SetCurrentLayerCommand::SetCurrentLayerCommand(draft::LayerManager& mgr,
                                               const std::string& layerName)
    : m_mgr(mgr), m_newLayer(layerName) {}

void SetCurrentLayerCommand::execute() {
    m_oldLayer = m_mgr.currentLayer();
    m_mgr.setCurrentLayer(m_newLayer);
}

void SetCurrentLayerCommand::undo() {
    m_mgr.setCurrentLayer(m_oldLayer);
}

std::string SetCurrentLayerCommand::description() const {
    return "Set Current Layer";
}

// --- CreateBlockCommand ---

CreateBlockCommand::CreateBlockCommand(draft::DraftDocument& doc,
                                       const std::string& blockName,
                                       const std::vector<uint64_t>& entityIds)
    : m_doc(doc), m_blockName(blockName), m_entityIds(entityIds) {}

void CreateBlockCommand::execute() {
    // Gather the source entities.
    m_savedEntities.clear();
    math::Vec2 centroid;
    for (uint64_t id : m_entityIds) {
        for (const auto& e : m_doc.entities()) {
            if (e->id() == id) {
                m_savedEntities.push_back(e);
                auto bb = e->boundingBox();
                if (bb.isValid()) {
                    auto c = bb.center();
                    centroid += math::Vec2(c.x, c.y);
                }
                break;
            }
        }
    }
    if (!m_savedEntities.empty()) {
        centroid = centroid * (1.0 / static_cast<double>(m_savedEntities.size()));
    }

    // Create block definition with cloned entities.
    m_definition = std::make_shared<draft::BlockDefinition>();
    m_definition->name = m_blockName;
    m_definition->basePoint = centroid;
    for (const auto& e : m_savedEntities) {
        m_definition->entities.push_back(e->clone());
    }
    m_doc.blockTable().addBlock(m_definition);

    // Remove originals from document.
    for (uint64_t id : m_entityIds) {
        m_doc.removeEntity(id);
    }

    // Insert a block reference at the centroid.
    m_blockRef = std::make_shared<draft::DraftBlockRef>(m_definition, centroid);
    m_doc.addEntity(m_blockRef);
}

void CreateBlockCommand::undo() {
    // Remove the block reference.
    if (m_blockRef) {
        m_doc.removeEntity(m_blockRef->id());
    }
    // Remove the block definition.
    m_doc.blockTable().removeBlock(m_blockName);
    // Restore original entities.
    for (const auto& e : m_savedEntities) {
        m_doc.addEntity(e);
    }
}

std::string CreateBlockCommand::description() const {
    return "Create Block";
}

uint64_t CreateBlockCommand::blockRefId() const {
    return m_blockRef ? m_blockRef->id() : 0;
}

// --- ExplodeBlockCommand ---

ExplodeBlockCommand::ExplodeBlockCommand(draft::DraftDocument& doc, uint64_t blockRefId)
    : m_doc(doc), m_blockRefId(blockRefId) {}

void ExplodeBlockCommand::execute() {
    // Find the block reference.
    for (const auto& e : m_doc.entities()) {
        if (e->id() == m_blockRefId) {
            m_savedBlockRef = e;
            break;
        }
    }
    auto* ref = dynamic_cast<draft::DraftBlockRef*>(m_savedBlockRef.get());
    if (!ref) return;

    // Create transformed copies of all definition entities.
    m_explodedEntities.clear();
    for (const auto& defEnt : ref->definition()->entities) {
        auto worldEnt = defEnt->clone();
        // Apply the block ref transform: scale, rotate, translate.
        worldEnt->scale(ref->definition()->basePoint, ref->uniformScale());
        worldEnt->rotate(ref->definition()->basePoint, ref->rotation());
        worldEnt->translate(ref->insertPos() - ref->definition()->basePoint);
        // Inherit layer from block ref if entity is on default layer.
        if (worldEnt->layer().empty() || worldEnt->layer() == "0") {
            worldEnt->setLayer(ref->layer());
        }
        // ByBlock color: if entity color is 0, inherit from block ref.
        if (worldEnt->color() == 0x00000000) {
            worldEnt->setColor(ref->color());
        }
        // ByBlock lineWidth: if entity lineWidth is 0, inherit from block ref.
        if (worldEnt->lineWidth() == 0.0) {
            worldEnt->setLineWidth(ref->lineWidth());
        }
        m_explodedEntities.push_back(worldEnt);
        m_doc.addEntity(worldEnt);
    }

    // Remove the block reference.
    m_doc.removeEntity(m_blockRefId);
}

void ExplodeBlockCommand::undo() {
    // Remove exploded entities.
    for (const auto& e : m_explodedEntities) {
        m_doc.removeEntity(e->id());
    }
    m_explodedEntities.clear();
    // Restore the block reference.
    if (m_savedBlockRef) {
        m_doc.addEntity(m_savedBlockRef);
    }
}

std::string ExplodeBlockCommand::description() const {
    return "Explode Block";
}

std::vector<uint64_t> ExplodeBlockCommand::explodedIds() const {
    std::vector<uint64_t> ids;
    ids.reserve(m_explodedEntities.size());
    for (const auto& e : m_explodedEntities) {
        ids.push_back(e->id());
    }
    return ids;
}

// --- ChangeBlockRefRotationCommand ---

ChangeBlockRefRotationCommand::ChangeBlockRefRotationCommand(draft::DraftDocument& doc,
                                                             uint64_t entityId,
                                                             double newRotation)
    : m_doc(doc), m_entityId(entityId), m_newRotation(newRotation) {}

void ChangeBlockRefRotationCommand::execute() {
    for (const auto& e : m_doc.entities()) {
        if (e->id() == m_entityId) {
            if (auto* ref = dynamic_cast<draft::DraftBlockRef*>(e.get())) {
                m_oldRotation = ref->rotation();
                ref->setRotation(m_newRotation);
            }
            break;
        }
    }
}

void ChangeBlockRefRotationCommand::undo() {
    for (const auto& e : m_doc.entities()) {
        if (e->id() == m_entityId) {
            if (auto* ref = dynamic_cast<draft::DraftBlockRef*>(e.get())) {
                ref->setRotation(m_oldRotation);
            }
            break;
        }
    }
}

std::string ChangeBlockRefRotationCommand::description() const {
    return "Change Block Rotation";
}

// --- ChangeBlockRefScaleCommand ---

ChangeBlockRefScaleCommand::ChangeBlockRefScaleCommand(draft::DraftDocument& doc,
                                                       uint64_t entityId,
                                                       double newScale)
    : m_doc(doc), m_entityId(entityId), m_newScale(newScale) {}

void ChangeBlockRefScaleCommand::execute() {
    for (const auto& e : m_doc.entities()) {
        if (e->id() == m_entityId) {
            if (auto* ref = dynamic_cast<draft::DraftBlockRef*>(e.get())) {
                m_oldScale = ref->uniformScale();
                ref->setUniformScale(m_newScale);
            }
            break;
        }
    }
}

void ChangeBlockRefScaleCommand::undo() {
    for (const auto& e : m_doc.entities()) {
        if (e->id() == m_entityId) {
            if (auto* ref = dynamic_cast<draft::DraftBlockRef*>(e.get())) {
                ref->setUniformScale(m_oldScale);
            }
            break;
        }
    }
}

std::string ChangeBlockRefScaleCommand::description() const {
    return "Change Block Scale";
}

// --- ChangeTextContentCommand ---

ChangeTextContentCommand::ChangeTextContentCommand(draft::DraftDocument& doc,
                                                   uint64_t entityId,
                                                   const std::string& newText)
    : m_doc(doc), m_entityId(entityId), m_newText(newText) {}

void ChangeTextContentCommand::execute() {
    for (const auto& e : m_doc.entities()) {
        if (e->id() == m_entityId) {
            if (auto* txt = dynamic_cast<draft::DraftText*>(e.get())) {
                m_oldText = txt->text();
                txt->setText(m_newText);
            }
            break;
        }
    }
}

void ChangeTextContentCommand::undo() {
    for (const auto& e : m_doc.entities()) {
        if (e->id() == m_entityId) {
            if (auto* txt = dynamic_cast<draft::DraftText*>(e.get())) {
                txt->setText(m_oldText);
            }
            break;
        }
    }
}

std::string ChangeTextContentCommand::description() const {
    return "Change Text Content";
}

// --- ChangeTextHeightCommand ---

ChangeTextHeightCommand::ChangeTextHeightCommand(draft::DraftDocument& doc,
                                                 uint64_t entityId,
                                                 double newHeight)
    : m_doc(doc), m_entityId(entityId), m_newHeight(newHeight) {}

void ChangeTextHeightCommand::execute() {
    for (const auto& e : m_doc.entities()) {
        if (e->id() == m_entityId) {
            if (auto* txt = dynamic_cast<draft::DraftText*>(e.get())) {
                m_oldHeight = txt->textHeight();
                txt->setTextHeight(m_newHeight);
            }
            break;
        }
    }
}

void ChangeTextHeightCommand::undo() {
    for (const auto& e : m_doc.entities()) {
        if (e->id() == m_entityId) {
            if (auto* txt = dynamic_cast<draft::DraftText*>(e.get())) {
                txt->setTextHeight(m_oldHeight);
            }
            break;
        }
    }
}

std::string ChangeTextHeightCommand::description() const {
    return "Change Text Height";
}

// --- ChangeTextRotationCommand ---

ChangeTextRotationCommand::ChangeTextRotationCommand(draft::DraftDocument& doc,
                                                     uint64_t entityId,
                                                     double newRotation)
    : m_doc(doc), m_entityId(entityId), m_newRotation(newRotation) {}

void ChangeTextRotationCommand::execute() {
    for (const auto& e : m_doc.entities()) {
        if (e->id() == m_entityId) {
            if (auto* txt = dynamic_cast<draft::DraftText*>(e.get())) {
                m_oldRotation = txt->rotation();
                txt->setRotation(m_newRotation);
            }
            break;
        }
    }
}

void ChangeTextRotationCommand::undo() {
    for (const auto& e : m_doc.entities()) {
        if (e->id() == m_entityId) {
            if (auto* txt = dynamic_cast<draft::DraftText*>(e.get())) {
                txt->setRotation(m_oldRotation);
            }
            break;
        }
    }
}

std::string ChangeTextRotationCommand::description() const {
    return "Change Text Rotation";
}

// --- ChangeTextAlignmentCommand ---

ChangeTextAlignmentCommand::ChangeTextAlignmentCommand(draft::DraftDocument& doc,
                                                       uint64_t entityId,
                                                       int newAlignment)
    : m_doc(doc), m_entityId(entityId), m_newAlignment(newAlignment) {}

void ChangeTextAlignmentCommand::execute() {
    for (const auto& e : m_doc.entities()) {
        if (e->id() == m_entityId) {
            if (auto* txt = dynamic_cast<draft::DraftText*>(e.get())) {
                m_oldAlignment = static_cast<int>(txt->alignment());
                txt->setAlignment(static_cast<draft::TextAlignment>(m_newAlignment));
            }
            break;
        }
    }
}

void ChangeTextAlignmentCommand::undo() {
    for (const auto& e : m_doc.entities()) {
        if (e->id() == m_entityId) {
            if (auto* txt = dynamic_cast<draft::DraftText*>(e.get())) {
                txt->setAlignment(static_cast<draft::TextAlignment>(m_oldAlignment));
            }
            break;
        }
    }
}

std::string ChangeTextAlignmentCommand::description() const {
    return "Change Text Alignment";
}

// ---------------------------------------------------------------------------
// ChangeSplineClosedCommand
// ---------------------------------------------------------------------------

ChangeSplineClosedCommand::ChangeSplineClosedCommand(draft::DraftDocument& doc,
                                                     uint64_t entityId,
                                                     bool newClosed)
    : m_doc(doc), m_entityId(entityId), m_newClosed(newClosed) {}

void ChangeSplineClosedCommand::execute() {
    for (auto& e : m_doc.entities()) {
        if (e->id() == m_entityId) {
            if (auto* sp = dynamic_cast<draft::DraftSpline*>(e.get())) {
                m_oldClosed = sp->closed();
                sp->setClosed(m_newClosed);
            }
            break;
        }
    }
}

void ChangeSplineClosedCommand::undo() {
    for (auto& e : m_doc.entities()) {
        if (e->id() == m_entityId) {
            if (auto* sp = dynamic_cast<draft::DraftSpline*>(e.get())) {
                sp->setClosed(m_oldClosed);
            }
            break;
        }
    }
}

std::string ChangeSplineClosedCommand::description() const {
    return "Change Spline Closed";
}

// ---------------------------------------------------------------------------
// ChangeHatchPatternCommand
// ---------------------------------------------------------------------------

ChangeHatchPatternCommand::ChangeHatchPatternCommand(draft::DraftDocument& doc,
                                                     uint64_t entityId,
                                                     int newPattern)
    : m_doc(doc), m_entityId(entityId), m_newPattern(newPattern) {}

void ChangeHatchPatternCommand::execute() {
    for (auto& e : m_doc.entities()) {
        if (e->id() == m_entityId) {
            if (auto* h = dynamic_cast<draft::DraftHatch*>(e.get())) {
                m_oldPattern = static_cast<int>(h->pattern());
                h->setPattern(static_cast<draft::HatchPattern>(m_newPattern));
            }
            break;
        }
    }
}

void ChangeHatchPatternCommand::undo() {
    for (auto& e : m_doc.entities()) {
        if (e->id() == m_entityId) {
            if (auto* h = dynamic_cast<draft::DraftHatch*>(e.get())) {
                h->setPattern(static_cast<draft::HatchPattern>(m_oldPattern));
            }
            break;
        }
    }
}

std::string ChangeHatchPatternCommand::description() const {
    return "Change Hatch Pattern";
}

// ---------------------------------------------------------------------------
// ChangeHatchAngleCommand
// ---------------------------------------------------------------------------

ChangeHatchAngleCommand::ChangeHatchAngleCommand(draft::DraftDocument& doc,
                                                 uint64_t entityId,
                                                 double newAngle)
    : m_doc(doc), m_entityId(entityId), m_newAngle(newAngle) {}

void ChangeHatchAngleCommand::execute() {
    for (auto& e : m_doc.entities()) {
        if (e->id() == m_entityId) {
            if (auto* h = dynamic_cast<draft::DraftHatch*>(e.get())) {
                m_oldAngle = h->angle();
                h->setAngle(m_newAngle);
            }
            break;
        }
    }
}

void ChangeHatchAngleCommand::undo() {
    for (auto& e : m_doc.entities()) {
        if (e->id() == m_entityId) {
            if (auto* h = dynamic_cast<draft::DraftHatch*>(e.get())) {
                h->setAngle(m_oldAngle);
            }
            break;
        }
    }
}

std::string ChangeHatchAngleCommand::description() const {
    return "Change Hatch Angle";
}

// ---------------------------------------------------------------------------
// ChangeHatchSpacingCommand
// ---------------------------------------------------------------------------

ChangeHatchSpacingCommand::ChangeHatchSpacingCommand(draft::DraftDocument& doc,
                                                     uint64_t entityId,
                                                     double newSpacing)
    : m_doc(doc), m_entityId(entityId), m_newSpacing(newSpacing) {}

void ChangeHatchSpacingCommand::execute() {
    for (auto& e : m_doc.entities()) {
        if (e->id() == m_entityId) {
            if (auto* h = dynamic_cast<draft::DraftHatch*>(e.get())) {
                m_oldSpacing = h->spacing();
                h->setSpacing(m_newSpacing);
            }
            break;
        }
    }
}

void ChangeHatchSpacingCommand::undo() {
    for (auto& e : m_doc.entities()) {
        if (e->id() == m_entityId) {
            if (auto* h = dynamic_cast<draft::DraftHatch*>(e.get())) {
                h->setSpacing(m_oldSpacing);
            }
            break;
        }
    }
}

std::string ChangeHatchSpacingCommand::description() const {
    return "Change Hatch Spacing";
}

// ---------------------------------------------------------------------------
// ChangeEllipseSemiMajorCommand
// ---------------------------------------------------------------------------

ChangeEllipseSemiMajorCommand::ChangeEllipseSemiMajorCommand(
    draft::DraftDocument& doc, uint64_t entityId, double newValue)
    : m_doc(doc), m_entityId(entityId), m_newValue(newValue) {}

void ChangeEllipseSemiMajorCommand::execute() {
    for (auto& e : m_doc.entities()) {
        if (e->id() == m_entityId) {
            if (auto* el = dynamic_cast<draft::DraftEllipse*>(e.get())) {
                m_oldValue = el->semiMajor();
                el->setSemiMajor(m_newValue);
            }
            break;
        }
    }
}

void ChangeEllipseSemiMajorCommand::undo() {
    for (auto& e : m_doc.entities()) {
        if (e->id() == m_entityId) {
            if (auto* el = dynamic_cast<draft::DraftEllipse*>(e.get())) {
                el->setSemiMajor(m_oldValue);
            }
            break;
        }
    }
}

std::string ChangeEllipseSemiMajorCommand::description() const {
    return "Change Ellipse Semi-Major";
}

// ---------------------------------------------------------------------------
// ChangeEllipseSemiMinorCommand
// ---------------------------------------------------------------------------

ChangeEllipseSemiMinorCommand::ChangeEllipseSemiMinorCommand(
    draft::DraftDocument& doc, uint64_t entityId, double newValue)
    : m_doc(doc), m_entityId(entityId), m_newValue(newValue) {}

void ChangeEllipseSemiMinorCommand::execute() {
    for (auto& e : m_doc.entities()) {
        if (e->id() == m_entityId) {
            if (auto* el = dynamic_cast<draft::DraftEllipse*>(e.get())) {
                m_oldValue = el->semiMinor();
                el->setSemiMinor(m_newValue);
            }
            break;
        }
    }
}

void ChangeEllipseSemiMinorCommand::undo() {
    for (auto& e : m_doc.entities()) {
        if (e->id() == m_entityId) {
            if (auto* el = dynamic_cast<draft::DraftEllipse*>(e.get())) {
                el->setSemiMinor(m_oldValue);
            }
            break;
        }
    }
}

std::string ChangeEllipseSemiMinorCommand::description() const {
    return "Change Ellipse Semi-Minor";
}

// ---------------------------------------------------------------------------
// ChangeEllipseRotationCommand
// ---------------------------------------------------------------------------

ChangeEllipseRotationCommand::ChangeEllipseRotationCommand(
    draft::DraftDocument& doc, uint64_t entityId, double newRotation)
    : m_doc(doc), m_entityId(entityId), m_newRotation(newRotation) {}

void ChangeEllipseRotationCommand::execute() {
    for (auto& e : m_doc.entities()) {
        if (e->id() == m_entityId) {
            if (auto* el = dynamic_cast<draft::DraftEllipse*>(e.get())) {
                m_oldRotation = el->rotation();
                el->setRotation(m_newRotation);
            }
            break;
        }
    }
}

void ChangeEllipseRotationCommand::undo() {
    for (auto& e : m_doc.entities()) {
        if (e->id() == m_entityId) {
            if (auto* el = dynamic_cast<draft::DraftEllipse*>(e.get())) {
                el->setRotation(m_oldRotation);
            }
            break;
        }
    }
}

std::string ChangeEllipseRotationCommand::description() const {
    return "Change Ellipse Rotation";
}

// ---------------------------------------------------------------------------
// GripMoveCommand
// ---------------------------------------------------------------------------

GripMoveCommand::GripMoveCommand(draft::DraftDocument& doc, uint64_t entityId,
                                  std::shared_ptr<draft::DraftEntity> beforeState,
                                  std::shared_ptr<draft::DraftEntity> afterState)
    : m_doc(doc), m_entityId(entityId),
      m_beforeState(std::move(beforeState)),
      m_afterState(std::move(afterState)) {}

void GripMoveCommand::execute() {
    if (m_firstExec) {
        // State is already applied by the caller (live grip drag).
        m_firstExec = false;
        return;
    }
    applyState(*m_afterState);
}

void GripMoveCommand::undo() {
    applyState(*m_beforeState);
}

std::string GripMoveCommand::description() const {
    return "Grip Edit";
}

void GripMoveCommand::applyState(const draft::DraftEntity& state) {
    auto& entities = m_doc.entities();
    for (auto& e : entities) {
        if (e->id() == m_entityId) {
            auto replacement = state.clone();
            replacement->setId(m_entityId);
            replacement->setLayer(state.layer());
            replacement->setColor(state.color());
            replacement->setLineWidth(state.lineWidth());
            replacement->setLineType(state.lineType());
            replacement->setGroupId(state.groupId());
            e = replacement;
            return;
        }
    }
}

// ---------------------------------------------------------------------------
// GroupEntitiesCommand
// ---------------------------------------------------------------------------

GroupEntitiesCommand::GroupEntitiesCommand(draft::DraftDocument& doc,
                                           const std::vector<uint64_t>& entityIds)
    : m_doc(doc), m_entityIds(entityIds) {}

void GroupEntitiesCommand::execute() {
    if (m_newGroupId == 0) {
        m_newGroupId = m_doc.nextGroupId();
    }
    m_oldGroupIds.clear();
    for (uint64_t id : m_entityIds) {
        for (auto& e : m_doc.entities()) {
            if (e->id() == id) {
                m_oldGroupIds.emplace_back(id, e->groupId());
                e->setGroupId(m_newGroupId);
                break;
            }
        }
    }
}

void GroupEntitiesCommand::undo() {
    for (const auto& [id, oldGid] : m_oldGroupIds) {
        for (auto& e : m_doc.entities()) {
            if (e->id() == id) {
                e->setGroupId(oldGid);
                break;
            }
        }
    }
}

std::string GroupEntitiesCommand::description() const {
    return "Group";
}

// ---------------------------------------------------------------------------
// UngroupEntitiesCommand
// ---------------------------------------------------------------------------

UngroupEntitiesCommand::UngroupEntitiesCommand(draft::DraftDocument& doc,
                                                 const std::vector<uint64_t>& groupIds)
    : m_doc(doc), m_groupIds(groupIds) {}

void UngroupEntitiesCommand::execute() {
    m_savedGroupIds.clear();
    for (auto& e : m_doc.entities()) {
        uint64_t gid = e->groupId();
        if (gid == 0) continue;
        for (uint64_t target : m_groupIds) {
            if (gid == target) {
                m_savedGroupIds.emplace_back(e->id(), gid);
                e->setGroupId(0);
                break;
            }
        }
    }
}

void UngroupEntitiesCommand::undo() {
    for (const auto& [id, gid] : m_savedGroupIds) {
        for (auto& e : m_doc.entities()) {
            if (e->id() == id) {
                e->setGroupId(gid);
                break;
            }
        }
    }
}

std::string UngroupEntitiesCommand::description() const {
    return "Ungroup";
}

// ---------------------------------------------------------------------------
// remapCloneGroupIds
// ---------------------------------------------------------------------------

void remapCloneGroupIds(draft::DraftDocument& doc,
                        std::vector<std::shared_ptr<draft::DraftEntity>>& clones) {
    std::unordered_map<uint64_t, uint64_t> remap;
    for (auto& clone : clones) {
        uint64_t gid = clone->groupId();
        if (gid == 0) continue;
        auto it = remap.find(gid);
        if (it == remap.end()) {
            uint64_t newGid = doc.nextGroupId();
            remap[gid] = newGid;
            clone->setGroupId(newGid);
        } else {
            clone->setGroupId(it->second);
        }
    }
}

}  // namespace hz::doc
