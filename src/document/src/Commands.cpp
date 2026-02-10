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
    m_clones.clear();
    for (uint64_t id : m_sourceIds) {
        for (const auto& e : m_doc.entities()) {
            if (e->id() == id) {
                auto clone = e->clone();
                clone->translate(m_offset);
                m_clones.push_back(clone);
                m_doc.addEntity(clone);
                break;
            }
        }
    }
}

void DuplicateEntityCommand::undo() {
    for (const auto& clone : m_clones) {
        m_doc.removeEntity(clone->id());
    }
    m_clones.clear();
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
    m_mirroredEntities.clear();
    for (uint64_t id : m_sourceIds) {
        for (const auto& e : m_doc.entities()) {
            if (e->id() == id) {
                auto mirrored = e->clone();
                mirrored->mirror(m_axisP1, m_axisP2);
                m_mirroredEntities.push_back(mirrored);
                m_doc.addEntity(mirrored);
                break;
            }
        }
    }
}

void MirrorEntityCommand::undo() {
    for (const auto& e : m_mirroredEntities) {
        m_doc.removeEntity(e->id());
    }
    m_mirroredEntities.clear();
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
    m_rotatedEntities.clear();
    for (uint64_t id : m_sourceIds) {
        for (const auto& e : m_doc.entities()) {
            if (e->id() == id) {
                auto rotated = e->clone();
                rotated->rotate(m_center, m_angle);
                m_rotatedEntities.push_back(rotated);
                m_doc.addEntity(rotated);
                break;
            }
        }
    }
}

void RotateEntityCommand::undo() {
    for (const auto& e : m_rotatedEntities) {
        m_doc.removeEntity(e->id());
    }
    m_rotatedEntities.clear();
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
    m_scaledEntities.clear();
    for (uint64_t id : m_sourceIds) {
        for (const auto& e : m_doc.entities()) {
            if (e->id() == id) {
                auto scaled = e->clone();
                scaled->scale(m_basePoint, m_factor);
                m_scaledEntities.push_back(scaled);
                m_doc.addEntity(scaled);
                break;
            }
        }
    }
}

void ScaleEntityCommand::undo() {
    for (const auto& e : m_scaledEntities) {
        m_doc.removeEntity(e->id());
    }
    m_scaledEntities.clear();
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

}  // namespace hz::doc
