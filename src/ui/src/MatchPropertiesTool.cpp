#include "horizon/ui/MatchPropertiesTool.h"

#include <QKeyEvent>
#include <QMouseEvent>
#include <memory>
#include <vector>

#include "horizon/document/Commands.h"
#include "horizon/document/Document.h"
#include "horizon/drafting/DraftDocument.h"
#include "horizon/drafting/Layer.h"
#include "horizon/math/BoundingBox.h"
#include "horizon/ui/ViewportWidget.h"

namespace hz::ui {

const draft::DraftEntity* MatchPropertiesTool::entityAt(const draft::DraftDocument& drawing,
                                                        const draft::LayerManager& layers,
                                                        const math::Vec2& point, double tolerance,
                                                        bool unlockedOnly) {
    const math::BoundingBox around(math::Vec3(point.x - tolerance, point.y - tolerance, -1e9),
                                   math::Vec3(point.x + tolerance, point.y + tolerance, 1e9));
    for (const uint64_t id : drawing.spatialIndex().query(around)) {
        const draft::DraftEntity* entity = drawing.findEntity(id);
        if (entity == nullptr) continue;
        const auto* layer = layers.getLayer(entity->layer());
        if (!layer || !layer->visible || (unlockedOnly && layer->locked)) continue;
        if (entity->hitTest(point, tolerance)) return entity;
    }
    return nullptr;
}

void MatchPropertiesTool::activate(ViewportWidget* viewport) {
    Tool::activate(viewport);
    m_sourceId = 0;
    m_note.clear();
    // One entity selected is the one to match, as AutoCAD takes it.
    if (m_viewport && m_viewport->document()) {
        const auto ids = m_viewport->selectionManager().selectedIds();
        if (ids.size() == 1) {
            m_sourceId = ids.front();
            if (source() == nullptr) m_sourceId = 0;
        }
    }
}

void MatchPropertiesTool::deactivate() {
    m_sourceId = 0;
    m_note.clear();
    Tool::deactivate();
}

const draft::DraftEntity* MatchPropertiesTool::source() const {
    if (m_sourceId == 0 || !m_viewport || !m_viewport->document()) return nullptr;
    doc::Document& document = *m_viewport->document();
    const draft::DraftEntity* entity = document.activeDrawing().findEntity(m_sourceId);
    if (entity == nullptr) return nullptr;
    const auto* layer = document.layerManager().getLayer(entity->layer());
    return layer && layer->visible ? entity : nullptr;
}

bool MatchPropertiesTool::matchTo(const draft::DraftEntity& target) {
    const draft::DraftEntity* from = source();
    if (from == nullptr || !m_viewport || !m_viewport->document()) return false;
    doc::Document& document = *m_viewport->document();
    auto& drawing = document.activeDrawing();
    const std::vector<uint64_t> ids{target.id()};
    auto composite = std::make_unique<doc::CompositeCommand>("Match Properties");
    if (target.layer() != from->layer()) {
        composite->addCommand(
            std::make_unique<doc::ChangeEntityLayerCommand>(drawing, ids, from->layer()));
    }
    if (target.color() != from->color()) {
        composite->addCommand(
            std::make_unique<doc::ChangeEntityColorCommand>(drawing, ids, from->color()));
    }
    if (target.lineWidth() != from->lineWidth()) {
        composite->addCommand(
            std::make_unique<doc::ChangeEntityLineWidthCommand>(drawing, ids, from->lineWidth()));
    }
    if (target.lineType() != from->lineType()) {
        composite->addCommand(
            std::make_unique<doc::ChangeEntityLineTypeCommand>(drawing, ids, from->lineType()));
    }
    if (composite->empty()) return false;
    document.undoStack().push(std::move(composite));
    return true;
}

bool MatchPropertiesTool::mousePressEvent(QMouseEvent* event, const math::Vec2& worldPos) {
    if (event->button() != Qt::LeftButton) return false;
    if (!m_viewport || !m_viewport->document()) return false;
    doc::Document& document = *m_viewport->document();
    const double tolerance = m_viewport->pickTolerance(10.0);
    const auto& drawing = document.activeDrawing();
    const auto& layers = document.layerManager();

    if (source() == nullptr) {
        // The first may be on a locked layer: nothing is done to it.
        const draft::DraftEntity* picked = entityAt(drawing, layers, worldPos, tolerance, false);
        m_sourceId = picked != nullptr ? picked->id() : 0;
        m_note = picked != nullptr ? "" : "nothing there to match";
        if (picked != nullptr) {
            auto& sel = m_viewport->selectionManager();
            sel.clearSelection();
            sel.select(picked->id());
        }
        m_viewport->update();
        return true;
    }

    const draft::DraftEntity* target = entityAt(drawing, layers, worldPos, tolerance, true);
    if (target == nullptr) {
        m_note = "nothing there that can be changed";
    } else if (target->id() == m_sourceId) {
        m_note = "that is the entity being matched";
    } else {
        m_note = matchTo(*target) ? "" : "it matches already";
    }
    m_viewport->update();
    return true;
}

bool MatchPropertiesTool::mouseMoveEvent(QMouseEvent* /*event*/, const math::Vec2& /*worldPos*/) {
    return false;
}

bool MatchPropertiesTool::mouseReleaseEvent(QMouseEvent* event, const math::Vec2& /*worldPos*/) {
    return event->button() == Qt::LeftButton;
}

bool MatchPropertiesTool::keyPressEvent(QKeyEvent* event) {
    if (event->key() != Qt::Key_Escape) return false;
    if (m_sourceId == 0) return false;  // nothing picked: Escape leaves the tool
    cancel();
    return true;
}

void MatchPropertiesTool::cancel() {
    m_sourceId = 0;
    m_note.clear();
    if (m_viewport) {
        m_viewport->selectionManager().clearSelection();
        m_viewport->update();
    }
}

std::string MatchPropertiesTool::promptText() const {
    std::string prompt = source() == nullptr
                             ? "Match Properties: click the entity whose properties to copy"
                             : "Match Properties: click entities to give them its layer, colour, "
                               "width and line type (Escape to pick another)";
    if (!m_note.empty()) prompt += " - " + m_note;
    return prompt;
}

}  // namespace hz::ui
