#include "horizon/ui/PasteTool.h"
#include "horizon/ui/Clipboard.h"
#include "horizon/ui/ViewportWidget.h"
#include "horizon/document/Document.h"
#include "horizon/document/Commands.h"
#include "horizon/document/UndoStack.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/Intersection.h"

#include <QMouseEvent>
#include <QKeyEvent>

namespace hz::ui {

PasteTool::PasteTool(Clipboard* clipboard)
    : m_clipboard(clipboard) {}

void PasteTool::deactivate() {
    cancel();
    Tool::deactivate();
}

bool PasteTool::mousePressEvent(QMouseEvent* event, const math::Vec2& worldPos) {
    if (event->button() != Qt::LeftButton) return false;
    if (!m_viewport || !m_viewport->document()) return false;
    if (!m_clipboard || !m_clipboard->hasContent()) return false;

    auto& doc = m_viewport->document()->draftDocument();
    auto result = m_viewport->snapEngine().snap(worldPos, doc.entities());
    math::Vec2 placement = result.point;
    m_viewport->setLastSnapResult(result);

    math::Vec2 offset = placement - m_clipboard->centroid();

    auto composite = std::make_unique<doc::CompositeCommand>("Paste");
    std::vector<std::shared_ptr<draft::DraftEntity>> newEntities;

    for (const auto& clipEntity : m_clipboard->entities()) {
        auto clone = clipEntity->clone();
        clone->translate(offset);
        newEntities.push_back(clone);
        composite->addCommand(std::make_unique<doc::AddEntityCommand>(doc, clone));
    }

    doc::remapCloneGroupIds(doc, newEntities);
    m_viewport->document()->undoStack().push(std::move(composite));

    // Select the pasted entities.
    auto& sel = m_viewport->selectionManager();
    sel.clearSelection();
    for (const auto& e : newEntities) {
        sel.select(e->id());
    }

    return true;
}

bool PasteTool::mouseMoveEvent(QMouseEvent* /*event*/, const math::Vec2& worldPos) {
    m_currentPos = worldPos;
    if (m_viewport && m_viewport->document()) {
        auto& doc = m_viewport->document()->draftDocument();
        auto result = m_viewport->snapEngine().snap(worldPos, doc.entities());
        m_currentPos = result.point;
        m_viewport->setLastSnapResult(result);
    }
    return true;
}

bool PasteTool::mouseReleaseEvent(QMouseEvent* /*event*/, const math::Vec2& /*worldPos*/) {
    return false;
}

bool PasteTool::keyPressEvent(QKeyEvent* event) {
    if (event->key() == Qt::Key_Escape) {
        cancel();
        return true;
    }
    return false;
}

void PasteTool::cancel() {
    if (m_viewport) {
        m_viewport->setLastSnapResult({});
    }
}

std::vector<std::pair<math::Vec2, math::Vec2>> PasteTool::getPreviewLines() const {
    std::vector<std::pair<math::Vec2, math::Vec2>> result;
    if (!m_clipboard || !m_clipboard->hasContent()) return result;

    math::Vec2 offset = m_currentPos - m_clipboard->centroid();

    for (const auto& entity : m_clipboard->entities()) {
        auto segs = draft::extractSegments(*entity);
        for (const auto& [s, e] : segs) {
            result.emplace_back(s + offset, e + offset);
        }
    }

    return result;
}

std::vector<std::pair<math::Vec2, double>> PasteTool::getPreviewCircles() const {
    std::vector<std::pair<math::Vec2, double>> result;
    if (!m_clipboard || !m_clipboard->hasContent()) return result;

    math::Vec2 offset = m_currentPos - m_clipboard->centroid();

    for (const auto& entity : m_clipboard->entities()) {
        if (auto* c = dynamic_cast<const draft::DraftCircle*>(entity.get())) {
            result.emplace_back(c->center() + offset, c->radius());
        }
    }

    return result;
}

std::string PasteTool::promptText() const {
    return "Click to place pasted entities";
}

bool PasteTool::wantsCrosshair() const { return false; }

}  // namespace hz::ui
