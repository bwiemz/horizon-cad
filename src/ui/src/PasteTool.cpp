#include "horizon/ui/PasteTool.h"

#include <QKeyEvent>
#include <QMouseEvent>
#include <map>
#include <memory>
#include <set>
#include <string>

#include "horizon/document/Commands.h"
#include "horizon/document/Document.h"
#include "horizon/document/UndoStack.h"
#include "horizon/drafting/BlockDefinition.h"
#include "horizon/drafting/DraftBlockRef.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/Intersection.h"
#include "horizon/drafting/Layer.h"
#include "horizon/ui/Clipboard.h"
#include "horizon/ui/ViewportWidget.h"

namespace hz::ui {

namespace {

/// Brings into a drawing what pasted entities name and it may lack: the
/// blocks their references show (Copy in one drawing, Paste in another),
/// each added in the paste's own step. A reference kept the other drawing's
/// block, which this drawing's file did not have: it was lost, or bound to
/// another block of the same name, when the file was read.
class BlockAdopter {
public:
    BlockAdopter(draft::DraftDocument& doc, doc::CompositeCommand& step)
        : m_doc(doc), m_step(step) {}

    /// @p entity as the drawing takes it: a reference to this drawing's
    /// block; the rest as they are.
    std::shared_ptr<draft::DraftEntity> adopt(std::shared_ptr<draft::DraftEntity> entity) {
        const auto* ref = dynamic_cast<const draft::DraftBlockRef*>(entity.get());
        if (ref == nullptr || !ref->definition()) return entity;
        const auto block = adoptBlock(ref->definition());
        if (block == ref->definition()) return entity;
        auto bound = std::make_shared<draft::DraftBlockRef>(block, ref->insertPos(),
                                                            ref->rotation(), ref->uniformScale());
        bound->setMirrored(ref->mirrored());
        bound->copyStyleFrom(*ref);
        return bound;
    }

private:
    /// This drawing's block for @p block: the block itself, if it is this
    /// drawing's, or one of the same name that draws the same; else a copy,
    /// added under its name, or under a new one where the drawing has
    /// another block of that name.
    std::shared_ptr<draft::BlockDefinition> adoptBlock(
        const std::shared_ptr<draft::BlockDefinition>& block) {
        const auto known = m_adopted.find(block.get());
        if (known != m_adopted.end()) return known->second;
        const auto& table = m_doc.blockTable();
        if (auto here = table.findBlock(block->name)) {
            if (here == block || (here->basePoint.distanceTo(block->basePoint) <= 1e-9 &&
                                  doc::drawSame(here->entities, block->entities))) {
                m_adopted[block.get()] = here;
                return here;
            }
        }
        auto copy = std::make_shared<draft::BlockDefinition>();
        m_adopted[block.get()] = copy;  // a block within itself ends here
        copy->name = block->name;
        for (int n = 2; table.findBlock(copy->name) || m_names.count(copy->name) != 0; ++n) {
            copy->name = block->name + " (" + std::to_string(n) + ")";
        }
        m_names.insert(copy->name);
        copy->basePoint = block->basePoint;
        for (const auto& inner : block->entities) {
            if (inner) copy->entities.push_back(adopt(inner->clone()));
        }
        m_step.addCommand(std::make_unique<doc::AddBlockDefinitionCommand>(m_doc, copy));
        return copy;
    }

    draft::DraftDocument& m_doc;
    doc::CompositeCommand& m_step;
    std::map<const draft::BlockDefinition*, std::shared_ptr<draft::BlockDefinition>> m_adopted;
    std::set<std::string> m_names;  ///< given to copies in this paste
};

}  // namespace

PasteTool::PasteTool(Clipboard* clipboard) : m_clipboard(clipboard) {}

void PasteTool::deactivate() {
    cancel();
    Tool::deactivate();
}

bool PasteTool::mousePressEvent(QMouseEvent* event, const math::Vec2& worldPos) {
    if (event->button() != Qt::LeftButton) return false;
    if (!m_viewport || !m_viewport->document()) return false;
    if (!m_clipboard || !m_clipboard->hasContent()) return false;

    auto& doc = m_viewport->document()->activeDrawing();
    auto result = m_viewport->snap(worldPos);
    math::Vec2 placement = result.point;
    m_viewport->setLastSnapResult(result);

    math::Vec2 offset = placement - m_clipboard->centroid();

    auto composite = std::make_unique<doc::CompositeCommand>("Paste");
    std::vector<std::shared_ptr<draft::DraftEntity>> newEntities;

    // The layers of what was copied that this drawing lacks (copied from
    // another): on a layer the drawing does not have, an entity is neither
    // drawn as its layer nor picked. One that is still not here goes on 0.
    auto& layers = m_viewport->document()->layerManager();
    std::set<std::string> added;
    for (const auto& props : m_clipboard->layers()) {
        if (layers.getLayer(props.name) == nullptr && added.insert(props.name).second) {
            composite->addCommand(std::make_unique<doc::AddLayerCommand>(layers, props));
        }
    }
    const auto onKnownLayer = [&layers, &added](draft::DraftEntity& entity) {
        if (layers.getLayer(entity.layer()) == nullptr && added.count(entity.layer()) == 0) {
            entity.setLayer("0");
        }
    };

    BlockAdopter blocks(doc, *composite);
    for (const auto& clipEntity : m_clipboard->entities()) {
        auto clone = blocks.adopt(clipEntity->clone());
        clone->translate(offset);
        onKnownLayer(*clone);
        newEntities.push_back(clone);
    }
    doc::adoptClones(doc, newEntities);
    for (const auto& clone : newEntities) {
        composite->addCommand(std::make_unique<doc::AddEntityCommand>(doc, clone));
    }
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
        auto result = m_viewport->snap(worldPos);
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

bool PasteTool::wantsCrosshair() const {
    return false;
}

}  // namespace hz::ui
