#include "horizon/ui/DraftingCommands.h"

#include <QComboBox>
#include <QDialog>
#include <QDoubleSpinBox>
#include <QLineEdit>
#include <QMessageBox>
#include <QSpinBox>
#include <QStringList>
#include <algorithm>
#include <memory>
#include <set>
#include <string>

#include "horizon/document/Commands.h"
#include "horizon/document/Document.h"
#include "horizon/drafting/DraftBlockRef.h"
#include "horizon/math/BoundingBox.h"
#include "horizon/math/Constants.h"
#include "horizon/math/MathUtils.h"
#include "horizon/ui/FeatureForm.h"
#include "horizon/ui/InsertBlockDialog.h"
#include "horizon/ui/InsertBlockTool.h"
#include "horizon/ui/PasteTool.h"
#include "horizon/ui/PolarArrayDialog.h"
#include "horizon/ui/Preferences.h"
#include "horizon/ui/QuantitySpinBox.h"
#include "horizon/ui/RectArrayDialog.h"
#include "horizon/ui/ViewportWidget.h"
#include "horizon/ui/WorkbenchHost.h"

namespace hz::ui {

DraftingCommands::DraftingCommands(WorkbenchHost& host, QObject* parent)
    : QObject(parent), m_host(host) {}

std::vector<uint64_t> DraftingCommands::editableSelection() {
    const auto& sel = m_host.viewport().selectionManager();
    doc::Document& document = *m_host.currentDocument();
    const auto& layers = document.layerManager();
    std::vector<uint64_t> ids;
    for (const auto& entity : document.activeDrawing().entities()) {
        if (!sel.isSelected(entity->id())) continue;
        const auto* layer = layers.getLayer(entity->layer());
        if (!layer || !layer->visible || layer->locked) continue;
        ids.push_back(entity->id());
    }
    return ids;
}

void DraftingCommands::selectOnly(const std::vector<uint64_t>& ids) {
    auto& sel = m_host.viewport().selectionManager();
    sel.clearSelection();
    for (const uint64_t id : ids) sel.select(id);
    m_host.viewport().update();
    m_host.selectionChanged();
}

// ---------------------------------------------------------------------------
// Edit
// ---------------------------------------------------------------------------

void DraftingCommands::onDuplicate() {
    const std::vector<uint64_t> ids = editableSelection();
    if (ids.empty()) return;
    doc::Document& document = *m_host.currentDocument();
    auto cmd = std::make_unique<doc::DuplicateEntityCommand>(document.activeDrawing(), ids,
                                                             math::Vec2(1.0, -1.0));
    auto* rawCmd = cmd.get();
    document.undoStack().push(std::move(cmd));
    selectOnly(rawCmd->clonedIds());
}

void DraftingCommands::onCopy() {
    const auto& sel = m_host.viewport().selectionManager();
    if (sel.selectedIds().empty()) return;
    std::vector<std::shared_ptr<draft::DraftEntity>> entities;
    for (const auto& entity : m_host.currentDocument()->activeDrawing().entities()) {
        if (sel.isSelected(entity->id())) entities.push_back(entity);
    }
    m_clipboard.copy(entities, &m_host.currentDocument()->layerManager());
}

void DraftingCommands::onCut() {
    onCopy();
    auto& sel = m_host.viewport().selectionManager();
    if (sel.selectedIds().empty()) return;
    // Only what is on a shown, unlocked layer goes.
    std::vector<uint64_t> removable = editableSelection();
    if (!removable.empty()) {
        doc::Document& document = *m_host.currentDocument();
        auto composite = std::make_unique<doc::CompositeCommand>("Cut");
        composite->addCommand(std::make_unique<doc::RemoveEntitiesCommand>(document.activeDrawing(),
                                                                           std::move(removable)));
        document.undoStack().push(std::move(composite));
    }
    selectOnly({});
}

void DraftingCommands::onPaste() {
    if (!m_clipboard.hasContent()) return;
    m_host.runTool(std::make_unique<PasteTool>(&m_clipboard));
}

void DraftingCommands::onGroupEntities() {
    if (m_host.viewport().selectionManager().selectedIds().size() < 2) return;
    const std::vector<uint64_t> ids = editableSelection();
    if (ids.size() < 2) return;  // a group is of two or more
    doc::Document& document = *m_host.currentDocument();
    document.undoStack().push(
        std::make_unique<doc::GroupEntitiesCommand>(document.activeDrawing(), ids));
    m_host.viewport().update();
}

void DraftingCommands::onUngroupEntities() {
    const auto& sel = m_host.viewport().selectionManager();
    if (sel.selectedIds().empty()) return;
    doc::Document& document = *m_host.currentDocument();
    // The groups of what is selected.
    std::set<uint64_t> groupIds;
    for (const auto& entity : document.activeDrawing().entities()) {
        if (sel.isSelected(entity->id()) && entity->groupId() != 0) {
            groupIds.insert(entity->groupId());
        }
    }
    if (groupIds.empty()) return;
    document.undoStack().push(std::make_unique<doc::UngroupEntitiesCommand>(
        document.activeDrawing(), std::vector<uint64_t>(groupIds.begin(), groupIds.end())));
    m_host.viewport().update();
}

// ---------------------------------------------------------------------------
// Arrays
// ---------------------------------------------------------------------------

void DraftingCommands::onRectangularArray() {
    const std::vector<uint64_t> ids = editableSelection();
    if (ids.empty()) return;
    doc::Document& document = *m_host.currentDocument();
    RectArrayDialog dlg(m_host.dialogParent(), document.lengthUnit());
    if (dlg.exec() != QDialog::Accepted) return;

    const int cols = dlg.columns();
    const int rows = dlg.rows();
    const double sx = dlg.spacingX();
    const double sy = dlg.spacingY();
    auto& drawing = document.activeDrawing();
    auto composite = std::make_unique<doc::CompositeCommand>("Rectangular Array");
    std::vector<uint64_t> newIds;
    std::vector<std::shared_ptr<draft::DraftEntity>> allClones;
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            if (r == 0 && c == 0) continue;  // the original's place
            const math::Vec2 offset(c * sx, r * sy);
            for (const uint64_t id : ids) {
                if (const auto entity = drawing.sharedEntity(id)) {
                    auto clone = entity->clone();
                    clone->translate(offset);
                    newIds.push_back(clone->id());
                    allClones.push_back(clone);
                    composite->addCommand(std::make_unique<doc::AddEntityCommand>(drawing, clone));
                }
            }
        }
    }
    doc::adoptClones(drawing, allClones);
    document.undoStack().push(std::move(composite));
    selectOnly(newIds);
}

void DraftingCommands::onPolarArray() {
    const std::vector<uint64_t> ids = editableSelection();
    if (ids.empty()) return;
    doc::Document& document = *m_host.currentDocument();
    PolarArrayDialog dlg(m_host.dialogParent(), document.lengthUnit());
    if (dlg.exec() != QDialog::Accepted) return;

    const int count = dlg.count();
    const math::Vec2 center(dlg.centerX(), dlg.centerY());
    const double step = math::degToRad(dlg.totalAngle()) / count;
    auto& drawing = document.activeDrawing();
    auto composite = std::make_unique<doc::CompositeCommand>("Polar Array");
    std::vector<uint64_t> newIds;
    std::vector<std::shared_ptr<draft::DraftEntity>> allClones;
    for (int i = 1; i < count; ++i) {
        const double angle = step * i;
        for (const uint64_t id : ids) {
            if (const auto entity = drawing.sharedEntity(id)) {
                auto clone = entity->rotatedCopy(center, angle);
                newIds.push_back(clone->id());
                allClones.push_back(clone);
                composite->addCommand(std::make_unique<doc::AddEntityCommand>(drawing, clone));
            }
        }
    }
    doc::adoptClones(drawing, allClones);
    document.undoStack().push(std::move(composite));
    selectOnly(newIds);
}

// ---------------------------------------------------------------------------
// Dimensions and blocks
// ---------------------------------------------------------------------------

void DraftingCommands::onDimensionStyle() {
    doc::Document& document = *m_host.currentDocument();
    draft::DraftDocument& drawing = document.activeDrawing();
    const draft::DimensionStyle& now = drawing.dimensionStyle();
    const QStringList units = Preferences::lengthUnits();

    FeatureForm form(m_host.dialogParent(), tr("Dimension Style"), document.lengthUnit());
    auto* height = form.length(QStringLiteral("textHeight"), tr("Text height:"), now.textHeight,
                               0.01, 1000.0, 3);
    auto* arrow =
        form.length(QStringLiteral("arrowSize"), tr("Arrow size:"), now.arrowSize, 0.0, 1000.0, 3);
    auto* angle = form.angle(QStringLiteral("arrowAngle"), tr("Arrow half-angle:"),
                             now.arrowAngle * math::kRadToDeg, 1.0, 89.0, 1);
    auto* gap = form.length(QStringLiteral("extensionGap"), tr("Extension gap:"), now.extensionGap,
                            0.0, 1000.0, 3);
    auto* overshoot = form.length(QStringLiteral("extensionOvershoot"), tr("Extension overshoot:"),
                                  now.extensionOvershoot, 0.0, 1000.0, 3);
    auto* precision =
        form.count(QStringLiteral("precision"), tr("Decimal places:"), now.precision, 0, 12);
    auto* unit = form.choice(QStringLiteral("unit"), tr("Unit:"), units);
    unit->setCurrentIndex(
        std::max(0, static_cast<int>(units.indexOf(QString::fromStdString(now.unit)))));
    auto* showUnit =
        form.choice(QStringLiteral("showUnits"), tr("Show the unit:"), {tr("No"), tr("Yes")});
    showUnit->setCurrentIndex(now.showUnits ? 1 : 0);
    // A field shows its value rounded to its decimals (the arrow's 0.3 radians
    // as 17.2 degrees); left as shown, it keeps the value exactly.
    const auto field = [](QDoubleSpinBox* spin, double was, double scale = 1.0) {
        return [spin, was, scale, shown = spin->value()] {
            return spin->value() == shown ? was : spin->value() * scale;
        };
    };
    const auto newHeight = field(height, now.textHeight);
    const auto newArrow = field(arrow, now.arrowSize);
    const auto newAngle = field(angle, now.arrowAngle, math::kDegToRad);
    const auto newGap = field(gap, now.extensionGap);
    const auto newOvershoot = field(overshoot, now.extensionOvershoot);
    if (!form.exec()) return;

    draft::DimensionStyle style = now;
    style.textHeight = newHeight();
    style.arrowSize = newArrow();
    style.arrowAngle = newAngle();
    style.extensionGap = newGap();
    style.extensionOvershoot = newOvershoot();
    style.precision = precision->value();
    style.unit = unit->currentText().toStdString();
    style.showUnits = showUnit->currentIndex() == 1;
    if (style == now) return;  // OK with nothing changed is not a step to undo
    document.undoStack().push(std::make_unique<doc::ChangeDimensionStyleCommand>(drawing, style));
    m_host.viewport().update();
}

void DraftingCommands::onCreateBlock() {
    if (m_host.viewport().selectionManager().selectedIds().empty()) {
        QMessageBox::information(m_host.dialogParent(), tr("Create Block"),
                                 tr("Select entities first."));
        return;
    }
    const std::vector<uint64_t> ids = editableSelection();
    if (ids.empty()) return;
    doc::Document& document = *m_host.currentDocument();
    auto& drawing = document.activeDrawing();

    // The base point, where the block is inserted from: by default the centre
    // of what was selected, or typed.
    math::BoundingBox bounds;
    for (const uint64_t id : ids) {
        if (const auto* e = drawing.findEntity(id)) {
            const auto bb = e->boundingBox();
            if (bb.isValid()) bounds.expand(bb);
        }
    }
    const math::Vec3 centre = bounds.isValid() ? bounds.center() : math::Vec3(0, 0, 0);
    FeatureForm form(m_host.dialogParent(), tr("Create Block"), document.lengthUnit());
    auto* nameField = form.text(QStringLiteral("blockName"), tr("Block name:"));
    auto* baseX = form.length(QStringLiteral("baseX"), tr("Base point X:"), centre.x, -1e9, 1e9, 4);
    auto* baseY = form.length(QStringLiteral("baseY"), tr("Base point Y:"), centre.y, -1e9, 1e9, 4);
    if (!form.exec()) return;
    const QString name = nameField->text().trimmed();
    if (name.isEmpty()) return;

    const std::string blockName = name.toStdString();
    if (drawing.blockTable().findBlock(blockName)) {
        QMessageBox::warning(m_host.dialogParent(), tr("Create Block"),
                             tr("A block with that name already exists."));
        return;
    }
    auto cmd = std::make_unique<doc::CreateBlockCommand>(drawing, blockName, ids,
                                                         math::Vec2(baseX->value(), baseY->value()),
                                                         document.layerManager().currentLayer());
    auto* rawCmd = cmd.get();
    document.undoStack().push(std::move(cmd));
    selectOnly({rawCmd->blockRefId()});
}

void DraftingCommands::onInsertBlock() {
    auto& blocks = m_host.currentDocument()->activeDrawing().blockTable();
    const auto names = blocks.blockNames();
    if (names.empty()) {
        QMessageBox::information(m_host.dialogParent(), tr("Insert Block"),
                                 tr("No blocks defined. Create a block first."));
        return;
    }
    InsertBlockDialog dlg(names, m_host.dialogParent());
    if (dlg.exec() != QDialog::Accepted) return;
    auto def = blocks.findBlock(dlg.selectedBlock());
    if (!def) return;
    // It replaces the Insert Block tool before it, which the window lets go
    // of first.
    m_host.runTool(std::make_unique<InsertBlockTool>(def, dlg.rotation(), dlg.scale()));
}

void DraftingCommands::onExplode() {
    const auto& sel = m_host.viewport().selectionManager();
    if (sel.selectedIds().empty()) return;
    doc::Document& document = *m_host.currentDocument();
    auto& drawing = document.activeDrawing();

    // The block references among the selection.
    std::vector<uint64_t> blockRefIds;
    for (const auto& entity : drawing.entities()) {
        if (sel.isSelected(entity->id()) &&
            dynamic_cast<const draft::DraftBlockRef*>(entity.get()) != nullptr) {
            blockRefIds.push_back(entity->id());
        }
    }
    if (blockRefIds.empty()) {
        QMessageBox::information(m_host.dialogParent(), tr("Explode"),
                                 tr("Select one or more block references to explode."));
        return;
    }
    auto composite = std::make_unique<doc::CompositeCommand>("Explode");
    std::vector<doc::ExplodeBlockCommand*> explodeCmds;
    for (const uint64_t id : blockRefIds) {
        auto cmd = std::make_unique<doc::ExplodeBlockCommand>(drawing, id);
        explodeCmds.push_back(cmd.get());
        composite->addCommand(std::move(cmd));
    }
    document.undoStack().push(std::move(composite));

    // What they held, selected.
    std::vector<uint64_t> exploded;
    for (auto* cmd : explodeCmds) {
        const auto& ids = cmd->explodedIds();
        exploded.insert(exploded.end(), ids.begin(), ids.end());
    }
    selectOnly(exploded);
}

}  // namespace hz::ui
