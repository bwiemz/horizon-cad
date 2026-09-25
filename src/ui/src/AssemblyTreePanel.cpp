#include "horizon/ui/AssemblyTreePanel.h"

#include <QAction>
#include <QEvent>
#include <QKeyEvent>
#include <QSignalBlocker>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <numbers>

#include "horizon/document/AssemblyDocument.h"

namespace hz::ui {

namespace {

// Each row's kind and id.
constexpr int kKindRole = Qt::UserRole;
constexpr int kIdRole = Qt::UserRole + 1;
constexpr int kSuppressedRole = Qt::UserRole + 2;
enum Kind : int { Heading = 0, Component = 1, Mate = 2 };

QString mateName(doc::MateType type) {
    switch (type) {
        case doc::MateType::Coincident:
            return AssemblyTreePanel::tr("Coincident");
        case doc::MateType::Concentric:
            return AssemblyTreePanel::tr("Concentric");
        case doc::MateType::Distance:
            return AssemblyTreePanel::tr("Distance");
        case doc::MateType::Angle:
            return AssemblyTreePanel::tr("Angle");
        case doc::MateType::Parallel:
            return AssemblyTreePanel::tr("Parallel");
        case doc::MateType::Perpendicular:
            return AssemblyTreePanel::tr("Perpendicular");
        case doc::MateType::Tangent:
            return AssemblyTreePanel::tr("Tangent");
        case doc::MateType::Fixed:
            return AssemblyTreePanel::tr("Fixed");
    }
    return {};
}

}  // namespace

AssemblyTreePanel::AssemblyTreePanel(QWidget* parent) : QDockWidget(tr("Assembly"), parent) {
    setObjectName("AssemblyTreePanel");
    m_tree = new QTreeWidget(this);
    m_tree->setObjectName("assemblyTree");
    m_tree->setHeaderHidden(true);
    m_tree->setContextMenuPolicy(Qt::ActionsContextMenu);
    setWidget(m_tree);

    // Delete while the tree has focus (a key event, not a shortcut: the
    // feature tree's Delete is the one action bound to the key).
    m_removeAction = new QAction(tr("Remove"), m_tree);
    m_tree->installEventFilter(this);
    m_suppressAction = new QAction(tr("Suppress"), m_tree);
    m_renameAction = new QAction(tr("Rename..."), m_tree);
    m_editMateAction = new QAction(tr("Edit Value..."), m_tree);
    m_openPartAction = new QAction(tr("Open Part"), m_tree);
    m_tree->addActions(
        {m_openPartAction, m_removeAction, m_suppressAction, m_renameAction, m_editMateAction});

    connect(m_removeAction, &QAction::triggered, this, [this] {
        if (const uint64_t id = currentComponent()) emit removeComponentRequested(id);
        if (const uint64_t id = currentMate()) emit removeMateRequested(id);
    });
    connect(m_suppressAction, &QAction::triggered, this, [this] {
        if (const uint64_t id = currentComponent())
            emit suppressRequested(id, !m_currentSuppressed);
    });
    connect(m_renameAction, &QAction::triggered, this, [this] {
        if (const uint64_t id = currentComponent()) emit renameRequested(id);
    });
    connect(m_openPartAction, &QAction::triggered, this, [this] {
        if (const uint64_t id = currentComponent()) emit openPartRequested(id);
    });
    connect(m_editMateAction, &QAction::triggered, this, [this] {
        if (const uint64_t id = currentMate()) emit editMateRequested(id);
    });
    connect(m_tree, &QTreeWidget::currentItemChanged, this, [this] {
        updateActions();
        if (const uint64_t id = currentComponent()) emit componentSelected(id);
    });
    connect(m_tree, &QTreeWidget::itemDoubleClicked, this, [this](QTreeWidgetItem* item) {
        if (item == nullptr) return;
        const uint64_t id = item->data(0, kIdRole).toULongLong();
        if (item->data(0, kKindRole).toInt() == Mate) emit editMateRequested(id);
        if (item->data(0, kKindRole).toInt() == Component) emit renameRequested(id);
    });
    updateActions();
}

void AssemblyTreePanel::refresh(const doc::AssemblyDocument* assembly) {
    // The current row is kept within one assembly only: another's component
    // #1 is not the one chosen, and Delete would have removed it.
    const bool same = assembly == m_shown;
    const uint64_t component = same ? currentComponent() : 0;
    const uint64_t mate = same ? currentMate() : 0;
    m_shown = assembly;
    const QSignalBlocker quiet(m_tree);
    m_tree->clear();
    if (assembly == nullptr) {
        updateActions();
        return;
    }
    const auto nameOf = [assembly](uint64_t id) {
        const auto* comp = assembly->component(id);
        return comp == nullptr
                   ? tr("(missing)")
                   : QString::fromStdString(comp->name.empty() ? "component" : comp->name);
    };

    auto* components =
        new QTreeWidgetItem(m_tree, {tr("Components (%1)").arg(assembly->components().size())});
    components->setData(0, kKindRole, Heading);
    QTreeWidgetItem* current = nullptr;
    for (const auto& comp : assembly->components()) {
        QString text = nameOf(comp.id);
        if (comp.suppressed) text += tr(" (suppressed)");
        auto* item = new QTreeWidgetItem(components, {text});
        item->setData(0, kKindRole, Component);
        item->setData(0, kIdRole, QVariant::fromValue<qulonglong>(comp.id));
        item->setData(0, kSuppressedRole, comp.suppressed);
        item->setToolTip(0, QString::fromStdString(comp.partPath));
        if (comp.suppressed)
            item->setForeground(0, palette().brush(QPalette::Disabled, QPalette::Text));
        if (comp.id == component) current = item;
    }

    auto* mates = new QTreeWidgetItem(m_tree, {tr("Mates (%1)").arg(assembly->mates().size())});
    mates->setData(0, kKindRole, Heading);
    for (const auto& m : assembly->mates()) {
        QString text =
            m.type == doc::MateType::Fixed
                ? tr("%1: %2").arg(mateName(m.type), nameOf(m.a.componentId))
                : tr("%1: %2 and %3")
                      .arg(mateName(m.type), nameOf(m.a.componentId), nameOf(m.b.componentId));
        if (m.type == doc::MateType::Distance) text += tr(", %1 mm").arg(m.value);
        if (m.type == doc::MateType::Angle) {
            text += tr(", %1%2").arg(m.value * 180.0 / std::numbers::pi).arg(QChar(0x00B0));
        }
        auto* item = new QTreeWidgetItem(mates, {text});
        item->setData(0, kKindRole, Mate);
        item->setData(0, kIdRole, QVariant::fromValue<qulonglong>(m.id));
        if (m.id == mate) current = item;
    }
    m_tree->expandAll();
    if (current != nullptr) m_tree->setCurrentItem(current);
    updateActions();
}

bool AssemblyTreePanel::eventFilter(QObject* watched, QEvent* event) {
    if (watched == m_tree && event->type() == QEvent::KeyPress &&
        static_cast<QKeyEvent*>(event)->key() == Qt::Key_Delete && m_removeAction->isEnabled()) {
        m_removeAction->trigger();
        return true;
    }
    return QDockWidget::eventFilter(watched, event);
}

uint64_t AssemblyTreePanel::currentComponent() const {
    const QTreeWidgetItem* item = m_tree->currentItem();
    if (item == nullptr || item->data(0, kKindRole).toInt() != Component) return 0;
    return item->data(0, kIdRole).toULongLong();
}

uint64_t AssemblyTreePanel::currentMate() const {
    const QTreeWidgetItem* item = m_tree->currentItem();
    if (item == nullptr || item->data(0, kKindRole).toInt() != Mate) return 0;
    return item->data(0, kIdRole).toULongLong();
}

void AssemblyTreePanel::showComponent(uint64_t id) {
    const QSignalBlocker quiet(m_tree);
    for (int top = 0; top < m_tree->topLevelItemCount(); ++top) {
        QTreeWidgetItem* heading = m_tree->topLevelItem(top);
        for (int row = 0; row < heading->childCount(); ++row) {
            QTreeWidgetItem* item = heading->child(row);
            if (item->data(0, kKindRole).toInt() == Component &&
                item->data(0, kIdRole).toULongLong() == id) {
                m_tree->setCurrentItem(item);
                updateActions();
                return;
            }
        }
    }
}

void AssemblyTreePanel::updateActions() {
    const QTreeWidgetItem* item = m_tree->currentItem();
    const bool component = currentComponent() != 0;
    const bool mate = currentMate() != 0;
    m_currentSuppressed = component && item->data(0, kSuppressedRole).toBool();
    m_removeAction->setEnabled(component || mate);
    m_suppressAction->setEnabled(component);
    m_suppressAction->setText(m_currentSuppressed ? tr("Unsuppress") : tr("Suppress"));
    m_renameAction->setEnabled(component);
    m_openPartAction->setEnabled(component);
    m_editMateAction->setEnabled(mate);
}

}  // namespace hz::ui
