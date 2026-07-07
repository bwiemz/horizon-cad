#include "horizon/ui/CommandPalette.h"

#include <QAction>
#include <QKeyEvent>
#include <QLineEdit>
#include <QListWidget>
#include <QMenu>
#include <QVBoxLayout>

namespace hz::ui {

CommandPalette::CommandPalette(const QList<QAction*>& actions, QWidget* parent) : QDialog(parent) {
    setObjectName("commandPalette");
    setWindowTitle(tr("Command Palette"));
    setModal(true);
    // Frameless popup that floats over the window like a search overlay.
    setWindowFlags(Qt::Dialog | Qt::FramelessWindowHint);
    resize(480, 340);

    // Keep only real, invokable commands (skip separators, submenu openers,
    // and anything currently unavailable).
    for (QAction* act : actions) {
        if (!act || act->isSeparator() || act->menu()) continue;
        if (act->text().isEmpty() || !act->isEnabled() || !act->isVisible()) continue;
        m_actions.push_back(act);
    }

    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(10, 10, 10, 10);
    layout->setSpacing(6);

    m_search = new QLineEdit(this);
    m_search->setObjectName("commandPaletteSearch");
    m_search->setPlaceholderText(tr("Type a command…  (e.g. line, extrude, export)"));
    m_search->setClearButtonEnabled(true);
    layout->addWidget(m_search);

    m_list = new QListWidget(this);
    m_list->setObjectName("commandPaletteList");
    m_list->setUniformItemSizes(true);
    layout->addWidget(m_list, 1);

    connect(m_search, &QLineEdit::textChanged, this, &CommandPalette::rebuild);
    connect(m_list, &QListWidget::itemActivated, this,
            [this](QListWidgetItem*) { triggerCurrent(); });
    connect(m_list, &QListWidget::itemClicked, this,
            [this](QListWidgetItem*) { triggerCurrent(); });

    // Route navigation/confirm keys pressed in the search box to the list.
    m_search->installEventFilter(this);

    rebuild(QString());
    m_search->setFocus();
}

void CommandPalette::rebuild(const QString& filter) {
    m_list->clear();
    const QString needle = filter.trimmed();
    for (QAction* act : m_actions) {
        // Strip the '&' mnemonic markers from menu text for matching/display.
        QString label = act->text();
        label.remove(QLatin1Char('&'));
        if (!needle.isEmpty() && !label.contains(needle, Qt::CaseInsensitive)) continue;

        const QString shortcut = act->shortcut().toString(QKeySequence::NativeText);
        auto* item = new QListWidgetItem(
            shortcut.isEmpty() ? label : QStringLiteral("%1\t%2").arg(label, shortcut));
        item->setData(Qt::UserRole, QVariant::fromValue(reinterpret_cast<quintptr>(act)));
        if (!act->icon().isNull()) item->setIcon(act->icon());
        m_list->addItem(item);
    }
    if (m_list->count() > 0) m_list->setCurrentRow(0);
}

void CommandPalette::moveSelection(int delta) {
    const int count = m_list->count();
    if (count == 0) return;
    int row = m_list->currentRow() + delta;
    if (row < 0) row = count - 1;
    if (row >= count) row = 0;
    m_list->setCurrentRow(row);
}

void CommandPalette::triggerCurrent() {
    QListWidgetItem* item = m_list->currentItem();
    if (!item) return;
    auto* act = reinterpret_cast<QAction*>(item->data(Qt::UserRole).value<quintptr>());
    accept();
    if (act) act->trigger();
}

bool CommandPalette::eventFilter(QObject* watched, QEvent* event) {
    if (watched == m_search && event->type() == QEvent::KeyPress) {
        auto* key = static_cast<QKeyEvent*>(event);
        switch (key->key()) {
            case Qt::Key_Down:
                moveSelection(1);
                return true;
            case Qt::Key_Up:
                moveSelection(-1);
                return true;
            case Qt::Key_Return:
            case Qt::Key_Enter:
                triggerCurrent();
                return true;
            default:
                break;
        }
    }
    return QDialog::eventFilter(watched, event);
}

}  // namespace hz::ui
