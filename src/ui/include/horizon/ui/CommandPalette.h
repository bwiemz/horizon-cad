#pragma once

#include <QDialog>
#include <QList>
#include <QVector>

class QLineEdit;
class QListWidget;
class QAction;

namespace hz::ui {

/// A searchable command launcher (Ctrl+K), like the command palettes in
/// VS Code / Onshape. It lists existing QActions by their text (with their
/// shortcut, when set) and triggers the chosen one, so every menu/ribbon
/// command is reachable by typing a few letters.
class CommandPalette : public QDialog {
    Q_OBJECT

public:
    /// @param actions  Commands to offer. Separators, submenu openers, and
    ///                 disabled/invisible actions are filtered out.
    CommandPalette(const QList<QAction*>& actions, QWidget* parent = nullptr);

protected:
    bool eventFilter(QObject* watched, QEvent* event) override;

private:
    void rebuild(const QString& filter);
    void triggerCurrent();
    void moveSelection(int delta);

    QLineEdit* m_search = nullptr;
    QListWidget* m_list = nullptr;
    QVector<QAction*> m_actions;  ///< the filtered, offerable action set
};

}  // namespace hz::ui
