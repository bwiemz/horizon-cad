#pragma once

#include <QWidget>

class QTabBar;
class QStackedWidget;
class QToolBar;

namespace hz::ui {

/// A tabbed toolbar widget that organizes tools into categories,
/// similar to the ribbon interface in professional CAD software.
class RibbonBar : public QWidget {
    Q_OBJECT

public:
    explicit RibbonBar(QWidget* parent = nullptr);

    /// Add a tab with the given name and toolbar content.
    /// Ownership of the toolbar is transferred to this widget.
    void addTab(const QString& name, QToolBar* toolbar);

    /// Set active tab by index.
    void setCurrentTab(int index);

    /// Set active tab by name.
    void setCurrentTab(const QString& name);

private slots:
    void onTabChanged(int index);

private:
    QTabBar* m_tabBar = nullptr;
    QStackedWidget* m_stack = nullptr;
};

}  // namespace hz::ui
