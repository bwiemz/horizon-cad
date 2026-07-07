#pragma once

#include <QHash>
#include <QWidget>

class QTabBar;
class QStackedWidget;
class QToolBar;
class QHBoxLayout;

namespace hz::ui {

/// A tabbed command bar that organizes tools into categories, each split into
/// captioned command groups (like the ribbon in professional CAD software).
///
/// A tab page is a horizontal row of "command groups"; each group is a small
/// framed panel with a toolbar of text-under-icon buttons above a group
/// caption. Build a tab by calling addGroup() once per group and adding
/// actions to the returned toolbar.
class RibbonBar : public QWidget {
    Q_OBJECT

public:
    explicit RibbonBar(QWidget* parent = nullptr);

    /// Add (or reuse) the tab @p tabName and append a captioned command group
    /// titled @p groupTitle. Returns the group's toolbar so the caller can add
    /// actions to it via QToolBar::addAction. The toolbar is preconfigured for
    /// the ribbon look (larger icons, text under icon).
    QToolBar* addGroup(const QString& tabName, const QString& groupTitle);

    /// Legacy: add a tab whose entire content is a single icon-only toolbar.
    /// Ownership of the toolbar is transferred to this widget.
    void addTab(const QString& name, QToolBar* toolbar);

    /// Set active tab by index.
    void setCurrentTab(int index);

    /// Set active tab by name.
    void setCurrentTab(const QString& name);

private slots:
    void onTabChanged(int index);

private:
    /// Return the horizontal layout of the page for @p tabName, creating the
    /// tab (and its trailing stretch) on first use.
    QHBoxLayout* pageLayoutForTab(const QString& tabName);

    QTabBar* m_tabBar = nullptr;
    QStackedWidget* m_stack = nullptr;
    QHash<QString, QHBoxLayout*> m_pageLayouts;
};

}  // namespace hz::ui
