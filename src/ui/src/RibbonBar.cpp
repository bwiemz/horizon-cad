#include "horizon/ui/RibbonBar.h"

#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>
#include <QStackedWidget>
#include <QTabBar>
#include <QToolBar>
#include <QVBoxLayout>

namespace hz::ui {

RibbonBar::RibbonBar(QWidget* parent) : QWidget(parent) {
    setObjectName("ribbonBar");
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(0);

    // Tab bar for switching between categories.
    m_tabBar = new QTabBar(this);
    m_tabBar->setObjectName("ribbonTabBar");
    m_tabBar->setDocumentMode(true);
    m_tabBar->setExpanding(false);
    layout->addWidget(m_tabBar);

    // Stacked widget holds one page (row of command groups) per tab.
    m_stack = new QStackedWidget(this);
    m_stack->setObjectName("ribbonStack");
    layout->addWidget(m_stack);

    connect(m_tabBar, &QTabBar::currentChanged, this, &RibbonBar::onTabChanged);

    // Room for a captioned group row: tab bar (~28px) + 28px icon + button
    // label + group caption + paddings.
    setMinimumHeight(104);
    setMaximumHeight(118);
}

QHBoxLayout* RibbonBar::pageLayoutForTab(const QString& tabName) {
    auto it = m_pageLayouts.find(tabName);
    if (it != m_pageLayouts.end()) return it.value();

    auto* page = new QWidget(this);
    page->setObjectName("ribbonPage");
    auto* row = new QHBoxLayout(page);
    row->setContentsMargins(4, 3, 4, 3);
    row->setSpacing(0);
    row->addStretch(1);  // trailing stretch — groups insert before it

    m_stack->addWidget(page);
    m_tabBar->addTab(tabName);
    m_pageLayouts.insert(tabName, row);
    return row;
}

QToolBar* RibbonBar::addGroup(const QString& tabName, const QString& groupTitle) {
    QHBoxLayout* row = pageLayoutForTab(tabName);

    auto* group = new QFrame();
    group->setObjectName("ribbonGroup");
    auto* groupLayout = new QVBoxLayout(group);
    groupLayout->setContentsMargins(6, 4, 6, 3);
    groupLayout->setSpacing(2);

    auto* bar = new QToolBar(group);
    bar->setObjectName("ribbonGroupBar");
    bar->setIconSize(QSize(28, 28));
    bar->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    bar->setMovable(false);
    bar->setFloatable(false);
    groupLayout->addWidget(bar, 1);

    auto* caption = new QLabel(groupTitle, group);
    caption->setObjectName("ribbonGroupTitle");
    caption->setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
    groupLayout->addWidget(caption);

    row->insertWidget(row->count() - 1, group);  // before the trailing stretch
    return bar;
}

void RibbonBar::addTab(const QString& name, QToolBar* toolbar) {
    toolbar->setIconSize(QSize(24, 24));
    toolbar->setToolButtonStyle(Qt::ToolButtonIconOnly);
    toolbar->setMovable(false);

    auto* wrapper = new QWidget(this);
    auto* wrapperLayout = new QHBoxLayout(wrapper);
    wrapperLayout->setContentsMargins(0, 0, 0, 0);
    wrapperLayout->setSpacing(0);
    wrapperLayout->addWidget(toolbar);
    wrapperLayout->addStretch();

    m_stack->addWidget(wrapper);
    m_tabBar->addTab(name);
}

void RibbonBar::setCurrentTab(int index) {
    if (index >= 0 && index < m_tabBar->count()) {
        m_tabBar->setCurrentIndex(index);
    }
}

void RibbonBar::setCurrentTab(const QString& name) {
    for (int i = 0; i < m_tabBar->count(); ++i) {
        if (m_tabBar->tabText(i) == name) {
            m_tabBar->setCurrentIndex(i);
            return;
        }
    }
}

void RibbonBar::onTabChanged(int index) {
    m_stack->setCurrentIndex(index);
}

}  // namespace hz::ui
