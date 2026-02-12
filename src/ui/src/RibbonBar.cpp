#include "horizon/ui/RibbonBar.h"

#include <QHBoxLayout>
#include <QStackedWidget>
#include <QTabBar>
#include <QToolBar>
#include <QVBoxLayout>

namespace hz::ui {

RibbonBar::RibbonBar(QWidget* parent) : QWidget(parent) {
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(0);

    // Tab bar for switching between categories.
    m_tabBar = new QTabBar(this);
    m_tabBar->setDocumentMode(true);
    m_tabBar->setExpanding(false);
    layout->addWidget(m_tabBar);

    // Stacked widget holds one toolbar per tab.
    m_stack = new QStackedWidget(this);
    layout->addWidget(m_stack);

    connect(m_tabBar, &QTabBar::currentChanged, this, &RibbonBar::onTabChanged);

    // Keep the ribbon compact: tab bar (~25px) + toolbar (~35px) = ~60px.
    setMinimumHeight(50);
    setMaximumHeight(60);
}

void RibbonBar::addTab(const QString& name, QToolBar* toolbar) {
    // Configure toolbar appearance.
    toolbar->setIconSize(QSize(24, 24));
    toolbar->setToolButtonStyle(Qt::ToolButtonIconOnly);
    toolbar->setMovable(false);

    // Wrap the toolbar in a plain widget so it fills the stacked area.
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
