#pragma once

#include <QDockWidget>

class QTreeWidget;
class QTreeWidgetItem;
class QPushButton;

namespace hz::ui {

class MainWindow;

class LayerPanel : public QDockWidget {
    Q_OBJECT

public:
    explicit LayerPanel(MainWindow* mainWindow, QWidget* parent = nullptr);

    void refresh();

private slots:
    void onAddLayer();
    void onDeleteLayer();
    void onItemDoubleClicked(QTreeWidgetItem* item, int column);
    void onItemChanged(QTreeWidgetItem* item, int column);
    void onColorClicked();

private:
    void createWidgets();

    MainWindow* m_mainWindow;
    QTreeWidget* m_tree = nullptr;
    QPushButton* m_addBtn = nullptr;
    QPushButton* m_deleteBtn = nullptr;
    bool m_refreshing = false;
};

}  // namespace hz::ui
