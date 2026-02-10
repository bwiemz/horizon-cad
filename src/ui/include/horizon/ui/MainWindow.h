#pragma once

#include <QMainWindow>

#include "horizon/math/Vec2.h"
#include "horizon/document/Document.h"

#include <memory>

namespace hz::ui {

class ViewportWidget;
class ToolManager;

/// The main application window for Horizon CAD.
class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override;

private slots:
    void onNewFile();
    void onOpenFile();
    void onSaveFile();
    void onSaveFileAs();

    void onUndo();
    void onRedo();

    void onViewFront();
    void onViewTop();
    void onViewRight();
    void onViewIsometric();
    void onFitAll();

    void onSelectTool();
    void onLineTool();
    void onCircleTool();

    void onMouseMoved(const hz::math::Vec2& worldPos);

private:
    void createMenus();
    void createToolBar();
    void createStatusBar();
    void registerTools();

    ViewportWidget* m_viewport = nullptr;
    std::unique_ptr<ToolManager> m_toolManager;
    std::unique_ptr<doc::Document> m_document;
};

}  // namespace hz::ui
