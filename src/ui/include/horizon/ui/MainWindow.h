#pragma once

#include <QMainWindow>

#include "horizon/math/Vec2.h"
#include "horizon/document/Document.h"
#include "horizon/ui/Clipboard.h"

#include <memory>

namespace hz::ui {

class ViewportWidget;
class ToolManager;
class PropertyPanel;
class LayerPanel;

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
    void onDuplicate();
    void onCopy();
    void onCut();
    void onPaste();

    void onViewFront();
    void onViewTop();
    void onViewRight();
    void onViewIsometric();
    void onFitAll();

    void onSelectTool();
    void onLineTool();
    void onCircleTool();
    void onArcTool();
    void onRectangleTool();
    void onPolylineTool();
    void onMoveTool();
    void onOffsetTool();
    void onMirrorTool();
    void onRotateTool();
    void onScaleTool();
    void onTrimTool();
    void onFilletTool();
    void onRectangularArray();
    void onPolarArray();

    void onLinearDimTool();
    void onRadialDimTool();
    void onAngularDimTool();
    void onLeaderTool();

    void onMouseMoved(const hz::math::Vec2& worldPos);
    void onSelectionChanged();

private:
    void createMenus();
    void createToolBar();
    void createStatusBar();
    void registerTools();

    ViewportWidget* m_viewport = nullptr;
    std::unique_ptr<ToolManager> m_toolManager;
    std::unique_ptr<doc::Document> m_document;
    Clipboard m_clipboard;
    PropertyPanel* m_propertyPanel = nullptr;
    LayerPanel* m_layerPanel = nullptr;
};

}  // namespace hz::ui
