#pragma once

#include <QMainWindow>

#include "horizon/math/Vec2.h"
#include "horizon/document/Document.h"
#include "horizon/ui/Clipboard.h"

#include <memory>

class QLabel;

namespace hz::ui {

class ViewportWidget;
class ToolManager;
class PropertyPanel;
class LayerPanel;
class RibbonBar;

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
    void onBreakTool();
    void onExtendTool();
    void onRectangularArray();
    void onPolarArray();

    void onLinearDimTool();
    void onRadialDimTool();
    void onAngularDimTool();
    void onLeaderTool();
    void onTextTool();
    void onSplineTool();
    void onHatchTool();
    void onEllipseTool();

    void onMeasureDistanceTool();
    void onMeasureAngleTool();
    void onMeasureAreaTool();

    void onConstraintCoincident();
    void onConstraintHorizontal();
    void onConstraintVertical();
    void onConstraintPerpendicular();
    void onConstraintParallel();
    void onConstraintTangent();
    void onConstraintEqual();
    void onConstraintFixed();
    void onConstraintDistance();
    void onConstraintAngle();

    void onCreateBlock();
    void onInsertBlock();
    void onExplode();

    void onMouseMoved(const hz::math::Vec2& worldPos);
    void onSelectionChanged();

private:
    void createMenus();
    void createRibbonBar();
    void createStatusBar();
    void registerTools();
    void updateStatusBar();

    ViewportWidget* m_viewport = nullptr;
    std::unique_ptr<ToolManager> m_toolManager;
    std::unique_ptr<doc::Document> m_document;
    Clipboard m_clipboard;
    PropertyPanel* m_propertyPanel = nullptr;
    LayerPanel* m_layerPanel = nullptr;
    RibbonBar* m_ribbonBar = nullptr;

    // Status bar widgets
    QLabel* m_statusCoords = nullptr;
    QLabel* m_statusPrompt = nullptr;
    QLabel* m_statusSnap = nullptr;
    QLabel* m_statusSelection = nullptr;
    QLabel* m_statusTool = nullptr;
};

}  // namespace hz::ui
