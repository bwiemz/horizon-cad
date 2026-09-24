#pragma once

#include <QMainWindow>
#include <memory>
#include <vector>

#include "horizon/document/Document.h"
#include "horizon/document/DocumentManager.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/math/Vec2.h"
#include "horizon/ui/Clipboard.h"

class QCloseEvent;
class QLabel;
class QTabBar;

namespace hz::ui {

class ViewportWidget;
class ToolManager;
class PropertyPanel;
class LayerPanel;
class RibbonBar;
class FeatureTreePanel;

/// The main application window for Horizon CAD.
class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override;

    /// The document shown in the active tab (the blank backing document for
    /// an assembly tab).
    doc::Document* activeDocument() const { return m_document.get(); }

protected:
    /// Offers to save every modified document; ignores the close if the user
    /// cancels or a save fails.
    void closeEvent(QCloseEvent* event) override;

private slots:
    void onCommandPalette();
    void onNewFile();
    void onNewPart();
    void onNewAssembly();
    void onOpenFile();
    void onSaveFile();
    void onSaveFileAs();
    void onInsertComponent();
    void onAddMate();
    void onCheckInterference();
    void onTabChanged(int index);
    void onTabCloseRequested(int index);

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
    void onChamferTool();
    void onBreakTool();
    void onExtendTool();
    void onStretchTool();
    void onPolylineEditTool();
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

    void onGroupEntities();
    void onUngroupEntities();

    void onPrimitiveBox();
    void onPrimitiveCylinder();
    void onPrimitiveSphere();
    void onPrimitiveCone();
    void onPrimitiveTorus();

    void onExtrudeSketch();
    void onRevolveSketch();

    void onBooleanUnion();
    void onBooleanSubtract();
    void onBooleanIntersect();

    void onFillet();
    void onChamfer();

    void onCreateBlock();
    void onInsertBlock();
    void onExplode();

    void onMouseMoved(const hz::math::Vec2& worldPos);
    void onSelectionChanged();

    void onFeatureDoubleClicked(int featureIndex);
    void onFeatureReordered(int fromIndex, int toIndex);
    void onRollbackChanged(int newIndex);

private:
    /// One open document tab. Part/drawing tabs hold `document`; assembly
    /// tabs additionally hold `assembly` (with `document` acting as a blank
    /// backing document so the shared viewport always has one).
    struct DocTab {
        std::shared_ptr<doc::Document> document;
        std::shared_ptr<doc::AssemblyDocument> assembly;
        /// Tab caption without the modified marker.
        QString title;
    };

    void createMenus();
    void createRibbonBar();
    void createStatusBar();
    void registerTools();
    void updateStatusBar();
    void rebuildFeatureTree();

    DocTab* activeTab();
    bool saveActiveDocument();
    std::shared_ptr<doc::Sketch> resolveProfileSketch(bool& createdWrapper);
    bool solveAssemblyMates(doc::AssemblyDocument& asmDoc);
    int addDocumentTab(std::shared_ptr<doc::Document> document,
                       std::shared_ptr<doc::AssemblyDocument> assembly, const QString& title);
    void activateTabDocument();
    void rebuildScene();
    void refreshAllPanels();
    void updateWindowTitle();

    bool isTabModified(const DocTab& tab) const;
    /// Tab captions and the window title show which documents are modified.
    void refreshModifiedIndicators();
    /// If the tab's document is modified, focus it and ask Save / Discard /
    /// Cancel. Returns false when the user cancels or the save fails.
    bool maybeSaveTab(int index);

    /// Tell the user a file operation failed and why, and log it.
    /// `summary` is e.g. "Could not open".
    void reportFileError(const QString& summary, const std::string& path,
                         const std::string& reason);
    QString tabTitleForPath(const std::string& path, const QString& fallback) const;

    ViewportWidget* m_viewport = nullptr;
    QTabBar* m_tabBar = nullptr;
    std::unique_ptr<ToolManager> m_toolManager;
    doc::DocumentManager m_docManager;
    /// Why the document manager's last part/assembly load failed.
    std::string m_lastLoadError;
    std::vector<DocTab> m_tabs;
    std::shared_ptr<doc::Document> m_document;
    std::shared_ptr<doc::AssemblyDocument> m_assembly;
    Clipboard m_clipboard;
    PropertyPanel* m_propertyPanel = nullptr;
    LayerPanel* m_layerPanel = nullptr;
    RibbonBar* m_ribbonBar = nullptr;
    FeatureTreePanel* m_featureTreePanel = nullptr;

    // Status bar widgets
    QLabel* m_statusCoords = nullptr;
    QLabel* m_statusPrompt = nullptr;
    QLabel* m_statusSnap = nullptr;
    QLabel* m_statusSelection = nullptr;
    QLabel* m_statusTool = nullptr;
};

}  // namespace hz::ui
