#pragma once

#include <QMainWindow>
#include <functional>
#include <memory>
#include <vector>

#include "horizon/document/AssemblyDocument.h"
#include "horizon/document/Document.h"
#include "horizon/document/DocumentManager.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/math/Vec2.h"
#include "horizon/ui/Clipboard.h"

class QCloseEvent;
class QTimer;
class QLabel;
class QTabBar;

namespace hz::ui {

class ViewportWidget;
class ToolManager;
class PropertyPanel;
class LayerPanel;
class RibbonBar;
class FeatureTreePanel;
class RecoveryManager;

/// The main application window for Horizon CAD.
class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override;

    /// The document shown in the active tab (the blank backing document for
    /// an assembly tab).
    doc::Document* activeDocument() const { return m_document.get(); }

    /// The assembly shown in the active tab, or null when it is not one.
    doc::AssemblyDocument* activeAssembly() const { return m_assembly.get(); }

    /// Autosave and crash recovery for this window's documents.
    const RecoveryManager& recovery() const { return *m_recovery; }

public slots:
    /// Write a recovery snapshot of every modified document that changed since
    /// its last one, and drop the snapshots of documents no longer modified.
    /// Runs on a timer (the "autosave/intervalSeconds" setting, default 120).
    void autosave();

    /// If an earlier session crashed, offer to reopen the documents it left
    /// modified. Called once the window is on screen.
    void offerRecovery();

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
    void onShell();
    void onDraft();

    void onLinearPattern();
    void onCircularPattern();

    void onCreateBlock();
    void onInsertBlock();
    void onExplode();

    void onMouseMoved(const hz::math::Vec2& worldPos);
    void onSelectionChanged();

    void onFeatureDoubleClicked(int featureIndex);
    void onFeatureReordered(int fromIndex, int toIndex);
    void onFeatureDeleteRequested(int featureIndex);
    void onFeatureSuppressRequested(int featureIndex, bool suppress);
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
        /// Identifies this document's autosave snapshot.
        quint64 recoveryKey = 0;
        /// Changed since its last snapshot (or never snapshotted).
        bool snapshotStale = true;
        /// Reopened from a crashed session and not saved since.
        bool recovered = false;
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
    /// Make the assembly edit just done (from `before`) one undo step on the
    /// active tab. `wasDirty` is the assembly's flag before the edit.
    void recordAssemblyEdit(doc::AssemblyState before, bool wasDirty, const QString& description);
    /// The feature at a panel row of the active part, or null.
    const doc::Feature* featureAt(int featureIndex) const;
    /// Undo (or redo) on the active document, and rebuild what it changed.
    void undoOrRedo(bool undo);
    /// Tab captions and the window title show which documents are modified.
    void refreshModifiedIndicators();
    /// If the tab's document is modified, focus it and ask Save / Discard /
    /// Cancel. Returns false when the user cancels or the save fails.
    bool maybeSaveTab(int index);

    /// Ask for a body-creating feature's size (`value`) and how its body
    /// combines with the part. False when cancelled.
    bool askForBodyFeature(const QString& title, const QString& valueLabel, double& value,
                           double min, double max, int decimals, doc::BodyOperation& operation);
    /// Add a feature at the end of the history, as one undoable step, and
    /// rebuild. A feature that fails there itself (a Cut that would leave
    /// nothing, a fillet too big for its edge) is not added: the part, and the
    /// undo history, stay as they were, and false is returned with the reason
    /// in the status bar. `wrapperSketch` is a profile sketch made for the
    /// feature, added and undone with it.
    bool addModelFeature(std::unique_ptr<doc::Feature> feature, const QString& verb,
                         const std::shared_ptr<doc::Sketch>& wrapperSketch = nullptr);

    /// False, with a word in the status bar, when the active tab is not a
    /// part (`verb` names the command).
    bool requirePart(const QString& verb);
    /// The part's solid, or null — with a word in the status bar — when there
    /// is no part or it has no body yet.
    const topo::Solid* requireBody(const QString& verb);
    /// Join once the part has a body; a new body for the first.
    doc::BodyOperation proposedOperation() const;

    struct PrimitiveField {
        QString label;
        double value;
        double min;
    };
    /// Ask for a primitive's sizes (`fields`) and body operation, then add it.
    void addPrimitive(
        const QString& verb, const std::vector<PrimitiveField>& fields,
        const std::function<std::unique_ptr<doc::PrimitiveFeature>(const std::vector<double>&)>&
            make);
    /// Add a feature combining the part's bodies.
    void combineBodies(model::BooleanType type, const QString& verb);
    /// Ask for edges and a size, then add a fillet (or chamfer) on them.
    void addEdgeFeature(bool fillet);

    /// Route a document's change notifications to the markers and autosave.
    void watchDocument(const std::shared_ptr<doc::Document>& document);
    /// Drop the tab's snapshot once its document is saved or closed.
    void forgetSnapshot(DocTab& tab);

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
    std::unique_ptr<RecoveryManager> m_recovery;
    QTimer* m_autosaveTimer = nullptr;
    quint64 m_nextRecoveryKey = 1;
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
