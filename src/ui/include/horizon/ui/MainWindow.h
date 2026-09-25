#pragma once

#include <QElapsedTimer>
#include <QMainWindow>
#include <QPointer>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <vector>

#include "horizon/document/AssemblyDocument.h"
#include "horizon/document/Document.h"
#include "horizon/document/DocumentManager.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/fileio/ImportReport.h"
#include "horizon/fileio/StepFormat.h"
#include "horizon/geometry/MeshData.h"
#include "horizon/math/Vec2.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/topology/Solid.h"
#include "horizon/ui/BackgroundTask.h"
#include "horizon/ui/Clipboard.h"
#include "horizon/ui/PendingAdds.h"
#include "horizon/ui/Preferences.h"
#include "horizon/ui/RebuildJob.h"
#include "horizon/ui/WorkbenchHost.h"

class QCloseEvent;
class QComboBox;
class QTimer;
class QLabel;
class QTabBar;
class QMenu;
class QProgressBar;
class QToolButton;
class QMessageBox;

namespace hz::doc {
class Command;
}  // namespace hz::doc

namespace hz::ui {

class ViewportWidget;
class ToolManager;
class PropertyPanel;
class LayerPanel;
class RibbonBar;
class FeatureTreePanel;
class RecoveryManager;
class AssemblyTreePanel;
class AssemblyWorkbench;
class DrawingWorkbench;
class FeatureForm;

/// The main application window for Horizon CAD.
class MainWindow : public QMainWindow, private WorkbenchHost {
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

    /// Open @p fileName (a drawing, part, assembly or DXF) in a tab of its
    /// own, or show its tab if it is already open, and remember it in File ▸
    /// Open Recent. Reports a failure to the user and returns false.
    bool openPath(const QString& fileName) override;

    /// Open each of @p fileNames, as from the command line.
    void openFiles(const QStringList& fileNames);

    /// Put @p prefs into effect: the autosave interval, the grid snap and the
    /// snap reach. The display unit is read where lengths are shown.
    void applyPreferences(const Preferences& prefs);

    /// When long work — a model rebuild, a STEP import, an interference
    /// check — runs on a worker thread: when it would freeze the window
    /// (Auto), always, or never.
    enum class RebuildMode { Auto, Always, Never };
    void setRebuildMode(RebuildMode mode) { m_rebuildMode = mode; }

    /// A rebuild is running on a worker.
    bool rebuildRunning() const { return m_rebuildJob != nullptr; }
    /// Anything is running on a worker.
    bool backgroundWorkRunning() const;

    /// What goes to a worker in Auto: a rebuild after one that took longer
    /// than this, a STEP file at least this big, an interference check of at
    /// least this many faces.
    static constexpr qint64 kWorkerRebuildMs = 300;
    /// A part's build time before it was first built here (opened, not
    /// built): Auto builds it on a worker, as a slow one.
    static constexpr qint64 kBuildTimeUnknown = -1;

    /// How many times a model has been tessellated for the view (for tests).
    std::uint64_t tessellations() const { return m_tessellations; }
    static constexpr qint64 kWorkerImportBytes = 1'000'000;  ///< and a file opened
    /// Mass Properties measures the ideal on a worker, in Auto, for a part
    /// with at least this many faces on curved surfaces.
    static constexpr std::size_t kWorkerIdealFaces = 100;

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
    void onPreferences();
    /// Edit ▸ Document Units: the unit the active document shows and takes
    /// lengths in (Phase 154), as one undo step.
    void onDocumentUnits();
    void onAbout();
    void onSaveFile();
    void onSaveFileAs();
    void onImportStep();
    /// A STEP file kept as an assembly: its parts as part files (Phase 153).
    void onImportStepAssembly();
    void onImportDxf();
    void onExportStep();
    void onExportStl();
    void onExportGltf();
    void onExportDxf();
    /// Plot the drawing to a PDF (@p pdf) or an SVG file, on a paper, at a
    /// scale, chosen in a form.
    void onExportPlot(bool pdf);
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
    /// Dimension > Style: the drawing's dimension style, in a form.
    void onDimensionStyle();
    /// Make the registered tool called @p name the active one.
    void activateTool(const std::string& name);
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

    // Sketches (Phase 131): a new one on a principal plane (0 XY, 1 XZ,
    // 2 YZ), a face or a datum plane; editing one, and finishing it.
    void onNewSketchOnPlane(int which);
    void onNewSketchOnFace();
    void onNewSketchOnDatum();
    void onEditSketch();
    void onFinishSketch();

    // Loft, Sweep and datums (Phase 133).
    void onLoft();
    void onSweep();
    void onMassProperties();
    void onSectionPlane();
    void onDatumPlane();
    void onDatumAxis();
    void onDatumPoint();

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
        /// Times it has been recovered without being saved since, this time
        /// included (RecoveryManager::Entry::recoveries, plus one).
        int recoveries = 0;
        /// How long its last model rebuild took, in milliseconds.
        qint64 lastBuildMs = 0;  ///< kBuildTimeUnknown before its first build here
        /// Its model's mesh, tessellated from its build number meshBuild:
        /// the scene is rebuilt for more than a new model (a sketch opened or
        /// closed, the tab shown again), and tessellates only a new one.
        std::shared_ptr<const geo::MeshData> mesh{};
        std::uint64_t meshBuild = 0;
        /// Its model is out of date: a rebuild for it was dropped while
        /// another tab was shown.
        bool modelStale = false;
    };

    void createMenus();
    void createRibbonBar();
    void createStatusBar();
    /// A drafting aid was switched: keep it, with ortho and polar tracking
    /// never on together, and put it into effect.
    void onDraftingAidToggled(QAction* changed);
    /// The active tool's prompt, with what is typed for it.
    QString toolPrompt() const;
    void registerTools();
    void updateStatusBar();
    void rebuildFeatureTree();

    DocTab* activeTab();
    bool saveActiveDocument();

    /// File ▸ Open Recent, rebuilt each time it opens.
    void rebuildRecentMenu();

    /// Show what the active document's last build gave: failures in the
    /// feature tree panel, and the model in the viewport.
    void showBuildResult();
    /// Rebuild the active document on a worker (see RebuildJob). One runs at
    /// a time: asked again, the running one stops and a new one starts from
    /// the document as it is by then.
    void startRebuild();
    /// A worker's rebuild is done: apply it if the document is where it was.
    void onRebuildFinished();
    void updateRebuildProgress();
    /// Show the progress bar and Cancel while anything runs on a worker.
    void updateBusyIndicator();

    /// A STEP file read — on the GUI thread or a worker: into solids, or,
    /// to be kept as an assembly at `assemblyPath`, into its parts and their
    /// placements (Phase 153).
    struct StepLoad {
        std::vector<std::unique_ptr<topo::Solid>> solids;
        io::StepAssembly assembly;
        std::string assemblyPath;
        io::ImportReport report;
        std::string error;  ///< why nothing was read (lastError is per thread)
    };
    static StepLoad loadStep(const std::string& path, const std::string& assemblyPath = {},
                             const std::atomic<bool>* cancelled = nullptr);
    /// Read @p fileName, on a worker when it is large; into a new part, or
    /// kept as an assembly at @p assemblyPath when one is given.
    void startStepImport(const QString& fileName, const QString& assemblyPath);

    /// A drawing or part file read into a document of its own, on a worker
    /// when the file is large (openOnWorker).
    struct FileOpen {
        std::shared_ptr<doc::Document> document;  ///< null when it could not be read
        io::ImportReport report;
        std::string error;
    };
    static FileOpen readFile(const std::string& path, bool drawing);
    bool openOnWorker(const QString& fileName) const;
    /// Read @p fileName on a worker; its tab is added when it is read.
    bool startOpen(const QString& fileName, bool drawing);
    void onOpenFinished();
    /// Show @p document, just opened from @p fileName: in the tab already
    /// showing it if there is one, else a new tab; what could not be read of
    /// it; and in the recent files.
    void showOpened(std::shared_ptr<doc::Document> document, const QString& fileName,
                    const QString& fallbackTitle, io::ImportReport report);
    void finishStepImport(const QString& fileName, StepLoad load);
    /// @p load's parts written as part files beside its assembly, and the
    /// assembly opened (Phase 153).
    void finishStepAssemblyImport(const QString& fileName, StepLoad load);
    /// The active assembly written as a STEP assembly (Phase 153).
    void exportAssemblyStep();
    /// What a STEP export could not write as asked: the curved faces kept in
    /// facets, and the components left out (@p unread, whose parts could
    /// not be read, and those in @p report).
    void showStepExportReport(const io::StepWriteReport& report,
                              const std::vector<std::string>& unread = {});
    void onImportFinished();
    void onMassPropertiesFinished();

    /// The window's size and position and where its docks are, kept across
    /// sessions.
    void saveWindowLayout() const;
    void restoreWindowLayout();
    std::shared_ptr<doc::Sketch> resolveProfileSketch(bool& createdWrapper);
    /// Add a sketch on @p plane (one undo step) and start editing it.
    void newSketchOn(const draft::SketchPlane& plane, const QString& where);
    /// Edit @p sketch (null: stop editing), and show it.
    void editSketch(const std::shared_ptr<doc::Sketch>& sketch);
    /// Show what the document is editing: the sketch view, with the solids
    /// placed in its frame, or the model; and the Finish Sketch action and
    /// the sketch list as they are.
    void syncSketchView();
    void refreshSketchList();
    /// For every watched file changed on disk: its tabs read again, then
    /// the components placing it refreshed.
    void pollPartFiles();
    /// Tabs showing @p path, changed on disk by another program: one without
    /// unsaved changes is read again; one with them asks whether to read it
    /// again, losing them, or keep its own. An assembly's tab says so only.
    /// True when a reading took over refreshing the components placing it.
    bool reloadTabsOf(const std::string& path);
    /// Read tab @p index's file again, into a new document in place of its
    /// own, and refresh the components placing it: here, or on a worker for
    /// a file openOnWorker() would read there, when the reading is done.
    /// @p asked: its unsaved changes were given up.
    void reloadTab(size_t index, bool asked);
    /// Put what was read of @p old's file in its tab's place. Refused, and
    /// said why, when it could not be read, or its tab has gone or has been
    /// changed meanwhile without @p asked.
    bool replaceTabDocument(const std::shared_ptr<doc::Document>& old, FileOpen read, bool asked);
    void onReloadFinished();
    // --- WorkbenchHost: what the workbenches ask of the window ---
    QWidget* dialogParent() override { return this; }
    doc::Document* currentDocument() override { return m_document.get(); }
    std::shared_ptr<doc::AssemblyDocument> currentAssembly() override { return m_assembly; }
    std::vector<std::shared_ptr<doc::AssemblyDocument>> openAssemblies() override;
    doc::DocumentManager& documents() override { return m_docManager; }
    ViewportWidget& viewport() override { return *m_viewport; }
    void showStatus(const QString& message, int timeoutMs) override;
    QString currentStatus() override;
    void setPrompt(const QString& text) override;
    bool onWorker(bool large) override;
    void backgroundWorkChanged() override { updateBusyIndicator(); }
    void addTab(std::shared_ptr<doc::Document> document, const QString& title) override;
    void runTool(std::unique_ptr<Tool> tool) override;
    void endTool() override;

    int addDocumentTab(std::shared_ptr<doc::Document> document,
                       std::shared_ptr<doc::AssemblyDocument> assembly, const QString& title);
    void activateTabDocument();
    void rebuildScene() override;
    void refreshAllPanels();
    void updateWindowTitle();

    bool isTabModified(const DocTab& tab) const;
    /// Tell the user what reading `file` left out or changed, if anything —
    /// before a save could drop it for good.
    void showImportReport(const QString& file, const io::ImportReport& report);
    /// The part's solid for an export, or null with a word in the status bar.
    const topo::Solid* solidToExport(const QString& format);
    /// Ask where to export; empty when cancelled.
    QString askExportPath(const QString& format, const QString& filter, const QString& suffix);
    /// The feature at a panel row of the active part, or null.
    const doc::Feature* featureAt(int featureIndex) const;
    /// Undo (or redo) on the active document, and rebuild what it changed.
    void undoOrRedo(bool undo);
    /// Tab captions and the window title show which documents are modified.
    void refreshModifiedIndicators() override;
    /// The file of the part at @p path changed (saved, read again, changed
    /// by another program, or its edits discarded): the assemblies placing
    /// it and the drawings of it follow. @p report as refreshComponentsOf.
    void refreshUsersOf(const std::string& path, bool report = true);
    /// If the tab's document is modified, focus it and ask Save / Discard /
    /// Cancel. Returns false when the user cancels or the save fails.
    bool maybeSaveTab(int index);

    /// Ask for a body-creating feature's size (`value`) and how its body
    /// combines with the part. False when cancelled.
    bool askForBodyFeature(const QString& title, const QString& valueLabel, double& value,
                           double min, double max, int decimals, doc::BodyOperation& operation);
    /// Add a feature at the end of the history, as one undoable step, and
    /// build the model once, on a worker when builds are slow. A feature
    /// that fails there itself (a Cut that would leave nothing, a fillet too
    /// big for its edge) is refused when that build is shown: the step is
    /// withdrawn, so the part and the undo history are as they were, with
    /// the reason in the status bar. Returns false when it was refused at
    /// once (a build here); true when it was added, or its build is still
    /// running. `wrapperSketch` is a profile sketch made for the feature,
    /// added and undone with it.
    bool addModelFeature(std::unique_ptr<doc::Feature> feature, const QString& verb,
                         const std::shared_ptr<doc::Sketch>& wrapperSketch = nullptr);
    /// A build of @p document is shown or applied: if it was the one made
    /// for a feature just added, and nothing changed since, and the feature
    /// failed itself, withdraw it and build the part as it was. Returns
    /// whether it did.
    bool settlePendingAdd(doc::Document& document);

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
    /// Show in the status bar that autosave is off (it could not start) or
    /// that its last write failed, with @p failure saying why; else nothing.
    /// Autosave turned off in the preferences shows nothing either.
    void showAutosaveState(const QString& failure = {});

    /// Tell the user a file operation failed and why, and log it.
    /// `summary` is e.g. "Could not open".
    void reportFileError(const QString& summary, const std::string& path,
                         const std::string& reason) override;
    QString tabTitleForPath(const std::string& path, const QString& fallback) const;

    ViewportWidget* m_viewport = nullptr;
    QTabBar* m_tabBar = nullptr;
    std::unique_ptr<ToolManager> m_toolManager;
    doc::DocumentManager m_docManager;
    /// Why the document manager's last part/assembly load failed.
    std::string m_lastLoadError;
    /// What the last native load left out (it skips malformed items).
    io::ImportReport m_lastLoadReport;
    std::unique_ptr<RecoveryManager> m_recovery;
    QTimer* m_autosaveTimer = nullptr;
    QTimer* m_partWatch = nullptr;  ///< polls the files open or placed for changes
    quint64 m_nextRecoveryKey = 1;
    std::vector<DocTab> m_tabs;
    std::shared_ptr<doc::Document> m_document;
    std::shared_ptr<doc::AssemblyDocument> m_assembly;
    Clipboard m_clipboard;
    PropertyPanel* m_propertyPanel = nullptr;
    LayerPanel* m_layerPanel = nullptr;
    RibbonBar* m_ribbonBar = nullptr;
    FeatureTreePanel* m_featureTreePanel = nullptr;
    AssemblyTreePanel* m_assemblyTreePanel = nullptr;  ///< tabbed with the feature tree
    QAction* m_finishSketchAction = nullptr;
    QMenu* m_modelMenu = nullptr;
    QMenu* m_viewMenu = nullptr;
    QAction* m_viewFitAllPlaceholder = nullptr;
    /// Put the ribbon's modelling commands in the Model menu (so the command
    /// palette, which reads the menus, finds them) and its Fit All in View.
    void completeMenusFromRibbon();
    /// The sketch Extrude and Revolve take when none is being edited: the
    /// one chosen in the sketch list, or last made or finished. Sketch ids
    /// are unique across documents, so another document's is simply not found.
    uint64_t m_profileSketchId = 0;
    QMenu* m_recentMenu = nullptr;

    // Model rebuilds on a worker thread (Phase 114).
    RebuildMode m_rebuildMode = RebuildMode::Auto;
    std::unique_ptr<RebuildJob> m_rebuildJob;
    std::shared_ptr<doc::Document> m_rebuildDocument;  ///< what the job builds; kept alive
    bool m_rebuildAgain = false;                       ///< the document changed while the job ran
    PendingAdds m_pendingAdds;  ///< features added whose builds are yet to be seen
    bool m_addRefused = false;  ///< the last add was refused at once
    std::uint64_t m_tessellations = 0;
    QElapsedTimer m_rebuildClock;  ///< since the job started (monotonic)
    QProgressBar* m_rebuildProgress = nullptr;
    QToolButton* m_rebuildCancel = nullptr;
    QTimer* m_rebuildPoll = nullptr;
    std::unique_ptr<BackgroundTask<StepLoad>> m_importTask;
    QString m_importFile;
    std::unique_ptr<BackgroundTask<FileOpen>> m_openTask;
    /// A tab's file read again on a worker, the document it replaces and
    /// whether that one's changes were given up. Files changed meanwhile
    /// wait, each with the document whose changes were given up, if one's
    /// were: that answer holds for that document only.
    std::unique_ptr<BackgroundTask<FileOpen>> m_reloadTask;
    std::shared_ptr<doc::Document> m_reloadDocument;
    bool m_reloadAsked = false;
    std::vector<std::pair<std::string, std::weak_ptr<doc::Document>>> m_reloadQueue;
    QString m_openFile;
    bool m_openDrawing = false;
    /// The assembly commands (Phase 146); they reach the window through
    /// WorkbenchHost.
    std::unique_ptr<AssemblyWorkbench> m_assemblies;
    /// The drawing sheets made from parts (Phase 148), likewise.
    std::unique_ptr<DrawingWorkbench> m_drawings;
    /// The ideal mass properties being measured, the dialog waiting for them,
    /// and its text given them (null: still measuring; a reason: none).
    std::unique_ptr<BackgroundTask<model::IdealMassProperties>> m_massTask;
    QPointer<QMessageBox> m_massBox;
    std::function<QString(const model::IdealMassProperties*, const QString&)> m_massText;

    // Status bar widgets
    QLabel* m_statusCoords = nullptr;
    QLabel* m_statusPrompt = nullptr;
    QAction* m_actObjectSnap = nullptr;
    QAction* m_actGridSnap = nullptr;
    QAction* m_actOrtho = nullptr;
    QAction* m_actPolar = nullptr;
    QLabel* m_statusSelection = nullptr;
    QLabel* m_statusTool = nullptr;
    QLabel* m_autosaveWarning = nullptr;
};

}  // namespace hz::ui
