#include "horizon/ui/MainWindow.h"

#include <spdlog/spdlog.h>

#include <QAbstractButton>
#include <QAction>
#include <QActionGroup>
#include <QApplication>
#include <QCloseEvent>
#include <QComboBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDir>
#include <QDoubleSpinBox>
#include <QElapsedTimer>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QInputDialog>
#include <QKeySequence>
#include <QLabel>
#include <QLocale>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QProgressBar>
#include <QSettings>
#include <QSignalBlocker>
#include <QStandardPaths>
#include <QStatusBar>
#include <QSysInfo>
#include <QTabBar>
#include <QTimer>
#include <QToolBar>
#include <QToolButton>
#include <QVBoxLayout>
#include <filesystem>
#include <functional>
#include <map>
#include <numbers>
#include <optional>
#include <utility>

#include "horizon/Revision.h"
#include "horizon/Version.h"
#include "horizon/document/Commands.h"
#include "horizon/document/ModelCommands.h"
#include "horizon/document/UndoStack.h"
#include "horizon/drafting/DraftBlockRef.h"
#include "horizon/fileio/DxfFormat.h"
#include "horizon/fileio/GltfExport.h"
#include "horizon/fileio/NativeFormat.h"
#include "horizon/fileio/StepFormat.h"
#include "horizon/fileio/StlExport.h"
#include "horizon/fileio/SvgExport.h"
#include "horizon/math/BoundingBox.h"
#include "horizon/math/MathUtils.h"
#include "horizon/modeling/AssemblySolver.h"
#include "horizon/modeling/BooleanOp.h"
#include "horizon/modeling/Extrude.h"
#include "horizon/modeling/MateGeometry.h"
#include "horizon/modeling/Pattern.h"
#include "horizon/modeling/Revolve.h"
#include "horizon/modeling/SolidTessellator.h"
#include "horizon/render/SceneGraph.h"
#include "horizon/topology/Solid.h"
#include "horizon/ui/AngularDimensionTool.h"
#include "horizon/ui/ArcTool.h"
#include "horizon/ui/BreakTool.h"
#include "horizon/ui/ChainDimensionTool.h"
#include "horizon/ui/ChamferTool.h"
#include "horizon/ui/CircleTool.h"
#include "horizon/ui/Clipboard.h"
#include "horizon/ui/CommandPalette.h"
#include "horizon/ui/ConstraintTool.h"
#include "horizon/ui/EllipseTool.h"
#include "horizon/ui/ExtendTool.h"
#include "horizon/ui/FeatureForm.h"
#include "horizon/ui/FeatureTreePanel.h"
#include "horizon/ui/FilletTool.h"
#include "horizon/ui/HatchTool.h"
#include "horizon/ui/IconGenerator.h"
#include "horizon/ui/InsertBlockDialog.h"
#include "horizon/ui/InsertBlockTool.h"
#include "horizon/ui/LayerPanel.h"
#include "horizon/ui/LeaderTool.h"
#include "horizon/ui/LineTool.h"
#include "horizon/ui/LinearDimensionTool.h"
#include "horizon/ui/LocaleManager.h"
#include "horizon/ui/MeasureAngleTool.h"
#include "horizon/ui/MeasureAreaTool.h"
#include "horizon/ui/MeasureDistanceTool.h"
#include "horizon/ui/MirrorTool.h"
#include "horizon/ui/MoveTool.h"
#include "horizon/ui/OffsetTool.h"
#include "horizon/ui/PasteTool.h"
#include "horizon/ui/PdfExport.h"
#include "horizon/ui/PolarArrayDialog.h"
#include "horizon/ui/PolylineEditTool.h"
#include "horizon/ui/PolylineTool.h"
#include "horizon/ui/PreferencesDialog.h"
#include "horizon/ui/PropertyPanel.h"
#include "horizon/ui/RadialDimensionTool.h"
#include "horizon/ui/RecentFiles.h"
#include "horizon/ui/RecoveryManager.h"
#include "horizon/ui/RectArrayDialog.h"
#include "horizon/ui/RectangleTool.h"
#include "horizon/ui/RibbonBar.h"
#include "horizon/ui/RotateTool.h"
#include "horizon/ui/ScaleTool.h"
#include "horizon/ui/SelectTool.h"
#include "horizon/ui/SplineTool.h"
#include "horizon/ui/StretchTool.h"
#include "horizon/ui/TextTool.h"
#include "horizon/ui/Tool.h"
#include "horizon/ui/ToolManager.h"
#include "horizon/ui/TrimTool.h"
#include "horizon/ui/ViewportWidget.h"

namespace hz::ui {

namespace {
/// saveState() layout version: bump it when docks or toolbars change, so an
/// old saved layout is ignored instead of misplacing them.
constexpr int kWindowStateVersion = 1;
}  // namespace

namespace {

/// A point or direction as the dialogs show it: "(10, 0, 2.5)".
QString formatPoint(const math::Vec3& p) {
    const auto n = [](double v) { return QString::number(std::abs(v) < 5e-10 ? 0.0 : v, 'g', 6); };
    return QStringLiteral("(%1, %2, %3)").arg(n(p.x), n(p.y), n(p.z));
}

/// Edges or faces of the part to choose from in a dialog, until the viewport
/// can pick them: each one's name, and how it is listed ({text, tooltip}).
struct PickList {
    std::vector<topo::TopologyID> ids;
    std::vector<std::pair<QString, QString>> items;
};

/// The part's edges, listed by their end points.
PickList edgesOf(const topo::Solid& solid) {
    PickList list;
    for (const auto& edge : solid.edges()) {
        const topo::HalfEdge* he = edge.halfEdge;
        if (!edge.topoId.isValid() || !he || !he->origin || !he->next || !he->next->origin) {
            continue;
        }
        list.ids.push_back(edge.topoId);
        list.items.emplace_back(formatPoint(he->origin->point) + QStringLiteral(" – ") +
                                    formatPoint(he->next->origin->point),
                                QString::fromStdString(edge.topoId.tag()));
    }
    return list;
}

/// The part's faces, listed by which way they face and where their middle is.
PickList facesOf(const topo::Solid& solid) {
    PickList list;
    for (const auto& face : solid.faces()) {
        if (!face.topoId.isValid() || !face.outerLoop || !face.outerLoop->halfEdge) continue;
        // Newell's normal and the vertex average of the outer loop.
        math::Vec3 normal, centre;
        int count = 0;
        const topo::HalfEdge* start = face.outerLoop->halfEdge;
        const topo::HalfEdge* he = start;
        do {
            if (!he->origin || !he->next || !he->next->origin) break;
            const math::Vec3& a = he->origin->point;
            const math::Vec3& b = he->next->origin->point;
            normal.x += (a.y - b.y) * (a.z + b.z);
            normal.y += (a.z - b.z) * (a.x + b.x);
            normal.z += (a.x - b.x) * (a.y + b.y);
            centre = centre + a;
            ++count;
            he = he->next;
        } while (he && he != start && count < 100000);
        if (count == 0 || normal.length() < 1e-12) continue;
        list.ids.push_back(face.topoId);
        list.items.emplace_back(
            MainWindow::tr("facing %1 at %2")
                .arg(formatPoint(normal.normalized()), formatPoint(centre / count)),
            QString::fromStdString(face.topoId.tag()));
    }
    return list;
}

/// Directions offered for a pull or a pattern: the six axis directions.
const std::vector<std::pair<QString, math::Vec3>>& axisDirections() {
    static const std::vector<std::pair<QString, math::Vec3>> directions = {
        {QStringLiteral("+X"), math::Vec3(1, 0, 0)}, {QStringLiteral("−X"), math::Vec3(-1, 0, 0)},
        {QStringLiteral("+Y"), math::Vec3(0, 1, 0)}, {QStringLiteral("−Y"), math::Vec3(0, -1, 0)},
        {QStringLiteral("+Z"), math::Vec3(0, 0, 1)}, {QStringLiteral("−Z"), math::Vec3(0, 0, -1)},
    };
    return directions;
}

QComboBox* directionChoice(FeatureForm& form, const QString& name, const QString& label,
                           const QString& initial) {
    QStringList names;
    for (const auto& [text, direction] : axisDirections()) names << text;
    auto* combo = form.choice(name, label, names);
    combo->setCurrentText(initial);
    return combo;
}

math::Vec3 chosenDirection(const QComboBox* combo) {
    return axisDirections().at(static_cast<size_t>(combo->currentIndex())).second;
}

}  // namespace

// ---------------------------------------------------------------------------
// Construction / destruction
// ---------------------------------------------------------------------------

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent), m_toolManager(std::make_unique<ToolManager>()) {
    // "[*]" is where Qt shows the modified marker; see updateWindowTitle().
    setWindowTitle("Horizon CAD[*]");
    resize(1280, 800);

    // Wire the document manager to the native file format.
    m_docManager.setPartLoader([this](const std::string& path, doc::Document& doc) {
        m_lastLoadError.clear();
        m_lastLoadReport = {};
        return io::NativeFormat::load(path, doc, &m_lastLoadError, &m_lastLoadReport);
    });
    m_docManager.setMeshLoader(
        [](const std::string& path) { return io::NativeFormat::loadPartMesh(path); });
    m_docManager.setAssemblyLoader([this](const std::string& path, doc::AssemblyDocument& doc) {
        m_lastLoadError.clear();
        m_lastLoadReport = {};
        return io::NativeFormat::loadAssembly(path, doc, &m_lastLoadError, &m_lastLoadReport);
    });

    // Autosave snapshots go to a per-session directory; a crash leaves them
    // for offerRecovery() on the next start.
    m_recovery = std::make_unique<RecoveryManager>(
        QStandardPaths::writableLocation(QStandardPaths::AppLocalDataLocation) + "/recovery");
    m_autosaveTimer = new QTimer(this);
    m_autosaveTimer->setObjectName(QStringLiteral("autosaveTimer"));
    connect(m_autosaveTimer, &QTimer::timeout, this, &MainWindow::autosave);

    // Central area: document tab bar above the shared viewport.
    m_viewport = new ViewportWidget(this);
    m_tabBar = new QTabBar(this);
    m_tabBar->setObjectName(QStringLiteral("documentTabs"));
    m_tabBar->setTabsClosable(true);
    m_tabBar->setMovable(false);
    m_tabBar->setExpanding(false);
    m_tabBar->setDocumentMode(true);

    auto* central = new QWidget(this);
    auto* centralLayout = new QVBoxLayout(central);
    centralLayout->setContentsMargins(0, 0, 0, 0);
    centralLayout->setSpacing(0);
    centralLayout->addWidget(m_tabBar);
    centralLayout->addWidget(m_viewport, 1);
    setCentralWidget(central);

    // Initial empty drawing document. The tab-bar signals are connected
    // AFTER the panels exist (below) — addTab would otherwise fire
    // currentChanged into slots that touch not-yet-created widgets.
    m_document = m_docManager.newDocument(doc::DocumentType::Drawing);
    watchDocument(m_document);
    m_tabs.push_back(DocTab{m_document, nullptr, tr("Drawing 1"), m_nextRecoveryKey++});
    m_tabBar->addTab(tr("Drawing 1"));
    m_viewport->setDocument(m_document.get());

    // Dock panels (must exist before createMenus, which adds toggleViewAction).
    // Each dock is named: restoreState() finds them by name.
    m_propertyPanel = new PropertyPanel(this, this);
    m_propertyPanel->setObjectName(QStringLiteral("PropertyPanel"));
    addDockWidget(Qt::RightDockWidgetArea, m_propertyPanel);

    m_layerPanel = new LayerPanel(this, this);
    m_layerPanel->setObjectName(QStringLiteral("LayerPanel"));
    addDockWidget(Qt::RightDockWidgetArea, m_layerPanel);

    tabifyDockWidget(m_propertyPanel, m_layerPanel);
    m_propertyPanel->raise();

    // Feature tree panel (left dock)
    m_featureTreePanel = new FeatureTreePanel(this);
    addDockWidget(Qt::LeftDockWidgetArea, m_featureTreePanel);

    connect(m_featureTreePanel, &FeatureTreePanel::featureDoubleClicked, this,
            &MainWindow::onFeatureDoubleClicked);
    connect(m_featureTreePanel, &FeatureTreePanel::featureDeleteRequested, this,
            &MainWindow::onFeatureDeleteRequested);
    connect(m_featureTreePanel, &FeatureTreePanel::featureSuppressRequested, this,
            &MainWindow::onFeatureSuppressRequested);
    connect(m_featureTreePanel, &FeatureTreePanel::featureReordered, this,
            &MainWindow::onFeatureReordered);
    connect(m_featureTreePanel, &FeatureTreePanel::rollbackChanged, this,
            &MainWindow::onRollbackChanged);
    connect(m_featureTreePanel, &FeatureTreePanel::createBoxRequested, this,
            &MainWindow::onPrimitiveBox);
    connect(m_featureTreePanel, &FeatureTreePanel::openFileRequested, this,
            &MainWindow::onOpenFile);

    // Build UI chrome.
    createMenus();
    createRibbonBar();
    createStatusBar();
    registerTools();

    // Global command palette (Ctrl+K) — a searchable launcher for every command.
    auto* paletteAct = new QAction(tr("Command Palette…"), this);
    paletteAct->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_K));
    paletteAct->setShortcutContext(Qt::ApplicationShortcut);
    connect(paletteAct, &QAction::triggered, this, &MainWindow::onCommandPalette);
    addAction(paletteAct);

    // Tab switching (connected only now that all panels and the status bar
    // exist — the slots refresh them).
    connect(m_tabBar, &QTabBar::currentChanged, this, &MainWindow::onTabChanged);
    connect(m_tabBar, &QTabBar::tabCloseRequested, this, &MainWindow::onTabCloseRequested);

    // Wire up the status bar coordinate display.
    connect(m_viewport, &ViewportWidget::mouseMoved, this, &MainWindow::onMouseMoved);
    connect(m_viewport, &ViewportWidget::typedInputChanged, this, &MainWindow::updateStatusBar);

    // Wire up selection changes to property panel.
    connect(m_viewport, &ViewportWidget::selectionChanged, this, &MainWindow::onSelectionChanged);

    // Start with the Select tool active.
    onSelectTool();

    applyPreferences(Preferences::current());
    restoreWindowLayout();
}

MainWindow::~MainWindow() {
    // Stop work on workers before anything it could report to is gone.
    m_rebuildJob.reset();
    m_importTask.reset();
    m_interferenceTask.reset();
}

void MainWindow::onCommandPalette() {
    // Gather every leaf command from the menu bar (the menus mirror the ribbon
    // and cover all commands). Recurse into submenus; skip separators.
    QList<QAction*> commands;
    std::function<void(QMenu*)> collect = [&](QMenu* menu) {
        for (QAction* act : menu->actions()) {
            if (act->isSeparator()) continue;
            if (act->menu()) {
                collect(act->menu());
            } else {
                commands.push_back(act);
            }
        }
    };
    for (QAction* top : menuBar()->actions()) {
        if (top->menu()) collect(top->menu());
    }

    CommandPalette palette(commands, this);
    palette.move(x() + (width() - palette.width()) / 2, y() + 120);
    palette.exec();
}

// ---------------------------------------------------------------------------
// Menu bar
// ---------------------------------------------------------------------------

void MainWindow::createMenus() {
    // ---- File ----
    QMenu* fileMenu = menuBar()->addMenu(tr("&File"));

    QAction* newAction = fileMenu->addAction(tr("&New Drawing"), this, &MainWindow::onNewFile);
    newAction->setShortcut(QKeySequence::New);

    fileMenu->addAction(tr("New &Part"), this, &MainWindow::onNewPart);
    fileMenu->addAction(tr("New Asse&mbly"), this, &MainWindow::onNewAssembly);

    fileMenu->addSeparator();

    fileMenu->addAction(tr("&Insert Component..."), this, &MainWindow::onInsertComponent);
    fileMenu->addAction(tr("Add &Mate..."), this, &MainWindow::onAddMate);
    fileMenu->addAction(tr("Check &Interference"), this, &MainWindow::onCheckInterference);

    fileMenu->addSeparator();

    QAction* openAction = fileMenu->addAction(tr("&Open..."), this, &MainWindow::onOpenFile);
    openAction->setShortcut(QKeySequence::Open);

    m_recentMenu = fileMenu->addMenu(tr("Open &Recent"));
    m_recentMenu->setObjectName(QStringLiteral("recentFilesMenu"));
    connect(m_recentMenu, &QMenu::aboutToShow, this, &MainWindow::rebuildRecentMenu);
    rebuildRecentMenu();

    QAction* saveAction = fileMenu->addAction(tr("&Save"), this, &MainWindow::onSaveFile);
    saveAction->setShortcut(QKeySequence::Save);

    QAction* saveAsAction = fileMenu->addAction(tr("Save &As..."), this, &MainWindow::onSaveFileAs);
    saveAsAction->setShortcut(QKeySequence::SaveAs);

    fileMenu->addSeparator();

    QMenu* importMenu = fileMenu->addMenu(tr("&Import"));
    importMenu->addAction(tr("&STEP as a New Part..."), this, &MainWindow::onImportStep)
        ->setObjectName(QStringLiteral("import_step"));
    importMenu->addAction(tr("&DXF into This Drawing..."), this, &MainWindow::onImportDxf)
        ->setObjectName(QStringLiteral("import_dxf"));

    QMenu* exportMenu = fileMenu->addMenu(tr("&Export"));
    QAction* exportStep = exportMenu->addAction(tr("&STEP..."), this, &MainWindow::onExportStep);
    QAction* exportStl = exportMenu->addAction(tr("S&TL..."), this, &MainWindow::onExportStl);
    QAction* exportGltf = exportMenu->addAction(tr("&glTF..."), this, &MainWindow::onExportGltf);
    QAction* exportDxf = exportMenu->addAction(tr("&DXF..."), this, &MainWindow::onExportDxf);
    QAction* exportPdf = exportMenu->addAction(tr("&PDF..."), this, [this] { onExportPlot(true); });
    QAction* exportSvg =
        exportMenu->addAction(tr("S&VG..."), this, [this] { onExportPlot(false); });
    exportPdf->setObjectName(QStringLiteral("export_pdf"));
    exportSvg->setObjectName(QStringLiteral("export_svg"));
    exportStep->setObjectName(QStringLiteral("export_step"));
    exportStl->setObjectName(QStringLiteral("export_stl"));
    exportGltf->setObjectName(QStringLiteral("export_gltf"));
    exportDxf->setObjectName(QStringLiteral("export_dxf"));
    // Offer what the active document has: a part's body, or a drawing.
    connect(exportMenu, &QMenu::aboutToShow, this, [=, this] {
        const bool hasBody = !m_assembly && m_document->solid() != nullptr;
        for (QAction* a : {exportStep, exportStl, exportGltf}) a->setEnabled(hasBody);
        const bool hasDrawing = !m_assembly && !m_document->draftDocument().entities().empty();
        for (QAction* a : {exportDxf, exportPdf, exportSvg}) a->setEnabled(hasDrawing);
    });

    fileMenu->addSeparator();

    QAction* exitAction = fileMenu->addAction(tr("E&xit"), this, &QWidget::close);
    exitAction->setShortcut(QKeySequence::Quit);

    // ---- Edit ----
    QMenu* editMenu = menuBar()->addMenu(tr("&Edit"));

    QAction* undoAction = editMenu->addAction(tr("&Undo"), this, &MainWindow::onUndo);
    undoAction->setShortcut(QKeySequence::Undo);

    QAction* redoAction = editMenu->addAction(tr("&Redo"), this, &MainWindow::onRedo);
    redoAction->setShortcut(QKeySequence::Redo);

    editMenu->addSeparator();

    QAction* duplicateAction =
        editMenu->addAction(tr("&Duplicate"), this, &MainWindow::onDuplicate);
    duplicateAction->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_D));

    editMenu->addSeparator();

    QAction* copyAction = editMenu->addAction(tr("&Copy"), this, &MainWindow::onCopy);
    copyAction->setShortcut(QKeySequence::Copy);

    QAction* cutAction = editMenu->addAction(tr("Cu&t"), this, &MainWindow::onCut);
    cutAction->setShortcut(QKeySequence::Cut);

    QAction* pasteAction = editMenu->addAction(tr("&Paste"), this, &MainWindow::onPaste);
    pasteAction->setShortcut(QKeySequence::Paste);

    editMenu->addSeparator();

    QAction* groupAction = editMenu->addAction(tr("&Group"), this, &MainWindow::onGroupEntities);
    groupAction->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_G));

    QAction* ungroupAction =
        editMenu->addAction(tr("U&ngroup"), this, &MainWindow::onUngroupEntities);
    ungroupAction->setShortcut(QKeySequence(Qt::CTRL | Qt::SHIFT | Qt::Key_G));

    editMenu->addSeparator();
    QAction* prefsAction =
        editMenu->addAction(tr("Pre&ferences..."), this, &MainWindow::onPreferences);
    prefsAction->setObjectName(QStringLiteral("action_preferences"));
    prefsAction->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_Comma));
    prefsAction->setMenuRole(QAction::PreferencesRole);

    // ---- View ----
    QMenu* viewMenu = menuBar()->addMenu(tr("&View"));

    viewMenu->addAction(tr("&Front"), this, &MainWindow::onViewFront);
    viewMenu->addAction(tr("&Top"), this, &MainWindow::onViewTop);
    viewMenu->addAction(tr("&Right"), this, &MainWindow::onViewRight);
    viewMenu->addAction(tr("&Isometric"), this, &MainWindow::onViewIsometric);
    viewMenu->addSeparator();
    viewMenu->addAction(tr("Fit &All"), this, &MainWindow::onFitAll);
    viewMenu->addSeparator();
    viewMenu->addAction(m_featureTreePanel->toggleViewAction());
    viewMenu->addAction(m_propertyPanel->toggleViewAction());
    viewMenu->addAction(m_layerPanel->toggleViewAction());

    // ---- Tools ----
    QMenu* toolsMenu = menuBar()->addMenu(tr("&Tools"));

    toolsMenu->addAction(tr("&Select"), this, &MainWindow::onSelectTool);
    toolsMenu->addAction(tr("&Line"), this, &MainWindow::onLineTool);
    toolsMenu->addAction(tr("&Circle"), this, &MainWindow::onCircleTool);
    toolsMenu->addAction(tr("&Arc"), this, &MainWindow::onArcTool);
    toolsMenu->addAction(tr("&Rectangle"), this, &MainWindow::onRectangleTool);
    toolsMenu->addAction(tr("&Polyline"), this, &MainWindow::onPolylineTool);
    toolsMenu->addAction(tr("&Text"), this, &MainWindow::onTextTool);
    toolsMenu->addAction(tr("&Spline"), this, &MainWindow::onSplineTool);
    toolsMenu->addAction(tr("&Hatch"), this, &MainWindow::onHatchTool);
    toolsMenu->addAction(tr("&Ellipse"), this, &MainWindow::onEllipseTool);
    toolsMenu->addSeparator();
    toolsMenu->addAction(tr("&Move"), this, &MainWindow::onMoveTool);
    toolsMenu->addAction(tr("&Offset"), this, &MainWindow::onOffsetTool);
    toolsMenu->addAction(tr("M&irror"), this, &MainWindow::onMirrorTool);
    toolsMenu->addAction(tr("&Rotate"), this, &MainWindow::onRotateTool);
    toolsMenu->addAction(tr("&Scale"), this, &MainWindow::onScaleTool);
    toolsMenu->addSeparator();
    toolsMenu->addAction(tr("&Trim"), this, &MainWindow::onTrimTool);
    toolsMenu->addAction(tr("&Fillet"), this, &MainWindow::onFilletTool);
    toolsMenu->addAction(tr("C&hamfer"), this, &MainWindow::onChamferTool);
    toolsMenu->addAction(tr("&Break"), this, &MainWindow::onBreakTool);
    toolsMenu->addAction(tr("&Extend"), this, &MainWindow::onExtendTool);
    toolsMenu->addAction(tr("&Stretch"), this, &MainWindow::onStretchTool);
    toolsMenu->addAction(tr("Polyline Ed&it"), this, &MainWindow::onPolylineEditTool);
    toolsMenu->addSeparator();
    toolsMenu->addAction(tr("Rectangular &Array"), this, &MainWindow::onRectangularArray);
    toolsMenu->addAction(tr("Polar Arra&y"), this, &MainWindow::onPolarArray);
    toolsMenu->addSeparator();

    // Drafting aids: the status bar shows each as a toggle (iconText).
    QMenu* aidsMenu = toolsMenu->addMenu(tr("Drafting &Aids"));
    const auto aid = [this, aidsMenu](const QString& text, const QString& shortName,
                                      const char* name, Qt::Key key) {
        QAction* act = aidsMenu->addAction(text);
        act->setObjectName(QString::fromLatin1(name));
        act->setIconText(shortName);
        act->setShortcut(QKeySequence(key));
        act->setCheckable(true);
        connect(act, &QAction::toggled, this, [this, act] { onDraftingAidToggled(act); });
        return act;
    };
    m_actObjectSnap = aid(tr("Object &Snap"), tr("SNAP"), "action_object_snap", Qt::Key_F3);
    m_actGridSnap = aid(tr("&Grid Snap"), tr("GRID"), "action_grid_snap", Qt::Key_F9);
    m_actOrtho = aid(tr("&Ortho"), tr("ORTHO"), "action_ortho", Qt::Key_F8);
    m_actPolar = aid(tr("&Polar Tracking"), tr("POLAR"), "action_polar", Qt::Key_F10);

    // ---- Measure ----
    QMenu* measureMenu = menuBar()->addMenu(tr("&Measure"));
    measureMenu->addAction(tr("&Distance"), this, &MainWindow::onMeasureDistanceTool);
    measureMenu->addAction(tr("&Angle"), this, &MainWindow::onMeasureAngleTool);
    measureMenu->addAction(tr("A&rea"), this, &MainWindow::onMeasureAreaTool);

    // ---- Dimension ----
    QMenu* dimMenu = menuBar()->addMenu(tr("&Dimension"));
    dimMenu->addAction(tr("&Linear"), this, &MainWindow::onLinearDimTool);
    dimMenu->addAction(tr("&Radial"), this, &MainWindow::onRadialDimTool);
    dimMenu->addAction(tr("&Angular"), this, &MainWindow::onAngularDimTool);
    QAction* dimContinue =
        dimMenu->addAction(tr("&Continue"), this, [this] { activateTool("Continue Dimension"); });
    dimContinue->setObjectName(QStringLiteral("action_dim_continue"));
    QAction* dimBaseline =
        dimMenu->addAction(tr("&Baseline"), this, [this] { activateTool("Baseline Dimension"); });
    dimBaseline->setObjectName(QStringLiteral("action_dim_baseline"));
    dimMenu->addSeparator();
    dimMenu->addAction(tr("L&eader"), this, &MainWindow::onLeaderTool);
    dimMenu->addSeparator();
    QAction* dimStyle = dimMenu->addAction(tr("&Style..."), this, &MainWindow::onDimensionStyle);
    dimStyle->setObjectName(QStringLiteral("action_dim_style"));

    // ---- Constraint ----
    QMenu* cstrMenu = menuBar()->addMenu(tr("&Constraint"));
    cstrMenu->addAction(tr("&Coincident"), this, &MainWindow::onConstraintCoincident);
    cstrMenu->addAction(tr("&Horizontal"), this, &MainWindow::onConstraintHorizontal);
    cstrMenu->addAction(tr("&Vertical"), this, &MainWindow::onConstraintVertical);
    cstrMenu->addSeparator();
    cstrMenu->addAction(tr("Per&pendicular"), this, &MainWindow::onConstraintPerpendicular);
    cstrMenu->addAction(tr("P&arallel"), this, &MainWindow::onConstraintParallel);
    cstrMenu->addAction(tr("&Tangent"), this, &MainWindow::onConstraintTangent);
    cstrMenu->addAction(tr("&Equal"), this, &MainWindow::onConstraintEqual);
    cstrMenu->addSeparator();
    cstrMenu->addAction(tr("&Fixed"), this, &MainWindow::onConstraintFixed);
    cstrMenu->addAction(tr("&Distance"), this, &MainWindow::onConstraintDistance);
    cstrMenu->addAction(tr("A&ngle"), this, &MainWindow::onConstraintAngle);

    // ---- Block ----
    QMenu* blockMenu = menuBar()->addMenu(tr("&Block"));
    blockMenu->addAction(tr("&Create Block..."), this, &MainWindow::onCreateBlock);
    blockMenu->addAction(tr("&Insert Block..."), this, &MainWindow::onInsertBlock);
    blockMenu->addSeparator();
    blockMenu->addAction(tr("&Explode"), this, &MainWindow::onExplode);

    // ---- Help ----
    QMenu* helpMenu = menuBar()->addMenu(tr("&Help"));
    QAction* aboutAction =
        helpMenu->addAction(tr("&About Horizon CAD"), this, &MainWindow::onAbout);
    aboutAction->setObjectName(QStringLiteral("action_about"));
    aboutAction->setMenuRole(QAction::AboutRole);
    helpMenu->addAction(tr("About &Qt"), qApp, &QApplication::aboutQt)
        ->setMenuRole(QAction::AboutQtRole);
}

// ---------------------------------------------------------------------------
// Ribbon toolbar
// ---------------------------------------------------------------------------

void MainWindow::createRibbonBar() {
    m_ribbonBar = new RibbonBar(this);

    // Helper: add a checkable tool action to a toolbar with icon & shortcut.
    auto* toolGroup = new QActionGroup(this);
    toolGroup->setExclusive(true);

    auto addToolAction = [&](QToolBar* tb, const QString& iconName, const QString& tooltip,
                             auto slot, const QKeySequence& shortcut = {}) -> QAction* {
        auto* act = tb->addAction(IconGenerator::icon(iconName), tooltip, this, slot);
        act->setObjectName(QStringLiteral("tool_") + iconName);  // for tests and automation
        act->setCheckable(true);
        act->setToolTip(
            shortcut.isEmpty()
                ? tooltip
                : QString("%1 (%2)").arg(tooltip, shortcut.toString(QKeySequence::NativeText)));
        if (!shortcut.isEmpty()) act->setShortcut(shortcut);
        toolGroup->addAction(act);
        return act;
    };

    // A command the menus already have (the same shortcut) is the menu's own
    // action, shown on the ribbon too: two actions with one shortcut make it
    // ambiguous, and Qt then runs neither.
    auto menuActionFor = [this](const QKeySequence& shortcut) -> QAction* {
        if (shortcut.isEmpty()) return nullptr;
        for (QAction* existing : menuBar()->findChildren<QAction*>()) {
            if (existing->shortcut() == shortcut) return existing;
        }
        return nullptr;
    };
    auto addAction = [&menuActionFor](QToolBar* tb, const QString& iconName, const QString& tooltip,
                                      auto* receiver, auto slot,
                                      const QKeySequence& shortcut = {}) -> QAction* {
        QAction* act = menuActionFor(shortcut);
        if (act != nullptr) {
            act->setIcon(IconGenerator::icon(iconName));
            // The ribbon's short label ("New"), not the menu's ("New Drawing").
            act->setIconText(tooltip);
            tb->addAction(act);
        } else {
            act = tb->addAction(IconGenerator::icon(iconName), tooltip, receiver, slot);
            if (!shortcut.isEmpty()) act->setShortcut(shortcut);
        }
        act->setObjectName(QStringLiteral("action_") + iconName);  // for tests and automation
        act->setToolTip(
            shortcut.isEmpty()
                ? tooltip
                : QString("%1 (%2)").arg(tooltip, shortcut.toString(QKeySequence::NativeText)));
        return act;
    };

    // A ribbon "group" is a small captioned panel; addGroup returns its
    // toolbar so the same addToolAction/addAction helpers fill it.
    auto group = [&](const QString& tab, const QString& title) {
        return m_ribbonBar->addGroup(tab, title);
    };

    // ---- Home tab ----
    QToolBar* g = group(tr("Home"), tr("File"));
    addAction(g, "new", tr("New"), this, &MainWindow::onNewFile, QKeySequence::New);
    addAction(g, "open", tr("Open"), this, &MainWindow::onOpenFile, QKeySequence::Open);
    addAction(g, "save", tr("Save"), this, &MainWindow::onSaveFile, QKeySequence::Save);

    g = group(tr("Home"), tr("Edit"));
    addAction(g, "undo", tr("Undo"), this, &MainWindow::onUndo, QKeySequence::Undo);
    addAction(g, "redo", tr("Redo"), this, &MainWindow::onRedo, QKeySequence::Redo);
    addAction(g, "copy", tr("Copy"), this, &MainWindow::onCopy, QKeySequence::Copy);
    addAction(g, "paste", tr("Paste"), this, &MainWindow::onPaste, QKeySequence::Paste);
    addAction(g, "duplicate", tr("Duplicate"), this, &MainWindow::onDuplicate,
              QKeySequence(Qt::CTRL | Qt::Key_D));

    g = group(tr("Home"), tr("Organize"));
    addAction(g, "group", tr("Group"), this, &MainWindow::onGroupEntities,
              QKeySequence(Qt::CTRL | Qt::Key_G));
    addAction(g, "ungroup", tr("Ungroup"), this, &MainWindow::onUngroupEntities,
              QKeySequence(Qt::CTRL | Qt::SHIFT | Qt::Key_G));

    g = group(tr("Home"), tr("View"));
    auto* selectAct = addToolAction(g, "select", tr("Select"), &MainWindow::onSelectTool,
                                    QKeySequence(Qt::Key_Space));
    selectAct->setChecked(true);
    addAction(g, "fit-all", tr("Fit All"), this, &MainWindow::onFitAll, QKeySequence(Qt::Key_F));

    // ---- Draw tab ----
    g = group(tr("Draw"), tr("Basic"));
    addToolAction(g, "line", tr("Line"), &MainWindow::onLineTool, QKeySequence(Qt::Key_L));
    addToolAction(g, "rectangle", tr("Rectangle"), &MainWindow::onRectangleTool,
                  QKeySequence(Qt::Key_R));
    addToolAction(g, "circle", tr("Circle"), &MainWindow::onCircleTool, QKeySequence(Qt::Key_C));
    addToolAction(g, "arc", tr("Arc"), &MainWindow::onArcTool, QKeySequence(Qt::Key_A));

    g = group(tr("Draw"), tr("Curves"));
    addToolAction(g, "polyline", tr("Polyline"), &MainWindow::onPolylineTool,
                  QKeySequence(Qt::Key_P));
    addToolAction(g, "ellipse", tr("Ellipse"), &MainWindow::onEllipseTool, QKeySequence(Qt::Key_E));
    addToolAction(g, "spline", tr("Spline"), &MainWindow::onSplineTool, QKeySequence(Qt::Key_S));

    g = group(tr("Draw"), tr("Detail"));
    addToolAction(g, "text", tr("Text"), &MainWindow::onTextTool, QKeySequence(Qt::Key_T));
    addToolAction(g, "hatch", tr("Hatch"), &MainWindow::onHatchTool, QKeySequence(Qt::Key_H));

    // ---- Modify tab ----
    g = group(tr("Modify"), tr("Transform"));
    addToolAction(g, "move", tr("Move"), &MainWindow::onMoveTool, QKeySequence(Qt::Key_M));
    addToolAction(g, "rotate", tr("Rotate"), &MainWindow::onRotateTool,
                  QKeySequence(Qt::SHIFT | Qt::Key_R));
    addToolAction(g, "scale", tr("Scale"), &MainWindow::onScaleTool,
                  QKeySequence(Qt::SHIFT | Qt::Key_S));
    addToolAction(g, "mirror", tr("Mirror"), &MainWindow::onMirrorTool,
                  QKeySequence(Qt::SHIFT | Qt::Key_M));

    g = group(tr("Modify"), tr("Modify"));
    addToolAction(g, "trim", tr("Trim"), &MainWindow::onTrimTool, QKeySequence(Qt::Key_X));
    addToolAction(g, "extend", tr("Extend"), &MainWindow::onExtendTool,
                  QKeySequence(Qt::SHIFT | Qt::Key_E));
    addToolAction(g, "offset", tr("Offset"), &MainWindow::onOffsetTool, QKeySequence(Qt::Key_O));
    addToolAction(g, "fillet", tr("Fillet"), &MainWindow::onFilletTool);
    addToolAction(g, "chamfer", tr("Chamfer"), &MainWindow::onChamferTool);
    addToolAction(g, "break", tr("Break"), &MainWindow::onBreakTool, QKeySequence(Qt::Key_B));
    addToolAction(g, "stretch", tr("Stretch"), &MainWindow::onStretchTool, QKeySequence(Qt::Key_W));
    addToolAction(g, "polyline-edit", tr("PL Edit"), &MainWindow::onPolylineEditTool);

    g = group(tr("Modify"), tr("Array"));
    addAction(g, "rect-array", tr("Rect Array"), this, &MainWindow::onRectangularArray);
    addAction(g, "polar-array", tr("Polar Array"), this, &MainWindow::onPolarArray);

    // ---- Annotate tab ----
    g = group(tr("Annotate"), tr("Dimensions"));
    addToolAction(g, "dim-linear", tr("Linear"), &MainWindow::onLinearDimTool,
                  QKeySequence(Qt::Key_D));
    addToolAction(g, "dim-radial", tr("Radial"), &MainWindow::onRadialDimTool);
    addToolAction(g, "dim-angular", tr("Angular"), &MainWindow::onAngularDimTool);
    addToolAction(g, "leader", tr("Leader"), &MainWindow::onLeaderTool);

    g = group(tr("Annotate"), tr("Measure"));
    addAction(g, "measure-distance", tr("Distance"), this, &MainWindow::onMeasureDistanceTool);
    addAction(g, "measure-angle", tr("Angle"), this, &MainWindow::onMeasureAngleTool);
    addAction(g, "measure-area", tr("Area"), this, &MainWindow::onMeasureAreaTool);

    // ---- Constrain tab ----
    g = group(tr("Constrain"), tr("Geometric"));
    addAction(g, "cstr-coincident", tr("Coincident"), this, &MainWindow::onConstraintCoincident);
    addAction(g, "cstr-horizontal", tr("Horizontal"), this, &MainWindow::onConstraintHorizontal);
    addAction(g, "cstr-vertical", tr("Vertical"), this, &MainWindow::onConstraintVertical);
    addAction(g, "cstr-perpendicular", tr("Perpendicular"), this,
              &MainWindow::onConstraintPerpendicular);
    addAction(g, "cstr-parallel", tr("Parallel"), this, &MainWindow::onConstraintParallel);
    addAction(g, "cstr-tangent", tr("Tangent"), this, &MainWindow::onConstraintTangent);
    addAction(g, "cstr-equal", tr("Equal"), this, &MainWindow::onConstraintEqual);

    g = group(tr("Constrain"), tr("Dimensional"));
    addAction(g, "cstr-fixed", tr("Fixed"), this, &MainWindow::onConstraintFixed);
    addAction(g, "cstr-distance", tr("Distance"), this, &MainWindow::onConstraintDistance);
    addAction(g, "cstr-angle", tr("Angle"), this, &MainWindow::onConstraintAngle);

    // ---- Block tab ----
    g = group(tr("Block"), tr("Blocks"));
    addAction(g, "block-create", tr("Create"), this, &MainWindow::onCreateBlock);
    addAction(g, "block-insert", tr("Insert"), this, &MainWindow::onInsertBlock);
    addAction(g, "block-explode", tr("Explode"), this, &MainWindow::onExplode);

    // ---- 3D tab ----
    g = group(tr("3D"), tr("Primitives"));
    addAction(g, "box", tr("Box"), this, &MainWindow::onPrimitiveBox);
    addAction(g, "cylinder", tr("Cylinder"), this, &MainWindow::onPrimitiveCylinder);
    addAction(g, "sphere", tr("Sphere"), this, &MainWindow::onPrimitiveSphere);
    addAction(g, "cone", tr("Cone"), this, &MainWindow::onPrimitiveCone);
    addAction(g, "torus", tr("Torus"), this, &MainWindow::onPrimitiveTorus);

    g = group(tr("3D"), tr("Features"));
    addAction(g, "extrude", tr("Extrude"), this, &MainWindow::onExtrudeSketch);
    addAction(g, "revolve", tr("Revolve"), this, &MainWindow::onRevolveSketch);

    g = group(tr("3D"), tr("Combine Bodies"));
    addAction(g, "boolean-union", tr("Union"), this, &MainWindow::onBooleanUnion);
    addAction(g, "boolean-subtract", tr("Subtract"), this, &MainWindow::onBooleanSubtract);
    addAction(g, "boolean-intersect", tr("Intersect"), this, &MainWindow::onBooleanIntersect);

    g = group(tr("3D"), tr("Modify"));
    addAction(g, "fillet-3d", tr("Fillet"), this, &MainWindow::onFillet);
    addAction(g, "chamfer-3d", tr("Chamfer"), this, &MainWindow::onChamfer);
    addAction(g, "shell", tr("Shell"), this, &MainWindow::onShell);
    addAction(g, "draft", tr("Draft"), this, &MainWindow::onDraft);

    g = group(tr("3D"), tr("Pattern"));
    addAction(g, "pattern-linear", tr("Linear"), this, &MainWindow::onLinearPattern);
    addAction(g, "pattern-circular", tr("Circular"), this, &MainWindow::onCircularPattern);

    // Wrap the ribbon in a QToolBar so QMainWindow places it below the menu bar.
    auto* ribbonToolBar = new QToolBar(tr("Ribbon"), this);
    ribbonToolBar->setObjectName("RibbonToolBar");
    ribbonToolBar->setMovable(false);
    ribbonToolBar->setFloatable(false);
    ribbonToolBar->addWidget(m_ribbonBar);
    addToolBar(Qt::TopToolBarArea, ribbonToolBar);
}

// ---------------------------------------------------------------------------
// Status bar
// ---------------------------------------------------------------------------

void MainWindow::createStatusBar() {
    auto* sb = statusBar();

    // Coordinates (left).
    m_statusCoords = new QLabel(tr("X: 0.000  Y: 0.000"), this);
    m_statusCoords->setMinimumWidth(180);
    m_statusCoords->setStyleSheet("QLabel { padding: 0 6px; }");
    sb->addWidget(m_statusCoords);

    // Tool prompt (center, stretch).
    m_statusPrompt = new QLabel(tr("Ready"), this);
    m_statusPrompt->setObjectName(QStringLiteral("statusPrompt"));
    m_statusPrompt->setStyleSheet("QLabel { padding: 0 6px; color: #a0c4ff; }");
    sb->addWidget(m_statusPrompt, 1);

    // Drafting aids, each a toggle: lit when on.
    for (QAction* act : {m_actObjectSnap, m_actGridSnap, m_actOrtho, m_actPolar}) {
        auto* button = new QToolButton(this);
        button->setDefaultAction(act);
        button->setToolButtonStyle(Qt::ToolButtonTextOnly);
        button->setAutoRaise(true);
        button->setStyleSheet(
            "QToolButton { padding: 0 4px; color: #707070; }"
            "QToolButton:checked { color: #80cc80; background: transparent; }");
        sb->addPermanentWidget(button);
    }

    // Selection count.
    m_statusSelection = new QLabel(tr("0 selected"), this);
    m_statusSelection->setMinimumWidth(80);
    m_statusSelection->setStyleSheet("QLabel { padding: 0 6px; }");
    sb->addPermanentWidget(m_statusSelection);

    // A rebuild running on a worker: how far it is, and a way to stop it.
    m_rebuildProgress = new QProgressBar(this);
    m_rebuildProgress->setObjectName(QStringLiteral("rebuildProgress"));
    m_rebuildProgress->setMaximumWidth(140);
    m_rebuildProgress->setTextVisible(false);
    m_rebuildProgress->hide();
    sb->addPermanentWidget(m_rebuildProgress);
    m_rebuildCancel = new QToolButton(this);
    m_rebuildCancel->setObjectName(QStringLiteral("cancelRebuild"));
    m_rebuildCancel->setText(tr("Cancel"));
    m_rebuildCancel->setToolTip(tr("Stop the work running in the background"));
    m_rebuildCancel->hide();
    connect(m_rebuildCancel, &QToolButton::clicked, this, [this] {
        if (m_rebuildJob) {
            m_rebuildAgain = false;
            m_rebuildJob->cancel();
        }
        if (m_importTask) m_importTask->cancel();
        if (m_interferenceTask) m_interferenceTask->cancel();
    });
    sb->addPermanentWidget(m_rebuildCancel);
    m_rebuildPoll = new QTimer(this);
    m_rebuildPoll->setInterval(100);
    connect(m_rebuildPoll, &QTimer::timeout, this, &MainWindow::updateRebuildProgress);

    // Autosave that is off or failing, which would otherwise go unnoticed
    // until a crash left nothing to recover.
    m_autosaveWarning = new QLabel(this);
    m_autosaveWarning->setObjectName(QStringLiteral("autosaveWarning"));
    m_autosaveWarning->setStyleSheet("QLabel { padding: 0 6px; color: #ff9a8a; }");
    m_autosaveWarning->hide();
    sb->addPermanentWidget(m_autosaveWarning);

    // Active tool name.
    m_statusTool = new QLabel(tr("Select"), this);
    m_statusTool->setMinimumWidth(80);
    m_statusTool->setStyleSheet("QLabel { padding: 0 6px; font-weight: bold; color: #ffd080; }");
    sb->addPermanentWidget(m_statusTool);
}

void MainWindow::onDraftingAidToggled(QAction* changed) {
    // Ortho and polar tracking each hold the direction: one at a time.
    if (changed == m_actOrtho && m_actOrtho->isChecked()) {
        const QSignalBlocker quiet(m_actPolar);
        m_actPolar->setChecked(false);
    } else if (changed == m_actPolar && m_actPolar->isChecked()) {
        const QSignalBlocker quiet(m_actOrtho);
        m_actOrtho->setChecked(false);
    }
    Preferences prefs = Preferences::current();
    prefs.objectSnap = m_actObjectSnap->isChecked();
    prefs.gridSnap = m_actGridSnap->isChecked();
    prefs.ortho = m_actOrtho->isChecked();
    prefs.polarTracking = m_actPolar->isChecked();
    prefs.save();
    applyPreferences(prefs);
    statusBar()->showMessage(changed->isChecked() ? tr("%1 on").arg(changed->iconText())
                                                  : tr("%1 off").arg(changed->iconText()),
                             3000);
}

QString MainWindow::toolPrompt() const {
    const Tool* tool = m_viewport ? m_viewport->activeTool() : nullptr;
    if (tool == nullptr) return tr("Ready");
    const std::string prompt = tool->promptText() + m_viewport->typedPoint().prompt();
    return prompt.empty() ? tr("Ready") : QString::fromStdString(prompt);
}

void MainWindow::updateStatusBar() {
    if (m_viewport && m_viewport->activeTool()) {
        auto* tool = m_viewport->activeTool();
        m_statusTool->setText(QString::fromStdString(tool->name()));
        m_statusPrompt->setText(toolPrompt());
    } else {
        m_statusTool->setText(tr("None"));
        m_statusPrompt->setText(tr("Ready"));
    }

    auto ids = m_viewport->selectionManager().selectedIds();
    int count = static_cast<int>(ids.size());
    m_statusSelection->setText(count == 1 ? tr("1 selected") : tr("%1 selected").arg(count));
}

// ---------------------------------------------------------------------------
// Tool registration
// ---------------------------------------------------------------------------

void MainWindow::registerTools() {
    m_toolManager->registerTool(std::make_unique<SelectTool>());
    m_toolManager->registerTool(std::make_unique<LineTool>());
    m_toolManager->registerTool(std::make_unique<CircleTool>());
    m_toolManager->registerTool(std::make_unique<ArcTool>());
    m_toolManager->registerTool(std::make_unique<RectangleTool>());
    m_toolManager->registerTool(std::make_unique<PolylineTool>());
    m_toolManager->registerTool(std::make_unique<MoveTool>());
    m_toolManager->registerTool(std::make_unique<OffsetTool>());
    m_toolManager->registerTool(std::make_unique<TrimTool>());
    m_toolManager->registerTool(std::make_unique<FilletTool>());
    m_toolManager->registerTool(std::make_unique<ChamferTool>());
    m_toolManager->registerTool(std::make_unique<BreakTool>());
    m_toolManager->registerTool(std::make_unique<ExtendTool>());
    m_toolManager->registerTool(std::make_unique<StretchTool>());
    m_toolManager->registerTool(std::make_unique<PolylineEditTool>());
    m_toolManager->registerTool(std::make_unique<MirrorTool>());
    m_toolManager->registerTool(std::make_unique<RotateTool>());
    m_toolManager->registerTool(std::make_unique<ScaleTool>());
    m_toolManager->registerTool(std::make_unique<PasteTool>(&m_clipboard));
    m_toolManager->registerTool(std::make_unique<LinearDimensionTool>());
    m_toolManager->registerTool(
        std::make_unique<ChainDimensionTool>(ChainDimensionTool::Mode::Continue));
    m_toolManager->registerTool(
        std::make_unique<ChainDimensionTool>(ChainDimensionTool::Mode::Baseline));
    m_toolManager->registerTool(std::make_unique<RadialDimensionTool>());
    m_toolManager->registerTool(std::make_unique<AngularDimensionTool>());
    m_toolManager->registerTool(std::make_unique<LeaderTool>());
    m_toolManager->registerTool(std::make_unique<ConstraintTool>());
    m_toolManager->registerTool(std::make_unique<TextTool>());
    m_toolManager->registerTool(std::make_unique<SplineTool>());
    m_toolManager->registerTool(std::make_unique<HatchTool>());
    m_toolManager->registerTool(std::make_unique<EllipseTool>());
    m_toolManager->registerTool(std::make_unique<MeasureDistanceTool>());
    m_toolManager->registerTool(std::make_unique<MeasureAngleTool>());
    m_toolManager->registerTool(std::make_unique<MeasureAreaTool>());
}

// ---------------------------------------------------------------------------
// Document tabs
// ---------------------------------------------------------------------------

MainWindow::DocTab* MainWindow::activeTab() {
    int index = m_tabBar->currentIndex();
    if (index < 0 || index >= static_cast<int>(m_tabs.size())) return nullptr;
    return &m_tabs[static_cast<size_t>(index)];
}

QString MainWindow::tabTitleForPath(const std::string& path, const QString& fallback) const {
    if (path.empty()) return fallback;
    // Paths are UTF-8; std::filesystem::path::string() would go through the
    // Windows code page and throw on characters it cannot represent.
    return QFileInfo(QString::fromStdString(path)).fileName();
}

int MainWindow::addDocumentTab(std::shared_ptr<doc::Document> document,
                               std::shared_ptr<doc::AssemblyDocument> assembly,
                               const QString& title) {
    watchDocument(document);
    m_tabs.push_back(DocTab{std::move(document), std::move(assembly), title, m_nextRecoveryKey++});
    int index = m_tabBar->addTab(title);
    m_tabBar->setCurrentIndex(index);  // triggers onTabChanged
    return index;
}

void MainWindow::activateTabDocument() {
    DocTab* tab = activeTab();
    if (!tab) return;

    // Abort any in-flight tool interaction: tools may hold entity references
    // (copy buffers, first-click state) from the previous document.
    if (m_viewport->activeTool()) {
        m_viewport->activeTool()->cancel();
    }

    m_document = tab->document;
    m_assembly = tab->assembly;

    m_viewport->setActiveSketch(nullptr);
    m_viewport->setDocument(m_document.get());
    if (tab->modelStale) {
        rebuildFeatureTree();
    } else {
        rebuildScene();
    }
    refreshAllPanels();
    updateWindowTitle();
}

void MainWindow::onTabChanged(int /*index*/) {
    activateTabDocument();
}

void MainWindow::onTabCloseRequested(int index) {
    if (index < 0 || index >= static_cast<int>(m_tabs.size())) return;

    if (!maybeSaveTab(index)) return;

    DocTab tab = m_tabs[static_cast<size_t>(index)];
    tab.document->setChangeCallback(nullptr);
    forgetSnapshot(tab);

    if (tab.assembly) {
        m_docManager.closeAssembly(tab.assembly);
        m_docManager.closeDocument(tab.document);  // backing document
    } else {
        m_docManager.closeDocument(tab.document);
    }
    m_tabs.erase(m_tabs.begin() + index);
    m_tabBar->removeTab(index);

    // Never leave the window without a document.
    if (m_tabs.empty()) {
        auto document = m_docManager.newDocument(doc::DocumentType::Drawing);
        addDocumentTab(std::move(document), nullptr, tr("Drawing 1"));
    } else {
        activateTabDocument();
    }
}

void MainWindow::rebuildScene() {
    m_viewport->sceneGraph().clear();

    if (m_assembly) {
        const std::string asmDir =
            m_assembly->filePath().empty()
                ? std::string()
                : std::filesystem::path(m_assembly->filePath()).parent_path().string();
        for (auto& comp : m_assembly->components()) {
            if (comp.suppressed) continue;
            if (!comp.cachedMesh) {
                m_docManager.resolveComponent(comp, doc::ComponentState::Lightweight, asmDir);
            }
            if (!comp.cachedMesh) continue;
            auto node =
                std::make_shared<render::SceneNode>(comp.name.empty() ? "Component" : comp.name);
            node->setMesh(std::make_unique<render::MeshData>(*comp.cachedMesh));
            node->setLocalTransform(comp.transform);
            node->setMaterial(render::Material{math::Vec3{0.62, 0.68, 0.75}, 0.15f, 0.5f, 32.0f});
            m_viewport->sceneGraph().addNode(node);
        }
    } else if (m_document->featureTree().featureCount() > 0) {
        // Build only a model that has not been built: a failed build keeps
        // its partial solid (or none), and building it again here, on the
        // GUI thread, would only fail again.
        if (!m_document->solid() && m_document->needsBuild()) m_document->rebuildModel();
        if (m_document->solid()) {
            auto meshData = model::SolidTessellator::tessellate(*m_document->solid(), 0.1);
            auto node = std::make_shared<render::SceneNode>("FeatureTree Result");
            node->setMesh(std::make_unique<render::MeshData>(std::move(meshData)));
            node->setMaterial(render::Material{math::Vec3{0.55, 0.75, 0.85}, 0.15f, 0.5f, 32.0f});
            m_viewport->sceneGraph().addNode(node);
        }
    }

    m_viewport->update();
}

void MainWindow::refreshAllPanels() {
    m_viewport->selectionManager().clearSelection();
    m_layerPanel->refresh();
    m_propertyPanel->refreshLayerList();
    onSelectionChanged();

    m_featureTreePanel->clearFailures();
    m_featureTreePanel->refresh(m_document->featureTree());
    if (m_document->failedFeatureIndex() >= 0) {
        m_featureTreePanel->markFailed(m_document->failedFeatureIndex(),
                                       m_document->lastBuildMessage());
    }
}

void MainWindow::updateWindowTitle() {
    const std::string& path = m_assembly ? m_assembly->filePath() : m_document->filePath();
    // "[*]" is where Qt shows the modified marker (setWindowModified).
    if (path.empty()) {
        setWindowTitle("Horizon CAD[*]");
    } else {
        setWindowTitle(QString("Horizon CAD - %1[*]").arg(QString::fromStdString(path)));
    }
    if (DocTab* tab = activeTab()) {
        tab->title = tabTitleForPath(path, tab->title);
    }
    refreshModifiedIndicators();
}

bool MainWindow::isTabModified(const DocTab& tab) const {
    // An assembly's edits are commands on its tab's (backing) document; the
    // assembly's own flag covers changes made outside them (a recovery).
    const bool edited = tab.document->isDirty();
    return tab.assembly ? tab.assembly->isDirty() || edited : edited;
}

void MainWindow::recordAssemblyEdit(doc::AssemblyState before, bool wasDirty,
                                    const QString& description) {
    // From here the undo stack carries the change — undoing back to the saved
    // state must clear the modified marker, which the assembly's own flag,
    // set by the edit itself, would not.
    m_assembly->setDirty(wasDirty);
    m_document->undoStack().push(std::make_unique<doc::AssemblyEditCommand>(
        *m_assembly, std::move(before), m_assembly->snapshot(), description.toStdString()));
}

void MainWindow::refreshModifiedIndicators() {
    for (size_t i = 0; i < m_tabs.size(); ++i) {
        const DocTab& tab = m_tabs[i];
        QString text = tab.title;
        if (tab.recovered) text += tr(" (recovered)");
        if (isTabModified(tab)) text += QStringLiteral(" *");
        const int index = static_cast<int>(i);
        if (index < m_tabBar->count() && m_tabBar->tabText(index) != text) {
            m_tabBar->setTabText(index, text);
        }
    }
    const DocTab* active = activeTab();
    setWindowModified(active != nullptr && isTabModified(*active));
}

bool MainWindow::maybeSaveTab(int index) {
    if (index < 0 || index >= static_cast<int>(m_tabs.size())) return true;
    if (!isTabModified(m_tabs[static_cast<size_t>(index)])) return true;

    // Show the document being asked about; Save acts on the active tab.
    m_tabBar->setCurrentIndex(index);

    QMessageBox box(QMessageBox::Warning, tr("Unsaved Changes"),
                    tr("\"%1\" has unsaved changes.").arg(m_tabs[static_cast<size_t>(index)].title),
                    QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel, this);
    box.setInformativeText(tr("Do you want to save them before closing?"));
    box.setDefaultButton(QMessageBox::Save);
    box.setEscapeButton(QMessageBox::Cancel);
    switch (box.exec()) {
        case QMessageBox::Save:
            return saveActiveDocument();
        case QMessageBox::Discard:
            return true;
        default:
            return false;
    }
}

void MainWindow::watchDocument(const std::shared_ptr<doc::Document>& document) {
    // Keep the modified markers current however the document changes: undo
    // stack pushes, undo/redo, and explicit setDirty() from tools.
    document->setChangeCallback([this, changed = document.get()] {
        for (DocTab& tab : m_tabs) {
            if (tab.document.get() == changed) tab.snapshotStale = true;
        }
        refreshModifiedIndicators();
    });
}

void MainWindow::forgetSnapshot(DocTab& tab) {
    m_recovery->remove(tab.recoveryKey);
    tab.snapshotStale = true;
    tab.recovered = false;  // saved (or closed): it is an ordinary document again
    tab.recoveries = 0;
}

void MainWindow::showAutosaveState(const QString& failure) {
    if (!m_autosaveTimer->isActive()) {
        m_autosaveWarning->hide();  // turned off in the preferences
        return;
    }
    if (!m_recovery->isActive()) {
        m_autosaveWarning->setText(tr("Autosave off"));
        m_autosaveWarning->setToolTip(
            tr("Unsaved changes cannot be recovered after a crash: autosave could not start "
               "(%1).")
                .arg(m_recovery->problem()));
        m_autosaveWarning->show();
    } else if (!failure.isEmpty()) {
        if (m_autosaveWarning->isHidden()) {
            statusBar()->showMessage(tr("Autosave failed: %1").arg(failure), 15000);
        }
        m_autosaveWarning->setText(tr("Autosave failed"));
        m_autosaveWarning->setToolTip(
            tr("The last autosave could not be written (%1), so recent changes cannot be "
               "recovered after a crash. Save your work.")
                .arg(failure));
        m_autosaveWarning->show();
    } else {
        m_autosaveWarning->hide();
    }
}

void MainWindow::autosave() {
    QString failure;
    for (DocTab& tab : m_tabs) {
        if (!isTabModified(tab)) {
            m_recovery->remove(tab.recoveryKey);
            continue;
        }
        // An assembly's edits do not come through the document callback, so
        // a modified assembly is simply written each time.
        if (!tab.snapshotStale && !tab.assembly) continue;
        const std::string& path =
            tab.assembly ? tab.assembly->filePath() : tab.document->filePath();
        const bool written =
            tab.assembly ? m_recovery->snapshot(tab.recoveryKey, *tab.assembly, tab.title,
                                                QString::fromStdString(path), tab.recoveries)
                         : m_recovery->snapshot(tab.recoveryKey, *tab.document, tab.title,
                                                QString::fromStdString(path), tab.recoveries);
        if (written) {
            tab.snapshotStale = false;
        } else if (failure.isEmpty()) {
            failure = m_recovery->problem();
        }
    }
    showAutosaveState(failure);
}

void MainWindow::offerRecovery() {
    const std::vector<RecoveryManager::Entry> orphans = m_recovery->claimOrphans();
    if (orphans.empty()) return;

    QString list;
    QStringList again;
    for (const auto& entry : orphans) {
        list += QStringLiteral("\n  \u2022 %1 (%2)")
                    .arg(entry.title,
                         QLocale().toString(entry.savedAt.toLocalTime(), QLocale::ShortFormat));
        if (entry.recoveries > 0) again << entry.title;
    }
    QMessageBox box(QMessageBox::Warning, tr("Recover Documents"),
                    tr("Horizon CAD did not shut down properly. These documents had unsaved "
                       "changes and can be recovered:%1")
                        .arg(list),
                    QMessageBox::Yes | QMessageBox::Discard | QMessageBox::Cancel, this);
    box.button(QMessageBox::Yes)->setText(tr("Recover"));
    box.button(QMessageBox::Cancel)->setText(tr("Later"));
    if (again.isEmpty()) {
        box.setDefaultButton(QMessageBox::Yes);
    } else {
        // Recovered before, and Horizon CAD stopped again: opening one of
        // them may be what stops it. Recover stays there, but it is not the
        // default, so pressing Enter at every start does not repeat it.
        box.setInformativeText(
            tr("Horizon CAD stopped again after these were last recovered: %1. Opening them may "
               "be what stops it. Save your other work before recovering them, or choose Later "
               "to keep them for now.")
                .arg(again.join(QStringLiteral(", "))));
        box.setDefaultButton(QMessageBox::Cancel);
    }
    box.setEscapeButton(QMessageBox::Cancel);
    const int choice = box.exec();
    if (choice == QMessageBox::Cancel) return;  // kept for the next start
    if (choice == QMessageBox::Discard) {
        m_recovery->discardClaimedOrphans();
        return;
    }

    // Counted before anything is opened: if opening one stops the
    // application, the next start knows.
    m_recovery->noteRecoveryAttempt();

    QStringList failed;
    for (const auto& entry : orphans) {
        const std::string snapshot = entry.snapshotPath.toStdString();
        std::string error;
        if (entry.type == QStringLiteral("hzasm")) {
            auto assembly = m_docManager.newAssembly();
            if (!io::NativeFormat::loadAssembly(snapshot, *assembly, &error)) {
                m_docManager.closeAssembly(assembly);
                failed << QStringLiteral("%1: %2").arg(entry.title, QString::fromStdString(error));
                continue;
            }
            assembly->setFilePath(entry.originalPath.toStdString());
            assembly->setDirty(true);
            solveAssemblyMates(*assembly);
            auto backing = m_docManager.newDocument(doc::DocumentType::Assembly);
            addDocumentTab(std::move(backing), std::move(assembly), entry.title);
            m_tabs.back().recovered = true;
            m_tabs.back().recoveries = entry.recoveries + 1;
        } else {
            auto document = m_docManager.newDocument(entry.type == QStringLiteral("hzpart")
                                                         ? doc::DocumentType::Part
                                                         : doc::DocumentType::Drawing);
            if (!io::NativeFormat::load(snapshot, *document, &error)) {
                m_docManager.closeDocument(document);
                failed << QStringLiteral("%1: %2").arg(entry.title, QString::fromStdString(error));
                continue;
            }
            // Saving writes back where the document came from; until then it
            // is modified, and autosaved again under this session.
            document->setFilePath(entry.originalPath.toStdString());
            document->setDirty(true);
            addDocumentTab(std::move(document), nullptr, entry.title);
            m_tabs.back().recovered = true;
            m_tabs.back().recoveries = entry.recoveries + 1;
        }
    }
    refreshModifiedIndicators();
    // Into this session's snapshots before the old ones go: a crash from
    // here on still finds them, counted.
    autosave();
    m_recovery->discardClaimedOrphans();
    if (!failed.isEmpty()) {
        spdlog::error("Recovery failed for: {}", failed.join("; ").toStdString());
        QMessageBox::warning(
            this, tr("Recover Documents"),
            tr("These documents could not be recovered:\n%1").arg(failed.join('\n')));
    }
}

void MainWindow::closeEvent(QCloseEvent* event) {
    for (int i = 0; i < static_cast<int>(m_tabs.size()); ++i) {
        if (!maybeSaveTab(i)) {
            event->ignore();
            return;
        }
    }
    saveWindowLayout();
    event->accept();
}

// ---------------------------------------------------------------------------
// Slots -- File
// ---------------------------------------------------------------------------

void MainWindow::onNewFile() {
    auto document = m_docManager.newDocument(doc::DocumentType::Drawing);
    addDocumentTab(std::move(document), nullptr, tr("Drawing %1").arg(m_tabs.size() + 1));
}

void MainWindow::onNewPart() {
    auto document = m_docManager.newDocument(doc::DocumentType::Part);
    addDocumentTab(std::move(document), nullptr, tr("Part %1").arg(m_tabs.size() + 1));
}

void MainWindow::onNewAssembly() {
    auto assembly = m_docManager.newAssembly();
    // Assemblies still need a backing Document for the shared viewport.
    auto backing = m_docManager.newDocument(doc::DocumentType::Assembly);
    addDocumentTab(std::move(backing), std::move(assembly),
                   tr("Assembly %1").arg(m_tabs.size() + 1));
}

void MainWindow::applyPreferences(const Preferences& prefs) {
    m_viewport->setDraftingAids(DraftingAids{prefs.objectSnap, prefs.gridSnap, prefs.ortho,
                                             prefs.polarTracking, prefs.polarAngle});
    const std::pair<QAction*, bool> aids[] = {
        {m_actObjectSnap, prefs.objectSnap},
        {m_actGridSnap, prefs.gridSnap},
        {m_actOrtho, prefs.ortho},
        {m_actPolar, prefs.polarTracking},
    };
    for (const auto& [act, on] : aids) {
        const QSignalBlocker quiet(act);
        act->setChecked(on);
    }
    m_autosaveTimer->stop();
    if (prefs.autosaveSeconds > 0) m_autosaveTimer->start(prefs.autosaveSeconds * 1000);
    // As the last write left it: a failure still stands until a write works.
    showAutosaveState(m_recovery->problem());
    m_viewport->snapEngine().setGridSpacing(prefs.gridSpacing);
    m_viewport->setSnapPixels(prefs.snapPixels);
}

void MainWindow::onPreferences() {
    const QString translations = QDir(QApplication::applicationDirPath()).filePath("translations");
    PreferencesDialog dialog(Preferences::current(), LocaleManager::availableLocales(translations),
                             this);
    if (dialog.exec() != QDialog::Accepted) return;
    const Preferences prefs = dialog.preferences();
    prefs.save();
    applyPreferences(prefs);
}

void MainWindow::onAbout() {
    const QString version = QString::fromLatin1(hz::version::kString);
#if defined(_MSC_VER)
    const QString compiler = QStringLiteral("MSVC %1").arg(_MSC_VER);
#elif defined(__clang__)
    const QString compiler = QStringLiteral("Clang %1").arg(QStringLiteral(__clang_version__));
#elif defined(__GNUC__)
    const QString compiler = QStringLiteral("GCC %1").arg(QStringLiteral(__VERSION__));
#else
    const QString compiler = tr("an unknown compiler");
#endif
    QMessageBox box(this);
    box.setObjectName(QStringLiteral("aboutDialog"));
    box.setWindowTitle(tr("About Horizon CAD"));
    box.setIconPixmap(windowIcon().pixmap(64, 64));
    box.setText(tr("<h3>Horizon CAD %1</h3><p>2D drafting and 3D parametric modelling.</p>")
                    .arg(version.toHtmlEscaped()));
    box.setInformativeText(
        tr("<p>Source revision %1, %2 build.<br>Built with %3 against Qt %4; running on Qt %5, "
           "%6.</p>"
           "<p>Copyright &copy; 2026 the Horizon CAD contributors.</p>"
           "<p>Horizon CAD is free software: you can redistribute it and/or modify it under the "
           "terms of the GNU General Public License, version 3 or (at your option) any later "
           "version. It comes with ABSOLUTELY NO WARRANTY. The full licence is in the LICENSE "
           "file distributed with Horizon CAD.</p>")
            .arg(QString::fromLatin1(hz::version::kRevision), QStringLiteral(HZ_BUILD_TYPE),
                 compiler.toHtmlEscaped(), QStringLiteral(QT_VERSION_STR), qVersion(),
                 QSysInfo::prettyProductName().toHtmlEscaped()));
    box.setStandardButtons(QMessageBox::Ok);
    box.exec();
}

void MainWindow::onOpenFile() {
    QString fileName = QFileDialog::getOpenFileName(
        this, tr("Open File"), QString(),
        tr("All Supported Files (*.hcad *.hzpart *.hzasm *.dxf);;"
           "Horizon CAD Drawings (*.hcad);;Horizon Parts (*.hzpart);;"
           "Horizon Assemblies (*.hzasm);;DXF Files (*.dxf);;All Files (*)"));
    if (fileName.isEmpty()) return;
    openPath(fileName);
}

bool MainWindow::openPath(const QString& fileName) {
    const std::string path = fileName.toStdString();

    // If the file is already open, just focus its tab.
    for (size_t i = 0; i < m_tabs.size(); ++i) {
        const std::string& tabPath =
            m_tabs[i].assembly ? m_tabs[i].assembly->filePath() : m_tabs[i].document->filePath();
        std::error_code ec;
        if (!tabPath.empty() &&
            std::filesystem::equivalent(std::filesystem::path(tabPath), std::filesystem::path(path),
                                        ec) &&
            !ec) {
            m_tabBar->setCurrentIndex(static_cast<int>(i));
            RecentFiles::add(fileName);
            return true;
        }
    }

    if (fileName.endsWith(".hzasm", Qt::CaseInsensitive)) {
        auto assembly = m_docManager.openAssembly(path);
        if (!assembly) {
            reportFileError(tr("Could not open"), path, m_lastLoadError);
            return false;
        }
        // The manager dedups by canonical path — an existing instance means
        // some tab already shows this assembly; focus it instead of adding
        // a second tab aliasing the same object.
        for (size_t i = 0; i < m_tabs.size(); ++i) {
            if (m_tabs[i].assembly == assembly) {
                m_tabBar->setCurrentIndex(static_cast<int>(i));
                RecentFiles::add(fileName);
                return true;
            }
        }
        const io::ImportReport report = m_lastLoadReport;
        // Saved assemblies come back positioned by their mates.
        solveAssemblyMates(*assembly);
        auto backing = m_docManager.newDocument(doc::DocumentType::Assembly);
        addDocumentTab(std::move(backing), std::move(assembly),
                       tabTitleForPath(path, tr("Assembly")));
        showImportReport(QFileInfo(fileName).fileName(), report);
        RecentFiles::add(fileName);
        return true;
    }

    if (fileName.endsWith(".dxf", Qt::CaseInsensitive)) {
        auto document = m_docManager.newDocument(doc::DocumentType::Drawing);
        std::string error;
        io::ImportReport report;
        if (!io::DxfFormat::load(path, *document, &error, &report)) {
            m_docManager.closeDocument(document);
            reportFileError(tr("Could not open"), path, error);
            return false;
        }
        document->setFilePath(path);
        document->setDirty(false);
        addDocumentTab(std::move(document), nullptr, tabTitleForPath(path, tr("Drawing")));
        showImportReport(QFileInfo(fileName).fileName(), report);
        RecentFiles::add(fileName);
        return true;
    }

    // .hcad and .hzpart both load through NativeFormat (full document —
    // entities, sketches, feature tree, design variables).
    auto document = m_docManager.openPart(path);
    if (!document) {
        reportFileError(tr("Could not open"), path, m_lastLoadError);
        return false;
    }
    // Dedup hit → the document is already shown in some tab; focus it.
    for (size_t i = 0; i < m_tabs.size(); ++i) {
        if (m_tabs[i].document == document) {
            m_tabBar->setCurrentIndex(static_cast<int>(i));
            RecentFiles::add(fileName);
            return true;
        }
    }
    const io::ImportReport report = m_lastLoadReport;
    addDocumentTab(std::move(document), nullptr, tabTitleForPath(path, tr("Document")));
    showImportReport(QFileInfo(fileName).fileName(), report);
    RecentFiles::add(fileName);
    return true;
}

void MainWindow::openFiles(const QStringList& fileNames) {
    for (const QString& fileName : fileNames) openPath(fileName);
}

void MainWindow::rebuildRecentMenu() {
    m_recentMenu->clear();
    const QStringList files = RecentFiles::list();
    for (int i = 0; i < files.size(); ++i) {
        const QString& file = files[i];
        const QFileInfo info(file);
        // &1 to &9 are mnemonics; the tenth has none.
        const QString label =
            i < 9 ? tr("&%1 %2").arg(i + 1).arg(info.fileName()) : info.fileName();
        QAction* item = m_recentMenu->addAction(label);
        item->setToolTip(QDir::toNativeSeparators(file));
        item->setStatusTip(QDir::toNativeSeparators(file));
        connect(item, &QAction::triggered, this, [this, file] {
            if (!QFileInfo::exists(file)) {
                RecentFiles::remove(file);
                reportFileError(tr("Could not open"), file.toStdString(),
                                "the file no longer exists; it has been taken off the list");
                return;
            }
            openPath(file);
        });
    }
    if (files.isEmpty()) {
        m_recentMenu->addAction(tr("No Recent Files"))->setEnabled(false);
        return;
    }
    m_recentMenu->addSeparator();
    connect(m_recentMenu->addAction(tr("&Clear Recent Files")), &QAction::triggered, this,
            [] { RecentFiles::clear(); });
}

void MainWindow::saveWindowLayout() const {
    QSettings settings;
    settings.setValue(QStringLiteral("window/geometry"), saveGeometry());
    settings.setValue(QStringLiteral("window/state"), saveState(kWindowStateVersion));
}

void MainWindow::restoreWindowLayout() {
    const QSettings settings;
    restoreGeometry(settings.value(QStringLiteral("window/geometry")).toByteArray());
    restoreState(settings.value(QStringLiteral("window/state")).toByteArray(), kWindowStateVersion);
}

bool MainWindow::saveActiveDocument() {
    DocTab* tab = activeTab();
    if (!tab) return false;

    if (m_assembly) {
        if (m_assembly->filePath().empty()) {
            onSaveFileAs();
            return !isTabModified(*tab);
        }
        std::string error;
        if (io::NativeFormat::saveAssembly(m_assembly->filePath(), *m_assembly, &error)) {
            m_assembly->setDirty(false);
            m_document->setDirty(false);  // the undo stack's saved state
            m_docManager.noteSaved(m_assembly);
            forgetSnapshot(*tab);
            RecentFiles::add(QString::fromStdString(m_assembly->filePath()));
            m_statusPrompt->setText(tr("Assembly saved."));
            updateWindowTitle();
            return true;
        }
        reportFileError(tr("Could not save"), m_assembly->filePath(), error);
        return false;
    }

    if (m_document->filePath().empty()) {
        onSaveFileAs();
        return !m_document->isDirty();
    }
    std::string path = m_document->filePath();
    bool ok = false;
    std::string error;
    if (QString::fromStdString(path).endsWith(".dxf", Qt::CaseInsensitive)) {
        ok = io::DxfFormat::save(path, *m_document, &error);
    } else {
        // Make sure parts carry a fresh tessellation cache for lightweight
        // assembly loading. A solid may be there and still be out of date:
        // while a rebuild runs on a worker, it is the part before the edit.
        if (m_document->needsBuild()) m_document->rebuildModel();
        ok = io::NativeFormat::save(path, *m_document, &error);
    }
    if (ok) {
        m_document->setDirty(false);
        m_docManager.noteSaved(m_document);
        forgetSnapshot(*tab);
        RecentFiles::add(QString::fromStdString(path));
        m_statusPrompt->setText(tr("File saved."));
        updateWindowTitle();
        return true;
    }
    reportFileError(tr("Could not save"), path, error);
    return false;
}

void MainWindow::reportFileError(const QString& summary, const std::string& path,
                                 const std::string& reason) {
    spdlog::error("{} '{}': {}", summary.toStdString(), path,
                  reason.empty() ? std::string("no reason given") : reason);

    // Reasons arrive as clauses ("the file does not exist"); show a sentence.
    QString detail = reason.empty() ? tr("The reason is unknown; see the log for details.")
                                    : QString::fromStdString(reason);
    detail[0] = detail[0].toUpper();
    if (!detail.endsWith('.')) detail += '.';

    const QString name = QFileInfo(QString::fromStdString(path)).fileName();
    QMessageBox::warning(this, tr("Error"), tr("%1 \"%2\".\n\n%3").arg(summary, name, detail));
}

void MainWindow::onSaveFile() {
    saveActiveDocument();
}

void MainWindow::onSaveFileAs() {
    QString filter;
    if (m_assembly) {
        filter = tr("Horizon Assemblies (*.hzasm);;All Files (*)");
    } else if (m_document->type() == doc::DocumentType::Part) {
        filter =
            tr("Horizon Parts (*.hzpart);;Horizon CAD Drawings (*.hcad);;"
               "All Files (*)");
    } else {
        filter =
            tr("Horizon CAD Drawings (*.hcad);;Horizon Parts (*.hzpart);;"
               "DXF Files (*.dxf);;All Files (*)");
    }

    QString fileName = QFileDialog::getSaveFileName(this, tr("Save File"), QString(), filter);
    if (fileName.isEmpty()) return;

    // Apply the new path (and extension-driven type), but roll everything
    // back if the save fails so a bad path doesn't silently retarget the
    // document or flip its type.
    if (m_assembly) {
        const std::string oldPath = m_assembly->filePath();
        m_assembly->setFilePath(fileName.toStdString());
        if (!saveActiveDocument()) {
            m_assembly->setFilePath(oldPath);
        }
        return;
    }

    const std::string oldPath = m_document->filePath();
    const doc::DocumentType oldType = m_document->type();
    // The chosen extension drives the document type.
    if (fileName.endsWith(".hzpart", Qt::CaseInsensitive)) {
        m_document->setType(doc::DocumentType::Part);
    } else if (fileName.endsWith(".hcad", Qt::CaseInsensitive)) {
        m_document->setType(doc::DocumentType::Drawing);
    }
    m_document->setFilePath(fileName.toStdString());
    if (!saveActiveDocument()) {
        m_document->setFilePath(oldPath);
        m_document->setType(oldType);
    }
}

// ---------------------------------------------------------------------------
// Slots -- Import / Export
// ---------------------------------------------------------------------------

void MainWindow::showImportReport(const QString& file, const io::ImportReport& report) {
    QStringList converted;
    for (const auto& item : report.converted) converted << QString::fromStdString(item);
    if (report.empty()) {
        // Nothing was lost: a conversion is worth a line, not a warning.
        if (!converted.isEmpty()) {
            statusBar()->showMessage(
                tr("\"%1\": %2").arg(file, converted.join(QStringLiteral("; "))), 15000);
        }
        return;
    }
    QStringList lines;
    for (const auto& item : report.skipped) {
        lines << tr("Left out: %1").arg(QString::fromStdString(item));
    }
    for (const auto& item : report.approximated) {
        lines << tr("Approximated: %1").arg(QString::fromStdString(item));
    }
    for (const auto& item : converted) lines << tr("Converted: %1").arg(item);
    QMessageBox box(QMessageBox::Warning, tr("Not Everything Was Read"),
                    tr("\"%1\": %2").arg(file, QString::fromStdString(report.summary())),
                    QMessageBox::Ok, this);
    box.setInformativeText(
        tr("What was left out is not in the document, and saving will not keep it."));
    box.setDetailedText(lines.join(QLatin1Char('\n')));
    box.exec();
}

MainWindow::StepLoad MainWindow::loadStep(const std::string& path,
                                          const std::atomic<bool>* cancelled) {
    StepLoad load;
    load.solids = io::StepFormat::load(path, &load.report, cancelled);
    if (load.solids.empty()) load.error = io::StepFormat::lastError();
    return load;
}

void MainWindow::onImportStep() {
    const QString fileName = QFileDialog::getOpenFileName(
        this, tr("Import STEP"), QString(), tr("STEP Files (*.step *.stp);;All Files (*)"));
    if (fileName.isEmpty()) return;
    const std::string path = fileName.toStdString();
    const bool onWorker =
        m_rebuildMode == RebuildMode::Always ||
        (m_rebuildMode == RebuildMode::Auto && QFileInfo(fileName).size() >= kWorkerImportBytes);
    if (!onWorker) {
        finishStepImport(fileName, loadStep(path));
        return;
    }
    if (m_importTask) {
        statusBar()->showMessage(tr("A STEP import is already running"));
        return;
    }
    m_importFile = fileName;
    m_importTask = std::make_unique<BackgroundTask<StepLoad>>(
        [path](const std::atomic<bool>& cancelled) { return loadStep(path, &cancelled); });
    m_importTask->start([this] {
        QMetaObject::invokeMethod(this, &MainWindow::onImportFinished, Qt::QueuedConnection);
    });
    m_statusPrompt->setText(tr("Importing %1...").arg(QFileInfo(fileName).fileName()));
    updateBusyIndicator();
}

void MainWindow::onImportFinished() {
    if (!m_importTask || !m_importTask->finished()) return;
    const std::unique_ptr<BackgroundTask<StepLoad>> task = std::move(m_importTask);
    updateBusyIndicator();
    if (task->cancelled()) {
        m_statusPrompt->setText(tr("Ready"));
        statusBar()->showMessage(tr("Import cancelled"), 10000);
        return;
    }
    StepLoad load = task->take();
    if (!task->error().empty()) load.error = task->error();
    finishStepImport(m_importFile, std::move(load));
}

void MainWindow::finishStepImport(const QString& fileName, StepLoad load) {
    const std::string path = fileName.toStdString();
    if (load.solids.empty()) {
        reportFileError(tr("Could not import"), path, load.error);
        return;
    }

    // A new part, one body per solid, each kept in the part itself so it does
    // not depend on the STEP file any more.
    auto document = m_docManager.newDocument(doc::DocumentType::Part);
    const std::string source = QFileInfo(fileName).fileName().toStdString();
    const auto count = static_cast<int>(load.solids.size());
    for (auto& solid : load.solids) {
        document->featureTree().addFeature(std::make_unique<doc::ImportedBodyFeature>(
            std::shared_ptr<const topo::Solid>(std::move(solid)), source));
    }
    document->rebuildModel();
    document->setDirty(true);  // it has not been saved anywhere yet
    addDocumentTab(std::move(document), nullptr, QFileInfo(fileName).completeBaseName());
    rebuildFeatureTree();
    m_viewport->camera().setIsometricView();
    m_viewport->update();
    showImportReport(QFileInfo(fileName).fileName(), load.report);
    m_statusPrompt->setText(tr("Imported %n bodies.", "", count));
}

void MainWindow::onImportDxf() {
    if (m_assembly) {
        statusBar()->showMessage(tr("A DXF is imported into a drawing or part, not an assembly"));
        return;
    }
    const QString fileName = QFileDialog::getOpenFileName(this, tr("Import DXF"), QString(),
                                                          tr("DXF Files (*.dxf);;All Files (*)"));
    if (fileName.isEmpty()) return;
    const std::string path = fileName.toStdString();
    doc::Document imported;
    std::string error;
    io::ImportReport report;
    if (!io::DxfFormat::load(path, imported, &error, &report)) {
        reportFileError(tr("Could not import"), path, error);
        return;
    }

    // Its layers and block definitions (one of the same name already here is
    // kept) and its entities come in as one undoable step.
    auto composite = std::make_unique<doc::CompositeCommand>(tr("Import DXF").toStdString());
    auto& layers = m_document->layerManager();
    for (const auto& name : imported.layerManager().layerNames()) {
        if (!layers.getLayer(name)) {
            composite->addCommand(std::make_unique<doc::AddLayerCommand>(
                layers, *imported.layerManager().getLayer(name)));
        }
    }
    auto& target = m_document->draftDocument();
    for (const auto& name : imported.draftDocument().blockTable().blockNames()) {
        if (!target.blockTable().findBlock(name)) {
            composite->addCommand(std::make_unique<doc::AddBlockDefinitionCommand>(
                target, imported.draftDocument().blockTable().findBlock(name)));
        }
    }
    for (const auto& entity : imported.draftDocument().entities()) {
        composite->addCommand(std::make_unique<doc::AddEntityCommand>(target, entity));
    }
    const auto count = static_cast<int>(imported.draftDocument().entities().size());
    if (!composite->empty()) m_document->undoStack().push(std::move(composite));
    refreshAllPanels();
    m_viewport->update();
    m_statusPrompt->setText(tr("Imported %n entities.", "", count));
    showImportReport(QFileInfo(fileName).fileName(), report);
}

const topo::Solid* MainWindow::solidToExport(const QString& format) {
    const topo::Solid* solid = m_assembly ? nullptr : m_document->solid();
    if (!solid) {
        statusBar()->showMessage(
            tr("%1 export writes a part's body; this document has none").arg(format));
    }
    return solid;
}

QString MainWindow::askExportPath(const QString& format, const QString& filter,
                                  const QString& suffix) {
    QString fileName =
        QFileDialog::getSaveFileName(this, tr("Export %1").arg(format), QString(), filter);
    if (!fileName.isEmpty() && QFileInfo(fileName).suffix().isEmpty()) fileName += suffix;
    return fileName;
}

void MainWindow::onExportStep() {
    const topo::Solid* solid = solidToExport(tr("STEP"));
    if (!solid) return;
    const QString fileName =
        askExportPath(tr("STEP"), tr("STEP Files (*.step *.stp)"), QStringLiteral(".step"));
    if (fileName.isEmpty()) return;
    if (!io::StepFormat::save(fileName.toStdString(), {solid})) {
        reportFileError(tr("Could not export"), fileName.toStdString(),
                        io::StepFormat::lastError());
        return;
    }
    m_statusPrompt->setText(tr("Exported %1.").arg(QFileInfo(fileName).fileName()));
}

void MainWindow::onExportStl() {
    const topo::Solid* solid = solidToExport(tr("STL"));
    if (!solid) return;
    const QString fileName =
        askExportPath(tr("STL"), tr("STL Files (*.stl)"), QStringLiteral(".stl"));
    if (fileName.isEmpty()) return;
    std::string error;
    if (!io::StlExport::save(fileName.toStdString(), *solid, &error)) {
        reportFileError(tr("Could not export"), fileName.toStdString(), error);
        return;
    }
    m_statusPrompt->setText(tr("Exported %1.").arg(QFileInfo(fileName).fileName()));
}

void MainWindow::onExportGltf() {
    const topo::Solid* solid = solidToExport(tr("glTF"));
    if (!solid) return;
    const QString fileName =
        askExportPath(tr("glTF"), tr("glTF Binary (*.glb)"), QStringLiteral(".glb"));
    if (fileName.isEmpty()) return;
    const DocTab* tab = activeTab();
    const std::string name = tab ? tab->title.toStdString() : std::string("Part");
    if (!io::GltfExport::saveSolid(
            fileName.toStdString(), *solid,
            render::Material{math::Vec3{0.6, 0.75, 0.85}, 0.15f, 0.5f, 32.0f}, name)) {
        reportFileError(tr("Could not export"), fileName.toStdString(),
                        "the file could not be written");
        return;
    }
    m_statusPrompt->setText(tr("Exported %1.").arg(QFileInfo(fileName).fileName()));
}

void MainWindow::onExportDxf() {
    if (m_assembly || m_document->draftDocument().entities().empty()) {
        statusBar()->showMessage(
            tr("DXF export writes a drawing; this document has nothing drawn"));
        return;
    }
    const QString fileName =
        askExportPath(tr("DXF"), tr("DXF Files (*.dxf)"), QStringLiteral(".dxf"));
    if (fileName.isEmpty()) return;
    std::string error;
    if (!io::DxfFormat::save(fileName.toStdString(), *m_document, &error)) {
        reportFileError(tr("Could not export"), fileName.toStdString(), error);
        return;
    }
    m_statusPrompt->setText(tr("Exported %1.").arg(QFileInfo(fileName).fileName()));
}

namespace {

/// "1:50" as paper millimetres per drawing millimetre (0.02); "2:1" as 2; 0
/// for anything else ("Fit to paper").
double plotScaleFrom(const QString& text) {
    const QStringList parts = text.split(QLatin1Char(':'));
    if (parts.size() != 2) return 0.0;
    bool paperOk = false;
    bool drawingOk = false;
    const double paper = parts[0].toDouble(&paperOk);
    const double drawing = parts[1].toDouble(&drawingOk);
    return paperOk && drawingOk && paper > 0.0 && drawing > 0.0 ? paper / drawing : 0.0;
}

}  // namespace

void MainWindow::onExportPlot(bool pdf) {
    const QString format = pdf ? tr("PDF") : tr("SVG");
    if (m_assembly) {
        statusBar()->showMessage(tr("%1 export plots a drawing, not an assembly").arg(format));
        return;
    }
    const draft::DraftDocument& drawing = m_document->draftDocument();
    const draft::PlotScene scene =
        draft::buildPlotScene(drawing, m_document->layerManager(), drawing.dimensionStyle());
    if (scene.empty()) {
        statusBar()->showMessage(
            tr("%1 export plots a drawing; nothing visible is drawn").arg(format));
        return;
    }

    FeatureForm form(this, tr("Export %1").arg(format));
    QStringList papers;
    for (const auto& size : draft::standardPaperSizes()) papers << QString::fromLatin1(size.name);
    auto* paper = form.choice(QStringLiteral("paper"), tr("Paper:"), papers);
    auto* orientation = form.choice(QStringLiteral("orientation"), tr("Orientation:"),
                                    {tr("Landscape"), tr("Portrait")});
    auto* scale =
        form.choice(QStringLiteral("scale"), tr("Scale:"),
                    {tr("Fit to paper"), QStringLiteral("1:1"), QStringLiteral("1:2"),
                     QStringLiteral("1:5"), QStringLiteral("1:10"), QStringLiteral("1:20"),
                     QStringLiteral("1:50"), QStringLiteral("1:100"), QStringLiteral("1:200"),
                     QStringLiteral("2:1"), QStringLiteral("5:1"), QStringLiteral("10:1")});
    auto* colours =
        form.choice(QStringLiteral("colours"), tr("Colours:"), {tr("As drawn"), tr("Black")});
    if (!form.exec()) return;

    draft::PlotLayout layout;
    const draft::PaperSize& size =
        draft::standardPaperSizes()[static_cast<size_t>(std::max(paper->currentIndex(), 0))];
    const bool landscape = orientation->currentIndex() == 0;
    layout.paperWidthMm = landscape ? size.heightMm : size.widthMm;
    layout.paperHeightMm = landscape ? size.widthMm : size.heightMm;
    layout.scale = plotScaleFrom(scale->currentText());
    layout.monochrome = colours->currentIndex() == 1;

    // At a chosen scale the drawing may not fit: say so before cutting it off.
    bool fits = true;
    draft::plotTransform(scene, layout, &fits);
    if (!fits) {
        QMessageBox box(QMessageBox::Warning, tr("Export %1").arg(format),
                        tr("At %1 the drawing is larger than the paper: what is outside it will be "
                           "cut off.")
                            .arg(scale->currentText()),
                        QMessageBox::Save | QMessageBox::Cancel, this);
        box.setDefaultButton(QMessageBox::Cancel);
        if (box.exec() != QMessageBox::Save) return;
    }

    const QString fileName =
        pdf ? askExportPath(format, tr("PDF Files (*.pdf)"), QStringLiteral(".pdf"))
            : askExportPath(format, tr("SVG Files (*.svg)"), QStringLiteral(".svg"));
    if (fileName.isEmpty()) return;
    std::string error;
    bool ok = false;
    if (pdf) {
        QString why;
        ok = exportPdf(fileName, scene, layout, &why);
        error = why.toStdString();
    } else {
        ok = io::SvgExport::save(fileName.toStdString(), scene, layout, &error);
    }
    if (!ok) {
        reportFileError(tr("Could not export"), fileName.toStdString(), error);
        return;
    }
    m_statusPrompt->setText(tr("Exported %1.").arg(QFileInfo(fileName).fileName()));
}

void MainWindow::onInsertComponent() {
    if (!m_assembly) {
        statusBar()->showMessage(tr("Insert Component is only available in an assembly document"));
        return;
    }

    QString fileName = QFileDialog::getOpenFileName(this, tr("Insert Component"), QString(),
                                                    tr("Horizon Parts (*.hzpart);;All Files (*)"));
    if (fileName.isEmpty()) return;

    doc::ComponentInstance comp;
    comp.partPath = fileName.toStdString();
    comp.name = std::filesystem::path(comp.partPath).stem().string();

    const std::string asmDir =
        m_assembly->filePath().empty()
            ? std::string()
            : std::filesystem::path(m_assembly->filePath()).parent_path().string();
    if (!m_docManager.resolveComponent(comp, doc::ComponentState::Lightweight, asmDir)) {
        QMessageBox::warning(this, tr("Error"),
                             tr("Failed to load the part (no geometry could be produced)."));
        return;
    }

    const bool wasDirty = m_assembly->isDirty();
    doc::AssemblyState before = m_assembly->snapshot();
    m_assembly->addComponent(std::move(comp));
    recordAssemblyEdit(std::move(before), wasDirty, tr("Insert Component"));
    rebuildScene();
    m_viewport->camera().setIsometricView();
    m_statusPrompt->setText(tr("Component inserted."));
}

bool MainWindow::solveAssemblyMates(doc::AssemblyDocument& asmDoc) {
    if (asmDoc.mates().empty()) return true;

    const std::string asmDir =
        asmDoc.filePath().empty() ? std::string()
                                  : std::filesystem::path(asmDoc.filePath()).parent_path().string();

    // Frames come from B-Rep faces, so mate solving needs resolved parts.
    std::vector<model::SolverComponent> solverComponents;
    for (auto& comp : asmDoc.components()) {
        if (!comp.resolvedPart) {
            m_docManager.resolveComponent(comp, doc::ComponentState::Resolved, asmDir);
        }
        model::SolverComponent sc;
        sc.id = comp.id;
        sc.transform = comp.transform;
        solverComponents.push_back(sc);
    }

    std::vector<model::SolverMate> solverMates;
    for (const auto& mate : asmDoc.mates()) {
        model::SolverMate sm;
        sm.type = mate.type;
        sm.componentA = mate.a.componentId;
        sm.componentB = mate.b.componentId;
        sm.value = mate.value;

        if (mate.type != doc::MateType::Fixed) {
            auto frameFor = [&](const doc::MateReference& ref, model::MateFrame& out) -> bool {
                const auto* comp = asmDoc.component(ref.componentId);
                if (!comp || !comp->resolvedPart || !comp->resolvedPart->solid()) return false;
                const auto* face =
                    model::MateGeometry::findFace(*comp->resolvedPart->solid(), ref.faceId);
                if (!face) return false;
                auto frame = model::MateGeometry::frameForFace(*face);
                if (!frame) return false;
                out = *frame;
                return true;
            };
            if (!frameFor(mate.a, sm.frameA) || !frameFor(mate.b, sm.frameB)) {
                statusBar()->showMessage(
                    tr("Mate %1 references geometry that could not be resolved").arg(mate.id));
                return false;
            }
        }
        solverMates.push_back(sm);
    }

    model::AssemblySolver solver;
    auto result = solver.solve(solverComponents, solverMates);

    if (result.status == model::AssemblySolveStatus::Success) {
        for (auto& comp : asmDoc.components()) {
            auto it = result.transforms.find(comp.id);
            if (it != result.transforms.end()) comp.transform = it->second;
        }
        QString status = tr("Mates solved (%1 iterations)").arg(result.iterations);
        if (result.redundantCount > 0) {
            status += tr("; %1 redundant constraint(s)").arg(result.redundantCount);
        }
        if (!result.ungroundedComponents.empty()) {
            status += tr("; %1 component(s) not connected to ground")
                          .arg(result.ungroundedComponents.size());
        }
        statusBar()->showMessage(status);
        return true;
    }

    statusBar()->showMessage(
        tr("Mate solve failed: %1")
            .arg(QString::fromStdString(result.message.empty() ? "did not converge"
                                                               : result.message)));
    return false;
}

void MainWindow::onCheckInterference() {
    if (!m_assembly) {
        statusBar()->showMessage(
            tr("Check Interference is only available in an assembly document"));
        return;
    }

    // Interference is measured on the B-Rep, so every component is resolved.
    const std::string asmDir =
        m_assembly->filePath().empty()
            ? std::string()
            : std::filesystem::path(m_assembly->filePath()).parent_path().string();
    for (auto& comp : m_assembly->components()) {
        if (!comp.suppressed && !comp.resolvedPart) {
            m_docManager.resolveComponent(comp, doc::ComponentState::Resolved, asmDir);
        }
    }

    auto input =
        std::make_shared<doc::AssemblyDocument::InterferenceInput>(m_assembly->interferenceInput());
    const bool onWorker =
        m_rebuildMode == RebuildMode::Always ||
        (m_rebuildMode == RebuildMode::Auto && input->faceCount() >= kWorkerInterferenceFaces);
    if (!onWorker) {
        showInterference(*m_assembly, doc::AssemblyDocument::measureInterference(*input));
        return;
    }
    if (m_interferenceTask) {
        statusBar()->showMessage(tr("An interference check is already running"));
        return;
    }
    // The input is copies of the placed solids: the assembly can be edited,
    // or its tab closed, while they are measured.
    m_interferenceAssembly = m_assembly;
    m_interferenceTask = std::make_unique<BackgroundTask<doc::InterferenceReport>>(
        [input](const std::atomic<bool>& cancelled) {
            return doc::AssemblyDocument::measureInterference(*input, &cancelled);
        });
    m_interferenceTask->start([this] {
        QMetaObject::invokeMethod(this, &MainWindow::onInterferenceFinished, Qt::QueuedConnection);
    });
    m_statusPrompt->setText(tr("Checking interference..."));
    updateBusyIndicator();
}

void MainWindow::onInterferenceFinished() {
    if (!m_interferenceTask || !m_interferenceTask->finished()) return;
    const std::unique_ptr<BackgroundTask<doc::InterferenceReport>> task =
        std::move(m_interferenceTask);
    const std::shared_ptr<doc::AssemblyDocument> assembly = std::move(m_interferenceAssembly);
    updateBusyIndicator();
    m_statusPrompt->setText(tr("Ready"));
    if (task->cancelled()) {
        statusBar()->showMessage(tr("Interference check cancelled"), 10000);
        return;
    }
    if (!task->error().empty()) {
        statusBar()->showMessage(
            tr("The interference check failed: %1").arg(QString::fromStdString(task->error())));
        return;
    }
    showInterference(*assembly, task->take());
}

void MainWindow::showInterference(const doc::AssemblyDocument& assembly,
                                  const doc::InterferenceReport& report) {
    auto nameOf = [&assembly](uint64_t id) {
        const auto* comp = assembly.component(id);
        const std::string name = comp && !comp->name.empty() ? comp->name : "component";
        return QString("%1 (#%2)").arg(QString::fromStdString(name)).arg(id);
    };

    QStringList lines;
    for (const auto& pair : report.pairs) {
        const QString amount = pair.volumeResolved
                                   ? tr("%1 cubic units shared").arg(pair.volume, 0, 'g', 6)
                                   : tr("overlap could not be measured");
        lines << tr("%1 and %2: %3").arg(nameOf(pair.componentA), nameOf(pair.componentB), amount);
    }
    for (uint64_t id : report.unchecked) {
        lines << tr("%1 was not checked: its part could not be resolved").arg(nameOf(id));
    }

    if (report.pairs.empty()) {
        statusBar()->showMessage(report.unchecked.empty()
                                     ? tr("No interference found")
                                     : tr("No interference among the resolved components"));
    } else {
        statusBar()->showMessage(
            tr("%n interfering pair(s)", "", static_cast<int>(report.pairs.size())));
    }
    if (!lines.isEmpty()) {
        QMessageBox::information(this, tr("Interference"), lines.join('\n'));
    }
}

void MainWindow::onAddMate() {
    if (!m_assembly) {
        statusBar()->showMessage(tr("Add Mate is only available in an assembly document"));
        return;
    }
    if (m_assembly->components().size() < 2) {
        statusBar()->showMessage(tr("Insert at least two components first"));
        return;
    }

    // Resolve parts so faces are available for picking.
    const std::string asmDir =
        m_assembly->filePath().empty()
            ? std::string()
            : std::filesystem::path(m_assembly->filePath()).parent_path().string();
    for (auto& comp : m_assembly->components()) {
        if (!comp.resolvedPart) {
            m_docManager.resolveComponent(comp, doc::ComponentState::Resolved, asmDir);
        }
    }

    // Mate-capable faces per component (those with extractable frames).
    auto faceTags = [](const doc::ComponentInstance& comp) {
        QStringList tags;
        if (comp.resolvedPart && comp.resolvedPart->solid()) {
            for (const auto& face : comp.resolvedPart->solid()->faces()) {
                if (!face.topoId.isValid()) continue;
                if (model::MateGeometry::frameForFace(face)) {
                    tags << QString::fromStdString(face.topoId.tag());
                }
            }
        }
        return tags;
    };

    QDialog dialog(this);
    dialog.setWindowTitle(tr("Add Mate"));
    auto* form = new QFormLayout(&dialog);

    auto* typeCombo = new QComboBox(&dialog);
    typeCombo->addItems({tr("Coincident"), tr("Concentric"), tr("Distance"), tr("Angle"),
                         tr("Parallel"), tr("Perpendicular"), tr("Tangent"), tr("Fixed")});
    form->addRow(tr("Type:"), typeCombo);

    auto* compACombo = new QComboBox(&dialog);
    auto* compBCombo = new QComboBox(&dialog);
    for (const auto& comp : m_assembly->components()) {
        QString label =
            QString("%1 (#%2)")
                .arg(QString::fromStdString(comp.name.empty() ? "component" : comp.name))
                .arg(comp.id);
        compACombo->addItem(label, QVariant::fromValue<qulonglong>(comp.id));
        compBCombo->addItem(label, QVariant::fromValue<qulonglong>(comp.id));
    }
    if (compBCombo->count() > 1) compBCombo->setCurrentIndex(1);

    auto* faceACombo = new QComboBox(&dialog);
    auto* faceBCombo = new QComboBox(&dialog);
    auto refreshFaces = [&](QComboBox* compCombo, QComboBox* faceCombo) {
        faceCombo->clear();
        const auto id = static_cast<uint64_t>(compCombo->currentData().toULongLong());
        if (const auto* comp = m_assembly->component(id)) {
            faceCombo->addItems(faceTags(*comp));
        }
    };
    refreshFaces(compACombo, faceACombo);
    refreshFaces(compBCombo, faceBCombo);
    connect(compACombo, &QComboBox::currentIndexChanged, &dialog,
            [&] { refreshFaces(compACombo, faceACombo); });
    connect(compBCombo, &QComboBox::currentIndexChanged, &dialog,
            [&] { refreshFaces(compBCombo, faceBCombo); });

    form->addRow(tr("Component A:"), compACombo);
    form->addRow(tr("Face A:"), faceACombo);
    form->addRow(tr("Component B:"), compBCombo);
    form->addRow(tr("Face B:"), faceBCombo);

    auto* valueSpin = new QDoubleSpinBox(&dialog);
    valueSpin->setRange(-1e6, 1e6);
    valueSpin->setDecimals(3);
    form->addRow(tr("Value (distance / angle°):"), valueSpin);

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog);
    connect(buttons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
    form->addRow(buttons);

    if (dialog.exec() != QDialog::Accepted) return;

    doc::Mate mate;
    mate.type = static_cast<doc::MateType>(typeCombo->currentIndex());
    mate.a.componentId = static_cast<uint64_t>(compACombo->currentData().toULongLong());
    mate.a.faceId = topo::TopologyID::fromTag(faceACombo->currentText().toStdString());
    if (mate.type != doc::MateType::Fixed) {
        mate.b.componentId = static_cast<uint64_t>(compBCombo->currentData().toULongLong());
        mate.b.faceId = topo::TopologyID::fromTag(faceBCombo->currentText().toStdString());
    }
    mate.value = mate.type == doc::MateType::Angle ? valueSpin->value() * std::numbers::pi / 180.0
                                                   : valueSpin->value();

    // The solve moves components; a mate that cannot be solved leaves the
    // assembly exactly as it was, and one that can is a single undo step.
    const bool wasDirty = m_assembly->isDirty();
    doc::AssemblyState before = m_assembly->snapshot();
    m_assembly->addMate(std::move(mate));
    if (solveAssemblyMates(*m_assembly)) {
        recordAssemblyEdit(std::move(before), wasDirty, tr("Add Mate"));
    } else {
        m_assembly->restore(std::move(before));
        m_assembly->setDirty(wasDirty);
    }
    rebuildScene();
}

// ---------------------------------------------------------------------------
// Slots -- Edit
// ---------------------------------------------------------------------------

void MainWindow::onUndo() {
    undoOrRedo(true);
}

void MainWindow::onRedo() {
    undoOrRedo(false);
}

void MainWindow::undoOrRedo(bool undo) {
    const uint64_t revision = m_document->featureTree().revision();
    if (undo) {
        m_document->undoStack().undo();
    } else {
        m_document->undoStack().redo();
    }
    // The model and the assembly are drawn from what they were last built
    // into; an undo that changed them has to rebuild that.
    if (m_assembly) {
        rebuildScene();
    } else if (m_document->featureTree().revision() != revision) {
        rebuildFeatureTree();
    }
    m_viewport->update();
    m_layerPanel->refresh();
    onSelectionChanged();
}

void MainWindow::onDuplicate() {
    auto& sel = m_viewport->selectionManager();
    auto ids = sel.selectedIds();
    if (ids.empty()) return;

    // Filter out entities on hidden/locked layers.
    const auto& layerMgr = m_document->layerManager();
    std::vector<uint64_t> idVec;
    for (const auto& entity : m_document->draftDocument().entities()) {
        if (!sel.isSelected(entity->id())) continue;
        const auto* lp = layerMgr.getLayer(entity->layer());
        if (!lp || !lp->visible || lp->locked) continue;
        idVec.push_back(entity->id());
    }
    if (idVec.empty()) return;

    math::Vec2 offset(1.0, -1.0);
    auto cmd =
        std::make_unique<doc::DuplicateEntityCommand>(m_document->draftDocument(), idVec, offset);
    auto* rawCmd = cmd.get();
    m_document->undoStack().push(std::move(cmd));

    // Select the clones.
    sel.clearSelection();
    for (uint64_t id : rawCmd->clonedIds()) {
        sel.select(id);
    }
    m_viewport->update();
    onSelectionChanged();
}

void MainWindow::onCopy() {
    auto& sel = m_viewport->selectionManager();
    auto ids = sel.selectedIds();
    if (ids.empty()) return;

    std::vector<std::shared_ptr<draft::DraftEntity>> entities;
    for (const auto& entity : m_document->draftDocument().entities()) {
        if (sel.isSelected(entity->id())) {
            entities.push_back(entity);
        }
    }
    m_clipboard.copy(entities);
}

void MainWindow::onCut() {
    onCopy();

    auto& sel = m_viewport->selectionManager();
    auto ids = sel.selectedIds();
    if (ids.empty()) return;

    // Only remove entities on visible/unlocked layers.
    const auto& layerMgr = m_document->layerManager();
    auto& drawing = m_document->draftDocument();
    std::vector<uint64_t> removable;
    removable.reserve(ids.size());
    for (uint64_t id : ids) {
        const draft::DraftEntity* entity = drawing.findEntity(id);
        if (entity == nullptr) continue;
        const auto* lp = layerMgr.getLayer(entity->layer());
        if (!lp || !lp->visible || lp->locked) continue;
        removable.push_back(id);
    }
    if (!removable.empty()) {
        auto composite = std::make_unique<doc::CompositeCommand>("Cut");
        composite->addCommand(
            std::make_unique<doc::RemoveEntitiesCommand>(drawing, std::move(removable)));
        m_document->undoStack().push(std::move(composite));
    }

    sel.clearSelection();
    m_viewport->update();
    onSelectionChanged();
}

void MainWindow::onPaste() {
    if (!m_clipboard.hasContent()) return;
    m_toolManager->setActiveTool("Paste");
    m_viewport->setActiveTool(m_toolManager->activeTool());
    updateStatusBar();
}

// ---------------------------------------------------------------------------
// Slots -- View
// ---------------------------------------------------------------------------

void MainWindow::onViewFront() {
    m_viewport->camera().setFrontView();
    m_viewport->update();
}

void MainWindow::onViewTop() {
    m_viewport->camera().setTopView();
    m_viewport->update();
}

void MainWindow::onViewRight() {
    m_viewport->camera().setRightView();
    m_viewport->update();
}

void MainWindow::onViewIsometric() {
    m_viewport->camera().setIsometricView();
    m_viewport->update();
}

void MainWindow::onFitAll() {
    math::BoundingBox bbox;
    for (const auto& entity : m_document->draftDocument().entities()) {
        auto entityBBox = entity->boundingBox();
        if (entityBBox.isValid()) {
            bbox.expand(entityBBox);
        }
    }
    if (bbox.isValid()) {
        m_viewport->camera().fitAll(bbox);
    } else {
        m_viewport->camera().setIsometricView();
    }
    m_viewport->update();
}

// ---------------------------------------------------------------------------
// Slots -- Tools
// ---------------------------------------------------------------------------

void MainWindow::onSelectTool() {
    m_toolManager->setActiveTool("Select");
    m_viewport->setActiveTool(m_toolManager->activeTool());
    updateStatusBar();
}

void MainWindow::onLineTool() {
    m_toolManager->setActiveTool("Line");
    m_viewport->setActiveTool(m_toolManager->activeTool());
    updateStatusBar();
}

void MainWindow::onCircleTool() {
    m_toolManager->setActiveTool("Circle");
    m_viewport->setActiveTool(m_toolManager->activeTool());
    updateStatusBar();
}

void MainWindow::onArcTool() {
    m_toolManager->setActiveTool("Arc");
    m_viewport->setActiveTool(m_toolManager->activeTool());
    updateStatusBar();
}

void MainWindow::onRectangleTool() {
    m_toolManager->setActiveTool("Rectangle");
    m_viewport->setActiveTool(m_toolManager->activeTool());
    updateStatusBar();
}

void MainWindow::onPolylineTool() {
    m_toolManager->setActiveTool("Polyline");
    m_viewport->setActiveTool(m_toolManager->activeTool());
    updateStatusBar();
}

void MainWindow::onMoveTool() {
    m_toolManager->setActiveTool("Move");
    m_viewport->setActiveTool(m_toolManager->activeTool());
    updateStatusBar();
}

void MainWindow::onOffsetTool() {
    m_toolManager->setActiveTool("Offset");
    m_viewport->setActiveTool(m_toolManager->activeTool());
    updateStatusBar();
}

void MainWindow::onMirrorTool() {
    m_toolManager->setActiveTool("Mirror");
    m_viewport->setActiveTool(m_toolManager->activeTool());
    updateStatusBar();
}

void MainWindow::onTrimTool() {
    m_toolManager->setActiveTool("Trim");
    m_viewport->setActiveTool(m_toolManager->activeTool());
    updateStatusBar();
}

void MainWindow::onFilletTool() {
    m_toolManager->setActiveTool("Fillet");
    m_viewport->setActiveTool(m_toolManager->activeTool());
    updateStatusBar();
}

void MainWindow::onChamferTool() {
    m_toolManager->setActiveTool("Chamfer");
    m_viewport->setActiveTool(m_toolManager->activeTool());
    updateStatusBar();
}

void MainWindow::onBreakTool() {
    m_toolManager->setActiveTool("Break");
    m_viewport->setActiveTool(m_toolManager->activeTool());
    updateStatusBar();
}

void MainWindow::onExtendTool() {
    m_toolManager->setActiveTool("Extend");
    m_viewport->setActiveTool(m_toolManager->activeTool());
    updateStatusBar();
}

void MainWindow::onStretchTool() {
    m_toolManager->setActiveTool("Stretch");
    m_viewport->setActiveTool(m_toolManager->activeTool());
    updateStatusBar();
}

void MainWindow::onPolylineEditTool() {
    m_toolManager->setActiveTool("PolylineEdit");
    m_viewport->setActiveTool(m_toolManager->activeTool());
    updateStatusBar();
}

void MainWindow::onRotateTool() {
    m_toolManager->setActiveTool("Rotate");
    m_viewport->setActiveTool(m_toolManager->activeTool());
    updateStatusBar();
}

void MainWindow::onScaleTool() {
    m_toolManager->setActiveTool("Scale");
    m_viewport->setActiveTool(m_toolManager->activeTool());
    updateStatusBar();
}

void MainWindow::onRectangularArray() {
    auto& sel = m_viewport->selectionManager();
    auto ids = sel.selectedIds();
    if (ids.empty()) return;

    // Filter out entities on hidden/locked layers.
    const auto& layerMgr = m_document->layerManager();
    std::vector<uint64_t> filteredIds;
    for (const auto& entity : m_document->draftDocument().entities()) {
        if (!sel.isSelected(entity->id())) continue;
        const auto* lp = layerMgr.getLayer(entity->layer());
        if (!lp || !lp->visible || lp->locked) continue;
        filteredIds.push_back(entity->id());
    }
    if (filteredIds.empty()) return;

    RectArrayDialog dlg(this);
    if (dlg.exec() != QDialog::Accepted) return;

    int cols = dlg.columns();
    int rows = dlg.rows();
    double sx = dlg.spacingX();
    double sy = dlg.spacingY();

    auto composite = std::make_unique<doc::CompositeCommand>("Rectangular Array");
    std::vector<uint64_t> newIds;
    std::vector<std::shared_ptr<draft::DraftEntity>> allClones;

    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            if (r == 0 && c == 0) continue;  // Skip original position.
            math::Vec2 offset(c * sx, r * sy);
            for (uint64_t id : filteredIds) {
                if (const auto entity = m_document->draftDocument().sharedEntity(id)) {
                    auto clone = entity->clone();
                    clone->translate(offset);
                    newIds.push_back(clone->id());
                    allClones.push_back(clone);
                    composite->addCommand(std::make_unique<doc::AddEntityCommand>(
                        m_document->draftDocument(), clone));
                }
            }
        }
    }

    doc::remapCloneGroupIds(m_document->draftDocument(), allClones);
    m_document->undoStack().push(std::move(composite));

    sel.clearSelection();
    for (uint64_t id : newIds) {
        sel.select(id);
    }
    m_viewport->update();
    onSelectionChanged();
}

void MainWindow::onPolarArray() {
    auto& sel = m_viewport->selectionManager();
    auto ids = sel.selectedIds();
    if (ids.empty()) return;

    // Filter out entities on hidden/locked layers.
    const auto& layerMgr = m_document->layerManager();
    std::vector<uint64_t> filteredIds;
    for (const auto& entity : m_document->draftDocument().entities()) {
        if (!sel.isSelected(entity->id())) continue;
        const auto* lp = layerMgr.getLayer(entity->layer());
        if (!lp || !lp->visible || lp->locked) continue;
        filteredIds.push_back(entity->id());
    }
    if (filteredIds.empty()) return;

    PolarArrayDialog dlg(this);
    if (dlg.exec() != QDialog::Accepted) return;

    int count = dlg.count();
    double totalAngleDeg = dlg.totalAngle();
    math::Vec2 center(dlg.centerX(), dlg.centerY());
    double totalAngleRad = math::degToRad(totalAngleDeg);
    double step = totalAngleRad / count;

    auto composite = std::make_unique<doc::CompositeCommand>("Polar Array");
    std::vector<uint64_t> newIds;
    std::vector<std::shared_ptr<draft::DraftEntity>> allClones;

    for (int i = 1; i < count; ++i) {
        double angle = step * i;
        for (uint64_t id : filteredIds) {
            if (const auto entity = m_document->draftDocument().sharedEntity(id)) {
                auto clone = entity->clone();
                clone->rotate(center, angle);
                newIds.push_back(clone->id());
                allClones.push_back(clone);
                composite->addCommand(
                    std::make_unique<doc::AddEntityCommand>(m_document->draftDocument(), clone));
            }
        }
    }

    doc::remapCloneGroupIds(m_document->draftDocument(), allClones);
    m_document->undoStack().push(std::move(composite));

    sel.clearSelection();
    for (uint64_t id : newIds) {
        sel.select(id);
    }
    m_viewport->update();
    onSelectionChanged();
}

// ---------------------------------------------------------------------------
// Slots -- Dimension tools
// ---------------------------------------------------------------------------

void MainWindow::activateTool(const std::string& name) {
    m_toolManager->setActiveTool(name);
    m_viewport->setActiveTool(m_toolManager->activeTool());
    updateStatusBar();
}

void MainWindow::onLinearDimTool() {
    m_toolManager->setActiveTool("Linear Dimension");
    m_viewport->setActiveTool(m_toolManager->activeTool());
    updateStatusBar();
}

void MainWindow::onRadialDimTool() {
    m_toolManager->setActiveTool("Radial Dimension");
    m_viewport->setActiveTool(m_toolManager->activeTool());
    updateStatusBar();
}

void MainWindow::onAngularDimTool() {
    m_toolManager->setActiveTool("Angular Dimension");
    m_viewport->setActiveTool(m_toolManager->activeTool());
    updateStatusBar();
}

void MainWindow::onDimensionStyle() {
    draft::DraftDocument& drawing = m_document->draftDocument();
    const draft::DimensionStyle& now = drawing.dimensionStyle();
    const QStringList units = {QStringLiteral("mm"), QStringLiteral("cm"), QStringLiteral("m"),
                               QStringLiteral("in"), QStringLiteral("ft")};

    FeatureForm form(this, tr("Dimension Style"));
    auto* height = form.number(QStringLiteral("textHeight"), tr("Text height:"), now.textHeight,
                               0.01, 1000.0, 3);
    auto* arrow =
        form.number(QStringLiteral("arrowSize"), tr("Arrow size:"), now.arrowSize, 0.0, 1000.0, 3);
    auto* angle = form.number(QStringLiteral("arrowAngle"), tr("Arrow half-angle (degrees):"),
                              now.arrowAngle * math::kRadToDeg, 1.0, 89.0, 1);
    auto* gap = form.number(QStringLiteral("extensionGap"), tr("Extension gap:"), now.extensionGap,
                            0.0, 1000.0, 3);
    auto* overshoot = form.number(QStringLiteral("extensionOvershoot"), tr("Extension overshoot:"),
                                  now.extensionOvershoot, 0.0, 1000.0, 3);
    auto* precision =
        form.count(QStringLiteral("precision"), tr("Decimal places:"), now.precision, 0, 8);
    auto* unit = form.choice(QStringLiteral("unit"), tr("Unit:"), units);
    unit->setCurrentIndex(
        std::max(0, static_cast<int>(units.indexOf(QString::fromStdString(now.unit)))));
    auto* showUnit =
        form.choice(QStringLiteral("showUnits"), tr("Show the unit:"), {tr("No"), tr("Yes")});
    showUnit->setCurrentIndex(now.showUnits ? 1 : 0);
    // A field shows its value rounded to its decimals (the arrow's 0.3 radians
    // as 17.2 degrees); left as shown, it keeps the value exactly.
    const auto field = [](QDoubleSpinBox* spin, double was, double scale = 1.0) {
        return [spin, was, scale, shown = spin->value()] {
            return spin->value() == shown ? was : spin->value() * scale;
        };
    };
    const auto newHeight = field(height, now.textHeight);
    const auto newArrow = field(arrow, now.arrowSize);
    const auto newAngle = field(angle, now.arrowAngle, math::kDegToRad);
    const auto newGap = field(gap, now.extensionGap);
    const auto newOvershoot = field(overshoot, now.extensionOvershoot);
    if (!form.exec()) return;

    draft::DimensionStyle style = now;
    style.textHeight = newHeight();
    style.arrowSize = newArrow();
    style.arrowAngle = newAngle();
    style.extensionGap = newGap();
    style.extensionOvershoot = newOvershoot();
    style.precision = precision->value();
    style.unit = unit->currentText().toStdString();
    style.showUnits = showUnit->currentIndex() == 1;
    if (style == now) return;  // OK with nothing changed is not a step to undo
    m_document->undoStack().push(
        std::make_unique<doc::ChangeDimensionStyleCommand>(drawing, style));
    m_viewport->update();
}

void MainWindow::onLeaderTool() {
    m_toolManager->setActiveTool("Leader");
    m_viewport->setActiveTool(m_toolManager->activeTool());
    updateStatusBar();
}

void MainWindow::onTextTool() {
    m_toolManager->setActiveTool("Text");
    m_viewport->setActiveTool(m_toolManager->activeTool());
    updateStatusBar();
}

void MainWindow::onSplineTool() {
    m_toolManager->setActiveTool("Spline");
    m_viewport->setActiveTool(m_toolManager->activeTool());
    updateStatusBar();
}

void MainWindow::onHatchTool() {
    m_toolManager->setActiveTool("Hatch");
    m_viewport->setActiveTool(m_toolManager->activeTool());
    updateStatusBar();
}

void MainWindow::onEllipseTool() {
    m_toolManager->setActiveTool("Ellipse");
    m_viewport->setActiveTool(m_toolManager->activeTool());
    updateStatusBar();
}

// ---------------------------------------------------------------------------
// Slots -- Measure tools
// ---------------------------------------------------------------------------

void MainWindow::onMeasureDistanceTool() {
    m_toolManager->setActiveTool("MeasureDistance");
    m_viewport->setActiveTool(m_toolManager->activeTool());
    updateStatusBar();
}

void MainWindow::onMeasureAngleTool() {
    m_toolManager->setActiveTool("MeasureAngle");
    m_viewport->setActiveTool(m_toolManager->activeTool());
    updateStatusBar();
}

void MainWindow::onMeasureAreaTool() {
    m_toolManager->setActiveTool("MeasureArea");
    m_viewport->setActiveTool(m_toolManager->activeTool());
    updateStatusBar();
}

// ---------------------------------------------------------------------------
// Slots -- Constraint tools
// ---------------------------------------------------------------------------

static void activateConstraintMode(ToolManager& tm, ViewportWidget* vp, ConstraintTool::Mode mode) {
    tm.setActiveTool("Constraint");
    auto* tool = dynamic_cast<ConstraintTool*>(tm.activeTool());
    if (tool) tool->setMode(mode);
    vp->setActiveTool(tm.activeTool());
}

void MainWindow::onConstraintCoincident() {
    activateConstraintMode(*m_toolManager, m_viewport, ConstraintTool::Mode::Coincident);
    updateStatusBar();
}

void MainWindow::onConstraintHorizontal() {
    activateConstraintMode(*m_toolManager, m_viewport, ConstraintTool::Mode::Horizontal);
    updateStatusBar();
}

void MainWindow::onConstraintVertical() {
    activateConstraintMode(*m_toolManager, m_viewport, ConstraintTool::Mode::Vertical);
    updateStatusBar();
}

void MainWindow::onConstraintPerpendicular() {
    activateConstraintMode(*m_toolManager, m_viewport, ConstraintTool::Mode::Perpendicular);
    updateStatusBar();
}

void MainWindow::onConstraintParallel() {
    activateConstraintMode(*m_toolManager, m_viewport, ConstraintTool::Mode::Parallel);
    updateStatusBar();
}

void MainWindow::onConstraintTangent() {
    activateConstraintMode(*m_toolManager, m_viewport, ConstraintTool::Mode::Tangent);
    updateStatusBar();
}

void MainWindow::onConstraintEqual() {
    activateConstraintMode(*m_toolManager, m_viewport, ConstraintTool::Mode::Equal);
    updateStatusBar();
}

void MainWindow::onConstraintFixed() {
    activateConstraintMode(*m_toolManager, m_viewport, ConstraintTool::Mode::Fixed);
    updateStatusBar();
}

void MainWindow::onConstraintDistance() {
    activateConstraintMode(*m_toolManager, m_viewport, ConstraintTool::Mode::Distance);
    updateStatusBar();
}

void MainWindow::onConstraintAngle() {
    activateConstraintMode(*m_toolManager, m_viewport, ConstraintTool::Mode::Angle);
    updateStatusBar();
}

// ---------------------------------------------------------------------------
// Slots -- Block operations
// ---------------------------------------------------------------------------

void MainWindow::onCreateBlock() {
    auto& sel = m_viewport->selectionManager();
    auto ids = sel.selectedIds();
    if (ids.empty()) {
        QMessageBox::information(this, tr("Create Block"), tr("Select entities first."));
        return;
    }

    // Filter to visible/unlocked layers.
    const auto& layerMgr = m_document->layerManager();
    std::vector<uint64_t> filteredIds;
    for (const auto& entity : m_document->draftDocument().entities()) {
        if (!sel.isSelected(entity->id())) continue;
        const auto* lp = layerMgr.getLayer(entity->layer());
        if (!lp || !lp->visible || lp->locked) continue;
        filteredIds.push_back(entity->id());
    }
    if (filteredIds.empty()) return;

    // The base point, where the block is inserted from: by default the centre
    // of what was selected, or typed.
    math::BoundingBox bounds;
    for (uint64_t id : filteredIds) {
        if (const auto* e = m_document->draftDocument().findEntity(id)) {
            const auto bb = e->boundingBox();
            if (bb.isValid()) bounds.expand(bb);
        }
    }
    const math::Vec3 centre = bounds.isValid() ? bounds.center() : math::Vec3(0, 0, 0);
    FeatureForm form(this, tr("Create Block"));
    auto* nameField = form.text(QStringLiteral("blockName"), tr("Block name:"));
    auto* baseX = form.number(QStringLiteral("baseX"), tr("Base point X:"), centre.x, -1e9, 1e9, 4);
    auto* baseY = form.number(QStringLiteral("baseY"), tr("Base point Y:"), centre.y, -1e9, 1e9, 4);
    if (!form.exec()) return;
    const QString name = nameField->text().trimmed();
    if (name.isEmpty()) return;

    std::string blockName = name.toStdString();
    if (m_document->draftDocument().blockTable().findBlock(blockName)) {
        QMessageBox::warning(this, tr("Create Block"),
                             tr("A block with that name already exists."));
        return;
    }

    const math::Vec2 base(baseX->value(), baseY->value());
    auto cmd = std::make_unique<doc::CreateBlockCommand>(m_document->draftDocument(), blockName,
                                                         filteredIds, base,
                                                         m_document->layerManager().currentLayer());
    auto* rawCmd = cmd.get();
    m_document->undoStack().push(std::move(cmd));

    sel.clearSelection();
    sel.select(rawCmd->blockRefId());
    m_viewport->update();
    onSelectionChanged();
}

void MainWindow::onInsertBlock() {
    auto names = m_document->draftDocument().blockTable().blockNames();
    if (names.empty()) {
        QMessageBox::information(this, tr("Insert Block"),
                                 tr("No blocks defined. Create a block first."));
        return;
    }

    InsertBlockDialog dlg(names, this);
    if (dlg.exec() != QDialog::Accepted) return;

    auto def = m_document->draftDocument().blockTable().findBlock(dlg.selectedBlock());
    if (!def) return;

    // A new InsertBlockTool replaces the previous one, which may be the active
    // tool: the viewport lets go of it (deactivating it while it still exists)
    // before it is destroyed.
    m_viewport->setActiveTool(nullptr);
    auto tool = std::make_unique<InsertBlockTool>(def, dlg.rotation(), dlg.scale());
    m_toolManager->registerTool(std::move(tool));
    m_toolManager->setActiveTool("Insert Block");
    m_viewport->setActiveTool(m_toolManager->activeTool());
    updateStatusBar();
}

void MainWindow::onExplode() {
    auto& sel = m_viewport->selectionManager();
    auto ids = sel.selectedIds();
    if (ids.empty()) return;

    // Find block refs among the selection.
    std::vector<uint64_t> blockRefIds;
    for (const auto& entity : m_document->draftDocument().entities()) {
        if (!sel.isSelected(entity->id())) continue;
        if (dynamic_cast<const draft::DraftBlockRef*>(entity.get())) {
            blockRefIds.push_back(entity->id());
        }
    }
    if (blockRefIds.empty()) {
        QMessageBox::information(this, tr("Explode"),
                                 tr("Select one or more block references to explode."));
        return;
    }

    auto composite = std::make_unique<doc::CompositeCommand>("Explode");
    std::vector<doc::ExplodeBlockCommand*> explodeCmds;
    for (uint64_t id : blockRefIds) {
        auto cmd = std::make_unique<doc::ExplodeBlockCommand>(m_document->draftDocument(), id);
        explodeCmds.push_back(cmd.get());
        composite->addCommand(std::move(cmd));
    }
    m_document->undoStack().push(std::move(composite));

    // Select the exploded entities.
    sel.clearSelection();
    for (auto* cmd : explodeCmds) {
        for (uint64_t id : cmd->explodedIds()) {
            sel.select(id);
        }
    }
    m_viewport->update();
    onSelectionChanged();
}

// ---------------------------------------------------------------------------
// Slots -- Group / Ungroup
// ---------------------------------------------------------------------------

void MainWindow::onGroupEntities() {
    auto& sel = m_viewport->selectionManager();
    auto ids = sel.selectedIds();
    if (ids.size() < 2) return;  // Need at least 2 entities to group.

    // Filter to visible/unlocked layers.
    const auto& layerMgr = m_document->layerManager();
    std::vector<uint64_t> filteredIds;
    for (const auto& entity : m_document->draftDocument().entities()) {
        if (!sel.isSelected(entity->id())) continue;
        const auto* lp = layerMgr.getLayer(entity->layer());
        if (!lp || !lp->visible || lp->locked) continue;
        filteredIds.push_back(entity->id());
    }
    if (filteredIds.size() < 2) return;

    auto cmd =
        std::make_unique<doc::GroupEntitiesCommand>(m_document->draftDocument(), filteredIds);
    m_document->undoStack().push(std::move(cmd));
    m_viewport->update();
}

void MainWindow::onUngroupEntities() {
    auto& sel = m_viewport->selectionManager();
    auto ids = sel.selectedIds();
    if (ids.empty()) return;

    // Collect groupIds from selected entities.
    std::set<uint64_t> groupIds;
    for (const auto& entity : m_document->draftDocument().entities()) {
        if (!sel.isSelected(entity->id())) continue;
        if (entity->groupId() != 0) {
            groupIds.insert(entity->groupId());
        }
    }
    if (groupIds.empty()) return;

    std::vector<uint64_t> groupIdVec(groupIds.begin(), groupIds.end());
    auto cmd =
        std::make_unique<doc::UngroupEntitiesCommand>(m_document->draftDocument(), groupIdVec);
    m_document->undoStack().push(std::move(cmd));
    m_viewport->update();
}

// ---------------------------------------------------------------------------
// Slots -- Status bar updates
// ---------------------------------------------------------------------------

void MainWindow::onMouseMoved(const hz::math::Vec2& worldPos) {
    const Preferences& prefs = Preferences::current();
    m_statusCoords->setText(
        tr("X: %1  Y: %2").arg(prefs.formatLength(worldPos.x), prefs.formatLength(worldPos.y)));

    // Update tool prompt dynamically as mouse moves.
    if (m_viewport && m_viewport->activeTool()) m_statusPrompt->setText(toolPrompt());
}

void MainWindow::onSelectionChanged() {
    auto ids = m_viewport->selectionManager().selectedIds();
    std::vector<uint64_t> idVec(ids.begin(), ids.end());
    m_propertyPanel->updateForSelection(idVec);
    updateStatusBar();
}

// ---------------------------------------------------------------------------
// Slots -- 3D Primitives
// ---------------------------------------------------------------------------

bool MainWindow::requirePart(const QString& verb) {
    if (m_assembly) {
        statusBar()->showMessage(tr("%1 works on a part; open or create one").arg(verb));
        return false;
    }
    return true;
}

doc::BodyOperation MainWindow::proposedOperation() const {
    // Joining is what a second body usually means; the first has nothing to
    // join, so it starts one.
    return m_document->solid() ? doc::BodyOperation::Join : doc::BodyOperation::NewBody;
}

void MainWindow::addPrimitive(
    const QString& verb, const std::vector<PrimitiveField>& fields,
    const std::function<std::unique_ptr<doc::PrimitiveFeature>(const std::vector<double>&)>& make) {
    if (!requirePart(verb)) return;
    FeatureForm form(this, verb);
    std::vector<QDoubleSpinBox*> sizes;
    sizes.reserve(fields.size());
    for (size_t i = 0; i < fields.size(); ++i) {
        sizes.push_back(form.number(QStringLiteral("size%1").arg(i), fields[i].label,
                                    fields[i].value, fields[i].min, 1e6));
    }
    auto* result = form.operationChoice(proposedOperation());
    if (!form.exec()) return;

    std::vector<double> values;
    values.reserve(sizes.size());
    for (const auto* spin : sizes) values.push_back(spin->value());
    auto feature = make(values);
    feature->setOperation(FeatureForm::operation(result));
    addModelFeature(std::move(feature), verb);
}

void MainWindow::onPrimitiveBox() {
    // From the origin to (width, height, depth).
    addPrimitive(tr("Box"),
                 {{tr("Width (X):"), 10.0, 0.001},
                  {tr("Height (Y):"), 10.0, 0.001},
                  {tr("Depth (Z):"), 10.0, 0.001}},
                 [](const std::vector<double>& v) {
                     return doc::PrimitiveFeature::makeBox(v[0], v[1], v[2]);
                 });
}

void MainWindow::onPrimitiveCylinder() {
    addPrimitive(tr("Cylinder"), {{tr("Radius:"), 5.0, 0.001}, {tr("Height:"), 10.0, 0.001}},
                 [](const std::vector<double>& v) {
                     return doc::PrimitiveFeature::makeCylinder(v[0], v[1]);
                 });
}

void MainWindow::onPrimitiveSphere() {
    addPrimitive(tr("Sphere"), {{tr("Radius:"), 5.0, 0.001}}, [](const std::vector<double>& v) {
        return doc::PrimitiveFeature::makeSphere(v[0]);
    });
}

void MainWindow::onPrimitiveCone() {
    // A top radius of 0 is a pointed cone.
    addPrimitive(tr("Cone"),
                 {{tr("Bottom radius:"), 5.0, 0.0},
                  {tr("Top radius:"), 0.0, 0.0},
                  {tr("Height:"), 10.0, 0.001}},
                 [](const std::vector<double>& v) {
                     return doc::PrimitiveFeature::makeCone(v[0], v[1], v[2]);
                 });
}

void MainWindow::onPrimitiveTorus() {
    addPrimitive(
        tr("Torus"), {{tr("Ring radius:"), 6.0, 0.001}, {tr("Tube radius:"), 2.0, 0.001}},
        [](const std::vector<double>& v) { return doc::PrimitiveFeature::makeTorus(v[0], v[1]); });
}

// ---------------------------------------------------------------------------
// Slots -- Extrude and Revolve
// ---------------------------------------------------------------------------

std::shared_ptr<doc::Sketch> MainWindow::resolveProfileSketch(bool& createdWrapper) {
    createdWrapper = false;

    // The active sketch, when one is being edited.
    if (auto* activeSketch = m_viewport->activeSketch()) {
        for (const auto& sk : m_document->sketches()) {
            if (sk.get() == activeSketch) return sk;
        }
    }

    const auto& topEntities = m_document->draftDocument().entities();
    if (topEntities.empty()) return nullptr;

    // Reuse an existing wrapper sketch when the top-level profile has not
    // changed — repeated extrudes must not accumulate duplicate sketches.
    for (const auto& sk : m_document->sketches()) {
        if (sk->entities() == topEntities) return sk;
    }

    // Wrap the top-level profile in a sketch so the feature is replayable
    // (parametric history requires a sketch reference). The caller must add
    // it to the document only once the operation is validated.
    auto sketch = std::make_shared<doc::Sketch>();
    sketch->setName(tr("Profile %1").arg(m_document->sketches().size()).toStdString());
    for (const auto& entity : topEntities) sketch->addEntity(entity);
    createdWrapper = true;
    return sketch;
}

void MainWindow::onExtrudeSketch() {
    if (!m_viewport || !m_viewport->document()) return;

    bool createdWrapper = false;
    auto sketch = resolveProfileSketch(createdWrapper);
    if (!sketch || sketch->entities().empty()) {
        statusBar()->showMessage(tr("Draw a closed profile first"));
        return;
    }

    double distance = 10.0;
    doc::BodyOperation operation = doc::BodyOperation::Join;
    if (!askForBodyFeature(tr("Extrude"), tr("Distance:"), distance, 0.01, 1e6, 2, operation)) {
        return;
    }

    math::Vec3 direction = sketch->plane().normal();

    // Validate the profile BEFORE mutating the document: a failed extrude
    // must not leave a wrapper sketch or a dead feature behind.
    std::string why;
    auto probe = model::Extrude::execute(sketch->entities(), sketch->plane(), direction, distance,
                                         "probe", model::Extrude::kDefaultSegments, 0.0, &why);
    if (!probe) {
        statusBar()->showMessage(tr("Extrude failed: %1").arg(QString::fromStdString(why)));
        return;
    }

    auto feature = std::make_unique<doc::ExtrudeFeature>(sketch, direction, distance);
    feature->setOperation(operation);
    if (!addModelFeature(std::move(feature), tr("Extrude"), createdWrapper ? sketch : nullptr)) {
        return;
    }

    if (m_viewport->activeSketch()) m_viewport->setActiveSketch(nullptr);
    m_viewport->camera().setIsometricView();
    m_viewport->update();
}

void MainWindow::onRevolveSketch() {
    if (!m_viewport || !m_viewport->document()) return;

    bool createdWrapper = false;
    auto sketch = resolveProfileSketch(createdWrapper);
    if (!sketch || sketch->entities().empty()) {
        statusBar()->showMessage(tr("Draw a closed profile first"));
        return;
    }

    double angleDeg = 360.0;
    doc::BodyOperation operation = doc::BodyOperation::Join;
    if (!askForBodyFeature(tr("Revolve"), tr("Angle (degrees):"), angleDeg, 1.0, 360.0, 1,
                           operation)) {
        return;
    }

    const double angle = angleDeg * std::numbers::pi / 180.0;

    // Default revolve axis: Y axis through origin (world space)
    math::Vec3 axisPoint = math::Vec3::Zero;
    math::Vec3 axisDir = math::Vec3::UnitY;

    std::string why;
    auto probe =
        model::Revolve::execute(sketch->entities(), sketch->plane(), axisPoint, axisDir, angle,
                                "probe", model::Revolve::kDefaultSegments, 0.0, &why);
    if (!probe) {
        statusBar()->showMessage(tr("Revolve failed: %1").arg(QString::fromStdString(why)));
        return;
    }

    auto feature = std::make_unique<doc::RevolveFeature>(sketch, axisPoint, axisDir, angle);
    feature->setOperation(operation);
    if (!addModelFeature(std::move(feature), tr("Revolve"), createdWrapper ? sketch : nullptr)) {
        return;
    }

    if (m_viewport->activeSketch()) m_viewport->setActiveSketch(nullptr);
    m_viewport->camera().setIsometricView();
    m_viewport->update();
}

bool MainWindow::askForBodyFeature(const QString& title, const QString& valueLabel, double& value,
                                   double min, double max, int decimals,
                                   doc::BodyOperation& operation) {
    FeatureForm form(this, title);
    auto* size = form.number(QStringLiteral("size"), valueLabel, value, min, max, decimals);
    auto* result = form.operationChoice(proposedOperation());
    if (!form.exec()) return false;
    value = size->value();
    operation = FeatureForm::operation(result);
    return true;
}

bool MainWindow::addModelFeature(std::unique_ptr<doc::Feature> feature, const QString& verb,
                                 const std::shared_ptr<doc::Sketch>& wrapperSketch) {
    // Try the feature at the end of the active history first. One that fails
    // there itself (a Cut that would leave nothing, an Intersect of bodies
    // that do not touch) is refused, leaving the part — and the undo
    // history — as they were.
    auto& tree = m_document->featureTree();
    const int rollback = tree.rollbackIndex();
    tree.setRollbackIndex(-1);
    tree.addFeature(std::move(feature));
    const size_t index = tree.featureCount() - 1;
    // However the trial ends, the feature comes back out of the tree and the
    // rollback is put back: a feature left in the tree but not in the undo
    // history could never be undone.
    bool failsItself = true;
    QString reason;
    try {
        m_document->rebuildModel();
        failsItself = m_document->failedFeatureIndex() == static_cast<int>(index);
        reason = QString::fromStdString(m_document->lastBuildMessage());
    } catch (const std::exception& e) {
        reason = QString::fromUtf8(e.what());
    } catch (...) {
        reason = tr("an unknown error");
    }
    feature = tree.takeFeature(index);
    tree.setRollbackIndex(rollback);

    if (failsItself) {
        rebuildFeatureTree();
        statusBar()->showMessage(tr("%1 not added: %2").arg(verb, reason));
        return false;
    }

    m_document->undoStack().push(
        std::make_unique<doc::AddFeatureCommand>(*m_document, std::move(feature), wrapperSketch));
    rebuildFeatureTree();
    if (m_document->failedFeatureIndex() >= 0) {
        statusBar()->showMessage(tr("%1 added, but an earlier feature fails to rebuild").arg(verb));
    } else {
        m_statusPrompt->setText(tr("%1 added.").arg(verb));
    }
    return true;
}

// ---------------------------------------------------------------------------
// Slots -- Boolean Operations
// ---------------------------------------------------------------------------

void MainWindow::onBooleanUnion() {
    combineBodies(model::BooleanType::Union, tr("Union"));
}

void MainWindow::onBooleanSubtract() {
    combineBodies(model::BooleanType::Subtract, tr("Subtract"));
}

void MainWindow::onBooleanIntersect() {
    combineBodies(model::BooleanType::Intersect, tr("Intersect"));
}

void MainWindow::combineBodies(model::BooleanType type, const QString& verb) {
    if (!requirePart(verb)) return;
    // Bodies, not shells: a cavity is a shell of the body around it.
    const topo::Solid* solid = m_document->solid();
    if (!solid || model::Pattern::separate(*solid).size() < 2) {
        statusBar()->showMessage(
            tr("%1 combines the part's bodies, and it has fewer than two (make one with "
               "Result: New body)")
                .arg(verb));
        return;
    }
    addModelFeature(std::make_unique<doc::BooleanFeature>(type), verb);
}

// ---------------------------------------------------------------------------
// Slots -- Fillet, Chamfer, Shell, Draft (3D solid operations)
// ---------------------------------------------------------------------------

const topo::Solid* MainWindow::requireBody(const QString& verb) {
    if (!requirePart(verb)) return nullptr;
    const topo::Solid* solid = m_document->solid();
    if (!solid) {
        statusBar()->showMessage(tr("%1 works on a body: make one first").arg(verb));
    }
    return solid;
}

void MainWindow::onFillet() {
    addEdgeFeature(true);
}

void MainWindow::onChamfer() {
    addEdgeFeature(false);
}

void MainWindow::addEdgeFeature(bool fillet) {
    const QString verb = fillet ? tr("Fillet") : tr("Chamfer");
    const topo::Solid* solid = requireBody(verb);
    if (!solid) return;
    const PickList edges = edgesOf(*solid);

    FeatureForm form(this, verb);
    auto* size = form.number(QStringLiteral("size"), fillet ? tr("Radius:") : tr("Distance:"), 1.0,
                             0.001, 1e6);
    auto* list = form.checklist(QStringLiteral("edges"), tr("Edges:"), edges.items);
    if (!form.exec()) return;

    std::vector<topo::TopologyID> chosen;
    for (const int row : FeatureForm::checkedRows(list)) {
        chosen.push_back(edges.ids[static_cast<size_t>(row)]);
    }
    if (chosen.empty()) {
        statusBar()->showMessage(tr("%1 not added: no edges were chosen").arg(verb));
        return;
    }
    std::unique_ptr<doc::Feature> feature;
    if (fillet) {
        feature = std::make_unique<doc::FilletFeature>(std::move(chosen), size->value());
    } else {
        feature = std::make_unique<doc::ChamferFeature>(std::move(chosen), size->value());
    }
    addModelFeature(std::move(feature), verb);
}

void MainWindow::onShell() {
    const QString verb = tr("Shell");
    const topo::Solid* solid = requireBody(verb);
    if (!solid) return;
    const PickList faces = facesOf(*solid);

    FeatureForm form(this, verb);
    auto* thickness =
        form.number(QStringLiteral("thickness"), tr("Wall thickness:"), 1.0, 0.001, 1e6);
    auto* list = form.checklist(QStringLiteral("faces"), tr("Faces to open:"), faces.items);
    if (!form.exec()) return;

    std::vector<topo::TopologyID> open;
    for (const int row : FeatureForm::checkedRows(list)) {
        open.push_back(faces.ids[static_cast<size_t>(row)]);
    }
    addModelFeature(std::make_unique<doc::ShellFeature>(thickness->value(), std::move(open)), verb);
}

void MainWindow::onDraft() {
    const QString verb = tr("Draft");
    if (!requireBody(verb)) return;

    FeatureForm form(this, verb);
    auto* pull =
        directionChoice(form, QStringLiteral("pull"), tr("Pull direction:"), QStringLiteral("+Z"));
    auto* neutral = form.number(QStringLiteral("neutral"), tr("Neutral plane at:"), 0.0, -1e6, 1e6);
    auto* angle = form.number(QStringLiteral("angle"), tr("Angle (degrees):"), 3.0, 0.01, 89.0, 2);
    if (!form.exec()) return;

    // The neutral plane is square to the pull, at that distance along it.
    const math::Vec3 direction = chosenDirection(pull);
    addModelFeature(std::make_unique<doc::DraftFeature>(direction, direction * neutral->value(),
                                                        angle->value() * std::numbers::pi / 180.0),
                    verb);
}

// ---------------------------------------------------------------------------
// Slots -- Patterns
// ---------------------------------------------------------------------------

void MainWindow::onLinearPattern() {
    const QString verb = tr("Linear Pattern");
    if (!requireBody(verb)) return;

    FeatureForm form(this, verb);
    auto* direction =
        directionChoice(form, QStringLiteral("direction"), tr("Direction:"), QStringLiteral("+X"));
    auto* spacing = form.number(QStringLiteral("spacing"), tr("Spacing:"), 20.0, 0.001, 1e6);
    auto* count =
        form.count(QStringLiteral("count"), tr("Instances:"), 3, 2, doc::kMaxPatternCount);
    if (!form.exec()) return;

    addModelFeature(doc::PatternFeature::makeLinear(chosenDirection(direction), spacing->value(),
                                                    count->value()),
                    verb);
}

void MainWindow::onCircularPattern() {
    const QString verb = tr("Circular Pattern");
    if (!requireBody(verb)) return;

    FeatureForm form(this, verb);
    auto* axis =
        directionChoice(form, QStringLiteral("axis"), tr("About the axis:"), QStringLiteral("+Z"));
    auto* count =
        form.count(QStringLiteral("count"), tr("Instances:"), 4, 2, doc::kMaxPatternCount);
    auto* total = form.number(QStringLiteral("angle"), tr("Over (degrees):"), 360.0, 1.0, 360.0, 2);
    if (!form.exec()) return;

    // A full turn spaces the instances evenly around it; a partial one puts
    // the first and last at its ends.
    const int n = count->value();
    const double degrees = total->value();
    const double step = degrees >= 360.0 ? 360.0 / n : degrees / (n - 1);
    addModelFeature(doc::PatternFeature::makeCircular(math::Vec3::Zero, chosenDirection(axis),
                                                      step * std::numbers::pi / 180.0, n),
                    verb);
}

// ---------------------------------------------------------------------------
// Slots -- Feature Tree Panel
// ---------------------------------------------------------------------------

const doc::Feature* MainWindow::featureAt(int featureIndex) const {
    const auto& tree = m_document->featureTree();
    if (featureIndex < 0 || static_cast<size_t>(featureIndex) >= tree.featureCount()) {
        return nullptr;
    }
    return tree.feature(static_cast<size_t>(featureIndex));
}

void MainWindow::onFeatureDoubleClicked(int featureIndex) {
    const doc::Feature* feat = featureAt(featureIndex);
    if (!feat) return;

    // One dialog for all of the feature's values (it used to be one prompt
    // per parameter, with no way to back out of the ones already answered).
    const auto params = feat->parameters();
    const bool buildsBody = feat->createsNewBody();
    if (params.empty() && !buildsBody) {
        statusBar()->showMessage(
            tr("%1 has nothing to edit").arg(QString::fromStdString(feat->name())));
        return;
    }

    FeatureForm form(this, tr("Edit %1").arg(QString::fromStdString(feat->name())));
    // Each box shows the stored value as it is — a floor would turn a legal 0
    // (a pointed cone's radius, a Union's operation code) into something else
    // before anyone touched it. The feature refuses a value it cannot use.
    // What each box showed is the baseline: a spin box rounds what it is given
    // to its decimals (a 360° revolve's 2π shows as 6.2832), so an untouched
    // box is one that still shows that, not one equal to the stored value.
    std::map<std::string, std::pair<QDoubleSpinBox*, double>> spins;
    for (const auto& [name, value] : params) {
        auto* spin =
            form.number(QString::fromStdString(name),
                        QString::fromStdString(name) + QStringLiteral(":"), value, -1e9, 1e9, 4);
        spins[name] = {spin, spin->value()};
    }
    QComboBox* result = buildsBody ? form.operationChoice(feat->operation()) : nullptr;
    if (!form.exec()) return;

    std::map<std::string, double> changed;
    for (const auto& [name, box] : spins) {
        const auto& [spin, shown] = box;
        if (spin->value() != shown) changed[name] = spin->value();
    }
    // Leave out what the feature refuses (a zero distance, too few segments),
    // and say so.
    QStringList refused;
    if (doc::Feature* target =
            m_document->featureTree().feature(static_cast<size_t>(featureIndex))) {
        for (const std::string& name : doc::refusedParameters(*target, changed)) {
            refused << QString::fromStdString(name);
            changed.erase(name);
        }
    }
    if (!refused.isEmpty()) {
        statusBar()->showMessage(
            tr("%1 cannot use the value given for: %2")
                .arg(QString::fromStdString(feat->name()), refused.join(QStringLiteral(", "))));
    }
    std::optional<doc::BodyOperation> operation;
    if (result) {
        const auto chosen = FeatureForm::operation(result);
        if (chosen != feat->operation()) operation = chosen;
    }
    if (changed.empty() && !operation) return;

    m_document->undoStack().push(std::make_unique<doc::EditFeatureCommand>(
        *m_document, feat, std::move(changed), operation));
    rebuildFeatureTree();
}

void MainWindow::onFeatureReordered(int fromIndex, int toIndex) {
    const doc::Feature* feat = featureAt(fromIndex);
    if (!feat || toIndex < 0 || fromIndex == toIndex) {
        rebuildFeatureTree();  // the panel may show a move that did not happen
        return;
    }
    m_document->undoStack().push(
        std::make_unique<doc::MoveFeatureCommand>(*m_document, feat, static_cast<size_t>(toIndex)));
    rebuildFeatureTree();
}

void MainWindow::onFeatureDeleteRequested(int featureIndex) {
    const doc::Feature* feat = featureAt(featureIndex);
    if (!feat) return;
    m_document->undoStack().push(std::make_unique<doc::RemoveFeatureCommand>(*m_document, feat));
    rebuildFeatureTree();
}

void MainWindow::onFeatureSuppressRequested(int featureIndex, bool suppress) {
    const doc::Feature* feat = featureAt(featureIndex);
    if (!feat || feat->isSuppressed() == suppress) return;
    m_document->undoStack().push(
        std::make_unique<doc::SetFeatureSuppressedCommand>(*m_document, feat, suppress));
    rebuildFeatureTree();
}

void MainWindow::onRollbackChanged(int newIndex) {
    m_document->featureTree().setRollbackIndex(newIndex);
    rebuildFeatureTree();
}

void MainWindow::rebuildFeatureTree() {
    DocTab* tab = activeTab();
    const bool onWorker =
        tab != nullptr && !m_assembly &&
        (m_rebuildMode == RebuildMode::Always ||
         (m_rebuildMode == RebuildMode::Auto && tab->lastBuildMs >= kWorkerRebuildMs));
    if (onWorker) {
        startRebuild();
        return;
    }
    QElapsedTimer clock;
    clock.start();
    m_document->rebuildModel();
    if (tab) {
        tab->lastBuildMs = clock.elapsed();
        tab->modelStale = false;
    }
    showBuildResult();
}

void MainWindow::showBuildResult() {
    m_featureTreePanel->clearFailures();
    m_featureTreePanel->refresh(m_document->featureTree());

    if (m_document->failedFeatureIndex() >= 0) {
        m_featureTreePanel->markFailed(m_document->failedFeatureIndex(),
                                       m_document->lastBuildMessage());
        statusBar()->showMessage(tr("Feature rebuild failed at feature %1: %2")
                                     .arg(m_document->failedFeatureIndex())
                                     .arg(QString::fromStdString(m_document->lastBuildMessage())));
    }

    rebuildScene();
}

void MainWindow::startRebuild() {
    if (m_rebuildJob) {
        // One at a time: this one starts when the running one is done, from
        // the active document as it is then. A running rebuild of this same
        // document is out of date already, so it stops at its next feature;
        // one of another document finishes, and is applied to it.
        m_rebuildAgain = true;
        if (m_rebuildDocument == m_document) m_rebuildJob->cancel();
        return;
    }
    m_rebuildDocument = m_document;
    m_rebuildJob = std::make_unique<RebuildJob>(*m_document);
    m_rebuildClock.start();
    // Posted from the worker to this window's thread. The destructor waits
    // for the worker, and Qt drops what is still queued for a deleted object.
    m_rebuildJob->start([this] {
        QMetaObject::invokeMethod(this, &MainWindow::onRebuildFinished, Qt::QueuedConnection);
    });
    m_rebuildPoll->start();
    m_statusPrompt->setText(tr("Rebuilding the model..."));
    updateBusyIndicator();
}

void MainWindow::updateBusyIndicator() {
    const bool busy = backgroundWorkRunning();
    m_rebuildProgress->setVisible(busy);
    m_rebuildCancel->setVisible(busy);
    // Busy (no steps) until a rebuild, alone, knows how many features it has.
    m_rebuildProgress->setRange(0, 0);
    if (busy) updateRebuildProgress();
}

void MainWindow::updateRebuildProgress() {
    if (!m_rebuildJob || m_importTask || m_interferenceTask) return;
    const int total = m_rebuildJob->total();
    if (total > 0) {
        m_rebuildProgress->setRange(0, total);
        m_rebuildProgress->setValue(m_rebuildJob->done());
    }
}

void MainWindow::onRebuildFinished() {
    if (!m_rebuildJob || !m_rebuildJob->finished()) return;
    // The worker is done; destroying the job only joins its thread.
    const std::unique_ptr<RebuildJob> job = std::move(m_rebuildJob);
    const std::shared_ptr<doc::Document> document = std::move(m_rebuildDocument);
    bool again = std::exchange(m_rebuildAgain, false);
    m_rebuildPoll->stop();
    updateBusyIndicator();

    DocTab* tab = nullptr;
    for (DocTab& t : m_tabs) {
        if (t.document == document) tab = &t;
    }
    // A tab closed meanwhile has nothing to apply to.
    if (tab != nullptr) {
        doc::BuildResult result = job->takeResult();
        const bool cancelled = result.cancelled;
        const bool active = document == m_document;
        if (!cancelled && RebuildJob::stampOf(*document) == job->stamp()) {
            document->applyBuild(std::move(result));
            tab->lastBuildMs = m_rebuildClock.elapsed();
            tab->modelStale = false;
            if (active && !again) {
                m_statusPrompt->setText(tr("Ready"));
                showBuildResult();
            }
        } else {
            // Not applied: the model is behind its features until a rebuild
            // catches up — at once if it changed while this one ran, or when
            // its tab is next shown (after a Cancel, too).
            tab->modelStale = true;
            if (active && !cancelled) {
                again = true;
            } else if (active && !again) {
                m_statusPrompt->setText(tr("Ready"));
                statusBar()->showMessage(
                    tr("Rebuild cancelled: the model is as it was before the last change."), 10000);
            }
        }
    }
    if (again) startRebuild();  // the active document, as it is now
}

}  // namespace hz::ui
