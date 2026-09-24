#include "horizon/ui/MainWindow.h"

#include <spdlog/spdlog.h>

#include <QAbstractButton>
#include <QAction>
#include <QActionGroup>
#include <QCloseEvent>
#include <QComboBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
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
#include <QSettings>
#include <QStandardPaths>
#include <QStatusBar>
#include <QTabBar>
#include <QTimer>
#include <QToolBar>
#include <QVBoxLayout>
#include <filesystem>
#include <functional>
#include <map>
#include <numbers>
#include <optional>
#include <utility>

#include "horizon/document/Commands.h"
#include "horizon/document/ModelCommands.h"
#include "horizon/document/UndoStack.h"
#include "horizon/drafting/DraftBlockRef.h"
#include "horizon/fileio/DxfFormat.h"
#include "horizon/fileio/NativeFormat.h"
#include "horizon/math/BoundingBox.h"
#include "horizon/math/MathUtils.h"
#include "horizon/modeling/AssemblySolver.h"
#include "horizon/modeling/BooleanOp.h"
#include "horizon/modeling/ChamferOp.h"
#include "horizon/modeling/Extrude.h"
#include "horizon/modeling/FilletOp.h"
#include "horizon/modeling/MateGeometry.h"
#include "horizon/modeling/PrimitiveFactory.h"
#include "horizon/modeling/Revolve.h"
#include "horizon/modeling/SolidTessellator.h"
#include "horizon/render/SceneGraph.h"
#include "horizon/topology/Solid.h"
#include "horizon/ui/AngularDimensionTool.h"
#include "horizon/ui/ArcTool.h"
#include "horizon/ui/BreakTool.h"
#include "horizon/ui/ChamferTool.h"
#include "horizon/ui/CircleTool.h"
#include "horizon/ui/Clipboard.h"
#include "horizon/ui/CommandPalette.h"
#include "horizon/ui/ConstraintTool.h"
#include "horizon/ui/EllipseTool.h"
#include "horizon/ui/ExtendTool.h"
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
#include "horizon/ui/MeasureAngleTool.h"
#include "horizon/ui/MeasureAreaTool.h"
#include "horizon/ui/MeasureDistanceTool.h"
#include "horizon/ui/MirrorTool.h"
#include "horizon/ui/MoveTool.h"
#include "horizon/ui/OffsetTool.h"
#include "horizon/ui/PasteTool.h"
#include "horizon/ui/PolarArrayDialog.h"
#include "horizon/ui/PolylineEditTool.h"
#include "horizon/ui/PolylineTool.h"
#include "horizon/ui/PropertyPanel.h"
#include "horizon/ui/RadialDimensionTool.h"
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

/// Add the "how does this body combine with the part" choice to a feature
/// dialog, showing `initial`.
QComboBox* addOperationChoice(QDialog& dialog, QFormLayout& form, doc::BodyOperation initial) {
    auto* combo = new QComboBox(&dialog);
    combo->setObjectName(QStringLiteral("bodyOperation"));
    combo->addItem(MainWindow::tr("Join the part"), static_cast<int>(doc::BodyOperation::Join));
    combo->addItem(MainWindow::tr("Cut from the part"), static_cast<int>(doc::BodyOperation::Cut));
    combo->addItem(MainWindow::tr("Keep the intersection"),
                   static_cast<int>(doc::BodyOperation::Intersect));
    combo->addItem(MainWindow::tr("New body"), static_cast<int>(doc::BodyOperation::NewBody));
    combo->setCurrentIndex(combo->findData(static_cast<int>(initial)));
    form.addRow(MainWindow::tr("Result:"), combo);
    return combo;
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
        return io::NativeFormat::load(path, doc, &m_lastLoadError);
    });
    m_docManager.setMeshLoader(
        [](const std::string& path) { return io::NativeFormat::loadPartMesh(path); });
    m_docManager.setAssemblyLoader([this](const std::string& path, doc::AssemblyDocument& doc) {
        m_lastLoadError.clear();
        return io::NativeFormat::loadAssembly(path, doc, &m_lastLoadError);
    });

    // Autosave snapshots go to a per-session directory; a crash leaves them
    // for offerRecovery() on the next start.
    m_recovery = std::make_unique<RecoveryManager>(
        QStandardPaths::writableLocation(QStandardPaths::AppLocalDataLocation) + "/recovery");
    m_autosaveTimer = new QTimer(this);
    const int autosaveSeconds = QSettings().value("autosave/intervalSeconds", 120).toInt();
    if (autosaveSeconds > 0) {
        connect(m_autosaveTimer, &QTimer::timeout, this, &MainWindow::autosave);
        m_autosaveTimer->start(autosaveSeconds * 1000);
    }

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
    m_propertyPanel = new PropertyPanel(this, this);
    addDockWidget(Qt::RightDockWidgetArea, m_propertyPanel);

    m_layerPanel = new LayerPanel(this, this);
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

    // Wire up selection changes to property panel.
    connect(m_viewport, &ViewportWidget::selectionChanged, this, &MainWindow::onSelectionChanged);

    // Start with the Select tool active.
    onSelectTool();
}

MainWindow::~MainWindow() = default;

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

    QAction* saveAction = fileMenu->addAction(tr("&Save"), this, &MainWindow::onSaveFile);
    saveAction->setShortcut(QKeySequence::Save);

    QAction* saveAsAction = fileMenu->addAction(tr("Save &As..."), this, &MainWindow::onSaveFileAs);
    saveAsAction->setShortcut(QKeySequence::SaveAs);

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

    // ---- View ----
    QMenu* viewMenu = menuBar()->addMenu(tr("&View"));

    viewMenu->addAction(tr("&Front"), this, &MainWindow::onViewFront);
    viewMenu->addAction(tr("&Top"), this, &MainWindow::onViewTop);
    viewMenu->addAction(tr("&Right"), this, &MainWindow::onViewRight);
    viewMenu->addAction(tr("&Isometric"), this, &MainWindow::onViewIsometric);
    viewMenu->addSeparator();
    viewMenu->addAction(tr("Fit &All"), this, &MainWindow::onFitAll);
    viewMenu->addSeparator();
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
    dimMenu->addSeparator();
    dimMenu->addAction(tr("L&eader"), this, &MainWindow::onLeaderTool);

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
        act->setCheckable(true);
        act->setToolTip(
            shortcut.isEmpty()
                ? tooltip
                : QString("%1 (%2)").arg(tooltip, shortcut.toString(QKeySequence::NativeText)));
        if (!shortcut.isEmpty()) act->setShortcut(shortcut);
        toolGroup->addAction(act);
        return act;
    };

    auto addAction = [](QToolBar* tb, const QString& iconName, const QString& tooltip,
                        auto* receiver, auto slot, const QKeySequence& shortcut = {}) -> QAction* {
        auto* act = tb->addAction(IconGenerator::icon(iconName), tooltip, receiver, slot);
        act->setObjectName(QStringLiteral("action_") + iconName);  // for tests and automation
        act->setToolTip(
            shortcut.isEmpty()
                ? tooltip
                : QString("%1 (%2)").arg(tooltip, shortcut.toString(QKeySequence::NativeText)));
        if (!shortcut.isEmpty()) act->setShortcut(shortcut);
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

    g = group(tr("3D"), tr("Boolean"));
    addAction(g, "boolean-union", tr("Union"), this, &MainWindow::onBooleanUnion);
    addAction(g, "boolean-subtract", tr("Subtract"), this, &MainWindow::onBooleanSubtract);
    addAction(g, "boolean-intersect", tr("Intersect"), this, &MainWindow::onBooleanIntersect);

    g = group(tr("3D"), tr("Modify"));
    addAction(g, "fillet-3d", tr("Fillet"), this, &MainWindow::onFillet);
    addAction(g, "chamfer-3d", tr("Chamfer"), this, &MainWindow::onChamfer);

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
    m_statusPrompt->setStyleSheet("QLabel { padding: 0 6px; color: #a0c4ff; }");
    sb->addWidget(m_statusPrompt, 1);

    // Snap/grid indicator.
    m_statusSnap = new QLabel(tr("SNAP  GRID"), this);
    m_statusSnap->setStyleSheet("QLabel { padding: 0 6px; color: #80cc80; }");
    sb->addPermanentWidget(m_statusSnap);

    // Selection count.
    m_statusSelection = new QLabel(tr("0 selected"), this);
    m_statusSelection->setMinimumWidth(80);
    m_statusSelection->setStyleSheet("QLabel { padding: 0 6px; }");
    sb->addPermanentWidget(m_statusSelection);

    // Active tool name.
    m_statusTool = new QLabel(tr("Select"), this);
    m_statusTool->setMinimumWidth(80);
    m_statusTool->setStyleSheet("QLabel { padding: 0 6px; font-weight: bold; color: #ffd080; }");
    sb->addPermanentWidget(m_statusTool);
}

void MainWindow::updateStatusBar() {
    if (m_viewport && m_viewport->activeTool()) {
        auto* tool = m_viewport->activeTool();
        m_statusTool->setText(QString::fromStdString(tool->name()));
        auto prompt = tool->promptText();
        if (!prompt.empty()) {
            m_statusPrompt->setText(QString::fromStdString(prompt));
        } else {
            m_statusPrompt->setText(tr("Ready"));
        }
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
    rebuildScene();
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
        if (!m_document->solid()) m_document->rebuildModel();
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
}

void MainWindow::autosave() {
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
        const bool written = tab.assembly
                                 ? m_recovery->snapshot(tab.recoveryKey, *tab.assembly, tab.title,
                                                        QString::fromStdString(path))
                                 : m_recovery->snapshot(tab.recoveryKey, *tab.document, tab.title,
                                                        QString::fromStdString(path));
        if (written) tab.snapshotStale = false;
    }
}

void MainWindow::offerRecovery() {
    const std::vector<RecoveryManager::Entry> orphans = m_recovery->claimOrphans();
    if (orphans.empty()) return;

    QString list;
    for (const auto& entry : orphans) {
        list += QStringLiteral("\n  \u2022 %1 (%2)")
                    .arg(entry.title,
                         QLocale().toString(entry.savedAt.toLocalTime(), QLocale::ShortFormat));
    }
    QMessageBox box(QMessageBox::Warning, tr("Recover Documents"),
                    tr("Horizon CAD did not shut down properly. These documents had unsaved "
                       "changes and can be recovered:%1")
                        .arg(list),
                    QMessageBox::Yes | QMessageBox::Discard | QMessageBox::Cancel, this);
    box.button(QMessageBox::Yes)->setText(tr("Recover"));
    box.button(QMessageBox::Cancel)->setText(tr("Later"));
    box.setDefaultButton(QMessageBox::Yes);
    box.setEscapeButton(QMessageBox::Cancel);
    const int choice = box.exec();
    if (choice == QMessageBox::Cancel) return;  // kept for the next start
    if (choice == QMessageBox::Discard) {
        m_recovery->discardClaimedOrphans();
        return;
    }

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
        }
    }
    refreshModifiedIndicators();
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

void MainWindow::onOpenFile() {
    QString fileName = QFileDialog::getOpenFileName(
        this, tr("Open File"), QString(),
        tr("All Supported Files (*.hcad *.hzpart *.hzasm *.dxf);;"
           "Horizon CAD Drawings (*.hcad);;Horizon Parts (*.hzpart);;"
           "Horizon Assemblies (*.hzasm);;DXF Files (*.dxf);;All Files (*)"));
    if (fileName.isEmpty()) return;

    std::string path = fileName.toStdString();

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
            return;
        }
    }

    if (fileName.endsWith(".hzasm", Qt::CaseInsensitive)) {
        auto assembly = m_docManager.openAssembly(path);
        if (!assembly) {
            reportFileError(tr("Could not open"), path, m_lastLoadError);
            return;
        }
        // The manager dedups by canonical path — an existing instance means
        // some tab already shows this assembly; focus it instead of adding
        // a second tab aliasing the same object.
        for (size_t i = 0; i < m_tabs.size(); ++i) {
            if (m_tabs[i].assembly == assembly) {
                m_tabBar->setCurrentIndex(static_cast<int>(i));
                return;
            }
        }
        // Saved assemblies come back positioned by their mates.
        solveAssemblyMates(*assembly);
        auto backing = m_docManager.newDocument(doc::DocumentType::Assembly);
        addDocumentTab(std::move(backing), std::move(assembly),
                       tabTitleForPath(path, tr("Assembly")));
        return;
    }

    if (fileName.endsWith(".dxf", Qt::CaseInsensitive)) {
        auto document = m_docManager.newDocument(doc::DocumentType::Drawing);
        std::string error;
        if (!io::DxfFormat::load(path, *document, &error)) {
            m_docManager.closeDocument(document);
            reportFileError(tr("Could not open"), path, error);
            return;
        }
        document->setFilePath(path);
        document->setDirty(false);
        addDocumentTab(std::move(document), nullptr, tabTitleForPath(path, tr("Drawing")));
        return;
    }

    // .hcad and .hzpart both load through NativeFormat (full document —
    // entities, sketches, feature tree, design variables).
    auto document = m_docManager.openPart(path);
    if (!document) {
        reportFileError(tr("Could not open"), path, m_lastLoadError);
        return;
    }
    // Dedup hit → the document is already shown in some tab; focus it.
    for (size_t i = 0; i < m_tabs.size(); ++i) {
        if (m_tabs[i].document == document) {
            m_tabBar->setCurrentIndex(static_cast<int>(i));
            return;
        }
    }
    addDocumentTab(std::move(document), nullptr, tabTitleForPath(path, tr("Document")));
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
        // assembly loading.
        if (m_document->featureTree().featureCount() > 0 && !m_document->solid()) {
            m_document->rebuildModel();
        }
        ok = io::NativeFormat::save(path, *m_document, &error);
    }
    if (ok) {
        m_document->setDirty(false);
        m_docManager.noteSaved(m_document);
        forgetSnapshot(*tab);
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

    const auto report = m_assembly->findInterference();
    auto nameOf = [this](uint64_t id) {
        const auto* comp = m_assembly->component(id);
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
    auto composite = std::make_unique<doc::CompositeCommand>("Cut");
    for (const auto& entity : m_document->draftDocument().entities()) {
        if (!sel.isSelected(entity->id())) continue;
        const auto* lp = layerMgr.getLayer(entity->layer());
        if (!lp || !lp->visible || lp->locked) continue;
        composite->addCommand(
            std::make_unique<doc::RemoveEntityCommand>(m_document->draftDocument(), entity->id()));
    }
    if (!composite->empty()) {
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
                for (const auto& entity : m_document->draftDocument().entities()) {
                    if (entity->id() == id) {
                        auto clone = entity->clone();
                        clone->translate(offset);
                        newIds.push_back(clone->id());
                        allClones.push_back(clone);
                        composite->addCommand(std::make_unique<doc::AddEntityCommand>(
                            m_document->draftDocument(), clone));
                        break;
                    }
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
            for (const auto& entity : m_document->draftDocument().entities()) {
                if (entity->id() == id) {
                    auto clone = entity->clone();
                    clone->rotate(center, angle);
                    newIds.push_back(clone->id());
                    allClones.push_back(clone);
                    composite->addCommand(std::make_unique<doc::AddEntityCommand>(
                        m_document->draftDocument(), clone));
                    break;
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

// ---------------------------------------------------------------------------
// Slots -- Dimension tools
// ---------------------------------------------------------------------------

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

    bool ok = false;
    QString name = QInputDialog::getText(this, tr("Create Block"), tr("Block name:"),
                                         QLineEdit::Normal, QString(), &ok);
    if (!ok || name.trimmed().isEmpty()) return;

    std::string blockName = name.trimmed().toStdString();
    if (m_document->draftDocument().blockTable().findBlock(blockName)) {
        QMessageBox::warning(this, tr("Create Block"),
                             tr("A block with that name already exists."));
        return;
    }

    auto cmd = std::make_unique<doc::CreateBlockCommand>(m_document->draftDocument(), blockName,
                                                         filteredIds);
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

    // Create the tool and set it active.  The tool is owned by ToolManager lifetime
    // so we manage it independently (it replaces any existing active tool).
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
    m_statusCoords->setText(
        QString("X: %1  Y: %2").arg(worldPos.x, 0, 'f', 3).arg(worldPos.y, 0, 'f', 3));

    // Update tool prompt dynamically as mouse moves.
    if (m_viewport && m_viewport->activeTool()) {
        auto prompt = m_viewport->activeTool()->promptText();
        if (!prompt.empty()) {
            m_statusPrompt->setText(QString::fromStdString(prompt));
        }
    }
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

void MainWindow::onPrimitiveBox() {
    auto solid = model::PrimitiveFactory::makeBox(10.0, 10.0, 10.0);
    auto meshData = model::SolidTessellator::tessellate(*solid, 0.1);

    auto node = std::make_shared<render::SceneNode>("Box");
    node->setMesh(std::make_unique<render::MeshData>(std::move(meshData)));
    node->setMaterial(render::Material{math::Vec3{0.6, 0.75, 0.85}, 0.15f, 0.5f, 32.0f});

    m_viewport->sceneGraph().addNode(node);
    m_viewport->camera().setIsometricView();
    m_viewport->update();
    m_statusPrompt->setText(tr("Box primitive added."));
}

void MainWindow::onPrimitiveCylinder() {
    auto solid = model::PrimitiveFactory::makeCylinder(5.0, 10.0);
    auto meshData = model::SolidTessellator::tessellate(*solid, 0.1);

    auto node = std::make_shared<render::SceneNode>("Cylinder");
    node->setMesh(std::make_unique<render::MeshData>(std::move(meshData)));
    node->setMaterial(render::Material{math::Vec3{0.85, 0.65, 0.55}, 0.15f, 0.5f, 32.0f});

    m_viewport->sceneGraph().addNode(node);
    m_viewport->camera().setIsometricView();
    m_viewport->update();
    m_statusPrompt->setText(tr("Cylinder primitive added."));
}

void MainWindow::onPrimitiveSphere() {
    auto solid = model::PrimitiveFactory::makeSphere(5.0);
    auto meshData = model::SolidTessellator::tessellate(*solid, 0.1);

    auto node = std::make_shared<render::SceneNode>("Sphere");
    node->setMesh(std::make_unique<render::MeshData>(std::move(meshData)));
    node->setMaterial(render::Material{math::Vec3{0.55, 0.8, 0.55}, 0.15f, 0.5f, 32.0f});

    m_viewport->sceneGraph().addNode(node);
    m_viewport->camera().setIsometricView();
    m_viewport->update();
    m_statusPrompt->setText(tr("Sphere primitive added."));
}

void MainWindow::onPrimitiveCone() {
    auto solid = model::PrimitiveFactory::makeCone(5.0, 0.0, 10.0);
    auto meshData = model::SolidTessellator::tessellate(*solid, 0.1);

    auto node = std::make_shared<render::SceneNode>("Cone");
    node->setMesh(std::make_unique<render::MeshData>(std::move(meshData)));
    node->setMaterial(render::Material{math::Vec3{0.85, 0.75, 0.4}, 0.15f, 0.5f, 32.0f});

    m_viewport->sceneGraph().addNode(node);
    m_viewport->camera().setIsometricView();
    m_viewport->update();
    m_statusPrompt->setText(tr("Cone primitive added."));
}

void MainWindow::onPrimitiveTorus() {
    auto solid = model::PrimitiveFactory::makeTorus(6.0, 2.0);
    auto meshData = model::SolidTessellator::tessellate(*solid, 0.1);

    auto node = std::make_shared<render::SceneNode>("Torus");
    node->setMesh(std::make_unique<render::MeshData>(std::move(meshData)));
    node->setMaterial(render::Material{math::Vec3{0.7, 0.55, 0.8}, 0.15f, 0.5f, 32.0f});

    m_viewport->sceneGraph().addNode(node);
    m_viewport->camera().setIsometricView();
    m_viewport->update();
    m_statusPrompt->setText(tr("Torus primitive added."));
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
    if (!addBodyFeature(std::move(feature), tr("Extrude"), createdWrapper ? sketch : nullptr)) {
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
    if (!addBodyFeature(std::move(feature), tr("Revolve"), createdWrapper ? sketch : nullptr)) {
        return;
    }

    if (m_viewport->activeSketch()) m_viewport->setActiveSketch(nullptr);
    m_viewport->camera().setIsometricView();
    m_viewport->update();
}

bool MainWindow::askForBodyFeature(const QString& title, const QString& valueLabel, double& value,
                                   double min, double max, int decimals,
                                   doc::BodyOperation& operation) {
    QDialog dialog(this);
    dialog.setWindowTitle(title);
    auto* form = new QFormLayout(&dialog);

    auto* size = new QDoubleSpinBox(&dialog);
    size->setRange(min, max);
    size->setDecimals(decimals);
    size->setValue(value);
    form->addRow(valueLabel, size);

    // Joining is what a second feature usually means; the first body has
    // nothing to join, so it starts one.
    const bool hasBody = m_document->solid() != nullptr;
    auto* result = addOperationChoice(
        dialog, *form, hasBody ? doc::BodyOperation::Join : doc::BodyOperation::NewBody);

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog);
    connect(buttons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
    form->addRow(buttons);

    if (dialog.exec() != QDialog::Accepted) return false;
    value = size->value();
    operation = static_cast<doc::BodyOperation>(result->currentData().toInt());
    return true;
}

bool MainWindow::addBodyFeature(std::unique_ptr<doc::Feature> feature, const QString& verb,
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
    m_document->rebuildModel();
    const bool failsItself = m_document->failedFeatureIndex() == static_cast<int>(index);
    const QString reason = QString::fromStdString(m_document->lastBuildMessage());
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
    // Demo: union of two overlapping boxes.
    auto boxA = model::PrimitiveFactory::makeBox(10, 10, 10);
    auto boxB = model::PrimitiveFactory::makeBox(10, 10, 10);
    for (auto& v : const_cast<std::deque<topo::Vertex>&>(boxB->vertices())) {
        v.point.x += 5.0;
    }

    auto result = model::BooleanOp::execute(*boxA, *boxB, model::BooleanType::Union);
    if (!result) {
        statusBar()->showMessage(tr("Boolean union failed"));
        return;
    }

    auto meshData = model::SolidTessellator::tessellate(*result, 0.1);
    auto node = std::make_shared<render::SceneNode>("Boolean Union");
    node->setMesh(std::make_unique<render::MeshData>(std::move(meshData)));
    node->setMaterial(render::Material{math::Vec3{0.4, 0.75, 0.55}, 0.15f, 0.5f, 32.0f});

    m_viewport->sceneGraph().addNode(node);
    m_viewport->camera().setIsometricView();
    m_viewport->update();
    m_statusPrompt->setText(tr("Boolean union completed."));
}

void MainWindow::onBooleanSubtract() {
    // Demo: box with a rectangular channel cut through it.
    auto boxA = model::PrimitiveFactory::makeBox(10, 10, 10);
    auto boxB = model::PrimitiveFactory::makeBox(4, 4, 20);
    for (auto& v : const_cast<std::deque<topo::Vertex>&>(boxB->vertices())) {
        v.point.x += 3.0;
        v.point.y += 3.0;
        v.point.z -= 5.0;
    }

    auto result = model::BooleanOp::execute(*boxA, *boxB, model::BooleanType::Subtract);
    if (!result) {
        statusBar()->showMessage(tr("Boolean subtract failed"));
        return;
    }

    auto meshData = model::SolidTessellator::tessellate(*result, 0.1);
    auto node = std::make_shared<render::SceneNode>("Boolean Subtract");
    node->setMesh(std::make_unique<render::MeshData>(std::move(meshData)));
    node->setMaterial(render::Material{math::Vec3{0.85, 0.45, 0.45}, 0.15f, 0.5f, 32.0f});

    m_viewport->sceneGraph().addNode(node);
    m_viewport->camera().setIsometricView();
    m_viewport->update();
    m_statusPrompt->setText(tr("Boolean subtract completed."));
}

void MainWindow::onBooleanIntersect() {
    // Demo: intersection of two overlapping boxes.
    auto boxA = model::PrimitiveFactory::makeBox(10, 10, 10);
    auto boxB = model::PrimitiveFactory::makeBox(10, 10, 10);
    for (auto& v : const_cast<std::deque<topo::Vertex>&>(boxB->vertices())) {
        v.point.x += 5.0;
        v.point.y += 5.0;
        v.point.z += 5.0;
    }

    auto result = model::BooleanOp::execute(*boxA, *boxB, model::BooleanType::Intersect);
    if (!result) {
        statusBar()->showMessage(tr("Boolean intersect failed"));
        return;
    }

    auto meshData = model::SolidTessellator::tessellate(*result, 0.1);
    auto node = std::make_shared<render::SceneNode>("Boolean Intersect");
    node->setMesh(std::make_unique<render::MeshData>(std::move(meshData)));
    node->setMaterial(render::Material{math::Vec3{0.55, 0.55, 0.85}, 0.15f, 0.5f, 32.0f});

    m_viewport->sceneGraph().addNode(node);
    m_viewport->camera().setIsometricView();
    m_viewport->update();
    m_statusPrompt->setText(tr("Boolean intersect completed."));
}

// ---------------------------------------------------------------------------
// Slots -- Fillet / Chamfer (3D solid operations)
// ---------------------------------------------------------------------------

void MainWindow::onFillet() {
    // Demo: fillet one edge of a box.
    auto box = model::PrimitiveFactory::makeBox(10, 10, 10);
    auto& edges = box->edges();
    if (edges.empty()) {
        statusBar()->showMessage(tr("No edges to fillet"));
        return;
    }
    std::vector<topo::TopologyID> edgeIds = {edges.front().topoId};

    auto result = model::FilletOp::execute(*box, edgeIds, 1.0, "fillet_demo");
    if (!result.solid || !result.errorMessage.empty()) {
        statusBar()->showMessage(
            tr("Fillet failed: %1").arg(QString::fromStdString(result.errorMessage)));
        return;
    }

    auto meshData = model::SolidTessellator::tessellate(*result.solid, 0.1);
    auto node = std::make_shared<render::SceneNode>("Fillet Demo");
    node->setMesh(std::make_unique<render::MeshData>(std::move(meshData)));
    node->setMaterial(render::Material{math::Vec3{0.85, 0.65, 0.35}, 0.15f, 0.5f, 32.0f});

    m_viewport->sceneGraph().addNode(node);
    m_viewport->camera().setIsometricView();
    m_viewport->update();
    m_statusPrompt->setText(tr("Fillet completed."));
}

void MainWindow::onChamfer() {
    // Demo: chamfer one edge of a box.
    auto box = model::PrimitiveFactory::makeBox(10, 10, 10);
    auto& edges = box->edges();
    if (edges.empty()) {
        statusBar()->showMessage(tr("No edges to chamfer"));
        return;
    }
    std::vector<topo::TopologyID> edgeIds = {edges.front().topoId};

    auto result = model::ChamferOp::executeEqual(*box, edgeIds, 1.0, "chamfer_demo");
    if (!result.solid || !result.errorMessage.empty()) {
        statusBar()->showMessage(
            tr("Chamfer failed: %1").arg(QString::fromStdString(result.errorMessage)));
        return;
    }

    auto meshData = model::SolidTessellator::tessellate(*result.solid, 0.1);
    auto node = std::make_shared<render::SceneNode>("Chamfer Demo");
    node->setMesh(std::make_unique<render::MeshData>(std::move(meshData)));
    node->setMaterial(render::Material{math::Vec3{0.35, 0.65, 0.85}, 0.15f, 0.5f, 32.0f});

    m_viewport->sceneGraph().addNode(node);
    m_viewport->camera().setIsometricView();
    m_viewport->update();
    m_statusPrompt->setText(tr("Chamfer completed."));
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

    QDialog dialog(this);
    dialog.setWindowTitle(tr("Edit %1").arg(QString::fromStdString(feat->name())));
    auto* form = new QFormLayout(&dialog);
    // What each box showed: a spin box rounds what it is given to its
    // decimals (a 360° revolve's 2π shows as 6.2832), so an untouched box is
    // one that still shows that, not one equal to the stored value.
    std::map<std::string, std::pair<QDoubleSpinBox*, double>> spins;
    for (const auto& [name, value] : params) {
        auto* spin = new QDoubleSpinBox(&dialog);
        spin->setObjectName(QString::fromStdString(name));
        // A chord tolerance of 0 means "use the facet count"; a 0.001 floor
        // would silently switch it on for anyone clicking through the dialog.
        spin->setRange(name == "chordTolerance" ? 0.0 : 0.001, 1e6);
        spin->setDecimals(4);
        spin->setValue(value);
        form->addRow(QString::fromStdString(name) + QStringLiteral(":"), spin);
        spins[name] = {spin, spin->value()};
    }
    QComboBox* result = buildsBody ? addOperationChoice(dialog, *form, feat->operation()) : nullptr;
    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog);
    connect(buttons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
    form->addRow(buttons);
    if (dialog.exec() != QDialog::Accepted) return;

    std::map<std::string, double> changed;
    for (const auto& [name, box] : spins) {
        const auto& [spin, shown] = box;
        if (spin->value() != shown) changed[name] = spin->value();
    }
    std::optional<doc::BodyOperation> operation;
    if (result) {
        const auto chosen = static_cast<doc::BodyOperation>(result->currentData().toInt());
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
    m_document->rebuildModel();

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

}  // namespace hz::ui
