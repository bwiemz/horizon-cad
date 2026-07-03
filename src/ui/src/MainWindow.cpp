#include "horizon/ui/MainWindow.h"

#include <QAction>
#include <QActionGroup>
#include <QComboBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QInputDialog>
#include <QKeySequence>
#include <QLabel>
#include <QMenuBar>
#include <QMessageBox>
#include <QStatusBar>
#include <QTabBar>
#include <QToolBar>
#include <QVBoxLayout>
#include <filesystem>
#include <numbers>

#include "horizon/document/Commands.h"
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

// ---------------------------------------------------------------------------
// Construction / destruction
// ---------------------------------------------------------------------------

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent), m_toolManager(std::make_unique<ToolManager>()) {
    setWindowTitle("Horizon CAD");
    resize(1280, 800);

    // Wire the document manager to the native file format.
    m_docManager.setPartLoader([](const std::string& path, doc::Document& doc) {
        return io::NativeFormat::load(path, doc);
    });
    m_docManager.setMeshLoader(
        [](const std::string& path) { return io::NativeFormat::loadPartMesh(path); });
    m_docManager.setAssemblyLoader([](const std::string& path, doc::AssemblyDocument& doc) {
        return io::NativeFormat::loadAssembly(path, doc);
    });

    // Central area: document tab bar above the shared viewport.
    m_viewport = new ViewportWidget(this);
    m_tabBar = new QTabBar(this);
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
    m_tabs.push_back(DocTab{m_document, nullptr});
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
    connect(m_featureTreePanel, &FeatureTreePanel::featureReordered, this,
            &MainWindow::onFeatureReordered);
    connect(m_featureTreePanel, &FeatureTreePanel::rollbackChanged, this,
            &MainWindow::onRollbackChanged);

    // Build UI chrome.
    createMenus();
    createRibbonBar();
    createStatusBar();
    registerTools();

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
        act->setToolTip(
            shortcut.isEmpty()
                ? tooltip
                : QString("%1 (%2)").arg(tooltip, shortcut.toString(QKeySequence::NativeText)));
        if (!shortcut.isEmpty()) act->setShortcut(shortcut);
        return act;
    };

    // ---- Home tab ----
    auto* homeBar = new QToolBar(this);
    addAction(homeBar, "new", tr("New"), this, &MainWindow::onNewFile, QKeySequence::New);
    addAction(homeBar, "open", tr("Open"), this, &MainWindow::onOpenFile, QKeySequence::Open);
    addAction(homeBar, "save", tr("Save"), this, &MainWindow::onSaveFile, QKeySequence::Save);
    homeBar->addSeparator();
    addAction(homeBar, "undo", tr("Undo"), this, &MainWindow::onUndo, QKeySequence::Undo);
    addAction(homeBar, "redo", tr("Redo"), this, &MainWindow::onRedo, QKeySequence::Redo);
    homeBar->addSeparator();
    addAction(homeBar, "copy", tr("Copy"), this, &MainWindow::onCopy, QKeySequence::Copy);
    addAction(homeBar, "paste", tr("Paste"), this, &MainWindow::onPaste, QKeySequence::Paste);
    addAction(homeBar, "duplicate", tr("Duplicate"), this, &MainWindow::onDuplicate,
              QKeySequence(Qt::CTRL | Qt::Key_D));
    homeBar->addSeparator();
    addAction(homeBar, "group", tr("Group"), this, &MainWindow::onGroupEntities,
              QKeySequence(Qt::CTRL | Qt::Key_G));
    addAction(homeBar, "ungroup", tr("Ungroup"), this, &MainWindow::onUngroupEntities,
              QKeySequence(Qt::CTRL | Qt::SHIFT | Qt::Key_G));
    homeBar->addSeparator();
    auto* selectAct = addToolAction(homeBar, "select", tr("Select"), &MainWindow::onSelectTool,
                                    QKeySequence(Qt::Key_Space));
    selectAct->setChecked(true);
    addAction(homeBar, "fit-all", tr("Fit All"), this, &MainWindow::onFitAll,
              QKeySequence(Qt::Key_F));
    m_ribbonBar->addTab(tr("Home"), homeBar);

    // ---- Draw tab ----
    auto* drawBar = new QToolBar(this);
    addToolAction(drawBar, "line", tr("Line"), &MainWindow::onLineTool, QKeySequence(Qt::Key_L));
    addToolAction(drawBar, "circle", tr("Circle"), &MainWindow::onCircleTool,
                  QKeySequence(Qt::Key_C));
    addToolAction(drawBar, "arc", tr("Arc"), &MainWindow::onArcTool, QKeySequence(Qt::Key_A));
    addToolAction(drawBar, "rectangle", tr("Rectangle"), &MainWindow::onRectangleTool,
                  QKeySequence(Qt::Key_R));
    addToolAction(drawBar, "polyline", tr("Polyline"), &MainWindow::onPolylineTool,
                  QKeySequence(Qt::Key_P));
    addToolAction(drawBar, "ellipse", tr("Ellipse"), &MainWindow::onEllipseTool,
                  QKeySequence(Qt::Key_E));
    addToolAction(drawBar, "spline", tr("Spline"), &MainWindow::onSplineTool,
                  QKeySequence(Qt::Key_S));
    addToolAction(drawBar, "text", tr("Text"), &MainWindow::onTextTool, QKeySequence(Qt::Key_T));
    addToolAction(drawBar, "hatch", tr("Hatch"), &MainWindow::onHatchTool, QKeySequence(Qt::Key_H));
    m_ribbonBar->addTab(tr("Draw"), drawBar);

    // ---- Modify tab ----
    auto* modifyBar = new QToolBar(this);
    addToolAction(modifyBar, "move", tr("Move"), &MainWindow::onMoveTool, QKeySequence(Qt::Key_M));
    addToolAction(modifyBar, "offset", tr("Offset"), &MainWindow::onOffsetTool,
                  QKeySequence(Qt::Key_O));
    addToolAction(modifyBar, "mirror", tr("Mirror"), &MainWindow::onMirrorTool,
                  QKeySequence(Qt::SHIFT | Qt::Key_M));
    addToolAction(modifyBar, "rotate", tr("Rotate"), &MainWindow::onRotateTool,
                  QKeySequence(Qt::SHIFT | Qt::Key_R));
    addToolAction(modifyBar, "scale", tr("Scale"), &MainWindow::onScaleTool,
                  QKeySequence(Qt::SHIFT | Qt::Key_S));
    modifyBar->addSeparator();
    addToolAction(modifyBar, "trim", tr("Trim"), &MainWindow::onTrimTool, QKeySequence(Qt::Key_X));
    addToolAction(modifyBar, "fillet", tr("Fillet"), &MainWindow::onFilletTool);
    addToolAction(modifyBar, "chamfer", tr("Chamfer"), &MainWindow::onChamferTool);
    addToolAction(modifyBar, "break", tr("Break"), &MainWindow::onBreakTool,
                  QKeySequence(Qt::Key_B));
    addToolAction(modifyBar, "extend", tr("Extend"), &MainWindow::onExtendTool,
                  QKeySequence(Qt::SHIFT | Qt::Key_E));
    addToolAction(modifyBar, "stretch", tr("Stretch"), &MainWindow::onStretchTool,
                  QKeySequence(Qt::Key_W));
    addToolAction(modifyBar, "polyline-edit", tr("PL Edit"), &MainWindow::onPolylineEditTool);
    modifyBar->addSeparator();
    addAction(modifyBar, "rect-array", tr("Rect Array"), this, &MainWindow::onRectangularArray);
    addAction(modifyBar, "polar-array", tr("Polar Array"), this, &MainWindow::onPolarArray);
    m_ribbonBar->addTab(tr("Modify"), modifyBar);

    // ---- Annotate tab ----
    auto* annotateBar = new QToolBar(this);
    addToolAction(annotateBar, "dim-linear", tr("Linear Dim"), &MainWindow::onLinearDimTool,
                  QKeySequence(Qt::Key_D));
    addToolAction(annotateBar, "dim-radial", tr("Radial Dim"), &MainWindow::onRadialDimTool);
    addToolAction(annotateBar, "dim-angular", tr("Angular Dim"), &MainWindow::onAngularDimTool);
    addToolAction(annotateBar, "leader", tr("Leader"), &MainWindow::onLeaderTool);
    annotateBar->addSeparator();
    addAction(annotateBar, "measure-distance", tr("Measure Distance"), this,
              &MainWindow::onMeasureDistanceTool);
    addAction(annotateBar, "measure-angle", tr("Measure Angle"), this,
              &MainWindow::onMeasureAngleTool);
    addAction(annotateBar, "measure-area", tr("Measure Area"), this,
              &MainWindow::onMeasureAreaTool);
    m_ribbonBar->addTab(tr("Annotate"), annotateBar);

    // ---- Constrain tab ----
    auto* constrainBar = new QToolBar(this);
    addAction(constrainBar, "cstr-coincident", tr("Coincident"), this,
              &MainWindow::onConstraintCoincident);
    addAction(constrainBar, "cstr-horizontal", tr("Horizontal"), this,
              &MainWindow::onConstraintHorizontal);
    addAction(constrainBar, "cstr-vertical", tr("Vertical"), this,
              &MainWindow::onConstraintVertical);
    addAction(constrainBar, "cstr-perpendicular", tr("Perpendicular"), this,
              &MainWindow::onConstraintPerpendicular);
    addAction(constrainBar, "cstr-parallel", tr("Parallel"), this,
              &MainWindow::onConstraintParallel);
    addAction(constrainBar, "cstr-tangent", tr("Tangent"), this, &MainWindow::onConstraintTangent);
    addAction(constrainBar, "cstr-equal", tr("Equal"), this, &MainWindow::onConstraintEqual);
    constrainBar->addSeparator();
    addAction(constrainBar, "cstr-fixed", tr("Fixed"), this, &MainWindow::onConstraintFixed);
    addAction(constrainBar, "cstr-distance", tr("Distance"), this,
              &MainWindow::onConstraintDistance);
    addAction(constrainBar, "cstr-angle", tr("Angle"), this, &MainWindow::onConstraintAngle);
    m_ribbonBar->addTab(tr("Constrain"), constrainBar);

    // ---- Block tab ----
    auto* blockBar = new QToolBar(this);
    addAction(blockBar, "block-create", tr("Create Block"), this, &MainWindow::onCreateBlock);
    addAction(blockBar, "block-insert", tr("Insert Block"), this, &MainWindow::onInsertBlock);
    addAction(blockBar, "block-explode", tr("Explode"), this, &MainWindow::onExplode);
    m_ribbonBar->addTab(tr("Block"), blockBar);

    // ---- 3D tab ----
    auto* solidBar = new QToolBar(this);
    addAction(solidBar, "box", tr("Box"), this, &MainWindow::onPrimitiveBox);
    addAction(solidBar, "cylinder", tr("Cylinder"), this, &MainWindow::onPrimitiveCylinder);
    addAction(solidBar, "sphere", tr("Sphere"), this, &MainWindow::onPrimitiveSphere);
    addAction(solidBar, "cone", tr("Cone"), this, &MainWindow::onPrimitiveCone);
    addAction(solidBar, "torus", tr("Torus"), this, &MainWindow::onPrimitiveTorus);
    addAction(solidBar, "extrude", tr("Extrude"), this, &MainWindow::onExtrudeSketch);
    addAction(solidBar, "revolve", tr("Revolve"), this, &MainWindow::onRevolveSketch);
    solidBar->addSeparator();
    addAction(solidBar, "boolean-union", tr("Union"), this, &MainWindow::onBooleanUnion);
    addAction(solidBar, "boolean-subtract", tr("Subtract"), this, &MainWindow::onBooleanSubtract);
    addAction(solidBar, "boolean-intersect", tr("Intersect"), this,
              &MainWindow::onBooleanIntersect);
    solidBar->addSeparator();
    addAction(solidBar, "fillet-3d", tr("Fillet"), this, &MainWindow::onFillet);
    addAction(solidBar, "chamfer-3d", tr("Chamfer"), this, &MainWindow::onChamfer);
    m_ribbonBar->addTab(tr("3D"), solidBar);

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
    return QString::fromStdString(std::filesystem::path(path).filename().string());
}

int MainWindow::addDocumentTab(std::shared_ptr<doc::Document> document,
                               std::shared_ptr<doc::AssemblyDocument> assembly,
                               const QString& title) {
    m_tabs.push_back(DocTab{std::move(document), std::move(assembly)});
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

    DocTab tab = m_tabs[static_cast<size_t>(index)];
    bool dirty = tab.assembly ? tab.assembly->isDirty() : tab.document->isDirty();
    if (dirty) {
        auto result =
            QMessageBox::question(this, tr("Unsaved Changes"),
                                  tr("Close \"%1\" without saving?").arg(m_tabBar->tabText(index)),
                                  QMessageBox::Close | QMessageBox::Cancel);
        if (result != QMessageBox::Close) return;
    }

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
    if (path.empty()) {
        setWindowTitle("Horizon CAD");
    } else {
        setWindowTitle(QString("Horizon CAD - %1").arg(QString::fromStdString(path)));
    }
    int index = m_tabBar->currentIndex();
    if (index >= 0) {
        QString fallback = m_tabBar->tabText(index);
        m_tabBar->setTabText(index, tabTitleForPath(path, fallback));
    }
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
            QMessageBox::warning(this, tr("Error"), tr("Failed to open assembly."));
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
        if (!io::DxfFormat::load(path, *document)) {
            m_docManager.closeDocument(document);
            QMessageBox::warning(this, tr("Error"), tr("Failed to open file."));
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
        QMessageBox::warning(this, tr("Error"), tr("Failed to open file."));
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
            return !m_assembly->isDirty();
        }
        if (io::NativeFormat::saveAssembly(m_assembly->filePath(), *m_assembly)) {
            m_assembly->setDirty(false);
            m_docManager.noteSaved(m_assembly);
            m_statusPrompt->setText(tr("Assembly saved."));
            updateWindowTitle();
            return true;
        }
        QMessageBox::warning(this, tr("Error"), tr("Failed to save assembly."));
        return false;
    }

    if (m_document->filePath().empty()) {
        onSaveFileAs();
        return !m_document->isDirty();
    }
    std::string path = m_document->filePath();
    bool ok = false;
    if (QString::fromStdString(path).endsWith(".dxf", Qt::CaseInsensitive)) {
        ok = io::DxfFormat::save(path, *m_document);
    } else {
        // Make sure parts carry a fresh tessellation cache for lightweight
        // assembly loading.
        if (m_document->featureTree().featureCount() > 0 && !m_document->solid()) {
            m_document->rebuildModel();
        }
        ok = io::NativeFormat::save(path, *m_document);
    }
    if (ok) {
        m_document->setDirty(false);
        m_docManager.noteSaved(m_document);
        m_statusPrompt->setText(tr("File saved."));
        updateWindowTitle();
        return true;
    }
    QMessageBox::warning(this, tr("Error"), tr("Failed to save file."));
    return false;
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

    m_assembly->addComponent(std::move(comp));
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

    uint64_t mateId = m_assembly->addMate(std::move(mate));
    if (!solveAssemblyMates(*m_assembly)) {
        m_assembly->removeMate(mateId);
        solveAssemblyMates(*m_assembly);
    }
    rebuildScene();
}

// ---------------------------------------------------------------------------
// Slots -- Edit
// ---------------------------------------------------------------------------

void MainWindow::onUndo() {
    m_document->undoStack().undo();
    m_viewport->update();
    m_layerPanel->refresh();
    onSelectionChanged();
}

void MainWindow::onRedo() {
    m_document->undoStack().redo();
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

    bool ok;
    double distance =
        QInputDialog::getDouble(this, tr("Extrude"), tr("Distance:"), 10.0, 0.01, 1e6, 2, &ok);
    if (!ok) return;

    math::Vec3 direction = sketch->plane().normal();

    // Validate the profile BEFORE mutating the document: a failed extrude
    // must not leave a wrapper sketch or a dead feature behind.
    auto probe =
        model::Extrude::execute(sketch->entities(), sketch->plane(), direction, distance, "probe");
    if (!probe) {
        statusBar()->showMessage(tr("Extrude failed: profile is not a closed loop"));
        return;
    }

    if (createdWrapper) m_document->addSketch(sketch);

    // New features always go to the end of the active history.
    m_document->featureTree().setRollbackIndex(-1);
    m_document->featureTree().addFeature(
        std::make_unique<doc::ExtrudeFeature>(sketch, direction, distance));
    m_document->setDirty(true);
    rebuildFeatureTree();

    if (m_viewport->activeSketch()) m_viewport->setActiveSketch(nullptr);
    m_viewport->camera().setIsometricView();
    m_viewport->update();
    if (m_document->failedFeatureIndex() >= 0) {
        statusBar()->showMessage(tr("Extrude added, but an earlier feature fails to rebuild"));
    } else {
        m_statusPrompt->setText(tr("Extruded successfully."));
    }
}

void MainWindow::onRevolveSketch() {
    if (!m_viewport || !m_viewport->document()) return;

    bool createdWrapper = false;
    auto sketch = resolveProfileSketch(createdWrapper);
    if (!sketch || sketch->entities().empty()) {
        statusBar()->showMessage(tr("Draw a closed profile first"));
        return;
    }

    bool ok;
    double angleDeg = QInputDialog::getDouble(this, tr("Revolve"), tr("Angle (degrees):"), 360.0,
                                              1.0, 360.0, 1, &ok);
    if (!ok) return;

    const double angle = angleDeg * std::numbers::pi / 180.0;

    // Default revolve axis: Y axis through origin (world space)
    math::Vec3 axisPoint = math::Vec3::Zero;
    math::Vec3 axisDir = math::Vec3::UnitY;

    auto probe = model::Revolve::execute(sketch->entities(), sketch->plane(), axisPoint, axisDir,
                                         angle, "probe");
    if (!probe) {
        statusBar()->showMessage(tr("Revolve failed: profile is not a closed loop"));
        return;
    }

    if (createdWrapper) m_document->addSketch(sketch);

    m_document->featureTree().setRollbackIndex(-1);
    m_document->featureTree().addFeature(
        std::make_unique<doc::RevolveFeature>(sketch, axisPoint, axisDir, angle));
    m_document->setDirty(true);
    rebuildFeatureTree();

    if (m_viewport->activeSketch()) m_viewport->setActiveSketch(nullptr);
    m_viewport->camera().setIsometricView();
    m_viewport->update();
    if (m_document->failedFeatureIndex() >= 0) {
        statusBar()->showMessage(tr("Revolve added, but an earlier feature fails to rebuild"));
    } else {
        m_statusPrompt->setText(tr("Revolved successfully."));
    }
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

void MainWindow::onFeatureDoubleClicked(int featureIndex) {
    auto& tree = m_document->featureTree();
    if (featureIndex < 0 || static_cast<size_t>(featureIndex) >= tree.featureCount()) return;
    auto* feat = tree.feature(static_cast<size_t>(featureIndex));
    if (!feat) return;

    auto params = feat->parameters();
    bool changed = false;

    for (auto& [paramName, paramValue] : params) {
        bool ok = false;
        double newValue = QInputDialog::getDouble(
            this, tr("Edit %1").arg(QString::fromStdString(feat->name())),
            QString::fromStdString(paramName) + ":", paramValue, 0.001, 1e6, 3, &ok);
        if (ok && newValue != paramValue) {
            feat->setParameter(paramName, newValue);
            changed = true;
        }
    }

    if (changed) {
        rebuildFeatureTree();
    }
}

void MainWindow::onFeatureReordered(int fromIndex, int toIndex) {
    m_document->featureTree().moveFeature(fromIndex, toIndex);
    m_document->setDirty(true);
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
