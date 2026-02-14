#include "horizon/ui/MainWindow.h"
#include "horizon/ui/ViewportWidget.h"
#include "horizon/ui/ToolManager.h"
#include "horizon/ui/Tool.h"
#include "horizon/ui/SelectTool.h"
#include "horizon/ui/LineTool.h"
#include "horizon/ui/CircleTool.h"
#include "horizon/ui/ArcTool.h"
#include "horizon/ui/RectangleTool.h"
#include "horizon/ui/PolylineTool.h"
#include "horizon/ui/MoveTool.h"
#include "horizon/ui/OffsetTool.h"
#include "horizon/ui/TrimTool.h"
#include "horizon/ui/FilletTool.h"
#include "horizon/ui/ChamferTool.h"
#include "horizon/ui/BreakTool.h"
#include "horizon/ui/ExtendTool.h"
#include "horizon/ui/StretchTool.h"
#include "horizon/ui/PolylineEditTool.h"
#include "horizon/ui/MirrorTool.h"
#include "horizon/ui/RotateTool.h"
#include "horizon/ui/ScaleTool.h"
#include "horizon/ui/PasteTool.h"
#include "horizon/ui/Clipboard.h"
#include "horizon/ui/RectArrayDialog.h"
#include "horizon/ui/PolarArrayDialog.h"
#include "horizon/ui/PropertyPanel.h"
#include "horizon/ui/LayerPanel.h"
#include "horizon/ui/LinearDimensionTool.h"
#include "horizon/ui/RadialDimensionTool.h"
#include "horizon/ui/AngularDimensionTool.h"
#include "horizon/ui/LeaderTool.h"
#include "horizon/ui/TextTool.h"
#include "horizon/ui/SplineTool.h"
#include "horizon/ui/HatchTool.h"
#include "horizon/ui/EllipseTool.h"
#include "horizon/ui/MeasureDistanceTool.h"
#include "horizon/ui/MeasureAngleTool.h"
#include "horizon/ui/MeasureAreaTool.h"
#include "horizon/ui/ConstraintTool.h"
#include "horizon/ui/InsertBlockTool.h"
#include "horizon/ui/InsertBlockDialog.h"
#include "horizon/ui/RibbonBar.h"
#include "horizon/ui/IconGenerator.h"
#include "horizon/drafting/DraftBlockRef.h"
#include "horizon/math/BoundingBox.h"
#include "horizon/math/MathUtils.h"
#include "horizon/document/UndoStack.h"
#include "horizon/document/Commands.h"
#include "horizon/fileio/NativeFormat.h"
#include "horizon/fileio/DxfFormat.h"

#include <QAction>
#include <QActionGroup>
#include <QFileDialog>
#include <QKeySequence>
#include <QLabel>
#include <QMenuBar>
#include <QInputDialog>
#include <QMessageBox>
#include <QStatusBar>
#include <QToolBar>

namespace hz::ui {

// ---------------------------------------------------------------------------
// Construction / destruction
// ---------------------------------------------------------------------------

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
    , m_toolManager(std::make_unique<ToolManager>())
    , m_document(std::make_unique<doc::Document>()) {
    setWindowTitle("Horizon CAD");
    resize(1280, 800);

    // Central viewport widget.
    m_viewport = new ViewportWidget(this);
    m_viewport->setDocument(m_document.get());
    setCentralWidget(m_viewport);

    // Dock panels (must exist before createMenus, which adds toggleViewAction).
    m_propertyPanel = new PropertyPanel(this, this);
    addDockWidget(Qt::RightDockWidgetArea, m_propertyPanel);

    m_layerPanel = new LayerPanel(this, this);
    addDockWidget(Qt::RightDockWidgetArea, m_layerPanel);

    tabifyDockWidget(m_propertyPanel, m_layerPanel);
    m_propertyPanel->raise();

    // Build UI chrome.
    createMenus();
    createRibbonBar();
    createStatusBar();
    registerTools();

    // Wire up the status bar coordinate display.
    connect(m_viewport, &ViewportWidget::mouseMoved,
            this, &MainWindow::onMouseMoved);

    // Wire up selection changes to property panel.
    connect(m_viewport, &ViewportWidget::selectionChanged,
            this, &MainWindow::onSelectionChanged);

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

    QAction* newAction = fileMenu->addAction(tr("&New"), this, &MainWindow::onNewFile);
    newAction->setShortcut(QKeySequence::New);

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

    QAction* duplicateAction = editMenu->addAction(tr("&Duplicate"), this, &MainWindow::onDuplicate);
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

    QAction* ungroupAction = editMenu->addAction(tr("U&ngroup"), this, &MainWindow::onUngroupEntities);
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

    auto addToolAction = [&](QToolBar* tb, const QString& iconName,
                             const QString& tooltip, auto slot,
                             const QKeySequence& shortcut = {}) -> QAction* {
        auto* act = tb->addAction(IconGenerator::icon(iconName), tooltip, this, slot);
        act->setCheckable(true);
        act->setToolTip(shortcut.isEmpty()
            ? tooltip
            : QString("%1 (%2)").arg(tooltip, shortcut.toString(QKeySequence::NativeText)));
        if (!shortcut.isEmpty()) act->setShortcut(shortcut);
        toolGroup->addAction(act);
        return act;
    };

    auto addAction = [](QToolBar* tb, const QString& iconName,
                        const QString& tooltip, auto* receiver, auto slot,
                        const QKeySequence& shortcut = {}) -> QAction* {
        auto* act = tb->addAction(IconGenerator::icon(iconName), tooltip, receiver, slot);
        act->setToolTip(shortcut.isEmpty()
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
    auto* selectAct = addToolAction(homeBar, "select", tr("Select"),
                                     &MainWindow::onSelectTool, QKeySequence(Qt::Key_Space));
    selectAct->setChecked(true);
    addAction(homeBar, "fit-all", tr("Fit All"), this, &MainWindow::onFitAll,
              QKeySequence(Qt::Key_F));
    m_ribbonBar->addTab(tr("Home"), homeBar);

    // ---- Draw tab ----
    auto* drawBar = new QToolBar(this);
    addToolAction(drawBar, "line", tr("Line"), &MainWindow::onLineTool,
                  QKeySequence(Qt::Key_L));
    addToolAction(drawBar, "circle", tr("Circle"), &MainWindow::onCircleTool,
                  QKeySequence(Qt::Key_C));
    addToolAction(drawBar, "arc", tr("Arc"), &MainWindow::onArcTool,
                  QKeySequence(Qt::Key_A));
    addToolAction(drawBar, "rectangle", tr("Rectangle"), &MainWindow::onRectangleTool,
                  QKeySequence(Qt::Key_R));
    addToolAction(drawBar, "polyline", tr("Polyline"), &MainWindow::onPolylineTool,
                  QKeySequence(Qt::Key_P));
    addToolAction(drawBar, "ellipse", tr("Ellipse"), &MainWindow::onEllipseTool,
                  QKeySequence(Qt::Key_E));
    addToolAction(drawBar, "spline", tr("Spline"), &MainWindow::onSplineTool,
                  QKeySequence(Qt::Key_S));
    addToolAction(drawBar, "text", tr("Text"), &MainWindow::onTextTool,
                  QKeySequence(Qt::Key_T));
    addToolAction(drawBar, "hatch", tr("Hatch"), &MainWindow::onHatchTool,
                  QKeySequence(Qt::Key_H));
    m_ribbonBar->addTab(tr("Draw"), drawBar);

    // ---- Modify tab ----
    auto* modifyBar = new QToolBar(this);
    addToolAction(modifyBar, "move", tr("Move"), &MainWindow::onMoveTool,
                  QKeySequence(Qt::Key_M));
    addToolAction(modifyBar, "offset", tr("Offset"), &MainWindow::onOffsetTool,
                  QKeySequence(Qt::Key_O));
    addToolAction(modifyBar, "mirror", tr("Mirror"), &MainWindow::onMirrorTool,
                  QKeySequence(Qt::SHIFT | Qt::Key_M));
    addToolAction(modifyBar, "rotate", tr("Rotate"), &MainWindow::onRotateTool,
                  QKeySequence(Qt::SHIFT | Qt::Key_R));
    addToolAction(modifyBar, "scale", tr("Scale"), &MainWindow::onScaleTool,
                  QKeySequence(Qt::SHIFT | Qt::Key_S));
    modifyBar->addSeparator();
    addToolAction(modifyBar, "trim", tr("Trim"), &MainWindow::onTrimTool,
                  QKeySequence(Qt::Key_X));
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
    addAction(modifyBar, "rect-array", tr("Rect Array"), this,
              &MainWindow::onRectangularArray);
    addAction(modifyBar, "polar-array", tr("Polar Array"), this,
              &MainWindow::onPolarArray);
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
    addAction(constrainBar, "cstr-tangent", tr("Tangent"), this,
              &MainWindow::onConstraintTangent);
    addAction(constrainBar, "cstr-equal", tr("Equal"), this,
              &MainWindow::onConstraintEqual);
    constrainBar->addSeparator();
    addAction(constrainBar, "cstr-fixed", tr("Fixed"), this,
              &MainWindow::onConstraintFixed);
    addAction(constrainBar, "cstr-distance", tr("Distance"), this,
              &MainWindow::onConstraintDistance);
    addAction(constrainBar, "cstr-angle", tr("Angle"), this,
              &MainWindow::onConstraintAngle);
    m_ribbonBar->addTab(tr("Constrain"), constrainBar);

    // ---- Block tab ----
    auto* blockBar = new QToolBar(this);
    addAction(blockBar, "block-create", tr("Create Block"), this,
              &MainWindow::onCreateBlock);
    addAction(blockBar, "block-insert", tr("Insert Block"), this,
              &MainWindow::onInsertBlock);
    addAction(blockBar, "block-explode", tr("Explode"), this,
              &MainWindow::onExplode);
    m_ribbonBar->addTab(tr("Block"), blockBar);

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
    m_statusSelection->setText(count == 1 ? tr("1 selected")
                                          : tr("%1 selected").arg(count));
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
// Slots -- File
// ---------------------------------------------------------------------------

void MainWindow::onNewFile() {
    if (m_document->isDirty()) {
        auto result = QMessageBox::question(this, tr("Unsaved Changes"),
            tr("Save changes before creating a new file?"),
            QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel);
        if (result == QMessageBox::Save) {
            onSaveFile();
        } else if (result == QMessageBox::Cancel) {
            return;
        }
    }
    m_document->clear();
    m_viewport->selectionManager().clearSelection();
    m_viewport->update();
    m_layerPanel->refresh();
    m_propertyPanel->refreshLayerList();
    onSelectionChanged();
    setWindowTitle("Horizon CAD");
}

void MainWindow::onOpenFile() {
    QString fileName = QFileDialog::getOpenFileName(
        this, tr("Open File"), QString(),
        tr("All Supported Files (*.hcad *.dxf);;Horizon CAD Files (*.hcad);;DXF Files (*.dxf);;All Files (*)"));
    if (fileName.isEmpty()) return;

    doc::Document tempDoc;
    std::string path = fileName.toStdString();
    bool ok = false;
    if (fileName.endsWith(".dxf", Qt::CaseInsensitive)) {
        ok = io::DxfFormat::load(path, tempDoc);
    } else {
        ok = io::NativeFormat::load(path, tempDoc);
    }
    if (ok) {
        m_document->clear();
        m_document->setFilePath(fileName.toStdString());
        // Copy layers.
        for (const auto& name : tempDoc.layerManager().layerNames()) {
            const auto* lp = tempDoc.layerManager().getLayer(name);
            if (lp && name == "0") {
                auto* dst = m_document->layerManager().getLayer("0");
                if (dst) *dst = *lp;
            } else if (lp) {
                m_document->layerManager().addLayer(*lp);
            }
        }
        m_document->layerManager().setCurrentLayer(tempDoc.layerManager().currentLayer());
        // Copy entities.
        for (const auto& entity : tempDoc.draftDocument().entities()) {
            m_document->draftDocument().addEntity(entity);
        }
        // Copy block definitions.
        for (const auto& name : tempDoc.draftDocument().blockTable().blockNames()) {
            auto def = tempDoc.draftDocument().blockTable().findBlock(name);
            if (def) m_document->draftDocument().blockTable().addBlock(def);
        }
        // Copy constraints.
        for (const auto& c : tempDoc.constraintSystem().constraints()) {
            m_document->constraintSystem().addConstraint(c);
        }
        m_document->setDirty(false);
        m_viewport->selectionManager().clearSelection();
        m_viewport->update();
        m_layerPanel->refresh();
        m_propertyPanel->refreshLayerList();
        onSelectionChanged();
        setWindowTitle(QString("Horizon CAD - %1").arg(fileName));
    } else {
        QMessageBox::warning(this, tr("Error"), tr("Failed to open file."));
    }
}

void MainWindow::onSaveFile() {
    if (m_document->filePath().empty()) {
        onSaveFileAs();
        return;
    }
    std::string path = m_document->filePath();
    bool ok = false;
    if (QString::fromStdString(path).endsWith(".dxf", Qt::CaseInsensitive)) {
        ok = io::DxfFormat::save(path, *m_document);
    } else {
        ok = io::NativeFormat::save(path, *m_document);
    }
    if (ok) {
        m_document->setDirty(false);
        m_statusPrompt->setText(tr("File saved."));
    } else {
        QMessageBox::warning(this, tr("Error"), tr("Failed to save file."));
    }
}

void MainWindow::onSaveFileAs() {
    QString fileName = QFileDialog::getSaveFileName(
        this, tr("Save File"), QString(),
        tr("Horizon CAD Files (*.hcad);;DXF Files (*.dxf);;All Files (*)"));
    if (fileName.isEmpty()) return;

    m_document->setFilePath(fileName.toStdString());
    onSaveFile();
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
    auto cmd = std::make_unique<doc::DuplicateEntityCommand>(
        m_document->draftDocument(), idVec, offset);
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
        composite->addCommand(std::make_unique<doc::RemoveEntityCommand>(
            m_document->draftDocument(), entity->id()));
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

static void activateConstraintMode(ToolManager& tm, ViewportWidget* vp,
                                    ConstraintTool::Mode mode) {
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
        QMessageBox::information(this, tr("Create Block"),
                                 tr("Select entities first."));
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
    QString name = QInputDialog::getText(this, tr("Create Block"),
                                          tr("Block name:"), QLineEdit::Normal,
                                          QString(), &ok);
    if (!ok || name.trimmed().isEmpty()) return;

    std::string blockName = name.trimmed().toStdString();
    if (m_document->draftDocument().blockTable().findBlock(blockName)) {
        QMessageBox::warning(this, tr("Create Block"),
                             tr("A block with that name already exists."));
        return;
    }

    auto cmd = std::make_unique<doc::CreateBlockCommand>(
        m_document->draftDocument(), blockName, filteredIds);
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

    auto cmd = std::make_unique<doc::GroupEntitiesCommand>(
        m_document->draftDocument(), filteredIds);
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
    auto cmd = std::make_unique<doc::UngroupEntitiesCommand>(
        m_document->draftDocument(), groupIdVec);
    m_document->undoStack().push(std::move(cmd));
    m_viewport->update();
}

// ---------------------------------------------------------------------------
// Slots -- Status bar updates
// ---------------------------------------------------------------------------

void MainWindow::onMouseMoved(const hz::math::Vec2& worldPos) {
    m_statusCoords->setText(
        QString("X: %1  Y: %2")
            .arg(worldPos.x, 0, 'f', 3)
            .arg(worldPos.y, 0, 'f', 3));

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

}  // namespace hz::ui
