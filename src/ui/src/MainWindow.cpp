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
#include "horizon/math/BoundingBox.h"
#include "horizon/math/MathUtils.h"
#include "horizon/document/UndoStack.h"
#include "horizon/document/Commands.h"
#include "horizon/fileio/NativeFormat.h"

#include <QAction>
#include <QActionGroup>
#include <QFileDialog>
#include <QKeySequence>
#include <QLabel>
#include <QMenuBar>
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

    // Build UI chrome.
    createMenus();
    createToolBar();
    createStatusBar();
    registerTools();

    // Dock panels.
    m_propertyPanel = new PropertyPanel(this, this);
    addDockWidget(Qt::RightDockWidgetArea, m_propertyPanel);

    m_layerPanel = new LayerPanel(this, this);
    addDockWidget(Qt::RightDockWidgetArea, m_layerPanel);

    tabifyDockWidget(m_propertyPanel, m_layerPanel);
    m_propertyPanel->raise();

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
    toolsMenu->addSeparator();
    toolsMenu->addAction(tr("&Move"), this, &MainWindow::onMoveTool);
    toolsMenu->addAction(tr("&Offset"), this, &MainWindow::onOffsetTool);
    toolsMenu->addAction(tr("M&irror"), this, &MainWindow::onMirrorTool);
    toolsMenu->addAction(tr("&Rotate"), this, &MainWindow::onRotateTool);
    toolsMenu->addAction(tr("&Scale"), this, &MainWindow::onScaleTool);
    toolsMenu->addSeparator();
    toolsMenu->addAction(tr("&Trim"), this, &MainWindow::onTrimTool);
    toolsMenu->addAction(tr("&Fillet"), this, &MainWindow::onFilletTool);
    toolsMenu->addSeparator();
    toolsMenu->addAction(tr("Rectangular &Array"), this, &MainWindow::onRectangularArray);
    toolsMenu->addAction(tr("Polar Arra&y"), this, &MainWindow::onPolarArray);

    // ---- Dimension ----
    QMenu* dimMenu = menuBar()->addMenu(tr("&Dimension"));
    dimMenu->addAction(tr("&Linear"), this, &MainWindow::onLinearDimTool);
    dimMenu->addAction(tr("&Radial"), this, &MainWindow::onRadialDimTool);
    dimMenu->addAction(tr("&Angular"), this, &MainWindow::onAngularDimTool);
    dimMenu->addSeparator();
    dimMenu->addAction(tr("L&eader"), this, &MainWindow::onLeaderTool);
}

// ---------------------------------------------------------------------------
// Toolbar
// ---------------------------------------------------------------------------

void MainWindow::createToolBar() {
    QToolBar* mainToolBar = addToolBar(tr("Main"));
    mainToolBar->setObjectName("MainToolBar");

    // File actions.
    mainToolBar->addAction(tr("New"), this, &MainWindow::onNewFile);
    mainToolBar->addAction(tr("Open"), this, &MainWindow::onOpenFile);
    mainToolBar->addAction(tr("Save"), this, &MainWindow::onSaveFile);

    mainToolBar->addSeparator();

    // Edit actions.
    mainToolBar->addAction(tr("Undo"), this, &MainWindow::onUndo);
    mainToolBar->addAction(tr("Redo"), this, &MainWindow::onRedo);

    mainToolBar->addSeparator();

    // Tool actions (checkable in an exclusive group).
    auto* toolGroup = new QActionGroup(this);
    toolGroup->setExclusive(true);

    QAction* selectAction = mainToolBar->addAction(tr("Select"), this, &MainWindow::onSelectTool);
    selectAction->setCheckable(true);
    selectAction->setChecked(true);
    toolGroup->addAction(selectAction);

    QAction* lineAction = mainToolBar->addAction(tr("Line"), this, &MainWindow::onLineTool);
    lineAction->setCheckable(true);
    toolGroup->addAction(lineAction);

    QAction* circleAction = mainToolBar->addAction(tr("Circle"), this, &MainWindow::onCircleTool);
    circleAction->setCheckable(true);
    toolGroup->addAction(circleAction);

    QAction* arcAction = mainToolBar->addAction(tr("Arc"), this, &MainWindow::onArcTool);
    arcAction->setCheckable(true);
    toolGroup->addAction(arcAction);

    QAction* rectAction = mainToolBar->addAction(tr("Rectangle"), this, &MainWindow::onRectangleTool);
    rectAction->setCheckable(true);
    toolGroup->addAction(rectAction);

    QAction* polylineAction = mainToolBar->addAction(tr("Polyline"), this, &MainWindow::onPolylineTool);
    polylineAction->setCheckable(true);
    toolGroup->addAction(polylineAction);

    mainToolBar->addSeparator();

    QAction* moveAction = mainToolBar->addAction(tr("Move"), this, &MainWindow::onMoveTool);
    moveAction->setCheckable(true);
    toolGroup->addAction(moveAction);

    QAction* offsetAction = mainToolBar->addAction(tr("Offset"), this, &MainWindow::onOffsetTool);
    offsetAction->setCheckable(true);
    toolGroup->addAction(offsetAction);

    QAction* mirrorAction = mainToolBar->addAction(tr("Mirror"), this, &MainWindow::onMirrorTool);
    mirrorAction->setCheckable(true);
    toolGroup->addAction(mirrorAction);

    QAction* rotateAction = mainToolBar->addAction(tr("Rotate"), this, &MainWindow::onRotateTool);
    rotateAction->setCheckable(true);
    toolGroup->addAction(rotateAction);

    QAction* scaleAction = mainToolBar->addAction(tr("Scale"), this, &MainWindow::onScaleTool);
    scaleAction->setCheckable(true);
    toolGroup->addAction(scaleAction);

    mainToolBar->addSeparator();

    QAction* trimAction = mainToolBar->addAction(tr("Trim"), this, &MainWindow::onTrimTool);
    trimAction->setCheckable(true);
    toolGroup->addAction(trimAction);

    QAction* filletAction = mainToolBar->addAction(tr("Fillet"), this, &MainWindow::onFilletTool);
    filletAction->setCheckable(true);
    toolGroup->addAction(filletAction);

    mainToolBar->addSeparator();

    // Dimension tools.
    QAction* linearDimAction = mainToolBar->addAction(tr("LinDim"), this, &MainWindow::onLinearDimTool);
    linearDimAction->setCheckable(true);
    toolGroup->addAction(linearDimAction);

    QAction* radialDimAction = mainToolBar->addAction(tr("RadDim"), this, &MainWindow::onRadialDimTool);
    radialDimAction->setCheckable(true);
    toolGroup->addAction(radialDimAction);

    QAction* angularDimAction = mainToolBar->addAction(tr("AngDim"), this, &MainWindow::onAngularDimTool);
    angularDimAction->setCheckable(true);
    toolGroup->addAction(angularDimAction);

    QAction* leaderAction = mainToolBar->addAction(tr("Leader"), this, &MainWindow::onLeaderTool);
    leaderAction->setCheckable(true);
    toolGroup->addAction(leaderAction);

    mainToolBar->addSeparator();

    // View presets.
    mainToolBar->addAction(tr("Fit All"), this, &MainWindow::onFitAll);
}

// ---------------------------------------------------------------------------
// Status bar
// ---------------------------------------------------------------------------

void MainWindow::createStatusBar() {
    statusBar()->showMessage(tr("Ready"));
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
    m_toolManager->registerTool(std::make_unique<MirrorTool>());
    m_toolManager->registerTool(std::make_unique<RotateTool>());
    m_toolManager->registerTool(std::make_unique<ScaleTool>());
    m_toolManager->registerTool(std::make_unique<PasteTool>(&m_clipboard));
    m_toolManager->registerTool(std::make_unique<LinearDimensionTool>());
    m_toolManager->registerTool(std::make_unique<RadialDimensionTool>());
    m_toolManager->registerTool(std::make_unique<AngularDimensionTool>());
    m_toolManager->registerTool(std::make_unique<LeaderTool>());
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
        tr("Horizon CAD Files (*.hcad);;All Files (*)"));
    if (fileName.isEmpty()) return;

    doc::Document tempDoc;
    if (io::NativeFormat::load(fileName.toStdString(), tempDoc)) {
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
    if (io::NativeFormat::save(m_document->filePath(), *m_document)) {
        m_document->setDirty(false);
        statusBar()->showMessage(tr("File saved."), 3000);
    } else {
        QMessageBox::warning(this, tr("Error"), tr("Failed to save file."));
    }
}

void MainWindow::onSaveFileAs() {
    QString fileName = QFileDialog::getSaveFileName(
        this, tr("Save File"), QString(),
        tr("Horizon CAD Files (*.hcad);;All Files (*)"));
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
}

void MainWindow::onLineTool() {
    m_toolManager->setActiveTool("Line");
    m_viewport->setActiveTool(m_toolManager->activeTool());
}

void MainWindow::onCircleTool() {
    m_toolManager->setActiveTool("Circle");
    m_viewport->setActiveTool(m_toolManager->activeTool());
}

void MainWindow::onArcTool() {
    m_toolManager->setActiveTool("Arc");
    m_viewport->setActiveTool(m_toolManager->activeTool());
}

void MainWindow::onRectangleTool() {
    m_toolManager->setActiveTool("Rectangle");
    m_viewport->setActiveTool(m_toolManager->activeTool());
}

void MainWindow::onPolylineTool() {
    m_toolManager->setActiveTool("Polyline");
    m_viewport->setActiveTool(m_toolManager->activeTool());
}

void MainWindow::onMoveTool() {
    m_toolManager->setActiveTool("Move");
    m_viewport->setActiveTool(m_toolManager->activeTool());
}

void MainWindow::onOffsetTool() {
    m_toolManager->setActiveTool("Offset");
    m_viewport->setActiveTool(m_toolManager->activeTool());
}

void MainWindow::onMirrorTool() {
    m_toolManager->setActiveTool("Mirror");
    m_viewport->setActiveTool(m_toolManager->activeTool());
}

void MainWindow::onTrimTool() {
    m_toolManager->setActiveTool("Trim");
    m_viewport->setActiveTool(m_toolManager->activeTool());
}

void MainWindow::onFilletTool() {
    m_toolManager->setActiveTool("Fillet");
    m_viewport->setActiveTool(m_toolManager->activeTool());
}

void MainWindow::onRotateTool() {
    m_toolManager->setActiveTool("Rotate");
    m_viewport->setActiveTool(m_toolManager->activeTool());
}

void MainWindow::onScaleTool() {
    m_toolManager->setActiveTool("Scale");
    m_viewport->setActiveTool(m_toolManager->activeTool());
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
                        composite->addCommand(std::make_unique<doc::AddEntityCommand>(
                            m_document->draftDocument(), clone));
                        break;
                    }
                }
            }
        }
    }

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

    for (int i = 1; i < count; ++i) {
        double angle = step * i;
        for (uint64_t id : filteredIds) {
            for (const auto& entity : m_document->draftDocument().entities()) {
                if (entity->id() == id) {
                    auto clone = entity->clone();
                    clone->rotate(center, angle);
                    newIds.push_back(clone->id());
                    composite->addCommand(std::make_unique<doc::AddEntityCommand>(
                        m_document->draftDocument(), clone));
                    break;
                }
            }
        }
    }

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
}

void MainWindow::onRadialDimTool() {
    m_toolManager->setActiveTool("Radial Dimension");
    m_viewport->setActiveTool(m_toolManager->activeTool());
}

void MainWindow::onAngularDimTool() {
    m_toolManager->setActiveTool("Angular Dimension");
    m_viewport->setActiveTool(m_toolManager->activeTool());
}

void MainWindow::onLeaderTool() {
    m_toolManager->setActiveTool("Leader");
    m_viewport->setActiveTool(m_toolManager->activeTool());
}

// ---------------------------------------------------------------------------
// Slots -- Status bar
// ---------------------------------------------------------------------------

void MainWindow::onMouseMoved(const hz::math::Vec2& worldPos) {
    statusBar()->showMessage(
        QString("X: %1  Y: %2")
            .arg(worldPos.x, 0, 'f', 3)
            .arg(worldPos.y, 0, 'f', 3));
}

void MainWindow::onSelectionChanged() {
    auto ids = m_viewport->selectionManager().selectedIds();
    std::vector<uint64_t> idVec(ids.begin(), ids.end());
    m_propertyPanel->updateForSelection(idVec);
}

}  // namespace hz::ui
