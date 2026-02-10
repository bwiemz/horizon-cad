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
#include "horizon/math/BoundingBox.h"
#include "horizon/document/UndoStack.h"
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

    // Wire up the status bar coordinate display.
    connect(m_viewport, &ViewportWidget::mouseMoved,
            this, &MainWindow::onMouseMoved);

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

    // ---- View ----
    QMenu* viewMenu = menuBar()->addMenu(tr("&View"));

    viewMenu->addAction(tr("&Front"), this, &MainWindow::onViewFront);
    viewMenu->addAction(tr("&Top"), this, &MainWindow::onViewTop);
    viewMenu->addAction(tr("&Right"), this, &MainWindow::onViewRight);
    viewMenu->addAction(tr("&Isometric"), this, &MainWindow::onViewIsometric);
    viewMenu->addSeparator();
    viewMenu->addAction(tr("Fit &All"), this, &MainWindow::onFitAll);

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
    setWindowTitle("Horizon CAD");
}

void MainWindow::onOpenFile() {
    QString fileName = QFileDialog::getOpenFileName(
        this, tr("Open File"), QString(),
        tr("Horizon CAD Files (*.hcad);;All Files (*)"));
    if (fileName.isEmpty()) return;

    draft::DraftDocument tempDoc;
    if (io::NativeFormat::load(fileName.toStdString(), tempDoc)) {
        m_document->clear();
        m_document->setFilePath(fileName.toStdString());
        for (const auto& entity : tempDoc.entities()) {
            m_document->draftDocument().addEntity(entity);
        }
        m_document->setDirty(false);
        m_viewport->selectionManager().clearSelection();
        m_viewport->update();
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
    if (io::NativeFormat::save(m_document->filePath(), m_document->draftDocument())) {
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
}

void MainWindow::onRedo() {
    m_document->undoStack().redo();
    m_viewport->update();
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

// ---------------------------------------------------------------------------
// Slots -- Status bar
// ---------------------------------------------------------------------------

void MainWindow::onMouseMoved(const hz::math::Vec2& worldPos) {
    statusBar()->showMessage(
        QString("X: %1  Y: %2")
            .arg(worldPos.x, 0, 'f', 3)
            .arg(worldPos.y, 0, 'f', 3));
}

}  // namespace hz::ui
