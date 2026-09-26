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
#include <QHeaderView>
#include <QInputDialog>
#include <QKeySequence>
#include <QLabel>
#include <QListWidget>
#include <QLocale>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QProgressBar>
#include <QPushButton>
#include <QSettings>
#include <QSignalBlocker>
#include <QStandardPaths>
#include <QStatusBar>
#include <QSysInfo>
#include <QTabBar>
#include <QTableWidget>
#include <QTimer>
#include <QToolBar>
#include <QToolButton>
#include <QVBoxLayout>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <filesystem>
#include <functional>
#include <map>
#include <numbers>
#include <optional>
#include <set>
#include <utility>

#include "horizon/Revision.h"
#include "horizon/Version.h"
#include "horizon/document/BillOfMaterials.h"
#include "horizon/document/Commands.h"
#include "horizon/document/ModelCommands.h"
#include "horizon/document/UndoStack.h"
#include "horizon/drafting/DraftBlockRef.h"
#include "horizon/fileio/BomExport.h"
#include "horizon/fileio/DxfFormat.h"
#include "horizon/fileio/GltfExport.h"
#include "horizon/fileio/NativeFormat.h"
#include "horizon/fileio/StepAssemblyFiles.h"
#include "horizon/fileio/StepFormat.h"
#include "horizon/fileio/StlExport.h"
#include "horizon/fileio/SvgExport.h"
#include "horizon/geometry/curves/NurbsCurve.h"
#include "horizon/math/BoundingBox.h"
#include "horizon/math/Expression.h"
#include "horizon/math/Mat4.h"
#include "horizon/math/MathUtils.h"
#include "horizon/math/Quantity.h"
#include "horizon/modeling/AssemblySolver.h"
#include "horizon/modeling/BooleanOp.h"
#include "horizon/modeling/EdgeProjection.h"
#include "horizon/modeling/Extrude.h"
#include "horizon/modeling/FacePlane.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/modeling/MateGeometry.h"
#include "horizon/modeling/Naming.h"
#include "horizon/modeling/Pattern.h"
#include "horizon/modeling/Revolve.h"
#include "horizon/modeling/Sheet.h"
#include "horizon/modeling/SolidTessellator.h"
#include "horizon/render/SceneGraph.h"
#include "horizon/topology/Solid.h"
#include "horizon/ui/AngularDimensionTool.h"
#include "horizon/ui/ArcTool.h"
#include "horizon/ui/AssemblyTreePanel.h"
#include "horizon/ui/AssemblyWorkbench.h"
#include "horizon/ui/BreakTool.h"
#include "horizon/ui/ChainDimensionTool.h"
#include "horizon/ui/ChamferTool.h"
#include "horizon/ui/CircleTool.h"
#include "horizon/ui/Clipboard.h"
#include "horizon/ui/CommandPalette.h"
#include "horizon/ui/ConfigurationsDialog.h"
#include "horizon/ui/ConstraintTool.h"
#include "horizon/ui/CrashReport.h"
#include "horizon/ui/DrawingWorkbench.h"
#include "horizon/ui/EllipseTool.h"
#include "horizon/ui/ExtendTool.h"
#include "horizon/ui/FeatureForm.h"
#include "horizon/ui/FeatureTreePanel.h"
#include "horizon/ui/FilletTool.h"
#include "horizon/ui/HatchTool.h"
#include "horizon/ui/HelpWindow.h"
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
#include "horizon/ui/QuantitySpinBox.h"
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
#include "horizon/ui/VariablesDialog.h"
#include "horizon/ui/ViewportWidget.h"

namespace hz::ui {

namespace {
/// saveState() layout version: bump it when docks or toolbars change, so an
/// old saved layout is ignored instead of misplacing them.
constexpr int kWindowStateVersion = 1;
}  // namespace

namespace {

/// How often the files open or placed are looked at for changes on disk.
constexpr int kPartWatchMs = 2000;

/// Edges or faces of the part to choose from in a dialog, until the viewport
/// can pick them: each one's name, and how it is listed ({text, tooltip}).
struct PickList {
    std::vector<topo::TopologyID> ids;
    std::vector<std::pair<QString, QString>> items;
};

/// The part's edges, listed by their end points: a curve in chords once, as
/// the curve (Stable names), and the seams between the facets of a curved
/// face, which are no edge of the part, not at all.
PickList edgesOf(const topo::Solid& solid) {
    PickList list;
    std::map<std::string, size_t> rowOf;
    std::vector<int> pieces;
    for (const auto& edge : solid.edges()) {
        const topo::HalfEdge* he = edge.halfEdge;
        if (!edge.topoId.isValid() || !he || !he->origin || !he->next || !he->next->origin) {
            continue;
        }
        if (edge.topoId.tag().find("/seam:") != std::string::npos) continue;
        const std::string logical = model::logicalEdge(edge.topoId.tag());
        const auto found = rowOf.find(logical);
        if (found != rowOf.end()) {
            ++pieces[found->second];
            continue;
        }
        rowOf.emplace(logical, list.ids.size());
        pieces.push_back(1);
        list.ids.push_back(topo::TopologyID::fromTag(logical));
        list.items.emplace_back(formatPoint(he->origin->point) + QStringLiteral(" – ") +
                                    formatPoint(he->next->origin->point),
                                QString::fromStdString(logical));
    }
    for (size_t row = 0; row < pieces.size(); ++row) {
        if (pieces[row] < 2) continue;
        auto& text = list.items[row].first;
        text = MainWindow::tr("a curve of %1 pieces, from %2")
                   .arg(pieces[row])
                   .arg(text.section(QStringLiteral(" – "), 0, 0));
    }
    return list;
}

/// The part's faces, listed by which way they face and where their middle is.
PickList facesOf(const topo::Solid& solid) {
    PickList list;
    const double outward = model::outwardSign(solid);
    struct Curved {
        int facets = 0;
        math::Vec3 centre;  ///< the sum of its facets' middles
    };
    std::map<std::string, size_t> rowOf;
    std::map<size_t, Curved> curvedRows;
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
        // A curved face in facets once, as the face (Stable names).
        const std::string logical = model::logicalFace(face.topoId.tag());
        const auto found = rowOf.find(logical);
        if (found != rowOf.end()) {
            Curved& curved = curvedRows[found->second];
            ++curved.facets;
            curved.centre = curved.centre + centre / count;
            continue;
        }
        rowOf.emplace(logical, list.ids.size());
        curvedRows[list.ids.size()] = Curved{1, centre / count};
        list.ids.push_back(topo::TopologyID::fromTag(logical));
        list.items.emplace_back(
            MainWindow::tr("facing %1 at %2")
                .arg(formatPoint(normal.normalized() * outward), formatPoint(centre / count)),
            QString::fromStdString(logical));
    }
    for (const auto& [row, curved] : curvedRows) {
        if (curved.facets < 2) continue;
        list.items[row].first = MainWindow::tr("a curved face of %1 facets, around %2")
                                    .arg(curved.facets)
                                    .arg(formatPoint(curved.centre / curved.facets));
    }
    return list;
}

/// Check, in a list of @p picks, the part's own edges (@p edges) or faces
/// chosen by clicking in the viewport, so a command offers what was clicked.
void checkClicked(QListWidget* list, const PickList& picks,
                  const std::vector<ViewportWidget::ModelPick>& clicked, bool edges) {
    for (size_t row = 0; row < picks.ids.size(); ++row) {
        const std::string& tag = picks.ids[row].tag();
        const bool wasClicked =
            std::any_of(clicked.begin(), clicked.end(), [&](const ViewportWidget::ModelPick& p) {
                return p.owner == 0 && p.edge == edges && p.tag == tag;
            });
        if (!wasClicked) continue;
        if (QListWidgetItem* item = list->item(static_cast<int>(row))) {
            item->setCheckState(Qt::Checked);
        }
    }
}

/// A tab's close button named for its tab, as a screen reader says it
/// (Phase 166): "Close Part 1". Qt gives it none. Whichever side the style
/// puts it on.
void nameCloseButton(QTabBar& bar, int index) {
    for (const auto side : {QTabBar::LeftSide, QTabBar::RightSide}) {
        if (QWidget* button = bar.tabButton(index, side)) {
            button->setAccessibleName(MainWindow::tr("Close %1").arg(bar.tabText(index)));
        }
    }
}

/// How a feature's parameter or vector is labelled in its edit form.
QString parameterLabel(const std::string& name) {
    static const std::map<std::string, const char*> labels = {
        {"distance", QT_TRANSLATE_NOOP("hz::ui::MainWindow", "Distance")},
        {"angle", QT_TRANSLATE_NOOP("hz::ui::MainWindow", "Angle")},
        {"segments", QT_TRANSLATE_NOOP("hz::ui::MainWindow", "Segments per turn")},
        {"arcSegments", QT_TRANSLATE_NOOP("hz::ui::MainWindow", "Segments across the round")},
        {"chordTolerance", QT_TRANSLATE_NOOP("hz::ui::MainWindow", "Chord tolerance")},
        {"radius", QT_TRANSLATE_NOOP("hz::ui::MainWindow", "Radius")},
        {"thickness", QT_TRANSLATE_NOOP("hz::ui::MainWindow", "Thickness")},
        {"operation", QT_TRANSLATE_NOOP("hz::ui::MainWindow", "Operation")},
        {"extent", QT_TRANSLATE_NOOP("hz::ui::MainWindow", "Goes")},
        {"upToFace", QT_TRANSLATE_NOOP("hz::ui::MainWindow", "Up to face")},
        {"count", QT_TRANSLATE_NOOP("hz::ui::MainWindow", "Count")},
        {"spacing", QT_TRANSLATE_NOOP("hz::ui::MainWindow", "Spacing")},
        {"width", QT_TRANSLATE_NOOP("hz::ui::MainWindow", "Width")},
        {"height", QT_TRANSLATE_NOOP("hz::ui::MainWindow", "Height")},
        {"depth", QT_TRANSLATE_NOOP("hz::ui::MainWindow", "Depth")},
        {"bottomRadius", QT_TRANSLATE_NOOP("hz::ui::MainWindow", "Bottom radius")},
        {"topRadius", QT_TRANSLATE_NOOP("hz::ui::MainWindow", "Top radius")},
        {"majorRadius", QT_TRANSLATE_NOOP("hz::ui::MainWindow", "Ring radius")},
        {"minorRadius", QT_TRANSLATE_NOOP("hz::ui::MainWindow", "Tube radius")},
        {"direction", QT_TRANSLATE_NOOP("hz::ui::MainWindow", "Direction")},
        {"axisPoint", QT_TRANSLATE_NOOP("hz::ui::MainWindow", "Axis through")},
        {"axisDirection", QT_TRANSLATE_NOOP("hz::ui::MainWindow", "Axis direction")},
        {"pullDirection", QT_TRANSLATE_NOOP("hz::ui::MainWindow", "Pull direction")},
        {"neutralPoint", QT_TRANSLATE_NOOP("hz::ui::MainWindow", "Neutral plane through")},
        {"planePoint", QT_TRANSLATE_NOOP("hz::ui::MainWindow", "Mirror plane through")},
        {"planeNormal", QT_TRANSLATE_NOOP("hz::ui::MainWindow", "Mirror plane facing")},
        {"planeFace", QT_TRANSLATE_NOOP("hz::ui::MainWindow", "Mirror in the face")},
        {"type", QT_TRANSLATE_NOOP("hz::ui::MainWindow", "Type")},
        {"diameter", QT_TRANSLATE_NOOP("hz::ui::MainWindow", "Diameter")},
        {"boreDiameter", QT_TRANSLATE_NOOP("hz::ui::MainWindow", "Counterbore diameter")},
        {"boreDepth", QT_TRANSLATE_NOOP("hz::ui::MainWindow", "Counterbore depth")},
        {"sinkDiameter", QT_TRANSLATE_NOOP("hz::ui::MainWindow", "Countersink diameter")},
        {"sinkAngle", QT_TRANSLATE_NOOP("hz::ui::MainWindow", "Countersink angle")},
        {"pointAngle", QT_TRANSLATE_NOOP("hz::ui::MainWindow", "Point angle (0: flat)")},
        {"positionPoint", QT_TRANSLATE_NOOP("hz::ui::MainWindow", "At")},
        {"face", QT_TRANSLATE_NOOP("hz::ui::MainWindow", "Into the face")},
    };
    const auto it = labels.find(name);
    return it != labels.end() ? MainWindow::tr(it->second) : QString::fromStdString(name);
}

/// A plane to sketch on, and how it is listed.
struct PlaneChoice {
    QString text;
    draft::SketchPlane plane;
    std::string tag;  ///< the face's persistent name
};

/// The part's flat faces as planes to sketch on: through the middle of the
/// face, facing out of the part, x along the world's x (or y, for a face
/// that faces along x). A face that is not flat (a loft's or a fillet's can
/// be twisted) is left out.
std::vector<PlaneChoice> planarFacesOf(const topo::Solid& solid) {
    std::vector<PlaneChoice> out;
    const double outward = model::outwardSign(solid);
    for (const auto& face : solid.faces()) {
        // As a build finds it again (Phase 157), so a sketch on it is placed
        // where it was drawn.
        const auto plane = model::planeOf(face, outward);
        if (!plane) continue;
        const math::Vec3& normal = plane->normal;
        const math::Vec3 across =
            std::abs(normal.dot(math::Vec3::UnitX)) < 0.9 ? math::Vec3::UnitX : math::Vec3::UnitY;
        out.push_back(
            {MainWindow::tr("facing %1 at %2").arg(formatPoint(normal), formatPoint(plane->origin)),
             draft::SketchPlane(plane->origin, normal, across), face.topoId.tag()});
    }
    return out;
}

/// The part's flat faces, each once (by its whole name: the pieces of a face
/// a Boolean split are one), for a command that works on a face by its name
/// (Phase 162: a hole drilled into one, a mirror in one).
struct FlatFace {
    QString text;
    math::Vec3 middle;
    math::Vec3 normal;  ///< out of the part
    std::string name;   ///< its whole name
};
std::vector<FlatFace> flatFacesOf(const topo::Solid& solid) {
    std::vector<FlatFace> out;
    std::set<std::string> listed;
    for (const auto& choice : planarFacesOf(solid)) {
        std::string whole = model::wholeFaceName(choice.tag);
        if (!listed.insert(whole).second) continue;
        out.push_back(
            {choice.text, choice.plane.origin(), choice.plane.normal(), std::move(whole)});
    }
    return out;
}

/// Which of @p faces was clicked first in @p picks; @p otherwise when none.
int clickedFlatFace(const std::vector<FlatFace>& faces,
                    const std::vector<ViewportWidget::ModelPick>& picks, int otherwise) {
    for (const auto& pick : picks) {
        if (pick.edge || pick.tag.empty()) continue;
        const std::string whole = model::wholeFaceName(pick.tag);
        for (size_t i = 0; i < faces.size(); ++i) {
            if (faces[i].name == whole) return static_cast<int>(i);
        }
    }
    return otherwise;
}

/// The way the first face or edge clicked on the part points: the face's
/// outward normal, or the edge from its first end to its second.
std::optional<math::Vec3> clickedDirection(const topo::Solid& solid,
                                           const std::vector<ViewportWidget::ModelPick>& picks) {
    for (const auto& pick : picks) {
        if (pick.owner != 0) continue;
        if (pick.edge) {
            for (const auto& edge : solid.edges()) {
                const topo::HalfEdge* he = edge.halfEdge;
                if (edge.topoId.tag() != pick.tag || !he || !he->origin || !he->next ||
                    !he->next->origin) {
                    continue;
                }
                const math::Vec3 along = he->next->origin->point - he->origin->point;
                if (along.length() > 1e-12) return along.normalized();
            }
            continue;
        }
        for (const auto& face : planarFacesOf(solid)) {
            if (face.tag == pick.tag) return face.plane.normal();
        }
    }
    return std::nullopt;
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
    // A part changed on disk, by another program, reaches the assemblies
    // that place it (Phase 144).
    m_partWatch = new QTimer(this);
    m_partWatch->setObjectName(QStringLiteral("partWatchTimer"));
    connect(m_partWatch, &QTimer::timeout, this, &MainWindow::pollPartFiles);
    m_partWatch->start(kPartWatchMs);

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
    m_docManager.setNewDocumentUnit(Preferences::current().newDocumentUnit());
    m_document = m_docManager.newDocument(doc::DocumentType::Drawing);
    watchDocument(m_document);
    m_tabs.push_back(DocTab{m_document, nullptr, tr("Drawing 1"), m_nextRecoveryKey++});
    nameCloseButton(*m_tabBar, m_tabBar->addTab(tr("Drawing 1")));
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
    // An assembly's components and mates, in a tab beside the feature tree.
    m_assemblyTreePanel = new AssemblyTreePanel(this);
    addDockWidget(Qt::LeftDockWidgetArea, m_assemblyTreePanel);
    tabifyDockWidget(m_featureTreePanel, m_assemblyTreePanel);
    m_featureTreePanel->raise();
    // The assembly commands, which wire the tree to themselves (Phase 146).
    m_assemblies = std::make_unique<AssemblyWorkbench>(static_cast<WorkbenchHost&>(*this),
                                                       *m_assemblyTreePanel);
    // A component dragged in the view, its mates solved as it moves (158).
    m_viewport->setComponentDragger(m_assemblies.get());
    // Drawing sheets made from parts (Phase 148).
    m_drawings = std::make_unique<DrawingWorkbench>(static_cast<WorkbenchHost&>(*this));

    connect(m_featureTreePanel, &FeatureTreePanel::featureDoubleClicked, this,
            &MainWindow::onFeatureDoubleClicked);
    connect(m_featureTreePanel, &FeatureTreePanel::featureDeleteRequested, this,
            &MainWindow::onFeatureDeleteRequested);
    connect(m_featureTreePanel, &FeatureTreePanel::featureSuppressRequested, this,
            &MainWindow::onFeatureSuppressRequested);
    connect(m_featureTreePanel, &FeatureTreePanel::featureReordered, this,
            &MainWindow::onFeatureReordered);
    connect(m_featureTreePanel, &FeatureTreePanel::configurationChosen, this,
            &MainWindow::onConfigurationChosen);
    connect(m_featureTreePanel, &FeatureTreePanel::rollbackChanged, this,
            &MainWindow::onRollbackChanged);
    connect(m_featureTreePanel, &FeatureTreePanel::createBoxRequested, this,
            &MainWindow::onPrimitiveBox);
    connect(m_featureTreePanel, &FeatureTreePanel::openFileRequested, this,
            &MainWindow::onOpenFile);
    connect(m_featureTreePanel, &FeatureTreePanel::sketchEditRequested, this, [this](uint64_t id) {
        if (auto sketch = m_document->findSketch(id)) editSketch(sketch);
    });
    connect(m_featureTreePanel, &FeatureTreePanel::sketchSelected, this,
            [this](uint64_t id) { m_profileSketchId = id; });

    // Build UI chrome.
    createMenus();
    createRibbonBar();
    completeMenusFromRibbon();
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
    m_viewport->setComponentDragger(nullptr);
    m_assemblies.reset();
    m_openTask.reset();
    m_reloadTask.reset();
    m_massTask.reset();
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

    fileMenu->addAction(tr("New &Part"), this, &MainWindow::onNewPart)
        ->setObjectName(QStringLiteral("action_new_part"));
    fileMenu->addAction(tr("New Asse&mbly"), this, &MainWindow::onNewAssembly)
        ->setObjectName(QStringLiteral("action_new_assembly"));

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
    importMenu->addAction(tr("STEP as an &Assembly..."), this, &MainWindow::onImportStepAssembly)
        ->setObjectName(QStringLiteral("import_step_assembly"));
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
        // An assembly goes out as a STEP assembly (Phase 153).
        if (m_assembly && !m_assembly->components().empty()) exportStep->setEnabled(true);
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
    editMenu->addAction(tr("Document &Units..."), this, &MainWindow::onDocumentUnits)
        ->setObjectName(QStringLiteral("action_document_units"));
    editMenu->addAction(tr("&Variables..."), this, &MainWindow::onVariables)
        ->setObjectName(QStringLiteral("action_variables"));
    editMenu->addAction(tr("C&onfigurations..."), this, &MainWindow::onConfigurations)
        ->setObjectName(QStringLiteral("action_configurations"));
    QAction* prefsAction =
        editMenu->addAction(tr("Pre&ferences..."), this, &MainWindow::onPreferences);
    prefsAction->setObjectName(QStringLiteral("action_preferences"));
    prefsAction->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_Comma));
    prefsAction->setMenuRole(QAction::PreferencesRole);

    // ---- View ----
    QMenu* viewMenu = menuBar()->addMenu(tr("&View"));
    m_viewMenu = viewMenu;

    viewMenu->addAction(tr("&Front"), this, &MainWindow::onViewFront);
    viewMenu->addAction(tr("&Top"), this, &MainWindow::onViewTop);
    viewMenu->addAction(tr("&Right"), this, &MainWindow::onViewRight);
    viewMenu->addAction(tr("&Back"), this, [this] {
        m_viewport->camera().setBackView();
        m_viewport->update();
    });
    viewMenu->addAction(tr("B&ottom"), this, [this] {
        m_viewport->camera().setBottomView();
        m_viewport->update();
    });
    viewMenu->addAction(tr("&Left"), this, [this] {
        m_viewport->camera().setLeftView();
        m_viewport->update();
    });
    viewMenu->addAction(tr("&Isometric"), this, &MainWindow::onViewIsometric);
    viewMenu->addSeparator();
    // The ribbon's Fit All, with its shortcut, takes this place
    // (completeMenusFromRibbon): one action, not two.
    m_viewFitAllPlaceholder = viewMenu->addSeparator();
    auto* ortho = viewMenu->addAction(tr("&Orthographic"));
    ortho->setObjectName(QStringLiteral("action_view_ortho"));
    ortho->setCheckable(true);
    connect(ortho, &QAction::toggled, this, [this](bool on) { m_viewport->setOrthographic(on); });
    QMenu* display = viewMenu->addMenu(tr("&Display"));
    auto* displayGroup = new QActionGroup(this);
    const auto displayAction = [&](const QString& text, const char* name,
                                   render::DisplayMode mode) {
        QAction* action = display->addAction(text);
        action->setObjectName(QString::fromLatin1(name));
        action->setCheckable(true);
        action->setChecked(mode == render::DisplayMode::ShadedWithEdges);
        displayGroup->addAction(action);
        connect(action, &QAction::triggered, this,
                [this, mode] { m_viewport->setDisplayMode(mode); });
    };
    displayAction(tr("&Shaded"), "action_display_shaded", render::DisplayMode::Shaded);
    displayAction(tr("Shaded with &Edges"), "action_display_edges",
                  render::DisplayMode::ShadedWithEdges);
    displayAction(tr("&Wireframe"), "action_display_wireframe", render::DisplayMode::Wireframe);
    auto* section = viewMenu->addAction(tr("&Section Plane..."), this, &MainWindow::onSectionPlane);
    section->setObjectName(QStringLiteral("action_section"));
    auto* noSection = viewMenu->addAction(tr("&No Section"), this,
                                          [this] { m_viewport->setSectionPlane(std::nullopt); });
    noSection->setObjectName(QStringLiteral("action_section_off"));
    viewMenu->addSeparator();
    viewMenu->addAction(m_featureTreePanel->toggleViewAction());
    viewMenu->addAction(m_assemblyTreePanel->toggleViewAction());
    viewMenu->addAction(m_propertyPanel->toggleViewAction());
    viewMenu->addAction(m_layerPanel->toggleViewAction());

    // ---- Model ----
    QMenu* modelMenu = menuBar()->addMenu(tr("&Model"));
    m_modelMenu = modelMenu;
    QMenu* newSketchMenu = modelMenu->addMenu(tr("New &Sketch"));
    const auto sketchAction = [](QMenu* menu, const QString& text, const char* name, auto slot) {
        QAction* action = menu->addAction(text);
        action->setObjectName(QString::fromLatin1(name));
        QObject::connect(action, &QAction::triggered, slot);
        return action;
    };
    sketchAction(newSketchMenu, tr("On the &XY Plane"), "action_sketch_xy",
                 [this] { onNewSketchOnPlane(0); });
    sketchAction(newSketchMenu, tr("On the X&Z Plane"), "action_sketch_xz",
                 [this] { onNewSketchOnPlane(1); });
    sketchAction(newSketchMenu, tr("On the &YZ Plane"), "action_sketch_yz",
                 [this] { onNewSketchOnPlane(2); });
    newSketchMenu->addSeparator();
    sketchAction(newSketchMenu, tr("On a &Face..."), "action_sketch_face",
                 [this] { onNewSketchOnFace(); });
    sketchAction(newSketchMenu, tr("On a &Datum Plane..."), "action_sketch_datum",
                 [this] { onNewSketchOnDatum(); });
    sketchAction(modelMenu, tr("&Edit Sketch..."), "action_sketch_edit",
                 [this] { onEditSketch(); });
    m_finishSketchAction = sketchAction(modelMenu, tr("&Finish Sketch"), "action_sketch_finish",
                                        [this] { onFinishSketch(); });
    m_finishSketchAction->setEnabled(false);
    m_projectEdgesAction = sketchAction(modelMenu, tr("&Project Edges..."), "action_project_edges",
                                        [this] { onProjectEdges(); });
    m_projectEdgesAction->setEnabled(false);
    m_constructionAction = sketchAction(modelMenu, tr("&Construction"), "action_construction",
                                        [this] { onToggleConstruction(); });
    m_constructionAction->setEnabled(false);
    modelMenu->addSeparator();
    sketchAction(modelMenu, tr("&Loft..."), "action_loft", [this] { onLoft(); });
    sketchAction(modelMenu, tr("S&weep..."), "action_sweep", [this] { onSweep(); });
    QMenu* datumMenu = modelMenu->addMenu(tr("&Datum"));
    sketchAction(datumMenu, tr("&Plane..."), "action_datum_plane", [this] { onDatumPlane(); });
    sketchAction(datumMenu, tr("&Axis..."), "action_datum_axis", [this] { onDatumAxis(); });
    sketchAction(datumMenu, tr("P&oint..."), "action_datum_point", [this] { onDatumPoint(); });

    // ---- Assembly (Phase 143: its commands were three in the File menu) ----
    QMenu* assemblyMenu = menuBar()->addMenu(tr("&Assembly"));
    sketchAction(assemblyMenu, tr("&Insert Component..."), "action_insert_component",
                 [this] { m_assemblies->onInsertComponent(); });
    sketchAction(assemblyMenu, tr("&Open Part"), "action_open_part",
                 [this] { m_assemblies->onOpenPart(); });
    sketchAction(assemblyMenu, tr("&Move Component..."), "action_move_component",
                 [this] { m_assemblies->onMoveComponent(); });
    sketchAction(assemblyMenu, tr("R&otate Component..."), "action_rotate_component",
                 [this] { m_assemblies->onRotateComponent(); });
    sketchAction(assemblyMenu, tr("Re&name Component..."), "action_rename_component",
                 [this] { m_assemblies->onRenameComponent(); });
    sketchAction(assemblyMenu, tr("&Suppress or Unsuppress Component"), "action_suppress_component",
                 [this] { m_assemblies->onSuppressComponent(); });
    sketchAction(assemblyMenu, tr("&Remove Component"), "action_remove_component",
                 [this] { m_assemblies->onRemoveComponent(); });
    assemblyMenu->addSeparator();
    sketchAction(assemblyMenu, tr("Add &Mate..."), "action_add_mate",
                 [this] { m_assemblies->onAddMate(); });
    sketchAction(assemblyMenu, tr("&Edit Mate..."), "action_edit_mate",
                 [this] { m_assemblies->onEditMate(); });
    sketchAction(assemblyMenu, tr("Remo&ve Mate..."), "action_remove_mate",
                 [this] { m_assemblies->onRemoveMate(); });
    assemblyMenu->addSeparator();
    sketchAction(assemblyMenu, tr("Check &Interference"), "action_check_interference",
                 [this] { m_assemblies->onCheckInterference(); });
    assemblyMenu->addSeparator();
    sketchAction(assemblyMenu, tr("E&xplode Components..."), "action_explode_components",
                 [this] { m_assemblies->onExplodeComponents(); });
    sketchAction(assemblyMenu, tr("Show Exploded Vie&w..."), "action_show_exploded_view",
                 [this] { m_assemblies->onShowExplodedView(); });
    sketchAction(assemblyMenu, tr("Remove Exp&loded View..."), "action_remove_exploded_view",
                 [this] { m_assemblies->onRemoveExplodedView(); });
    assemblyMenu->addSeparator();
    sketchAction(assemblyMenu, tr("&Pattern Components..."), "action_pattern_components",
                 [this] { m_assemblies->onPatternComponents(); });
    sketchAction(assemblyMenu, tr("Edit Component Pa&ttern..."), "action_edit_component_pattern",
                 [this] { m_assemblies->onEditComponentPattern(); });
    sketchAction(assemblyMenu, tr("Mirror Com&ponents..."), "action_mirror_components",
                 [this] { m_assemblies->onMirrorComponents(); });
    sketchAction(assemblyMenu, tr("Remove Component Patter&n..."),
                 "action_remove_component_pattern",
                 [this] { m_assemblies->onRemoveComponentPattern(); });
    sketchAction(assemblyMenu, tr("&Bill of Materials..."), "action_bill_of_materials",
                 [this] { m_assemblies->onBillOfMaterials(); });

    // ---- Drawing sheets (Phase 148) ----
    QMenu* drawingMenu = menuBar()->addMenu(tr("Drawin&g"));
    sketchAction(drawingMenu, tr("&New Drawing from Part or Assembly..."),
                 "action_new_drawing_from_part", [this] { m_drawings->onNewDrawingFromPart(); });
    drawingMenu->addSeparator();
    sketchAction(drawingMenu, tr("&Title Block..."), "action_drawing_title_block",
                 [this] { m_drawings->onTitleBlock(); });
    sketchAction(drawingMenu, tr("&Scale..."), "action_drawing_scale",
                 [this] { m_drawings->onScale(); });
    sketchAction(drawingMenu, tr("&Update from Part"), "action_update_drawing",
                 [this] { m_drawings->onUpdateFromPart(); });
    drawingMenu->addSeparator();
    sketchAction(drawingMenu, tr("Add Se&ction View..."), "action_add_section_view",
                 [this] { m_drawings->onAddSectionView(); });
    sketchAction(drawingMenu, tr("Add &Detail View"), "action_add_detail_view",
                 [this] { m_drawings->onAddDetailView(); });
    sketchAction(drawingMenu, tr("&Move View"), "action_move_view",
                 [this] { m_drawings->onMoveView(); });
    sketchAction(drawingMenu, tr("&Remove View..."), "action_remove_view",
                 [this] { m_drawings->onRemoveView(); });
    sketchAction(drawingMenu, tr("View &Properties..."), "action_view_properties",
                 [this] { m_drawings->onViewProperties(); });
    drawingMenu->addSeparator();
    sketchAction(drawingMenu, tr("Add D&imension"), "action_add_drawing_dimension",
                 [this] { m_drawings->onAddDimension(); });
    sketchAction(drawingMenu, tr("Remove Dime&nsion"), "action_remove_drawing_dimension",
                 [this] { m_drawings->onRemoveDimension(); });

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
    // "(2D)": the command palette lists the part's Fillet and Chamfer too.
    toolsMenu->addAction(tr("&Fillet (2D)"), this, &MainWindow::onFilletTool);
    toolsMenu->addAction(tr("C&hamfer (2D)"), this, &MainWindow::onChamferTool);
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
    QAction* guideAction = helpMenu->addAction(tr("&User Guide"), this, &MainWindow::onUserGuide);
    guideAction->setObjectName(QStringLiteral("action_user_guide"));
    guideAction->setShortcut(QKeySequence::HelpContents);
    helpMenu->addSeparator();
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
    addAction(g, "hole", tr("Hole"), this, &MainWindow::onHole);

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
    addAction(g, "mirror-3d", tr("Mirror"), this, &MainWindow::onMirror);

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
    m_statusCoords->setObjectName(QStringLiteral("statusCoords"));
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
        m_assemblies->cancelWork();
        if (m_openTask) m_openTask->cancel();  // dropped once it is read
        if (m_reloadTask) m_reloadTask->cancel();
        if (m_massTask) m_massTask->cancel();
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
    const std::string prompt =
        tool->promptText() + m_viewport->typedPoint().prompt(m_document->lengthUnit());
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
    document->undoStack().setLimit(static_cast<std::size_t>(Preferences::current().undoLimit));
    // An assembly's backing document shows its unit: the view's tools ask it.
    if (assembly) document->setLengthUnit(assembly->lengthUnit());
    DocTab tab{std::move(document), std::move(assembly), title, m_nextRecoveryKey++};
    if (!tab.assembly && tab.document->needsBuild()) {
        // A part come in unbuilt (opened, recovered): how long its model
        // takes to build is not known, so Auto builds it on a worker, and a
        // large part does not freeze the window as it opens.
        tab.lastBuildMs = kBuildTimeUnknown;
        tab.modelStale = true;
    }
    m_tabs.push_back(std::move(tab));
    int index = m_tabBar->addTab(title);
    nameCloseButton(*m_tabBar, index);
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
    m_viewport->setActiveSketch(m_document->editedSketch().get());
    for (QAction* action : {m_finishSketchAction, m_projectEdgesAction, m_constructionAction}) {
        if (action) action->setEnabled(m_document->editedSketch() != nullptr);
    }
    if (tab->modelStale) {
        rebuildFeatureTree();
    } else {
        rebuildScene();
    }
    refreshAllPanels();
    updateWindowTitle();
    // A sheet is paper, seen from above.
    if (m_drawings && m_drawings->isSheet(m_document.get())) m_drawings->shown(*m_document);
}

void MainWindow::addTab(std::shared_ptr<doc::Document> document, const QString& title) {
    addDocumentTab(std::move(document), nullptr, title);
}

void MainWindow::runTool(std::unique_ptr<Tool> tool) {
    // The viewport lets go of a tool of the same name before it is replaced.
    m_viewport->setActiveTool(nullptr);
    const std::string name = tool->name();
    m_toolManager->registerTool(std::move(tool));
    activateTool(name);
}

void MainWindow::endTool() {
    onSelectTool();
}

void MainWindow::refreshUsersOf(const std::string& path, bool report) {
    m_assemblies->refreshComponentsOf(path, report);
    m_drawings->refreshDrawingsOf(path);
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
    // A part closed with its edits discarded: components that shared its
    // document, or its model's mesh, had them. They show its file again.
    if (!tab.assembly && tab.document->isDirty() && !tab.document->filePath().empty()) {
        refreshUsersOf(tab.document->filePath());
    }
}

void MainWindow::rebuildScene() {
    // What was clicked may not be there any more: the model is built anew.
    m_viewport->setModelHover(std::nullopt);
    m_viewport->clearModelSelection();
    m_viewport->sceneGraph().clear();
    // The assembly tree lists what the assembly now holds.
    m_assemblyTreePanel->refresh(m_assembly.get());

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
            // One mesh for every instance of a part; a mirrored one's own
            // mirror of it (Phase 162).
            node->shareMesh(comp.mesh());
            // Where it is drawn: moved by the exploded view shown (Phase 161).
            node->setLocalTransform(m_assembly->displayTransform(comp));
            node->setOwnerId(comp.id);  // a click on it names the component
            node->setMaterial(render::Material{math::Vec3{0.62, 0.68, 0.75}, 0.15f, 0.5f, 32.0f});
            m_viewport->sceneGraph().addNode(node);
        }
    } else if (m_document->featureTree().featureCount() > 0) {
        // Build only a model that has not been built: a failed build keeps
        // its partial solid (or none), and building it again here, on the
        // GUI thread, would only fail again. Nor one a worker is building.
        const bool building = m_rebuildJob != nullptr && m_rebuildDocument == m_document;
        if (!m_document->solid() && m_document->needsBuild() && !building) {
            m_document->rebuildModel();
        }
        if (m_document->solid()) {
            DocTab* tab = activeTab();
            if (tab != nullptr && tab->document != m_document) tab = nullptr;
            std::shared_ptr<const geo::MeshData> mesh;
            if (tab != nullptr && tab->mesh && tab->meshBuild == m_document->builds()) {
                mesh = tab->mesh;
            } else {
                mesh = std::make_shared<const geo::MeshData>(
                    model::SolidTessellator::tessellate(*m_document->solid(), 0.1));
                ++m_tessellations;
                if (tab != nullptr) {
                    tab->mesh = mesh;
                    tab->meshBuild = m_document->builds();
                }
            }
            auto node = std::make_shared<render::SceneNode>("FeatureTree Result");
            node->shareMesh(mesh);  // the tab's, not a copy
            // While a sketch is edited the view works in its frame.
            if (const auto& sketch = m_document->editedSketch()) {
                node->setLocalTransform(sketch->plane().worldToLocalMatrix());
            }
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

    m_assemblyTreePanel->refresh(m_assembly.get());
    // The tab that fits the document in front.
    if (m_assembly) {
        m_assemblyTreePanel->raise();
    } else {
        m_featureTreePanel->raise();
    }
    m_featureTreePanel->clearFailures();
    m_featureTreePanel->refresh(m_document->featureTree());
    m_featureTreePanel->setConfigurations(m_document->configurations().configurationNames(),
                                          m_document->configurations().active());
    refreshSketchList();
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

void MainWindow::refreshModifiedIndicators() {
    for (size_t i = 0; i < m_tabs.size(); ++i) {
        const DocTab& tab = m_tabs[i];
        QString text = tab.title;
        if (tab.recovered) text += tr(" (recovered)");
        if (isTabModified(tab)) text += QStringLiteral(" *");
        const int index = static_cast<int>(i);
        if (index < m_tabBar->count() && m_tabBar->tabText(index) != text) {
            m_tabBar->setTabText(index, text);
            nameCloseButton(*m_tabBar, index);
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
        if (changed == m_document.get()) syncSketchView();
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

void MainWindow::offerCrashReports() {
    for (const QString& path : crash::pendingReports(crash::reportDirectory())) {
        const QString report = crash::readReport(path);
        QMessageBox box(this);
        box.setIcon(QMessageBox::Warning);
        box.setWindowTitle(tr("Horizon CAD Stopped"));
        box.setText(
            tr("Horizon CAD stopped unexpectedly last time. A report of what happened "
               "was kept on this computer; nothing has been sent."));
        box.setInformativeText(
            tr("To report the problem, save the report and attach it to an "
               "issue. Documents that had unsaved changes, if any, are offered "
               "next, from their autosaved copies."));
        box.setDetailedText(report);
        QPushButton* save = box.addButton(tr("Save Report..."), QMessageBox::ActionRole);
        QPushButton* discard = box.addButton(tr("Delete Report"), QMessageBox::DestructiveRole);
        box.addButton(QMessageBox::Close);
        box.exec();
        if (box.clickedButton() == discard) {
            QFile::remove(path.chopped(4) + QStringLiteral(".dmp"));  // Windows' minidump
            if (!QFile::remove(path)) {
                spdlog::warn("Could not delete the crash report {}", path.toStdString());
            }
            continue;
        }
        if (box.clickedButton() == save) {
            const QString to = QFileDialog::getSaveFileName(
                this, tr("Save Crash Report"), QFileInfo(path).fileName(), tr("Text (*.txt)"));
            QFile out(to);
            if (!to.isEmpty() && out.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
                out.write(report.toUtf8());
            }
        }
        // Kept, and not offered again (deleted, if it cannot be marked).
        if (!crash::markShown(path)) {
            spdlog::warn("Could not mark the crash report {} shown; it will be offered again",
                         path.toStdString());
        }
    }
    crash::prune(crash::reportDirectory());
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
            m_assemblies->solveAssemblyMates(*assembly);
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
    m_docManager.setNewDocumentUnit(prefs.newDocumentUnit());
    for (const DocTab& tab : m_tabs) {
        tab.document->undoStack().setLimit(static_cast<std::size_t>(prefs.undoLimit));
    }
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

void MainWindow::onDocumentUnits() {
    // Each unit by its name and symbol, in the order they are offered.
    const auto nameOf = [](math::LengthUnit unit) {
        switch (unit) {
            case math::LengthUnit::Millimetre:
                return tr("Millimetres (mm)");
            case math::LengthUnit::Centimetre:
                return tr("Centimetres (cm)");
            case math::LengthUnit::Metre:
                return tr("Metres (m)");
            case math::LengthUnit::Inch:
                return tr("Inches (in)");
            case math::LengthUnit::Foot:
                return tr("Feet (ft)");
        }
        return QString();
    };
    QStringList names;
    int current = 0;
    for (std::size_t k = 0; k < math::kLengthUnits.size(); ++k) {
        names << nameOf(math::kLengthUnits[k]);
        if (math::kLengthUnits[k] == m_document->lengthUnit()) current = static_cast<int>(k);
    }
    FeatureForm form(this, tr("Document Units"), m_document->lengthUnit());
    QComboBox* unit = form.choice(QStringLiteral("unit"), tr("Lengths in:"), names);
    unit->setCurrentIndex(current);
    auto* note = new QLabel(tr("The model is kept in millimetres: its unit changes only how its "
                               "lengths are shown and typed, and is saved with it."),
                            &form.dialog());
    note->setWordWrap(true);
    form.dialog().layout()->addWidget(note);
    if (!form.exec()) return;
    const math::LengthUnit chosen =
        math::kLengthUnits[static_cast<std::size_t>(std::max(unit->currentIndex(), 0))];
    if (chosen == m_document->lengthUnit()) return;
    m_document->undoStack().push(
        std::make_unique<doc::SetLengthUnitCommand>(*m_document, chosen, m_assembly.get()));
    refreshAllPanels();
    m_viewport->update();
    m_statusPrompt->setText(tr("Lengths are in %1.").arg(nameOf(chosen).toLower()));
}

void MainWindow::onVariables() {
    if (m_assembly) {
        statusBar()->showMessage(tr("Variables belong to a part or a drawing, not an assembly"));
        return;
    }
    const auto before = m_document->parameterRegistry().definitions();
    VariablesDialog dialog(before, m_document->lengthUnit(), this);
    dialog.setConfigurations(m_document->configurations());
    if (dialog.exec() != QDialog::Accepted) return;
    auto after = dialog.definitions();
    if (after == before) return;
    const auto count = static_cast<int>(after.size());
    m_document->undoStack().push(
        std::make_unique<doc::SetVariablesCommand>(*m_document, std::move(after)));
    // Features whose sizes are expressions of them are built again.
    rebuildFeatureTree();
    m_statusPrompt->setText(tr("%n variable(s).", "", count));
}

void MainWindow::onConfigurations() {
    if (m_assembly) {
        statusBar()->showMessage(tr("Configurations belong to a part, not an assembly"));
        return;
    }
    const doc::ConfigurationTable& before = m_document->configurations();
    ConfigurationsDialog dialog(before, m_document->parameterRegistry().definitions(), this);
    if (dialog.exec() != QDialog::Accepted) return;
    doc::ConfigurationTable after = dialog.table();
    if (after == before) return;
    const auto count = static_cast<int>(after.size());
    m_document->undoStack().push(
        std::make_unique<doc::SetConfigurationsCommand>(*m_document, std::move(after)));
    rebuildFeatureTree();
    m_statusPrompt->setText(tr("%n configuration(s).", "", count));
}

void MainWindow::onConfigurationChosen(const QString& name) {
    doc::ConfigurationTable table = m_document->configurations();
    if (!table.setActive(name.toStdString()) || table == m_document->configurations()) return;
    m_document->undoStack().push(std::make_unique<doc::SetConfigurationsCommand>(
        *m_document, std::move(table), "Configuration"));
    rebuildFeatureTree();
    m_statusPrompt->setText(name.isEmpty() ? tr("Built with its own variables.")
                                           : tr("Built as %1.").arg(name));
}

void MainWindow::onUserGuide() {
    if (m_help == nullptr) {
        m_help = new HelpWindow(this);
        m_help->setAttribute(Qt::WA_DeleteOnClose);
    }
    m_help->showPage(helpPageForContext());
    m_help->show();
    m_help->raise();
    m_help->activateWindow();
}

QString MainWindow::helpPageForContext() const {
    if (m_document == nullptr) return QStringLiteral("index.md");
    switch (m_document->type()) {
        case doc::DocumentType::Assembly:
            return QStringLiteral("assemblies.md");
        case doc::DocumentType::Part:
            return m_viewport->activeSketch() != nullptr ? QStringLiteral("sketches.md")
                                                         : QStringLiteral("parts.md");
        case doc::DocumentType::Drawing:
            break;
    }
    return QStringLiteral("drafting.md");
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
        tr("All Supported Files (*.hcad *.hzpart *.hzasm *.hzdwg *.dxf);;"
           "Horizon CAD Drawings (*.hcad);;Horizon Parts (*.hzpart);;"
           "Horizon Assemblies (*.hzasm);;Horizon Drawing Sheets (*.hzdwg);;"
           "DXF Files (*.dxf);;All Files (*)"));
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

    if (fileName.endsWith(".hzdwg", Qt::CaseInsensitive)) {
        if (!m_drawings->open(fileName)) return false;
        RecentFiles::add(fileName);
        return true;
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
        // Saved assemblies come back positioned by their mates: modified, if
        // that moved anything, so the placing can be saved.
        if (m_assemblies->placeOnOpen(*assembly)) assembly->setDirty(true);
        auto backing = m_docManager.newDocument(doc::DocumentType::Assembly);
        addDocumentTab(std::move(backing), std::move(assembly),
                       tabTitleForPath(path, tr("Assembly")));
        showImportReport(QFileInfo(fileName).fileName(), report);
        RecentFiles::add(fileName);
        return true;
    }

    // A large drawing or part is read on a worker, and the window stays free
    // while it is: reading one took seconds.
    const bool drawing = fileName.endsWith(".dxf", Qt::CaseInsensitive);
    if (openOnWorker(fileName)) return startOpen(fileName, drawing);

    if (drawing) {
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
        showOpened(std::move(document), fileName, tr("Drawing"), std::move(report));
        return true;
    }

    // .hcad and .hzpart both load through NativeFormat (full document —
    // entities, sketches, feature tree, design variables).
    auto document = m_docManager.openPart(path);
    if (!document) {
        reportFileError(tr("Could not open"), path, m_lastLoadError);
        return false;
    }
    showOpened(std::move(document), fileName, tr("Document"), m_lastLoadReport);
    return true;
}

void MainWindow::showOpened(std::shared_ptr<doc::Document> document, const QString& fileName,
                            const QString& fallbackTitle, io::ImportReport report) {
    // The manager dedups by canonical path: a document already open is
    // already in a tab, and that tab is shown.
    for (size_t i = 0; i < m_tabs.size(); ++i) {
        if (m_tabs[i].document == document) {
            m_tabBar->setCurrentIndex(static_cast<int>(i));
            RecentFiles::add(fileName);
            return;
        }
    }
    addDocumentTab(std::move(document), nullptr,
                   tabTitleForPath(fileName.toStdString(), fallbackTitle));
    showImportReport(QFileInfo(fileName).fileName(), report);
    RecentFiles::add(fileName);
}

MainWindow::FileOpen MainWindow::readFile(const std::string& path, bool drawing) {
    FileOpen open;
    auto document = std::make_shared<doc::Document>();
    bool read = false;
    if (drawing) {
        document->setType(doc::DocumentType::Drawing);
        read = io::DxfFormat::load(path, *document, &open.error, &open.report);
    } else {
        read = io::NativeFormat::load(path, *document, &open.error, &open.report);
    }
    if (read) open.document = std::move(document);
    return open;
}

bool MainWindow::openOnWorker(const QString& fileName) const {
    return m_rebuildMode == RebuildMode::Always ||
           (m_rebuildMode == RebuildMode::Auto && QFileInfo(fileName).size() >= kWorkerImportBytes);
}

bool MainWindow::startOpen(const QString& fileName, bool drawing) {
    if (m_openTask) {
        statusBar()->showMessage(
            tr("%1 is still being opened").arg(QFileInfo(m_openFile).fileName()));
        return false;
    }
    m_openFile = fileName;
    m_openDrawing = drawing;
    const std::string path = fileName.toStdString();
    m_openTask = std::make_unique<BackgroundTask<FileOpen>>(
        [path, drawing](const std::atomic<bool>& /*cancelled*/) {
            return readFile(path, drawing);
        });
    m_openTask->start([this] {
        QMetaObject::invokeMethod(this, &MainWindow::onOpenFinished, Qt::QueuedConnection);
    });
    m_statusPrompt->setText(tr("Opening %1...").arg(QFileInfo(fileName).fileName()));
    updateBusyIndicator();
    return true;
}

void MainWindow::onOpenFinished() {
    if (!m_openTask || !m_openTask->finished()) return;
    const std::unique_ptr<BackgroundTask<FileOpen>> task = std::move(m_openTask);
    const QString fileName = std::exchange(m_openFile, QString());
    updateBusyIndicator();
    m_statusPrompt->setText(tr("Ready"));
    if (task->cancelled()) {
        statusBar()->showMessage(tr("Open cancelled"), 10000);
        return;
    }
    FileOpen open = task->take();
    if (!task->error().empty()) open.error = task->error();
    const std::string path = fileName.toStdString();
    if (!open.document) {
        reportFileError(tr("Could not open"), path, open.error);
        return;
    }
    std::shared_ptr<doc::Document> document;
    if (m_openDrawing) {
        open.document->setFilePath(path);
        open.document->setDirty(false);
        m_docManager.adoptDocument(open.document);
        document = std::move(open.document);
    } else {
        // Opened meanwhile, from another path to it: that one is shown.
        document = m_docManager.adoptPart(path, std::move(open.document));
    }
    showOpened(std::move(document), fileName, m_openDrawing ? tr("Drawing") : tr("Document"),
               std::move(open.report));
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
            // Its drawings show it as saved, and so do the assemblies that
            // place it (Phase 159).
            refreshUsersOf(m_assembly->filePath(), /*report=*/false);
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
    if (m_drawings->isSheet(m_document.get())) {
        const std::string sheetPath = m_document->filePath();
        std::string error;
        if (!m_drawings->save(*m_document, sheetPath, &error)) {
            reportFileError(tr("Could not save"), sheetPath, error);
            return false;
        }
        m_document->setDirty(false);
        m_docManager.noteSaved(m_document);
        forgetSnapshot(*tab);
        RecentFiles::add(QString::fromStdString(sheetPath));
        m_statusPrompt->setText(tr("Drawing saved."));
        updateWindowTitle();
        return true;
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
        // The assemblies placing it show it as saved.
        refreshUsersOf(path);
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
    const bool sheet = !m_assembly && m_drawings->isSheet(m_document.get());
    if (m_assembly) {
        filter = tr("Horizon Assemblies (*.hzasm);;All Files (*)");
    } else if (sheet) {
        filter = tr("Horizon Drawing Sheets (*.hzdwg);;All Files (*)");
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

    if (sheet) {
        // A sheet is only ever a .hzdwg: that is how it is opened again.
        if (!fileName.endsWith(".hzdwg", Qt::CaseInsensitive)) fileName += QStringLiteral(".hzdwg");
        const std::string oldPath = m_document->filePath();
        m_document->setFilePath(fileName.toStdString());
        if (!saveActiveDocument()) m_document->setFilePath(oldPath);
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

MainWindow::StepLoad MainWindow::loadStep(const std::string& path, const std::string& assemblyPath,
                                          const std::atomic<bool>* cancelled) {
    StepLoad load;
    load.assemblyPath = assemblyPath;
    if (assemblyPath.empty()) {
        load.solids = io::StepFormat::load(path, &load.report, cancelled);
        if (load.solids.empty()) load.error = io::StepFormat::lastError();
    } else {
        load.assembly = io::StepFormat::loadAssembly(path, &load.report, cancelled);
        if (load.assembly.parts.empty()) load.error = io::StepFormat::lastError();
    }
    return load;
}

void MainWindow::onImportStep() {
    const QString fileName = QFileDialog::getOpenFileName(
        this, tr("Import STEP"), QString(), tr("STEP Files (*.step *.stp);;All Files (*)"));
    if (fileName.isEmpty()) return;
    startStepImport(fileName, QString());
}

void MainWindow::onImportStepAssembly() {
    const QString fileName =
        QFileDialog::getOpenFileName(this, tr("Import STEP as an Assembly"), QString(),
                                     tr("STEP Files (*.step *.stp);;All Files (*)"));
    if (fileName.isEmpty()) return;
    // Where the assembly goes; its parts go in a folder beside it.
    const QFileInfo step(fileName);
    QString assemblyPath = QFileDialog::getSaveFileName(
        this, tr("Save the Assembly As"),
        step.dir().filePath(step.completeBaseName() + QStringLiteral(".hzasm")),
        tr("Horizon Assemblies (*.hzasm)"));
    if (assemblyPath.isEmpty()) return;
    if (QFileInfo(assemblyPath).suffix().isEmpty()) assemblyPath += QStringLiteral(".hzasm");
    startStepImport(fileName, assemblyPath);
}

void MainWindow::startStepImport(const QString& fileName, const QString& assemblyPath) {
    const std::string path = fileName.toStdString();
    const std::string keptAt = assemblyPath.toStdString();
    const bool onWorker =
        m_rebuildMode == RebuildMode::Always ||
        (m_rebuildMode == RebuildMode::Auto && QFileInfo(fileName).size() >= kWorkerImportBytes);
    if (!onWorker) {
        finishStepImport(fileName, loadStep(path, keptAt));
        return;
    }
    if (m_importTask) {
        statusBar()->showMessage(tr("A STEP import is already running"));
        return;
    }
    m_importFile = fileName;
    m_importTask = std::make_unique<BackgroundTask<StepLoad>>(
        [path, keptAt](const std::atomic<bool>& cancelled) {
            return loadStep(path, keptAt, &cancelled);
        });
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
    if (!load.assemblyPath.empty()) {
        finishStepAssemblyImport(fileName, std::move(load));
        return;
    }
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

void MainWindow::finishStepAssemblyImport(const QString& fileName, StepLoad load) {
    if (load.assembly.parts.empty()) {
        reportFileError(tr("Could not import"), fileName.toStdString(), load.error);
        return;
    }
    // Each part a part file, in a folder named for the assembly beside it:
    // the assembly refers to its parts by their files.
    const QFileInfo kept(QString::fromStdString(load.assemblyPath));
    const QString partsDir =
        kept.dir().filePath(kept.completeBaseName() + QStringLiteral(" parts"));
    const auto parts = static_cast<int>(load.assembly.parts.size());
    const auto placed = static_cast<int>(load.assembly.occurrences.size());
    io::StepAssemblyFiles files;
    std::string error;
    if (!io::saveStepAssembly(load.assembly, load.assemblyPath, partsDir.toStdString(),
                              QFileInfo(fileName).fileName().toStdString(), &files, &error)) {
        reportFileError(tr("Could not import"), load.assemblyPath, error);
        return;
    }
    if (!openPath(QString::fromStdString(files.assembly))) return;
    showImportReport(QFileInfo(fileName).fileName(), load.report);
    m_statusPrompt->setText(
        tr("Imported %n part(s) into \"%1\", ", "", parts).arg(QDir::toNativeSeparators(partsDir)) +
        tr("placed %n time(s).", "", placed));
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
    if (m_assembly) {
        exportAssemblyStep();
        return;
    }
    const topo::Solid* solid = solidToExport(tr("STEP"));
    if (!solid) return;
    const QString fileName =
        askExportPath(tr("STEP"), tr("STEP Files (*.step *.stp)"), QStringLiteral(".step"));
    if (fileName.isEmpty()) return;
    io::StepFormat::WriteReport report;
    if (!io::StepFormat::save(fileName.toStdString(), {solid}, {}, &report)) {
        reportFileError(tr("Could not export"), fileName.toStdString(),
                        io::StepFormat::lastError());
        return;
    }
    m_statusPrompt->setText(tr("Exported %1.").arg(QFileInfo(fileName).fileName()));
    showStepExportReport(report);
}

void MainWindow::exportAssemblyStep() {
    // Each part once, and each component a use of it where it is placed.
    const AssemblyWorkbench::StepExport gathered = m_assemblies->stepExport();
    if (gathered.occurrences.empty()) {
        statusBar()->showMessage(
            tr("STEP export writes an assembly's components; none of this one's parts can be "
               "read"));
        return;
    }
    const QString fileName =
        askExportPath(tr("STEP"), tr("STEP Files (*.step *.stp)"), QStringLiteral(".step"));
    if (fileName.isEmpty()) return;
    const QString title =
        m_assembly->filePath().empty()
            ? tr("Assembly")
            : QFileInfo(QString::fromStdString(m_assembly->filePath())).completeBaseName();
    io::StepFormat::WriteReport report;
    if (!io::StepFormat::saveAssembly(fileName.toStdString(), title.toStdString(), gathered.parts,
                                      gathered.occurrences, {}, &report)) {
        reportFileError(tr("Could not export"), fileName.toStdString(),
                        io::StepFormat::lastError());
        return;
    }
    m_statusPrompt->setText(tr("Exported %1.").arg(QFileInfo(fileName).fileName()));
    showStepExportReport(report, gathered.unread);
}

void MainWindow::showStepExportReport(const io::StepWriteReport& report,
                                      const std::vector<std::string>& unread) {
    // Written as designed, but for faces that could not be: said, not
    // left for the other system to find as a mesh of facets. And any
    // component left out.
    QStringList lines;
    for (const std::string& name : unread) {
        lines
            << tr("Left out: \"%1\": its part could not be read").arg(QString::fromStdString(name));
    }
    for (const std::string& why : report.leftOut) {
        lines << tr("Left out: %1").arg(QString::fromStdString(why));
    }
    for (const std::string& why : report.faceted) lines << QString::fromStdString(why);
    if (lines.isEmpty()) return;
    const auto leftOut = static_cast<int>(unread.size() + report.leftOut.size());
    const QString faceted =
        report.faceted.empty()
            ? QString()
            : tr("%n curved face(s) were written as their facets, not on their surfaces.", nullptr,
                 static_cast<int>(report.faceted.size()));
    QMessageBox box(leftOut > 0 ? QMessageBox::Warning : QMessageBox::Information,
                    tr("Export STEP"),
                    leftOut > 0 ? (tr("%n component(s) were not written.", nullptr, leftOut) +
                                   (faceted.isEmpty() ? QString() : QStringLiteral(" ") + faceted))
                                : faceted,
                    QMessageBox::Ok, this);
    if (!report.faceted.empty()) {
        box.setInformativeText(
            tr("The part is exact as modelled; other systems will see those "
               "faces as flat facets."));
    }
    box.setDetailedText(lines.join(QLatin1Char('\n')));
    box.exec();
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

    FeatureForm form(this, tr("Export %1").arg(format), m_document->lengthUnit());
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
    // A drawing sheet is drawn to size: plotted on its own paper, at 1:1.
    if (const model::Sheet* sheet = m_drawings->paperOf(m_document.get())) {
        const double shortSide = std::min(sheet->widthMm(), sheet->heightMm());
        const double longSide = std::max(sheet->widthMm(), sheet->heightMm());
        const auto& sizes = draft::standardPaperSizes();
        for (size_t i = 0; i < sizes.size(); ++i) {
            if (std::abs(sizes[i].widthMm - shortSide) < 0.5 &&
                std::abs(sizes[i].heightMm - longSide) < 0.5) {
                paper->setCurrentIndex(static_cast<int>(i));
            }
        }
        orientation->setCurrentIndex(sheet->widthMm() >= sheet->heightMm() ? 0 : 1);
        scale->setCurrentIndex(scale->findText(QStringLiteral("1:1")));
    }
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

// ---------------------------------------------------------------------------
// Placing components (Phase 143)
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Living assemblies (Phase 144)
// ---------------------------------------------------------------------------

void MainWindow::pollPartFiles() {
    // Not under a dialog: one may be choosing among the components' faces.
    if (QApplication::activeModalWidget() != nullptr) return;
    for (const std::string& path : m_docManager.pollExternalChanges()) {
        // A tab showing the file first: a component of a part open in a tab
        // takes the tab's document, which is as old as its read.
        if (!reloadTabsOf(path)) refreshUsersOf(path);
    }
}

bool MainWindow::reloadTabsOf(const std::string& path) {
    const QString name = QFileInfo(QString::fromStdString(path)).fileName();
    for (size_t i = 0; i < m_tabs.size(); ++i) {
        const DocTab& tab = m_tabs[i];
        const std::string& tabPath =
            tab.assembly ? tab.assembly->filePath() : tab.document->filePath();
        if (!doc::DocumentManager::samePath(tabPath, path)) continue;
        if (tab.assembly) {
            statusBar()->showMessage(
                tr("\"%1\" was changed by another program: close it and open it again to "
                   "see the change")
                    .arg(name),
                15000);
            continue;
        }
        const bool modified = isTabModified(tab);
        if (modified) {
            // Never lose the user's edits unasked: keeping them is the default.
            m_tabBar->setCurrentIndex(static_cast<int>(i));
            const std::shared_ptr<doc::Document> asked = tab.document;
            QMessageBox box(QMessageBox::Warning, tr("File Changed on Disk"),
                            tr("\"%1\" was changed by another program, and has unsaved "
                               "changes here.")
                                .arg(name),
                            QMessageBox::Discard | QMessageBox::Ignore, this);
            box.setInformativeText(
                tr("Read it again, losing your changes, or keep your version? Saving yours "
                   "replaces the other program's."));
            box.button(QMessageBox::Discard)->setText(tr("Read Again"));
            box.button(QMessageBox::Ignore)->setText(tr("Keep Mine"));
            box.setDefaultButton(QMessageBox::Ignore);
            box.setEscapeButton(QMessageBox::Ignore);
            if (box.exec() != QMessageBox::Discard) return false;
            // Its place, found again: the tabs may have changed meanwhile.
            i = 0;
            while (i < m_tabs.size() && m_tabs[i].document != asked) ++i;
            if (i == m_tabs.size()) return false;
        }
        reloadTab(i, modified);
        return true;  // one tab shows a file
    }
    return false;
}

void MainWindow::reloadTab(size_t index, bool asked) {
    const std::shared_ptr<doc::Document> old = m_tabs[index].document;
    if (m_drawings->isSheet(old.get())) {
        m_drawings->readAgain(*old);  // a small file, and its part: read here
        return;
    }
    const std::string path = old->filePath();
    const QString fileName = QString::fromStdString(path);
    const bool drawing = fileName.endsWith(".dxf", Qt::CaseInsensitive);
    // A large file is read on a worker, as it is opened: here, reading it
    // again froze the window every time another program saved it.
    if (!openOnWorker(fileName)) {
        const bool replaced = replaceTabDocument(old, readFile(path, drawing), asked);
        refreshUsersOf(path, replaced);
        return;
    }
    if (m_reloadTask) {
        // Its turn comes. Queued by path: the running reading may be of this
        // file, and may have read it before this change.
        m_reloadQueue.emplace_back(path, asked ? old : nullptr);
        return;
    }
    m_reloadDocument = old;
    m_reloadAsked = asked;
    m_reloadTask = std::make_unique<BackgroundTask<FileOpen>>(
        [path, drawing](const std::atomic<bool>& /*cancelled*/) {
            return readFile(path, drawing);
        });
    m_reloadTask->start([this] {
        QMetaObject::invokeMethod(this, &MainWindow::onReloadFinished, Qt::QueuedConnection);
    });
    m_statusPrompt->setText(tr("Reading %1 again...").arg(QFileInfo(fileName).fileName()));
    updateBusyIndicator();
}

void MainWindow::onReloadFinished() {
    if (!m_reloadTask || !m_reloadTask->finished()) return;
    const std::unique_ptr<BackgroundTask<FileOpen>> task = std::move(m_reloadTask);
    const std::shared_ptr<doc::Document> old = std::move(m_reloadDocument);
    updateBusyIndicator();
    m_statusPrompt->setText(tr("Ready"));
    const std::string path = old->filePath();
    bool replaced = false;
    if (task->cancelled()) {
        statusBar()->showMessage(
            tr("Reading \"%1\" again was cancelled: its tab shows it as it was read before")
                .arg(QFileInfo(QString::fromStdString(path)).fileName()),
            10000);
    } else {
        FileOpen read = task->take();
        if (!task->error().empty()) read.error = task->error();
        replaced = replaceTabDocument(old, std::move(read), m_reloadAsked);
    }
    refreshUsersOf(path, replaced);
    // The files waiting, until one is read on a worker again.
    while (!m_reloadTask && !m_reloadQueue.empty()) {
        const auto [next, givenUp] = m_reloadQueue.front();
        m_reloadQueue.erase(m_reloadQueue.begin());
        const std::shared_ptr<doc::Document> askedAbout = givenUp.lock();
        size_t i = 0;
        while (i < m_tabs.size() && m_tabs[i].document != askedAbout) ++i;
        if (askedAbout && i < m_tabs.size()) {
            reloadTab(i, true);  // its changes given up already
        } else if (!reloadTabsOf(next)) {
            refreshUsersOf(next);
        }
    }
}

bool MainWindow::replaceTabDocument(const std::shared_ptr<doc::Document>& old, FileOpen read,
                                    bool asked) {
    size_t index = 0;
    while (index < m_tabs.size() && m_tabs[index].document != old) ++index;
    if (index == m_tabs.size()) return false;  // closed meanwhile
    DocTab& tab = m_tabs[index];
    const std::string path = old->filePath();
    const QString name = QFileInfo(QString::fromStdString(path)).fileName();
    if (!read.document) {
        statusBar()->showMessage(
            tr("\"%1\" was changed by another program, and could not be read again: %2")
                .arg(name, QString::fromStdString(read.error)),
            15000);
        return false;
    }
    // Edited while it was read on a worker, and not given up: kept.
    if (!asked && isTabModified(tab)) {
        statusBar()->showMessage(
            tr("\"%1\" was changed by another program; the changes you made meanwhile are "
               "kept, and saving them replaces the other program's")
                .arg(name),
            15000);
        return false;
    }
    const std::shared_ptr<doc::Document> fresh = std::move(read.document);
    fresh->setFilePath(path);
    fresh->setDirty(false);

    // The old document goes: its build, if one runs, is of no use.
    if (m_rebuildJob && m_rebuildDocument == old) m_rebuildJob->cancel();
    old->setChangeCallback(nullptr);
    forgetSnapshot(tab);
    m_docManager.closeDocument(old);
    m_docManager.adoptDocument(fresh);
    m_docManager.noteSaved(fresh);  // found by its path, and watched from now
    watchDocument(fresh);
    fresh->undoStack().setLimit(static_cast<std::size_t>(Preferences::current().undoLimit));
    tab.document = fresh;
    tab.mesh.reset();
    tab.meshBuild = 0;
    tab.lastBuildMs = kBuildTimeUnknown;
    tab.modelStale = fresh->needsBuild();

    if (&tab == activeTab()) activateTabDocument();
    refreshModifiedIndicators();
    statusBar()->showMessage(
        tr("\"%1\" was changed by another program, and has been read again").arg(name), 10000);
    return true;
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
    // A drag under way is put back first (Phase 158): its release would
    // record a step from a snapshot taken before this undo, and undo it.
    m_viewport->cancelComponentDrag();
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
    for (const auto& entity : m_document->activeDrawing().entities()) {
        if (!sel.isSelected(entity->id())) continue;
        const auto* lp = layerMgr.getLayer(entity->layer());
        if (!lp || !lp->visible || lp->locked) continue;
        idVec.push_back(entity->id());
    }
    if (idVec.empty()) return;

    math::Vec2 offset(1.0, -1.0);
    auto cmd =
        std::make_unique<doc::DuplicateEntityCommand>(m_document->activeDrawing(), idVec, offset);
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
    for (const auto& entity : m_document->activeDrawing().entities()) {
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
    auto& drawing = m_document->activeDrawing();
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
    for (const auto& entity : m_document->activeDrawing().entities()) {
        auto entityBBox = entity->boundingBox();
        if (entityBBox.isValid()) {
            bbox.expand(entityBBox);
        }
    }
    // And the solids, where they are shown: they were left out, so a part
    // with no drawing fitted nothing.
    for (const render::SceneNode* node : m_viewport->sceneGraph().collectVisibleMeshNodes()) {
        const math::Mat4 world = node->worldTransform();
        const auto& positions = node->mesh().positions;
        for (size_t i = 0; i + 2 < positions.size(); i += 3) {
            bbox.expand(
                world.transformPoint(math::Vec3(positions[i], positions[i + 1], positions[i + 2])));
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
    for (const auto& entity : m_document->activeDrawing().entities()) {
        if (!sel.isSelected(entity->id())) continue;
        const auto* lp = layerMgr.getLayer(entity->layer());
        if (!lp || !lp->visible || lp->locked) continue;
        filteredIds.push_back(entity->id());
    }
    if (filteredIds.empty()) return;

    RectArrayDialog dlg(this, m_document->lengthUnit());
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
                if (const auto entity = m_document->activeDrawing().sharedEntity(id)) {
                    auto clone = entity->clone();
                    clone->translate(offset);
                    newIds.push_back(clone->id());
                    allClones.push_back(clone);
                    composite->addCommand(std::make_unique<doc::AddEntityCommand>(
                        m_document->activeDrawing(), clone));
                }
            }
        }
    }

    doc::adoptClones(m_document->activeDrawing(), allClones);
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
    for (const auto& entity : m_document->activeDrawing().entities()) {
        if (!sel.isSelected(entity->id())) continue;
        const auto* lp = layerMgr.getLayer(entity->layer());
        if (!lp || !lp->visible || lp->locked) continue;
        filteredIds.push_back(entity->id());
    }
    if (filteredIds.empty()) return;

    PolarArrayDialog dlg(this, m_document->lengthUnit());
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
            if (const auto entity = m_document->activeDrawing().sharedEntity(id)) {
                auto clone = entity->clone();
                clone->rotate(center, angle);
                newIds.push_back(clone->id());
                allClones.push_back(clone);
                composite->addCommand(
                    std::make_unique<doc::AddEntityCommand>(m_document->activeDrawing(), clone));
            }
        }
    }

    doc::adoptClones(m_document->activeDrawing(), allClones);
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
    draft::DraftDocument& drawing = m_document->activeDrawing();
    const draft::DimensionStyle& now = drawing.dimensionStyle();
    const QStringList units = Preferences::lengthUnits();

    FeatureForm form(this, tr("Dimension Style"), m_document->lengthUnit());
    auto* height = form.length(QStringLiteral("textHeight"), tr("Text height:"), now.textHeight,
                               0.01, 1000.0, 3);
    auto* arrow =
        form.length(QStringLiteral("arrowSize"), tr("Arrow size:"), now.arrowSize, 0.0, 1000.0, 3);
    auto* angle = form.angle(QStringLiteral("arrowAngle"), tr("Arrow half-angle:"),
                             now.arrowAngle * math::kRadToDeg, 1.0, 89.0, 1);
    auto* gap = form.length(QStringLiteral("extensionGap"), tr("Extension gap:"), now.extensionGap,
                            0.0, 1000.0, 3);
    auto* overshoot = form.length(QStringLiteral("extensionOvershoot"), tr("Extension overshoot:"),
                                  now.extensionOvershoot, 0.0, 1000.0, 3);
    auto* precision =
        form.count(QStringLiteral("precision"), tr("Decimal places:"), now.precision, 0, 12);
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
    for (const auto& entity : m_document->activeDrawing().entities()) {
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
        if (const auto* e = m_document->activeDrawing().findEntity(id)) {
            const auto bb = e->boundingBox();
            if (bb.isValid()) bounds.expand(bb);
        }
    }
    const math::Vec3 centre = bounds.isValid() ? bounds.center() : math::Vec3(0, 0, 0);
    FeatureForm form(this, tr("Create Block"), m_document->lengthUnit());
    auto* nameField = form.text(QStringLiteral("blockName"), tr("Block name:"));
    auto* baseX = form.length(QStringLiteral("baseX"), tr("Base point X:"), centre.x, -1e9, 1e9, 4);
    auto* baseY = form.length(QStringLiteral("baseY"), tr("Base point Y:"), centre.y, -1e9, 1e9, 4);
    if (!form.exec()) return;
    const QString name = nameField->text().trimmed();
    if (name.isEmpty()) return;

    std::string blockName = name.toStdString();
    if (m_document->activeDrawing().blockTable().findBlock(blockName)) {
        QMessageBox::warning(this, tr("Create Block"),
                             tr("A block with that name already exists."));
        return;
    }

    const math::Vec2 base(baseX->value(), baseY->value());
    auto cmd = std::make_unique<doc::CreateBlockCommand>(m_document->activeDrawing(), blockName,
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
    auto names = m_document->activeDrawing().blockTable().blockNames();
    if (names.empty()) {
        QMessageBox::information(this, tr("Insert Block"),
                                 tr("No blocks defined. Create a block first."));
        return;
    }

    InsertBlockDialog dlg(names, this);
    if (dlg.exec() != QDialog::Accepted) return;

    auto def = m_document->activeDrawing().blockTable().findBlock(dlg.selectedBlock());
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
    for (const auto& entity : m_document->activeDrawing().entities()) {
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
        auto cmd = std::make_unique<doc::ExplodeBlockCommand>(m_document->activeDrawing(), id);
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
    for (const auto& entity : m_document->activeDrawing().entities()) {
        if (!sel.isSelected(entity->id())) continue;
        const auto* lp = layerMgr.getLayer(entity->layer());
        if (!lp || !lp->visible || lp->locked) continue;
        filteredIds.push_back(entity->id());
    }
    if (filteredIds.size() < 2) return;

    auto cmd =
        std::make_unique<doc::GroupEntitiesCommand>(m_document->activeDrawing(), filteredIds);
    m_document->undoStack().push(std::move(cmd));
    m_viewport->update();
}

void MainWindow::onUngroupEntities() {
    auto& sel = m_viewport->selectionManager();
    auto ids = sel.selectedIds();
    if (ids.empty()) return;

    // Collect groupIds from selected entities.
    std::set<uint64_t> groupIds;
    for (const auto& entity : m_document->activeDrawing().entities()) {
        if (!sel.isSelected(entity->id())) continue;
        if (entity->groupId() != 0) {
            groupIds.insert(entity->groupId());
        }
    }
    if (groupIds.empty()) return;

    std::vector<uint64_t> groupIdVec(groupIds.begin(), groupIds.end());
    auto cmd =
        std::make_unique<doc::UngroupEntitiesCommand>(m_document->activeDrawing(), groupIdVec);
    m_document->undoStack().push(std::move(cmd));
    m_viewport->update();
}

// ---------------------------------------------------------------------------
// Slots -- Status bar updates
// ---------------------------------------------------------------------------

void MainWindow::onMouseMoved(const hz::math::Vec2& worldPos) {
    const Preferences& prefs = Preferences::current();
    const math::LengthUnit unit = m_document->lengthUnit();
    m_statusCoords->setText(
        tr("X: %1  Y: %2")
            .arg(prefs.formatLength(worldPos.x, unit), prefs.formatLength(worldPos.y, unit)));

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
    FeatureForm form(this, verb, m_document->lengthUnit());
    std::vector<QDoubleSpinBox*> sizes;
    sizes.reserve(fields.size());
    for (size_t i = 0; i < fields.size(); ++i) {
        sizes.push_back(form.length(QStringLiteral("size%1").arg(i), fields[i].label,
                                    fields[i].value, fields[i].min, 1e6));
    }
    // Where it stands: from a base point, its own z axis along one of the
    // axis directions (Phase 134; primitives always stood at the origin).
    auto* atX = form.length(QStringLiteral("atX"), tr("At x:"), 0.0, -1e6, 1e6);
    auto* atY = form.length(QStringLiteral("atY"), tr("y:"), 0.0, -1e6, 1e6);
    auto* atZ = form.length(QStringLiteral("atZ"), tr("z:"), 0.0, -1e6, 1e6);
    auto* axis =
        directionChoice(form, QStringLiteral("axis"), tr("Standing along:"), QStringLiteral("+Z"));
    auto* result = form.operationChoice(proposedOperation());
    if (!form.exec()) return;

    std::vector<double> values;
    values.reserve(sizes.size());
    for (const auto* spin : sizes) values.push_back(spin->value());
    auto feature = make(values);
    feature->setVector("basePoint", math::Vec3(atX->value(), atY->value(), atZ->value()));
    feature->setVector("axisDirection", chosenDirection(axis));
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

    // The sketch being edited, finished; else the one chosen in the list, or
    // last made or finished.
    if (auto edited = m_document->editedSketch()) {
        onFinishSketch();
        return edited;
    }
    if (auto chosen = m_document->findSketch(m_profileSketchId);
        chosen && !chosen->entities().empty()) {
        return chosen;
    }

    // Otherwise the drawing itself, as it is shown: what is on a hidden
    // layer is not part of it. (Notes on it are passed over by the profile.)
    std::vector<std::shared_ptr<draft::DraftEntity>> shown;
    for (const auto& entity : m_document->draftDocument().entities()) {
        const auto* layer = m_document->layerManager().getLayer(entity->layer());
        if (layer == nullptr || layer->visible) shown.push_back(entity);
    }
    if (shown.empty()) return nullptr;

    // Reuse an existing wrapper sketch when the top-level profile has not
    // changed — repeated extrudes must not accumulate duplicate sketches.
    for (const auto& sk : m_document->sketches()) {
        if (sk->entities() == shown) return sk;
    }

    // Wrap the top-level profile in a sketch so the feature is replayable
    // (parametric history requires a sketch reference). The caller must add
    // it to the document only once the operation is validated.
    auto sketch = std::make_shared<doc::Sketch>();
    sketch->setName(tr("Profile %1").arg(m_document->sketches().size() + 1).toStdString());
    for (const auto& entity : shown) sketch->addEntity(entity);
    createdWrapper = true;
    return sketch;
}

void MainWindow::newSketchOn(const draft::SketchPlane& plane, const QString& where,
                             const std::string& face) {
    if (m_assembly) {
        statusBar()->showMessage(tr("An assembly has no sketches: sketch in a part"));
        return;
    }
    auto sketch = std::make_shared<doc::Sketch>(plane);
    sketch->setName(tr("Sketch %1").arg(m_document->sketches().size() + 1).toStdString());
    sketch->setFace(face);
    m_document->undoStack().push(std::make_unique<doc::AddSketchCommand>(*m_document, sketch));
    editSketch(sketch);
    statusBar()->showMessage(
        tr("Sketching on %1: draw the profile, then Model > Finish Sketch").arg(where));
}

void MainWindow::onProjectEdges() {
    const auto sketch = m_document->editedSketch();
    if (!sketch) {
        statusBar()->showMessage(tr("Edges are projected into a sketch: edit one first"));
        return;
    }
    const topo::Solid* solid = m_document->solid();
    if (solid == nullptr) {
        statusBar()->showMessage(tr("There is no part to project edges from"));
        return;
    }
    // Each edge once, by the name it is kept by: a Boolean's pieces of one
    // straight edge are listed apart by edgesOf, but are one edge here.
    PickList edges;
    {
        const PickList all = edgesOf(*solid);
        std::set<std::string> listed;
        for (size_t row = 0; row < all.ids.size(); ++row) {
            const std::string whole = model::wholeEdgeName(all.ids[row].tag());
            if (!listed.insert(whole).second) continue;
            edges.ids.push_back(topo::TopologyID::fromTag(whole));
            edges.items.push_back(all.items[row]);
        }
    }
    if (edges.ids.empty()) {
        statusBar()->showMessage(tr("The part has no edges to project"));
        return;
    }
    FeatureForm form(this, tr("Project Edges"), m_document->lengthUnit());
    auto* list = form.checklist(QStringLiteral("edges"), tr("Edges:"), edges.items);
    checkClicked(list, edges, m_viewport->modelSelection(), true);
    auto* kind = form.choice(QStringLiteral("kind"), tr("As:"),
                             {tr("Construction geometry"), tr("Part of the profile")});
    if (!form.exec()) return;

    auto& drawing = m_document->activeDrawing();
    auto add = std::make_unique<doc::CompositeCommand>(tr("Project Edges").toStdString());
    int made = 0;
    QStringList missed;
    for (const int row : FeatureForm::checkedRows(list)) {
        // By its whole name, which lasts through a later feature's Boolean.
        const std::string& name = edges.ids[static_cast<size_t>(row)].tag();
        std::string why;
        auto entity = model::projectEdge(*solid, name, sketch->plane(), &why);
        if (!entity) {
            missed << tr("%1 %2").arg(QString::fromStdString(name), QString::fromStdString(why));
            continue;
        }
        // It follows the edge: each build projects it again, from the part.
        entity->setSourceEdge(name);
        entity->setConstruction(kind->currentIndex() == 0);
        entity->setLayer(m_document->layerManager().currentLayer());
        add->addCommand(std::make_unique<doc::AddEntityCommand>(drawing, std::move(entity)));
        ++made;
    }
    if (made == 0) {
        statusBar()->showMessage(
            missed.isEmpty() ? tr("No edges were chosen")
                             : tr("Nothing projected: %1").arg(missed.join(QStringLiteral("; "))));
        return;
    }
    m_document->undoStack().push(std::move(add));
    m_viewport->update();
    QString message = tr("%n edge(s) projected: they follow the part", "", made);
    if (!missed.isEmpty()) {
        message += tr("; not %1").arg(missed.join(QStringLiteral("; ")));
    }
    statusBar()->showMessage(message);
}

void MainWindow::onToggleConstruction() {
    if (!m_document->editedSketch()) {
        statusBar()->showMessage(tr("Construction geometry is drawn in a sketch: edit one first"));
        return;
    }
    auto& drawing = m_document->activeDrawing();
    std::vector<uint64_t> ids;
    bool all = true;  // all construction already: made geometry again
    for (const uint64_t id : m_viewport->selectionManager().selectedIds()) {
        if (const auto entity = drawing.sharedEntity(id)) {
            ids.push_back(id);
            all = all && entity->construction();
        }
    }
    if (ids.empty()) {
        statusBar()->showMessage(tr("Select lines, arcs or circles of the sketch first"));
        return;
    }
    m_document->undoStack().push(
        std::make_unique<doc::ChangeEntityConstructionCommand>(drawing, ids, !all));
    m_viewport->update();
    const auto count = static_cast<int>(ids.size());
    statusBar()->showMessage(all ? tr("%n made part of the profile again", "", count)
                                 : tr("%n made construction geometry", "", count));
}

void MainWindow::onNewSketchOnPlane(int which) {
    using math::Vec3;
    // Each seen from outside the part's positive octant: XY from above, XZ
    // from the front, YZ from the right; x across and y up the screen.
    switch (which) {
        case 1:
            newSketchOn(draft::SketchPlane(Vec3::Zero, Vec3(0, -1, 0), Vec3::UnitX),
                        tr("the XZ plane"));
            break;
        case 2:
            newSketchOn(draft::SketchPlane(Vec3::Zero, Vec3::UnitX, Vec3::UnitY),
                        tr("the YZ plane"));
            break;
        default:
            newSketchOn(draft::SketchPlane(), tr("the XY plane"));
            break;
    }
}

void MainWindow::onNewSketchOnFace() {
    if (m_assembly || !m_document->solid()) {
        statusBar()->showMessage(tr("There is no part to sketch on a face of"));
        return;
    }
    const std::vector<PlaneChoice> faces = planarFacesOf(*m_document->solid());
    if (faces.empty()) {
        statusBar()->showMessage(tr("The part has no flat face to sketch on"));
        return;
    }
    // A face clicked in the viewport is the one: no need to ask. The first
    // flat one, if several were clicked.
    bool clickedAFace = false;
    for (const auto& pick : m_viewport->modelSelection()) {
        if (pick.edge || pick.owner != 0) continue;
        clickedAFace = true;
        const auto clicked =
            std::find_if(faces.begin(), faces.end(),
                         [&pick](const PlaneChoice& f) { return f.tag == pick.tag; });
        if (clicked != faces.end()) {
            newSketchOn(clicked->plane, tr("a face"), model::wholeFaceName(clicked->tag));
            return;
        }
    }
    if (clickedAFace) {
        statusBar()->showMessage(tr("The face clicked is not flat: a sketch needs a flat face"));
        return;
    }
    QStringList names;
    for (const auto& face : faces) names << face.text;
    FeatureForm form(this, tr("Sketch on a Face"), m_document->lengthUnit());
    auto* choice = form.choice(QStringLiteral("face"), tr("Face:"), names);
    if (!form.exec()) return;
    const auto& picked = faces[static_cast<size_t>(std::max(choice->currentIndex(), 0))];
    newSketchOn(picked.plane, tr("a face"), model::wholeFaceName(picked.tag));
}

void MainWindow::onNewSketchOnDatum() {
    std::vector<draft::SketchPlane> planes;
    QStringList names;
    const auto& tree = m_document->featureTree();
    for (size_t i = 0; i < tree.featureCount(); ++i) {
        const auto* datum = dynamic_cast<const doc::DatumFeature*>(tree.feature(i));
        if (!datum || datum->datumKind() != doc::DatumFeature::DatumKind::Plane) continue;
        planes.push_back(datum->asPlane().toSketchPlane());
        names << QString::fromStdString(datum->name());
    }
    if (m_assembly || planes.empty()) {
        statusBar()->showMessage(tr("The part has no datum plane to sketch on"));
        return;
    }
    FeatureForm form(this, tr("Sketch on a Datum Plane"), m_document->lengthUnit());
    auto* choice = form.choice(QStringLiteral("datum"), tr("Datum plane:"), names);
    if (!form.exec()) return;
    const int index = std::max(choice->currentIndex(), 0);
    newSketchOn(planes[static_cast<size_t>(index)], names[index]);
}

void MainWindow::onEditSketch() {
    const auto& sketches = m_document->sketches();
    if (m_assembly || sketches.empty()) {
        statusBar()->showMessage(
            tr("There is no sketch to edit: make one with Model > New Sketch"));
        return;
    }
    QStringList names;
    int current = 0;
    for (size_t i = 0; i < sketches.size(); ++i) {
        names << QString::fromStdString(sketches[i]->name());
        if (sketches[i]->id() == m_profileSketchId) current = static_cast<int>(i);
    }
    FeatureForm form(this, tr("Edit Sketch"), m_document->lengthUnit());
    auto* choice = form.choice(QStringLiteral("sketch"), tr("Sketch:"), names);
    choice->setCurrentIndex(current);
    if (!form.exec()) return;
    editSketch(sketches[static_cast<size_t>(std::max(choice->currentIndex(), 0))]);
}

void MainWindow::onFinishSketch() {
    const auto edited = m_document->editedSketch();
    if (!edited) return;
    m_profileSketchId = edited->id();
    editSketch(nullptr);
    statusBar()->showMessage(
        tr("%1 finished: Extrude or Revolve takes it").arg(QString::fromStdString(edited->name())));
}

void MainWindow::completeMenusFromRibbon() {
    const auto ribbon = [this](const char* name) {
        return findChild<QAction*>(QString::fromLatin1(name));
    };
    // One Fit All, the ribbon's, which has the shortcut F.
    if (QAction* fit = ribbon("action_fit-all"); fit && m_viewFitAllPlaceholder) {
        fit->setText(tr("Fit &All"));
        m_viewMenu->insertAction(m_viewFitAllPlaceholder, fit);
    }
    if (!m_modelMenu) return;
    // Shortcuts for the commands used most. The single letters are the 2D
    // tools', so these take Ctrl+Shift.
    const std::vector<std::pair<const char*, QKeySequence>> shortcuts = {
        {"action_extrude", QKeySequence(Qt::CTRL | Qt::SHIFT | Qt::Key_E)},
        {"action_revolve", QKeySequence(Qt::CTRL | Qt::SHIFT | Qt::Key_R)},
        {"action_fillet-3d", QKeySequence(Qt::CTRL | Qt::SHIFT | Qt::Key_F)},
        {"action_chamfer-3d", QKeySequence(Qt::CTRL | Qt::SHIFT | Qt::Key_C)},
        {"action_shell", QKeySequence(Qt::CTRL | Qt::SHIFT | Qt::Key_H)},
    };
    for (const auto& [name, keys] : shortcuts) {
        if (QAction* action = ribbon(name)) {
            action->setShortcut(keys);
            action->setToolTip(QStringLiteral("%1 (%2)").arg(
                action->text(), keys.toString(QKeySequence::NativeText)));
        }
    }
    // Named for the palette, where "Linear" alone would be a dimension too.
    if (QAction* linear = ribbon("action_pattern-linear")) linear->setText(tr("Linear Pattern"));
    if (QAction* circular = ribbon("action_pattern-circular")) {
        circular->setText(tr("Circular Pattern"));
    }
    m_modelMenu->addSeparator();
    const std::vector<std::vector<const char*>> groups = {
        {"action_box", "action_cylinder", "action_sphere", "action_cone", "action_torus"},
        {"action_extrude", "action_revolve", "action_hole"},
        {"action_boolean-union", "action_boolean-subtract", "action_boolean-intersect"},
        {"action_fillet-3d", "action_chamfer-3d", "action_shell", "action_draft"},
        {"action_pattern-linear", "action_pattern-circular", "action_mirror-3d"},
    };
    for (const auto& group : groups) {
        for (const char* name : group) {
            if (QAction* action = ribbon(name)) m_modelMenu->addAction(action);
        }
        m_modelMenu->addSeparator();
    }
    auto* mass =
        m_modelMenu->addAction(tr("&Mass Properties..."), this, &MainWindow::onMassProperties);
    mass->setObjectName(QStringLiteral("action_mass_properties"));
    mass->setShortcut(QKeySequence(Qt::CTRL | Qt::SHIFT | Qt::Key_M));
}

void MainWindow::onMassProperties() {
    const QString verb = tr("Mass Properties");
    const topo::Solid* solid = requireBody(verb);
    if (!solid) return;
    const std::vector<std::pair<QString, std::optional<model::Material>>> materials = {
        {tr("None (volume only)"), std::nullopt},
        {tr("Steel"), model::Material::steel()},
        {tr("Aluminium"), model::Material::aluminum()},
        {tr("Titanium"), model::Material::titanium()},
        {tr("ABS plastic"), model::Material::absPlastic()},
    };
    QStringList names;
    for (const auto& [name, material] : materials) names << name;
    FeatureForm form(this, verb, m_document->lengthUnit());
    auto* choice = form.choice(QStringLiteral("material"), tr("Material:"), names);
    if (!form.exec()) return;
    // Named, not a structured binding: the worker's lambda captures the
    // material, and Clang before 16 (CI's clang-tidy) cannot capture those.
    const auto& chosen = materials[static_cast<size_t>(std::max(choice->currentIndex(), 0))];
    const QString& materialName = chosen.first;
    const std::optional<model::Material>& material = chosen.second;
    const model::Material* density = material ? &*material : nullptr;
    const auto modelled = model::MassPropertiesCalculator::compute(*solid, density);
    if (!modelled.valid) {
        statusBar()->showMessage(tr("The part's mass properties could not be worked out"));
        return;
    }

    const auto n = [](double v) { return QString::number(v, 'g', 7); };
    // In the document's unit (Phase 154). Superscripts by code point: the
    // sources are not compiled as UTF-8.
    const math::LengthUnit lengthUnit = m_document->lengthUnit();
    const double per = math::millimetresPer(lengthUnit);
    const std::string_view symbol = math::symbolOf(lengthUnit);
    const QString u = QString::fromLatin1(symbol.data(), static_cast<qsizetype>(symbol.size()));
    const QString u2 = u + QChar(0x00B2);
    const QString u3 = u + QChar(0x00B3);
    const QString cm3 = QStringLiteral("cm") + QChar(0x00B3);
    // The model is in millimetres and densities in kg/m3: a cubic millimetre
    // at 1 kg/m3 weighs a millionth of a gram, and the kernel's inertia,
    // density times mm^5, is in g mm2 times a million.
    constexpr double kGramsPerUnit = 1e-6;
    const bool weighed = material.has_value();
    const auto measures = [=](const model::MassProperties& props) {
        const math::Vec3& c = props.centerOfMass;
        QString text =
            tr("Volume: %1 %2\nSurface area: %3 %4\nCentre of mass: %5 %6")
                .arg(n(props.volume / (per * per * per)), u3, n(props.surfaceArea / (per * per)),
                     u2, formatPoint(math::Vec3{c.x / per, c.y / per, c.z / per}), u);
        if (weighed) text += tr("\nMass: %1 g").arg(n(props.mass * kGramsPerUnit));
        return text;
    };
    const auto inertia = [=](const model::MassProperties& props, const QString& which) {
        const auto& I = props.inertia;
        // g mm2 into g unit2; per unit density, mm5 into unit5.
        const double scale =
            weighed ? kGramsPerUnit / (per * per) : 1.0 / (per * per * per * per * per);
        const QString unit =
            weighed ? QStringLiteral("g ") + u2 : tr("per unit density, %1").arg(u + QChar(0x2075));
        return tr("Inertia about the centre of mass (%1, %2):\n%3  %4  %5\n%6  %7  %8\n%9  %10  "
                  "%11")
            .arg(which, unit, n(I.at(0, 0) * scale), n(I.at(0, 1) * scale), n(I.at(0, 2) * scale),
                 n(I.at(1, 0) * scale), n(I.at(1, 1) * scale), n(I.at(1, 2) * scale),
                 n(I.at(2, 0) * scale))
            .arg(n(I.at(2, 1) * scale), n(I.at(2, 2) * scale));
    };
    const QString header =
        weighed ? tr("%1: density %2 g/%3\n\n").arg(materialName, n(modelled.density / 1000.0), cm3)
                : QString();

    // Faces that approximate a curved surface: without any, the part is
    // exactly as modelled.
    std::size_t curved = 0;
    for (const auto& face : solid->faces()) curved += face.analyticSurface ? 1 : 0;
    bool curvedEdges = false;
    for (const auto& edge : solid->edges()) {
        curvedEdges = curvedEdges || edge.analyticCurve || (edge.curve && edge.curve->degree() > 1);
    }
    if (curved == 0 && !curvedEdges) {
        QMessageBox::information(
            this, verb,
            header + measures(modelled) +
                tr("\n\nEvery face is flat, so the part is exactly as modelled.\n\n") +
                inertia(modelled, tr("as modelled")));
        return;
    }

    // As modelled, the facets (what Booleans and export use); ideally, the
    // surfaces they approximate. The ideal is null while it is measured, or
    // not there, for @p why.
    m_massText = [=](const model::IdealMassProperties* ideal, const QString& why) {
        QString text = header + tr("As modelled (its facets, which Booleans and export use):\n") +
                       measures(modelled) +
                       tr("\n\nIdeal (on the curved surfaces the facets approximate):\n");
        if (ideal == nullptr) {
            return text + why + QStringLiteral("\n\n") + inertia(modelled, tr("as modelled"));
        }
        text += measures(ideal->properties);
        if (ideal->exact) {
            text += tr("\nExact: every curved face is measured on its surface.");
        } else {
            if (!ideal->withoutIdeal.empty()) {
                QStringList faces;
                for (const auto& face : ideal->withoutIdeal) faces << QString::fromStdString(face);
                text += tr("\nNot exact: measured as modelled, with no curved surface recorded: "
                           "%1.")
                            .arg(faces.join(QStringLiteral(", ")));
            }
            if (ideal->partedEdges > 0) {
                text +=
                    tr("\nNot exact: %n edge(s) where the curved surfaces on either side do "
                       "not meet.",
                       "", ideal->partedEdges);
            }
        }
        return text + QStringLiteral("\n\n") + inertia(ideal->properties, tr("ideal"));
    };

    const bool onWorker = m_rebuildMode == RebuildMode::Always ||
                          (m_rebuildMode == RebuildMode::Auto && curved >= kWorkerIdealFaces);
    if (!onWorker) {
        const auto ideal = model::MassPropertiesCalculator::computeIdeal(*solid, density);
        QMessageBox::information(this, verb,
                                 ideal.properties.valid
                                     ? m_massText(&ideal, {})
                                     : m_massText(nullptr, tr("It could not be worked out.")));
        return;
    }
    if (m_massTask) {
        statusBar()->showMessage(tr("Mass properties are already being measured"));
        return;
    }
    // A copy: the part can be edited, or its tab closed, while it is measured.
    std::shared_ptr<const topo::Solid> copy =
        model::Pattern::transformed(*solid, math::Mat4::identity());
    m_massTask = std::make_unique<BackgroundTask<model::IdealMassProperties>>(
        [copy, material](const std::atomic<bool>& cancelled) {
            return model::MassPropertiesCalculator::computeIdeal(
                *copy, material ? &*material : nullptr, 1e-10, &cancelled);
        });
    // Shown now, and the ideal filled in when it is measured; closing it
    // stops the measuring.
    auto* box = new QMessageBox(QMessageBox::Information, verb,
                                m_massText(nullptr, tr("Measuring...")), QMessageBox::Ok, this);
    box->setAttribute(Qt::WA_DeleteOnClose);
    connect(box, &QDialog::finished, this, [this] {
        if (m_massTask) m_massTask->cancel();
    });
    m_massBox = box;
    box->open();
    m_massTask->start([this] {
        QMetaObject::invokeMethod(this, &MainWindow::onMassPropertiesFinished,
                                  Qt::QueuedConnection);
    });
    m_statusPrompt->setText(tr("Measuring mass properties..."));
    updateBusyIndicator();
}

void MainWindow::onMassPropertiesFinished() {
    if (!m_massTask || !m_massTask->finished()) return;
    const std::unique_ptr<BackgroundTask<model::IdealMassProperties>> task = std::move(m_massTask);
    updateBusyIndicator();
    m_statusPrompt->setText(tr("Ready"));
    if (!m_massBox) return;  // closed: nobody waits for it
    if (task->cancelled()) {
        m_massBox->setText(m_massText(nullptr, tr("Not measured: cancelled.")));
        return;
    }
    if (!task->error().empty()) {
        m_massBox->setText(m_massText(
            nullptr,
            tr("It could not be worked out: %1").arg(QString::fromStdString(task->error()))));
        return;
    }
    const auto ideal = task->take();
    m_massBox->setText(ideal.properties.valid
                           ? m_massText(&ideal, {})
                           : m_massText(nullptr, tr("It could not be worked out.")));
}

void MainWindow::onSectionPlane() {
    FeatureForm form(this, tr("Section Plane"), m_document->lengthUnit());
    auto* axis = form.choice(QStringLiteral("axis"), tr("Across:"), {tr("X"), tr("Y"), tr("Z")});
    auto* at = form.length(QStringLiteral("offset"), tr("At:"), 0.0, -1e6, 1e6);
    auto* keep = form.choice(QStringLiteral("keep"), tr("Keep:"),
                             {tr("What is below it"), tr("What is above it")});
    if (!form.exec()) return;
    math::Vec3 normal;
    (axis->currentIndex() == 0 ? normal.x : axis->currentIndex() == 1 ? normal.y : normal.z) = 1.0;
    // The shader keeps the points with n.p + w >= 0.
    const bool below = keep->currentIndex() == 0;
    const math::Vec3 n = below ? normal * -1.0 : normal;
    m_viewport->setSectionPlane(math::Vec4(n, below ? at->value() : -at->value()));
}

void MainWindow::onLoft() {
    if (m_assembly) return;
    if (m_document->editedSketch()) onFinishSketch();
    std::vector<std::shared_ptr<doc::Sketch>> drawn;
    std::vector<std::pair<QString, QString>> items;
    for (const auto& sketch : m_document->sketches()) {
        if (sketch->entities().empty()) continue;
        drawn.push_back(sketch);
        items.emplace_back(
            QString::fromStdString(sketch->name()),
            tr("on the plane through %1").arg(formatPoint(sketch->plane().origin())));
    }
    if (drawn.size() < 2) {
        statusBar()->showMessage(tr("A loft joins two or more sketches: make them first"));
        return;
    }
    FeatureForm form(this, tr("Loft"), m_document->lengthUnit());
    auto* list = form.checklist(QStringLiteral("sections"), tr("Sections, in order:"), items);
    auto* result = form.operationChoice(proposedOperation());
    if (!form.exec()) return;
    std::vector<std::shared_ptr<doc::Sketch>> sections;
    for (const int row : FeatureForm::checkedRows(list)) {
        sections.push_back(drawn[static_cast<size_t>(row)]);
    }
    if (sections.size() < 2) {
        statusBar()->showMessage(tr("Loft not added: choose two sections or more"));
        return;
    }
    auto feature = std::make_unique<doc::LoftFeature>(std::move(sections));
    feature->setOperation(FeatureForm::operation(result));
    addModelFeature(std::move(feature), tr("Loft"));
}

void MainWindow::onSweep() {
    if (m_assembly) return;
    if (m_document->editedSketch()) onFinishSketch();
    std::vector<std::shared_ptr<doc::Sketch>> drawn;
    QStringList names;
    for (const auto& sketch : m_document->sketches()) {
        if (sketch->entities().empty()) continue;
        drawn.push_back(sketch);
        names << QString::fromStdString(sketch->name());
    }
    if (drawn.size() < 2) {
        statusBar()->showMessage(
            tr("A sweep takes a profile sketch along a path sketch: make them first"));
        return;
    }
    FeatureForm form(this, tr("Sweep"), m_document->lengthUnit());
    auto* profile = form.choice(QStringLiteral("profile"), tr("Profile:"), names);
    auto* path = form.choice(QStringLiteral("path"), tr("Path:"), names);
    path->setCurrentIndex(1);
    auto* result = form.operationChoice(proposedOperation());
    if (!form.exec()) return;
    if (profile->currentIndex() == path->currentIndex()) {
        statusBar()->showMessage(tr("Sweep not added: the profile and the path are one sketch"));
        return;
    }
    auto feature =
        std::make_unique<doc::SweepFeature>(drawn[static_cast<size_t>(profile->currentIndex())],
                                            drawn[static_cast<size_t>(path->currentIndex())]);
    feature->setOperation(FeatureForm::operation(result));
    addModelFeature(std::move(feature), tr("Sweep"));
}

void MainWindow::onDatumPlane() {
    if (m_assembly) return;
    // Principal planes, and a flat face clicked on the part.
    std::vector<std::pair<QString, model::DatumPlane>> bases = {
        {tr("The XY plane"), {math::Vec3::Zero, math::Vec3::UnitZ, math::Vec3::UnitX}},
        {tr("The XZ plane"), {math::Vec3::Zero, math::Vec3(0, -1, 0), math::Vec3::UnitX}},
        {tr("The YZ plane"), {math::Vec3::Zero, math::Vec3::UnitX, math::Vec3::UnitY}},
    };
    if (m_document->solid()) {
        for (const auto& pick : m_viewport->modelSelection()) {
            if (pick.edge || pick.owner != 0) continue;
            for (const auto& face : planarFacesOf(*m_document->solid())) {
                if (face.tag != pick.tag) continue;
                bases.insert(bases.begin(),
                             {tr("The face clicked"),
                              {face.plane.origin(), face.plane.normal(), face.plane.xAxis()}});
                break;
            }
            break;
        }
    }
    QStringList names;
    for (const auto& [name, plane] : bases) names << name;
    FeatureForm form(this, tr("Datum Plane"), m_document->lengthUnit());
    auto* base = form.choice(QStringLiteral("base"), tr("From:"), names);
    auto* offset =
        form.length(QStringLiteral("offset"), tr("Offset along its normal:"), 10.0, -1e6, 1e6);
    auto* angle =
        form.angle(QStringLiteral("angle"), tr("Turned about its x axis:"), 0.0, -360.0, 360.0, 3);
    if (!form.exec()) return;
    model::DatumPlane plane = bases[static_cast<size_t>(std::max(base->currentIndex(), 0))].second;
    if (angle->value() != 0.0) {
        plane = model::refgeo::planeAtAngle(plane, plane.origin, plane.xAxis,
                                            angle->value() * math::kDegToRad);
    }
    plane = model::refgeo::planeOffset(plane, offset->value());
    addModelFeature(doc::DatumFeature::makePlane(plane), tr("Datum Plane"));
}

void MainWindow::onDatumAxis() {
    if (m_assembly) return;
    std::vector<std::pair<QString, math::Vec3>> directions = {
        {tr("X"), math::Vec3::UnitX}, {tr("Y"), math::Vec3::UnitY}, {tr("Z"), math::Vec3::UnitZ}};
    if (m_document->solid()) {
        if (const auto clicked =
                clickedDirection(*m_document->solid(), m_viewport->modelSelection())) {
            directions.insert(directions.begin(), {tr("As the face or edge clicked"), *clicked});
        }
    }
    QStringList names;
    for (const auto& [name, direction] : directions) names << name;
    FeatureForm form(this, tr("Datum Axis"), m_document->lengthUnit());
    auto* along = form.choice(QStringLiteral("direction"), tr("Along:"), names);
    auto* x = form.length(QStringLiteral("x"), tr("Through x:"), 0.0, -1e6, 1e6);
    auto* y = form.length(QStringLiteral("y"), tr("y:"), 0.0, -1e6, 1e6);
    auto* z = form.length(QStringLiteral("z"), tr("z:"), 0.0, -1e6, 1e6);
    if (!form.exec()) return;
    const auto axis = model::refgeo::axisFromDirection(
        math::Vec3(x->value(), y->value(), z->value()),
        directions[static_cast<size_t>(std::max(along->currentIndex(), 0))].second);
    addModelFeature(doc::DatumFeature::makeAxis(axis), tr("Datum Axis"));
}

void MainWindow::onDatumPoint() {
    if (m_assembly) return;
    FeatureForm form(this, tr("Datum Point"), m_document->lengthUnit());
    auto* x = form.length(QStringLiteral("x"), tr("x:"), 0.0, -1e6, 1e6);
    auto* y = form.length(QStringLiteral("y"), tr("y:"), 0.0, -1e6, 1e6);
    auto* z = form.length(QStringLiteral("z"), tr("z:"), 0.0, -1e6, 1e6);
    if (!form.exec()) return;
    addModelFeature(doc::DatumFeature::makePoint(
                        model::refgeo::pointAt(math::Vec3(x->value(), y->value(), z->value()))),
                    tr("Datum Point"));
}

void MainWindow::editSketch(const std::shared_ptr<doc::Sketch>& sketch) {
    if (sketch) m_profileSketchId = sketch->id();
    m_document->editSketch(sketch);
    syncSketchView();
}

void MainWindow::syncSketchView() {
    if (!m_document || !m_viewport) return;
    doc::Sketch* editing = m_document->editedSketch().get();
    if (m_viewport->activeSketch() != editing) {
        // A tool mid-way through something holds points in the frame it
        // began in; however the frame changes (Edit or Finish Sketch, or an
        // undo that takes the sketch away), it starts again.
        if (m_viewport->activeTool()) m_viewport->activeTool()->cancel();
        m_viewport->setActiveSketch(editing);
        rebuildScene();
        refreshAllPanels();
    }
    for (QAction* action : {m_finishSketchAction, m_projectEdgesAction, m_constructionAction}) {
        if (action) action->setEnabled(editing != nullptr);
    }
    refreshSketchList();
}

void MainWindow::refreshSketchList() {
    if (!m_featureTreePanel || !m_document) return;
    std::vector<FeatureTreePanel::SketchRow> rows;
    const auto& tree = m_document->featureTree();
    for (const auto& sketch : m_document->sketches()) {
        FeatureTreePanel::SketchRow row;
        row.id = sketch->id();
        row.name = sketch->name();
        row.face = sketch->face();
        row.editing = sketch == m_document->editedSketch();
        for (size_t i = 0; i < tree.featureCount() && row.usedBy.empty(); ++i) {
            const doc::Feature* feature = tree.feature(i);
            if (feature == nullptr) continue;
            const auto sketches = feature->sketches();
            if (std::find(sketches.begin(), sketches.end(), sketch) != sketches.end()) {
                row.usedBy = feature->name();
            }
        }
        rows.push_back(std::move(row));
    }
    m_featureTreePanel->refreshSketches(rows, m_profileSketchId);
}

void MainWindow::onExtrudeSketch() {
    if (!m_viewport || !m_viewport->document()) return;

    bool createdWrapper = false;
    auto sketch = resolveProfileSketch(createdWrapper);
    if (!sketch || sketch->entities().empty()) {
        statusBar()->showMessage(tr("Draw a closed profile first"));
        return;
    }

    // The part's flat faces parallel to the sketch, to go up to (Phase 157):
    // the one clicked, if any, first.
    std::vector<PlaneChoice> faces;
    if (const topo::Solid* part = m_document->solid()) {
        for (auto& face : planarFacesOf(*part)) {
            if (face.plane.normal().cross(sketch->plane().normal()).length() > 1e-9) continue;
            faces.push_back(std::move(face));
        }
    }
    const auto clicked = std::find_if(faces.begin(), faces.end(), [this](const PlaneChoice& f) {
        const auto& picks = m_viewport->modelSelection();
        return std::any_of(picks.begin(), picks.end(), [&f](const ViewportWidget::ModelPick& p) {
            return p.owner == 0 && !p.edge && p.tag == f.tag;
        });
    });
    if (clicked != faces.end()) std::rotate(faces.begin(), clicked, clicked + 1);

    FeatureForm form(this, tr("Extrude"), m_document->lengthUnit());
    auto* size = form.length(QStringLiteral("size"), tr("Distance:"), 10.0, 0.01, 1e6, 2);
    auto* goes = form.choice(QStringLiteral("extent"), tr("Goes:"),
                             {tr("To the distance"), tr("Both ways, half each"), tr("Through all"),
                              tr("Through all, both ways"), tr("Up to a face")});
    QStringList faceNames;
    for (const auto& face : faces) faceNames << face.text;
    if (faceNames.isEmpty())
        faceNames << tr("(no flat face of the part is parallel to the sketch)");
    auto* upTo = form.choice(QStringLiteral("upToFace"), tr("Up to face:"), faceNames);
    auto* way = form.choice(QStringLiteral("way"), tr("Direction:"),
                            {tr("Out of the sketch"), tr("Reversed")});
    auto* result = form.operationChoice(proposedOperation());
    if (!form.exec()) return;
    const double distance = size->value();
    const auto extent = static_cast<doc::ExtrudeFeature::Extent>(goes->currentIndex());
    const doc::BodyOperation operation = FeatureForm::operation(result);
    std::string upToFace;
    if (extent == doc::ExtrudeFeature::Extent::UpToFace) {
        if (faces.empty()) {
            statusBar()->showMessage(
                tr("Extrude not added: no flat face of the part is parallel to the sketch"));
            return;
        }
        const auto& chosen = faces[static_cast<size_t>(std::max(upTo->currentIndex(), 0))];
        upToFace = model::wholeFaceName(chosen.tag);
    }

    // As the sketch was drawn: one placed on a face takes it along (Phase 157).
    const draft::SketchPlane& drawn = sketch->drawnPlane();
    const math::Vec3 direction = drawn.normal() * (way->currentIndex() == 1 ? -1.0 : 1.0);

    // Validate the profile BEFORE mutating the document: a failed extrude
    // must not leave a wrapper sketch or a dead feature behind.
    std::string why;
    auto probe = model::Extrude::execute(sketch->entities(), drawn, direction, distance, "probe",
                                         model::Extrude::kDefaultSegments, 0.0, &why);
    if (!probe) {
        statusBar()->showMessage(tr("Extrude failed: %1").arg(QString::fromStdString(why)));
        return;
    }

    auto feature = std::make_unique<doc::ExtrudeFeature>(sketch, direction, distance);
    feature->setExtent(extent);
    feature->setUpToFace(upToFace);
    feature->setOperation(operation);
    if (!addModelFeature(std::move(feature), tr("Extrude"), createdWrapper ? sketch : nullptr)) {
        return;
    }

    refreshSketchList();
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

    FeatureForm form(this, tr("Revolve"), m_document->lengthUnit());
    auto* size = form.angle(QStringLiteral("size"), tr("Angle:"), 360.0, 1.0, 360.0, 1);
    // About one of the sketch's own axes, through its origin: on the XY
    // plane, the world's Y or X.
    auto* axis =
        form.choice(QStringLiteral("axis"), tr("Axis:"),
                    {tr("The sketch's vertical axis (Y)"), tr("The sketch's horizontal axis (X)")});
    auto* result = form.operationChoice(proposedOperation());
    if (!form.exec()) return;
    const double angle = size->value() * std::numbers::pi / 180.0;
    const doc::BodyOperation operation = FeatureForm::operation(result);

    // As the sketch was drawn: one placed on a face takes it along (Phase 157).
    const draft::SketchPlane& plane = sketch->drawnPlane();
    const math::Vec3 axisPoint = plane.origin();
    const math::Vec3 axisDir = axis->currentIndex() == 1 ? plane.xAxis() : plane.yAxis();

    std::string why;
    auto probe = model::Revolve::execute(sketch->entities(), plane, axisPoint, axisDir, angle,
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

    refreshSketchList();
    m_viewport->camera().setIsometricView();
    m_viewport->update();
}

bool MainWindow::askForBodyFeature(const QString& title, const QString& valueLabel, double& value,
                                   double min, double max, int decimals,
                                   doc::BodyOperation& operation) {
    FeatureForm form(this, title, m_document->lengthUnit());
    auto* size = form.number(QStringLiteral("size"), valueLabel, value, min, max, decimals);
    auto* result = form.operationChoice(proposedOperation());
    if (!form.exec()) return false;
    value = size->value();
    operation = FeatureForm::operation(result);
    return true;
}

bool MainWindow::addModelFeature(std::unique_ptr<doc::Feature> feature, const QString& verb,
                                 const std::shared_ptr<doc::Sketch>& wrapperSketch) {
    // The feature goes in as its step and the model is built once, on a
    // worker when builds are slow. It was built twice: first to try the
    // feature, always here on the GUI thread, then again to show it. One
    // that fails itself is refused when its build is shown
    // (settlePendingAdd).
    auto command =
        std::make_unique<doc::AddFeatureCommand>(*m_document, std::move(feature), wrapperSketch);
    PendingAdd pending;
    pending.document = m_document;
    pending.step = command.get();
    pending.feature = command->feature();
    pending.verb = verb;
    m_document->undoStack().push(std::move(command));
    pending.history = m_document->undoStack().revision();
    m_pendingAdds.put(std::move(pending));
    m_addRefused = false;
    rebuildFeatureTree();
    if (m_addRefused) return false;
    if (!rebuildRunning()) {
        if (m_document->failedFeatureIndex() >= 0) {
            statusBar()->showMessage(
                tr("%1 added, but an earlier feature fails to rebuild").arg(verb));
        } else {
            m_statusPrompt->setText(tr("%1 added.").arg(verb));
        }
    }
    return true;
}

bool MainWindow::settlePendingAdd(doc::Document& document) {
    const auto taken = m_pendingAdds.take(document);
    if (!taken) return false;  // nothing added to it waits
    const PendingAdd& pending = *taken;
    // Anything done since the add (an undo, another step) settles it: the
    // feature, if it is still there, stays, failing like any other.
    if (document.undoStack().revision() != pending.history) return false;
    const auto index = document.featureTree().indexOf(pending.feature);
    if (!index || document.failedFeatureIndex() != static_cast<int>(*index)) return false;

    // It fails itself (a Cut that would leave nothing, an Intersect of
    // bodies that do not touch): withdrawn, leaving the part, and the undo
    // history, as they were.
    const QString reason = QString::fromStdString(document.lastBuildMessage());
    document.undoStack().withdraw(pending.step);
    m_addRefused = true;
    if (&document == m_document.get()) {
        rebuildFeatureTree();
    } else {
        for (DocTab& tab : m_tabs) {
            if (tab.document.get() == &document) tab.modelStale = true;  // built when shown
        }
    }
    statusBar()->showMessage(tr("%1 not added: %2").arg(pending.verb, reason));
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

    FeatureForm form(this, verb, m_document->lengthUnit());
    auto* size = form.length(QStringLiteral("size"), fillet ? tr("Radius:") : tr("Distance:"), 1.0,
                             0.001, 1e6);
    auto* list = form.checklist(QStringLiteral("edges"), tr("Edges:"), edges.items);
    checkClicked(list, edges, m_viewport->modelSelection(), true);
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

    FeatureForm form(this, verb, m_document->lengthUnit());
    auto* thickness =
        form.length(QStringLiteral("thickness"), tr("Wall thickness:"), 1.0, 0.001, 1e6);
    auto* list = form.checklist(QStringLiteral("faces"), tr("Faces to open:"), faces.items);
    checkClicked(list, faces, m_viewport->modelSelection(), false);
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

    FeatureForm form(this, verb, m_document->lengthUnit());
    auto* pull =
        directionChoice(form, QStringLiteral("pull"), tr("Pull direction:"), QStringLiteral("+Z"));
    auto* neutral = form.length(QStringLiteral("neutral"), tr("Neutral plane at:"), 0.0, -1e6, 1e6);
    auto* angle = form.angle(QStringLiteral("angle"), tr("Angle:"), 3.0, 0.01, 89.0, 2);
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

namespace {

/// The features a pattern can repeat instead of the whole part: those that
/// add or cut material, active in the build.
std::vector<const doc::Feature*> repeatableFeatures(const doc::FeatureTree& tree) {
    std::vector<const doc::Feature*> out;
    const int last = tree.rollbackIndex() >= 0 ? tree.rollbackIndex()
                                               : static_cast<int>(tree.featureCount()) - 1;
    for (int i = 0; i <= last; ++i) {
        const doc::Feature* feature = tree.feature(static_cast<size_t>(i));
        if (feature && feature->createsNewBody() && !feature->isSuppressed())
            out.push_back(feature);
    }
    return out;
}

/// A checklist of @p features for a pattern to repeat (or a mirror to
/// mirror, said by @p label); the ids of those checked are read with
/// checkedTargets().
QListWidget* targetList(FeatureForm& form, const std::vector<const doc::Feature*>& features,
                        const QString& label = MainWindow::tr("Repeat only (none: the whole "
                                                              "part):")) {
    std::vector<std::pair<QString, QString>> items;
    items.reserve(features.size());
    for (const doc::Feature* feature : features) {
        items.emplace_back(QString::fromStdString(feature->name()),
                           QString::fromStdString(feature->featureID()));
    }
    return form.checklist(QStringLiteral("features"), label, items);
}

std::vector<std::string> checkedTargets(const QListWidget* list,
                                        const std::vector<const doc::Feature*>& features) {
    std::vector<std::string> ids;
    for (const int row : FeatureForm::checkedRows(list)) {
        ids.push_back(features[static_cast<size_t>(row)]->featureID());
    }
    return ids;
}

}  // namespace

void MainWindow::onLinearPattern() {
    const QString verb = tr("Linear Pattern");
    if (!requireBody(verb)) return;

    FeatureForm form(this, verb, m_document->lengthUnit());
    auto* direction =
        directionChoice(form, QStringLiteral("direction"), tr("Direction:"), QStringLiteral("+X"));
    auto* spacing = form.length(QStringLiteral("spacing"), tr("Spacing:"), 20.0, 0.001, 1e6);
    auto* count =
        form.count(QStringLiteral("count"), tr("Instances:"), 3, 2, doc::kMaxPatternCount);
    const auto repeatable = repeatableFeatures(m_document->featureTree());
    auto* targets = targetList(form, repeatable);
    if (!form.exec()) return;

    auto pattern = doc::PatternFeature::makeLinear(chosenDirection(direction), spacing->value(),
                                                   count->value());
    pattern->setTargets(checkedTargets(targets, repeatable));
    addModelFeature(std::move(pattern), verb);
}

void MainWindow::onCircularPattern() {
    const QString verb = tr("Circular Pattern");
    if (!requireBody(verb)) return;

    FeatureForm form(this, verb, m_document->lengthUnit());
    auto* axis =
        directionChoice(form, QStringLiteral("axis"), tr("About the axis:"), QStringLiteral("+Z"));
    auto* count =
        form.count(QStringLiteral("count"), tr("Instances:"), 4, 2, doc::kMaxPatternCount);
    auto* total = form.angle(QStringLiteral("angle"), tr("Over:"), 360.0, 1.0, 360.0, 2);
    const auto repeatable = repeatableFeatures(m_document->featureTree());
    auto* targets = targetList(form, repeatable);
    if (!form.exec()) return;

    // A full turn spaces the instances evenly around it; a partial one puts
    // the first and last at its ends.
    const int n = count->value();
    const double degrees = total->value();
    const double step = degrees >= 360.0 ? 360.0 / n : degrees / (n - 1);
    auto pattern = doc::PatternFeature::makeCircular(math::Vec3::Zero, chosenDirection(axis),
                                                     step * std::numbers::pi / 180.0, n);
    pattern->setTargets(checkedTargets(targets, repeatable));
    addModelFeature(std::move(pattern), verb);
}

void MainWindow::onMirror() {
    const QString verb = tr("Mirror");
    if (!requireBody(verb)) return;

    // In a base plane through the origin, or a flat face of the part (the
    // first clicked, at first), followed by its name.
    struct Plane {
        QString text;
        math::Vec3 point;
        math::Vec3 normal;
        std::string face;  ///< its whole name; empty for a base plane
    };
    std::vector<Plane> planes = {
        {tr("YZ plane (x = 0)"), math::Vec3::Zero, math::Vec3::UnitX, {}},
        {tr("ZX plane (y = 0)"), math::Vec3::Zero, math::Vec3::UnitY, {}},
        {tr("XY plane (z = 0)"), math::Vec3::Zero, math::Vec3::UnitZ, {}},
    };
    const auto faces = flatFacesOf(*m_document->solid());
    const int bases = static_cast<int>(planes.size());
    for (const auto& face : faces) {
        planes.push_back({tr("the face %1").arg(face.text), face.middle, face.normal, face.name});
    }
    const int clicked = clickedFlatFace(faces, m_viewport->modelSelection(), -1);
    QStringList names;
    for (const auto& plane : planes) names << plane.text;

    FeatureForm form(this, verb, m_document->lengthUnit());
    auto* which = form.choice(QStringLiteral("plane"), tr("In:"), names);
    which->setCurrentIndex(clicked < 0 ? 0 : bases + clicked);
    const auto mirrorable = repeatableFeatures(m_document->featureTree());
    auto* targets = targetList(form, mirrorable, tr("Mirror only (none: the whole part):"));
    if (!form.exec()) return;

    const Plane& plane = planes.at(static_cast<size_t>(std::max(which->currentIndex(), 0)));
    auto mirror = doc::MirrorFeature::make(plane.point, plane.normal);
    if (!plane.face.empty()) mirror->setReference("planeFace", plane.face);
    mirror->setTargets(checkedTargets(targets, mirrorable));
    addModelFeature(std::move(mirror), verb);
}

void MainWindow::onHole() {
    const QString verb = tr("Hole");
    if (!requireBody(verb)) return;
    const auto faces = flatFacesOf(*m_document->solid());
    if (faces.empty()) {
        statusBar()->showMessage(tr("%1: the part has no flat face to drill into").arg(verb));
        return;
    }
    // Into the face clicked (else the first facing up), at its middle.
    int facingUp = 0;
    for (size_t i = 0; i < faces.size(); ++i) {
        if (faces[i].normal.z > 0.99) {
            facingUp = static_cast<int>(i);
            break;
        }
    }
    const int start = clickedFlatFace(faces, m_viewport->modelSelection(), facingUp);
    QStringList names;
    for (const auto& face : faces) names << face.text;

    FeatureForm form(this, verb, m_document->lengthUnit());
    auto* face = form.choice(QStringLiteral("face"), tr("Into the face:"), names);
    face->setCurrentIndex(start);
    const math::Vec3& middle = faces.at(static_cast<size_t>(start)).middle;
    auto* x = form.length(QStringLiteral("x"), tr("At X:"), middle.x, -1e6, 1e6);
    auto* y = form.length(QStringLiteral("y"), tr("At Y:"), middle.y, -1e6, 1e6);
    auto* z = form.length(QStringLiteral("z"), tr("At Z:"), middle.z, -1e6, 1e6);
    // Another face chosen: at its middle, to be moved from there.
    connect(face, &QComboBox::currentIndexChanged, &form.dialog(), [&faces, x, y, z](int row) {
        if (row < 0 || row >= static_cast<int>(faces.size())) return;
        const math::Vec3& at = faces[static_cast<size_t>(row)].middle;
        x->setValue(at.x);
        y->setValue(at.y);
        z->setValue(at.z);
    });
    auto* type = form.choice(QStringLiteral("type"), tr("Type:"),
                             {tr("Simple"), tr("Counterbore"), tr("Countersink")});
    auto* extent = form.choice(QStringLiteral("extent"), tr("Goes:"),
                               {tr("To the depth"), tr("Through all"), tr("Up to a face")});
    auto* upTo = form.choice(QStringLiteral("upToFace"), tr("Up to the face:"), names);
    auto* diameter = form.length(QStringLiteral("diameter"), tr("Diameter:"), 5.0, 0.001, 1e6);
    auto* depth = form.length(QStringLiteral("depth"), tr("Depth:"), 10.0, 0.001, 1e6);
    auto* point =
        form.angle(QStringLiteral("pointAngle"), tr("Point (0: flat):"), 118.0, 0.0, 179.0);
    auto* boreDiameter =
        form.length(QStringLiteral("boreDiameter"), tr("Counterbore diameter:"), 9.0, 0.001, 1e6);
    auto* boreDepth =
        form.length(QStringLiteral("boreDepth"), tr("Counterbore depth:"), 3.0, 0.001, 1e6);
    auto* sinkDiameter =
        form.length(QStringLiteral("sinkDiameter"), tr("Countersink diameter:"), 10.0, 0.001, 1e6);
    auto* sinkAngle =
        form.angle(QStringLiteral("sinkAngle"), tr("Countersink angle:"), 90.0, 1.0, 179.0);
    const auto offer = [type, extent, upTo, depth, point, boreDiameter, boreDepth, sinkDiameter,
                        sinkAngle] {
        const bool blind = extent->currentIndex() == 0;
        upTo->setEnabled(extent->currentIndex() == 2);
        depth->setEnabled(blind);
        point->setEnabled(blind);
        boreDiameter->setEnabled(type->currentIndex() == 1);
        boreDepth->setEnabled(type->currentIndex() == 1);
        sinkDiameter->setEnabled(type->currentIndex() == 2);
        sinkAngle->setEnabled(type->currentIndex() == 2);
    };
    connect(type, &QComboBox::currentIndexChanged, &form.dialog(), offer);
    connect(extent, &QComboBox::currentIndexChanged, &form.dialog(), offer);
    offer();
    if (!form.exec()) return;

    const FlatFace& into = faces.at(static_cast<size_t>(std::max(face->currentIndex(), 0)));
    auto hole = doc::HoleFeature::make(into.name, math::Vec3(x->value(), y->value(), z->value()),
                                       diameter->value(), depth->value());
    const double toRadians = std::numbers::pi / 180.0;
    const std::map<std::string, double> sizes = {
        {"type", type->currentIndex()},
        {"extent", extent->currentIndex()},
        {"pointAngle", point->value() * toRadians},
        {"boreDiameter", boreDiameter->value()},
        {"boreDepth", boreDepth->value()},
        {"sinkDiameter", sinkDiameter->value()},
        {"sinkAngle", sinkAngle->value() * toRadians},
    };
    for (const auto& [name, value] : sizes) {
        if (!hole->setParameter(name, value)) {
            statusBar()->showMessage(
                tr("%1: the %2 is not one it can take").arg(verb, parameterLabel(name).toLower()));
            return;
        }
    }
    if (extent->currentIndex() == 2) {
        hole->setReference("upToFace",
                           faces.at(static_cast<size_t>(std::max(upTo->currentIndex(), 0))).name);
    }
    addModelFeature(std::move(hole), verb);
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
    const auto vectors = feat->vectors();
    const bool buildsBody = feat->createsNewBody();
    if (params.empty() && vectors.empty() && !buildsBody) {
        statusBar()->showMessage(
            tr("%1 has nothing to edit").arg(QString::fromStdString(feat->name())));
        return;
    }

    FeatureForm form(this, tr("Edit %1").arg(QString::fromStdString(feat->name())),
                     m_document->lengthUnit());
    // Each field shows the stored value as it is — a floor would turn a legal
    // 0 (a pointed cone's radius) into something else before anyone touched
    // it; the feature refuses a value it cannot use. Angles are shown in
    // degrees, counts as whole numbers, a choice as its names. What a field
    // showed is its baseline: a spin box rounds what it is given to its
    // decimals, so an untouched field is one that still shows that, and it
    // keeps the stored value exactly.
    using Kind = doc::Feature::ParameterKind;
    struct Field {
        std::function<double()> read;  ///< in the parameter's own units
        std::function<bool()> touched;
        /// The expression it is given, as kept; empty for a number
        /// (Phase 155). Null for a field that takes none.
        std::function<std::string()> expression;
    };
    std::map<std::string, Field> fields;
    // A length or an angle may be "=expression" of the document's
    // variables, kept with its units written.
    const auto resolverFor = [this](math::QuantityKind kind) -> QuantitySpinBox::Resolver {
        return [this, kind](const std::string& text,
                            std::string* why) -> std::optional<QuantitySpinBox::Worked> {
            const auto parsed = math::Expression::parse(text);
            if (!parsed) {
                if (why != nullptr) *why = "it is not an expression";
                return std::nullopt;
            }
            // As the part is built: its configuration laid over them (156).
            const auto variables = m_document->variables();
            const auto kept =
                math::normalized(*parsed, variables, kind, m_document->lengthUnit(), why);
            if (!kept) return std::nullopt;
            const auto q = math::evaluateQuantity(*kept, variables, why);
            if (!q) return std::nullopt;
            return QuantitySpinBox::Worked{
                kind == math::QuantityKind::Angle ? q->value * math::kRadToDeg : q->value,
                kept->toString()};
        };
    };
    const auto expressionOf = [feat](const std::string& name) {
        const auto found = feat->parameterExpressions().find(name);
        return found != feat->parameterExpressions().end() ? found->second : std::string();
    };
    for (const auto& [name, value] : params) {
        const QString key = QString::fromStdString(name);
        switch (feat->parameterKind(name)) {
            case Kind::Angle: {
                auto* spin = form.angle(key, parameterLabel(name) + QStringLiteral(":"),
                                        value * math::kRadToDeg, -1e6, 1e6, 3);
                spin->setResolver(resolverFor(math::QuantityKind::Angle));
                if (const std::string kept = expressionOf(name); !kept.empty()) {
                    spin->setExpression(kept, value * math::kRadToDeg);
                }
                fields[name] = {[spin] { return spin->value() * math::kDegToRad; },
                                [spin, shown = spin->value()] { return spin->value() != shown; },
                                [spin] { return spin->expression(); }};
                break;
            }
            case Kind::Count: {
                auto* count = form.count(key, parameterLabel(name) + QStringLiteral(":"),
                                         static_cast<int>(std::lround(value)), 0, 1000000);
                fields[name] = {[count] { return static_cast<double>(count->value()); },
                                [count, shown = count->value()] { return count->value() != shown; },
                                nullptr};
                break;
            }
            case Kind::Choice: {
                QStringList names;
                for (const auto& choiceName : feat->parameterChoices(name)) {
                    names << tr(choiceName.c_str());
                }
                auto* choice = form.choice(key, parameterLabel(name) + QStringLiteral(":"), names);
                choice->setCurrentIndex(
                    std::clamp(static_cast<int>(std::lround(value)), 0,
                               std::max(0, static_cast<int>(names.size()) - 1)));
                fields[name] = {[choice] { return static_cast<double>(choice->currentIndex()); },
                                [choice, shown = choice->currentIndex()] {
                                    return choice->currentIndex() != shown;
                                },
                                nullptr};
                break;
            }
            case Kind::Length: {
                auto* spin = form.length(key, parameterLabel(name) + QStringLiteral(":"), value,
                                         -1e9, 1e9, 4);
                spin->setResolver(resolverFor(math::QuantityKind::Length));
                if (const std::string kept = expressionOf(name); !kept.empty()) {
                    spin->setExpression(kept, value);
                }
                fields[name] = {[spin] { return spin->value(); },
                                [spin, shown = spin->value()] { return spin->value() != shown; },
                                [spin] { return spin->expression(); }};
                break;
            }
        }
    }

    // Directions: kept, one of the six axis directions, or the way the face
    // or edge clicked on the part points. Points: their coordinates.
    const std::optional<math::Vec3> clicked =
        m_document->solid() ? clickedDirection(*m_document->solid(), m_viewport->modelSelection())
                            : std::nullopt;
    const std::vector<std::pair<QString, math::Vec3>> axes = {
        {tr("+X"), math::Vec3::UnitX}, {tr("-X"), math::Vec3::UnitX * -1.0},
        {tr("+Y"), math::Vec3::UnitY}, {tr("-Y"), math::Vec3::UnitY * -1.0},
        {tr("+Z"), math::Vec3::UnitZ}, {tr("-Z"), math::Vec3::UnitZ * -1.0}};
    std::map<std::string, std::function<std::optional<math::Vec3>()>> vectorFields;
    for (const auto& [name, value] : vectors) {
        const QString key = QString::fromStdString(name);
        if (doc::Feature::isPoint(name)) {
            auto* x = form.length(key + QStringLiteral("X"), parameterLabel(name) + tr(" x:"),
                                  value.x, -1e9, 1e9, 4);
            auto* y = form.length(key + QStringLiteral("Y"), tr("y:"), value.y, -1e9, 1e9, 4);
            auto* z = form.length(key + QStringLiteral("Z"), tr("z:"), value.z, -1e9, 1e9, 4);
            vectorFields[name] = [x, y, z, sx = x->value(), sy = y->value(),
                                  sz = z->value()]() -> std::optional<math::Vec3> {
                if (x->value() == sx && y->value() == sy && z->value() == sz) return std::nullopt;
                return math::Vec3(x->value(), y->value(), z->value());
            };
            continue;
        }
        QStringList names{tr("As it is, %1").arg(formatPoint(value))};
        std::vector<math::Vec3> directions{value};
        for (const auto& [label, axis] : axes) {
            names << label;
            directions.push_back(axis);
        }
        if (clicked) {
            names << tr("As the face or edge clicked, %1").arg(formatPoint(*clicked));
            directions.push_back(*clicked);
            names << tr("Against the face or edge clicked");
            directions.push_back(*clicked * -1.0);
        }
        auto* choice = form.choice(key, parameterLabel(name) + QStringLiteral(":"), names);
        vectorFields[name] = [choice, directions]() -> std::optional<math::Vec3> {
            const int index = choice->currentIndex();
            if (index <= 0 || index >= static_cast<int>(directions.size())) return std::nullopt;
            return directions[static_cast<size_t>(index)];
        };
    }
    // What it refers to by name (Phase 157): a face of the part, kept as it
    // is or another of its flat faces.
    const auto references = feat->references();
    std::map<std::string, std::function<std::optional<std::string>()>> referenceFields;
    if (!references.empty()) {
        const std::vector<PlaneChoice> flat =
            m_document->solid() ? planarFacesOf(*m_document->solid()) : std::vector<PlaneChoice>{};
        for (const auto& [name, value] : references) {
            QStringList names{
                value.empty() ? tr("None") : tr("As it is, %1").arg(QString::fromStdString(value))};
            std::vector<std::string> kept{value};
            for (const auto& face : flat) {
                names << face.text;
                kept.push_back(model::wholeFaceName(face.tag));
            }
            auto* choice = form.choice(QString::fromStdString(name),
                                       parameterLabel(name) + QStringLiteral(":"), names);
            referenceFields[name] = [choice, kept]() -> std::optional<std::string> {
                const int index = choice->currentIndex();
                if (index <= 0 || index >= static_cast<int>(kept.size())) return std::nullopt;
                return kept[static_cast<size_t>(index)];
            };
        }
    }
    QComboBox* result = buildsBody ? form.operationChoice(feat->operation()) : nullptr;
    if (!form.exec()) return;

    std::map<std::string, double> changed;
    std::map<std::string, std::string> changedExpressions;  // empty: a number again
    for (const auto& [name, field] : fields) {
        if (field.touched()) changed[name] = field.read();
        if (field.expression && field.expression() != expressionOf(name)) {
            changedExpressions[name] = field.expression();
        }
    }
    std::map<std::string, math::Vec3> changedVectors;
    for (const auto& [name, read] : vectorFields) {
        if (const auto value = read()) changedVectors[name] = *value;
    }
    std::map<std::string, std::string> changedReferences;
    for (const auto& [name, read] : referenceFields) {
        if (const auto value = read(); value && *value != references.at(name)) {
            changedReferences[name] = *value;
        }
    }
    // Leave out what the feature refuses (a zero distance, too few segments,
    // an extrusion along its own sketch), and say so.
    QStringList refused;
    if (doc::Feature* target =
            m_document->featureTree().feature(static_cast<size_t>(featureIndex))) {
        for (const std::string& name : doc::refusedParameters(*target, changed)) {
            refused << parameterLabel(name);
            changed.erase(name);
        }
        for (const std::string& name : doc::refusedVectors(*target, changedVectors)) {
            refused << parameterLabel(name);
            changedVectors.erase(name);
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
    if (changed.empty() && changedVectors.empty() && !operation && changedExpressions.empty() &&
        changedReferences.empty()) {
        return;
    }

    m_document->undoStack().push(std::make_unique<doc::EditFeatureCommand>(
        *m_document, feat, std::move(changed), operation, std::move(changedVectors),
        std::move(changedExpressions), std::move(changedReferences)));
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
    if (newIndex == m_document->featureTree().rollbackIndex()) return;
    m_document->undoStack().push(std::make_unique<doc::SetRollbackCommand>(*m_document, newIndex));
    rebuildFeatureTree();
}

void MainWindow::rebuildFeatureTree() {
    DocTab* tab = activeTab();
    const bool onWorker =
        tab != nullptr && !m_assembly &&
        (m_rebuildMode == RebuildMode::Always ||
         (m_rebuildMode == RebuildMode::Auto &&
          (tab->lastBuildMs >= kWorkerRebuildMs || tab->lastBuildMs == kBuildTimeUnknown)));
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
    // A feature just added that fails itself is withdrawn, and the part as
    // it was is built and shown instead.
    if (settlePendingAdd(*m_document)) return;
    m_featureTreePanel->clearFailures();
    m_featureTreePanel->refresh(m_document->featureTree());
    m_featureTreePanel->setConfigurations(m_document->configurations().configurationNames(),
                                          m_document->configurations().active());
    refreshSketchList();

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
    // Expressions worked out here, on the document itself (Phase 155): the
    // worker works them out on its copy, which goes when it is done, and
    // the parameters kept (and saved) are these.
    m_document->applyExpressions();
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

// ---------------------------------------------------------------------------
// WorkbenchHost: what the workbenches ask of the window (Phase 146)
// ---------------------------------------------------------------------------

bool MainWindow::backgroundWorkRunning() const {
    return m_rebuildJob != nullptr || m_importTask != nullptr || m_assemblies->busy() ||
           m_openTask != nullptr || m_massTask != nullptr || m_reloadTask != nullptr;
}

std::vector<std::shared_ptr<doc::AssemblyDocument>> MainWindow::openAssemblies() {
    std::vector<std::shared_ptr<doc::AssemblyDocument>> assemblies;
    for (const DocTab& tab : m_tabs) {
        if (tab.assembly) assemblies.push_back(tab.assembly);
    }
    return assemblies;
}

void MainWindow::showStatus(const QString& message, int timeoutMs) {
    statusBar()->showMessage(message, timeoutMs);
}

QString MainWindow::currentStatus() {
    return statusBar()->currentMessage();
}

void MainWindow::setPrompt(const QString& text) {
    m_statusPrompt->setText(text);
}

bool MainWindow::onWorker(bool large) {
    return m_rebuildMode == RebuildMode::Always || (m_rebuildMode == RebuildMode::Auto && large);
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
    if (!m_rebuildJob || m_importTask || m_assemblies->busy() || m_openTask || m_massTask ||
        m_reloadTask) {
        return;
    }
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
            } else if (!active) {
                settlePendingAdd(*document);  // its tab builds it when shown
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
