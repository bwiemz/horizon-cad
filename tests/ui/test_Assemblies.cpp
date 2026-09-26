// Placing components (Phase 143): an inserted part lands beside the others,
// moves and turns exactly and undoably with its mates solved again, is
// listed with its mates in the assembly tree, which removes, suppresses and
// renames, and can be clicked at once: two clicked cylinders make a
// concentric mate.
//
// Living assemblies (Phase 144): a part saved here or changed on disk shows
// changed in the assemblies placing it, with their mates solved again; one
// closed unsaved leaves them as its file is; a component's part opens in its
// tab; and the bill of materials lists and exports.

#include <gtest/gtest.h>

#include <QAction>
#include <QApplication>
#include <QComboBox>
#include <QElapsedTimer>
#include <QFile>
#include <QFileInfo>
#include <QKeyEvent>
#include <QLabel>
#include <QPushButton>
#include <QStatusBar>
#include <QTabBar>
#include <QTableWidget>
#include <QTemporaryDir>
#include <QTimer>
#include <QTreeWidget>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <functional>
#include <map>
#include <numbers>
#include <string>

#include "UiTestSupport.h"
#include "horizon/document/AssemblyDocument.h"
#include "horizon/document/Document.h"
#include "horizon/document/DocumentManager.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/document/UndoStack.h"
#include "horizon/fileio/NativeFormat.h"
#include "horizon/fileio/StepFormat.h"
#include "horizon/math/Mat4.h"
#include "horizon/modeling/PrimitiveFactory.h"
#include "horizon/topology/Solid.h"
#include "horizon/ui/AssemblyTreePanel.h"
#include "horizon/ui/MainWindow.h"
#include "horizon/ui/ViewportWidget.h"

using hz::math::Vec3;
using hz::test::DialogResponder;
using hz::test::FilePicker;
using hz::test::FormAnswers;
using hz::test::FormFiller;
using hz::test::ToolDriver;
using hz::ui::MainWindow;

namespace {

void trigger(MainWindow& w, const char* name) {
    auto* action = w.findChild<QAction*>(QString::fromLatin1(name));
    ASSERT_NE(action, nullptr) << name;
    action->trigger();
}

/// A part of one primitive, built and saved at @p path; the tag of its face
/// whose name ends @p face (empty: none wanted).
std::string savePart(const QString& path, std::unique_ptr<hz::doc::Feature> feature,
                     const std::string& face = {}) {
    hz::doc::Document part;
    part.setType(hz::doc::DocumentType::Part);
    part.featureTree().addFeature(std::move(feature));
    EXPECT_TRUE(part.rebuildModel());
    EXPECT_TRUE(hz::io::NativeFormat::save(path.toStdString(), part));
    if (face.empty() || part.solid() == nullptr) return {};
    for (const auto& f : part.solid()->faces()) {
        const std::string& tag = f.topoId.tag();
        if (tag.size() >= face.size() &&
            tag.compare(tag.size() - face.size(), face.size(), face) == 0) {
            return tag;
        }
    }
    ADD_FAILURE() << "no face " << face;
    return {};
}

hz::doc::AssemblyDocument& newAssembly(MainWindow& w) {
    trigger(w, "action_new_assembly");
    EXPECT_NE(w.activeAssembly(), nullptr);
    return *w.activeAssembly();
}

void insert(MainWindow& w, const QString& path) {
    FilePicker picker(path);
    trigger(w, "action_insert_component");
}

Vec3 translationOf(const hz::doc::ComponentInstance& comp) {
    return comp.transform.transformPoint(Vec3());
}

/// Makes @p id the assembly tree's current row, as a click on it does.
void chooseInTree(MainWindow& w, uint64_t id) {
    auto* panel = w.findChild<hz::ui::AssemblyTreePanel*>();
    ASSERT_NE(panel, nullptr);
    QTreeWidget* tree = panel->tree();
    for (int top = 0; top < tree->topLevelItemCount(); ++top) {
        for (int row = 0; row < tree->topLevelItem(top)->childCount(); ++row) {
            QTreeWidgetItem* item = tree->topLevelItem(top)->child(row);
            if (item->data(0, Qt::UserRole).toInt() == 1 &&
                item->data(0, Qt::UserRole + 1).toULongLong() == id) {
                tree->setCurrentItem(item);
                return;
            }
        }
    }
    ADD_FAILURE() << "no row for component " << id;
}

/// How tall a component's mesh is (0 with none).
double heightOf(const hz::doc::ComponentInstance& comp) {
    if (!comp.cachedMesh || comp.cachedMesh->positions.empty()) return 0.0;
    const auto& p = comp.cachedMesh->positions;
    double low = p[2];
    double high = p[2];
    for (size_t i = 2; i < p.size(); i += 3) {
        low = std::min(low, static_cast<double>(p[i]));
        high = std::max(high, static_cast<double>(p[i]));
    }
    return high - low;
}

/// Run the event loop until @p done, or @p ms elapse; whether it was.
bool waitUntil(const std::function<bool()>& done, int ms) {
    QElapsedTimer clock;
    clock.start();
    while (!done() && clock.elapsed() < ms) {
        QCoreApplication::processEvents(QEventLoop::AllEvents, 20);
    }
    return done();
}

QTabBar* tabBar(MainWindow& w) {
    return w.findChild<QTabBar*>(QStringLiteral("documentTabs"));
}

/// Two 10 mm blocks inserted from @p block, the second on the first's top
/// (Coincident, then placed by a move): their ids.
std::pair<uint64_t, uint64_t> stackTwo(MainWindow& w, hz::doc::AssemblyDocument& assembly,
                                       const QString& block, const std::string& top) {
    insert(w, block);
    insert(w, block);
    const uint64_t base = assembly.components()[0].id;
    const uint64_t lid = assembly.components()[1].id;
    hz::doc::Mate on;
    on.type = hz::doc::MateType::Coincident;
    on.a = {base, hz::topo::TopologyID::fromTag(top)};
    on.b = {lid, hz::topo::TopologyID::fromTag(top.substr(0, top.size() - 3) + "bottom")};
    assembly.addMate(on);
    chooseInTree(w, lid);
    FormFiller move(QStringLiteral("Move Component"), FormAnswers().number("dx", 1));
    trigger(w, "action_move_component");
    EXPECT_TRUE(move.seen());
    EXPECT_NEAR(translationOf(*assembly.component(lid)).z, 10.0, 1e-6);
    return {base, lid};
}

/// A drag on screen, as a hand makes it (Phase 158): pressed where @p from
/// shows, moved by way of the middle to where @p to shows, released there.
void dragOnScreen(ToolDriver& drive, const Vec3& from, const Vec3& to) {
    auto& view = drive.viewport();
    const QPointF a = view.projectToScreen(from);
    const QPointF b = view.projectToScreen(to);
    ASSERT_GT((b - a).manhattanLength(), 2 * QApplication::startDragDistance())
        << "far enough to be a drag";
    drive.pressAt(a);
    drive.dragAt((a + b) / 2.0);
    drive.dragAt(b);
    drive.releaseAt(b);
}

/// The view from high above the blocks, looking straight down; or from far
/// in front of them (Phase 158). A preset view keeps the camera's distance,
/// which puts it inside a part this size.
void lookFromAbove(hz::ui::ViewportWidget& view) {
    view.camera().lookAt(Vec3(20, 10, 200), Vec3(20, 10, 0), Vec3(0, 1, 0));
}
void lookFromTheFront(hz::ui::ViewportWidget& view) {
    view.camera().lookAt(Vec3(20, -200, 10), Vec3(20, 0, 10), Vec3(0, 0, 1));
}

void expectAt(const hz::doc::ComponentInstance& comp, const Vec3& at, const char* what) {
    const Vec3 is = translationOf(comp);
    EXPECT_NEAR(is.x, at.x, 1e-6) << what;
    EXPECT_NEAR(is.y, at.y, 1e-6) << what;
    EXPECT_NEAR(is.z, at.z, 1e-6) << what;
}

/// Opens the part @p id places, from the Assembly menu; its document.
hz::doc::Document* openPartOf(MainWindow& w, uint64_t id) {
    chooseInTree(w, id);
    trigger(w, "action_open_part");
    EXPECT_EQ(w.activeAssembly(), nullptr) << "the part's tab is shown";
    return w.activeDocument();
}

/// Makes a box part's block @p depth deep, as an edit in its tab does.
void deepen(hz::doc::Document& part, double depth) {
    ASSERT_GT(part.featureTree().featureCount(), 0u);
    ASSERT_TRUE(part.featureTree().feature(0)->setParameter("depth", depth));
    ASSERT_TRUE(part.rebuildModel());
    part.setDirty(true);
}

}  // namespace

// Two inserts of one part sit side by side: the second clear of the first
// along +X. Both landed at the origin, one inside the other.
TEST(AssembliesTest, InsertsSitSideBySide) {
    QTemporaryDir dir;
    const QString block = dir.filePath(QStringLiteral("block.hzpart"));
    savePart(block, hz::doc::PrimitiveFeature::makeBox(10, 10, 10));
    MainWindow w;
    auto& assembly = newAssembly(w);
    insert(w, block);
    insert(w, block);
    ASSERT_EQ(assembly.components().size(), 2u);
    EXPECT_NEAR(translationOf(assembly.components()[0]).x, 0.0, 1e-12);
    EXPECT_NEAR(translationOf(assembly.components()[1]).x, 11.0, 1e-9) << "10 wide, 1 apart";
}

// Move and Rotate place a component exactly, each one undo step.
TEST(AssembliesTest, MoveAndRotateAreExactAndUndoable) {
    QTemporaryDir dir;
    const QString block = dir.filePath(QStringLiteral("block.hzpart"));
    savePart(block, hz::doc::PrimitiveFeature::makeBox(10, 10, 10));
    MainWindow w;
    auto& assembly = newAssembly(w);
    insert(w, block);
    ASSERT_EQ(assembly.components().size(), 1u);

    {
        FormFiller move(QStringLiteral("Move Component"),
                        FormAnswers().number("dx", 5).number("dy", -2).number("dz", 3));
        trigger(w, "action_move_component");
        ASSERT_TRUE(move.seen());
    }
    const Vec3 moved = translationOf(assembly.components()[0]);
    EXPECT_NEAR((moved - Vec3(5, -2, 3)).length(), 0.0, 1e-12);

    // A quarter turn about Z through its middle, (10, 3, 8) now: its corner
    // at (5, -2, 3) goes to (15, -2, 3).
    {
        FormFiller rotate(QStringLiteral("Rotate Component"),
                          FormAnswers().choose("axis", "Z").number("angle", 90));
        trigger(w, "action_rotate_component");
        ASSERT_TRUE(rotate.seen());
    }
    EXPECT_NEAR((translationOf(assembly.components()[0]) - Vec3(15, -2, 3)).length(), 0.0, 1e-9);

    trigger(w, "action_undo");
    EXPECT_NEAR((translationOf(assembly.components()[0]) - moved).length(), 0.0, 1e-12);
    trigger(w, "action_undo");
    EXPECT_NEAR(translationOf(assembly.components()[0]).length(), 0.0, 1e-12);
}

// A move keeps the mates: a block sitting on another slides along it when
// moved sideways and up, and stays on it. One held by a Fixed mate stays.
TEST(AssembliesTest, AMoveKeepsTheMates) {
    QTemporaryDir dir;
    const QString block = dir.filePath(QStringLiteral("block.hzpart"));
    const std::string top = savePart(block, hz::doc::PrimitiveFeature::makeBox(10, 10, 10), "/top");
    const std::string bottom = top.substr(0, top.size() - 3) + "bottom";
    MainWindow w;
    auto& assembly = newAssembly(w);
    insert(w, block);
    insert(w, block);
    const uint64_t base = assembly.components()[0].id;
    const uint64_t lid = assembly.components()[1].id;
    hz::doc::Mate on;
    on.type = hz::doc::MateType::Coincident;
    on.a = {base, hz::topo::TopologyID::fromTag(top)};
    on.b = {lid, hz::topo::TopologyID::fromTag(bottom)};
    assembly.addMate(on);

    chooseInTree(w, lid);
    {
        FormFiller move(QStringLiteral("Move Component"),
                        FormAnswers().number("dx", 3).number("dz", 7));
        trigger(w, "action_move_component");
        ASSERT_TRUE(move.seen());
    }
    const Vec3 at = translationOf(*assembly.component(lid));
    EXPECT_NEAR(at.x, 14.0, 1e-6) << "slid 3 along the base";
    EXPECT_NEAR(at.z, 10.0, 1e-6) << "and still on its top";

    // Fixed: not moved, and said so.
    hz::doc::Mate fixed;
    fixed.type = hz::doc::MateType::Fixed;
    fixed.a = {lid, hz::topo::TopologyID::fromTag(top)};
    assembly.addMate(fixed);
    chooseInTree(w, lid);
    {
        FormFiller move(QStringLiteral("Move Component"), FormAnswers().number("dx", 5));
        trigger(w, "action_move_component");
    }
    EXPECT_NEAR((translationOf(*assembly.component(lid)) - at).length(), 0.0, 1e-12);
    EXPECT_TRUE(w.statusBar()->currentMessage().contains(QStringLiteral("Fixed")));
}

// The tree lists the components and mates; from it a component is renamed,
// suppressed and removed, its mates with it, and a mate's value edited.
TEST(AssembliesTest, TheTreeListsAndEdits) {
    QTemporaryDir dir;
    const QString block = dir.filePath(QStringLiteral("block.hzpart"));
    const std::string top = savePart(block, hz::doc::PrimitiveFeature::makeBox(10, 10, 10), "/top");
    const std::string bottom = top.substr(0, top.size() - 3) + "bottom";
    MainWindow w;
    auto& assembly = newAssembly(w);
    insert(w, block);
    insert(w, block);
    const uint64_t base = assembly.components()[0].id;
    const uint64_t lid = assembly.components()[1].id;
    hz::doc::Mate gap;
    gap.type = hz::doc::MateType::Distance;
    gap.a = {base, hz::topo::TopologyID::fromTag(top)};
    gap.b = {lid, hz::topo::TopologyID::fromTag(bottom)};
    gap.value = 2.0;
    assembly.addMate(gap);

    // Renamed: the tree says so.
    chooseInTree(w, lid);
    {
        FormFiller rename(QStringLiteral("Rename Component"), FormAnswers().text("name", "lid"));
        trigger(w, "action_rename_component");
        ASSERT_TRUE(rename.seen());
    }
    EXPECT_EQ(assembly.component(lid)->name, "lid");
    auto* panel = w.findChild<hz::ui::AssemblyTreePanel*>();
    EXPECT_FALSE(panel->tree()
                     ->findItems(QStringLiteral("lid"), Qt::MatchExactly | Qt::MatchRecursive)
                     .isEmpty());

    // The mate's value: the lid 5 above the base, not 2.
    {
        FormFiller choose(QStringLiteral("Choose Mate"), FormAnswers());
        FormFiller edit(QStringLiteral("Edit Mate"), FormAnswers().number("value", 5));
        trigger(w, "action_edit_mate");
        ASSERT_TRUE(edit.seen());
    }
    EXPECT_NEAR(translationOf(*assembly.component(lid)).z, 15.0, 1e-6);

    // Suppressed, and back.
    chooseInTree(w, lid);
    trigger(w, "action_suppress_component");
    EXPECT_TRUE(assembly.component(lid)->suppressed);
    chooseInTree(w, lid);
    trigger(w, "action_suppress_component");
    EXPECT_FALSE(assembly.component(lid)->suppressed);

    // Removed, with its mate; undone, both back.
    chooseInTree(w, base);
    trigger(w, "action_remove_component");
    EXPECT_EQ(assembly.components().size(), 1u);
    EXPECT_TRUE(assembly.mates().empty());
    trigger(w, "action_undo");
    EXPECT_EQ(assembly.components().size(), 2u);
    EXPECT_EQ(assembly.mates().size(), 1u);
}

// A component can be clicked as soon as it is inserted (its part file's mesh
// names its faces), and the faces clicked are the mate's: two cylinders,
// clicked on their sides, made concentric. The dialog listed each facet and
// took the first for a curved face.
TEST(AssembliesTest, ClickedCylindersBecomeConcentric) {
    QTemporaryDir dir;
    const QString pin = dir.filePath(QStringLiteral("pin.hzpart"));
    savePart(pin, hz::doc::PrimitiveFeature::makeCylinder(3.0, 10.0));
    MainWindow w;
    auto& assembly = newAssembly(w);
    insert(w, pin);
    insert(w, pin);
    ASSERT_EQ(assembly.components().size(), 2u);
    const double shift = translationOf(assembly.components()[1]).x;
    ASSERT_GT(shift, 6.0) << "beside the first";

    ToolDriver drive(w);
    auto& view = drive.viewport();
    view.camera().lookAt(Vec3(shift / 2, -60, 5), Vec3(shift / 2, 0, 5), Vec3(0, 0, 1));
    trigger(w, "tool_select");
    // The middle of the facet facing the camera: between the rim's vertices
    // at 270 and 281.25 degrees.
    const double a0 = 1.5 * std::numbers::pi;
    const double a1 = a0 + 2.0 * std::numbers::pi / 32.0;
    const Vec3 onFacet(1.5 * (std::cos(a0) + std::cos(a1)), 1.5 * (std::sin(a0) + std::sin(a1)), 5);
    drive.clickAt(view.projectToScreen(onFacet));
    drive.clickAt(view.projectToScreen(onFacet + Vec3(shift, 0, 0)), Qt::ShiftModifier);
    ASSERT_EQ(view.modelSelection().size(), 2u) << "both sides clicked";

    {
        FormFiller mate(QStringLiteral("Add Mate"), FormAnswers().choose("type", "Concentric"));
        trigger(w, "action_add_mate");
        ASSERT_TRUE(mate.seen());
    }
    ASSERT_EQ(assembly.mates().size(), 1u) << w.statusBar()->currentMessage().toStdString();
    const auto& m = assembly.mates().front();
    const auto endsWith = [](const std::string& s, const std::string& end) {
        return s.size() >= end.size() && s.compare(s.size() - end.size(), end.size(), end) == 0;
    };
    EXPECT_TRUE(endsWith(m.a.faceId.tag(), "/side")) << m.a.faceId.tag();
    EXPECT_TRUE(endsWith(m.b.faceId.tag(), "/side")) << m.b.faceId.tag();
    const Vec3 second = translationOf(assembly.components()[1]);
    EXPECT_NEAR(std::hypot(second.x, second.y), 0.0, 1e-6) << "on the first's axis";
}

// A part saved in its tab shows saved in the assembly: both blocks taller,
// and the one on top moved up with the other's top (the mate solved again).
TEST(AssembliesTest, SavingAPartChangesTheAssembly) {
    QTemporaryDir dir;
    const QString block = dir.filePath(QStringLiteral("block.hzpart"));
    const std::string top = savePart(block, hz::doc::PrimitiveFeature::makeBox(10, 10, 10), "/top");
    MainWindow w;
    auto& assembly = newAssembly(w);
    const int assemblyTab = tabBar(w)->currentIndex();
    const auto [base, lid] = stackTwo(w, assembly, block, top);

    hz::doc::Document* part = openPartOf(w, base);
    ASSERT_NE(part, nullptr);
    deepen(*part, 20);
    trigger(w, "action_save");
    EXPECT_FALSE(part->isDirty());

    tabBar(w)->setCurrentIndex(assemblyTab);
    ASSERT_EQ(w.activeAssembly(), &assembly);
    EXPECT_NEAR(heightOf(*assembly.component(base)), 20.0, 1e-9);
    EXPECT_NEAR(heightOf(*assembly.component(lid)), 20.0, 1e-9);
    EXPECT_NEAR(translationOf(*assembly.component(lid)).z, 20.0, 1e-6) << "on the taller top";
    EXPECT_TRUE(assembly.isDirty()) << "moved, to be saved";
}

// A part changed on disk by another program is picked up by the watch.
TEST(AssembliesTest, APartChangedOnDiskIsPickedUp) {
    QTemporaryDir dir;
    const QString block = dir.filePath(QStringLiteral("block.hzpart"));
    savePart(block, hz::doc::PrimitiveFeature::makeBox(10, 10, 10));
    MainWindow w;
    auto& assembly = newAssembly(w);
    insert(w, block);
    ASSERT_EQ(assembly.components().size(), 1u);
    EXPECT_NEAR(heightOf(assembly.components()[0]), 10.0, 1e-9);

    auto* watch = w.findChild<QTimer*>(QStringLiteral("partWatchTimer"));
    ASSERT_NE(watch, nullptr);
    EXPECT_TRUE(watch->isActive());
    watch->setInterval(10);

    // Another program's save; its time moved on, however coarse the clock.
    savePart(block, hz::doc::PrimitiveFeature::makeBox(10, 10, 25));
    const std::filesystem::path file(block.toStdString());
    std::filesystem::last_write_time(
        file, std::filesystem::last_write_time(file) + std::chrono::seconds(2));

    EXPECT_TRUE(waitUntil([&] { return heightOf(assembly.components()[0]) > 20.0; }, 5000));
    EXPECT_NEAR(heightOf(assembly.components()[0]), 25.0, 1e-9);
}

/// Edits @p block as another program would: its box made @p depth deep,
/// its features (and so its faces' names) kept, and its time moved on
/// however coarse the clock.
void rewriteOnDisk(const QString& block, double depth) {
    hz::doc::Document part;
    ASSERT_TRUE(hz::io::NativeFormat::load(block.toStdString(), part));
    ASSERT_TRUE(part.featureTree().feature(0)->setParameter("depth", depth));
    ASSERT_TRUE(part.rebuildModel());
    ASSERT_TRUE(hz::io::NativeFormat::save(block.toStdString(), part));
    const std::filesystem::path file(block.toStdString());
    std::filesystem::last_write_time(
        file, std::filesystem::last_write_time(file) + std::chrono::seconds(2));
}

double depthOf(const hz::doc::Document& part) {
    if (part.featureTree().featureCount() == 0) return 0.0;
    return part.featureTree().feature(0)->parameters().at("depth");
}

// A part open in its tab and changed on disk is read again there, and the
// assembly follows. The tab's document was kept as the part, and the mates
// were solved on it, as it was before the change.
TEST(AssembliesTest, APartOpenInATabAndChangedOnDiskIsReadAgain) {
    QTemporaryDir dir;
    const QString block = dir.filePath(QStringLiteral("block.hzpart"));
    const std::string top = savePart(block, hz::doc::PrimitiveFeature::makeBox(10, 10, 10), "/top");
    MainWindow w;
    auto& assembly = newAssembly(w);
    const int assemblyTab = tabBar(w)->currentIndex();
    const auto [base, lid] = stackTwo(w, assembly, block, top);
    const hz::doc::Document* part = openPartOf(w, base);
    ASSERT_NE(part, nullptr);
    w.findChild<QTimer*>(QStringLiteral("partWatchTimer"))->setInterval(10);

    rewriteOnDisk(block, 25);
    ASSERT_TRUE(waitUntil([&] { return w.activeDocument() != part; }, 5000)) << "read again";
    EXPECT_NEAR(depthOf(*w.activeDocument()), 25.0, 1e-12);
    EXPECT_FALSE(w.activeDocument()->isDirty());

    tabBar(w)->setCurrentIndex(assemblyTab);
    EXPECT_NEAR(heightOf(*assembly.component(base)), 25.0, 1e-9);
    EXPECT_NEAR(translationOf(*assembly.component(lid)).z, 25.0, 1e-6) << "on the new top";
}

// A part changed on disk while it has unsaved changes here asks first:
// kept, the tab and the assembly stay with the user's version; read again,
// they take the file's.
TEST(AssembliesTest, APartWithUnsavedChangesAsksBeforeReadingAgain) {
    QTemporaryDir dir;
    const QString block = dir.filePath(QStringLiteral("block.hzpart"));
    const std::string top = savePart(block, hz::doc::PrimitiveFeature::makeBox(10, 10, 10), "/top");
    MainWindow w;
    auto& assembly = newAssembly(w);
    const auto [base, lid] = stackTwo(w, assembly, block, top);
    hz::doc::Document* part = openPartOf(w, base);
    ASSERT_NE(part, nullptr);
    deepen(*part, 20);
    w.findChild<QTimer*>(QStringLiteral("partWatchTimer"))->setInterval(10);

    {
        DialogResponder keep(QMessageBox::Ignore, QStringLiteral("File Changed on Disk"));
        rewriteOnDisk(block, 25);
        keep.waitForDialog(5000);
        ASSERT_TRUE(keep.seen());
        EXPECT_EQ(keep.defaultButton(), QMessageBox::Ignore) << "keeping is the default";
    }
    ASSERT_EQ(w.activeDocument(), part) << "kept";
    EXPECT_TRUE(part->isDirty());
    EXPECT_NEAR(translationOf(*assembly.component(lid)).z, 20.0, 1e-6) << "on the user's version";

    {
        DialogResponder readAgain(QMessageBox::Discard, QStringLiteral("File Changed on Disk"));
        rewriteOnDisk(block, 30);
        readAgain.waitForDialog(5000);
        ASSERT_TRUE(readAgain.seen());
    }
    ASSERT_TRUE(waitUntil([&] { return w.activeDocument() != part; }, 5000));
    EXPECT_NEAR(depthOf(*w.activeDocument()), 30.0, 1e-12);
    EXPECT_NEAR(translationOf(*assembly.component(lid)).z, 30.0, 1e-6) << "on the file's";
}

// A file read again on a worker, as a large one is (every file, here),
// swaps its tab when the reading is done, and the assembly follows; one
// edited while it was read keeps the edits. It was read on the GUI thread,
// freezing the window each time another program saved a large part.
TEST(AssembliesTest, APartIsReadAgainOnAWorker) {
    QTemporaryDir dir;
    const QString block = dir.filePath(QStringLiteral("block.hzpart"));
    const std::string top = savePart(block, hz::doc::PrimitiveFeature::makeBox(10, 10, 10), "/top");
    MainWindow w;
    auto& assembly = newAssembly(w);
    const int assemblyTab = tabBar(w)->currentIndex();
    const auto [base, lid] = stackTwo(w, assembly, block, top);
    hz::doc::Document* part = openPartOf(w, base);
    ASSERT_NE(part, nullptr);
    ASSERT_TRUE(waitUntil([&] { return !w.backgroundWorkRunning(); }, 10000)) << "opened, built";
    w.setRebuildMode(MainWindow::RebuildMode::Always);
    auto* prompt = w.findChild<QLabel*>(QStringLiteral("statusPrompt"));
    ASSERT_NE(prompt, nullptr);
    const auto reading = [&] { return prompt->text().contains(QStringLiteral("again...")); };
    // The watch looked now, not on its timer: a small file is read so fast
    // that its reading could start and end within one turn of the events,
    // and a wait for it to be under way never saw it.
    auto* watch = w.findChild<QTimer*>(QStringLiteral("partWatchTimer"));
    ASSERT_NE(watch, nullptr);
    watch->stop();
    const auto look = [watch] {
        return QMetaObject::invokeMethod(watch, "timeout", Qt::DirectConnection);
    };

    // Read on a worker: the tab's document is the old one until it is done.
    rewriteOnDisk(block, 25);
    ASSERT_TRUE(look());
    ASSERT_TRUE(reading()) << "under way; its end is not delivered until the events turn";
    EXPECT_EQ(w.activeDocument(), part);
    ASSERT_TRUE(waitUntil([&] { return w.activeDocument() != part; }, 5000));
    EXPECT_NEAR(depthOf(*w.activeDocument()), 25.0, 1e-12);
    ASSERT_TRUE(waitUntil([&] { return !w.backgroundWorkRunning(); }, 10000));
    tabBar(w)->setCurrentIndex(assemblyTab);
    EXPECT_NEAR(translationOf(*assembly.component(lid)).z, 25.0, 1e-6);

    // Edited while it is read: the edits are kept, and the assembly shows them.
    hz::doc::Document* fresh = openPartOf(w, base);
    ASSERT_NE(fresh, nullptr);
    ASSERT_TRUE(waitUntil([&] { return !w.backgroundWorkRunning(); }, 10000));
    rewriteOnDisk(block, 30);
    ASSERT_TRUE(look());
    ASSERT_TRUE(reading());
    deepen(*fresh, 40);
    ASSERT_TRUE(waitUntil([&] { return !reading(); }, 5000));
    ASSERT_EQ(w.activeDocument(), fresh) << "kept";
    EXPECT_NEAR(depthOf(*fresh), 40.0, 1e-12);
    EXPECT_TRUE(w.statusBar()->currentMessage().contains(QStringLiteral("kept")))
        << w.statusBar()->currentMessage().toStdString();
}

// A part edited in its tab and closed unsaved leaves the assembly as its
// file is: the components shared its document, and the mates were solved on
// its edits after it was gone.
TEST(AssembliesTest, APartClosedUnsavedLeavesTheAssemblyAsSaved) {
    QTemporaryDir dir;
    const QString block = dir.filePath(QStringLiteral("block.hzpart"));
    const std::string top = savePart(block, hz::doc::PrimitiveFeature::makeBox(10, 10, 10), "/top");
    MainWindow w;
    auto& assembly = newAssembly(w);
    const auto [base, lid] = stackTwo(w, assembly, block, top);

    hz::doc::Document* part = openPartOf(w, base);
    ASSERT_NE(part, nullptr);
    deepen(*part, 20);
    {
        DialogResponder discard(QMessageBox::Discard);
        emit tabBar(w)->tabCloseRequested(tabBar(w)->currentIndex());
        EXPECT_TRUE(discard.seen());
    }
    ASSERT_EQ(w.activeAssembly(), &assembly);

    chooseInTree(w, lid);
    {
        FormFiller move(QStringLiteral("Move Component"), FormAnswers().number("dx", 1));
        trigger(w, "action_move_component");
        ASSERT_TRUE(move.seen());
    }
    EXPECT_NEAR(translationOf(*assembly.component(lid)).z, 10.0, 1e-6) << "the file's top";
    EXPECT_NEAR(heightOf(*assembly.component(base)), 10.0, 1e-9);
}

// A component's part opens in its tab, from the menu or the tree; a second
// time, the tab it has is shown.
TEST(AssembliesTest, AComponentsPartOpensInItsTab) {
    QTemporaryDir dir;
    const QString block = dir.filePath(QStringLiteral("block.hzpart"));
    savePart(block, hz::doc::PrimitiveFeature::makeBox(10, 10, 10));
    MainWindow w;
    auto& assembly = newAssembly(w);
    insert(w, block);
    const uint64_t id = assembly.components()[0].id;
    const int assemblyTab = tabBar(w)->currentIndex();
    const int tabs = tabBar(w)->count();

    const hz::doc::Document* part = openPartOf(w, id);
    ASSERT_NE(part, nullptr);
    EXPECT_EQ(tabBar(w)->count(), tabs + 1);
    EXPECT_TRUE(hz::doc::DocumentManager::samePath(part->filePath(), block.toStdString()));
    EXPECT_GT(part->featureTree().featureCount(), 0u) << "the part, features and all";

    tabBar(w)->setCurrentIndex(assemblyTab);
    chooseInTree(w, id);
    auto* panel = w.findChild<hz::ui::AssemblyTreePanel*>();
    QAction* open = nullptr;
    for (QAction* action : panel->tree()->actions()) {
        if (action->text() == QStringLiteral("Open Part")) open = action;
    }
    ASSERT_NE(open, nullptr);
    ASSERT_TRUE(open->isEnabled());
    open->trigger();
    EXPECT_EQ(tabBar(w)->count(), tabs + 1) << "its tab again, not another";
    EXPECT_EQ(w.activeDocument(), part);
}

// The bill of materials lists each part once with its count, and exports.
TEST(AssembliesTest, TheBillOfMaterialsListsAndExports) {
    QTemporaryDir dir;
    const QString block = dir.filePath(QStringLiteral("block.hzpart"));
    const QString pin = dir.filePath(QStringLiteral("pin.hzpart"));
    savePart(block, hz::doc::PrimitiveFeature::makeBox(10, 10, 10));
    savePart(pin, hz::doc::PrimitiveFeature::makeCylinder(3.0, 10.0));
    MainWindow w;
    newAssembly(w);
    insert(w, block);
    insert(w, pin);
    insert(w, block);

    const QString csv = dir.filePath(QStringLiteral("bom.csv"));
    QStringList rows;
    bool seen = false;
    QTimer poll;
    QElapsedTimer clock;
    clock.start();
    FilePicker picker(csv);
    QObject::connect(&poll, &QTimer::timeout, [&] {
        auto* dialog = qobject_cast<QDialog*>(QApplication::activeModalWidget());
        if (dialog == nullptr || dialog->windowTitle() != QStringLiteral("Bill of Materials")) {
            if (clock.elapsed() > 10'000) poll.stop();
            return;
        }
        poll.stop();
        seen = true;
        auto* table = dialog->findChild<QTableWidget*>(QStringLiteral("bom"));
        for (int row = 0; table != nullptr && row < table->rowCount(); ++row) {
            rows << table->item(row, 1)->text() + " x" + table->item(row, 2)->text();
        }
        if (auto* exportButton = dialog->findChild<QPushButton*>(QStringLiteral("exportBom"))) {
            exportButton->click();
        }
        dialog->reject();
    });
    poll.start(5);
    trigger(w, "action_bill_of_materials");
    ASSERT_TRUE(seen);
    EXPECT_EQ(rows, QStringList({QStringLiteral("block x2"), QStringLiteral("pin x1")}));

    QFile file(csv);
    ASSERT_TRUE(file.open(QIODevice::ReadOnly));
    const QString text = QString::fromUtf8(file.readAll());
    EXPECT_TRUE(text.startsWith(QStringLiteral("Item,Part,Quantity,Path"))) << text.toStdString();
    EXPECT_TRUE(text.contains(QStringLiteral("1,block,2,"))) << text.toStdString();
    EXPECT_TRUE(text.contains(QStringLiteral("2,pin,1,"))) << text.toStdString();
}

// The tree's current row is not carried from one assembly to another: both
// number their components from 1, and the other's #1 was made current, for
// Delete to remove.
TEST(AssembliesTest, TheTreeKeepsNoChoiceFromAnotherAssembly) {
    QTemporaryDir dir;
    const QString block = dir.filePath(QStringLiteral("block.hzpart"));
    savePart(block, hz::doc::PrimitiveFeature::makeBox(10, 10, 10));
    MainWindow w;
    auto* tabs = w.findChild<QTabBar*>(QStringLiteral("documentTabs"));
    ASSERT_NE(tabs, nullptr);
    auto& first = newAssembly(w);
    const int firstTab = tabs->currentIndex();
    insert(w, block);
    auto& second = newAssembly(w);
    const int secondTab = tabs->currentIndex();
    insert(w, block);
    ASSERT_EQ(first.components().front().id, second.components().front().id);

    tabs->setCurrentIndex(firstTab);
    chooseInTree(w, first.components().front().id);
    tabs->setCurrentIndex(secondTab);
    auto* panel = w.findChild<hz::ui::AssemblyTreePanel*>();
    EXPECT_EQ(panel->currentComponent(), 0u) << "nothing chosen in this assembly";

    QKeyEvent del(QEvent::KeyPress, Qt::Key_Delete, Qt::NoModifier);
    QApplication::sendEvent(panel->tree(), &del);
    EXPECT_EQ(second.components().size(), 1u);
}

// An assembly goes out as a STEP assembly (Phase 153): each part once, and
// each component a use of it, by its name, where it is placed.
TEST(AssembliesTest, AnAssemblyExportsAsAStepAssembly) {
    QTemporaryDir dir;
    const QString block = dir.filePath(QStringLiteral("block.hzpart"));
    const QString pin = dir.filePath(QStringLiteral("pin.hzpart"));
    savePart(block, hz::doc::PrimitiveFeature::makeBox(10, 10, 10));
    savePart(pin, hz::doc::PrimitiveFeature::makeCylinder(3.0, 10.0));
    MainWindow w;
    auto& assembly = newAssembly(w);
    insert(w, block);
    insert(w, block);
    insert(w, pin);
    ASSERT_EQ(assembly.components().size(), 3u);

    const QString step = dir.filePath(QStringLiteral("rig.step"));
    DialogResponder told(QMessageBox::Ok, QStringLiteral("Export STEP"), 1000);
    {
        FilePicker picker(step);
        trigger(w, "export_step");
    }
    EXPECT_FALSE(told.seen()) << "everything written as designed: nothing to say";
    const auto read = hz::io::StepFormat::loadAssembly(step.toStdString());
    ASSERT_EQ(read.parts.size(), 2u) << hz::io::StepFormat::lastError();
    EXPECT_EQ(read.parts[0].name, "block");
    EXPECT_EQ(read.parts[1].name, "pin");
    ASSERT_EQ(read.occurrences.size(), 3u);
    const std::size_t partOf[] = {0, 0, 1};
    for (std::size_t k = 0; k < 3; ++k) {
        const auto& component = assembly.components()[k];
        EXPECT_EQ(read.occurrences[k].part, partOf[k]);
        EXPECT_EQ(read.occurrences[k].name, component.name);
        const Vec3 at = read.occurrences[k].transform.transformPoint(Vec3());
        EXPECT_NEAR(at.x, translationOf(component).x, 1e-9) << component.name;
        EXPECT_NEAR(at.y, translationOf(component).y, 1e-9) << component.name;
    }
}

// A STEP assembly kept as an assembly (Phase 153): its parts as part files,
// in a folder beside it, and it opened, placing them as the file did.
TEST(AssembliesTest, ImportingAStepAssemblyKeepsItsPartsAsFiles) {
    QTemporaryDir dir;
    const auto bracket = hz::model::PrimitiveFactory::makeBox(10, 20, 30);
    const auto pin = hz::model::PrimitiveFactory::makeCylinder(5, 10);
    const QString step = dir.filePath(QStringLiteral("rig.step"));
    ASSERT_TRUE(hz::io::StepFormat::saveAssembly(
        step.toStdString(), "Rig", {{"Bracket", {bracket.get()}}, {"Pin", {pin.get()}}},
        {{0, "Bracket:1", hz::math::Mat4::translation({100, 0, 0})},
         {0, "Bracket:2", hz::math::Mat4::identity()},
         {1, "Pin:1", hz::math::Mat4::translation({0, 50, 0})}}));

    MainWindow w;
    const QString kept = dir.filePath(QStringLiteral("Rig.hzasm"));
    {
        FilePicker picker(QStringList{step, kept});
        trigger(w, "import_step_assembly");
    }
    hz::doc::AssemblyDocument* assembly = w.activeAssembly();
    ASSERT_NE(assembly, nullptr);
    EXPECT_FALSE(assembly->isDirty()) << "kept, as its files";
    ASSERT_EQ(assembly->components().size(), 3u);
    EXPECT_EQ(assembly->components()[0].name, "Bracket:1");
    EXPECT_NEAR(translationOf(assembly->components()[0]).x, 100.0, 1e-9);
    EXPECT_NEAR(translationOf(assembly->components()[2]).y, 50.0, 1e-9);
    EXPECT_TRUE(QFileInfo::exists(kept));
    EXPECT_TRUE(QFileInfo::exists(dir.filePath(QStringLiteral("Rig parts/Bracket.hzpart"))));
    EXPECT_TRUE(QFileInfo::exists(dir.filePath(QStringLiteral("Rig parts/Pin.hzpart"))));
    EXPECT_NE(assembly->components()[0].cachedMesh, nullptr) << "its parts' shapes are shown";
}

// Phase 158: a component dragged in the view goes where the cursor takes it,
// in the plane it was grabbed in, facing the view; its mates still hold;
// the drag is one undo step.
TEST(AssembliesTest, ADraggedComponentFollowsTheCursorAndKeepsItsMates) {
    QTemporaryDir dir;
    const QString block = dir.filePath(QStringLiteral("block.hzpart"));
    const std::string top = savePart(block, hz::doc::PrimitiveFeature::makeBox(10, 10, 10), "/top");
    MainWindow w;
    ToolDriver drive(w);
    auto& assembly = newAssembly(w);
    const auto [base, lid] = stackTwo(w, assembly, block, top);
    expectAt(*assembly.component(lid), Vec3(12, 0, 10), "on the base's top, beside it");
    const size_t steps = w.activeDocument()->undoStack().undoCount();

    lookFromAbove(drive.viewport());
    // The lid's top, grabbed in its middle, taken 20 across and 20 back.
    dragOnScreen(drive, Vec3(17, 5, 20), Vec3(37, 25, 20));
    expectAt(*assembly.component(lid), Vec3(32, 20, 10), "under the cursor, on the base's top");
    expectAt(*assembly.component(base), Vec3(0, 0, 0), "the base where it was");
    EXPECT_EQ(w.activeDocument()->undoStack().undoCount(), steps + 1) << "one step";
    EXPECT_TRUE(assembly.isDirty());

    trigger(w, "action_undo");
    expectAt(*assembly.component(lid), Vec3(12, 0, 10), "undone");
}

// Dragged where its mates cannot follow (up off the base), it slides as far
// as they let it: along the base's top, under the cursor's way.
TEST(AssembliesTest, ADragTheMatesCannotFollowSlidesAsFarAsTheyLet) {
    QTemporaryDir dir;
    const QString block = dir.filePath(QStringLiteral("block.hzpart"));
    const std::string top = savePart(block, hz::doc::PrimitiveFeature::makeBox(10, 10, 10), "/top");
    MainWindow w;
    ToolDriver drive(w);
    auto& assembly = newAssembly(w);
    const auto [base, lid] = stackTwo(w, assembly, block, top);

    lookFromTheFront(drive.viewport());
    // Its front's middle, taken 20 along and 20 up.
    dragOnScreen(drive, Vec3(17, 0, 15), Vec3(37, 0, 35));
    expectAt(*assembly.component(lid), Vec3(32, 0, 10), "along, and still on the base's top");
    expectAt(*assembly.component(base), Vec3(0, 0, 0), "the base not lifted to it");
}

// Held by a Fixed mate, it is not dragged, and the status says so; nothing
// is recorded.
TEST(AssembliesTest, AHeldComponentIsNotDragged) {
    QTemporaryDir dir;
    const QString block = dir.filePath(QStringLiteral("block.hzpart"));
    const std::string top = savePart(block, hz::doc::PrimitiveFeature::makeBox(10, 10, 10), "/top");
    MainWindow w;
    ToolDriver drive(w);
    auto& assembly = newAssembly(w);
    const auto [base, lid] = stackTwo(w, assembly, block, top);
    hz::doc::Mate fixed;
    fixed.type = hz::doc::MateType::Fixed;
    fixed.a = {lid, hz::topo::TopologyID::fromTag(top)};
    assembly.addMate(fixed);
    const size_t steps = w.activeDocument()->undoStack().undoCount();

    lookFromAbove(drive.viewport());
    dragOnScreen(drive, Vec3(17, 5, 20), Vec3(37, 25, 20));
    expectAt(*assembly.component(lid), Vec3(12, 0, 10), "held");
    EXPECT_TRUE(w.statusBar()->currentMessage().contains(QStringLiteral("Fixed")))
        << w.statusBar()->currentMessage().toStdString();
    EXPECT_EQ(w.activeDocument()->undoStack().undoCount(), steps);
}

// Escape during a drag puts everything back; the release after it does
// nothing. A press and release in place chooses the component.
TEST(AssembliesTest, EscapePutsADragBackAndAClickChooses) {
    QTemporaryDir dir;
    const QString block = dir.filePath(QStringLiteral("block.hzpart"));
    const std::string top = savePart(block, hz::doc::PrimitiveFeature::makeBox(10, 10, 10), "/top");
    MainWindow w;
    ToolDriver drive(w);
    auto& assembly = newAssembly(w);
    const auto [base, lid] = stackTwo(w, assembly, block, top);
    const size_t steps = w.activeDocument()->undoStack().undoCount();
    auto& view = drive.viewport();
    lookFromAbove(view);

    const QPointF grab = view.projectToScreen(Vec3(17, 5, 20));
    const QPointF away = view.projectToScreen(Vec3(37, 25, 20));
    drive.pressAt(grab);
    drive.dragAt(away);
    ASSERT_TRUE(view.draggingComponent());
    expectAt(*assembly.component(lid), Vec3(32, 20, 10), "moving");
    drive.key(Qt::Key_Escape);
    expectAt(*assembly.component(lid), Vec3(12, 0, 10), "put back");
    drive.releaseAt(away);
    expectAt(*assembly.component(lid), Vec3(12, 0, 10), "the release does nothing");
    EXPECT_EQ(w.activeDocument()->undoStack().undoCount(), steps);

    view.clearModelSelection();
    drive.pressAt(grab);
    drive.releaseAt(grab);
    ASSERT_EQ(view.modelSelection().size(), 1u) << "a click, not a drag";
    EXPECT_EQ(view.modelSelection().front().owner, lid);
    expectAt(*assembly.component(lid), Vec3(12, 0, 10), "not moved");
}

// Another button pressed during a drag gives the drag up: everything back,
// and the left button's release after it chooses nothing and records
// nothing. An undo during a drag puts the drag back first, and its release
// records nothing over the undo, which can still be redone.
TEST(AssembliesTest, ASecondButtonOrAnUndoDuringADragPutsItBack) {
    QTemporaryDir dir;
    const QString block = dir.filePath(QStringLiteral("block.hzpart"));
    const std::string top = savePart(block, hz::doc::PrimitiveFeature::makeBox(10, 10, 10), "/top");
    MainWindow w;
    ToolDriver drive(w);
    auto& assembly = newAssembly(w);
    const auto [base, lid] = stackTwo(w, assembly, block, top);
    auto& undo = w.activeDocument()->undoStack();
    const size_t steps = undo.undoCount();
    auto& view = drive.viewport();
    lookFromAbove(view);
    view.clearModelSelection();

    const QPointF grab = view.projectToScreen(Vec3(17, 5, 20));
    const QPointF away = view.projectToScreen(Vec3(37, 25, 20));
    drive.pressAt(grab);
    drive.dragAt(away);
    ASSERT_TRUE(view.draggingComponent());
    drive.pressAlsoAt(away, Qt::MiddleButton);
    expectAt(*assembly.component(lid), Vec3(12, 0, 10), "put back");
    drive.releaseAlsoAt(away, Qt::MiddleButton);
    drive.dragAt(grab);
    drive.releaseAt(grab);
    expectAt(*assembly.component(lid), Vec3(12, 0, 10), "and left there");
    EXPECT_TRUE(view.modelSelection().empty()) << "the release chose nothing";
    EXPECT_EQ(undo.undoCount(), steps);

    // Undo mid-drag: the move stackTwo made is undone, not recorded over.
    drive.pressAt(grab);
    drive.dragAt(away);
    ASSERT_TRUE(view.draggingComponent());
    trigger(w, "action_undo");
    EXPECT_FALSE(view.draggingComponent());
    drive.releaseAt(away);
    EXPECT_EQ(undo.undoCount(), steps - 1) << "the undo, and no drag step";
    EXPECT_TRUE(undo.canRedo()) << "what was undone can be redone";

    // A release that never comes (a dialog a shortcut opened took the
    // mouse): the next move without the button puts the drag back.
    trigger(w, "action_redo");
    const Vec3 before = translationOf(*assembly.component(lid));
    drive.pressAt(grab);
    drive.dragAt(away);
    ASSERT_TRUE(view.draggingComponent());
    drive.moveAt(away);
    EXPECT_FALSE(view.draggingComponent());
    const Vec3 after = translationOf(*assembly.component(lid));
    EXPECT_NEAR((after - before).length(), 0.0, 1e-9) << "put back";
    EXPECT_EQ(undo.undoCount(), steps) << "nothing recorded";
}

// Phase 158b: the chosen component's triad. Dragged by its x arrow, the lid
// moves along x only, however the cursor strays; by its z ring, it turns
// about z through its middle. Nothing chosen, no triad.
TEST(AssembliesTest, TheTriadMovesAlongAnAxisAndTurnsAboutOne) {
    QTemporaryDir dir;
    const QString block = dir.filePath(QStringLiteral("block.hzpart"));
    const std::string top = savePart(block, hz::doc::PrimitiveFeature::makeBox(10, 10, 10), "/top");
    MainWindow w;
    ToolDriver drive(w);
    auto& assembly = newAssembly(w);
    const auto [base, lid] = stackTwo(w, assembly, block, top);
    auto& view = drive.viewport();
    lookFromAbove(view);

    view.clearModelSelection();
    EXPECT_FALSE(view.triad().has_value()) << "nothing chosen";
    drive.clickAt(view.projectToScreen(Vec3(17, 5, 20)));  // the lid's top
    auto triad = view.triad();
    if (!triad) FAIL() << "the lid chosen, and no triad";
    EXPECT_NEAR(triad->origin().x, 17.0, 1e-9) << "at its middle";
    EXPECT_NEAR(triad->origin().z, 15.0, 1e-9);

    // Along x: grabbed on the arrow, taken 10 further along it.
    using Handle = hz::ui::Triad::Handle;
    const Handle arrowX{hz::ui::Triad::Kind::Arrow, 0};
    ASSERT_EQ(triad->hitTest(triad->handlePoint(arrowX)), arrowX);
    const Vec3 onArrow = triad->origin() + triad->axis(0) * (0.7 * triad->size());
    drive.pressAt(triad->handlePoint(arrowX));
    drive.dragAt(view.projectToScreen(onArrow + Vec3(5, 3, 0)));
    drive.dragAt(view.projectToScreen(onArrow + Vec3(10, 0, 0)));
    drive.releaseAt(view.projectToScreen(onArrow + Vec3(10, 0, 0)));
    expectAt(*assembly.component(lid), Vec3(22, 0, 10), "10 along x, and nothing else");

    // About z: grabbed on the ring, taken a quarter turn round it.
    drive.clickAt(view.projectToScreen(Vec3(27, 5, 20)));  // its top, where it is now
    triad = view.triad();
    if (!triad) FAIL() << "the lid chosen again, and no triad";
    const Handle ringZ{hz::ui::Triad::Kind::Ring, 2};
    ASSERT_EQ(triad->hitTest(triad->handlePoint(ringZ)), ringZ);
    const double r = hz::ui::Triad::kRingShare * triad->size();
    const auto round = [&](double degrees) {
        const double t = degrees * std::numbers::pi / 180.0;
        return view.projectToScreen(
            triad->origin() + (triad->axis(0) * std::cos(t) + triad->axis(1) * std::sin(t)) * r);
    };
    const Vec3 middle = triad->origin();
    drive.pressAt(round(30));
    drive.dragAt(round(75));
    drive.dragAt(round(120));
    drive.releaseAt(round(120));
    const auto& turned = assembly.component(lid)->transform;
    const Vec3 x = turned.transformDirection(Vec3::UnitX);
    EXPECT_NEAR(x.x, 0.0, 1e-6) << "its x now along y";
    EXPECT_NEAR(x.y, 1.0, 1e-6);
    EXPECT_NEAR(x.z, 0.0, 1e-6) << "turned about z only: still on the base's top";
    const Vec3 centre = turned.transformPoint(Vec3(5, 5, 5));
    EXPECT_NEAR((centre - middle).length(), 0.0, 1e-6) << "about its middle";
}

namespace {

/// An assembly saved at @p path placing @p part twice, one on the other.
void savePair(const QString& path, const QString& part) {
    hz::doc::AssemblyDocument pair;
    for (const double z : {0.0, 10.0}) {
        hz::doc::ComponentInstance comp;
        comp.name = QFileInfo(part).completeBaseName().toStdString();
        comp.partPath = part.toStdString();
        comp.transform = hz::math::Mat4::translation(Vec3(0, 0, z));
        pair.addComponent(comp);
    }
    ASSERT_TRUE(hz::io::NativeFormat::saveAssembly(path.toStdString(), pair));
}

}  // namespace

// Phase 159: an assembly inserted into another is a component of it, shown
// whole; the tree lists its own components under it; the bill of materials
// is top level, indented or parts only.
TEST(AssembliesTest, AnAssemblyIsPlacedAsAComponent) {
    QTemporaryDir dir;
    const QString block = dir.filePath(QStringLiteral("block.hzpart"));
    const QString pair = dir.filePath(QStringLiteral("pair.hzasm"));
    savePart(block, hz::doc::PrimitiveFeature::makeBox(10, 10, 10));
    savePair(pair, block);
    MainWindow w;
    auto& assembly = newAssembly(w);
    insert(w, block);
    insert(w, pair);
    ASSERT_EQ(assembly.components().size(), 2u) << w.statusBar()->currentMessage().toStdString();
    const auto& sub = assembly.components()[1];
    EXPECT_TRUE(sub.isAssembly());
    ASSERT_NE(sub.resolvedAssembly, nullptr);
    ASSERT_NE(sub.cachedMesh, nullptr) << "shown whole";
    EXPECT_NEAR(heightOf(sub), 20.0, 1e-4) << "both of its blocks";

    auto* panel = w.findChild<hz::ui::AssemblyTreePanel*>();
    QTreeWidget* tree = panel->tree();
    QTreeWidgetItem* components = tree->topLevelItem(0);
    ASSERT_EQ(components->childCount(), 2);
    EXPECT_EQ(components->child(1)->childCount(), 2) << "its own components under it";

    // The bill of materials, each way.
    std::map<QString, QStringList> shown;
    QTimer poll;
    QElapsedTimer clock;
    clock.start();
    QObject::connect(&poll, &QTimer::timeout, [&] {
        auto* dialog = qobject_cast<QDialog*>(QApplication::activeModalWidget());
        if (dialog == nullptr || dialog->windowTitle() != QStringLiteral("Bill of Materials")) {
            if (clock.elapsed() > 10'000) poll.stop();
            return;
        }
        poll.stop();
        auto* table = dialog->findChild<QTableWidget*>(QStringLiteral("bom"));
        auto* kind = dialog->findChild<QComboBox*>(QStringLiteral("bomKind"));
        for (int k = 0; table != nullptr && kind != nullptr && k < kind->count(); ++k) {
            kind->setCurrentIndex(k);
            QStringList rows;
            for (int row = 0; row < table->rowCount(); ++row) {
                rows << table->item(row, 0)->text() + " " + table->item(row, 1)->text().trimmed() +
                            " x" + table->item(row, 2)->text();
            }
            shown[kind->itemText(k)] = rows;
        }
        dialog->reject();
    });
    poll.start(5);
    trigger(w, "action_bill_of_materials");
    EXPECT_EQ(shown[QStringLiteral("Top level")],
              QStringList({QStringLiteral("1 block x1"), QStringLiteral("2 pair x1")}));
    EXPECT_EQ(shown[QStringLiteral("Indented")],
              QStringList({QStringLiteral("1 block x1"), QStringLiteral("2 pair x1"),
                           QStringLiteral("2.1 block x2")}));
    EXPECT_EQ(shown[QStringLiteral("Parts only")], QStringList({QStringLiteral("1 block x3")}));

    // STEP: its parts placed with the rest, in one level, their transforms
    // composed with its.
    const QString step = dir.filePath(QStringLiteral("rig.step"));
    {
        FilePicker picker(step);
        trigger(w, "export_step");
    }
    const auto read = hz::io::StepFormat::loadAssembly(step.toStdString());
    ASSERT_EQ(read.parts.size(), 1u) << hz::io::StepFormat::lastError();
    ASSERT_EQ(read.occurrences.size(), 3u) << "the block, and the pair's two";
    const Vec3 pairAt = translationOf(sub);
    const Vec3 upper = read.occurrences[2].transform.transformPoint(Vec3());
    EXPECT_NEAR(upper.x, pairAt.x, 1e-9);
    EXPECT_NEAR(upper.z, pairAt.z + 10.0, 1e-9) << "the pair's upper block, where it puts it";
}

// An assembly is not inserted into itself, nor into one it places.
TEST(AssembliesTest, AnAssemblyIsNotInsertedIntoItself) {
    QTemporaryDir dir;
    const QString block = dir.filePath(QStringLiteral("block.hzpart"));
    const QString top = dir.filePath(QStringLiteral("top.hzasm"));
    savePart(block, hz::doc::PrimitiveFeature::makeBox(10, 10, 10));
    MainWindow w;
    auto& assembly = newAssembly(w);
    insert(w, block);
    ASSERT_TRUE(hz::io::NativeFormat::saveAssembly(top.toStdString(), assembly));
    assembly.setFilePath(top.toStdString());

    DialogResponder refusal(QMessageBox::Ok);
    insert(w, top);
    refusal.waitForDialog(5000);
    ASSERT_TRUE(refusal.seen());
    EXPECT_TRUE(refusal.text().contains(QStringLiteral("inside itself")))
        << refusal.text().toStdString();
    EXPECT_EQ(assembly.components().size(), 1u) << "not inserted";
}

// Phase 160: a Distance mate held between limits. The second block starts
// level with the first, below the least gap: it is lifted to it, and the
// tree says the limits.
TEST(AssembliesTest, ADistanceBetweenLimitsIsHeldAtTheNearestLimit) {
    QTemporaryDir dir;
    const QString block = dir.filePath(QStringLiteral("block.hzpart"));
    savePart(block, hz::doc::PrimitiveFeature::makeBox(10, 10, 10));
    MainWindow w;
    auto& assembly = newAssembly(w);
    insert(w, block);
    insert(w, block);
    const uint64_t lid = assembly.components()[1].id;
    {
        FormFiller mate(QStringLiteral("Add Mate"),
                        FormAnswers()
                            .choose(QStringLiteral("type"), QStringLiteral("Distance"))
                            .chooseContaining(QStringLiteral("faceA"), QStringLiteral("/top)"))
                            .chooseContaining(QStringLiteral("faceB"), QStringLiteral("/bottom)"))
                            .choose(QStringLiteral("held"), QStringLiteral("Between limits"))
                            .number(QStringLiteral("minimum"), 5.0)
                            .number(QStringLiteral("maximum"), 10.0));
        trigger(w, "action_add_mate");
        ASSERT_TRUE(mate.seen());
    }
    ASSERT_EQ(assembly.mates().size(), 1u) << w.statusBar()->currentMessage().toStdString();
    const auto& m = assembly.mates().front();
    ASSERT_TRUE(m.minimum.has_value() && m.maximum.has_value());
    EXPECT_NEAR(translationOf(*assembly.component(lid)).z, 15.0, 1e-6)
        << "its bottom 5 above the first's top";

    auto* tree = w.findChild<hz::ui::AssemblyTreePanel*>()->tree();
    const QString text = tree->topLevelItem(1)->child(0)->text(0);
    EXPECT_TRUE(text.contains(QStringLiteral(" to "))) << text.toStdString();
}
