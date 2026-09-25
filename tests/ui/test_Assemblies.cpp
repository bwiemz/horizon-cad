// Placing components (Phase 143): an inserted part lands beside the others,
// moves and turns exactly and undoably with its mates solved again, is
// listed with its mates in the assembly tree, which removes, suppresses and
// renames, and can be clicked at once: two clicked cylinders make a
// concentric mate.

#include <gtest/gtest.h>

#include <QAction>
#include <QApplication>
#include <QKeyEvent>
#include <QStatusBar>
#include <QTabBar>
#include <QTemporaryDir>
#include <QTreeWidget>
#include <cmath>
#include <numbers>
#include <string>

#include "UiTestSupport.h"
#include "horizon/document/AssemblyDocument.h"
#include "horizon/document/Document.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/fileio/NativeFormat.h"
#include "horizon/topology/Solid.h"
#include "horizon/ui/AssemblyTreePanel.h"
#include "horizon/ui/MainWindow.h"
#include "horizon/ui/ViewportWidget.h"

using hz::math::Vec3;
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
