// Workbenches (Phase 146): the assembly commands, moved out of MainWindow,
// work through WorkbenchHost alone, as do the drawing sheets (Phase 148),
// the commands that add a feature to a part, and the 2D drafting commands.
// Here they run against a stand-in host with no window at all, which is what
// the interface is for.

#include <gtest/gtest.h>

#include <QApplication>
#include <QComboBox>
#include <QDialog>
#include <QDoubleSpinBox>
#include <QFile>
#include <QLineEdit>
#include <QListWidget>
#include <QMouseEvent>
#include <QTemporaryDir>
#include <QTimer>
#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "horizon/document/AssemblyDocument.h"
#include "horizon/document/Commands.h"
#include "horizon/document/Document.h"
#include "horizon/document/DocumentManager.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/document/ModelCommands.h"
#include "horizon/document/Sketch.h"
#include "horizon/document/UndoStack.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftPolyline.h"
#include "horizon/drafting/DraftRectangle.h"
#include "horizon/drafting/DraftText.h"
#include "horizon/drafting/Layer.h"
#include "horizon/fileio/DrawingDocumentIO.h"
#include "horizon/fileio/NativeFormat.h"
#include "horizon/math/BoundingBox.h"
#include "horizon/math/Constants.h"
#include "horizon/modeling/DrawingView.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/topology/Solid.h"
#include "horizon/ui/AssemblyTreePanel.h"
#include "horizon/ui/AssemblyWorkbench.h"
#include "horizon/ui/DraftingCommands.h"
#include "horizon/ui/DrawingWorkbench.h"
#include "horizon/ui/PartCommands.h"
#include "horizon/ui/Tool.h"
#include "horizon/ui/ViewportWidget.h"
#include "horizon/ui/WorkbenchHost.h"

using hz::math::Vec3;

namespace {

/// A host with what the commands ask for and nothing else: documents, a
/// view, and a status bar that keeps what it was told.
class StandInHost : public hz::ui::WorkbenchHost {
public:
    StandInHost() {
        m_documents.setPartLoader([](const std::string& path, hz::doc::Document& doc) {
            return hz::io::NativeFormat::load(path, doc);
        });
        m_documents.setMeshLoader(
            [](const std::string& path) { return hz::io::NativeFormat::loadPartMesh(path); });
    }

    std::shared_ptr<hz::doc::AssemblyDocument> assembly;
    hz::doc::Document backing;
    /// The tabs the workbenches added, the last of them shown.
    std::vector<std::pair<std::shared_ptr<hz::doc::Document>, QString>> tabs;
    std::vector<QString> statuses;
    std::vector<std::string> fileErrors;
    std::vector<std::string> leftOut;
    std::unique_ptr<hz::ui::Tool> tool;  ///< the last a workbench ran
    int toolsEnded = 0;
    int rebuilds = 0;
    int selectionChanges = 0;
    std::vector<QString> added;               ///< the features added, by their commands
    std::shared_ptr<hz::doc::Sketch> chosen;  ///< as the sketch list has it
    int sketchesFinished = 0;

    QWidget* dialogParent() override { return &m_viewport; }
    hz::doc::Document* currentDocument() override {
        return tabs.empty() ? &backing : tabs.back().first.get();
    }
    std::shared_ptr<hz::doc::AssemblyDocument> currentAssembly() override { return assembly; }
    std::vector<std::shared_ptr<hz::doc::AssemblyDocument>> openAssemblies() override {
        if (!assembly) return {};
        return {assembly};
    }
    hz::doc::DocumentManager& documents() override { return m_documents; }
    hz::ui::ViewportWidget& viewport() override { return m_viewport; }
    void showStatus(const QString& message, int /*timeoutMs*/) override {
        statuses.push_back(message);
    }
    QString currentStatus() override { return statuses.empty() ? QString() : statuses.back(); }
    void setPrompt(const QString& /*text*/) override {}
    void rebuildScene() override { ++rebuilds; }
    void refreshModifiedIndicators() override {}
    void selectionChanged() override { ++selectionChanges; }
    bool openPath(const QString& /*fileName*/) override { return false; }
    void addTab(std::shared_ptr<hz::doc::Document> document, const QString& title) override {
        tabs.emplace_back(std::move(document), title);
    }
    void runTool(std::unique_ptr<hz::ui::Tool> given) override { tool = std::move(given); }
    void endTool() override { ++toolsEnded; }
    void reportFileError(const QString& /*summary*/, const std::string& path,
                         const std::string& reason) override {
        fileErrors.push_back(path + ": " + reason);
    }
    void reportLeftOut(const QString& fileName, const std::vector<std::string>& items) override {
        for (const std::string& item : items)
            leftOut.push_back(fileName.toStdString() + ": " + item);
    }
    void showImportReport(const QString& /*fileName*/, const hz::io::ImportReport& /*report*/,
                          bool /*notSavedOver*/) override {}
    bool mayReplaceSource(const QString& /*fileName*/) override { return true; }
    QString currentTitle() override { return {}; }
    void refreshPanels() override {}
    void rebuildModel() override {}
    bool onWorker(bool /*large*/) override { return false; }
    bool addFeature(std::unique_ptr<hz::doc::Feature> feature, const QString& verb,
                    const std::shared_ptr<hz::doc::Sketch>& wrapperSketch) override {
        hz::doc::Document& part = *currentDocument();
        part.undoStack().push(
            std::make_unique<hz::doc::AddFeatureCommand>(part, std::move(feature), wrapperSketch));
        added.push_back(verb);
        return part.rebuildModel();
    }
    std::shared_ptr<hz::doc::Sketch> chosenSketch() override { return chosen; }
    void finishSketch() override {
        ++sketchesFinished;
        hz::doc::Document& part = *currentDocument();
        if (part.editedSketch()) chosen = part.editedSketch();
        part.editSketch(nullptr);
    }
    void backgroundWorkChanged() override {}

private:
    hz::doc::DocumentManager m_documents;
    hz::ui::ViewportWidget m_viewport;
};

/// A 10 mm cube saved at @p path; its top face's name.
std::string saveCube(const QString& path) {
    hz::doc::Document part;
    part.setType(hz::doc::DocumentType::Part);
    part.featureTree().addFeature(hz::doc::PrimitiveFeature::makeBox(10, 10, 10));
    EXPECT_TRUE(part.rebuildModel());
    EXPECT_TRUE(hz::io::NativeFormat::save(path.toStdString(), part));
    for (const auto& f : part.solid()->faces()) {
        const std::string& tag = f.topoId.tag();
        if (tag.size() >= 4 && tag.compare(tag.size() - 4, 4, "/top") == 0) return tag;
    }
    ADD_FAILURE() << "no top face";
    return {};
}

/// Run @p command, answering the form titled @p title it opens: filled in
/// by @p fill, then OK. False when no such form opened in ten seconds.
bool answering(const QString& title, const std::function<void(QDialog&)>& fill,
               const std::function<void()>& command) {
    bool answered = false;
    QTimer answer;
    QObject::connect(&answer, &QTimer::timeout, [&] {
        auto* form = qobject_cast<QDialog*>(QApplication::activeModalWidget());
        if (form == nullptr || form->windowTitle() != title) return;
        answer.stop();
        fill(*form);
        answered = true;
        form->accept();
    });
    answer.start(5);
    QTimer::singleShot(10'000, &answer, [] {
        if (auto* form = qobject_cast<QDialog*>(QApplication::activeModalWidget())) form->reject();
    });
    command();
    return answered;
}

/// Set the length (or number) named @p name in @p form.
void setField(QDialog& form, const char* name, double value) {
    auto* spin = form.findChild<QDoubleSpinBox*>(QString::fromLatin1(name));
    ASSERT_NE(spin, nullptr) << name;
    spin->setValue(value);
}

}  // namespace

// Without an assembly, a command says so where the host shows messages.
TEST(WorkbenchesTest, AnAssemblyCommandWithoutAnAssemblySaysSo) {
    StandInHost host;
    hz::ui::AssemblyTreePanel tree;
    hz::ui::AssemblyWorkbench workbench(host, tree);
    workbench.onBillOfMaterials();
    ASSERT_FALSE(host.statuses.empty());
    EXPECT_TRUE(host.statuses.back().contains(QStringLiteral("only available in an assembly")));
    EXPECT_FALSE(workbench.busy());
}

// An assembly placed by its mates, and a part changed on disk shown anew,
// with no window: a lid sits on its base when opened, and moves up when the
// base grows.
TEST(WorkbenchesTest, TheWorkbenchPlacesAndRefreshesThroughItsHost) {
    QTemporaryDir dir;
    const QString cube = dir.filePath(QStringLiteral("cube.hzpart"));
    const std::string top = saveCube(cube);
    const std::string bottom = top.substr(0, top.size() - 3) + "bottom";

    StandInHost host;
    host.assembly = std::make_shared<hz::doc::AssemblyDocument>();
    hz::doc::ComponentInstance base;
    base.partPath = cube.toStdString();
    const uint64_t baseId = host.assembly->addComponent(base);
    hz::doc::ComponentInstance lid;
    lid.partPath = cube.toStdString();
    const uint64_t lidId = host.assembly->addComponent(lid);
    hz::doc::Mate on;
    on.type = hz::doc::MateType::Coincident;
    on.a = {baseId, hz::topo::TopologyID::fromTag(top)};
    on.b = {lidId, hz::topo::TopologyID::fromTag(bottom)};
    host.assembly->addMate(on);

    hz::ui::AssemblyTreePanel tree;
    hz::ui::AssemblyWorkbench workbench(host, tree);
    EXPECT_TRUE(workbench.placeOnOpen(*host.assembly)) << "the lid moved onto the base";
    const auto height = [&] {
        return host.assembly->component(lidId)->transform.transformPoint(Vec3()).z;
    };
    EXPECT_NEAR(height(), 10.0, 1e-6);

    // The cube made taller in its file (its features, and so its names, kept).
    {
        hz::doc::Document part;
        ASSERT_TRUE(hz::io::NativeFormat::load(cube.toStdString(), part));
        ASSERT_TRUE(part.featureTree().feature(0)->setParameter("depth", 25.0));
        ASSERT_TRUE(part.rebuildModel());
        ASSERT_TRUE(hz::io::NativeFormat::save(cube.toStdString(), part));
    }
    workbench.refreshComponentsOf(cube.toStdString());
    EXPECT_NEAR(height(), 25.0, 1e-6) << "the mates solved again on the new part";
    EXPECT_TRUE(host.assembly->isDirty());
    EXPECT_GE(host.rebuilds, 1) << "the host rebuilt its scene";
}

// A sheet opened and drawn from its part, and drawn again when the part
// changes, with no window: its own layers locked and as they were set, what
// was drawn on it by hand kept, and it no more modified than it was.
TEST(WorkbenchesTest, TheDrawingWorkbenchDrawsASheetThroughItsHost) {
    QTemporaryDir dir;
    const QString cube = dir.filePath(QStringLiteral("cube.hzpart"));
    saveCube(cube);
    hz::io::DrawingDocumentSpec spec;
    spec.partPath = cube.toStdString();
    spec.scale = 1.0;  // so that a larger part draws larger
    const std::string sheetPath = dir.filePath(QStringLiteral("cube.hzdwg")).toStdString();
    ASSERT_TRUE(hz::io::DrawingDocumentIO::save(sheetPath, spec));

    StandInHost host;
    hz::ui::DrawingWorkbench workbench(host);
    ASSERT_TRUE(workbench.open(QString::fromStdString(sheetPath)));
    ASSERT_EQ(host.tabs.size(), 1u);
    EXPECT_EQ(host.tabs.back().second, QStringLiteral("cube.hzdwg"));
    hz::doc::Document& sheet = *host.tabs.back().first;
    EXPECT_TRUE(workbench.isSheet(&sheet));
    EXPECT_FALSE(workbench.isSheet(&host.backing));
    EXPECT_EQ(sheet.filePath(), sheetPath);
    EXPECT_FALSE(sheet.isDirty());
    ASSERT_NE(sheet.layerManager().getLayer("Visible"), nullptr);
    EXPECT_TRUE(sheet.layerManager().getLayer("Visible")->locked);

    // The height of what the views draw.
    const auto drawnHeight = [&] {
        double low = 1e300;
        double high = -1e300;
        for (const auto& entity : sheet.draftDocument().entities()) {
            if (entity->layer() != "Visible") continue;
            const auto box = entity->boundingBox();
            low = std::min(low, box.min().y);
            high = std::max(high, box.max().y);
        }
        return high - low;
    };
    const double before = drawnHeight();
    ASSERT_GT(before, 10.0);

    // Set by the user: a line drawn by hand, and the hidden edges not shown.
    auto line = std::make_shared<hz::draft::DraftLine>(hz::math::Vec2(0, 0), hz::math::Vec2(5, 5));
    line->setLayer("0");
    sheet.draftDocument().addEntity(line);
    sheet.layerManager().getLayer("Hidden")->visible = false;
    sheet.setDirty(false);

    // The cube made taller in its file.
    {
        hz::doc::Document part;
        ASSERT_TRUE(hz::io::NativeFormat::load(cube.toStdString(), part));
        ASSERT_TRUE(part.featureTree().feature(0)->setParameter("depth", 25.0));
        ASSERT_TRUE(part.rebuildModel());
        ASSERT_TRUE(hz::io::NativeFormat::save(cube.toStdString(), part));
    }
    workbench.refreshDrawingsOf(cube.toStdString());
    EXPECT_GT(drawnHeight(), before + 10.0) << "drawn from the taller part";
    const auto& entities = sheet.draftDocument().entities();
    EXPECT_NE(std::find(entities.begin(), entities.end(), line), entities.end()) << "kept";
    EXPECT_FALSE(sheet.layerManager().getLayer("Hidden")->visible) << "as the user set it";
    EXPECT_TRUE(sheet.layerManager().getLayer("Hidden")->locked);
    EXPECT_FALSE(sheet.isDirty()) << "the part changed, not the sheet";
    EXPECT_TRUE(host.fileErrors.empty());
}

// A sheet whose part is gone does not open, and says why.
TEST(WorkbenchesTest, ASheetWithoutItsPartSaysWhy) {
    QTemporaryDir dir;
    hz::io::DrawingDocumentSpec spec;
    spec.partPath = dir.filePath(QStringLiteral("gone.hzpart")).toStdString();
    const std::string sheetPath = dir.filePath(QStringLiteral("gone.hzdwg")).toStdString();
    ASSERT_TRUE(hz::io::DrawingDocumentIO::save(sheetPath, spec));

    StandInHost host;
    hz::ui::DrawingWorkbench workbench(host);
    EXPECT_FALSE(workbench.open(QString::fromStdString(sheetPath)));
    EXPECT_TRUE(host.tabs.empty());
    ASSERT_EQ(host.fileErrors.size(), 1u);
    EXPECT_NE(host.fileErrors[0].find("part"), std::string::npos) << host.fileErrors[0];
}

// A sheet read in part says what it left out, and is kept from being saved
// over its file (the window's Save asks where instead).
TEST(WorkbenchesTest, ASheetReadInPartSaysSoAndIsMarked) {
    QTemporaryDir dir;
    saveCube(dir.filePath(QStringLiteral("cube.hzpart")));
    const QString sheet = dir.filePath(QStringLiteral("sheet.hzdwg"));
    {
        QFile out(sheet);
        ASSERT_TRUE(out.open(QIODevice::WriteOnly));
        out.write(R"({"format": "hzdwg", "version": 3, "part": "cube.hzpart",
                      "views": [{"kind": "Front"}, "not a view"]})");
    }
    StandInHost host;
    hz::ui::DrawingWorkbench workbench(host);
    ASSERT_TRUE(workbench.open(sheet));
    ASSERT_EQ(host.tabs.size(), 1u);
    EXPECT_TRUE(host.tabs[0].first->readInPart());
    ASSERT_EQ(host.leftOut.size(), 1u);
    EXPECT_EQ(host.leftOut[0], "sheet.hzdwg: view 2: it is not a view");
}

// A version 1 sheet (its part and gap alone) is drawn as version 1 laid it
// out, and saved so: opened again, it is the same. Saved without its views,
// it opened again laid out anew.
TEST(WorkbenchesTest, AVersionOneSheetKeepsItsLayoutWhenSaved) {
    QTemporaryDir dir;
    saveCube(dir.filePath(QStringLiteral("cube.hzpart")));
    const QString v1 = dir.filePath(QStringLiteral("v1.hzdwg"));
    {
        QFile out(v1);
        ASSERT_TRUE(out.open(QIODevice::WriteOnly));
        out.write(R"({"format": "hzdwg", "version": 1, "part": "cube.hzpart", "gap": 15})");
    }
    StandInHost host;
    hz::ui::DrawingWorkbench workbench(host);
    ASSERT_TRUE(workbench.open(v1));
    ASSERT_EQ(host.tabs.size(), 1u);
    const auto visibleBox = [](const hz::doc::Document& sheet) {
        hz::math::BoundingBox box;
        for (const auto& entity : sheet.draftDocument().entities()) {
            if (entity->layer() == "Visible") box.expand(entity->boundingBox());
        }
        return box;
    };
    const hz::math::BoundingBox drawn = visibleBox(*host.tabs[0].first);
    ASSERT_TRUE(drawn.isValid());

    const std::string v2 = dir.filePath(QStringLiteral("v2.hzdwg")).toStdString();
    std::string error;
    ASSERT_TRUE(workbench.save(*host.tabs[0].first, v2, &error)) << error;
    hz::io::DrawingDocumentSpec spec;
    ASSERT_TRUE(hz::io::DrawingDocumentIO::readSpec(v2, spec));
    EXPECT_EQ(spec.views.size(), 4u) << "its views, as laid out";

    ASSERT_TRUE(workbench.open(QString::fromStdString(v2)));
    ASSERT_EQ(host.tabs.size(), 2u);
    const hz::math::BoundingBox again = visibleBox(*host.tabs[1].first);
    EXPECT_NEAR(again.min().x, drawn.min().x, 1e-9);
    EXPECT_NEAR(again.min().y, drawn.min().y, 1e-9);
    EXPECT_NEAR(again.max().x, drawn.max().x, 1e-9);
    EXPECT_NEAR(again.max().y, drawn.max().y, 1e-9);
}

// A closed sheet forgotten while another's form is open (a reading on a
// worker ends in the form's events and draws the part's sheets again) leaves
// the form's sheet as it was: the form kept a pointer into the list of
// sheets, and forgetting one shifted the rest under it.
TEST(WorkbenchesTest, ASheetForgottenWhileAFormIsOpenLeavesTheOthers) {
    QTemporaryDir dir;
    const QString cube = dir.filePath(QStringLiteral("cube.hzpart"));
    saveCube(cube);
    hz::io::DrawingDocumentSpec spec;
    spec.partPath = cube.toStdString();
    const std::string first = dir.filePath(QStringLiteral("first.hzdwg")).toStdString();
    const std::string second = dir.filePath(QStringLiteral("second.hzdwg")).toStdString();
    ASSERT_TRUE(hz::io::DrawingDocumentIO::save(first, spec));
    ASSERT_TRUE(hz::io::DrawingDocumentIO::save(second, spec));

    StandInHost host;
    hz::ui::DrawingWorkbench workbench(host);
    ASSERT_TRUE(workbench.open(QString::fromStdString(first)));
    ASSERT_TRUE(workbench.open(QString::fromStdString(second)));
    // The first sheet's tab closed: the workbench still lists it.
    host.documents().closeDocument(host.tabs.front().first);
    host.tabs.erase(host.tabs.begin());
    hz::doc::Document& shown = *host.tabs.back().first;

    // While the Scale form is open, the part's sheets are drawn again (the
    // closed one forgotten), then 2:1 is chosen.
    bool answered = false;
    QTimer answer;
    QObject::connect(&answer, &QTimer::timeout, [&] {
        auto* form = qobject_cast<QDialog*>(QApplication::activeModalWidget());
        if (form == nullptr || form->windowTitle() != QStringLiteral("Drawing Scale")) return;
        answer.stop();
        workbench.refreshDrawingsOf(cube.toStdString());
        auto* scale = form->findChild<QComboBox*>(QStringLiteral("scale"));
        ASSERT_NE(scale, nullptr);
        scale->setCurrentIndex(scale->findText(QStringLiteral("2:1")));
        answered = true;
        form->accept();
    });
    answer.start(5);
    QTimer::singleShot(10'000, [&] {
        if (auto* form = qobject_cast<QDialog*>(QApplication::activeModalWidget())) form->reject();
    });
    workbench.onScale();
    ASSERT_TRUE(answered);
    EXPECT_TRUE(shown.isDirty()) << "drawn at the scale chosen";
    bool atTwo = false;
    for (const auto& entity : shown.draftDocument().entities()) {
        if (const auto* text = dynamic_cast<const hz::draft::DraftText*>(entity.get())) {
            atTwo = atTwo || text->text() == "SCALE: 2:1";
        }
    }
    EXPECT_TRUE(atTwo);
}

// Move View's clicks through its tool, with no window: a click off every
// view is refused and says why; the last click is handed over when its
// button comes up, not before, and the tool is ended after it, not in it.
TEST(WorkbenchesTest, AViewIsMovedByTheToolsClicks) {
    QTemporaryDir dir;
    const QString cube = dir.filePath(QStringLiteral("cube.hzpart"));
    saveCube(cube);
    hz::io::DrawingDocumentSpec spec;
    spec.partPath = cube.toStdString();
    const std::string sheetPath = dir.filePath(QStringLiteral("cube.hzdwg")).toStdString();
    ASSERT_TRUE(hz::io::DrawingDocumentIO::save(sheetPath, spec));
    StandInHost host;
    hz::ui::DrawingWorkbench workbench(host);
    ASSERT_TRUE(workbench.open(QString::fromStdString(sheetPath)));
    hz::doc::Document& sheet = *host.tabs.back().first;

    workbench.onMoveView();
    ASSERT_NE(host.tool, nullptr);
    hz::ui::Tool& tool = *host.tool;
    const auto button = [&](QEvent::Type type, const hz::math::Vec2& at) {
        QMouseEvent event(type, QPointF(0, 0), QPointF(0, 0), Qt::LeftButton,
                          type == QEvent::MouseButtonPress ? Qt::LeftButton : Qt::NoButton,
                          Qt::NoModifier);
        return type == QEvent::MouseButtonPress ? tool.mousePressEvent(&event, at)
                                                : tool.mouseReleaseEvent(&event, at);
    };
    const auto clickAt = [&](const hz::math::Vec2& at) {
        button(QEvent::MouseButtonPress, at);
        button(QEvent::MouseButtonRelease, at);
    };

    clickAt({1.0, 1.0});  // the sheet's corner: no view there
    EXPECT_NE(tool.promptText().find("Not on a view"), std::string::npos) << tool.promptText();

    // The sheet's layout, as the workbench drew it: the Isometric view's middle.
    hz::doc::Document part;
    ASSERT_TRUE(hz::io::NativeFormat::load(cube.toStdString(), part));
    ASSERT_TRUE(part.rebuildModel());
    const auto layout = hz::model::DrawingGenerator::sheetLayout(*part.solid(), spec.sheet,
                                                                 spec.titleBlock, spec.gap);
    const auto& iso = layout.views[3];
    const hz::math::Vec2 middle{iso.placement.x + iso.sheetWidth() / 2.0,
                                iso.placement.y + iso.sheetHeight() / 2.0};
    clickAt(middle);
    EXPECT_EQ(tool.promptText().find("Not on a view"), std::string::npos);
    button(QEvent::MouseButtonPress, {middle.x - 30.0, middle.y});
    QCoreApplication::processEvents();
    EXPECT_FALSE(sheet.isDirty()) << "not moved while the button is down";
    button(QEvent::MouseButtonRelease, {middle.x - 30.0, middle.y});
    EXPECT_EQ(host.toolsEnded, 0) << "not ended inside its own event";
    QCoreApplication::processEvents();
    EXPECT_EQ(host.toolsEnded, 1);
    EXPECT_TRUE(sheet.isDirty()) << "moved";

    const std::string moved = dir.filePath(QStringLiteral("moved.hzdwg")).toStdString();
    std::string error;
    ASSERT_TRUE(workbench.save(sheet, moved, &error)) << error;
    hz::io::DrawingDocumentSpec read;
    ASSERT_TRUE(hz::io::DrawingDocumentIO::readSpec(moved, read));
    ASSERT_EQ(read.views.size(), 4u);
    EXPECT_NEAR(read.views[3].placement.x, iso.placement.x - 30.0, 1e-9);
    EXPECT_NEAR(read.views[3].placement.y, iso.placement.y, 1e-9);
}

// ---------------------------------------------------------------------------
// What is drawn on a sheet by hand (Phase 150)
// ---------------------------------------------------------------------------

namespace {

/// A note on @p sheet at @p at, on layer @p layer: what it now holds.
std::shared_ptr<hz::draft::DraftText> note(hz::doc::Document& sheet, const hz::math::Vec2& at,
                                           const std::string& text, const std::string& layer) {
    auto t = std::make_shared<hz::draft::DraftText>(at, text, 3.5);
    t->setLayer(layer);
    sheet.draftDocument().addEntity(t);
    return t;
}

/// The notes on @p sheet saying @p text.
std::vector<const hz::draft::DraftText*> notesSaying(const hz::doc::Document& sheet,
                                                     const std::string& text) {
    std::vector<const hz::draft::DraftText*> found;
    for (const auto& e : sheet.draftDocument().entities()) {
        const auto* t = dynamic_cast<const hz::draft::DraftText*>(e.get());
        if (t != nullptr && t->text() == text) found.push_back(t);
    }
    return found;
}

/// The cube's sheet laid out as the workbench lays it (automatic, A3).
hz::model::Drawing cubeLayout(const QString& cube) {
    hz::doc::Document part;
    EXPECT_TRUE(hz::io::NativeFormat::load(cube.toStdString(), part));
    EXPECT_TRUE(part.rebuildModel());
    const hz::io::DrawingDocumentSpec spec;
    return hz::model::DrawingGenerator::sheetLayout(*part.solid(), spec.sheet, spec.titleBlock,
                                                    spec.gap);
}

hz::math::Vec2 centreOf(const hz::model::DrawingView& v) {
    return {v.placement.x + v.sheetWidth() / 2.0, v.placement.y + v.sheetHeight() / 2.0};
}

}  // namespace

// What is drawn on a sheet by hand is saved with it: its entities, their
// layers and the dimension style; opened again, it is where it was. Read
// again, the file's replace what is there, and nothing is left to undo.
TEST(WorkbenchesTest, WhatIsDrawnByHandIsSavedWithTheSheet) {
    QTemporaryDir dir;
    const QString cube = dir.filePath(QStringLiteral("cube.hzpart"));
    saveCube(cube);
    hz::io::DrawingDocumentSpec spec;
    spec.partPath = cube.toStdString();
    const std::string sheetPath = dir.filePath(QStringLiteral("cube.hzdwg")).toStdString();
    ASSERT_TRUE(hz::io::DrawingDocumentIO::save(sheetPath, spec));

    StandInHost host;
    hz::ui::DrawingWorkbench workbench(host);
    ASSERT_TRUE(workbench.open(QString::fromStdString(sheetPath)));
    hz::doc::Document& sheet = *host.tabs.back().first;
    hz::draft::LayerProperties notes;
    notes.name = "Notes";
    notes.color = 0xFFFF0000;
    sheet.layerManager().addLayer(notes);
    note(sheet, {50.0, 30.0}, "DEBURR ALL EDGES", "Notes");
    // Layer "0" as the user set it: a document starts with one of its own.
    sheet.layerManager().getLayer("0")->color = 0xFF00FF00;
    sheet.layerManager().getLayer("0")->locked = true;
    auto style = sheet.draftDocument().dimensionStyle();
    style.textHeight = 5.0;
    sheet.draftDocument().setDimensionStyle(style);
    std::string error;
    ASSERT_TRUE(workbench.save(sheet, sheetPath, &error)) << error;

    hz::io::DrawingDocumentSpec read;
    ASSERT_TRUE(hz::io::DrawingDocumentIO::readSpec(sheetPath, read, &error)) << error;
    ASSERT_NE(read.annotations, nullptr);
    EXPECT_EQ(read.annotations->draftDocument().entities().size(), 1u) << "the note, not the views";

    StandInHost again;
    hz::ui::DrawingWorkbench reopened(again);
    ASSERT_TRUE(reopened.open(QString::fromStdString(sheetPath)));
    hz::doc::Document& back = *again.tabs.back().first;
    const auto found = notesSaying(back, "DEBURR ALL EDGES");
    ASSERT_EQ(found.size(), 1u);
    EXPECT_NEAR(found[0]->position().x, 50.0, 1e-9);
    EXPECT_NEAR(found[0]->position().y, 30.0, 1e-9);
    ASSERT_NE(back.layerManager().getLayer("Notes"), nullptr);
    EXPECT_EQ(back.layerManager().getLayer("Notes")->color, 0xFFFF0000u);
    EXPECT_DOUBLE_EQ(back.draftDocument().dimensionStyle().textHeight, 5.0);
    ASSERT_NE(back.layerManager().getLayer("0"), nullptr);
    EXPECT_EQ(back.layerManager().getLayer("0")->color, 0xFF00FF00u) << "as set, not as new";
    EXPECT_TRUE(back.layerManager().getLayer("0")->locked);
    EXPECT_FALSE(back.isDirty());

    // Another note, then the file read again: the file's alone.
    note(back, {80.0, 30.0}, "NOT KEPT", "Notes");
    reopened.readAgain(back);
    EXPECT_TRUE(notesSaying(back, "NOT KEPT").empty());
    EXPECT_EQ(notesSaying(back, "DEBURR ALL EDGES").size(), 1u);
    EXPECT_FALSE(back.undoStack().canUndo());
}

// A note drawn in a view moves with it: when the view is moved, and when
// the part grows and the sheet is laid out again; one drawn outside every
// view stays where it is.
TEST(WorkbenchesTest, WhatIsDrawnInAViewMovesWithIt) {
    QTemporaryDir dir;
    const QString cube = dir.filePath(QStringLiteral("cube.hzpart"));
    saveCube(cube);
    hz::io::DrawingDocumentSpec spec;
    spec.partPath = cube.toStdString();
    const std::string sheetPath = dir.filePath(QStringLiteral("cube.hzdwg")).toStdString();
    ASSERT_TRUE(hz::io::DrawingDocumentIO::save(sheetPath, spec));
    StandInHost host;
    hz::ui::DrawingWorkbench workbench(host);
    ASSERT_TRUE(workbench.open(QString::fromStdString(sheetPath)));
    hz::doc::Document& sheet = *host.tabs.back().first;

    const auto layout = cubeLayout(cube);
    const hz::model::DrawingView& front = layout.views[0];
    const hz::math::Vec2 inFront = centreOf(front);
    auto inside = note(sheet, inFront, "IN FRONT", "0");
    auto outside = note(sheet, {15.0, 15.0}, "IN THE CORNER", "0");

    // The part grows: the sheet is laid out again, Front with it.
    {
        hz::doc::Document part;
        ASSERT_TRUE(hz::io::NativeFormat::load(cube.toStdString(), part));
        ASSERT_TRUE(part.featureTree().feature(0)->setParameter("depth", 25.0));
        ASSERT_TRUE(part.rebuildModel());
        ASSERT_TRUE(hz::io::NativeFormat::save(cube.toStdString(), part));
    }
    workbench.refreshDrawingsOf(cube.toStdString());
    const hz::math::Vec2 moved = centreOf(cubeLayout(cube).views[0]);
    ASSERT_GT(std::hypot(moved.x - inFront.x, moved.y - inFront.y), 1.0) << "Front moved";
    EXPECT_NEAR(inside->position().x, moved.x, 1e-9) << "at Front's centre still";
    EXPECT_NEAR(inside->position().y, moved.y, 1e-9);
    EXPECT_NEAR(outside->position().x, 15.0, 1e-12) << "on no view: where it was";
    EXPECT_NEAR(outside->position().y, 15.0, 1e-12);

    // Front moved by clicks: the note with it.
    workbench.onMoveView();
    ASSERT_NE(host.tool, nullptr);
    const auto button = [&](QEvent::Type type, const hz::math::Vec2& at) {
        QMouseEvent event(type, QPointF(0, 0), QPointF(0, 0), Qt::LeftButton,
                          type == QEvent::MouseButtonPress ? Qt::LeftButton : Qt::NoButton,
                          Qt::NoModifier);
        if (type == QEvent::MouseButtonPress) {
            host.tool->mousePressEvent(&event, at);
        } else {
            host.tool->mouseReleaseEvent(&event, at);
        }
    };
    for (const hz::math::Vec2& at : {moved, hz::math::Vec2{moved.x + 12.0, moved.y - 7.0}}) {
        button(QEvent::MouseButtonPress, at);
        button(QEvent::MouseButtonRelease, at);
    }
    QCoreApplication::processEvents();
    EXPECT_NEAR(inside->position().x, moved.x + 12.0, 1e-9);
    EXPECT_NEAR(inside->position().y, moved.y - 7.0, 1e-9);
    EXPECT_NEAR(outside->position().x, 15.0, 1e-12);
}

// A note saved in a view is where the view is when the sheet is opened
// again, though the part changed in between and the view moved.
TEST(WorkbenchesTest, ANoteFollowsItsViewAcrossAChangeWhileTheSheetWasClosed) {
    QTemporaryDir dir;
    const QString cube = dir.filePath(QStringLiteral("cube.hzpart"));
    saveCube(cube);
    hz::io::DrawingDocumentSpec spec;
    spec.partPath = cube.toStdString();
    const std::string sheetPath = dir.filePath(QStringLiteral("cube.hzdwg")).toStdString();
    ASSERT_TRUE(hz::io::DrawingDocumentIO::save(sheetPath, spec));
    const hz::math::Vec2 before = centreOf(cubeLayout(cube).views[0]);
    {
        StandInHost host;
        hz::ui::DrawingWorkbench workbench(host);
        ASSERT_TRUE(workbench.open(QString::fromStdString(sheetPath)));
        hz::doc::Document& sheet = *host.tabs.back().first;
        note(sheet, before, "IN FRONT", "0");
        std::string error;
        ASSERT_TRUE(workbench.save(sheet, sheetPath, &error)) << error;
    }
    // Closed; the part grows.
    {
        hz::doc::Document part;
        ASSERT_TRUE(hz::io::NativeFormat::load(cube.toStdString(), part));
        ASSERT_TRUE(part.featureTree().feature(0)->setParameter("depth", 25.0));
        ASSERT_TRUE(part.rebuildModel());
        ASSERT_TRUE(hz::io::NativeFormat::save(cube.toStdString(), part));
    }
    const hz::math::Vec2 after = centreOf(cubeLayout(cube).views[0]);
    ASSERT_GT(std::hypot(after.x - before.x, after.y - before.y), 1.0) << "Front moved";

    StandInHost host;
    hz::ui::DrawingWorkbench workbench(host);
    ASSERT_TRUE(workbench.open(QString::fromStdString(sheetPath)));
    const auto found = notesSaying(*host.tabs.back().first, "IN FRONT");
    ASSERT_EQ(found.size(), 1u);
    EXPECT_NEAR(found[0]->position().x, after.x, 1e-9) << "at Front's centre, where it is now";
    EXPECT_NEAR(found[0]->position().y, after.y, 1e-9);
}

// A sheet keeps what it was drawn from: an edit draws it again from that,
// without reading its part again (an assembly's parts were all read, and
// rebuilt, on every edit). It is read again when the part changes.
TEST(WorkbenchesTest, ASheetKeepsWhatItWasDrawnFromUntilItChanges) {
    QTemporaryDir dir;
    const QString cube = dir.filePath(QStringLiteral("cube.hzpart"));
    saveCube(cube);
    hz::io::DrawingDocumentSpec spec;
    spec.partPath = cube.toStdString();
    const std::string sheetPath = dir.filePath(QStringLiteral("cube.hzdwg")).toStdString();
    ASSERT_TRUE(hz::io::DrawingDocumentIO::save(sheetPath, spec));
    StandInHost host;
    hz::ui::DrawingWorkbench workbench(host);
    ASSERT_TRUE(workbench.open(QString::fromStdString(sheetPath)));
    hz::doc::Document& sheet = *host.tabs.back().first;
    const hz::math::Vec2 front = centreOf(cubeLayout(cube).views[0]);

    // The part's file gone: a view still moves, drawn from what the sheet keeps.
    ASSERT_TRUE(QFile::remove(cube));
    workbench.onMoveView();
    ASSERT_NE(host.tool, nullptr);
    for (const hz::math::Vec2& at : {front, hz::math::Vec2{front.x + 5.0, front.y}}) {
        QMouseEvent press(QEvent::MouseButtonPress, QPointF(0, 0), QPointF(0, 0), Qt::LeftButton,
                          Qt::LeftButton, Qt::NoModifier);
        host.tool->mousePressEvent(&press, at);
        QMouseEvent release(QEvent::MouseButtonRelease, QPointF(0, 0), QPointF(0, 0),
                            Qt::LeftButton, Qt::NoButton, Qt::NoModifier);
        host.tool->mouseReleaseEvent(&release, at);
    }
    QCoreApplication::processEvents();
    EXPECT_TRUE(sheet.isDirty()) << "moved, and drawn again";
    EXPECT_TRUE(host.fileErrors.empty());

    // Its part changed (here: gone): read again, and said.
    workbench.refreshDrawingsOf(cube.toStdString());
    ASSERT_FALSE(host.statuses.empty());
    EXPECT_TRUE(host.statuses.back().contains(QStringLiteral("could not be drawn again")))
        << host.statuses.back().toStdString();
}

// A part's commands through a stand-in host (the window's part commands,
// moved out of it): what one needs and the part has not got is said where the
// host shows messages, and nothing is added.
TEST(WorkbenchesTest, APartCommandSaysWhatThePartLacks) {
    StandInHost host;
    host.backing.setType(hz::doc::DocumentType::Part);
    hz::ui::PartCommands commands(host);

    commands.onFillet();
    EXPECT_TRUE(host.currentStatus().contains(QStringLiteral("works on a body")))
        << host.currentStatus().toStdString();
    commands.onBooleanUnion();
    EXPECT_TRUE(host.currentStatus().contains(QStringLiteral("fewer than two")))
        << host.currentStatus().toStdString();
    commands.onLoft();
    EXPECT_TRUE(host.currentStatus().contains(QStringLiteral("two or more sketches")))
        << host.currentStatus().toStdString();

    // In an assembly's tab, not at all: no form is shown.
    host.assembly = std::make_shared<hz::doc::AssemblyDocument>();
    commands.onPrimitiveBox();
    EXPECT_TRUE(host.currentStatus().contains(QStringLiteral("works on a part")))
        << host.currentStatus().toStdString();
    EXPECT_TRUE(host.added.empty());
    EXPECT_EQ(host.backing.featureTree().featureCount(), 0u);
}

// A box as its form says: its sizes, and where it stands.
TEST(WorkbenchesTest, ABoxIsAddedAsItsFormSays) {
    StandInHost host;
    host.backing.setType(hz::doc::DocumentType::Part);
    hz::ui::PartCommands commands(host);
    ASSERT_TRUE(answering(
        QStringLiteral("Box"),
        [](QDialog& form) {
            setField(form, "size0", 20.0);
            setField(form, "size1", 10.0);
            setField(form, "size2", 5.0);
            setField(form, "atZ", 3.0);
        },
        [&] { commands.onPrimitiveBox(); }));

    ASSERT_EQ(host.added, std::vector<QString>{QStringLiteral("Box")});
    const hz::topo::Solid* box = host.backing.solid();
    ASSERT_NE(box, nullptr);
    const auto measured = hz::model::MassPropertiesCalculator::compute(*box);
    EXPECT_NEAR(measured.volume, 1000.0, 1e-6);
    EXPECT_NEAR(measured.centerOfMass.z, 5.5, 1e-6) << "standing at z = 3, 5 deep";
}

// Extrude takes the sketch being edited, and finishes it (not a sketch made
// of the drawing), as far as its form says.
TEST(WorkbenchesTest, AnExtrudeTakesTheSketchEditedAndFinishesIt) {
    StandInHost host;
    hz::doc::Document& part = host.backing;
    part.setType(hz::doc::DocumentType::Part);
    auto sketch = std::make_shared<hz::doc::Sketch>();
    const std::vector<hz::math::Vec2> corners = {{0, 0}, {20, 0}, {20, 10}, {0, 10}};
    for (size_t i = 0; i < corners.size(); ++i) {
        sketch->addEntity(
            std::make_shared<hz::draft::DraftLine>(corners[i], corners[(i + 1) % corners.size()]));
    }
    part.undoStack().push(std::make_unique<hz::doc::AddSketchCommand>(part, sketch));
    part.editSketch(sketch);

    hz::ui::PartCommands commands(host);
    ASSERT_TRUE(answering(
        QStringLiteral("Extrude"), [](QDialog& form) { setField(form, "size", 5.0); },
        [&] { commands.onExtrudeSketch(); }));

    EXPECT_EQ(host.sketchesFinished, 1);
    EXPECT_EQ(part.editedSketch(), nullptr);
    EXPECT_EQ(host.chosen, sketch) << "the sketch finished is the one chosen";
    ASSERT_EQ(host.added, std::vector<QString>{QStringLiteral("Extrude")});
    EXPECT_EQ(part.sketches().size(), 1u) << "no sketch made of the drawing";
    ASSERT_NE(part.solid(), nullptr);
    EXPECT_NEAR(hz::model::MassPropertiesCalculator::compute(*part.solid()).volume, 1000.0, 1e-6);
}

// The drawing extruded is copied into the sketch made of it. The sketch
// shared the drawing's lines: a Move in the drawing after changed the
// extrude's profile, with no rebuild, and a grip edit did not. Extruded again
// unchanged, the drawing takes that sketch again.
TEST(WorkbenchesTest, ASketchMadeOfTheDrawingIsACopyOfIt) {
    StandInHost host;
    hz::doc::Document& part = host.backing;
    part.setType(hz::doc::DocumentType::Part);
    const std::vector<hz::math::Vec2> corners = {{0, 0}, {20, 0}, {20, 10}, {0, 10}};
    std::vector<uint64_t> lines;
    for (size_t i = 0; i < corners.size(); ++i) {
        auto line =
            std::make_shared<hz::draft::DraftLine>(corners[i], corners[(i + 1) % corners.size()]);
        lines.push_back(line->id());
        part.draftDocument().addEntity(line);
    }
    hz::ui::PartCommands commands(host);
    const auto extrude = [&] {
        return answering(
            QStringLiteral("Extrude"), [](QDialog& form) { setField(form, "size", 5.0); },
            [&] { commands.onExtrudeSketch(); });
    };
    ASSERT_TRUE(extrude());
    ASSERT_EQ(part.sketches().size(), 1u);
    const auto profile = part.sketches().front();
    ASSERT_EQ(profile->entities().size(), 4u);
    for (const auto& entity : profile->entities()) {
        EXPECT_EQ(part.draftDocument().findEntity(entity->id()), nullptr) << "a copy";
    }

    part.undoStack().push(std::make_unique<hz::doc::MoveEntityCommand>(
        part.draftDocument(), lines, hz::math::Vec2(0, 3), part.constraintSystem()));
    const auto* first = dynamic_cast<const hz::draft::DraftLine*>(profile->entities()[0].get());
    ASSERT_NE(first, nullptr);
    EXPECT_DOUBLE_EQ(first->start().y, 0.0) << "the profile as it was extruded";
    part.undoStack().undo();

    ASSERT_TRUE(extrude());
    EXPECT_EQ(part.sketches().size(), 1u) << "the drawing as it was: the same sketch";
    EXPECT_EQ(host.added.size(), 2u);
}

// A feature checked in a pattern's form and taken away for good while the
// form is open (a build that finished withdrew it) is looked for again when
// the form closes: the pattern is not added. The form's list held the
// feature, freed by then, and read its ID.
TEST(WorkbenchesTest, APatternOfAFeatureGoneWhileItsFormIsOpenIsNotAdded) {
    StandInHost host;
    hz::doc::Document& part = host.backing;
    part.setType(hz::doc::DocumentType::Part);
    part.featureTree().addFeature(hz::doc::PrimitiveFeature::makeBox(10, 10, 10));
    part.undoStack().push(std::make_unique<hz::doc::AddFeatureCommand>(
        part, hz::doc::PrimitiveFeature::makeBox(5, 5, 20), nullptr));
    ASSERT_TRUE(part.rebuildModel());

    hz::ui::PartCommands commands(host);
    ASSERT_TRUE(answering(
        QStringLiteral("Linear Pattern"),
        [&part](QDialog& form) {
            auto* list = form.findChild<QListWidget*>(QStringLiteral("features"));
            ASSERT_NE(list, nullptr);
            ASSERT_EQ(list->count(), 2);
            list->item(1)->setCheckState(Qt::Checked);
            // The second box undone, and a step after it: gone for good.
            part.undoStack().undo();
            part.undoStack().push(std::make_unique<hz::doc::AddEntityCommand>(
                part.draftDocument(), std::make_shared<hz::draft::DraftLine>(
                                          hz::math::Vec2(0, 0), hz::math::Vec2(1, 0))));
        },
        [&] { commands.onLinearPattern(); }));
    EXPECT_TRUE(host.added.empty()) << "not added";
    EXPECT_EQ(part.featureTree().featureCount(), 1u);
    EXPECT_TRUE(host.currentStatus().contains(QStringLiteral("gone")))
        << host.currentStatus().toStdString();
}

// Extrude again with the sketch chosen in the list, not edited: it is taken
// as it is, and the extrude cuts it through the first.
TEST(WorkbenchesTest, AnExtrudeTakesTheSketchChosen) {
    StandInHost host;
    hz::doc::Document& part = host.backing;
    part.setType(hz::doc::DocumentType::Part);
    part.featureTree().addFeature(hz::doc::PrimitiveFeature::makeBox(20, 20, 20));
    ASSERT_TRUE(part.rebuildModel());
    auto hole = std::make_shared<hz::doc::Sketch>();
    const std::vector<hz::math::Vec2> corners = {{5, 5}, {15, 5}, {15, 15}, {5, 15}};
    for (size_t i = 0; i < corners.size(); ++i) {
        hole->addEntity(
            std::make_shared<hz::draft::DraftLine>(corners[i], corners[(i + 1) % corners.size()]));
    }
    part.undoStack().push(std::make_unique<hz::doc::AddSketchCommand>(part, hole));
    part.editSketch(nullptr);
    host.chosen = hole;

    hz::ui::PartCommands commands(host);
    ASSERT_TRUE(answering(
        QStringLiteral("Extrude"),
        [](QDialog& form) {
            auto* goes = form.findChild<QComboBox*>(QStringLiteral("extent"));
            ASSERT_NE(goes, nullptr);
            goes->setCurrentIndex(2);  // through all
            auto* result = form.findChild<QComboBox*>(QStringLiteral("bodyOperation"));
            ASSERT_NE(result, nullptr);
            result->setCurrentIndex(
                result->findData(static_cast<int>(hz::doc::BodyOperation::Cut)));
            ASSERT_GE(result->currentIndex(), 0);
        },
        [&] { commands.onExtrudeSketch(); }));

    EXPECT_EQ(host.sketchesFinished, 0);
    ASSERT_EQ(host.added, std::vector<QString>{QStringLiteral("Extrude")});
    ASSERT_NE(part.solid(), nullptr);
    EXPECT_NEAR(hz::model::MassPropertiesCalculator::compute(*part.solid()).volume,
                20.0 * 20 * 20 - 10.0 * 10 * 20, 1e-6);
}

// ---------------------------------------------------------------------------
// The 2D drafting commands, on the selection
// ---------------------------------------------------------------------------

namespace {

/// The lines drawn and selected by drawThree: two on the current layer, and
/// one on a locked layer.
struct ThreeLines {
    uint64_t a = 0;
    uint64_t b = 0;
    uint64_t locked = 0;
};

ThreeLines drawThree(StandInHost& host) {
    hz::doc::Document& document = *host.currentDocument();
    hz::draft::LayerProperties lockedLayer;
    lockedLayer.name = "Locked";
    lockedLayer.locked = true;
    document.layerManager().addLayer(lockedLayer);
    using hz::math::Vec2;
    const auto a = std::make_shared<hz::draft::DraftLine>(Vec2(0, 0), Vec2(1, 0));
    const auto b = std::make_shared<hz::draft::DraftLine>(Vec2(0, 1), Vec2(1, 1));
    const auto c = std::make_shared<hz::draft::DraftLine>(Vec2(0, 2), Vec2(1, 2));
    c->setLayer("Locked");
    auto& sel = host.viewport().selectionManager();
    for (const auto& line : {a, b, c}) {
        document.activeDrawing().addEntity(line);
        sel.select(line->id());
    }
    return {a->id(), b->id(), c->id()};
}

std::size_t entityCount(StandInHost& host) {
    return host.currentDocument()->activeDrawing().entities().size();
}

}  // namespace

// Duplicate, Group and Ungroup, Cut and Paste through the host alone: what
// is on a locked layer is left as it is, and what a command makes is what is
// selected after it.
TEST(WorkbenchesTest, TheDraftingCommandsWorkOnTheSelectionThroughTheirHost) {
    StandInHost host;
    hz::ui::DraftingCommands drafting(host);
    const ThreeLines lines = drawThree(host);
    auto& sel = host.viewport().selectionManager();
    hz::doc::Document& document = *host.currentDocument();

    drafting.onDuplicate();
    EXPECT_EQ(entityCount(host), 5u) << "the two on an unlocked layer, copied";
    EXPECT_EQ(sel.selectedIds().size(), 2u) << "the copies selected";
    EXPECT_FALSE(sel.isSelected(lines.a));
    EXPECT_EQ(host.selectionChanges, 1);
    document.undoStack().undo();
    EXPECT_EQ(entityCount(host), 3u);

    for (const uint64_t id : {lines.a, lines.b, lines.locked}) sel.select(id);
    const auto groupOf = [&document](uint64_t id) {
        return document.activeDrawing().findEntity(id)->groupId();
    };
    drafting.onGroupEntities();
    EXPECT_NE(groupOf(lines.a), 0u);
    EXPECT_EQ(groupOf(lines.a), groupOf(lines.b));
    EXPECT_EQ(groupOf(lines.locked), 0u) << "a locked layer's line is left out";
    drafting.onUngroupEntities();
    EXPECT_EQ(groupOf(lines.a), 0u);

    drafting.onCut();
    EXPECT_EQ(entityCount(host), 1u) << "the locked layer's line stays";
    EXPECT_TRUE(sel.selectedIds().empty());
    EXPECT_TRUE(drafting.clipboard().hasContent());
    drafting.onPaste();
    ASSERT_NE(host.tool, nullptr);
    EXPECT_EQ(host.tool->name(), "Paste");
}

// A polar array of a rectangle keeps its shape in every copy: a copy turned
// off the axes is a closed polyline through the turned corners, and the one
// turned half round is still a rectangle. Each was a box across two of the
// turned corners.
TEST(WorkbenchesTest, APolarArrayKeepsARectanglesShape) {
    using hz::math::Vec2;
    StandInHost host;
    hz::ui::DraftingCommands drafting(host);
    hz::doc::Document& document = *host.currentDocument();
    const auto rect = std::make_shared<hz::draft::DraftRectangle>(Vec2(2, 0), Vec2(6, 2));
    document.activeDrawing().addEntity(rect);
    host.viewport().selectionManager().select(rect->id());

    // As the form has it: six round the origin.
    ASSERT_TRUE(answering(
        "Polar Array", [](QDialog&) {}, [&] { drafting.onPolarArray(); }));
    const auto& entities = document.activeDrawing().entities();
    ASSERT_EQ(entities.size(), 6u);
    const auto corners = rect->corners();
    for (size_t i = 1; i < 6; ++i) {
        SCOPED_TRACE(i);
        const double angle = static_cast<double>(i) * hz::math::kPi / 3.0;
        const auto turned = [angle](const Vec2& p) {
            return Vec2(p.x * std::cos(angle) - p.y * std::sin(angle),
                        p.x * std::sin(angle) + p.y * std::cos(angle));
        };
        if (i == 3) {
            const auto* half = dynamic_cast<const hz::draft::DraftRectangle*>(entities[i].get());
            ASSERT_NE(half, nullptr) << "half round, a rectangle";
            EXPECT_LT(half->corners()[0].distanceTo(Vec2(-6, -2)), 1e-9);
            EXPECT_LT(half->corners()[2].distanceTo(Vec2(-2, 0)), 1e-9);
            continue;
        }
        const auto* poly = dynamic_cast<const hz::draft::DraftPolyline*>(entities[i].get());
        ASSERT_NE(poly, nullptr);
        EXPECT_TRUE(poly->closed());
        ASSERT_EQ(poly->points().size(), 4u);
        for (size_t k = 0; k < 4; ++k) {
            EXPECT_LT(poly->points()[k].distanceTo(turned(corners[k])), 1e-9) << k;
        }
    }
}

// Arrays, a block made and exploded, a block inserted, and the dimension
// style, each through its form and the host alone.
TEST(WorkbenchesTest, TheDraftingFormsWorkThroughTheirHost) {
    StandInHost host;
    hz::ui::DraftingCommands drafting(host);
    const ThreeLines lines = drawThree(host);
    auto& sel = host.viewport().selectionManager();
    hz::doc::Document& document = *host.currentDocument();

    // Three by three, as the form has it: each unlocked line eight more times.
    ASSERT_TRUE(answering(
        "Rectangular Array", [](QDialog&) {}, [&] { drafting.onRectangularArray(); }));
    EXPECT_EQ(entityCount(host), 3u + 16u);
    EXPECT_EQ(sel.selectedIds().size(), 16u);
    document.undoStack().undo();
    sel.clearSelection();
    sel.select(lines.a);
    ASSERT_TRUE(answering(
        "Polar Array", [](QDialog&) {}, [&] { drafting.onPolarArray(); }));
    EXPECT_EQ(entityCount(host), 3u + 5u) << "six round, the line among them";
    document.undoStack().undo();

    // A block of two lines, where they were; exploded, they are back.
    sel.clearSelection();
    sel.select(lines.a);
    sel.select(lines.b);
    ASSERT_TRUE(answering(
        "Create Block",
        [](QDialog& form) {
            auto* name = form.findChild<QLineEdit*>(QStringLiteral("blockName"));
            ASSERT_NE(name, nullptr);
            name->setText(QStringLiteral("Pair"));
        },
        [&] { drafting.onCreateBlock(); }));
    EXPECT_EQ(entityCount(host), 2u) << "the locked layer's line, and the block";
    ASSERT_EQ(sel.selectedIds().size(), 1u) << "the block selected";
    drafting.onExplode();
    EXPECT_EQ(entityCount(host), 3u);
    EXPECT_EQ(sel.selectedIds().size(), 2u) << "what it held selected";

    ASSERT_TRUE(answering(
        "Insert Block", [](QDialog&) {}, [&] { drafting.onInsertBlock(); }));
    ASSERT_NE(host.tool, nullptr);
    EXPECT_EQ(host.tool->name(), "Insert Block");

    // The text height doubled, as one step to undo.
    const double was = document.activeDrawing().dimensionStyle().textHeight;
    ASSERT_TRUE(answering(
        "Dimension Style", [was](QDialog& form) { setField(form, "textHeight", was * 2.0); },
        [&] { drafting.onDimensionStyle(); }));
    EXPECT_DOUBLE_EQ(document.activeDrawing().dimensionStyle().textHeight, was * 2.0);
    document.undoStack().undo();
    EXPECT_DOUBLE_EQ(document.activeDrawing().dimensionStyle().textHeight, was);
}
