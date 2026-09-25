// Workbenches (Phase 146): the assembly commands, moved out of MainWindow,
// work through WorkbenchHost alone, as do the drawing sheets (Phase 148).
// Here they run against a stand-in host with no window at all, which is what
// the interface is for.

#include <gtest/gtest.h>

#include <QApplication>
#include <QComboBox>
#include <QDialog>
#include <QFile>
#include <QTemporaryDir>
#include <QTimer>
#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "horizon/document/AssemblyDocument.h"
#include "horizon/document/Document.h"
#include "horizon/document/DocumentManager.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftText.h"
#include "horizon/fileio/DrawingDocumentIO.h"
#include "horizon/fileio/NativeFormat.h"
#include "horizon/math/BoundingBox.h"
#include "horizon/topology/Solid.h"
#include "horizon/ui/AssemblyTreePanel.h"
#include "horizon/ui/AssemblyWorkbench.h"
#include "horizon/ui/DrawingWorkbench.h"
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
    int rebuilds = 0;

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
    bool openPath(const QString& /*fileName*/) override { return false; }
    void addTab(std::shared_ptr<hz::doc::Document> document, const QString& title) override {
        tabs.emplace_back(std::move(document), title);
    }
    void reportFileError(const QString& /*summary*/, const std::string& path,
                         const std::string& reason) override {
        fileErrors.push_back(path + ": " + reason);
    }
    bool onWorker(bool /*large*/) override { return false; }
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
