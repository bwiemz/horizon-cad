// Workbenches (Phase 146): the assembly commands, moved out of MainWindow,
// work through WorkbenchHost alone. Here they run against a stand-in host
// with no window at all, which is what the interface is for.

#include <gtest/gtest.h>

#include <QTemporaryDir>
#include <memory>
#include <string>
#include <vector>

#include "horizon/document/AssemblyDocument.h"
#include "horizon/document/Document.h"
#include "horizon/document/DocumentManager.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/fileio/NativeFormat.h"
#include "horizon/topology/Solid.h"
#include "horizon/ui/AssemblyTreePanel.h"
#include "horizon/ui/AssemblyWorkbench.h"
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
    std::vector<QString> statuses;
    int rebuilds = 0;

    QWidget* dialogParent() override { return &m_viewport; }
    hz::doc::Document* currentDocument() override { return &backing; }
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
    void reportFileError(const QString& /*summary*/, const std::string& /*path*/,
                         const std::string& /*reason*/) override {}
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
