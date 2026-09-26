// Phase 159: an assembly placed as a component of another, resolved from
// its file recursively, and refused when it places itself.

#include <gtest/gtest.h>

#include <filesystem>
#include <memory>
#include <string>

#include "horizon/document/AssemblyDocument.h"
#include "horizon/document/AssemblyMates.h"
#include "horizon/document/BillOfMaterials.h"
#include "horizon/document/Document.h"
#include "horizon/document/DocumentManager.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/fileio/NativeFormat.h"
#include "horizon/math/Mat4.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/topology/Solid.h"

using hz::doc::AssemblyDocument;
using hz::doc::ComponentInstance;
using hz::doc::ComponentState;
using hz::doc::Document;
using hz::doc::DocumentManager;
using hz::io::NativeFormat;
using hz::math::Mat4;
using hz::math::Vec3;

namespace fs = std::filesystem;

namespace {

void wireToNativeFormat(DocumentManager& manager) {
    manager.setPartLoader(
        [](const std::string& path, Document& doc) { return NativeFormat::load(path, doc); });
    manager.setMeshLoader([](const std::string& path) { return NativeFormat::loadPartMesh(path); });
    manager.setAssemblyLoader([](const std::string& path, AssemblyDocument& doc) {
        return NativeFormat::loadAssembly(path, doc);
    });
}

/// A folder of files for one test, removed after it.
struct Folder {
    fs::path dir;
    explicit Folder(const std::string& name) : dir(fs::temp_directory_path() / name) {
        fs::remove_all(dir);
        fs::create_directories(dir);
    }
    ~Folder() {
        std::error_code ec;
        fs::remove_all(dir, ec);
    }
    Folder(const Folder&) = delete;
    Folder& operator=(const Folder&) = delete;
    std::string file(const std::string& name) const { return (dir / name).string(); }
};

/// A 10 mm block saved at @p path.
void saveBlock(const std::string& path) {
    Document part;
    part.setType(hz::doc::DocumentType::Part);
    part.featureTree().addFeature(hz::doc::PrimitiveFeature::makeBox(10, 10, 10));
    ASSERT_TRUE(part.rebuildModel());
    ASSERT_TRUE(NativeFormat::save(path, part));
}

/// An assembly saved at @p path placing @p files, each at its offset.
void saveAssembly(const std::string& path, const std::vector<std::pair<std::string, Vec3>>& files) {
    AssemblyDocument assembly;
    for (const auto& [file, at] : files) {
        ComponentInstance comp;
        comp.name = fs::path(file).stem().string();
        comp.partPath = file;
        comp.transform = Mat4::translation(at);
        assembly.addComponent(comp);
    }
    ASSERT_TRUE(NativeFormat::saveAssembly(path, assembly));
}

double volumeOf(const hz::topo::Solid& solid) {
    return hz::model::MassPropertiesCalculator::compute(solid).volume;
}

}  // namespace

// Two blocks in a subassembly, placed in a top assembly beside a third: the
// subassembly resolves from its file into its components, its mesh theirs
// merged and its solid theirs gathered, named after them.
TEST(SubassembliesTest, AnAssemblyPlacedInAnotherResolvesFromItsFile) {
    Folder folder("hz_test_subassemblies");
    saveBlock(folder.file("block.hzpart"));
    saveAssembly(folder.file("pair.hzasm"), {{folder.file("block.hzpart"), Vec3(0, 0, 0)},
                                             {folder.file("block.hzpart"), Vec3(0, 0, 10)}});
    DocumentManager manager;
    wireToNativeFormat(manager);

    ComponentInstance pair;
    pair.partPath = "pair.hzasm";
    pair.transform = Mat4::translation(Vec3(20, 0, 0));
    ASSERT_TRUE(pair.isAssembly());
    std::string why;
    ASSERT_TRUE(manager.resolveComponent(pair, ComponentState::Resolved, folder.dir.string(), &why))
        << why;
    ASSERT_NE(pair.resolvedAssembly, nullptr);
    EXPECT_EQ(pair.resolvedAssembly->components().size(), 2u);
    ASSERT_NE(pair.cachedMesh, nullptr);
    EXPECT_TRUE(pair.cachedMesh->hasFaces());
    const hz::topo::Solid* solid = pair.solid();
    ASSERT_NE(solid, nullptr);
    EXPECT_NEAR(volumeOf(*solid), 2000.0, 1e-6) << "both blocks";
    const uint64_t lower = pair.resolvedAssembly->components()[0].id;
    bool named = false;
    for (const auto& face : solid->faces()) {
        named = named || face.topoId.tag().rfind(AssemblyDocument::namePrefix(lower), 0) == 0;
    }
    EXPECT_TRUE(named) << "its faces named after its own components";

    // Lightweight: its mesh, and no solid.
    ComponentInstance light;
    light.partPath = "pair.hzasm";
    ASSERT_TRUE(manager.resolveComponent(light, ComponentState::Lightweight, folder.dir.string()));
    EXPECT_NE(light.cachedMesh, nullptr);
    EXPECT_EQ(light.solid(), nullptr);
}

// An assembly that places itself, however deep, is refused, and the chain
// is said.
TEST(SubassembliesTest, AnAssemblyInsideItselfIsRefused) {
    Folder folder("hz_test_subassembly_cycle");
    saveBlock(folder.file("block.hzpart"));
    // outer places inner; inner places outer.
    saveAssembly(folder.file("inner.hzasm"), {{folder.file("block.hzpart"), Vec3()},
                                              {folder.file("outer.hzasm"), Vec3(0, 20, 0)}});
    saveAssembly(folder.file("outer.hzasm"), {{folder.file("inner.hzasm"), Vec3()}});
    DocumentManager manager;
    wireToNativeFormat(manager);

    ComponentInstance inner;
    inner.partPath = folder.file("inner.hzasm");
    std::string why;
    EXPECT_FALSE(
        manager.resolveComponent(inner, ComponentState::Lightweight, {}, &why,
                                 {DocumentManager::canonicalPath(folder.file("outer.hzasm"))}));
    EXPECT_NE(why.find("inside itself"), std::string::npos) << why;
    EXPECT_NE(why.find("outer > inner > outer"), std::string::npos) << why;

    // Placed in an unsaved assembly, the loop is found in the files.
    ComponentInstance outer;
    outer.partPath = folder.file("outer.hzasm");
    why.clear();
    EXPECT_FALSE(manager.resolveComponent(outer, ComponentState::Lightweight, {}, &why));
    EXPECT_NE(why.find("inside itself"), std::string::npos) << why;
}

// A subassembly's face is mated in its parent by its name there: the
// component's own prefixed ("c<id>/...").
TEST(SubassembliesTest, ASubassemblysFaceIsMated) {
    Folder folder("hz_test_subassembly_mate");
    saveBlock(folder.file("block.hzpart"));
    saveAssembly(folder.file("pair.hzasm"), {{folder.file("block.hzpart"), Vec3(0, 0, 0)},
                                             {folder.file("block.hzpart"), Vec3(0, 0, 10)}});
    DocumentManager manager;
    wireToNativeFormat(manager);

    AssemblyDocument top;
    ComponentInstance base;
    base.partPath = folder.file("block.hzpart");
    const uint64_t baseId = top.addComponent(base);
    ComponentInstance pair;
    pair.partPath = folder.file("pair.hzasm");
    pair.transform = Mat4::translation(Vec3(30, 0, 5));
    const uint64_t pairId = top.addComponent(pair);
    for (auto& comp : top.components()) {
        ASSERT_TRUE(manager.resolveComponent(comp, ComponentState::Resolved));
    }
    const auto* resolvedPair = top.component(pairId);
    const auto* resolvedBase = top.component(baseId);
    const uint64_t lower = resolvedPair->resolvedAssembly->components()[0].id;
    const std::string feature = resolvedBase->resolvedPart->featureTree().feature(0)->featureID();

    // The pair's lower block's bottom on the base's top.
    hz::doc::Mate on;
    on.type = hz::doc::MateType::Coincident;
    on.a = {baseId, hz::topo::TopologyID::fromTag(feature + "/top")};
    on.b = {pairId, hz::topo::TopologyID::fromTag(AssemblyDocument::namePrefix(lower) + feature +
                                                  "/bottom")};
    top.addMate(on);
    std::string why;
    const auto mates = hz::doc::AssemblyMates::gather(top, &why);
    if (!mates) FAIL() << why;
    const auto result = mates->solve();
    ASSERT_EQ(result.status, hz::model::AssemblySolveStatus::Success) << result.message;
    EXPECT_NEAR(result.transforms.at(pairId).transformPoint(Vec3()).z, 10.0, 1e-6)
        << "the pair stands on the base";
}

// Phase 159: the bill of materials at every depth. Top level: a
// subassembly is one line; indented: its own lines under it, numbered after
// it, as many as one of it holds; parts only: every part, multiplied
// through.
TEST(SubassembliesTest, TheBillOfMaterialsHasLevels) {
    Folder folder("hz_test_subassembly_bom");
    saveBlock(folder.file("block.hzpart"));
    saveBlock(folder.file("pin.hzpart"));
    saveAssembly(folder.file("pair.hzasm"), {{folder.file("block.hzpart"), Vec3(0, 0, 0)},
                                             {folder.file("block.hzpart"), Vec3(0, 0, 10)},
                                             {folder.file("pin.hzpart"), Vec3(0, 0, 20)}});
    saveAssembly(folder.file("top.hzasm"), {{folder.file("pin.hzpart"), Vec3(-20, 0, 0)},
                                            {folder.file("pair.hzasm"), Vec3(0, 0, 0)},
                                            {folder.file("pair.hzasm"), Vec3(20, 0, 0)}});
    DocumentManager manager;
    wireToNativeFormat(manager);
    auto top = manager.openAssembly(folder.file("top.hzasm"));
    ASSERT_NE(top, nullptr);
    for (auto& comp : top->components()) {
        ASSERT_TRUE(
            manager.resolveComponent(comp, ComponentState::Lightweight, folder.dir.string()));
    }

    using hz::doc::BomGenerator;
    using hz::doc::BomKind;
    const auto topLevel = BomGenerator::generate(*top, BomKind::TopLevel);
    ASSERT_EQ(topLevel.lines.size(), 2u);
    EXPECT_EQ(topLevel.lines[0].partName, "pin");
    EXPECT_EQ(topLevel.lines[1].partName, "pair");
    EXPECT_EQ(topLevel.lines[1].quantity, 2);
    EXPECT_TRUE(topLevel.lines[1].assembly);

    const auto indented = BomGenerator::generate(*top, BomKind::Indented);
    ASSERT_EQ(indented.lines.size(), 4u);
    EXPECT_EQ(indented.lines[2].index, "2.1");
    EXPECT_EQ(indented.lines[2].partName, "block");
    EXPECT_EQ(indented.lines[2].quantity, 2) << "in one pair";
    EXPECT_EQ(indented.lines[2].level, 1);
    EXPECT_EQ(indented.lines[3].index, "2.2");
    EXPECT_EQ(indented.lines[3].partName, "pin");
    EXPECT_EQ(indented.totalQuantity(), 3) << "the assembly's own: a pin and two pairs";

    const auto parts = BomGenerator::generate(*top, BomKind::PartsOnly);
    ASSERT_EQ(parts.lines.size(), 2u);
    EXPECT_EQ(parts.lines[0].partName, "pin");
    EXPECT_EQ(parts.lines[0].quantity, 3) << "one, and one in each pair";
    EXPECT_EQ(parts.lines[1].partName, "block");
    EXPECT_EQ(parts.lines[1].quantity, 4) << "two in each pair";
}
