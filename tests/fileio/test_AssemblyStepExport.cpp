// An assembly's STEP export (Phase 153) writes each part as the part's own File > Export > STEP
// does (Task 10): its exact counterpart, where it has one, as its faces are, not the facets a
// component no tab shows is built with to be drawn, mated and checked for interference.

#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "horizon/document/AssemblyDocument.h"
#include "horizon/document/Document.h"
#include "horizon/document/DocumentManager.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/fileio/AssemblyStepExport.h"
#include "horizon/fileio/StepFormat.h"
#include "horizon/math/Constants.h"
#include "horizon/math/Mat4.h"
#include "horizon/math/Vec3.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/topology/ExactTolerance.h"
#include "horizon/topology/Solid.h"

using hz::doc::ComponentInstance;
using hz::doc::ComponentState;
using hz::doc::Document;
using hz::doc::PrimitiveFeature;

namespace {

constexpr double kPi = hz::math::kPi;

/// A manager whose parts are primitives, by their file's name: "cylinder.hzpart" a cylinder of
/// radius 5 and height 10, "cone.hzpart" a cone of radii 4 and 1 and height 6. Stable, so exact.
void loadPrimitives(hz::doc::DocumentManager& manager) {
    manager.setPartLoader([](const std::string& path, Document& part) {
        part.setType(hz::doc::DocumentType::Part);
        if (path.find("cylinder.hzpart") != std::string::npos) {
            part.featureTree().addFeature(PrimitiveFeature::makeCylinder(5.0, 10.0));
        } else if (path.find("cone.hzpart") != std::string::npos) {
            part.featureTree().addFeature(PrimitiveFeature::makeCone(4.0, 1.0, 6.0));
        } else {
            return false;
        }
        return true;
    });
}

std::uint64_t place(hz::doc::AssemblyDocument& assembly, const std::string& file,
                    const hz::math::Vec3& at, bool mirrored = false) {
    ComponentInstance component;
    component.name = file;
    component.partPath = "/parts/" + file;
    component.transform = hz::math::Mat4::translation(at);
    component.mirrored = mirrored;
    return assembly.addComponent(component);
}

double volumeOf(const std::vector<std::unique_ptr<hz::topo::Solid>>& solids) {
    double volume = 0.0;
    for (const auto& solid : solids) {
        volume += hz::model::MassPropertiesCalculator::compute(*solid).volume;
    }
    return volume;
}

}  // namespace

// Two parts made of primitives, one of them resolved already for its mates, as a component no tab
// shows is (built without its exact counterpart), the other not read at all: both go out exact,
// the cylinder's side a CYLINDRICAL_SURFACE and the cone's a CONICAL_SURFACE, no face in facets,
// and they read back to their closed forms. Each was built exactly for the export alone, in a copy
// of its own: the resolved part is as it was, built in facets alone (Ruling 20), and the other
// component is not resolved.
TEST(AssemblyStepExport, TwoPrimitivePartsExportExact) {
    hz::doc::DocumentManager manager;
    loadPrimitives(manager);
    hz::doc::AssemblyDocument assembly;
    const std::uint64_t cylinder = place(assembly, "cylinder.hzpart", {0, 0, 0});
    const std::uint64_t cone = place(assembly, "cone.hzpart", {20, 0, 0});
    ASSERT_TRUE(manager.resolveComponent(*assembly.component(cylinder), ComponentState::Resolved));
    const auto mated = assembly.component(cylinder)->resolvedPart;
    ASSERT_NE(mated, nullptr);
    ASSERT_NE(mated->solid(), nullptr);
    EXPECT_EQ(mated->exactSolid(), nullptr) << "built for its mates alone, in facets";
    const auto builds = mated->builds();
    const hz::topo::Solid* faceted = mated->solid();

    const hz::io::AssemblyStepExport gathered = hz::io::gatherAssemblyStep(assembly, manager);
    EXPECT_TRUE(gathered.unread.empty());
    ASSERT_EQ(gathered.parts.size(), 2u);
    ASSERT_EQ(gathered.occurrences.size(), 2u);
    for (const auto& part : gathered.parts) {
        ASSERT_EQ(part.bodies.size(), 1u) << part.name;
        EXPECT_TRUE(hz::topo::hasExactFaces(*part.bodies.front())) << part.name;
    }
    EXPECT_EQ(mated->builds(), builds) << "the part is not built again";
    EXPECT_EQ(mated->solid(), faceted);
    EXPECT_EQ(mated->exactSolid(), nullptr) << "nor given its counterpart";
    EXPECT_TRUE(mated->needsBuild()) << "still built for its components alone";
    EXPECT_EQ(assembly.component(cone)->resolvedPart, nullptr) << "nor is the other resolved";

    hz::io::StepFormat::WriteReport report;
    const std::string text = hz::io::StepFormat::assemblyToString(
        "Assembly", gathered.parts, gathered.occurrences, {}, &report);
    EXPECT_TRUE(report.faceted.empty());
    EXPECT_NE(text.find("CYLINDRICAL_SURFACE"), std::string::npos);
    EXPECT_NE(text.find("CONICAL_SURFACE"), std::string::npos);
    EXPECT_EQ(text.find("B_SPLINE_SURFACE"), std::string::npos) << "no face in facets";

    const auto back = hz::io::StepFormat::fromString(text);
    ASSERT_EQ(back.size(), 2u) << hz::io::StepFormat::lastError();
    const double volume = kPi * 25.0 * 10.0 + kPi * 6.0 * (16.0 + 4.0 + 1.0) / 3.0;
    EXPECT_NEAR(volumeOf(back), volume, 1e-9 * volume);
}

// A mirrored component's part (Phase 162) is a part of its own, its mirror: exact too, the
// part's exact counterpart mirrored, written beside the part it mirrors.
TEST(AssemblyStepExport, AMirroredPrimitivePartExportsExact) {
    hz::doc::DocumentManager manager;
    loadPrimitives(manager);
    hz::doc::AssemblyDocument assembly;
    place(assembly, "cone.hzpart", {0, 0, 0});
    place(assembly, "cone.hzpart", {20, 0, 0}, /*mirrored=*/true);

    const hz::io::AssemblyStepExport gathered = hz::io::gatherAssemblyStep(assembly, manager);
    ASSERT_EQ(gathered.parts.size(), 2u);
    EXPECT_EQ(gathered.parts.back().name, "cone (mirrored)");
    for (const auto& part : gathered.parts) {
        ASSERT_EQ(part.bodies.size(), 1u) << part.name;
        EXPECT_TRUE(hz::topo::hasExactFaces(*part.bodies.front())) << part.name;
    }
    hz::io::StepFormat::WriteReport report;
    const std::string text = hz::io::StepFormat::assemblyToString(
        "Assembly", gathered.parts, gathered.occurrences, {}, &report);
    EXPECT_TRUE(report.faceted.empty());
    EXPECT_EQ(text.find("B_SPLINE_SURFACE"), std::string::npos) << "no face in facets";
    const auto back = hz::io::StepFormat::fromString(text);
    ASSERT_EQ(back.size(), 2u) << hz::io::StepFormat::lastError();
    const double cone = kPi * 6.0 * (16.0 + 4.0 + 1.0) / 3.0;
    EXPECT_NEAR(volumeOf(back), 2.0 * cone, 2e-9 * cone) << "the mirror faces out";
}

// A part a tab shows is built with its exact counterpart already: the export writes that very
// counterpart, and builds nothing.
TEST(AssemblyStepExport, APartATabShowsIsWrittenAsItsTabBuiltIt) {
    hz::doc::DocumentManager manager;
    loadPrimitives(manager);
    const auto shown = manager.openPart("/parts/cylinder.hzpart");
    ASSERT_NE(shown, nullptr);
    ASSERT_TRUE(shown->rebuildModel()) << shown->lastBuildMessage();
    ASSERT_NE(shown->exactSolid(), nullptr) << shown->exactNote();
    const auto builds = shown->builds();
    hz::doc::AssemblyDocument assembly;
    place(assembly, "cylinder.hzpart", {0, 0, 0});

    const hz::io::AssemblyStepExport gathered = hz::io::gatherAssemblyStep(assembly, manager);
    ASSERT_EQ(gathered.parts.size(), 1u);
    ASSERT_EQ(gathered.parts.front().bodies.size(), 1u);
    EXPECT_EQ(gathered.parts.front().bodies.front(), shown->exactSolid()) << "the tab's own";
    EXPECT_EQ(shown->builds(), builds);
}

// An export made ready on the window's thread (prepareAssemblyStep) is built and gathered on
// another (finishAssemblyStep), while the window rebuilds the part: it reads only what is its own,
// copies and counterparts, never the part.
TEST(AssemblyStepExport, APreparedExportIsBuiltOnAWorkerWhileThePartChanges) {
    hz::doc::DocumentManager manager;
    loadPrimitives(manager);
    hz::doc::AssemblyDocument assembly;
    const std::uint64_t cylinder = place(assembly, "cylinder.hzpart", {0, 0, 0});
    place(assembly, "cylinder.hzpart", {30, 0, 0}, /*mirrored=*/true);
    ASSERT_TRUE(manager.resolveComponent(*assembly.component(cylinder), ComponentState::Resolved));
    const auto part = assembly.component(cylinder)->resolvedPart;
    ASSERT_NE(part, nullptr);

    hz::io::AssemblyStepJob job = hz::io::prepareAssemblyStep(assembly, manager);
    EXPECT_EQ(job.builds(), 1u) << "one copy of the one part, mirrored or not";
    hz::io::AssemblyStepExport gathered;
    std::thread worker(
        [&job, &gathered] { gathered = hz::io::finishAssemblyStep(std::move(job)); });
    for (int k = 0; k < 3; ++k) {
        part->featureTree().markChanged();
        part->rebuildModel(hz::doc::BuildOptions{});
    }
    worker.join();
    ASSERT_EQ(gathered.parts.size(), 2u);
    ASSERT_EQ(gathered.occurrences.size(), 2u);
    for (const auto& written : gathered.parts) {
        ASSERT_EQ(written.bodies.size(), 1u) << written.name;
        EXPECT_TRUE(hz::topo::hasExactFaces(*written.bodies.front())) << written.name;
        EXPECT_NE(written.bodies.front(), part->exactSolid()) << "its own copy's";
    }
}
