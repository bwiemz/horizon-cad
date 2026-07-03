#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/SketchPlane.h"
#include "horizon/modeling/Loft.h"
#include "horizon/topology/Solid.h"
#include "horizon/topology/TopologyID.h"

using namespace hz::model;
using namespace hz::topo;
using namespace hz::draft;
using hz::math::Vec2;
using hz::math::Vec3;

namespace {

// A square profile of side @p s centered at the local origin.
std::vector<std::shared_ptr<DraftEntity>> squareProfile(double s) {
    const double h = s * 0.5;
    std::vector<std::shared_ptr<DraftEntity>> p;
    p.push_back(std::make_shared<DraftLine>(Vec2(-h, -h), Vec2(h, -h)));
    p.push_back(std::make_shared<DraftLine>(Vec2(h, -h), Vec2(h, h)));
    p.push_back(std::make_shared<DraftLine>(Vec2(h, h), Vec2(-h, h)));
    p.push_back(std::make_shared<DraftLine>(Vec2(-h, h), Vec2(-h, -h)));
    return p;
}

// A sketch plane parallel to XY at height z.
SketchPlane planeAtZ(double z) {
    return SketchPlane(Vec3(0, 0, z), Vec3(0, 0, 1), Vec3(1, 0, 0));
}

}  // namespace

// ---------------------------------------------------------------------------
// TwoSquareLoftEqualsBoxCounts
// ---------------------------------------------------------------------------

TEST(LoftTest, TwoSquareLoftEqualsBoxCounts) {
    std::vector<LoftSection> sections = {
        {squareProfile(4.0), planeAtZ(0.0)},
        {squareProfile(4.0), planeAtZ(10.0)},
    };

    auto solid = Loft::execute(sections, "loft_1");
    ASSERT_NE(solid, nullptr);

    // Two 4-vertex rings → box topology.
    EXPECT_EQ(solid->vertexCount(), 8u);
    EXPECT_EQ(solid->edgeCount(), 12u);
    EXPECT_EQ(solid->faceCount(), 6u);
    EXPECT_TRUE(solid->checkEulerFormula());
    EXPECT_TRUE(solid->checkManifold());
    EXPECT_TRUE(solid->isValid());
}

// ---------------------------------------------------------------------------
// TaperedLoftIsValid
// ---------------------------------------------------------------------------

TEST(LoftTest, TaperedLoftIsValid) {
    std::vector<LoftSection> sections = {
        {squareProfile(8.0), planeAtZ(0.0)},
        {squareProfile(2.0), planeAtZ(12.0)},
    };

    auto solid = Loft::execute(sections, "loft_taper");
    ASSERT_NE(solid, nullptr);
    EXPECT_TRUE(solid->isValid());
    EXPECT_TRUE(solid->checkManifold());

    // Every face has a bound surface.
    for (const auto& f : solid->faces()) {
        EXPECT_NE(f.surface, nullptr) << "face " << f.topoId.tag() << " has no surface";
        EXPECT_TRUE(f.topoId.isValid());
    }
}

// ---------------------------------------------------------------------------
// ThreeSectionLoftEuler
// ---------------------------------------------------------------------------

TEST(LoftTest, ThreeSectionLoftEuler) {
    std::vector<LoftSection> sections = {
        {squareProfile(6.0), planeAtZ(0.0)},
        {squareProfile(3.0), planeAtZ(5.0)},
        {squareProfile(6.0), planeAtZ(10.0)},
    };

    auto solid = Loft::execute(sections, "loft_3");
    ASSERT_NE(solid, nullptr);

    // S=2 levels, N=4: V=12, E=20, F=10.
    EXPECT_EQ(solid->vertexCount(), 12u);
    EXPECT_EQ(solid->edgeCount(), 20u);
    EXPECT_EQ(solid->faceCount(), 10u);
    EXPECT_TRUE(solid->checkEulerFormula());
    EXPECT_TRUE(solid->checkManifold());
}

// ---------------------------------------------------------------------------
// MismatchedVertexCountRejected
// ---------------------------------------------------------------------------

TEST(LoftTest, MismatchedVertexCountRejected) {
    // Square (4 verts) → triangle (3 verts): not supported in Era 2.
    std::vector<std::shared_ptr<DraftEntity>> tri;
    tri.push_back(std::make_shared<DraftLine>(Vec2(0, 0), Vec2(4, 0)));
    tri.push_back(std::make_shared<DraftLine>(Vec2(4, 0), Vec2(2, 3)));
    tri.push_back(std::make_shared<DraftLine>(Vec2(2, 3), Vec2(0, 0)));

    std::vector<LoftSection> sections = {
        {squareProfile(4.0), planeAtZ(0.0)},
        {tri, planeAtZ(8.0)},
    };
    EXPECT_EQ(Loft::execute(sections, "loft_bad"), nullptr);
}

// ---------------------------------------------------------------------------
// InvalidInputsRejected
// ---------------------------------------------------------------------------

TEST(LoftTest, InvalidInputsRejected) {
    // Single section: nothing to loft.
    std::vector<LoftSection> one = {{squareProfile(4.0), planeAtZ(0.0)}};
    EXPECT_EQ(Loft::execute(one, "loft_one"), nullptr);

    // Open profile in a section.
    std::vector<std::shared_ptr<DraftEntity>> open;
    open.push_back(std::make_shared<DraftLine>(Vec2(0, 0), Vec2(4, 0)));
    open.push_back(std::make_shared<DraftLine>(Vec2(4, 0), Vec2(4, 4)));
    std::vector<LoftSection> withOpen = {
        {open, planeAtZ(0.0)},
        {squareProfile(4.0), planeAtZ(8.0)},
    };
    EXPECT_EQ(Loft::execute(withOpen, "loft_open"), nullptr);

    // Empty.
    EXPECT_EQ(Loft::execute({}, "loft_empty"), nullptr);
}
