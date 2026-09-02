#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "horizon/modeling/MassProperties.h"
#include "horizon/modeling/PrimitiveFactory.h"
#include "horizon/topology/GeometryValidator.h"
#include "horizon/topology/Queries.h"
#include "horizon/topology/Solid.h"
#include "horizon/topology/TopologyID.h"

using namespace hz::model;
using namespace hz::topo;
using hz::math::Vec3;

// ---------------------------------------------------------------------------
// Box tests
// ---------------------------------------------------------------------------

TEST(PrimitiveFactoryTest, BoxEulerFormula) {
    auto solid = PrimitiveFactory::makeBox(2.0, 3.0, 4.0);
    ASSERT_NE(solid, nullptr);

    // Box: V=8, E=12, F=6, S=1.
    // Euler: 8 - 12 + 6 = 2, 2*(1-0) = 2.
    EXPECT_EQ(solid->vertexCount(), 8u);
    EXPECT_EQ(solid->edgeCount(), 12u);
    EXPECT_EQ(solid->faceCount(), 6u);
    EXPECT_EQ(solid->shellCount(), 1u);
    EXPECT_TRUE(solid->checkEulerFormula());
    EXPECT_TRUE(solid->checkManifold());
    EXPECT_TRUE(solid->isValid()) << solid->validationReport();
}

TEST(PrimitiveFactoryTest, BoxVertexPositions) {
    auto solid = PrimitiveFactory::makeBox(2.0, 3.0, 4.0);
    ASSERT_NE(solid, nullptr);

    // Expected corner positions.
    const Vec3 expected[8] = {
        {0, 0, 0}, {2, 0, 0}, {2, 3, 0}, {0, 3, 0}, {0, 0, 4}, {2, 0, 4}, {2, 3, 4}, {0, 3, 4},
    };

    const auto& verts = solid->vertices();
    ASSERT_EQ(verts.size(), 8u);

    // Each expected point must appear exactly once among the vertices.
    for (const auto& exp : expected) {
        int count = 0;
        for (const auto& v : verts) {
            if (v.point.isApproxEqual(exp, 1e-9)) {
                ++count;
            }
        }
        EXPECT_EQ(count, 1) << "Expected vertex (" << exp.x << ", " << exp.y << ", " << exp.z
                            << ") not found exactly once (found " << count << ")";
    }
}

TEST(PrimitiveFactoryTest, BoxEachFaceHas4Vertices) {
    auto solid = PrimitiveFactory::makeBox(1.0, 1.0, 1.0);
    ASSERT_NE(solid, nullptr);

    for (const auto& face : solid->faces()) {
        auto verts = faceVertices(&face);
        EXPECT_EQ(verts.size(), 4u)
            << "Face id=" << face.id << " has " << verts.size() << " vertices, expected 4";
    }
}

TEST(PrimitiveFactoryTest, BoxTopologyIDs) {
    auto solid = PrimitiveFactory::makeBox(1.0, 1.0, 1.0);
    ASSERT_NE(solid, nullptr);

    std::set<std::string> faceIds;
    for (const auto& face : solid->faces()) {
        EXPECT_TRUE(face.topoId.isValid()) << "Face id=" << face.id << " has no TopologyID";
        faceIds.insert(face.topoId.tag());
    }

    // All 6 faces should have distinct IDs.
    EXPECT_EQ(faceIds.size(), 6u);

    // Check specific face names exist.
    EXPECT_TRUE(faceIds.count("box/bottom") > 0);
    EXPECT_TRUE(faceIds.count("box/top") > 0);
    EXPECT_TRUE(faceIds.count("box/front") > 0);
    EXPECT_TRUE(faceIds.count("box/back") > 0);
    EXPECT_TRUE(faceIds.count("box/right") > 0);
    EXPECT_TRUE(faceIds.count("box/left") > 0);
}

TEST(PrimitiveFactoryTest, BoxHasNURBSGeometry) {
    auto solid = PrimitiveFactory::makeBox(2.0, 3.0, 4.0);
    ASSERT_NE(solid, nullptr);

    // Every face must have a NURBS surface.
    for (const auto& face : solid->faces()) {
        EXPECT_NE(face.surface, nullptr) << "Face " << face.topoId.tag() << " has no surface";
    }

    // Every edge must have a NURBS curve.
    for (const auto& edge : solid->edges()) {
        EXPECT_NE(edge.curve, nullptr) << "Edge id=" << edge.id << " has no curve";
    }
}

// ---------------------------------------------------------------------------
// Cylinder tests
// ---------------------------------------------------------------------------

TEST(PrimitiveFactoryTest, CylinderEulerFormula) {
    auto solid = PrimitiveFactory::makeCylinder(1.0, 5.0);
    ASSERT_NE(solid, nullptr);

    EXPECT_EQ(solid->vertexCount(), 8u);
    EXPECT_EQ(solid->edgeCount(), 12u);
    EXPECT_EQ(solid->faceCount(), 6u);
    EXPECT_EQ(solid->shellCount(), 1u);
    EXPECT_TRUE(solid->checkEulerFormula());
    EXPECT_TRUE(solid->checkManifold());
    EXPECT_TRUE(solid->isValid()) << solid->validationReport();
}

TEST(PrimitiveFactoryTest, CylinderHasGeometry) {
    auto solid = PrimitiveFactory::makeCylinder(1.0, 5.0);
    ASSERT_NE(solid, nullptr);

    for (const auto& face : solid->faces()) {
        EXPECT_NE(face.surface, nullptr) << "Face " << face.topoId.tag() << " has no surface";
    }
    for (const auto& edge : solid->edges()) {
        EXPECT_NE(edge.curve, nullptr) << "Edge id=" << edge.id << " has no curve";
    }
}

// ---------------------------------------------------------------------------
// Sphere tests
// ---------------------------------------------------------------------------

TEST(PrimitiveFactoryTest, SphereEulerFormula) {
    auto solid = PrimitiveFactory::makeSphere(2.0);
    ASSERT_NE(solid, nullptr);

    EXPECT_EQ(solid->vertexCount(), 8u);
    EXPECT_EQ(solid->edgeCount(), 12u);
    EXPECT_EQ(solid->faceCount(), 6u);
    EXPECT_EQ(solid->shellCount(), 1u);
    EXPECT_TRUE(solid->checkEulerFormula());
    EXPECT_TRUE(solid->checkManifold());
    EXPECT_TRUE(solid->isValid()) << solid->validationReport();
}

// ---------------------------------------------------------------------------
// Cone tests
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Sharp cone (topRadius == 0): apex topology, not a collapsed ring.
//
// The frustum path builds box topology from two 4-point rings.  With a zero
// radius that ring degenerates to a single point, producing four zero-length
// edges and a zero-area cap — a solid that passes Euler and manifold checks
// while being geometric nonsense.  The "Cone" command asks for exactly that
// shape, so the sharp case gets its own construction.
// ---------------------------------------------------------------------------

TEST(PrimitiveFactoryTest, SharpConeUsesApexTopology) {
    auto solid = PrimitiveFactory::makeCone(5.0, 0.0, 10.0);
    ASSERT_NE(solid, nullptr);

    // Apex topology: 4 base vertices + apex, 4 base edges + 4 lateral, 1 base
    // quad + 4 triangles.  Euler: 5 - 8 + 5 = 2.
    EXPECT_EQ(solid->vertexCount(), 5u);
    EXPECT_EQ(solid->edgeCount(), 8u);
    EXPECT_EQ(solid->faceCount(), 5u);
    EXPECT_TRUE(solid->isValid()) << solid->validationReport();
    EXPECT_TRUE(GeometryValidator::isGeometricallyValid(*solid))
        << GeometryValidator::report(*solid);
}

TEST(PrimitiveFactoryTest, SharpConeVolumeMatchesThePyramidItTessellatesTo) {
    // Four points on the base circle inscribe a square of area 2r^2, so the
    // faceted cone is a pyramid of volume (1/3) * 2r^2 * h.
    auto solid = PrimitiveFactory::makeCone(5.0, 0.0, 10.0);
    ASSERT_NE(solid, nullptr);
    const auto props = MassPropertiesCalculator::compute(*solid);
    ASSERT_TRUE(props.valid);
    EXPECT_NEAR(props.volume, 2.0 * 25.0 * 10.0 / 3.0, 1e-9);
}

TEST(PrimitiveFactoryTest, InvertedSharpConeUsesApexTopology) {
    auto solid = PrimitiveFactory::makeCone(0.0, 5.0, 10.0);
    ASSERT_NE(solid, nullptr);
    EXPECT_EQ(solid->vertexCount(), 5u);
    EXPECT_EQ(solid->faceCount(), 5u);
    EXPECT_TRUE(solid->isValid()) << solid->validationReport();
    EXPECT_TRUE(GeometryValidator::isGeometricallyValid(*solid))
        << GeometryValidator::report(*solid);

    const auto props = MassPropertiesCalculator::compute(*solid);
    ASSERT_TRUE(props.valid);
    EXPECT_NEAR(props.volume, 2.0 * 25.0 * 10.0 / 3.0, 1e-9);
}

TEST(PrimitiveFactoryTest, FullyDegenerateConeIsRefused) {
    EXPECT_EQ(PrimitiveFactory::makeCone(0.0, 0.0, 10.0), nullptr);
    EXPECT_EQ(PrimitiveFactory::makeCone(5.0, 2.0, 0.0), nullptr);
}

// ---------------------------------------------------------------------------
// Every primitive must be geometrically valid, not merely combinatorially so.
// ---------------------------------------------------------------------------

TEST(PrimitiveFactoryTest, AllPrimitivesAreGeometricallyValid) {
    struct Case {
        const char* name;
        std::unique_ptr<Solid> solid;
    };
    std::vector<Case> cases;
    cases.push_back({"box", PrimitiveFactory::makeBox(2, 3, 4)});
    cases.push_back({"cylinder", PrimitiveFactory::makeCylinder(2, 5)});
    cases.push_back({"sphere", PrimitiveFactory::makeSphere(3)});
    cases.push_back({"frustum", PrimitiveFactory::makeCone(2, 1, 5)});
    cases.push_back({"sharp cone", PrimitiveFactory::makeCone(2, 0, 5)});
    cases.push_back({"torus", PrimitiveFactory::makeTorus(5, 2)});

    for (const auto& c : cases) {
        SCOPED_TRACE(c.name);
        ASSERT_NE(c.solid, nullptr);
        EXPECT_TRUE(GeometryValidator::isGeometricallyValid(*c.solid))
            << GeometryValidator::report(*c.solid);
    }
}

TEST(PrimitiveFactoryTest, ConeEulerFormula) {
    auto solid = PrimitiveFactory::makeCone(2.0, 1.0, 5.0);
    ASSERT_NE(solid, nullptr);

    EXPECT_EQ(solid->vertexCount(), 8u);
    EXPECT_EQ(solid->edgeCount(), 12u);
    EXPECT_EQ(solid->faceCount(), 6u);
    EXPECT_EQ(solid->shellCount(), 1u);
    EXPECT_TRUE(solid->checkEulerFormula());
    EXPECT_TRUE(solid->checkManifold());
    EXPECT_TRUE(solid->isValid()) << solid->validationReport();
}

// ---------------------------------------------------------------------------
// Torus tests
// ---------------------------------------------------------------------------

TEST(PrimitiveFactoryTest, TorusEulerFormula) {
    auto solid = PrimitiveFactory::makeTorus(3.0, 1.0);
    ASSERT_NE(solid, nullptr);

    EXPECT_EQ(solid->vertexCount(), 8u);
    EXPECT_EQ(solid->edgeCount(), 12u);
    EXPECT_EQ(solid->faceCount(), 6u);
    EXPECT_EQ(solid->shellCount(), 1u);
    EXPECT_TRUE(solid->checkEulerFormula());
    EXPECT_TRUE(solid->checkManifold());
    EXPECT_TRUE(solid->isValid()) << solid->validationReport();
}

// ---------------------------------------------------------------------------
// Cross-cutting: all primitives have TopologyIDs
// ---------------------------------------------------------------------------

static void checkPrimitiveTopologyIDs(const std::string& name, const Solid& solid) {
    for (const auto& face : solid.faces()) {
        EXPECT_TRUE(face.topoId.isValid())
            << name << ": face id=" << face.id << " has no TopologyID";
    }
    for (const auto& edge : solid.edges()) {
        EXPECT_TRUE(edge.topoId.isValid())
            << name << ": edge id=" << edge.id << " has no TopologyID";
    }
}

TEST(PrimitiveFactoryTest, AllPrimitivesHaveTopologyIDs) {
    {
        SCOPED_TRACE("box");
        auto s = PrimitiveFactory::makeBox(1, 1, 1);
        ASSERT_NE(s, nullptr);
        checkPrimitiveTopologyIDs("box", *s);
    }
    {
        SCOPED_TRACE("cylinder");
        auto s = PrimitiveFactory::makeCylinder(1, 2);
        ASSERT_NE(s, nullptr);
        checkPrimitiveTopologyIDs("cylinder", *s);
    }
    {
        SCOPED_TRACE("sphere");
        auto s = PrimitiveFactory::makeSphere(1);
        ASSERT_NE(s, nullptr);
        checkPrimitiveTopologyIDs("sphere", *s);
    }
    {
        SCOPED_TRACE("cone");
        auto s = PrimitiveFactory::makeCone(2, 1, 3);
        ASSERT_NE(s, nullptr);
        checkPrimitiveTopologyIDs("cone", *s);
    }
    {
        SCOPED_TRACE("torus");
        auto s = PrimitiveFactory::makeTorus(3, 1);
        ASSERT_NE(s, nullptr);
        checkPrimitiveTopologyIDs("torus", *s);
    }
}
