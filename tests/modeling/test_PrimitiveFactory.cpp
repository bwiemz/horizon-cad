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

    // An n-sided prism: two n-gon caps and n lateral quads.
    const auto n = static_cast<size_t>(PrimitiveFactory::kDefaultSegments);
    EXPECT_EQ(solid->vertexCount(), 2 * n);
    EXPECT_EQ(solid->edgeCount(), 3 * n);
    EXPECT_EQ(solid->faceCount(), n + 2);
    EXPECT_EQ(solid->shellCount(), 1u);
    EXPECT_TRUE(solid->checkEulerFormula());
    EXPECT_TRUE(solid->checkManifold());
    EXPECT_TRUE(solid->isValid()) << solid->validationReport();
}

TEST(PrimitiveFactoryTest, CylinderVolumeConvergesToTheAnalyticValue) {
    // Faceting inscribes the cylinder, so the volume approaches pi*r^2*h from
    // below.  The n-gon cap area is exact: (n/2) r^2 sin(2pi/n).
    const double r = 5.0;
    const double h = 10.0;
    double previousError = 1e30;
    for (int n : {8, 32, 128}) {
        auto solid = PrimitiveFactory::makeCylinder(r, h, n);
        ASSERT_NE(solid, nullptr) << "n=" << n;
        const double got = MassPropertiesCalculator::compute(*solid).volume;
        const double prism = 0.5 * n * r * r * std::sin(2.0 * M_PI / n) * h;
        EXPECT_NEAR(got, prism, 1e-9) << "n=" << n;

        const double error = M_PI * r * r * h - got;
        EXPECT_GT(error, 0.0) << "inscribed volume must stay under the analytic one";
        EXPECT_LT(error, previousError) << "refining must reduce the error";
        previousError = error;
    }
    // The default resolution is good to a tenth of a percent.
    auto solid = PrimitiveFactory::makeCylinder(r, h);
    EXPECT_NEAR(MassPropertiesCalculator::compute(*solid).volume, M_PI * r * r * h,
                0.01 * M_PI * r * r * h);
}

TEST(PrimitiveFactoryTest, CylinderRemembersTheSurfaceAndRimItFacets) {
    auto solid = PrimitiveFactory::makeCylinder(5.0, 10.0);
    ASSERT_NE(solid, nullptr);

    // Lateral facets are planar (that is their carrier) but record the
    // cylinder they approximate, so one pick still resolves the cylinder.
    int lateral = 0;
    for (const auto& f : solid->faces()) {
        if (f.topoId.tag().find("side") == std::string::npos) continue;
        ++lateral;
        EXPECT_NE(f.surface, nullptr);
        ASSERT_NE(f.analyticSurface, nullptr) << f.topoId.tag();
    }
    EXPECT_EQ(lateral, PrimitiveFactory::kDefaultSegments);

    // Rim edges are chords that remember their arc.
    int rim = 0;
    for (const auto& e : solid->edges()) {
        if (e.analyticCurve != nullptr) ++rim;
    }
    EXPECT_EQ(rim, 2 * PrimitiveFactory::kDefaultSegments);
}

TEST(PrimitiveFactoryTest, SegmentsForToleranceInvertsTheChordSag) {
    // Sag of an n-gon chord on a circle of radius r is r*(1 - cos(pi/n)).
    for (double tol : {1.0, 0.1, 0.01, 0.001}) {
        const int n = PrimitiveFactory::segmentsForTolerance(5.0, tol);
        const double sag = 5.0 * (1.0 - std::cos(M_PI / n));
        EXPECT_LE(sag, tol) << "tol=" << tol << " n=" << n;
        EXPECT_GE(n, 3);
    }
    // Finer tolerance never asks for fewer facets.
    EXPECT_GE(PrimitiveFactory::segmentsForTolerance(5.0, 0.001),
              PrimitiveFactory::segmentsForTolerance(5.0, 0.1));
    // Degenerate inputs fall back to the default rather than diverging.
    EXPECT_EQ(PrimitiveFactory::segmentsForTolerance(0.0, 0.1), PrimitiveFactory::kDefaultSegments);
    EXPECT_EQ(PrimitiveFactory::segmentsForTolerance(5.0, 0.0), PrimitiveFactory::kDefaultSegments);
}

TEST(PrimitiveFactoryTest, DegenerateCurvedPrimitivesAreRefused) {
    EXPECT_EQ(PrimitiveFactory::makeCylinder(0.0, 5.0), nullptr);
    EXPECT_EQ(PrimitiveFactory::makeCylinder(1.0, 0.0), nullptr);
    EXPECT_EQ(PrimitiveFactory::makeCylinder(1.0, 5.0, 2), nullptr);
    EXPECT_EQ(PrimitiveFactory::makeSphere(0.0), nullptr);
    EXPECT_EQ(PrimitiveFactory::makeTorus(1.0, 2.0), nullptr) << "tube thicker than the ring";
    EXPECT_EQ(PrimitiveFactory::makeTorus(3.0, 0.0), nullptr);
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

    // A UV-sphere: n facets around, m bands pole to pole, poles as single
    // vertices.  V = 2 + n(m-1), E = nm + n(m-1), F = nm.
    const auto n = static_cast<size_t>(PrimitiveFactory::kDefaultSegments);
    const size_t m = n / 2;
    EXPECT_EQ(solid->vertexCount(), 2 + n * (m - 1));
    EXPECT_EQ(solid->edgeCount(), n * m + n * (m - 1));
    EXPECT_EQ(solid->faceCount(), n * m);
    EXPECT_EQ(solid->shellCount(), 1u);
    EXPECT_TRUE(solid->checkEulerFormula());
    EXPECT_TRUE(solid->checkManifold());
    EXPECT_TRUE(solid->isValid()) << solid->validationReport();
}

TEST(PrimitiveFactoryTest, SphereVolumeConvergesToTheAnalyticValue) {
    const double r = 5.0;
    const double exact = 4.0 / 3.0 * M_PI * r * r * r;
    double previousError = 1e30;
    for (int n : {8, 32, 64}) {
        auto solid = PrimitiveFactory::makeSphere(r, n);
        ASSERT_NE(solid, nullptr) << "n=" << n;
        EXPECT_TRUE(solid->checkManifold()) << "n=" << n;
        const double error = exact - MassPropertiesCalculator::compute(*solid).volume;
        EXPECT_GT(error, 0.0) << "n=" << n << ": inscribed volume must stay under the analytic one";
        EXPECT_LT(error, previousError) << "n=" << n << ": refining must reduce the error";
        previousError = error;
    }
    EXPECT_LT(previousError, 0.01 * exact) << "64 facets should be within a percent";
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

    // Apex topology: an n-gon base plus n triangles meeting at a point, not a
    // ring of zero-length edges around a zero-area cap.
    const auto n = static_cast<size_t>(PrimitiveFactory::kDefaultSegments);
    EXPECT_EQ(solid->vertexCount(), n + 1);
    EXPECT_EQ(solid->edgeCount(), 2 * n);
    EXPECT_EQ(solid->faceCount(), n + 1);
    EXPECT_TRUE(solid->isValid()) << solid->validationReport();
    EXPECT_TRUE(GeometryValidator::isGeometricallyValid(*solid))
        << GeometryValidator::report(*solid);
}

TEST(PrimitiveFactoryTest, SharpConeVolumeMatchesThePyramidItFacetsTo) {
    // The base is the inscribed n-gon, so the solid is exactly the pyramid
    // over it: (1/3) * (n/2) r^2 sin(2pi/n) * h.
    const double r = 5.0;
    const double h = 10.0;
    const int n = PrimitiveFactory::kDefaultSegments;
    auto solid = PrimitiveFactory::makeCone(r, 0.0, h, n);
    ASSERT_NE(solid, nullptr);
    const double base = 0.5 * n * r * r * std::sin(2.0 * M_PI / n);
    EXPECT_NEAR(MassPropertiesCalculator::compute(*solid).volume, base * h / 3.0, 1e-9);

    // And it converges to the true cone.
    auto fine = PrimitiveFactory::makeCone(r, 0.0, h, 256);
    ASSERT_NE(fine, nullptr);
    const double exact = M_PI * r * r * h / 3.0;
    EXPECT_NEAR(MassPropertiesCalculator::compute(*fine).volume, exact, 0.001 * exact);
}

TEST(PrimitiveFactoryTest, InvertedSharpConeUsesApexTopology) {
    auto solid = PrimitiveFactory::makeCone(0.0, 5.0, 10.0);
    ASSERT_NE(solid, nullptr);
    const auto n = static_cast<size_t>(PrimitiveFactory::kDefaultSegments);
    EXPECT_EQ(solid->vertexCount(), n + 1);
    EXPECT_EQ(solid->faceCount(), n + 1);
    EXPECT_TRUE(solid->isValid()) << solid->validationReport();
    EXPECT_TRUE(GeometryValidator::isGeometricallyValid(*solid))
        << GeometryValidator::report(*solid);

    const double base = 0.5 * n * 25.0 * std::sin(2.0 * M_PI / static_cast<double>(n));
    EXPECT_NEAR(MassPropertiesCalculator::compute(*solid).volume, base * 10.0 / 3.0, 1e-9);
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

    // A frustum is a prism between two n-gon caps.
    const auto n = static_cast<size_t>(PrimitiveFactory::kDefaultSegments);
    EXPECT_EQ(solid->vertexCount(), 2 * n);
    EXPECT_EQ(solid->edgeCount(), 3 * n);
    EXPECT_EQ(solid->faceCount(), n + 2);
    EXPECT_EQ(solid->shellCount(), 1u);
    EXPECT_TRUE(solid->checkEulerFormula());
    EXPECT_TRUE(solid->checkManifold());
    EXPECT_TRUE(solid->isValid()) << solid->validationReport();
}

TEST(PrimitiveFactoryTest, FrustumVolumeConvergesToTheAnalyticValue) {
    const double rb = 2.0;
    const double rt = 1.0;
    const double h = 5.0;
    const double exact = M_PI * h / 3.0 * (rb * rb + rb * rt + rt * rt);
    auto solid = PrimitiveFactory::makeCone(rb, rt, h, 128);
    ASSERT_NE(solid, nullptr);
    EXPECT_NEAR(MassPropertiesCalculator::compute(*solid).volume, exact, 0.001 * exact);
}

// ---------------------------------------------------------------------------
// Torus tests
// ---------------------------------------------------------------------------

TEST(PrimitiveFactoryTest, TorusIsAManifoldGenusOneShell) {
    auto solid = PrimitiveFactory::makeTorus(3.0, 1.0);
    ASSERT_NE(solid, nullptr);

    // A quad grid wrapped both ways: V = E/2 = F = n*m.
    const auto n = static_cast<size_t>(PrimitiveFactory::kDefaultSegments);
    const size_t m = n / 2;
    EXPECT_EQ(solid->vertexCount(), n * m);
    EXPECT_EQ(solid->edgeCount(), 2 * n * m);
    EXPECT_EQ(solid->faceCount(), n * m);
    EXPECT_EQ(solid->shellCount(), 1u);
    EXPECT_TRUE(solid->checkManifold());

    // A torus has genus 1, so its Euler characteristic is 0, not 2.
    // Solid::checkEulerFormula() carries no genus term and therefore reports
    // false here — that is the check's documented limit, not a defect in the
    // solid, which the geometric validator confirms is sound.
    EXPECT_FALSE(solid->checkEulerFormula())
        << "the genus-0 Euler check is expected to reject a torus";
    EXPECT_TRUE(GeometryValidator::isGeometricallyValid(*solid))
        << GeometryValidator::report(*solid);
}

TEST(PrimitiveFactoryTest, TorusVolumeConvergesToTheAnalyticValue) {
    const double major = 10.0;
    const double minor = 3.0;
    const double exact = 2.0 * M_PI * M_PI * major * minor * minor;

    // The old box-topology torus enclosed no volume at all; faceting gives a
    // real solid whose volume converges.
    double previousError = 1e30;
    for (int n : {16, 64, 128}) {
        auto solid = PrimitiveFactory::makeTorus(major, minor, n);
        ASSERT_NE(solid, nullptr) << "n=" << n;
        const double error = exact - MassPropertiesCalculator::compute(*solid).volume;
        EXPECT_GT(error, 0.0) << "n=" << n;
        EXPECT_LT(error, previousError) << "n=" << n;
        previousError = error;
    }
    EXPECT_LT(previousError, 0.01 * exact);
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
