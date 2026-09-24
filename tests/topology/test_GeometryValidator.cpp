#include <gtest/gtest.h>

#include <cmath>
#include <map>
#include <memory>
#include <utility>
#include <vector>

#include "horizon/geometry/curves/NurbsCurve.h"
#include "horizon/geometry/surfaces/NurbsSurface.h"
#include "horizon/topology/GeometryValidator.h"
#include "horizon/topology/Queries.h"
#include "horizon/topology/Solid.h"

using namespace hz::topo;
using hz::math::Vec3;
using GV = GeometryValidator;

namespace {

/// Minimal half-edge assembler for the tests.
///
/// Deliberately dumb — it wires exactly the structure it is handed, with no
/// welding, no repair, and no validation.  That is the point: every defect
/// the validator is supposed to catch has to be constructible here, which a
/// self-healing builder (SolidSewer) would silently fix out from under the
/// test.
class Assembler {
public:
    /// @param points  Vertex positions, indexed by the face lists.
    /// @param faces   Per-face vertex index loops, outward wound.
    static std::unique_ptr<Solid> build(const std::vector<Vec3>& points,
                                        const std::vector<std::vector<int>>& faces) {
        auto solid = std::make_unique<Solid>();

        std::vector<Vertex*> verts;
        verts.reserve(points.size());
        for (const auto& p : points) {
            Vertex* v = solid->allocVertex();
            v->point = p;
            verts.push_back(v);
        }

        Shell* shell = solid->allocShell();
        shell->solid = solid.get();

        // Directed half-edge lookup, so the second use of an undirected edge
        // finds its twin.
        std::map<std::pair<int, int>, HalfEdge*> directed;

        for (const auto& loop : faces) {
            Face* face = solid->allocFace();
            face->shell = shell;
            shell->faces.push_back(face);
            Wire* wire = solid->allocWire();
            face->outerLoop = wire;

            const int n = static_cast<int>(loop.size());
            std::vector<HalfEdge*> hes(static_cast<size_t>(n));
            for (int i = 0; i < n; ++i) {
                HalfEdge* he = solid->allocHalfEdge();
                he->origin = verts[static_cast<size_t>(loop[static_cast<size_t>(i)])];
                he->face = face;
                hes[static_cast<size_t>(i)] = he;
                if (he->origin->halfEdge == nullptr) {
                    he->origin->halfEdge = he;
                }
            }
            for (int i = 0; i < n; ++i) {
                hes[static_cast<size_t>(i)]->next = hes[static_cast<size_t>((i + 1) % n)];
                hes[static_cast<size_t>((i + 1) % n)]->prev = hes[static_cast<size_t>(i)];
            }
            wire->halfEdge = hes[0];

            for (int i = 0; i < n; ++i) {
                const int a = loop[static_cast<size_t>(i)];
                const int b = loop[static_cast<size_t>((i + 1) % n)];
                HalfEdge* he = hes[static_cast<size_t>(i)];
                auto twinIt = directed.find({b, a});
                if (twinIt != directed.end()) {
                    HalfEdge* twin = twinIt->second;
                    he->twin = twin;
                    twin->twin = he;
                    he->edge = twin->edge;
                } else {
                    Edge* edge = solid->allocEdge();
                    edge->halfEdge = he;
                    he->edge = edge;
                }
                directed[{a, b}] = he;
            }
        }

        // Bind linear curves so the edge-curve advisory has something to check.
        for (auto& e : const_cast<std::deque<Edge>&>(solid->edges())) {
            if (e.halfEdge == nullptr || e.halfEdge->twin == nullptr) {
                continue;
            }
            const Vec3 a = e.halfEdge->origin->point;
            const Vec3 b = e.halfEdge->twin->origin->point;
            e.curve = std::make_shared<hz::geo::NurbsCurve>(
                std::vector<Vec3>{a, b}, std::vector<double>{1.0, 1.0},
                std::vector<double>{0.0, 0.0, 1.0, 1.0}, 1);
        }
        return solid;
    }
};

/// Unit cube, outward wound, as an index soup.
std::unique_ptr<Solid> makeCube() {
    const std::vector<Vec3> pts = {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0},
                                   {0, 0, 1}, {1, 0, 1}, {1, 1, 1}, {0, 1, 1}};
    const std::vector<std::vector<int>> faces = {
        {0, 3, 2, 1},  // bottom (-Z)
        {4, 5, 6, 7},  // top (+Z)
        {0, 1, 5, 4},  // front (-Y)
        {1, 2, 6, 5},  // right (+X)
        {2, 3, 7, 6},  // back (+Y)
        {3, 0, 4, 7},  // left (-X)
    };
    return Assembler::build(pts, faces);
}

}  // namespace

// ---------------------------------------------------------------------------
// A well-formed solid passes every check.
// ---------------------------------------------------------------------------

TEST(GeometryValidatorTest, WellFormedCubePassesEverything) {
    auto cube = makeCube();
    ASSERT_NE(cube, nullptr);
    ASSERT_TRUE(cube->checkManifold()) << cube->validationReport();

    const auto issues = GV::check(*cube);
    EXPECT_TRUE(issues.ok()) << GV::report(*cube);
    EXPECT_EQ(issues.vertexChainErrors, 0);
    EXPECT_EQ(issues.twinCoincidenceErrors, 0);
    EXPECT_EQ(issues.degenerateEdges, 0);
    EXPECT_EQ(issues.degenerateFaces, 0);
    EXPECT_EQ(issues.nonPlanarLoops, 0);
    EXPECT_EQ(issues.selfIntersectingLoops, 0);
    EXPECT_EQ(issues.openShells, 0);
    EXPECT_EQ(issues.edgeCurveMismatches, 0);
    EXPECT_EQ(issues.coincidentVertices, 0);
    EXPECT_TRUE(GV::isGeometricallyValid(*cube));
}

TEST(GeometryValidatorTest, ReportIsCleanForAValidSolid) {
    auto cube = makeCube();
    EXPECT_EQ(GV::report(*cube), "Geometry checks OK\n");
}

// ---------------------------------------------------------------------------
// The defects the structural validators are blind to.
// ---------------------------------------------------------------------------

TEST(GeometryValidatorTest, NonPlanarLoopIsCaughtWhileStructuralChecksPass) {
    // Lift one top corner off the top face's plane.  Every structural
    // invariant still holds — this is exactly the class of defect that the
    // Euler/manifold checks cannot see.
    auto cube = makeCube();
    for (auto& v : const_cast<std::deque<Vertex>&>(cube->vertices())) {
        if (v.point.x > 0.5 && v.point.y > 0.5 && v.point.z > 0.5) {
            v.point.z = 2.0;
        }
    }

    EXPECT_TRUE(cube->checkManifold());
    EXPECT_TRUE(cube->checkEulerFormula());
    EXPECT_TRUE(cube->isValid()) << "structural validation must still pass";

    const auto issues = GV::check(*cube);
    EXPECT_FALSE(issues.ok());
    EXPECT_GT(issues.nonPlanarLoops, 0);
    EXPECT_NE(GV::report(*cube).find("Non-planar loops"), std::string::npos);
}

TEST(GeometryValidatorTest, SelfIntersectingLoopIsCaught) {
    // A crossed ("bowtie") quad and its reverse: a closed, manifold, planar
    // lamina whose boundary crosses itself.  The lobes are given different
    // areas so the loop does not also read as degenerate — this test is about
    // the crossing alone.
    const std::vector<Vec3> pts = {{0, 0, 0}, {3, 0, 0}, {0, 1, 0}, {1, 1, 0}};
    const std::vector<std::vector<int>> faces = {
        {0, 1, 2, 3},
        {3, 2, 1, 0},
    };
    auto solid = Assembler::build(pts, faces);
    ASSERT_TRUE(solid->checkManifold()) << solid->validationReport();

    const auto issues = GV::check(*solid);
    EXPECT_EQ(issues.degenerateFaces, 0) << GV::report(*solid);
    EXPECT_EQ(issues.selfIntersectingLoops, 2) << GV::report(*solid);
    EXPECT_FALSE(issues.ok());
    EXPECT_NE(GV::report(*solid).find("Self-intersecting loops"), std::string::npos);
}

TEST(GeometryValidatorTest, DegenerateEdgeAndFaceAreCaught) {
    // Collapse the whole top face onto one point: four zero-length edges and
    // a zero-area face, with the structure left intact.
    auto cube = makeCube();
    for (auto& v : const_cast<std::deque<Vertex>&>(cube->vertices())) {
        if (v.point.z > 0.5) {
            v.point = Vec3(0.5, 0.5, 1.0);
        }
    }

    EXPECT_TRUE(cube->checkManifold());
    const auto issues = GV::check(*cube);
    EXPECT_EQ(issues.degenerateEdges, 4);
    EXPECT_GT(issues.degenerateFaces, 0);
    EXPECT_GT(issues.coincidentVertices, 0);
    EXPECT_FALSE(issues.ok());
}

TEST(GeometryValidatorTest, MispairedTwinsAreCaughtWhileStructuralChecksPass) {
    // Cross-pair the twins of two unrelated edges, keeping every structural
    // invariant intact: twins stay reciprocal, loops stay closed, prev/next
    // stay consistent, and each Edge still owns exactly two half-edges.
    // Only the *geometry* gives it away — the two half-edges of an edge no
    // longer span the same segment.  This is the shape of the defect greedy
    // twin pairing can produce at a non-manifold edge.
    auto cube = makeCube();
    auto& faces = const_cast<std::deque<Face>&>(cube->faces());
    HalfEdge* a = faces[0].outerLoop->halfEdge;
    HalfEdge* b = faces[1].outerLoop->halfEdge;
    HalfEdge* ta = a->twin;
    HalfEdge* tb = b->twin;
    ASSERT_NE(ta, nullptr);
    ASSERT_NE(tb, nullptr);

    Edge* edgeA = a->edge;
    Edge* edgeB = b->edge;
    a->twin = b;
    b->twin = a;
    ta->twin = tb;
    tb->twin = ta;
    a->edge = edgeA;
    b->edge = edgeA;
    edgeA->halfEdge = a;
    ta->edge = edgeB;
    tb->edge = edgeB;
    edgeB->halfEdge = ta;

    EXPECT_TRUE(cube->checkManifold()) << cube->validationReport();
    EXPECT_TRUE(cube->checkEulerFormula());
    EXPECT_TRUE(cube->isValid()) << "structural validation must still pass";

    const auto issues = GV::check(*cube);
    EXPECT_FALSE(issues.ok()) << GV::report(*cube);
    EXPECT_GT(issues.vertexChainErrors, 0);
    EXPECT_GT(issues.twinCoincidenceErrors, 0);
    EXPECT_NE(GV::report(*cube).find("Vertex chain errors"), std::string::npos);
}

TEST(GeometryValidatorTest, OpenShellIsCaught) {
    // Cube with the top face removed: still a coherent set of faces, but the
    // area vectors no longer cancel.
    const std::vector<Vec3> pts = {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0},
                                   {0, 0, 1}, {1, 0, 1}, {1, 1, 1}, {0, 1, 1}};
    const std::vector<std::vector<int>> faces = {
        {0, 3, 2, 1}, {0, 1, 5, 4}, {1, 2, 6, 5}, {2, 3, 7, 6}, {3, 0, 4, 7},
    };
    auto solid = Assembler::build(pts, faces);

    const auto issues = GV::check(*solid);
    EXPECT_EQ(issues.openShells, 1) << GV::report(*solid);
    EXPECT_FALSE(issues.ok());
}

TEST(GeometryValidatorTest, InconsistentFaceOrientationIsCaught) {
    // Reverse one face's winding: the skin is closed but no longer coherently
    // oriented, so the area vectors sum to twice that face.
    const std::vector<Vec3> pts = {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0},
                                   {0, 0, 1}, {1, 0, 1}, {1, 1, 1}, {0, 1, 1}};
    const std::vector<std::vector<int>> faces = {
        {0, 3, 2, 1}, {7, 6, 5, 4},  // top reversed (inward)
        {0, 1, 5, 4}, {1, 2, 6, 5}, {2, 3, 7, 6}, {3, 0, 4, 7},
    };
    auto solid = Assembler::build(pts, faces);

    const auto issues = GV::check(*solid);
    EXPECT_EQ(issues.openShells, 1) << GV::report(*solid);
}

// ---------------------------------------------------------------------------
// Advisories: legal but usually unintended.
// ---------------------------------------------------------------------------

TEST(GeometryValidatorTest, EdgeCurveMismatchIsAdvisoryOnly) {
    auto cube = makeCube();
    // Repoint one edge's curve at geometry that is not its own.
    for (auto& e : const_cast<std::deque<Edge>&>(cube->edges())) {
        e.curve = std::make_shared<hz::geo::NurbsCurve>(
            std::vector<Vec3>{Vec3(9, 9, 9), Vec3(9, 9, 10)}, std::vector<double>{1.0, 1.0},
            std::vector<double>{0.0, 0.0, 1.0, 1.0}, 1);
        break;
    }

    const auto issues = GV::check(*cube);
    EXPECT_EQ(issues.edgeCurveMismatches, 1);
    EXPECT_TRUE(issues.ok()) << "advisories must not fail the gate";
    EXPECT_NE(GV::report(*cube).find("with advisories"), std::string::npos);
}

// ---------------------------------------------------------------------------
// Building blocks.
// ---------------------------------------------------------------------------

TEST(GeometryValidatorTest, LoopAreaVectorPointsOutwardWithMagnitudeArea) {
    auto cube = makeCube();
    for (const auto& f : cube->faces()) {
        const Vec3 a = GV::loopAreaVector(f);
        EXPECT_NEAR(a.length(), 1.0, 1e-12);
    }
    // The bottom face is the first one built and faces -Z.
    const Vec3 bottom = GV::loopAreaVector(cube->faces().front());
    EXPECT_NEAR(bottom.z, -1.0, 1e-12);
}

TEST(GeometryValidatorTest, FacePlaneUsesTheLoopWhenNoSurfaceIsBound) {
    auto cube = makeCube();
    Vec3 origin;
    Vec3 normal;
    ASSERT_TRUE(GV::facePlane(cube->faces().front(), origin, normal));
    EXPECT_NEAR(normal.z, -1.0, 1e-12);
}

TEST(GeometryValidatorTest, FacePlaneRejectsACurvedCarrier) {
    auto cube = makeCube();
    Face& f = const_cast<std::deque<Face>&>(cube->faces()).front();
    f.surface = std::make_shared<hz::geo::NurbsSurface>(
        hz::geo::NurbsSurface::makeCylinder(Vec3(0, 0, 0), Vec3(0, 0, 1), 1.0, 1.0));
    Vec3 origin;
    Vec3 normal;
    EXPECT_FALSE(GV::facePlane(f, origin, normal));

    // A curved carrier means planarity is not asserted, not that it fails.
    const auto issues = GV::check(*cube);
    EXPECT_EQ(issues.nonPlanarLoops, 0);
}

TEST(GeometryValidatorTest, FacePlaneAcceptsAPlanarCarrier) {
    auto cube = makeCube();
    Face& f = const_cast<std::deque<Face>&>(cube->faces()).front();
    f.surface = std::make_shared<hz::geo::NurbsSurface>(
        hz::geo::NurbsSurface::makePlane(Vec3(0, 0, 0), Vec3(1, 0, 0), Vec3(0, 1, 0), 1.0, 1.0));
    Vec3 origin;
    Vec3 normal;
    ASSERT_TRUE(GV::facePlane(f, origin, normal));
    EXPECT_NEAR(std::abs(normal.z), 1.0, 1e-9);
    EXPECT_TRUE(GV::check(*cube).ok());
}

TEST(GeometryValidatorTest, LoopOffItsPlanarCarrierIsCaught) {
    // The loop is perfectly planar in itself, but sits off the plane its face
    // claims to lie on — the carrier and the boundary disagree.
    auto cube = makeCube();
    Face& f = const_cast<std::deque<Face>&>(cube->faces()).front();
    f.surface = std::make_shared<hz::geo::NurbsSurface>(
        hz::geo::NurbsSurface::makePlane(Vec3(0, 0, 0.25), Vec3(1, 0, 0), Vec3(0, 1, 0), 1.0, 1.0));

    const auto issues = GV::check(*cube);
    EXPECT_EQ(issues.nonPlanarLoops, 1) << GV::report(*cube);
}

TEST(GeometryValidatorTest, EmptySolidIsVacuouslyValid) {
    Solid empty;
    EXPECT_TRUE(GV::check(empty).ok());
}

// ---------------------------------------------------------------------------
// facePlane on bilinear carriers (Phase 105): decided from the control net
// ---------------------------------------------------------------------------

namespace {

Face faceOn(const Vec3& p00, const Vec3& p10, const Vec3& p01, const Vec3& p11) {
    Face face;
    face.surface = std::make_shared<hz::geo::NurbsSurface>(
        std::vector<std::vector<Vec3>>{{p00, p01}, {p10, p11}},
        std::vector<std::vector<double>>{{1, 1}, {1, 1}}, std::vector<double>{0, 0, 1, 1},
        std::vector<double>{0, 0, 1, 1}, 1, 1);
    return face;
}

}  // namespace

TEST(GeometryValidatorTest, AFlatBilinearFaceFarFromTheOriginIsFlat) {
    // Decided from the control net. (Sampling the normal at nine points called
    // about one in nine faces of a 10,000-box pattern curved, and so never
    // checked them for flatness.)
    const Vec3 o(12345.678, 9876.5, 20000.25);
    const Face face = faceOn(o, o + Vec3(1, 0, 0), o + Vec3(0, 1, 0), o + Vec3(1, 1, 0));
    Vec3 origin, normal;
    ASSERT_TRUE(GV::facePlane(face, origin, normal));
    EXPECT_NEAR(std::abs(normal.z), 1.0, 1e-12);
    EXPECT_NEAR(origin.x, o.x + 0.5, 1e-9);
}

TEST(GeometryValidatorTest, ATwistedBilinearFaceIsCurved) {
    const Face face =
        faceOn(Vec3(0, 0, 0), Vec3(1, 0, 0), Vec3(0, 1, 0), Vec3(1, 1, 0.25));  // one corner lifted
    Vec3 origin, normal;
    EXPECT_FALSE(GV::facePlane(face, origin, normal));
}
