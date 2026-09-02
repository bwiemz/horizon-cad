#include <gtest/gtest.h>

#include <cmath>
#include <set>
#include <string>
#include <vector>

#include "horizon/math/Constants.h"
#include "horizon/modeling/ChamferOp.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/modeling/PrimitiveFactory.h"
#include "horizon/topology/GeometryValidator.h"
#include "horizon/topology/Solid.h"

using hz::math::kPi;
using hz::model::ChamferOp;
using hz::model::ChamferResult;
using hz::model::MassPropertiesCalculator;
using hz::model::PrimitiveFactory;
using hz::topo::TopologyID;

// ---------------------------------------------------------------------------
// Chamfer a single edge with equal distance
// ---------------------------------------------------------------------------

TEST(ChamferOpTest, ChamferOneEdge) {
    auto box = PrimitiveFactory::makeBox(10, 10, 10);
    auto& edges = box->edges();
    ASSERT_FALSE(edges.empty());
    std::vector<TopologyID> edgeIds = {edges.front().topoId};

    auto result = ChamferOp::executeEqual(*box, edgeIds, 1.0, "chamfer_1");
    ASSERT_TRUE(result.errorMessage.empty()) << result.errorMessage;
    ASSERT_NE(result.solid, nullptr);
    EXPECT_GT(result.solid->faceCount(), box->faceCount());
    EXPECT_TRUE(result.solid->checkEulerFormula());
}

// ---------------------------------------------------------------------------
// Two-distance chamfer
// ---------------------------------------------------------------------------

TEST(ChamferOpTest, TwoDistanceChamfer) {
    auto box = PrimitiveFactory::makeBox(10, 10, 10);
    auto& edges = box->edges();
    ASSERT_FALSE(edges.empty());
    std::vector<TopologyID> edgeIds = {edges.front().topoId};

    auto result = ChamferOp::executeTwoDistance(*box, edgeIds, 1.0, 2.0, "chamfer_1");
    ASSERT_TRUE(result.errorMessage.empty()) << result.errorMessage;
    ASSERT_NE(result.solid, nullptr);
    EXPECT_TRUE(result.solid->checkEulerFormula());
}

// ---------------------------------------------------------------------------
// Chamfer faces have valid TopologyIDs
// ---------------------------------------------------------------------------

TEST(ChamferOpTest, ChamferHasTopologyIDs) {
    auto box = PrimitiveFactory::makeBox(10, 10, 10);
    auto& edges = box->edges();
    ASSERT_FALSE(edges.empty());
    std::vector<TopologyID> edgeIds = {edges.front().topoId};

    auto result = ChamferOp::executeEqual(*box, edgeIds, 1.0, "chamfer_1");
    ASSERT_NE(result.solid, nullptr);
    for (const auto& face : result.solid->faces()) {
        EXPECT_TRUE(face.topoId.isValid());
    }
}

// ---------------------------------------------------------------------------
// Closed-form volumes.
//
// A chamfer of distances (a, b) along an edge of length L removes a
// triangular prism of volume (1/2) a b L.  These are exact, so they are the
// real check on the reconstruction — face counts and Euler consistency were
// already satisfied by the previous implementation while its loop geometry
// was wrong.
// ---------------------------------------------------------------------------

TEST(ChamferOpTest, EqualDistanceRemovesATriangularPrism) {
    auto box = PrimitiveFactory::makeBox(10, 10, 10);
    ASSERT_NE(box, nullptr);
    const auto edgeIds = std::vector<TopologyID>{box->edges().front().topoId};

    auto result = ChamferOp::executeEqual(*box, edgeIds, 1.0, "chamfer_1");
    ASSERT_NE(result.solid, nullptr) << result.errorMessage;
    EXPECT_NEAR(MassPropertiesCalculator::compute(*result.solid).volume,
                1000.0 - 0.5 * 1.0 * 1.0 * 10.0, 1e-9);
}

TEST(ChamferOpTest, TwoDistanceRemovesTheAsymmetricPrism) {
    auto box = PrimitiveFactory::makeBox(10, 10, 10);
    ASSERT_NE(box, nullptr);
    const auto edgeIds = std::vector<TopologyID>{box->edges().front().topoId};

    auto result = ChamferOp::executeTwoDistance(*box, edgeIds, 1.0, 2.0, "chamfer_1");
    ASSERT_NE(result.solid, nullptr) << result.errorMessage;
    EXPECT_NEAR(MassPropertiesCalculator::compute(*result.solid).volume,
                1000.0 - 0.5 * 1.0 * 2.0 * 10.0, 1e-9);
}

TEST(ChamferOpTest, ChamferingTwoParallelEdgesIsAdditive) {
    // Two chamfers that do not share a vertex remove independent prisms.
    auto box = PrimitiveFactory::makeBox(20, 20, 20);
    ASSERT_NE(box, nullptr);

    std::vector<TopologyID> topEdges;
    for (const auto& e : box->edges()) {
        if (e.halfEdge == nullptr || e.halfEdge->twin == nullptr) continue;
        const auto a = e.halfEdge->origin->point;
        const auto b = e.halfEdge->twin->origin->point;
        if (a.z > 19.9 && b.z > 19.9 && std::abs(a.y - b.y) < 1e-9) topEdges.push_back(e.topoId);
    }
    ASSERT_EQ(topEdges.size(), 2u);

    auto result = ChamferOp::executeEqual(*box, topEdges, 3.0, "chamfer_2");
    ASSERT_NE(result.solid, nullptr) << result.errorMessage;
    EXPECT_NEAR(MassPropertiesCalculator::compute(*result.solid).volume,
                8000.0 - 2.0 * 0.5 * 3.0 * 3.0 * 20.0, 1e-9);
}

// ---------------------------------------------------------------------------
// Geometric validity.
//
// The reconstruction is a SolidSewer sew of rewritten face loops, and the op
// refuses to hand back a result the geometric validator rejects — so these
// assert the contract rather than a happy path.
// ---------------------------------------------------------------------------

TEST(ChamferOpTest, ResultIsGeometricallyValidNotJustManifold) {
    auto box = PrimitiveFactory::makeBox(10, 10, 10);
    ASSERT_NE(box, nullptr);
    const auto edgeIds = std::vector<TopologyID>{box->edges().front().topoId};

    auto result = ChamferOp::executeEqual(*box, edgeIds, 2.0, "chamfer_1");
    ASSERT_NE(result.solid, nullptr) << result.errorMessage;
    EXPECT_TRUE(result.solid->isValid()) << result.solid->validationReport();
    EXPECT_TRUE(hz::topo::GeometryValidator::isGeometricallyValid(*result.solid))
        << hz::topo::GeometryValidator::report(*result.solid);
}

TEST(ChamferOpTest, OriginalFacesKeepTheirIdentityAndCarriers) {
    auto box = PrimitiveFactory::makeBox(10, 10, 10);
    ASSERT_NE(box, nullptr);
    const auto edgeIds = std::vector<TopologyID>{box->edges().front().topoId};

    auto result = ChamferOp::executeEqual(*box, edgeIds, 1.0, "chamfer_1");
    ASSERT_NE(result.solid, nullptr) << result.errorMessage;

    // Six originals plus one chamfer face; the originals keep their tags
    // (offsets slide along the neighbouring faces, so no plane moves).
    EXPECT_EQ(result.solid->faceCount(), 7u);
    std::set<std::string> tags;
    for (const auto& f : result.solid->faces()) {
        EXPECT_NE(f.surface, nullptr);
        tags.insert(f.topoId.tag());
    }
    for (const auto* name :
         {"box/bottom", "box/top", "box/front", "box/back", "box/right", "box/left"}) {
        EXPECT_EQ(tags.count(name), 1u) << "missing original face " << name;
    }
}

// ---------------------------------------------------------------------------
// Chained use: a chamfered solid must be a first-class modeling input.
// ---------------------------------------------------------------------------

TEST(ChamferOpTest, ChamfersChain) {
    auto box = PrimitiveFactory::makeBox(20, 20, 20);
    ASSERT_NE(box, nullptr);

    // Chamfer a bottom edge, then a top edge of the result — the two are
    // disjoint, so the removed prisms simply add up.
    std::vector<TopologyID> first;
    for (const auto& e : box->edges()) {
        if (e.halfEdge == nullptr || e.halfEdge->twin == nullptr) continue;
        const auto a = e.halfEdge->origin->point;
        const auto b = e.halfEdge->twin->origin->point;
        if (a.z < 0.1 && b.z < 0.1 && std::abs(a.y - b.y) < 1e-9) {
            first.push_back(e.topoId);
            break;
        }
    }
    ASSERT_EQ(first.size(), 1u);

    auto step1 = ChamferOp::executeEqual(*box, first, 2.0, "c1");
    ASSERT_NE(step1.solid, nullptr) << step1.errorMessage;

    std::vector<TopologyID> second;
    for (const auto& e : step1.solid->edges()) {
        if (e.halfEdge == nullptr || e.halfEdge->twin == nullptr) continue;
        const auto a = e.halfEdge->origin->point;
        const auto b = e.halfEdge->twin->origin->point;
        if (a.z > 19.9 && b.z > 19.9 && std::abs(a.y - b.y) < 1e-9) {
            second.push_back(e.topoId);
            break;
        }
    }
    ASSERT_EQ(second.size(), 1u);

    auto step2 = ChamferOp::executeEqual(*step1.solid, second, 2.0, "c2");
    ASSERT_NE(step2.solid, nullptr) << step2.errorMessage;
    EXPECT_TRUE(hz::topo::GeometryValidator::isGeometricallyValid(*step2.solid))
        << hz::topo::GeometryValidator::report(*step2.solid);
    EXPECT_NEAR(MassPropertiesCalculator::compute(*step2.solid).volume,
                8000.0 - 2.0 * 0.5 * 2.0 * 2.0 * 20.0, 1e-9);
}

// ---------------------------------------------------------------------------
// Refusals.
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Vertex blends: two or three selected edges meeting at a corner.
//
// The chamfer planes cut each other, so the corner is whatever their
// intersection leaves — no extra "corner patch" is invented.  Each case is
// checked against the closed-form volume of the union of the cutting prisms,
// by inclusion–exclusion:
//
//   one prism        (1/2) d^2 L
//   two prisms meet  (1/2) d^2 L * 2  -  d^3/3
//   three prisms     (1/2) d^2 L * 3  -  d^3      +  d^3/4
//
// These are exact, and they are the only thing that distinguishes a correct
// blend from a plausible-looking one — face counts and Euler consistency do
// not.
// ---------------------------------------------------------------------------

namespace {

std::vector<TopologyID> edgesAtCorner(const hz::topo::Solid& solid, const hz::math::Vec3& corner) {
    std::vector<TopologyID> ids;
    for (const auto& e : solid.edges()) {
        if (e.halfEdge == nullptr || e.halfEdge->twin == nullptr) continue;
        const auto& a = e.halfEdge->origin->point;
        const auto& b = e.halfEdge->twin->origin->point;
        if (a.distanceTo(corner) < 1e-9 || b.distanceTo(corner) < 1e-9) ids.push_back(e.topoId);
    }
    return ids;
}

}  // namespace

TEST(ChamferOpTest, ThreeEdgeVertexBlendHasExactVolume) {
    auto box = PrimitiveFactory::makeBox(10, 10, 10);
    ASSERT_NE(box, nullptr);
    const auto ids = edgesAtCorner(*box, hz::math::Vec3(0, 0, 0));
    ASSERT_EQ(ids.size(), 3u);

    const double d = 2.0;
    const double L = 10.0;
    auto result = ChamferOp::executeEqual(*box, ids, d, "blend3");
    ASSERT_NE(result.solid, nullptr) << result.errorMessage;

    // Six originals, three chamfer faces.  The three chamfer planes meet at a
    // single point, so no corner face is added.
    EXPECT_EQ(result.solid->faceCount(), 9u);
    EXPECT_TRUE(result.solid->checkEulerFormula());
    EXPECT_TRUE(hz::topo::GeometryValidator::isGeometricallyValid(*result.solid))
        << hz::topo::GeometryValidator::report(*result.solid);

    const double removed = 3.0 * 0.5 * d * d * L - d * d * d + d * d * d / 4.0;
    EXPECT_NEAR(MassPropertiesCalculator::compute(*result.solid).volume, 1000.0 - removed, 1e-9);
}

TEST(ChamferOpTest, TwoEdgeVertexBlendHasExactVolume) {
    auto box = PrimitiveFactory::makeBox(10, 10, 10);
    ASSERT_NE(box, nullptr);
    const auto corner = edgesAtCorner(*box, hz::math::Vec3(0, 0, 0));
    ASSERT_EQ(corner.size(), 3u);
    const std::vector<TopologyID> ids = {corner[0], corner[1]};

    const double d = 2.0;
    const double L = 10.0;
    auto result = ChamferOp::executeEqual(*box, ids, d, "blend2");
    ASSERT_NE(result.solid, nullptr) << result.errorMessage;
    EXPECT_TRUE(result.solid->checkEulerFormula());
    EXPECT_TRUE(hz::topo::GeometryValidator::isGeometricallyValid(*result.solid))
        << hz::topo::GeometryValidator::report(*result.solid);

    const double removed = 2.0 * 0.5 * d * d * L - d * d * d / 3.0;
    EXPECT_NEAR(MassPropertiesCalculator::compute(*result.solid).volume, 1000.0 - removed, 1e-9);
}

TEST(ChamferOpTest, EveryEdgeOfACubeBlendsIntoAChamferedCube) {
    auto box = PrimitiveFactory::makeBox(10, 10, 10);
    ASSERT_NE(box, nullptr);
    std::vector<TopologyID> ids;
    for (const auto& e : box->edges()) ids.push_back(e.topoId);
    ASSERT_EQ(ids.size(), 12u);

    const double d = 2.0;
    const double L = 10.0;
    auto result = ChamferOp::executeEqual(*box, ids, d, "blendAll");
    ASSERT_NE(result.solid, nullptr) << result.errorMessage;

    // The chamfered cube: 6 squares + 12 chamfer faces, 32 vertices.
    EXPECT_EQ(result.solid->faceCount(), 18u);
    EXPECT_EQ(result.solid->vertexCount(), 32u);
    EXPECT_TRUE(result.solid->isValid()) << result.solid->validationReport();
    EXPECT_TRUE(hz::topo::GeometryValidator::isGeometricallyValid(*result.solid))
        << hz::topo::GeometryValidator::report(*result.solid);

    // 12 prisms; each of the 8 corners contributes 3 pairwise overlaps and
    // one triple overlap.
    const double removed = 12.0 * 0.5 * d * d * L - 24.0 * d * d * d / 3.0 + 8.0 * d * d * d / 4.0;
    EXPECT_NEAR(MassPropertiesCalculator::compute(*result.solid).volume, 1000.0 - removed, 1e-9);
}

TEST(ChamferOpTest, BlendedResultIsAUsableBooleanOperand) {
    auto box = PrimitiveFactory::makeBox(10, 10, 10);
    ASSERT_NE(box, nullptr);
    const auto ids = edgesAtCorner(*box, hz::math::Vec3(0, 0, 0));
    ASSERT_EQ(ids.size(), 3u);

    auto blended = ChamferOp::executeEqual(*box, ids, 2.0, "blend3");
    ASSERT_NE(blended.solid, nullptr) << blended.errorMessage;

    // Chamfer a fourth edge of the result, far from the blended corner.
    TopologyID far;
    for (const auto& e : blended.solid->edges()) {
        if (e.halfEdge == nullptr || e.halfEdge->twin == nullptr) continue;
        const auto& a = e.halfEdge->origin->point;
        const auto& b = e.halfEdge->twin->origin->point;
        if (a.z > 9.9 && b.z > 9.9 && std::abs(a.y - b.y) < 1e-9 && a.y > 9.9) {
            far = e.topoId;
            break;
        }
    }
    ASSERT_TRUE(far.isValid());

    auto second = ChamferOp::executeEqual(*blended.solid, {far}, 1.0, "c2");
    ASSERT_NE(second.solid, nullptr) << second.errorMessage;
    EXPECT_TRUE(hz::topo::GeometryValidator::isGeometricallyValid(*second.solid))
        << hz::topo::GeometryValidator::report(*second.solid);

    const double removed3 = 3.0 * 0.5 * 4.0 * 10.0 - 8.0 + 2.0;
    EXPECT_NEAR(MassPropertiesCalculator::compute(*second.solid).volume,
                1000.0 - removed3 - 0.5 * 1.0 * 1.0 * 10.0, 1e-9);
}

TEST(ChamferOpTest, InvalidInputsAreRefused) {
    auto box = PrimitiveFactory::makeBox(10, 10, 10);
    ASSERT_NE(box, nullptr);
    const auto edgeIds = std::vector<TopologyID>{box->edges().front().topoId};

    EXPECT_EQ(ChamferOp::executeEqual(*box, edgeIds, 0.0, "c").solid, nullptr);
    EXPECT_EQ(ChamferOp::executeEqual(*box, {}, 1.0, "c").solid, nullptr);
    // The cap is the width of the adjacent face, so a 10mm offset on a 10mm
    // box is the extreme case that still builds (it bisects the cube); past
    // that the offset edge has left the face.
    EXPECT_NE(ChamferOp::executeEqual(*box, edgeIds, 10.0, "c").solid, nullptr);
    EXPECT_EQ(ChamferOp::executeEqual(*box, edgeIds, 10.5, "c").solid, nullptr);
    EXPECT_EQ(ChamferOp::executeEqual(*box, {TopologyID::make("nope", "edge")}, 1.0, "c").solid,
              nullptr);
}

// ---------------------------------------------------------------------------
// Chamfering faceted geometry.
//
// Curved primitives are faceted (Phase 84), so "chamfer the rim of a boss" is
// a chain of facet edges meeting two-at-a-vertex — the vertex-blend case.
// Two things used to make that impossible:
//
//  * the capacity check capped the distance at half the shortest edge of the
//    adjacent face, which on a 32-sided cylinder is the facet chord (0.98 at
//    r = 5), refusing anything over 0.49; and
//  * the clip deduplicated at a tighter tolerance than the sewer welds at, so
//    near-coincident points survived into loops as sub-tolerance segments that
//    read downstream as self-intersecting.
//
// The removed material is a truncated cone over the facet cap, which has a
// closed form: for an n-gon cap of circumradius R the area is
// k*R^2 with k = (n/2) sin(2pi/n), and the frustum between R and the shrunken
// R' takes the usual (h/3)(A + sqrt(AA') + A') form.  Because both areas share
// k, that reduces to the expression below.
// ---------------------------------------------------------------------------

namespace {

std::vector<TopologyID> topRimEdges(const hz::topo::Solid& solid, double z) {
    std::vector<TopologyID> rim;
    for (const auto& e : solid.edges()) {
        if (e.halfEdge == nullptr || e.halfEdge->twin == nullptr) continue;
        const auto& a = e.halfEdge->origin->point;
        const auto& b = e.halfEdge->twin->origin->point;
        if (a.z > z - 1e-6 && b.z > z - 1e-6) rim.push_back(e.topoId);
    }
    return rim;
}

/// Material a rim chamfer of distance d removes from an n-faceted cylinder.
double rimChamferWedge(double r, double d, int n) {
    const double k = 0.5 * n * std::sin(2.0 * kPi / n);
    const double apothemRatio = std::cos(kPi / n);
    const double rIn = r - d / apothemRatio;
    return d * (k * r * r) - (d / 3.0) * (k * (r * r + r * rIn + rIn * rIn));
}

}  // namespace

TEST(ChamferOpTest, CylinderRimChamferMatchesTheTruncatedConeItRemoves) {
    const double r = 5.0;
    const double h = 10.0;
    const int n = PrimitiveFactory::kDefaultSegments;

    for (double d : {0.1, 0.5, 1.0, 2.0, 4.0}) {
        SCOPED_TRACE(d);
        auto cyl = PrimitiveFactory::makeCylinder(r, h, n);
        ASSERT_NE(cyl, nullptr);
        const double before = MassPropertiesCalculator::compute(*cyl).volume;

        const auto rim = topRimEdges(*cyl, h);
        ASSERT_EQ(rim.size(), static_cast<size_t>(n));

        auto result = ChamferOp::executeEqual(*cyl, rim, d, "rim");
        ASSERT_NE(result.solid, nullptr) << result.errorMessage;
        EXPECT_TRUE(hz::topo::GeometryValidator::isGeometricallyValid(*result.solid))
            << hz::topo::GeometryValidator::report(*result.solid);

        // Two faces per facet plus the two caps: the cap survives, each lateral
        // facet survives, and each facet edge gains a chamfer face.
        EXPECT_EQ(result.solid->faceCount(), static_cast<size_t>(2 * n + 2));

        const double removed = before - MassPropertiesCalculator::compute(*result.solid).volume;
        EXPECT_NEAR(removed, rimChamferWedge(r, d, n), 1e-6);
    }
}

TEST(ChamferOpTest, RimChamferReachesTheGeometricLimitNotAnEdgeLengthProxy) {
    // The cap's inradius is what actually bounds a rim chamfer: past it the
    // cap has nothing left.  The old proxy stopped at half a facet chord,
    // roughly a tenth of that.
    const double r = 5.0;
    const int n = PrimitiveFactory::kDefaultSegments;
    const double capInradius = r * std::cos(kPi / n);
    EXPECT_GT(capInradius, 4.9);

    auto atDistance = [&](double d) {
        auto cyl = PrimitiveFactory::makeCylinder(r, 10.0, n);
        return ChamferOp::executeEqual(*cyl, topRimEdges(*cyl, 10.0), d, "rim").solid != nullptr;
    };

    EXPECT_TRUE(atDistance(0.49)) << "the old proxy's ceiling";
    EXPECT_TRUE(atDistance(2.0)) << "four times the old ceiling";
    EXPECT_TRUE(atDistance(capInradius - 0.08)) << "up against the real limit";
    EXPECT_FALSE(atDistance(capInradius + 0.5)) << "past the cap, refused";
}

TEST(ChamferOpTest, CapacityIsTheFaceWidthNotHalfItsShortestEdge) {
    // A 10mm chamfer on a 10mm cube bisects it — the extreme case that still
    // builds.  The old check refused anything over 5.
    auto box = PrimitiveFactory::makeBox(10, 10, 10);
    ASSERT_NE(box, nullptr);
    const auto edgeIds = std::vector<TopologyID>{box->edges().front().topoId};

    for (double d : {6.0, 9.9}) {
        SCOPED_TRACE(d);
        auto result = ChamferOp::executeEqual(*box, edgeIds, d, "c");
        ASSERT_NE(result.solid, nullptr) << result.errorMessage;
        EXPECT_NEAR(MassPropertiesCalculator::compute(*result.solid).volume,
                    1000.0 - 0.5 * d * d * 10.0, 1e-9);
        EXPECT_TRUE(hz::topo::GeometryValidator::isGeometricallyValid(*result.solid));
    }

    auto past = ChamferOp::executeEqual(*box, edgeIds, 10.5, "c");
    EXPECT_EQ(past.solid, nullptr);
    EXPECT_NE(past.errorMessage.find("exceeds the width"), std::string::npos) << past.errorMessage;
}
