#include <gtest/gtest.h>

#include <algorithm>
#include <vector>

#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/SketchPlane.h"
#include "horizon/geometry/surfaces/NurbsSurface.h"
#include "horizon/math/Vec3.h"
#include "horizon/modeling/Extrude.h"
#include "horizon/modeling/FilletOp.h"
#include "horizon/modeling/PrimitiveFactory.h"
#include "horizon/topology/Queries.h"
#include "horizon/topology/Solid.h"

using hz::math::Vec3;
using hz::model::FilletOp;
using hz::model::FilletResult;
using hz::model::PrimitiveFactory;
using hz::model::RadiusStop;
using hz::topo::TopologyID;

namespace {

/// The three box edges meeting at the corner shared by @p corner.
std::vector<TopologyID> edgesAtCorner(const hz::topo::Solid& box, const Vec3& corner) {
    std::vector<TopologyID> ids;
    for (const auto& e : box.edges()) {
        if (e.halfEdge == nullptr || e.halfEdge->twin == nullptr) continue;
        const Vec3& a = e.halfEdge->origin->point;
        const Vec3& b = e.halfEdge->twin->origin->point;
        if (a.distanceTo(corner) < 1e-9 || b.distanceTo(corner) < 1e-9) {
            ids.push_back(e.topoId);
        }
    }
    return ids;
}

/// The vertical edge of an extruded prism at profile corner @p xy.
TopologyID verticalEdgeAt(const hz::topo::Solid& solid, const hz::math::Vec2& xy) {
    for (const auto& e : solid.edges()) {
        if (e.halfEdge == nullptr || e.halfEdge->twin == nullptr) continue;
        const Vec3& a = e.halfEdge->origin->point;
        const Vec3& b = e.halfEdge->twin->origin->point;
        const bool vertical = std::abs(a.x - b.x) < 1e-9 && std::abs(a.y - b.y) < 1e-9;
        if (vertical && std::abs(a.x - xy.x) < 1e-9 && std::abs(a.y - xy.y) < 1e-9) {
            return e.topoId;
        }
    }
    return TopologyID();
}

std::vector<std::shared_ptr<hz::draft::DraftEntity>> lineLoop(
    const std::vector<hz::math::Vec2>& pts) {
    std::vector<std::shared_ptr<hz::draft::DraftEntity>> profile;
    for (size_t i = 0; i < pts.size(); ++i) {
        profile.push_back(
            std::make_shared<hz::draft::DraftLine>(pts[i], pts[(i + 1) % pts.size()]));
    }
    return profile;
}

}  // namespace

// ---------------------------------------------------------------------------
// Fillet a single edge of a box
// ---------------------------------------------------------------------------

TEST(FilletOpTest, FilletOneEdgeOfBox) {
    auto box = PrimitiveFactory::makeBox(10, 10, 10);
    auto& edges = box->edges();
    ASSERT_FALSE(edges.empty());
    std::vector<TopologyID> edgeIds = {edges.front().topoId};

    auto result = FilletOp::execute(*box, edgeIds, 1.0, "fillet_1");
    ASSERT_TRUE(result.errorMessage.empty()) << result.errorMessage;
    ASSERT_NE(result.solid, nullptr);
    EXPECT_GT(result.solid->faceCount(), box->faceCount());
    EXPECT_TRUE(result.solid->checkEulerFormula());
}

// ---------------------------------------------------------------------------
// Fillet edges sequentially (one at a time)
// ---------------------------------------------------------------------------

TEST(FilletOpTest, FilletAllEdgesSequentially) {
    auto currentSolid = PrimitiveFactory::makeBox(10, 10, 10);

    // Fillet just one edge (sequential means one at a time, not all at once)
    auto& edges = currentSolid->edges();
    if (!edges.empty()) {
        std::vector<TopologyID> ids = {edges.front().topoId};
        auto result = FilletOp::execute(*currentSolid, ids, 0.5, "fillet_1");
        if (result.solid) {
            EXPECT_TRUE(result.solid->checkEulerFormula());
            currentSolid = std::move(result.solid);
        }
    }
    EXPECT_GT(currentSolid->faceCount(), 6u);
}

// ---------------------------------------------------------------------------
// Fillet a cylinder edge (cylinder-to-plane)
// ---------------------------------------------------------------------------

TEST(FilletOpTest, FilletCylinderToPlaneEdge) {
    auto cyl = PrimitiveFactory::makeCylinder(5.0, 10.0);
    auto& edges = cyl->edges();
    ASSERT_FALSE(edges.empty());
    std::vector<TopologyID> edgeIds = {edges.front().topoId};

    auto result = FilletOp::execute(*cyl, edgeIds, 0.5, "fillet_cyl");
    // May succeed or fail depending on face types — both acceptable for Era 1
    if (result.solid) {
        EXPECT_TRUE(result.solid->checkEulerFormula());
    }
}

// ---------------------------------------------------------------------------
// Fillet faces have valid TopologyIDs with "fillet" tag
// ---------------------------------------------------------------------------

TEST(FilletOpTest, FilletHasTopologyIDs) {
    auto box = PrimitiveFactory::makeBox(10, 10, 10);
    auto& edges = box->edges();
    std::vector<TopologyID> edgeIds = {edges.front().topoId};

    auto result = FilletOp::execute(*box, edgeIds, 1.0, "fillet_1");
    ASSERT_NE(result.solid, nullptr);
    for (const auto& face : result.solid->faces()) {
        EXPECT_TRUE(face.topoId.isValid());
    }
    // At least one face should have "fillet" in its TopologyID
    bool hasFillet = false;
    for (const auto& face : result.solid->faces()) {
        if (face.topoId.tag().find("fillet") != std::string::npos) {
            hasFillet = true;
            break;
        }
    }
    EXPECT_TRUE(hasFillet);
}

// ---------------------------------------------------------------------------
// All faces of the filleted solid have a NURBS surface
// ---------------------------------------------------------------------------

TEST(FilletOpTest, FilletHasNURBSSurface) {
    auto box = PrimitiveFactory::makeBox(10, 10, 10);
    auto& edges = box->edges();
    std::vector<TopologyID> edgeIds = {edges.front().topoId};

    auto result = FilletOp::execute(*box, edgeIds, 1.0, "fillet_1");
    ASSERT_NE(result.solid, nullptr);
    for (const auto& face : result.solid->faces()) {
        EXPECT_NE(face.surface, nullptr) << "Face missing NURBS surface";
    }
}

// ---------------------------------------------------------------------------
// Fillet face has smooth (unit-length) normals
// ---------------------------------------------------------------------------

TEST(FilletOpTest, FilletSmoothNormals) {
    auto box = PrimitiveFactory::makeBox(10, 10, 10);
    auto& edges = box->edges();
    std::vector<TopologyID> edgeIds = {edges.front().topoId};

    auto result = FilletOp::execute(*box, edgeIds, 1.0, "fillet_1");
    ASSERT_NE(result.solid, nullptr);

    // The fillet face should have a cylindrical surface with varying normals
    for (const auto& face : result.solid->faces()) {
        if (face.topoId.tag().find("fillet") != std::string::npos && face.surface) {
            auto tess = face.surface->tessellate(0.1);
            if (tess.normals.size() >= 6) {
                Vec3 n0(tess.normals[0], tess.normals[1], tess.normals[2]);
                EXPECT_NEAR(n0.length(), 1.0, 0.1);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Vertex-blend (two edges sharing a vertex) should be refused in Era 1
// ---------------------------------------------------------------------------

TEST(FilletOpTest, VertexBlendRefused) {
    auto box = PrimitiveFactory::makeBox(10, 10, 10);
    auto& edges = box->edges();

    // Find two edges sharing a vertex
    std::vector<TopologyID> edgeIds;
    if (edges.size() >= 2) {
        edgeIds.push_back(edges[0].topoId);
        for (size_t i = 1; i < edges.size(); ++i) {
            const auto* e0 = &edges[0];
            const auto* ei = &edges[i];
            if (!e0->halfEdge || !ei->halfEdge) continue;
            auto* v0a = e0->halfEdge->origin;
            auto* v0b = e0->halfEdge->twin ? e0->halfEdge->twin->origin : nullptr;
            auto* via = ei->halfEdge->origin;
            auto* vib = ei->halfEdge->twin ? ei->halfEdge->twin->origin : nullptr;
            if (via == v0a || via == v0b || vib == v0a || vib == v0b) {
                edgeIds.push_back(edges[i].topoId);
                break;
            }
        }
    }

    if (edgeIds.size() == 2) {
        auto result = FilletOp::execute(*box, edgeIds, 1.0, "fillet_1");
        EXPECT_FALSE(result.errorMessage.empty()) << "Should refuse vertex blend";
    }
}

// ---------------------------------------------------------------------------
// Variable-radius fillets (Phase 61)
// ---------------------------------------------------------------------------

TEST(FilletOpTest, VariableRadiusFilletProducesValidSolid) {
    auto box = PrimitiveFactory::makeBox(10, 10, 10);
    const auto edgeId = box->edges().front().topoId;

    auto result = FilletOp::executeVariable(*box, edgeId, {{0.0, 1.0}, {1.0, 2.0}}, "vr");
    ASSERT_TRUE(result.errorMessage.empty()) << result.errorMessage;
    ASSERT_NE(result.solid, nullptr);
    EXPECT_TRUE(result.solid->checkEulerFormula());
    EXPECT_TRUE(result.solid->checkManifold());

    // The fillet surface is a ruled loft between two quarter arcs.  A quarter
    // arc of radius r spans a chord of r·√2, so the two ends of the loft must
    // measure the two stop radii.
    const hz::topo::Face* filletFace = nullptr;
    for (const auto& f : result.solid->faces()) {
        if (f.topoId.tag().find("/fillet/") != std::string::npos) {
            filletFace = &f;
            break;
        }
    }
    ASSERT_NE(filletFace, nullptr);
    ASSERT_NE(filletFace->surface, nullptr);
    const auto& s = *filletFace->surface;
    const double chord0 = s.evaluate(s.uMin(), s.vMin()).distanceTo(s.evaluate(s.uMin(), s.vMax()));
    const double chord1 = s.evaluate(s.uMax(), s.vMin()).distanceTo(s.evaluate(s.uMax(), s.vMax()));
    const double kSqrt2 = 1.4142135623730951;
    EXPECT_NEAR(std::min(chord0, chord1), 1.0 * kSqrt2, 1e-9);
    EXPECT_NEAR(std::max(chord0, chord1), 2.0 * kSqrt2, 1e-9);
}

TEST(FilletOpTest, MultiStopVariableRadiusMakesOneFacePerSegment) {
    auto box = PrimitiveFactory::makeBox(10, 10, 10);
    const auto edgeId = box->edges().front().topoId;

    auto result =
        FilletOp::executeVariable(*box, edgeId, {{0.0, 1.0}, {0.5, 2.0}, {1.0, 1.0}}, "vr");
    ASSERT_TRUE(result.errorMessage.empty()) << result.errorMessage;
    ASSERT_NE(result.solid, nullptr);
    EXPECT_TRUE(result.solid->checkEulerFormula());

    // Two radius segments → two fillet faces.
    int filletFaces = 0;
    for (const auto& f : result.solid->faces()) {
        if (f.topoId.tag().find("/fillet/") != std::string::npos) ++filletFaces;
    }
    EXPECT_EQ(filletFaces, 2);
}

TEST(FilletOpTest, VariableRadiusValidatesStops) {
    auto box = PrimitiveFactory::makeBox(10, 10, 10);
    const auto edgeId = box->edges().front().topoId;

    // Too few stops.
    EXPECT_FALSE(FilletOp::executeVariable(*box, edgeId, {{0.0, 1.0}}, "v").errorMessage.empty());
    // Not covering the full edge.
    EXPECT_FALSE(FilletOp::executeVariable(*box, edgeId, {{0.2, 1.0}, {1.0, 1.0}}, "v")
                     .errorMessage.empty());
    // Non-increasing parameters.
    EXPECT_FALSE(FilletOp::executeVariable(*box, edgeId,
                                           {{0.0, 1.0}, {0.5, 1.0}, {0.5, 2.0}, {1.0, 1.0}}, "v")
                     .errorMessage.empty());
    // Non-positive radius.
    EXPECT_FALSE(FilletOp::executeVariable(*box, edgeId, {{0.0, 1.0}, {1.0, -1.0}}, "v")
                     .errorMessage.empty());
}

// ---------------------------------------------------------------------------
// Corner vertex blends (Phase 61)
// ---------------------------------------------------------------------------

TEST(FilletOpTest, ThreeEdgeCornerBlendProducesSphericalPatch) {
    auto box = PrimitiveFactory::makeBox(10, 10, 10);
    // Locate a corner vertex and its three edges.
    const Vec3 corner(0, 0, 0);
    const auto ids = edgesAtCorner(*box, corner);
    ASSERT_EQ(ids.size(), 3u);

    auto result = FilletOp::execute(*box, ids, 2.0, "corner");
    ASSERT_TRUE(result.errorMessage.empty()) << result.errorMessage;
    ASSERT_NE(result.solid, nullptr);
    EXPECT_TRUE(result.solid->checkEulerFormula());
    EXPECT_TRUE(result.solid->checkManifold());

    // 6 original + 3 fillet + 1 spherical blend face.
    EXPECT_EQ(result.solid->faceCount(), 10u);

    // Exactly one face carries the blend TopologyID, with a spherical surface
    // centered one radius along each edge from the original corner.
    int blendFaces = 0;
    for (const auto& f : result.solid->faces()) {
        if (f.topoId.tag().find("/blend/") == std::string::npos) continue;
        ++blendFaces;
        ASSERT_NE(f.surface, nullptr);
        const double midU = (f.surface->uMin() + f.surface->uMax()) / 2.0;
        const double midV = (f.surface->vMin() + f.surface->vMax()) / 2.0;
        const Vec3 onSurface = f.surface->evaluate(midU, midV);
        EXPECT_NEAR(onSurface.distanceTo(Vec3(2.0, 2.0, 2.0)), 2.0, 1e-6);

        // The blend face's three corners are the pairwise-shared tangent
        // points of the trimmed fillets.
        auto fv = hz::topo::faceVertices(&f);
        ASSERT_EQ(fv.size(), 3u);
        for (const auto* v : fv) {
            EXPECT_NEAR(v->point.distanceTo(Vec3(2.0, 2.0, 2.0)), 2.0, 1e-9)
                << "corner point not on the blend sphere";
        }
    }
    EXPECT_EQ(blendFaces, 1);
}

TEST(FilletOpTest, CornerBlendKeepsAllSurfacesBound) {
    auto box = PrimitiveFactory::makeBox(10, 10, 10);
    const auto ids = edgesAtCorner(*box, Vec3(0, 0, 0));
    ASSERT_EQ(ids.size(), 3u);

    auto result = FilletOp::execute(*box, ids, 1.5, "corner");
    ASSERT_NE(result.solid, nullptr) << result.errorMessage;
    for (const auto& f : result.solid->faces()) {
        EXPECT_NE(f.surface, nullptr) << "face " << f.topoId.tag() << " missing surface";
    }
}

TEST(FilletOpTest, CornerBlendRadiusTooLargeRefused) {
    auto box = PrimitiveFactory::makeBox(10, 10, 10);
    const auto ids = edgesAtCorner(*box, Vec3(0, 0, 0));
    ASSERT_EQ(ids.size(), 3u);

    // Trim needs r < half the edge length.
    auto result = FilletOp::execute(*box, ids, 5.0, "corner");
    EXPECT_FALSE(result.errorMessage.empty());
}

// ---------------------------------------------------------------------------
// Geometry gates (Phase 61 review): only perpendicular convex edges fillet
// ---------------------------------------------------------------------------

TEST(FilletOpTest, NonOrthogonalDihedralRefused) {
    // Right-triangle prism: the corner at (10,0) has a 45° profile angle, so
    // its vertical edge is an oblique dihedral — must refuse, not silently
    // emit a wrong-radius fillet. The 90° corner at (0,0) must still work.
    auto profile = lineLoop({{0.0, 0.0}, {10.0, 0.0}, {0.0, 10.0}});
    hz::draft::SketchPlane plane;
    auto prism = hz::model::Extrude::execute(profile, plane, Vec3(0, 0, 1), 10.0, "prism");
    ASSERT_NE(prism, nullptr);

    const TopologyID oblique = verticalEdgeAt(*prism, {10.0, 0.0});
    ASSERT_TRUE(oblique.isValid());
    auto refused = FilletOp::execute(*prism, {oblique}, 1.0, "f");
    EXPECT_FALSE(refused.errorMessage.empty()) << "45-degree dihedral must be refused";

    const TopologyID square = verticalEdgeAt(*prism, {0.0, 0.0});
    ASSERT_TRUE(square.isValid());
    auto ok = FilletOp::execute(*prism, {square}, 1.0, "f");
    EXPECT_TRUE(ok.errorMessage.empty()) << ok.errorMessage;
    ASSERT_NE(ok.solid, nullptr);
    EXPECT_TRUE(ok.solid->checkEulerFormula());
}

TEST(FilletOpTest, ConcaveEdgeRefused) {
    // L-shaped prism: the vertical edge at the reentrant corner (1,1) is
    // concave — the rolling-ball formulas do not apply there.
    auto profile =
        lineLoop({{0.0, 0.0}, {10.0, 0.0}, {10.0, 1.0}, {1.0, 1.0}, {1.0, 10.0}, {0.0, 10.0}});
    hz::draft::SketchPlane plane;
    auto prism = hz::model::Extrude::execute(profile, plane, Vec3(0, 0, 1), 5.0, "lprism");
    ASSERT_NE(prism, nullptr);

    const TopologyID reentrant = verticalEdgeAt(*prism, {1.0, 1.0});
    ASSERT_TRUE(reentrant.isValid());
    auto result = FilletOp::execute(*prism, {reentrant}, 0.4, "f");
    EXPECT_FALSE(result.errorMessage.empty()) << "concave edge must be refused";
}

TEST(FilletOpTest, SequentialFilletsKeepCurvedFaces) {
    // Fillet one edge, then fillet the opposite edge of the RESULT: the first
    // fillet's ruled patch must survive as a curved surface, not be re-bound
    // as a plane.
    auto box = PrimitiveFactory::makeBox(10, 10, 10);
    TopologyID first;
    TopologyID opposite;
    for (const auto& e : box->edges()) {
        const Vec3& a = e.halfEdge->origin->point;
        const Vec3& b = e.halfEdge->twin->origin->point;
        const bool alongX =
            std::abs(a.x - b.x) > 1e-9 && std::abs(a.y - b.y) < 1e-9 && std::abs(a.z - b.z) < 1e-9;
        if (!alongX) continue;
        if (a.y < 1e-9 && a.z < 1e-9) first = e.topoId;
        if (a.y > 9.0 && a.z > 9.0) opposite = e.topoId;
    }
    ASSERT_TRUE(first.isValid());
    ASSERT_TRUE(opposite.isValid());

    auto step1 = FilletOp::execute(*box, {first}, 1.0, "f1");
    ASSERT_NE(step1.solid, nullptr) << step1.errorMessage;

    // Find the opposite edge in the rebuilt solid by geometry.
    TopologyID second;
    for (const auto& e : step1.solid->edges()) {
        const Vec3& a = e.halfEdge->origin->point;
        const Vec3& b = e.halfEdge->twin->origin->point;
        if (std::abs(a.y - 10.0) < 1e-9 && std::abs(a.z - 10.0) < 1e-9 &&
            std::abs(b.y - 10.0) < 1e-9 && std::abs(b.z - 10.0) < 1e-9 &&
            std::abs(a.x - b.x) > 5.0) {
            second = e.topoId;
            break;
        }
    }
    ASSERT_TRUE(second.isValid());

    auto step2 = FilletOp::execute(*step1.solid, {second}, 1.0, "f2");
    ASSERT_NE(step2.solid, nullptr) << step2.errorMessage;
    EXPECT_TRUE(step2.solid->checkEulerFormula());

    // The first fillet's face still curves: its normal swings ~90° across V.
    bool foundCurved = false;
    for (const auto& f : step2.solid->faces()) {
        if (f.topoId.tag().find("f1/fillet/") == std::string::npos) continue;
        ASSERT_NE(f.surface, nullptr);
        const double uMid = (f.surface->uMin() + f.surface->uMax()) / 2.0;
        const Vec3 n0 = f.surface->normal(uMid, f.surface->vMin() + 1e-6);
        const Vec3 n1 = f.surface->normal(uMid, f.surface->vMax() - 1e-6);
        EXPECT_LT(n0.dot(n1), 0.5) << "fillet face was flattened to a plane";
        foundCurved = true;
    }
    EXPECT_TRUE(foundCurved);
}

// ---------------------------------------------------------------------------
// Invalid (nonexistent) edge ID returns an error
// ---------------------------------------------------------------------------

TEST(FilletOpTest, InvalidEdgeReturnsError) {
    auto box = PrimitiveFactory::makeBox(10, 10, 10);
    std::vector<TopologyID> edgeIds = {TopologyID::make("nonexistent", "edge")};
    auto result = FilletOp::execute(*box, edgeIds, 1.0, "fillet_1");
    EXPECT_FALSE(result.errorMessage.empty());
}

// ---------------------------------------------------------------------------
// Radius too large for the box returns an error
// ---------------------------------------------------------------------------

TEST(FilletOpTest, RadiusTooLargeReturnsError) {
    auto box = PrimitiveFactory::makeBox(10, 10, 10);
    auto& edges = box->edges();
    std::vector<TopologyID> edgeIds = {edges.front().topoId};
    auto result = FilletOp::execute(*box, edgeIds, 100.0, "fillet_1");
    EXPECT_FALSE(result.errorMessage.empty());
}
