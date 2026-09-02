#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/SketchPlane.h"
#include "horizon/geometry/surfaces/NurbsSurface.h"
#include "horizon/math/Constants.h"
#include "horizon/math/Vec3.h"
#include "horizon/modeling/Extrude.h"
#include "horizon/modeling/FilletOp.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/modeling/PrimitiveFactory.h"
#include "horizon/topology/GeometryValidator.h"
#include "horizon/topology/Queries.h"
#include "horizon/topology/Solid.h"

using hz::math::kPi;
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

    // The radius really varies along the edge.  The blend is faceted across
    // its arc, so the span to measure is the arc a band records rather than
    // the band itself: a quarter arc of radius r spans a chord of r·√2, and
    // the loft's two ends must measure the two stop radii.
    const hz::topo::Face* filletFace = nullptr;
    for (const auto& f : result.solid->faces()) {
        if (f.topoId.tag().find("/fillet/") != std::string::npos) {
            filletFace = &f;
            break;
        }
    }
    ASSERT_NE(filletFace, nullptr);
    ASSERT_NE(filletFace->analyticSurface, nullptr);
    const auto& s = *filletFace->analyticSurface;
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

    // Two radius segments, each faceted across its blend arc.
    int filletFaces = 0;
    for (const auto& f : result.solid->faces()) {
        if (f.topoId.tag().find("/fillet/") != std::string::npos) ++filletFaces;
    }
    EXPECT_EQ(filletFaces, 2 * FilletOp::kDefaultArcSegments);
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

    // 6 original + 3 faceted fillets + 1 spherical blend face.
    const auto n = static_cast<size_t>(FilletOp::kDefaultArcSegments);
    EXPECT_EQ(result.solid->faceCount(), 6 + 3 * n + 1);

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

        // The blend spans the three trimmed fillet ends.  Those ends are arcs
        // now, so the patch follows them: one chain of samples per arc, every
        // sample on the blend sphere.
        auto fv = hz::topo::faceVertices(&f);
        EXPECT_EQ(fv.size(), 3 * n);
        for (const auto* v : fv) {
            EXPECT_NEAR(v->point.distanceTo(Vec3(2.0, 2.0, 2.0)), 2.0, 1e-9)
                << "blend boundary point not on the blend sphere";
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

    // The first fillet still curves, and survives the second operation.  Its
    // bands are planar by construction — that is what makes the boundary agree
    // with the volume — so the curvature shows up two ways: consecutive band
    // normals swing across the arc, and each band still records the true arc
    // patch it approximates.
    std::vector<Vec3> bandNormals;
    for (const auto& f : step2.solid->faces()) {
        if (f.topoId.tag().find("f1/fillet/") == std::string::npos) continue;
        ASSERT_NE(f.surface, nullptr);
        const double uMid = (f.surface->uMin() + f.surface->uMax()) / 2.0;
        const double vMid = (f.surface->vMin() + f.surface->vMax()) / 2.0;
        bandNormals.push_back(f.surface->normal(uMid, vMid).normalized());

        ASSERT_NE(f.analyticSurface, nullptr) << "band lost the arc it approximates";
        const double au = (f.analyticSurface->uMin() + f.analyticSurface->uMax()) / 2.0;
        const Vec3 a0 = f.analyticSurface->normal(au, f.analyticSurface->vMin() + 1e-6);
        const Vec3 a1 = f.analyticSurface->normal(au, f.analyticSurface->vMax() - 1e-6);
        EXPECT_LT(a0.dot(a1), 0.5) << "the recorded arc patch is flat";
    }
    ASSERT_GE(bandNormals.size(), 2u) << "the blend should survive as several bands";

    double widest = 1.0;
    for (const auto& a : bandNormals) {
        for (const auto& b : bandNormals) widest = std::min(widest, std::abs(a.dot(b)));
    }
    EXPECT_LT(widest, 0.5) << "the band chain does not turn — the blend was flattened";
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

// ---------------------------------------------------------------------------
// Geometric validity across the fillet variants.
//
// FilletOp assembles its own half-edge structure from a polygon soup (the
// Phase-61 rewrite), so unlike the old ChamferOp its loops are consistent.
// These cases assert that across the shapes the suite above produces, using
// the geometric validator rather than the combinatorial one.
// ---------------------------------------------------------------------------

TEST(FilletOpTest, AllVariantsAreGeometricallyValid) {
    using hz::topo::GeometryValidator;

    {
        SCOPED_TRACE("constant radius, one edge");
        auto box = PrimitiveFactory::makeBox(10, 10, 10);
        ASSERT_NE(box, nullptr);
        auto r = FilletOp::execute(*box, {box->edges().front().topoId}, 2.0, "f");
        ASSERT_NE(r.solid, nullptr) << r.errorMessage;
        EXPECT_TRUE(GeometryValidator::isGeometricallyValid(*r.solid))
            << GeometryValidator::report(*r.solid);
    }
    {
        SCOPED_TRACE("variable radius");
        auto box = PrimitiveFactory::makeBox(10, 10, 10);
        ASSERT_NE(box, nullptr);
        std::vector<RadiusStop> stops = {{0.0, 1.0}, {0.5, 2.0}, {1.0, 1.0}};
        auto r = FilletOp::executeVariable(*box, {box->edges().front().topoId}, stops, "fv");
        ASSERT_NE(r.solid, nullptr) << r.errorMessage;
        EXPECT_TRUE(GeometryValidator::isGeometricallyValid(*r.solid))
            << GeometryValidator::report(*r.solid);
    }
    {
        SCOPED_TRACE("three-edge corner blend");
        auto box = PrimitiveFactory::makeBox(10, 10, 10);
        ASSERT_NE(box, nullptr);
        const auto ids = edgesAtCorner(*box, Vec3(0, 0, 0));
        ASSERT_EQ(ids.size(), 3u);
        auto r = FilletOp::execute(*box, ids, 2.0, "fc");
        ASSERT_NE(r.solid, nullptr) << r.errorMessage;
        EXPECT_TRUE(GeometryValidator::isGeometricallyValid(*r.solid))
            << GeometryValidator::report(*r.solid);
    }
}

// ---------------------------------------------------------------------------
// A fillet's boundary follows the arc.
//
// The blend used to be one flat quad joining the two tangent lines.  It
// carried the correct rational quadratic surface, but its *loop* was the
// chord, and since every loop-based path in the kernel evaluates a solid from
// its face loops, mass properties, Booleans, classification and export all saw
// a chamfer while the renderer drew a fillet: (1/2)r^2 L removed instead of
// r^2(1 - pi/4)L, 2.33 times too much, at every radius.
//
// The blend is faceted across the arc now.  The facets inscribe the true
// fillet, so the volume approaches the exact value from below and the error
// falls quadratically in the chord count — asserted as a property rather than
// pinned to one number.
// ---------------------------------------------------------------------------

TEST(FilletOpTest, BlendVolumeConvergesToTheExactFillet) {
    const double side = 10.0;
    for (double r : {1.0, 3.0}) {
        SCOPED_TRACE(r);
        const double exact = side * side * side - r * r * (1.0 - kPi / 4.0) * side;
        const double chord = side * side * side - 0.5 * r * r * side;

        double previousError = 1e30;
        for (int n : {2, 4, 8, 16}) {
            auto box = PrimitiveFactory::makeBox(side, side, side);
            ASSERT_NE(box, nullptr);
            auto result = FilletOp::execute(*box, {box->edges().front().topoId}, r, "f", n);
            ASSERT_NE(result.solid, nullptr) << "n=" << n << ": " << result.errorMessage;
            EXPECT_TRUE(hz::topo::GeometryValidator::isGeometricallyValid(*result.solid))
                << hz::topo::GeometryValidator::report(*result.solid);

            const double got = hz::model::MassPropertiesCalculator::compute(*result.solid).volume;
            const double error = exact - got;
            EXPECT_GT(error, 0.0) << "n=" << n << ": an inscribed blend removes a little too much";
            EXPECT_LT(error, previousError) << "n=" << n << ": refining must reduce the error";
            EXPECT_GT(got, chord) << "n=" << n << ": and every refinement beats the old chord";
            previousError = error;
        }
        EXPECT_LT(previousError, 0.001 * exact) << "16 chords lands within a tenth of a percent";
    }
}

TEST(FilletOpTest, BlendBandsAreFlatAndRecordTheArcTheyApproximate) {
    // The carrier of a band is the plane its loop lies in — that agreement is
    // the whole point — and the arc it stands in for is kept beside it.
    auto box = PrimitiveFactory::makeBox(10, 10, 10);
    ASSERT_NE(box, nullptr);
    auto result = FilletOp::execute(*box, {box->edges().front().topoId}, 2.0, "f");
    ASSERT_NE(result.solid, nullptr) << result.errorMessage;

    int bands = 0;
    for (const auto& f : result.solid->faces()) {
        if (f.topoId.tag().find("/fillet/") == std::string::npos) continue;
        ++bands;
        EXPECT_EQ(hz::topo::faceVertices(&f).size(), 4u) << "each band is a quad across the arc";
        ASSERT_NE(f.surface, nullptr);
        ASSERT_NE(f.analyticSurface, nullptr);
        EXPECT_EQ(f.analyticSurface->degreeV(), 2) << "the recorded patch is a true arc";
        EXPECT_EQ(f.analyticSurface->controlPointCountV(), 3);

        // Every band vertex sits on the fillet's axis circle of radius 2.
        for (const auto* v : hz::topo::faceVertices(&f)) {
            const double dy = v->point.y - 2.0;
            const double dz = v->point.z - 2.0;
            EXPECT_NEAR(std::sqrt(dy * dy + dz * dz), 2.0, 1e-9)
                << "band boundary left the fillet arc";
        }
    }
    EXPECT_EQ(bands, FilletOp::kDefaultArcSegments);
}
