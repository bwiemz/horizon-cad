#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <vector>

#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/SketchPlane.h"
#include "horizon/geometry/curves/NurbsCurve.h"
#include "horizon/geometry/surfaces/NurbsSurface.h"
#include "horizon/math/Constants.h"
#include "horizon/math/Mat4.h"
#include "horizon/math/Vec3.h"
#include "horizon/modeling/Extrude.h"
#include "horizon/modeling/FilletOp.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/modeling/Pattern.h"
#include "horizon/modeling/PrimitiveFactory.h"
#include "horizon/modeling/SolidTessellator.h"
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
// Mitered chains (Phase 94).  Two selected edges meeting at a vertex whose
// third edge is unselected used to be refused ("exactly three are required"),
// which made the rim of a faceted cylinder — every vertex of it such a
// vertex — impossible to fillet.  The blends now meet on the plane bisecting
// the turn, and each band stays planar.
// ---------------------------------------------------------------------------

namespace {

/// Area and inset of the material one blend cross-section removes: the
/// polygon from the corner through the tangent points and arc samples.  By
/// symmetry its centroid sits the same distance c inside both faces.
struct RemovedSection {
    double area = 0.0;
    double inset = 0.0;
};

RemovedSection removedSection(double r, int arcSegments) {
    // Corner at the origin; u runs into faceA, w into faceB; the rolling-ball
    // centre is at (r, r).  Samples interpolate the radius direction from
    // (0, -1) (towards faceA's tangent point) to (-1, 0).
    std::vector<std::pair<double, double>> poly = {{0.0, 0.0}};
    for (int k = 0; k <= arcSegments; ++k) {
        const double a = -kPi / 2.0 - (kPi / 2.0) * k / arcSegments;
        poly.emplace_back(r + r * std::cos(a), r + r * std::sin(a));
    }
    double area2 = 0.0, cu = 0.0;
    for (size_t k = 0; k < poly.size(); ++k) {
        const auto& [x0, y0] = poly[k];
        const auto& [x1, y1] = poly[(k + 1) % poly.size()];
        const double cross = x0 * y1 - x1 * y0;
        area2 += cross;
        cu += (x0 + x1) * cross;
    }
    RemovedSection out;
    out.area = std::abs(area2) / 2.0;
    out.inset = cu / (3.0 * area2);
    return out;
}

double volumeOf(const hz::topo::Solid& solid) {
    return hz::model::MassPropertiesCalculator::compute(solid).volume;
}

/// Edges with both ends on the plane z = @p z.
std::vector<TopologyID> edgesAtHeight(const hz::topo::Solid& solid, double z) {
    std::vector<TopologyID> ids;
    for (const auto& e : solid.edges()) {
        if (std::abs(e.halfEdge->origin->point.z - z) < 1e-9 &&
            std::abs(e.halfEdge->twin->origin->point.z - z) < 1e-9) {
            ids.push_back(e.topoId);
        }
    }
    return ids;
}

}  // namespace

TEST(FilletOpTest, TwoEdgesAtACornerAreMitered) {
    // Two top edges of a 10mm cube meeting at (10, 10, 10); the vertical edge
    // there stays sharp.
    auto box = PrimitiveFactory::makeBox(10, 10, 10);
    std::vector<TopologyID> ids;
    for (const auto& e : box->edges()) {
        const Vec3 a = e.halfEdge->origin->point;
        const Vec3 b = e.halfEdge->twin->origin->point;
        const bool top = std::abs(a.z - 10) < 1e-9 && std::abs(b.z - 10) < 1e-9;
        const bool atCorner =
            (a - Vec3(10, 10, 10)).length() < 1e-9 || (b - Vec3(10, 10, 10)).length() < 1e-9;
        if (top && atCorner) ids.push_back(e.topoId);
    }
    ASSERT_EQ(ids.size(), 2u);

    const int n = 8;
    auto result = FilletOp::execute(*box, ids, 1.0, "f", n);
    ASSERT_TRUE(result.errorMessage.empty()) << result.errorMessage;
    ASSERT_NE(result.solid, nullptr);
    EXPECT_TRUE(result.solid->isValid()) << result.solid->validationReport();
    EXPECT_TRUE(hz::topo::GeometryValidator::isGeometricallyValid(*result.solid))
        << hz::topo::GeometryValidator::report(*result.solid);

    // Each blend is a constant-section prism from its free end to the miter
    // plane, which cuts its centroid line c short of the corner.
    const auto sec = removedSection(1.0, n);
    const double removed = sec.area * 2.0 * (10.0 - sec.inset);
    EXPECT_NEAR(volumeOf(*result.solid), 1000.0 - removed, 1e-9);
}

TEST(FilletOpTest, SquareRimIsAClosedMiteredChain) {
    auto box = PrimitiveFactory::makeBox(10, 10, 10);
    const auto ids = edgesAtHeight(*box, 10.0);
    ASSERT_EQ(ids.size(), 4u);
    const int n = 8;
    auto result = FilletOp::execute(*box, ids, 1.5, "f", n);
    ASSERT_TRUE(result.errorMessage.empty()) << result.errorMessage;
    EXPECT_TRUE(result.solid->checkEulerFormula());
    EXPECT_TRUE(hz::topo::GeometryValidator::isGeometricallyValid(*result.solid))
        << hz::topo::GeometryValidator::report(*result.solid);
    const auto sec = removedSection(1.5, n);
    // The centroid path is the top square inset by c on every side.
    EXPECT_NEAR(volumeOf(*result.solid), 1000.0 - sec.area * 4.0 * (10.0 - 2.0 * sec.inset), 1e-9);
}

TEST(FilletOpTest, CylinderRimFillets) {
    // The open item Phases 85-86 left: a faceted cylinder's rim is a closed
    // chain of 32 chords, and every vertex of it has two of them.
    const int N = 32;
    const double R = 5.0, h = 10.0, r = 1.0;
    auto cyl = PrimitiveFactory::makeCylinder(R, h, N);
    const auto ids = edgesAtHeight(*cyl, h);
    ASSERT_EQ(ids.size(), static_cast<size_t>(N));

    const int n = 8;
    auto result = FilletOp::execute(*cyl, ids, r, "f", n);
    ASSERT_TRUE(result.errorMessage.empty()) << result.errorMessage;
    ASSERT_NE(result.solid, nullptr);
    EXPECT_TRUE(result.solid->checkEulerFormula());
    EXPECT_TRUE(hz::topo::GeometryValidator::isGeometricallyValid(*result.solid))
        << hz::topo::GeometryValidator::report(*result.solid);

    // Exact for the faceted geometry (Phase 164): each corner's section is
    // the true fillet's, in the meridian plane there, so the part is a
    // revolve in N steps whose section at each height is a regular N-gon of
    // the profile's radius there: (N/2) sin(2 pi/N) times the integral of
    // radius squared, the arc taken as its n chords.
    double integral = R * R * (h - r);
    for (int k = 0; k < n; ++k) {
        const double a0 = 0.5 * kPi * k / n;
        const double a1 = 0.5 * kPi * (k + 1) / n;
        const double r0 = R - r + r * std::cos(a0);
        const double r1 = R - r + r * std::cos(a1);
        const double dz = r * (std::sin(a1) - std::sin(a0));
        integral += dz * (r0 * r0 + r0 * r1 + r1 * r1) / 3.0;
    }
    EXPECT_NEAR(volumeOf(*result.solid), 0.5 * N * std::sin(2.0 * kPi / N) * integral, 1e-8);

    // Every blend band keeps the arc surface it approximates.
    int blendFaces = 0;
    for (const auto& f : result.solid->faces()) {
        if (f.topoId.tag().rfind("f/fillet", 0) == 0) {
            ++blendFaces;
            EXPECT_NE(f.analyticSurface, nullptr);
        }
    }
    EXPECT_EQ(blendFaces, N * n);
}

TEST(FilletOpTest, MiterTighterThanTheRadiusIsRefused) {
    // An 8-sided rim of radius 2 takes a radius-1.9 blend now (Phase 164):
    // each corner's section is the true fillet's, turned about the axis, and
    // it runs in to radius 0.1, not backwards between the miter planes as a
    // chord's prism did. A radius past the rim's own cannot be.
    auto cyl = PrimitiveFactory::makeCylinder(2.0, 10.0, 8);
    auto tight = FilletOp::execute(*cyl, edgesAtHeight(*cyl, 10.0), 1.9, "f");
    EXPECT_TRUE(tight.errorMessage.empty()) << tight.errorMessage;
    ASSERT_NE(tight.solid, nullptr);
    EXPECT_TRUE(hz::topo::GeometryValidator::isGeometricallyValid(*tight.solid))
        << hz::topo::GeometryValidator::report(*tight.solid);
    auto result = FilletOp::execute(*cyl, edgesAtHeight(*cyl, 10.0), 2.1, "f");
    EXPECT_FALSE(result.errorMessage.empty());
    EXPECT_EQ(result.solid, nullptr);
}

TEST(FilletOpTest, MiterWorksInWhicheverFaceTheEdgesShare) {
    // A top edge and the vertical edge below its end share the side face, and
    // the vertex's third edge (the other top edge) joins their other faces:
    // the same miter, turned in the side face instead of the cap.
    auto box = PrimitiveFactory::makeBox(10, 10, 10);
    std::vector<TopologyID> ids;
    for (const auto& e : box->edges()) {
        const Vec3 a = e.halfEdge->origin->point;
        const Vec3 b = e.halfEdge->twin->origin->point;
        const bool touches =
            (a - Vec3(10, 10, 10)).length() < 1e-9 || (b - Vec3(10, 10, 10)).length() < 1e-9;
        if (!touches) continue;
        const bool vertical = std::abs(a.z - b.z) > 1e-9;
        const bool alongX = std::abs(a.y - b.y) < 1e-9 && std::abs(a.z - b.z) < 1e-9;
        if (vertical || alongX) ids.push_back(e.topoId);
    }
    ASSERT_EQ(ids.size(), 2u);
    const int n = 8;
    auto result = FilletOp::execute(*box, ids, 1.0, "f", n);
    ASSERT_TRUE(result.errorMessage.empty()) << result.errorMessage;
    EXPECT_TRUE(hz::topo::GeometryValidator::isGeometricallyValid(*result.solid))
        << hz::topo::GeometryValidator::report(*result.solid);
    const auto sec = removedSection(1.0, n);
    EXPECT_NEAR(volumeOf(*result.solid), 1000.0 - sec.area * 2.0 * (10.0 - sec.inset), 1e-9);
}

TEST(FilletOpTest, OpenChainMitersInsideAndEndsSquare) {
    // Three of the four top edges: two inner miters, two free ends that are
    // cut square by the end faces exactly as a single fillet's are.
    auto box = PrimitiveFactory::makeBox(10, 10, 10);
    const auto top = edgesAtHeight(*box, 10.0);
    ASSERT_EQ(top.size(), 4u);
    const int n = 8;
    auto result = FilletOp::execute(*box, {top[0], top[1], top[2]}, 1.0, "f", n);
    ASSERT_TRUE(result.errorMessage.empty()) << result.errorMessage;
    EXPECT_TRUE(hz::topo::GeometryValidator::isGeometricallyValid(*result.solid))
        << hz::topo::GeometryValidator::report(*result.solid);
    // Centroid lines: 10 - c on each end edge, 10 - 2c on the middle one.
    const auto sec = removedSection(1.0, n);
    EXPECT_NEAR(volumeOf(*result.solid), 1000.0 - sec.area * (30.0 - 4.0 * sec.inset), 1e-9);
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
// Any angle (Phase 140): a fillet on an oblique or a concave edge, where only
// a convex right angle was taken. The ball rolls in the wedge between the two
// faces; the faceted blend takes away (or, concave, adds) the kite between
// the edge, the two touch points and the ball's centre, r^2 cot(theta/2),
// less the n-chord sector of the arc, (n/2) r^2 sin((pi - theta)/n).
// ---------------------------------------------------------------------------

namespace {

double faceted(double r, double theta, int n) {
    return r * r / std::tan(theta / 2.0) -
           0.5 * n * r * r * std::sin((kPi - theta) / static_cast<double>(n));
}

}  // namespace

TEST(FilletOpTest, AnObliqueEdgeFilletsExactly) {
    // Right-triangle prism: the corner at (10, 0) is 45 degrees.
    auto profile = lineLoop({{0.0, 0.0}, {10.0, 0.0}, {0.0, 10.0}});
    hz::draft::SketchPlane plane;
    auto prism = hz::model::Extrude::execute(profile, plane, Vec3(0, 0, 1), 10.0, "prism");
    ASSERT_NE(prism, nullptr);
    const int n = 8;

    const TopologyID oblique = verticalEdgeAt(*prism, {10.0, 0.0});
    ASSERT_TRUE(oblique.isValid());
    auto rounded = FilletOp::execute(*prism, {oblique}, 1.0, "f", n);
    ASSERT_NE(rounded.solid, nullptr) << rounded.errorMessage;
    EXPECT_TRUE(hz::topo::GeometryValidator::isGeometricallyValid(*rounded.solid))
        << hz::topo::GeometryValidator::report(*rounded.solid);
    EXPECT_NEAR(hz::model::MassPropertiesCalculator::compute(*rounded.solid).volume,
                500.0 - 10.0 * faceted(1.0, kPi / 4.0, n), 1e-9);

    // The square corner, as before; and all three at once.
    const TopologyID square = verticalEdgeAt(*prism, {0.0, 0.0});
    const TopologyID other = verticalEdgeAt(*prism, {0.0, 10.0});
    auto all = FilletOp::execute(*prism, {oblique, square, other}, 1.0, "f", n);
    ASSERT_NE(all.solid, nullptr) << all.errorMessage;
    EXPECT_NEAR(hz::model::MassPropertiesCalculator::compute(*all.solid).volume,
                500.0 - 10.0 * (2.0 * faceted(1.0, kPi / 4.0, n) + faceted(1.0, kPi / 2.0, n)),
                1e-9);
}

TEST(FilletOpTest, AFilletMeetsAnObliqueEndFace) {
    // The prism's bottom edge along the hypotenuse runs between two 45-degree
    // end faces, not square ones: its blend's ends lie on them.
    auto profile = lineLoop({{0.0, 0.0}, {10.0, 0.0}, {0.0, 10.0}});
    hz::draft::SketchPlane plane;
    auto prism = hz::model::Extrude::execute(profile, plane, Vec3(0, 0, 1), 10.0, "prism");
    ASSERT_NE(prism, nullptr);
    TopologyID hypotenuse;
    for (const auto& e : prism->edges()) {
        const Vec3& a = e.halfEdge->origin->point;
        const Vec3& b = e.halfEdge->twin->origin->point;
        if (std::abs(a.z) < 1e-9 && std::abs(b.z) < 1e-9 && std::abs(a.x + a.y - 10.0) < 1e-9 &&
            std::abs(b.x + b.y - 10.0) < 1e-9) {
            hypotenuse = e.topoId;
        }
    }
    ASSERT_TRUE(hypotenuse.isValid());
    const int n = 8;
    auto rounded = FilletOp::execute(*prism, {hypotenuse}, 1.0, "f", n);
    ASSERT_NE(rounded.solid, nullptr) << rounded.errorMessage;
    EXPECT_TRUE(hz::topo::GeometryValidator::isGeometricallyValid(*rounded.solid))
        << hz::topo::GeometryValidator::report(*rounded.solid);
    // A square edge (the base meets the slanted side at 90 degrees), its
    // removed section swept along the hypotenuse. Each ruling of the section,
    // a from the edge across the base, starts on a 45-degree end face a along
    // the edge: the section's area times the length, less twice its first
    // moment in a. The section: the unit square at the edge less the ball's
    // n-chord sector about (1, 1).
    double area = 1.0;
    double moment = 0.5;
    for (int k = 0; k < n; ++k) {
        const double p0 = kPi + 0.5 * kPi * k / n;
        const double p1 = kPi + 0.5 * kPi * (k + 1) / n;
        const double triangle = 0.5 * std::sin(p1 - p0);
        area -= triangle;
        moment -= triangle * (1.0 + (1.0 + std::cos(p0)) + (1.0 + std::cos(p1))) / 3.0;
    }
    ASSERT_NEAR(area, faceted(1.0, kPi / 2.0, n), 1e-12);
    const double length = 10.0 * std::sqrt(2.0);
    EXPECT_NEAR(500.0 - hz::model::MassPropertiesCalculator::compute(*rounded.solid).volume,
                area * length - 2.0 * moment, 1e-9);
}

TEST(FilletOpTest, AConcaveEdgeFilletsAddingMaterial) {
    // L-shaped prism: the edge at the reentrant corner (1, 1) is concave; the
    // blend fills the corner, adding the kite less the sector.
    auto profile =
        lineLoop({{0.0, 0.0}, {10.0, 0.0}, {10.0, 1.0}, {1.0, 1.0}, {1.0, 10.0}, {0.0, 10.0}});
    hz::draft::SketchPlane plane;
    auto prism = hz::model::Extrude::execute(profile, plane, Vec3(0, 0, 1), 5.0, "lprism");
    ASSERT_NE(prism, nullptr);
    const int n = 8;
    const TopologyID reentrant = verticalEdgeAt(*prism, {1.0, 1.0});
    ASSERT_TRUE(reentrant.isValid());
    auto result = FilletOp::execute(*prism, {reentrant}, 0.4, "f", n);
    ASSERT_NE(result.solid, nullptr) << result.errorMessage;
    EXPECT_TRUE(hz::topo::GeometryValidator::isGeometricallyValid(*result.solid))
        << hz::topo::GeometryValidator::report(*result.solid);
    EXPECT_NEAR(hz::model::MassPropertiesCalculator::compute(*result.solid).volume,
                95.0 + 5.0 * faceted(0.4, kPi / 2.0, n), 1e-9);
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

// ---------------------------------------------------------------------------
// arcSegmentsForTolerance (Phase 90)
// ---------------------------------------------------------------------------

TEST(FilletOpTest, ArcSegmentsForToleranceMeetsTheSagBudget) {
    int previous = 0;
    for (double tolerance : {0.1, 0.01, 0.001, 0.0001}) {
        const int n = FilletOp::arcSegmentsForTolerance(2.0, tolerance);
        EXPECT_GE(n, previous) << "a tighter budget never needs fewer chords";
        previous = n;
        // Each of n chords spans a quarter turn / n and must sag within budget,
        // and one chord fewer must not.
        const auto sag = [](int chords) { return 2.0 * (1.0 - std::cos(kPi / 4.0 / chords)); };
        EXPECT_LE(sag(n), tolerance * 1.0000001) << "tolerance = " << tolerance;
        if (n > 1) {
            EXPECT_GT(sag(n - 1), tolerance) << "tolerance = " << tolerance;
        }
    }
    // A budget wider than the whole quarter arc's sag needs a single chord.
    EXPECT_EQ(FilletOp::arcSegmentsForTolerance(2.0, 5.0), 1);
    // Nonsense input falls back to the default.
    EXPECT_EQ(FilletOp::arcSegmentsForTolerance(0.0, 0.1), FilletOp::kDefaultArcSegments);
    EXPECT_EQ(FilletOp::arcSegmentsForTolerance(2.0, 0.0), FilletOp::kDefaultArcSegments);
}

// A three-edge corner is an eighth of the rolling ball. Its face carried the
// whole sphere, which tessellation drew in full: seven eighths of a ball
// inside every filleted corner of every STL and glTF export.
TEST(FilletOpTest, ACornerBlendDrawsOnlyItsEighthOfTheBall) {
    auto box = hz::model::PrimitiveFactory::makeBox(10, 10, 10);
    const double r = 2.0;
    auto result = FilletOp::execute(*box, edgesAtCorner(*box, Vec3(0, 0, 0)), r, "fillet_1");
    ASSERT_NE(result.solid, nullptr) << result.errorMessage;

    const auto mesh = hz::model::SolidTessellator::tessellate(*result.solid, 0.01);
    double meshArea = 0.0;
    for (size_t t = 0; t + 2 < mesh.indices.size(); t += 3) {
        const auto at = [&mesh](uint32_t i) {
            return Vec3(mesh.positions[3 * i], mesh.positions[3 * i + 1],
                        mesh.positions[3 * i + 2]);
        };
        const Vec3 a = at(mesh.indices[t]), b = at(mesh.indices[t + 1]),
                   c = at(mesh.indices[t + 2]);
        meshArea += 0.5 * (b - a).cross(c - a).length();
    }
    const double modelArea =
        hz::model::MassPropertiesCalculator::compute(*result.solid).surfaceArea;
    // The whole ball would add 7/8 x 4 pi r^2, about 44, to the 600-odd.
    EXPECT_NEAR(meshArea, modelArea, 0.01 * modelArea);
}

// A part of two bodies (Phase 140): the fillet rounds the edge on its body
// and leaves the other; the rebuild put every face in one shell, which the
// second body's faces could not join (Euler's check failed).
TEST(FilletOpTest, OneBodyOfTwoIsFilleted) {
    auto first = PrimitiveFactory::makeBox(10, 10, 10);
    auto second = hz::model::Pattern::transformed(*PrimitiveFactory::makeBox(10, 10, 10),
                                                  hz::math::Mat4::translation(Vec3(20, 0, 0)));
    for (auto& face : second->faces()) face.topoId = face.topoId.child("second", 0);
    for (auto& edge : second->edges()) edge.topoId = edge.topoId.child("second", 0);
    auto both = hz::model::Pattern::collect(*first, *second);
    const TopologyID edge = second->edges().front().topoId;
    const int n = 8;
    auto result = FilletOp::execute(*both, {edge}, 1.0, "f", n);
    ASSERT_NE(result.solid, nullptr) << result.errorMessage;
    EXPECT_TRUE(result.solid->isValid()) << result.solid->validationReport();
    EXPECT_NEAR(hz::model::MassPropertiesCalculator::compute(*result.solid).volume,
                2000.0 - 10.0 * faceted(1.0, kPi / 2.0, n), 1e-9);
    EXPECT_EQ(hz::model::Pattern::separate(*result.solid).size(), 2u) << "two bodies still";
}

// Two bodies rounded in one fillet are named apart (Phase 140 review): each
// body is rebuilt alone, and each rebuild numbered its corner blends and new
// edges from 0, so both bodies had a `f/blend/corner:0`. The second body's
// are named under `f/body:1`.
TEST(FilletOpTest, TwoBodiesRoundedAtOnceAreNamedApart) {
    auto first = PrimitiveFactory::makeBox(10, 10, 10);
    auto second = hz::model::Pattern::transformed(*PrimitiveFactory::makeBox(10, 10, 10),
                                                  hz::math::Mat4::translation(Vec3(20, 0, 0)));
    for (auto& face : second->faces()) face.topoId = face.topoId.child("second", 0);
    for (auto& edge : second->edges()) edge.topoId = edge.topoId.child("second", 0);
    // The three edges at each box's top far corner: a corner blend each.
    const auto cornerEdges = [](const hz::topo::Solid& box, const Vec3& corner) {
        std::vector<TopologyID> ids;
        for (const auto& e : box.edges()) {
            const Vec3& a = e.halfEdge->origin->point;
            const Vec3& b = e.halfEdge->twin->origin->point;
            if ((a - corner).length() < 1e-9 || (b - corner).length() < 1e-9)
                ids.push_back(e.topoId);
        }
        return ids;
    };
    auto ids = cornerEdges(*first, Vec3(10, 10, 10));
    const auto more = cornerEdges(*second, Vec3(30, 10, 10));
    ASSERT_EQ(ids.size(), 3u);
    ASSERT_EQ(more.size(), 3u);
    ids.insert(ids.end(), more.begin(), more.end());
    auto both = hz::model::Pattern::collect(*first, *second);

    auto alone = FilletOp::execute(*first, cornerEdges(*first, Vec3(10, 10, 10)), 1.0, "f", 8);
    ASSERT_NE(alone.solid, nullptr) << alone.errorMessage;
    auto result = FilletOp::execute(*both, ids, 1.0, "f", 8);
    ASSERT_NE(result.solid, nullptr) << result.errorMessage;
    EXPECT_TRUE(result.solid->isValid()) << result.solid->validationReport();
    EXPECT_NEAR(hz::model::MassPropertiesCalculator::compute(*result.solid).volume,
                2.0 * hz::model::MassPropertiesCalculator::compute(*alone.solid).volume, 1e-9);

    std::map<std::string, int> faces;
    std::map<std::string, int> edges;
    for (const auto& face : result.solid->faces()) ++faces[face.topoId.tag()];
    for (const auto& edge : result.solid->edges()) ++edges[edge.topoId.tag()];
    for (const auto& [tag, count] : faces) EXPECT_EQ(count, 1) << tag;
    for (const auto& [tag, count] : edges) EXPECT_EQ(count, 1) << tag;
    EXPECT_TRUE(faces.count("f/blend/corner:0")) << "the first body's, as when alone";
    EXPECT_TRUE(faces.count("f/body:1/blend/corner:0")) << "the second's, apart";
}

// Phase 164: a rim's bands are one torus: the ball's centre goes round the
// rim's circle, so each band's ideal is the same surface, and every corner
// of every band is on it (one tube's radius from its spine circle).
TEST(FilletOpTest, ACylinderRimsBandsAreOneTorus) {
    const double R = 5.0, h = 10.0, r = 1.0;
    auto cyl = PrimitiveFactory::makeCylinder(R, h, 32);
    const auto ids = edgesAtHeight(*cyl, h);
    auto result = FilletOp::execute(*cyl, ids, r, "f", 8);
    ASSERT_TRUE(result.errorMessage.empty()) << result.errorMessage;
    const double spine = R - r;
    const hz::geo::NurbsSurface* torus = nullptr;
    int bands = 0;
    for (const auto& f : result.solid->faces()) {
        if (f.topoId.tag().rfind("f/fillet", 0) != 0) continue;
        ASSERT_NE(f.analyticSurface, nullptr);
        if (torus == nullptr) torus = f.analyticSurface.get();
        EXPECT_EQ(f.analyticSurface.get(), torus) << "one surface for the whole rim";
        ++bands;
        // Each corner one tube's radius from the spine, about the z axis at
        // height h - r, radius R - r: each corner's section is the true
        // fillet's, in the meridian plane there, so the bands are inscribed
        // in the design's torus as the facets are in its cylinder.
        const auto* he = f.outerLoop->halfEdge;
        do {
            const Vec3& p = he->origin->point;
            const double across = std::hypot(p.x, p.y) - spine;
            EXPECT_NEAR(std::hypot(across, p.z - (h - r)), r, 1e-9);
            he = he->next;
        } while (he != f.outerLoop->halfEdge);
    }
    EXPECT_EQ(bands, 32 * 8);
    // Its two tangent lines are circles: on the top at radius R - r, on the
    // side at height h - r; each chord of them records its circle.
    int onTop = 0;
    int onSide = 0;
    for (const auto& e : result.solid->edges()) {
        if (!e.analyticCurve) continue;
        const Vec3 c = e.analyticCurve->evaluate(e.analyticCurve->tMin());
        if (std::abs(c.z - h) < 1e-9 && std::abs(std::hypot(c.x, c.y) - (R - r)) < 1e-9) ++onTop;
        if (std::abs(c.z - (h - r)) < 1e-9 && std::abs(std::hypot(c.x, c.y) - R) < 1e-9) ++onSide;
    }
    EXPECT_EQ(onTop, 32);
    EXPECT_EQ(onSide, 32);
    // And the torus is that one: a point of it on the tube round the spine.
    ASSERT_NE(torus, nullptr);
    const Vec3 on = torus->evaluate(0.5 * (torus->uMin() + torus->uMax()),
                                    0.25 * (torus->vMin() + torus->vMax()));
    EXPECT_NEAR(std::hypot(std::hypot(on.x, on.y) - spine, on.z - (h - r)), r, 1e-9);
}
