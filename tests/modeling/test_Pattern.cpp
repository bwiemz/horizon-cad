#include <gtest/gtest.h>

#include <cmath>
#include <numbers>
#include <set>

#include "horizon/geometry/curves/NurbsCurve.h"
#include "horizon/geometry/surfaces/NurbsSurface.h"
#include "horizon/math/Mat4.h"
#include "horizon/modeling/BooleanOp.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/modeling/MateGeometry.h"
#include "horizon/modeling/Pattern.h"
#include "horizon/modeling/PrimitiveFactory.h"
#include "horizon/topology/GeometryValidator.h"
#include "horizon/topology/Queries.h"
#include "horizon/topology/Solid.h"
#include "horizon/topology/TopologyID.h"

using namespace hz::model;
using hz::math::Vec3;

// ---------------------------------------------------------------------------
// LinearPatternReplicatesBox
// ---------------------------------------------------------------------------

TEST(PatternTest, LinearPatternReplicatesBox) {
    auto box = PrimitiveFactory::makeBox(2.0, 2.0, 2.0);
    ASSERT_NE(box, nullptr);

    // 4 boxes spaced 5 apart along +X.
    auto pattern = Pattern::linear(*box, Vec3(1, 0, 0), 5.0, 4);
    ASSERT_NE(pattern, nullptr);

    // Each box: 8V, 12E, 6F, 1 shell → 4 copies.
    EXPECT_EQ(pattern->vertexCount(), 32u);
    EXPECT_EQ(pattern->edgeCount(), 48u);
    EXPECT_EQ(pattern->faceCount(), 24u);
    EXPECT_EQ(pattern->shellCount(), 4u);
    EXPECT_TRUE(pattern->checkEulerFormula());
    EXPECT_TRUE(pattern->checkManifold());

    // Instances span x in [0, 2] .. [15, 17].
    double minX = 1e9, maxX = -1e9;
    for (const auto& v : pattern->vertices()) {
        minX = std::min(minX, v.point.x);
        maxX = std::max(maxX, v.point.x);
    }
    EXPECT_NEAR(minX, 0.0, 1e-9);
    EXPECT_NEAR(maxX, 17.0, 1e-9);  // 3*5 + 2
}

// ---------------------------------------------------------------------------
// LinearPatternTopologyIds
// ---------------------------------------------------------------------------

TEST(PatternTest, LinearPatternTopologyIds) {
    auto box = PrimitiveFactory::makeBox(2.0, 2.0, 2.0);
    auto pattern = Pattern::linear(*box, Vec3(1, 0, 0), 5.0, 3);
    ASSERT_NE(pattern, nullptr);

    // Instance 0 keeps "box/top"; instances 1,2 get pattern children.
    std::set<std::string> tags;
    for (const auto& f : pattern->faces()) tags.insert(f.topoId.tag());

    EXPECT_TRUE(tags.count("box/top"));            // seed
    EXPECT_TRUE(tags.count("box/top/pattern:1"));  // copy 1
    EXPECT_TRUE(tags.count("box/top/pattern:2"));  // copy 2

    // The genealogy relationship holds.
    auto seed = hz::topo::TopologyID::make("box", "top");
    auto copy1 = seed.child("pattern", 1);
    EXPECT_TRUE(copy1.isDescendantOf(seed));
}

// ---------------------------------------------------------------------------
// LinearPatternSuppression
// ---------------------------------------------------------------------------

TEST(PatternTest, LinearPatternSuppression) {
    auto box = PrimitiveFactory::makeBox(2.0, 2.0, 2.0);
    // 5 instances, suppress indices 1 and 3 → 3 bodies.
    auto pattern = Pattern::linear(*box, Vec3(1, 0, 0), 5.0, 5, {1, 3});
    ASSERT_NE(pattern, nullptr);
    EXPECT_EQ(pattern->shellCount(), 3u);
    EXPECT_EQ(pattern->faceCount(), 18u);
}

// ---------------------------------------------------------------------------
// CircularPatternPlacesInstances
// ---------------------------------------------------------------------------

TEST(PatternTest, CircularPatternPlacesInstances) {
    // A small box offset from the origin, patterned 4x at 90 degrees about Z.
    auto box = PrimitiveFactory::makeBox(1.0, 1.0, 1.0);
    // Shift it out along +X so rotation moves it around.
    for (auto& v : const_cast<std::deque<hz::topo::Vertex>&>(box->vertices())) {
        v.point.x += 10.0;
    }

    auto pattern = Pattern::circular(*box, Vec3(0, 0, 0), Vec3(0, 0, 1), std::numbers::pi / 2.0, 4);
    ASSERT_NE(pattern, nullptr);
    EXPECT_EQ(pattern->shellCount(), 4u);
    EXPECT_TRUE(pattern->checkEulerFormula());
    EXPECT_TRUE(pattern->checkManifold());

    // The four instances sit near +X, +Y, -X, -Y (radius ~10-11).
    double minX = 1e9, maxX = -1e9, minY = 1e9, maxY = -1e9;
    for (const auto& v : pattern->vertices()) {
        minX = std::min(minX, v.point.x);
        maxX = std::max(maxX, v.point.x);
        minY = std::min(minY, v.point.y);
        maxY = std::max(maxY, v.point.y);
    }
    // Instance at +X reaches x~11; instance rotated to -X reaches x~-10.
    EXPECT_GT(maxX, 10.0);
    EXPECT_LT(minX, -9.0);
    EXPECT_GT(maxY, 10.0);
    EXPECT_LT(minY, -9.0);
}

// ---------------------------------------------------------------------------
// CountOneReturnsSingleBody
// ---------------------------------------------------------------------------

TEST(PatternTest, CountOneReturnsSingleBody) {
    auto box = PrimitiveFactory::makeBox(3.0, 3.0, 3.0);
    auto pattern = Pattern::linear(*box, Vec3(1, 0, 0), 5.0, 1);
    ASSERT_NE(pattern, nullptr);
    EXPECT_EQ(pattern->shellCount(), 1u);
    EXPECT_EQ(pattern->faceCount(), 6u);
    EXPECT_TRUE(pattern->isValid());

    // Invalid count.
    EXPECT_EQ(Pattern::linear(*box, Vec3(1, 0, 0), 5.0, 0), nullptr);
}

// ---------------------------------------------------------------------------
// Overlapping instances (Phase 93).  Instances used to coexist as separate
// shells whatever their spacing, so three 10mm boxes 5 apart integrated to
// 3000 against the 2000 they actually occupy, with faces buried inside the
// part — and every structural and geometric check passed.
// ---------------------------------------------------------------------------

static double volumeOf(const hz::topo::Solid& solid) {
    return MassPropertiesCalculator::compute(solid).volume;
}

TEST(PatternTest, OverlappingInstancesAreMerged) {
    auto box = PrimitiveFactory::makeBox(10.0, 10.0, 10.0);
    auto pattern = Pattern::linear(*box, Vec3(1, 0, 0), 5.0, 3);
    ASSERT_NE(pattern, nullptr);
    EXPECT_EQ(pattern->shellCount(), 1u);
    EXPECT_NEAR(volumeOf(*pattern), 2000.0, 1e-6);
    EXPECT_TRUE(pattern->checkManifold());
    EXPECT_TRUE(hz::topo::GeometryValidator::isGeometricallyValid(*pattern))
        << hz::topo::GeometryValidator::report(*pattern);
}

TEST(PatternTest, TouchingInstancesBecomeOneBody) {
    auto box = PrimitiveFactory::makeBox(10.0, 10.0, 10.0);
    auto pattern = Pattern::linear(*box, Vec3(1, 0, 0), 10.0, 3);
    ASSERT_NE(pattern, nullptr);
    EXPECT_EQ(pattern->shellCount(), 1u) << "a shared face is not a boundary of the part";
    EXPECT_NEAR(volumeOf(*pattern), 3000.0, 1e-6);
    EXPECT_TRUE(pattern->checkManifold());
}

TEST(PatternTest, OnlyInstancesThatMeetAreMerged) {
    // Five instances, suppress the middle one: two touching pairs 30 apart.
    auto box = PrimitiveFactory::makeBox(10.0, 10.0, 10.0);
    auto pattern = Pattern::linear(*box, Vec3(1, 0, 0), 8.0, 5, {2});
    ASSERT_NE(pattern, nullptr);
    EXPECT_EQ(pattern->shellCount(), 2u);
    EXPECT_NEAR(volumeOf(*pattern), 2.0 * 1800.0, 1e-6);
}

TEST(PatternTest, CoincidentCircularInstancesCountOnce) {
    // A cylinder on the pattern axis maps onto itself at every step.
    auto cyl = PrimitiveFactory::makeCylinder(1.0, 2.0);
    const double single = volumeOf(*cyl);
    auto pattern = Pattern::circular(*cyl, Vec3(0, 0, 0), Vec3(0, 0, 1), std::numbers::pi / 2.0, 4);
    ASSERT_NE(pattern, nullptr);
    EXPECT_NEAR(volumeOf(*pattern), single, 1e-6);
}

TEST(PatternTest, InstancesKeepTheIdealTheyApproximate) {
    // Each instance of a faceted boss must still resolve to its own cylinder
    // from a single pick, so the ideal is carried and moved with the facets.
    auto cyl = PrimitiveFactory::makeCylinder(2.0, 5.0);
    auto pattern = Pattern::linear(*cyl, Vec3(1, 0, 0), 10.0, 3);
    ASSERT_NE(pattern, nullptr);
    ASSERT_EQ(pattern->shellCount(), 3u);
    std::set<long> axesSeen;
    for (const auto& f : pattern->faces()) {
        if (f.topoId.tag().find("side") == std::string::npos) continue;
        ASSERT_NE(f.analyticSurface, nullptr) << f.topoId.tag();
        auto frame = MateGeometry::frameForFace(f);
        ASSERT_TRUE(frame.has_value());
        EXPECT_EQ(frame->kind, MateFrameKind::Cylindrical);
        EXPECT_NEAR(frame->radius, 2.0, 1e-6);
        // The axis passes through x = 0, 10 or 20.
        const double x = frame->origin.x;
        EXPECT_NEAR(x, std::round(x / 10.0) * 10.0, 1e-6);
        axesSeen.insert(std::lround(x / 10.0));
    }
    EXPECT_EQ(axesSeen.size(), 3u);
    for (const auto& e : pattern->edges()) {
        const Vec3 a = e.halfEdge->origin->point;
        const Vec3 b = e.halfEdge->twin->origin->point;
        if (std::abs(a.z - b.z) < 1e-12) {
            EXPECT_NE(e.analyticCurve, nullptr) << "rim chord";
        }
    }
}

// ---------------------------------------------------------------------------
// separate — each body of a multi-body solid on its own (Phase 104b)
// ---------------------------------------------------------------------------

TEST(PatternTest, SeparateUndoesCollect) {
    auto small = PrimitiveFactory::makeBox(1, 1, 1);
    auto large = Pattern::transformed(*PrimitiveFactory::makeBox(2, 2, 2),
                                      hz::math::Mat4::translation(Vec3(10, 0, 0)));
    auto both = Pattern::collect(*small, *large);
    ASSERT_EQ(both->shellCount(), 2u);

    const auto bodies = Pattern::separate(*both);
    ASSERT_EQ(bodies.size(), 2u);
    for (const auto& body : bodies) {
        EXPECT_EQ(body->shellCount(), 1u);
        EXPECT_TRUE(body->isValid()) << body->validationReport();
        EXPECT_TRUE(hz::topo::GeometryValidator::isGeometricallyValid(*body))
            << hz::topo::GeometryValidator::report(*body);
    }
    EXPECT_NEAR(MassPropertiesCalculator::compute(*bodies[0]).volume, 1.0, 1e-9);
    EXPECT_NEAR(MassPropertiesCalculator::compute(*bodies[1]).volume, 8.0, 1e-9);
    EXPECT_EQ(bodies[0]->faceCount(), 6u);
    EXPECT_EQ(bodies[0]->edgeCount(), 12u) << "only the body's own edges";
    EXPECT_EQ(bodies[0]->vertexCount(), 8u);
    EXPECT_EQ(bodies[1]->faces().front().topoId, large->faces().front().topoId) << "names are kept";
}

TEST(PatternTest, SeparatingOneBodyGivesItBack) {
    auto box = PrimitiveFactory::makeCylinder(2.0, 3.0);
    const auto bodies = Pattern::separate(*box);
    ASSERT_EQ(bodies.size(), 1u);
    EXPECT_NEAR(MassPropertiesCalculator::compute(*bodies[0]).volume,
                MassPropertiesCalculator::compute(*box).volume, 1e-9);
    EXPECT_EQ(bodies[0]->faceCount(), box->faceCount());
}

TEST(PatternTest, ACavityStaysWithTheBodyAroundIt) {
    // A box with a closed void inside: the void is a second shell of the same
    // body, facing into it. Separating must not make it a body of its own.
    auto outer = PrimitiveFactory::makeBox(10, 10, 10);
    auto core = Pattern::transformed(*PrimitiveFactory::makeBox(4, 4, 4),
                                     hz::math::Mat4::translation(Vec3(3, 3, 3)));
    auto hollow = BooleanOp::execute(*outer, *core, BooleanType::Subtract);
    ASSERT_NE(hollow, nullptr);
    ASSERT_EQ(hollow->shellCount(), 2u) << "outside and cavity";
    const double volume = MassPropertiesCalculator::compute(*hollow).volume;
    ASSERT_NEAR(volume, 1000.0 - 64.0, 1e-6);

    auto bodies = Pattern::separate(*hollow);
    ASSERT_EQ(bodies.size(), 1u);
    EXPECT_EQ(bodies[0]->shellCount(), 2u);
    EXPECT_NEAR(MassPropertiesCalculator::compute(*bodies[0]).volume, volume, 1e-6);

    // Beside a second, separate body, each keeps what is its own.
    auto apart = Pattern::transformed(*PrimitiveFactory::makeBox(2, 2, 2),
                                      hz::math::Mat4::translation(Vec3(20, 0, 0)));
    auto both = Pattern::collect(*hollow, *apart);
    bodies = Pattern::separate(*both);
    ASSERT_EQ(bodies.size(), 2u);
    EXPECT_NEAR(MassPropertiesCalculator::compute(*bodies[0]).volume, volume, 1e-6);
    EXPECT_NEAR(MassPropertiesCalculator::compute(*bodies[1]).volume, 8.0, 1e-9);
}

// A part moved keeps its curved faces whole: the facets of one cylinder
// still share one cylinder, moved once, and a rim's chords one circle.
// Moved facet by facet, each had a cylinder of its own, and a moved part's
// side was many faces to everything that asks which facets are one.
TEST(PatternTest, AMovedPartsFacetsStillShareTheirSurface) {
    auto cyl = PrimitiveFactory::makeCylinder(4.0, 10.0, 16);
    std::set<const hz::geo::NurbsSurface*> before;
    for (const auto& f : cyl->faces()) {
        if (f.analyticSurface) before.insert(f.analyticSurface.get());
    }
    ASSERT_EQ(before.size(), 1u) << "a cylinder's side facets share one surface";

    const auto moved = Pattern::transformed(*cyl, hz::math::Mat4::translation({7, 0, 3}));
    std::set<const hz::geo::NurbsSurface*> after;
    std::size_t facets = 0;
    for (const auto& f : moved->faces()) {
        if (!f.analyticSurface) continue;
        after.insert(f.analyticSurface.get());
        ++facets;
    }
    EXPECT_EQ(facets, 16u);
    EXPECT_EQ(after.size(), 1u) << "still one, moved";
    EXPECT_NE(*after.begin(), *before.begin()) << "a copy, not the original's";
}

// ---------------------------------------------------------------------------
// Mirrored copies (Phase 162)
// ---------------------------------------------------------------------------

namespace {

double enclosed(const hz::topo::Solid& solid) {
    double v = 0.0;
    for (const auto& shell : solid.shells()) v += hz::topo::signedVolume(shell);
    return v;
}

/// For each face, whether its carrier faces the way its loop does.
std::vector<bool> carriersAgree(const hz::topo::Solid& solid) {
    std::vector<bool> out;
    for (const auto& face : solid.faces()) {
        if (!face.surface) continue;
        const auto& s = *face.surface;
        const Vec3 n = s.normal(0.5 * (s.uMin() + s.uMax()), 0.5 * (s.vMin() + s.vMax()));
        out.push_back(n.dot(hz::topo::loopNormal(&face)) > 0.0);
    }
    return out;
}

}  // namespace

// Through a mirror, a copy is reversed as it is made: it encloses what the
// original does, with the same sign (not inside out), each carrier facing
// as its loop does, and each edge's curve still running from its
// half-edge's origin.
TEST(PatternTest, AMirrorImageFacesOutAsTheOriginalDoes) {
    auto box = PrimitiveFactory::makeBox(2.0, 3.0, 4.0);
    ASSERT_NE(box, nullptr);
    const hz::math::Mat4 mirror = hz::math::Mat4::reflection(Vec3(5, 0, 0), Vec3(1, 0, 0));
    auto image = Pattern::transformed(*box, mirror);
    ASSERT_NE(image, nullptr);
    EXPECT_TRUE(image->checkManifold());
    EXPECT_TRUE(image->checkEulerFormula());
    ASSERT_GT(std::abs(enclosed(*box)), 1.0);
    EXPECT_NEAR(enclosed(*image), enclosed(*box), 1e-9) << "the same sign: not inside out";
    EXPECT_EQ(carriersAgree(*image), carriersAgree(*box));
    for (const auto& v : image->vertices()) {
        EXPECT_GE(v.point.x, 8.0 - 1e-9);
        EXPECT_LE(v.point.x, 10.0 + 1e-9);
    }
    for (const auto& edge : image->edges()) {
        if (!edge.curve || edge.halfEdge == nullptr) continue;
        const Vec3 start = edge.curve->evaluate(edge.curve->tMin());
        EXPECT_NEAR((start - edge.halfEdge->origin->point).length(), 0.0, 1e-9);
    }
}

// A cylinder's true surface is mirrored facing out too: a mate still takes
// its side for a cylinder, and its normal points away from the axis.
TEST(PatternTest, AMirroredCylinderKeepsItsTrueSurfaceFacingOut) {
    auto cylinder = PrimitiveFactory::makeCylinder(2.0, 5.0);
    ASSERT_NE(cylinder, nullptr);
    // A plane through the axis: the image is where the original is.
    auto image =
        Pattern::transformed(*cylinder, hz::math::Mat4::reflection(Vec3(0, 0, 0), Vec3(1, 1, 0)));
    ASSERT_NE(image, nullptr);
    EXPECT_NEAR(enclosed(*image), enclosed(*cylinder), 1e-9);
    const auto outward = [](const hz::topo::Solid& solid) {
        int out = 0;
        int in = 0;
        for (const auto& face : solid.faces()) {
            if (!face.analyticSurface) continue;
            const auto& s = *face.analyticSurface;
            const double u = 0.5 * (s.uMin() + s.uMax());
            const double v = 0.5 * (s.vMin() + s.vMax());
            const Vec3 at = s.evaluate(u, v);
            const Vec3 radial(at.x, at.y, 0.0);
            if (radial.length() < 1e-9) continue;  // a cap's centre
            (s.normal(u, v).dot(radial) > 0.0 ? out : in) += 1;
        }
        return std::pair<int, int>{out, in};
    };
    EXPECT_EQ(outward(*image), outward(*cylinder));
    int cylinders = 0;
    for (const auto& face : image->faces()) {
        if (!face.analyticSurface) continue;
        const auto frame = MateGeometry::frameForFace(face);
        if (frame && frame->kind == MateFrameKind::Cylindrical) {
            EXPECT_NEAR(frame->radius, 2.0, 1e-6);
            ++cylinders;
        }
    }
    EXPECT_GT(cylinders, 0);
}

// A block and its mirror in its own side join into one solid twice its
// size; a drilled block mirrored keeps its volume and its bore.
TEST(PatternTest, AMirrorImageJoinsItsOriginal) {
    auto box = PrimitiveFactory::makeBox(2.0, 3.0, 4.0);
    ASSERT_NE(box, nullptr);
    double maxX = -1e9;
    for (const auto& v : box->vertices()) maxX = std::max(maxX, v.point.x);
    auto image =
        Pattern::transformed(*box, hz::math::Mat4::reflection(Vec3(maxX, 0, 0), Vec3(1, 0, 0)));
    ASSERT_NE(image, nullptr);
    auto joined = BooleanOp::execute(*box, *image, BooleanType::Union);
    ASSERT_NE(joined, nullptr);
    EXPECT_NEAR(MassPropertiesCalculator::compute(*joined).volume, 48.0, 1e-6);
    EXPECT_EQ(joined->shellCount(), 1u);

    auto block = PrimitiveFactory::makeBox(10.0, 10.0, 10.0);
    auto bore = PrimitiveFactory::makeCylinder(2.0, 30.0);
    ASSERT_NE(block, nullptr);
    ASSERT_NE(bore, nullptr);
    auto drilled = BooleanOp::execute(*block, *bore, BooleanType::Subtract);
    ASSERT_NE(drilled, nullptr);
    const double volume = MassPropertiesCalculator::compute(*drilled).volume;
    auto mirrored =
        Pattern::transformed(*drilled, hz::math::Mat4::reflection(Vec3(20, 0, 0), Vec3(1, 0, 0)));
    ASSERT_NE(mirrored, nullptr);
    EXPECT_NEAR(enclosed(*mirrored), enclosed(*drilled), 1e-6);
    EXPECT_NEAR(MassPropertiesCalculator::compute(*mirrored).volume, volume, 1e-6);
    EXPECT_TRUE(hz::topo::GeometryValidator::isGeometricallyValid(*mirrored))
        << hz::topo::GeometryValidator::report(*mirrored);
}
