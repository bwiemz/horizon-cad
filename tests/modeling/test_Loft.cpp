#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <vector>

#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/SketchPlane.h"
#include "horizon/geometry/surfaces/NurbsSurface.h"
#include "horizon/math/Constants.h"
#include "horizon/modeling/Loft.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/modeling/SolidTessellator.h"
#include "horizon/topology/GeometryValidator.h"
#include "horizon/topology/Queries.h"
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

// ---------------------------------------------------------------------------
// Circle sections (Phase 92).  Profile extraction used to skip a DraftCircle
// entirely, so lofting two circles returned nothing; it is now faceted like
// every other profile, and a loft between two coaxial circles is the inscribed
// frustum.
// ---------------------------------------------------------------------------

TEST(LoftTest, CircleToCircleIsTheInscribedFrustum) {
    std::vector<LoftSection> sections(2);
    sections[0].profile = {std::make_shared<DraftCircle>(Vec2(0, 0), 4.0)};
    sections[0].plane = SketchPlane(Vec3(0, 0, 0), Vec3(0, 0, 1), Vec3(1, 0, 0));
    sections[1].profile = {std::make_shared<DraftCircle>(Vec2(0, 0), 2.0)};
    sections[1].plane = SketchPlane(Vec3(0, 0, 6), Vec3(0, 0, 1), Vec3(1, 0, 0));

    auto solid = Loft::execute(sections, "loft_c");
    ASSERT_NE(solid, nullptr);
    EXPECT_TRUE(GeometryValidator::isGeometricallyValid(*solid))
        << GeometryValidator::report(*solid);

    // Two similar regular 32-gons of area A = (n/2) r^2 sin(2pi/n):
    // V = h/3 (A1 + A2 + sqrt(A1 A2)).
    const double n = 32.0;
    const double k = 0.5 * n * std::sin(2.0 * hz::math::kPi / n);
    const double a1 = k * 16.0;
    const double a2 = k * 4.0;
    const double exact = 6.0 / 3.0 * (a1 + a2 + std::sqrt(a1 * a2));
    EXPECT_NEAR(MassPropertiesCalculator::compute(*solid).volume, exact, 1e-9);
}

// ---------------------------------------------------------------------------
// Twisted sections (Phase 95).  A band between a square and the same square
// turned has four corners that are not coplanar, so its loop encloses no
// well-defined volume: mass properties and Booleans fanned it along one
// diagonal and integrated 180.8 for a 0.6 rad twist, while the renderer drew
// the ruled patch, which encloses 150.7.
// ---------------------------------------------------------------------------

namespace {

std::vector<std::shared_ptr<DraftEntity>> turnedSquare(double s, double angle) {
    const double h = s * 0.5;
    std::vector<Vec2> c = {{-h, -h}, {h, -h}, {h, h}, {-h, h}};
    for (auto& p : c) {
        p = Vec2(p.x * std::cos(angle) - p.y * std::sin(angle),
                 p.x * std::sin(angle) + p.y * std::cos(angle));
    }
    std::vector<std::shared_ptr<DraftEntity>> out;
    for (size_t i = 0; i < 4; ++i) out.push_back(std::make_shared<DraftLine>(c[i], c[(i + 1) % 4]));
    return out;
}

/// The ruled loft between the two squares: its section at height t is the
/// quad through the linearly interpolated corners, whose area is quadratic
/// in t, so Simpson's rule is exact.
double ruledVolume(double s, double angle, double height) {
    auto area = [&](double t) {
        const double h = s * 0.5;
        const Vec2 base[4] = {{-h, -h}, {h, -h}, {h, h}, {-h, h}};
        Vec2 q[4];
        for (int i = 0; i < 4; ++i) {
            const Vec2 turned(base[i].x * std::cos(angle) - base[i].y * std::sin(angle),
                              base[i].x * std::sin(angle) + base[i].y * std::cos(angle));
            q[i] = base[i] * (1.0 - t) + turned * t;
        }
        double a2 = 0.0;
        for (int i = 0; i < 4; ++i) a2 += q[i].x * q[(i + 1) % 4].y - q[(i + 1) % 4].x * q[i].y;
        return std::abs(a2) / 2.0;
    };
    return height / 6.0 * (area(0.0) + 4.0 * area(0.5) + area(1.0));
}

double displayedVolume(const Solid& solid) {
    const auto mesh = SolidTessellator::tessellate(solid, 0.001);
    double v6 = 0.0;
    for (size_t k = 0; k + 2 < mesh.indices.size(); k += 3) {
        auto at = [&](uint32_t i) {
            return Vec3(mesh.positions[3 * i], mesh.positions[3 * i + 1],
                        mesh.positions[3 * i + 2]);
        };
        v6 += at(mesh.indices[k]).dot(at(mesh.indices[k + 1]).cross(at(mesh.indices[k + 2])));
    }
    return std::abs(v6) / 6.0;
}

}  // namespace

TEST(LoftTest, TwistedLoftEnclosesTheRuledVolume) {
    for (double angle : {0.3, 0.6, 1.0}) {
        std::vector<LoftSection> sections = {{squareProfile(4.0), planeAtZ(0.0)},
                                             {turnedSquare(4.0, angle), planeAtZ(10.0)}};
        auto solid = Loft::execute(sections, "twist");
        ASSERT_NE(solid, nullptr) << "angle " << angle;
        EXPECT_TRUE(GeometryValidator::isGeometricallyValid(*solid))
            << GeometryValidator::report(*solid);

        // Loft re-indexes a section to minimize twist, so a square turned
        // past 45 degrees is ruled to the corner a quarter turn back.
        const double ruled = angle > hz::math::kPi / 4.0 ? angle - hz::math::kPi / 2.0 : angle;
        const double exact = ruledVolume(4.0, ruled, 10.0);
        const double computed = MassPropertiesCalculator::compute(*solid).volume;
        EXPECT_NEAR(computed, exact, 1e-9) << "angle " << angle;
        // What is drawn is what is computed (the mesh is single precision).
        EXPECT_NEAR(displayedVolume(*solid), computed, 1e-5 * computed) << "angle " << angle;
    }
}

TEST(LoftTest, TwistedBandsRecordTheirRuledPatch) {
    std::vector<LoftSection> sections = {{squareProfile(4.0), planeAtZ(0.0)},
                                         {turnedSquare(4.0, 0.6), planeAtZ(10.0)}};
    auto solid = Loft::execute(sections, "twist", 4);
    ASSERT_NE(solid, nullptr);
    int facets = 0;
    for (const auto& f : solid->faces()) {
        if (f.topoId.tag().find("lateral") == std::string::npos) continue;
        ++facets;
        ASSERT_NE(f.analyticSurface, nullptr) << f.topoId.tag();
        // Every facet corner lies on the patch it approximates: the ruled
        // surface passes through every point of every ruling.
        const auto& patch = *f.analyticSurface;
        for (const auto* v : faceVertices(&f)) {
            const auto [u, w] = patch.closestPoint(v->point, 1e-12);
            EXPECT_NEAR((patch.evaluate(u, w) - v->point).length(), 0.0, 1e-6);
        }
    }
    // Four bands of four strips, each strip two triangles.
    EXPECT_EQ(facets, 4 * 4 * 2);
}

TEST(LoftTest, UntwistedLoftStaysOneQuadPerBand) {
    std::vector<LoftSection> sections = {{squareProfile(6.0), planeAtZ(0.0)},
                                         {squareProfile(3.0), planeAtZ(10.0)}};
    auto solid = Loft::execute(sections, "frustum", 16);
    ASSERT_NE(solid, nullptr);
    EXPECT_EQ(solid->faceCount(), 6u) << "a planar band needs no facets";
}
