// Stage 3's regression set (the spec's stage 3 regression list): what master built that stage 3
// refuses, or builds but cannot show exactly, as a user makes it. Each case is built as master
// built it (a feature named the older ways builds master's faceted construction) and as stage 3
// builds it (Stable, exactly). Decision 1b: where the exact build fails, the feature fails with
// its reason, and master's construction never runs in its place. Measured against master just
// before stage 3 (6e822892, Ruling 42).

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "horizon/document/Document.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/document/Sketch.h"
#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftEntity.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/math/Constants.h"
#include "horizon/math/Vec2.h"
#include "horizon/math/Vec3.h"
#include "horizon/modeling/Naming.h"

using hz::doc::Document;
using hz::doc::DocumentType;
using hz::doc::ExactNoteKind;
using hz::math::Vec2;
using hz::math::Vec3;

namespace {

constexpr double kPi = hz::math::kPi;

using Profile = std::vector<std::shared_ptr<hz::draft::DraftEntity>>;

std::shared_ptr<hz::draft::DraftEntity> line(Vec2 a, Vec2 b) {
    return std::make_shared<hz::draft::DraftLine>(a, b);
}

std::shared_ptr<hz::draft::DraftEntity> arc(Vec2 centre, double radius, double from, double to) {
    return std::make_shared<hz::draft::DraftArc>(centre, radius, from, to);
}

Profile rectangle(double x0, double y0, double x1, double y1) {
    return {line({x0, y0}, {x1, y0}), line({x1, y0}, {x1, y1}), line({x1, y1}, {x0, y1}),
            line({x0, y1}, {x0, y0})};
}

/// A part of one revolve of @p profile about the sketch's Y axis through @p angle, its feature
/// named @p naming; built.
struct Revolved {
    Document part;
    bool built = false;
};

std::unique_ptr<Revolved> revolved(const Profile& profile, double angle,
                                   hz::model::NamingScheme naming) {
    auto out = std::make_unique<Revolved>();
    out->part.setType(DocumentType::Part);
    auto sketch = std::make_shared<hz::doc::Sketch>();
    for (const auto& entity : profile) sketch->addEntity(entity);
    out->part.addSketch(sketch);
    auto revolve =
        std::make_unique<hz::doc::RevolveFeature>(sketch, Vec3(0, 0, 0), Vec3(0, 1, 0), angle);
    revolve->setNaming(naming);
    out->part.featureTree().addFeature(std::move(revolve));
    out->built = out->part.rebuildModel();
    return out;
}

/// Master built @p profile turned through @p angle, and stage 3 refuses it with @p why after
/// "could not be built exactly: ", leaving no part, faceted or exact.
void expectRefusedWhereMasterBuilt(const Profile& profile, double angle, const std::string& why) {
    const auto masters = revolved(profile, angle, hz::model::NamingScheme::FromGeometry);
    EXPECT_TRUE(masters->built) << "master's construction built it: "
                                << masters->part.lastBuildMessage();
    EXPECT_NE(masters->part.solid(), nullptr);

    const auto exact = revolved(profile, angle, hz::model::NamingScheme::Stable);
    EXPECT_FALSE(exact->built);
    EXPECT_EQ(exact->part.lastBuildMessage(),
              "Feature 'Revolve' failed: could not be built exactly: " + why);
    EXPECT_EQ(exact->part.solid(), nullptr) << "master's construction never runs in its place";
    EXPECT_EQ(exact->part.exactSolid(), nullptr);
}

}  // namespace

// A revolve whose profile's arc reaches the axis between its ends turns into a surface pinched to
// a cusp on the axis inside the face: a circle that touches the axis (a horn torus), an arc
// through the axis closed by a line (a D whose round side touches it), and a ring's inner side
// bulging to the axis. The exact faces have no topology for a pole inside a face's region, so
// the revolve is refused; master faceted each, a whole turn and a quarter.
TEST(RegressionSet, AnArcThatReachesTheAxisBetweenItsEndsIsRefused) {
    for (const double angle : {2 * kPi, kPi / 2}) {
        SCOPED_TRACE(angle);
        expectRefusedWhereMasterBuilt({std::make_shared<hz::draft::DraftCircle>(Vec2(1, 0), 1.0)},
                                      angle,
                                      "an arc of the profile about (1, 0) reaches the axis "
                                      "between its ends");
        expectRefusedWhereMasterBuilt({arc({1, 0}, 1, kPi / 2, 3 * kPi / 2), line({1, -1}, {1, 1})},
                                      angle,
                                      "an arc of the profile about (1, 0) reaches the axis "
                                      "between its ends");
        expectRefusedWhereMasterBuilt({line({2, -2}, {4, -2}), line({4, -2}, {4, 2}),
                                       line({4, 2}, {2, 2}), arc({2, 0}, 2, kPi / 2, 3 * kPi / 2)},
                                      angle,
                                      "an arc of the profile about (2, 0) reaches the axis "
                                      "between its ends");
    }
}

// A revolve whose profile has regions on both sides of the axis: master turned each region and
// joined them with its Boolean. The exact revolve makes each region a shell, and shells turned
// from both sides of one axis overlap or meet, which needs an exact Boolean (stage 5). Refused,
// whether the regions' turns overlap (a whole turn, or regions whose radii overlap) or not.
TEST(RegressionSet, RegionsOnBothSidesOfTheAxisAreRefused) {
    Profile apart = rectangle(1, 0, 2, 3);
    for (auto& entity : rectangle(-3, 0, -2, 3)) apart.push_back(entity);
    Profile overlapping = rectangle(1, 0, 3, 3);
    for (auto& entity : rectangle(-2, 1, -1.5, 2)) overlapping.push_back(entity);
    for (const double angle : {2 * kPi, kPi / 2}) {
        SCOPED_TRACE(angle);
        expectRefusedWhereMasterBuilt(apart, angle, "its regions lie on both sides of the axis");
        expectRefusedWhereMasterBuilt(overlapping, angle,
                                      "its regions lie on both sides of the axis");
    }
}

// Built, but shown and measured from its facets, as master showed it: a revolve whose face comes
// to a cusp at a pole, its surface opening from the pole at under 30 degrees and wider further
// out (Ruling 9). The read point refuses such a pole at once, saying why, and the part's exact
// counterpart is dropped with that note. An apple (the part of a circle outside the axis it
// crosses) with R/r 0.87 has one; 0.86 does not, and is exact. So has an arc that meets the axis
// tangentially at its end. Their faceted parts are master's: the counts master's Stable revolve
// gave at 6e822892.
TEST(RegressionSet, ACuspAtAPoleIsShownFromItsFacets) {
    const auto apple = [](double R) {
        const double h = std::sqrt(1 - R * R);
        const double a = std::acos(-R);
        return Profile{arc({R, 0}, 1, -a, a), line({0, h}, {0, -h})};
    };
    struct Case {
        const char* what;
        Profile profile;
        std::size_t faces, edges, vertices;
    };
    const std::vector<Case> cases = {
        {"an apple of R/r 0.87", apple(0.87), 864, 1696, 834},
        {"an arc tangent to the axis at its end",
         {arc({1, 0}, 1, kPi, 3 * kPi / 2), line({1, -1}, {1, 0}), line({1, 0}, {0, 0})},
         289,
         576,
         289}};
    for (const Case& c : cases) {
        SCOPED_TRACE(c.what);
        const auto masters = revolved(c.profile, 2 * kPi, hz::model::NamingScheme::FromGeometry);
        EXPECT_TRUE(masters->built) << masters->part.lastBuildMessage();
        const auto exact = revolved(c.profile, 2 * kPi, hz::model::NamingScheme::Stable);
        ASSERT_TRUE(exact->built) << exact->part.lastBuildMessage();
        ASSERT_NE(exact->part.solid(), nullptr);
        EXPECT_EQ(exact->part.solid()->faceCount(), c.faces);
        EXPECT_EQ(exact->part.solid()->edgeCount(), c.edges);
        EXPECT_EQ(exact->part.solid()->vertexCount(), c.vertices);
        EXPECT_EQ(exact->part.exactSolid(), nullptr) << "shown and measured from its facets";
        EXPECT_EQ(exact->part.exactNoteKind(), ExactNoteKind::ShownFromFacets);
        EXPECT_NE(exact->part.exactNote().find("the face comes to a cusp at a pole"),
                  std::string::npos)
            << exact->part.exactNote();
    }
    const auto clear = revolved(apple(0.86), 2 * kPi, hz::model::NamingScheme::Stable);
    ASSERT_TRUE(clear->built) << clear->part.lastBuildMessage();
    EXPECT_NE(clear->part.exactSolid(), nullptr) << clear->part.exactNote();
}
