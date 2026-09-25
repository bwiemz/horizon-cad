// Phase 157: a sketch on a part's face follows the face when the part
// changes, and takes an extrusion's direction and a revolve's axis along.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <numbers>
#include <string>

#include "horizon/document/Document.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/document/Sketch.h"
#include "horizon/drafting/DraftRectangle.h"
#include "horizon/drafting/SketchPlane.h"
#include "horizon/modeling/FacePlane.h"
#include "horizon/modeling/MassProperties.h"

using hz::doc::BodyOperation;
using hz::doc::Document;
using hz::doc::Sketch;
using hz::draft::SketchPlane;
using hz::math::Vec2;
using hz::math::Vec3;

namespace {

double volumeOf(const Document& document) {
    return document.solid() != nullptr
               ? hz::model::MassPropertiesCalculator::compute(*document.solid()).volume
               : 0.0;
}

double topOf(const Document& document) {
    double top = -std::numeric_limits<double>::infinity();
    for (const auto& v : document.solid()->vertices()) top = std::max(top, v.point.z);
    return top;
}

void expectNear(const Vec3& a, const Vec3& b, const char* what) {
    EXPECT_NEAR(a.x, b.x, 1e-9) << what;
    EXPECT_NEAR(a.y, b.y, 1e-9) << what;
    EXPECT_NEAR(a.z, b.z, 1e-9) << what;
}

/// A 10 x 10 x 10 box, and a sketch on its top face, as Sketch on a Face
/// makes it: through the face's middle, facing out, x along the world's.
struct BoxWithASketchOnTop {
    Document doc;
    hz::doc::Feature* box = nullptr;
    std::shared_ptr<Sketch> sketch;
    std::string top;

    BoxWithASketchOnTop() {
        doc.setType(hz::doc::DocumentType::Part);
        doc.featureTree().addFeature(hz::doc::PrimitiveFeature::makeBox(10, 10, 10));
        box = doc.featureTree().feature(0);
        EXPECT_TRUE(doc.rebuildModel()) << doc.lastBuildMessage();
        top = box->featureID() + "/top";
        const auto face = hz::model::planeOfFace(*doc.solid(), top);
        if (!face) {
            ADD_FAILURE() << "no plane for " << top;
            return;
        }
        sketch = std::make_shared<Sketch>(SketchPlane(face->origin, face->normal, Vec3::UnitX));
        sketch->setName("On top");
        sketch->setFace(top);
        doc.addSketch(sketch);
    }

    void setBoxDepth(double depth) {
        ASSERT_TRUE(box->setParameter("depth", depth));
        doc.featureTree().markChanged();
        ASSERT_TRUE(doc.rebuildModel()) << doc.lastBuildMessage();
    }
};

}  // namespace

TEST(SketchFollowTest, ASketchOnAFaceFollowsItWhenThePartChanges) {
    BoxWithASketchOnTop part;
    part.sketch->addEntity(std::make_shared<hz::draft::DraftRectangle>(Vec2(-1, -1), Vec2(1, 1)));
    auto boss = std::make_unique<hz::doc::ExtrudeFeature>(part.sketch, Vec3::UnitZ, 5.0);
    boss->setOperation(BodyOperation::Join);
    part.doc.featureTree().addFeature(std::move(boss));
    ASSERT_TRUE(part.doc.rebuildModel()) << part.doc.lastBuildMessage();
    EXPECT_NEAR(volumeOf(part.doc), 1000.0 + 20.0, 1e-6);
    EXPECT_NEAR(topOf(part.doc), 15.0, 1e-9);
    expectNear(part.sketch->plane().origin(), Vec3(5, 5, 10), "on the face it was drawn on");

    part.setBoxDepth(20.0);
    EXPECT_NEAR(volumeOf(part.doc), 2000.0 + 20.0, 1e-6) << "the boss on the taller box";
    EXPECT_NEAR(topOf(part.doc), 25.0, 1e-9);
    expectNear(part.sketch->plane().origin(), Vec3(5, 5, 20), "placed on the face");
    expectNear(part.sketch->plane().normal(), Vec3::UnitZ, "facing the way it did");
    expectNear(part.sketch->plane().xAxis(), Vec3::UnitX, "unturned");
    expectNear(part.sketch->drawnPlane().origin(), Vec3(5, 5, 10), "drawn where it was");

    part.setBoxDepth(10.0);
    EXPECT_NEAR(volumeOf(part.doc), 1020.0, 1e-6);
    const auto placedPlane = part.sketch->placed();
    if (!placedPlane) FAIL() << "placed by the build";
    const Vec3 placed = placedPlane->origin();
    const Vec3 drawn = part.sketch->drawnPlane().origin();
    EXPECT_TRUE(placed.x == drawn.x && placed.y == drawn.y && placed.z == drawn.z)
        << "back on the plane it was drawn on, exactly";
}

// A face that grows leaves the sketch where it is: the face moves under it,
// not the sketch.
TEST(SketchFollowTest, AFaceThatGrowsLeavesTheSketchWhereItIs) {
    BoxWithASketchOnTop part;
    part.sketch->addEntity(std::make_shared<hz::draft::DraftRectangle>(Vec2(-1, -1), Vec2(1, 1)));
    part.doc.featureTree().addFeature(
        std::make_unique<hz::doc::ExtrudeFeature>(part.sketch, Vec3::UnitZ, 5.0));
    ASSERT_TRUE(part.box->setParameter("width", 30.0));
    part.doc.featureTree().markChanged();
    ASSERT_TRUE(part.doc.rebuildModel()) << part.doc.lastBuildMessage();
    expectNear(part.sketch->plane().origin(), Vec3(5, 5, 10), "where it was");
}

// A revolve's axis is given with its sketch: the sketch placed higher takes
// it along.
TEST(SketchFollowTest, ARevolvesAxisFollowsItsSketch) {
    BoxWithASketchOnTop part;
    // A ring about the sketch's y axis, through the face's middle.
    part.sketch->addEntity(std::make_shared<hz::draft::DraftRectangle>(Vec2(1, 0), Vec2(2, 1)));
    const SketchPlane& drawn = part.sketch->drawnPlane();
    auto ring = std::make_unique<hz::doc::RevolveFeature>(part.sketch, drawn.origin(),
                                                          drawn.yAxis(), 2.0 * std::numbers::pi);
    ring->setOperation(BodyOperation::Join);
    part.doc.featureTree().addFeature(std::move(ring));
    ASSERT_TRUE(part.doc.rebuildModel()) << part.doc.lastBuildMessage();
    EXPECT_NEAR(topOf(part.doc), 12.0, 1e-6) << "the ring's top, 2 above the face";

    part.setBoxDepth(20.0);
    EXPECT_NEAR(topOf(part.doc), 22.0, 1e-6) << "about the axis on the face where it is now";
}

TEST(SketchFollowTest, AFaceThatIsGoneFailsTheFeatureSayingSo) {
    BoxWithASketchOnTop part;
    part.sketch->addEntity(std::make_shared<hz::draft::DraftRectangle>(Vec2(-1, -1), Vec2(1, 1)));
    part.doc.featureTree().addFeature(
        std::make_unique<hz::doc::ExtrudeFeature>(part.sketch, Vec3::UnitZ, 5.0));
    ASSERT_TRUE(part.doc.rebuildModel()) << part.doc.lastBuildMessage();
    part.setBoxDepth(20.0);
    const SketchPlane was = part.sketch->plane();

    part.sketch->setFace(part.box->featureID() + "/nosuch");
    part.doc.featureTree().markChanged();
    EXPECT_FALSE(part.doc.rebuildModel());
    const std::string& message = part.doc.lastBuildMessage();
    EXPECT_NE(message.find("On top"), std::string::npos) << message;
    EXPECT_NE(message.find("is not there"), std::string::npos) << message;
    EXPECT_EQ(part.doc.failedFeatureIndex(), 1);
    expectNear(part.sketch->plane().origin(), was.origin(), "left where it was last placed");
}

// Two features from one sketch: it is placed by the first, and the second
// builds on it there, though the first has cut the face away.
TEST(SketchFollowTest, ASketchIsPlacedOnceInABuild) {
    BoxWithASketchOnTop part;
    part.sketch->addEntity(std::make_shared<hz::draft::DraftRectangle>(Vec2(-6, -6), Vec2(6, 6)));
    for (const double depth : {1.0, 2.0}) {
        auto cut =
            std::make_unique<hz::doc::ExtrudeFeature>(part.sketch, Vec3::UnitZ * -1.0, depth);
        cut->setOperation(BodyOperation::Cut);
        part.doc.featureTree().addFeature(std::move(cut));
    }
    ASSERT_TRUE(part.doc.rebuildModel()) << part.doc.lastBuildMessage();
    EXPECT_NEAR(volumeOf(part.doc), 800.0, 1e-6) << "the second from where the sketch was placed";
    expectNear(part.sketch->plane().origin(), Vec3(5, 5, 10), "placed once");
}

TEST(SketchFollowTest, PlacedOnATurnedFaceTheSketchTurnsWithIt) {
    Sketch sketch(SketchPlane(Vec3(1, 2, 3), Vec3::UnitZ, Vec3::UnitX));
    const double c = std::cos(std::numbers::pi / 6.0);
    const double s = std::sin(std::numbers::pi / 6.0);
    const Vec3 tilted(0.0, -s, c);  // turned 30 degrees about x
    sketch.placeOn(Vec3(0, 0, 5), tilted);
    const SketchPlane& placed = sketch.plane();
    expectNear(placed.normal(), tilted, "facing the face");
    EXPECT_NEAR((placed.origin() - Vec3(0, 0, 5)).dot(tilted), 0.0, 1e-12) << "on it";
    expectNear(placed.xAxis(), Vec3::UnitX, "x about which it turned, kept");
    expectNear(placed.yAxis(), tilted.cross(Vec3::UnitX), "right-handed");

    // A direction given with it turns too: the drawn normal is the placed one.
    expectNear(sketch.placement().transformDirection(Vec3::UnitZ), tilted, "along");
    expectNear(sketch.placement().transformPoint(Vec3(1, 2, 3)), placed.origin(), "origin");

    // Its x axis now along the face's normal: y is kept, and x made from it.
    sketch.placeOn(Vec3(9, 0, 0), Vec3::UnitX);
    expectNear(sketch.plane().yAxis(), Vec3::UnitY, "y kept");
    expectNear(sketch.plane().xAxis(), Vec3::UnitZ * -1.0, "x = y x normal");

    sketch.setPlane(SketchPlane());
    EXPECT_FALSE(sketch.placed().has_value()) << "drawn on a plane again, placed nowhere else";
}
