// Phase 157: a sketch on a part's face follows the face when the part
// changes, and takes an extrusion's direction and a revolve's axis along.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <numbers>
#include <string>

#include "horizon/document/Document.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/document/ModelCommands.h"
#include "horizon/document/Sketch.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftRectangle.h"
#include "horizon/drafting/SketchPlane.h"
#include "horizon/modeling/EdgeProjection.h"
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
        EXPECT_TRUE(face.has_value()) << top;
        sketch = std::make_shared<Sketch>(SketchPlane(face->origin, face->normal, Vec3::UnitX));
        sketch->setName("On top");
        sketch->setFace(top);
        doc.addSketch(sketch);
    }

    /// The logical name of the box's top edge at x = @p x.
    std::string topEdgeAtX(double x) const {
        for (const auto& e : doc.solid()->edges()) {
            const auto* he = e.halfEdge;
            if (he == nullptr || he->origin == nullptr || he->next == nullptr) continue;
            const Vec3& a = he->origin->point;
            const Vec3& b = he->next->origin->point;
            if (a.x == x && b.x == x && a.z == b.z && a.z > 1.0 && e.topoId.isValid()) {
                return hz::model::wholeEdgeName(e.topoId.tag());
            }
        }
        return {};
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
    ASSERT_TRUE(part.sketch->placed().has_value());
    const Vec3 placed = part.sketch->placed()->origin();
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

// Construction geometry guides the drawing: a line across the profile is not
// part of it (Phase 157b).
TEST(SketchFollowTest, AConstructionLineIsNotPartOfTheProfile) {
    Document doc;
    doc.setType(hz::doc::DocumentType::Part);
    auto sketch = std::make_shared<Sketch>();
    sketch->addEntity(std::make_shared<hz::draft::DraftRectangle>(Vec2(0, 0), Vec2(2, 2)));
    auto across = std::make_shared<hz::draft::DraftLine>(Vec2(-5, -5), Vec2(5, 5));
    across->setConstruction(true);
    sketch->addEntity(across);
    doc.addSketch(sketch);
    doc.featureTree().addFeature(
        std::make_unique<hz::doc::ExtrudeFeature>(sketch, Vec3::UnitZ, 1.0));
    ASSERT_TRUE(doc.rebuildModel()) << doc.lastBuildMessage();
    EXPECT_NEAR(volumeOf(doc), 4.0, 1e-9);

    across->setConstruction(false);
    doc.featureTree().markChanged();
    EXPECT_FALSE(doc.rebuildModel()) << "a line that goes nowhere, in the profile";
}

// An edge of the part projected into a sketch follows the edge: a build
// draws it again from the part, the same entity where the edge now is.
TEST(SketchFollowTest, AProjectedEdgeFollowsThePart) {
    BoxWithASketchOnTop part;
    part.sketch->addEntity(std::make_shared<hz::draft::DraftRectangle>(Vec2(-1, -1), Vec2(1, 1)));
    const std::string edge = part.topEdgeAtX(10.0);
    ASSERT_FALSE(edge.empty());
    // Where Project Edges puts it: at x = 5 in the sketch, whose origin is
    // the face's middle.
    auto projected = std::make_shared<hz::draft::DraftLine>(Vec2(5, -5), Vec2(5, 5));
    projected->setSourceEdge(edge);
    projected->setConstruction(true);
    part.sketch->addEntity(projected);
    const uint64_t id = projected->id();
    auto boss = std::make_unique<hz::doc::ExtrudeFeature>(part.sketch, Vec3::UnitZ, 5.0);
    boss->setOperation(BodyOperation::Join);
    part.doc.featureTree().addFeature(std::move(boss));
    ASSERT_TRUE(part.doc.rebuildModel()) << part.doc.lastBuildMessage();

    ASSERT_TRUE(part.box->setParameter("width", 30.0));
    part.doc.featureTree().markChanged();
    ASSERT_TRUE(part.doc.rebuildModel()) << part.doc.lastBuildMessage();
    std::shared_ptr<hz::draft::DraftEntity> now;
    for (const auto& entity : part.sketch->entities()) {
        if (entity->id() == id) now = entity;
    }
    const auto* line = dynamic_cast<const hz::draft::DraftLine*>(now.get());
    ASSERT_NE(line, nullptr) << "the same entity, by id";
    EXPECT_NEAR(line->start().x, 25.0, 1e-9) << "the edge is at x = 30, the sketch's middle at 5";
    EXPECT_NEAR(line->end().x, 25.0, 1e-9);
    EXPECT_TRUE(now->construction()) << "what it was, kept";
    EXPECT_EQ(now->sourceEdge(), edge);
    EXPECT_NEAR(volumeOf(part.doc), 3000.0 + 20.0, 1e-6) << "a guide: the part as it was";
}

// An edge that shapes the part (not construction) and is gone fails the
// feature; one that is a guide stays where it was.
TEST(SketchFollowTest, AProjectedEdgeThatIsGoneFailsTheFeatureOnlyIfItShapesThePart) {
    BoxWithASketchOnTop part;
    part.sketch->addEntity(std::make_shared<hz::draft::DraftRectangle>(Vec2(-1, -1), Vec2(1, 1)));
    auto guide = std::make_shared<hz::draft::DraftLine>(Vec2(5, -5), Vec2(5, 5));
    guide->setSourceEdge("primitive_nosuch/edge");
    guide->setConstruction(true);
    part.sketch->addEntity(guide);
    part.doc.featureTree().addFeature(
        std::make_unique<hz::doc::ExtrudeFeature>(part.sketch, Vec3::UnitZ, 5.0));
    ASSERT_TRUE(part.doc.rebuildModel()) << part.doc.lastBuildMessage();
    EXPECT_NEAR(guide->start().x, 5.0, 1e-12) << "left where it was";

    guide->setConstruction(false);
    part.doc.featureTree().markChanged();
    EXPECT_FALSE(part.doc.rebuildModel());
    EXPECT_NE(part.doc.lastBuildMessage().find("primitive_nosuch/edge"), std::string::npos)
        << part.doc.lastBuildMessage();
}

namespace {

/// A 10 x 10 x 10 box, and a 2 x 2 column extruded down from z = 20 up to
/// the face named @p face (the box's top, by default), joined to it.
struct ColumnUpToAFace {
    Document doc;
    hz::doc::Feature* box = nullptr;
    hz::doc::ExtrudeFeature* column = nullptr;

    explicit ColumnUpToAFace(const std::string& face = "top", const Vec3& way = Vec3(0, 0, -1)) {
        doc.setType(hz::doc::DocumentType::Part);
        doc.featureTree().addFeature(hz::doc::PrimitiveFeature::makeBox(10, 10, 10));
        box = doc.featureTree().feature(0);
        auto sketch =
            std::make_shared<Sketch>(SketchPlane(Vec3(5, 5, 20), Vec3::UnitZ, Vec3::UnitX));
        sketch->addEntity(std::make_shared<hz::draft::DraftRectangle>(Vec2(-1, -1), Vec2(1, 1)));
        doc.addSketch(sketch);
        auto extrude = std::make_unique<hz::doc::ExtrudeFeature>(sketch, way, 1.0);
        extrude->setExtent(hz::doc::ExtrudeFeature::Extent::UpToFace);
        extrude->setUpToFace(face.empty() ? std::string() : box->featureID() + "/" + face);
        extrude->setOperation(BodyOperation::Join);
        column = extrude.get();
        doc.featureTree().addFeature(std::move(extrude));
    }
};

}  // namespace

// Phase 157c: an extrusion up to a face goes as far as the face, wherever the
// part puts it; its distance is not used.
TEST(ExtrudeUpToFaceTest, ItGoesAsFarAsTheFaceWhereverItIs) {
    ColumnUpToAFace part;
    ASSERT_TRUE(part.doc.rebuildModel()) << part.doc.lastBuildMessage();
    EXPECT_NEAR(volumeOf(part.doc), 1000.0 + 4.0 * 10.0, 1e-6) << "down to the top at 10";
    EXPECT_NEAR(topOf(part.doc), 20.0, 1e-9);

    ASSERT_TRUE(part.box->setParameter("depth", 15.0));
    part.doc.featureTree().markChanged();
    ASSERT_TRUE(part.doc.rebuildModel()) << part.doc.lastBuildMessage();
    EXPECT_NEAR(volumeOf(part.doc), 1500.0 + 4.0 * 5.0, 1e-6) << "down to the top at 15";
}

TEST(ExtrudeUpToFaceTest, AFaceItCannotGoUpToIsSaidSo) {
    const auto failure = [](ColumnUpToAFace& part) {
        EXPECT_FALSE(part.doc.rebuildModel());
        return part.doc.lastBuildMessage();
    };
    {
        ColumnUpToAFace part("right");  // facing x: across the way it goes
        const std::string why = failure(part);
        EXPECT_NE(why.find("at a slant"), std::string::npos) << why;
    }
    {
        ColumnUpToAFace part("top", Vec3::UnitZ);  // up, away from the box
        const std::string why = failure(part);
        EXPECT_NE(why.find("not in front"), std::string::npos) << why;
    }
    {
        ColumnUpToAFace part("");
        const std::string why = failure(part);
        EXPECT_NE(why.find("no face is chosen"), std::string::npos) << why;
    }
    {
        ColumnUpToAFace part("nosuch");
        const std::string why = failure(part);
        EXPECT_NE(why.find("is not there"), std::string::npos) << why;
    }
}

// The face it goes up to is changed by Edit Feature, and undone as one step.
TEST(ExtrudeUpToFaceTest, TheFaceIsEditedAndUndone) {
    ColumnUpToAFace part;
    ASSERT_TRUE(part.doc.rebuildModel()) << part.doc.lastBuildMessage();
    const std::string top = part.column->upToFace();
    const std::string bottom = part.box->featureID() + "/bottom";
    part.doc.undoStack().push(std::make_unique<hz::doc::EditFeatureCommand>(
        part.doc, part.column, std::map<std::string, double>{}, std::nullopt,
        std::map<std::string, Vec3>{}, std::map<std::string, std::string>{},
        std::map<std::string, std::string>{{"upToFace", bottom}}));
    EXPECT_EQ(part.column->upToFace(), bottom);
    ASSERT_TRUE(part.doc.rebuildModel()) << part.doc.lastBuildMessage();
    EXPECT_NEAR(volumeOf(part.doc), 1000.0 + 4.0 * 10.0, 1e-6)
        << "down through the box to its bottom: joined, only what is above it adds";
    part.doc.undoStack().undo();
    EXPECT_EQ(part.column->upToFace(), top);
}
