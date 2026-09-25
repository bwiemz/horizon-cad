// Phase 157: a sketch that follows a face keeps the face, the plane it was
// drawn on and where it was placed; a build of a copy (a worker's) places
// the document's own.

#include <gtest/gtest.h>

#include <algorithm>
#include <memory>
#include <nlohmann/json.hpp>
#include <string>

#include "horizon/document/Document.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/document/Sketch.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftRectangle.h"
#include "horizon/drafting/SketchPlane.h"
#include "horizon/fileio/NativeFormat.h"
#include "horizon/modeling/EdgeProjection.h"
#include "horizon/modeling/FacePlane.h"

using hz::doc::Document;
using hz::doc::Sketch;
using hz::draft::SketchPlane;
using hz::io::NativeFormat;
using hz::math::Vec2;
using hz::math::Vec3;

namespace {

/// A 10 x 10 x 10 box, a sketch on its top face, and a boss from it.
struct BossOnABox {
    Document doc;
    hz::doc::Feature* box = nullptr;
    std::shared_ptr<Sketch> sketch;

    BossOnABox() {
        doc.setType(hz::doc::DocumentType::Part);
        doc.featureTree().addFeature(hz::doc::PrimitiveFeature::makeBox(10, 10, 10));
        box = doc.featureTree().feature(0);
        EXPECT_TRUE(doc.rebuildModel());
        const std::string top = box->featureID() + "/top";
        const auto face = hz::model::planeOfFace(*doc.solid(), top);
        if (!face) {
            ADD_FAILURE() << "no plane for " << top;
            return;
        }
        sketch = std::make_shared<Sketch>(SketchPlane(face->origin, face->normal, Vec3::UnitX));
        sketch->setFace(top);
        sketch->addEntity(std::make_shared<hz::draft::DraftRectangle>(Vec2(-1, -1), Vec2(1, 1)));
        doc.addSketch(sketch);
        auto boss = std::make_unique<hz::doc::ExtrudeFeature>(sketch, Vec3::UnitZ, 5.0);
        boss->setOperation(hz::doc::BodyOperation::Join);
        doc.featureTree().addFeature(std::move(boss));
        EXPECT_TRUE(doc.rebuildModel()) << doc.lastBuildMessage();
    }
};

}  // namespace

TEST(SketchFollowIOTest, TheFaceTheDrawnPlaneAndThePlacementAreKept) {
    BossOnABox part;
    ASSERT_TRUE(part.box->setParameter("depth", 20.0));
    part.doc.featureTree().markChanged();
    ASSERT_TRUE(part.doc.rebuildModel()) << part.doc.lastBuildMessage();

    const std::string text = NativeFormat::documentToJson(part.doc, false);
    const auto root = nlohmann::json::parse(text);
    EXPECT_GE(root.at("version").get<int>(), 20) << "an older build would not follow the face";

    Document loaded;
    ASSERT_TRUE(NativeFormat::documentFromJson(text, loaded));
    ASSERT_EQ(loaded.sketches().size(), 1u);
    const Sketch& sketch = *loaded.sketches().front();
    EXPECT_EQ(sketch.face(), part.sketch->face());
    EXPECT_NEAR(sketch.drawnPlane().origin().z, 10.0, 1e-12) << "drawn on the face at 10";
    ASSERT_TRUE(sketch.placed().has_value()) << "where it is before any build";
    EXPECT_NEAR(sketch.plane().origin().z, 20.0, 1e-12);

    // A sketch on a plane of its own keeps none of it.
    Document plain;
    plain.addSketch(std::make_shared<Sketch>());
    const auto plainRoot = nlohmann::json::parse(NativeFormat::documentToJson(plain, false));
    const auto& written = plainRoot.at("sketches").at(0);
    EXPECT_FALSE(written.contains("face"));
    EXPECT_FALSE(written.contains("placed"));
}

// A worker builds a copy of the document: the copy's sketch is placed, and
// the build tells the document where, so its own sketch is there too.
TEST(SketchFollowIOTest, ABuildOfACopyPlacesTheDocumentsSketch) {
    BossOnABox part;
    ASSERT_TRUE(part.box->setParameter("depth", 20.0));
    part.doc.featureTree().markChanged();

    Document copy;
    ASSERT_TRUE(
        NativeFormat::documentFromJson(NativeFormat::documentToJson(part.doc, false), copy));
    hz::doc::BuildResult result = copy.buildWithDiagnostics();
    ASSERT_EQ(result.failedFeatureIndex, -1) << result.failureMessage;
    ASSERT_EQ(result.placements.count(part.sketch->id()), 1u);

    EXPECT_NEAR(part.sketch->plane().origin().z, 10.0, 1e-12) << "not built here yet";
    EXPECT_TRUE(part.doc.applyBuild(std::move(result)));
    EXPECT_NEAR(part.sketch->plane().origin().z, 20.0, 1e-12) << "placed where the copy's was";
}

// Phase 157b: construction geometry, and an edge projected with the edge it
// follows, are kept; a build of a copy draws the document's projected edge
// again where the part now puts it.
TEST(SketchFollowIOTest, AProjectedEdgeIsKeptAndDrawnAgainFromACopy) {
    BossOnABox part;
    auto guide = std::make_shared<hz::draft::DraftLine>(Vec2(5, -5), Vec2(5, 5));
    std::string edge;
    for (const auto& e : part.doc.solid()->edges()) {
        const auto* he = e.halfEdge;
        if (he == nullptr || he->origin == nullptr || he->next == nullptr) continue;
        const Vec3& a = he->origin->point;
        const Vec3& b = he->next->origin->point;
        if (a.x == 10.0 && b.x == 10.0 && a.z == 10.0 && b.z == 10.0) {
            edge = hz::model::wholeEdgeName(e.topoId.tag());
        }
    }
    ASSERT_FALSE(edge.empty());
    guide->setSourceEdge(edge);
    guide->setConstruction(true);
    part.sketch->addEntity(guide);

    const std::string text = NativeFormat::documentToJson(part.doc, false);
    EXPECT_EQ(nlohmann::json::parse(text).at("version").get<int>(), 21)
        << "an older build would take the guide for part of the profile";
    Document loaded;
    ASSERT_TRUE(NativeFormat::documentFromJson(text, loaded));
    const auto& entities = loaded.sketches().front()->entities();
    const auto kept = std::find_if(entities.begin(), entities.end(),
                                   [&](const auto& e) { return e->id() == guide->id(); });
    ASSERT_NE(kept, entities.end());
    EXPECT_TRUE((*kept)->construction());
    EXPECT_EQ((*kept)->sourceEdge(), edge);

    // Wider, built from a copy, as a worker builds.
    ASSERT_TRUE(part.box->setParameter("width", 30.0));
    part.doc.featureTree().markChanged();
    Document copy;
    ASSERT_TRUE(
        NativeFormat::documentFromJson(NativeFormat::documentToJson(part.doc, false), copy));
    hz::doc::BuildResult result = copy.buildWithDiagnostics();
    ASSERT_EQ(result.failedFeatureIndex, -1) << result.failureMessage;
    EXPECT_TRUE(part.doc.applyBuild(std::move(result)));
    const auto& now = part.sketch->entities();
    const auto drawn =
        std::find_if(now.begin(), now.end(), [&](const auto& e) { return e->id() == guide->id(); });
    ASSERT_NE(drawn, now.end());
    const auto* line = dynamic_cast<const hz::draft::DraftLine*>(drawn->get());
    ASSERT_NE(line, nullptr);
    EXPECT_NEAR(line->start().x, 25.0, 1e-9) << "the document's own, drawn again";
    EXPECT_TRUE(part.sketch->spatialIndex().query(line->boundingBox()).size() >= 1u)
        << "and found where it now is";
}
