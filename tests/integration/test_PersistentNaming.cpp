// Names that survive edits (Phase 106): an extrusion names its side faces
// after the sketch entities they come from and its edges after the faces they
// separate, so an edit elsewhere in the sketch does not move a fillet.

#include <gtest/gtest.h>

#include <map>
#include <memory>
#include <nlohmann/json.hpp>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "horizon/document/Document.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/document/Sketch.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftRectangle.h"
#include "horizon/fileio/NativeFormat.h"
#include "horizon/modeling/BooleanOp.h"
#include "horizon/modeling/MateGeometry.h"
#include "horizon/modeling/Naming.h"
#include "horizon/topology/Solid.h"
#include "horizon/topology/TopologyID.h"

using hz::doc::BodyOperation;
using hz::doc::Document;
using hz::doc::ExtrudeFeature;
using hz::doc::Sketch;
using hz::draft::DraftLine;
using hz::math::Vec2;
using hz::math::Vec3;
using hz::model::NamingScheme;
using hz::topo::TopologyID;

namespace {

/// The vertical edge standing on (x, y) of the part, and its name.
std::optional<TopologyID> verticalEdgeAt(const hz::topo::Solid& solid, double x, double y) {
    for (const auto& edge : solid.edges()) {
        const Vec3& a = edge.halfEdge->origin->point;
        const Vec3& b = edge.halfEdge->next->origin->point;
        if (std::abs(a.x - x) < 1e-9 && std::abs(a.y - y) < 1e-9 && std::abs(b.x - x) < 1e-9 &&
            std::abs(b.y - y) < 1e-9) {
            return edge.topoId;
        }
    }
    return std::nullopt;
}

/// Where the edge a name refers to stands, as (x, y) of its first end.
std::optional<Vec2> whereIs(const hz::topo::Solid& solid, const TopologyID& id) {
    for (const auto& edge : solid.edges()) {
        if (edge.topoId == id) {
            const Vec3& a = edge.halfEdge->origin->point;
            return Vec2(a.x, a.y);
        }
    }
    return std::nullopt;
}

/// A 10 x 10 square of four lines, extruded 5 with `naming`.
struct Square {
    std::shared_ptr<Sketch> sketch = std::make_shared<Sketch>();
    std::shared_ptr<DraftLine> bottom, right, top, left;
    Document doc;
    ExtrudeFeature* extrude = nullptr;

    explicit Square(NamingScheme naming) {
        bottom = std::make_shared<DraftLine>(Vec2(0, 0), Vec2(10, 0));
        right = std::make_shared<DraftLine>(Vec2(10, 0), Vec2(10, 10));
        top = std::make_shared<DraftLine>(Vec2(10, 10), Vec2(0, 10));
        left = std::make_shared<DraftLine>(Vec2(0, 10), Vec2(0, 0));
        for (const auto& line : {bottom, right, top, left}) sketch->addEntity(line);
        doc.addSketch(sketch);
        auto feature = std::make_unique<ExtrudeFeature>(sketch, Vec3(0, 0, 1), 5.0);
        feature->setNaming(naming);
        extrude = feature.get();
        doc.featureTree().addFeature(std::move(feature));
    }

    /// Split the top edge in two at (5, 10): the kind of edit that adds a
    /// vertex, and so a side face and two edges, to the profile.
    void splitTop() {
        sketch->removeEntity(top->id());
        sketch->addEntity(std::make_shared<DraftLine>(Vec2(10, 10), Vec2(5, 10)));
        sketch->addEntity(std::make_shared<DraftLine>(Vec2(5, 10), Vec2(0, 10)));
    }
};

}  // namespace

TEST(PersistentNamingTest, AnEditElsewhereInTheSketchLeavesAnEdgeItsName) {
    Square part(NamingScheme::FromGeometry);
    ASSERT_TRUE(part.doc.rebuildModel()) << part.doc.lastBuildMessage();
    const auto corner = verticalEdgeAt(*part.doc.solid(), 10, 0);  // between bottom and right
    ASSERT_TRUE(corner);
    EXPECT_EQ(corner->tag(), part.extrude->featureID() + "/edge:side:e" +
                                 std::to_string(part.bottom->id()) + "|side:e" +
                                 std::to_string(part.right->id()));

    part.splitTop();
    ASSERT_TRUE(part.doc.rebuildModel()) << part.doc.lastBuildMessage();
    const auto after = whereIs(*part.doc.solid(), *corner);
    ASSERT_TRUE(after) << "the name still exists";
    EXPECT_NEAR(after->x, 10.0, 1e-9);
    EXPECT_NEAR(after->y, 0.0, 1e-9) << "and still names the same corner";

    // So a fillet on it keeps rounding that corner.
    part.doc.featureTree().addFeature(
        std::make_unique<hz::doc::FilletFeature>(std::vector<TopologyID>{*corner}, 1.0));
    EXPECT_TRUE(part.doc.rebuildModel()) << part.doc.lastBuildMessage();
}

TEST(PersistentNamingTest, PositionalNamesMoveWhenTheSketchChanges) {
    // The defect the new names fix, kept for files saved before them: the
    // same edit gives the old name to a different edge.
    Square part(NamingScheme::Positional);
    ASSERT_TRUE(part.doc.rebuildModel());
    const auto corner = verticalEdgeAt(*part.doc.solid(), 10, 0);
    ASSERT_TRUE(corner);

    part.splitTop();
    ASSERT_TRUE(part.doc.rebuildModel());
    const auto after = whereIs(*part.doc.solid(), *corner);
    ASSERT_TRUE(after);
    EXPECT_FALSE(std::abs(after->x - 10.0) < 1e-9 && std::abs(after->y) < 1e-9)
        << "positional names are expected to retarget";
}

TEST(PersistentNamingTest, SidesAreNamedAfterTheirSource) {
    // A rectangle's sides are its segments; a circle's are its facets.
    auto rectSketch = std::make_shared<Sketch>();
    auto rect = std::make_shared<hz::draft::DraftRectangle>(Vec2(0, 0), Vec2(4, 2));
    rectSketch->addEntity(rect);
    ExtrudeFeature box(rectSketch, Vec3(0, 0, 1), 1.0);
    auto boxSolid = box.execute(nullptr);
    ASSERT_NE(boxSolid, nullptr);
    std::set<std::string> names;
    for (const auto& face : boxSolid->faces()) names.insert(face.topoId.tag());
    const std::string r = box.featureID() + "/side:e" + std::to_string(rect->id());
    for (int k = 0; k < 4; ++k) EXPECT_TRUE(names.count(r + "." + std::to_string(k))) << k;
    EXPECT_TRUE(names.count(box.featureID() + "/cap_top"));

    auto circleSketch = std::make_shared<Sketch>();
    auto circle = std::make_shared<hz::draft::DraftCircle>(Vec2(0, 0), 3.0);
    circleSketch->addEntity(circle);
    ExtrudeFeature disc(circleSketch, Vec3(0, 0, 1), 1.0);
    auto discSolid = disc.execute(nullptr);
    ASSERT_NE(discSolid, nullptr);
    names.clear();
    for (const auto& face : discSolid->faces()) names.insert(face.topoId.tag());
    EXPECT_TRUE(names.count(disc.featureID() + "/side:e" + std::to_string(circle->id()) + ".f0"));
    EXPECT_EQ(names.size(), discSolid->faceCount()) << "every face named, none twice";

    std::set<std::string> edgeNames;
    for (const auto& edge : discSolid->edges()) edgeNames.insert(edge.topoId.tag());
    EXPECT_EQ(edgeNames.size(), discSolid->edgeCount()) << "every edge named, none twice";
}

TEST(PersistentNamingTest, TheNamingSchemeIsSavedAndOlderFilesKeepTheirs) {
    Square part(NamingScheme::FromGeometry);
    nlohmann::json root =
        nlohmann::json::parse(hz::io::NativeFormat::documentToJson(part.doc, false));
    EXPECT_GE(root.at("version").get<int>(), 18);
    ASSERT_EQ(root.at("featureTree").at(0).at("naming").get<int>(), 2);

    Document loaded;
    std::string error;
    ASSERT_TRUE(hz::io::NativeFormat::documentFromJson(root.dump(), loaded, &error)) << error;
    const auto* extrude = dynamic_cast<const ExtrudeFeature*>(loaded.featureTree().feature(0));
    ASSERT_NE(extrude, nullptr);
    EXPECT_EQ(extrude->naming(), NamingScheme::FromGeometry);

    // A file from before persistent naming has no "naming": its references
    // were made against positional names, which it keeps.
    root.at("featureTree").at(0).erase("naming");
    root["version"] = 17;
    Document old;
    ASSERT_TRUE(hz::io::NativeFormat::documentFromJson(root.dump(), old, &error)) << error;
    const auto* oldExtrude = dynamic_cast<const ExtrudeFeature*>(old.featureTree().feature(0));
    ASSERT_NE(oldExtrude, nullptr);
    EXPECT_EQ(oldExtrude->naming(), NamingScheme::Positional);
    EXPECT_EQ(old.featureTree().feature(0)->naming(), NamingScheme::Positional)
        << "every feature of an older file, not only extrusions";
    ASSERT_TRUE(old.rebuildModel());
    EXPECT_TRUE(hz::model::MateGeometry::findFace(
        *old.solid(), TopologyID::make(oldExtrude->featureID(), "lateral_0")));
}

TEST(PersistentNamingTest, ABooleanNamesEachPieceOfASplitFace) {
    // A 10 x 10 x 5 block with a groove cut across its top: the top face comes
    // out in pieces. Each gets a name of its own (they used to share the
    // face's), and a reference to the whole face still finds one.
    Square part(NamingScheme::FromGeometry);
    ASSERT_TRUE(part.doc.rebuildModel());
    const std::string top = part.extrude->featureID() + "/cap_top";

    auto groove = std::make_shared<Sketch>();
    groove->addEntity(std::make_shared<hz::draft::DraftRectangle>(Vec2(4, -1), Vec2(6, 11)));
    groove->setPlane(hz::draft::SketchPlane(Vec3(0, 0, 3), Vec3(0, 0, 1), Vec3(1, 0, 0)));
    auto cut = std::make_unique<ExtrudeFeature>(groove, Vec3(0, 0, 1), 5.0);
    cut->setOperation(BodyOperation::Cut);
    part.doc.featureTree().addFeature(std::move(cut));
    ASSERT_TRUE(part.doc.rebuildModel()) << part.doc.lastBuildMessage();
    const auto& solid = *part.doc.solid();

    std::map<std::string, int> faces;
    for (const auto& face : solid.faces()) ++faces[face.topoId.tag()];
    for (const auto& [name, count] : faces) EXPECT_EQ(count, 1) << name << " named twice";
    EXPECT_TRUE(faces.count(top + "/piece:0"));
    EXPECT_TRUE(faces.count(top + "/piece:1"));
    EXPECT_NE(hz::model::MateGeometry::findFace(solid, TopologyID::fromTag(top)), nullptr)
        << "a reference to the whole face still finds it";

    std::map<std::string, int> edges;
    for (const auto& edge : solid.edges()) ++edges[edge.topoId.tag()];
    for (const auto& [name, count] : edges) EXPECT_EQ(count, 1) << name << " named twice";
}

TEST(PersistentNamingTest, AnOlderDocumentsBooleanKeepsItsOldNames) {
    // A Cut in a file saved before persistent naming: its result's names are
    // the sewer's own, which that file's fillets and chamfers were made
    // against — so the Cut must not rename them.
    const auto build = [](NamingScheme naming) {
        auto part = std::make_unique<Square>(naming);
        auto hole = std::make_shared<Sketch>();
        hole->addEntity(std::make_shared<hz::draft::DraftRectangle>(Vec2(3, 3), Vec2(5, 5)));
        auto cut = std::make_unique<ExtrudeFeature>(hole, Vec3(0, 0, 1), 5.0);
        cut->setOperation(BodyOperation::Cut);
        cut->setNaming(naming);
        part->doc.featureTree().addFeature(std::move(cut));
        EXPECT_TRUE(part->doc.rebuildModel()) << part->doc.lastBuildMessage();
        std::vector<std::string> names;
        for (const auto& edge : part->doc.solid()->edges()) names.push_back(edge.topoId.tag());
        return names;
    };

    // What the sewer names an edge when nothing renames it: its first face's
    // tag, then a running number.
    const auto numbered = [](const std::string& tag) {
        const auto at = tag.rfind("/edge:");
        return at != std::string::npos && tag.find('|') == std::string::npos &&
               tag.find_first_not_of("0123456789", at + 6) == std::string::npos;
    };
    for (const auto& name : build(NamingScheme::Positional)) {
        EXPECT_TRUE(numbered(name)) << name;
    }
    for (const auto& name : build(NamingScheme::FromGeometry)) {
        EXPECT_FALSE(numbered(name)) << name;
    }
}
