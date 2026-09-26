// Names that survive edits (Phase 106): an extrusion names its side faces
// after the sketch entities they come from and its edges after the faces they
// separate, so an edit elsewhere in the sketch does not move a fillet.

#include <gtest/gtest.h>

#include <cmath>
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
#include "horizon/drafting/DraftPolyline.h"
#include "horizon/drafting/DraftRectangle.h"
#include "horizon/drafting/SketchPlane.h"
#include "horizon/fileio/NativeFormat.h"
#include "horizon/modeling/BooleanOp.h"
#include "horizon/modeling/FacePlane.h"
#include "horizon/modeling/Loft.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/modeling/MateGeometry.h"
#include "horizon/modeling/Naming.h"
#include "horizon/modeling/Sweep.h"
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
    // Stable (Phase 139): the facets are parts of the one curved side.
    const std::string side = disc.featureID() + "/side:e" + std::to_string(circle->id());
    EXPECT_TRUE(names.count(side + "/facet:0"));
    EXPECT_EQ(names.size(), discSolid->faceCount()) << "every face named, none twice";
    // FromGeometry, as documents saved since Phase 106 name them.
    ExtrudeFeature older(circleSketch, Vec3(0, 0, 1), 1.0);
    older.setNaming(NamingScheme::FromGeometry);
    auto olderSolid = older.execute(nullptr);
    ASSERT_NE(olderSolid, nullptr);
    std::set<std::string> olderNames;
    for (const auto& face : olderSolid->faces()) olderNames.insert(face.topoId.tag());
    EXPECT_TRUE(
        olderNames.count(older.featureID() + "/side:e" + std::to_string(circle->id()) + ".f0"));

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

TEST(PersistentNamingTest, AnEdgeTheCutNeverTouchedKeepsItsName) {
    // With each face put back together after the Boolean (106b), the corner
    // at (10, 0) is still between the same two whole faces, so it keeps the
    // name the extrusion gave it.
    Square part(NamingScheme::FromGeometry);
    ASSERT_TRUE(part.doc.rebuildModel());
    const auto corner = verticalEdgeAt(*part.doc.solid(), 10, 0);
    ASSERT_TRUE(corner);

    auto groove = std::make_shared<Sketch>();
    groove->addEntity(std::make_shared<hz::draft::DraftRectangle>(Vec2(4, -1), Vec2(6, 11)));
    groove->setPlane(hz::draft::SketchPlane(Vec3(0, 0, 3), Vec3(0, 0, 1), Vec3(1, 0, 0)));
    auto cut = std::make_unique<ExtrudeFeature>(groove, Vec3(0, 0, 1), 5.0);
    cut->setOperation(BodyOperation::Cut);
    part.doc.featureTree().addFeature(std::move(cut));
    ASSERT_TRUE(part.doc.rebuildModel()) << part.doc.lastBuildMessage();

    const auto after = whereIs(*part.doc.solid(), *corner);
    ASSERT_TRUE(after) << "the corner the cut did not touch keeps its name";
    EXPECT_NEAR(after->x, 10.0, 1e-9);
    EXPECT_NEAR(after->y, 0.0, 1e-9);
}

TEST(PersistentNamingTest, APlateWithAHoleCanBeFilleted) {
    // The Milestone 2 goal. Fillet refused every edge of a Boolean result
    // ("non box-like corner"): the CSG left each face as triangles, so extra
    // edges met at every corner.
    Square part(NamingScheme::FromGeometry);  // 10 x 10 x 5
    auto hole = std::make_shared<Sketch>();
    hole->addEntity(std::make_shared<hz::draft::DraftCircle>(Vec2(5, 5), 2.0));
    auto cut = std::make_unique<ExtrudeFeature>(hole, Vec3(0, 0, 1), 5.0);
    cut->setOperation(BodyOperation::Cut);
    part.doc.featureTree().addFeature(std::move(cut));
    ASSERT_TRUE(part.doc.rebuildModel()) << part.doc.lastBuildMessage();
    ASSERT_EQ(part.doc.solid()->genus(), 1);
    const double before = hz::model::MassPropertiesCalculator::compute(*part.doc.solid()).volume;

    const auto corner = verticalEdgeAt(*part.doc.solid(), 10, 0);
    ASSERT_TRUE(corner);
    part.doc.featureTree().addFeature(
        std::make_unique<hz::doc::FilletFeature>(std::vector<TopologyID>{*corner}, 1.0));
    ASSERT_TRUE(part.doc.rebuildModel()) << part.doc.lastBuildMessage();
    const double after = hz::model::MassPropertiesCalculator::compute(*part.doc.solid()).volume;
    // The corner loses (1 - pi/4) r^2 per unit height, a little more faceted.
    const double exact = (1.0 - std::acos(-1.0) / 4.0) * 5.0;
    EXPECT_GT(before - after, exact - 1e-9);
    EXPECT_LT(before - after, exact + 0.1);
    EXPECT_EQ(part.doc.solid()->genus(), 1) << "the hole is still there";
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

// Two primitives in one part have names of their own (Phase 139). Every
// box's top was `box/top` and its edges `box/edge<i>` in storage order, so a
// fillet named on the second box could land on the first. An older file's
// primitives, positional, keep the names they had.
TEST(PersistentNamingTest, TwoPrimitivesAreNamedApart) {
    hz::doc::FeatureTree tree;
    auto first = hz::doc::PrimitiveFeature::makeBox(10, 10, 10);
    auto second = hz::doc::PrimitiveFeature::makeBox(10, 10, 10);
    ASSERT_TRUE(second->setVector("basePoint", Vec3(5, 5, 5)));
    second->setOperation(BodyOperation::Join);
    const std::string a = first->featureID();
    const std::string b = second->featureID();
    tree.addFeature(std::move(first));
    tree.addFeature(std::move(second));  // joins the first
    auto joined = tree.build();
    ASSERT_NE(joined, nullptr);
    std::set<std::string> faces;
    for (const auto& face : joined->faces()) faces.insert(face.topoId.tag());
    EXPECT_TRUE(faces.count(a + "/bottom")) << "the first box's own";
    EXPECT_TRUE(faces.count(b + "/top")) << "the second's, apart";
    EXPECT_FALSE(faces.count("box/top"));

    // The second box's far top edge, by its name: rounded there, and only
    // there. 1000 + 1000 - 125 overlap, less (1 - pi/4) for each unit of the
    // 10 long edge.
    tree.addFeature(std::make_unique<hz::doc::FilletFeature>(
        std::vector<TopologyID>{TopologyID::fromTag(b + "/edge:right|top")}, 1.0));
    auto rounded = tree.build();
    ASSERT_NE(rounded, nullptr);
    const double pi = std::acos(-1.0);
    EXPECT_NEAR(hz::model::MassPropertiesCalculator::compute(*rounded).volume,
                1875.0 - 10.0 * (1.0 - pi / 4.0), 0.5)
        << "within the arc's faceting";
    double highest = 0.0;
    for (const auto& v : rounded->vertices()) {
        if (v.point.x > 14.999 && v.point.z > 14.999) highest = std::max(highest, v.point.y);
    }
    EXPECT_EQ(highest, 0.0) << "the corner at (15, y, 15) is rounded away";

    // Positional, as an older file's: `box/top` as it always was.
    hz::doc::FeatureTree old;
    auto box = hz::doc::PrimitiveFeature::makeBox(2, 2, 2);
    box->setNaming(NamingScheme::Positional);
    old.addFeature(std::move(box));
    auto built = old.build();
    ASSERT_NE(built, nullptr);
    bool kept = false;
    for (const auto& face : built->faces()) kept = kept || face.topoId.tag() == "box/top";
    EXPECT_TRUE(kept);
}

// A fillet leaves the names of the edges it did not touch (Phase 139). It
// renamed every edge in storage order, so a second fillet on an edge named
// before the first found nothing, or another edge.
TEST(PersistentNamingTest, AFilletKeepsTheNamesOfTheEdgesItDidNotTouch) {
    hz::doc::FeatureTree tree;
    auto box = hz::doc::PrimitiveFeature::makeBox(10, 10, 10);
    const std::string id = box->featureID();
    tree.addFeature(std::move(box));
    auto plain = tree.build();
    ASSERT_NE(plain, nullptr);
    std::set<std::string> before;
    for (const auto& edge : plain->edges()) before.insert(edge.topoId.tag());
    const std::string first = id + "/edge:right|top";
    const std::string second = id + "/edge:front|left";
    ASSERT_TRUE(before.count(first) && before.count(second));

    tree.addFeature(std::make_unique<hz::doc::FilletFeature>(
        std::vector<TopologyID>{TopologyID::fromTag(first)}, 1.0));
    auto once = tree.build();
    ASSERT_NE(once, nullptr);
    std::set<std::string> after;
    for (const auto& edge : once->edges()) after.insert(edge.topoId.tag());
    for (const auto& name : before) {
        if (name != first) {
            EXPECT_TRUE(after.count(name)) << name << " kept";
        }
    }

    tree.addFeature(std::make_unique<hz::doc::FilletFeature>(
        std::vector<TopologyID>{TopologyID::fromTag(second)}, 1.0));
    const auto built = tree.buildWithDiagnostics();
    ASSERT_NE(built.solid, nullptr) << built.failureMessage;
    const double pi = std::acos(-1.0);
    EXPECT_NEAR(hz::model::MassPropertiesCalculator::compute(*built.solid).volume,
                1000.0 - 2.0 * 10.0 * (1.0 - pi / 4.0), 0.5)
        << "both edges rounded, within the arcs' faceting";
}

// Each feature keeps the scheme it was made with through a file (Phase 139).
// A part saved since Phase 106 (naming 2) was named `box/top`, and its
// references were made against that: it keeps it. New features (naming 3)
// are named after themselves.
TEST(PersistentNamingTest, APrimitiveKeepsTheSchemeItWasSavedWith) {
    Document doc;
    auto older = hz::doc::PrimitiveFeature::makeBox(2, 2, 2);
    older->setNaming(NamingScheme::FromGeometry);
    auto newer = hz::doc::PrimitiveFeature::makeBox(2, 2, 2);
    ASSERT_TRUE(newer->setVector("basePoint", Vec3(10, 0, 0)));
    const std::string id = newer->featureID();
    doc.featureTree().addFeature(std::move(older));
    doc.featureTree().addFeature(std::move(newer));

    nlohmann::json root = nlohmann::json::parse(hz::io::NativeFormat::documentToJson(doc, false));
    EXPECT_GE(root.at("version").get<int>(), 19);
    EXPECT_EQ(root.at("featureTree").at(0).at("naming").get<int>(), 2);
    EXPECT_EQ(root.at("featureTree").at(1).at("naming").get<int>(), 3);

    Document loaded;
    std::string error;
    ASSERT_TRUE(hz::io::NativeFormat::documentFromJson(root.dump(), loaded, &error)) << error;
    ASSERT_TRUE(loaded.rebuildModel()) << loaded.lastBuildMessage();
    std::set<std::string> faces;
    for (const auto& face : loaded.solid()->faces()) faces.insert(face.topoId.tag());
    EXPECT_TRUE(faces.count("box/top")) << "the older box, as it was";
    EXPECT_TRUE(faces.count(id + "/top")) << "the newer, after its feature";
}

// A chamfer leaves the names of the edges it did not touch, as a fillet does
// (Phase 139): the sewer named every edge afresh, in the order it met them.
TEST(PersistentNamingTest, AChamferKeepsTheNamesOfTheEdgesItDidNotTouch) {
    hz::doc::FeatureTree tree;
    auto box = hz::doc::PrimitiveFeature::makeBox(10, 10, 10);
    const std::string id = box->featureID();
    tree.addFeature(std::move(box));
    tree.addFeature(std::make_unique<hz::doc::ChamferFeature>(
        std::vector<TopologyID>{TopologyID::fromTag(id + "/edge:right|top")}, 1.0));
    tree.addFeature(std::make_unique<hz::doc::ChamferFeature>(
        std::vector<TopologyID>{TopologyID::fromTag(id + "/edge:front|left")}, 1.0));
    const auto built = tree.buildWithDiagnostics();
    ASSERT_NE(built.solid, nullptr) << built.failureMessage;
    EXPECT_EQ(built.failedFeatureIndex, -1) << built.failureMessage;
    EXPECT_NEAR(hz::model::MassPropertiesCalculator::compute(*built.solid).volume,
                1000.0 - 2.0 * 10.0 * 0.5, 1e-6)
        << "two chamfers, each taking half of a unit square along ten";
}

// A prism's shell (as one read from a file before Phase 163 is built): its
// faces are named after the shell feature, not every shell's `shell/…`
// (Phase 139).
TEST(PersistentNamingTest, AShellIsNamedAfterItsFeature) {
    hz::doc::FeatureTree tree;
    auto box = hz::doc::PrimitiveFeature::makeBox(10, 10, 10);
    const std::string id = box->featureID();
    tree.addFeature(std::move(box));
    auto shell = std::make_unique<hz::doc::ShellFeature>(
        1.0, std::vector<TopologyID>{TopologyID::fromTag(id + "/top")},
        hz::doc::ShellFeature::Method::Prism);
    const std::string shellId = shell->featureID();
    tree.addFeature(std::move(shell));
    const auto built = tree.buildWithDiagnostics();
    ASSERT_NE(built.solid, nullptr) << built.failureMessage;
    for (const auto& face : built.solid->faces()) {
        EXPECT_EQ(face.topoId.tag().rfind(shellId + "/", 0), 0u) << face.topoId.tag();
    }
    for (const auto& edge : built.solid->edges()) {
        EXPECT_EQ(edge.topoId.tag().rfind(shellId + "/edge:", 0), 0u) << edge.topoId.tag();
    }
}

// Phase 163: a shell made by offsetting each face keeps the part's faces
// and their names (a fillet or a mate on one still finds it); the cavity's
// faces are named after them, as this shell's.
TEST(PersistentNamingTest, AnOffsetShellKeepsThePartsNames) {
    hz::doc::FeatureTree tree;
    auto box = hz::doc::PrimitiveFeature::makeBox(10, 10, 10);
    const std::string id = box->featureID();
    tree.addFeature(std::move(box));
    auto shell = std::make_unique<hz::doc::ShellFeature>(
        1.0, std::vector<TopologyID>{TopologyID::fromTag(id + "/top")});
    const std::string shellId = shell->featureID();
    tree.addFeature(std::move(shell));
    const auto built = tree.buildWithDiagnostics();
    ASSERT_NE(built.solid, nullptr) << built.failureMessage;
    std::set<std::string> names;
    for (const auto& face : built.solid->faces()) {
        names.insert(hz::model::wholeFaceName(face.topoId.tag()));
    }
    for (const char* side : {"/left", "/right", "/front", "/back", "/bottom"}) {
        EXPECT_EQ(names.count(id + side), 1u) << "the box's own " << side;
        EXPECT_EQ(names.count(shellId + "/inner:" + id + side), 1u) << "the cavity's " << side;
    }
}

// A curved face is one face in facets and a curve one edge in chords (Phase
// 139): a cylinder's side is `<id>/side`, its rim `<id>/edge:side|top`, and
// a fillet on that one name rounds the whole rim. Each facet and chord was a
// face and an edge of its own, named apart, picked and listed one by one.
TEST(PersistentNamingTest, ACylinderRimIsOneEdgeToFillet) {
    hz::doc::FeatureTree tree;
    auto cylinder = hz::doc::PrimitiveFeature::makeCylinder(5.0, 10.0);
    const std::string id = cylinder->featureID();
    tree.addFeature(std::move(cylinder));
    auto plain = tree.build();
    ASSERT_NE(plain, nullptr);
    size_t facets = 0;
    for (const auto& face : plain->faces()) {
        facets += face.topoId.tag().rfind(id + "/side/facet:", 0) == 0 ? 1 : 0;
    }
    EXPECT_GT(facets, 8u) << "the side, in facets";
    size_t chords = 0;
    size_t seams = 0;
    for (const auto& edge : plain->edges()) {
        chords += edge.topoId.isDescendantOf(TopologyID::fromTag(id + "/edge:side|top")) ? 1 : 0;
        seams += edge.topoId.tag().rfind(id + "/side/seam:", 0) == 0 ? 1 : 0;
    }
    EXPECT_EQ(chords, facets) << "the top rim, a chord for each facet";
    EXPECT_EQ(seams, facets) << "a seam between each two facets";

    const double before = hz::model::MassPropertiesCalculator::compute(*plain).volume;
    tree.addFeature(std::make_unique<hz::doc::FilletFeature>(
        std::vector<TopologyID>{TopologyID::fromTag(id + "/edge:side|top")}, 1.0));
    const auto built = tree.buildWithDiagnostics();
    ASSERT_NE(built.solid, nullptr) << built.failureMessage;
    EXPECT_EQ(built.failedFeatureIndex, -1) << built.failureMessage;
    // Pappus: the corner's section, (1 - pi/4) r^2, turned about the axis at
    // its centroid, 0.2234 r in from the rim.
    const double pi = std::acos(-1.0);
    const double removed = 2.0 * pi * (5.0 - 0.2234) * (1.0 - pi / 4.0);
    const double after = hz::model::MassPropertiesCalculator::compute(*built.solid).volume;
    EXPECT_NEAR(before - after, removed, 0.15 * removed) << "the whole rim, within faceting";
}

// A revolve names each band after the profile element it sweeps (Phase 139):
// a ring from a rectangle has four faces, each in facets, and the names hold
// when the facet count changes. They were `revolved_<i>_<k>`, by position;
// the top outer rim is one edge. (Filleting it waits for Phase 140: both
// faces at the rim are faceted, and a chain of chords that share no face is
// refused.)
TEST(PersistentNamingTest, ARevolveIsNamedAfterItsProfile) {
    const auto logicalFaces = [](const hz::topo::Solid& solid) {
        std::set<std::string> names;
        for (const auto& face : solid.faces()) {
            names.insert(hz::model::logicalFace(face.topoId.tag()));
        }
        return names;
    };
    auto sketch = std::make_shared<Sketch>();
    sketch->addEntity(std::make_shared<hz::draft::DraftRectangle>(Vec2(2, 0), Vec2(4, 3)));
    auto revolve = std::make_unique<hz::doc::RevolveFeature>(sketch, Vec3(0, 0, 0), Vec3(0, 1, 0),
                                                             2.0 * std::acos(-1.0));
    const std::string id = revolve->featureID();
    hz::doc::RevolveFeature* feature = revolve.get();
    hz::doc::FeatureTree tree;
    tree.addFeature(std::move(revolve));
    auto coarse = tree.build();
    ASSERT_NE(coarse, nullptr);
    const auto names = logicalFaces(*coarse);
    EXPECT_EQ(names.size(), 4u) << "inner, outer, top and bottom";
    for (const auto& name : names) {
        EXPECT_EQ(name.rfind(id + "/revolved:e", 0), 0u) << name;
    }
    ASSERT_TRUE(feature->setParameter("segments", 48.0));
    auto fine = tree.build();
    ASSERT_NE(fine, nullptr);
    EXPECT_GT(fine->faceCount(), coarse->faceCount());
    EXPECT_EQ(logicalFaces(*fine), names) << "the same faces, in more facets";

    // The outer top rim, one edge in chords, a chord for each step round:
    // the edges between the two faces that meet at radius 4, height 3.
    std::string rim;
    for (const auto& edge : fine->edges()) {
        const Vec3& p = edge.halfEdge->origin->point;
        if (std::abs(p.y - 3.0) < 1e-9 && std::abs(std::hypot(p.x, p.z) - 4.0) < 1e-9 &&
            edge.topoId.tag().find("/chord:") != std::string::npos) {
            rim = edge.topoId.tag().substr(0, edge.topoId.tag().find("/chord:"));
            break;
        }
    }
    ASSERT_FALSE(rim.empty());
    size_t chords = 0;
    for (const auto& edge : fine->edges()) {
        chords += edge.topoId.isDescendantOf(TopologyID::fromTag(rim)) ? 1 : 0;
    }
    EXPECT_EQ(chords, 48u) << rim;
}

// A sweep names each side after the profile element it sweeps (Phase 139),
// one facet a path segment, even when the profile runs against the sweep and
// its ring is turned round: the side named for the rectangle's first
// segment is the side along it, whichever way the path goes.
TEST(PersistentNamingTest, ASweepIsNamedAfterItsProfile) {
    std::vector<std::shared_ptr<hz::draft::DraftEntity>> profile{
        std::make_shared<hz::draft::DraftRectangle>(Vec2(0, 0), Vec2(2, 1))};
    const std::string first = "sweep_t/swept:e" + std::to_string(profile.front()->id()) + ".0";
    for (const double way : {1.0, -1.0}) {
        const std::vector<Vec3> path{Vec3(0, 0, 0), Vec3(0, 0, 5 * way), Vec3(3, 0, 8 * way)};
        std::string why;
        auto solid = hz::model::Sweep::execute(profile, hz::draft::SketchPlane(), path, "sweep_t",
                                               hz::model::Sweep::kDefaultArcSegments, 0.0, &why,
                                               NamingScheme::Stable);
        ASSERT_NE(solid, nullptr) << why;
        std::set<std::string> logical;
        size_t facetsOfFirst = 0;
        for (const auto& face : solid->faces()) {
            logical.insert(hz::model::logicalFace(face.topoId.tag()));
            if (hz::model::logicalFace(face.topoId.tag()) != first) continue;
            ++facetsOfFirst;
            // Where the segment from (0, 0) to (2, 0) goes: along the path,
            // the first leg of which keeps y = 0 for it.
            if (face.topoId.tag() == first + "/facet:0") {
                const auto* he = face.outerLoop->halfEdge;
                const auto* start = he;
                do {
                    EXPECT_NEAR(he->origin->point.y, 0.0, 1e-9) << "way " << way;
                    he = he->next;
                } while (he != start);
            }
        }
        EXPECT_EQ(logical.size(), 6u) << "four sides and two caps";
        EXPECT_EQ(facetsOfFirst, 2u) << "a facet for each leg of the path";
    }
}

// A loft names each side after the first section's profile element it
// starts from (Phase 139), a facet for each level; the side named for the
// first segment starts on it, and a twisted level's pieces are parts of it.
TEST(PersistentNamingTest, ALoftIsNamedAfterItsFirstSection) {
    auto rect = std::make_shared<hz::draft::DraftRectangle>(Vec2(0, 0), Vec2(4, 4));
    auto small = std::make_shared<hz::draft::DraftRectangle>(Vec2(1, 1), Vec2(3, 3));
    const hz::draft::SketchPlane base;
    const hz::draft::SketchPlane lifted(Vec3(0, 0, 5), Vec3(0, 0, 1), Vec3(1, 0.3, 0));  // twisted
    std::string why;
    auto solid = hz::model::Loft::execute({{{rect}, base}, {{small}, lifted}}, "loft_t",
                                          hz::model::Loft::kDefaultTwistSegments, &why,
                                          NamingScheme::Stable);
    ASSERT_NE(solid, nullptr) << why;
    const std::string first = "loft_t/lofted:e" + std::to_string(rect->id()) + ".0";
    std::set<std::string> logical;
    std::set<std::pair<double, double>> atBase;  // the side's points on the first section
    for (const auto& face : solid->faces()) {
        const std::string name = hz::model::logicalFace(face.topoId.tag());
        logical.insert(name);
        if (name != first) continue;
        const auto* start = face.outerLoop->halfEdge;
        const auto* he = start;
        do {
            const Vec3& p = he->origin->point;
            if (std::abs(p.z) < 1e-9) atBase.insert({std::round(p.x), std::round(p.y)});
            he = he->next;
        } while (he != start);
    }
    EXPECT_EQ(logical.size(), 6u) << "four sides and two caps";
    const std::set<std::pair<double, double>> segment{{0.0, 0.0}, {4.0, 0.0}};
    EXPECT_EQ(atBase, segment) << "the side starts on the segment from (0, 0) to (4, 0), all of it";

    // Drawn clockwise, the first ring is turned round to wind with the loft:
    // the names still follow the profile, the first segment (0, 0)-(0, 4).
    auto clockwise = std::make_shared<hz::draft::DraftPolyline>(
        std::vector<Vec2>{Vec2(0, 0), Vec2(0, 4), Vec2(4, 4), Vec2(4, 0)}, true);
    auto turned = hz::model::Loft::execute({{{clockwise}, base}, {{small}, lifted}}, "loft_c",
                                           hz::model::Loft::kDefaultTwistSegments, &why,
                                           NamingScheme::Stable);
    ASSERT_NE(turned, nullptr) << why;
    const std::string firstOfTurned = "loft_c/lofted:e" + std::to_string(clockwise->id()) + ".0";
    atBase.clear();
    for (const auto& face : turned->faces()) {
        if (hz::model::logicalFace(face.topoId.tag()) != firstOfTurned) continue;
        const auto* start = face.outerLoop->halfEdge;
        const auto* he = start;
        do {
            const Vec3& p = he->origin->point;
            if (std::abs(p.z) < 1e-9) atBase.insert({std::round(p.x), std::round(p.y)});
            he = he->next;
        } while (he != start);
    }
    const std::set<std::pair<double, double>> firstSegment{{0.0, 0.0}, {0.0, 4.0}};
    EXPECT_EQ(atBase, firstSegment);
}

// A pattern copy's curved faces and curves are its own (Phase 139): the
// original's side is `<id>/side`, the copy's `<id>/side/pattern:1`, and a
// lookup of the original's finds the original's. Copies that meet are joined
// at these names, where they were joined by position and lost them.
TEST(PersistentNamingTest, APatternCopysCurvesAreItsOwn) {
    hz::doc::FeatureTree tree;
    auto cylinder = hz::doc::PrimitiveFeature::makeCylinder(5.0, 10.0);
    const std::string id = cylinder->featureID();
    tree.addFeature(std::move(cylinder));
    tree.addFeature(hz::doc::PatternFeature::makeLinear(Vec3(1, 0, 0), 8.0, 2));  // overlapping
    const auto built = tree.buildWithDiagnostics();
    ASSERT_NE(built.solid, nullptr) << built.failureMessage;
    std::set<std::string> logical;
    for (const auto& face : built.solid->faces()) {
        logical.insert(hz::model::logicalFace(face.topoId.tag()));
    }
    // Split by the join, a side is in pieces; its facets are still its own.
    const auto has = [&logical](const std::string& prefix) {
        return std::any_of(logical.begin(), logical.end(),
                           [&prefix](const std::string& n) { return n.rfind(prefix, 0) == 0; });
    };
    EXPECT_TRUE(has(id + "/side")) << "the original's side";
    EXPECT_TRUE(has(id + "/side/pattern:1")) << "the copy's, apart";

    const hz::topo::Face* found =
        hz::model::MateGeometry::findFace(*built.solid, TopologyID::fromTag(id + "/side"));
    ASSERT_NE(found, nullptr);
    EXPECT_EQ(found->topoId.tag().find("/pattern:"), std::string::npos) << "the original's";
}

namespace {

/// The logical faces of @p solid, and its logical edges less the seams: what
/// the Shell and Fillet dialogs list.
std::pair<std::set<std::string>, std::set<std::string>> logicalNames(const hz::topo::Solid& solid) {
    std::set<std::string> faces;
    std::set<std::string> edges;
    for (const auto& face : solid.faces()) faces.insert(hz::model::logicalFace(face.topoId.tag()));
    for (const auto& edge : solid.edges()) {
        if (edge.topoId.tag().find("/seam:") != std::string::npos) continue;
        edges.insert(hz::model::logicalEdge(edge.topoId.tag()));
    }
    return {faces, edges};
}

}  // namespace

// A rounded or chamfered rim is one face, and its two edges are one curve
// each (Phase 139 review). The faces were named after the chords and bands
// they round, 256 of them for a rim, and the edges between them took the
// chords' `/chord:` into their names: hundreds of rows, some of them two
// or three edges under one name.
TEST(PersistentNamingTest, ARoundedRimIsOneFace) {
    for (const bool fillet : {true, false}) {
        hz::doc::FeatureTree tree;
        auto cylinder = hz::doc::PrimitiveFeature::makeCylinder(5.0, 10.0);
        const std::string id = cylinder->featureID();
        tree.addFeature(std::move(cylinder));
        const std::vector<TopologyID> rim{TopologyID::fromTag(id + "/edge:side|top")};
        std::unique_ptr<hz::doc::Feature> round;
        if (fillet) {
            round = std::make_unique<hz::doc::FilletFeature>(rim, 1.0);
        } else {
            round = std::make_unique<hz::doc::ChamferFeature>(rim, 1.0);
        }
        const std::string blend =
            round->featureID() + (fillet ? "/fillet/" : "/chamfer/") + id + "/edge:side|top";
        tree.addFeature(std::move(round));
        const auto built = tree.buildWithDiagnostics();
        ASSERT_NE(built.solid, nullptr) << built.failureMessage;

        const auto [faces, edges] = logicalNames(*built.solid);
        const std::set<std::string> expected{id + "/bottom", id + "/top", id + "/side", blend};
        EXPECT_EQ(faces, expected) << (fillet ? "fillet" : "chamfer");
        const std::set<std::string> curves{id + "/edge:bottom|side",
                                           "edge:" + blend + "|" + id + "/side",
                                           "edge:" + blend + "|" + id + "/top"};
        EXPECT_EQ(edges, curves) << (fillet ? "fillet" : "chamfer");
    }
}

// A cylinder a cut parts in two is still one side (Phase 139 review): the
// pieces of its facets, `side/facet:<k>/piece:<n>`, were grouped as
// `side/piece:<n>`, each group the first or second piece of every facet,
// whichever half of the cylinder that was.
TEST(PersistentNamingTest, ACutCylinderIsStillOneSide) {
    hz::doc::FeatureTree tree;
    auto cylinder = hz::doc::PrimitiveFeature::makeCylinder(5.0, 10.0);
    const std::string id = cylinder->featureID();
    tree.addFeature(std::move(cylinder));
    auto notch = hz::doc::PrimitiveFeature::makeBox(10, 20, 2);  // x 3..13, z 4..6
    ASSERT_TRUE(notch->setVector("basePoint", Vec3(3, -10, 4)));
    notch->setOperation(BodyOperation::Cut);
    tree.addFeature(std::move(notch));
    const auto built = tree.buildWithDiagnostics();
    ASSERT_NE(built.solid, nullptr) << built.failureMessage;

    bool pieces = false;
    std::set<std::string> sides;
    for (const auto& face : built.solid->faces()) {
        const std::string& tag = face.topoId.tag();
        if (tag.rfind(id + "/side/", 0) != 0) continue;
        pieces = pieces || tag.find("/piece:") != std::string::npos;
        sides.insert(hz::model::logicalFace(tag));
    }
    EXPECT_TRUE(pieces) << "the notch parts the facets it crosses";
    EXPECT_EQ(sides, std::set<std::string>{id + "/side"});
    const hz::topo::Face* found =
        hz::model::MateGeometry::findFace(*built.solid, TopologyID::fromTag(id + "/side"));
    ASSERT_NE(found, nullptr);
    EXPECT_NE(found->analyticSurface, nullptr) << "the cylinder, for a concentric mate";
}

// An older loft's twisted level, cut into triangles named `<side>/facet:<k>`
// in every scheme, is grouped as its side (Phase 139 review): each triangle
// carries the side's ruled patch as its ideal, so a reference to the side
// finds one with the same ideal, and a mate on it the same frame.
TEST(PersistentNamingTest, AnOlderLoftsTwistedSideIsOneFace) {
    auto rect = std::make_shared<hz::draft::DraftRectangle>(Vec2(0, 0), Vec2(4, 4));
    auto small = std::make_shared<hz::draft::DraftRectangle>(Vec2(1, 1), Vec2(3, 3));
    const hz::draft::SketchPlane base;
    const hz::draft::SketchPlane lifted(Vec3(0, 0, 5), Vec3(0, 0, 1), Vec3(1, 0.3, 0));  // twisted
    std::string why;
    auto solid = hz::model::Loft::execute({{{rect}, base}, {{small}, lifted}}, "loft_t",
                                          hz::model::Loft::kDefaultTwistSegments, &why,
                                          NamingScheme::FromGeometry);
    ASSERT_NE(solid, nullptr) << why;
    std::map<std::string, std::set<const hz::geo::NurbsSurface*>> ideals;
    for (const auto& face : solid->faces()) {
        const std::string& tag = face.topoId.tag();
        if (tag.find("/facet:") == std::string::npos) continue;
        ideals[hz::model::logicalFace(tag)].insert(face.analyticSurface.get());
    }
    ASSERT_EQ(ideals.size(), 4u) << "four twisted sides";
    EXPECT_TRUE(ideals.count("loft_t/lateral_0_0")) << "the names they always had";
    for (const auto& [side, surfaces] : ideals) {
        ASSERT_EQ(surfaces.size(), 1u) << side;
        ASSERT_NE(*surfaces.begin(), nullptr) << side;
        const hz::topo::Face* found =
            hz::model::MateGeometry::findFace(*solid, TopologyID::fromTag(side));
        ASSERT_NE(found, nullptr) << side;
        EXPECT_EQ(found->analyticSurface.get(), *surfaces.begin()) << side;
    }
}

// A pattern copy's side a later cut parts is still one side (Phase 139
// review): the piece comes after the copy's `/pattern:1`, where it was kept,
// so the copy's side was two, and a mate saved on it found neither.
TEST(PersistentNamingTest, ACutPatternCopyIsStillOneSide) {
    hz::doc::FeatureTree tree;
    auto cylinder = hz::doc::PrimitiveFeature::makeCylinder(5.0, 10.0);
    const std::string id = cylinder->featureID();
    tree.addFeature(std::move(cylinder));
    tree.addFeature(hz::doc::PatternFeature::makeLinear(Vec3(1, 0, 0), 20.0, 2));  // apart
    // Across the +y side of both, and part of the way round the first.
    auto notch = hz::doc::PrimitiveFeature::makeBox(50, 8, 2);
    ASSERT_TRUE(notch->setVector("basePoint", Vec3(3, 2, 4)));
    notch->setOperation(BodyOperation::Cut);
    tree.addFeature(std::move(notch));
    const auto built = tree.buildWithDiagnostics();
    ASSERT_NE(built.solid, nullptr) << built.failureMessage;

    bool pieces = false;
    std::set<std::string> sides;
    for (const auto& face : built.solid->faces()) {
        const std::string& tag = face.topoId.tag();
        if (tag.rfind(id + "/side/", 0) != 0) continue;
        pieces = pieces || tag.find("/pattern:1/piece:") != std::string::npos;
        sides.insert(hz::model::logicalFace(tag));
    }
    EXPECT_TRUE(pieces) << "the notch parts the copy's facets";
    EXPECT_EQ(sides, (std::set<std::string>{id + "/side", id + "/side/pattern:1"}));
    const hz::topo::Face* found = hz::model::MateGeometry::findFace(
        *built.solid, TopologyID::fromTag(id + "/side/pattern:1"));
    ASSERT_NE(found, nullptr);
    EXPECT_NE(found->topoId.tag().find("/pattern:1"), std::string::npos) << "the copy's";
}

// Phase 164: a revolve's flat end is one face, so the rim where it meets
// the revolve's side is filleted as a cylinder's is: a disk's rim (a
// profile touching the axis) and a ring's outer rim (the end is a ring,
// its hole kept). Checked by Pappus, as the cylinder's rim is.
TEST(PersistentNamingTest, ARevolvesRimIsFilleted) {
    const double pi = std::acos(-1.0);
    const auto rimFillet = [pi](double inner, double outer, double& removedOut) {
        auto sketch = std::make_shared<Sketch>();
        sketch->addEntity(std::make_shared<hz::draft::DraftRectangle>(Vec2(inner, 0), Vec2(outer, 3)));
        auto revolve = std::make_unique<hz::doc::RevolveFeature>(sketch, Vec3(0, 0, 0),
                                                                 Vec3(0, 1, 0), 2.0 * pi);
        const std::string id = revolve->featureID();
        hz::doc::FeatureTree tree;
        tree.addFeature(std::move(revolve));
        auto plain = tree.build();
        EXPECT_NE(plain, nullptr);
        if (!plain) return std::string();
        // The rim at the top (y = 3) and the outer radius: its whole name.
        std::string rim;
        for (const auto& edge : plain->edges()) {
            const auto* h = edge.halfEdge;
            const Vec3 a = h->origin->point;
            const Vec3 b = h->twin->origin->point;
            if (std::abs(a.y - 3.0) < 1e-9 && std::abs(b.y - 3.0) < 1e-9 &&
                std::abs(std::hypot(a.x, a.z) - outer) < 1e-6 &&
                std::abs(std::hypot(b.x, b.z) - outer) < 1e-6) {
                rim = hz::model::logicalEdge(edge.topoId.tag());
                break;
            }
        }
        EXPECT_FALSE(rim.empty());
        const double before = hz::model::MassPropertiesCalculator::compute(*plain).volume;
        tree.addFeature(std::make_unique<hz::doc::FilletFeature>(
            std::vector<TopologyID>{TopologyID::fromTag(rim)}, 0.5));
        const auto built = tree.buildWithDiagnostics();
        if (!built.solid || built.failedFeatureIndex != -1) return built.failureMessage;
        removedOut = before - hz::model::MassPropertiesCalculator::compute(*built.solid).volume;
        EXPECT_TRUE(built.solid->isValid());
        return std::string();
    };
    // The corner's section (1 - pi/4) r^2, about the axis at its centroid,
    // 0.2234 r in from the rim.
    const double r = 0.5;
    for (const double inner : {0.0, 2.0}) {
        double removed = 0.0;
        const std::string why = rimFillet(inner, 4.0, removed);
        ASSERT_TRUE(why.empty()) << "inner radius " << inner << ": " << why;
        const double expected = 2.0 * pi * (4.0 - 0.2234 * r) * (1.0 - pi / 4.0) * r * r;
        EXPECT_NEAR(removed, expected, 0.15 * expected) << "inner radius " << inner;
    }
}
