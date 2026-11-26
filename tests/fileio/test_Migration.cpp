// Format 32 (stage 3, Task 11): a part of format 31 or older is migrated on open and says what
// changed. Its features named the old ways become Stable, and each reference that can no longer
// mean what it meant is dropped and reported (so the part is read in part); a curve drawn with few
// chords before is said to be round now; its file's tessellation cache is not read.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <nlohmann/json.hpp>
#include <sstream>
#include <string>
#include <vector>

#include "horizon/document/AssemblyDocument.h"
#include "horizon/document/Document.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/document/Sketch.h"
#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/SketchPlane.h"
#include "horizon/fileio/ImportReport.h"
#include "horizon/fileio/NativeFormat.h"
#include "horizon/math/Constants.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/modeling/Naming.h"
#include "horizon/topology/Solid.h"
#include "horizon/topology/TopologyID.h"

using namespace hz::doc;
using hz::io::ImportReport;
using hz::io::NativeFormat;
using hz::math::kPi;
using hz::math::Vec2;
using hz::math::Vec3;
using hz::model::NamingScheme;
using hz::topo::TopologyID;
using json = nlohmann::json;

namespace {

/// @p doc as a file of format @p version holds it.
std::string asFormat(const Document& doc, int version) {
    json root = json::parse(NativeFormat::documentToJson(doc, false));
    root["version"] = version;
    return root.dump();
}

/// A document read from text, with what its read reported.
struct Opened {
    std::unique_ptr<Document> doc = std::make_unique<Document>();
    ImportReport report;
};

Opened openText(const std::string& text) {
    Opened opened;
    std::string error;
    EXPECT_TRUE(NativeFormat::documentFromJson(text, *opened.doc, &error, &opened.report)) << error;
    return opened;
}

std::string joined(const std::vector<std::string>& lines) {
    std::string all;
    for (const auto& line : lines) all += line + "\n";
    return all;
}

/// The line a reference not carried over is reported with.
std::string notCarried(const std::string& owner, const std::string& name, const std::string& what,
                       const std::string& why) {
    return owner + ": its reference to " + name + ", " + what + ", is not carried over (" + why +
           ")";
}

/// Lines through @p points, end to end, and back to the first when @p closed.
std::shared_ptr<Sketch> lines(const std::vector<Vec2>& points, bool closed = true) {
    auto sketch = std::make_shared<Sketch>();
    const std::size_t n = points.size();
    const std::size_t count = closed ? n : n - 1;
    for (std::size_t i = 0; i < count; ++i) {
        sketch->addEntity(std::make_shared<hz::draft::DraftLine>(points[i], points[(i + 1) % n]));
    }
    return sketch;
}

std::shared_ptr<Sketch> square(double side) {
    return lines({{0, 0}, {side, 0}, {side, side}, {0, side}});
}

/// A line along x and the half circle over it: an extrusion of it has an arc's rim.
std::shared_ptr<Sketch> dShape() {
    auto sketch = std::make_shared<Sketch>();
    sketch->addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(-5, 0), Vec2(5, 0)));
    sketch->addEntity(std::make_shared<hz::draft::DraftArc>(Vec2(0, 0), 5.0, 0.0, kPi));
    return sketch;
}

/// Entity @p i of @p sketch as a profile names it.
std::string entity(const Sketch& sketch, std::size_t i) {
    return "e" + std::to_string(sketch.entities().at(i)->id());
}

double volumeOf(const hz::topo::Solid& solid) {
    return hz::model::MassPropertiesCalculator::compute(solid).volume;
}

std::vector<std::string> tagsOf(const std::vector<TopologyID>& ids) {
    std::vector<std::string> tags;
    tags.reserve(ids.size());
    for (const auto& id : ids) tags.push_back(id.tag());
    return tags;
}

std::filesystem::path tempPath(const std::string& name) {
    return std::filesystem::temp_directory_path() / name;
}

std::string readText(const std::filesystem::path& path) {
    std::ifstream in(path, std::ios::binary);
    std::ostringstream text;
    text << in.rdbuf();
    return text.str();
}

}  // namespace

// Review Focus 4, part side: a Stable part saved before format 32, with a fillet on one chord of an
// extruded arc's rim. Ruling 21 numbers the bridge's chords as master did, so the reference means
// what it meant and is kept as it is (Ruling 40): not widened to the whole rim, nothing reported,
// and the fillet builds as on master.
TEST(Migration, AStablePartCarriesItsFacetNames) {
    Document doc;
    doc.setType(DocumentType::Part);
    auto sketch = dShape();
    doc.addSketch(sketch);
    auto extrude = std::make_unique<ExtrudeFeature>(sketch, Vec3(0, 0, 1), 10.0);
    const std::string chord =
        extrude->featureID() + "/edge:cap_top|side:" + entity(*sketch, 1) + "/chord:3";
    doc.featureTree().addFeature(std::move(extrude));
    doc.featureTree().addFeature(
        std::make_unique<FilletFeature>(std::vector<TopologyID>{TopologyID::fromTag(chord)}, 0.1));

    Opened opened = openText(asFormat(doc, 31));
    EXPECT_TRUE(opened.report.empty())
        << opened.report.summary() << "\n"
        << joined(opened.report.skipped) << joined(opened.report.approximated);
    EXPECT_FALSE(opened.doc->readInPart());
    const auto* fillet = dynamic_cast<const FilletFeature*>(opened.doc->featureTree().feature(1));
    ASSERT_NE(fillet, nullptr);
    EXPECT_EQ(tagsOf(fillet->edgeIds()), std::vector<std::string>{chord}) << "kept as it was";

    ASSERT_TRUE(opened.doc->rebuildModel()) << opened.doc->lastBuildMessage();
    // Master's: its faceted extrusion (the FromGeometry construction, which builds as master's
    // did), the same chord (`side:<arc>.f3`) filleted 0.1, recorded at 9b88fecf.
    constexpr double kMasters = 390.17799077914088;
    EXPECT_NEAR(volumeOf(*opened.doc->solid()), kMasters, kMasters * 1e-9);
}

// A part from before persistent naming: its fillet's edge was named by position, which an exact
// rebuild cannot keep, so it is dropped and said; the part is read in part, and its features are
// Stable.
TEST(Migration, APositionalPartsReferencesAreReported) {
    Document doc;
    doc.setType(DocumentType::Part);
    auto sketch = square(10.0);
    doc.addSketch(sketch);
    auto extrude = std::make_unique<ExtrudeFeature>(sketch, Vec3(0, 0, 1), 5.0);
    extrude->setNaming(NamingScheme::Positional);
    const std::string edge7 = extrude->featureID() + "/edge7";
    doc.featureTree().addFeature(std::move(extrude));
    auto fillet =
        std::make_unique<FilletFeature>(std::vector<TopologyID>{TopologyID::fromTag(edge7)}, 1.0);
    fillet->setNaming(NamingScheme::Positional);
    doc.featureTree().addFeature(std::move(fillet));
    const std::string text = asFormat(doc, 18);
    ASSERT_EQ(text.find("\"naming\""), std::string::npos) << "positional: as format 18 kept it";

    Opened opened = openText(text);
    const std::string line =
        notCarried("feature 2 (Fillet)", edge7, "an edge it rounds", "named the old way");
    EXPECT_EQ(opened.report.skipped, std::vector<std::string>{line});
    EXPECT_TRUE(opened.doc->readInPart());
    EXPECT_EQ(opened.doc->leftOut(), opened.report.skipped);
    const FeatureTree& tree = opened.doc->featureTree();
    EXPECT_EQ(tree.feature(0)->naming(), NamingScheme::Stable);
    EXPECT_EQ(tree.feature(1)->naming(), NamingScheme::Stable);
    const auto* kept = dynamic_cast<const FilletFeature*>(tree.feature(1));
    ASSERT_NE(kept, nullptr);
    EXPECT_TRUE(kept->edgeIds().empty()) << "dropped, never guessed";

    // Read without a report, it is read in part all the same.
    Document quiet;
    std::string error;
    ASSERT_TRUE(NativeFormat::documentFromJson(text, quiet, &error)) << error;
    EXPECT_EQ(quiet.leftOut(), std::vector<std::string>{line});
}

TEST(Migration, ASixSidedCylinderIsNowRound) {
    Document doc;
    doc.setType(DocumentType::Part);
    auto cylinder = PrimitiveFeature::makeCylinder(2.0, 5.0);
    ASSERT_TRUE(cylinder->setParameter("segments", 6));
    doc.featureTree().addFeature(std::move(cylinder));

    Opened opened = openText(asFormat(doc, 31));
    EXPECT_EQ(opened.report.approximated,
              std::vector<std::string>{
                  "feature 1 (Cylinder): drawn as a 6-sided prism before; round now"});
    EXPECT_TRUE(opened.report.skipped.empty()) << joined(opened.report.skipped);
    EXPECT_FALSE(opened.doc->readInPart()) << "nothing was left out";

    ASSERT_TRUE(opened.doc->rebuildModel()) << opened.doc->lastBuildMessage();
    ASSERT_NE(opened.doc->exactSolid(), nullptr) << opened.doc->exactNote();
    const double volume = kPi * 4.0 * 5.0;
    EXPECT_NEAR(volumeOf(*opened.doc->exactSolid()), volume, volume * 1e-9);
}

TEST(Migration, SavedPartsAreVersion32) {
    Document doc;
    doc.setType(DocumentType::Part);
    doc.featureTree().addFeature(PrimitiveFeature::makeBox(1, 2, 3));
    EXPECT_EQ(json::parse(NativeFormat::documentToJson(doc, false)).at("version").get<int>(), 32);

    // An old part opened and saved again is a format 32 part, its features Stable.
    auto old = PrimitiveFeature::makeBox(1, 2, 3);
    old->setNaming(NamingScheme::FromGeometry);
    Document older;
    older.setType(DocumentType::Part);
    older.featureTree().addFeature(std::move(old));
    Opened opened = openText(asFormat(older, 25));
    const json saved = json::parse(NativeFormat::documentToJson(*opened.doc, false));
    EXPECT_EQ(saved.at("version").get<int>(), 32);
    EXPECT_EQ(saved.at("featureTree").at(0).value("naming", 1), 3);

    // And an assembly, whose mates a later step checks (Task 12).
    AssemblyDocument assembly;
    ComponentInstance component;
    component.name = "base";
    component.partPath = "base.hzpart";
    assembly.addComponent(component);
    EXPECT_EQ(json::parse(NativeFormat::assemblyToJson(assembly, "")).at("version").get<int>(), 32);
}

// An assembly saved before format 32 still opens.
TEST(Migration, AnAssemblyOfFormat31StillOpens) {
    AssemblyDocument assembly;
    ComponentInstance component;
    component.name = "base";
    component.partPath = "base.hzpart";
    assembly.addComponent(component);
    json root = json::parse(NativeFormat::assemblyToJson(assembly, ""));
    root["version"] = 31;

    AssemblyDocument opened;
    std::string error;
    ImportReport report;
    ASSERT_TRUE(NativeFormat::assemblyFromJson(root.dump(), opened, "", &error, &report)) << error;
    EXPECT_EQ(opened.components().size(), 1u);
    EXPECT_TRUE(report.empty()) << joined(report.skipped);
}

// The mesh a part file keeps for an assembly to draw it without building it: one of format 31 or
// older is master's facets, not the exact part this build makes, and is not read.
TEST(Migration, AStaleCacheIsDropped) {
    Document doc;
    doc.setType(DocumentType::Part);
    doc.featureTree().addFeature(PrimitiveFeature::makeCylinder(2.0, 5.0));
    ASSERT_TRUE(doc.rebuildModel()) << doc.lastBuildMessage();
    const auto path = tempPath("hz_test_migration_cache.hzpart");
    ASSERT_TRUE(NativeFormat::save(path.string(), doc));
    ASSERT_NE(NativeFormat::loadPartMesh(path.string()), nullptr) << "format 32's is read";

    json root = json::parse(readText(path));
    ASSERT_TRUE(root.contains("tessellationCache"));
    root["version"] = 31;
    std::ofstream(path, std::ios::binary | std::ios::trunc) << root.dump();
    EXPECT_EQ(NativeFormat::loadPartMesh(path.string()), nullptr) << "format 31's is not";

    // The part itself still opens, and builds.
    Document reopened;
    std::string error;
    ASSERT_TRUE(NativeFormat::load(path.string(), reopened, &error)) << error;
    EXPECT_TRUE(reopened.rebuildModel()) << reopened.lastBuildMessage();
    std::filesystem::remove(path);
}

// A part of format 32 is not migrated: what it holds is as this build wrote it (a scheme set by
// code or a test stays as it was set).
TEST(Migration, APartOfFormat32IsNotMigrated) {
    Document doc;
    doc.setType(DocumentType::Part);
    auto cylinder = PrimitiveFeature::makeCylinder(2.0, 5.0);
    ASSERT_TRUE(cylinder->setParameter("segments", 6));
    cylinder->setNaming(NamingScheme::Positional);
    doc.featureTree().addFeature(std::move(cylinder));
    doc.featureTree().addFeature(std::make_unique<FilletFeature>(
        std::vector<TopologyID>{TopologyID::fromTag("cylinder/edge3")}, 0.1));

    Opened opened = openText(NativeFormat::documentToJson(doc, false));
    EXPECT_TRUE(opened.report.empty()) << joined(opened.report.skipped);
    EXPECT_EQ(opened.doc->featureTree().feature(0)->naming(), NamingScheme::Positional);
}

// A sketch on a face of a feature named the old way stops following it, and an edge projected
// from it stops being drawn again from it: each is said, and the sketch stays where it was last
// placed, its edge as it was last drawn.
TEST(Migration, AnOldFaceASketchIsOnIsReportedAndTheSketchStaysWhereItWas) {
    Document doc;
    doc.setType(DocumentType::Part);
    auto base = square(10.0);
    doc.addSketch(base);
    auto extrude = std::make_unique<ExtrudeFeature>(base, Vec3(0, 0, 1), 5.0);
    extrude->setNaming(NamingScheme::FromGeometry);
    const std::string id = extrude->featureID();
    doc.featureTree().addFeature(std::move(extrude));

    auto top = lines({{2, 2}, {4, 2}, {4, 4}, {2, 4}});
    top->setName("Top");
    top->setFace(id + "/cap_top");
    top->setPlaced(hz::draft::SketchPlane(Vec3(0, 0, 5), Vec3(0, 0, 1), Vec3(1, 0, 0)));
    auto guide = std::make_shared<hz::draft::DraftLine>(Vec2(0, 0), Vec2(10, 0));
    guide->setConstruction(true);
    const std::string edge = id + "/edge:cap_top|side:" + entity(*base, 0);
    guide->setSourceEdge(edge);
    top->addEntity(guide);
    doc.addSketch(top);
    doc.featureTree().addFeature(std::make_unique<ExtrudeFeature>(top, Vec3(0, 0, 1), 2.0));

    Opened opened = openText(asFormat(doc, 25));
    const std::vector<std::string> expected = {
        notCarried("feature 2 (Extrude)", id + "/cap_top", "the face sketch 'Top' is on",
                   "named the old way"),
        notCarried("feature 2 (Extrude)", edge, "an edge projected into sketch 'Top'",
                   "named the old way")};
    EXPECT_EQ(opened.report.skipped, expected) << joined(opened.report.skipped);
    EXPECT_TRUE(opened.doc->readInPart());

    const Sketch* sketch = opened.doc->sketches().at(1).get();
    EXPECT_TRUE(sketch->face().empty());
    ASSERT_TRUE(sketch->placed().has_value()) << "where it was last placed";
    EXPECT_NEAR((sketch->plane().origin() - Vec3(0, 0, 5)).length(), 0.0, 1e-12);
    for (const auto& e : sketch->entities()) {
        EXPECT_TRUE(e->sourceEdge().empty());
    }
    EXPECT_EQ(sketch->entities().size(), 5u) << "the guide stays, as it was last drawn";
}

// Ruling 23: a profile with holes (or of several regions) was master's Boolean, which named the
// pieces of its caps and the chords and seams of what it merged; the exact extrusion is one face a
// cap, numbered its own way. A reference to such a part is said and dropped; one to a whole face or
// edge is kept.
TEST(Migration, AHoledProfilesPiecesChordsAndSeamsAreReported) {
    Document doc;
    doc.setType(DocumentType::Part);
    auto plate = square(20.0);
    plate->addEntity(std::make_shared<hz::draft::DraftCircle>(Vec2(10, 10), 3.0));
    doc.addSketch(plate);
    auto extrude = std::make_unique<ExtrudeFeature>(plate, Vec3(0, 0, 1), 5.0);
    const std::string id = extrude->featureID();
    doc.featureTree().addFeature(std::move(extrude));
    const std::string rim =
        "edge:" + id + "/cap_top|" + id + "~hole0/side:" + entity(*plate, 4) + "/chord:2";
    const std::string side = id + "/edge:cap_top|side:" + entity(*plate, 0);
    doc.featureTree().addFeature(std::make_unique<ChamferFeature>(
        std::vector<TopologyID>{TopologyID::fromTag(side), TopologyID::fromTag(rim)}, 0.2));
    auto mirror = MirrorFeature::make(Vec3(0, 0, 0), Vec3(1, 0, 0));
    mirror->setReference("planeFace", id + "/cap_bottom/piece:0");
    doc.featureTree().addFeature(std::move(mirror));
    doc.featureTree().addFeature(HoleFeature::make(id + "/cap_top", Vec3(3, 3, 5), 1.0, 2.0));

    Opened opened = openText(asFormat(doc, 31));
    const std::vector<std::string> expected = {
        notCarried("feature 2 (Chamfer)", rim, "an edge it bevels", "built from pieces before"),
        notCarried("feature 3 (Mirror)", id + "/cap_bottom/piece:0", "the face it mirrors in",
                   "built from pieces before")};
    EXPECT_EQ(opened.report.skipped, expected) << joined(opened.report.skipped);
    const FeatureTree& tree = opened.doc->featureTree();
    const auto* chamfer = dynamic_cast<const ChamferFeature*>(tree.feature(1));
    ASSERT_NE(chamfer, nullptr);
    EXPECT_EQ(tagsOf(chamfer->edgeIds()), std::vector<std::string>{side}) << "a whole edge stays";
    const auto* mirrored = dynamic_cast<const MirrorFeature*>(tree.feature(2));
    ASSERT_NE(mirrored, nullptr);
    EXPECT_TRUE(mirrored->planeFace().empty());
    const auto* hole = dynamic_cast<const HoleFeature*>(tree.feature(3));
    ASSERT_NE(hole, nullptr);
    EXPECT_EQ(hole->face(), id + "/cap_top") << "a whole face stays";
}

// Ruling 25: through part of a turn, a face square to the axis was a quad a step on master, and is
// one face now; the seams between its quads are gone. A seam of a curved face is as it was.
TEST(Migration, APartTurnsFlatFacesSeamsAreReported) {
    Document doc;
    doc.setType(DocumentType::Part);
    auto ring = lines({{2, 0}, {4, 0}, {4, 3}, {2, 3}});
    doc.addSketch(ring);
    auto revolve = std::make_unique<RevolveFeature>(ring, Vec3(0, 0, 0), Vec3(0, 1, 0), kPi / 2.0);
    const std::string id = revolve->featureID();
    doc.featureTree().addFeature(std::move(revolve));
    const std::string flat = id + "/revolved:" + entity(*ring, 0) + "/seam:2";
    const std::string curved = id + "/revolved:" + entity(*ring, 1) + "/seam:2";
    doc.featureTree().addFeature(std::make_unique<FilletFeature>(
        std::vector<TopologyID>{TopologyID::fromTag(flat), TopologyID::fromTag(curved)}, 0.1));

    Opened opened = openText(asFormat(doc, 31));
    EXPECT_EQ(opened.report.skipped,
              std::vector<std::string>{notCarried("feature 2 (Fillet)", flat, "an edge it rounds",
                                                  "a flat face is now one face")})
        << joined(opened.report.skipped);
    const auto* fillet = dynamic_cast<const FilletFeature*>(opened.doc->featureTree().feature(1));
    ASSERT_NE(fillet, nullptr);
    EXPECT_EQ(tagsOf(fillet->edgeIds()), std::vector<std::string>{curved});

    // A whole turn's flat faces were one face on master too: nothing about them is said.
    Document whole;
    whole.setType(DocumentType::Part);
    auto again = lines({{2, 0}, {4, 0}, {4, 3}, {2, 3}});
    whole.addSketch(again);
    auto turned = std::make_unique<RevolveFeature>(again, Vec3(0, 0, 0), Vec3(0, 1, 0), 2.0 * kPi);
    const std::string seam = turned->featureID() + "/revolved:" + entity(*again, 0) + "/seam:2";
    whole.featureTree().addFeature(std::move(turned));
    whole.featureTree().addFeature(
        std::make_unique<FilletFeature>(std::vector<TopologyID>{TopologyID::fromTag(seam)}, 0.1));
    const Opened wholeOpened = openText(asFormat(whole, 31));
    EXPECT_TRUE(wholeOpened.report.skipped.empty()) << joined(wholeOpened.report.skipped);
}

// Ruling 28: master named a sheet's faces after lines it made anew in each build, so no reference
// into a sheet from before format 32 means what it meant.
TEST(Migration, EveryReferenceIntoAnOldSheetIsReported) {
    Document doc;
    doc.setType(DocumentType::Part);
    auto bent = lines({{0, 0}, {50, 0}, {50, 30}}, /*closed=*/false);
    doc.addSketch(bent);
    auto sheet = std::make_unique<SheetMetalFeature>(bent, 2.0, 1.0, 20.0);
    const std::string id = sheet->featureID();
    doc.featureTree().addFeature(std::move(sheet));
    doc.featureTree().addFeature(HoleFeature::make(id + "/side:e9", Vec3(10, 0, 10), 3.0, 2.0));

    Opened opened = openText(asFormat(doc, 31));
    EXPECT_EQ(opened.report.skipped,
              std::vector<std::string>{notCarried("feature 2 (Hole)", id + "/side:e9",
                                                  "the face it is drilled into",
                                                  "a sheet's faces are named anew")})
        << joined(opened.report.skipped);
    const auto* hole = dynamic_cast<const HoleFeature*>(opened.doc->featureTree().feature(1));
    ASSERT_NE(hole, nullptr);
    EXPECT_TRUE(hole->face().empty());
}

// A Join made the old way leaves the names of the faces it does not make: a reference made after
// it, to a face of a Stable feature before it, means what it meant. It is kept, nothing is said,
// and it finds its face once the Join is named Stable (the review of Task 11, finding 1). A name
// the old Join made or rewrote carries its id, and is said (APositionalPartsReferencesAreReported).
TEST(Migration, AStableFeaturesFaceIsKeptThroughAnOldJoin) {
    Document doc;
    doc.setType(DocumentType::Part);
    auto box = PrimitiveFeature::makeBox(10, 10, 10);
    const std::string bottom = box->featureID() + "/bottom";
    doc.featureTree().addFeature(std::move(box));
    // A post standing out of the box's top, joined the old way.
    auto post = lines({{1, 1}, {3, 1}, {3, 3}, {1, 3}});
    post->setPlane(hz::draft::SketchPlane(Vec3(0, 0, 5), Vec3(0, 0, 1), Vec3(1, 0, 0)));
    doc.addSketch(post);
    auto join = std::make_unique<ExtrudeFeature>(post, Vec3(0, 0, 1), 10.0);
    join->setOperation(BodyOperation::Join);
    join->setNaming(NamingScheme::Positional);
    doc.featureTree().addFeature(std::move(join));
    // A hole drilled up into the box's bottom, which the Join does not reach.
    doc.featureTree().addFeature(HoleFeature::make(bottom, Vec3(7, 7, 0), 2.0, 3.0));

    Opened opened = openText(asFormat(doc, 25));
    EXPECT_TRUE(opened.report.skipped.empty()) << joined(opened.report.skipped);
    EXPECT_FALSE(opened.doc->readInPart());
    const FeatureTree& tree = opened.doc->featureTree();
    EXPECT_EQ(tree.feature(1)->naming(), NamingScheme::Stable);
    const auto* hole = dynamic_cast<const HoleFeature*>(tree.feature(2));
    ASSERT_NE(hole, nullptr);
    EXPECT_EQ(hole->face(), bottom) << "kept as it was";
    ASSERT_TRUE(opened.doc->rebuildModel()) << opened.doc->lastBuildMessage();
    // The box and the post above its top (1000 + 2 x 2 x 5), less the hole: it found its face.
    const double volume = volumeOf(*opened.doc->solid());
    EXPECT_LT(volume, 1020.0 - kPi * 3.0) << "the hole is cut";
    EXPECT_GT(volume, 1000.0);

    // The old one first: a Stable feature after it names what it makes its own way.
    Document later;
    later.setType(DocumentType::Part);
    auto oldBox = PrimitiveFeature::makeBox(10, 10, 10);
    oldBox->setNaming(NamingScheme::Positional);
    later.featureTree().addFeature(std::move(oldBox));
    auto profile = square(4.0);
    later.addSketch(profile);
    auto stable = std::make_unique<ExtrudeFeature>(profile, Vec3(0, 0, 1), 12.0);
    stable->setOperation(BodyOperation::Join);
    const std::string own = stable->featureID() + "/edge:cap_top|side:" + entity(*profile, 0);
    later.featureTree().addFeature(std::move(stable));
    later.featureTree().addFeature(
        std::make_unique<FilletFeature>(std::vector<TopologyID>{TopologyID::fromTag(own)}, 0.5));
    const Opened laterOpened = openText(asFormat(later, 25));
    EXPECT_TRUE(laterOpened.report.skipped.empty()) << joined(laterOpened.report.skipped);
}

// A face to go up to refers to something only while the extent goes up to a face: one kept beside
// another extent (a file keeps it, and a change of extent leaves it) is not judged, says nothing,
// and stays as it was (the review of Task 11, finding 2). One in use is judged as any reference.
TEST(Migration, AFaceToGoUpToIsJudgedOnlyWhileItIsGoneUpTo) {
    Document doc;
    doc.setType(DocumentType::Part);
    auto base = square(10.0);
    doc.addSketch(base);
    auto old = std::make_unique<ExtrudeFeature>(base, Vec3(0, 0, 1), 5.0);
    old->setNaming(NamingScheme::Positional);
    const std::string top = old->featureID() + "/cap_top";
    const std::string bottom = old->featureID() + "/cap_bottom";
    doc.featureTree().addFeature(std::move(old));

    auto blindSketch = square(2.0);
    doc.addSketch(blindSketch);
    auto blind = std::make_unique<ExtrudeFeature>(blindSketch, Vec3(0, 0, 1), 2.0);
    blind->setUpToFace(top);  // to the distance: its face to go up to is kept, unused
    doc.featureTree().addFeature(std::move(blind));
    auto box = PrimitiveFeature::makeBox(4, 4, 4);
    const std::string boxTop = box->featureID() + "/top";
    doc.featureTree().addFeature(std::move(box));
    auto blindHole = HoleFeature::make(boxTop, Vec3(2, 2, 4), 1.0, 2.0);
    blindHole->setReference("upToFace", bottom);  // blind: kept, unused
    doc.featureTree().addFeature(std::move(blindHole));

    auto upToSketch = square(2.0);
    doc.addSketch(upToSketch);
    auto upTo = std::make_unique<ExtrudeFeature>(upToSketch, Vec3(0, 0, 1), 2.0);
    upTo->setExtent(ExtrudeFeature::Extent::UpToFace);
    upTo->setUpToFace(top);
    doc.featureTree().addFeature(std::move(upTo));
    auto upToHole = HoleFeature::make(boxTop, Vec3(1, 1, 4), 1.0, 2.0);
    ASSERT_TRUE(upToHole->setParameter("extent", 2));  // up to a face
    upToHole->setReference("upToFace", bottom);
    doc.featureTree().addFeature(std::move(upToHole));

    Opened opened = openText(asFormat(doc, 31));
    const std::vector<std::string> expected = {
        notCarried("feature 5 (Extrude)", top, "the face it goes up to", "named the old way"),
        notCarried("feature 6 (Hole)", bottom, "the face it goes up to", "named the old way")};
    EXPECT_EQ(opened.report.skipped, expected) << joined(opened.report.skipped);
    const FeatureTree& tree = opened.doc->featureTree();
    const auto* keptExtrude = dynamic_cast<const ExtrudeFeature*>(tree.feature(1));
    ASSERT_NE(keptExtrude, nullptr);
    EXPECT_EQ(keptExtrude->upToFace(), top) << "as it was";
    const auto* keptHole = dynamic_cast<const HoleFeature*>(tree.feature(3));
    ASSERT_NE(keptHole, nullptr);
    EXPECT_EQ(keptHole->face(), boxTop);
    EXPECT_EQ(keptHole->upToFace(), bottom) << "as it was";
    const auto* droppedExtrude = dynamic_cast<const ExtrudeFeature*>(tree.feature(4));
    ASSERT_NE(droppedExtrude, nullptr);
    EXPECT_TRUE(droppedExtrude->upToFace().empty());
    const auto* droppedHole = dynamic_cast<const HoleFeature*>(tree.feature(5));
    ASSERT_NE(droppedHole, nullptr);
    EXPECT_TRUE(droppedHole->upToFace().empty());
    EXPECT_EQ(droppedHole->face(), boxTop);
}

// Each converted feature drawn with fewer than 12 chords a turn is said to be round now, in words
// for what it is; one drawn finer, a box and a polygon say nothing.
TEST(Migration, ACoarseFeatureIsSaidToBeRoundNow) {
    Document doc;
    doc.setType(DocumentType::Part);
    auto sphere = PrimitiveFeature::makeSphere(3.0);
    ASSERT_TRUE(sphere->setParameter("segments", 8));
    doc.featureTree().addFeature(std::move(sphere));
    auto fine = PrimitiveFeature::makeCylinder(1.0, 1.0);
    ASSERT_TRUE(fine->setParameter("segments", 12));
    doc.featureTree().addFeature(std::move(fine));
    doc.featureTree().addFeature(PrimitiveFeature::makeBox(1, 1, 1));

    auto disc = std::make_shared<Sketch>();
    disc->addEntity(std::make_shared<hz::draft::DraftCircle>(Vec2(0, 0), 2.0));
    doc.addSketch(disc);
    auto rod = std::make_unique<ExtrudeFeature>(disc, Vec3(0, 0, 1), 3.0);
    ASSERT_TRUE(rod->setParameter("segments", 6));
    doc.featureTree().addFeature(std::move(rod));
    auto d = dShape();
    doc.addSketch(d);
    auto slab = std::make_unique<ExtrudeFeature>(d, Vec3(0, 0, 1), 3.0);
    ASSERT_TRUE(slab->setParameter("segments", 8));
    doc.featureTree().addFeature(std::move(slab));
    auto plain = square(3.0);
    doc.addSketch(plain);
    doc.featureTree().addFeature(std::make_unique<ExtrudeFeature>(plain, Vec3(0, 0, 1), 1.0));

    auto ring = lines({{2, 0}, {4, 0}, {4, 3}, {2, 3}});
    doc.addSketch(ring);
    auto turned = std::make_unique<RevolveFeature>(ring, Vec3(0, 0, 0), Vec3(0, 1, 0), 2.0 * kPi);
    ASSERT_TRUE(turned->setParameter("segments", 10));
    doc.featureTree().addFeature(std::move(turned));
    auto drilled = HoleFeature::make("primitive_1/top", Vec3(0, 0, 0), 1.0, 1.0);
    ASSERT_TRUE(drilled->setParameter("segments", 6));
    doc.featureTree().addFeature(std::move(drilled));
    auto bent = lines({{0, 0}, {50, 0}, {50, 30}}, /*closed=*/false);
    doc.addSketch(bent);
    auto sheet = std::make_unique<SheetMetalFeature>(bent, 2.0, 1.0, 20.0);
    ASSERT_TRUE(sheet->setParameter("bendSegments", 2));
    doc.featureTree().addFeature(std::move(sheet));

    Opened opened = openText(asFormat(doc, 31));
    const std::vector<std::string> expected = {
        "feature 1 (Sphere): drawn with 8 sides a turn before; round now",
        "feature 4 (Extrude): drawn as a 6-sided prism before; round now",
        "feature 5 (Extrude): its arcs were drawn with 8 chords a turn before; round now",
        "feature 7 (Revolve): turned in 10 steps a turn before; round now",
        "feature 8 (Hole): drawn as a 6-sided hole before; round now",
        "feature 9 (Base Flange): its bends were drawn with 2 chords each before; round now"};
    EXPECT_EQ(opened.report.approximated, expected) << joined(opened.report.approximated);
}

// Task 12: a migrated part is marked so (Document::migrated), and its first exact build that fails
// (a fillet on an edge its migration kept as it was, which the exact build does not have) is left
// out of it, "after the exact rebuild: <why>", once: a cancelled build is no first build, and a
// later one says nothing more. A copy of it, as a rebuild on a worker makes one (format 32), is not
// migrated, so its build applied to the part is the part's; a part of format 32 that fails is not
// read in part.
TEST(Migration, AMigratedPartsFirstBuildThatFailsIsLeftOutOnce) {
    Document doc;
    doc.setType(DocumentType::Part);
    auto sketch = square(10.0);
    doc.addSketch(sketch);
    auto extrude = std::make_unique<ExtrudeFeature>(sketch, Vec3(0, 0, 1), 5.0);
    const std::string gone = extrude->featureID() + "/edge:cap_top|side:e0";
    doc.featureTree().addFeature(std::move(extrude));
    doc.featureTree().addFeature(
        std::make_unique<FilletFeature>(std::vector<TopologyID>{TopologyID::fromTag(gone)}, 1.0));

    Opened opened = openText(asFormat(doc, 31));
    Document& part = *opened.doc;
    EXPECT_TRUE(opened.report.empty()) << joined(opened.report.skipped);
    EXPECT_TRUE(part.migrated());
    EXPECT_TRUE(part.firstBuildPending());
    EXPECT_FALSE(part.readInPart()) << "nothing left out until it is built";

    Document copy;
    std::string error;
    ASSERT_TRUE(
        NativeFormat::documentFromJson(NativeFormat::documentToJson(part, false), copy, &error))
        << error;
    EXPECT_FALSE(copy.migrated()) << "a copy is of format 32";
    EXPECT_FALSE(copy.firstBuildPending());

    BuildResult cancelled;
    cancelled.cancelled = true;
    part.applyBuild(std::move(cancelled));
    EXPECT_TRUE(part.firstBuildPending()) << "a cancelled build is none";

    EXPECT_FALSE(part.applyBuild(copy.buildWithDiagnostics()));
    EXPECT_FALSE(part.firstBuildPending());
    ASSERT_EQ(part.failedFeatureIndex(), 1) << part.lastBuildMessage();
    const std::string failure = "after the exact rebuild: " + part.lastBuildMessage();
    EXPECT_EQ(part.leftOut(), std::vector<std::string>{failure});
    EXPECT_NE(failure.find(gone), std::string::npos) << failure;
    EXPECT_TRUE(part.readInPart());

    part.featureTree().markChanged();
    EXPECT_FALSE(part.rebuildModel());
    EXPECT_EQ(part.leftOut(), std::vector<std::string>{failure}) << "said once";

    // Of format 32: its build fails as any does, and nothing is left out.
    Opened current = openText(asFormat(doc, 32));
    EXPECT_FALSE(current.doc->migrated());
    EXPECT_FALSE(current.doc->rebuildModel());
    EXPECT_FALSE(current.doc->readInPart());

    // A part read whole, and built whole, says nothing.
    Document plain;
    plain.setType(DocumentType::Part);
    auto block = square(4.0);
    plain.addSketch(block);
    plain.featureTree().addFeature(std::make_unique<ExtrudeFeature>(block, Vec3(0, 0, 1), 2.0));
    Opened builds = openText(asFormat(plain, 31));
    EXPECT_TRUE(builds.doc->rebuildModel()) << builds.doc->lastBuildMessage();
    EXPECT_FALSE(builds.doc->readInPart());
}
