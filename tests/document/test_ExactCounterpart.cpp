// The part's exact counterpart (stage 2): while only imported bodies have made a part, its build
// carries those bodies as read, exact, beside the faceted copy every feature works on, named as
// the facets are, and meshed once for the display. Any other feature drops it; so does an exact
// form that cannot be meshed or measured, or that disagrees with its facets, each with why.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "StepTestText.h"
#include "horizon/document/Document.h"
#include "horizon/document/DocumentManager.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/document/ModelCommands.h"
#include "horizon/document/Sketch.h"
#include "horizon/document/UndoStack.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/fileio/NativeFormat.h"
#include "horizon/fileio/StepFormat.h"
#include "horizon/geometry/Frame.h"
#include "horizon/geometry/MeshData.h"
#include "horizon/geometry/surfaces/Surface.h"
#include "horizon/math/Constants.h"
#include "horizon/math/Units.h"
#include "horizon/modeling/ExactPrimitives.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/modeling/Naming.h"
#include "horizon/modeling/PrimitiveFactory.h"
#include "horizon/modeling/SolidTessellator.h"
#include "horizon/topology/ExactTolerance.h"
#include "horizon/topology/ExactValidator.h"
#include "horizon/topology/HalfEdge.h"
#include "horizon/topology/Solid.h"

using hz::doc::BodyOperation;
using hz::doc::Document;
using hz::doc::DocumentType;
using hz::doc::ImportedBodyFeature;
using hz::math::Vec2;
using hz::math::Vec3;

namespace {

constexpr double kPi = hz::math::kPi;

/// The solids @p text holds, read as File > Import > STEP reads them, in millimetres.
std::unique_ptr<hz::topo::Solid> read(const std::string& text) {
    hz::io::StepReadOptions inMillimetres;
    inMillimetres.unknownLengthUnit = hz::math::LengthUnit::Millimetre;
    auto solids = hz::io::StepFormat::fromString(text, nullptr, nullptr, inMillimetres);
    EXPECT_EQ(solids.size(), 1u) << hz::io::StepFormat::lastError();
    return solids.empty() ? nullptr : std::move(solids.front());
}

std::unique_ptr<ImportedBodyFeature> imported(std::unique_ptr<hz::topo::Solid> body,
                                              const std::string& source = "part.step") {
    return std::make_unique<ImportedBodyFeature>(
        std::shared_ptr<const hz::topo::Solid>(std::move(body)), source);
}

/// @p part made a part of @p body alone, imported.
void importInto(Document& part, std::unique_ptr<hz::topo::Solid> body) {
    part.setType(DocumentType::Part);
    part.featureTree().addFeature(imported(std::move(body)));
}

/// A cut of the rectangle from (x0, y0) to (x1, y1) on the XY plane, @p height up.
std::unique_ptr<hz::doc::ExtrudeFeature> cut(double x0, double y0, double x1, double y1,
                                             double height) {
    auto sketch = std::make_shared<hz::doc::Sketch>();
    const auto line = [&sketch](Vec2 a, Vec2 b) {
        sketch->addEntity(std::make_shared<hz::draft::DraftLine>(a, b));
    };
    line({x0, y0}, {x1, y0});
    line({x1, y0}, {x1, y1});
    line({x1, y1}, {x0, y1});
    line({x0, y1}, {x0, y0});
    auto feature = std::make_unique<hz::doc::ExtrudeFeature>(sketch, Vec3(0, 0, 1), height);
    feature->setOperation(BodyOperation::Cut);
    return feature;
}

double volumeOf(const hz::topo::Solid& solid) {
    return hz::model::MassPropertiesCalculator::compute(solid).volume;
}

std::set<std::string> faceNames(const hz::topo::Solid& solid) {
    std::set<std::string> names;
    for (const auto& face : solid.faces()) names.insert(face.topoId.tag());
    return names;
}

/// The logical faces of @p solid's faces (model::logicalFace): a curved face's facets, its one.
std::set<std::string> logicalFaces(const hz::topo::Solid& solid) {
    std::set<std::string> names;
    for (const auto& face : solid.faces()) names.insert(hz::model::logicalFace(face.topoId.tag()));
    return names;
}

/// The logical edges of @p solid's edges that part two logical faces (model::logicalEdge): a
/// curve's chords, or its pieces, its one; a seam, or a line between two facets of one face, none.
std::set<std::string> logicalEdges(const hz::topo::Solid& solid) {
    std::set<std::string> names;
    for (const auto& edge : solid.edges()) {
        const hz::topo::HalfEdge* he = edge.halfEdge;
        if (he == nullptr || he->twin == nullptr || he->face == nullptr ||
            he->twin->face == nullptr) {
            continue;
        }
        if (hz::model::logicalFace(he->face->topoId.tag()) ==
            hz::model::logicalFace(he->twin->face->topoId.tag())) {
            continue;
        }
        names.insert(hz::model::logicalEdge(edge.topoId.tag()));
    }
    return names;
}

/// The counterpart named as the faceted part is: the same logical faces and the same logical
/// edges between them, so a pick on either resolves in the other; and every face exact.
void expectNamedAsItsFacets(const hz::topo::Solid& exact, const hz::topo::Solid& faceted) {
    EXPECT_EQ(faceNames(exact), logicalFaces(faceted));
    EXPECT_EQ(logicalEdges(exact), logicalEdges(faceted));
    for (const auto& face : exact.faces()) {
        EXPECT_NE(face.exactSurface, nullptr) << face.topoId.tag();
    }
}

/// The counterpart's kept mesh, as the display draws it, covers each of its faces, one entry of
/// faceTags a face, and encloses its exact volume within the mesh's own error, its area times its
/// tolerance.
void expectMeshed(const Document& part, double volume) {
    ASSERT_NE(part.exactMesh(), nullptr);
    const hz::geo::MeshData& mesh = *part.exactMesh();
    ASSERT_TRUE(mesh.hasFaces());
    EXPECT_EQ(mesh.faceTags.size(), part.exactSolid()->faceCount());
    EXPECT_EQ(std::set<std::string>(mesh.faceTags.begin(), mesh.faceTags.end()),
              faceNames(*part.exactSolid()));
    double vol6 = 0.0;
    double area = 0.0;
    const auto corner = [&mesh](std::size_t i) {
        return Vec3(mesh.positions[3 * i], mesh.positions[3 * i + 1], mesh.positions[3 * i + 2]);
    };
    for (std::size_t t = 0; t + 2 < mesh.indices.size(); t += 3) {
        const Vec3 a = corner(mesh.indices[t]);
        const Vec3 b = corner(mesh.indices[t + 1]);
        const Vec3 c = corner(mesh.indices[t + 2]);
        vol6 += a.dot(b.cross(c));
        area += 0.5 * (b - a).cross(c - a).length();
    }
    EXPECT_NEAR(vol6 / 6.0, volume,
                area * hz::model::SolidTessellator::displayTolerance(*part.exactSolid()));
}

/// @p faces of @p solid, a closed shell of it, turned inside out as an exact solid can be and still
/// hold together: each loop run the other way (each half-edge from its old end, its old prev
/// after it, along its pcurve backwards; each vertex left by the half-edge that came into it;
/// each edge kept running as its curve does, by its old twin) and each face's sense turned
/// with it.
void turnInsideOut(hz::topo::Solid& solid, const std::set<hz::topo::Face*>& faces) {
    struct Was {
        hz::topo::Vertex* origin;
        hz::topo::HalfEdge* next;
        hz::topo::HalfEdge* prev;
        double start;
        double end;
    };
    std::map<hz::topo::HalfEdge*, Was> was;
    for (hz::topo::Face* face : faces) {
        std::vector<hz::topo::Wire*> loops{face->outerLoop};
        loops.insert(loops.end(), face->innerLoops.begin(), face->innerLoops.end());
        for (hz::topo::Wire* loop : loops) {
            hz::topo::HalfEdge* he = loop->halfEdge;
            do {
                was[he] = {he->origin, he->next, he->prev, he->pcurveStart, he->pcurveEnd};
                he = he->next;
            } while (he != loop->halfEdge);
        }
        face->reversed = !face->reversed;
    }
    for (auto& vertex : solid.vertices()) {
        if (const auto it = was.find(vertex.halfEdge); it != was.end()) {
            vertex.halfEdge = it->second.prev;
        }
    }
    for (auto& edge : solid.edges()) {
        if (was.count(edge.halfEdge) != 0) edge.halfEdge = edge.halfEdge->twin;
    }
    for (auto& [he, old] : was) {
        he->origin = was.at(old.next).origin;
        he->next = old.prev;
        he->prev = old.next;
        he->pcurveStart = old.end;
        he->pcurveEnd = old.start;
    }
}

/// An import that sets its build's cancel as it is read for the @p at-th time: a cancel that comes
/// at a known point of a build, in any build.
class Cancelling : public ImportedBodyFeature {
public:
    Cancelling(std::unique_ptr<hz::topo::Solid> body, hz::doc::BuildControl& control, int at)
        : ImportedBodyFeature(std::shared_ptr<const hz::topo::Solid>(std::move(body)), "part.step"),
          m_control(control),
          m_at(at) {}

    std::unique_ptr<hz::topo::Solid> execute(std::unique_ptr<hz::topo::Solid> input,
                                             std::string* reason) const override {
        if (++m_reads == m_at) m_control.cancel = true;
        return ImportedBodyFeature::execute(std::move(input), reason);
    }

    int reads() const { return m_reads; }

private:
    hz::doc::BuildControl& m_control;
    int m_at;
    mutable int m_reads = 0;
};

}  // namespace

// An imported cylinder's part has its exact counterpart: three whole faces on their surfaces,
// named as the facets of the part's faceted copy are, which stays as it was.
TEST(ExactCounterpart, AnImportedBodyHasOne) {
    Document part;
    importInto(part, read(hz::step_text::externalStyleCylinder()));
    ASSERT_TRUE(part.rebuildModel()) << part.lastBuildMessage();
    ASSERT_NE(part.solid(), nullptr);
    ASSERT_NE(part.exactSolid(), nullptr) << part.exactNote();
    EXPECT_TRUE(part.exactNote().empty()) << part.exactNote();
    EXPECT_EQ(part.exactSolid()->faceCount(), 3u);
    expectNamedAsItsFacets(*part.exactSolid(), *part.solid());
    EXPECT_NEAR(volumeOf(*part.exactSolid()), 2.0 * kPi, 2.0 * kPi * 1e-9);
    expectMeshed(part, 2.0 * kPi);

    // The faceted copy is the part the plain build gives.
    const auto built = part.featureTree().build();
    ASSERT_NE(built, nullptr);
    EXPECT_EQ(part.solid()->faceCount(), built->faceCount());
    EXPECT_EQ(faceNames(*part.solid()), faceNames(*built));
    EXPECT_EQ(volumeOf(*part.solid()), volumeOf(*built));

    // A band with no seam, its top rim's vertex two radians round: the seam inserted as it is
    // read splits the top rim in two. Each is a chord of the one curve, as its facets' are, and
    // model::logicalEdge names the curve from either.
    std::string band = hz::step_text::bandCylinder();
    const std::string top = "#12 = CARTESIAN_POINT('',(1.,0.,2.));";
    ASSERT_NE(band.find(top), std::string::npos);
    band.replace(band.find(top), top.size(),
                 "#12 = CARTESIAN_POINT('',(" + hz::step_text::real(std::cos(2.0)) + "," +
                     hz::step_text::real(std::sin(2.0)) + ",2.));");
    Document seamed;
    importInto(seamed, read(band));
    ASSERT_TRUE(seamed.rebuildModel()) << seamed.lastBuildMessage();
    ASSERT_NE(seamed.exactSolid(), nullptr) << seamed.exactNote();
    expectNamedAsItsFacets(*seamed.exactSolid(), *seamed.solid());
    std::map<std::string, std::set<std::string>> pieces;
    for (const auto& edge : seamed.exactSolid()->edges()) {
        const std::string& name = edge.topoId.tag();
        pieces[hz::model::logicalEdge(name)].insert(name);
    }
    const auto split = std::find_if(pieces.begin(), pieces.end(),
                                    [](const auto& curve) { return curve.second.size() == 2; });
    ASSERT_NE(split, pieces.end()) << "the top rim in two";
    EXPECT_EQ(split->second,
              (std::set<std::string>{split->first + "/chord:0", split->first + "/chord:1"}));
    EXPECT_NEAR(volumeOf(*seamed.exactSolid()), 2.0 * kPi, 2.0 * kPi * 1e-9);
}

// Fix round 1 (review): a part of flat faces whose corner stands off its faces' planes, by what
// its reader accepted as a gap, keeps its counterpart. Its facets' polygons stand off the exact
// planes by that much, and the bound holds it. A 4 mm block and a 4 inch one, each with a corner
// 1e-7 and 1e-5 (in its unit) high.
TEST(ExactCounterpart, AFlatPartWithAGapItsReaderAcceptedKeepsIt) {
    const std::string corner = "CARTESIAN_POINT('',(4.,4.,4.))";
    for (const auto unit : {hz::math::LengthUnit::Millimetre, hz::math::LengthUnit::Inch}) {
        for (const std::string raised : {"4.0000001", "4.00001"}) {
            std::string text = hz::step_text::block(false);
            ASSERT_NE(text.find(corner), std::string::npos);
            text.replace(text.find(corner), corner.size(),
                         "CARTESIAN_POINT('',(4.,4.," + raised + "))");
            hz::io::StepReadOptions options;
            options.unknownLengthUnit = unit;
            auto solids = hz::io::StepFormat::fromString(text, nullptr, nullptr, options);
            ASSERT_EQ(solids.size(), 1u) << hz::io::StepFormat::lastError();
            Document part;
            importInto(part, std::move(solids.front()));
            ASSERT_TRUE(part.rebuildModel()) << part.lastBuildMessage();
            EXPECT_NE(part.exactSolid(), nullptr)
                << raised << " " << hz::math::symbolOf(unit) << ": " << part.exactNote();
            EXPECT_TRUE(part.exactNote().empty()) << part.exactNote();
        }
    }
}

// Review focus 5: a cut after the import runs on the faceted copy exactly as on master, and the
// part has no exact counterpart. The faceted result's figures are master's (acf5bde9). So does an
// import joined to the part (a Boolean); a datum changes nothing.
TEST(ExactCounterpart, AFeatureAfterItDropsIt) {
    Document part;
    part.setType(DocumentType::Part);
    auto cylinder = imported(read(hz::step_text::externalStyleCylinder()), "cylinder.step");
    cylinder->restoreFeatureID("imported_1");
    part.featureTree().addFeature(std::move(cylinder));
    auto rectangle = cut(0, -2, 2, 2, 1.0);
    rectangle->restoreFeatureID("extrude_1");
    part.featureTree().addFeature(std::move(rectangle));
    ASSERT_TRUE(part.rebuildModel()) << part.lastBuildMessage();
    EXPECT_EQ(part.exactSolid(), nullptr);
    EXPECT_FALSE(part.exactMesh());
    EXPECT_TRUE(part.exactNote().empty()) << "no counterpart, by design: " << part.exactNote();

    ASSERT_NE(part.solid(), nullptr);
    EXPECT_EQ(part.solid()->faceCount(), 36u);
    EXPECT_EQ(part.solid()->edges().size(), 100u);
    EXPECT_NEAR(volumeOf(*part.solid()), 4.6821677283800227, 4.68 * 1e-12);
    std::set<std::string> imports;
    for (int k = 0; k < 32; ++k) imports.insert("imported_1/face:0/facet:" + std::to_string(k));
    imports.insert("imported_1/face:1");
    imports.insert("imported_1/face:2");
    std::set<std::string> names = faceNames(*part.solid());
    for (const auto& name : imports) EXPECT_EQ(names.erase(name), 1u) << name;
    EXPECT_EQ(names.size(), 2u) << "the cut's floor and wall";
    for (const auto& name : names) EXPECT_EQ(name.rfind("extrude_1/", 0), 0u) << name;
    const auto built = part.featureTree().build();
    ASSERT_NE(built, nullptr);
    EXPECT_EQ(faceNames(*part.solid()), faceNames(*built));

    // An import joined to the part is a Boolean on the facets: no counterpart.
    Document joined;
    importInto(joined, read(hz::step_text::externalStyleCylinder()));
    auto again = imported(read(hz::step_text::cClip()));
    again->setOperation(BodyOperation::Join);
    joined.featureTree().addFeature(std::move(again));
    ASSERT_TRUE(joined.rebuildModel()) << joined.lastBuildMessage();
    EXPECT_EQ(joined.exactSolid(), nullptr);
    EXPECT_TRUE(joined.exactNote().empty()) << joined.exactNote();

    // A datum is no change to the part.
    Document datum;
    importInto(datum, read(hz::step_text::externalStyleCylinder()));
    datum.featureTree().addFeature(hz::doc::DatumFeature::makePlane(
        hz::model::DatumPlane{Vec3(0, 0, 5), Vec3(0, 0, 1), Vec3(1, 0, 0)}));
    ASSERT_TRUE(datum.rebuildModel()) << datum.lastBuildMessage();
    EXPECT_NE(datum.exactSolid(), nullptr) << datum.exactNote();
}

// Two bodies imported as new bodies are both exact in the counterpart, each named for its own
// feature. One is a C-shaped clip whose corners wind against its faces: it is gathered facing
// out, as its faces say, so the two measure their sum.
TEST(ExactCounterpart, TwoImportedBodiesAreBothExact) {
    Document part;
    importInto(part, read(hz::step_text::externalStyleCylinder()));
    part.featureTree().addFeature(
        imported(read(hz::step_text::cClip(Vec3(10, 0, 0))), "clip.step"));
    ASSERT_TRUE(part.rebuildModel()) << part.lastBuildMessage();
    ASSERT_NE(part.exactSolid(), nullptr) << part.exactNote();
    EXPECT_EQ(part.exactSolid()->faceCount(), 9u);
    EXPECT_EQ(part.exactSolid()->shellCount(), 2u);
    expectNamedAsItsFacets(*part.exactSolid(), *part.solid());
    std::set<std::string> features;
    for (const auto& name : faceNames(*part.exactSolid())) {
        features.insert(name.substr(0, name.find('/')));
    }
    EXPECT_EQ(features.size(), 2u);
    EXPECT_NEAR(volumeOf(*part.exactSolid()), 4.5 * kPi, 4.5 * kPi * 1e-9);
    expectMeshed(part, 4.5 * kPi);
}

// A face the read point cannot mesh (its side's half-edge with no pcurve to lay it out by) drops
// the counterpart, the note naming the face and why. The faceted copy, which needs no pcurve, is
// the part as before.
TEST(ExactCounterpart, OneThatCannotBeMeshedIsDroppedWithANote) {
    auto body = read(hz::step_text::externalStyleCylinder());
    ASSERT_NE(body, nullptr);
    for (auto& face : body->faces()) {
        if (face.exactSurface->kind() != hz::geo::SurfaceKind::Cylinder) continue;
        face.outerLoop->halfEdge->pcurve = nullptr;
    }
    Document part;
    importInto(part, std::move(body));
    ASSERT_TRUE(part.rebuildModel()) << part.lastBuildMessage();
    EXPECT_EQ(part.exactSolid(), nullptr);
    EXPECT_FALSE(part.exactMesh());
    const std::string feature = part.featureTree().feature(0)->featureID();
    const std::string said = "shown from facets: " + feature + "/face:";
    EXPECT_EQ(part.exactNote().rfind(said, 0), 0u) << part.exactNote();
    ASSERT_NE(part.solid(), nullptr);
    EXPECT_EQ(logicalFaces(*part.solid()).size(), 3u);
    EXPECT_NEAR(volumeOf(*part.solid()), 2.0 * kPi, 2.0 * kPi * 1e-2);
}

// Ruling 46. A face turned inside out by its flag alone is refused before it is compared: its
// loop runs against its sense, and its exact mass says so. A body turned inside out whole holds
// together, validates clean and meshes, but measures wrong. A cylinder so turned, imported before
// a larger clip: the facets are gathered facing out, as their corners wind, and the exact form as
// its faces face, enclosing the clip less the cylinder; the two disagree, and the counterpart is
// dropped with both figures. (Re-baselined for M6: the clip was the body turned, which left the
// part's exact form facing inward, now said as such; the figures are the same.)
TEST(ExactCounterpart, AnExactFormThatDisagreesWithItsFacetsIsDropped) {
    auto flipped = read(hz::step_text::externalStyleCylinder());
    ASSERT_NE(flipped, nullptr);
    for (auto& face : flipped->faces()) {
        if (face.exactSurface->kind() == hz::geo::SurfaceKind::Cylinder) {
            face.reversed = !face.reversed;
        }
    }
    Document one;
    importInto(one, std::move(flipped));
    ASSERT_TRUE(one.rebuildModel()) << one.lastBuildMessage();
    EXPECT_EQ(one.exactSolid(), nullptr);
    EXPECT_EQ(one.exactNote().rfind("shown from facets: ", 0), 0u) << one.exactNote();
    EXPECT_NE(one.exactNote().find("runs against its sense"), std::string::npos) << one.exactNote();

    auto cylinder = read(hz::step_text::externalStyleCylinder());
    ASSERT_NE(cylinder, nullptr);
    std::set<hz::topo::Face*> faces;
    for (auto& face : cylinder->faces()) faces.insert(&face);
    turnInsideOut(*cylinder, faces);
    ASSERT_TRUE(hz::topo::validateExact(*cylinder, hz::topo::exactTolerance(*cylinder)).empty());

    Document part;
    importInto(part, std::move(cylinder));
    part.featureTree().addFeature(
        imported(read(hz::step_text::cClip(Vec3(10, 0, 0))), "clip.step"));
    ASSERT_TRUE(part.rebuildModel()) << part.lastBuildMessage();
    EXPECT_EQ(part.exactSolid(), nullptr);
    EXPECT_FALSE(part.exactMesh());
    const std::string said = "exact form disagrees with its facets: ";
    ASSERT_EQ(part.exactNote().rfind(said, 0), 0u) << part.exactNote();
    // "<exact> vs <faceted>": the clip less the cylinder, against both in facets.
    const std::string figures = part.exactNote().substr(said.size());
    const auto vs = figures.find(" vs ");
    ASSERT_NE(vs, std::string::npos) << figures;
    EXPECT_NEAR(std::stod(figures.substr(0, vs)), 0.5 * kPi, 1e-6);
    ASSERT_NE(part.solid(), nullptr);
    const double faceted = volumeOf(*part.solid());
    EXPECT_NEAR(std::stod(figures.substr(vs + 4)), faceted, 1e-6 * faceted);
    EXPECT_NEAR(faceted, 4.5 * kPi, 4.5 * kPi * 1e-2);
}

// M6: a lone body turned inside out whole (each loop run the other way and each face's sense
// turned with it) holds together and validates clean, and its facets, gathered by their corners,
// face out and measure its volume: only its exact form's sign tells. A part of it alone encloses
// a negative volume, said as its own note; it is shown from facets, never inside out.
TEST(ExactCounterpart, ALoneBodyTurnedInsideOutIsSaidToFaceInward) {
    auto clip = read(hz::step_text::cClip());
    ASSERT_NE(clip, nullptr);
    std::set<hz::topo::Face*> faces;
    for (auto& face : clip->faces()) faces.insert(&face);
    turnInsideOut(*clip, faces);
    ASSERT_TRUE(hz::topo::validateExact(*clip, hz::topo::exactTolerance(*clip)).empty());

    Document part;
    importInto(part, std::move(clip));
    ASSERT_TRUE(part.rebuildModel()) << part.lastBuildMessage();
    EXPECT_EQ(part.exactSolid(), nullptr) << "shown inside out";
    EXPECT_FALSE(part.exactMesh());
    const std::string said = "shown from facets: the exact form faces inward (it encloses ";
    ASSERT_EQ(part.exactNote().rfind(said, 0), 0u) << part.exactNote();
    EXPECT_NEAR(std::stod(part.exactNote().substr(said.size())), -2.5 * kPi, 1e-6);
    ASSERT_NE(part.solid(), nullptr);
    EXPECT_NEAR(volumeOf(*part.solid()), 2.5 * kPi, 2.5 * kPi * 1e-2) << "its facets face out";
}

// Ruling 49: a part saved before stage 2 whose imported body's STEP no exact form holds opens
// with the body as it was, and no counterpart: the note is the import's.
TEST(ExactCounterpart, AnOldPartsBodyThatIsNotExactHasNone) {
    auto box = hz::model::PrimitiveFactory::makeBox(7.0, 11.0, 13.0);
    std::string text = hz::io::StepFormat::toString({box.get()});
    const std::string corner = "(7.,11.,13.)";
    ASSERT_NE(text.find(corner), std::string::npos);
    text.replace(text.find(corner), corner.size(), "(7.,11.,13.2)");
    hz::io::StepReadOptions keep;
    keep.keepWhenNotExact = true;
    auto solids = hz::io::StepFormat::fromString(text, nullptr, nullptr, keep);
    ASSERT_EQ(solids.size(), 1u) << hz::io::StepFormat::lastError();

    Document original;
    original.setType(DocumentType::Part);
    original.featureTree().addFeature(imported(std::move(solids.front()), "creo.step"));
    Document opened;
    std::string error;
    hz::io::ImportReport report;
    ASSERT_TRUE(hz::io::NativeFormat::documentFromJson(
        hz::io::NativeFormat::documentToJson(original, false), opened, &error, &report))
        << error;
    ASSERT_TRUE(opened.rebuildModel()) << opened.lastBuildMessage();
    ASSERT_NE(opened.solid(), nullptr);
    EXPECT_EQ(opened.exactSolid(), nullptr);
    EXPECT_FALSE(opened.exactMesh());
    ASSERT_EQ(report.converted.size(), 1u);
    EXPECT_EQ(opened.exactNote(), "shown from facets: " + report.converted.front());
    const auto* body = dynamic_cast<const ImportedBodyFeature*>(opened.featureTree().feature(0));
    ASSERT_NE(body, nullptr);
    EXPECT_EQ(body->exactBody(), nullptr);
}

// The counterpart follows the history through undo and redo: an undo of the feature that dropped
// it brings it back, a redo drops it again. setSolid() clears it.
TEST(ExactCounterpart, AnUndoAfterAFeatureBringsItBack) {
    Document part;
    importInto(part, read(hz::step_text::externalStyleCylinder()));
    ASSERT_TRUE(part.rebuildModel()) << part.lastBuildMessage();
    ASSERT_NE(part.exactSolid(), nullptr) << part.exactNote();
    const auto names = faceNames(*part.exactSolid());

    part.undoStack().push(
        std::make_unique<hz::doc::AddFeatureCommand>(part, cut(0, -2, 2, 2, 1.0)));
    ASSERT_TRUE(part.needsBuild());
    ASSERT_TRUE(part.rebuildModel()) << part.lastBuildMessage();
    EXPECT_EQ(part.exactSolid(), nullptr);

    part.undoStack().undo();
    ASSERT_TRUE(part.needsBuild());
    ASSERT_TRUE(part.rebuildModel()) << part.lastBuildMessage();
    ASSERT_NE(part.exactSolid(), nullptr) << part.exactNote();
    EXPECT_EQ(faceNames(*part.exactSolid()), names);
    EXPECT_NE(part.exactMesh(), nullptr);

    part.undoStack().redo();
    ASSERT_TRUE(part.rebuildModel()) << part.lastBuildMessage();
    EXPECT_EQ(part.exactSolid(), nullptr);
    EXPECT_FALSE(part.exactMesh());

    part.undoStack().undo();
    ASSERT_TRUE(part.rebuildModel()) << part.lastBuildMessage();
    ASSERT_NE(part.exactSolid(), nullptr) << part.exactNote();
    part.setSolid(hz::model::PrimitiveFactory::makeBox(1.0, 1.0, 1.0));
    EXPECT_EQ(part.exactSolid(), nullptr);
    EXPECT_FALSE(part.exactMesh());
    EXPECT_TRUE(part.exactNote().empty());
}

// Ruling 51: the counterpart is made once, and reused while the imports it was made from are as
// they were. A rebuild for anything else (a datum added, an edit marked) gives the same object;
// so does an undo of a feature that dropped it. A changed import makes it again.
TEST(ExactCounterpart, ARebuildWithItsImportsUnchangedReusesIt) {
    Document part;
    importInto(part, read(hz::step_text::externalStyleCylinder()));
    part.featureTree().addFeature(imported(read(hz::step_text::cClip(Vec3(10, 0, 0))), "clip"));
    ASSERT_TRUE(part.rebuildModel()) << part.lastBuildMessage();
    const hz::topo::Solid* made = part.exactSolid();
    ASSERT_NE(made, nullptr) << part.exactNote();
    const auto* mesh = part.exactMesh().get();
    ASSERT_NE(mesh, nullptr);

    part.featureTree().markChanged();
    ASSERT_TRUE(part.rebuildModel()) << part.lastBuildMessage();
    EXPECT_EQ(part.exactSolid(), made) << "made again for no change to its imports";
    EXPECT_EQ(part.exactMesh().get(), mesh);

    part.featureTree().addFeature(hz::doc::DatumFeature::makePlane(
        hz::model::DatumPlane{Vec3(0, 0, 5), Vec3(0, 0, 1), Vec3(1, 0, 0)}));
    ASSERT_TRUE(part.rebuildModel()) << part.lastBuildMessage();
    EXPECT_EQ(part.exactSolid(), made);

    part.undoStack().push(
        std::make_unique<hz::doc::AddFeatureCommand>(part, cut(0, -2, 2, 2, 1.0)));
    ASSERT_TRUE(part.rebuildModel()) << part.lastBuildMessage();
    EXPECT_EQ(part.exactSolid(), nullptr);
    part.undoStack().undo();
    ASSERT_TRUE(part.rebuildModel()) << part.lastBuildMessage();
    EXPECT_EQ(part.exactSolid(), made) << "the one made before the cut, not made again";
    EXPECT_EQ(part.exactMesh().get(), mesh);

    // A worker builds a copy, its imports read again from the part's STEP text. The first such
    // read may define them a little otherwise than the import did (a frame renormalised in its
    // last bit, a scale folded in), so its build may make the counterpart again; the document
    // takes that build, and every later copy reads them as that one did, and reuses it.
    const auto workerBuild = [&part](Document& copy) {
        std::string error;
        ASSERT_TRUE(hz::io::NativeFormat::documentFromJson(
            hz::io::NativeFormat::documentToJson(part, false), copy, &error))
            << error;
        hz::doc::BuildOptions given;
        given.previous = part.exactCounterpart();
        ASSERT_TRUE(part.applyBuild(copy.buildWithDiagnostics(nullptr, given)))
            << part.lastBuildMessage();
    };
    Document firstCopy;
    workerBuild(firstCopy);
    const hz::topo::Solid* fromCopy = part.exactSolid();
    ASSERT_NE(fromCopy, nullptr) << part.exactNote();
    Document secondCopy;
    workerBuild(secondCopy);
    EXPECT_EQ(part.exactSolid(), fromCopy) << "made again for a copy of the same imports";
    std::string error;
    hz::doc::BuildOptions options;

    // Another body under the same feature's name is not taken for it.
    Document other;
    importInto(other, read(hz::step_text::block(false)));
    other.featureTree().feature(0)->restoreFeatureID(part.featureTree().feature(0)->featureID());
    Document cylinder;
    importInto(cylinder, read(hz::step_text::externalStyleCylinder()));
    cylinder.featureTree().feature(0)->restoreFeatureID(part.featureTree().feature(0)->featureID());
    ASSERT_TRUE(cylinder.rebuildModel()) << cylinder.lastBuildMessage();
    ASSERT_NE(cylinder.exactSolid(), nullptr) << cylinder.exactNote();
    options.previous = cylinder.exactCounterpart();
    ASSERT_TRUE(other.applyBuild(other.buildWithDiagnostics(nullptr, options)))
        << other.lastBuildMessage();
    ASSERT_NE(other.exactSolid(), nullptr) << other.exactNote();
    EXPECT_NE(other.exactSolid(), cylinder.exactSolid());
    EXPECT_EQ(other.exactSolid()->faceCount(), 6u);

    // Nor one with the same counts, senses and corners whose surfaces are not the same: the
    // block with two of its faces' planes swapped. Its build goes on to make one of its own,
    // and is cancelled here as it begins (a body no exact form holds is refused, but only after
    // the mesher has searched for seconds).
    Document block;
    importInto(block, read(hz::step_text::block(false)));
    ASSERT_TRUE(block.rebuildModel()) << block.lastBuildMessage();
    ASSERT_NE(block.exactSolid(), nullptr) << block.exactNote();
    auto swapped = read(hz::step_text::block(false));
    ASSERT_NE(swapped, nullptr);
    std::swap(swapped->faces()[2].exactSurface, swapped->faces()[3].exactSurface);
    hz::doc::BuildControl control;
    auto wrongBody = std::make_unique<Cancelling>(std::move(swapped), control, 1);
    wrongBody->restoreFeatureID(block.featureTree().feature(0)->featureID());
    Document wrong;
    wrong.setType(DocumentType::Part);
    wrong.featureTree().addFeature(std::move(wrongBody));
    options.previous = block.exactCounterpart();
    const hz::doc::BuildResult swappedBuild = wrong.buildWithDiagnostics(&control, options);
    EXPECT_TRUE(swappedBuild.cancelled) << "taken for the block";
    EXPECT_EQ(control.total.load(), 2) << "its counterpart's step begun";

    // Nor a B-spline cap whose net differs inside its rim, its corners the same.
    const auto cap = [](double spread) {
        return read(hz::step_text::seamedHemisphere(
            "#20,#21,#22", hz::step_text::splineSphereSurface(10.0, spread)));
    };
    Document dome;
    importInto(dome, cap(0.0));
    ASSERT_TRUE(dome.rebuildModel()) << dome.lastBuildMessage();
    ASSERT_NE(dome.exactSolid(), nullptr) << dome.exactNote();
    Document revised;
    importInto(revised, cap(1e-6));
    revised.featureTree().feature(0)->restoreFeatureID(dome.featureTree().feature(0)->featureID());
    options.previous = dome.exactCounterpart();
    ASSERT_TRUE(revised.applyBuild(revised.buildWithDiagnostics(nullptr, options)))
        << revised.lastBuildMessage();
    EXPECT_NE(revised.exactSolid(), dome.exactSolid()) << "taken for the first dome";

    // A document read in place of what it held forgets its counterpart.
    ASSERT_TRUE(hz::io::NativeFormat::documentFromJson(
        hz::io::NativeFormat::documentToJson(block, false), dome, &error))
        << error;
    EXPECT_EQ(dome.exactSolid(), nullptr);
    EXPECT_EQ(dome.exactCounterpart(), nullptr);

    // The clip suppressed: the cylinder alone, made again; and back, made again with both.
    part.featureTree().feature(1)->setSuppressed(true);
    part.featureTree().markChanged();
    ASSERT_TRUE(part.rebuildModel()) << part.lastBuildMessage();
    ASSERT_NE(part.exactSolid(), nullptr) << part.exactNote();
    EXPECT_NE(part.exactSolid(), made);
    EXPECT_EQ(part.exactSolid()->faceCount(), 3u);
    part.featureTree().feature(1)->setSuppressed(false);
    part.featureTree().markChanged();
    ASSERT_TRUE(part.rebuildModel()) << part.lastBuildMessage();
    ASSERT_NE(part.exactSolid(), nullptr) << part.exactNote();
    EXPECT_EQ(part.exactSolid()->faceCount(), 9u);
    expectNamedAsItsFacets(*part.exactSolid(), *part.solid());
}

// Ruling 51: a cancel that comes once the features are built stops the counterpart's step before
// it meshes, and the build is cancelled, with no part. The cancel comes from the last import as it
// is read, so it falls between the features and the counterpart every time, in any build: no
// race. (That meshing stops between faces is TrimmedFaceMesher.ACancelStopsItBetweenFaces.)
TEST(ExactCounterpart, ACancelOnceItsFeaturesAreBuiltStopsIt) {
    hz::doc::BuildControl control;
    Document part;
    importInto(part, read(hz::step_text::externalStyleCylinder()));
    auto last =
        std::make_unique<Cancelling>(read(hz::step_text::cClip(Vec3(10, 0, 0))), control, 1);
    const Cancelling* cancelling = last.get();
    part.featureTree().addFeature(std::move(last));
    const hz::doc::BuildResult result = part.buildWithDiagnostics(&control);
    EXPECT_EQ(cancelling->reads(), 1);
    EXPECT_TRUE(result.cancelled);
    EXPECT_EQ(result.solid.get(), nullptr) << "a cancelled build has no part";
    EXPECT_EQ(result.exact.get(), nullptr);
    EXPECT_EQ(control.total.load(), 3) << "the counterpart a step of its own, after the 2 features";
    EXPECT_EQ(control.done.load(), 3);
}

// Ruling 53: a build whose feature fails builds the part again up to it. A cancel during that
// replay leaves it cancelled, with no part, as a cancel anywhere else does.
TEST(ExactCounterpart, ACancelWhileTheFailedFeaturesPartIsReplayedLeavesNoPart) {
    hz::doc::BuildControl control;
    Document part;
    part.setType(DocumentType::Part);
    auto first =
        std::make_unique<Cancelling>(read(hz::step_text::externalStyleCylinder()), control, 2);
    const Cancelling* cancelling = first.get();
    part.featureTree().addFeature(std::move(first));
    part.featureTree().addFeature(cut(-5, -5, 5, 5, 5.0));  // the whole of it: nothing left
    const hz::doc::BuildResult result = part.buildWithDiagnostics(&control);
    ASSERT_EQ(result.failedFeatureIndex, 1) << result.failureMessage;
    EXPECT_EQ(cancelling->reads(), 2) << "read again in the replay";
    EXPECT_TRUE(result.cancelled);
    EXPECT_EQ(result.solid.get(), nullptr) << "a cancelled build has no part";
    EXPECT_EQ(result.exact.get(), nullptr);
}

// Review (Minor 5): a build for what draws the faceted part alone (an assembly's component, its
// mesh made from solid()) makes no counterpart, and pays for no mesh.
TEST(ExactCounterpart, ABuildThatAsksForNoneMakesNone) {
    Document part;
    importInto(part, read(hz::step_text::externalStyleCylinder()));
    hz::doc::BuildControl control;
    hz::doc::BuildOptions none;
    none.exactCounterpart = false;
    ASSERT_TRUE(part.applyBuild(part.buildWithDiagnostics(&control, none)))
        << part.lastBuildMessage();
    ASSERT_NE(part.solid(), nullptr);
    EXPECT_EQ(part.exactSolid(), nullptr);
    EXPECT_TRUE(part.exactNote().empty()) << part.exactNote();
    EXPECT_EQ(control.total.load(), 1) << "no step for a counterpart";
    EXPECT_EQ(part.exactCounterpart(), nullptr);
    EXPECT_TRUE(part.needsBuild()) << "to be built with it before it is shown";

    // A part with no import has no counterpart to leave out: built so, it is built.
    Document box;
    box.setType(DocumentType::Part);
    box.featureTree().addFeature(hz::doc::PrimitiveFeature::makeBox(1.0, 2.0, 3.0));
    ASSERT_TRUE(box.rebuildModel(none)) << box.lastBuildMessage();
    EXPECT_FALSE(box.needsBuild());
}

// Ruling 53: an assembly's component part is built without its counterpart, but a part the user
// has open in a tab is built with it; and one built for its components first is built again, with
// it, when a tab shows it (it needs a build).
TEST(ExactCounterpart, APartOpenInATabIsBuiltWithIt) {
    const auto loader = [](const std::string& /*path*/, Document& doc) {
        importInto(doc, read(hz::step_text::externalStyleCylinder()));
        return true;
    };
    const auto file = [](const std::string& name) {
        const std::string path = (std::filesystem::temp_directory_path() / name).string();
        std::ofstream(path) << "{}";
        return path;
    };

    // Open in a tab, not yet built, then placed: built as the tab shows it.
    hz::doc::DocumentManager first;
    first.setPartLoader(loader);
    const std::string shownPath = file("hz_exact_counterpart_tab.hzpart");
    const auto shown = first.openPart(shownPath);
    ASSERT_NE(shown, nullptr);
    hz::doc::ComponentInstance placed;
    placed.partPath = shownPath;
    ASSERT_TRUE(first.resolveComponent(placed, hz::doc::ComponentState::Resolved));
    EXPECT_EQ(placed.resolvedPart, shown);
    EXPECT_NE(shown->exactSolid(), nullptr) << shown->exactNote();
    EXPECT_FALSE(shown->needsBuild());

    // Placed first, built for its component alone; opened in a tab, it needs a build, and that
    // build has it.
    hz::doc::DocumentManager second;
    second.setPartLoader(loader);
    const std::string placedPath = file("hz_exact_counterpart_component.hzpart");
    hz::doc::ComponentInstance component;
    component.partPath = placedPath;
    ASSERT_TRUE(second.resolveComponent(component, hz::doc::ComponentState::Resolved));
    ASSERT_NE(component.resolvedPart, nullptr);
    ASSERT_NE(component.resolvedPart->solid(), nullptr);
    EXPECT_EQ(component.resolvedPart->exactSolid(), nullptr);
    const auto opened = second.openPart(placedPath);
    EXPECT_EQ(opened, component.resolvedPart);
    EXPECT_TRUE(opened->needsBuild()) << "shown as it was built for its component";
    ASSERT_TRUE(opened->rebuildModel()) << opened->lastBuildMessage();
    EXPECT_NE(opened->exactSolid(), nullptr) << opened->exactNote();
    EXPECT_FALSE(opened->needsBuild());

    std::filesystem::remove(shownPath);
    std::filesystem::remove(placedPath);
}

// The owner's display tolerance: the counterpart is meshed for the display, and so for its part
// file's cache, to 0.1 up to about 667 across, and beyond that to 1.5e-4 of its size (its
// diagonal, topo::exactExtent). A cylinder of radius 200 and height 200, written as STEP and
// imported, is 600 across; one of radius 1000 and height 2000 is 2000 sqrt 3 across, meshed to
// 0.52.
TEST(ExactCounterpart, ALargePartIsMeshedToItsSizeASmallOneToATenth) {
    struct Case {
        double radius;
        double height;
        double across;
        double tolerance;
    };
    const double across = 2000.0 * std::sqrt(3.0);
    for (const Case& c :
         {Case{200.0, 200.0, 600.0, 0.1}, Case{1000.0, 2000.0, across, 1.5e-4 * across}}) {
        SCOPED_TRACE(c.radius);
        const auto cylinder = hz::model::exact::makeCylinder(
            hz::geo::Frame::fromAxis({0, 0, 0}, {0, 0, 1}), c.radius, c.height);
        ASSERT_NE(cylinder, nullptr);
        Document part;
        importInto(part, read(hz::io::StepFormat::toString({cylinder.get()})));
        ASSERT_TRUE(part.rebuildModel()) << part.lastBuildMessage();
        ASSERT_NE(part.exactSolid(), nullptr) << part.exactNote();
        EXPECT_NEAR(hz::topo::exactExtent(*part.exactSolid()), c.across, 1e-9 * c.across);
        ASSERT_NE(part.exactMesh(), nullptr);
        EXPECT_EQ(part.exactMesh()->indices,
                  hz::model::SolidTessellator::tessellate(*part.exactSolid(), c.tolerance).indices);
        ASSERT_NE(part.exactOrFacetedMesh(), nullptr);
        EXPECT_EQ(part.exactOrFacetedMesh()->indices, part.exactMesh()->indices);
    }
}
