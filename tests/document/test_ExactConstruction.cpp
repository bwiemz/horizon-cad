// Exact construction (stage 3): a part made of primitives is exact. Each Stable primitive builds
// its body through the exact primitives, in the frame it stands in, and the faceted part every
// operation not yet converted reads is that body's bridge copy (model::facetExact at the
// feature's own resolution): master's facets, vertex for vertex, named for the exact faces and
// edges. The build carries the exact part beside it while every body is a new body with an exact
// form; any other operation drops it, and what that operation builds is master's. A primitive
// whose exact build fails fails, saying why; one named by position builds as master did.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "StepTestText.h"
#include "horizon/document/Document.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/document/Sketch.h"
#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/SketchPlane.h"
#include "horizon/fileio/StepFormat.h"
#include "horizon/geometry/MeshData.h"
#include "horizon/math/BoundingBox.h"
#include "horizon/math/Constants.h"
#include "horizon/math/Mat4.h"
#include "horizon/math/Quaternion.h"
#include "horizon/math/Units.h"
#include "horizon/modeling/Extrude.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/modeling/Naming.h"
#include "horizon/modeling/Pattern.h"
#include "horizon/modeling/PrimitiveFactory.h"
#include "horizon/modeling/Revolve.h"
#include "horizon/modeling/SolidTessellator.h"
#include "horizon/topology/HalfEdge.h"
#include "horizon/topology/Solid.h"

using hz::doc::BodyOperation;
using hz::doc::Document;
using hz::doc::DocumentType;
using hz::doc::PrimitiveFeature;
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

std::unique_ptr<hz::doc::ImportedBodyFeature> imported(std::unique_ptr<hz::topo::Solid> body,
                                                       const std::string& source) {
    return std::make_unique<hz::doc::ImportedBodyFeature>(
        std::shared_ptr<const hz::topo::Solid>(std::move(body)), source);
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

/// The logical edges of @p solid's edges that part two logical faces (model::logicalEdge).
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

/// The exact part named as the faceted part is: the same logical faces and the same logical
/// edges between them, and every face exact.
void expectNamedAsItsFacets(const hz::topo::Solid& exact, const hz::topo::Solid& faceted) {
    EXPECT_EQ(faceNames(exact), logicalFaces(faceted));
    EXPECT_EQ(logicalEdges(exact), logicalEdges(faceted));
    for (const auto& face : exact.faces()) {
        EXPECT_NE(face.exactSurface, nullptr) << face.topoId.tag();
    }
}

/// The exact part's kept mesh covers each of its faces and encloses @p volume within the mesh's
/// own error, its area times its tolerance.
void expectMeshed(const Document& part, double volume) {
    ASSERT_NE(part.exactMesh(), nullptr);
    const hz::geo::MeshData& mesh = *part.exactMesh();
    ASSERT_TRUE(mesh.hasFaces());
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

/// Master's faceted body for @p feature, as its PrimitiveFeature::execute built it: the factory's
/// solid at the feature's segment count, stood where the feature stands.
std::unique_ptr<hz::topo::Solid> mastersBody(const PrimitiveFeature& feature) {
    using hz::model::PrimitiveFactory;
    const int n = feature.segments();
    std::unique_ptr<hz::topo::Solid> solid;
    switch (feature.kind()) {
        case PrimitiveFeature::Kind::Box:
            solid = PrimitiveFactory::makeBox(feature.p0(), feature.p1(), feature.p2());
            break;
        case PrimitiveFeature::Kind::Cylinder:
            solid = PrimitiveFactory::makeCylinder(feature.p0(), feature.p1(), n);
            break;
        case PrimitiveFeature::Kind::Sphere:
            solid = PrimitiveFactory::makeSphere(feature.p0(), n);
            break;
        case PrimitiveFeature::Kind::Cone:
            solid = PrimitiveFactory::makeCone(feature.p0(), feature.p1(), feature.p2(), n);
            break;
        case PrimitiveFeature::Kind::Torus:
            solid = PrimitiveFactory::makeTorus(feature.p0(), feature.p1(), n);
            break;
    }
    if (!solid || !feature.isPlaced()) return solid;
    const Vec3 z(0.0, 0.0, 1.0);
    const Vec3 to = feature.axisDirection().normalized();
    const Vec3 turnAxis = z.cross(to);
    const double angle = std::atan2(turnAxis.length(), z.dot(to));
    const hz::math::Quaternion turn =
        turnAxis.length() > 1e-12
            ? hz::math::Quaternion::fromAxisAngle(turnAxis.normalized(), angle)
            : hz::math::Quaternion::fromAxisAngle(Vec3(1.0, 0.0, 0.0), angle);
    return hz::model::Pattern::transformed(
        *solid, hz::math::Mat4::translation(feature.basePoint()) * hz::math::Mat4::rotation(turn));
}

/// A part of @p feature alone.
void buildAlone(Document& part, std::unique_ptr<hz::doc::Feature> feature) {
    part.setType(DocumentType::Part);
    part.featureTree().addFeature(std::move(feature));
}

}  // namespace

// A cylinder of radius 5 and height 10 is three exact faces, and the mass tool measures its exact
// volume. Its faceted copy is master's: 32 facets round its side and its two caps, each named
// for the exact face it lies on.
TEST(ExactConstruction, ACylinderPrimitiveIsThreeExactFaces) {
    Document part;
    buildAlone(part, PrimitiveFeature::makeCylinder(5.0, 10.0));
    ASSERT_TRUE(part.rebuildModel()) << part.lastBuildMessage();
    ASSERT_NE(part.exactSolid(), nullptr) << part.exactNote();
    EXPECT_EQ(part.exactSolid()->faceCount(), 3u);
    const double volume = kPi * 25.0 * 10.0;
    const auto mass = hz::model::MassPropertiesCalculator::compute(*part.exactOrFacetedSolid());
    ASSERT_TRUE(mass.valid) << mass.problem;
    EXPECT_NEAR(mass.volume, volume, volume * 1e-9);

    ASSERT_NE(part.solid(), nullptr);
    EXPECT_EQ(part.solid()->faceCount(), 34u);
    EXPECT_EQ(part.solid()->edges().size(), 96u);
    EXPECT_EQ(part.solid()->vertices().size(), 64u);
    EXPECT_EQ(logicalFaces(*part.solid()), faceNames(*part.exactSolid()));
    expectNamedAsItsFacets(*part.exactSolid(), *part.solid());
    expectMeshed(part, volume);
}

// Two primitives made new bodies are both exact in the part, each named for its own feature.
TEST(ExactConstruction, TwoNewBodiesAreBothExact) {
    Document part;
    part.setType(DocumentType::Part);
    part.featureTree().addFeature(PrimitiveFeature::makeCylinder(2.0, 3.0));
    auto box = PrimitiveFeature::makeBox(1.0, 2.0, 3.0);
    ASSERT_TRUE(box->setVector("basePoint", Vec3(10.0, 0.0, 0.0)));
    part.featureTree().addFeature(std::move(box));
    ASSERT_TRUE(part.rebuildModel()) << part.lastBuildMessage();
    ASSERT_NE(part.exactSolid(), nullptr) << part.exactNote();
    EXPECT_EQ(part.exactSolid()->faceCount(), 9u);
    EXPECT_EQ(part.exactSolid()->shellCount(), 2u);
    expectNamedAsItsFacets(*part.exactSolid(), *part.solid());
    std::set<std::string> features;
    for (const auto& name : faceNames(*part.exactSolid())) {
        features.insert(name.substr(0, name.find('/')));
    }
    EXPECT_EQ(features, (std::set<std::string>{part.featureTree().feature(0)->featureID(),
                                               part.featureTree().feature(1)->featureID()}));
    const double volume = kPi * 4.0 * 3.0 + 6.0;
    EXPECT_NEAR(volumeOf(*part.exactSolid()), volume, volume * 1e-9);
    EXPECT_EQ(part.solid()->faceCount(), 34u + 6u);
    expectMeshed(part, volume);
}

// An imported cylinder and a sphere primitive, each a new body: the part is exact, the import's
// faces with their NURBS carriers, the sphere's without. Its bodies are told apart by their exact
// volumes.
TEST(ExactConstruction, AnImportAndAPrimitiveAreBothExact) {
    Document part;
    part.setType(DocumentType::Part);
    part.featureTree().addFeature(
        imported(read(hz::step_text::externalStyleCylinder()), "cylinder.step"));
    auto sphere = PrimitiveFeature::makeSphere(1.5);
    ASSERT_TRUE(sphere->setVector("basePoint", Vec3(5.0, 0.0, 0.5)));
    part.featureTree().addFeature(std::move(sphere));
    ASSERT_TRUE(part.rebuildModel()) << part.lastBuildMessage();
    const hz::topo::Solid* exact = part.exactSolid();
    ASSERT_NE(exact, nullptr) << part.exactNote();
    EXPECT_EQ(exact->faceCount(), 4u);
    EXPECT_EQ(exact->shellCount(), 2u);
    expectNamedAsItsFacets(*exact, *part.solid());
    const double volume = 2.0 * kPi + 4.5 * kPi;
    EXPECT_NEAR(volumeOf(*exact), volume, volume * 1e-9);
    expectMeshed(part, volume);

    const auto bodies = hz::model::Pattern::separate(*exact);
    ASSERT_EQ(bodies.size(), 2u);
    std::multiset<long> volumes;  // in millionths
    for (const auto& body : bodies) volumes.insert(std::lround(volumeOf(*body) * 1e6));
    EXPECT_EQ(volumes,
              (std::multiset<long>{std::lround(2.0 * kPi * 1e6), std::lround(4.5 * kPi * 1e6)}));
}

// A part mixing an import that encloses a void (a block with a ball out of it) and a primitive
// sphere: the void, one face bounded by its seam, has corners that span nothing, and is told from a
// body by its exact signed volume, so the part is two bodies, the block with its void and the
// sphere.
TEST(ExactConstruction, AMixedPartsVoidStaysWithItsBody) {
    Document part;
    part.setType(DocumentType::Part);
    part.featureTree().addFeature(imported(read(hz::step_text::blockWithABall()), "block.step"));
    auto sphere = PrimitiveFeature::makeSphere(1.5);
    ASSERT_TRUE(sphere->setVector("basePoint", Vec3(10.0, 2.0, 2.0)));
    part.featureTree().addFeature(std::move(sphere));
    ASSERT_TRUE(part.rebuildModel()) << part.lastBuildMessage();
    const hz::topo::Solid* exact = part.exactSolid();
    ASSERT_NE(exact, nullptr) << part.exactNote();
    EXPECT_EQ(exact->shellCount(), 3u);
    const auto groups = hz::model::Pattern::bodyShells(*exact);
    ASSERT_EQ(groups.size(), 2u);
    std::multiset<std::size_t> sizes;
    for (const auto& group : groups) sizes.insert(group.size());
    EXPECT_EQ(sizes, (std::multiset<std::size_t>{1u, 2u})) << "the void with the block";
}

// Review focus 5: a cut after a primitive and an import (two new bodies) drops the exact part, and
// what it builds on their faceted copies is master's: the face count, the volume and every face's
// name master's build gave the same document (recorded from 0fe10934, before primitives were
// exact): the sphere's 16 rows of 32 facets less the two rows inside the slab (facets 224 to 287),
// the cylinder's 32 and its caps, and the slab's caps and its one wall inside the part.
TEST(ExactConstruction, ACutAfterItDropsTheExactPart) {
    Document part;
    part.setType(DocumentType::Part);
    auto cylinder = imported(read(hz::step_text::externalStyleCylinder()), "cylinder.step");
    cylinder->restoreFeatureID("imported_1");
    part.featureTree().addFeature(std::move(cylinder));
    auto sphere = PrimitiveFeature::makeSphere(1.5);
    sphere->restoreFeatureID("primitive_1");
    ASSERT_TRUE(sphere->setVector("basePoint", Vec3(5.0, 0.0, 0.5)));
    part.featureTree().addFeature(std::move(sphere));
    auto slab = cut(0, -2, 7, 2, 1.0);
    slab->restoreFeatureID("extrude_1");
    // The wall at x = 0, the one inside the part, is named for its line, the sketch's fourth (its
    // id is the process's: e4 when this test ran alone on master).
    const std::string wall =
        "extrude_1/side:e" + std::to_string(slab->sketch()->entities()[3]->id());
    part.featureTree().addFeature(std::move(slab));
    ASSERT_TRUE(part.rebuildModel()) << part.lastBuildMessage();
    EXPECT_EQ(part.exactSolid(), nullptr);
    EXPECT_FALSE(part.exactMesh());
    EXPECT_TRUE(part.exactNote().empty()) << "none, by design: " << part.exactNote();

    ASSERT_NE(part.solid(), nullptr);
    EXPECT_EQ(part.solid()->faceCount(), 486u);
    EXPECT_EQ(part.solid()->edges().size(), 996u);
    EXPECT_EQ(part.solid()->shellCount(), 3u);
    const double masters = 11.877898820156553;
    EXPECT_NEAR(volumeOf(*part.solid()), masters, masters * 1e-12);
    std::set<std::string> expected{"imported_1/face:1",         "imported_1/face:2",
                                   "extrude_1/cap_bottom",      "extrude_1/cap_top/piece:0",
                                   "extrude_1/cap_top/piece:1", wall};
    for (int k = 0; k < 32; ++k) expected.insert("imported_1/face:0/facet:" + std::to_string(k));
    for (int k = 0; k < 512; ++k) {
        if (k < 224 || k >= 288) expected.insert("primitive_1/band/facet:" + std::to_string(k));
    }
    EXPECT_EQ(faceNames(*part.solid()), expected);
}

// Decision 1b: a primitive whose exact build fails fails, saying why; it is never built by
// master's faceted construction in its place. A torus with no tube has no exact form.
TEST(ExactConstruction, AFailedExactBuildFailsTheFeature) {
    const auto torus = PrimitiveFeature::makeTorus(5.0, 0.0);
    std::string why;
    const hz::doc::FeatureOutput out = torus->executeWithExact({}, nullptr, &why);
    EXPECT_EQ(out.faceted, nullptr);
    EXPECT_EQ(out.exact, nullptr);
    EXPECT_EQ(why.rfind("could not be built exactly: ", 0), 0u) << why;
    why.clear();
    EXPECT_EQ(torus->execute(nullptr, &why), nullptr);
    EXPECT_EQ(why.rfind("could not be built exactly: ", 0), 0u) << why;

    Document part;
    buildAlone(part, PrimitiveFeature::makeTorus(5.0, 0.0));
    EXPECT_FALSE(part.rebuildModel());
    EXPECT_EQ(part.failedFeatureIndex(), 0);
    EXPECT_EQ(
        part.lastBuildMessage().rfind("Feature 'Torus' failed: could not be built exactly: ", 0),
        0u)
        << part.lastBuildMessage();
    EXPECT_EQ(part.solid(), nullptr);
    EXPECT_EQ(part.exactSolid(), nullptr);
}

// A primitive named by position (a part saved before persistent naming) builds as master did:
// its factory's faces, `cylinder/side<i>`, and no exact part.
TEST(ExactConstruction, APositionalPrimitiveBuildsAsMasterDid) {
    auto cylinder = PrimitiveFeature::makeCylinder(5.0, 10.0);
    cylinder->setNaming(hz::model::NamingScheme::Positional);
    const auto masters = mastersBody(*cylinder);
    ASSERT_NE(masters, nullptr);
    Document part;
    buildAlone(part, std::move(cylinder));
    ASSERT_TRUE(part.rebuildModel()) << part.lastBuildMessage();
    EXPECT_EQ(part.exactSolid(), nullptr);
    EXPECT_TRUE(part.exactNote().empty()) << part.exactNote();
    ASSERT_NE(part.solid(), nullptr);
    std::set<std::string> expected{"cylinder/top", "cylinder/bottom"};
    for (int i = 0; i < 32; ++i) expected.insert("cylinder/side" + std::to_string(i));
    EXPECT_EQ(faceNames(*part.solid()), expected);
    EXPECT_EQ(faceNames(*part.solid()), faceNames(*masters));
    EXPECT_DOUBLE_EQ(volumeOf(*part.solid()), volumeOf(*masters));
}

// Each primitive, stood anywhere, is exact, and its faceted copy is master's body vertex for
// vertex, with master's counts: built in the frame master's placement moves it to, its seam
// along that frame's x where master's ring starts, and cut at master's counts (a sphere's rows
// and a torus's tube at half its segments).
TEST(ExactConstruction, EachPrimitiveStandsWhereMastersStood) {
    struct Case {
        const char* what;
        std::unique_ptr<PrimitiveFeature> feature;
        double volume;
    };
    std::vector<Case> cases;
    cases.push_back({"a box", PrimitiveFeature::makeBox(2.0, 3.0, 4.0), 24.0});
    cases.push_back({"a cylinder", PrimitiveFeature::makeCylinder(2.0, 5.0), kPi * 4.0 * 5.0});
    cases.push_back({"a sphere", PrimitiveFeature::makeSphere(3.0), 4.0 / 3.0 * kPi * 27.0});
    cases.push_back({"a frustum", PrimitiveFeature::makeCone(3.0, 1.0, 4.0),
                     kPi * 4.0 / 3.0 * (9.0 + 3.0 + 1.0)});
    cases.push_back({"a cone", PrimitiveFeature::makeCone(0.0, 2.0, 3.0), kPi * 4.0 * 3.0 / 3.0});
    cases.push_back(
        {"a torus", PrimitiveFeature::makeTorus(5.0, 1.5), 2.0 * kPi * kPi * 5.0 * 2.25});
    for (Case& c : cases) {
        SCOPED_TRACE(c.what);
        ASSERT_TRUE(c.feature->setVector("basePoint", Vec3(1.0, -2.0, 3.0)));
        ASSERT_TRUE(c.feature->setVector("axisDirection", Vec3(0.3, 1.0, 0.6)));
        if (c.feature->isFaceted()) {
            ASSERT_TRUE(c.feature->setParameter("segments", 24));
        }
        const auto masters = mastersBody(*c.feature);
        ASSERT_NE(masters, nullptr);
        // Named as master's Stable primitive named it: its faces' logical names are the exact
        // faces'.
        hz::model::scopeToFeature(*masters, c.feature->featureID());
        hz::model::nameFacetsLogically(*masters);
        Document part;
        buildAlone(part, std::move(c.feature));
        ASSERT_TRUE(part.rebuildModel()) << part.lastBuildMessage();
        ASSERT_NE(part.exactSolid(), nullptr) << part.exactNote();
        EXPECT_NEAR(volumeOf(*part.exactSolid()), c.volume, c.volume * 1e-9);
        const hz::topo::Solid& faceted = *part.solid();
        EXPECT_EQ(faceted.faceCount(), masters->faceCount());
        EXPECT_EQ(faceted.edges().size(), masters->edges().size());
        EXPECT_EQ(faceted.vertices().size(), masters->vertices().size());
        for (const auto& v : faceted.vertices()) {
            double nearest = 1e300;
            for (const auto& w : masters->vertices()) {
                nearest = std::min(nearest, (v.point - w.point).length());
            }
            EXPECT_LT(nearest, 1e-12 * 10.0) << v.point.x << " " << v.point.y << " " << v.point.z;
        }
        EXPECT_NEAR(volumeOf(faceted), volumeOf(*masters), 1e-12 * c.volume);
        EXPECT_EQ(faceNames(*part.exactSolid()), logicalFaces(*masters));
        expectNamedAsItsFacets(*part.exactSolid(), faceted);
    }
}

// Ruling 2: a torus whose tube reaches its axis (a tube as wide as its ring, or wider), which
// master's factory refused, is refused as master refused it, in master's words (recorded at
// 0fe10934). The exact primitive's apple stays for the revolves that will need it. A torus with
// no tube has no exact form, and says so (AFailedExactBuildFailsTheFeature).
TEST(ExactConstruction, ATorusWhoseTubeReachesItsAxisIsRefusedAsBefore) {
    const std::string masters =
        "Feature 'Torus' failed: these dimensions do not make a solid: they must be positive (a "
        "cone may have one zero radius, a torus's tube must be thinner than its ring)";
    for (const double tube : {5.0, 6.0}) {
        SCOPED_TRACE(tube);
        auto torus = PrimitiveFeature::makeTorus(5.0, tube);
        std::string why;
        const hz::doc::FeatureOutput out = torus->executeWithExact({}, nullptr, &why);
        EXPECT_EQ(out.faceted, nullptr);
        EXPECT_EQ(out.exact, nullptr);
        Document part;
        buildAlone(part, std::move(torus));
        EXPECT_FALSE(part.rebuildModel());
        EXPECT_EQ(part.lastBuildMessage(), masters);
        EXPECT_EQ(part.solid(), nullptr);
        EXPECT_EQ(part.exactSolid(), nullptr);
    }
}

// Ruling 19: a box with a negative size is built as master built it, on the other side of its base
// along that axis: the exact box of its extents, each face named as master's factory names the
// face in that place (`left` at x = 0 and `right` at x = width, whichever side that is). Its
// faceted copy is master's: its counts, volume and box, and each face's name and where it is, and
// every edge's name, as recorded at 0fe10934. Its exact part is the same box.
TEST(ExactConstruction, ABoxWithANegativeSizeIsBuiltAsBefore) {
    struct Case {
        const char* what;
        std::unique_ptr<PrimitiveFeature> feature;
        Vec3 min;
        Vec3 max;
        std::map<std::string, Vec3> centres;  ///< each face's corners' mean, by its role
    };
    std::vector<Case> cases;
    cases.push_back({"-2 x 3 x -4, at the origin",
                     PrimitiveFeature::makeBox(-2.0, 3.0, -4.0),
                     {-2, 0, -4},
                     {0, 3, 0},
                     {{"bottom", {-1, 1.5, 0}},
                      {"front", {-1, 0, -2}},
                      {"right", {-2, 1.5, -2}},
                      {"back", {-1, 3, -2}},
                      {"left", {0, 1.5, -2}},
                      {"top", {-1, 1.5, -4}}}});
    cases.push_back({"-2 x -3 x 4, stood at (1, 2, 3) along (0.3, 1, 0.6)",
                     PrimitiveFeature::makeBox(-2.0, -3.0, 4.0),
                     {-0.91714598003856818, 0.38090033269053247, 3},
                     {2.4108158580520391, 5.5979992606877049, 7.9827287912243978},
                     {{"bottom", {0.24856205988429569, 1.3285401996143196, 4.4948186373673185}},
                      {"front", {0.53969988910315569, 3.7989996303438529, 4.2456821978060999}},
                      {"right", {-0.21173805101254867, 3.1275398299581716, 5.7405008351734192}},
                      {"back", {0.95396998891031537, 2.1798999630343849, 6.7370465934182988}},
                      {"left", {1.7054079290260198, 2.8513597634200654, 5.2422279560509795}},
                      {"top", {1.2451078181291753, 4.6503593937639183, 6.4879101538570794}}}});
    ASSERT_TRUE(cases[1].feature->setVector("basePoint", Vec3(1.0, 2.0, 3.0)));
    ASSERT_TRUE(cases[1].feature->setVector("axisDirection", Vec3(0.3, 1.0, 0.6)));
    const char* edges[] = {"back|bottom",  "back|left",   "back|right",   "back|top",
                           "bottom|front", "bottom|left", "bottom|right", "front|left",
                           "front|right",  "front|top",   "left|top",     "right|top"};
    for (Case& c : cases) {
        SCOPED_TRACE(c.what);
        const std::string id = c.feature->featureID();
        Document part;
        buildAlone(part, std::move(c.feature));
        ASSERT_TRUE(part.rebuildModel()) << part.lastBuildMessage();
        ASSERT_NE(part.solid(), nullptr);
        const hz::topo::Solid& faceted = *part.solid();
        EXPECT_EQ(faceted.faceCount(), 6u);
        EXPECT_EQ(faceted.edges().size(), 12u);
        EXPECT_EQ(faceted.vertices().size(), 8u);
        EXPECT_NEAR(volumeOf(faceted), 24.0, 24.0 * 1e-12);
        hz::math::BoundingBox box;
        for (const auto& v : faceted.vertices()) box.expand(v.point);
        EXPECT_LT((box.min() - c.min).length(), 1e-12 * 10.0);
        EXPECT_LT((box.max() - c.max).length(), 1e-12 * 10.0);
        std::set<std::string> names;
        for (const auto& face : faceted.faces()) {
            const std::string& tag = face.topoId.tag();
            names.insert(tag);
            ASSERT_EQ(tag.rfind(id + "/", 0), 0u) << tag;
            const auto expected = c.centres.find(tag.substr(id.size() + 1));
            ASSERT_NE(expected, c.centres.end()) << tag;
            Vec3 centre;
            int corners = 0;
            const hz::topo::HalfEdge* he = face.outerLoop->halfEdge;
            do {
                centre = centre + he->origin->point;
                ++corners;
                he = he->next;
            } while (he != face.outerLoop->halfEdge);
            centre = centre / static_cast<double>(corners);
            EXPECT_LT((centre - expected->second).length(), 1e-12 * 10.0) << tag;
        }
        EXPECT_EQ(names.size(), 6u);
        std::set<std::string> expectedEdges;
        for (const char* e : edges) expectedEdges.insert(id + "/edge:" + e);
        std::set<std::string> edgeNames;
        for (const auto& e : faceted.edges()) edgeNames.insert(e.topoId.tag());
        EXPECT_EQ(edgeNames, expectedEdges);

        ASSERT_NE(part.exactSolid(), nullptr) << part.exactNote();
        EXPECT_NEAR(volumeOf(*part.exactSolid()), 24.0, 24.0 * 1e-9);
        expectNamedAsItsFacets(*part.exactSolid(), faceted);
    }
}

namespace {

/// A slot of two lines of 20 and two half circles of radius 5 on @p sketch: (20 x 10 + 25 pi) in
/// area.
void drawSlot(hz::doc::Sketch& sketch) {
    sketch.addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(0, -5), Vec2(20, -5)));
    sketch.addEntity(std::make_shared<hz::draft::DraftArc>(Vec2(20, 0), 5, -kPi / 2, kPi / 2));
    sketch.addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(20, 5), Vec2(0, 5)));
    sketch.addEntity(std::make_shared<hz::draft::DraftArc>(Vec2(0, 0), 5, kPi / 2, 3 * kPi / 2));
}

/// The names @p recorded, "{k}" standing for the k-th entity of @p sketch ("e<id>").
std::set<std::string> namedFor(const std::set<std::string>& recorded,
                               const hz::doc::Sketch& sketch) {
    std::set<std::string> out;
    for (std::string name : recorded) {
        for (size_t k = 0; k < sketch.entities().size(); ++k) {
            const std::string mark = "{" + std::to_string(k) + "}";
            const size_t at = name.find(mark);
            if (at != std::string::npos) {
                name.replace(at, mark.size(), "e" + std::to_string(sketch.entities()[k]->id()));
            }
        }
        out.insert(name);
    }
    return out;
}

}  // namespace

// Extrusions are exact (stage 3): a Symmetric one, and one up to a face of a part built before it
// as a new body, plane and distance found as before. Each faceted part is master's: its counts and
// logical faces recorded at 1709c556, before the extrude was exact (the symmetric slot 36 faces,
// 102 edges and 68 vertices; the box and the rod up to its top 40, 108 and 72 in two shells).
TEST(ExactConstruction, SymmetricAndUpToFaceExtrudesAreExact) {
    {
        SCOPED_TRACE("a slot, 6 each way");
        auto sketch = std::make_shared<hz::doc::Sketch>();
        drawSlot(*sketch);
        auto extrude = std::make_unique<hz::doc::ExtrudeFeature>(sketch, Vec3(0, 0, 1), 6.0);
        extrude->setExtent(hz::doc::ExtrudeFeature::Extent::Symmetric);
        extrude->restoreFeatureID("extrude_1");
        Document part;
        part.setType(DocumentType::Part);
        part.addSketch(sketch);
        part.featureTree().addFeature(std::move(extrude));
        ASSERT_TRUE(part.rebuildModel()) << part.lastBuildMessage();
        ASSERT_NE(part.exactSolid(), nullptr) << part.exactNote();
        EXPECT_EQ(part.exactSolid()->faceCount(), 6u);
        const double volume = (200.0 + 25.0 * kPi) * 6.0;
        EXPECT_NEAR(volumeOf(*part.exactSolid()), volume, volume * 1e-9);
        hz::math::BoundingBox box;
        for (const auto& v : part.exactSolid()->vertices()) box.expand(v.point);
        EXPECT_NEAR(box.min().z, -3.0, 1e-12);
        EXPECT_NEAR(box.max().z, 3.0, 1e-12);

        const hz::topo::Solid& faceted = *part.solid();
        EXPECT_EQ(faceted.faceCount(), 36u);
        EXPECT_EQ(faceted.edges().size(), 102u);
        EXPECT_EQ(faceted.vertices().size(), 68u);
        EXPECT_EQ(logicalFaces(faceted),
                  namedFor({"extrude_1/cap_bottom", "extrude_1/cap_top", "extrude_1/side:{0}",
                            "extrude_1/side:{1}", "extrude_1/side:{2}", "extrude_1/side:{3}"},
                           *sketch));
        expectNamedAsItsFacets(*part.exactSolid(), faceted);
        expectMeshed(part, volume);
    }
    {
        SCOPED_TRACE("a rod down to a box's top, a new body");
        Document part;
        part.setType(DocumentType::Part);
        auto box = PrimitiveFeature::makeBox(10, 10, 10);
        box->restoreFeatureID("primitive_1");
        part.featureTree().addFeature(std::move(box));
        auto sketch = std::make_shared<hz::doc::Sketch>(
            hz::draft::SketchPlane(Vec3(25, 5, 20), Vec3(0, 0, 1), Vec3(1, 0, 0)));
        sketch->addEntity(std::make_shared<hz::draft::DraftCircle>(Vec2(0, 0), 2.0));
        part.addSketch(sketch);
        auto extrude = std::make_unique<hz::doc::ExtrudeFeature>(sketch, Vec3(0, 0, -1), 1.0);
        extrude->setExtent(hz::doc::ExtrudeFeature::Extent::UpToFace);
        extrude->setUpToFace("primitive_1/top");
        extrude->setOperation(BodyOperation::NewBody);
        extrude->restoreFeatureID("extrude_2");
        part.featureTree().addFeature(std::move(extrude));
        ASSERT_TRUE(part.rebuildModel()) << part.lastBuildMessage();
        const hz::topo::Solid* exact = part.exactSolid();
        ASSERT_NE(exact, nullptr) << part.exactNote();
        EXPECT_EQ(exact->shellCount(), 2u);
        EXPECT_EQ(exact->faceCount(), 9u);
        const double volume = 1000.0 + kPi * 4.0 * 10.0;
        EXPECT_NEAR(volumeOf(*exact), volume, volume * 1e-9);

        const hz::topo::Solid& faceted = *part.solid();
        EXPECT_EQ(faceted.faceCount(), 40u);
        EXPECT_EQ(faceted.edges().size(), 108u);
        EXPECT_EQ(faceted.vertices().size(), 72u);
        EXPECT_EQ(faceted.shellCount(), 2u);
        EXPECT_EQ(logicalFaces(faceted),
                  namedFor({"extrude_2/cap_bottom", "extrude_2/cap_top", "extrude_2/side:{0}",
                            "primitive_1/back", "primitive_1/bottom", "primitive_1/front",
                            "primitive_1/left", "primitive_1/right", "primitive_1/top"},
                           *sketch));
        expectNamedAsItsFacets(*exact, faceted);
        expectMeshed(part, volume);
    }
}

// An extrude named by position (a part saved before persistent naming) builds as master did: its
// faceted prism, `lateral_<i>`, and no exact part.
TEST(ExactConstruction, APositionalExtrudeBuildsAsMasterDid) {
    auto sketch = std::make_shared<hz::doc::Sketch>();
    drawSlot(*sketch);
    auto extrude = std::make_unique<hz::doc::ExtrudeFeature>(sketch, Vec3(0, 0, 1), 3.0);
    extrude->setNaming(hz::model::NamingScheme::Positional);
    extrude->restoreFeatureID("extrude_1");
    hz::doc::FeatureOutput out = extrude->executeWithExact({}, nullptr, nullptr);
    ASSERT_NE(out.faceted, nullptr);
    EXPECT_EQ(out.exact, nullptr);
    std::set<std::string> expected{"extrude_1/cap_bottom", "extrude_1/cap_top"};
    for (int i = 0; i < 34; ++i) expected.insert("extrude_1/lateral_" + std::to_string(i));
    EXPECT_EQ(faceNames(*out.faceted), expected);

    Document part;
    part.setType(DocumentType::Part);
    part.addSketch(sketch);
    part.featureTree().addFeature(std::move(extrude));
    ASSERT_TRUE(part.rebuildModel()) << part.lastBuildMessage();
    EXPECT_EQ(part.exactSolid(), nullptr);
    EXPECT_TRUE(part.exactNote().empty()) << part.exactNote();
    EXPECT_EQ(faceNames(*part.solid()), expected);
}

// Decision 1b: an extrude whose exact build fails fails, saying why; master's faceted
// construction never builds it in its place. A slot a third of a micrometre across whose top line
// misses its arcs by 0.9 nm (within the profile's join tolerance of 1 nm), which master's prism
// took, is too loose a fit for an exact part of that size: its corners may be off its curves by a
// thousandth of its size at most (topo::ToleranceGrowth).
TEST(ExactConstruction, AFailedExactExtrudeFailsTheFeature) {
    const double s = 1e-5;
    const double gap = 9e-7;
    auto sketch = std::make_shared<hz::doc::Sketch>();
    sketch->addEntity(
        std::make_shared<hz::draft::DraftLine>(Vec2(0, -5 * s), Vec2(20 * s, -5 * s)));
    sketch->addEntity(
        std::make_shared<hz::draft::DraftArc>(Vec2(20 * s, 0), 5 * s, -kPi / 2, kPi / 2));
    sketch->addEntity(
        std::make_shared<hz::draft::DraftLine>(Vec2(20 * s, 5 * s + gap), Vec2(0, 5 * s + gap)));
    sketch->addEntity(
        std::make_shared<hz::draft::DraftArc>(Vec2(0, 0), 5 * s, kPi / 2, 3 * kPi / 2));
    std::string masters;
    EXPECT_NE(
        hz::model::Extrude::execute(sketch->entities(), sketch->plane(), Vec3(0, 0, 1), 3 * s, "x",
                                    32, 0.0, &masters, hz::model::NamingScheme::FromGeometry),
        nullptr)
        << "master's prism took it: " << masters;
    auto extrude = std::make_unique<hz::doc::ExtrudeFeature>(sketch, Vec3(0, 0, 1), 3 * s);
    std::string why;
    const hz::doc::FeatureOutput out = extrude->executeWithExact({}, nullptr, &why);
    EXPECT_EQ(out.faceted, nullptr);
    EXPECT_EQ(out.exact, nullptr);
    EXPECT_EQ(why.rfind("could not be built exactly: ", 0), 0u) << why;

    Document part;
    part.setType(DocumentType::Part);
    part.addSketch(sketch);
    part.featureTree().addFeature(std::move(extrude));
    EXPECT_FALSE(part.rebuildModel());
    EXPECT_EQ(
        part.lastBuildMessage().rfind("Feature 'Extrude' failed: could not be built exactly: ", 0),
        0u)
        << part.lastBuildMessage();
    EXPECT_EQ(part.solid(), nullptr);
    EXPECT_EQ(part.exactSolid(), nullptr);
}

namespace {

/// A quarter circle of radius 3 about the origin from the axis at (0, -3) to (3, 0), then a line
/// up to the axis at (0, 4), on @p sketch: a hemisphere under a cone, 30 pi a whole turn.
void drawPoleAndApex(hz::doc::Sketch& sketch) {
    sketch.addEntity(std::make_shared<hz::draft::DraftArc>(Vec2(0, 0), 3, -kPi / 2, 0));
    sketch.addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(3, 0), Vec2(0, 4)));
    sketch.addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(0, 4), Vec2(0, -3)));
}

/// @p sketch turned by @p angle about its y axis, "revolve_1".
std::unique_ptr<hz::doc::RevolveFeature> revolveOf(const std::shared_ptr<hz::doc::Sketch>& sketch,
                                                   double angle) {
    auto revolve =
        std::make_unique<hz::doc::RevolveFeature>(sketch, Vec3(0, 0, 0), Vec3(0, 1, 0), angle);
    revolve->restoreFeatureID("revolve_1");
    return revolve;
}

}  // namespace

// A revolve is exact (stage 3): a hemisphere under a cone (a pole and an apex), a whole turn and a
// quarter, 30 pi and a quarter of it. Its faceted part is master's: its counts recorded at
// 9314271e, before the revolve was exact (288 faces, 544 edges, 258 vertices a whole turn; 74, 146
// and 74 a quarter), its logical faces master's logical names.
TEST(ExactConstruction, ARevolveIsExact) {
    struct Case {
        double angle;
        std::size_t faces, edges, vertices;
        std::set<std::string> names;
    };
    const std::vector<Case> cases = {
        {2 * kPi, 288, 544, 258, {"revolve_1/revolved:{0}", "revolve_1/revolved:{1}"}},
        {kPi / 2,
         74,
         146,
         74,
         {"revolve_1/cap_end", "revolve_1/cap_start", "revolve_1/revolved:{0}",
          "revolve_1/revolved:{1}"}}};
    for (const Case& c : cases) {
        SCOPED_TRACE(c.angle);
        auto sketch = std::make_shared<hz::doc::Sketch>();
        drawPoleAndApex(*sketch);
        Document part;
        part.setType(DocumentType::Part);
        part.addSketch(sketch);
        part.featureTree().addFeature(revolveOf(sketch, c.angle));
        ASSERT_TRUE(part.rebuildModel()) << part.lastBuildMessage();
        ASSERT_NE(part.exactSolid(), nullptr) << part.exactNote();
        const double volume = 15.0 * c.angle;
        EXPECT_NEAR(volumeOf(*part.exactSolid()), volume, volume * 1e-9);

        const hz::topo::Solid& faceted = *part.solid();
        EXPECT_EQ(faceted.faceCount(), c.faces);
        EXPECT_EQ(faceted.edges().size(), c.edges);
        EXPECT_EQ(faceted.vertices().size(), c.vertices);
        EXPECT_EQ(logicalFaces(faceted), namedFor(c.names, *sketch));
        expectNamedAsItsFacets(*part.exactSolid(), faceted);
        expectMeshed(part, volume);
    }
}

// A revolve named by position (a part saved before persistent naming) builds as master did: its
// faceted revolve, `revolved_<i>_<k>`, and no exact part.
TEST(ExactConstruction, APositionalRevolveBuildsAsMasterDid) {
    auto sketch = std::make_shared<hz::doc::Sketch>();
    drawPoleAndApex(*sketch);
    auto revolve = revolveOf(sketch, kPi / 2);
    revolve->setNaming(hz::model::NamingScheme::Positional);
    hz::doc::FeatureOutput out = revolve->executeWithExact({}, nullptr, nullptr);
    ASSERT_NE(out.faceted, nullptr);
    EXPECT_EQ(out.exact, nullptr);
    EXPECT_EQ(out.faceted->faceCount(), 74u);
    EXPECT_EQ(faceNames(*out.faceted).count("revolve_1/revolved_0_0"), 1u);

    Document part;
    part.setType(DocumentType::Part);
    part.addSketch(sketch);
    part.featureTree().addFeature(std::move(revolve));
    ASSERT_TRUE(part.rebuildModel()) << part.lastBuildMessage();
    EXPECT_EQ(part.exactSolid(), nullptr);
    EXPECT_TRUE(part.exactNote().empty()) << part.exactNote();
}

// Decision 1b: a revolve whose exact build fails fails, saying why; master's faceted construction
// never builds it in its place. A disc that touches the axis turns to a horn, its cusp on the
// axis, which master faceted and the exact revolve cannot build.
TEST(ExactConstruction, AFailedExactRevolveFailsTheFeature) {
    auto sketch = std::make_shared<hz::doc::Sketch>();
    sketch->addEntity(std::make_shared<hz::draft::DraftCircle>(Vec2(1, 0), 1.0));
    std::string masters;
    EXPECT_NE(hz::model::Revolve::execute(sketch->entities(), sketch->plane(), Vec3(),
                                          Vec3(0, 1, 0), 2 * kPi, "x", 32, 0.0, &masters,
                                          hz::model::NamingScheme::FromGeometry),
              nullptr)
        << "master faceted it: " << masters;
    auto revolve = revolveOf(sketch, 2 * kPi);
    std::string why;
    const hz::doc::FeatureOutput out = revolve->executeWithExact({}, nullptr, &why);
    EXPECT_EQ(out.faceted, nullptr);
    EXPECT_EQ(out.exact, nullptr);
    EXPECT_EQ(why.rfind("could not be built exactly: ", 0), 0u) << why;

    Document part;
    part.setType(DocumentType::Part);
    part.addSketch(sketch);
    part.featureTree().addFeature(std::move(revolve));
    EXPECT_FALSE(part.rebuildModel());
    EXPECT_EQ(
        part.lastBuildMessage().rfind("Feature 'Revolve' failed: could not be built exactly: ", 0),
        0u)
        << part.lastBuildMessage();
    EXPECT_EQ(part.solid(), nullptr);
    EXPECT_EQ(part.exactSolid(), nullptr);
}
