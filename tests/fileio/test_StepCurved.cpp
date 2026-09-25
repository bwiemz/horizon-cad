// Curved STEP faces made the kernel's way (Phase 141): a cylinder, cone,
// sphere or torus as a file describes it (a face bounded by circles and a
// seam, a pole a vertex) cut into facets that record the surface, so its
// volume is right as modelled and exact as designed. As read, a cylinder's
// caps are loops of one vertex, which the boundary mesh skips.

#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <numbers>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "horizon/document/Document.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/fileio/ImportReport.h"
#include "horizon/fileio/StepFormat.h"
#include "horizon/geometry/curves/NurbsCurve.h"
#include "horizon/math/Mat4.h"
#include "horizon/modeling/BooleanOp.h"
#include "horizon/modeling/Faceting.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/modeling/Naming.h"
#include "horizon/modeling/Pattern.h"
#include "horizon/modeling/PrimitiveFactory.h"
#include "horizon/topology/Solid.h"

namespace fs = std::filesystem;
using hz::io::StepFormat;
using hz::model::MassPropertiesCalculator;

namespace {

constexpr double kPi = std::numbers::pi;

std::string step(const std::string& data, const std::string& units = {}) {
    return "ISO-10303-21;\nHEADER;\nFILE_SCHEMA(('AUTOMOTIVE_DESIGN'));\nENDSEC;\nDATA;\n" + units +
           data + "ENDSEC;\nEND-ISO-10303-21;\n";
}

/// A cylinder of radius 1 and height 2, as OpenCASCADE writes one: V = 2,
/// E = 3, F = 3, each cap bounded by one circle.
std::string cylinder() {
    return step(R"(#1 = CARTESIAN_POINT('',(0.,0.,0.));
#2 = DIRECTION('',(0.,0.,1.));
#3 = DIRECTION('',(0.,1.,0.));
#4 = AXIS2_PLACEMENT_3D('',#1,#2,#3);
#5 = CIRCLE('',#4,1.);
#6 = CARTESIAN_POINT('',(1.,0.,0.));
#7 = VERTEX_POINT('',#6);
#8 = CARTESIAN_POINT('',(0.,0.,2.));
#9 = DIRECTION('',(1.,0.,0.));
#10 = AXIS2_PLACEMENT_3D('',#8,#2,#9);
#11 = CIRCLE('',#10,1.);
#12 = CARTESIAN_POINT('',(1.,0.,2.));
#13 = VERTEX_POINT('',#12);
#14 = EDGE_CURVE('',#7,#7,#5,.T.);
#15 = EDGE_CURVE('',#13,#13,#11,.T.);
#16 = LINE('',#6,VECTOR('',#2,1.));
#17 = SURFACE_CURVE('',#16,(),.CURVE_3D.);
#18 = EDGE_CURVE('',#7,#13,#17,.T.);
#19 = AXIS2_PLACEMENT_3D('',#1,#2,#9);
#20 = CYLINDRICAL_SURFACE('',#19,1.);
#21 = ORIENTED_EDGE('',*,*,#14,.T.);
#22 = ORIENTED_EDGE('',*,*,#18,.T.);
#23 = ORIENTED_EDGE('',*,*,#15,.F.);
#24 = ORIENTED_EDGE('',*,*,#18,.F.);
#25 = EDGE_LOOP('',(#21,#22,#23,#24));
#26 = FACE_OUTER_BOUND('',#25,.T.);
#27 = ADVANCED_FACE('',(#26),#20,.T.);
#28 = AXIS2_PLACEMENT_3D('',#8,#2,$);
#29 = PLANE('',#28);
#30 = ORIENTED_EDGE('',*,*,#15,.T.);
#31 = EDGE_LOOP('',(#30));
#32 = FACE_OUTER_BOUND('',#31,.T.);
#33 = ADVANCED_FACE('',(#32),#29,.T.);
#34 = AXIS2_PLACEMENT_3D('',#1,#2,$);
#35 = PLANE('',#34);
#36 = ORIENTED_EDGE('',*,*,#14,.F.);
#37 = EDGE_LOOP('',(#36));
#38 = FACE_OUTER_BOUND('',#37,.T.);
#39 = ADVANCED_FACE('',(#38),#35,.F.);
#40 = CLOSED_SHELL('',(#27,#33,#39));
#41 = MANIFOLD_SOLID_BREP('cylinder',#40);
)");
}

/// A cone: base radius 2 at z = 0, apex at z = 3; the side a face from the
/// base circle up a seam to the apex and back. @p semiAngle as written.
std::string cone(const std::string& semiAngle = "0.5880026035475675",
                 const std::string& units = {}) {
    return step(R"(#1 = CARTESIAN_POINT('',(0.,0.,0.));
#2 = DIRECTION('',(0.,0.,1.));
#3 = DIRECTION('',(1.,0.,0.));
#4 = AXIS2_PLACEMENT_3D('',#1,#2,#3);
#5 = CIRCLE('',#4,2.);
#6 = CARTESIAN_POINT('',(2.,0.,0.));
#7 = VERTEX_POINT('',#6);
#8 = CARTESIAN_POINT('',(0.,0.,3.));
#9 = VERTEX_POINT('',#8);
#10 = EDGE_CURVE('',#7,#7,#5,.T.);
#11 = DIRECTION('',(-0.55470019622522915,0.,0.83205029433784372));
#12 = VECTOR('',#11,1.);
#13 = LINE('',#6,#12);
#14 = EDGE_CURVE('',#7,#9,#13,.T.);
#15 = DIRECTION('',(0.,0.,-1.));
#16 = AXIS2_PLACEMENT_3D('',#1,#15,#3);
#17 = CONICAL_SURFACE('',#16,2.,)" +
                    semiAngle + R"();
#18 = ORIENTED_EDGE('',*,*,#10,.T.);
#19 = ORIENTED_EDGE('',*,*,#14,.T.);
#20 = ORIENTED_EDGE('',*,*,#14,.F.);
#21 = EDGE_LOOP('',(#18,#19,#20));
#22 = FACE_OUTER_BOUND('',#21,.T.);
#23 = ADVANCED_FACE('',(#22),#17,.T.);
#24 = PLANE('',#4);
#25 = ORIENTED_EDGE('',*,*,#10,.F.);
#26 = EDGE_LOOP('',(#25));
#27 = FACE_OUTER_BOUND('',#26,.T.);
#28 = ADVANCED_FACE('',(#27),#24,.F.);
#29 = CLOSED_SHELL('',(#23,#28));
#30 = MANIFOLD_SOLID_BREP('cone',#29);
)",
                units);
}

/// A sphere of radius 1.5: one face, its boundary a seam from pole to pole
/// and back.
std::string sphere() {
    return step(R"(#1 = CARTESIAN_POINT('',(0.,0.,0.));
#2 = DIRECTION('',(0.,0.,1.));
#3 = DIRECTION('',(1.,0.,0.));
#4 = AXIS2_PLACEMENT_3D('',#1,#2,#3);
#5 = SPHERICAL_SURFACE('',#4,1.5);
#6 = CARTESIAN_POINT('',(0.,0.,-1.5));
#7 = VERTEX_POINT('',#6);
#8 = CARTESIAN_POINT('',(0.,0.,1.5));
#9 = VERTEX_POINT('',#8);
#10 = DIRECTION('',(0.,-1.,0.));
#11 = AXIS2_PLACEMENT_3D('',#1,#10,#3);
#12 = CIRCLE('',#11,1.5);
#13 = EDGE_CURVE('',#7,#9,#12,.T.);
#14 = ORIENTED_EDGE('',*,*,#13,.T.);
#15 = ORIENTED_EDGE('',*,*,#13,.F.);
#16 = EDGE_LOOP('',(#14,#15));
#17 = FACE_OUTER_BOUND('',#16,.T.);
#18 = ADVANCED_FACE('',(#17),#5,.T.);
#19 = CLOSED_SHELL('',(#18));
#20 = MANIFOLD_SOLID_BREP('sphere',#19);
)");
}

/// A torus of radii 4 and 1: one face, its boundary two seams, each there
/// and back.
std::string torus() {
    return step(R"(#1 = CARTESIAN_POINT('',(0.,0.,0.));
#2 = DIRECTION('',(0.,0.,1.));
#3 = DIRECTION('',(1.,0.,0.));
#4 = AXIS2_PLACEMENT_3D('',#1,#2,#3);
#5 = TOROIDAL_SURFACE('',#4,4.,1.);
#6 = CARTESIAN_POINT('',(5.,0.,0.));
#7 = VERTEX_POINT('',#6);
#8 = CIRCLE('',#4,5.);
#9 = EDGE_CURVE('',#7,#7,#8,.T.);
#10 = CARTESIAN_POINT('',(4.,0.,0.));
#11 = DIRECTION('',(0.,-1.,0.));
#12 = AXIS2_PLACEMENT_3D('',#10,#11,#3);
#13 = CIRCLE('',#12,1.);
#14 = EDGE_CURVE('',#7,#7,#13,.T.);
#15 = ORIENTED_EDGE('',*,*,#9,.T.);
#16 = ORIENTED_EDGE('',*,*,#14,.T.);
#17 = ORIENTED_EDGE('',*,*,#9,.F.);
#18 = ORIENTED_EDGE('',*,*,#14,.F.);
#19 = EDGE_LOOP('',(#15,#16,#17,#18));
#20 = FACE_OUTER_BOUND('',#19,.T.);
#21 = ADVANCED_FACE('',(#20),#5,.T.);
#22 = CLOSED_SHELL('',(#21));
#23 = MANIFOLD_SOLID_BREP('torus',#22);
)");
}

std::string fixture(const std::string& name) {
    std::ifstream in(fs::path(HZ_STEP_FIXTURE_DIR) / "import_ok" / name, std::ios::binary);
    std::stringstream text;
    text << in.rdbuf();
    return text.str();
}

/// @p text read, its one solid cut into facets, and measured both ways.
struct Measured {
    double modelled = 0.0;
    hz::model::IdealMassProperties ideal;
    size_t outlined = 0;
};

Measured measure(const std::string& text) {
    Measured m;
    const auto solids = StepFormat::fromString(text);
    EXPECT_EQ(solids.size(), 1u) << StepFormat::lastError();
    if (solids.size() != 1) return m;
    const auto faceted = hz::model::facetCurved(*solids[0]);
    EXPECT_NE(faceted.solid, nullptr) << faceted.error;
    if (!faceted.solid) return m;
    EXPECT_TRUE(faceted.solid->isValid()) << faceted.solid->validationReport();
    m.modelled = MassPropertiesCalculator::compute(*faceted.solid).volume;
    m.ideal = MassPropertiesCalculator::computeIdeal(*faceted.solid);
    m.outlined = faceted.outlined.size();
    return m;
}

void expectRelative(double actual, double expected, double relative, const char* what) {
    EXPECT_NEAR(actual, expected, relative * std::abs(expected)) << what;
}

}  // namespace

// The cylinder's caps, bounded by one circle each, are polygons now; its
// side is in facets. Its volume as modelled is the 32-gon prism's, exactly,
// and as designed the cylinder's.
TEST(StepCurvedTest, AnOccStyleCylinderHasTheRightVolumeBothWays) {
    const auto m = measure(cylinder());
    const double prism = 16.0 * std::sin(2.0 * kPi / 32.0) * 2.0;  // (n/2) r^2 sin(2 pi / n) h
    EXPECT_NEAR(m.modelled, prism, 1e-9);
    EXPECT_TRUE(m.ideal.exact);
    expectRelative(m.ideal.properties.volume, 2.0 * kPi, 1e-9, "the cylinder");
    expectRelative(m.ideal.properties.surfaceArea, 2.0 * kPi * 2.0 + 2.0 * kPi, 1e-9, "its area");
    EXPECT_EQ(m.outlined, 0u);
}

TEST(StepCurvedTest, AConeIsReadAndMeasured) {
    const auto m = measure(cone());
    EXPECT_TRUE(m.ideal.exact);
    expectRelative(m.ideal.properties.volume, kPi * 4.0 * 3.0 / 3.0, 1e-9, "the cone");
    expectRelative(m.ideal.properties.surfaceArea, kPi * 4.0 + kPi * 2.0 * std::sqrt(13.0), 1e-9,
                   "its area");
    expectRelative(m.modelled, kPi * 4.0, 0.01, "its facets, within 1 %");
}

TEST(StepCurvedTest, ASphereIsReadAndMeasured) {
    const auto m = measure(sphere());
    EXPECT_TRUE(m.ideal.exact);
    expectRelative(m.ideal.properties.volume, 4.0 / 3.0 * kPi * 3.375, 1e-9, "the sphere");
    expectRelative(m.ideal.properties.surfaceArea, 4.0 * kPi * 2.25, 1e-9, "its area");
    expectRelative(m.modelled, 4.0 / 3.0 * kPi * 3.375, 0.02, "its facets, within 2 %");
}

TEST(StepCurvedTest, ATorusIsReadAndMeasured) {
    const auto m = measure(torus());
    EXPECT_TRUE(m.ideal.exact);
    expectRelative(m.ideal.properties.volume, 2.0 * kPi * kPi * 4.0, 1e-9, "the torus");
    expectRelative(m.ideal.properties.surfaceArea, 4.0 * kPi * kPi * 4.0, 1e-9, "its area");
    expectRelative(m.modelled, 2.0 * kPi * kPi * 4.0, 0.02, "its facets, within 2 %");
}

// A semi-angle in degrees is read in degrees.
TEST(StepCurvedTest, AConeInDegreesIsTheSameCone) {
    const std::string degrees =
        "#100 = (NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.));\n"
        "#101 = PLANE_ANGLE_MEASURE_WITH_UNIT(PLANE_ANGLE_MEASURE(0.017453292519943295),#100);\n"
        "#102 = (CONVERSION_BASED_UNIT('DEGREE',#101) NAMED_UNIT(*) PLANE_ANGLE_UNIT());\n"
        "#103 = (LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.));\n"
        "#104 = (GEOMETRIC_REPRESENTATION_CONTEXT(3) GLOBAL_UNIT_ASSIGNED_CONTEXT((#103,#102)) "
        "REPRESENTATION_CONTEXT('',''));\n";
    const auto m = measure(cone("33.690067525979785", degrees));  // atan(2/3)
    expectRelative(m.ideal.properties.volume, kPi * 4.0, 1e-9, "the same cone");
}

// A plate with a round hole: its faces keep the hole, and the hole's wall,
// two half-cylinders, is in facets. 200 less the bore.
TEST(StepCurvedTest, APlateWithABoreHasTheRightVolumeBothWays) {
    const auto m = measure(fixture("plate_with_round_hole.step"));
    EXPECT_TRUE(m.ideal.exact);
    expectRelative(m.ideal.properties.volume, 200.0 - 8.0 * kPi, 1e-9, "the plate");
    expectRelative(m.modelled, 200.0 - 8.0 * kPi, 0.001, "its facets");
}

// An imported body is cut into facets when it is built, each named for the
// face it is part of; the file's own faces are kept for saving.
TEST(StepCurvedTest, AnImportedBodyIsBuiltInFacets) {
    auto solids = StepFormat::fromString(cylinder());
    ASSERT_EQ(solids.size(), 1u);
    hz::doc::FeatureTree tree;
    auto feature = std::make_unique<hz::doc::ImportedBodyFeature>(
        std::shared_ptr<const hz::topo::Solid>(std::move(solids[0])), "cylinder.step");
    const std::string id = feature->featureID();
    const auto* imported = feature.get();
    tree.addFeature(std::move(feature));
    const auto built = tree.build();
    ASSERT_NE(built, nullptr);
    EXPECT_EQ(imported->solid()->faceCount(), 3u) << "as read";
    EXPECT_GT(built->faceCount(), 32u) << "in facets";
    std::set<std::string> faces;
    for (const auto& face : built->faces()) faces.insert(hz::model::logicalFace(face.topoId.tag()));
    EXPECT_EQ(faces.size(), 3u) << "three faces still";
    EXPECT_TRUE(faces.count(id + "/face:0")) << *faces.begin();
    const auto ideal = MassPropertiesCalculator::computeIdeal(*built);
    expectRelative(ideal.properties.volume, 2.0 * kPi, 1e-9, "the cylinder");
}

// ---------------------------------------------------------------------------
// Written as designed (Phase 151)
// ---------------------------------------------------------------------------

namespace {

std::size_t count(const std::string& text, const std::string& what) {
    std::size_t n = 0;
    for (std::size_t at = text.find(what); at != std::string::npos; at = text.find(what, at + 1)) {
        ++n;
    }
    return n;
}

/// @p solid written as designed: the file, and the faces kept in facets.
std::pair<std::string, std::vector<std::string>> designed(const hz::topo::Solid& solid) {
    StepFormat::WriteReport report;
    const std::string text = StepFormat::toString({&solid}, {}, &report);
    return {text, report.faceted};
}

}  // namespace

// A cylinder goes out as three faces, its side one face on the cylinder,
// bounded by its rims' circles: other systems read a cylinder, not a
// 34-faced prism. Read back, it measures as the cylinder it is.
TEST(StepCurvedTest, ACylinderIsWrittenAsDesigned) {
    const auto cyl = hz::model::PrimitiveFactory::makeCylinder(4.0, 12.0);
    const auto [text, faceted] = designed(*cyl);
    EXPECT_TRUE(faceted.empty()) << faceted.front();
    EXPECT_EQ(count(text, "ADVANCED_FACE("), 3u);
    EXPECT_EQ(count(text, "RATIONAL_B_SPLINE_SURFACE("), 1u) << "the side, on the cylinder";
    EXPECT_EQ(count(text, "CIRCLE("), 64u) << "each rim chord an arc of its circle";
    const auto m = measure(text);
    EXPECT_TRUE(m.ideal.exact);
    expectRelative(m.ideal.properties.volume, kPi * 16.0 * 12.0, 1e-9, "the cylinder");
    expectRelative(m.ideal.properties.surfaceArea, 2.0 * kPi * 4.0 * 12.0 + 2.0 * kPi * 16.0, 1e-9,
                   "its area");
}

// A cylinder made as a part is made (a feature, stably named), and one
// moved there, go out as designed too.
TEST(StepCurvedTest, APartsCylinderAndAMovedOneAreWrittenAsDesigned) {
    hz::doc::Document part;
    part.featureTree().addFeature(hz::doc::PrimitiveFeature::makeCylinder(3.0, 5.0));
    ASSERT_TRUE(part.rebuildModel());
    const auto moved = hz::model::Pattern::transformed(
        *part.solid(), hz::math::Mat4::translation(hz::math::Vec3(10, -4, 2)));
    for (const hz::topo::Solid* solid : {part.solid(), moved.get()}) {
        const auto [text, faceted] = designed(*solid);
        EXPECT_TRUE(faceted.empty());
        EXPECT_EQ(count(text, "ADVANCED_FACE("), 3u);
        const auto m = measure(text);
        expectRelative(m.ideal.properties.volume, kPi * 9.0 * 5.0, 1e-9, "the cylinder");
    }
}

// A boss joined to a block: its side one face, its foot's circle found
// where the Boolean left none, and the whole measured exactly.
TEST(StepCurvedTest, ABossJoinedToABlockIsWrittenAsDesigned) {
    const auto block = hz::model::PrimitiveFactory::makeBox(40, 40, 10);
    const auto boss =
        hz::model::Pattern::transformed(*hz::model::PrimitiveFactory::makeCylinder(5, 10),
                                        hz::math::Mat4::translation(hz::math::Vec3(20, 20, 10)));
    const auto joined = hz::model::BooleanOp::execute(*block, *boss, hz::model::BooleanType::Union);
    ASSERT_NE(joined, nullptr);
    const auto [text, faceted] = designed(*joined);
    EXPECT_TRUE(faceted.empty()) << faceted.front();
    EXPECT_EQ(count(text, "RATIONAL_B_SPLINE_SURFACE("), 1u);
    const auto m = measure(text);
    EXPECT_TRUE(m.ideal.exact);
    expectRelative(m.ideal.properties.volume, 40.0 * 40.0 * 10.0 + kPi * 25.0 * 10.0, 1e-9,
                   "block and boss");
}

// A hole cut through a plate as a part's features cut one (names kept
// stably): its wall one face on the cylinder, its rims circles, and the
// plate measured exactly with the hole out of it.
TEST(StepCurvedTest, AHoleCutThroughAPlateIsWrittenAsDesigned) {
    const auto plate = hz::model::PrimitiveFactory::makeBox(40, 40, 10);
    const auto pin =
        hz::model::Pattern::transformed(*hz::model::PrimitiveFactory::makeCylinder(5, 30),
                                        hz::math::Mat4::translation(hz::math::Vec3(20, 20, -10)));
    const auto holed = hz::model::BooleanOp::execute(*plate, *pin, hz::model::BooleanType::Subtract,
                                                     nullptr, hz::model::NamingScheme::Stable);
    ASSERT_NE(holed, nullptr);
    const auto [text, faceted] = designed(*holed);
    EXPECT_TRUE(faceted.empty()) << faceted.front();
    EXPECT_EQ(count(text, "RATIONAL_B_SPLINE_SURFACE("), 1u) << "the hole's wall";
    const auto m = measure(text);
    EXPECT_TRUE(m.ideal.exact);
    expectRelative(m.ideal.properties.volume, 40.0 * 40.0 * 10.0 - kPi * 25.0 * 10.0, 1e-9,
                   "the plate less the hole");
}

// What cannot yet go out as designed goes out as its facets, and says why:
// a sphere (closed all round), a cone (its apex inside it), and a hole cut
// by a Boolean named by position, as older files are (it leaves the cut's
// triangles, its rim's chords split off the circle). Each still reads back
// as the solid it is.
TEST(StepCurvedTest, WhatCannotBeWrittenAsDesignedIsKeptInFacetsAndSaid) {
    const auto sphere = hz::model::PrimitiveFactory::makeSphere(5.0);
    const auto cone = hz::model::PrimitiveFactory::makeCone(4.0, 0.0, 12.0);
    const auto plate = hz::model::PrimitiveFactory::makeBox(40, 40, 10);
    const auto pin =
        hz::model::Pattern::transformed(*hz::model::PrimitiveFactory::makeCylinder(5, 30),
                                        hz::math::Mat4::translation(hz::math::Vec3(20, 20, -10)));
    const auto holed =
        hz::model::BooleanOp::execute(*plate, *pin, hz::model::BooleanType::Subtract);
    ASSERT_NE(holed, nullptr);
    const std::vector<std::pair<const hz::topo::Solid*, std::string>> cases{
        {sphere.get(), "closed all round"},
        {cone.get(), "comes to a point"},
        {holed.get(), "not all on one circle"}};
    for (const auto& [solid, why] : cases) {
        const auto [text, faceted] = designed(*solid);
        ASSERT_EQ(faceted.size(), 1u) << why;
        EXPECT_NE(faceted.front().find(why), std::string::npos) << faceted.front();
        EXPECT_EQ(count(text, "RATIONAL_B_SPLINE_SURFACE("), 0u) << "its facets, as they are";
        const auto back = StepFormat::fromString(text);
        ASSERT_EQ(back.size(), 1u);
        EXPECT_NEAR(MassPropertiesCalculator::compute(*back[0]).volume,
                    MassPropertiesCalculator::compute(*solid).volume,
                    1e-9 * MassPropertiesCalculator::compute(*solid).volume)
            << why;
    }
    // A box has no curved face: as it was.
    const auto box = hz::model::PrimitiveFactory::makeBox(2, 3, 4);
    const auto [text, faceted] = designed(*box);
    EXPECT_TRUE(faceted.empty());
    EXPECT_EQ(count(text, "ADVANCED_FACE("), 6u);
}

// A rim's circle is shared by its chords, and a Boolean gives it to the
// pieces of a chord it cuts, one end inside the circle: an edge whose own
// ends are not on its circle is not written as an arc of it (the arc would
// not end at its vertex), and its face goes out in facets, said.
TEST(StepCurvedTest, AnEdgeWhoseEndsAreOffItsCircleIsNotWrittenOnIt) {
    auto cyl = hz::model::Pattern::transformed(*hz::model::PrimitiveFactory::makeCylinder(4.0, 6.0),
                                               hz::math::Mat4::identity());
    hz::topo::Edge* rim = nullptr;
    for (auto& e : cyl->edges()) {
        if (e.analyticCurve && e.halfEdge != nullptr &&
            std::abs(e.halfEdge->origin->point.z) < 1e-12) {
            rim = &e;
            break;
        }
    }
    ASSERT_NE(rim, nullptr) << "a bottom rim chord";
    rim->analyticCurve = std::make_shared<hz::geo::NurbsCurve>(
        hz::geo::NurbsCurve::makeCircle(hz::math::Vec3(0, 0, 0), 4.1));
    const auto [text, faceted] = designed(*cyl);
    ASSERT_EQ(faceted.size(), 1u);
    EXPECT_NE(faceted.front().find("off its circle"), std::string::npos) << faceted.front();
    EXPECT_EQ(count(text, "RATIONAL_B_SPLINE_SURFACE("), 0u);
    const auto back = StepFormat::fromString(text);
    ASSERT_EQ(back.size(), 1u) << StepFormat::lastError();
}
