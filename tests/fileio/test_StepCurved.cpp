// Curved STEP faces made the kernel's way (Phase 141): a cylinder, cone,
// sphere or torus as a file describes it (a face bounded by circles and a
// seam, a pole a vertex) cut into facets that record the surface, so its
// volume is right as modelled and exact as designed. As read, a cylinder's
// caps are loops of one vertex, which the boundary mesh skips.

#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <map>
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

// A frustum goes out as designed too, its side one face on the cone it is
// part of. Its ideal cone ran from the wrong end until Phase 160, which the
// side did not fit, so it went out as its facets.
TEST(StepCurvedTest, AFrustumIsWrittenAsDesigned) {
    const auto cone = hz::model::PrimitiveFactory::makeCone(5.0, 2.0, 8.0);
    const auto [text, faceted] = designed(*cone);
    EXPECT_TRUE(faceted.empty()) << faceted.front();
    EXPECT_EQ(count(text, "ADVANCED_FACE("), 3u) << "bottom, top, and one side";
    const auto m = measure(text);
    // pi h (R^2 + R r + r^2) / 3, as near as its side is faceted when read:
    // 32 chords a turn hold (32 / 2 pi) sin(2 pi / 32), 0.64% less.
    const double frustum = kPi * 8.0 * (25.0 + 10.0 + 4.0) / 3.0;
    expectRelative(m.modelled, frustum, 1e-2, "the frustum");
    EXPECT_LT(m.modelled, frustum) << "facets inside it";
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

// ---------------------------------------------------------------------------
// Trimmed faces read (Phase 152)
// ---------------------------------------------------------------------------

namespace {

/// A cylinder of radius 1 standing on z = 0, cut off by the plane
/// z = 1 + x / 2: its side is a face on the cylinder whose top edge is an
/// ellipse, in (u, v) a wave, no rectangle. Its volume is pi (its height is
/// 1 on average), its side 2 pi, its top an ellipse of pi sqrt(1.25).
std::string slantedCylinder() {
    return step(R"(#1 = CARTESIAN_POINT('',(0.,0.,0.));
#2 = DIRECTION('',(0.,0.,1.));
#3 = DIRECTION('',(1.,0.,0.));
#4 = AXIS2_PLACEMENT_3D('',#1,#2,#3);
#5 = CIRCLE('',#4,1.);
#6 = CARTESIAN_POINT('',(1.,0.,0.));
#7 = VERTEX_POINT('',#6);
#8 = CARTESIAN_POINT('',(1.,0.,1.5));
#9 = VERTEX_POINT('',#8);
#10 = EDGE_CURVE('',#7,#7,#5,.T.);
#11 = CARTESIAN_POINT('',(1.,0.,1.5));
#12 = CARTESIAN_POINT('',(1.,1.,1.5));
#13 = CARTESIAN_POINT('',(0.,1.,1.));
#14 = CARTESIAN_POINT('',(-1.,1.,0.5));
#15 = CARTESIAN_POINT('',(-1.,0.,0.5));
#16 = CARTESIAN_POINT('',(-1.,-1.,0.5));
#17 = CARTESIAN_POINT('',(0.,-1.,1.));
#18 = CARTESIAN_POINT('',(1.,-1.,1.5));
#19 = CARTESIAN_POINT('',(1.,0.,1.5));
#20 = (BOUNDED_CURVE() B_SPLINE_CURVE(2,(#11,#12,#13,#14,#15,#16,#17,#18,#19),.UNSPECIFIED.,.T.,.F.) B_SPLINE_CURVE_WITH_KNOTS((3,2,2,2,3),(0.,0.25,0.5,0.75,1.),.UNSPECIFIED.) CURVE() GEOMETRIC_REPRESENTATION_ITEM() RATIONAL_B_SPLINE_CURVE((1.,@W,1.,@W,1.,@W,1.,@W,1.)) REPRESENTATION_ITEM(''));
#21 = EDGE_CURVE('',#9,#9,#20,.T.);
#22 = LINE('',#6,VECTOR('',#2,1.));
#23 = EDGE_CURVE('',#7,#9,#22,.T.);
#24 = AXIS2_PLACEMENT_3D('',#1,#2,#3);
#25 = CYLINDRICAL_SURFACE('',#24,1.);
#26 = ORIENTED_EDGE('',*,*,#10,.T.);
#27 = ORIENTED_EDGE('',*,*,#23,.T.);
#28 = ORIENTED_EDGE('',*,*,#21,.F.);
#29 = ORIENTED_EDGE('',*,*,#23,.F.);
#30 = EDGE_LOOP('',(#26,#27,#28,#29));
#31 = FACE_OUTER_BOUND('',#30,.T.);
#32 = ADVANCED_FACE('',(#31),#25,.T.);
#33 = CARTESIAN_POINT('',(0.,0.,1.));
#34 = DIRECTION('',(-0.4472135954999579,0.,0.8944271909999159));
#35 = DIRECTION('',(0.8944271909999159,0.,0.4472135954999579));
#36 = AXIS2_PLACEMENT_3D('',#33,#34,#35);
#37 = PLANE('',#36);
#38 = ORIENTED_EDGE('',*,*,#21,.T.);
#39 = EDGE_LOOP('',(#38));
#40 = FACE_OUTER_BOUND('',#39,.T.);
#41 = ADVANCED_FACE('',(#40),#37,.T.);
#42 = PLANE('',#4);
#43 = ORIENTED_EDGE('',*,*,#10,.F.);
#44 = EDGE_LOOP('',(#43));
#45 = FACE_OUTER_BOUND('',#44,.T.);
#46 = ADVANCED_FACE('',(#45),#42,.F.);
#47 = CLOSED_SHELL('',(#32,#41,#46));
#48 = MANIFOLD_SOLID_BREP('slanted',#47);
)");
}

}  // namespace

// A curved face that is no rectangle of its surface (a cylinder's side cut
// off on a slant) is cut into facets on the cylinder within its outline,
// not left one flat facet: measured as the part it is.
TEST(StepCurvedTest, ASlantCutCylindersSideIsCutWithinItsOutline) {
    std::string text = slantedCylinder();
    // The rational weights, written out.
    for (std::size_t at = text.find("@W"); at != std::string::npos; at = text.find("@W")) {
        text.replace(at, 2, "0.70710678118654757");
    }
    const auto m = measure(text);
    EXPECT_EQ(m.outlined, 0u) << "every curved face in facets on its surface";
    expectRelative(m.modelled, kPi, 0.01, "its facets, within 1 %");
    EXPECT_TRUE(m.ideal.exact);
    expectRelative(m.ideal.properties.volume, kPi, 1e-8, "the cut cylinder");
}

namespace {

/// Part-21 text built entity by entity.
class Entities {
public:
    int add(const std::string& rhs) {
        m_text += "#" + std::to_string(m_next) + " = " + rhs + ";\n";
        return m_next++;
    }
    int point(double x, double y, double z) {
        return add("CARTESIAN_POINT('',(" + num(x) + "," + num(y) + "," + num(z) + "))");
    }
    int direction(double x, double y, double z) {
        return add("DIRECTION('',(" + num(x) + "," + num(y) + "," + num(z) + "))");
    }
    int placement(int origin, int axis, int ref) {
        return add("AXIS2_PLACEMENT_3D('',#" + std::to_string(origin) + ",#" +
                   std::to_string(axis) + ",#" + std::to_string(ref) + ")");
    }
    int vertex(double x, double y, double z) {
        return add("VERTEX_POINT('',#" + std::to_string(point(x, y, z)) + ")");
    }
    int edge(int from, int to, int curve) {
        return add("EDGE_CURVE('',#" + std::to_string(from) + ",#" + std::to_string(to) + ",#" +
                   std::to_string(curve) + ",.T.)");
    }
    /// A face on @p surface bounded by loops of (edge, forward); the first
    /// its outer bound.
    int face(const std::vector<std::vector<std::pair<int, bool>>>& loops, int surface,
             bool sameSense) {
        std::string bounds;
        for (std::size_t l = 0; l < loops.size(); ++l) {
            std::string edges;
            for (const auto& [e, forward] : loops[l]) {
                const int oriented = add("ORIENTED_EDGE('',*,*,#" + std::to_string(e) + "," +
                                         (forward ? ".T." : ".F.") + ")");
                edges += (edges.empty() ? "#" : ",#") + std::to_string(oriented);
            }
            const int loop = add("EDGE_LOOP('',(" + edges + "))");
            const int bound = add(std::string(l == 0 ? "FACE_OUTER_BOUND" : "FACE_BOUND") +
                                  "('',#" + std::to_string(loop) + ",.T.)");
            bounds += (bounds.empty() ? "#" : ",#") + std::to_string(bound);
        }
        return add("ADVANCED_FACE('',(" + bounds + "),#" + std::to_string(surface) + "," +
                   (sameSense ? ".T." : ".F.") + ")");
    }
    const std::string& text() const { return m_text; }

private:
    static std::string num(double v) {
        std::ostringstream out;
        out.precision(17);
        out << v;
        std::string s = out.str();
        if (s.find_first_of(".eE") == std::string::npos) s += ".";
        return s;
    }
    std::string m_text;
    int m_next = 1;
};

/// A cylinder of radius 2 and height 4 with a pocket cut into its side:
/// 2 @p a radians wide about +x, from z = @p low to @p high, down to radius
/// 1.5. Its side is a face with a hole, the pocket's window. Its volume is
/// 16 pi less the pocket's a (4 - 2.25) (high - low): 0.525 by default.
std::string pocketedCylinder(double a = 0.3, double low = 1.5, double high = 2.5) {
    Entities e;
    const double c = std::cos(a);
    const double s = std::sin(a);
    const int origin = e.point(0, 0, 0);
    const int z = e.direction(0, 0, 1);
    const int x = e.direction(1, 0, 0);
    const auto circle = [&](double height, double radius) {
        const int placement = e.placement(e.point(0, 0, height), z, x);
        return e.add("CIRCLE('',#" + std::to_string(placement) + "," + std::to_string(radius) +
                     ")");
    };
    const auto line = [&](int from, double dx, double dy, double dz) {
        const int d = e.direction(dx, dy, dz);
        return e.add("LINE('',#" + std::to_string(from) + ",VECTOR('',#" + std::to_string(d) +
                     ",1.))");
    };
    // The whole cylinder: its seam at -x.
    const int vb = e.vertex(-2, 0, 0);
    const int vt = e.vertex(-2, 0, 4);
    const int bottom = e.edge(vb, vb, circle(0, 2));
    const int top = e.edge(vt, vt, circle(4, 2));
    const int seamFrom = e.point(-2, 0, 0);
    const int seam = e.edge(vb, vt, line(seamFrom, 0, 0, 1));
    // The pocket's corners: at radius r, angle -a or +a, height h.
    const auto corner = [&](double r, double sign, double h) {
        return e.vertex(r * c, sign * r * s, h);
    };
    const int o1 = corner(2, -1, low), o2 = corner(2, 1, low), o3 = corner(2, 1, high),
              o4 = corner(2, -1, high);
    const int i1 = corner(1.5, -1, low), i2 = corner(1.5, 1, low), i3 = corner(1.5, 1, high),
              i4 = corner(1.5, -1, high);
    // Arcs, each from -a to +a.
    const int arcOuterLow = e.edge(o1, o2, circle(low, 2));
    const int arcOuterHigh = e.edge(o4, o3, circle(high, 2));
    const int arcInnerLow = e.edge(i1, i2, circle(low, 1.5));
    const int arcInnerHigh = e.edge(i4, i3, circle(high, 1.5));
    // Up the pocket's corners.
    const int upOuterMinus = e.edge(o1, o4, line(e.point(2 * c, -2 * s, low), 0, 0, 1));
    const int upOuterPlus = e.edge(o2, o3, line(e.point(2 * c, 2 * s, low), 0, 0, 1));
    const int upInnerMinus = e.edge(i1, i4, line(e.point(1.5 * c, -1.5 * s, low), 0, 0, 1));
    const int upInnerPlus = e.edge(i2, i3, line(e.point(1.5 * c, 1.5 * s, low), 0, 0, 1));
    // In from the side, from radius 2 to 1.5.
    const int inLowMinus = e.edge(o1, i1, line(e.point(2 * c, -2 * s, low), -c, s, 0));
    const int inLowPlus = e.edge(o2, i2, line(e.point(2 * c, 2 * s, low), -c, -s, 0));
    const int inHighMinus = e.edge(o4, i4, line(e.point(2 * c, -2 * s, high), -c, s, 0));
    const int inHighPlus = e.edge(o3, i3, line(e.point(2 * c, 2 * s, high), -c, -s, 0));

    const auto cylinderSurface = [&](double radius) {
        return e.add("CYLINDRICAL_SURFACE('',#" + std::to_string(e.placement(origin, z, x)) + "," +
                     std::to_string(radius) + ")");
    };
    const auto plane = [&](int at, int normal, int ref) {
        return e.add("PLANE('',#" + std::to_string(e.placement(at, normal, ref)) + ")");
    };
    std::vector<int> faces;
    // The side: round the cylinder, and the window, clockwise in (u, v).
    faces.push_back(e.face(
        {{{bottom, true}, {seam, true}, {top, false}, {seam, false}},
         {{upOuterMinus, true}, {arcOuterHigh, true}, {upOuterPlus, false}, {arcOuterLow, false}}},
        cylinderSurface(2), true));
    // The pocket's floor, facing out.
    faces.push_back(e.face(
        {{{arcInnerLow, true}, {upInnerPlus, true}, {arcInnerHigh, false}, {upInnerMinus, false}}},
        cylinderSurface(1.5), true));
    // Its low wall, facing up; its high wall, facing down.
    faces.push_back(e.face(
        {{{arcOuterLow, true}, {inLowPlus, true}, {arcInnerLow, false}, {inLowMinus, false}}},
        plane(e.point(0, 0, low), z, x), true));
    faces.push_back(e.face(
        {{{arcOuterHigh, false}, {inHighMinus, true}, {arcInnerHigh, true}, {inHighPlus, false}}},
        plane(e.point(0, 0, high), e.direction(0, 0, -1), x), true));
    // Its sides: at +a, facing towards -a; at -a, facing towards +a.
    faces.push_back(e.face(
        {{{inLowPlus, false}, {upOuterPlus, true}, {inHighPlus, true}, {upInnerPlus, false}}},
        plane(e.point(2 * c, 2 * s, low), e.direction(s, -c, 0), e.direction(c, s, 0)), true));
    faces.push_back(e.face(
        {{{upInnerMinus, true}, {inHighMinus, false}, {upOuterMinus, false}, {inLowMinus, true}}},
        plane(e.point(2 * c, -2 * s, low), e.direction(s, c, 0), e.direction(c, -s, 0)), true));
    // The caps.
    faces.push_back(e.face({{{bottom, false}}}, plane(origin, z, x), false));
    faces.push_back(e.face({{{top, true}}}, plane(e.point(0, 0, 4), z, x), true));
    std::string list;
    for (const int f : faces) list += (list.empty() ? "#" : ",#") + std::to_string(f);
    const int shell = e.add("CLOSED_SHELL('',(" + list + "))");
    e.add("MANIFOLD_SOLID_BREP('pocketed',#" + std::to_string(shell) + ")");
    return step(e.text());
}

}  // namespace

// A curved face with a hole in it (a cylinder's side, a pocket's window cut
// into it) is cut into facets round the hole, not left one flat facet with
// the hole ignored.
TEST(StepCurvedTest, ACurvedFaceWithAHoleIsCutRoundIt) {
    const auto m = measure(pocketedCylinder());
    EXPECT_EQ(m.outlined, 0u);
    expectRelative(m.modelled, 16.0 * kPi - 0.525, 0.01, "its facets, within 1 %");
    EXPECT_TRUE(m.ideal.exact);
    expectRelative(m.ideal.properties.volume, 16.0 * kPi - 0.525, 1e-8, "the pocketed cylinder");
}

// A hole over much of a curved face, cut finely: the points on the grid in
// the hole are not made (each was looked for in every triangle), and those
// between coarser ones, on their edges, cut them (each was left out).
TEST(StepCurvedTest, AWideHoleIsCutRoundFinely) {
    const auto solids = StepFormat::fromString(pocketedCylinder(1.4, 0.3, 3.7));
    ASSERT_EQ(solids.size(), 1u) << StepFormat::lastError();
    const double exact = 16.0 * kPi - 1.4 * 1.75 * 3.4;
    const auto coarse = hz::model::facetCurved(*solids[0]);
    const auto fine = hz::model::facetCurved(*solids[0], 0.05);
    ASSERT_NE(coarse.solid, nullptr) << coarse.error;
    ASSERT_NE(fine.solid, nullptr) << fine.error;
    EXPECT_TRUE(fine.solid->isValid()) << fine.solid->validationReport();
    EXPECT_TRUE(fine.outlined.empty());
    EXPECT_GT(fine.solid->faces().size(), 4 * coarse.solid->faces().size());
    const double coarseVolume = MassPropertiesCalculator::compute(*coarse.solid).volume;
    const double fineVolume = MassPropertiesCalculator::compute(*fine.solid).volume;
    expectRelative(fineVolume, exact, 1e-3, "its fine facets, within 0.1 %");
    EXPECT_LT(std::abs(fineVolume - exact), std::abs(coarseVolume - exact)) << "nearer when finer";
}
