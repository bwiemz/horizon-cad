// STEP solids come in exact (stage 2 of the exact kernel): each face on its file's own surface,
// a plane, cylinder, cone, sphere or torus as itself and a B-spline as that B-spline; each edge
// on its own curve, over the range its vertices give it; and each half-edge's course in its
// face's (u, v) built here, never read. A solid whose exact form does not hold together is left
// out, and the report says why: nothing faceted takes its place.

#include <gtest/gtest.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "StepTestText.h"
#include "horizon/fileio/ImportReport.h"
#include "horizon/fileio/StepFormat.h"
#include "horizon/geometry/Frame.h"
#include "horizon/geometry/curves/AnalyticCurves.h"
#include "horizon/geometry/curves/Curve2D.h"
#include "horizon/geometry/curves/NurbsCurve.h"
#include "horizon/geometry/surfaces/AnalyticSurfaces.h"
#include "horizon/geometry/surfaces/NurbsSurface.h"
#include "horizon/math/Constants.h"
#include "horizon/math/Mat4.h"
#include "horizon/math/Vec3.h"
#include "horizon/modeling/BoundaryMesh.h"
#include "horizon/modeling/ExactPrimitives.h"
#include "horizon/modeling/Faceting.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/modeling/Pattern.h"
#include "horizon/modeling/PrimitiveFactory.h"
#include "horizon/topology/ExactTolerance.h"
#include "horizon/topology/ExactValidator.h"
#include "horizon/topology/Solid.h"

using hz::io::ImportReport;
using hz::io::StepFormat;
using hz::math::Vec3;
using hz::step_text::bandCylinder;
using hz::step_text::block;
using hz::step_text::blockWithABall;
using hz::step_text::cone;
using hz::step_text::Data;
using hz::step_text::externalStyleCylinder;
using hz::step_text::real;
using hz::step_text::seamedHemisphere;
using hz::step_text::sphere;
using hz::step_text::splineCylinder;
using hz::step_text::torus;

namespace {

namespace geo = hz::geo;
namespace topo = hz::topo;

/// These files name no length unit, as hand-written ones often do: read in millimetres.
hz::io::StepReadOptions inMillimetres() {
    hz::io::StepReadOptions options;
    options.unknownLengthUnit = hz::math::LengthUnit::Millimetre;
    return options;
}

/// @p text with @p from replaced by @p to, which it must hold once.
std::string replaced(std::string text, const std::string& from, const std::string& to) {
    const auto at = text.find(from);
    EXPECT_NE(at, std::string::npos) << from;
    if (at != std::string::npos) text.replace(at, from.size(), to);
    return text;
}

/// @p entities added to the end of @p text's DATA section.
std::string withEntities(const std::string& text, const std::string& entities) {
    return replaced(text, "ENDSEC;\nEND-ISO-10303-21;", entities + "ENDSEC;\nEND-ISO-10303-21;");
}

/// A context in millimetres that gives its lengths' uncertainty as @p uncertainty mm: the one the
/// file's solids, outside any representation, are read in.
std::string millimetresWithin(const std::string& uncertainty) {
    return "#900 = (LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.));\n"
           "#901 = UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(" +
           uncertainty +
           "),#900,'distance_accuracy_value','');\n"
           "#902 = (GEOMETRIC_REPRESENTATION_CONTEXT(3) "
           "GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#901)) GLOBAL_UNIT_ASSIGNED_CONTEXT((#900)) "
           "REPRESENTATION_CONTEXT('',''));\n";
}

/// A context in inches.
std::string inches() {
    return "#900 = (LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.));\n"
           "#901 = LENGTH_MEASURE_WITH_UNIT(LENGTH_MEASURE(25.4),#900);\n"
           "#902 = (CONVERSION_BASED_UNIT('INCH',#901) LENGTH_UNIT() NAMED_UNIT(*));\n"
           "#903 = (GEOMETRIC_REPRESENTATION_CONTEXT(3) GLOBAL_UNIT_ASSIGNED_CONTEXT((#902)) "
           "REPRESENTATION_CONTEXT('',''));\n";
}

/// The entities of @p text's DATA section, each #id moved up by @p by: a second solid beside it.
std::string renumbered(const std::string& text, int by) {
    const auto from = text.find("DATA;\n") + 6;
    const auto to = text.rfind("ENDSEC;");
    std::string out;
    for (std::size_t i = from; i < to; ++i) {
        out += text[i];
        if (text[i] != '#') continue;
        std::size_t end = i + 1;
        while (end < to && std::isdigit(static_cast<unsigned char>(text[end])) != 0) ++end;
        out += std::to_string(std::stoi(text.substr(i + 1, end - i - 1)) + by);
        i = end - 1;
    }
    return out;
}

/// The faces of @p solid, each with its exact surface: none may lack one.
std::vector<const topo::Face*> exactFaces(const topo::Solid& solid) {
    std::vector<const topo::Face*> faces;
    for (const auto& f : solid.faces()) {
        EXPECT_NE(f.exactSurface, nullptr) << f.topoId.tag();
        if (f.exactSurface) faces.push_back(&f);
    }
    return faces;
}

/// The face of @p solid whose surface is of @p kind; null when there is not exactly one.
const topo::Face* faceOfKind(const topo::Solid& solid, geo::SurfaceKind kind) {
    const topo::Face* found = nullptr;
    for (const topo::Face* f : exactFaces(solid)) {
        if (f->exactSurface->kind() != kind) continue;
        if (found != nullptr) return nullptr;
        found = f;
    }
    return found;
}

/// @p face's outward normal at its point nearest @p p.
Vec3 outward(const topo::Face& face, const Vec3& p) {
    const auto uv = face.exactSurface->invert(p);
    const Vec3 n = face.exactSurface->normal(uv.x, uv.y);
    return face.reversed ? n * -1.0 : n;
}

/// The first of @p solid's exact issues, or "" when it has none.
std::string firstIssue(const topo::Solid& solid) {
    const auto issues = topo::validateExact(solid, topo::exactTolerance(solid));
    return issues.empty() ? std::string() : issues.front().detail;
}

/// Every edge of @p solid has its exact curve, and every half-edge its pcurve.
void expectExactEdges(const topo::Solid& solid) {
    for (const auto& e : solid.edges()) {
        EXPECT_NE(e.exactCurve, nullptr) << e.topoId.tag();
        EXPECT_NE(e.curve, nullptr) << "its carrier stays: " << e.topoId.tag();
    }
    for (const auto& f : solid.faces()) {
        std::vector<const topo::Wire*> loops{f.outerLoop};
        loops.insert(loops.end(), f.innerLoops.begin(), f.innerLoops.end());
        for (const topo::Wire* w : loops) {
            const topo::HalfEdge* h = w->halfEdge;
            do {
                EXPECT_NE(h->pcurve, nullptr) << f.topoId.tag();
                h = h->next;
            } while (h != w->halfEdge);
        }
    }
}

double largestEdgeTolerance(const topo::Solid& solid) {
    double largest = 0.0;
    for (const auto& e : solid.edges()) largest = std::max(largest, e.tolerance);
    return largest;
}

bool anySays(const std::vector<std::string>& lines, const std::string& what) {
    return std::any_of(lines.begin(), lines.end(), [&](const std::string& line) {
        return line.find(what) != std::string::npos;
    });
}

}  // namespace

// An OpenCASCADE cylinder: its side on its CYLINDRICAL_SURFACE, its caps on their PLANEs, each
// as the file gives it; its seam used twice by its side, which lays out clean; the read point
// meshes it, and its volume, integrated on its exact faces, is the cylinder's.
TEST(StepExact, ACylinderComesInAsThreeExactFaces) {
    const auto solids =
        StepFormat::fromString(externalStyleCylinder(), nullptr, nullptr, inMillimetres());
    ASSERT_EQ(solids.size(), 1u) << StepFormat::lastError();
    const topo::Solid& cyl = *solids[0];

    std::multiset<geo::SurfaceKind> kinds;
    for (const topo::Face* f : exactFaces(cyl)) {
        kinds.insert(f->exactSurface->kind());
        EXPECT_NE(f->surface, nullptr) << "its carrier stays";
    }
    EXPECT_EQ(kinds,
              (std::multiset<geo::SurfaceKind>{geo::SurfaceKind::Plane, geo::SurfaceKind::Cylinder,
                                               geo::SurfaceKind::Plane}));
    const topo::Face* side = faceOfKind(cyl, geo::SurfaceKind::Cylinder);
    ASSERT_NE(side, nullptr);
    const auto* cylinder = dynamic_cast<const geo::Cylinder*>(side->exactSurface.get());
    ASSERT_NE(cylinder, nullptr);
    EXPECT_DOUBLE_EQ(cylinder->radius(), 1.0);
    EXPECT_DOUBLE_EQ(cylinder->axialScale(), 1.0);
    EXPECT_NEAR(cylinder->frame().origin.length(), 0.0, 1e-15);
    EXPECT_NEAR((cylinder->frame().z - Vec3{0, 0, 1}).length(), 0.0, 1e-15);
    EXPECT_NEAR((cylinder->frame().x - Vec3{1, 0, 0}).length(), 0.0, 1e-15);
    expectExactEdges(cyl);
    EXPECT_EQ(firstIssue(cyl), "");

    std::string problem;
    const auto polygons = hz::model::BoundaryMesh::extractFacePolygons(cyl, 1e-3, &problem);
    EXPECT_EQ(problem, "");
    EXPECT_FALSE(polygons.empty());
    const double volume = hz::model::MassPropertiesCalculator::compute(cyl).volume;
    EXPECT_NEAR(volume, 2.0 * hz::math::kPi, 2.0 * hz::math::kPi * 1e-9);
}

// A cone, a sphere and a torus each come in on their own surface; a cylinder written as a
// B-spline comes in as that B-spline, never recognised as the quadric it is.
TEST(StepExact, EachSurfaceKindKeepsItsKind) {
    struct Case {
        const char* what;
        std::string text;
        geo::SurfaceKind kind;
        /// Whether the read point meshes it: not yet a face that comes to a point (an apex, a
        /// pole), which stage 3 takes on.
        bool meshes;
    };
    const Case cases[] = {
        {"a cone", cone(), geo::SurfaceKind::Cone, false},
        {"a sphere", sphere(), geo::SurfaceKind::Sphere, false},
        {"a torus", torus(), geo::SurfaceKind::Torus, true},
        {"a B-spline", splineCylinder(), geo::SurfaceKind::Nurbs, true},
    };
    for (const Case& c : cases) {
        SCOPED_TRACE(c.what);
        const auto solids = StepFormat::fromString(c.text, nullptr, nullptr, inMillimetres());
        ASSERT_EQ(solids.size(), 1u) << StepFormat::lastError();
        const topo::Face* face = faceOfKind(*solids[0], c.kind);
        ASSERT_NE(face, nullptr);
        expectExactEdges(*solids[0]);
        EXPECT_EQ(firstIssue(*solids[0]), "");
        if (c.meshes) {
            std::string problem;
            hz::model::BoundaryMesh::extractFacePolygons(*solids[0], 1e-3, &problem);
            EXPECT_EQ(problem, "");
        }
    }

    const auto solids = StepFormat::fromString(splineCylinder(), nullptr, nullptr, inMillimetres());
    ASSERT_EQ(solids.size(), 1u) << StepFormat::lastError();
    const topo::Face* side = faceOfKind(*solids[0], geo::SurfaceKind::Nurbs);
    ASSERT_NE(side, nullptr);
    EXPECT_EQ(dynamic_cast<const geo::Cylinder*>(side->exactSurface.get()), nullptr);
    const auto* spline = dynamic_cast<const geo::NurbsSurface*>(side->exactSurface.get());
    ASSERT_NE(spline, nullptr);
    EXPECT_EQ(spline->degreeU(), 2);
    EXPECT_EQ(spline->degreeV(), 1);
    ASSERT_EQ(spline->controlPoints().size(), 9u) << "the file's own net";
    EXPECT_NEAR(spline->weights()[1][0], std::sqrt(0.5), 1e-15);
}

// The bottom cap's PLANE faces into the cylinder and its face says so (same_sense .F.): the
// face is reversed, its surface kept as the file gives it, and every face's normal points out.
TEST(StepExact, ASurfaceFacingAgainstItsFaceIsReversed) {
    const auto solids =
        StepFormat::fromString(externalStyleCylinder(), nullptr, nullptr, inMillimetres());
    ASSERT_EQ(solids.size(), 1u) << StepFormat::lastError();
    int caps = 0;
    for (const topo::Face* f : exactFaces(*solids[0])) {
        const Vec3 p = f->outerLoop->halfEdge->origin->point;
        if (f->exactSurface->kind() == geo::SurfaceKind::Cylinder) {
            EXPECT_FALSE(f->reversed);
            EXPECT_GT(outward(*f, p).dot(Vec3{p.x, p.y, 0.0}), 0.99);
            continue;
        }
        const auto* plane = dynamic_cast<const geo::Plane*>(f->exactSurface.get());
        ASSERT_NE(plane, nullptr);
        EXPECT_NEAR((plane->frame().z - Vec3{0, 0, 1}).length(), 0.0, 1e-15) << "as written";
        const bool bottom = p.z < 1.0;
        EXPECT_EQ(f->reversed, bottom);
        EXPECT_NEAR(outward(*f, p).z, bottom ? -1.0 : 1.0, 1e-12);
        ++caps;
    }
    EXPECT_EQ(caps, 2);
    EXPECT_EQ(firstIssue(*solids[0]), "");
}

// Review Focus 1: another system's edges stand off their surfaces within the uncertainty its
// file gives. Rims 5e-6 off the cylinder, in a file of 1e-5: each edge's tolerance grows to
// cover its gap, the solid is clean, and the report says how exact it is.
TEST(StepExact, EdgesOffTheirSurfaceWithinTheFileTolerance) {
    const std::string text =
        withEntities(replaced(externalStyleCylinder(), "#20 = CYLINDRICAL_SURFACE('',#19,1.);",
                              "#20 = CYLINDRICAL_SURFACE('',#19,1.000005);"),
                     millimetresWithin("1.E-5"));
    ImportReport report;
    const auto solids = StepFormat::fromString(text, &report);
    ASSERT_EQ(solids.size(), 1u) << StepFormat::lastError();
    EXPECT_TRUE(report.skipped.empty()) << report.skipped.front();
    const double largest = largestEdgeTolerance(*solids[0]);
    EXPECT_GE(largest, 5e-6);
    EXPECT_LE(largest, 1e-5) << "within the file's uncertainty";
    expectExactEdges(*solids[0]);
    EXPECT_EQ(firstIssue(*solids[0]), "");
    EXPECT_TRUE(anySays(report.converted, "exact to")) << report.converted.size();
    std::string problem;
    hz::model::BoundaryMesh::extractFacePolygons(*solids[0], 1e-3, &problem);
    EXPECT_EQ(problem, "");
}

// Review Focus 3: a file in inches comes in in millimetres, its exact geometry with it: the
// cylinder's radius and frame, the plane's origin, and the seam LINE's magnitude, each 25.4
// times the file's.
TEST(StepExact, AFileInInchesIsScaledExactly) {
    const std::string text = withEntities(
        replaced(externalStyleCylinder(), "VECTOR('',#2,1.)", "VECTOR('',#2,2.)"), inches());
    const auto solids = StepFormat::fromString(text);
    ASSERT_EQ(solids.size(), 1u) << StepFormat::lastError();
    const topo::Solid& cyl = *solids[0];

    const topo::Face* side = faceOfKind(cyl, geo::SurfaceKind::Cylinder);
    ASSERT_NE(side, nullptr);
    const auto* cylinder = dynamic_cast<const geo::Cylinder*>(side->exactSurface.get());
    ASSERT_NE(cylinder, nullptr);
    EXPECT_NEAR(cylinder->radius(), 25.4, 1e-12);
    EXPECT_NEAR((cylinder->frame().z - Vec3{0, 0, 1}).length(), 0.0, 1e-15);

    bool sawTop = false;
    for (const topo::Face* f : exactFaces(cyl)) {
        const auto* plane = dynamic_cast<const geo::Plane*>(f->exactSurface.get());
        if (plane != nullptr && plane->frame().origin.z > 1.0) {
            EXPECT_NEAR((plane->frame().origin - Vec3{0, 0, 50.8}).length(), 0.0, 1e-12);
            sawTop = true;
        }
    }
    EXPECT_TRUE(sawTop);

    const geo::LineCurve* seam = nullptr;
    for (const auto& e : cyl.edges()) {
        if (const auto* line = dynamic_cast<const geo::LineCurve*>(e.exactCurve.get())) {
            seam = line;
        }
    }
    ASSERT_NE(seam, nullptr);
    EXPECT_NEAR(seam->scale(), 2.0 * 25.4, 1e-12) << "its magnitude, in millimetres";
    EXPECT_NEAR((seam->origin() - Vec3{25.4, 0, 0}).length(), 0.0, 1e-12);
    EXPECT_NEAR((seam->evaluate(1.0) - Vec3{25.4, 0, 50.8}).length(), 0.0, 1e-12);
    EXPECT_EQ(firstIssue(cyl), "");
}

// Review Focus 4: a void's faces face out of the material, into the void, so its ball's
// normals point at its centre; the block's point out, and the ball's volume subtracts from the
// block's (integrated on the exact faces: the read point cannot mesh a face that comes to a pole
// yet).
TEST(StepExact, AVoidShellIsInward) {
    const auto solids = StepFormat::fromString(blockWithABall(), nullptr, nullptr, inMillimetres());
    ASSERT_EQ(solids.size(), 1u) << StepFormat::lastError();
    const topo::Solid& block = *solids[0];
    EXPECT_EQ(block.shells().size(), 2u);

    const topo::Face* ball = faceOfKind(block, geo::SurfaceKind::Sphere);
    ASSERT_NE(ball, nullptr);
    EXPECT_TRUE(ball->reversed);
    const Vec3 centre{2, 2, 2};
    for (const Vec3& p : {Vec3{3, 2, 2}, Vec3{2, 1, 2}, Vec3{2, 2, 2.9}}) {
        EXPECT_LT(outward(*ball, p).dot(p - centre), -0.5) << "toward the ball's centre";
    }
    for (const topo::Face* f : exactFaces(block)) {
        if (f == ball) continue;
        const Vec3 p = f->outerLoop->halfEdge->origin->point;
        const Vec3 middle = (p + f->outerLoop->halfEdge->next->next->origin->point) * 0.5;
        EXPECT_GT(outward(*f, middle).dot(middle - centre), 0.0) << "out of the block";
    }
    expectExactEdges(block);
    EXPECT_EQ(firstIssue(block), "");
    const double volume = 64.0 - 4.0 / 3.0 * hz::math::kPi;
    EXPECT_NEAR(hz::model::MassPropertiesCalculator::compute(block).volume, volume, volume * 1e-9);
}

// A side whose rims lie a hundredth off its cylinder, in a file of a micrometre's uncertainty,
// cannot be made exact: that solid is left out and the report says why, and nothing faceted
// comes in for it. The file's other solid still comes in.
TEST(StepExact, ASolidThatCannotBeMadeExactIsLeftOutWithWhy) {
    const std::string off =
        replaced(externalStyleCylinder(), "#20 = CYLINDRICAL_SURFACE('',#19,1.);",
                 "#20 = CYLINDRICAL_SURFACE('',#19,1.01);");
    const std::string text =
        withEntities(off, renumbered(externalStyleCylinder(), 1000) + millimetresWithin("1.E-6"));
    ImportReport report;
    const auto solids = StepFormat::fromString(text, &report);
    ASSERT_EQ(solids.size(), 1u) << StepFormat::lastError();
    EXPECT_EQ(firstIssue(*solids[0]), "") << "the other one comes in exact";
    ASSERT_EQ(report.skipped.size(), 1u);
    const std::string& why = report.skipped.front();
    EXPECT_EQ(why.rfind("solid 1: ", 0), 0u) << why;
    EXPECT_NE(why.find("beyond the limit"), std::string::npos) << "and says why: " << why;
}

// Ruling 9: the file's uncertainty is a floor, not a ceiling. Rims 5e-5 off their cylinder, in a
// file that says 1e-5, still come in: each edge's tolerance grows to cover its gap, as an
// OpenCASCADE reader's per-edge tolerances do, up to a thousandth of the solid's size; the
// solid is clean, and the report says how exact it is.
TEST(StepExact, EdgesBeyondTheFileUncertaintyGrowAndAreReported) {
    const std::string text =
        withEntities(replaced(externalStyleCylinder(), "#20 = CYLINDRICAL_SURFACE('',#19,1.);",
                              "#20 = CYLINDRICAL_SURFACE('',#19,1.00005);"),
                     millimetresWithin("1.E-5"));
    ImportReport report;
    const auto solids = StepFormat::fromString(text, &report);
    ASSERT_EQ(solids.size(), 1u) << StepFormat::lastError();
    EXPECT_TRUE(report.skipped.empty()) << report.skipped.front();
    EXPECT_GE(largestEdgeTolerance(*solids[0]), 5e-5);
    expectExactEdges(*solids[0]);
    EXPECT_EQ(firstIssue(*solids[0]), "");
    EXPECT_TRUE(anySays(report.converted, "solid 1: exact to ")) << report.converted.size();
    std::string problem;
    hz::model::BoundaryMesh::extractFacePolygons(*solids[0], 1e-3, &problem);
    EXPECT_EQ(problem, "");
}

// Ruling 11: a face's sense comes from its loops. A block whose front face's plane faces into it,
// its face saying same_sense .T. regardless (as Horizon's own writer wrote faces), comes in with
// that face reversed and every face's normal pointing out of the block.
TEST(StepExact, AFaceWhoseSurfaceFacesInIsTurnedByItsLoops) {
    const auto solids = StepFormat::fromString(block(true), nullptr, nullptr, inMillimetres());
    ASSERT_EQ(solids.size(), 1u) << StepFormat::lastError();
    const Vec3 centre{2, 2, 2};
    int turned = 0;
    for (const topo::Face* f : exactFaces(*solids[0])) {
        const Vec3 p = f->outerLoop->halfEdge->origin->point;
        const Vec3 middle = (p + f->outerLoop->halfEdge->next->next->origin->point) * 0.5;
        EXPECT_GT(outward(*f, middle).dot(middle - centre), 0.0) << "out of the block";
        if (f->reversed) ++turned;
    }
    EXPECT_EQ(turned, 1) << "the front face alone";
    EXPECT_EQ(firstIssue(*solids[0]), "");
}

// ---------------------------------------------------------------------------
// Fix round 1
// ---------------------------------------------------------------------------

namespace {

/// Why a face on a sphere, a torus or a B-spline with a pole or closed both ways, whose loop runs
/// against its same_sense, is left out: it cannot tell which is wrong.
constexpr const char* kAgainstItsLoops =
    "its loops run against its same_sense: either the face is the surface outside its loop (it "
    "holds a pole or wraps the surface), which this stage cannot represent, or the file's "
    "same_sense is wrong";

/// A unit ball with a flat at x = 0.5, written consistently, as Parasolid-based writers write
/// one: the sphere's face bounded by the flat's circle alone, used .F., with no seam (the
/// reviewer's probe).
std::string ballWithAFlat() {
    return hz::step_text::stepFile(R"(#1 = CARTESIAN_POINT('',(0.,0.,0.));
#2 = DIRECTION('',(0.,0.,1.));
#3 = DIRECTION('',(1.,0.,0.));
#4 = AXIS2_PLACEMENT_3D('',#1,#2,#3);
#5 = SPHERICAL_SURFACE('',#4,1.);
#6 = CARTESIAN_POINT('',(0.5,0.,0.));
#8 = AXIS2_PLACEMENT_3D('',#6,#3,#2);
#9 = CIRCLE('',#8,0.8660254037844386);
#10 = CARTESIAN_POINT('',(0.5,0.,0.8660254037844386));
#11 = VERTEX_POINT('',#10);
#12 = EDGE_CURVE('',#11,#11,#9,.T.);
#13 = ORIENTED_EDGE('',*,*,#12,.F.);
#14 = EDGE_LOOP('',(#13));
#15 = FACE_OUTER_BOUND('',#14,.T.);
#16 = ADVANCED_FACE('',(#15),#5,.T.);
#17 = PLANE('',#8);
#18 = ORIENTED_EDGE('',*,*,#12,.T.);
#19 = EDGE_LOOP('',(#18));
#20 = FACE_OUTER_BOUND('',#19,.T.);
#21 = ADVANCED_FACE('',(#20),#17,.T.);
#22 = CLOSED_SHELL('',(#16,#21));
#23 = MANIFOLD_SOLID_BREP('ball with a flat',#22);
)");
}

/// A torus of radii 4 and 1 cut into two faces along four arcs (two parallels, two meridians)
/// round its patch u, v in [0, pi/2]: the patch, and the rest of the torus, whose loop runs
/// round the patch the other way. Written consistently, both faces .T.
std::string torusPatchAndTheRest() {
    return hz::step_text::stepFile(R"(#1 = CARTESIAN_POINT('',(0.,0.,0.));
#2 = DIRECTION('',(0.,0.,1.));
#3 = DIRECTION('',(1.,0.,0.));
#4 = DIRECTION('',(0.,1.,0.));
#5 = DIRECTION('',(0.,-1.,0.));
#6 = AXIS2_PLACEMENT_3D('',#1,#2,#3);
#7 = TOROIDAL_SURFACE('',#6,4.,1.);
#10 = CARTESIAN_POINT('',(5.,0.,0.));
#11 = CARTESIAN_POINT('',(0.,5.,0.));
#12 = CARTESIAN_POINT('',(0.,4.,1.));
#13 = CARTESIAN_POINT('',(4.,0.,1.));
#14 = VERTEX_POINT('',#10);
#15 = VERTEX_POINT('',#11);
#16 = VERTEX_POINT('',#12);
#17 = VERTEX_POINT('',#13);
#20 = CIRCLE('',#6,5.);
#21 = CARTESIAN_POINT('',(0.,4.,0.));
#22 = AXIS2_PLACEMENT_3D('',#21,#3,#4);
#23 = CIRCLE('',#22,1.);
#24 = CARTESIAN_POINT('',(0.,0.,1.));
#25 = AXIS2_PLACEMENT_3D('',#24,#2,#3);
#26 = CIRCLE('',#25,4.);
#27 = CARTESIAN_POINT('',(4.,0.,0.));
#28 = AXIS2_PLACEMENT_3D('',#27,#5,#3);
#29 = CIRCLE('',#28,1.);
#30 = EDGE_CURVE('',#14,#15,#20,.T.);
#31 = EDGE_CURVE('',#15,#16,#23,.T.);
#32 = EDGE_CURVE('',#17,#16,#26,.T.);
#33 = EDGE_CURVE('',#14,#17,#29,.T.);
#40 = ORIENTED_EDGE('',*,*,#30,.T.);
#41 = ORIENTED_EDGE('',*,*,#31,.T.);
#42 = ORIENTED_EDGE('',*,*,#32,.F.);
#43 = ORIENTED_EDGE('',*,*,#33,.F.);
#44 = EDGE_LOOP('',(#40,#41,#42,#43));
#45 = FACE_OUTER_BOUND('',#44,.T.);
#46 = ADVANCED_FACE('',(#45),#7,.T.);
#50 = ORIENTED_EDGE('',*,*,#33,.T.);
#51 = ORIENTED_EDGE('',*,*,#32,.T.);
#52 = ORIENTED_EDGE('',*,*,#31,.F.);
#53 = ORIENTED_EDGE('',*,*,#30,.F.);
#54 = EDGE_LOOP('',(#50,#51,#52,#53));
#55 = FACE_OUTER_BOUND('',#54,.T.);
#56 = ADVANCED_FACE('',(#55),#7,.T.);
#60 = CLOSED_SHELL('',(#46,#56));
#61 = MANIFOLD_SOLID_BREP('torus in two',#60);
)");
}

/// The nine control points (CCW about +z from (cos a, sin a, @p z), r = 1) of the rational
/// quadratic circle, as CARTESIAN_POINTs from #@p first, and the curve as #@p id: a closed
/// B-spline whose join is at angle @p a.
std::string splineCircle(int id, int first, double a, double z = 0.0) {
    const double w = std::sqrt(0.5);
    const double xs[9] = {1, 1, 0, -1, -1, -1, 0, 1, 1};
    const double ys[9] = {0, 1, 1, 1, 0, -1, -1, -1, 0};
    std::string text;
    std::string points;
    std::string weights;
    for (int i = 0; i < 9; ++i) {
        const double x = xs[i] * std::cos(a) - ys[i] * std::sin(a);
        const double y = xs[i] * std::sin(a) + ys[i] * std::cos(a);
        text += "#" + std::to_string(first + i) + " = CARTESIAN_POINT('',(" + real(x) + "," +
                real(y) + "," + real(z) + "));\n";
        points += std::string(i == 0 ? "" : ",") + "#" + std::to_string(first + i);
        weights += std::string(i == 0 ? "" : ",") + real(i % 2 == 1 ? w : 1.0);
    }
    return text + "#" + std::to_string(id) + " = ( BOUNDED_CURVE() B_SPLINE_CURVE(2,(" + points +
           "),.CIRCULAR_ARC.,.T.,.F.) B_SPLINE_CURVE_WITH_KNOTS((3,2,2,2,3),"
           "(0.,0.25,0.5,0.75,1.),.UNSPECIFIED.) CURVE() GEOMETRIC_REPRESENTATION_ITEM() "
           "RATIONAL_B_SPLINE_CURVE((" +
           weights + ")) REPRESENTATION_ITEM('') );\n";
}

/// externalStyleCylinder() with its bottom rim two edges, each half of a closed B-spline circle
/// joined at (1, 0, 0), meeting at a new vertex at (-1, 0, 0).
std::string cylinderWithASplitSplineRim() {
    std::string text = replaced(externalStyleCylinder(), "#14 = EDGE_CURVE('',#7,#7,#5,.T.);\n",
                                splineCircle(300, 310, 0.0) +
                                    "#301 = CARTESIAN_POINT('',(-1.,0.,0.));\n"
                                    "#302 = VERTEX_POINT('',#301);\n"
                                    "#14 = EDGE_CURVE('',#7,#302,#300,.T.);\n"
                                    "#303 = EDGE_CURVE('',#302,#7,#300,.T.);\n"
                                    "#304 = ORIENTED_EDGE('',*,*,#303,.T.);\n"
                                    "#305 = ORIENTED_EDGE('',*,*,#303,.F.);\n");
    text = replaced(text, "#25 = EDGE_LOOP('',(#21,#22,#23,#24));",
                    "#25 = EDGE_LOOP('',(#21,#304,#22,#23,#24));");
    return replaced(text, "#37 = EDGE_LOOP('',(#36));", "#37 = EDGE_LOOP('',(#305,#36));");
}

/// externalStyleCylinder() with its top cap a flat bilinear B-spline patch over the rim, facing up:
/// the rim's course on it is a circle in (u, v), which the general path fits, as closely as asked.
std::string splineCappedCylinder() {
    return replaced(externalStyleCylinder(), "#29 = PLANE('',#28);",
                    "#320 = CARTESIAN_POINT('',(-1.5,-1.5,2.));\n"
                    "#321 = CARTESIAN_POINT('',(-1.5,1.5,2.));\n"
                    "#322 = CARTESIAN_POINT('',(1.5,-1.5,2.));\n"
                    "#323 = CARTESIAN_POINT('',(1.5,1.5,2.));\n"
                    "#29 = B_SPLINE_SURFACE_WITH_KNOTS('',1,1,((#320,#321),(#322,#323)),"
                    ".UNSPECIFIED.,.F.,.F.,.F.,(2,2),(2,2),(0.,1.),(0.,1.),.UNSPECIFIED.);");
}

}  // namespace

// Ruling 17: on a sphere a loop bounds two regions, and its (u, v) area always picks the one that
// holds no pole. A ball with a flat, its skin the sphere outside the flat's circle, is that other
// region: this stage cannot represent it, so the solid is left out with why. It never comes in
// turned inside out as the cap (whose volume, 0.654, read for the ball's 3.534).
TEST(StepExact, ABallWithAFlatIsNeverTurnedInsideOut) {
    const auto solids = StepFormat::fromString(ballWithAFlat(), nullptr, nullptr, inMillimetres());
    EXPECT_TRUE(solids.empty()) << "it came in";
    EXPECT_NE(StepFormat::lastError().find(kAgainstItsLoops + std::string(" (face #16)")),
              std::string::npos)
        << StepFormat::lastError();
}

// The same on a torus: cut into its patch u, v in [0, pi/2] and the rest, the rest is the torus
// outside its loop, which this stage cannot represent. Its patch alone would be read.
TEST(StepExact, ATorusFaceOutsideItsLoopIsNeverTurnedInsideOut) {
    const auto solids =
        StepFormat::fromString(torusPatchAndTheRest(), nullptr, nullptr, inMillimetres());
    EXPECT_TRUE(solids.empty()) << "it came in";
    EXPECT_NE(StepFormat::lastError().find(kAgainstItsLoops + std::string(" (face #56)")),
              std::string::npos)
        << StepFormat::lastError();
}

// Ruling 18: a clean file with a loose UNCERTAINTY (0.01) is fitted to the model resolution, not
// to its uncertainty: its edges' tolerances stay at the resolution and nothing says it is exact
// only to the file's. Its top a B-spline patch, the rim's pcurve on it a circle in (u, v), which
// the general path fitted to the file's 0.01 had stray 1e-4 or so.
TEST(StepExact, ACleanFileWithALooseUncertaintyIsFitToTheResolution) {
    ImportReport report;
    const auto solids = StepFormat::fromString(
        withEntities(splineCappedCylinder(), millimetresWithin("0.01")), &report);
    ASSERT_EQ(solids.size(), 1u) << StepFormat::lastError();
    const double resolution = topo::exactTolerance(*solids[0]).linear;
    for (const auto& e : solids[0]->edges()) {
        EXPECT_LE(e.tolerance, resolution) << e.topoId.tag();
    }
    EXPECT_FALSE(anySays(report.converted, "exact to")) << report.converted.front();
    EXPECT_EQ(firstIssue(*solids[0]), "");
}

// Minor (a): a cylinder of radius 1e300 in a file in exametres overflows when scaled into
// millimetres. The geometry's constructor throws; the read says why, and throws nothing.
TEST(StepExact, AScaleIntoMillimetresThatOverflowsIsReportedNotThrown) {
    const std::string text = withEntities(
        replaced(externalStyleCylinder(), "#20 = CYLINDRICAL_SURFACE('',#19,1.);",
                 "#20 = CYLINDRICAL_SURFACE('',#19,1.E300);"),
        "#900 = (LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.EXA.,.METRE.));\n"
        "#902 = (GEOMETRIC_REPRESENTATION_CONTEXT(3) GLOBAL_UNIT_ASSIGNED_CONTEXT((#900)) "
        "REPRESENTATION_CONTEXT('',''));\n");
    std::vector<std::unique_ptr<topo::Solid>> solids;
    EXPECT_NO_THROW(solids = StepFormat::fromString(text));
    EXPECT_TRUE(solids.empty());
    EXPECT_NE(StepFormat::lastError().find("failed to reconstruct solid #41"), std::string::npos)
        << StepFormat::lastError();
}

// Minor (b): an edge on part of a closed B-spline curve, in a file whose UNCERTAINTY (5) is
// larger than the curve: its far vertex is not taken for the curve's join, so each edge keeps its
// half of the curve.
TEST(StepExact, AnArcOfAClosedSplineKeepsItsHalfInALooseFile) {
    ImportReport report;
    const auto solids = StepFormat::fromString(
        withEntities(cylinderWithASplitSplineRim(), millimetresWithin("5.")), &report);
    ASSERT_EQ(solids.size(), 1u) << StepFormat::lastError();
    int halves = 0;
    for (const auto& e : solids[0]->edges()) {
        if (dynamic_cast<const geo::NurbsCurve*>(e.exactCurve.get()) == nullptr) continue;
        EXPECT_NEAR(std::abs(e.curveEnd - e.curveStart), 0.5, 1e-9) << e.topoId.tag();
        ++halves;
    }
    EXPECT_EQ(halves, 2);
    EXPECT_EQ(firstIssue(*solids[0]), "");
}

// Minor (c): a closed edge on a closed B-spline curve whose vertex is not at the curve's join has
// no one range: it says so, not that its curve ends off its vertex.
TEST(StepExact, AClosedSplineEdgeWhoseVertexIsOffItsJoinSaysSo) {
    const std::string text = replaced(
        externalStyleCylinder(), "#14 = EDGE_CURVE('',#7,#7,#5,.T.);",
        splineCircle(300, 310, hz::math::kPi / 2) + "#14 = EDGE_CURVE('',#7,#7,#300,.T.);");
    EXPECT_TRUE(StepFormat::fromString(text, nullptr, nullptr, inMillimetres()).empty());
    EXPECT_NE(StepFormat::lastError().find(
                  "its vertex is off where its closed B-spline curve joins (edge #14)"),
              std::string::npos)
        << StepFormat::lastError();
}

namespace {

/// A cylinder whose bottom rim is two half circles, one .T. across its circle's t = 0, one .F. on
/// a circle about -z, and whose top rim is one closed edge .F. on a circle about -z (the
/// reviewer's probe).
std::string cylinderWithSplitRims() {
    return hz::step_text::stepFile(R"(#1 = CARTESIAN_POINT('',(0.,0.,0.));
#2 = DIRECTION('',(0.,0.,1.));
#3 = DIRECTION('',(0.,0.,-1.));
#4 = DIRECTION('',(1.,0.,0.));
#5 = DIRECTION('',(0.,1.,0.));
#6 = CARTESIAN_POINT('',(0.,0.,2.));
#10 = AXIS2_PLACEMENT_3D('',#1,#2,#5);
#11 = CIRCLE('',#10,1.);
#12 = AXIS2_PLACEMENT_3D('',#1,#3,#4);
#13 = CIRCLE('',#12,1.);
#14 = AXIS2_PLACEMENT_3D('',#6,#3,#5);
#15 = CIRCLE('',#14,1.);
#20 = CARTESIAN_POINT('',(1.,0.,0.));
#21 = CARTESIAN_POINT('',(-1.,0.,0.));
#22 = CARTESIAN_POINT('',(1.,0.,2.));
#23 = VERTEX_POINT('',#20);
#24 = VERTEX_POINT('',#21);
#25 = VERTEX_POINT('',#22);
#30 = EDGE_CURVE('',#23,#24,#11,.T.);
#31 = EDGE_CURVE('',#24,#23,#13,.F.);
#32 = EDGE_CURVE('',#25,#25,#15,.F.);
#33 = LINE('',#20,VECTOR('',#2,1.));
#34 = EDGE_CURVE('',#23,#25,#33,.T.);
#40 = AXIS2_PLACEMENT_3D('',#1,#2,#4);
#41 = CYLINDRICAL_SURFACE('',#40,1.);
#42 = EDGE_LOOP('',(#43,#44,#45,#46,#47));
#43 = ORIENTED_EDGE('',*,*,#30,.T.);
#44 = ORIENTED_EDGE('',*,*,#31,.T.);
#45 = ORIENTED_EDGE('',*,*,#34,.T.);
#46 = ORIENTED_EDGE('',*,*,#32,.F.);
#47 = ORIENTED_EDGE('',*,*,#34,.F.);
#48 = FACE_OUTER_BOUND('',#42,.T.);
#49 = ADVANCED_FACE('',(#48),#41,.T.);
#50 = AXIS2_PLACEMENT_3D('',#1,#2,#4);
#51 = PLANE('',#50);
#52 = ORIENTED_EDGE('',*,*,#31,.F.);
#53 = ORIENTED_EDGE('',*,*,#30,.F.);
#54 = EDGE_LOOP('',(#52,#53));
#55 = FACE_OUTER_BOUND('',#54,.T.);
#56 = ADVANCED_FACE('',(#55),#51,.F.);
#60 = AXIS2_PLACEMENT_3D('',#6,#2,#4);
#61 = PLANE('',#60);
#62 = ORIENTED_EDGE('',*,*,#32,.T.);
#63 = EDGE_LOOP('',(#62));
#64 = FACE_OUTER_BOUND('',#63,.T.);
#65 = ADVANCED_FACE('',(#64),#61,.T.);
#70 = CLOSED_SHELL('',(#49,#56,#65));
#71 = MANIFOLD_SOLID_BREP('split rims',#70);
)");
}

}  // namespace

// Minor (e): edge ranges follow their sense. cylinderWithSplitRims(): each edge runs from its
// start vertex through the half it bounds, and the solid is clean and measures 2 pi.
TEST(StepExact, EdgeRangesFollowTheirSenseAcrossAJoin) {
    const auto solids =
        StepFormat::fromString(cylinderWithSplitRims(), nullptr, nullptr, inMillimetres());
    ASSERT_EQ(solids.size(), 1u) << StepFormat::lastError();
    const topo::Solid& s = *solids[0];
    std::multiset<std::string> middles;
    for (const auto& e : s.edges()) {
        ASSERT_NE(e.exactCurve, nullptr);
        const Vec3 from = e.exactCurve->evaluate(e.curveStart);
        const Vec3 to = e.exactCurve->evaluate(e.curveEnd);
        EXPECT_NEAR((from - e.halfEdge->origin->point).length(), 0.0, 1e-12) << e.topoId.tag();
        EXPECT_NEAR((to - e.halfEdge->twin->origin->point).length(), 0.0, 1e-12) << e.topoId.tag();
        const Vec3 mid = e.exactCurve->evaluate(0.5 * (e.curveStart + e.curveEnd));
        if (std::abs(mid.z) < 1e-9) middles.insert(mid.y > 0.0 ? "+y" : "-y");
    }
    EXPECT_EQ(middles, (std::multiset<std::string>{"+y", "-y"})) << "each half its own";
    EXPECT_EQ(firstIssue(s), "");
    const double volume = hz::model::MassPropertiesCalculator::compute(s).volume;
    EXPECT_NEAR(volume, 2.0 * hz::math::kPi, 2.0 * hz::math::kPi * 1e-9);
}

// ---------------------------------------------------------------------------
// Fix round 2
// ---------------------------------------------------------------------------

namespace {

/// The reviewer's ball with a flat at x = 0.5 on a rational B-spline sphere (closed in u, a pole
/// at each end of v), its poles opened to circles of radius @p eps (0: exact poles), the sphere's
/// face written .T. and used against the flat's circle.
std::string splineBallWithAFlat(double eps) {
    const double w = std::sqrt(0.5);
    const double xs[9] = {1, 1, 0, -1, -1, -1, 0, 1, 1};
    const double ys[9] = {0, 1, 1, 1, 0, -1, -1, -1, 0};
    const double rs[5] = {0, 1, 1, 1, 0};
    const double zs[5] = {-1, -1, 0, 1, 1};
    const double wv[5] = {1, w, 1, w, 1};
    std::string text;
    std::string net;
    std::string weights;
    int id = 200;
    for (int i = 0; i < 9; ++i) {
        const double wu = i % 2 == 1 ? w : 1.0;
        net += i == 0 ? "(" : ",(";
        weights += i == 0 ? "(" : ",(";
        for (int j = 0; j < 5; ++j) {
            const double r = (j == 0 || j == 4) ? eps : rs[j];
            text += "#" + std::to_string(id) + " = CARTESIAN_POINT('',(" + real(xs[i] * r) + "," +
                    real(ys[i] * r) + "," + real(zs[j]) + "));\n";
            net += std::string(j == 0 ? "#" : ",#") + std::to_string(id);
            weights += std::string(j == 0 ? "" : ",") + real(wu * wv[j]);
            ++id;
        }
        net += ")";
        weights += ")";
    }
    text += "#5 = ( BOUNDED_SURFACE() B_SPLINE_SURFACE(2,2,(" + net +
            "),.UNSPECIFIED.,.T.,.F.,.F.) B_SPLINE_SURFACE_WITH_KNOTS((3,2,2,2,3),(3,2,3),"
            "(0.,0.25,0.5,0.75,1.),(0.,0.5,1.),.UNSPECIFIED.) GEOMETRIC_REPRESENTATION_ITEM() "
            "RATIONAL_B_SPLINE_SURFACE((" +
            weights + ")) REPRESENTATION_ITEM('') SURFACE() );\n";
    text += R"(#2 = DIRECTION('',(0.,0.,1.));
#3 = DIRECTION('',(1.,0.,0.));
#6 = CARTESIAN_POINT('',(0.5,0.,0.));
#8 = AXIS2_PLACEMENT_3D('',#6,#3,#2);
#9 = CIRCLE('',#8,0.8660254037844386);
#10 = CARTESIAN_POINT('',(0.5,0.,0.8660254037844386));
#11 = VERTEX_POINT('',#10);
#12 = EDGE_CURVE('',#11,#11,#9,.T.);
#13 = ORIENTED_EDGE('',*,*,#12,.F.);
#14 = EDGE_LOOP('',(#13));
#15 = FACE_OUTER_BOUND('',#14,.T.);
#16 = ADVANCED_FACE('',(#15),#5,.T.);
#17 = PLANE('',#8);
#18 = ORIENTED_EDGE('',*,*,#12,.T.);
#19 = EDGE_LOOP('',(#18));
#20 = FACE_OUTER_BOUND('',#19,.T.);
#21 = ADVANCED_FACE('',(#20),#17,.T.);
#22 = CLOSED_SHELL('',(#16,#21));
#23 = MANIFOLD_SOLID_BREP('ball with a flat',#22);
)";
    return hz::step_text::stepFile(text);
}

/// externalStyleCylinder() with a hemispherical dimple of radius 1 in its top: the top cap a
/// sphere about (0, 0, 2), below its rim, bounded by the rim and a seam from it down to the pole
/// and back. The sphere's surface faces out of the sphere, into the material, yet its face says
/// same_sense .T., as Horizon's writer wrote faces whichever way their surfaces faced.
std::string cylinderWithADimpleWrittenFacingIn() {
    std::string text = replaced(externalStyleCylinder(), "#29 = PLANE('',#28);",
                                "#29 = SPHERICAL_SURFACE('',#28,1.);\n"
                                "#340 = AXIS2_PLACEMENT_3D('',#8,#3,#9);\n"
                                "#341 = CIRCLE('',#340,1.);\n"
                                "#342 = CARTESIAN_POINT('',(0.,0.,1.));\n"
                                "#343 = VERTEX_POINT('',#342);\n"
                                "#344 = EDGE_CURVE('',#13,#343,#341,.T.);\n"
                                "#345 = ORIENTED_EDGE('',*,*,#344,.T.);\n"
                                "#346 = ORIENTED_EDGE('',*,*,#344,.F.);");
    return replaced(text, "#31 = EDGE_LOOP('',(#30));", "#31 = EDGE_LOOP('',(#30,#345,#346));");
}

}  // namespace

// The Important finding: a B-spline ball's poles opened a hair (as a fitted surface's are) are
// still poles within the solid's limit, so its face outside the flat is never taken for one its
// loops decide, and never comes in turned inside out as the cap.
TEST(StepExact, ASplineBallWithItsPolesOpenedIsNeverTurnedInsideOut) {
    for (const double eps : {0.0, 1e-7, 1e-5}) {
        SCOPED_TRACE(eps);
        const auto solids =
            StepFormat::fromString(splineBallWithAFlat(eps), nullptr, nullptr, inMillimetres());
        EXPECT_TRUE(solids.empty()) << "it came in";
        EXPECT_NE(StepFormat::lastError().find(kAgainstItsLoops + std::string(" (face #16)")),
                  std::string::npos)
            << StepFormat::lastError();
    }
}

// Item 1: a loop that uses a seam of its own fixes its region, so on a sphere it decides the
// face's sense as on a plane. A dimple written .T. with its surface facing into the material
// comes in reversed, every normal pointing out of the material.
TEST(StepExact, ASeamedSphereCapWrittenFacingInComesInFacingOut) {
    const auto solids = StepFormat::fromString(cylinderWithADimpleWrittenFacingIn(), nullptr,
                                               nullptr, inMillimetres());
    ASSERT_EQ(solids.size(), 1u) << StepFormat::lastError();
    const topo::Face* dimple = faceOfKind(*solids[0], geo::SurfaceKind::Sphere);
    ASSERT_NE(dimple, nullptr);
    EXPECT_TRUE(dimple->reversed);
    const Vec3 centre{0, 0, 2};
    for (const Vec3& p : {Vec3{0.6, 0, 1.2}, Vec3{0, -0.6, 1.2}, Vec3{0, 0, 1}}) {
        EXPECT_GT(outward(*dimple, p).dot((centre - p).normalized()), 0.99) << "into the dimple";
    }
    const topo::Face* side = faceOfKind(*solids[0], geo::SurfaceKind::Cylinder);
    ASSERT_NE(side, nullptr);
    EXPECT_GT(outward(*side, Vec3{1, 0, 1}).x, 0.99);
    EXPECT_EQ(firstIssue(*solids[0]), "");
}

// Item 4: a closed B-spline rim whose vertex is 5e-3 off the curve's join, in a file of 0.01
// uncertainty: off the join by a thousandth of the curve (2.8e-3), but within the solid's limit
// (a thousandth of its height of 100), so the vertex is at the join and the edge grows to cover
// its gap.
TEST(StepExact, AClosedSplineJoinIsFoundWithinTheSolidsLimit) {
    std::string text = replaced(externalStyleCylinder(), "#8 = CARTESIAN_POINT('',(0.,0.,2.));",
                                "#8 = CARTESIAN_POINT('',(0.,0.,100.));");
    text = replaced(text, "#12 = CARTESIAN_POINT('',(1.,0.,2.));",
                    "#12 = CARTESIAN_POINT('',(1.,0.,100.));");
    text = replaced(text, "#14 = EDGE_CURVE('',#7,#7,#5,.T.);",
                    splineCircle(300, 310, 5e-3) + "#14 = EDGE_CURVE('',#7,#7,#300,.T.);");
    ImportReport report;
    const auto solids =
        StepFormat::fromString(withEntities(text, millimetresWithin("0.01")), &report);
    ASSERT_EQ(solids.size(), 1u) << StepFormat::lastError();
    EXPECT_GE(largestEdgeTolerance(*solids[0]), 4.9e-3);
    EXPECT_TRUE(anySays(report.converted, "exact to")) << report.converted.size();
    EXPECT_EQ(firstIssue(*solids[0]), "");
}

// A seamed face whose loop joins at a pole (a spherical cap with a seam meridian) is exact
// wherever its loop starts: its seam's two sides share one (u, v) window, so some starts unwrap
// only once the seam is laid out a period over. The loop that does not close is a band only
// when it has no seam (review C2: the fail-fast read every such loop as a band).
TEST(StepExact, ASeamedCapIsExactWhereverItsLoopStarts) {
    for (const std::string order : {"#20,#21,#22", "#21,#22,#20", "#22,#20,#21"}) {
        ImportReport report;
        const auto solids =
            StepFormat::fromString(seamedHemisphere(order), &report, nullptr, inMillimetres());
        ASSERT_EQ(solids.size(), 1u) << order << ": " << StepFormat::lastError();
        EXPECT_EQ(firstIssue(*solids[0]), "") << order;
        const topo::Face* cap = faceOfKind(*solids[0], geo::SurfaceKind::Sphere);
        ASSERT_NE(cap, nullptr) << order;
        EXPECT_GT(outward(*cap, Vec3{0, 0, 10}).z, 0.99) << order;
        EXPECT_GT(outward(*cap, Vec3{10, 0, 0}).x, 0.99) << order;
    }
}

namespace {

/// A cylinder of radius 1.15 along x from -3.8 to 3.8, its rims written as Solid Edge writes the
/// bore of the corpus's knife holder (face #665): its surface's frame along x with its x along
/// -z; each rim's CIRCLE facing the other way along the axis, each EDGE_CURVE against its circle
/// (.F.), and the side's two loops plain FACE_BOUNDs using both rims forward. The rims' vertices
/// are half a turn apart.
std::string solidEdgeCylinder() {
    return hz::step_text::stepFile(R"(#1 = CARTESIAN_POINT('',(-14.8812075884992,2.5,-0.5));
#2 = DIRECTION('',(1.,0.,0.));
#3 = DIRECTION('',(0.,0.,-1.));
#4 = AXIS2_PLACEMENT_3D('',#1,#2,#3);
#5 = CYLINDRICAL_SURFACE('',#4,1.15);
#6 = CARTESIAN_POINT('',(3.8,2.5,-0.5));
#7 = DIRECTION('',(0.,0.,1.));
#8 = AXIS2_PLACEMENT_3D('',#6,#2,#7);
#9 = CIRCLE('',#8,1.15);
#10 = CARTESIAN_POINT('',(-3.8,2.5,-0.5));
#11 = DIRECTION('',(-1.,0.,0.));
#12 = AXIS2_PLACEMENT_3D('',#10,#11,#3);
#13 = CIRCLE('',#12,1.15);
#14 = CARTESIAN_POINT('',(3.8,2.5,0.65));
#15 = VERTEX_POINT('',#14);
#16 = CARTESIAN_POINT('',(-3.8,2.5,-1.65));
#17 = VERTEX_POINT('',#16);
#18 = EDGE_CURVE('',#15,#15,#9,.F.);
#19 = EDGE_CURVE('',#17,#17,#13,.F.);
#20 = ORIENTED_EDGE('',*,*,#18,.T.);
#21 = EDGE_LOOP('',(#20));
#22 = FACE_BOUND('',#21,.T.);
#23 = ORIENTED_EDGE('',*,*,#19,.T.);
#24 = EDGE_LOOP('',(#23));
#25 = FACE_BOUND('',#24,.T.);
#26 = ADVANCED_FACE('',(#22,#25),#5,.T.);
#27 = AXIS2_PLACEMENT_3D('',#6,#2,#7);
#28 = PLANE('',#27);
#29 = ORIENTED_EDGE('',*,*,#18,.F.);
#30 = EDGE_LOOP('',(#29));
#31 = FACE_OUTER_BOUND('',#30,.T.);
#32 = ADVANCED_FACE('',(#31),#28,.T.);
#33 = AXIS2_PLACEMENT_3D('',#10,#11,#3);
#34 = PLANE('',#33);
#35 = ORIENTED_EDGE('',*,*,#19,.F.);
#36 = EDGE_LOOP('',(#35));
#37 = FACE_OUTER_BOUND('',#36,.T.);
#38 = ADVANCED_FACE('',(#37),#34,.T.);
#39 = CLOSED_SHELL('',(#26,#32,#38));
#40 = MANIFOLD_SOLID_BREP('bore',#39);
)");
}

}  // namespace

// Review Focus 2: a periodic face that is a band between two rims, with no seam, comes in exact
// with a seam inserted: test_StepCurved's band (its rims' vertices in line, so the seam joins
// them), and Solid Edge's (both loops forward, its vertices half a turn apart, so a rim is split
// where the seam crosses it). The seam is a line up the side, named as the reader names its
// seams; every edge keeps a carrier, and the faceted copy measures the cylinder over them.
TEST(StepExact, ABandComesInExact) {
    struct Case {
        const char* what;
        std::string text;
        double volume;
        std::size_t vertices;
        const char* side;  // the side's ADVANCED_FACE, which names its seam
    };
    const Case cases[] = {
        {"test_StepCurved's band", bandCylinder(), 2.0 * hz::math::kPi, 2, "/f:27/seam"},
        {"Solid Edge's", solidEdgeCylinder(), hz::math::kPi * 1.15 * 1.15 * 7.6, 3, "/f:26/seam"},
    };
    for (const Case& c : cases) {
        SCOPED_TRACE(c.what);
        ImportReport report;
        const auto solids = StepFormat::fromString(c.text, &report, nullptr, inMillimetres());
        ASSERT_EQ(solids.size(), 1u) << StepFormat::lastError();
        EXPECT_TRUE(report.skipped.empty()) << report.skipped.front();
        const topo::Solid& solid = *solids[0];
        const topo::Face* side = faceOfKind(solid, geo::SurfaceKind::Cylinder);
        ASSERT_NE(side, nullptr);
        EXPECT_TRUE(side->innerLoops.empty()) << "its rims are one loop with its seam";
        std::vector<const topo::Edge*> seams;
        const topo::HalfEdge* h = side->outerLoop->halfEdge;
        do {
            if (h->twin->face == side && h == h->edge->halfEdge) seams.push_back(h->edge);
            h = h->next;
        } while (h != side->outerLoop->halfEdge);
        ASSERT_EQ(seams.size(), 1u);
        EXPECT_EQ(seams[0]->exactCurve->kind(), geo::CurveKind::Line);
        EXPECT_NE(seams[0]->topoId.tag().find(c.side), std::string::npos) << seams[0]->topoId.tag();
        EXPECT_EQ(solid.vertexCount(), c.vertices);
        expectExactEdges(solid);
        EXPECT_EQ(firstIssue(solid), "");
        std::string problem;
        hz::model::BoundaryMesh::extractFacePolygons(solid, 1e-3, &problem);
        EXPECT_EQ(problem, "");
        // Integrated on its exact faces.
        EXPECT_NEAR(hz::model::MassPropertiesCalculator::compute(solid).volume, c.volume,
                    c.volume * 1e-9);
        // The faceted copy, over the carriers split with the exact edges.
        const auto faceted = hz::model::facetCurved(solid);
        ASSERT_NE(faceted.solid, nullptr) << faceted.error;
        const auto ideal = hz::model::MassPropertiesCalculator::computeIdeal(*faceted.solid);
        EXPECT_TRUE(ideal.onIdealSurfaces);
        EXPECT_NEAR(ideal.properties.volume, c.volume, c.volume * 1e-8);
    }
}

// Ruling 9 holds for what a seam is given: its tolerance covers how far its line passes from the
// vertices it joins, and that is held to the solid's limit, a thousandth of its size, as every
// tolerance grown from the file is. Rims 0.003 off a cylinder of radius 1.003 (within the limit
// of 0.00346), the top rim's vertex 0.005 rad round from the bottom's: within its joins' allowance
// of the seam's line, so the seam joins it there, and passes 0.0058 from it.
TEST(StepExact, ASeamPastTheLimitLeavesItsSolidOut) {
    const std::string text =
        replaced(bandCylinder("1.003"), "#12 = CARTESIAN_POINT('',(1.,0.,2.));",
                 "#12 = CARTESIAN_POINT('',(0.99998750002604,0.00499997916669,2.));");
    ImportReport report;
    EXPECT_TRUE(StepFormat::fromString(text, &report, nullptr, inMillimetres()).empty());
    const std::string& why = StepFormat::lastError();
    EXPECT_NE(why.find("its seam passes 0.00584 from a vertex it joins, beyond the limit of "
                       "0.00346 (a thousandth of the solid's size) (face #27"),
              std::string::npos)
        << why;
}

// ---------------------------------------------------------------------------
// The writer writes exact faces (Task 8)
// ---------------------------------------------------------------------------

namespace {

/// @p solid written to STEP and read back: its one solid, or null. @p text, when given, receives
/// what was written.
std::unique_ptr<topo::Solid> writtenAndRead(const topo::Solid& solid, std::string* text = nullptr) {
    const std::string written = StepFormat::toString({&solid});
    if (text != nullptr) *text = written;
    auto back = StepFormat::fromString(written);
    EXPECT_EQ(back.size(), 1u) << StepFormat::lastError();
    return back.size() == 1 ? std::move(back[0]) : nullptr;
}

bool near(const Vec3& a, const Vec3& b, double tolerance) {
    return (a - b).length() <= tolerance;
}

/// Empty when @p b is @p a as the writer writes it, within @p tolerance: the same kind, origin,
/// x axis and z axis (a plane's z its x × y, which its PLANE's axis is, whichever hand its frame
/// is), and radii; a B-spline's same degrees, net and weights. Otherwise what differs. A
/// parameter scale (a plane's, an axial one: a file in inches) is not geometry, and STEP's typed
/// surfaces carry none.
std::string surfaceDiffers(const geo::Surface& a, const geo::Surface& b, double tolerance) {
    if (a.kind() != b.kind()) return "its kind";
    const auto frames = [&](const geo::Frame& fa, const geo::Frame& fb,
                            const Vec3& axis) -> std::string {
        if (!near(fa.origin, fb.origin, tolerance)) return "its origin";
        if (!near(fa.x, fb.x, 1e-12)) return "its x axis";
        if (!near(axis, fb.z, 1e-12)) return "its axis";
        return {};
    };
    const auto same = [&](double x, double y) { return std::abs(x - y) <= tolerance; };
    if (const auto* p = dynamic_cast<const geo::Plane*>(&a)) {
        const auto& q = dynamic_cast<const geo::Plane&>(b);
        return frames(p->frame(), q.frame(), p->frame().x.cross(p->frame().y));
    }
    if (const auto* c = dynamic_cast<const geo::Cylinder*>(&a)) {
        const auto& d = dynamic_cast<const geo::Cylinder&>(b);
        if (!same(c->radius(), d.radius())) return "its radius";
        return frames(c->frame(), d.frame(), c->frame().z);
    }
    if (const auto* c = dynamic_cast<const geo::Cone*>(&a)) {
        const auto& d = dynamic_cast<const geo::Cone&>(b);
        if (!same(c->radius(), d.radius())) return "its radius";
        if (std::abs(c->halfAngle() - d.halfAngle()) > 1e-12) return "its half-angle";
        return frames(c->frame(), d.frame(), c->frame().z);
    }
    if (const auto* s = dynamic_cast<const geo::Sphere*>(&a)) {
        const auto& t = dynamic_cast<const geo::Sphere&>(b);
        if (!same(s->radius(), t.radius())) return "its radius";
        return frames(s->frame(), t.frame(), s->frame().z);
    }
    if (const auto* s = dynamic_cast<const geo::Torus*>(&a)) {
        const auto& t = dynamic_cast<const geo::Torus&>(b);
        if (!same(s->majorRadius(), t.majorRadius()) || !same(s->minorRadius(), t.minorRadius())) {
            return "its radii";
        }
        return frames(s->frame(), t.frame(), s->frame().z);
    }
    const auto& n = dynamic_cast<const geo::NurbsSurface&>(a);
    const auto& m = dynamic_cast<const geo::NurbsSurface&>(b);
    if (n.degreeU() != m.degreeU() || n.degreeV() != m.degreeV() ||
        n.controlPoints().size() != m.controlPoints().size()) {
        return "its degrees or net";
    }
    for (std::size_t i = 0; i < n.controlPoints().size(); ++i) {
        if (n.controlPoints()[i].size() != m.controlPoints()[i].size()) return "its net";
        for (std::size_t j = 0; j < n.controlPoints()[i].size(); ++j) {
            if (!near(n.controlPoints()[i][j], m.controlPoints()[i][j], tolerance) ||
                std::abs(n.weights()[i][j] - m.weights()[i][j]) > 1e-12) {
                return "its net";
            }
        }
    }
    return {};
}

/// Empty when @p b is @p a as the writer writes it, within @p tolerance: a line's same point,
/// direction and magnitude (its scale); a circle's or an ellipse's same centre, radii, x axis and
/// plane (a closed edge's conic is written turned, where the edge runs back down it, so that it
/// runs with its edge); a B-spline's same degree (an open edge's is written over its own span).
/// Where each runs is the caller's to compare. Otherwise what differs.
std::string curveDiffers(const geo::Curve3& a, const geo::Curve3& b, double tolerance) {
    if (a.kind() != b.kind()) return "its kind";
    const auto conics = [&](const geo::Frame& fa, const geo::Frame& fb) -> std::string {
        if (!near(fa.origin, fb.origin, tolerance)) return "its centre";
        if (!near(fa.x, fb.x, 1e-12)) return "its x axis";
        if (std::abs(fa.x.cross(fa.y).dot(fb.x.cross(fb.y))) < 1.0 - 1e-12) return "its plane";
        return {};
    };
    const auto same = [&](double x, double y) { return std::abs(x - y) <= tolerance; };
    if (const auto* l = dynamic_cast<const geo::LineCurve*>(&a)) {
        const auto& m = dynamic_cast<const geo::LineCurve&>(b);
        if (!near(l->origin(), m.origin(), tolerance)) return "its point";
        if (!near(l->direction(), m.direction(), 1e-12)) return "its direction";
        if (!same(l->scale(), m.scale())) return "its magnitude";
        return {};
    }
    if (const auto* c = dynamic_cast<const geo::CircleCurve*>(&a)) {
        const auto& d = dynamic_cast<const geo::CircleCurve&>(b);
        if (!same(c->radius(), d.radius())) return "its radius";
        return conics(c->frame(), d.frame());
    }
    if (const auto* e = dynamic_cast<const geo::EllipseCurve*>(&a)) {
        const auto& f = dynamic_cast<const geo::EllipseCurve&>(b);
        if (!same(e->majorRadius(), f.majorRadius()) || !same(e->minorRadius(), f.minorRadius())) {
            return "its radii";
        }
        return conics(e->frame(), f.frame());
    }
    const auto& n = dynamic_cast<const geo::NurbsCurve&>(a);
    const auto& m = dynamic_cast<const geo::NurbsCurve&>(b);
    if (n.degree() != m.degree()) return "its degree";
    return {};
}

/// The points of @p e's span at a quarter of its parameter each, from its start to its end.
std::vector<Vec3> spanOf(const topo::Edge& e) {
    std::vector<Vec3> points;
    for (int k = 0; k <= 4; ++k) {
        points.push_back(
            e.exactCurve->evaluate(e.curveStart + (e.curveEnd - e.curveStart) * k / 4));
    }
    return points;
}

/// Whether @p a and @p b are the same points within @p tolerance, in order or the other way.
bool sameSpan(const std::vector<Vec3>& a, const std::vector<Vec3>& b, double tolerance) {
    const auto inOrder = [&](bool reversed) {
        for (std::size_t k = 0; k < a.size(); ++k) {
            if (!near(a[k], b[reversed ? b.size() - 1 - k : k], tolerance)) return false;
        }
        return true;
    };
    return a.size() == b.size() && (inOrder(false) || inOrder(true));
}

/// The points of @p face's loops' vertices.
std::vector<Vec3> cornersOf(const topo::Face& face) {
    std::vector<Vec3> corners;
    std::vector<const topo::Wire*> loops{face.outerLoop};
    loops.insert(loops.end(), face.innerLoops.begin(), face.innerLoops.end());
    for (const topo::Wire* w : loops) {
        const topo::HalfEdge* h = w->halfEdge;
        do {
            corners.push_back(h->origin->point);
            h = h->next;
        } while (h != w->halfEdge);
    }
    return corners;
}

/// Whether every point of @p a is one of @p b's and the other way round, within @p tolerance.
bool sameCorners(const std::vector<Vec3>& a, const std::vector<Vec3>& b, double tolerance) {
    const auto within = [tolerance](const std::vector<Vec3>& x, const std::vector<Vec3>& y) {
        return std::all_of(x.begin(), x.end(), [&](const Vec3& p) {
            return std::any_of(y.begin(), y.end(),
                               [&](const Vec3& q) { return near(p, q, tolerance); });
        });
    };
    return within(a, b) && within(b, a);
}

/// A point on @p face's boundary where its outward normal is defined (none at an apex), and that
/// normal: the middle of one of its outer loop's half-edges, on its pcurve.
std::pair<Vec3, Vec3> pointAndNormal(const topo::Face& face) {
    const topo::HalfEdge* h = face.outerLoop->halfEdge;
    do {
        const auto uv = h->pcurve->evaluate(0.5 * (h->pcurveStart + h->pcurveEnd));
        Vec3 n = face.exactSurface->normal(uv.x, uv.y);
        if (n.length() > 0.5) {
            return {face.exactSurface->evaluate(uv.x, uv.y), face.reversed ? n * -1.0 : n};
        }
        h = h->next;
    } while (h != face.outerLoop->halfEdge);
    ADD_FAILURE() << "no point on its outline has a normal";
    return {};
}

/// Expects @p back to be @p solid written and read back with nothing changed: each face on the
/// same surface (surfaceDiffers), with the same corners, facing the same way; each edge on the
/// same curve (curveDiffers) over the same points; the same exact volume to 1e-9; clean.
void expectUnchanged(const topo::Solid& solid, const topo::Solid& back) {
    const double tolerance = topo::exactTolerance(solid).linear;
    EXPECT_EQ(back.faceCount(), solid.faceCount());
    EXPECT_EQ(back.edgeCount(), solid.edgeCount());
    EXPECT_EQ(back.vertexCount(), solid.vertexCount());
    std::set<const topo::Face*> matched;
    for (const auto& f : solid.faces()) {
        ASSERT_NE(f.exactSurface, nullptr);
        const auto corners = cornersOf(f);
        const auto [p, n] = pointAndNormal(f);
        const topo::Face* found = nullptr;
        std::string why = "no face with its corners";
        for (const auto& g : back.faces()) {
            if (matched.count(&g) != 0 || !g.exactSurface ||
                !sameCorners(corners, cornersOf(g), tolerance)) {
                continue;
            }
            why = surfaceDiffers(*f.exactSurface, *g.exactSurface, tolerance);
            if (!why.empty()) continue;
            const auto uv = g.exactSurface->invert(p);
            const Vec3 m = g.exactSurface->normal(uv.x, uv.y) * (g.reversed ? -1.0 : 1.0);
            if (!near(g.exactSurface->evaluate(uv.x, uv.y), p, tolerance) || m.dot(n) < 0.999) {
                why = "it faces the other way, or is not where it was";
                continue;
            }
            found = &g;
            break;
        }
        EXPECT_NE(found, nullptr) << f.topoId.tag() << ": " << why;
        if (found != nullptr) matched.insert(found);
    }
    for (const auto& e : solid.edges()) {
        ASSERT_NE(e.exactCurve, nullptr);
        const auto span = spanOf(e);
        bool found = false;
        std::string why = "no edge over the same points";
        for (const auto& g : back.edges()) {
            if (!g.exactCurve || !sameSpan(span, spanOf(g), tolerance)) continue;
            why = curveDiffers(*e.exactCurve, *g.exactCurve, tolerance);
            if (why.empty()) {
                found = true;
                break;
            }
        }
        EXPECT_TRUE(found) << e.topoId.tag() << ": " << why;
    }
    const double volume = hz::model::MassPropertiesCalculator::compute(solid).volume;
    EXPECT_NEAR(hz::model::MassPropertiesCalculator::compute(back).volume, volume, volume * 1e-9);
    EXPECT_EQ(firstIssue(back), "");
}

/// @p text with its first @p count FACE_OUTER_BOUNDs (all of them, when it has no more) turned
/// round: their loops run the other way, so on a plane, where the loops decide, the face faces
/// the other way.
std::string boundsTurned(std::string text, int count) {
    std::size_t at = 0;
    for (int i = 0; i < count; ++i) {
        at = text.find("FACE_OUTER_BOUND(", at);
        if (at == std::string::npos) break;
        const std::size_t sense = text.find(",.T.)", at);
        if (sense == std::string::npos) break;
        text.replace(sense, 5, ",.F.)");
        at = sense;
    }
    return text;
}

/// Expects every face of @p shell to face away from @p centre, or toward it with @p toward: its
/// outward normal at the middle of a half-edge of its outer loop (pointAndNormal).
void expectFacing(const topo::Shell& shell, const Vec3& centre, bool toward = false) {
    for (const topo::Face* f : shell.faces) {
        const auto [p, n] = pointAndNormal(*f);
        const double along = n.dot((p - centre).normalized());
        if (toward) {
            EXPECT_LT(along, -0.1) << f->topoId.tag() << " faces away from its void's centre";
        } else {
            EXPECT_GT(along, 0.1) << f->topoId.tag() << " faces into its solid";
        }
    }
}

}  // namespace

// Every solid the reader's tests read, and the exact primitives, each written and read back,
// comes back as it was: each face on the same surface, of the same kind (a CYLINDRICAL_SURFACE
// as one, a B-spline as that B-spline), its frame and radii the same within the model
// resolution, facing the same way; each edge on the same curve, a line keeping its magnitude;
// the same exact volume. A mirror image's left-handed frames and a file in inches' scales are
// written as the geometry they make (Ruling 42).
TEST(StepExact, ARoundTripChangesNoSurface) {
    std::vector<std::pair<std::string, std::unique_ptr<topo::Solid>>> solids;
    const auto read = [&](const std::string& what, const std::string& text,
                          const hz::io::StepReadOptions& options) {
        auto in = StepFormat::fromString(text, nullptr, nullptr, options);
        ASSERT_EQ(in.size(), 1u) << what << ": " << StepFormat::lastError();
        solids.emplace_back(what, std::move(in[0]));
    };
    read("an OpenCASCADE cylinder", externalStyleCylinder(), inMillimetres());
    read("a cone to its apex", cone(), inMillimetres());
    read("a sphere", sphere(), inMillimetres());
    read("a torus", torus(), inMillimetres());
    read("a B-spline cylinder", splineCylinder(), inMillimetres());
    read("a block with a ball out of it", blockWithABall(), inMillimetres());
    read("a block with a face written facing in", block(true), inMillimetres());
    read("a seamed hemisphere", seamedHemisphere(), inMillimetres());
    read("a dimple written facing in", cylinderWithADimpleWrittenFacingIn(), inMillimetres());
    read("split rims", cylinderWithSplitRims(), inMillimetres());
    read("a band", bandCylinder(), inMillimetres());
    read("Solid Edge's bore", solidEdgeCylinder(), inMillimetres());
    read("a file in inches",
         withEntities(replaced(externalStyleCylinder(), "VECTOR('',#2,1.)", "VECTOR('',#2,2.)"),
                      inches()),
         {});
    solids.emplace_back("an exact block", hz::model::exact::makeBlock({-1, 0, 2}, {3, 5, 6}));
    solids.emplace_back("an exact cylinder",
                        hz::model::exact::makeCylinder(
                            geo::Frame::fromAxis({1, 2, 3}, {0, 1, 1}, {1, 0, 0}), 2.0, 5.0));
    solids.emplace_back("an exact plate with a hole",
                        hz::model::exact::makePlateWithHole({0, 0, 0}, {8, 6, 2}, {3, 3}, 1.5));
    const auto mirror = hz::math::Mat4::reflection(Vec3{4, 0, 0}, Vec3{1, 1, 0});
    for (std::size_t i = 0; i < 2; ++i) {
        // A mirror image: its frames left-handed, every face's sense turned with its loops (a
        // B-spline's among them).
        solids.emplace_back(
            "mirrored " + solids[i == 0 ? 0 : 4].first,
            hz::model::Pattern::transformed(*solids[i == 0 ? 0 : 4].second, mirror));
    }
    for (const auto& [what, solid] : solids) {
        SCOPED_TRACE(what);
        ASSERT_NE(solid, nullptr);
        ASSERT_EQ(firstIssue(*solid), "");
        const auto back = writtenAndRead(*solid);
        ASSERT_NE(back, nullptr);
        expectUnchanged(*solid, *back);
    }
}

// An exact cylinder goes out on its own surfaces: its side a CYLINDRICAL_SURFACE, its caps PLANEs,
// its rims CIRCLEs and its seam a LINE; no B-spline surface or curve.
TEST(StepExact, TypedSurfacesAreWrittenAsTheirKind) {
    const auto cylinder =
        hz::model::exact::makeCylinder(geo::Frame::fromAxis({0, 0, 0}, {0, 0, 1}), 2.0, 5.0);
    ASSERT_NE(cylinder, nullptr);
    const std::string text = StepFormat::toString({cylinder.get()});
    const auto count = [&text](const std::string& what) {
        std::size_t n = 0;
        for (auto at = text.find(what); at != std::string::npos; at = text.find(what, at + 1)) ++n;
        return n;
    };
    EXPECT_EQ(count("CYLINDRICAL_SURFACE("), 1u);
    EXPECT_EQ(count(" PLANE("), 2u);
    EXPECT_EQ(count("CIRCLE("), 2u);
    EXPECT_EQ(count("LINE("), 1u);
    EXPECT_EQ(count("B_SPLINE_SURFACE"), 0u);
    EXPECT_EQ(count("B_SPLINE_CURVE"), 0u);
    EXPECT_EQ(count("PCURVE"), 0u);

    // A cone, a sphere and a torus as read, each as its own kind.
    for (const auto& [text2, kind] : {std::pair{cone(), std::string("CONICAL_SURFACE(")},
                                      std::pair{sphere(), std::string("SPHERICAL_SURFACE(")},
                                      std::pair{torus(), std::string("TOROIDAL_SURFACE(")}}) {
        SCOPED_TRACE(kind);
        const auto in = StepFormat::fromString(text2, nullptr, nullptr, inMillimetres());
        ASSERT_EQ(in.size(), 1u) << StepFormat::lastError();
        const std::string out = StepFormat::toString({in[0].get()});
        EXPECT_NE(out.find(kind), std::string::npos);
        EXPECT_EQ(out.find("B_SPLINE_SURFACE"), std::string::npos);
    }
}

// Ruling 41: the kernel's box is wound facing in. Written and read back it is a solid whose every
// face faces out, away from its centre, and that holds together.
TEST(StepExact, TheKernelsBoxWrittenAndReadBackFacesOut) {
    const auto box = hz::model::PrimitiveFactory::makeBox(10.0, 20.0, 30.0);
    ASSERT_NE(box, nullptr);
    const auto back = writtenAndRead(*box);
    ASSERT_NE(back, nullptr);
    ASSERT_TRUE(topo::hasExactFaces(*back));
    Vec3 centre;
    for (const auto& v : box->vertices()) centre = centre + v.point * (1.0 / 8.0);
    ASSERT_EQ(back->shells().size(), 1u);
    expectFacing(back->shells().front(), centre);
    EXPECT_EQ(firstIssue(*back), "");
    EXPECT_NEAR(hz::model::MassPropertiesCalculator::compute(*back).volume, 6000.0, 6000.0 * 1e-12);
}

// Ruling 41, the reader's half: a file that writes a block's shell inside out (every loop turned
// round, so each face faces in) is read with its shell turned outward, its topology with it: every
// face faces away from the block's centre and the validator is clean. With a ball's void inside,
// the void still faces into itself (Review Focus 4) and its volume subtracts.
TEST(StepExact, AShellWrittenInsideOutIsReadFacingOut) {
    const Vec3 centre{2, 2, 2};
    {
        const auto in = StepFormat::fromString(boundsTurned(block(false), 6), nullptr, nullptr,
                                               inMillimetres());
        ASSERT_EQ(in.size(), 1u) << StepFormat::lastError();
        ASSERT_EQ(in[0]->shells().size(), 1u);
        expectFacing(in[0]->shells().front(), centre);
        EXPECT_EQ(firstIssue(*in[0]), "");
        EXPECT_NEAR(hz::model::MassPropertiesCalculator::compute(*in[0]).volume, 64.0, 64e-12);
    }
    {
        const auto in = StepFormat::fromString(boundsTurned(blockWithABall(), 6), nullptr, nullptr,
                                               inMillimetres());
        ASSERT_EQ(in.size(), 1u) << StepFormat::lastError();
        ASSERT_EQ(in[0]->shells().size(), 2u);
        expectFacing(in[0]->shells()[0], centre);
        expectFacing(in[0]->shells()[1], centre, true);
        EXPECT_EQ(firstIssue(*in[0]), "");
        const double volume = 64.0 - 4.0 / 3.0 * hz::math::kPi;
        EXPECT_NEAR(hz::model::MassPropertiesCalculator::compute(*in[0]).volume, volume,
                    volume * 1e-9);
    }
}

namespace {

/// bandCylinder() with its top rim a closed B-spline circle joined, and its vertex, half a turn
/// round from the bottom rim's, as the rims of Onshape's airframe are: the seam put in where it
/// is read splits the top rim, into two edges each on part of the closed B-spline.
std::string splineBand() {
    std::string text = replaced(bandCylinder(), "#11 = CIRCLE('',#10,1.);",
                                splineCircle(11, 310, hz::math::kPi, 2.0));
    return replaced(text, "#12 = CARTESIAN_POINT('',(1.,0.,2.));",
                    "#12 = CARTESIAN_POINT('',(-1.,0.,2.));");
}

}  // namespace

// A band given its seam where it is read, which splits a rim: Solid Edge's bore (its rims each a
// closed edge against its circle, .F.) and a band whose split rim is a closed B-spline (Onshape's
// airframe's). Written and read back, every face is still cut into facets, as it was when first
// read, so an imported part saved and opened again builds: a closed edge goes out running with
// its curve, and an open edge on a B-spline on its own span of it, which a reader cannot take
// for the other half of a closed curve.
TEST(StepExact, ASeamedBandWrittenAndReadBackIsCutIntoFacetsAsBefore) {
    for (const auto& [what, text] :
         {std::pair<std::string, std::string>{"Solid Edge's bore", solidEdgeCylinder()},
          {"a band with a B-spline rim", splineBand()}}) {
        SCOPED_TRACE(what);
        const auto in = StepFormat::fromString(text, nullptr, nullptr, inMillimetres());
        ASSERT_EQ(in.size(), 1u) << StepFormat::lastError();
        const auto first = hz::model::facetCurved(*in[0]);
        ASSERT_NE(first.solid, nullptr) << first.error;
        const auto back = writtenAndRead(*in[0]);
        ASSERT_NE(back, nullptr);
        EXPECT_EQ(firstIssue(*back), "");
        const auto again = hz::model::facetCurved(*back);
        ASSERT_NE(again.solid, nullptr) << again.error;
        EXPECT_EQ(again.solid->faceCount(), first.solid->faceCount());
        const double volume = hz::model::MassPropertiesCalculator::compute(*first.solid).volume;
        EXPECT_NEAR(hz::model::MassPropertiesCalculator::compute(*again.solid).volume, volume,
                    volume * 1e-9);
    }
}

// A solid of an exact body and a body of the kernel's (in facets, wound facing in) goes out with
// each face as it is: the exact cylinder's on its cylinder and planes, the box's on their planes
// (Ruling 13), each body facing out. Read back, both bodies are exact and face out, and the solid
// measures the two.
TEST(StepExact, AMixedSolidGoesOutWithEachFaceAsItIs) {
    const auto cylinder =
        hz::model::exact::makeCylinder(geo::Frame::fromAxis({0, 0, 0}, {0, 0, 1}), 1.0, 2.0);
    const auto box = hz::model::Pattern::transformed(*hz::model::PrimitiveFactory::makeBox(2, 3, 4),
                                                     hz::math::Mat4::translation({5, 0, 0}));
    ASSERT_NE(cylinder, nullptr);
    ASSERT_NE(box, nullptr);
    topo::Solid mixed;
    hz::model::Pattern::append(mixed, *cylinder, hz::math::Mat4::identity());
    hz::model::Pattern::append(mixed, *box, hz::math::Mat4::identity());
    std::string text;
    const auto back = writtenAndRead(mixed, &text);
    ASSERT_NE(back, nullptr);
    EXPECT_NE(text.find("CYLINDRICAL_SURFACE("), std::string::npos);
    EXPECT_EQ(text.find("B_SPLINE_SURFACE"), std::string::npos);
    ASSERT_EQ(back->shells().size(), 2u);
    std::multiset<geo::SurfaceKind> kinds;
    for (const topo::Face* f : exactFaces(*back)) kinds.insert(f->exactSurface->kind());
    EXPECT_EQ(kinds.count(geo::SurfaceKind::Cylinder), 1u);
    EXPECT_EQ(kinds.count(geo::SurfaceKind::Plane), 8u);
    expectFacing(back->shells()[0], Vec3{0, 0, 1});
    expectFacing(back->shells()[1], Vec3{6, 1.5, 2});
    EXPECT_EQ(firstIssue(*back), "");
    const double volume = 2.0 * hz::math::kPi + 24.0;
    EXPECT_NEAR(hz::model::MassPropertiesCalculator::compute(*back).volume, volume, volume * 1e-9);
}
