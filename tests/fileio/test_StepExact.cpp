// STEP solids come in exact (stage 2 of the exact kernel): each face on its file's own surface,
// a plane, cylinder, cone, sphere or torus as itself and a B-spline as that B-spline; each edge
// on its own curve, over the range its vertices give it; and each half-edge's course in its
// face's (u, v) built here, never read. A solid whose exact form does not hold together is left
// out, and the report says why: nothing faceted takes its place.

#include <gtest/gtest.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <iomanip>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "StepTestText.h"
#include "horizon/fileio/ImportReport.h"
#include "horizon/fileio/StepFormat.h"
#include "horizon/geometry/curves/AnalyticCurves.h"
#include "horizon/geometry/curves/NurbsCurve.h"
#include "horizon/geometry/surfaces/AnalyticSurfaces.h"
#include "horizon/geometry/surfaces/NurbsSurface.h"
#include "horizon/math/Constants.h"
#include "horizon/math/Vec3.h"
#include "horizon/modeling/BoundaryMesh.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/topology/ExactTolerance.h"
#include "horizon/topology/ExactValidator.h"
#include "horizon/topology/Solid.h"

using hz::io::ImportReport;
using hz::io::StepFormat;
using hz::math::Vec3;
using hz::step_text::bandCylinder;
using hz::step_text::cone;
using hz::step_text::externalStyleCylinder;
using hz::step_text::sphere;
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

/// A real as Part-21 writes one: always with its point.
std::string real(double v) {
    std::ostringstream out;
    out << std::setprecision(17) << v;
    std::string s = out.str();
    if (s.find_first_of(".eE") == std::string::npos) s += ".";
    return s;
}

/// externalStyleCylinder() with its side a rational B-spline: the same cylinder, a quadric
/// written as a B_SPLINE_SURFACE, from (1, 0) round about +z, as nine control points a row.
std::string splineCylinder() {
    const double w = std::sqrt(0.5);
    const double xs[9] = {1, 1, 0, -1, -1, -1, 0, 1, 1};
    const double ys[9] = {0, 1, 1, 1, 0, -1, -1, -1, 0};
    std::string points;
    std::string net;
    std::string weights;
    int id = 200;
    for (int i = 0; i < 9; ++i) {
        const double wi = i % 2 == 1 ? w : 1.0;
        net += std::string(i == 0 ? "(" : ",(");
        weights += std::string(i == 0 ? "(" : ",(") + real(wi) + "," + real(wi) + ")";
        for (int k = 0; k < 2; ++k) {
            points += "#" + std::to_string(id) + " = CARTESIAN_POINT('',(" + real(xs[i]) + "," +
                      real(ys[i]) + "," + real(2.0 * k) + "));\n";
            net += std::string(k == 0 ? "#" : ",#") + std::to_string(id);
            ++id;
        }
        net += ")";
    }
    const std::string surface =
        "#20 = ( BOUNDED_SURFACE() B_SPLINE_SURFACE(2,1,(" + net +
        "),.UNSPECIFIED.,.T.,.F.,.F.) B_SPLINE_SURFACE_WITH_KNOTS((3,2,2,2,3),(2,2),"
        "(0.,0.25,0.5,0.75,1.),(0.,1.),.UNSPECIFIED.) GEOMETRIC_REPRESENTATION_ITEM() "
        "RATIONAL_B_SPLINE_SURFACE((" +
        weights + ")) REPRESENTATION_ITEM('') SURFACE() );\n";
    return replaced(externalStyleCylinder(), "#20 = CYLINDRICAL_SURFACE('',#19,1.);\n",
                    points + surface);
}

/// A Part-21 DATA section written an entity at a time.
struct Data {
    std::string text;
    int next = 1;
    int add(const std::string& entity) {
        text += "#" + std::to_string(next) + " = " + entity + ";\n";
        return next++;
    }
    static std::string ref(int id) { return "#" + std::to_string(id); }
    int point(const Vec3& p) {
        return add("CARTESIAN_POINT('',(" + real(p.x) + "," + real(p.y) + "," + real(p.z) + "))");
    }
    int direction(const Vec3& d) {
        return add("DIRECTION('',(" + real(d.x) + "," + real(d.y) + "," + real(d.z) + "))");
    }
    int placement(const Vec3& origin, const Vec3& z, const Vec3& x) {
        const int o = point(origin);
        const int zd = direction(z);
        const int xd = direction(x);
        return add("AXIS2_PLACEMENT_3D(''," + ref(o) + "," + ref(zd) + "," + ref(xd) + ")");
    }
    /// A face of @p loop, each of whose edges is (EDGE_CURVE, along it), on @p surface.
    int face(const std::vector<std::pair<int, bool>>& loop, int surface, bool sameSense = true) {
        std::string list;
        for (const auto& [edge, along] : loop) {
            const int oe =
                add("ORIENTED_EDGE('',*,*," + ref(edge) + "," + (along ? ".T." : ".F.") + ")");
            list += (list.empty() ? "" : ",") + ref(oe);
        }
        const int edgeLoop = add("EDGE_LOOP('',(" + list + "))");
        const int bound = add("FACE_OUTER_BOUND(''," + ref(edgeLoop) + ",.T.)");
        return add("ADVANCED_FACE('',(" + ref(bound) + ")," + ref(surface) + "," +
                   (sameSense ? ".T." : ".F.") + ")");
    }
};

/// The closed shell of a block from (0, 0, 0) to (4, 4, 4) into @p d: six PLANE faces, each loop
/// counter-clockwise seen from outside. With @p frontFacingIn, the front face's (y = 0) plane
/// faces into the block while its face still says same_sense .T., as Horizon's own writer wrote
/// faces whichever way their surfaces were turned. Its CLOSED_SHELL's id.
int blockShell(Data& d, bool frontFacingIn = false) {
    const Vec3 corner[8] = {{0, 0, 0}, {4, 0, 0}, {4, 4, 0}, {0, 4, 0},
                            {0, 0, 4}, {4, 0, 4}, {4, 4, 4}, {0, 4, 4}};
    int points[8];
    int vertices[8];
    for (int i = 0; i < 8; ++i) {
        points[i] = d.point(corner[i]);
        vertices[i] = d.add("VERTEX_POINT(''," + Data::ref(points[i]) + ")");
    }
    const auto edge = [&](int a, int b) {
        const int dir = d.direction((corner[b] - corner[a]).normalized());
        const int vec = d.add("VECTOR(''," + Data::ref(dir) + ",1.)");
        const int line = d.add("LINE(''," + Data::ref(points[a]) + "," + Data::ref(vec) + ")");
        return d.add("EDGE_CURVE(''," + Data::ref(vertices[a]) + "," + Data::ref(vertices[b]) +
                     "," + Data::ref(line) + ",.T.)");
    };
    const int e1 = edge(0, 1);
    const int e2 = edge(1, 2);
    const int e3 = edge(2, 3);
    const int e4 = edge(3, 0);
    const int e5 = edge(4, 5);
    const int e6 = edge(5, 6);
    const int e7 = edge(6, 7);
    const int e8 = edge(7, 4);
    const int e9 = edge(0, 4);
    const int e10 = edge(1, 5);
    const int e11 = edge(2, 6);
    const int e12 = edge(3, 7);
    const auto plane = [&](const Vec3& origin, const Vec3& normal, const Vec3& x) {
        return d.add("PLANE(''," + Data::ref(d.placement(origin, normal, x)) + ")");
    };
    std::vector<int> faces;
    // Each loop counter-clockwise seen from outside.
    faces.push_back(d.face({{e4, false}, {e3, false}, {e2, false}, {e1, false}},
                           plane({0, 0, 0}, {0, 0, -1}, {1, 0, 0})));
    faces.push_back(d.face({{e5, true}, {e6, true}, {e7, true}, {e8, true}},
                           plane({0, 0, 4}, {0, 0, 1}, {1, 0, 0})));
    faces.push_back(d.face({{e1, true}, {e10, true}, {e5, false}, {e9, false}},
                           plane({0, 0, 0}, {0, frontFacingIn ? 1.0 : -1.0, 0}, {1, 0, 0})));
    faces.push_back(d.face({{e3, true}, {e12, true}, {e7, false}, {e11, false}},
                           plane({0, 4, 0}, {0, 1, 0}, {-1, 0, 0})));
    faces.push_back(d.face({{e9, true}, {e8, false}, {e12, false}, {e4, true}},
                           plane({0, 0, 0}, {-1, 0, 0}, {0, 1, 0})));
    faces.push_back(d.face({{e2, true}, {e11, true}, {e6, false}, {e10, false}},
                           plane({4, 0, 0}, {1, 0, 0}, {0, 1, 0})));
    std::string list;
    for (int f : faces) list += (list.empty() ? "" : ",") + Data::ref(f);
    return d.add("CLOSED_SHELL('',(" + list + "))");
}

/// blockShell()'s block alone, as one solid.
std::string block(bool frontFacingIn) {
    Data d;
    d.add("MANIFOLD_SOLID_BREP('block'," + Data::ref(blockShell(d, frontFacingIn)) + ")");
    return hz::step_text::stepFile(d.text);
}

/// The block with a ball of radius 1 cut out of its middle: its void a sphere whose one face is
/// bounded by a seam from pole to pole and back, as OpenCASCADE writes one, the shell facing out
/// of the void and turned round by its ORIENTED_CLOSED_SHELL, as the standard has every void.
std::string blockWithABall() {
    Data d;
    const int outer = blockShell(d);

    // The ball: a meridian from its south pole to its north, on its +x side.
    const Vec3 centre{2, 2, 2};
    const int ball = d.add("SPHERICAL_SURFACE(''," +
                           Data::ref(d.placement(centre, {0, 0, 1}, {1, 0, 0})) + ",1.)");
    const int south = d.add("VERTEX_POINT(''," + Data::ref(d.point({2, 2, 1})) + ")");
    const int north = d.add("VERTEX_POINT(''," + Data::ref(d.point({2, 2, 3})) + ")");
    const int meridian =
        d.add("CIRCLE(''," + Data::ref(d.placement(centre, {0, -1, 0}, {1, 0, 0})) + ",1.)");
    const int seam = d.add("EDGE_CURVE(''," + Data::ref(south) + "," + Data::ref(north) + "," +
                           Data::ref(meridian) + ",.T.)");
    const int skin = d.face({{seam, true}, {seam, false}}, ball);
    const int voidShell = d.add("CLOSED_SHELL('',(" + Data::ref(skin) + "))");
    const int inward = d.add("ORIENTED_CLOSED_SHELL('',*," + Data::ref(voidShell) + ",.F.)");
    d.add("BREP_WITH_VOIDS('block'," + Data::ref(outer) + ",(" + Data::ref(inward) + "))");
    return hz::step_text::stepFile(d.text);
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
// as the file gives it; its seam used twice by its side, which lays out clean; and the read point
// meshes it, its volume the cylinder's.
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
    EXPECT_NEAR(volume, 2.0 * hz::math::kPi, 2.0 * hz::math::kPi * 1e-3);
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
// normals point at its centre; the block's point out. (Its volume is Task 7's: the read point
// cannot mesh a face that comes to a pole yet.)
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

/// The nine control points (CCW about +z from (cos a, sin a, 0), r = 1) of the rational
/// quadratic circle, as CARTESIAN_POINTs from #@p first, and the curve as #@p id: a closed
/// B-spline whose join is at angle @p a.
std::string splineCircle(int id, int first, double a) {
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
                real(y) + ",0.));\n";
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

// Minor (e): edge ranges follow their sense. A cylinder whose bottom rim is two half circles, one
// .T. across its circle's t = 0, one .F. on a circle about -z, and whose top rim is one closed
// edge .F. on a circle about -z (the reviewer's probe): each edge runs from its start vertex
// through the half it bounds, and the solid is clean and measures 2 pi.
TEST(StepExact, EdgeRangesFollowTheirSenseAcrossAJoin) {
    const std::string data = R"(#1 = CARTESIAN_POINT('',(0.,0.,0.));
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
)";
    const auto solids =
        StepFormat::fromString(hz::step_text::stepFile(data), nullptr, nullptr, inMillimetres());
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
    EXPECT_NEAR(volume, 2.0 * hz::math::kPi, 2.0 * hz::math::kPi * 1e-3);
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

namespace {

/// A hemisphere of radius 10 as OpenCASCADE writes one: its cap bounded by the equator and a
/// seam meridian from the equator to the pole, used once each way, in the loop order @p order
/// (a rotation of "#20,#21,#22"), and a flat disc.
std::string seamedHemisphere(const std::string& order) {
    return hz::step_text::stepFile(R"(#1 = CARTESIAN_POINT('',(0.,0.,0.));
#2 = DIRECTION('',(0.,0.,1.));
#3 = DIRECTION('',(1.,0.,0.));
#4 = AXIS2_PLACEMENT_3D('',#1,#2,#3);
#5 = SPHERICAL_SURFACE('',#4,10.);
#6 = CIRCLE('',#4,10.);
#7 = CARTESIAN_POINT('',(10.,0.,0.));
#8 = VERTEX_POINT('',#7);
#9 = CARTESIAN_POINT('',(0.,0.,10.));
#10 = VERTEX_POINT('',#9);
#11 = DIRECTION('',(0.,-1.,0.));
#12 = AXIS2_PLACEMENT_3D('',#1,#11,#3);
#13 = CIRCLE('',#12,10.);
#14 = EDGE_CURVE('',#8,#8,#6,.T.);
#15 = EDGE_CURVE('',#8,#10,#13,.T.);
#16 = DIRECTION('',(0.,0.,-1.));
#17 = AXIS2_PLACEMENT_3D('',#1,#16,#3);
#18 = PLANE('',#17);
#20 = ORIENTED_EDGE('',*,*,#14,.T.);
#21 = ORIENTED_EDGE('',*,*,#15,.T.);
#22 = ORIENTED_EDGE('',*,*,#15,.F.);
#23 = EDGE_LOOP('',()" + order + R"());
#24 = FACE_OUTER_BOUND('',#23,.T.);
#25 = ADVANCED_FACE('',(#24),#5,.T.);
#26 = ORIENTED_EDGE('',*,*,#14,.F.);
#27 = EDGE_LOOP('',(#26));
#28 = FACE_OUTER_BOUND('',#27,.T.);
#29 = ADVANCED_FACE('',(#28),#18,.T.);
#30 = CLOSED_SHELL('',(#25,#29));
#31 = MANIFOLD_SOLID_BREP('hemisphere',#30);
)");
}

}  // namespace

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
        // The read point's mesh, to its own error until Task 7, which tightens this to 1e-9.
        EXPECT_NEAR(hz::model::MassPropertiesCalculator::compute(solid).volume, c.volume,
                    c.volume * 1e-3);
        // The faceted copy, over the carriers split with the exact edges.
        const auto ideal = hz::model::MassPropertiesCalculator::computeIdeal(solid);
        EXPECT_TRUE(ideal.onIdealSurfaces);
        EXPECT_NEAR(ideal.properties.volume, c.volume, c.volume * 1e-8);
    }
}
