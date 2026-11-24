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
