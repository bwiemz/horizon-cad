#include <gtest/gtest.h>

#include <cctype>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "horizon/fileio/StepFormat.h"
#include "horizon/geometry/curves/NurbsCurve.h"
#include "horizon/geometry/surfaces/NurbsSurface.h"
#include "horizon/math/Constants.h"
#include "horizon/modeling/BooleanOp.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/modeling/Pattern.h"
#include "horizon/modeling/PrimitiveFactory.h"
#include "horizon/topology/Solid.h"

using hz::io::StepFormat;
using hz::math::Vec3;
using hz::model::BooleanOp;
using hz::model::BooleanType;
using hz::model::MassPropertiesCalculator;
using hz::model::Pattern;
using hz::model::PrimitiveFactory;

namespace {

std::vector<const hz::topo::Solid*> refs(const hz::topo::Solid& s) {
    return {&s};
}

/// Volume/area comparison between an original solid and its re-import.
void expectSameMassProperties(const hz::topo::Solid& original, const hz::topo::Solid& imported,
                              double relTol = 1e-6) {
    const auto mpA = MassPropertiesCalculator::compute(original);
    const auto mpB = MassPropertiesCalculator::compute(imported);
    ASSERT_TRUE(mpA.valid);
    ASSERT_TRUE(mpB.valid);
    EXPECT_NEAR(mpA.volume, mpB.volume, std::abs(mpA.volume) * relTol);
    EXPECT_NEAR(mpA.surfaceArea, mpB.surfaceArea, std::abs(mpA.surfaceArea) * relTol);
    EXPECT_NEAR((mpA.centerOfMass - mpB.centerOfMass).length(), 0.0, 1e-6);
}

std::filesystem::path tempStepPath(const std::string& name) {
    return std::filesystem::temp_directory_path() / name;
}

}  // namespace

// ===========================================================================
// Export
// ===========================================================================

TEST(StepFormat, ExportBoxContainsRequiredEntities) {
    auto box = PrimitiveFactory::makeBox(10.0, 20.0, 30.0);
    ASSERT_NE(box, nullptr);

    const std::string text = StepFormat::toString(refs(*box));

    EXPECT_NE(text.find("ISO-10303-21;"), std::string::npos);
    EXPECT_NE(text.find("AP242_MANAGED_MODEL_BASED_3D_ENGINEERING"), std::string::npos);
    EXPECT_NE(text.find("MANIFOLD_SOLID_BREP"), std::string::npos);
    EXPECT_NE(text.find("CLOSED_SHELL"), std::string::npos);
    EXPECT_NE(text.find("ADVANCED_BREP_SHAPE_REPRESENTATION"), std::string::npos);
    EXPECT_NE(text.find("SHAPE_DEFINITION_REPRESENTATION"), std::string::npos);
    EXPECT_NE(text.find("END-ISO-10303-21;"), std::string::npos);

    // A box has 6 faces, 12 edges, 8 vertices.
    size_t faceCount = 0;
    for (size_t pos = text.find("ADVANCED_FACE"); pos != std::string::npos;
         pos = text.find("ADVANCED_FACE", pos + 1)) {
        ++faceCount;
    }
    EXPECT_EQ(faceCount, 6u);
}

TEST(StepFormat, SaveWritesFileAndLoadReadsIt) {
    auto box = PrimitiveFactory::makeBox(5.0, 5.0, 5.0);
    ASSERT_NE(box, nullptr);

    const auto path = tempStepPath("hz_step_box.step");
    ASSERT_TRUE(StepFormat::save(path.string(), refs(*box)));

    auto solids = StepFormat::load(path.string());
    EXPECT_EQ(solids.size(), 1u) << StepFormat::lastError();
    std::filesystem::remove(path);
}

TEST(StepFormat, SaveEmptyFails) {
    EXPECT_FALSE(StepFormat::save(tempStepPath("hz_step_none.step").string(), {}));
    EXPECT_FALSE(StepFormat::lastError().empty());
}

// ===========================================================================
// Round-trips
// ===========================================================================

TEST(StepFormat, BoxRoundTrip) {
    auto box = PrimitiveFactory::makeBox(10.0, 20.0, 30.0);
    ASSERT_NE(box, nullptr);

    auto solids = StepFormat::fromString(StepFormat::toString(refs(*box)));
    ASSERT_EQ(solids.size(), 1u) << StepFormat::lastError();

    const auto& imported = *solids[0];
    EXPECT_EQ(imported.vertexCount(), 8u);
    EXPECT_EQ(imported.edgeCount(), 12u);
    EXPECT_EQ(imported.faceCount(), 6u);
    EXPECT_TRUE(imported.isValid()) << imported.validationReport();
    EXPECT_TRUE(imported.checkEulerFormula());
    EXPECT_TRUE(imported.checkManifold());
    expectSameMassProperties(*box, imported);
}

TEST(StepFormat, CylinderRoundTripPreservesRationalGeometry) {
    auto cyl = PrimitiveFactory::makeCylinder(4.0, 12.0);
    ASSERT_NE(cyl, nullptr);

    const std::string text = StepFormat::toString(refs(*cyl));
    // The lateral surface is rational — the complex-instance form must appear.
    EXPECT_NE(text.find("RATIONAL_B_SPLINE_SURFACE"), std::string::npos);

    auto solids = StepFormat::fromString(text);
    ASSERT_EQ(solids.size(), 1u) << StepFormat::lastError();

    const auto& imported = *solids[0];
    EXPECT_EQ(imported.faceCount(), cyl->faceCount());
    EXPECT_TRUE(imported.isValid()) << imported.validationReport();
    expectSameMassProperties(*cyl, imported);
}

TEST(StepFormat, SphereRoundTrip) {
    auto sphere = PrimitiveFactory::makeSphere(6.0);
    ASSERT_NE(sphere, nullptr);

    auto solids = StepFormat::fromString(StepFormat::toString(refs(*sphere)));
    ASSERT_EQ(solids.size(), 1u) << StepFormat::lastError();
    EXPECT_TRUE(solids[0]->isValid()) << solids[0]->validationReport();
    expectSameMassProperties(*sphere, *solids[0]);
}

TEST(StepFormat, ConeRoundTrip) {
    auto cone = PrimitiveFactory::makeCone(5.0, 2.0, 8.0);
    ASSERT_NE(cone, nullptr);

    auto solids = StepFormat::fromString(StepFormat::toString(refs(*cone)));
    ASSERT_EQ(solids.size(), 1u) << StepFormat::lastError();
    EXPECT_TRUE(solids[0]->isValid()) << solids[0]->validationReport();
    EXPECT_EQ(solids[0]->faceCount(), cone->faceCount());
    expectSameMassProperties(*cone, *solids[0]);
}

/// The BSP-CSG Boolean splits faces at the A/B boundary and sews a manifold
/// result, so Boolean output now survives a full STEP round trip.  (The
/// phase-36 face-level Boolean left an unstitched seam here, which this test
/// used to pin as a detected import failure.)
TEST(StepFormat, BooleanResultRoundTripsThroughStep) {
    auto a = PrimitiveFactory::makeBox(10.0, 10.0, 10.0);
    auto b = PrimitiveFactory::makeBox(4.0, 4.0, 20.0);
    ASSERT_NE(a, nullptr);
    ASSERT_NE(b, nullptr);

    auto cut = BooleanOp::execute(*a, *b, BooleanType::Subtract);
    ASSERT_NE(cut, nullptr);
    EXPECT_TRUE(cut->checkManifold()) << cut->validationReport();

    const std::string text = StepFormat::toString(refs(*cut));
    EXPECT_NE(text.find("MANIFOLD_SOLID_BREP"), std::string::npos);

    auto solids = StepFormat::fromString(text);
    ASSERT_EQ(solids.size(), 1u) << StepFormat::lastError();
    expectSameMassProperties(*cut, *solids[0]);
}

TEST(StepFormat, MultiSolidRoundTrip) {
    auto box = PrimitiveFactory::makeBox(2.0, 3.0, 4.0);
    auto cyl = PrimitiveFactory::makeCylinder(1.5, 5.0);
    ASSERT_NE(box, nullptr);
    ASSERT_NE(cyl, nullptr);

    auto solids = StepFormat::fromString(StepFormat::toString({box.get(), cyl.get()}));
    ASSERT_EQ(solids.size(), 2u) << StepFormat::lastError();
    EXPECT_TRUE(solids[0]->isValid());
    EXPECT_TRUE(solids[1]->isValid());
    expectSameMassProperties(*box, *solids[0]);
    expectSameMassProperties(*cyl, *solids[1]);
}

/// Horizon multi-shell solids (Pattern results) export one MANIFOLD_SOLID_BREP
/// per shell inside a shared representation and re-import as ONE solid.
TEST(StepFormat, MultiShellPatternRoundTrip) {
    auto box = PrimitiveFactory::makeBox(2.0, 2.0, 2.0);
    ASSERT_NE(box, nullptr);
    auto pattern = Pattern::linear(*box, Vec3{1.0, 0.0, 0.0}, 5.0, 2);
    ASSERT_NE(pattern, nullptr);
    ASSERT_EQ(pattern->shellCount(), 2u);

    const std::string text = StepFormat::toString(refs(*pattern));
    auto solids = StepFormat::fromString(text);
    ASSERT_EQ(solids.size(), 1u) << StepFormat::lastError();

    const auto& imported = *solids[0];
    EXPECT_EQ(imported.shellCount(), 2u);
    EXPECT_EQ(imported.faceCount(), 12u);
    EXPECT_EQ(imported.vertexCount(), 16u);
    EXPECT_TRUE(imported.isValid()) << imported.validationReport();
    expectSameMassProperties(*pattern, imported);
}

/// Part-21 reals must always carry a '.' in the mantissa (ISO 10303-21
/// clause 6.4.2): "1E-07" is invalid, "1.E-07" is required.
TEST(StepFormat, SmallCoordinatesEmitValidPart21Reals) {
    auto box = PrimitiveFactory::makeBox(1e-7, 1.0, 2e+16);
    ASSERT_NE(box, nullptr);
    const std::string text = StepFormat::toString(refs(*box));

    for (size_t pos = text.find('E'); pos != std::string::npos; pos = text.find('E', pos + 1)) {
        const char next = pos + 1 < text.size() ? text[pos + 1] : '\0';
        if (std::isdigit(static_cast<unsigned char>(next)) == 0 && next != '+' && next != '-') {
            continue;  // not an exponent (keyword letter)
        }
        // Scan the mantissa backwards: digits are fine, but a '.' must appear
        // before any non-numeric character.
        size_t q = pos;
        bool sawDot = false;
        bool sawDigit = false;
        while (q > 0) {
            const char c = text[q - 1];
            if (std::isdigit(static_cast<unsigned char>(c)) != 0) {
                sawDigit = true;
                --q;
            } else if (c == '.') {
                sawDot = true;
                break;
            } else {
                break;
            }
        }
        if (sawDigit) {
            EXPECT_TRUE(sawDot) << "invalid Part-21 real near: "
                                << text.substr(q, std::min<size_t>(16, text.size() - q));
        }
    }
}

/// Era-2 stabilization criterion: export → import → export must be stable.
TEST(StepFormat, RoundTripIsIdempotent) {
    auto box = PrimitiveFactory::makeBox(7.0, 8.0, 9.0);
    ASSERT_NE(box, nullptr);

    const std::string first = StepFormat::toString(refs(*box));
    auto imported = StepFormat::fromString(first);
    ASSERT_EQ(imported.size(), 1u) << StepFormat::lastError();

    const std::string second = StepFormat::toString(refs(*imported[0]));
    auto reimported = StepFormat::fromString(second);
    ASSERT_EQ(reimported.size(), 1u) << StepFormat::lastError();

    // Semantic stability: identical topology counts and mass properties.
    EXPECT_EQ(imported[0]->vertexCount(), reimported[0]->vertexCount());
    EXPECT_EQ(imported[0]->edgeCount(), reimported[0]->edgeCount());
    EXPECT_EQ(imported[0]->faceCount(), reimported[0]->faceCount());
    expectSameMassProperties(*imported[0], *reimported[0]);
}

// ===========================================================================
// External-file interop (analytic geometry)
// ===========================================================================

namespace {

/// A minimal AP214/AP242-style unit cube authored the way external CAD systems
/// typically export planar solids: PLANE surfaces and LINE edge geometry.
std::string externalStyleCube() {
    std::string s = R"(ISO-10303-21;
HEADER;
FILE_DESCRIPTION((''),'2;1');
FILE_NAME('','',(''),(''),'','','');
FILE_SCHEMA(('AP242_MANAGED_MODEL_BASED_3D_ENGINEERING_MIM_LF { 1 0 10303 442 3 1 4 }'));
ENDSEC;
DATA;
)";
    int id = 1;
    auto add = [&](const std::string& rhs) {
        s += "#" + std::to_string(id) + " = " + rhs + ";\n";
        return id++;
    };
    auto num = [](double v) {
        char buf[32];
        std::snprintf(buf, sizeof(buf), "%.9g", v);
        std::string r(buf);
        if (r.find('.') == std::string::npos && r.find('E') == std::string::npos &&
            r.find('e') == std::string::npos) {
            r += '.';
        }
        return r;
    };

    // Cube corners: index bit 0 → x, bit 1 → y, bit 2 → z.
    int pts[8];
    int vps[8];
    for (int i = 0; i < 8; ++i) {
        const double x = (i & 1) ? 1.0 : 0.0;
        const double y = (i & 2) ? 1.0 : 0.0;
        const double z = (i & 4) ? 1.0 : 0.0;
        pts[i] = add("CARTESIAN_POINT('',(" + num(x) + "," + num(y) + "," + num(z) + "))");
        vps[i] = add("VERTEX_POINT('',#" + std::to_string(pts[i]) + ")");
    }

    const int dirX = add("DIRECTION('',(1.,0.,0.))");

    // Undirected cube edges as vertex-index pairs.
    const int edgeVerts[12][2] = {{0, 1}, {1, 3}, {3, 2}, {2, 0},   // bottom (z=0)
                                  {4, 5}, {5, 7}, {7, 6}, {6, 4},   // top (z=1)
                                  {0, 4}, {1, 5}, {3, 7}, {2, 6}};  // verticals
    int edges[12];
    for (int e = 0; e < 12; ++e) {
        const int a = edgeVerts[e][0];
        const int b = edgeVerts[e][1];
        const int line = add("LINE('',#" + std::to_string(pts[a]) + ",VECTOR('',#" +
                             std::to_string(dirX) + ",1.))");
        edges[e] = add("EDGE_CURVE('',#" + std::to_string(vps[a]) + ",#" + std::to_string(vps[b]) +
                       ",#" + std::to_string(line) + ",.T.)");
    }

    // Faces as loops of (edge index, forward?) — CCW seen from outside.
    struct FaceDef {
        int loop[4][2];
        double origin[3];
        double normal[3];
    };
    const FaceDef faceDefs[6] = {
        // bottom z=0, normal -Z: 0→2→3→1
        {{{3, false}, {2, false}, {1, false}, {0, false}}, {0, 0, 0}, {0, 0, -1}},
        // top z=1, normal +Z: 4→5→7→6
        {{{4, true}, {5, true}, {6, true}, {7, true}}, {0, 0, 1}, {0, 0, 1}},
        // front y=0, normal -Y: 0→1→5→4
        {{{0, true}, {9, true}, {4, false}, {8, false}}, {0, 0, 0}, {0, -1, 0}},
        // back y=1, normal +Y: 2→6→7→3
        {{{11, true}, {6, false}, {10, false}, {2, true}}, {0, 1, 0}, {0, 1, 0}},
        // left x=0, normal -X: 0→4→6→2
        {{{8, true}, {7, false}, {11, false}, {3, true}}, {0, 0, 0}, {-1, 0, 0}},
        // right x=1, normal +X: 1→3→7→5
        {{{1, true}, {10, true}, {5, false}, {9, false}}, {1, 0, 0}, {1, 0, 0}},
    };

    std::vector<int> faces;
    for (const FaceDef& fd : faceDefs) {
        std::string oeList;
        for (int k = 0; k < 4; ++k) {
            const int oe = add("ORIENTED_EDGE('',*,*,#" + std::to_string(edges[fd.loop[k][0]]) +
                               "," + (fd.loop[k][1] ? ".T." : ".F.") + ")");
            if (k) oeList += ',';
            oeList += "#" + std::to_string(oe);
        }
        const int loop = add("EDGE_LOOP('',(" + oeList + "))");
        const int bound = add("FACE_OUTER_BOUND('',#" + std::to_string(loop) + ",.T.)");
        const int origin = add("CARTESIAN_POINT('',(" + num(fd.origin[0]) + "," +
                               num(fd.origin[1]) + "," + num(fd.origin[2]) + "))");
        const int normal = add("DIRECTION('',(" + num(fd.normal[0]) + "," + num(fd.normal[1]) +
                               "," + num(fd.normal[2]) + "))");
        const int placement = add("AXIS2_PLACEMENT_3D('',#" + std::to_string(origin) + ",#" +
                                  std::to_string(normal) + ",$)");
        const int plane = add("PLANE('',#" + std::to_string(placement) + ")");
        faces.push_back(add("ADVANCED_FACE('',(#" + std::to_string(bound) + "),#" +
                            std::to_string(plane) + ",.T.)"));
    }

    std::string faceList;
    for (size_t i = 0; i < faces.size(); ++i) {
        if (i) faceList += ',';
        faceList += "#" + std::to_string(faces[i]);
    }
    const int shell = add("CLOSED_SHELL('',(" + faceList + "))");
    add("MANIFOLD_SOLID_BREP('cube',#" + std::to_string(shell) + ")");

    s += "ENDSEC;\nEND-ISO-10303-21;\n";
    return s;
}

}  // namespace

TEST(StepFormat, ImportsExternalPlanarSolid) {
    auto solids = StepFormat::fromString(externalStyleCube());
    ASSERT_EQ(solids.size(), 1u) << StepFormat::lastError();

    const auto& cube = *solids[0];
    EXPECT_EQ(cube.vertexCount(), 8u);
    EXPECT_EQ(cube.edgeCount(), 12u);
    EXPECT_EQ(cube.faceCount(), 6u);
    EXPECT_TRUE(cube.isValid()) << cube.validationReport();

    const auto mp = MassPropertiesCalculator::compute(cube);
    ASSERT_TRUE(mp.valid);
    EXPECT_NEAR(mp.volume, 1.0, 1e-6);
    EXPECT_NEAR(mp.surfaceArea, 6.0, 1e-6);
}

namespace {

/// A foreign-style cylinder (radius 1, height 2, axis +Z) the way OCC-based
/// systems export it: CIRCLE rim edges (closed, seam vertex NOT on the
/// placement X-axis), a seam LINE wrapped in SURFACE_CURVE, PLANE caps —
/// the bottom cap with an inward normal declared via same_sense = .F. —
/// and a CYLINDRICAL_SURFACE lateral face.
std::string externalStyleCylinder() {
    return R"(ISO-10303-21;
HEADER;
FILE_SCHEMA(('AP242_MANAGED_MODEL_BASED_3D_ENGINEERING_MIM_LF { 1 0 10303 442 3 1 4 }'));
ENDSEC;
DATA;
#1 = CARTESIAN_POINT('',(0.,0.,0.));
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
#41 = MANIFOLD_SOLID_BREP('cyl',#40);
ENDSEC;
END-ISO-10303-21;
)";
}

}  // namespace

TEST(StepFormat, ImportsExternalAnalyticCylinder) {
    auto solids = StepFormat::fromString(externalStyleCylinder());
    ASSERT_EQ(solids.size(), 1u) << StepFormat::lastError();

    const auto& cyl = *solids[0];
    EXPECT_EQ(cyl.vertexCount(), 2u);
    EXPECT_EQ(cyl.edgeCount(), 3u);
    EXPECT_EQ(cyl.faceCount(), 3u);
    EXPECT_TRUE(cyl.isValid()) << cyl.validationReport();

    // Seam anchoring: the bottom rim's CIRCLE placement uses X = +Y, but the
    // seam vertex sits at (1,0,0) — the reconstructed closed curve must
    // start/end at the vertex, not at the placement X-axis.
    bool checkedBottomRim = false;
    for (const auto& e : cyl.edges()) {
        if (e.halfEdge == nullptr || e.curve == nullptr) continue;
        const bool closed = e.halfEdge->origin == e.halfEdge->twin->origin;
        if (!closed || e.halfEdge->origin->point.z > 0.5) continue;
        const Vec3 start = e.curve->evaluate(e.curve->tMin());
        EXPECT_NEAR((start - Vec3{1.0, 0.0, 0.0}).length(), 0.0, 1e-9);
        checkedBottomRim = true;
    }
    EXPECT_TRUE(checkedBottomRim);

    // same_sense handling: the bottom cap's PLANE normal points +Z (into the
    // solid) with same_sense=.F., so the imported surface normal must point
    // -Z (outward); the top cap keeps +Z.
    bool sawDown = false;
    bool sawUp = false;
    for (const auto& f : cyl.faces()) {
        if (f.surface == nullptr) continue;
        const double midU = (f.surface->uMin() + f.surface->uMax()) / 2.0;
        const double midV = (f.surface->vMin() + f.surface->vMax()) / 2.0;
        const Vec3 n = f.surface->normal(midU, midV);
        if (n.z < -0.99) sawDown = true;
        if (n.z > 0.99) sawUp = true;
    }
    EXPECT_TRUE(sawDown) << "bottom cap normal was not flipped by same_sense = .F.";
    EXPECT_TRUE(sawUp);
}

// ===========================================================================
// Robustness
// ===========================================================================

TEST(StepFormat, RejectsDeeplyNestedInput) {
    std::string text = "ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\n#1 = FOO(";
    for (int i = 0; i < 100000; ++i) text += '(';
    text += ");\nENDSEC;\nEND-ISO-10303-21;\n";

    // Must fail cleanly (bounded recursion), not overflow the stack.
    auto solids = StepFormat::fromString(text);
    EXPECT_TRUE(solids.empty());
    EXPECT_FALSE(StepFormat::lastError().empty());
}

TEST(StepFormat, RejectsAbsurdKnotMultiplicities) {
    // A curve claiming two billion-fold knots must be rejected before the
    // expansion allocates, not after.
    const std::string text = R"(ISO-10303-21;
HEADER;
ENDSEC;
DATA;
#1 = CARTESIAN_POINT('',(0.,0.,0.));
#2 = CARTESIAN_POINT('',(1.,0.,0.));
#3 = VERTEX_POINT('',#1);
#4 = VERTEX_POINT('',#2);
#5 = B_SPLINE_CURVE_WITH_KNOTS('',1,(#1,#2),.UNSPECIFIED.,.F.,.F.,(2000000000,2000000000),(0.,1.),.UNSPECIFIED.);
#6 = EDGE_CURVE('',#3,#4,#5,.T.);
#7 = ORIENTED_EDGE('',*,*,#6,.T.);
#8 = EDGE_LOOP('',(#7));
#9 = FACE_OUTER_BOUND('',#8,.T.);
#10 = AXIS2_PLACEMENT_3D('',#1,$,$);
#11 = PLANE('',#10);
#12 = ADVANCED_FACE('',(#9),#11,.T.);
#13 = CLOSED_SHELL('',(#12));
#14 = MANIFOLD_SOLID_BREP('bad',#13);
ENDSEC;
END-ISO-10303-21;
)";
    auto solids = StepFormat::fromString(text);
    EXPECT_TRUE(solids.empty());
    EXPECT_FALSE(StepFormat::lastError().empty());
}

TEST(StepFormat, RejectsGarbageInput) {
    auto solids = StepFormat::fromString("this is not a step file");
    EXPECT_TRUE(solids.empty());
    EXPECT_FALSE(StepFormat::lastError().empty());
}

TEST(StepFormat, RejectsFileWithoutSolids) {
    const std::string text =
        "ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\n#1 = CARTESIAN_POINT('',(0.,0.,0.));\n"
        "ENDSEC;\nEND-ISO-10303-21;\n";
    auto solids = StepFormat::fromString(text);
    EXPECT_TRUE(solids.empty());
    EXPECT_FALSE(StepFormat::lastError().empty());
}

TEST(StepFormat, RejectsTruncatedFile) {
    auto box = PrimitiveFactory::makeBox(1.0, 1.0, 1.0);
    ASSERT_NE(box, nullptr);
    std::string text = StepFormat::toString(refs(*box));
    text.resize(text.size() / 2);

    auto solids = StepFormat::fromString(text);
    // Must not crash; a half file either fails cleanly or yields nothing.
    EXPECT_TRUE(solids.empty());
}

TEST(StepFormat, RejectsNonManifoldShell) {
    // A single square face is not a closed solid — edge use count is 1.
    std::string text = R"(ISO-10303-21;
HEADER;
FILE_SCHEMA(('AP242_MANAGED_MODEL_BASED_3D_ENGINEERING_MIM_LF { 1 0 10303 442 3 1 4 }'));
ENDSEC;
DATA;
#1 = CARTESIAN_POINT('',(0.,0.,0.));
#2 = CARTESIAN_POINT('',(1.,0.,0.));
#3 = CARTESIAN_POINT('',(1.,1.,0.));
#4 = CARTESIAN_POINT('',(0.,1.,0.));
#5 = VERTEX_POINT('',#1);
#6 = VERTEX_POINT('',#2);
#7 = VERTEX_POINT('',#3);
#8 = VERTEX_POINT('',#4);
#9 = DIRECTION('',(1.,0.,0.));
#10 = LINE('',#1,VECTOR('',#9,1.));
#11 = EDGE_CURVE('',#5,#6,#10,.T.);
#12 = EDGE_CURVE('',#6,#7,#10,.T.);
#13 = EDGE_CURVE('',#7,#8,#10,.T.);
#14 = EDGE_CURVE('',#8,#5,#10,.T.);
#15 = ORIENTED_EDGE('',*,*,#11,.T.);
#16 = ORIENTED_EDGE('',*,*,#12,.T.);
#17 = ORIENTED_EDGE('',*,*,#13,.T.);
#18 = ORIENTED_EDGE('',*,*,#14,.T.);
#19 = EDGE_LOOP('',(#15,#16,#17,#18));
#20 = FACE_OUTER_BOUND('',#19,.T.);
#21 = CARTESIAN_POINT('',(0.,0.,0.));
#22 = DIRECTION('',(0.,0.,1.));
#23 = AXIS2_PLACEMENT_3D('',#21,#22,$);
#24 = PLANE('',#23);
#25 = ADVANCED_FACE('',(#20),#24,.T.);
#26 = CLOSED_SHELL('',(#25));
#27 = MANIFOLD_SOLID_BREP('open',#26);
ENDSEC;
END-ISO-10303-21;
)";
    auto solids = StepFormat::fromString(text);
    EXPECT_TRUE(solids.empty());
    EXPECT_NE(StepFormat::lastError().find("manifold"), std::string::npos)
        << StepFormat::lastError();
}

TEST(StepFormat, LoadMissingFileFails) {
    auto solids = StepFormat::load("Z:/definitely/not/a/real/path.step");
    EXPECT_TRUE(solids.empty());
    EXPECT_FALSE(StepFormat::lastError().empty());
}
