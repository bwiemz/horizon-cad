// Mass properties of solids with exact faces (stage 2 of the exact kernel). Each face's share of
// the volume, its moments and the area is integrated on its own surface, over its own region in
// (u, v), by the divergence theorem and then Green's theorem along its pcurves: a solid bounded by
// closed-form surfaces measures as its closed forms, to 1e-9, and no mesh is made for it. A face
// that cannot be integrated leaves the solid's mass not valid, and says why.

#include <gtest/gtest.h>

#include <atomic>
#include <cmath>
#include <memory>
#include <numbers>
#include <stdexcept>
#include <string>
#include <vector>

#include "StepTestText.h"
#include "horizon/fileio/StepFormat.h"
#include "horizon/geometry/Frame.h"
#include "horizon/geometry/curves/Curve2D.h"
#include "horizon/geometry/curves/Curve3.h"
#include "horizon/geometry/surfaces/AnalyticSurfaces.h"
#include "horizon/geometry/surfaces/NurbsSurface.h"
#include "horizon/math/Mat4.h"
#include "horizon/math/Vec2.h"
#include "horizon/math/Vec3.h"
#include "horizon/modeling/ExactPrimitives.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/modeling/Pattern.h"
#include "horizon/modeling/PrimitiveFactory.h"
#include "horizon/topology/ExactTolerance.h"
#include "horizon/topology/HalfEdge.h"
#include "horizon/topology/Solid.h"

namespace {

namespace geo = hz::geo;
namespace topo = hz::topo;
using hz::io::StepFormat;
using hz::math::Vec2;
using hz::math::Vec3;
using hz::model::IdealMassProperties;
using hz::model::MassProperties;
using hz::model::MassPropertiesCalculator;

constexpr double kPi = std::numbers::pi;

/// These files name no length unit, as hand-written ones often do: read in millimetres.
hz::io::StepReadOptions inMillimetres() {
    hz::io::StepReadOptions options;
    options.unknownLengthUnit = hz::math::LengthUnit::Millimetre;
    return options;
}

/// The one solid @p text holds, as the reader makes it; null (and a failure) when it holds none.
std::unique_ptr<topo::Solid> readOne(const std::string& text) {
    auto solids = StepFormat::fromString(text, nullptr, nullptr, inMillimetres());
    EXPECT_EQ(solids.size(), 1u) << StepFormat::lastError();
    if (solids.size() != 1) return nullptr;
    return std::move(solids.front());
}

void expectRelative(double actual, double expected, double relative, const std::string& what) {
    EXPECT_NEAR(actual, expected, relative * std::abs(expected)) << what;
}

/// A frustum from radius 1 at z = 0 to radius 2 at z = 2, written as externalStyleCylinder()
/// writes its cylinder: a CONICAL_SURFACE side between two rims and a seam, two PLANE caps.
std::string frustum() {
    return hz::step_text::stepFile(R"(#1 = CARTESIAN_POINT('',(0.,0.,0.));
#2 = DIRECTION('',(0.,0.,1.));
#3 = DIRECTION('',(0.,1.,0.));
#4 = AXIS2_PLACEMENT_3D('',#1,#2,#3);
#5 = CIRCLE('',#4,1.);
#6 = CARTESIAN_POINT('',(1.,0.,0.));
#7 = VERTEX_POINT('',#6);
#8 = CARTESIAN_POINT('',(0.,0.,2.));
#9 = DIRECTION('',(1.,0.,0.));
#10 = AXIS2_PLACEMENT_3D('',#8,#2,#9);
#11 = CIRCLE('',#10,2.);
#12 = CARTESIAN_POINT('',(2.,0.,2.));
#13 = VERTEX_POINT('',#12);
#14 = EDGE_CURVE('',#7,#7,#5,.T.);
#15 = EDGE_CURVE('',#13,#13,#11,.T.);
#50 = DIRECTION('',(0.44721359549995793,0.,0.89442719099991586));
#16 = LINE('',#6,VECTOR('',#50,1.));
#17 = SURFACE_CURVE('',#16,(),.CURVE_3D.);
#18 = EDGE_CURVE('',#7,#13,#17,.T.);
#19 = AXIS2_PLACEMENT_3D('',#1,#2,#9);
#20 = CONICAL_SURFACE('',#19,1.,0.46364760900080612);
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
#41 = MANIFOLD_SOLID_BREP('frustum',#40);
)");
}

/// The live face of @p solid tagged @p tag; null when there is none.
topo::Face* faceTagged(topo::Solid& solid, const std::string& tag) {
    for (auto& f : solid.faces()) {
        if (f.alive && f.topoId.tag() == tag) return &f;
    }
    return nullptr;
}

/// The solid's volume and area as computeIdeal gives them agree with compute()'s, which is
/// exact: no refinement, nothing extrapolated.
void expectIdealIsTheSame(const topo::Solid& solid, const MassProperties& props) {
    const IdealMassProperties ideal = MassPropertiesCalculator::computeIdeal(solid);
    EXPECT_TRUE(ideal.properties.valid) << ideal.properties.problem;
    EXPECT_DOUBLE_EQ(ideal.properties.volume, props.volume);
    EXPECT_DOUBLE_EQ(ideal.properties.surfaceArea, props.surfaceArea);
    EXPECT_TRUE(ideal.onIdealSurfaces);
    EXPECT_TRUE(ideal.withinTolerance);
    EXPECT_LE(ideal.estimatedError, 1e-9);
    EXPECT_GT(ideal.refinement, 1) << "the mass dialog says what was estimated";
}

/// A straight pcurve from @p a to @p b that wiggles a millionth across its course 100,000 times:
/// a boundary whose integral no number of subdivisions within the budget settles.
class Wiggle : public geo::Curve2D {
public:
    Wiggle(const Vec2& a, const Vec2& b) : a_(a), b_(b) {
        const Vec2 d = b - a;
        across_ = Vec2(-d.y, d.x) * (1e-6 / d.length());
    }
    Vec2 evaluate(double t) const override {
        return a_ + (b_ - a_) * t + across_ * std::sin(kWaves * t);
    }
    Vec2 derivative(double t, int order) const override {
        if (order == 1) return (b_ - a_) + across_ * (kWaves * std::cos(kWaves * t));
        return across_ * (-kWaves * kWaves * std::sin(kWaves * t));
    }
    double tMin() const override { return 0.0; }
    double tMax() const override { return 1.0; }
    bool isClosed() const override { return false; }

private:
    static constexpr double kWaves = 2.0 * kPi * 1e5;
    Vec2 a_;
    Vec2 b_;
    Vec2 across_;
};

}  // namespace

// Volume, area, centre and inertia, on a tilted axis away from the origin, within 1e-9 of the
// closed forms.
TEST(ExactMass, ACylinderIsExact) {
    const Vec3 axis = Vec3{1, 2, 2} * (1.0 / 3.0);
    const geo::Frame frame = geo::Frame::fromAxis({3, -2, 7}, axis, {0, 0, 1});
    const double r = 5.0;
    const double h = 10.0;
    const auto cyl = hz::model::exact::makeCylinder(frame, r, h);
    ASSERT_NE(cyl, nullptr);
    const MassProperties props = MassPropertiesCalculator::compute(*cyl);
    ASSERT_TRUE(props.valid) << props.problem;
    EXPECT_TRUE(props.problem.empty());

    const double volume = kPi * r * r * h;
    expectRelative(props.volume, volume, 1e-9, "its volume");
    expectRelative(props.surfaceArea, 2.0 * kPi * r * h + 2.0 * kPi * r * r, 1e-9, "its area");
    const Vec3 centre = frame.origin + axis * (h / 2.0);
    EXPECT_NEAR((props.centerOfMass - centre).length(), 0.0, 1e-9 * centre.length());

    // About its centre: m r^2 / 2 about its axis, m (3 r^2 + h^2) / 12 across it.
    const double along = volume * r * r / 2.0;
    const double across = volume * (3.0 * r * r + h * h) / 12.0;
    const double a[3] = {axis.x, axis.y, axis.z};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            const double expected = (i == j ? across : 0.0) + (along - across) * a[i] * a[j];
            EXPECT_NEAR(props.inertia.at(i, j), expected, 1e-9 * across) << i << ", " << j;
        }
    }
    expectIdealIsTheSame(*cyl, props);
}

TEST(ExactMass, APlateWithAHoleIsExact) {
    const auto plate = hz::model::exact::makePlateWithHole({0, 0, 0}, {40, 40, 10}, {20, 20}, 5.0);
    ASSERT_NE(plate, nullptr);
    const MassProperties props = MassPropertiesCalculator::compute(*plate);
    ASSERT_TRUE(props.valid) << props.problem;
    expectRelative(props.volume, 40.0 * 40.0 * 10.0 - kPi * 25.0 * 10.0, 1e-9, "its volume");
    expectRelative(props.surfaceArea,
                   2.0 * (40.0 * 40.0 - kPi * 25.0) + 4.0 * 40.0 * 10.0 + 2.0 * kPi * 5.0 * 10.0,
                   1e-9, "its area");
    EXPECT_NEAR((props.centerOfMass - Vec3{20, 20, 5}).length(), 0.0, 1e-9 * 30.0);
    expectIdealIsTheSame(*plate, props);
}

// As the reader makes them from a file: a sphere, its one face bounded by a seam from pole to pole
// and back; a torus, by two seams; a frustum's side, by its rims and a seam.
TEST(ExactMass, ImportedSphereTorusAndFrustumAreExact) {
    struct Case {
        const char* what;
        std::string text;
        double volume;
        double area;
    };
    const Case cases[] = {
        {"the sphere", hz::step_text::sphere(), 4.0 / 3.0 * kPi * 1.5 * 1.5 * 1.5,
         4.0 * kPi * 1.5 * 1.5},
        {"the torus", hz::step_text::torus(), 2.0 * kPi * kPi * 4.0 * 1.0 * 1.0,
         4.0 * kPi * kPi * 4.0 * 1.0},
        {"the frustum", frustum(), kPi * 2.0 / 3.0 * (4.0 + 2.0 + 1.0),
         kPi * 3.0 * std::sqrt(5.0) + kPi * 4.0 + kPi},
    };
    for (const Case& c : cases) {
        SCOPED_TRACE(c.what);
        const auto solid = readOne(c.text);
        ASSERT_NE(solid, nullptr);
        const MassProperties props = MassPropertiesCalculator::compute(*solid);
        ASSERT_TRUE(props.valid) << props.problem;
        expectRelative(props.volume, c.volume, 1e-9, "its volume");
        expectRelative(props.surfaceArea, c.area, 1e-9, "its area");
        expectIdealIsTheSame(*solid, props);
    }
}

// A cone's side comes to its apex, a point where its pcurves run along the line the apex is in
// (u, v): a degenerate stretch of the boundary, and integrated as one.
TEST(ExactMass, AConeToItsApexIsExact) {
    const auto cone = readOne(hz::step_text::cone());
    ASSERT_NE(cone, nullptr);
    const MassProperties props = MassPropertiesCalculator::compute(*cone);
    ASSERT_TRUE(props.valid) << props.problem;
    expectRelative(props.volume, kPi * 4.0 * 3.0 / 3.0, 1e-9, "its volume");
    expectRelative(props.surfaceArea, kPi * 4.0 + kPi * 2.0 * std::sqrt(13.0), 1e-9, "its area");
    EXPECT_NEAR((props.centerOfMass - Vec3{0, 0, 0.75}).length(), 0.0, 1e-9);
}

// A mirror image turns every face round (Pattern::transformed reverses its loops and its faces'
// sense): it measures the same, positive, and its centre is mirrored.
TEST(ExactMass, AMirroredCylinderIsPositive) {
    const auto cyl =
        hz::model::exact::makeCylinder(geo::Frame::fromAxis({1, 2, 3}, {0, 0, 1}), 5.0, 10.0);
    ASSERT_NE(cyl, nullptr);
    const double volume = kPi * 25.0 * 10.0;
    for (const hz::math::Mat4& m :
         {hz::math::Mat4::scale({-1, 1, 1}), hz::math::Mat4::reflection({0, 0, 0}, {0, 1, 0})}) {
        const auto mirrored = hz::model::Pattern::transformed(*cyl, m);
        ASSERT_NE(mirrored, nullptr);
        const MassProperties props = MassPropertiesCalculator::compute(*mirrored);
        ASSERT_TRUE(props.valid) << props.problem;
        expectRelative(props.volume, volume, 1e-9, "its volume, positive");
        const Vec3 centre = m.transformPoint({1, 2, 8});
        EXPECT_NEAR((props.centerOfMass - centre).length(), 0.0, 1e-9 * centre.length());
        // Positive face by face, not by the solid's sign being normalised: each face's outward
        // normal (its surface's, turned where it is reversed) points out of the cylinder.
        for (const auto& f : mirrored->faces()) {
            const topo::HalfEdge* h = f.outerLoop->halfEdge;
            const Vec2 uv = h->pcurve->evaluate(0.5 * (h->pcurveStart + h->pcurveEnd));
            const auto d = f.exactSurface->derivatives(uv.x, uv.y);
            const Vec3 outward = d.du.cross(d.dv) * (f.reversed ? -1.0 : 1.0);
            EXPECT_GT(outward.normalized().dot((d.point - centre).normalized()), 0.1)
                << f.topoId.tag() << " faces into the mirror image";
        }
    }
}

// Review Focus 4: a block with a ball-shaped void (BREP_WITH_VOIDS), the void's face facing into
// it. The ball's volume subtracts, and the total is positive.
TEST(ExactMass, AVoidSubtracts) {
    const auto block = readOne(hz::step_text::blockWithABall());
    ASSERT_NE(block, nullptr);
    ASSERT_EQ(block->shells().size(), 2u);
    const MassProperties props = MassPropertiesCalculator::compute(*block);
    ASSERT_TRUE(props.valid) << props.problem;
    expectRelative(props.volume, 64.0 - 4.0 / 3.0 * kPi, 1e-9, "the block less the ball");
    expectRelative(props.surfaceArea, 96.0 + 4.0 * kPi, 1e-9, "both shells' area");
    EXPECT_NEAR((props.centerOfMass - Vec3{2, 2, 2}).length(), 0.0, 1e-9 * 4.0);
}

// A cylinder whose side is a B_SPLINE_SURFACE (a rational circle swept along z), read as that
// B-spline: integrated on it, it is the cylinder to 1e-9.
TEST(ExactMass, ARationalSplineCylinderIsExact) {
    const auto cyl = readOne(hz::step_text::splineCylinder());
    ASSERT_NE(cyl, nullptr);
    int splines = 0;
    for (const auto& f : cyl->faces()) {
        if (f.exactSurface && f.exactSurface->kind() == geo::SurfaceKind::Nurbs) ++splines;
    }
    ASSERT_EQ(splines, 1);
    const MassProperties props = MassPropertiesCalculator::compute(*cyl);
    ASSERT_TRUE(props.valid) << props.problem;
    expectRelative(props.volume, 2.0 * kPi, 1e-9, "its volume");
    expectRelative(props.surfaceArea, 6.0 * kPi, 1e-9, "its area");
    EXPECT_NEAR((props.centerOfMass - Vec3{0, 0, 1}).length(), 0.0, 1e-9);
    expectIdealIsTheSame(*cyl, props);
}

// An exact face whose boundary has lost a pcurve cannot be integrated: the solid's mass is not
// valid and says which face and why. It is never measured some other way (a mesh, its corners).
TEST(ExactMass, AFaceItCannotIntegrateGivesWhy) {
    auto cyl =
        hz::model::exact::makeCylinder(geo::Frame::fromAxis({0, 0, 0}, {0, 0, 1}), 5.0, 10.0);
    ASSERT_NE(cyl, nullptr);
    topo::Face* side = faceTagged(*cyl, "cylinder/side");
    ASSERT_NE(side, nullptr);
    side->outerLoop->halfEdge->pcurve = nullptr;

    const MassProperties props = MassPropertiesCalculator::compute(*cyl);
    EXPECT_FALSE(props.valid);
    EXPECT_EQ(props.volume, 0.0);
    EXPECT_NE(props.problem.find("cylinder/side"), std::string::npos) << props.problem;
    EXPECT_NE(props.problem.find("pcurve"), std::string::npos) << props.problem;

    const IdealMassProperties ideal = MassPropertiesCalculator::computeIdeal(*cyl);
    EXPECT_FALSE(ideal.properties.valid);
    EXPECT_EQ(ideal.properties.problem, props.problem);
    EXPECT_FALSE(ideal.onIdealSurfaces);
    EXPECT_FALSE(ideal.withinTolerance);
}

// A face whose surface gives no finite point (a radius that is not a number) says so.
TEST(ExactMass, ASurfaceThatIsNotFiniteGivesWhy) {
    auto cyl =
        hz::model::exact::makeCylinder(geo::Frame::fromAxis({0, 0, 0}, {0, 0, 1}), 5.0, 10.0);
    ASSERT_NE(cyl, nullptr);
    topo::Face* side = faceTagged(*cyl, "cylinder/side");
    ASSERT_NE(side, nullptr);
    side->exactSurface =
        std::make_shared<geo::Cylinder>(geo::Frame::fromAxis({0, 0, 0}, {0, 0, 1}), std::nan(""));

    const MassProperties props = MassPropertiesCalculator::compute(*cyl);
    EXPECT_FALSE(props.valid);
    EXPECT_NE(props.problem.find("cylinder/side"), std::string::npos) << props.problem;
    EXPECT_NE(props.problem.find("not finite"), std::string::npos) << props.problem;
}

// A boundary that wiggles faster than any subdivision within the budget can follow is not
// integrated to a guess: the mass is not valid, and says the face's integral did not settle.
TEST(ExactMass, ABoundaryThatWillNotSettleGivesWhy) {
    auto block = hz::model::exact::makeBlock({0, 0, 0}, {2, 3, 4});
    ASSERT_NE(block, nullptr);
    topo::Face* top = faceTagged(*block, "box/top");
    ASSERT_NE(top, nullptr);
    topo::HalfEdge* h = top->outerLoop->halfEdge;
    const Vec2 a = h->pcurve->evaluate(h->pcurveStart);
    const Vec2 b = h->pcurve->evaluate(h->pcurveEnd);
    h->pcurve = std::make_shared<Wiggle>(a, b);
    h->pcurveStart = 0.0;
    h->pcurveEnd = 1.0;

    const MassProperties props = MassPropertiesCalculator::compute(*block);
    EXPECT_FALSE(props.valid);
    EXPECT_NE(props.problem.find("box/top"), std::string::npos) << props.problem;
    EXPECT_NE(props.problem.find("did not settle"), std::string::npos) << props.problem;
}

namespace {

/// makeBlock's block with its top face (z = 4, u along x from 0 to 2, v along y from 0 to 3, as
/// its plane is parametrised) on a flat B-spline patch of the same parameters, of degree 1 and
/// cut into @p spansU along u and @p spansV along v: the same face, its pcurves unchanged, with
/// that many knots.
std::unique_ptr<topo::Solid> blockWithAKnottedTop(int spansU, int spansV) {
    auto block = hz::model::exact::makeBlock({0, 0, 0}, {2, 3, 4});
    topo::Face* top = faceTagged(*block, "box/top");
    if (top == nullptr) return nullptr;
    const auto knots = [](int spans, double length) {
        std::vector<double> k{0.0};
        for (int i = 0; i <= spans; ++i) k.push_back(length * i / spans);
        k.push_back(length);
        return k;
    };
    std::vector<std::vector<Vec3>> points;
    for (int i = 0; i <= spansU; ++i) {
        points.emplace_back();
        for (int j = 0; j <= spansV; ++j) {
            points.back().push_back({2.0 * i / spansU, 3.0 * j / spansV, 4.0});
        }
    }
    const std::vector<std::vector<double>> weights(
        points.size(), std::vector<double>(static_cast<std::size_t>(spansV) + 1, 1.0));
    top->exactSurface = std::make_shared<geo::NurbsSurface>(points, weights, knots(spansU, 2.0),
                                                            knots(spansV, 3.0), 1, 1);
    return block;
}

}  // namespace

// A B-spline face with more knots than one integral may cross, or so many that its integral
// would cost more evaluations than a face's budget, says so: within the budget it is integrated,
// past it it is not guessed.
TEST(ExactMass, AFaceBeyondItsBudgetGivesWhy) {
    const auto fine = blockWithAKnottedTop(100, 10);
    ASSERT_NE(fine, nullptr);
    const MassProperties props = MassPropertiesCalculator::compute(*fine);
    ASSERT_TRUE(props.valid) << props.problem;
    EXPECT_NEAR(props.volume, 24.0, 24.0 * 1e-12);

    const auto knotted = blockWithAKnottedTop(20000, 1);
    ASSERT_NE(knotted, nullptr);
    const MassProperties tooMany = MassPropertiesCalculator::compute(*knotted);
    EXPECT_FALSE(tooMany.valid);
    EXPECT_NE(tooMany.problem.find("box/top"), std::string::npos) << tooMany.problem;
    EXPECT_NE(tooMany.problem.find("16384 of its knots"), std::string::npos) << tooMany.problem;

    const auto costly = blockWithAKnottedTop(12000, 20);
    ASSERT_NE(costly, nullptr);
    const MassProperties tooDear = MassPropertiesCalculator::compute(*costly);
    EXPECT_FALSE(tooDear.valid);
    EXPECT_NE(tooDear.problem.find("box/top"), std::string::npos) << tooDear.problem;
    EXPECT_NE(tooDear.problem.find("4000000 evaluations"), std::string::npos) << tooDear.problem;
}

// A solid with some faces exact and others plain loops: the plain ones are summed by the polygon
// formulas, as every face was before the exact kernel, and the whole is right.
TEST(ExactMass, AMixedSolidSumsItsPlainFacesAsPolygons) {
    auto block = hz::model::exact::makeBlock({0, 0, 0}, {2, 3, 4});
    ASSERT_NE(block, nullptr);
    topo::Face* top = faceTagged(*block, "box/top");
    ASSERT_NE(top, nullptr);
    top->exactSurface = nullptr;
    topo::HalfEdge* h = top->outerLoop->halfEdge;
    do {
        h->pcurve = nullptr;
        h = h->next;
    } while (h != top->outerLoop->halfEdge);

    const MassProperties props = MassPropertiesCalculator::compute(*block);
    ASSERT_TRUE(props.valid) << props.problem;
    EXPECT_NEAR(props.volume, 24.0, 24.0 * 1e-12);
    EXPECT_NEAR(props.surfaceArea, 52.0, 52.0 * 1e-12);
    EXPECT_NEAR((props.centerOfMass - Vec3{1, 1.5, 2}).length(), 0.0, 1e-12);
    const IdealMassProperties ideal = MassPropertiesCalculator::computeIdeal(*block);
    EXPECT_DOUBLE_EQ(ideal.properties.volume, props.volume);
    EXPECT_FALSE(ideal.onIdealSurfaces) << "one face is measured as modelled";
}

// The kernel's box is wound facing in; written to STEP and read back, every one of its exact
// faces faces in, consistently. Each face is signed by its own sense and the solid's sign is
// normalised, as the polygon formulas normalise it: it measures as the box it bounds.
TEST(ExactMass, ASolidWhoseFacesAllFaceInMeasuresAsWhatTheyBound) {
    const auto box = hz::model::PrimitiveFactory::makeBox(10.0, 20.0, 30.0);
    ASSERT_NE(box, nullptr);
    const auto read = readOne(StepFormat::toString({box.get()}));
    ASSERT_NE(read, nullptr);
    ASSERT_TRUE(topo::hasExactFaces(*read));
    const MassProperties props = MassPropertiesCalculator::compute(*read);
    ASSERT_TRUE(props.valid) << props.problem;
    EXPECT_NEAR(props.volume, 6000.0, 6000.0 * 1e-12);
    EXPECT_NEAR(props.surfaceArea, 2.0 * (200.0 + 600.0 + 300.0), 2200.0 * 1e-12);
    const MassProperties faceted = MassPropertiesCalculator::compute(*box);
    EXPECT_NEAR((props.centerOfMass - faceted.centerOfMass).length(), 0.0, 1e-9);
}

namespace {

/// A unit sphere's cap beyond the flat x = 0.5, as the reader takes it from a file that bounds the
/// sphere's face by the flat's circle alone (the reviewer's probe): the circle is off the sphere's
/// axis, so its course on the sphere is a fitted pcurve, which strays from the circle by about
/// 1.6e-7. With @p spline, the sphere is a rational B-spline (2 x 2) instead of a
/// SPHERICAL_SURFACE.
std::string capBeyondAFlat(bool spline) {
    std::string text;
    if (spline) {
        text = hz::step_text::splineSphereSurface(1.0);
    } else {
        text +=
            "#97 = CARTESIAN_POINT('',(0.,0.,0.));\n#99 = AXIS2_PLACEMENT_3D('',#97,#2,#3);\n"
            "#5 = SPHERICAL_SURFACE('',#99,1.);\n";
    }
    text += R"(#2 = DIRECTION('',(0.,0.,1.));
#3 = DIRECTION('',(1.,0.,0.));
#6 = CARTESIAN_POINT('',(0.5,0.,0.));
#8 = AXIS2_PLACEMENT_3D('',#6,#3,#2);
#9 = CIRCLE('',#8,0.8660254037844386);
#10 = CARTESIAN_POINT('',(0.5,0.,0.8660254037844386));
#11 = VERTEX_POINT('',#10);
#12 = EDGE_CURVE('',#11,#11,#9,.T.);
#13 = ORIENTED_EDGE('',*,*,#12,.T.);
#14 = EDGE_LOOP('',(#13));
#15 = FACE_OUTER_BOUND('',#14,.T.);
#16 = ADVANCED_FACE('',(#15),#5,.T.);
#17 = PLANE('',#8);
#18 = ORIENTED_EDGE('',*,*,#12,.F.);
#19 = EDGE_LOOP('',(#18));
#20 = FACE_OUTER_BOUND('',#19,.T.);
#21 = ADVANCED_FACE('',(#20),#17,.T.);
#22 = CLOSED_SHELL('',(#16,#21));
#23 = MANIFOLD_SOLID_BREP('cap beyond a flat',#22);
)";
    return hz::step_text::stepFile(text);
}

}  // namespace

// Spec goal 6: where a face's boundary is a fitted pcurve, the face is integrated against its
// edge's exact curve, each point of which is put on the surface, so a solid bounded by closed
// forms is exact even where its pcurves are not. The cap is turned and moved off the origin, so
// no coordinate of the solid's own is zero along the flat's circle.
TEST(ExactMass, AFaceBoundedByAFittedCurveIsExact) {
    const hz::math::Mat4 place = hz::math::Mat4::translation({2, -1, 3}) *
                                 hz::math::Mat4::rotationX(0.3) * hz::math::Mat4::rotationZ(0.5);
    const double volume = kPi * 0.5 * 0.5 * (3.0 - 0.5) / 3.0;  // pi h^2 (3R - h) / 3
    const double area = 2.0 * kPi * 0.5 + kPi * 0.75;           // 2 pi R h, and the flat
    const Vec3 centre = place.transformPoint({0.675, 0, 0});    // 3 (2R - h)^2 / 4 (3R - h)
    for (const bool spline : {false, true}) {
        SCOPED_TRACE(spline ? "a rational B-spline sphere" : "a SPHERICAL_SURFACE");
        const auto read = readOne(capBeyondAFlat(spline));
        ASSERT_NE(read, nullptr);
        const auto cap = hz::model::Pattern::transformed(*read, place);
        ASSERT_NE(cap, nullptr);
        const MassProperties props = MassPropertiesCalculator::compute(*cap);
        ASSERT_TRUE(props.valid) << props.problem;
        expectRelative(props.volume, volume, 1e-9, "its volume");
        expectRelative(props.surfaceArea, area, 1e-9, "its area");
        EXPECT_NEAR((props.centerOfMass - centre).length(), 0.0, 1e-9 * centre.length());
        expectIdealIsTheSame(*cap, props);
    }
}

// An edge that stands off its surface by more than the resolution (a file's own gap, its
// tolerance grown to cover it) bounds a region known only to that tolerance: the estimate says
// so, and a tight tolerance is not reached, however exactly the rest is integrated.
TEST(ExactMass, AnEdgeStandingOffItsSurfaceKeepsItsEstimateHonest) {
    const auto band = readOne(hz::step_text::bandCylinder("1.001"));
    ASSERT_NE(band, nullptr);
    double largest = 0.0;
    for (const auto& e : band->edges()) largest = std::max(largest, e.tolerance);
    ASSERT_GT(largest, 5e-4) << "its rims stand off its side";
    const IdealMassProperties tight = MassPropertiesCalculator::computeIdeal(*band);
    ASSERT_TRUE(tight.properties.valid) << tight.properties.problem;
    EXPECT_GT(tight.estimatedError, 1e-5);
    EXPECT_FALSE(tight.withinTolerance);
    EXPECT_GT(tight.refinement, 1) << "the mass dialog says what was estimated";
    const IdealMassProperties loose = MassPropertiesCalculator::computeIdeal(*band, nullptr, 1e-1);
    EXPECT_TRUE(loose.withinTolerance) << loose.estimatedError;
}

// A thin-walled analytic part, a coin a thousandth as thick as it is wide on a tilted axis, is
// exact to rounding; its faces are far larger than its volume, yet the estimate summed over them
// still comes under the default tolerance, as it must for the mass dialog to say so.
TEST(ExactMass, AThinWalledPartIsWithinTheDefaultTolerance) {
    const auto coin =
        hz::model::exact::makeCylinder(geo::Frame::fromAxis({0, 0, 0}, {1, 1, 0}), 10.0, 0.01);
    ASSERT_NE(coin, nullptr);
    const IdealMassProperties ideal = MassPropertiesCalculator::computeIdeal(*coin);
    ASSERT_TRUE(ideal.properties.valid) << ideal.properties.problem;
    expectRelative(ideal.properties.volume, kPi * 100.0 * 0.01, 1e-9, "its volume");
    EXPECT_TRUE(ideal.withinTolerance) << ideal.estimatedError;
    EXPECT_LE(ideal.estimatedError, 1e-10);
}

namespace {

/// A cylinder of radius @p r, its foot the circle at z = 0 and its top cut by the plane
/// z = 2r + x/2 (the reviewer's probe): the top rim is an ELLIPSE whose course on the cylinder,
/// a sinusoid in (u, v), is a fitted pcurve, integrated on the ellipse itself. u is an angle and v
/// a length, so the smaller the radius the more lopsided the surface's metric.
std::string obliquelyCutCylinder(double r) {
    using hz::step_text::real;
    std::string text = R"(#1 = CARTESIAN_POINT('',(0.,0.,0.));
#2 = DIRECTION('',(0.,0.,1.));
#3 = DIRECTION('',(0.,1.,0.));
#4 = AXIS2_PLACEMENT_3D('',#1,#2,#3);
#5 = CIRCLE('',#4,@R@);
#6 = CARTESIAN_POINT('',(@R@,0.,0.));
#7 = VERTEX_POINT('',#6);
#8 = CARTESIAN_POINT('',(0.,0.,@2R@));
#9 = DIRECTION('',(1.,0.,0.));
#50 = DIRECTION('',(-0.44721359549995793,0.,0.89442719099991586));
#51 = DIRECTION('',(0.89442719099991586,0.,0.44721359549995793));
#10 = AXIS2_PLACEMENT_3D('',#8,#50,#51);
#11 = ELLIPSE('',#10,@A@,@R@);
#12 = CARTESIAN_POINT('',(@R@,0.,@TOP@));
#13 = VERTEX_POINT('',#12);
#14 = EDGE_CURVE('',#7,#7,#5,.T.);
#15 = EDGE_CURVE('',#13,#13,#11,.T.);
#16 = LINE('',#6,VECTOR('',#2,1.));
#17 = SURFACE_CURVE('',#16,(),.CURVE_3D.);
#18 = EDGE_CURVE('',#7,#13,#17,.T.);
#19 = AXIS2_PLACEMENT_3D('',#1,#2,#9);
#20 = CYLINDRICAL_SURFACE('',#19,@R@);
#21 = ORIENTED_EDGE('',*,*,#14,.T.);
#22 = ORIENTED_EDGE('',*,*,#18,.T.);
#23 = ORIENTED_EDGE('',*,*,#15,.F.);
#24 = ORIENTED_EDGE('',*,*,#18,.F.);
#25 = EDGE_LOOP('',(#21,#22,#23,#24));
#26 = FACE_OUTER_BOUND('',#25,.T.);
#27 = ADVANCED_FACE('',(#26),#20,.T.);
#28 = AXIS2_PLACEMENT_3D('',#8,#50,#51);
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
#41 = MANIFOLD_SOLID_BREP('oblique',#40);
)";
    const auto put = [&text](const std::string& name, double value) {
        for (auto at = text.find(name); at != std::string::npos; at = text.find(name)) {
            text.replace(at, name.size(), real(value));
        }
    };
    put("@2R@", 2.0 * r);
    put("@A@", std::sqrt(1.25) * r);
    put("@TOP@", 2.5 * r);
    put("@R@", r);
    return hz::step_text::stepFile(text);
}

}  // namespace

// The oblique cut's rim on a cylinder of radius 1, a hundredth and a thousandth: integrated on
// the ellipse, each point's course on the cylinder resolved without regard to the metric's
// proportions, it is exact at every size.
TEST(ExactMass, AnObliqueCutIsExactOnACylinderOfAnySize) {
    for (const double r : {1.0, 0.01, 0.001}) {
        SCOPED_TRACE("radius " + std::to_string(r));
        const auto cut = readOne(obliquelyCutCylinder(r));
        ASSERT_NE(cut, nullptr);
        const MassProperties props = MassPropertiesCalculator::compute(*cut);
        ASSERT_TRUE(props.valid) << props.problem;
        expectRelative(props.volume, 2.0 * kPi * r * r * r, 1e-9, "its volume");
        expectRelative(props.surfaceArea, (5.0 * kPi + kPi * std::sqrt(1.25)) * r * r, 1e-9,
                       "its area");
        const Vec3 centre = Vec3{1.0 / 16.0, 0.0, 1.0 + 1.0 / 64.0} * r;
        EXPECT_NEAR((props.centerOfMass - centre).length(), 0.0, 1e-9 * centre.length());
        expectIdealIsTheSame(*cut, props);
    }
}

// An edge said to lie on its surface (its tolerance the resolution) whose curve does not is not
// integrated where its points land: the ellipse moved a hundredth off the cylinder is a stated
// problem, naming the edge.
TEST(ExactMass, AnEdgeThatCannotBePutOnItsSurfaceGivesWhy) {
    auto cut = readOne(obliquelyCutCylinder(1.0));
    ASSERT_NE(cut, nullptr);
    topo::Edge* rim = nullptr;
    for (auto& e : cut->edges()) {
        if (e.exactCurve && e.exactCurve->kind() == geo::CurveKind::Ellipse) rim = &e;
    }
    ASSERT_NE(rim, nullptr);
    ASSERT_EQ(rim->tolerance, 0.0);
    rim->exactCurve = rim->exactCurve->transformed(hz::math::Mat4::translation({0.01, 0, 0}));
    const MassProperties props = MassPropertiesCalculator::compute(*cut);
    EXPECT_FALSE(props.valid);
    EXPECT_NE(props.problem.find("off its surface"), std::string::npos) << props.problem;
}

// Thinner still: a coin a hundred thousandth as thick as it is wide, whose estimate no pass can
// bring under the default tolerance, is measured all the same, valid, its estimate honest; and a
// tolerance finer than the default does not make a valid result invalid either.
TEST(ExactMass, ATighterTargetNeverMakesAValidResultInvalid) {
    const auto coin =
        hz::model::exact::makeCylinder(geo::Frame::fromAxis({0, 0, 0}, {1, 1, 0}), 10.0, 1e-4);
    ASSERT_NE(coin, nullptr);
    const MassProperties props = MassPropertiesCalculator::compute(*coin);
    ASSERT_TRUE(props.valid) << props.problem;
    expectRelative(props.volume, kPi * 100.0 * 1e-4, 1e-9, "its volume");
    const IdealMassProperties ideal = MassPropertiesCalculator::computeIdeal(*coin);
    ASSERT_TRUE(ideal.properties.valid) << ideal.properties.problem;
    EXPECT_EQ(ideal.withinTolerance, ideal.estimatedError <= 1e-10) << ideal.estimatedError;

    const auto thicker =
        hz::model::exact::makeCylinder(geo::Frame::fromAxis({0, 0, 0}, {1, 1, 0}), 10.0, 1e-2);
    ASSERT_NE(thicker, nullptr);
    const IdealMassProperties fine =
        MassPropertiesCalculator::computeIdeal(*thicker, nullptr, 1e-12);
    ASSERT_TRUE(fine.properties.valid) << fine.properties.problem;
    expectRelative(fine.properties.volume, kPi * 100.0 * 1e-2, 1e-9, "the thicker coin");
    EXPECT_EQ(fine.withinTolerance, fine.estimatedError <= 1e-12) << fine.estimatedError;
}

// A part so large that its second moments pass the largest double is not reported valid with
// infinite inertia: every sum is checked.
TEST(ExactMass, MomentsBeyondTheLargestNumberAreNotValid) {
    const auto block = hz::model::exact::makeBlock({0, 0, 0}, {1e70, 1e70, 1e70});
    ASSERT_NE(block, nullptr);
    const MassProperties props = MassPropertiesCalculator::compute(*block);
    EXPECT_FALSE(props.valid);
    EXPECT_NE(props.problem.find("not finite"), std::string::npos) << props.problem;
}

namespace {

/// A plane that throws from every evaluation: what a surface with a fault would do. @p standard
/// picks a std::exception, else a plain int.
class Throwing : public geo::Plane {
public:
    Throwing(const geo::Plane& plane, bool standard) : geo::Plane(plane), standard_(standard) {}
    Vec3 evaluate(double /*u*/, double /*v*/) const override { throwIt(); }
    geo::SurfaceDerivatives derivatives(double /*u*/, double /*v*/) const override { throwIt(); }

private:
    [[noreturn]] void throwIt() const {
        if (standard_) throw std::runtime_error("a fault");
        throw 42;
    }
    bool standard_;
};

}  // namespace

// Nothing a surface throws escapes compute(), a std::exception or not: the result is not valid.
TEST(ExactMass, WhatASurfaceThrowsIsCaught) {
    for (const bool standard : {true, false}) {
        auto block = hz::model::exact::makeBlock({0, 0, 0}, {2, 3, 4});
        ASSERT_NE(block, nullptr);
        topo::Face* top = faceTagged(*block, "box/top");
        ASSERT_NE(top, nullptr);
        const auto* plane = dynamic_cast<const geo::Plane*>(top->exactSurface.get());
        ASSERT_NE(plane, nullptr);
        top->exactSurface = std::make_shared<Throwing>(*plane, standard);
        MassProperties props;
        EXPECT_NO_THROW(props = MassPropertiesCalculator::compute(*block));
        EXPECT_FALSE(props.valid);
        EXPECT_FALSE(props.problem.empty());
        IdealMassProperties ideal;
        EXPECT_NO_THROW(ideal = MassPropertiesCalculator::computeIdeal(*block));
        EXPECT_FALSE(ideal.properties.valid);
    }
}

// A request to stop is heard: the result is not valid.
TEST(ExactMass, AStoppedRequestIsNotValid) {
    const auto cyl =
        hz::model::exact::makeCylinder(geo::Frame::fromAxis({0, 0, 0}, {0, 0, 1}), 5.0, 10.0);
    ASSERT_NE(cyl, nullptr);
    const std::atomic<bool> cancelled{true};
    const IdealMassProperties ideal =
        MassPropertiesCalculator::computeIdeal(*cyl, nullptr, 1e-10, &cancelled);
    EXPECT_FALSE(ideal.properties.valid);
}

// Each shell's volume, signed by which way its faces face (compute() normalises the sign away): a
// block with a ball's void is +64 and -4/3 pi, each integrated on its exact faces; the same block
// mirrored, its faces turned with its loops, measures the same; the kernel's box, wound facing in
// and measured by its polygons, is negative; a shell no face of which can be integrated says why.
TEST(ExactMass, ShellVolumesAreSignedByWhichWayTheirFacesFace) {
    const auto block = readOne(hz::step_text::blockWithABall());
    ASSERT_NE(block, nullptr);
    const double ball = 4.0 / 3.0 * std::numbers::pi;
    std::string problem;
    auto volumes = MassPropertiesCalculator::shellVolumes(*block, &problem);
    ASSERT_EQ(volumes.size(), 2u) << problem;
    EXPECT_NEAR(volumes[0], 64.0, 64.0 * 1e-6);
    EXPECT_NEAR(volumes[1], -ball, ball * 1e-6);

    const auto mirrored = hz::model::Pattern::transformed(
        *block, hz::math::Mat4::reflection(Vec3{0, 0, 0}, Vec3{1, 0, 0}));
    volumes = MassPropertiesCalculator::shellVolumes(*mirrored, &problem);
    ASSERT_EQ(volumes.size(), 2u) << problem;
    EXPECT_NEAR(volumes[0], 64.0, 64.0 * 1e-6);
    EXPECT_NEAR(volumes[1], -ball, ball * 1e-6);

    const auto box = hz::model::PrimitiveFactory::makeBox(10.0, 20.0, 30.0);
    volumes = MassPropertiesCalculator::shellVolumes(*box, &problem);
    ASSERT_EQ(volumes.size(), 1u) << problem;
    EXPECT_NEAR(volumes[0], -6000.0, 6000.0 * 1e-12);

    std::atomic<bool> cancelled{true};
    EXPECT_TRUE(MassPropertiesCalculator::shellVolumes(*block, &problem, 1e-6, &cancelled).empty());
    EXPECT_EQ(problem, "stopped");
}
