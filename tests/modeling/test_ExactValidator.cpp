#include <gtest/gtest.h>

#include <algorithm>
#include <memory>
#include <set>

#include "horizon/geometry/Frame.h"
#include "horizon/geometry/curves/AnalyticCurves.h"
#include "horizon/geometry/curves/Circle2D.h"
#include "horizon/geometry/curves/Curve3.h"
#include "horizon/geometry/curves/Line2D.h"
#include "horizon/geometry/curves/Nurbs2D.h"
#include "horizon/geometry/curves/NurbsCurve.h"
#include "horizon/geometry/surfaces/AnalyticSurfaces.h"
#include "horizon/math/Constants.h"
#include "horizon/math/ModelTolerance.h"
#include "horizon/modeling/ExactPrimitives.h"
#include "horizon/topology/ExactValidator.h"

using namespace hz;
using hz::math::Vec3;
using hz::topo::ExactIssueKind;

namespace {

const math::ModelTolerance kTol = math::ModelTolerance::forExtent(300.0);

std::set<ExactIssueKind> kindsOf(const topo::Solid& s) {
    std::set<ExactIssueKind> kinds;
    for (const auto& i : topo::validateExact(s, kTol)) kinds.insert(i.kind);
    return kinds;
}

/// The mutation must give @p expected, and nothing outside @p allowed (the kinds that the same
/// mutation unavoidably trips as well).
void expectKinds(const topo::Solid& s, ExactIssueKind expected,
                 const std::set<ExactIssueKind>& allowed = {}) {
    const auto kinds = kindsOf(s);
    EXPECT_TRUE(kinds.count(expected) == 1);
    for (const auto k : kinds) {
        EXPECT_TRUE(k == expected || allowed.count(k) == 1) << "unexpected kind " << int(k);
    }
}

}  // namespace

TEST(ExactValidator, TheExactPrimitivesAtAnyPlacementAreClean) {
    for (const Vec3& at : {Vec3{0, 0, 0}, Vec3{-7, 3, 11}, Vec3{100, -50, 20}}) {
        const auto block = model::exact::makeBlock(at, at + Vec3{2, 3, 4});
        EXPECT_TRUE(topo::validateExact(*block, kTol).empty());
        const auto plate =
            model::exact::makePlateWithHole(at, at + Vec3{40, 30, 10}, {at.x + 15, at.y + 12}, 5.0);
        EXPECT_TRUE(topo::validateExact(*plate, kTol).empty());
        for (const Vec3& dir : {Vec3{0, 0, 1}, Vec3{1, 1, 1}, Vec3{-2, 1, 0.5}}) {
            const auto cyl = model::exact::makeCylinder(geo::Frame::fromAxis(at, dir), 5.0, 10.0);
            EXPECT_TRUE(topo::validateExact(*cyl, kTol).empty());
            geo::Frame left = geo::Frame::fromAxis(at, dir);
            left.y = left.y * -1.0;
            EXPECT_TRUE(
                topo::validateExact(*model::exact::makeCylinder(left, 5.0, 10.0), kTol).empty());
        }
    }
}

TEST(ExactValidator, ATinyAndAHugePartAreCleanAtTheirOwnScale) {
    for (const double s : {1e-4, 1e-3, 1e4, 1e5}) {
        const auto tol = math::ModelTolerance::forExtent(s * 40.0);
        const auto plate = model::exact::makePlateWithHole({0, 0, 0}, {s * 40, s * 40, s * 10},
                                                           {s * 20, s * 20}, s * 5);
        EXPECT_TRUE(topo::validateExact(*plate, tol).empty()) << s;
        const auto cyl = model::exact::makeCylinder(
            geo::Frame::fromAxis({s, 2 * s, 3 * s}, {1, 2, 3}), s * 5, s * 10);
        EXPECT_TRUE(topo::validateExact(*cyl, math::ModelTolerance::forExtent(s * 40.0)).empty())
            << s;
    }
}

TEST(ExactValidator, AMovedVertexIsOffItsCurves) {
    auto block = model::exact::makeBlock({0, 0, 0}, {2, 3, 4});
    block->vertices()[0].point.x += 0.5;
    // Its pcurves' ends no longer meet it either: the same disagreement seen from the face.
    expectKinds(*block, ExactIssueKind::VertexOffCurve, {ExactIssueKind::PcurveOffVertex});
}

TEST(ExactValidator, APcurveShiftedInVIsOffItsCurve) {
    auto block = model::exact::makeBlock({0, 0, 0}, {2, 3, 4});
    bool done = false;
    for (auto& e : block->edges()) {
        if (done) break;
        topo::HalfEdge* h = e.halfEdge;
        const auto* line = dynamic_cast<const geo::Line2D*>(h->pcurve.get());
        // An edge running along u: shifting it in v leaves the line it lies on.
        if (!line || std::abs(line->end().y - line->start().y) > 1e-9) continue;
        h->pcurve = std::make_shared<geo::Line2D>(line->start() + math::Vec2{0, 0.1},
                                                  line->end() + math::Vec2{0, 0.1});
        done = true;
    }
    ASSERT_TRUE(done);
    // The shifted pcurve also misses its vertices and no longer joins its neighbours.
    expectKinds(*block, ExactIssueKind::PcurveOffCurve,
                {ExactIssueKind::PcurveOffVertex, ExactIssueKind::LoopNotClosed});
}

TEST(ExactValidator, AFaceWithoutItsSurfaceIsMissingGeometry) {
    auto block = model::exact::makeBlock({0, 0, 0}, {2, 3, 4});
    block->faces()[2].exactSurface.reset();
    expectKinds(*block, ExactIssueKind::MissingGeometry);
    for (const auto& i : topo::validateExact(*block, kTol)) EXPECT_EQ(i.face, &block->faces()[2]);
}

TEST(ExactValidator, ASeamEdgeWithoutItsCurveIsMissingGeometry) {
    auto cyl = model::exact::makeCylinder(geo::Frame::fromAxis({0, 0, 0}, {0, 0, 1}), 5.0, 10.0);
    bool done = false;
    for (auto& e : cyl->edges()) {
        if (e.halfEdge->face == e.halfEdge->twin->face) {
            e.exactCurve.reset();
            done = true;
        }
    }
    ASSERT_TRUE(done);
    expectKinds(*cyl, ExactIssueKind::MissingGeometry);
    for (const auto& i : topo::validateExact(*cyl, kTol)) {
        ASSERT_NE(i.edge, nullptr);
        EXPECT_EQ(i.edge->exactCurve, nullptr);
    }
}

TEST(ExactValidator, AFlippedFaceIsWronglyOriented) {
    auto block = model::exact::makeBlock({0, 0, 0}, {2, 3, 4});
    block->faces()[1].reversed = !block->faces()[1].reversed;
    expectKinds(*block, ExactIssueKind::LoopWrongOrientation);
    auto cyl = model::exact::makeCylinder(geo::Frame::fromAxis({0, 0, 0}, {0, 0, 1}), 5.0, 10.0);
    for (auto& f : cyl->faces()) f.reversed = !f.reversed;
    expectKinds(*cyl, ExactIssueKind::LoopWrongOrientation);
}

TEST(ExactValidator, ABrokenJoinIsALoopNotClosed) {
    auto block = model::exact::makeBlock({0, 0, 0}, {2, 3, 4});
    topo::HalfEdge* h = block->faces()[0].outerLoop->halfEdge;
    h->next = h->next->next;  // skip a half-edge: the cycle no longer closes properly
    // Skipping a half-edge also leaves the skipped-over origin out of its neighbour's pcurve end.
    expectKinds(*block, ExactIssueKind::LoopNotClosed, {ExactIssueKind::PcurveOffVertex});
}

TEST(ExactValidator, AHoleMovedOutsideThePlateIsOutsideItsOuterLoop) {
    auto plate = model::exact::makePlateWithHole({0, 0, 0}, {40, 40, 10}, {20, 20}, 5.0);
    // Move the whole hole 100 along x, every piece of it, so nothing else disagrees.
    const auto move = math::Mat4::translation({100, 0, 0});
    std::set<topo::Vertex*> moved;
    for (auto& e : plate->edges()) {
        const bool onWall = e.halfEdge->face->topoId.tag() == "hole/wall" ||
                            e.halfEdge->twin->face->topoId.tag() == "hole/wall";
        if (!onWall) continue;
        e.exactCurve = e.exactCurve->transformed(move);
        for (topo::HalfEdge* h : {e.halfEdge, e.halfEdge->twin}) {
            moved.insert(h->origin);
            if (h->face->topoId.tag() == "hole/wall") continue;
            const auto* c = dynamic_cast<const geo::Circle2D*>(h->pcurve.get());
            ASSERT_NE(c, nullptr);
            h->pcurve =
                std::make_shared<geo::Circle2D>(c->center() + math::Vec2{100, 0}, c->radius());
        }
    }
    for (auto* v : moved) v->point = move.transformPoint(v->point);
    for (auto& f : plate->faces()) {
        if (f.topoId.tag() == "hole/wall") f.exactSurface = f.exactSurface->transformed(move);
    }
    expectKinds(*plate, ExactIssueKind::HoleOutsideOuterLoop);
}

TEST(ExactValidator, ARimCurveRunningTheWrongWayIsOffItsPcurves) {
    // The bug class a left-handed cylinder once had: same points, same start, opposite direction.
    // Only the order of the pcurve along the curve is wrong; ends and positions all agree.
    auto cyl = model::exact::makeCylinder(geo::Frame::fromAxis({0, 0, 0}, {0, 0, 1}), 5.0, 10.0);
    bool done = false;
    for (auto& e : cyl->edges()) {
        if (!dynamic_cast<const geo::CircleCurve*>(e.exactCurve.get())) continue;
        std::swap(e.curveStart, e.curveEnd);
        done = true;
        break;
    }
    ASSERT_TRUE(done);
    expectKinds(*cyl, ExactIssueKind::PcurveOffCurve);
}

TEST(ExactValidator, ABulgingPcurveMakesALoopCrossItself) {
    auto block = model::exact::makeBlock({0, 0, 0}, {2, 3, 4});
    topo::Face& f = block->faces()[0];
    topo::HalfEdge* h = f.outerLoop->halfEdge;
    const math::Vec2 a = h->pcurve->evaluate(h->pcurveStart);
    const math::Vec2 b = h->pcurve->evaluate(h->pcurveEnd);
    math::Vec2 centroid{0, 0};
    int n = 0;
    for (const topo::HalfEdge* g = h; n == 0 || g != h; g = g->next, ++n) {
        centroid = centroid + g->pcurve->evaluate(g->pcurveStart);
    }
    centroid = centroid * (1.0 / n);
    const math::Vec2 mid = (a + b) * 0.5;
    const math::Vec2 far =
        mid + (centroid - mid) * 2.4;  // just beyond the opposite side (a larger bulge flips the
                                       // loop's area, adding LoopWrongOrientation)
    h->pcurve = std::make_shared<geo::Nurbs2D>(std::vector<math::Vec2>{a, far, b},
                                               std::vector<double>{1, 1, 1},
                                               std::vector<double>{0, 0, 0.5, 1, 1}, 1);
    h->pcurveStart = h->pcurve->tMin();
    h->pcurveEnd = h->pcurve->tMax();
    // Its ends and the joins are unchanged; it only leaves the edge's 3D line.
    expectKinds(*block, ExactIssueKind::LoopSelfIntersects, {ExactIssueKind::PcurveOffCurve});
}

TEST(ExactValidator, AHoleLoopCrossingTheOuterLoopIsLoopsCross) {
    auto plate = model::exact::makePlateWithHole({0, 0, 0}, {40, 40, 10}, {20, 20}, 5.0);
    bool done = false;
    for (auto& f : plate->faces()) {
        if (f.innerLoops.empty() || done) continue;
        topo::HalfEdge* h = f.innerLoops[0]->halfEdge;
        const auto* c = dynamic_cast<const geo::Circle2D*>(h->pcurve.get());
        ASSERT_NE(c, nullptr);
        // A larger circle through the same start point (c.x + r, c.y), same parameter range.
        const double big = 30.0;
        h->pcurve = std::make_shared<geo::Circle2D>(
            math::Vec2{c->center().x + c->radius() - big, c->center().y}, big);
        done = true;
    }
    ASSERT_TRUE(done);
    // The crossing is reported instead of HoleOutsideOuterLoop (only non-crossing loops are
    // classified); the larger circle also leaves the edge's 3D circle.
    expectKinds(*plate, ExactIssueKind::LoopsCross, {ExactIssueKind::PcurveOffCurve});
}

namespace {

/// Replaces each rim circle of a cylinder along z by the same circle as a NURBS curve (range
/// [0, 1], closed, not periodic), running the same way.
void makeRimsNurbs(topo::Solid& cyl) {
    for (auto& e : cyl.edges()) {
        const auto* c = dynamic_cast<const geo::CircleCurve*>(e.exactCurve.get());
        if (!c) continue;
        const Vec3 center = (c->evaluate(0.0) + c->evaluate(math::kPi)) * 0.5;
        e.exactCurve = std::make_shared<geo::NurbsCurve>(
            geo::NurbsCurve::makeCircle(center, c->radius(), Vec3{0, 0, 1}));
        e.curveStart = 0.0;
        e.curveEnd = 1.0;
    }
}

}  // namespace

TEST(ExactValidator, AClosedNurbsRimIsCleanAndItsReverseIsReported) {
    auto cyl = model::exact::makeCylinder(geo::Frame::fromAxis({0, 0, 0}, {0, 0, 1}), 5.0, 10.0);
    makeRimsNurbs(*cyl);
    EXPECT_TRUE(topo::validateExact(*cyl, kTol).empty());
    for (auto& e : cyl->edges()) {
        if (dynamic_cast<const geo::NurbsCurve*>(e.exactCurve.get())) {
            std::swap(e.curveStart, e.curveEnd);
        }
    }
    expectKinds(*cyl, ExactIssueKind::PcurveOffCurve);
}

TEST(ExactValidator, APcurveWindingTwiceRoundAClosedEdgeIsReported) {
    auto cyl = model::exact::makeCylinder(geo::Frame::fromAxis({0, 0, 0}, {0, 0, 1}), 5.0, 10.0);
    for (auto& f : cyl->faces()) {
        if (f.topoId.tag() == "cylinder/side") continue;
        topo::HalfEdge* h = f.outerLoop->halfEdge;  // a cap's rim: one full circle
        h->pcurveEnd = h->pcurveStart + 2.0 * (h->pcurveEnd - h->pcurveStart);
        break;
    }
    // The loop retraces itself, which is a self-overlap too.
    expectKinds(*cyl, ExactIssueKind::PcurveOffCurve, {ExactIssueKind::LoopSelfIntersects});
}

namespace {

constexpr double kRadius = 2.0;
constexpr double kHeight = 3.0;

/// Adds a quadrilateral loop of four half-edges to @p face on the cylinder of @p frame and
/// radius kRadius: from corner i to corner i+1, alternately along an arc (v constant) and a line
/// along z (u constant). Corners are (u, v) and may lie beyond 2 pi; @p written[i] is added to
/// half-edge i's pcurve (a whole-period shift, to write it in another window of the surface).
/// The twins have no face.
topo::Wire* addQuadLoop(topo::Solid& solid, topo::Face* face, const geo::Frame& frame,
                        const math::Vec2 (&corner)[4], const math::Vec2 (&written)[4], bool outer) {
    const geo::Cylinder surface(frame, kRadius);
    topo::Wire* w = solid.allocWire();
    if (outer) {
        face->outerLoop = w;
    } else {
        face->innerLoops.push_back(w);
    }
    topo::Vertex* v[4];
    topo::HalfEdge* he[4];
    topo::HalfEdge* tw[4];
    topo::Edge* ed[4];
    for (int i = 0; i < 4; ++i) {
        v[i] = solid.allocVertex();
        v[i]->point = surface.evaluate(corner[i].x, corner[i].y);
        he[i] = solid.allocHalfEdge();
        tw[i] = solid.allocHalfEdge();
        ed[i] = solid.allocEdge();
    }
    for (int i = 0; i < 4; ++i) {
        const int j = (i + 1) % 4;
        he[i]->origin = v[i];
        he[i]->next = he[j];
        he[i]->prev = he[(i + 3) % 4];
        he[i]->face = face;
        he[i]->twin = tw[i];
        he[i]->edge = ed[i];
        tw[i]->origin = v[j];
        tw[i]->twin = he[i];
        tw[i]->edge = ed[i];
        ed[i]->halfEdge = he[i];
        v[i]->halfEdge = he[i];
        he[i]->pcurve =
            std::make_shared<geo::Line2D>(corner[i] + written[i], corner[j] + written[i]);
        he[i]->pcurveStart = 0.0;
        he[i]->pcurveEnd = 1.0;
        if (corner[i].y == corner[j].y) {  // an arc about the axis at this height
            geo::Frame at = frame;
            at.origin = frame.origin + frame.z * corner[i].y;
            ed[i]->exactCurve = std::make_shared<geo::CircleCurve>(at, kRadius);
            ed[i]->curveStart = corner[i].x;
            ed[i]->curveEnd = corner[j].x;
        } else {  // a line along z
            const double up = corner[j].y > corner[i].y ? 1.0 : -1.0;
            ed[i]->exactCurve = std::make_shared<geo::LineCurve>(v[i]->point, frame.z * up);
            ed[i]->curveStart = 0.0;
            ed[i]->curveEnd = std::fabs(corner[j].y - corner[i].y);
        }
    }
    w->halfEdge = he[0];
    return w;
}

/// One face on a half-cylinder spanning u in [3 pi/2, 5 pi/2], across the seam of the surface's
/// [0, 2 pi) range. @p seamU is the u the right side's pcurve is written at (5 pi/2 is
/// continuous, pi/2 is what inversion would give). With @p holeU, a hole 0.4 wide and 1 high
/// about that u; @p holeInverted writes its first pcurve a period lower.
struct HalfCylinderFace {
    topo::Solid solid;
    explicit HalfCylinderFace(double seamU, const double* holeU = nullptr,
                              bool holeInverted = false) {
        const geo::Frame frame = geo::Frame::fromAxis({0, 0, 0}, {0, 0, 1});
        const double a = 1.5 * math::kPi;
        const double b = 2.5 * math::kPi;
        topo::Face* f = solid.allocFace();
        f->exactSurface = std::make_shared<geo::Cylinder>(frame, kRadius);
        const math::Vec2 outerCorner[4] = {{a, 0}, {b, 0}, {b, kHeight}, {a, kHeight}};
        const math::Vec2 outerWritten[4] = {{0, 0}, {seamU - b, 0}, {0, 0}, {0, 0}};
        addQuadLoop(solid, f, frame, outerCorner, outerWritten, true);
        if (holeU) {
            // Clockwise, starting at the bottom right.
            const double u = *holeU;
            const math::Vec2 holeCorner[4] = {
                {u + 0.2, 1}, {u - 0.2, 1}, {u - 0.2, 2}, {u + 0.2, 2}};
            const math::Vec2 holeWritten[4] = {
                {holeInverted ? -math::kTwoPi : 0.0, 0}, {0, 0}, {0, 0}, {0, 0}};
            addQuadLoop(solid, f, frame, holeCorner, holeWritten, false);
        }
    }
};

}  // namespace

TEST(ExactValidator, AHoleStraddlingTheSeamIsCleanWhicheverWindowItIsWrittenIn) {
    const double u = math::kTwoPi;
    for (const bool inverted : {false, true}) {
        HalfCylinderFace face(2.5 * math::kPi, &u, inverted);
        EXPECT_TRUE(topo::validateExact(face.solid, kTol).empty()) << inverted;
    }
}

TEST(ExactValidator, AHoleCrossingTheEdgeIsFoundWhicheverWindowItIsWrittenIn) {
    const double u = 2.5 * math::kPi - 0.03;  // astride the face's own right side, off a sample
    for (const bool inverted : {false, true}) {
        HalfCylinderFace face(2.5 * math::kPi, &u, inverted);
        expectKinds(face.solid, ExactIssueKind::LoopsCross);
    }
}

TEST(ExactValidator, AFaceAcrossTheSeamIsCleanWhetherItsPcurvesAreContinuousOrInverted) {
    for (const double seamU : {2.5 * math::kPi, 0.5 * math::kPi}) {
        HalfCylinderFace face(seamU);
        EXPECT_TRUE(topo::validateExact(face.solid, kTol).empty()) << seamU;
    }
}

namespace {

/// A planar annulus: an outer circle of radius R about the origin, counter-clockwise, and an
/// inner circle of radius r about @p inner, clockwise, starting @p phase round from its +x.
struct Annulus {
    topo::Solid solid;
    Annulus(double R, double r, const math::Vec2& inner, double phase) {
        const geo::Frame frame = geo::Frame::fromAxis({0, 0, 0}, {0, 0, 1});
        topo::Face* f = solid.allocFace();
        f->exactSurface = std::make_shared<geo::Plane>(frame);
        for (int k = 0; k < 2; ++k) {
            const bool outer = k == 0;
            const math::Vec2 c = outer ? math::Vec2{0, 0} : inner;
            const double rad = outer ? R : r;
            const double ph = outer ? 0.0 : phase;
            topo::Wire* w = solid.allocWire();
            if (outer) {
                f->outerLoop = w;
            } else {
                f->innerLoops.push_back(w);
            }
            topo::Vertex* v = solid.allocVertex();
            v->point = {c.x + rad * std::cos(ph), c.y + rad * std::sin(ph), 0};
            topo::HalfEdge* he = solid.allocHalfEdge();
            topo::HalfEdge* tw = solid.allocHalfEdge();
            topo::Edge* e = solid.allocEdge();
            he->origin = tw->origin = v;
            he->next = he->prev = he;
            he->face = f;
            he->twin = tw;
            tw->twin = he;
            he->edge = tw->edge = e;
            e->halfEdge = he;
            v->halfEdge = he;
            w->halfEdge = he;
            geo::Frame cf = frame;
            cf.origin = {c.x, c.y, 0};
            if (outer) {
                he->pcurve = std::make_shared<geo::Circle2D>(c, rad);
                he->pcurveStart = ph;
                he->pcurveEnd = ph + math::kTwoPi;
                e->exactCurve = std::make_shared<geo::CircleCurve>(cf, rad);
                e->curveStart = ph;
                e->curveEnd = ph + math::kTwoPi;
            } else {
                // Clockwise: the 3D circle's frame is mirrored so it runs the same way.
                he->pcurve = std::make_shared<geo::Circle2D>(c, rad);
                he->pcurveStart = ph + math::kTwoPi;
                he->pcurveEnd = ph;
                cf.y = cf.y * -1.0;
                e->exactCurve = std::make_shared<geo::CircleCurve>(cf, rad);
                e->curveStart = -ph;
                e->curveEnd = math::kTwoPi - ph;
            }
        }
    }
};

}  // namespace

TEST(ExactValidator, AThinAnnulusIsCleanWhateverWhereItsLoopsStart) {
    const double phase =
        math::kPi / 16.0;  // a chord of the inner circle faces a vertex of the outer
    for (const double ph : {0.0, phase}) {
        Annulus thin(10.0, 9.85, {0, 0}, ph);
        EXPECT_TRUE(topo::validateExact(thin.solid, kTol).empty()) << ph;
    }
}

TEST(ExactValidator, AnAnnulusWhoseInnerCircleReallyCrossesTheOuterIsLoopsCross) {
    Annulus overlapping(10.0, 9.85, {1.0, 0}, math::kPi / 16.0);
    expectKinds(overlapping.solid, ExactIssueKind::LoopsCross);
}

namespace {

/// A lune of the unit-scaled sphere of radius r (u in [0, pi]) whose two meridians meet at the
/// poles; each meridian's pcurve stops @p shortBy (a distance on the sphere) before a pole,
/// while the vertices sit exactly at the poles.
struct SphereLune {
    topo::Solid solid;
    SphereLune(double r, double shortBy) {
        const geo::Frame frame = geo::Frame::fromAxis({0, 0, 0}, {0, 0, 1});
        topo::Face* f = solid.allocFace();
        f->exactSurface = std::make_shared<geo::Sphere>(frame, r);
        topo::Wire* w = solid.allocWire();
        f->outerLoop = w;
        const double half = 0.5 * math::kPi;
        const double pcHalf = half - shortBy / r;
        topo::Vertex* south = solid.allocVertex();
        topo::Vertex* north = solid.allocVertex();
        south->point = {0, 0, -r};
        north->point = {0, 0, r};
        // Counter-clockwise in (u, v): up the meridian u = pi, along the north pole, down u = 0.
        topo::HalfEdge* he[2] = {solid.allocHalfEdge(), solid.allocHalfEdge()};
        for (int i = 0; i < 2; ++i) {
            const double u = i == 0 ? math::kPi : 0.0;
            const double sign = i == 0 ? 1.0 : -1.0;
            topo::HalfEdge* tw = solid.allocHalfEdge();
            topo::Edge* e = solid.allocEdge();
            he[i]->origin = i == 0 ? south : north;
            tw->origin = i == 0 ? north : south;
            he[i]->next = he[1 - i];
            he[i]->prev = he[1 - i];
            he[i]->face = f;
            he[i]->twin = tw;
            tw->twin = he[i];
            he[i]->edge = tw->edge = e;
            e->halfEdge = he[i];
            he[i]->pcurve = std::make_shared<geo::Line2D>(math::Vec2{u, -sign * pcHalf},
                                                          math::Vec2{u, sign * pcHalf});
            he[i]->pcurveStart = 0.0;
            he[i]->pcurveEnd = 1.0;
            // The meridian as a great circle: v is the angle from the equator toward z.
            geo::Frame mf;
            mf.x = {std::cos(u), std::sin(u), 0};
            mf.y = {0, 0, 1};
            mf.z = mf.x.cross(mf.y);
            e->exactCurve = std::make_shared<geo::CircleCurve>(mf, r);
            e->curveStart = -sign * half;
            e->curveEnd = sign * half;
        }
        south->halfEdge = he[0];
        north->halfEdge = he[1];
        w->halfEdge = he[0];
    }
};

}  // namespace

TEST(ExactValidator, ASphereLuneWhoseMeridiansMeetAtThePolesIsClean) {
    SphereLune lune(1.0, 0.0);
    EXPECT_TRUE(topo::validateExact(lune.solid, kTol).empty());
}

TEST(ExactValidator, ALuneWhosePcurvesStopShortOfThePolesWithinToleranceIsClean) {
    for (const double r : {1.0, 1000.0}) {
        const auto tol = math::ModelTolerance::forExtent(2.0 * r);
        SphereLune lune(r, 0.3 * tol.linear);
        EXPECT_TRUE(topo::validateExact(lune.solid, tol).empty()) << r;
    }
}

TEST(ExactValidator, TwoCrossingHolesAreFoundBesideABrokenOuterLoop) {
    const geo::Frame frame = geo::Frame::fromAxis({0, 0, 0}, {0, 0, 1});
    HalfCylinderFace base(2.5 * math::kPi);
    topo::Solid& solid = base.solid;
    topo::Face* f = &solid.faces()[0];
    // Break the outer loop: its second pcurve no longer meets its neighbours.
    f->outerLoop->halfEdge->next->pcurve = std::make_shared<geo::Line2D>(
        math::Vec2{3.0 * math::kPi, 0}, math::Vec2{3.0 * math::kPi, kHeight});
    const math::Vec2 none[4] = {{0, 0}, {0, 0}, {0, 0}, {0, 0}};
    for (const double d : {0.0, 0.13}) {  // the second hole is shifted both ways
        const double u = 2.0 * math::kPi + d;
        const double v = 1.0 + 1.7 * d;
        const math::Vec2 hole[4] = {{u + 0.2, v}, {u - 0.2, v}, {u - 0.2, v + 1}, {u + 0.2, v + 1}};
        addQuadLoop(solid, f, frame, hole, none, false);
    }
    // The outer loop's own faults come with it; the holes' crossing must still be reported.
    expectKinds(solid, ExactIssueKind::LoopsCross,
                {ExactIssueKind::LoopNotClosed, ExactIssueKind::PcurveOffCurve,
                 ExactIssueKind::PcurveOffVertex});
}

namespace {

/// A planar face outside a circle of radius 1000 about the origin's upper half and above the
/// line y = 0: a straight side and a semicircle, with a clockwise hole of radius 1 at (0, y).
struct SemicircleFace {
    topo::Solid solid;
    explicit SemicircleFace(double holeY) {
        const geo::Frame frame = geo::Frame::fromAxis({0, 0, 0}, {0, 0, 1});
        topo::Face* f = solid.allocFace();
        f->exactSurface = std::make_shared<geo::Plane>(frame);
        topo::Wire* w = solid.allocWire();
        f->outerLoop = w;
        topo::Vertex* v0 = solid.allocVertex();
        topo::Vertex* v1 = solid.allocVertex();
        v0->point = {-1000, 0, 0};
        v1->point = {1000, 0, 0};
        topo::HalfEdge* h0 = solid.allocHalfEdge();
        topo::HalfEdge* h1 = solid.allocHalfEdge();
        topo::Edge* e0 = solid.allocEdge();
        topo::Edge* e1 = solid.allocEdge();
        topo::HalfEdge* t0 = solid.allocHalfEdge();
        topo::HalfEdge* t1 = solid.allocHalfEdge();
        h0->origin = v0;
        t0->origin = v1;
        h1->origin = v1;
        t1->origin = v0;
        h0->next = h0->prev = h1;
        h1->next = h1->prev = h0;
        h0->face = h1->face = f;
        h0->twin = t0;
        t0->twin = h0;
        h1->twin = t1;
        t1->twin = h1;
        h0->edge = t0->edge = e0;
        h1->edge = t1->edge = e1;
        e0->halfEdge = h0;
        e1->halfEdge = h1;
        v0->halfEdge = h0;
        v1->halfEdge = h1;
        w->halfEdge = h0;
        h0->pcurve = std::make_shared<geo::Line2D>(math::Vec2{-1000, 0}, math::Vec2{1000, 0});
        h0->pcurveStart = 0.0;
        h0->pcurveEnd = 1.0;
        e0->exactCurve = std::make_shared<geo::LineCurve>(Vec3{-1000, 0, 0}, Vec3{1, 0, 0});
        e0->curveStart = 0.0;
        e0->curveEnd = 2000.0;
        h1->pcurve = std::make_shared<geo::Circle2D>(math::Vec2{0, 0}, 1000.0);
        h1->pcurveStart = 0.0;
        h1->pcurveEnd = math::kPi;
        e1->exactCurve = std::make_shared<geo::CircleCurve>(frame, 1000.0);
        e1->curveStart = 0.0;
        e1->curveEnd = math::kPi;
        // The hole: one clockwise circle, the mirrored frame running it the same way.
        topo::Wire* hw = solid.allocWire();
        f->innerLoops.push_back(hw);
        topo::Vertex* hv = solid.allocVertex();
        hv->point = {1, holeY, 0};
        topo::HalfEdge* hh = solid.allocHalfEdge();
        topo::HalfEdge* ht = solid.allocHalfEdge();
        topo::Edge* he = solid.allocEdge();
        hh->origin = ht->origin = hv;
        hh->next = hh->prev = hh;
        hh->face = f;
        hh->twin = ht;
        ht->twin = hh;
        hh->edge = ht->edge = he;
        he->halfEdge = hh;
        hv->halfEdge = hh;
        hw->halfEdge = hh;
        hh->pcurve = std::make_shared<geo::Circle2D>(math::Vec2{0, holeY}, 1.0);
        hh->pcurveStart = math::kTwoPi;
        hh->pcurveEnd = 0.0;
        geo::Frame mirrored = frame;
        mirrored.origin = {0, holeY, 0};
        mirrored.y = mirrored.y * -1.0;
        he->exactCurve = std::make_shared<geo::CircleCurve>(mirrored, 1.0);
        he->curveStart = 0.0;
        he->curveEnd = math::kTwoPi;
    }
};

}  // namespace

TEST(ExactValidator, AHoleBesideAStraightSideIsNotHiddenByAnotherSidesSag) {
    // The semicircle's 16 chords sag about 4.8; the hole is 1.5 to 3.5 beyond the straight side,
    // which has no sag at all.
    const auto tol = math::ModelTolerance::forExtent(2000.0);
    SemicircleFace outside(-2.5);
    const auto issues = topo::validateExact(outside.solid, tol);
    ASSERT_EQ(issues.size(), 1u);
    EXPECT_EQ(issues[0].kind, ExactIssueKind::HoleOutsideOuterLoop);
    SemicircleFace inside(500.0);
    EXPECT_TRUE(topo::validateExact(inside.solid, tol).empty());
}
