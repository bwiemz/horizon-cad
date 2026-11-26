#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "StepTestText.h"
#include "horizon/fileio/StepFormat.h"
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
#include "horizon/modeling/TrimmedFaceMesher.h"
#include "horizon/topology/ExactTolerance.h"
#include "horizon/topology/ExactValidator.h"
#include "horizon/topology/LoopUnwrap.h"

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

// The loop measures the validator judges by are one definition each (LoopUnwrap.h), which the
// STEP reader's face sense and model::insertSeams measure through too: a loop's signed area, its
// jumps included, and a half-edge's join to the next, whether linked to it or only named.
TEST(ExactValidator, ItsLoopMeasuresAreShared) {
    topo::LoopPolygon square;
    const math::Vec2 corners[4] = {{1.0, 1.0}, {3.0, 1.0}, {3.0, 3.0}, {1.0, 3.0}};
    for (int i = 0; i < 4; ++i) {
        square.push_back({nullptr, 0.0, 1.0, {}, corners[i], corners[(i + 1) % 4]});
    }
    EXPECT_EQ(topo::signedArea(square), 4.0);
    // Chords that do not join: the jump from one's end to the next one's start counts.
    topo::LoopPolygon gapped = {{nullptr, 0.0, 1.0, {}, {0.0, 0.0}, {2.0, 0.0}},
                                {nullptr, 0.0, 1.0, {}, {2.0, 2.0}, {0.0, 2.0}}};
    EXPECT_EQ(topo::signedArea(gapped), 4.0);
    std::reverse(square.begin(), square.end());
    for (auto& seg : square) std::swap(seg.a, seg.b);
    EXPECT_EQ(topo::signedArea(square), -4.0);

    const auto cyl = model::exact::makeCylinder(geo::Frame::fromAxis({}, {0, 0, 1}), 5.0, 10.0);
    for (const auto& f : cyl->faces()) {
        const topo::HalfEdge* h = f.outerLoop->halfEdge;
        do {
            EXPECT_EQ(topo::joinAllowance(*h, kTol), topo::joinAllowance(*h, *h->next, kTol));
            h = h->next;
        } while (h != f.outerLoop->halfEdge);
    }
}

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

/// A lune of the unit-scaled sphere of radius r (u in [0, pi], or with @p up and @p down the
/// lune up the meridian at @p up and down the one at @p down, each pcurve written at that u)
/// whose two meridians meet at the poles; each meridian's pcurve stops @p shortBy (a distance on
/// the sphere) before a pole, while the vertices sit exactly at the poles.
struct SphereLune {
    topo::Solid solid;
    SphereLune(double r, double shortBy, double up = math::kPi, double down = 0.0) {
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
            const double u = i == 0 ? up : down;
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

// Each pcurve may stop as far short of its pole as its vertex's tolerance. The join from one
// meridian's end to the next one's start at the pole runs along the pole: measured by the
// derivative at the end off the pole alone (a parallel's radius times the turn between the two
// longitudes, here a half turn), 0.9 of the tolerance short read as 2.8 tolerances, past the
// join's allowance of 2, and the join was refused; it is measured along the way. A lune of three
// quarters of a turn so short of its pole had its pole run taken for a wrap round the seam, a
// whole period off, and ran against its sense.
TEST(ExactValidator, ALuneWhosePcurvesStopShortOfThePolesWithinToleranceIsClean) {
    for (const double r : {1.0, 1000.0}) {
        const auto tol = math::ModelTolerance::forExtent(2.0 * r);
        for (const double shortBy : {0.3, 0.9}) {
            for (const double up : {math::kPi, 1.5 * math::kPi}) {
                SphereLune lune(r, shortBy * tol.linear, up);
                const auto issues = topo::validateExact(lune.solid, tol);
                EXPECT_TRUE(issues.empty()) << r << " " << shortBy << " " << up << ": "
                                            << (issues.empty() ? "" : issues.front().detail);
            }
        }
    }
}

// A lune does not fix which way round the poles its loop runs: its two meridians bound two lunes,
// and its face's sense picks one. Written within [0, 2 pi) with the sphere's seam inside it (up
// the meridian at pi / 2, down the one at 3 pi / 2), it is the lune through u = 0, and is clean;
// so is the lune the other way round, the same loop on the face reversed. Its pole runs are laid
// round the poles as the sense says; a loop with one pole run (an apex cone's side, its rim fixing
// its region) still runs against a flipped face.
TEST(ExactValidator, ALuneIsCleanWhicheverWayRoundItsSpheresSeamLies) {
    for (const bool reversed : {false, true}) {
        SphereLune lune(1.0, 0.0, 0.5 * math::kPi, 1.5 * math::kPi);
        lune.solid.faces()[0].reversed = reversed;
        const auto issues = topo::validateExact(lune.solid, kTol);
        EXPECT_TRUE(issues.empty())
            << reversed << ": " << (issues.empty() ? "" : issues.front().detail);
    }
    auto cone = model::exact::makeCone(geo::Frame{}, 2.0, 0.0, 4.0);
    for (auto& f : cone->faces()) {
        if (f.topoId.tag() == "cone/side") f.reversed = !f.reversed;
    }
    expectKinds(*cone, ExactIssueKind::LoopWrongOrientation);
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

namespace {

/// A planar face: an outer circle of radius @p outer about the origin, counter-clockwise, and
/// one clockwise circular hole for each of @p holes (its centre and radius), in that order.
struct CircleWithHoles {
    struct Hole {
        math::Vec2 centre;
        double radius;
    };
    topo::Solid solid;
    CircleWithHoles(double outer, const std::vector<Hole>& holes) {
        const geo::Frame frame = geo::Frame::fromAxis({0, 0, 0}, {0, 0, 1});
        topo::Face* f = solid.allocFace();
        f->exactSurface = std::make_shared<geo::Plane>(frame);
        std::vector<Hole> loops{{{0, 0}, outer}};
        loops.insert(loops.end(), holes.begin(), holes.end());
        for (std::size_t k = 0; k < loops.size(); ++k) {
            const bool isOuter = k == 0;
            const Hole& c = loops[k];
            topo::Wire* w = solid.allocWire();
            if (isOuter) {
                f->outerLoop = w;
            } else {
                f->innerLoops.push_back(w);
            }
            topo::Vertex* v = solid.allocVertex();
            v->point = {c.centre.x + c.radius, c.centre.y, 0};
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
            cf.origin = {c.centre.x, c.centre.y, 0};
            he->pcurve = std::make_shared<geo::Circle2D>(c.centre, c.radius);
            // A hole runs clockwise: its 3D circle's frame is mirrored so it runs the same way.
            he->pcurveStart = isOuter ? 0.0 : math::kTwoPi;
            he->pcurveEnd = isOuter ? math::kTwoPi : 0.0;
            if (!isOuter) cf.y = cf.y * -1.0;
            e->exactCurve = std::make_shared<geo::CircleCurve>(cf, c.radius);
            e->curveStart = 0.0;
            e->curveEnd = math::kTwoPi;
        }
    }
};

}  // namespace

TEST(ExactValidator, AHoleInsideAnotherHoleIsReportedAsTheMesherReportsIt) {
    // Concentric holes of radius 5 and 2 in a circle of radius 10: no loops cross and each hole
    // is inside the outer loop, but the smaller hole lies inside the larger one. The mesher
    // rejects that face, and a face the validator calls clean must mesh (Ruling 15).
    for (const auto& holes : {std::vector<CircleWithHoles::Hole>{{{0, 0}, 5.0}, {{0, 0}, 2.0}},
                              std::vector<CircleWithHoles::Hole>{{{0, 0}, 2.0}, {{0, 0}, 5.0}},
                              std::vector<CircleWithHoles::Hole>{{{-4, 0}, 5.0}, {{-5, 1}, 2.0}}}) {
        CircleWithHoles face(10.0, holes);
        const auto issues = topo::validateExact(face.solid, kTol);
        ASSERT_EQ(issues.size(), 1u) << holes[0].radius;
        EXPECT_EQ(issues[0].kind, ExactIssueKind::HoleInsideHole);
        const auto meshes = model::TrimmedFaceMesher::meshSolid(face.solid, {});
        ASSERT_EQ(meshes.size(), 1u);
        EXPECT_NE(meshes[0].problem.find("is inside hole"), std::string::npos) << meshes[0].problem;
    }
    // Side by side, the same holes are clean and mesh.
    CircleWithHoles apart(10.0, {{{-4, 0}, 5.0}, {{5, 0}, 2.0}});
    EXPECT_TRUE(topo::validateExact(apart.solid, kTol).empty());
    const auto meshes = model::TrimmedFaceMesher::meshSolid(apart.solid, {});
    ASSERT_EQ(meshes.size(), 1u);
    EXPECT_TRUE(meshes[0].problem.empty()) << meshes[0].problem;
}

TEST(ExactValidator, AnEdgeOverMoreThanOnePeriodIsRangeTooLong) {
    auto cyl = model::exact::makeCylinder(geo::Frame::fromAxis({0, 0, 0}, {0, 0, 1}), 5.0, 10.0);
    topo::Edge* rim = nullptr;
    for (auto& e : cyl->edges()) {
        if (dynamic_cast<const geo::CircleCurve*>(e.exactCurve.get())) {
            rim = &e;
            break;
        }
    }
    ASSERT_NE(rim, nullptr);
    rim->curveEnd += math::kTwoPi;
    int count = 0;
    for (const auto& i : topo::validateExact(*cyl, kTol)) {
        if (i.kind == ExactIssueKind::RangeTooLong && i.edge == rim) ++count;
    }
    EXPECT_EQ(count, 1);
}

TEST(ExactValidator, ALoopWiderThanAPeriodIsRangeTooLong) {
    auto cyl = model::exact::makeCylinder(geo::Frame::fromAxis({0, 0, 0}, {0, 0, 1}), 5.0, 10.0);
    topo::Face* side = nullptr;
    for (auto& f : cyl->faces()) {
        if (f.topoId.tag() == "cylinder/side") side = &f;
    }
    ASSERT_NE(side, nullptr);
    const double wide = 3.0 * math::kPi;
    const double h = 10.0;
    // The side's four pcurves, in loop order, rewritten to span u over 3 pi and still join.
    const math::Vec2 pts[4][2] = {
        {{0, 0}, {wide, 0}}, {{wide, 0}, {wide, h}}, {{wide, h}, {0, h}}, {{0, h}, {0, 0}}};
    topo::HalfEdge* he = side->outerLoop->halfEdge;
    for (const auto& seg : pts) {
        he->pcurve = std::make_shared<geo::Line2D>(seg[0], seg[1]);
        he->pcurveStart = 0.0;
        he->pcurveEnd = 1.0;
        he = he->next;
    }
    int count = 0;
    for (const auto& i : topo::validateExact(*cyl, kTol)) {
        if (i.kind == ExactIssueKind::RangeTooLong && i.face == side) ++count;
    }
    EXPECT_EQ(count, 1);
}

namespace {

/// The block with one of its straight edges given a straight B-spline of degree 1 for its curve
/// (as Horizon writes every edge), moved @p off across the edge, perpendicular to both of its
/// faces' normals... along the bisector of its two faces, so that both pcurves stray alike.
struct StraightSplineEdge {
    std::unique_ptr<topo::Solid> block = model::exact::makeBlock({0, 0, 0}, {2, 3, 4});
    topo::Edge* edge = nullptr;
    explicit StraightSplineEdge(double off) {
        for (auto& e : block->edges()) {
            const Vec3 a = e.exactCurve->evaluate(e.curveStart);
            const Vec3 b = e.exactCurve->evaluate(e.curveEnd);
            // The edge along z at x = 2, y = 3: its faces face +x and +y.
            if (std::abs(a.x - 2) > 1e-12 || std::abs(a.y - 3) > 1e-12 ||
                std::abs(b.x - 2) > 1e-12 || std::abs(b.y - 3) > 1e-12) {
                continue;
            }
            const Vec3 shift = Vec3{1, 1, 0}.normalized() * off;
            e.exactCurve = std::make_shared<geo::NurbsCurve>(
                std::vector<Vec3>{a + shift, b + shift}, std::vector<double>{1.0, 1.0},
                std::vector<double>{0, 0, 1, 1}, 1);
            e.curveStart = 0.0;
            e.curveEnd = 1.0;
            edge = &e;
            break;
        }
    }
};

}  // namespace

// Pins for the validator's pruning (Ruling 20): a straight B-spline edge whose pcurves stray just
// past its tolerance is reported, with the distance it strays, and one just within is clean.
TEST(ExactValidator, AStraightSplineEdgeJustOffItsPcurvesIsReportedWithHowFar) {
    const double limit = kTol.linear;
    StraightSplineEdge within(0.9 * limit);
    ASSERT_NE(within.edge, nullptr);
    EXPECT_TRUE(topo::validateExact(*within.block, kTol).empty());

    StraightSplineEdge past(1.5 * limit);
    ASSERT_NE(past.edge, nullptr);
    const auto issues = topo::validateExact(*past.block, kTol);
    ASSERT_EQ(issues.size(), 2u);
    for (const auto& i : issues) {
        EXPECT_EQ(i.kind, ExactIssueKind::PcurveOffCurve);
        EXPECT_EQ(i.edge, past.edge);
        EXPECT_EQ(i.detail, "pcurve strays 4.5e-05 from the edge's curve");
    }
}

namespace {

/// The most topo::pcurveGaps measures on either half-edge of @p e.
double mostMeasured(const topo::Edge& e) {
    double most = 0.0;
    for (const topo::HalfEdge* h : {e.halfEdge, e.halfEdge->twin}) {
        const topo::PcurveGaps gaps = topo::pcurveGaps(*h, kTol);
        most = std::max({most, gaps.start, gaps.end, gaps.strays});
    }
    return most;
}

/// With @p e's tolerance @p measured, @p solid is clean; with it a last bit lower, its only issues
/// are @p e's pcurves straying, each naming its half-edge, whose own measure is past the tolerance.
void expectStraysOnlyPast(topo::Solid& solid, topo::Edge& e, double measured) {
    e.tolerance = measured;
    EXPECT_TRUE(topo::validateExact(solid, kTol).empty()) << "a tolerance of the measure";
    e.tolerance = std::nextafter(measured, 0.0);
    const auto issues = topo::validateExact(solid, kTol);
    EXPECT_FALSE(issues.empty()) << "a tolerance a last bit below it";
    for (const auto& i : issues) {
        EXPECT_EQ(i.kind, ExactIssueKind::PcurveOffCurve) << i.detail;
        EXPECT_EQ(i.edge, &e);
        ASSERT_NE(i.halfEdge, nullptr) << i.detail;
        EXPECT_EQ(i.halfEdge->edge, &e);
        EXPECT_EQ(i.halfEdge->face, i.face);
        EXPECT_GT(topo::pcurveGaps(*i.halfEdge, kTol).strays, e.tolerance);
    }
}

}  // namespace

// The STEP reader measures a pcurve the validator reports with topo::pcurveGaps, to tell a last
// bit's rounding from a stray (Ruling 64), so it is what validateExact measures, to the last bit:
// on a cylinder whose rim's circle is moved out from its side (each sample's nearest point on the
// circle searched for) and on a straight B-spline edge moved off its faces (the fast path's
// segment), a tolerance of the measure is clean, and one a last bit below it is reported as the
// pcurves straying. Where the fast path accepts a pcurve as it stands, there is nothing to cover.
TEST(ExactValidator, PcurveGapsIsWhatItMeasures) {
    auto cyl = model::exact::makeCylinder(geo::Frame::fromAxis({3, -2, 7}, {0, 0, 1}), 5.0, 10.0);
    topo::Edge* rim = nullptr;
    for (auto& e : cyl->edges()) {
        if (const auto* c = dynamic_cast<const geo::CircleCurve*>(e.exactCurve.get())) {
            e.exactCurve = std::make_shared<geo::CircleCurve>(c->frame(), c->radius() + 1e-3);
            rim = &e;
            break;
        }
    }
    ASSERT_NE(rim, nullptr);
    const double onTheRim = mostMeasured(*rim);
    EXPECT_NEAR(onTheRim, 1e-3, 1e-12);
    expectStraysOnlyPast(*cyl, *rim, onTheRim);

    StraightSplineEdge off(1.5 * kTol.linear);
    ASSERT_NE(off.edge, nullptr);
    const double onTheSegment = mostMeasured(*off.edge);
    EXPECT_NEAR(onTheSegment, 1.5 * kTol.linear, 1e-12);
    expectStraysOnlyPast(*off.block, *off.edge, onTheSegment);

    StraightSplineEdge within(0.5 * kTol.linear);
    ASSERT_NE(within.edge, nullptr);
    for (const topo::HalfEdge* h : {within.edge->halfEdge, within.edge->halfEdge->twin}) {
        EXPECT_EQ(topo::pcurveGaps(*h, kTol).strays, 0.0);
    }
}

namespace {

/// A cylinder (radius 5, height 10) whose bottom rim's vertex is moved @p vertexOff out from its
/// side and given a tolerance a hundredth of a unit in the last place short of that, and whose
/// rim's circle is moved out so that it passes 1e-9 inside the vertex: every pcurve at the vertex
/// ends past the vertex's tolerance by rounding, and the rim's two pcurves stray @p vertexOff -
/// 1e-9 from its circle. The rim's tolerance is @p rimTolerance; the seam's is the resolution.
struct RimOffItsVertex {
    std::unique_ptr<topo::Solid> cyl =
        model::exact::makeCylinder(geo::Frame::fromAxis({3, -2, 7}, {0, 0, 1}), 5.0, 10.0);
    topo::Edge* rim = nullptr;
    topo::Vertex* vertex = nullptr;
    double strays = 0.0;
    RimOffItsVertex(double vertexOff, double rimTolerance) {
        for (auto& e : cyl->edges()) {
            const auto* c = dynamic_cast<const geo::CircleCurve*>(e.exactCurve.get());
            if (c == nullptr || std::abs(c->frame().origin.z - 7.0) > 1e-12) continue;
            rim = &e;
            vertex = e.halfEdge->origin;
            const Vec3 centre = c->frame().origin;
            const double radius = c->radius();
            vertex->point = centre + (vertex->point - centre) * ((radius + vertexOff) / radius);
            vertex->tolerance = vertexOff - 1e-14;
            strays = vertexOff - 1e-9;
            e.exactCurve = std::make_shared<geo::CircleCurve>(c->frame(), radius + strays);
            e.tolerance = rimTolerance;
            break;
        }
    }
};

}  // namespace

// Rounding is taken up all or nothing (Ruling 66). Every pcurve at the rim's vertex ends past the
// vertex's tolerance by rounding, and the rim's pcurves stray 1e-3 from its circle while its own
// tolerance is half that: a clear stray beside the shortfalls. Nothing is taken up, and the stray
// is what is reported: the ends' shortfalls are bound by the vertex's tolerance, which is above
// the rim's, and growing the rim's to cover them would have covered its stray too.
TEST(ExactValidator, ARoundingShortfallBesideAStrayTakesUpNothing) {
    RimOffItsVertex beside(1e-3, 0.5e-3);
    ASSERT_NE(beside.rim, nullptr);
    const auto issues = topo::validateExact(*beside.cyl, kTol);
    std::set<ExactIssueKind> kinds;
    for (const auto& i : issues) kinds.insert(i.kind);
    EXPECT_EQ(kinds, (std::set<ExactIssueKind>{ExactIssueKind::PcurveOffCurve,
                                               ExactIssueKind::PcurveOffVertex}));
    const topo::RoundingTakeUp takeUp = topo::roundingShortfalls(*beside.cyl, issues, kTol);
    ASSERT_NE(takeUp.beyond, nullptr);
    EXPECT_EQ(takeUp.beyond->kind, ExactIssueKind::PcurveOffCurve);
    EXPECT_EQ(takeUp.beyond->edge, beside.rim);
    EXPECT_EQ(takeUp.beyond->detail, "pcurve strays 0.001 from the edge's curve");
    EXPECT_TRUE(takeUp.shortfalls.empty());
}

// With the stray gone (the rim's tolerance its strays less a hundredth of a unit in the last
// place), every issue is a rounding shortfall, each of the tolerance its check was bound by: the
// rim's for its strays, the vertex's for the ends there, the larger. Grown to what the validator
// measured, the solid is clean, and the rim's tolerance has grown by rounding alone.
TEST(ExactValidator, RoundingShortfallsAloneAreEachTakenUpWhereTheyAreBound) {
    RimOffItsVertex alone(1e-3, 1e-3 - 1e-9 - 1e-14);
    ASSERT_NE(alone.rim, nullptr);
    const double rimBefore = alone.rim->tolerance;
    const auto issues = topo::validateExact(*alone.cyl, kTol);
    ASSERT_FALSE(issues.empty());
    const topo::RoundingTakeUp takeUp = topo::roundingShortfalls(*alone.cyl, issues, kTol);
    ASSERT_EQ(takeUp.beyond, nullptr) << (takeUp.beyond ? takeUp.beyond->detail : "");
    ASSERT_FALSE(takeUp.shortfalls.empty());
    for (const topo::RoundingShortfall& s : takeUp.shortfalls) {
        if (s.issue->kind == ExactIssueKind::PcurveOffCurve) {
            EXPECT_EQ(s.tolerance, &alone.rim->tolerance);
        } else {
            EXPECT_EQ(s.tolerance, &alone.vertex->tolerance) << s.issue->detail;
        }
        *s.tolerance = std::max(*s.tolerance, s.measured);
    }
    EXPECT_TRUE(topo::validateExact(*alone.cyl, kTol).empty());
    EXPECT_LE(alone.rim->tolerance - rimBefore, 1e-12);
}

// The rounding allowed is that of the magnitudes a measure is computed from: the same rim on a
// cylinder placed a million along its own axis is allowed a million times the rounding.
TEST(ExactValidator, RoundingIsOfTheMagnitudesAMeasureIsComputedFrom) {
    const auto near =
        model::exact::makeCylinder(geo::Frame::fromAxis({0, 0, 0}, {0, 0, 1}), 1.0, 2.0);
    const topo::HalfEdge* h = nullptr;
    for (const auto& f : near->faces()) {
        if (f.exactSurface->kind() == geo::SurfaceKind::Cylinder) h = f.outerLoop->halfEdge;
    }
    ASSERT_NE(h, nullptr);
    const double here = topo::pcurveRounding(*near, *h);
    EXPECT_GT(here, 0.0);
    EXPECT_LT(here, 1e-12);

    auto far = model::exact::makeCylinder(geo::Frame::fromAxis({0, 0, 0}, {0, 0, 1}), 1.0, 2.0);
    for (auto& f : far->faces()) {
        if (f.exactSurface->kind() != geo::SurfaceKind::Cylinder) continue;
        // The same surface, placed a million down its axis: v runs a million up it.
        f.exactSurface =
            std::make_shared<geo::Cylinder>(geo::Frame::fromAxis({0, 0, -1e6}, {0, 0, 1}), 1.0);
        h = f.outerLoop->halfEdge;
    }
    for (auto& e : far->edges()) {
        for (topo::HalfEdge* he : {e.halfEdge, e.halfEdge->twin}) {
            if (he->face->exactSurface->kind() != geo::SurfaceKind::Cylinder) continue;
            const auto* line = dynamic_cast<const geo::Line2D*>(he->pcurve.get());
            ASSERT_NE(line, nullptr);
            he->pcurve = std::make_shared<geo::Line2D>(line->start() + math::Vec2{0, 1e6},
                                                       line->end() + math::Vec2{0, 1e6});
        }
    }
    EXPECT_TRUE(topo::validateExact(*far, kTol).empty()) << "the same cylinder";
    EXPECT_GT(topo::pcurveRounding(*far, *h), 1e5 * here);
}

// A straight B-spline edge (two points, the validator's straight segment) one of whose pcurves
// runs there and back and there again in (u, v): every point of it lies on the segment, so only
// the order test can tell, and it is reported as running against the curve, as it was.
TEST(ExactValidator, APcurveRunningBackAlongAStraightSplineEdgeIsReported) {
    StraightSplineEdge edge(0.0);
    ASSERT_NE(edge.edge, nullptr);
    ASSERT_EQ(static_cast<const geo::NurbsCurve&>(*edge.edge->exactCurve).controlPointCount(), 2);
    EXPECT_TRUE(topo::validateExact(*edge.block, kTol).empty());
    topo::HalfEdge* h = edge.edge->halfEdge;
    const math::Vec2 a = h->pcurve->evaluate(h->pcurveStart);
    const math::Vec2 b = h->pcurve->evaluate(h->pcurveEnd);
    h->pcurve = std::make_shared<geo::Nurbs2D>(
        std::vector<math::Vec2>{a, b, a, b}, std::vector<double>{1, 1, 1, 1},
        std::vector<double>{0, 0, 1.0 / 3, 2.0 / 3, 1, 1}, 1);
    h->pcurveStart = 0.0;
    h->pcurveEnd = 1.0;
    const auto issues = topo::validateExact(*edge.block, kTol);
    ASSERT_EQ(issues.size(), 1u);
    EXPECT_EQ(issues[0].kind, ExactIssueKind::PcurveOffCurve);
    EXPECT_EQ(issues[0].edge, edge.edge);
    EXPECT_EQ(issues[0].face, h->face);
    EXPECT_EQ(issues[0].detail,
              "pcurve runs against, or winds differently round, the edge's curve");
}

// Holes whose boxes overlap without the holes meeting keep the face clean, and holes that do
// meet, or lie one inside the other, are reported as before, among many: what the validator
// prunes is only what cannot cross or contain.
TEST(ExactValidator, HolesWhoseBoxesOverlapKeepTheirVerdicts) {
    // Diagonal neighbours: boxes overlap in [-0.6, 0.6]^2, the circles are 0.46 apart.
    CircleWithHoles near(10.0, {{{-2, -2}, 2.6}, {{2, 2}, 2.6}});
    EXPECT_TRUE(topo::validateExact(near.solid, kTol).empty());
    CircleWithHoles meeting(10.0, {{{-1.5, 0}, 2.0}, {{1.5, 0}, 2.0}});
    const auto crossing = topo::validateExact(meeting.solid, kTol);
    ASSERT_EQ(crossing.size(), 1u);
    EXPECT_EQ(crossing[0].kind, ExactIssueKind::LoopsCross);
    EXPECT_EQ(crossing[0].detail, "two loops of the face cross");
    EXPECT_EQ(crossing[0].edge, meeting.solid.faces()[0].innerLoops[1]->halfEdge->edge);

    // A grid of 25 holes 3 apart, clean; then the middle one moved onto its neighbour (it meets
    // that one only), and a small one added inside the middle one.
    std::vector<CircleWithHoles::Hole> grid;
    for (int i = 0; i < 5; ++i) {
        for (int j = 0; j < 5; ++j) grid.push_back({{-6.0 + 3.0 * i, -6.0 + 3.0 * j}, 1.2});
    }
    CircleWithHoles many(20.0, grid);
    EXPECT_TRUE(topo::validateExact(many.solid, kTol).empty());
    auto touching = grid;
    touching[12].centre.x += 1.0;
    CircleWithHoles crowded(20.0, touching);
    const auto met = topo::validateExact(crowded.solid, kTol);
    ASSERT_EQ(met.size(), 1u);
    EXPECT_EQ(met[0].kind, ExactIssueKind::LoopsCross);
    EXPECT_EQ(met[0].edge, crowded.solid.faces()[0].innerLoops[17]->halfEdge->edge);
    auto nested = grid;
    nested.push_back({{0.0, 0.0}, 0.5});
    CircleWithHoles inside(20.0, nested);
    const auto inner = topo::validateExact(inside.solid, kTol);
    ASSERT_EQ(inner.size(), 1u);
    EXPECT_EQ(inner[0].kind, ExactIssueKind::HoleInsideHole);
    EXPECT_EQ(inner[0].detail, "inner loops 12 and 25 lie one inside the other");
    // A hole moved out past the outer loop is outside it.
    auto out = grid;
    out[0].centre = {30.0, 0.0};
    CircleWithHoles outside(20.0, out);
    const auto beyond = topo::validateExact(outside.solid, kTol);
    ASSERT_EQ(beyond.size(), 1u);
    EXPECT_EQ(beyond[0].kind, ExactIssueKind::HoleOutsideOuterLoop);
}

namespace {

/// What the validator read a pcurve's gap as before its peaks were searched for: the farthest of
/// its 17 samples, on the surface, from the nearest point of its edge's curve.
double atItsSamplesAlone(const topo::HalfEdge& h) {
    const geo::Surface& s = *h.face->exactSurface;
    const geo::Curve3& c = *h.edge->exactCurve;
    double most = 0.0;
    for (int i = 0; i < topo::kLoopSamples; ++i) {
        const math::Vec2 uv =
            h.pcurve->evaluate(topo::loopSampleParameter(h, i, topo::kLoopSamples));
        const Vec3 p = s.evaluate(uv.x, uv.y);
        most = std::max(most, (c.evaluate(c.closestParameter(p)) - p).length());
    }
    return most;
}

/// The farthest of 20,000 points of @p h's pcurve, on the surface, from its edge's curve.
double densely(const topo::HalfEdge& h) {
    const geo::Surface& s = *h.face->exactSurface;
    const geo::Curve3& c = *h.edge->exactCurve;
    double most = 0.0;
    constexpr int kN = 20000;
    for (int k = 0; k <= kN; ++k) {
        const double t = h.pcurveStart + (h.pcurveEnd - h.pcurveStart) * k / kN;
        const math::Vec2 uv = h.pcurve->evaluate(t);
        const Vec3 p = s.evaluate(uv.x, uv.y);
        most = std::max(most, (c.evaluate(c.closestParameter(p)) - p).length());
    }
    return most;
}

/// These files name no length unit, as hand-written ones often do: read in millimetres.
io::StepReadOptions inMillimetres() {
    io::StepReadOptions options;
    options.unknownLengthUnit = math::LengthUnit::Millimetre;
    return options;
}

}  // namespace

// A gap that peaks between the validator's samples is measured at its peak, not at the samples
// (stage 2's lesson: the reader grows a tolerance to what one measure finds, and the validator
// checks it with the same). The cylinder's top rim is moved to the circle through its vertex whose
// centre is 1e-3 off the axis, towards 1 rad round it: the rim stands off the side's parallel and
// the cap's circle by a gap peaking half a turn from there, between the 17 samples of either
// pcurve, which read it 1.6e-5 short. With the rim's tolerance what the samples read, it strays;
// with it the peak, it is clean, and a last bit below that, it strays.
//
// Ruling 23's tilted band rim too: read, its rim is exact to the peak of its gap; with its pieces'
// tolerance what pcurveOn read at its samples before (0.0015042933), the piece holding the peak
// strays 3.7e-7 past it.
TEST(ExactValidator, APeakBetweenSamplesStrays) {
    auto cyl = model::exact::makeCylinder(geo::Frame::fromAxis({0, 0, 0}, {0, 0, 1}), 1.0, 2.0);
    topo::Edge* rim = nullptr;
    for (auto& e : cyl->edges()) {
        const auto* c = dynamic_cast<const geo::CircleCurve*>(e.exactCurve.get());
        if (c == nullptr || std::abs(c->frame().origin.z - 2.0) > 1e-12) continue;
        geo::Frame f = c->frame();
        const Vec3 vertex = e.halfEdge->origin->point;
        f.origin = f.origin + Vec3{std::cos(1.0), std::sin(1.0), 0.0} * 1e-3;
        f.x = (vertex - f.origin) * (1.0 / (vertex - f.origin).length());
        f.y = f.z.cross(f.x);
        ASSERT_EQ(e.curveStart, 0.0);
        e.exactCurve = std::make_shared<geo::CircleCurve>(f, (vertex - f.origin).length());
        rim = &e;
    }
    ASSERT_NE(rim, nullptr);
    const double sampled =
        std::max(atItsSamplesAlone(*rim->halfEdge), atItsSamplesAlone(*rim->halfEdge->twin));
    const double peak = std::max(densely(*rim->halfEdge), densely(*rim->halfEdge->twin));
    ASSERT_GT(peak - sampled, 1.5e-5) << "the peak lies between the samples";

    rim->tolerance = sampled;
    const auto issues = topo::validateExact(*cyl, kTol);
    ASSERT_FALSE(issues.empty()) << "a tolerance of what the samples read";
    for (const auto& i : issues) {
        EXPECT_EQ(i.kind, ExactIssueKind::PcurveOffCurve) << i.detail;
        EXPECT_EQ(i.edge, rim);
    }
    const double measured = mostMeasured(*rim);
    EXPECT_NEAR(measured, peak, 0.01 * peak);
    EXPECT_GE(measured, peak - 1e-15) << "never below the peak";
    expectStraysOnlyPast(*cyl, *rim, measured);

    const auto band = io::StepFormat::fromString(
        hz::step_text::bandWithATiltedRim(8e-4, 0.27, 6.06), nullptr, nullptr, inMillimetres());
    ASSERT_EQ(band.size(), 1u) << io::StepFormat::lastError();
    const math::ModelTolerance resolution = topo::exactTolerance(*band[0]);
    EXPECT_TRUE(topo::validateExact(*band[0], resolution).empty());
    constexpr double kAtSamples = 0.0015042933256460393;
    std::vector<topo::Edge*> pieces;
    for (auto& e : band[0]->edges()) {
        if (e.alive && e.tolerance > 1e-3) pieces.push_back(&e);
    }
    ASSERT_FALSE(pieces.empty());
    for (topo::Edge* e : pieces) {
        EXPECT_GE(e->tolerance - kAtSamples, 3.69e-7) << "grown to the peak";
        e->tolerance = kAtSamples;
    }
    const auto strays = topo::validateExact(*band[0], resolution);
    ASSERT_FALSE(strays.empty());
    for (const auto& i : strays) {
        EXPECT_EQ(i.kind, ExactIssueKind::PcurveOffCurve) << i.detail;
        ASSERT_NE(i.halfEdge, nullptr);
        EXPECT_GE(topo::pcurveGaps(*i.halfEdge, resolution).strays - kAtSamples, 3.69e-7);
    }
}
