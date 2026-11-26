// Seams where a periodic face has none (stage 2 of the exact kernel). Other systems write a
// cylinder's side between its two rims as a band: two loops that each go once round it, and no
// seam. The exact kernel's loops close in (u, v), so insertSeams splits the rims where the
// seam's line crosses them and joins them by a seam along the surface's own iso-curve, used once
// each way. The solids here are built as a STEP reader builds one: each face on its surface,
// each edge on its curve with a carrier beside it, every pcurve from geo::pcurveOn.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "horizon/geometry/Pcurve.h"
#include "horizon/geometry/curves/AnalyticCurves.h"
#include "horizon/geometry/curves/NurbsCurve.h"
#include "horizon/geometry/surfaces/AnalyticSurfaces.h"
#include "horizon/geometry/surfaces/NurbsSurface.h"
#include "horizon/math/Constants.h"
#include "horizon/modeling/BoundaryMesh.h"
#include "horizon/modeling/ExactPrimitives.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/modeling/SeamInsertion.h"
#include "horizon/topology/ExactTolerance.h"
#include "horizon/topology/ExactValidator.h"
#include "horizon/topology/Solid.h"

namespace {

namespace geo = hz::geo;
namespace topo = hz::topo;
using hz::math::kPi;
using hz::math::kTwoPi;
using hz::math::Vec3;

using Use = std::pair<topo::Edge*, bool>;  // an edge, and whether the loop runs it forward
using Loop = std::vector<Use>;

/// An arc of the circle of @p radius in @p frame's x and y, from angle @p a0 to @p a1 the way
/// the frame turns, as a rational B-spline: a carrier, as the reader makes its arcs.
std::shared_ptr<geo::NurbsCurve> arcCarrier(const geo::Frame& frame, double radius, double a0,
                                            double a1) {
    const geo::NurbsCurve flat = geo::NurbsCurve::makeArc({0.0, 0.0, 0.0}, radius, a0, a1);
    std::vector<Vec3> points;
    for (const Vec3& p : flat.controlPoints()) {
        points.push_back(frame.origin + frame.x * p.x + frame.y * p.y);
    }
    return std::make_shared<geo::NurbsCurve>(points, flat.weights(), flat.knots(), flat.degree());
}

/// The straight carrier from @p a to @p b.
std::shared_ptr<geo::NurbsCurve> lineCarrier(const Vec3& a, const Vec3& b) {
    return std::make_shared<geo::NurbsCurve>(std::vector<Vec3>{a, b}, std::vector<double>{1, 1},
                                             std::vector<double>{0, 0, 1, 1}, 1);
}

/// A solid built face by face, as the STEP reader builds one.
class Builder {
public:
    Builder() : m_solid(std::make_unique<topo::Solid>()), m_shell(m_solid->allocShell()) {
        m_shell->solid = m_solid.get();
    }

    topo::Vertex* vertex(const Vec3& p) {
        topo::Vertex* v = m_solid->allocVertex();
        v->point = p;
        return v;
    }

    /// An edge on @p curve from @p from at @p t0 to @p to at @p t1, its carrier @p carrier.
    topo::Edge* edge(topo::Vertex* from, topo::Vertex* to, std::shared_ptr<const geo::Curve3> curve,
                     double t0, double t1, std::shared_ptr<geo::NurbsCurve> carrier) {
        topo::Edge* e = m_solid->allocEdge();
        topo::HalfEdge* f = m_solid->allocHalfEdge();
        topo::HalfEdge* b = m_solid->allocHalfEdge();
        f->origin = from;
        b->origin = to;
        f->twin = b;
        b->twin = f;
        f->edge = b->edge = e;
        e->halfEdge = f;
        e->exactCurve = std::move(curve);
        e->curveStart = t0;
        e->curveEnd = t1;
        e->curve = std::move(carrier);
        e->topoId = topo::TopologyID::make("test", "e:" + std::to_string(m_solid->edgeCount()));
        if (from->halfEdge == nullptr) from->halfEdge = f;
        if (to->halfEdge == nullptr) to->halfEdge = b;
        return e;
    }

    /// A circle's whole round as an edge, from its point at @p t0 the way it turns (@p withCurve)
    /// or the other way, with a vertex there; its carrier the whole circle, the way it turns,
    /// from that vertex.
    topo::Edge* rim(const geo::Frame& frame, double radius, double t0, bool withCurve) {
        const auto circle = std::make_shared<geo::CircleCurve>(frame, radius);
        topo::Vertex* v = vertex(circle->evaluate(t0));
        return edge(v, v, circle, t0, withCurve ? t0 + kTwoPi : t0 - kTwoPi,
                    arcCarrier(frame, radius, t0, t0 + kTwoPi));
    }

    /// A face on @p surface: its first loop its outer loop. A face given no sense (a plane's)
    /// faces the way its outer loop runs, once its pcurves are built.
    topo::Face* face(std::shared_ptr<const geo::Surface> surface, std::optional<bool> reversed,
                     const std::vector<Loop>& loops, const std::string& name) {
        topo::Face* f = m_solid->allocFace();
        f->topoId = topo::TopologyID::make("test", name);
        f->exactSurface = std::move(surface);
        f->reversed = reversed.value_or(false);
        f->shell = m_shell;
        m_shell->faces.push_back(f);
        for (std::size_t k = 0; k < loops.size(); ++k) {
            topo::Wire* w = m_solid->allocWire();
            std::vector<topo::HalfEdge*> hes;
            for (const auto& [e, forward] : loops[k]) {
                hes.push_back(forward ? e->halfEdge : e->halfEdge->twin);
            }
            w->halfEdge = hes.front();
            for (std::size_t i = 0; i < hes.size(); ++i) {
                hes[i]->face = f;
                hes[i]->next = hes[(i + 1) % hes.size()];
                hes[i]->next->prev = hes[i];
            }
            if (k == 0) {
                f->outerLoop = w;
            } else {
                f->innerLoops.push_back(w);
            }
        }
        if (!reversed) m_byLoop.push_back(f);
        return f;
    }

    /// The solid, every half-edge's pcurve built by geo::pcurveOn at its model resolution; null
    /// where one cannot be.
    std::unique_ptr<topo::Solid> take() {
        const double resolution = topo::exactTolerance(*m_solid).linear;
        for (auto& e : m_solid->edges()) {
            for (topo::HalfEdge* h : {e.halfEdge, e.halfEdge->twin}) {
                const bool forward = h == e.halfEdge;
                const auto pc = geo::pcurveOn(*h->face->exactSurface, *e.exactCurve,
                                              forward ? e.curveStart : e.curveEnd,
                                              forward ? e.curveEnd : e.curveStart, resolution);
                if (!pc.problem.empty() || pc.curve == nullptr) {
                    ADD_FAILURE() << h->face->topoId.tag() << ": " << pc.problem;
                    return nullptr;
                }
                h->pcurve = pc.curve;
                h->pcurveStart = pc.start;
                h->pcurveEnd = pc.end;
            }
        }
        for (topo::Face* f : m_byLoop) f->reversed = outerArea(*f) < 0.0;
        return std::move(m_solid);
    }

private:
    /// The signed area in (u, v) of @p f's outer loop, on a surface that does not wrap.
    static double outerArea(const topo::Face& f) {
        std::vector<hz::math::Vec2> points;
        const topo::HalfEdge* h = f.outerLoop->halfEdge;
        do {
            for (int i = 0; i < 64; ++i) {
                points.push_back(
                    h->pcurve->evaluate(h->pcurveStart + (h->pcurveEnd - h->pcurveStart) * i / 64));
            }
            h = h->next;
        } while (h != f.outerLoop->halfEdge);
        double twice = 0.0;
        for (std::size_t i = 0; i < points.size(); ++i) {
            const auto& p = points[i];
            const auto& q = points[(i + 1) % points.size()];
            twice += p.x * q.y - q.x * p.y;
        }
        return twice / 2.0;
    }

    std::unique_ptr<topo::Solid> m_solid;
    topo::Shell* m_shell;
    std::vector<topo::Face*> m_byLoop;
};

/// A plane through @p origin facing @p normal.
std::shared_ptr<const geo::Surface> plane(const Vec3& origin, const Vec3& normal) {
    return std::make_shared<geo::Plane>(geo::Frame::fromAxis(origin, normal, {1.0, 0.0, 0.0}));
}

geo::Frame frameAt(const Vec3& origin, const Vec3& axis, const Vec3& x = {1.0, 0.0, 0.0}) {
    return geo::Frame::fromAxis(origin, axis, x);
}

const Vec3 kZ{0.0, 0.0, 1.0};

/// A cylinder of radius 1 and height 2 about z, its side a band between its rims as
/// test_StepCurved's bandCylinder writes it: its bottom rim's vertex at u = 0, its top rim's
/// @p topAt round.
std::unique_ptr<topo::Solid> bandCylinder(double topAt = 0.0) {
    Builder b;
    topo::Edge* bottom = b.rim(frameAt({0, 0, 0}, kZ), 1.0, 0.0, true);
    topo::Edge* top = b.rim(frameAt({0, 0, 2}, kZ), 1.0, topAt, true);
    b.face(std::make_shared<geo::Cylinder>(frameAt({0, 0, 0}, kZ), 1.0), false,
           {{{bottom, true}}, {{top, false}}}, "band/side");
    b.face(plane({0, 0, 0}, kZ), std::nullopt, {{{bottom, false}}}, "band/bottom");
    b.face(plane({0, 0, 2}, kZ), std::nullopt, {{{top, true}}}, "band/top");
    return b.take();
}

/// bandCylinder() with its bottom rim's vertex at u = pi / 2 and its top rim two half circles,
/// from 3 pi / 2 round to pi / 2 and on round back: the top loop meets the seam's line at its
/// second vertex.
std::unique_ptr<topo::Solid> bandWithATwoArcRim() {
    Builder b;
    topo::Edge* bottom = b.rim(frameAt({0, 0, 0}, kZ), 1.0, kPi / 2.0, true);
    const geo::Frame top = frameAt({0, 0, 2}, kZ);
    const auto circle = std::make_shared<geo::CircleCurve>(top, 1.0);
    topo::Vertex* west = b.vertex(circle->evaluate(1.5 * kPi));
    topo::Vertex* east = b.vertex(circle->evaluate(0.5 * kPi));
    topo::Edge* front = b.edge(west, east, circle, 1.5 * kPi, 2.5 * kPi,
                               arcCarrier(top, 1.0, 1.5 * kPi, 2.5 * kPi));
    topo::Edge* back = b.edge(east, west, circle, 0.5 * kPi, 1.5 * kPi,
                              arcCarrier(top, 1.0, 0.5 * kPi, 1.5 * kPi));
    b.face(std::make_shared<geo::Cylinder>(frameAt({0, 0, 0}, kZ), 1.0), false,
           {{{bottom, true}}, {{back, false}, {front, false}}}, "band/side");
    b.face(plane({0, 0, 0}, kZ), std::nullopt, {{{bottom, false}}}, "band/bottom");
    b.face(plane({0, 0, 2}, kZ), std::nullopt, {{{front, true}, {back, true}}}, "band/top");
    return b.take();
}

// ---- what to check ----

/// The first of @p solid's exact issues, or "" when it has none.
std::string firstIssue(const topo::Solid& solid) {
    const auto issues = topo::validateExact(solid, topo::exactTolerance(solid));
    return issues.empty()
               ? std::string()
               : issues.front().detail + " (" +
                     (issues.front().face ? issues.front().face->topoId.tag() : "") + ")";
}

/// The edges both of whose half-edges bound @p face.
std::vector<const topo::Edge*> seamsOf(const topo::Face& face) {
    std::vector<const topo::Edge*> seams;
    std::vector<const topo::Wire*> loops{face.outerLoop};
    loops.insert(loops.end(), face.innerLoops.begin(), face.innerLoops.end());
    for (const topo::Wire* w : loops) {
        const topo::HalfEdge* h = w->halfEdge;
        do {
            if (h->twin->face == &face && h == h->edge->halfEdge) seams.push_back(h->edge);
            h = h->next;
        } while (h != w->halfEdge);
    }
    return seams;
}

const topo::Face* faceNamed(const topo::Solid& solid, const std::string& name) {
    for (const auto& f : solid.faces()) {
        if (f.topoId.tag() == "test/" + name) return &f;
    }
    return nullptr;
}

/// Why @p solid's exact faces do not mesh, or "".
std::string meshProblem(const topo::Solid& solid) {
    std::string problem;
    hz::model::BoundaryMesh::extractFacePolygons(solid, 1e-3, &problem);
    return problem;
}

/// The volume of @p solid, integrated on its exact faces: a seam laid wrong (a region cut short,
/// a rim split at the wrong place) shows in it to the integration's 1e-9.
double exactVolume(const topo::Solid& solid) {
    const auto props = hz::model::MassPropertiesCalculator::compute(solid);
    EXPECT_TRUE(props.valid) << props.problem;
    return props.volume;
}
constexpr double kExactVolumeError = 1e-9;

/// Every edge's carrier ends at its vertices and lies along its exact curve. With
/// @p withinTolerance, its ends lie within the edge's tolerance of its vertices instead: a seam
/// joining pcurves fitted to their curves passes as near its vertices as they do, and its
/// tolerance covers that.
void expectCarriersOnTheirCurves(const topo::Solid& solid, bool withinTolerance = false) {
    const auto tolerance = topo::exactTolerance(solid);
    for (const auto& e : solid.edges()) {
        if (!e.curve) continue;
        const Vec3 a = e.halfEdge->origin->point;
        const Vec3 b = e.halfEdge->twin->origin->point;
        const Vec3 c0 = e.curve->evaluate(e.curve->tMin());
        const Vec3 c1 = e.curve->evaluate(e.curve->tMax());
        const double ends = std::min(std::max((c0 - a).length(), (c1 - b).length()),
                                     std::max((c0 - b).length(), (c1 - a).length()));
        const double near =
            withinTolerance
                ? tolerance.effective(std::max({e.tolerance, e.halfEdge->origin->tolerance,
                                                e.halfEdge->twin->origin->tolerance}))
                : 1e-9;
        EXPECT_LT(ends, near) << e.topoId.tag();
        for (int k = 1; k < 8; ++k) {
            const double t = e.curveStart + (e.curveEnd - e.curveStart) * k / 8.0;
            const Vec3 p = e.exactCurve->evaluate(t);
            EXPECT_LT((e.curve->evaluate(e.curve->closestPoint(p)) - p).length(), 1e-9)
                << e.topoId.tag() << " at " << k << "/8 of its curve";
        }
    }
}

}  // namespace

// Review Focus 2: test_StepCurved's bandCylinder, built as the reader builds it, given its seam.
// Its rims' vertices are both at u = 0, so the seam joins them without a split: a line up the
// side, used once each way. The validator is clean, the read point meshes it, and its volume is
// the cylinder's.
TEST(SeamInsertion, ABandGetsASeamAndStaysExact) {
    const auto solid = bandCylinder();
    ASSERT_NE(solid, nullptr);
    EXPECT_NE(firstIssue(*solid), "") << "a band is not exact without its seam";

    EXPECT_EQ(hz::model::insertSeams(*solid, topo::exactTolerance(*solid)), "");
    const topo::Face* side = faceNamed(*solid, "band/side");
    ASSERT_NE(side, nullptr);
    const auto seams = seamsOf(*side);
    ASSERT_EQ(seams.size(), 1u);
    EXPECT_EQ(seams[0]->exactCurve->kind(), geo::CurveKind::Line);
    EXPECT_TRUE(side->innerLoops.empty()) << "its rims are one loop with its seam";
    EXPECT_EQ(solid->vertexCount(), 2u) << "nothing split";
    EXPECT_EQ(solid->edgeCount(), 3u);
    EXPECT_TRUE(solid->isValid()) << solid->validationReport();
    EXPECT_EQ(firstIssue(*solid), "");
    EXPECT_EQ(meshProblem(*solid), "");
    expectCarriersOnTheirCurves(*solid);
    EXPECT_NEAR(exactVolume(*solid), 2.0 * kPi, 2.0 * kPi * kExactVolumeError);

    // Its top rim's vertex two radians round: the top rim is split where the seam's line, up
    // from the bottom rim's vertex, crosses it.
    const auto turned = bandCylinder(2.0);
    ASSERT_NE(turned, nullptr);
    EXPECT_EQ(hz::model::insertSeams(*turned, topo::exactTolerance(*turned)), "");
    EXPECT_EQ(turned->vertexCount(), 3u) << "the top rim split";
    EXPECT_EQ(turned->edgeCount(), 4u);
    EXPECT_TRUE(turned->isValid()) << turned->validationReport();
    EXPECT_EQ(firstIssue(*turned), "");
    EXPECT_EQ(meshProblem(*turned), "");
    expectCarriersOnTheirCurves(*turned);
    EXPECT_NEAR(exactVolume(*turned), 2.0 * kPi, 2.0 * kPi * kExactVolumeError);

    // Its top rim two arcs, the second's vertex on the seam's line: joined there, nothing split.
    const auto arcs = bandWithATwoArcRim();
    ASSERT_NE(arcs, nullptr);
    EXPECT_EQ(hz::model::insertSeams(*arcs, topo::exactTolerance(*arcs)), "");
    EXPECT_EQ(arcs->vertexCount(), 3u) << "nothing split";
    EXPECT_EQ(arcs->edgeCount(), 4u);
    const topo::Face* arcSide = faceNamed(*arcs, "band/side");
    ASSERT_NE(arcSide, nullptr);
    ASSERT_EQ(seamsOf(*arcSide).size(), 1u);
    EXPECT_TRUE(arcs->isValid()) << arcs->validationReport();
    EXPECT_EQ(firstIssue(*arcs), "");
    EXPECT_EQ(meshProblem(*arcs), "");
    EXPECT_NEAR(exactVolume(*arcs), 2.0 * kPi, 2.0 * kPi * kExactVolumeError);
}

namespace {

/// A cylinder whose side is written as Solid Edge writes the bore of the corpus's knife holder
/// (face #665): its axis along x, its frame's x along -z, radius 1.15 from x = -3.8 to 3.8. Each
/// rim's circle faces the other way along the axis, each edge runs against its circle, and the
/// side's two loops use both rims forward. The rims' vertices are half a turn apart.
std::unique_ptr<topo::Solid> solidEdgeBand() {
    const Vec3 x{1.0, 0.0, 0.0};
    const Vec3 down{0.0, 0.0, -1.0};
    const Vec3 up{0.0, 0.0, 1.0};
    Builder b;
    topo::Edge* right = b.rim(frameAt({3.8, 2.5, -0.5}, x, up), 1.15, 0.0, false);
    topo::Edge* left = b.rim(frameAt({-3.8, 2.5, -0.5}, x * -1.0, down), 1.15, 0.0, false);
    b.face(std::make_shared<geo::Cylinder>(frameAt({-14.8812075884992, 2.5, -0.5}, x, down), 1.15),
           false, {{{right, true}}, {{left, true}}}, "bore/side");
    b.face(plane({3.8, 2.5, -0.5}, x), std::nullopt, {{{right, false}}}, "bore/right");
    b.face(plane({-3.8, 2.5, -0.5}, x), std::nullopt, {{{left, false}}}, "bore/left");
    return b.take();
}

}  // namespace

// Solid Edge writes a band's two loops the same way: each uses its rim forward, the rims' circles
// facing opposite ways. Its rims' vertices are half a turn apart, so the seam's line from the
// outer loop's vertex crosses the other rim inside it, where the rim is split: its curve runs
// backwards, from 0 to -2 pi, and the split falls at -pi.
TEST(SeamInsertion, LoopsWindingTheSameWayAreSeamed) {
    const auto solid = solidEdgeBand();
    ASSERT_NE(solid, nullptr);
    EXPECT_EQ(hz::model::insertSeams(*solid, topo::exactTolerance(*solid)), "");
    const topo::Face* side = faceNamed(*solid, "bore/side");
    ASSERT_NE(side, nullptr);
    ASSERT_EQ(seamsOf(*side).size(), 1u);
    EXPECT_TRUE(side->innerLoops.empty());
    EXPECT_EQ(solid->vertexCount(), 3u) << "one rim split";
    EXPECT_EQ(solid->edgeCount(), 4u);
    bool splitAtHalfATurn = false;
    for (const auto& e : solid->edges()) {
        if (e.exactCurve->kind() != geo::CurveKind::Circle) continue;
        EXPECT_LE(std::fabs(e.curveEnd - e.curveStart), kTwoPi + 1e-12);
        if (std::fabs(e.curveEnd + kPi) < 1e-9 || std::fabs(e.curveStart + kPi) < 1e-9) {
            splitAtHalfATurn = true;
        }
    }
    EXPECT_TRUE(splitAtHalfATurn) << "the backward rim split at -pi";
    EXPECT_TRUE(solid->isValid()) << solid->validationReport();
    EXPECT_EQ(firstIssue(*solid), "");
    EXPECT_EQ(meshProblem(*solid), "");
    expectCarriersOnTheirCurves(*solid);
    const double volume = kPi * 1.15 * 1.15 * 7.6;
    EXPECT_NEAR(exactVolume(*solid), volume, volume * kExactVolumeError);

    // The rim left whole, its carrier the whole circle, now runs its edge's way, as the split
    // rim's parts do: the faceted copy samples a closed carrier its own way, and reads the
    // seamed side as one rectangle whose sides run the way its loop does (test_StepExact's
    // ABandComesInExact measures it).
    for (const auto& e : solid->edges()) {
        if (e.halfEdge->origin != e.halfEdge->twin->origin) continue;
        const Vec3 along =
            e.exactCurve->derivatives(e.curveStart).d1 * (e.curveEnd >= e.curveStart ? 1.0 : -1.0);
        EXPECT_GT(along.dot(e.curve->derivative(e.curve->tMin(), 1)), 0.0) << e.topoId.tag();
    }
}

namespace {

/// A cylinder of radius 1 from z = 0 to 2 and, on it, a cone from radius 1 at z = 2 to 2 at
/// z = 3, as master's writer wrote the corpus's Solid Edge bore and its countersink (faces #1084
/// and #1210 of its old part body): each side a closed B-spline band between its rims, a flat at
/// each end. The rim the two bands share is used against its circle by the cylinder and with it
/// by the cone, as the solid runs it. With @p readBackwards each band uses it the other way, as a
/// part file holds such a rim where its carrier ran against it: master's writer told which way a
/// curve runs by the vertex it starts at, and a closed rim's two ends are one vertex.
std::unique_ptr<topo::Solid> boreAndCountersink(bool readBackwards) {
    Builder b;
    topo::Edge* bottom = b.rim(frameAt({0, 0, 0}, kZ), 1.0, 0.0, true);
    topo::Edge* middle = b.rim(frameAt({0, 0, 2}, kZ), 1.0, 0.0, true);
    topo::Edge* top = b.rim(frameAt({0, 0, 3}, kZ), 2.0, 0.0, true);
    b.face(std::make_shared<geo::NurbsSurface>(
               geo::NurbsSurface::makeCylinder({0, 0, 0}, kZ, 1.0, 2.0)),
           false, {{{bottom, true}}, {{middle, readBackwards}}}, "bore/side");
    b.face(std::make_shared<geo::NurbsSurface>(
               geo::NurbsSurface::makeCone({0, 0, 1}, kZ, kPi / 4.0, 2.0)),
           false, {{{middle, !readBackwards}}, {{top, false}}}, "bore/countersink");
    b.face(plane({0, 0, 0}, kZ), std::nullopt, {{{bottom, false}}}, "bore/bottom");
    b.face(plane({0, 0, 3}, kZ), std::nullopt, {{{top, true}}}, "bore/top");
    return b.take();
}

/// The closed rim of @p solid at height @p z.
const topo::Edge* rimAt(const topo::Solid& solid, double z) {
    for (const auto& e : solid.edges()) {
        const topo::HalfEdge* h = e.halfEdge;
        if (h->origin == h->twin->origin && std::fabs(h->origin->point.z - z) < 1e-12) return &e;
    }
    return nullptr;
}

}  // namespace

// The probe's cause for the corpus's Onshape and Solid Edge old part bodies ("its two loops wind
// round it the same way"): master's writer gave a closed rim's EDGE_CURVE same_sense .T. whichever
// way its carrier ran, and where the carrier ran against the rim (the source files' EDGE_CURVEs
// #101 and #314 have same_sense .F.), the rim reads back the wrong way round on both its faces.
// Between two bands, each band's two loops then wind round it the same way. That rim is turned on
// both, before any seam is laid (one turn; turning the other two rims would do too), and each
// band is then seamed. The solid is the one written right, to its volume.
TEST(SeamInsertion, ARimBothItsBandsRunTheWrongWayRoundIsTurned) {
    const auto solid = boreAndCountersink(true);
    ASSERT_NE(solid, nullptr);
    EXPECT_NE(firstIssue(*solid), "") << "neither band is exact so";
    const topo::Face* side = faceNamed(*solid, "bore/side");
    const topo::Face* countersink = faceNamed(*solid, "bore/countersink");
    ASSERT_NE(side, nullptr);
    ASSERT_NE(countersink, nullptr);
    EXPECT_EQ(hz::model::insertSeams(*solid, topo::exactTolerance(*solid)),
              "no seam can be laid on it: its two loops wind round it the same way")
        << "until its rim is turned";
    std::vector<std::string> turned;
    for (const topo::Face* f : hz::model::turnBackwardRims(*solid, topo::exactTolerance(*solid))) {
        turned.push_back(f->topoId.tag());
    }
    std::sort(turned.begin(), turned.end());
    EXPECT_EQ(turned, (std::vector<std::string>{"test/bore/countersink", "test/bore/side"}));
    const topo::Edge* middle = rimAt(*solid, 2.0);
    ASSERT_NE(middle, nullptr);
    EXPECT_EQ(middle->halfEdge->face, countersink) << "the cone runs it with its circle again";
    EXPECT_EQ(middle->halfEdge->twin->face, side);
    EXPECT_EQ(hz::model::insertSeams(*solid, topo::exactTolerance(*solid)), "");
    EXPECT_EQ(seamsOf(*side).size(), 1u);
    EXPECT_EQ(seamsOf(*countersink).size(), 1u);
    EXPECT_EQ(solid->vertexCount(), 3u) << "nothing split";
    EXPECT_TRUE(solid->isValid()) << solid->validationReport();
    EXPECT_EQ(firstIssue(*solid), "");
    EXPECT_EQ(meshProblem(*solid), "");
    expectCarriersOnTheirCurves(*solid);
    const double volume = 2.0 * kPi + 7.0 * kPi / 3.0;
    EXPECT_NEAR(exactVolume(*solid), volume, volume * kExactVolumeError);

    // Written right, nothing is turned, and it comes out the same.
    const auto right = boreAndCountersink(false);
    ASSERT_NE(right, nullptr);
    EXPECT_TRUE(hz::model::turnBackwardRims(*right, topo::exactTolerance(*right)).empty());
    EXPECT_EQ(hz::model::insertSeams(*right, topo::exactTolerance(*right)), "");
    const topo::Edge* rightMiddle = rimAt(*right, 2.0);
    ASSERT_NE(rightMiddle, nullptr);
    EXPECT_EQ(rightMiddle->halfEdge->face, faceNamed(*right, "bore/countersink"));
    EXPECT_EQ(firstIssue(*right), "");
    EXPECT_NEAR(exactVolume(*right), volume, volume * kExactVolumeError);
}

namespace {

/// A washer: radius 3 outside, a bore of radius 1 through it, from z = 0 to 2. Its flats' outer
/// rims, and its outer band's, are each two half circles (no rim alone), and its bore is a band
/// between two rims, each a hole in a flat. With @p readBackwards both the bore's rims read the
/// wrong way round, as the corpus's Solid Edge bore (its rims' EDGE_CURVEs #288 and #289,
/// same_sense .F.) reads in its old part body: the bore's two loops still wind opposite ways, and
/// each flat's hole runs the way its outer rim does.
std::unique_ptr<topo::Solid> washer(bool readBackwards) {
    Builder b;
    const auto halves = [&](double z) {
        const geo::Frame f = frameAt({0, 0, z}, kZ);
        const auto circle = std::make_shared<geo::CircleCurve>(f, 3.0);
        topo::Vertex* east = b.vertex(circle->evaluate(0.0));
        topo::Vertex* west = b.vertex(circle->evaluate(kPi));
        return std::make_pair(
            b.edge(east, west, circle, 0.0, kPi, arcCarrier(f, 3.0, 0.0, kPi)),
            b.edge(west, east, circle, kPi, kTwoPi, arcCarrier(f, 3.0, kPi, kTwoPi)));
    };
    const auto lower = halves(0.0);
    const auto upper = halves(2.0);
    topo::Edge* boreLow = b.rim(frameAt({0, 0, 0}, kZ), 1.0, 0.0, true);
    topo::Edge* boreHigh = b.rim(frameAt({0, 0, 2}, kZ), 1.0, 0.0, true);
    b.face(std::make_shared<geo::Cylinder>(frameAt({0, 0, 0}, kZ), 3.0), false,
           {{{lower.first, true}, {lower.second, true}},
            {{upper.second, false}, {upper.first, false}}},
           "washer/outside");
    b.face(std::make_shared<geo::Cylinder>(frameAt({0, 0, 0}, kZ), 1.0), true,
           {{{boreLow, readBackwards}}, {{boreHigh, !readBackwards}}}, "washer/bore");
    b.face(plane({0, 0, 0}, kZ), std::nullopt,
           {{{lower.second, false}, {lower.first, false}}, {{boreLow, !readBackwards}}},
           "washer/bottom");
    b.face(plane({0, 0, 2}, kZ), std::nullopt,
           {{{upper.first, true}, {upper.second, true}}, {{boreHigh, readBackwards}}},
           "washer/top");
    return b.take();
}

}  // namespace

// The probe's second finding in the Solid Edge old part body: a band both of whose rims read the
// wrong way round. Its loops still wind opposite ways, so it shows nothing; the faces beyond its
// rims do (there, its chamfers; here, its flats, whose holes run the way their outer rims do).
// A loop that is not a rim alone is as written, so the flats' outer rims decide: both bore rims
// are turned, before any seam is laid, and the washer is the one written right.
TEST(SeamInsertion, ABandBothOfWhoseRimsReadTheWrongWayRoundIsTurnedByTheFacesBeyond) {
    const auto solid = washer(true);
    ASSERT_NE(solid, nullptr);
    EXPECT_NE(firstIssue(*solid), "") << "its flats' holes run the wrong way round";
    const auto turned = hz::model::turnBackwardRims(*solid, topo::exactTolerance(*solid));
    std::vector<std::string> names;
    for (const topo::Face* f : turned) names.push_back(f->topoId.tag());
    std::sort(names.begin(), names.end());
    EXPECT_EQ(names, (std::vector<std::string>{"test/washer/bore", "test/washer/bottom",
                                               "test/washer/top"}));
    const topo::Edge* low = rimAt(*solid, 0.0);
    const topo::Edge* high = rimAt(*solid, 2.0);
    ASSERT_NE(low, nullptr);
    ASSERT_NE(high, nullptr);
    EXPECT_EQ(low->halfEdge->face, faceNamed(*solid, "washer/bottom")) << "as written right";
    EXPECT_EQ(high->halfEdge->face, faceNamed(*solid, "washer/bore"));
    EXPECT_EQ(hz::model::insertSeams(*solid, topo::exactTolerance(*solid)), "");
    EXPECT_TRUE(solid->isValid()) << solid->validationReport();
    EXPECT_EQ(firstIssue(*solid), "");
    EXPECT_EQ(meshProblem(*solid), "");
    const double volume = kPi * (9.0 - 1.0) * 2.0;
    EXPECT_NEAR(exactVolume(*solid), volume, volume * kExactVolumeError);

    const auto right = washer(false);
    ASSERT_NE(right, nullptr);
    EXPECT_TRUE(hz::model::turnBackwardRims(*right, topo::exactTolerance(*right)).empty());
    EXPECT_EQ(hz::model::insertSeams(*right, topo::exactTolerance(*right)), "");
    EXPECT_EQ(firstIssue(*right), "");
    EXPECT_NEAR(exactVolume(*right), volume, volume * kExactVolumeError);
}

namespace {

/// bandCylinder() with a window cut in its side round u = 0, where its rims' vertices are:
/// u from -0.3 to 0.3, v from 0.5 to 1.5. The window is a face of its own on the same cylinder,
/// so the solid is still the whole cylinder; to the band it is a hole.
std::unique_ptr<topo::Solid> bandWithAWindow() {
    const auto cylinder = std::make_shared<geo::Cylinder>(frameAt({0, 0, 0}, kZ), 1.0);
    Builder b;
    topo::Edge* bottom = b.rim(frameAt({0, 0, 0}, kZ), 1.0, 0.0, true);
    topo::Edge* top = b.rim(frameAt({0, 0, 2}, kZ), 1.0, 0.0, true);
    const auto at = [&](double u, double v) { return cylinder->evaluate(u, v); };
    topo::Vertex* p00 = b.vertex(at(-0.3, 0.5));
    topo::Vertex* p10 = b.vertex(at(0.3, 0.5));
    topo::Vertex* p11 = b.vertex(at(0.3, 1.5));
    topo::Vertex* p01 = b.vertex(at(-0.3, 1.5));
    const auto arc = [&](double z, topo::Vertex* from, topo::Vertex* to) {
        const geo::Frame f = frameAt({0, 0, z}, kZ);
        return b.edge(from, to, std::make_shared<geo::CircleCurve>(f, 1.0), kTwoPi - 0.3,
                      kTwoPi + 0.3, arcCarrier(f, 1.0, -0.3, 0.3));
    };
    const auto line = [&](topo::Vertex* from, topo::Vertex* to) {
        return b.edge(from, to,
                      std::make_shared<geo::LineCurve>(from->point, to->point - from->point), 0.0,
                      (to->point - from->point).length(), lineCarrier(from->point, to->point));
    };
    topo::Edge* lower = arc(0.5, p00, p10);
    topo::Edge* rightSide = line(p10, p11);
    topo::Edge* upper = arc(1.5, p01, p11);
    topo::Edge* leftSide = line(p01, p00);
    b.face(cylinder, false,
           {{{bottom, true}},
            {{top, false}},
            {{leftSide, false}, {upper, true}, {rightSide, false}, {lower, false}}},
           "band/side");
    b.face(cylinder, false, {{{lower, true}, {rightSide, true}, {upper, false}, {leftSide, true}}},
           "band/window");
    b.face(plane({0, 0, 0}, kZ), std::nullopt, {{{bottom, false}}}, "band/bottom");
    b.face(plane({0, 0, 2}, kZ), std::nullopt, {{{top, true}}}, "band/top");
    return b.take();
}

}  // namespace

// A band with a hole round u = 0, where the outer loop's vertex is: a seam up from it would run
// through the hole. It runs up the middle of the widest gap instead, half a turn round, and
// both rims are split there. The hole stays a hole, and the window is untouched.
TEST(SeamInsertion, ABandWithAHoleIsSeamed) {
    const auto solid = bandWithAWindow();
    ASSERT_NE(solid, nullptr);
    EXPECT_EQ(hz::model::insertSeams(*solid, topo::exactTolerance(*solid)), "");
    const topo::Face* side = faceNamed(*solid, "band/side");
    const topo::Face* window = faceNamed(*solid, "band/window");
    ASSERT_NE(side, nullptr);
    ASSERT_NE(window, nullptr);
    const auto seams = seamsOf(*side);
    ASSERT_EQ(seams.size(), 1u);
    EXPECT_EQ(side->innerLoops.size(), 1u) << "the window's hole";
    EXPECT_TRUE(seamsOf(*window).empty());
    const Vec3 foot = seams[0]->exactCurve->evaluate(seams[0]->curveStart);
    EXPECT_NEAR(std::fabs(std::atan2(foot.y, foot.x)), kPi, 1e-9) << "half a turn from the hole";
    EXPECT_EQ(solid->vertexCount(), 8u) << "both rims split";
    EXPECT_TRUE(solid->isValid()) << solid->validationReport();
    EXPECT_EQ(firstIssue(*solid), "");
    EXPECT_EQ(meshProblem(*solid), "");
    expectCarriersOnTheirCurves(*solid);
    EXPECT_NEAR(exactVolume(*solid), 2.0 * kPi, 2.0 * kPi * kExactVolumeError);
}

// A face that already has a seam, the exact primitives' cylinder's, is untouched: the same
// vertices, edges and loops, each half-edge on the same pcurve.
TEST(SeamInsertion, ASeamedFaceIsUntouched) {
    const auto solid = hz::model::exact::makeCylinder(frameAt({0, 0, 0}, kZ), 2.0, 5.0);
    ASSERT_NE(solid, nullptr);
    struct Course {
        const topo::HalfEdge* h;
        const topo::HalfEdge* next;
        const geo::Curve2D* pcurve;
        double start;
        double end;
    };
    const auto uses = [&] {
        std::vector<Course> all;
        for (const auto& f : solid->faces()) {
            const topo::HalfEdge* h = f.outerLoop->halfEdge;
            do {
                all.push_back({h, h->next, h->pcurve.get(), h->pcurveStart, h->pcurveEnd});
                h = h->next;
            } while (h != f.outerLoop->halfEdge);
        }
        return all;
    };
    const auto before = uses();
    EXPECT_EQ(hz::model::insertSeams(*solid, topo::exactTolerance(*solid)), "");
    const auto after = uses();
    ASSERT_EQ(after.size(), before.size());
    for (std::size_t k = 0; k < after.size(); ++k) {
        EXPECT_EQ(after[k].h, before[k].h);
        EXPECT_EQ(after[k].next, before[k].next);
        EXPECT_EQ(after[k].pcurve, before[k].pcurve);
        EXPECT_EQ(after[k].start, before[k].start);
        EXPECT_EQ(after[k].end, before[k].end);
    }
    EXPECT_EQ(solid->vertexCount(), 2u);
    EXPECT_EQ(solid->edgeCount(), 3u);
    EXPECT_EQ(firstIssue(*solid), "");
}

namespace {

struct Kind {
    const char* what;
    std::unique_ptr<topo::Solid> solid;
    geo::CurveKind seam;  // each seam's curve
    double volume;
    std::size_t vertices;  // after its seams: each rim split where a seam's line crosses it
};

/// A frustum of a cone, radius 2 at z = 0 to 1 at z = 3, its frame at the top facing down, as a
/// CONICAL_SURFACE grows along its axis; its top rim's vertex half a turn round.
std::unique_ptr<topo::Solid> coneBand() {
    const Vec3 down{0.0, 0.0, -1.0};
    Builder b;
    topo::Edge* top = b.rim(frameAt({0, 0, 3}, down), 1.0, kPi, true);
    topo::Edge* bottom = b.rim(frameAt({0, 0, 0}, down), 2.0, 0.0, true);
    b.face(std::make_shared<geo::Cone>(frameAt({0, 0, 3}, down), 1.0, std::atan(1.0 / 3.0)), false,
           {{{top, true}}, {{bottom, false}}}, "kind/side");
    b.face(plane({0, 0, 3}, kZ), std::nullopt, {{{top, false}}}, "kind/top");
    b.face(plane({0, 0, 0}, kZ), std::nullopt, {{{bottom, true}}}, "kind/bottom");
    return b.take();
}

/// A sphere of radius 2 between its latitudes -30 and 45 degrees; its top rim's vertex a
/// radian round.
std::unique_ptr<topo::Solid> sphereZone() {
    const double lo = -kPi / 6.0;
    const double hi = kPi / 4.0;
    Builder b;
    topo::Edge* bottom =
        b.rim(frameAt({0, 0, 2.0 * std::sin(lo)}, kZ), 2.0 * std::cos(lo), 0.0, true);
    topo::Edge* top = b.rim(frameAt({0, 0, 2.0 * std::sin(hi)}, kZ), 2.0 * std::cos(hi), 1.0, true);
    b.face(std::make_shared<geo::Sphere>(frameAt({0, 0, 0}, kZ), 2.0), false,
           {{{bottom, true}}, {{top, false}}}, "kind/side");
    b.face(plane({0, 0, 2.0 * std::sin(lo)}, kZ), std::nullopt, {{{bottom, false}}}, "kind/bottom");
    b.face(plane({0, 0, 2.0 * std::sin(hi)}, kZ), std::nullopt, {{{top, true}}}, "kind/top");
    return b.take();
}

/// The outer half of a torus of radii 4 and 1, between the circles of radius 4 round its top and
/// its bottom, and the cylinder of radius 4 inside it between the same circles: two bands, on the
/// same two rims. The torus is periodic both ways, so its seam could run round through its
/// inner equator or its outer; the face, not reversed, is the outer half. The top rim's vertex is
/// half a radian round from the bottom's.
std::unique_ptr<topo::Solid> torusRing() {
    Builder b;
    topo::Edge* bottom = b.rim(frameAt({0, 0, -1}, kZ), 4.0, 0.0, true);
    topo::Edge* top = b.rim(frameAt({0, 0, 1}, kZ), 4.0, 0.5, true);
    b.face(std::make_shared<geo::Torus>(frameAt({0, 0, 0}, kZ), 4.0, 1.0), false,
           {{{bottom, true}}, {{top, false}}}, "kind/side");
    b.face(std::make_shared<geo::Cylinder>(frameAt({0, 0, 0}, kZ), 4.0), true,
           {{{top, true}}, {{bottom, false}}}, "kind/inside");
    return b.take();
}

/// Half of a torus of radii 4 and 1, y >= 0, between two of its meridians: a pipe's elbow. Its
/// loops wind round the tube, in v, so its seam is a parallel; the u = pi meridian's vertex is a
/// radian round the tube from the other's.
std::unique_ptr<topo::Solid> torusElbow() {
    const Vec3 y{0.0, 1.0, 0.0};
    const Vec3 x{1.0, 0.0, 0.0};
    Builder b;
    // Each meridian's circle turns as v does: from the outer equator towards +z.
    topo::Edge* near = b.rim(frameAt({4, 0, 0}, y * -1.0, x), 1.0, 0.0, false);
    topo::Edge* far = b.rim(frameAt({-4, 0, 0}, y, x * -1.0), 1.0, 1.0, true);
    b.face(std::make_shared<geo::Torus>(frameAt({0, 0, 0}, kZ), 4.0, 1.0), false,
           {{{near, true}}, {{far, true}}}, "kind/side");
    b.face(plane({4, 0, 0}, y), std::nullopt, {{{near, false}}}, "kind/near");
    b.face(plane({-4, 0, 0}, y), std::nullopt, {{{far, false}}}, "kind/far");
    return b.take();
}

/// A cylinder of radius 1 and height 2 whose side is a B-spline closed in u (a rational
/// quadratic round, linear up); its rims' vertices at 0.7 and 2 radians, so its seam's
/// iso-curve is the surface's inside its knot span, not an edge of its net.
std::unique_ptr<topo::Solid> splineBand() {
    Builder b;
    topo::Edge* bottom = b.rim(frameAt({0, 0, 0}, kZ), 1.0, 0.7, true);
    topo::Edge* top = b.rim(frameAt({0, 0, 2}, kZ), 1.0, 2.0, true);
    b.face(std::make_shared<geo::NurbsSurface>(
               geo::NurbsSurface::makeCylinder({0, 0, 0}, kZ, 1.0, 2.0)),
           false, {{{bottom, true}}, {{top, false}}}, "kind/side");
    b.face(plane({0, 0, 0}, kZ), std::nullopt, {{{bottom, false}}}, "kind/bottom");
    b.face(plane({0, 0, 2}, kZ), std::nullopt, {{{top, true}}}, "kind/top");
    return b.take();
}

}  // namespace

// Every surface periodic in u, and the torus in v too, is given a seam along its own iso-curve:
// a cone's generator, a sphere's meridian, a torus's meridian or, round its tube, its parallel,
// and a B-spline's exact iso-curve. Each comes out exact, meshes, and measures its volume.
TEST(SeamInsertion, EachPeriodicSurfaceIsSeamedAlongItsOwnIsoCurve) {
    std::vector<Kind> kinds;
    kinds.push_back({"a cone's frustum", coneBand(), geo::CurveKind::Line, 7.0 * kPi, 3});
    const double z0 = 2.0 * std::sin(-kPi / 6.0);
    const double z1 = 2.0 * std::sin(kPi / 4.0);
    kinds.push_back({"a sphere's zone", sphereZone(), geo::CurveKind::Circle,
                     kPi * (4.0 * (z1 - z0) - (z1 * z1 * z1 - z0 * z0 * z0) / 3.0), 3});
    kinds.push_back({"a torus's outer half, and the cylinder inside it", torusRing(),
                     geo::CurveKind::Circle, kPi * kPi * 4.0 + 4.0 * kPi / 3.0, 4});
    kinds.push_back({"a torus's elbow", torusElbow(), geo::CurveKind::Circle, kPi * kPi * 4.0, 3});
    kinds.push_back({"a B-spline closed in u", splineBand(), geo::CurveKind::Nurbs, 2.0 * kPi, 3});
    for (Kind& k : kinds) {
        SCOPED_TRACE(k.what);
        ASSERT_NE(k.solid, nullptr);
        topo::Solid& solid = *k.solid;
        EXPECT_EQ(hz::model::insertSeams(solid, topo::exactTolerance(solid)), "");
        const topo::Face* side = faceNamed(solid, "kind/side");
        ASSERT_NE(side, nullptr);
        const auto seams = seamsOf(*side);
        ASSERT_EQ(seams.size(), 1u);
        EXPECT_EQ(seams[0]->exactCurve->kind(), k.seam);
        EXPECT_TRUE(side->innerLoops.empty());
        EXPECT_EQ(solid.vertexCount(), k.vertices);
        EXPECT_TRUE(solid.isValid()) << solid.validationReport();
        EXPECT_EQ(firstIssue(solid), "");
        EXPECT_EQ(meshProblem(solid), "");
        expectCarriersOnTheirCurves(solid);
        EXPECT_NEAR(exactVolume(solid), k.volume, k.volume * kExactVolumeError);
    }
    // The torus ring's seam runs round through the outer equator, radius 5, where its face is,
    // not the inner; its cylinder is a band too, on the same rims, with a seam of its own.
    const auto ring = torusRing();
    ASSERT_NE(ring, nullptr);
    EXPECT_EQ(hz::model::insertSeams(*ring, topo::exactTolerance(*ring)), "");
    const topo::Face* outside = faceNamed(*ring, "kind/side");
    const topo::Face* inside = faceNamed(*ring, "kind/inside");
    ASSERT_NE(outside, nullptr);
    ASSERT_NE(inside, nullptr);
    ASSERT_EQ(seamsOf(*outside).size(), 1u);
    const topo::Edge& meridian = *seamsOf(*outside)[0];
    const Vec3 middle =
        meridian.exactCurve->evaluate((meridian.curveStart + meridian.curveEnd) / 2.0);
    EXPECT_NEAR(std::hypot(middle.x, middle.y), 5.0, 1e-9);
    ASSERT_EQ(seamsOf(*inside).size(), 1u);
    EXPECT_EQ(seamsOf(*inside)[0]->exactCurve->kind(), geo::CurveKind::Line);
}

namespace {

/// A torus of radii 4 and 1 cut along two of its Villarceau circles (radius 4, each in a plane
/// tangent to it twice) of one family, half a turn apart about its axis: two bands that each
/// wind round it once in u and once in v, a helix between the two circles.
std::unique_ptr<topo::Solid> helicalTorusBands() {
    const double sinA = 0.25;  // r / R
    const double cosA = std::sqrt(1.0 - sinA * sinA);
    const auto torus = std::make_shared<geo::Torus>(frameAt({0, 0, 0}, kZ), 4.0, 1.0);
    Builder b;
    const Vec3 across1{cosA, 0.0, sinA};
    const Vec3 across2{-cosA, 0.0, sinA};
    topo::Edge* one = b.rim(frameAt({0, 1, 0}, across1.cross({0, 1, 0}), across1), 4.0, 0.0, true);
    topo::Edge* two =
        b.rim(frameAt({0, -1, 0}, across2.cross({0, -1, 0}), across2), 4.0, 0.0, true);
    b.face(torus, false, {{{one, true}}, {{two, false}}}, "helix/one");
    b.face(torus, false, {{{one, false}}, {{two, true}}}, "helix/two");
    return b.take();
}

}  // namespace

// Ruling 29's class, as stage 3 rules it: a band whose loops wind round it both ways (a helical
// band on a torus) gets both seams. A seam along u alone leaves its region over more than a
// period in v, which no face's may be (the validator's one-period window, a guard on imported
// data): its two loops each run a whole period both ways, one beside the other. So it is seamed
// along u and then cut in two across, along the parallel that halves its span in v (seam:1), as
// topo::euler::makeEdgeFace splits a face. Each piece spans less than a period each way. The two
// bands are the whole torus of radii 4 and 1: 2 pi^2 R r^2 = 8 pi^2 inside, 4 pi^2 R r = 16 pi^2
// round it, each band half of that.
TEST(SeamInsertion, ATorusBandWindingBothWaysGetsTwoSeams) {
    const auto solid = helicalTorusBands();
    ASSERT_NE(solid, nullptr);
    EXPECT_NE(firstIssue(*solid), "") << "no band is exact without its seams";
    EXPECT_EQ(hz::model::insertSeams(*solid, topo::exactTolerance(*solid)), "");

    std::vector<std::string> faces;
    for (const auto& f : solid->faces()) {
        if (f.alive) faces.push_back(f.topoId.tag());
        EXPECT_TRUE(seamsOf(f).empty()) << f.topoId.tag() << ": its seams lie between its pieces";
    }
    std::sort(faces.begin(), faces.end());
    EXPECT_EQ(faces,
              (std::vector<std::string>{"test/helix/one/piece:0", "test/helix/one/piece:1",
                                        "test/helix/two/piece:0", "test/helix/two/piece:1"}));
    for (const std::string band : {"one", "two"}) {
        for (const int k : {0, 1}) {
            const std::string name = "test/helix/" + band + "/seam:" + std::to_string(k);
            const topo::Edge* seam = nullptr;
            for (const auto& e : solid->edges()) {
                if (e.topoId.tag() == name) seam = &e;
            }
            ASSERT_NE(seam, nullptr) << name;
            EXPECT_EQ(seam->exactCurve->kind(), geo::CurveKind::Circle) << name;
            const std::string a = seam->halfEdge->face->topoId.tag();
            const std::string b = seam->halfEdge->twin->face->topoId.tag();
            EXPECT_NE(a, b) << name;
            EXPECT_EQ(a.rfind("test/helix/" + band + "/piece:", 0), 0u) << name << " on " << a;
            EXPECT_EQ(b.rfind("test/helix/" + band + "/piece:", 0), 0u) << name << " on " << b;
        }
    }
    EXPECT_TRUE(solid->isValid()) << solid->validationReport();
    EXPECT_EQ(firstIssue(*solid), "");
    EXPECT_EQ(meshProblem(*solid), "");
    expectCarriersOnTheirCurves(*solid, true);

    const auto props = hz::model::MassPropertiesCalculator::compute(*solid);
    ASSERT_TRUE(props.valid) << props.problem;
    const double volume = 2.0 * kPi * kPi * 4.0 * 1.0;
    const double area = 4.0 * kPi * kPi * 4.0 * 1.0;
    EXPECT_NEAR(props.volume, volume, volume * kExactVolumeError);
    EXPECT_NEAR(props.surfaceArea, area, area * kExactVolumeError);
    EXPECT_NEAR(props.centerOfMass.length(), 0.0, 1e-9);

    // Each band is half the torus: its pieces' triangles, at a chord of 1e-3.
    std::string problem;
    const auto mesh = hz::model::BoundaryMesh::extractFacePolygons(*solid, 1e-3, &problem);
    ASSERT_EQ(problem, "");
    double one = 0.0;
    double two = 0.0;
    for (const auto& t : mesh) {
        if (t.points.size() != 3) continue;
        const double a =
            (t.points[1] - t.points[0]).cross(t.points[2] - t.points[0]).length() / 2.0;
        (t.topoId.tag().rfind("test/helix/one/", 0) == 0 ? one : two) += a;
    }
    EXPECT_NEAR(one, area / 2.0, area * 1e-3);
    EXPECT_NEAR(two, area / 2.0, area * 1e-3);
}

// The parts of a rim a seam's line splits are pieces of it, `<rim>/piece:0` and `/piece:1`: a
// reference to the rim, as a feature keeps it, has no exact match left and takes both (FilletOp
// and ChamferOp take an exact match alone where there is one).
TEST(SeamInsertion, ASplitRimsPartsAreItsPieces) {
    const auto solid = bandCylinder(2.0);
    ASSERT_NE(solid, nullptr);
    std::string rim;
    for (const auto& e : solid->edges()) {
        if (std::abs(e.halfEdge->origin->point.z - 2.0) < 1e-12) rim = e.topoId.tag();
    }
    ASSERT_FALSE(rim.empty());
    EXPECT_EQ(hz::model::insertSeams(*solid, topo::exactTolerance(*solid)), "");
    const auto whole = topo::TopologyID::fromTag(rim);
    std::vector<std::string> pieces;
    for (const auto& e : solid->edges()) {
        EXPECT_NE(e.topoId, whole) << "no part keeps the whole rim's name";
        if (e.topoId.isDescendantOf(whole)) pieces.push_back(e.topoId.tag());
    }
    std::sort(pieces.begin(), pieces.end());
    EXPECT_EQ(pieces, (std::vector<std::string>{rim + "/piece:0", rim + "/piece:1"}));
}

// A face no seam can join says why, and is left as it was: a sphere's cap bounded by its rim
// alone (one loop round it, and no pole to run a seam to), and a band whose two loops run round
// the same way in (u, v), which no seam closes, where turnBackwardRims cannot tell which of its
// rims reads the wrong way round.
TEST(SeamInsertion, AFaceNoSeamCanJoinSaysWhy) {
    Builder cap;
    topo::Edge* rim = cap.rim(frameAt({0, 0, 1}, kZ), std::sqrt(3.0), 0.0, true);
    cap.face(std::make_shared<geo::Sphere>(frameAt({0, 0, 0}, kZ), 2.0), false, {{{rim, false}}},
             "cap/dome");
    cap.face(plane({0, 0, 1}, kZ), std::nullopt, {{{rim, true}}}, "cap/flat");
    const auto dome = cap.take();
    ASSERT_NE(dome, nullptr);
    const std::string why = hz::model::insertSeams(*dome, topo::exactTolerance(*dome));
    EXPECT_NE(why.find("one of its loops winds round it"), std::string::npos) << why;
    EXPECT_EQ(dome->edgeCount(), 1u);

    Builder same;
    topo::Edge* bottom = same.rim(frameAt({0, 0, 0}, kZ), 1.0, 0.0, true);
    topo::Edge* top = same.rim(frameAt({0, 0, 2}, kZ), 1.0, 0.0, true);
    same.face(std::make_shared<geo::Cylinder>(frameAt({0, 0, 0}, kZ), 1.0), false,
              {{{bottom, true}}, {{top, true}}}, "same/side");
    same.face(plane({0, 0, 0}, kZ), std::nullopt, {{{bottom, false}}}, "same/bottom");
    same.face(plane({0, 0, 2}, kZ), std::nullopt, {{{top, false}}}, "same/top");
    const auto band = same.take();
    ASSERT_NE(band, nullptr);
    EXPECT_TRUE(hz::model::turnBackwardRims(*band, topo::exactTolerance(*band)).empty())
        << "either rim could be the one read the wrong way round: the flats beyond them, each "
           "a rim alone, say nothing";
    const std::string sameWay = hz::model::insertSeams(*band, topo::exactTolerance(*band));
    EXPECT_NE(sameWay.find("the same way"), std::string::npos) << sameWay;
    EXPECT_EQ(band->edgeCount(), 2u);

    // Three bands stacked, each written right using its lower rim with its circle and its upper
    // against it, but for the rims at z = 1 and z = 3, each read the wrong way round on both its
    // faces. Each band's two loops wind the same way. Turning the rims at z = 0 and 2 would
    // answer as well as turning those at 1 and 3, so none is turned, and each band is refused.
    Builder stack;
    std::vector<topo::Edge*> rims;
    for (int k = 0; k < 4; ++k) {
        rims.push_back(stack.rim(frameAt({0, 0, k * 1.0}, kZ), 1.0, 0.0, true));
    }
    const auto cylinder = std::make_shared<geo::Cylinder>(frameAt({0, 0, 0}, kZ), 1.0);
    stack.face(cylinder, false, {{{rims[0], true}}, {{rims[1], true}}}, "stack/lower");
    stack.face(cylinder, false, {{{rims[1], false}}, {{rims[2], false}}}, "stack/middle");
    stack.face(cylinder, false, {{{rims[2], true}}, {{rims[3], true}}}, "stack/upper");
    stack.face(plane({0, 0, 0}, kZ), std::nullopt, {{{rims[0], false}}}, "stack/bottom");
    stack.face(plane({0, 0, 3}, kZ), std::nullopt, {{{rims[3], false}}}, "stack/top");
    const auto stacked = stack.take();
    ASSERT_NE(stacked, nullptr);
    EXPECT_TRUE(hz::model::turnBackwardRims(*stacked, topo::exactTolerance(*stacked)).empty());
    const std::string either = hz::model::insertSeams(*stacked, topo::exactTolerance(*stacked));
    EXPECT_NE(either.find("the same way"), std::string::npos) << either;
    EXPECT_EQ(stacked->edgeCount(), 4u);
    const topo::Edge* second = rimAt(*stacked, 1.0);
    ASSERT_NE(second, nullptr);
    EXPECT_EQ(second->halfEdge->face, faceNamed(*stacked, "stack/lower")) << "nothing turned";
}
