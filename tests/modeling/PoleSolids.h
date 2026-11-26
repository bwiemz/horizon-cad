#pragma once

// Solids whose faces meet at a pole or an apex between two straight runs of their surface's
// parameters, as a partial revolve makes them and as other systems write them: the class the
// spec's regression list names "a ball split into two faces by two meridians". Shared by the
// exact primitives', mesher's and mass's tests.

#include <cmath>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "horizon/geometry/Frame.h"
#include "horizon/geometry/curves/AnalyticCurves.h"
#include "horizon/geometry/curves/Arc2D.h"
#include "horizon/geometry/curves/Line2D.h"
#include "horizon/geometry/surfaces/AnalyticSurfaces.h"
#include "horizon/math/Constants.h"
#include "horizon/math/Vec2.h"
#include "horizon/math/Vec3.h"
#include "horizon/modeling/ExactPrimitives.h"
#include "horizon/topology/Solid.h"
#include "horizon/topology/TopologyID.h"

namespace hz::pole_solids {

/// A solid put together by hand: vertices, faces with loops of half-edges, and edges pairing
/// them, every pcurve straight or an arc over [0, 1].
class Assembly {
public:
    Assembly() : solid_(std::make_unique<topo::Solid>()), shell_(solid_->allocShell()) {
        shell_->solid = solid_.get();
    }

    topo::Vertex* vertex(const math::Vec3& p) {
        topo::Vertex* v = solid_->allocVertex();
        v->point = p;
        return v;
    }

    topo::Face* face(const std::string& tag, std::shared_ptr<const geo::Surface> surface) {
        topo::Face* f = solid_->allocFace();
        f->topoId = topo::TopologyID::fromTag(tag);
        f->exactSurface = std::move(surface);
        f->shell = shell_;
        shell_->faces.push_back(f);
        return f;
    }

    /// A half-edge from @p origin along @p pcurve (over [0, 1]).
    topo::HalfEdge* halfEdge(topo::Vertex* origin, std::shared_ptr<const geo::Curve2D> pcurve) {
        topo::HalfEdge* h = solid_->allocHalfEdge();
        h->origin = origin;
        if (origin->halfEdge == nullptr) origin->halfEdge = h;
        h->pcurve = std::move(pcurve);
        h->pcurveStart = 0.0;
        h->pcurveEnd = 1.0;
        return h;
    }

    topo::HalfEdge* line(topo::Vertex* origin, const math::Vec2& from, const math::Vec2& to) {
        return halfEdge(origin, std::make_shared<geo::Line2D>(from, to));
    }

    /// Closes @p hes, in order, into the outer loop of @p f.
    void loop(topo::Face* f, const std::vector<topo::HalfEdge*>& hes) {
        topo::Wire* w = solid_->allocWire();
        w->halfEdge = hes.front();
        for (std::size_t i = 0; i < hes.size(); ++i) {
            hes[i]->face = f;
            hes[i]->next = hes[(i + 1) % hes.size()];
            hes[i]->next->prev = hes[i];
        }
        f->outerLoop = w;
    }

    /// Pairs @p a and @p b along @p curve, which runs from a's origin at @p start to b's at
    /// @p end.
    void edge(topo::HalfEdge* a, topo::HalfEdge* b, std::shared_ptr<const geo::Curve3> curve,
              double start, double end) {
        topo::Edge* e = solid_->allocEdge();
        e->halfEdge = a;
        a->twin = b;
        b->twin = a;
        a->edge = b->edge = e;
        e->exactCurve = std::move(curve);
        e->curveStart = start;
        e->curveEnd = end;
    }

    /// A straight edge from @p a's origin to @p b's.
    void straight(topo::HalfEdge* a, topo::HalfEdge* b) {
        const math::Vec3 d = b->origin->point - a->origin->point;
        edge(a, b, std::make_shared<geo::LineCurve>(a->origin->point, d), 0.0, d.length());
    }

    std::unique_ptr<topo::Solid> take() { return std::move(solid_); }

private:
    std::unique_ptr<topo::Solid> solid_;
    topo::Shell* shell_;
};

/// The meridian of a sphere of radius @p r about @p base's origin at longitude @p u: from the
/// south pole at t = 0 to the north at t = pi.
inline std::shared_ptr<geo::CircleCurve> meridian(const geo::Frame& base, double r, double u) {
    geo::Frame f;
    f.origin = base.origin;
    f.x = base.z * -1.0;
    f.y = base.x * std::cos(u) + base.y * std::sin(u);
    f.z = f.x.cross(f.y);
    return std::make_shared<geo::CircleCurve>(f, r);
}

/// model::exact::makeSphereSector(base, r, angle) with its lune's sphere placed as another
/// system may place it: its seam (u = 0) in the middle of the lune, its x turned by angle / 2,
/// and its meridians' pcurves written within [0, 2 pi): the meridian at angle at u = angle / 2,
/// the one at 0 at u = 2 pi - angle / 2. Straight across, its pole runs go round the poles the
/// wrong way: only the face's sense says which way they go.
inline std::unique_ptr<topo::Solid> luneAcrossTheSeam(const geo::Frame& frame, double r,
                                                      double angle) {
    auto sector = model::exact::makeSphereSector(frame, r, angle);
    const geo::Frame base = geo::Frame::fromAxis(frame.origin, frame.z, frame.x);
    geo::Frame turned = base;
    turned.x = base.x * std::cos(angle / 2.0) + base.y * std::sin(angle / 2.0);
    turned.y = base.y * std::cos(angle / 2.0) - base.x * std::sin(angle / 2.0);
    for (auto& f : sector->faces()) {
        if (f.topoId.tag() != "sphere/band") continue;
        f.exactSurface = std::make_shared<geo::Sphere>(turned, r);
        topo::HalfEdge* up = f.outerLoop->halfEdge;  // the meridian at angle, south to north
        topo::HalfEdge* down = up->next;             // the meridian at 0, north to south
        const double half = 0.5 * math::kPi;
        const double hi = angle / 2.0;
        const double lo = math::kTwoPi - angle / 2.0;
        up->pcurve = std::make_shared<geo::Line2D>(math::Vec2{hi, -half}, math::Vec2{hi, half});
        down->pcurve = std::make_shared<geo::Line2D>(math::Vec2{lo, half}, math::Vec2{lo, -half});
    }
    return sector;
}

/// A ball of radius @p r about @p frame's origin split into two faces by its meridians at u = 0
/// and u = @p split: "sphere/a" from 0 to split, "sphere/b" from split round to 2 pi, both on
/// one sphere, every pcurve written within [0, 2 pi).
inline std::unique_ptr<topo::Solid> ballSplitByMeridians(const geo::Frame& frame, double r,
                                                         double split) {
    const geo::Frame base = geo::Frame::fromAxis(frame.origin, frame.z, frame.x);
    const double half = 0.5 * math::kPi;
    Assembly as;
    topo::Vertex* south = as.vertex(base.origin - base.z * r);
    topo::Vertex* north = as.vertex(base.origin + base.z * r);
    const auto sphere = std::make_shared<geo::Sphere>(base, r);
    topo::Face* a = as.face("sphere/a", sphere);
    topo::Face* b = as.face("sphere/b", sphere);
    // Each counter-clockwise in (u, v): up its later meridian, down its earlier one.
    topo::HalfEdge* aUp = as.line(south, {split, -half}, {split, half});
    topo::HalfEdge* aDown = as.line(north, {0.0, half}, {0.0, -half});
    as.loop(a, {aUp, aDown});
    topo::HalfEdge* bUp = as.line(south, {0.0, -half}, {0.0, half});
    topo::HalfEdge* bDown = as.line(north, {split, half}, {split, -half});
    as.loop(b, {bUp, bDown});
    as.edge(aUp, bDown, meridian(base, r, split), 0.0, math::kPi);
    as.edge(bUp, aDown, meridian(base, r, 0.0), 0.0, math::kPi);
    return as.take();
}

/// The sector a partial revolve of a right triangle makes: a cone from its apex at @p frame's
/// origin, opening up its z to a rim of radius @p r at height @p h, between the half-planes at
/// u = 0 and u = @p angle; a flat top ("cone/top", a sector of a disc) and two triangular caps
/// ("cone/cap_start", "cone/cap_end") meeting along the axis. Its side ("cone/side") comes to the
/// apex between its two generators, its loop running along the apex from one to the other.
inline std::unique_ptr<topo::Solid> coneSector(const geo::Frame& frame, double r, double h,
                                               double angle) {
    const geo::Frame base = geo::Frame::fromAxis(frame.origin, frame.z, frame.x);
    const math::Vec3 radial = base.x * std::cos(angle) + base.y * std::sin(angle);
    const math::Vec3 tangent = base.y * std::cos(angle) - base.x * std::sin(angle);
    const math::Vec3 top = base.origin + base.z * h;
    Assembly as;
    topo::Vertex* apex = as.vertex(base.origin);
    topo::Vertex* centre = as.vertex(top);
    topo::Vertex* rim0 = as.vertex(top + base.x * r);
    topo::Vertex* rimA = as.vertex(top + radial * r);

    // The side, counter-clockwise in (u, v): up the generator at angle, back along the rim, down
    // the generator at 0, and along the apex (v = 0) back.
    topo::Face* side =
        as.face("cone/side", std::make_shared<geo::Cone>(base, 0.0, std::atan2(r, h)));
    topo::HalfEdge* genUp = as.line(apex, {angle, 0.0}, {angle, h});
    topo::HalfEdge* rimBack = as.line(rimA, {angle, h}, {0.0, h});
    topo::HalfEdge* genDown = as.line(rim0, {0.0, h}, {0.0, 0.0});
    as.loop(side, {genUp, rimBack, genDown});

    // The top, in its own (x, y) about its centre: out along x, round the rim, back in.
    topo::Face* cap = as.face(
        "cone/top", std::make_shared<geo::Plane>(geo::Frame::fromAxis(top, base.z, base.x)));
    topo::HalfEdge* out = as.line(centre, {0.0, 0.0}, {r, 0.0});
    topo::HalfEdge* arc =
        as.halfEdge(rim0, std::make_shared<geo::Arc2D>(math::Vec2{0.0, 0.0}, r, 0.0, angle));
    topo::HalfEdge* in = as.line(rimA, {r * std::cos(angle), r * std::sin(angle)}, {0.0, 0.0});
    as.loop(cap, {out, arc, in});

    // The caps: the start's (u, v) is (distance from the axis, height), the end's (distance from
    // the axis, depth); each triangle counter-clockwise in it.
    topo::Face* start = as.face("cone/cap_start", std::make_shared<geo::Plane>(geo::Frame::fromAxis(
                                                      base.origin, base.y * -1.0, base.x)));
    topo::HalfEdge* s1 = as.line(apex, {0.0, 0.0}, {r, h});
    topo::HalfEdge* s2 = as.line(rim0, {r, h}, {0.0, h});
    topo::HalfEdge* s3 = as.line(centre, {0.0, h}, {0.0, 0.0});
    as.loop(start, {s1, s2, s3});
    topo::Face* end =
        as.face("cone/cap_end",
                std::make_shared<geo::Plane>(geo::Frame::fromAxis(base.origin, tangent, radial)));
    topo::HalfEdge* e1 = as.line(apex, {0.0, 0.0}, {0.0, -h});
    topo::HalfEdge* e2 = as.line(centre, {0.0, -h}, {r, -h});
    topo::HalfEdge* e3 = as.line(rimA, {r, -h}, {0.0, 0.0});
    as.loop(end, {e1, e2, e3});

    as.straight(s1, genDown);
    as.straight(genUp, e3);
    geo::Frame rimFrame = base;
    rimFrame.origin = top;
    as.edge(arc, rimBack, std::make_shared<geo::CircleCurve>(rimFrame, r), 0.0, angle);
    as.straight(out, s2);
    as.straight(e2, in);
    as.straight(e1, s3);
    return as.take();
}

}  // namespace hz::pole_solids
