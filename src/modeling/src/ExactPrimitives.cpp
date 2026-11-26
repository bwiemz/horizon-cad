#include "horizon/modeling/ExactPrimitives.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "horizon/geometry/curves/AnalyticCurves.h"
#include "horizon/geometry/curves/Arc2D.h"
#include "horizon/geometry/curves/Circle2D.h"
#include "horizon/geometry/curves/Line2D.h"
#include "horizon/geometry/curves/NurbsCurve.h"
#include "horizon/geometry/surfaces/AnalyticSurfaces.h"
#include "horizon/geometry/surfaces/NurbsSurface.h"
#include "horizon/math/Constants.h"
#include "horizon/math/ModelTolerance.h"
#include "horizon/topology/TopologyID.h"

namespace hz::model::exact {

using math::Vec2;
using math::Vec3;
using topo::Edge;
using topo::Face;
using topo::HalfEdge;
using topo::Solid;
using topo::TopologyID;
using topo::Vertex;

namespace {

struct Builder {
    Solid& solid;
    topo::Shell* shell;

    explicit Builder(Solid& s) : solid(s), shell(s.allocShell()) { shell->solid = &s; }

    Vertex* vertex(const Vec3& p) {
        Vertex* v = solid.allocVertex();
        v->point = p;
        return v;
    }

    Face* face(const std::string& source, const std::string& role,
               std::shared_ptr<const geo::Surface> surface, bool reversed = false) {
        Face* f = solid.allocFace();
        f->topoId = TopologyID::make(source, role);
        f->exactSurface = std::move(surface);
        f->reversed = reversed;
        f->shell = shell;
        shell->faces.push_back(f);
        return f;
    }

    HalfEdge* halfEdge(Vertex* origin) {
        HalfEdge* he = solid.allocHalfEdge();
        he->origin = origin;
        if (origin->halfEdge == nullptr) origin->halfEdge = he;
        return he;
    }

    /// Closes @p hes into a loop of @p f: the outer loop, or one more inner loop.
    void loop(Face* f, const std::vector<HalfEdge*>& hes, bool outer) {
        topo::Wire* w = solid.allocWire();
        w->halfEdge = hes.front();
        for (size_t i = 0; i < hes.size(); ++i) {
            HalfEdge* he = hes[i];
            he->face = f;
            he->next = hes[(i + 1) % hes.size()];
            he->next->prev = he;
        }
        if (outer) {
            f->outerLoop = w;
        } else {
            f->innerLoops.push_back(w);
        }
    }

    /// Joins @p a and @p b as the two sides of a new edge along @p curve, which runs from a's
    /// origin at @p start to b's origin at @p end.
    Edge* edge(HalfEdge* a, HalfEdge* b, std::shared_ptr<const geo::Curve3> curve, double start,
               double end) {
        Edge* e = solid.allocEdge();
        e->halfEdge = a;
        a->twin = b;
        b->twin = a;
        a->edge = b->edge = e;
        e->exactCurve = std::move(curve);
        e->curveStart = start;
        e->curveEnd = end;
        return e;
    }
};

void setPcurve(HalfEdge* he, std::shared_ptr<const geo::Curve2D> curve, double start, double end) {
    he->pcurve = std::move(curve);
    he->pcurveStart = start;
    he->pcurveEnd = end;
}

/// A straight pcurve from (u0, v0) to (u1, v1), parametrised over [0, 1].
void setLinePcurve(HalfEdge* he, const Vec2& from, const Vec2& to) {
    setPcurve(he, std::make_shared<geo::Line2D>(from, to), 0.0, 1.0);
}

bool positiveFinite(double x) {
    return x > 0.0 && std::isfinite(x);
}

/// The corners of the axis-aligned block from @p min to @p max, numbered as buildBlock wants.
std::array<Vec3, 8> blockCorners(const Vec3& min, const Vec3& max) {
    return {{{min.x, min.y, min.z},
             {max.x, min.y, min.z},
             {max.x, max.y, min.z},
             {min.x, max.y, min.z},
             {min.x, min.y, max.z},
             {max.x, min.y, max.z},
             {max.x, max.y, max.z},
             {min.x, max.y, max.z}}};
}

/// The pieces of the block that a hole through it needs.
struct Block {
    Builder builder;
    Face* bottom = nullptr;
    Face* top = nullptr;
};

/// Builds the six planar faces of the block whose corners are @p pts: the bottom four (z = 0)
/// counter-clockwise about @p axes' z from the origin's corner, then the top four above them, as
/// PrimitiveFactory::makeBox numbers them; @p axes' x, y and z run along the block's sides. Every
/// face's frame has z along its outward normal, so no face is reversed, and each outer loop runs
/// counter-clockwise seen from outside, which is counter-clockwise in the face's (u, v). Where
/// @p axes is left-handed the corners as numbered run clockwise, and each loop is reversed.
Block buildBlock(Solid& solid, const std::array<Vec3, 8>& pts, const geo::Frame& axes) {
    Block b{Builder(solid), nullptr, nullptr};
    Builder& bd = b.builder;
    const bool rightHanded = axes.x.cross(axes.y).dot(axes.z) > 0.0;

    std::array<Vertex*, 8> v{};
    for (size_t i = 0; i < 8; ++i) v[i] = bd.vertex(pts[i]);

    struct Side {
        const char* role;
        std::array<int, 4> corners;  // counter-clockwise seen from outside, right-handed
        Vec3 normal;
    };
    const std::array<Side, 6> sides = {{{"bottom", {0, 3, 2, 1}, axes.z * -1.0},
                                        {"top", {4, 5, 6, 7}, axes.z},
                                        {"front", {0, 1, 5, 4}, axes.y * -1.0},
                                        {"back", {2, 3, 7, 6}, axes.y},
                                        {"right", {1, 2, 6, 5}, axes.x},
                                        {"left", {3, 0, 4, 7}, axes.x * -1.0}}};

    std::map<std::pair<Vertex*, Vertex*>, HalfEdge*> open;
    for (const Side& s : sides) {
        std::array<int, 4> corners = s.corners;
        if (!rightHanded) std::reverse(corners.begin(), corners.end());
        const geo::Frame frame =
            geo::Frame::fromAxis(pts[static_cast<size_t>(corners[0])], s.normal, axes.x);
        const auto plane = std::make_shared<geo::Plane>(frame);
        Face* f = bd.face("box", s.role, plane);
        std::vector<HalfEdge*> hes;
        hes.reserve(4);
        for (size_t i = 0; i < 4; ++i) {
            hes.push_back(bd.halfEdge(v[static_cast<size_t>(corners[i])]));
        }
        bd.loop(f, hes, true);
        for (size_t i = 0; i < 4; ++i) {
            const Vec3& a = hes[i]->origin->point;
            const Vec3& c = hes[i]->next->origin->point;
            setLinePcurve(hes[i], plane->invert(a), plane->invert(c));
            const auto twin = open.find({hes[i]->next->origin, hes[i]->origin});
            if (twin == open.end()) {
                open[{hes[i]->origin, hes[i]->next->origin}] = hes[i];
                continue;
            }
            // The edge's curve starts at the half-edge that was built first.
            const Vec3 d = c - a;
            bd.edge(twin->second, hes[i], std::make_shared<geo::LineCurve>(c, d / (-d.length())),
                    0.0, d.length());
            open.erase(twin);
        }
        if (std::string(s.role) == "bottom") {
            b.bottom = f;
        } else if (std::string(s.role) == "top") {
            b.top = f;
        }
    }
    return b;
}

/// The frame of a circle in @p base's half-plane at u = 0 (spanned by base.x and base.z),
/// centred @p centreRadial along base.x and @p centreHeight along base.z, of radius @p radius,
/// from the point where it meets the axis below its centre (its south pole; the bottom of the
/// circle where it does not reach the axis), with t at 0, turning out from the axis first, then
/// up: a sphere's meridian at t from 0 to pi, an apple's arc from 0 to the angle it spans.
geo::Frame meridianFrame(const geo::Frame& base, double centreRadial, double centreHeight,
                         double radius) {
    // From the centre (c, h) to the south pole (0, h - sqrt(r^2 - c^2)), as a unit vector in
    // (radial, height); y is it turned a quarter counter-clockwise, toward larger radii.
    const double c = std::min(centreRadial, radius);
    const double drop = std::sqrt(std::max(0.0, radius * radius - c * c));
    geo::Frame f;
    f.origin = base.origin + base.x * centreRadial + base.z * centreHeight;
    f.x = (base.x * -c + base.z * -drop) / radius;
    f.y = (base.x * drop + base.z * -c) / radius;
    f.z = f.x.cross(f.y);
    return f;
}

/// A solid turned about the right-handed frame with @p frame's origin, z and x: its side from a
/// rim of @p bottomRadius at the origin to one of @p topRadius at @p height up the axis, a
/// `Cylinder` where they are equal and otherwise a `Cone` that opens toward the wider rim (its
/// frame's z points that way, so its half-angle is positive and its normal points out); a rim of
/// radius 0 is an apex. Each other rim is a flat disc. Faces "<source>/side", "<source>/top",
/// "<source>/bottom".
std::unique_ptr<Solid> buildTurnedSide(const geo::Frame& frame, const std::string& source,
                                       double bottomRadius, double topRadius, double height) {
    // Whatever the handedness of the frame given, build on a right-handed one with the same
    // origin, axis and seam direction: the surfaces' normals and the rims' directions follow
    // from it.
    const geo::Frame base = geo::Frame::fromAxis(frame.origin, frame.z, frame.x);
    // The side's own frame runs from its narrower rim ("lo", at v = 0) to its wider ("hi", at
    // v = height): upward where the top is as wide or wider, else downward from the top, its y
    // turned with its z so that it stays right-handed (u then runs clockwise about base.z).
    const bool downward = bottomRadius > topRadius;
    geo::Frame side = base;
    if (downward) {
        side.origin = base.origin + base.z * height;
        side.y = base.y * -1.0;
        side.z = base.z * -1.0;
    }
    const double lo = downward ? topRadius : bottomRadius;
    const double hi = downward ? bottomRadius : topRadius;
    const char* loRole = downward ? "top" : "bottom";
    const char* hiRole = downward ? "bottom" : "top";

    auto solid = std::make_unique<Solid>();
    Builder bd(*solid);
    const Vec3 up = side.z * height;
    Vertex* vLo = bd.vertex(side.origin + side.x * lo);
    Vertex* vHi = bd.vertex(side.origin + side.x * hi + up);

    // Caps. Frames have z along the outward normal; the lo cap's y is therefore -side.y.
    const geo::Frame hiFrame = geo::Frame::fromAxis(side.origin + up, side.z, side.x);
    Face* hiCap = bd.face(source, hiRole, std::make_shared<geo::Plane>(hiFrame));
    Face* loCap = nullptr;
    if (lo > 0.0) {
        const geo::Frame loFrame = geo::Frame::fromAxis(side.origin, side.z * -1.0, side.x);
        loCap = bd.face(source, loRole, std::make_shared<geo::Plane>(loFrame));
    }
    std::shared_ptr<const geo::Surface> surface;
    if (lo == hi) {
        surface = std::make_shared<geo::Cylinder>(side, lo);
    } else {
        surface = std::make_shared<geo::Cone>(side, lo, std::atan2(hi - lo, height));
    }
    Face* sideFace = bd.face(source, "side", surface);

    // The side's outer loop runs counter-clockwise in (u, v): along the lo rim to the right
    // (where there is one; at an apex the loop jumps along it), up the seam at u = 2 pi, back
    // along the hi rim, down the seam at u = 0.
    HalfEdge* sideLo = lo > 0.0 ? bd.halfEdge(vLo) : nullptr;
    HalfEdge* seamUp = bd.halfEdge(vLo);
    HalfEdge* sideHi = bd.halfEdge(vHi);
    HalfEdge* seamDown = bd.halfEdge(vHi);
    if (sideLo != nullptr) {
        bd.loop(sideFace, {sideLo, seamUp, sideHi, seamDown}, true);
        setLinePcurve(sideLo, {0.0, 0.0}, {math::kTwoPi, 0.0});
    } else {
        bd.loop(sideFace, {seamUp, sideHi, seamDown}, true);
    }
    setLinePcurve(seamUp, {math::kTwoPi, 0.0}, {math::kTwoPi, height});
    setLinePcurve(sideHi, {math::kTwoPi, height}, {0.0, height});
    setLinePcurve(seamDown, {0.0, height}, {0.0, 0.0});

    // The hi cap's rim runs counter-clockwise about side.z, the lo cap's clockwise: in its own
    // (u, v) each is counter-clockwise.
    HalfEdge* capHi = bd.halfEdge(vHi);
    bd.loop(hiCap, {capHi}, true);
    setPcurve(capHi, std::make_shared<geo::Circle2D>(Vec2{0.0, 0.0}, hi), 0.0, math::kTwoPi);
    HalfEdge* capLo = nullptr;
    if (loCap != nullptr) {
        capLo = bd.halfEdge(vLo);
        bd.loop(loCap, {capLo}, true);
        setPcurve(capLo, std::make_shared<geo::Circle2D>(Vec2{0.0, 0.0}, lo), 0.0, math::kTwoPi);
    }

    geo::Frame hiRimFrame = side;
    hiRimFrame.origin = side.origin + up;
    if (capLo != nullptr) {
        bd.edge(sideLo, capLo, std::make_shared<geo::CircleCurve>(side, lo), 0.0, math::kTwoPi);
    }
    bd.edge(capHi, sideHi, std::make_shared<geo::CircleCurve>(hiRimFrame, hi), 0.0, math::kTwoPi);
    const double slant = std::hypot(hi - lo, height);
    const Vec3 along = lo == hi ? side.z : (side.x * (hi - lo) + up) / slant;
    bd.edge(seamUp, seamDown, std::make_shared<geo::LineCurve>(vLo->point, along), 0.0,
            lo == hi ? height : slant);
    return solid;
}

}  // namespace

std::unique_ptr<Solid> makeBlock(const Vec3& min, const Vec3& max) {
    if (!(min.x < max.x) || !(min.y < max.y) || !(min.z < max.z)) return nullptr;
    auto solid = std::make_unique<Solid>();
    buildBlock(*solid, blockCorners(min, max), geo::Frame{});
    return solid;
}

std::unique_ptr<Solid> makeBox(const geo::Frame& base, double width, double height, double depth) {
    if (!positiveFinite(width) || !positiveFinite(height) || !positiveFinite(depth) ||
        !(base.z.length() > 0.0) || !(base.x.length() > 0.0) || !(base.y.length() > 0.0)) {
        return nullptr;
    }
    const Vec3 o = base.origin;
    const Vec3 w = base.x * width;
    const Vec3 h = base.y * height;
    const Vec3 d = base.z * depth;
    const std::array<Vec3, 8> pts = {
        {o, o + w, o + w + h, o + h, o + d, o + w + d, o + w + h + d, o + h + d}};
    auto solid = std::make_unique<Solid>();
    buildBlock(*solid, pts, base);
    return solid;
}

std::unique_ptr<Solid> makeCylinder(const geo::Frame& frame, double radius, double height) {
    if (!positiveFinite(radius) || !positiveFinite(height) || !(frame.z.length() > 0.0)) {
        return nullptr;
    }
    return buildTurnedSide(frame, "cylinder", radius, radius, height);
}

std::unique_ptr<Solid> makeCone(const geo::Frame& frame, double bottomRadius, double topRadius,
                                double height) {
    if (!std::isfinite(bottomRadius) || !std::isfinite(topRadius) || bottomRadius < 0.0 ||
        topRadius < 0.0 || !positiveFinite(height) || !(frame.z.length() > 0.0)) {
        return nullptr;
    }
    // A radius, or a difference of radii, the model cannot tell from nothing at this size is
    // nothing: an apex, or a cylinder (as PrimitiveFactory tags one).
    const math::ModelTolerance tol = math::ModelTolerance::forExtent(
        std::hypot(2.0 * std::max(bottomRadius, topRadius), height));
    if (tol.within(bottomRadius)) bottomRadius = 0.0;
    if (tol.within(topRadius)) topRadius = 0.0;
    if (bottomRadius == 0.0 && topRadius == 0.0) return nullptr;
    const bool alike = tol.within(std::fabs(bottomRadius - topRadius));
    if (alike) topRadius = bottomRadius;
    return buildTurnedSide(frame, "cone", bottomRadius, topRadius, height);
}

std::unique_ptr<Solid> makeSphere(const geo::Frame& frame, double radius) {
    if (!positiveFinite(radius) || !(frame.z.length() > 0.0)) return nullptr;
    const geo::Frame base = geo::Frame::fromAxis(frame.origin, frame.z, frame.x);
    auto solid = std::make_unique<Solid>();
    Builder bd(*solid);
    Vertex* south = bd.vertex(base.origin - base.z * radius);
    Vertex* north = bd.vertex(base.origin + base.z * radius);
    Face* band = bd.face("sphere", "band", std::make_shared<geo::Sphere>(base, radius));
    // Counter-clockwise in (u, v): up the seam at u = 2 pi, along the north pole to u = 0, down
    // the seam there, along the south pole back.
    HalfEdge* up = bd.halfEdge(south);
    HalfEdge* down = bd.halfEdge(north);
    bd.loop(band, {up, down}, true);
    setLinePcurve(up, {math::kTwoPi, -math::kHalfPi}, {math::kTwoPi, math::kHalfPi});
    setLinePcurve(down, {0.0, math::kHalfPi}, {0.0, -math::kHalfPi});
    bd.edge(up, down,
            std::make_shared<geo::CircleCurve>(meridianFrame(base, 0.0, 0.0, radius), radius), 0.0,
            math::kPi);
    return solid;
}

std::unique_ptr<Solid> makeTorus(const geo::Frame& frame, double majorRadius, double minorRadius) {
    if (!positiveFinite(majorRadius) || !positiveFinite(minorRadius) || !(frame.z.length() > 0.0)) {
        return nullptr;
    }
    const geo::Frame base = geo::Frame::fromAxis(frame.origin, frame.z, frame.x);
    // A tube that clears the axis by no more than the model resolution touches it: a horn.
    const math::ModelTolerance tol =
        math::ModelTolerance::forExtent(2.0 * (majorRadius + minorRadius));
    if (tol.within(std::fabs(majorRadius - minorRadius))) majorRadius = minorRadius;
    auto solid = std::make_unique<Solid>();
    Builder bd(*solid);
    if (majorRadius > minorRadius) {
        // A ring torus: bounded by its outer equator (v = 0) and its tube's circle at u = 0, each
        // a seam, meeting at one vertex; counter-clockwise in (u, v).
        Vertex* v = bd.vertex(base.origin + base.x * (majorRadius + minorRadius));
        Face* patch =
            bd.face("torus", "patch", std::make_shared<geo::Torus>(base, majorRadius, minorRadius));
        HalfEdge* outward = bd.halfEdge(v);
        HalfEdge* up = bd.halfEdge(v);
        HalfEdge* back = bd.halfEdge(v);
        HalfEdge* down = bd.halfEdge(v);
        bd.loop(patch, {outward, up, back, down}, true);
        setLinePcurve(outward, {0.0, 0.0}, {math::kTwoPi, 0.0});
        setLinePcurve(up, {math::kTwoPi, 0.0}, {math::kTwoPi, math::kTwoPi});
        setLinePcurve(back, {math::kTwoPi, math::kTwoPi}, {0.0, math::kTwoPi});
        setLinePcurve(down, {0.0, math::kTwoPi}, {0.0, 0.0});
        bd.edge(outward, back, std::make_shared<geo::CircleCurve>(base, majorRadius + minorRadius),
                0.0, math::kTwoPi);
        geo::Frame tube;
        tube.origin = base.origin + base.x * majorRadius;
        tube.x = base.x;
        tube.y = base.z;
        tube.z = base.x.cross(base.z);
        bd.edge(up, down, std::make_shared<geo::CircleCurve>(tube, minorRadius), 0.0, math::kTwoPi);
        return solid;
    }
    // The apple: the arc of the tube's circle on base.x's side of the axis, from the south pole
    // round to the north, turned a whole turn about the axis.
    const geo::Frame arcFrame = meridianFrame(base, majorRadius, 0.0, minorRadius);
    const double sweep = math::kTwoPi - 2.0 * std::acos(std::min(1.0, majorRadius / minorRadius));
    const auto arc = std::make_shared<geo::CircleCurve>(arcFrame, minorRadius);
    const auto profile = arc->toNurbs(0.0, sweep);
    auto surface = std::make_shared<geo::NurbsSurface>(
        geo::NurbsSurface::makeRevolution(*profile, base.origin, base.z, math::kTwoPi));
    const double v0 = surface->vMin();
    const double v1 = surface->vMax();
    const double u1 = surface->uMax();
    // The poles, on the axis; a horn's circle meets it once, so its two poles are one point.
    const double reach = std::sqrt(minorRadius * minorRadius - majorRadius * majorRadius);
    Vertex* south = bd.vertex(base.origin - base.z * reach);
    Vertex* north = reach == 0.0 ? south : bd.vertex(base.origin + base.z * reach);
    Face* patch = bd.face("torus", "patch", std::move(surface));
    HalfEdge* up = bd.halfEdge(south);
    HalfEdge* down = bd.halfEdge(north);
    bd.loop(patch, {up, down}, true);
    setLinePcurve(up, {u1, v0}, {u1, v1});
    setLinePcurve(down, {0.0, v1}, {0.0, v0});
    bd.edge(up, down, arc, 0.0, sweep);
    return solid;
}

std::unique_ptr<Solid> makeSphereSector(const geo::Frame& frame, double radius, double angle) {
    if (!positiveFinite(radius) || !(angle > 0.0) || !(angle < math::kTwoPi) ||
        !(frame.z.length() > 0.0)) {
        return nullptr;
    }
    const geo::Frame base = geo::Frame::fromAxis(frame.origin, frame.z, frame.x);
    auto solid = std::make_unique<Solid>();
    Builder bd(*solid);
    Vertex* south = bd.vertex(base.origin - base.z * radius);
    Vertex* north = bd.vertex(base.origin + base.z * radius);

    // The lune, counter-clockwise in (u, v): up the meridian at u = angle, along the north pole
    // to u = 0, down the meridian there, along the south pole back.
    Face* band = bd.face("sphere", "band", std::make_shared<geo::Sphere>(base, radius));
    HalfEdge* up = bd.halfEdge(south);
    HalfEdge* down = bd.halfEdge(north);
    bd.loop(band, {up, down}, true);
    setLinePcurve(up, {angle, -math::kHalfPi}, {angle, math::kHalfPi});
    setLinePcurve(down, {0.0, math::kHalfPi}, {0.0, -math::kHalfPi});

    // The caps face away from the lune: the start's outward normal is -y (the direction u
    // leaves u = 0 by, reversed), the end's the direction u leaves u = angle by. With x the
    // radial direction, each cap's (u, v) is (distance from the axis, height) at the start and
    // (distance from the axis, depth) at the end; each half-disc runs counter-clockwise in it,
    // along its meridian then back along the axis.
    const Vec3 endRadial = base.x * std::cos(angle) + base.y * std::sin(angle);
    const Vec3 endTangent = base.y * std::cos(angle) - base.x * std::sin(angle);
    Face* start = bd.face(
        "sphere", "cap_start",
        std::make_shared<geo::Plane>(geo::Frame::fromAxis(base.origin, base.y * -1.0, base.x)));
    Face* end = bd.face(
        "sphere", "cap_end",
        std::make_shared<geo::Plane>(geo::Frame::fromAxis(base.origin, endTangent, endRadial)));
    HalfEdge* startArc = bd.halfEdge(south);
    HalfEdge* startAxis = bd.halfEdge(north);
    bd.loop(start, {startArc, startAxis}, true);
    const auto halfCircle =
        std::make_shared<geo::Arc2D>(Vec2{0.0, 0.0}, radius, -math::kHalfPi, math::kHalfPi);
    setPcurve(startArc, halfCircle, 0.0, 1.0);
    setLinePcurve(startAxis, {0.0, radius}, {0.0, -radius});
    HalfEdge* endArc = bd.halfEdge(north);
    HalfEdge* endAxis = bd.halfEdge(south);
    bd.loop(end, {endArc, endAxis}, true);
    setPcurve(endArc, halfCircle, 0.0, 1.0);
    setLinePcurve(endAxis, {0.0, radius}, {0.0, -radius});

    // Each meridian runs from the south pole to the north.
    geo::Frame endBase = base;
    endBase.x = endRadial;
    endBase.y = endTangent;
    bd.edge(startArc, down,
            std::make_shared<geo::CircleCurve>(meridianFrame(base, 0.0, 0.0, radius), radius), 0.0,
            math::kPi);
    bd.edge(up, endArc,
            std::make_shared<geo::CircleCurve>(meridianFrame(endBase, 0.0, 0.0, radius), radius),
            0.0, math::kPi);
    bd.edge(endAxis, startAxis, std::make_shared<geo::LineCurve>(south->point, base.z), 0.0,
            2.0 * radius);
    return solid;
}

std::unique_ptr<Solid> makePlateWithHole(const Vec3& min, const Vec3& max, const Vec2& holeCentre,
                                         double holeRadius) {
    if (!(min.x < max.x) || !(min.y < max.y) || !(min.z < max.z) || !(holeRadius > 0.0) ||
        !(holeCentre.x - holeRadius > min.x) || !(holeCentre.x + holeRadius < max.x) ||
        !(holeCentre.y - holeRadius > min.y) || !(holeCentre.y + holeRadius < max.y)) {
        return nullptr;
    }
    auto solid = std::make_unique<Solid>();
    Block block = buildBlock(*solid, blockCorners(min, max), geo::Frame{});
    Builder& bd = block.builder;

    const double height = max.z - min.z;
    const geo::Frame base =
        geo::Frame::fromAxis({holeCentre.x, holeCentre.y, min.z}, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0});
    const Vec3 up = base.z * height;
    Vertex* vb = bd.vertex(base.origin + base.x * holeRadius);
    Vertex* vt = bd.vertex(base.origin + base.x * holeRadius + up);

    // The wall faces the hole's axis: its cylinder is reversed, so its outer loop runs clockwise
    // in (u, v): along the bottom rim to the left, up the seam at u = 0, along the top rim to
    // the right, down the seam at u = 2 pi.
    Face* wall = bd.face("hole", "wall", std::make_shared<geo::Cylinder>(base, holeRadius), true);
    HalfEdge* wallBottom = bd.halfEdge(vb);
    HalfEdge* seamUp = bd.halfEdge(vb);
    HalfEdge* wallTop = bd.halfEdge(vt);
    HalfEdge* seamDown = bd.halfEdge(vt);
    bd.loop(wall, {wallBottom, seamUp, wallTop, seamDown}, true);
    setLinePcurve(wallBottom, {math::kTwoPi, 0.0}, {0.0, 0.0});
    setLinePcurve(seamUp, {0.0, 0.0}, {0.0, height});
    setLinePcurve(wallTop, {0.0, height}, {math::kTwoPi, height});
    setLinePcurve(seamDown, {math::kTwoPi, height}, {math::kTwoPi, 0.0});

    // The hole's inner loop on each cap runs the opposite way to an outer loop: clockwise in
    // (u, v), which is the circle traversed from 2 pi down to 0.
    HalfEdge* capTop = bd.halfEdge(vt);
    HalfEdge* capBottom = bd.halfEdge(vb);
    bd.loop(block.top, {capTop}, false);
    bd.loop(block.bottom, {capBottom}, false);
    setPcurve(capTop,
              std::make_shared<geo::Circle2D>(
                  block.top->exactSurface->invert(vt->point - base.x * holeRadius), holeRadius),
              math::kTwoPi, 0.0);
    setPcurve(capBottom,
              std::make_shared<geo::Circle2D>(
                  block.bottom->exactSurface->invert(vb->point - base.x * holeRadius), holeRadius),
              math::kTwoPi, 0.0);

    // Both rim curves run counter-clockwise about z, as the wall's u does.
    geo::Frame topRimFrame = base;
    topRimFrame.origin = base.origin + up;
    bd.edge(capBottom, wallBottom, std::make_shared<geo::CircleCurve>(base, holeRadius), 0.0,
            math::kTwoPi);
    bd.edge(wallTop, capTop, std::make_shared<geo::CircleCurve>(topRimFrame, holeRadius), 0.0,
            math::kTwoPi);
    bd.edge(seamUp, seamDown, std::make_shared<geo::LineCurve>(vb->point, base.z), 0.0, height);
    return solid;
}

}  // namespace hz::model::exact
