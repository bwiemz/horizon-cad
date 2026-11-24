#include "horizon/modeling/ExactPrimitives.h"

#include <array>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "horizon/geometry/curves/AnalyticCurves.h"
#include "horizon/geometry/curves/Circle2D.h"
#include "horizon/geometry/curves/Line2D.h"
#include "horizon/geometry/surfaces/AnalyticSurfaces.h"
#include "horizon/math/Constants.h"
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

/// The pieces of the block that a hole through it needs.
struct Block {
    Builder builder;
    Face* bottom = nullptr;
    Face* top = nullptr;
};

/// Builds the six planar faces of the block. Every frame has z along the face's outward normal,
/// so no face is reversed, and each outer loop runs counter-clockwise seen from outside, which
/// is counter-clockwise in the face's (u, v).
Block buildBlock(Solid& solid, const Vec3& min, const Vec3& max) {
    Block b{Builder(solid), nullptr, nullptr};
    Builder& bd = b.builder;

    const std::array<Vec3, 8> pts = {{{min.x, min.y, min.z},
                                      {max.x, min.y, min.z},
                                      {max.x, max.y, min.z},
                                      {min.x, max.y, min.z},
                                      {min.x, min.y, max.z},
                                      {max.x, min.y, max.z},
                                      {max.x, max.y, max.z},
                                      {min.x, max.y, max.z}}};
    std::array<Vertex*, 8> v{};
    for (size_t i = 0; i < 8; ++i) v[i] = bd.vertex(pts[i]);

    struct Side {
        const char* role;
        std::array<int, 4> corners;  // counter-clockwise seen from outside
        Vec3 normal;
    };
    const std::array<Side, 6> sides = {{{"bottom", {0, 3, 2, 1}, {0, 0, -1}},
                                        {"top", {4, 5, 6, 7}, {0, 0, 1}},
                                        {"front", {0, 1, 5, 4}, {0, -1, 0}},
                                        {"back", {2, 3, 7, 6}, {0, 1, 0}},
                                        {"right", {1, 2, 6, 5}, {1, 0, 0}},
                                        {"left", {3, 0, 4, 7}, {-1, 0, 0}}}};

    std::map<std::pair<Vertex*, Vertex*>, HalfEdge*> open;
    for (const Side& s : sides) {
        const geo::Frame frame =
            geo::Frame::fromAxis(pts[static_cast<size_t>(s.corners[0])], s.normal, {1.0, 0.0, 0.0});
        const auto plane = std::make_shared<geo::Plane>(frame);
        Face* f = bd.face("box", s.role, plane);
        std::vector<HalfEdge*> hes;
        hes.reserve(4);
        for (size_t i = 0; i < 4; ++i) {
            hes.push_back(bd.halfEdge(v[static_cast<size_t>(s.corners[i])]));
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

}  // namespace

std::unique_ptr<Solid> makeBlock(const Vec3& min, const Vec3& max) {
    if (!(min.x < max.x) || !(min.y < max.y) || !(min.z < max.z)) return nullptr;
    auto solid = std::make_unique<Solid>();
    buildBlock(*solid, min, max);
    return solid;
}

std::unique_ptr<Solid> makeCylinder(const geo::Frame& frame, double radius, double height) {
    if (!(radius > 0.0) || !(height > 0.0) || !(frame.z.length() > 0.0)) return nullptr;
    // Whatever the handedness of the frame given, build on a right-handed one with the same
    // origin, axis and seam direction: the surfaces' normals and the rims' directions follow
    // from it.
    const geo::Frame base = geo::Frame::fromAxis(frame.origin, frame.z, frame.x);
    auto solid = std::make_unique<Solid>();
    Builder bd(*solid);

    const Vec3 up = base.z * height;
    Vertex* vb = bd.vertex(base.origin + base.x * radius);
    Vertex* vt = bd.vertex(base.origin + base.x * radius + up);

    // Caps. Frames have z along the outward normal; the bottom's y is therefore -base.y.
    const geo::Frame topFrame = geo::Frame::fromAxis(base.origin + up, base.z, base.x);
    const geo::Frame bottomFrame = geo::Frame::fromAxis(base.origin, base.z * -1.0, base.x);
    Face* top = bd.face("cylinder", "top", std::make_shared<geo::Plane>(topFrame));
    Face* bottom = bd.face("cylinder", "bottom", std::make_shared<geo::Plane>(bottomFrame));
    Face* side = bd.face("cylinder", "side", std::make_shared<geo::Cylinder>(base, radius));

    // The side's outer loop runs counter-clockwise in (u, v): along the bottom rim to the right,
    // up the seam at u = 2 pi, back along the top rim, down the seam at u = 0.
    HalfEdge* sideBottom = bd.halfEdge(vb);
    HalfEdge* seamUp = bd.halfEdge(vb);
    HalfEdge* sideTop = bd.halfEdge(vt);
    HalfEdge* seamDown = bd.halfEdge(vt);
    bd.loop(side, {sideBottom, seamUp, sideTop, seamDown}, true);
    const double h = height;
    setLinePcurve(sideBottom, {0.0, 0.0}, {math::kTwoPi, 0.0});
    setLinePcurve(seamUp, {math::kTwoPi, 0.0}, {math::kTwoPi, h});
    setLinePcurve(sideTop, {math::kTwoPi, h}, {0.0, h});
    setLinePcurve(seamDown, {0.0, h}, {0.0, 0.0});

    // The top cap's rim runs counter-clockwise about z, the bottom cap's clockwise: in its own
    // (u, v) each is counter-clockwise.
    HalfEdge* capTop = bd.halfEdge(vt);
    HalfEdge* capBottom = bd.halfEdge(vb);
    bd.loop(top, {capTop}, true);
    bd.loop(bottom, {capBottom}, true);
    const auto rim = std::make_shared<geo::Circle2D>(Vec2{0.0, 0.0}, radius);
    setPcurve(capTop, rim, 0.0, math::kTwoPi);
    setPcurve(capBottom, rim, 0.0, math::kTwoPi);

    const auto bottomRim = std::make_shared<geo::CircleCurve>(base, radius);
    geo::Frame topRimFrame = base;
    topRimFrame.origin = base.origin + up;
    const auto topRim = std::make_shared<geo::CircleCurve>(topRimFrame, radius);
    bd.edge(sideBottom, capBottom, bottomRim, 0.0, math::kTwoPi);
    bd.edge(capTop, sideTop, topRim, 0.0, math::kTwoPi);
    bd.edge(seamUp, seamDown, std::make_shared<geo::LineCurve>(vb->point, base.z), 0.0, height);
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
    Block block = buildBlock(*solid, min, max);
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
