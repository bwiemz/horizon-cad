#include "MeshCsg.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace hz::model {

using hz::math::Vec3;

namespace {

/// Distance from a plane within which a point counts as lying on it.
/// Downstream sewing welds at a tolerance >= this (see kCsgPlaneEps in the
/// header) so on-plane split points the CSG treats as coincident are always
/// reconcilable during sewing.
constexpr double kPlaneEps = kCsgPlaneEps;

struct Plane {
    Vec3 normal;   // unit
    double w = 0;  // normal · x = w
    bool ok = false;

    static Plane fromPolygon(const std::vector<Vec3>& pts) {
        Plane p;
        Vec3 n = Vec3::Zero;
        const size_t count = pts.size();
        for (size_t i = 0; i < count; ++i) {
            const Vec3& a = pts[i];
            const Vec3& b = pts[(i + 1) % count];
            n.x += (a.y - b.y) * (a.z + b.z);
            n.y += (a.z - b.z) * (a.x + b.x);
            n.z += (a.x - b.x) * (a.y + b.y);
        }
        const double len = n.length();
        if (len < 1e-30) return p;
        p.normal = n / len;
        p.w = p.normal.dot(pts[0]);
        p.ok = true;
        return p;
    }

    void flip() {
        normal = normal * -1.0;
        w = -w;
    }
};

struct Poly {
    CsgPolygon data;
    Plane plane;

    void flip() {
        std::reverse(data.points.begin(), data.points.end());
        plane.flip();
    }
};

enum : int { COPLANAR = 0, FRONT = 1, BACK = 2, SPANNING = 3 };

/// Split `poly` by `plane` into the four output buckets (csg.js algorithm).
void splitPolygon(const Plane& plane, const Poly& poly, std::vector<Poly>& coplanarFront,
                  std::vector<Poly>& coplanarBack, std::vector<Poly>& front,
                  std::vector<Poly>& back) {
    const auto& pts = poly.data.points;
    const size_t n = pts.size();

    int polygonType = 0;
    std::vector<int> types(n);
    for (size_t i = 0; i < n; ++i) {
        const double t = plane.normal.dot(pts[i]) - plane.w;
        const int type = (t < -kPlaneEps) ? BACK : (t > kPlaneEps) ? FRONT : COPLANAR;
        polygonType |= type;
        types[i] = type;
    }

    switch (polygonType) {
        case COPLANAR:
            (plane.normal.dot(poly.plane.normal) > 0 ? coplanarFront : coplanarBack)
                .push_back(poly);
            break;
        case FRONT:
            front.push_back(poly);
            break;
        case BACK:
            back.push_back(poly);
            break;
        case SPANNING: {
            std::vector<Vec3> f;
            std::vector<Vec3> b;
            f.reserve(n + 1);
            b.reserve(n + 1);
            for (size_t i = 0; i < n; ++i) {
                const size_t j = (i + 1) % n;
                const int ti = types[i];
                const int tj = types[j];
                const Vec3& vi = pts[i];
                const Vec3& vj = pts[j];
                if (ti != BACK) f.push_back(vi);
                if (ti != FRONT) b.push_back(vi);
                if ((ti | tj) == SPANNING) {
                    const double denom = plane.normal.dot(vj - vi);
                    if (std::abs(denom) > 1e-30) {
                        const double t = (plane.w - plane.normal.dot(vi)) / denom;
                        const Vec3 v = vi + (vj - vi) * t;
                        f.push_back(v);
                        b.push_back(v);
                    }
                }
            }
            auto emit = [&poly](std::vector<Vec3>&& loop, std::vector<Poly>& out) {
                if (loop.size() < 3) return;
                Poly piece;
                piece.data.points = std::move(loop);
                piece.data.topoId = poly.data.topoId;
                piece.data.surface = nullptr;  // fragment no longer matches source patch
                piece.data.fromA = poly.data.fromA;
                piece.plane = poly.plane;  // splitting preserves the carrier plane
                out.push_back(std::move(piece));
            };
            emit(std::move(f), front);
            emit(std::move(b), back);
            break;
        }
        default:
            break;
    }
}

/// One node of the CSG BSP tree (csg.js structure, double precision).
struct Node {
    Plane plane;
    bool hasPlane = false;
    std::unique_ptr<Node> front;
    std::unique_ptr<Node> back;
    std::vector<Poly> polygons;

    void invert() {
        for (auto& p : polygons) p.flip();
        if (hasPlane) plane.flip();
        if (front) front->invert();
        if (back) back->invert();
        std::swap(front, back);
    }

    std::vector<Poly> clipPolygons(std::vector<Poly> list) const {
        if (!hasPlane) return list;
        std::vector<Poly> f;
        std::vector<Poly> b;
        for (auto& poly : list) {
            splitPolygon(plane, poly, f, b, f, b);
        }
        if (front) f = front->clipPolygons(std::move(f));
        if (back) {
            b = back->clipPolygons(std::move(b));
        } else {
            b.clear();  // back of a leaf plane is inside the solid — discard
        }
        f.insert(f.end(), std::make_move_iterator(b.begin()), std::make_move_iterator(b.end()));
        return f;
    }

    void clipTo(const Node& bsp) {
        polygons = bsp.clipPolygons(std::move(polygons));
        if (front) front->clipTo(bsp);
        if (back) back->clipTo(bsp);
    }

    void allPolygons(std::vector<Poly>& out) const {
        out.insert(out.end(), polygons.begin(), polygons.end());
        if (front) front->allPolygons(out);
        if (back) back->allPolygons(out);
    }

    void build(std::vector<Poly> list) {
        if (list.empty()) return;
        if (!hasPlane) {
            plane = list[0].plane;
            hasPlane = true;
        }
        std::vector<Poly> frontList;
        std::vector<Poly> backList;
        for (auto& poly : list) {
            splitPolygon(plane, poly, polygons, polygons, frontList, backList);
        }
        if (!frontList.empty()) {
            if (!front) front = std::make_unique<Node>();
            front->build(std::move(frontList));
        }
        if (!backList.empty()) {
            if (!back) back = std::make_unique<Node>();
            back->build(std::move(backList));
        }
    }
};

std::vector<Poly> toPolys(const std::vector<CsgPolygon>& in) {
    std::vector<Poly> out;
    out.reserve(in.size());
    for (const auto& p : in) {
        Poly poly;
        poly.plane = Plane::fromPolygon(p.points);
        if (!poly.plane.ok) continue;  // degenerate input polygon
        poly.data = p;
        out.push_back(std::move(poly));
    }
    return out;
}

}  // namespace

std::vector<CsgPolygon> csgExecute(const std::vector<CsgPolygon>& a,
                                   const std::vector<CsgPolygon>& b, BooleanType type) {
    Node nodeA;
    Node nodeB;
    nodeA.build(toPolys(a));
    nodeB.build(toPolys(b));

    switch (type) {
        case BooleanType::Union:
            nodeA.clipTo(nodeB);
            nodeB.clipTo(nodeA);
            nodeB.invert();
            nodeB.clipTo(nodeA);
            nodeB.invert();
            break;
        case BooleanType::Subtract:
            nodeA.invert();
            nodeA.clipTo(nodeB);
            nodeB.clipTo(nodeA);
            nodeB.invert();
            nodeB.clipTo(nodeA);
            nodeB.invert();
            break;
        case BooleanType::Intersect:
            nodeA.invert();
            nodeB.clipTo(nodeA);
            nodeB.invert();
            nodeA.clipTo(nodeB);
            nodeB.clipTo(nodeA);
            break;
    }

    std::vector<Poly> bAll;
    nodeB.allPolygons(bAll);
    nodeA.build(std::move(bAll));
    if (type != BooleanType::Union) {
        nodeA.invert();
    }

    std::vector<Poly> merged;
    nodeA.allPolygons(merged);

    std::vector<CsgPolygon> result;
    result.reserve(merged.size());
    for (auto& poly : merged) {
        result.push_back(std::move(poly.data));
    }
    return result;
}

std::vector<CsgPolygon> csgTriangles(const std::vector<BoundaryPolygon>& polygons, bool fromA) {
    std::vector<CsgPolygon> out;
    out.reserve(polygons.size() * 2);
    for (const auto& poly : polygons) {
        for (const auto& tri : BoundaryMesh::triangulatePolygon(poly.points)) {
            CsgPolygon p;
            p.points.assign(tri.begin(), tri.end());
            p.topoId = poly.topoId;
            p.surface = nullptr;
            p.fromA = fromA;
            out.push_back(std::move(p));
        }
    }
    return out;
}

double csgVolume(const std::vector<CsgPolygon>& polygons) {
    double vol6 = 0.0;
    for (const auto& poly : polygons) {
        const auto& p = poly.points;
        for (size_t i = 1; i + 1 < p.size(); ++i) {
            vol6 += p[0].dot(p[i].cross(p[i + 1]));
        }
    }
    return vol6 / 6.0;
}

}  // namespace hz::model
