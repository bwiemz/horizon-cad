// The ideal mass properties (Phase 141): a solid measured on the surfaces and
// curves its facets and chords approximate.
//
// Each edge is cut into m pieces whose points lie on its ideal: the curve it
// approximates, or where the ideal surfaces of its two faces meet (found by
// projecting onto each in turn). A face with a curved ideal is triangulated,
// each triangle cut into m² and its points put on the surface; a flat face
// keeps its plane and takes its edges' points as its boundary. The pieces
// share their edges' points, so the refined boundary is closed and the
// divergence theorem measures what it encloses. Its error falls as 1/m², in
// even powers: the measures at m = 1, 2, 4, ... are extrapolated (Romberg)
// until they settle.

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <map>
#include <set>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "MassIntegrals.h"
#include "horizon/geometry/curves/NurbsCurve.h"
#include "horizon/geometry/surfaces/NurbsSurface.h"
#include "horizon/modeling/BoundaryMesh.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/modeling/Naming.h"
#include "horizon/topology/HalfEdge.h"
#include "horizon/topology/Solid.h"

namespace hz::model {

using hz::math::Vec3;
using topo::Edge;
using topo::Face;
using topo::HalfEdge;
using topo::Wire;

namespace {

using UV = std::pair<double, double>;

/// The largest refinement measured: each edge in 64 pieces.
constexpr int kMaxRefinement = 64;
/// No refinement past this many triangles.
constexpr double kTriangleBudget = 6e6;
/// Two faces without ideals that bend by less than this are taken for the
/// facets of one curved surface (as the display takes them).
constexpr double kSmoothCosine = 0.8660254037844387;  // cos 30 degrees

std::vector<const HalfEdge*> loopOf(const Wire* wire) {
    std::vector<const HalfEdge*> loop;
    if (wire == nullptr || wire->halfEdge == nullptr) return loop;
    const HalfEdge* start = wire->halfEdge;
    const HalfEdge* cur = start;
    do {
        if (cur->origin == nullptr || cur->next == nullptr) return {};
        loop.push_back(cur);
        cur = cur->next;
    } while (cur != start && loop.size() < 100000);
    return loop;
}

/// Twice the vector area of a loop (Newell).
Vec3 vectorArea2(const std::vector<Vec3>& points) {
    Vec3 n;
    for (size_t i = 0; i < points.size(); ++i) {
        n += points[i].cross(points[(i + 1) % points.size()]);
    }
    return n;
}

/// A surface is flat when all its control points lie on the plane through
/// its middle: a plane, or a loft's planar band.
bool isFlat(const geo::NurbsSurface& s) {
    const auto mid =
        s.evaluateWithDerivatives(0.5 * (s.uMin() + s.uMax()), 0.5 * (s.vMin() + s.vMax()));
    Vec3 n = mid.du.cross(mid.dv);
    double scale = 0.0;
    for (const auto& row : s.controlPoints()) {
        for (const auto& p : row) scale = std::max(scale, (p - mid.point).length());
    }
    if (n.length() <= 1e-300 || scale <= 0.0) return false;
    n = n * (1.0 / n.length());
    for (const auto& row : s.controlPoints()) {
        for (const auto& p : row) {
            if (std::abs((p - mid.point).dot(n)) > 1e-12 * scale) return false;
        }
    }
    return true;
}

struct Plane {
    Vec3 point;
    Vec3 normal;  ///< unit
};

/// What one face is measured on.
struct FaceInfo {
    const geo::NurbsSurface* ideal = nullptr;         ///< curved ideal; null when flat
    Plane plane;                                      ///< of its outer loop
    std::vector<std::vector<const HalfEdge*>> loops;  ///< outer first
};

/// A point's coordinates, as a key.
using PointKey = std::tuple<double, double, double>;
PointKey keyOf(const Vec3& p) {
    return {p.x, p.y, p.z};
}

/// A solid's ideal geometry, gathered once and measured at any refinement.
class IdealSolid {
public:
    explicit IdealSolid(const topo::Solid& solid) : m_solid(solid) {
        for (const auto& face : solid.faces()) {
            FaceInfo info;
            info.loops.push_back(loopOf(face.outerLoop));
            if (info.loops.front().size() < 3) continue;
            for (const Wire* inner : face.innerLoops) info.loops.push_back(loopOf(inner));
            if (face.analyticSurface && !isFlat(*face.analyticSurface)) {
                info.ideal = face.analyticSurface.get();
                m_curved = true;
            }
            std::vector<Vec3> outer;
            for (const HalfEdge* he : info.loops.front()) outer.push_back(he->origin->point);
            Vec3 centre;
            for (const Vec3& p : outer) centre += p;
            info.plane.point = centre * (1.0 / static_cast<double>(outer.size()));
            const Vec3 n = vectorArea2(outer);
            info.plane.normal = n.length() > 0.0 ? n * (1.0 / n.length()) : Vec3(0, 0, 1);
            m_faces.emplace(&face, std::move(info));
        }
        for (const auto& v : solid.vertices()) {
            m_scale = std::max(m_scale, (v.point - solid.vertices().front().point).length());
        }
        m_scale = std::max(m_scale, 1e-300);
        for (const auto& e : solid.edges()) m_curved = m_curved || curveOf(e) != nullptr;
    }

    /// Whether anything is curved: else the ideal is the modelled solid.
    bool curved() const { return m_curved; }

    /// Triangles a measure at refinement @p m makes, roughly.
    double triangles(int m) const {
        double n = 0.0;
        for (const auto& [face, info] : m_faces) {
            size_t points = 0;
            for (const auto& loop : info.loops) points += loop.size();
            n += info.ideal ? static_cast<double>(points) * m * m : static_cast<double>(points) * m;
        }
        return n;
    }

    /// The solid's boundary refined @p m times along each edge, measured.
    detail::Measures measure(int m) {
        m_samples.clear();
        m_parted = 0;
        for (const auto& e : m_solid.edges()) {
            if (e.halfEdge == nullptr || e.halfEdge->origin == nullptr ||
                e.halfEdge->next == nullptr || e.halfEdge->next->origin == nullptr) {
                continue;
            }
            m_samples.emplace(&e, sampleEdge(e, m));
        }
        detail::Measures measures;
        for (const auto& [face, info] : m_faces) measureFace(info, m, measures);
        return measures;
    }

    int partedEdges() const { return m_parted; }

    /// The faces measured as modelled though they are facets of a curved
    /// surface: without an ideal, and bending by under 30 degrees from a
    /// neighbour without one either, across an edge with no curve.
    std::vector<std::string> withoutIdeal() const {
        std::set<std::string> names;
        for (const auto& e : m_solid.edges()) {
            const HalfEdge* he = e.halfEdge;
            if (he == nullptr || he->twin == nullptr || he->face == nullptr ||
                he->twin->face == nullptr || curveOf(e) != nullptr) {
                continue;
            }
            const Face& a = *he->face;
            const Face& b = *he->twin->face;
            if (a.analyticSurface || b.analyticSurface) continue;
            const auto ia = m_faces.find(&a);
            const auto ib = m_faces.find(&b);
            if (ia == m_faces.end() || ib == m_faces.end()) continue;
            const double cosine = ia->second.plane.normal.dot(ib->second.plane.normal);
            if (cosine > 1.0 - 1e-9 || cosine <= kSmoothCosine) continue;
            for (const Face* f : {&a, &b}) {
                names.insert(f->topoId.isValid() ? logicalFace(f->topoId.tag()) : "(unnamed)");
            }
        }
        return {names.begin(), names.end()};
    }

private:
    /// The curve an edge approximates, or its own when curved.
    static const geo::NurbsCurve* curveOf(const Edge& e) {
        if (e.analyticCurve) return e.analyticCurve.get();
        if (e.curve && e.curve->degree() > 1) return e.curve.get();
        return nullptr;
    }

    /// The (u, v) on @p s of a vertex @p p: found once, by a search of the
    /// whole surface, and then the start for every point near it.
    UV seed(const geo::NurbsSurface& s, const Vec3& p) {
        const auto key = std::make_tuple(static_cast<const void*>(&s), p.x, p.y, p.z);
        const auto it = m_seeds.find(key);
        if (it != m_seeds.end()) return it->second;
        const auto miss = [&s, &p](const UV& uv) {
            return (s.evaluateWithDerivatives(uv.first, uv.second).point - p).length();
        };
        const auto [u, v] = s.closestPoint(p);
        UV best = s.project(p, u, v);
        const double tol = 1e-9 * sizeOf(s);
        if (miss(best) > tol) {
            // closestPoint's coarse grid can choose a pole for a point near
            // one, and no search leaves a pole: start from every cell of a
            // finer grid instead.
            constexpr int kStarts = 8;
            const double du = s.uMax() - s.uMin();
            const double dv = s.vMax() - s.vMin();
            for (int i = 0; i < kStarts && miss(best) > tol; ++i) {
                for (int j = 0; j < kStarts; ++j) {
                    const UV uv = s.project(p, s.uMin() + du * (i + 0.5) / kStarts,
                                            s.vMin() + dv * (j + 0.5) / kStarts);
                    if (miss(uv) < miss(best)) best = uv;
                }
            }
        }
        m_seeds.emplace(key, best);
        return best;
    }

    /// The size of @p s's control net.
    double sizeOf(const geo::NurbsSurface& s) {
        const auto it = m_sizes.find(&s);
        if (it != m_sizes.end()) return it->second;
        double size = 0.0;
        for (const auto& row : s.controlPoints()) {
            for (const auto& q : row) size = std::max(size, (q - s.controlPoints()[0][0]).length());
        }
        m_sizes.emplace(&s, size);
        return size;
    }

    /// Whether @p uv is where the surface degenerates to a point along one
    /// direction (a pole, an apex): there u (or v) is arbitrary, and no start
    /// for a point beside it, which Newton may take round to the far side.
    bool degenerate(const geo::NurbsSurface& s, const UV& uv) {
        const auto key = std::make_tuple(static_cast<const void*>(&s), uv.first, uv.second);
        const auto it = m_degenerate.find(key);
        if (it != m_degenerate.end()) return it->second;
        // Where u (or v) far off, at the same v (or u), is the same point.
        // (Where dS/du vanishes is too fine a test: a search stops a hair
        // from the pole, where u still turns the point on a tiny circle.)
        const auto at = [&s](double u, double v) { return s.evaluateWithDerivatives(u, v).point; };
        const Vec3 p = at(uv.first, uv.second);
        const double tol = 1e-6 * sizeOf(s);
        const double du = s.uMax() - s.uMin();
        const double dv = s.vMax() - s.vMin();
        const bool alongU = (at(s.uMin() + 0.31 * du, uv.second) - p).length() <= tol &&
                            (at(s.uMin() + 0.77 * du, uv.second) - p).length() <= tol;
        const bool alongV = (at(uv.first, s.vMin() + 0.31 * dv) - p).length() <= tol &&
                            (at(uv.first, s.vMin() + 0.77 * dv) - p).length() <= tol;
        const bool is = alongU || alongV;
        m_degenerate.emplace(key, is);
        return is;
    }

    /// A start for a point near the points @p near, nearest first: the first
    /// that is not where the surface degenerates.
    UV start(const geo::NurbsSurface& s, std::initializer_list<const Vec3*> near) {
        for (const Vec3* p : near) {
            const UV uv = seed(s, *p);
            if (!degenerate(s, uv)) return uv;
        }
        return seed(s, **near.begin());
    }

    /// @p x put on @p s, from the start @p uv (which it updates).
    static Vec3 onSurface(const geo::NurbsSurface& s, const Vec3& x, UV& uv) {
        uv = s.project(x, uv.first, uv.second);
        return s.evaluateWithDerivatives(uv.first, uv.second).point;
    }

    static Vec3 onPlane(const Plane& plane, const Vec3& x) {
        return x - plane.normal * (x - plane.point).dot(plane.normal);
    }

    /// The m + 1 points of an edge, from its half-edge's origin to its
    /// twin's, the ends its vertices.
    std::vector<Vec3> sampleEdge(const Edge& e, int m) {
        const Vec3 p = e.halfEdge->origin->point;
        const Vec3 q = e.halfEdge->next->origin->point;
        std::vector<Vec3> out(static_cast<size_t>(m) + 1);
        out.front() = p;
        out.back() = q;
        for (int i = 1; i < m; ++i) {
            out[static_cast<size_t>(i)] = p + (q - p) * (static_cast<double>(i) / m);
        }
        if (m == 1 || (q - p).length() <= 1e-15 * m_scale) return out;
        if (const geo::NurbsCurve* curve = curveOf(e)) {
            if (sampleCurve(*curve, p, q, m, out)) return out;
        }

        // Where the ideals of its two faces meet, or on the one ideal.
        const Face* faces[2] = {e.halfEdge->face,
                                e.halfEdge->twin ? e.halfEdge->twin->face : nullptr};
        const FaceInfo* infos[2] = {nullptr, nullptr};
        for (int k = 0; k < 2; ++k) {
            if (faces[k] == nullptr) continue;
            const auto it = m_faces.find(faces[k]);
            if (it != m_faces.end()) infos[k] = &it->second;
        }
        const bool curvedA = infos[0] && infos[0]->ideal;
        const bool curvedB = infos[1] && infos[1]->ideal;
        if (!curvedA && !curvedB) return out;
        const bool one = curvedA && curvedB && infos[0]->ideal == infos[1]->ideal;
        // The starts on each face's ideal, from either end.
        UV fromP[2];
        UV fromQ[2];
        for (int k = 0; k < 2; ++k) {
            if (infos[k] == nullptr || infos[k]->ideal == nullptr) continue;
            fromP[k] = start(*infos[k]->ideal, {&p, &q});
            fromQ[k] = start(*infos[k]->ideal, {&q, &p});
        }
        for (int i = 1; i < m; ++i) {
            UV uv[2] = {2 * i <= m ? fromP[0] : fromQ[0], 2 * i <= m ? fromP[1] : fromQ[1]};
            Vec3 x = out[static_cast<size_t>(i)];
            const auto meet = [&](const Vec3& from) {
                Vec3 y = from;
                for (int k = 0; k < (one ? 1 : 2); ++k) {
                    if (infos[k] == nullptr) continue;
                    y = infos[k]->ideal ? onSurface(*infos[k]->ideal, y, uv[k])
                                        : onPlane(infos[k]->plane, y);
                }
                return y;
            };
            for (int iter = 0; iter < 64; ++iter) {
                const Vec3 y = meet(x);
                const double moved = (y - x).length();
                x = y;
                if (moved <= 1e-14 * m_scale) break;
            }
            out[static_cast<size_t>(i)] = x;
            if (2 * i == m) {
                // Parted where the point cannot lie on both.
                if ((meet(x) - x).length() > 1e-9 * m_scale) ++m_parted;
            }
        }
        return out;
    }

    /// Points along @p curve from @p p to @p q, if it runs between them:
    /// the way round the shorter to the chord on a closed curve.
    bool sampleCurve(const geo::NurbsCurve& curve, const Vec3& p, const Vec3& q, int m,
                     std::vector<Vec3>& out) const {
        const double t0 = curve.tMin();
        const double t1 = curve.tMax();
        const double span = t1 - t0;
        const double tp = curve.closestPoint(p, 1e-14);
        const double tq = curve.closestPoint(q, 1e-14);
        const bool closed = (curve.evaluate(t0) - curve.evaluate(t1)).length() <= 1e-12 * m_scale;
        const auto at = [&](double t) {
            if (closed) {
                t = t0 + std::fmod(t - t0, span);
                if (t < t0) t += span;
            }
            return curve.evaluate(t);
        };
        const Vec3 middle = (p + q) * 0.5;
        const double chord = (q - p).length();
        double d = tq - tp;
        if (closed) {
            const double other = d > 0.0 ? d - span : d + span;
            if ((at(tp + 0.5 * other) - middle).length() < (at(tp + 0.5 * d) - middle).length()) {
                d = other;
            }
        }
        // A curve that does not run between the ends is not this edge's.
        if ((curve.evaluate(tp) - p).length() > 1e-6 * std::max(chord, 1e-300) ||
            (curve.evaluate(tq) - q).length() > 1e-6 * std::max(chord, 1e-300) ||
            (at(tp + 0.5 * d) - middle).length() > chord) {
            return false;
        }
        // Each point where the chord's point is nearest the curve, as a
        // face's points are put where they are nearest its surface: points
        // even in the curve's parameter sat off those beside them by an
        // amount no refinement shrank (a full circle's quarter is far from
        // even in angle), and the error fell only as 1/m.
        const auto derivative = [&](double t, int order) {
            if (closed) {
                t = t0 + std::fmod(t - t0, span);
                if (t < t0) t += span;
            }
            return curve.derivative(t, order);
        };
        for (int i = 1; i < m; ++i) {
            const Vec3 x = p + (q - p) * (static_cast<double>(i) / m);
            double t = tp + d * i / m;
            for (int iter = 0; iter < 50; ++iter) {
                const Vec3 r = at(t) - x;
                const Vec3 d1 = derivative(t, 1);
                const double slope = d1.dot(d1) + r.dot(derivative(t, 2));
                if (!(slope > 0.0)) break;
                const double step = r.dot(d1) / slope;
                t -= step;
                if (!closed) t = std::clamp(t, t0, t1);
                if (std::abs(step) <= 1e-15 * span) break;
            }
            out[static_cast<size_t>(i)] = at(t);
        }
        return true;
    }

    /// The points of a half-edge's edge in the half-edge's direction.
    std::vector<Vec3> samplesAlong(const HalfEdge* he) const {
        std::vector<Vec3> s = m_samples.at(he->edge);
        if (he != he->edge->halfEdge) std::reverse(s.begin(), s.end());
        return s;
    }

    void measureFace(const FaceInfo& info, int m, detail::Measures& out) {
        if (info.ideal == nullptr) {
            measureFlat(info, out);
        } else {
            measureCurved(info, m, out);
        }
    }

    /// A flat face: its refined loops, fanned for the volume; its area, the
    /// net vector area of its loops (or, not flat after all, as modelled).
    void measureFlat(const FaceInfo& info, detail::Measures& out) {
        std::vector<std::vector<Vec3>> loops;
        Vec3 vectorArea;
        bool planar = true;
        for (const auto& loop : info.loops) {
            std::vector<Vec3> points;
            for (const HalfEdge* he : loop) {
                if (he->edge == nullptr || m_samples.count(he->edge) == 0) {
                    points.push_back(he->origin->point);
                    continue;
                }
                const auto s = samplesAlong(he);
                points.insert(points.end(), s.begin(), s.end() - 1);
            }
            if (points.size() < 3) continue;
            for (size_t k = 1; k + 1 < points.size(); ++k) {
                out.addVolume(points[0], points[k], points[k + 1]);
            }
            for (const Vec3& x : points) {
                planar = planar &&
                         std::abs((x - info.plane.point).dot(info.plane.normal)) <= 1e-9 * m_scale;
            }
            vectorArea += vectorArea2(points);
            loops.push_back(std::move(points));
        }
        if (loops.empty()) return;
        if (planar) {
            out.area += 0.5 * vectorArea.length();
            return;
        }
        std::vector<std::vector<Vec3>> holes(loops.begin() + 1, loops.end());
        const auto polygon = BoundaryMesh::keyholePolygon(loops.front(), std::move(holes));
        for (const auto& t : BoundaryMesh::triangulatePolygon(polygon)) {
            out.area += 0.5 * (t[1] - t[0]).cross(t[2] - t[0]).length();
        }
    }

    /// A face with a curved ideal: its polygon triangulated, each triangle
    /// cut into m² and put on the surface, its sides on its edges' points.
    void measureCurved(const FaceInfo& info, int m, detail::Measures& out) {
        const geo::NurbsSurface& s = *info.ideal;
        std::map<std::pair<PointKey, PointKey>, std::vector<Vec3>> sides;
        std::vector<std::vector<Vec3>> loops;
        for (const auto& loop : info.loops) {
            std::vector<Vec3> points;
            for (const HalfEdge* he : loop) {
                points.push_back(he->origin->point);
                if (he->edge != nullptr && m_samples.count(he->edge) != 0) {
                    sides[{keyOf(he->origin->point), keyOf(he->next->origin->point)}] =
                        samplesAlong(he);
                }
            }
            if (points.size() >= 3) loops.push_back(std::move(points));
        }
        if (loops.empty()) return;
        std::vector<Vec3> polygon = loops.front();
        if (loops.size() > 1) {
            polygon = BoundaryMesh::keyholePolygon(
                loops.front(), std::vector<std::vector<Vec3>>(loops.begin() + 1, loops.end()));
        }

        // A side inside the face, the same for the two triangles on it.
        std::map<std::pair<PointKey, PointKey>, std::vector<Vec3>> inside;
        const auto side = [&](const Vec3& a, const Vec3& b) {
            const auto found = sides.find({keyOf(a), keyOf(b)});
            if (found != sides.end()) return found->second;
            const bool flip = keyOf(b) < keyOf(a);
            const auto key =
                flip ? std::make_pair(keyOf(b), keyOf(a)) : std::make_pair(keyOf(a), keyOf(b));
            auto it = inside.find(key);
            if (it == inside.end()) {
                const Vec3& from = flip ? b : a;
                const Vec3& to = flip ? a : b;
                std::vector<Vec3> pts(static_cast<size_t>(m) + 1);
                pts.front() = from;
                pts.back() = to;
                const UV fromStart = start(s, {&from, &to});
                const UV toStart = start(s, {&to, &from});
                for (int i = 1; i < m; ++i) {
                    UV uv = 2 * i <= m ? fromStart : toStart;
                    pts[static_cast<size_t>(i)] =
                        onSurface(s, from + (to - from) * (static_cast<double>(i) / m), uv);
                }
                it = inside.emplace(key, std::move(pts)).first;
            }
            std::vector<Vec3> pts = it->second;
            if (flip) std::reverse(pts.begin(), pts.end());
            return pts;
        };

        for (const auto& t : BoundaryMesh::triangulatePolygon(polygon)) {
            // A corner where the surface degenerates (a cone's apex, a
            // sphere's pole) comes first, to be the grid's apex.
            int turn = 0;
            for (int k = 0; k < 3; ++k) {
                if (degenerate(s, seed(s, t[static_cast<size_t>(k)]))) turn = k;
            }
            const Vec3& a = t[static_cast<size_t>(turn)];
            const Vec3& b = t[static_cast<size_t>((turn + 1) % 3)];
            const Vec3& c = t[static_cast<size_t>((turn + 2) % 3)];
            if (degenerate(s, seed(s, a))) {
                refineFromApex(s, a, b, c, side(a, b), side(b, c), side(c, a), m, out);
            } else {
                refine(s, a, b, c, side(a, b), side(b, c), side(c, a), m, out);
            }
        }
    }

    /// Triangle (a, b, c) cut into m² triangles, a barycentric grid, its
    /// sides the points @p ab, @p bc and @p ca and the rest on @p s.
    void refine(const geo::NurbsSurface& s, const Vec3& a, const Vec3& b, const Vec3& c,
                const std::vector<Vec3>& ab, const std::vector<Vec3>& bc,
                const std::vector<Vec3>& ca, int m, detail::Measures& out) {
        const auto index = [m](int i, int j) {
            // Row j of the grid holds m + 1 - j points.
            return static_cast<size_t>(j * (m + 1) - j * (j - 1) / 2 + i);
        };
        std::vector<Vec3>& grid = m_grid;
        grid.resize(index(0, m) + 1);
        // Each corner's start, or, where the surface degenerates, the next's.
        const UV at[3] = {start(s, {&a, &b, &c}), start(s, {&b, &c, &a}), start(s, {&c, &a, &b})};
        for (int j = 0; j <= m; ++j) {
            for (int i = 0; i + j <= m; ++i) {
                Vec3& x = grid[index(i, j)];
                if (j == 0) {
                    x = ab[static_cast<size_t>(i)];
                } else if (i + j == m) {
                    x = bc[static_cast<size_t>(j)];
                } else if (i == 0) {
                    x = ca[static_cast<size_t>(m - j)];
                } else {
                    const double wb = static_cast<double>(i) / m;
                    const double wc = static_cast<double>(j) / m;
                    const double wa = 1.0 - wb - wc;
                    // From the nearest corner.
                    UV uv = at[wa >= wb && wa >= wc ? 0 : (wb >= wc ? 1 : 2)];
                    x = onSurface(s, a * wa + b * wb + c * wc, uv);
                }
            }
        }
        for (int j = 0; j < m; ++j) {
            for (int i = 0; i + j < m; ++i) {
                out.add(grid[index(i, j)], grid[index(i + 1, j)], grid[index(i, j + 1)]);
                if (i + j + 1 < m) {
                    out.add(grid[index(i + 1, j)], grid[index(i + 1, j + 1)],
                            grid[index(i, j + 1)]);
                }
            }
        }
    }

    /// Triangle (a, b, c) whose corner @p a is where the surface
    /// degenerates, cut as a quad collapsed there: m + 1 rows from side bc to
    /// a, m + 1 points each. A barycentric grid has fewer points in each row
    /// nearer a, so its rows there turned through the same angle in ever
    /// fewer steps, and the error fell as log(m) / m², which no
    /// extrapolation in powers of 1/m² removes.
    void refineFromApex(const geo::NurbsSurface& s, const Vec3& a, const Vec3& b, const Vec3& c,
                        const std::vector<Vec3>& ab, const std::vector<Vec3>& bc,
                        const std::vector<Vec3>& ca, int m, detail::Measures& out) {
        const auto index = [m](int i, int j) { return static_cast<size_t>(j * (m + 1) + i); };
        std::vector<Vec3>& grid = m_grid;
        grid.resize(index(m, m) + 1);
        const UV fromB = start(s, {&b, &c});
        const UV fromC = start(s, {&c, &b});
        for (int j = 0; j <= m; ++j) {
            for (int i = 0; i <= m; ++i) {
                Vec3& x = grid[index(i, j)];
                if (j == m) {
                    x = a;
                } else if (j == 0) {
                    x = bc[static_cast<size_t>(i)];
                } else if (i == 0) {
                    x = ab[static_cast<size_t>(m - j)];
                } else if (i == m) {
                    x = ca[static_cast<size_t>(j)];
                } else {
                    const double along = static_cast<double>(i) / m;
                    const double up = static_cast<double>(j) / m;
                    const Vec3 base = b + (c - b) * along;
                    UV uv = 2 * i <= m ? fromB : fromC;
                    x = onSurface(s, base + (a - base) * up, uv);
                }
            }
        }
        for (int j = 0; j < m; ++j) {
            for (int i = 0; i < m; ++i) {
                const Vec3& p00 = grid[index(i, j)];
                const Vec3& p10 = grid[index(i + 1, j)];
                const Vec3& p11 = grid[index(i + 1, j + 1)];
                const Vec3& p01 = grid[index(i, j + 1)];
                out.add(p00, p10, p11);
                if (j + 1 < m) out.add(p00, p11, p01);  // the last row closes on a
            }
        }
    }

    const topo::Solid& m_solid;
    std::unordered_map<const Face*, FaceInfo> m_faces;
    std::unordered_map<const Edge*, std::vector<Vec3>> m_samples;
    std::map<std::tuple<const void*, double, double, double>, UV> m_seeds;
    std::map<std::tuple<const void*, double, double>, bool> m_degenerate;
    std::unordered_map<const geo::NurbsSurface*, double> m_sizes;
    std::vector<Vec3> m_grid;
    double m_scale = 0.0;
    bool m_curved = false;
    int m_parted = 0;
};

detail::Measures extrapolate(const detail::Measures& fine, const detail::Measures& coarse,
                             double factor) {
    detail::Measures out;
    for (size_t k = 0; k < out.sums.size(); ++k) {
        out.sums[k] = fine.sums[k] + (fine.sums[k] - coarse.sums[k]) / factor;
    }
    out.area = fine.area + (fine.area - coarse.area) / factor;
    return out;
}

}  // namespace

IdealMassProperties MassPropertiesCalculator::computeIdeal(const topo::Solid& solid,
                                                           const Material* material,
                                                           double tolerance) {
    IdealMassProperties result;
    const double density = material ? material->density : 1.0;
    IdealSolid ideal(solid);
    result.withoutIdeal = ideal.withoutIdeal();
    if (!ideal.curved()) {
        result.properties = compute(solid, material);
        result.exact = result.withoutIdeal.empty();
        return result;
    }

    // Romberg: row k holds the measure at m = 2^k and its extrapolations.
    std::vector<std::vector<detail::Measures>> table;
    int parted = 0;
    for (int m = 1; m <= kMaxRefinement; m *= 2) {
        if (m > 1 && ideal.triangles(m) > kTriangleBudget) break;
        std::vector<detail::Measures> row{ideal.measure(m)};
        parted = ideal.partedEdges();
        for (size_t j = 1; j <= table.size(); ++j) {
            row.push_back(extrapolate(row[j - 1], table.back()[j - 1], std::pow(4.0, j) - 1.0));
        }
        result.refinement = m;
        table.push_back(std::move(row));
        if (table.size() >= 2) {
            // The last correction the extrapolation made: the error of the
            // value before it, and more than this one's (the sequence settles
            // as 1/m^2, and each column extrapolates a power further).
            const auto& best = table.back().back();
            const auto& before = table.back()[table.back().size() - 2];
            const double volume = std::abs(best.sums[0]);
            result.estimatedError =
                std::max(std::abs(best.sums[0] - before.sums[0]) / std::max(volume, 1e-300),
                         std::abs(best.area - before.area) / std::max(best.area, 1e-300));
        }
        // Where ideals part, the sliver between them is no nearer the design
        // for being finer: stop short.
        if (parted > 0 && m >= 8) break;
        if (table.size() >= 4 && result.estimatedError <= tolerance) break;
    }
    result.properties = detail::finish(table.back().back(), density);
    result.partedEdges = parted;
    result.exact = result.withoutIdeal.empty() && parted == 0;
    return result;
}

}  // namespace hz::model
