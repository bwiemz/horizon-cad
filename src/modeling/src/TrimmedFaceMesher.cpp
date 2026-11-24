#include "horizon/modeling/TrimmedFaceMesher.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include "MeshEdgeSampler.h"
#include "horizon/geometry/Triangulate2D.h"
#include "horizon/geometry/curves/Curve2D.h"
#include "horizon/geometry/surfaces/Surface.h"
#include "horizon/math/Constants.h"
#include "horizon/math/ModelTolerance.h"
#include "horizon/math/Vec2.h"
#include "horizon/topology/LoopUnwrap.h"
namespace hz::model {

namespace {

using detail::angleBetween;
using detail::distanceToSegment;
using detail::EdgeSamples;
using detail::Plan;
using detail::Segment;
using detail::Spacing;
using math::Vec2;
using math::Vec3;
using topo::Face;
using topo::HalfEdge;

// Rounds of refinement. A front of splits can move one cell a round (each split leaves a stray
// diagonal in the next cell along a band), and on cone patches the slowest took 131 rounds;
// rounds that change little are cheap, as triangles found to fit are not measured again.
constexpr int kRefineRounds = 256;
constexpr double kKeepAway = 0.5;    // grid points nearer the boundary (in spacings) go
constexpr int kBoundaryRounds = 32;  // rounds of halving edges that faces ask to have halved
constexpr std::uint32_t kUnused = std::numeric_limits<std::uint32_t>::max();
constexpr int kFits = -1;     // a triangle that needs no point
constexpr int kCentroid = 3;  // one that needs its centroid; 0 to 2 name an edge to split

std::uint64_t edgeKey(std::uint32_t a, std::uint32_t b) {
    return (static_cast<std::uint64_t>(std::min(a, b)) << 32) | std::max(a, b);
}

using TriangleKey = std::array<std::uint32_t, 3>;

/// A triangle's vertices from its lowest index on, in the same turn: one key however written.
TriangleKey canonical(const TriangleKey& t) {
    const std::size_t k = t[0] < t[1] ? (t[0] < t[2] ? 0 : 2) : (t[1] < t[2] ? 1 : 2);
    return {t[k], t[(k + 1) % 3], t[(k + 2) % 3]};
}

struct TriangleHash {
    std::size_t operator()(const TriangleKey& t) const {
        std::uint64_t h = t[0];
        h = h * 0x9E3779B97F4A7C15ULL ^ t[1];
        h = h * 0x9E3779B97F4A7C15ULL ^ t[2];
        return static_cast<std::size_t>(h ^ (h >> 29));
    }
};

/// The distance from @p p to the triangle abc. Written with cross products: the usual
/// barycentric form multiplies dot products together, and on a long thin triangle (a sliver
/// face's) that loses most of the precision.
double distanceToTriangle(const Vec3& p, const Vec3& a, const Vec3& b, const Vec3& c) {
    const Vec3 n = (b - a).cross(c - a);
    const double ln = n.length();
    if (ln > 0.0 && (b - a).cross(p - a).dot(n) >= 0.0 && (c - b).cross(p - b).dot(n) >= 0.0 &&
        (a - c).cross(p - c).dot(n) >= 0.0) {
        return std::fabs((p - a).dot(n)) / ln;
    }
    return std::min(
        {distanceToSegment(p, a, b), distanceToSegment(p, b, c), distanceToSegment(p, c, a)});
}

/// A face's loops (outer first) in unwrapped (u, v), with each sample's position and the edge
/// segment from it to the next sample.
struct Boundary {
    std::vector<std::vector<Vec2>> uv;
    std::vector<std::vector<Vec3>> xyz;
    std::vector<std::vector<Segment>> segment;
};

/// Meshes the faces of a solid from its edges' samples.
class FaceMesher {
public:
    FaceMesher(const topo::Solid& solid, const MeshOptions& options,
               const detail::EdgeSampler& edges)
        : solid_(solid), opt_(options), edges_(edges) {}

    /// @p f meshed, or why not. With @p halve not empty, the face needs those boundary
    /// segments halved first and its mesh is to be made again.
    FaceMesh meshFace(const Face& f, std::vector<Segment>& halve) const {
        FaceMesh out;
        out.face = &f;
        if (!f.exactSurface) {
            out.problem = "the face has no exact surface";
            return out;
        }
        Boundary b;
        out.problem = boundaryOf(f, b);
        if (out.problem.empty()) out.problem = triangulate(f, b, out, halve);
        if (!out.problem.empty()) out = FaceMesh{&f, {}, {}, {}, out.problem};
        return out;
    }

private:
    /// The face's loops in (u, v), unwrapped and aligned as the validator does it, each sample
    /// at its pcurve parameter; empty, or why the face cannot be laid out.
    std::string boundaryOf(const Face& f, Boundary& b) const {
        const geo::Surface& s = *f.exactSurface;
        const double pu = topo::uPeriod(s);
        const double pv = topo::vPeriod(s);
        std::vector<const topo::Wire*> wires{f.outerLoop};
        wires.insert(wires.end(), f.innerLoops.begin(), f.innerLoops.end());
        topo::LoopPolygon outer;
        for (std::size_t w = 0; w < wires.size(); ++w) {
            std::vector<const HalfEdge*> ring;
            if (!wires[w] ||
                !topo::loopHalfEdges(f, *wires[w], 2 * solid_.edges().size() + 1, ring)) {
                return "a loop of the face does not close";
            }
            std::vector<double> allowances;
            for (const HalfEdge* h : ring) {
                const EdgeSamples* es = edges_.samples(h->edge);
                if (es == nullptr) return "a half-edge of the face has no edge";
                if (!es->problem.empty()) return es->problem;
                if (!h->pcurve) return "a half-edge of the face has no pcurve";
                if (edges_.parameters(h) == nullptr)
                    return "a half-edge of the face was not sampled";
                allowances.push_back(topo::joinAllowance(*h, edges_.tolerance()));
            }
            topo::LoopPolygon poly;
            if (!topo::unwrapLoop(ring, topo::kLoopSamples, s, pu, pv, allowances, poly)) {
                return "a loop of the face closes only after winding round the surface";
            }
            if (w == 0) {
                outer = poly;
            } else {
                topo::alignLoop(poly, outer, pu, pv);
            }
            std::vector<Vec2> uv;
            std::vector<Vec3> xyz;
            std::vector<Segment> segment;
            for (std::size_t k = 0; k < ring.size(); ++k) {
                const HalfEdge* h = ring[k];
                const Vec2 shift = poly[k * (topo::kLoopSamples - 1)].shift;
                const std::vector<Vec3>& pts = edges_.samples(h->edge)->points;
                const std::vector<double>& ps = *edges_.parameters(h);
                const std::size_t n = pts.size() - 1;
                const bool forward = h == h->edge->halfEdge;
                for (std::size_t j = 0; j < n; ++j) {  // the last is the next one's first
                    uv.push_back(h->pcurve->evaluate(ps[j]) + shift);
                    xyz.push_back(pts[forward ? j : n - j]);
                    segment.emplace_back(h->edge, forward ? j : n - 1 - j);
                }
            }
            b.uv.push_back(std::move(uv));
            b.xyz.push_back(std::move(xyz));
            b.segment.push_back(std::move(segment));
        }
        return {};
    }

    /// Interior points (in w) on a grid of the spacing over [lo, hi], without those within
    /// kKeepAway spacings of a boundary segment, which would make slivers along it. A torus's
    /// rows each take their own u step (Spacing::rowSteps); a cone's are staggered
    /// (Spacing::staggered).
    std::string gridPoints(const Spacing& sp, const Vec2& lo, const Vec2& hi,
                           const std::vector<std::vector<Vec2>>& loopsW,
                           std::vector<Vec2>& out) const {
        const double nu = std::ceil((hi.x - lo.x) / sp.du);
        const double nv = std::ceil((hi.y - lo.y) / sp.dv);
        // Checked before any count is cast or stored: a box that is not a number, and a step of 0
        // (a tolerance too fine for doubles) or a huge count, fail here and say why.
        if (std::isnan(nu) || std::isnan(nv)) return "the face's grid spacing is not a number";
        if (!(nu > 1.0) || !(nv > 1.0)) return {};
        const double cap = 0.5 * static_cast<double>(opt_.maxTrianglesPerFace);
        if (!(nu - 1.0 <= cap) || !((sp.staggered ? nv : nv - 1.0) <= cap)) {
            return "the face's grid alone needs more triangles than the cap";
        }
        const double stepV = (hi.y - lo.y) / nv;
        // Row j is at v = lo.y + stepV (j + rowFirst); its point i at u = lo.x + step (i + first),
        // for i below count.
        const double rowFirst = sp.staggered ? 0.5 : 1.0;
        const auto cv = static_cast<std::size_t>(sp.staggered ? nv : nv - 1.0);
        struct Row {
            double step;
            double first;
            std::size_t count;
        };
        std::vector<Row> rows;
        rows.reserve(cv);
        double points = 0.0;
        for (std::size_t j = 0; j < cv; ++j) {
            const double n = j < sp.rowSteps.size()
                                 ? std::clamp(std::ceil((hi.x - lo.x) / sp.rowSteps[j]), 1.0, nu)
                                 : nu;
            const bool shifted = sp.staggered && (j % 2 == 1) != sp.shiftFirst;
            rows.push_back({(hi.x - lo.x) / n, shifted ? 0.5 : 1.0,
                            static_cast<std::size_t>(shifted ? n : n - 1.0)});
            points += static_cast<double>(rows.back().count);
        }
        if (!(points <= cap)) return "the face's grid alone needs more triangles than the cap";
        const auto pointAt = [&](std::size_t i, std::size_t j) {
            const Row& r = rows[j];
            return Vec2{(lo.x + r.step * (static_cast<double>(i) + r.first)) * sp.su,
                        (lo.y + stepV * (static_cast<double>(j) + rowFirst)) * sp.sv};
        };
        // A staggered row's ends are half a step from the face's side by design.
        const double keepAway = sp.staggered ? 0.5 * kKeepAway : kKeepAway;
        std::vector<std::vector<bool>> keep(cv);
        double widest = 0.0;  // the largest keep-away distance of any row
        for (std::size_t j = 0; j < cv; ++j) {
            keep[j].assign(rows[j].count, true);
            widest = std::max(widest, keepAway * std::min(rows[j].step * sp.su, stepV * sp.sv));
        }
        // The grid indices whose lines lie within [x0, x1] (in w) or a line beyond, clamped,
        // before any cast: ends that are NaN or infinite (a loop that is not finite) included.
        const auto lines = [](double x0, double x1, double from, double step, std::size_t count) {
            const double floorAt = std::floor((x0 - from) / step) - 2.0;
            const double ceilAt = std::ceil((x1 - from) / step);
            const double first = floorAt >= 0.0 ? floorAt : 0.0;
            const double top = static_cast<double>(count) - 1.0;
            const double last = ceilAt <= top ? ceilAt : top;
            if (!(first <= last)) return std::pair<std::size_t, std::size_t>{0, 0};
            return std::pair<std::size_t, std::size_t>{static_cast<std::size_t>(first),
                                                       static_cast<std::size_t>(last) + 1};
        };
        for (const auto& loop : loopsW) {
            for (std::size_t k = 0; k < loop.size(); ++k) {
                const Vec2& a = loop[k];
                const Vec2& b = loop[(k + 1) % loop.size()];
                const auto [j0, j1] =
                    lines(std::min(a.y, b.y) - widest, std::max(a.y, b.y) + widest,
                          (lo.y + stepV * (rowFirst - 1.0)) * sp.sv, stepV * sp.sv, cv);
                for (std::size_t j = j0; j < j1; ++j) {
                    const Row& r = rows[j];
                    const double away = keepAway * std::min(r.step * sp.su, stepV * sp.sv);
                    const auto [i0, i1] =
                        lines(std::min(a.x, b.x) - away, std::max(a.x, b.x) + away,
                              (lo.x + r.step * (r.first - 1.0)) * sp.su, r.step * sp.su, r.count);
                    for (std::size_t i = i0; i < i1; ++i) {
                        if (distanceToSegment(pointAt(i, j), a, b) < away) keep[j][i] = false;
                    }
                }
            }
        }
        // Column by column, as a grid of equal rows always was: the triangulation's insertion
        // order, and so its choice among cocircular points, follows the input's.
        std::size_t columns = 0;
        for (const Row& r : rows) columns = std::max(columns, r.count);
        for (std::size_t i = 0; i < columns; ++i) {
            for (std::size_t j = 0; j < cv; ++j) {
                if (i < rows[j].count && keep[j][i]) out.push_back(pointAt(i, j));
            }
        }
        return {};
    }

    /// kFits when the triangle's (u, v) centroid, and the midpoints of its edges that are not
    /// boundary segments, map within the chord tolerance of it, and it faces the way the
    /// surface's du x dv does at its centroid. Otherwise where it needs a point: an edge k (from
    /// vertex k to k + 1) whose midpoint strays, as the centroid would leave that edge in place,
    /// or kCentroid.
    int misfit(const geo::Surface& s, const std::array<Vec2, 3>& uv, const std::array<Vec3, 3>& p,
               const std::array<bool, 3>& onBoundary) const {
        const Vec3 n = (p[1] - p[0]).cross(p[2] - p[0]);
        const double ln = n.length();
        if (!(ln > 0.0)) return kCentroid;
        const auto near = [&](const Vec2& q) {
            return distanceToTriangle(s.evaluate(q.x, q.y), p[0], p[1], p[2]) <=
                   opt_.chordTolerance;
        };
        for (int k = 0; k < 3; ++k) {
            // A boundary segment's midpoint was held to its chord when the edge was sampled.
            const auto a = static_cast<std::size_t>(k);
            if (!onBoundary[a] && !near((uv[a] + uv[(a + 1) % 3]) * 0.5)) return k;
        }
        const Vec2 c = (uv[0] + uv[1] + uv[2]) / 3.0;
        if (!near(c)) return kCentroid;
        const Vec3 sn = s.normal(c.x, c.y);  // zero at a pole or apex: nothing to keep to
        return sn.lengthSquared() == 0.0 || n.dot(sn) > 0.0 ? kFits : kCentroid;
    }

    /// Triangulates the face's region, refining until every triangle fits, into @p out; or
    /// stops at the first round that needs boundary segments halved, listed in @p halve. Each
    /// round's points go into the triangulation in place; a halving changes the boundary, and
    /// meshSolid then meshes the face again from its grid.
    std::string triangulate(const Face& f, const Boundary& b, FaceMesh& out,
                            std::vector<Segment>& halve) const {
        const geo::Surface& s = *f.exactSurface;
        const Plan* planned = edges_.plan(&f);
        if (planned == nullptr) return "the face's grid was not planned";
        const Vec2& lo = planned->lo;
        const Vec2& hi = planned->hi;
        const Spacing& sp = planned->spacing;
        std::vector<std::vector<Vec2>> loopsW;
        std::vector<Vec2> uvs;
        std::vector<Vec3> xyzs;
        for (std::size_t l = 0; l < b.uv.size(); ++l) {
            loopsW.emplace_back();
            for (const Vec2& p : b.uv[l]) loopsW.back().push_back({p.x * sp.su, p.y * sp.sv});
            uvs.insert(uvs.end(), b.uv[l].begin(), b.uv[l].end());
            xyzs.insert(xyzs.end(), b.xyz[l].begin(), b.xyz[l].end());
        }
        const std::vector<std::vector<Vec2>> holesW(loopsW.begin() + 1, loopsW.end());
        std::vector<Vec2> interior;
        std::string problem = gridPoints(sp, lo, hi, loopsW, interior);
        if (!problem.empty()) return problem;

        const std::size_t boundaryCount = uvs.size();
        geo::RegionTriangulation region(loopsW[0], holesW, interior);
        const geo::Triangulation2D& t = region.result();
        if (!t.problem.empty()) return "the face's (u, v) region: " + t.problem;
        for (std::size_t i = 0; i < boundaryCount; ++i) {
            const Vec3& a = xyzs[i];
            const Vec3& c = xyzs[t.boundaryVertex[i]];
            if (a.x != c.x || a.y != c.y || a.z != c.z) {
                return "two boundary points of the face meet in (u, v) at different positions";
            }
        }
        std::unordered_map<std::uint64_t, Segment> walls;  // boundary segments
        std::size_t base = 0;
        for (std::size_t l = 0; l < loopsW.size(); ++l) {
            const std::size_t n = loopsW[l].size();
            for (std::size_t i = 0; i < n; ++i) {
                walls.emplace(
                    edgeKey(t.boundaryVertex[base + i], t.boundaryVertex[base + (i + 1) % n]),
                    b.segment[l][i]);
            }
            base += n;
        }
        std::vector<Vec2> uv(uvs);
        std::vector<Vec3> xyz(xyzs);
        // Triangles found to fit. A triangle's verdict depends on its vertices alone (they fix
        // which of its edges are boundary segments), so one that refinement leaves in place is
        // not measured again; the points each round adds are the same.
        std::unordered_set<TriangleKey, TriangleHash> fits;
        std::size_t kept = 0;
        for (int round = 0;; ++round) {
            if (t.triangles.size() > opt_.maxTrianglesPerFace) {
                return "the face needs more triangles than the cap of " +
                       std::to_string(opt_.maxTrianglesPerFace);
            }
            if (round > 0 && t.points.size() <= kept) return "refining the face stalled";
            kept = t.points.size();
            uv.reserve(t.points.size());
            xyz.reserve(t.points.size());
            for (std::size_t i = uv.size(); i < t.points.size(); ++i) {
                uv.push_back({t.points[i].x / sp.su, t.points[i].y / sp.sv});
                xyz.push_back(s.evaluate(uv.back().x, uv.back().y));
            }
            std::vector<Vec2> added;
            std::unordered_set<std::uint64_t> split;
            for (const auto& tri : t.triangles) {
                const TriangleKey key = canonical(tri);
                if (fits.count(key) != 0) continue;
                const std::array<Vec2, 3> tuv = {uv[tri[0]], uv[tri[1]], uv[tri[2]]};
                const std::array<Vec3, 3> txyz = {xyz[tri[0]], xyz[tri[1]], xyz[tri[2]]};
                std::array<bool, 3> onBoundary{};
                for (std::size_t k = 0; k < 3; ++k) {
                    onBoundary[k] = walls.count(edgeKey(tri[k], tri[(k + 1) % 3])) != 0;
                }
                const int m = misfit(s, tuv, txyz, onBoundary);
                if (m == kFits) {
                    fits.insert(key);
                } else if (m == kCentroid) {
                    added.push_back((t.points[tri[0]] + t.points[tri[1]] + t.points[tri[2]]) / 3.0);
                } else if (m != kFits) {
                    const std::uint32_t a = tri[static_cast<std::size_t>(m)];
                    const std::uint32_t b = tri[static_cast<std::size_t>(m + 1) % 3];
                    if (split.insert(edgeKey(a, b)).second) {
                        added.push_back((t.points[a] + t.points[b]) * 0.5);
                    }
                }
            }
            // Triangles that share an edge turn by no more than maxAngle. Two that turn more are
            // each made smaller by splitting their longest edge (the edge they share may be
            // short). A boundary segment is split by halving its edge for both its faces: a
            // face cannot split its boundary alone, and refining beside a curved chord instead
            // only makes slivers that lean towards the curve's osculating plane.
            std::vector<Vec3> normals;
            normals.reserve(t.triangles.size());
            for (const auto& tri : t.triangles) {
                normals.push_back((xyz[tri[1]] - xyz[tri[0]]).cross(xyz[tri[2]] - xyz[tri[0]]));
            }
            std::unordered_map<std::uint64_t, std::size_t> sides;
            std::vector<bool> turned(t.triangles.size(), false);
            for (std::size_t i = 0; i < t.triangles.size(); ++i) {
                for (std::size_t k = 0; k < 3; ++k) {
                    const std::uint64_t key =
                        edgeKey(t.triangles[i][k], t.triangles[i][(k + 1) % 3]);
                    if (walls.count(key) != 0) continue;
                    const auto [other, first] = sides.emplace(key, i);
                    if (!first &&
                        angleBetween(normals[other->second], normals[i]) > opt_.maxAngle) {
                        turned[other->second] = turned[i] = true;
                    }
                }
            }
            for (std::size_t i = 0; i < t.triangles.size(); ++i) {
                if (!turned[i]) continue;
                const auto& tri = t.triangles[i];
                double longest = -1.0;
                std::uint64_t pick = 0;
                for (std::size_t k = 0; k < 3; ++k) {
                    const double length = (t.points[tri[k]] - t.points[tri[(k + 1) % 3]]).length();
                    if (length > longest) {
                        longest = length;
                        pick = edgeKey(tri[k], tri[(k + 1) % 3]);
                    }
                }
                const auto wall = walls.find(pick);
                if (wall != walls.end()) {
                    halve.push_back(wall->second);
                } else if (split.insert(pick).second) {
                    const auto a = static_cast<std::size_t>(pick >> 32);
                    const auto c = static_cast<std::size_t>(pick & 0xFFFFFFFFULL);
                    added.push_back((t.points[a] + t.points[c]) * 0.5);
                }
            }
            if (!halve.empty()) return "the face's boundary is to be halved";
            if (added.empty()) {
                emit(f, t, uv, xyz, out);
                return {};
            }
            if (round == kRefineRounds) return "refining the face did not meet the tolerance";
            // A point the triangulation refuses (on a boundary segment, or on a point) is dropped,
            // as before; a round that keeps none has stalled.
            for (const Vec2& p : added) region.insert(p);
        }
    }

    /// The triangles' vertices, compacted, with the surface's outward normals; triangles wound
    /// outward.
    static void emit(const Face& f, const geo::Triangulation2D& t, const std::vector<Vec2>& uv,
                     const std::vector<Vec3>& xyz, FaceMesh& out) {
        const geo::Surface& s = *f.exactSurface;
        const double sign = f.reversed ? -1.0 : 1.0;
        std::vector<std::uint32_t> index(t.points.size(), kUnused);
        for (const auto& tri : t.triangles) {
            std::array<std::uint32_t, 3> v{};
            for (std::size_t k = 0; k < 3; ++k) {
                if (index[tri[k]] == kUnused) {
                    index[tri[k]] = static_cast<std::uint32_t>(out.points.size());
                    out.points.push_back(xyz[tri[k]]);
                    out.normals.push_back(s.normal(uv[tri[k]].x, uv[tri[k]].y) * sign);
                }
                v[k] = index[tri[k]];
            }
            // Counter-clockwise in (u, v) faces along du x dv: outward unless reversed.
            if (f.reversed) std::swap(v[1], v[2]);
            out.triangles.push_back(v);
        }
        // Where the surface has no normal (a pole, an apex), the triangles' own.
        for (std::size_t i = 0; i < out.normals.size(); ++i) {
            if (out.normals[i].lengthSquared() > 0.0) continue;
            Vec3 sum;
            for (const auto& tri : out.triangles) {
                if (tri[0] != i && tri[1] != i && tri[2] != i) continue;
                sum += (out.points[tri[1]] - out.points[tri[0]])
                           .cross(out.points[tri[2]] - out.points[tri[0]]);
            }
            const double len = sum.length();
            if (len > 0.0) out.normals[i] = sum / len;
        }
    }

    const topo::Solid& solid_;
    const MeshOptions& opt_;
    const detail::EdgeSampler& edges_;
};

}  // namespace

std::vector<FaceMesh> TrimmedFaceMesher::meshSolid(const topo::Solid& solid,
                                                   const MeshOptions& options) {
    std::vector<FaceMesh> out;
    std::string invalid;
    if (!(options.chordTolerance > 0.0) || !std::isfinite(options.chordTolerance)) {
        invalid = "the chord tolerance must be positive and finite";
    } else if (!(options.maxAngle > 0.0) || !(options.maxAngle <= math::kPi)) {
        invalid = "the angle must be above 0 and at most pi";
    } else if (options.maxTrianglesPerFace == 0) {
        invalid = "the triangle cap must be above 0";
    }
    std::vector<const Face*> faces;
    for (const Face& f : solid.faces()) {
        if (f.alive) faces.push_back(&f);
    }
    if (!invalid.empty()) {
        for (const Face* f : faces) out.push_back(FaceMesh{f, {}, {}, {}, invalid});
        return out;
    }
    detail::EdgeSampler edges(solid, options);
    edges.sampleEdges();
    const FaceMesher mesher(solid, options, edges);
    out.resize(faces.size());
    // A face may ask for boundary segments to be halved; that changes the faces on both sides
    // of each, which are meshed again, until none asks.
    std::unordered_set<const Face*> todo(faces.begin(), faces.end());
    for (int round = 0;; ++round) {
        std::vector<detail::Segment> halve;
        std::unordered_set<const Face*> asking;
        for (std::size_t i = 0; i < faces.size(); ++i) {
            if (todo.count(faces[i]) == 0) continue;
            std::vector<detail::Segment> segments;
            out[i] = mesher.meshFace(*faces[i], segments);
            if (!segments.empty()) {
                asking.insert(faces[i]);
                halve.insert(halve.end(), segments.begin(), segments.end());
            }
        }
        if (halve.empty()) break;
        if (round == kBoundaryRounds) {
            for (std::size_t i = 0; i < faces.size(); ++i) {
                if (asking.count(faces[i]) != 0) {
                    out[i] = FaceMesh{
                        faces[i], {}, {}, {}, "halving the face's boundary did not meet the angle"};
                }
            }
            break;
        }
        const std::vector<const Face*> changed = edges.halveSegments(halve);
        todo = std::unordered_set<const Face*>(changed.begin(), changed.end());
    }
    return out;
}

}  // namespace hz::model
