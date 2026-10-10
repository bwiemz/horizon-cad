#include "horizon/modeling/SolidSewer.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>
#include <unordered_map>
#include <vector>

#include "horizon/geometry/curves/NurbsCurve.h"
#include "horizon/geometry/surfaces/NurbsSurface.h"
#include "horizon/math/Constants.h"
#include "horizon/topology/Solid.h"

namespace hz::model {

using hz::math::Vec3;

namespace {

// -- Vertex welding -----------------------------------------------------------

struct CellKey {
    int64_t x, y, z;
    bool operator==(const CellKey& o) const { return x == o.x && y == o.y && z == o.z; }
};

struct CellKeyHash {
    size_t operator()(const CellKey& k) const {
        const auto h = [](int64_t v) { return std::hash<int64_t>()(v); };
        return h(k.x) ^ (h(k.y) << 21) ^ (h(k.z) << 42);
    }
};

/// Merges positions closer than `tol` and hands out stable indices.
///
/// Welded by union-find over every point seen, so which points are one vertex
/// does not depend on the order they arrived in. Welding each point against
/// those before it was order-dependent and not transitive: A and B within the
/// tolerance, B and C within it, A and C not, welds A to B and leaves C on
/// its own — while C first leaves two pairs. A Boolean hands faces to the
/// sewer in whatever order its split produced them, so the same two solids
/// welded to different vertex counts, and a solid's face said it had more
/// vertices than it did.
///
/// Every point within the tolerance of another is a cluster, and the cluster
/// is one vertex at the position nearest the origin among its members, so the
/// choice is the same whichever way round the points came.
class VertexWelder {
public:
    explicit VertexWelder(double tol) : m_tol(tol), m_cell(tol * 2.0) {}

    /// Every position the faces named, one index each, before welding.
    size_t gather(const Vec3& p) {
        const size_t idx = m_gathered.size();
        m_gathered.push_back(p);
        return idx;
    }

    /// Weld, and hand back each gathered index as the index of the vertex it
    /// belongs to. Call once, after every position has been gathered.
    void weld() {
        const size_t n = m_gathered.size();
        m_parent.resize(n);
        for (size_t i = 0; i < n; ++i) {
            m_parent[i] = i;
        }

        // Each point against those in the cells around it, by index, so every
        // pair within the tolerance is joined however they were gathered.
        std::unordered_map<CellKey, std::vector<size_t>, CellKeyHash> grid;
        for (size_t i = 0; i < n; ++i) {
            grid[keyFor(m_gathered[i])].push_back(i);
        }
        for (size_t i = 0; i < n; ++i) {
            const CellKey base = keyFor(m_gathered[i]);
            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dz = -1; dz <= 1; ++dz) {
                        auto it = grid.find(CellKey{base.x + dx, base.y + dy, base.z + dz});
                        if (it == grid.end()) continue;
                        for (const size_t j : it->second) {
                            if (j > i && m_gathered[i].distanceTo(m_gathered[j]) <= m_tol) {
                                join(i, j);
                            }
                        }
                    }
                }
            }
        }

        // One vertex per cluster, taken at the first of its members, and every
        // member handed that vertex's index. First by gathered index, so the
        // choice is the same whichever order the points arrived in.
        m_points.clear();
        m_pointOfCluster.clear();
        m_weldedTo.resize(n);
        for (size_t i = 0; i < n; ++i) {
            const size_t root = find(i);
            const auto [it, fresh] = m_pointOfCluster.try_emplace(root, m_points.size());
            if (fresh) {
                m_points.push_back(m_gathered[i]);
            }
            m_weldedTo[i] = it->second;
        }
    }

    /// The welded index of a gathered one. Only valid after weld().
    size_t index(size_t gathered) const { return m_weldedTo[gathered]; }

    const std::vector<Vec3>& points() const { return m_points; }

private:
    size_t find(size_t i) {
        while (m_parent[i] != i) {
            m_parent[i] = m_parent[m_parent[i]];  // halve the path
            i = m_parent[i];
        }
        return i;
    }

    void join(size_t a, size_t b) {
        const size_t ra = find(a);
        const size_t rb = find(b);
        if (ra != rb) {
            // The lower root wins, so the choice does not depend on the order
            // the pairs were found in either.
            if (ra < rb) {
                m_parent[rb] = ra;
            } else {
                m_parent[ra] = rb;
            }
        }
    }

    CellKey keyFor(const Vec3& p) const {
        return {static_cast<int64_t>(std::floor(p.x / m_cell)),
                static_cast<int64_t>(std::floor(p.y / m_cell)),
                static_cast<int64_t>(std::floor(p.z / m_cell))};
    }

    double m_tol;
    double m_cell;
    std::vector<Vec3> m_gathered;
    std::vector<Vec3> m_points;
    std::vector<size_t> m_parent;
    std::vector<size_t> m_weldedTo;
    std::unordered_map<size_t, size_t> m_pointOfCluster;
};

struct IndexedFace {
    std::vector<size_t> loop;
    std::vector<std::vector<size_t>> holes;
    topo::TopologyID topoId;
    std::shared_ptr<geo::NurbsSurface> surface;
    std::shared_ptr<geo::NurbsSurface> analyticSurface;
};

/// Remove consecutive duplicate indices (and a duplicated closing vertex).
void dedupeLoop(std::vector<size_t>& loop) {
    std::vector<size_t> out;
    out.reserve(loop.size());
    for (size_t idx : loop) {
        if (out.empty() || out.back() != idx) out.push_back(idx);
    }
    while (out.size() > 1 && out.front() == out.back()) out.pop_back();
    loop = std::move(out);
}

double loopAreaSq(const std::vector<size_t>& loop, const std::vector<Vec3>& pts) {
    // Area vector as a fan of triangles anchored at the first vertex. This is
    // translation-invariant but, unlike sum(a x b), its terms scale with the
    // loop's own extent rather than its distance from the origin — so a far-
    // from-origin loop's true zero area does not drown in R^2 cancellation
    // noise and evade the degenerate-face filter.
    if (loop.size() < 3) return 0.0;
    const Vec3& p0 = pts[loop[0]];
    Vec3 n = Vec3::Zero;
    for (size_t i = 1; i + 1 < loop.size(); ++i) {
        n = n + (pts[loop[i]] - p0).cross(pts[loop[i + 1]] - p0);
    }
    return 0.25 * n.dot(n);
}

/// The unit normal of an outward-wound loop (its area vector, from its first
/// point), or zero for a loop with no area.
Vec3 loopNormal(const std::vector<size_t>& loop, const std::vector<Vec3>& pts) {
    if (loop.size() < 3) return Vec3::Zero;
    const Vec3& p0 = pts[loop[0]];
    Vec3 n = Vec3::Zero;
    for (size_t i = 1; i + 1 < loop.size(); ++i) {
        n = n + (pts[loop[i]] - p0).cross(pts[loop[i + 1]] - p0);
    }
    const double len = n.length();
    return len > 0.0 ? n * (1.0 / len) : Vec3::Zero;
}

/// Of the half-edges running back along an edge that several faces share,
/// the twin of the one in a face of normal @p normal running along @p along:
/// the face met first turning about the edge from this one into the material
/// behind it. Two bodies touching along an edge give it four faces, and the
/// first one found, which pairing took, was as often the other body's: the
/// sewn solid read as one body, closed, with every count right, and was not
/// one. Turning into the material pairs each face with its own body's.
/// @p normals gives each candidate's face normal.
///
/// A face of the other body lying back to back on this one (bodies touching
/// face to face) is met at no turn at all, and its material is the other
/// body's: it comes last, after a whole turn. Facets of a curved face are
/// not quite flat, and their normals put such a face a few ten-thousandths
/// of a radian either side; so a face met within kBackToBack of no turn is
/// taken for one lying back to back. No solid is that thin at an edge.
size_t radialTwin(const Vec3& along, const Vec3& normal, const std::vector<Vec3>& normals) {
    constexpr double kBackToBack = 1e-2;  // radians
    const Vec3 d = along.normalized();
    // The way into this face from the edge, and the two axes about the edge.
    const Vec3 e1 = normal.cross(d).normalized();
    const Vec3 e2 = d.cross(e1);
    const Vec3 material = normal * -1.0;
    const double sense = material.dot(e2) >= 0.0 ? 1.0 : -1.0;
    size_t best = 0;
    double bestTurn = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < normals.size(); ++i) {
        // Into the candidate's face: its edge runs the other way.
        const Vec3 into = normals[i].cross(d * -1.0);
        double turn = sense * std::atan2(into.dot(e2), into.dot(e1));
        while (turn <= kBackToBack) turn += math::kTwoPi;
        if (turn < bestTurn) {
            bestTurn = turn;
            best = i;
        }
    }
    return best;
}

// -- T-junction elimination ---------------------------------------------------

/// Insert welded vertices that lie in the interior of a face edge, so that
/// adjacent faces sharing a subdivided boundary get matching vertex chains
/// and twin pairing can succeed.
void eliminateTJunctions(std::vector<IndexedFace>& faces, const std::vector<Vec3>& pts,
                         double tol) {
    if (pts.empty()) return;

    // Spatial grid over all welded points for edge-interval queries.
    Vec3 bbMin = pts[0];
    Vec3 bbMax = pts[0];
    for (const auto& p : pts) {
        bbMin = Vec3(std::min(bbMin.x, p.x), std::min(bbMin.y, p.y), std::min(bbMin.z, p.z));
        bbMax = Vec3(std::max(bbMax.x, p.x), std::max(bbMax.y, p.y), std::max(bbMax.z, p.z));
    }
    const double diag = (bbMax - bbMin).length();
    const double cell = std::max(diag / 64.0, tol * 16.0);

    auto cellOf = [&](double v, double lo) { return static_cast<int64_t>((v - lo) / cell); };
    std::unordered_map<CellKey, std::vector<size_t>, CellKeyHash> grid;
    for (size_t i = 0; i < pts.size(); ++i) {
        grid[{cellOf(pts[i].x, bbMin.x), cellOf(pts[i].y, bbMin.y), cellOf(pts[i].z, bbMin.z)}]
            .push_back(i);
    }

    const auto fix = [&](std::vector<size_t>& loop) {
        std::vector<size_t> newLoop;
        const size_t n = loop.size();
        for (size_t i = 0; i < n; ++i) {
            const size_t ia = loop[i];
            const size_t ib = loop[(i + 1) % n];
            const Vec3& a = pts[ia];
            const Vec3& b = pts[ib];
            newLoop.push_back(ia);

            const Vec3 d = b - a;
            const double len2 = d.dot(d);
            if (len2 < tol * tol) continue;

            // Collect interior points on segment (a, b).
            struct Hit {
                double t;
                size_t idx;
            };
            std::vector<Hit> hits;

            const Vec3 lo(std::min(a.x, b.x) - tol, std::min(a.y, b.y) - tol,
                          std::min(a.z, b.z) - tol);
            const Vec3 hi(std::max(a.x, b.x) + tol, std::max(a.y, b.y) + tol,
                          std::max(a.z, b.z) + tol);
            for (int64_t cx = cellOf(lo.x, bbMin.x); cx <= cellOf(hi.x, bbMin.x); ++cx) {
                for (int64_t cy = cellOf(lo.y, bbMin.y); cy <= cellOf(hi.y, bbMin.y); ++cy) {
                    for (int64_t cz = cellOf(lo.z, bbMin.z); cz <= cellOf(hi.z, bbMin.z); ++cz) {
                        auto it = grid.find({cx, cy, cz});
                        if (it == grid.end()) continue;
                        for (size_t idx : it->second) {
                            if (idx == ia || idx == ib) continue;
                            const Vec3& p = pts[idx];
                            const double t = (p - a).dot(d) / len2;
                            if (t <= 1e-9 || t >= 1.0 - 1e-9) continue;
                            const Vec3 proj = a + d * t;
                            if (proj.distanceTo(p) <= tol) hits.push_back({t, idx});
                        }
                    }
                }
            }

            std::sort(hits.begin(), hits.end(),
                      [](const Hit& l, const Hit& r) { return l.t < r.t; });
            for (const Hit& h : hits) {
                if (newLoop.back() != h.idx) newLoop.push_back(h.idx);
            }
        }
        dedupeLoop(newLoop);
        loop = std::move(newLoop);
    };
    for (auto& face : faces) {
        fix(face.loop);
        for (auto& hole : face.holes) fix(hole);
    }
}

// -- Surface synthesis --------------------------------------------------------

std::shared_ptr<geo::NurbsSurface> synthesizePlanarPatch(const std::vector<size_t>& loop,
                                                         const std::vector<Vec3>& pts) {
    // Newell normal, about the first point (its error then does not grow
    // with the distance from the origin).
    Vec3 n = Vec3::Zero;
    for (size_t i = 0; i < loop.size(); ++i) {
        const Vec3 a = pts[loop[i]] - pts[loop[0]];
        const Vec3 b = pts[loop[(i + 1) % loop.size()]] - pts[loop[0]];
        n.x += (a.y - b.y) * (a.z + b.z);
        n.y += (a.z - b.z) * (a.x + b.x);
        n.z += (a.x - b.x) * (a.y + b.y);
    }
    const double nLen = n.length();
    if (nLen < 1e-30) return nullptr;
    n = n / nLen;

    // In-plane basis from the first usable edge (same convention as Extrude).
    Vec3 u;
    bool haveU = false;
    for (size_t i = 0; i < loop.size(); ++i) {
        Vec3 e = pts[loop[(i + 1) % loop.size()]] - pts[loop[i]];
        e = e - n * e.dot(n);
        if (e.length() > 1e-12) {
            u = e.normalized();
            haveU = true;
            break;
        }
    }
    if (!haveU) return nullptr;
    const Vec3 v = n.cross(u);

    const Vec3& p0 = pts[loop[0]];
    double uMin = 0, uMax = 0, vMin = 0, vMax = 0;
    for (size_t idx : loop) {
        const Vec3 d = pts[idx] - p0;
        uMin = std::min(uMin, d.dot(u));
        uMax = std::max(uMax, d.dot(u));
        vMin = std::min(vMin, d.dot(v));
        vMax = std::max(vMax, d.dot(v));
    }
    const double uSize = std::max(uMax - uMin, 1e-9);
    const double vSize = std::max(vMax - vMin, 1e-9);
    const Vec3 origin = p0 + u * uMin + v * vMin;
    return std::make_shared<geo::NurbsSurface>(
        geo::NurbsSurface::makePlane(origin, u, v, uSize, vSize));
}

std::shared_ptr<geo::NurbsCurve> makeLineCurve(const Vec3& a, const Vec3& b) {
    return std::make_shared<geo::NurbsCurve>(std::vector<Vec3>{a, b}, std::vector<double>{1.0, 1.0},
                                             std::vector<double>{0.0, 0.0, 1.0, 1.0}, 1);
}

}  // namespace

std::unique_ptr<topo::Solid> SolidSewer::sew(const std::vector<InputFace>& faces, double weldTol) {
    using namespace hz::topo;

    // 1. Gather every position the faces name, weld them, and index the loops.
    VertexWelder welder(weldTol);
    std::vector<IndexedFace> indexed;
    indexed.reserve(faces.size());
    for (const auto& face : faces) {
        if (face.points.size() < 3) continue;
        IndexedFace f;
        f.loop.reserve(face.points.size());
        for (const auto& p : face.points) f.loop.push_back(welder.gather(p));
        for (const auto& hole : face.holes) {
            std::vector<size_t> loop;
            loop.reserve(hole.size());
            for (const auto& p : hole) loop.push_back(welder.gather(p));
            f.holes.push_back(std::move(loop));
        }
        f.topoId = face.topoId;
        f.surface = face.surface;
        f.analyticSurface = face.analyticSurface;
        indexed.push_back(std::move(f));
    }
    // Once every position is in, so which of them are one vertex is settled by
    // the set rather than by the order they arrived in.
    welder.weld();
    for (IndexedFace& f : indexed) {
        for (size_t& i : f.loop) i = welder.index(i);
        dedupeLoop(f.loop);
        for (auto& hole : f.holes) {
            for (size_t& i : hole) i = welder.index(i);
            dedupeLoop(hole);
            if (hole.size() < 3) hole.clear();
        }
        std::erase_if(f.holes, [](const auto& h) { return h.size() < 3; });
    }
    indexed.erase(std::remove_if(indexed.begin(), indexed.end(),
                                 [](const IndexedFace& f) { return f.loop.size() < 3; }),
                  indexed.end());
    const std::vector<Vec3>& pts = welder.points();

    // 2. Drop degenerate (near zero-area) faces.
    const double areaTol2 = weldTol * weldTol * 1e-4;
    indexed.erase(
        std::remove_if(indexed.begin(), indexed.end(),
                       [&](const IndexedFace& f) { return loopAreaSq(f.loop, pts) < areaTol2; }),
        indexed.end());
    if (indexed.empty()) return nullptr;

    // 3. Matching vertex chains along shared boundaries.
    eliminateTJunctions(indexed, pts, weldTol * 8.0);
    indexed.erase(std::remove_if(indexed.begin(), indexed.end(),
                                 [](const IndexedFace& f) { return f.loop.size() < 3; }),
                  indexed.end());
    for (auto& f : indexed) {
        f.holes.erase(std::remove_if(f.holes.begin(), f.holes.end(),
                                     [](const std::vector<size_t>& h) { return h.size() < 3; }),
                      f.holes.end());
    }
    if (indexed.empty()) return nullptr;

    // 4. Build the half-edge structure.
    auto solid = std::make_unique<Solid>();

    std::unordered_map<size_t, Vertex*> vertexMap;
    auto vertexFor = [&](size_t idx) {
        auto it = vertexMap.find(idx);
        if (it != vertexMap.end()) return it->second;
        Vertex* v = solid->allocVertex();
        v->point = pts[idx];
        vertexMap[idx] = v;
        return v;
    };

    /// A face's loops (outer first) and their half-edges, loop by loop.
    struct FaceBuild {
        Face* face = nullptr;
        std::vector<std::vector<HalfEdge*>> hes;
        std::vector<std::vector<size_t>> loops;
    };
    std::vector<FaceBuild> built;
    built.reserve(indexed.size());

    std::unordered_map<uint64_t, std::vector<HalfEdge*>> directed;
    auto dirKey = [](size_t a, size_t b) {
        return (static_cast<uint64_t>(a) << 32) | static_cast<uint64_t>(b);
    };

    for (const auto& f : indexed) {
        FaceBuild fb;
        fb.face = solid->allocFace();
        fb.face->topoId = f.topoId;
        fb.loops.push_back(f.loop);
        fb.loops.insert(fb.loops.end(), f.holes.begin(), f.holes.end());

        for (size_t l = 0; l < fb.loops.size(); ++l) {
            const std::vector<size_t>& loop = fb.loops[l];
            Wire* wire = solid->allocWire();
            if (l == 0) {
                fb.face->outerLoop = wire;
            } else {
                fb.face->innerLoops.push_back(wire);
            }
            const size_t n = loop.size();
            std::vector<HalfEdge*> hes(n);
            for (size_t i = 0; i < n; ++i) hes[i] = solid->allocHalfEdge();
            for (size_t i = 0; i < n; ++i) {
                HalfEdge* he = hes[i];
                he->origin = vertexFor(loop[i]);
                he->face = fb.face;
                he->next = hes[(i + 1) % n];
                he->prev = hes[(i + n - 1) % n];
                if (he->origin->halfEdge == nullptr) he->origin->halfEdge = he;
                directed[dirKey(loop[i], loop[(i + 1) % n])].push_back(he);
            }
            wire->halfEdge = hes[0];
            fb.hes.push_back(std::move(hes));
        }

        // Surface: reuse the provided patch, else synthesize the planar
        // bounding-rectangle patch used throughout the kernel.
        fb.face->surface = f.surface ? f.surface : synthesizePlanarPatch(f.loop, pts);
        fb.face->analyticSurface = f.analyticSurface;

        built.push_back(std::move(fb));
    }

    // 5. Twin pairing + edges.
    std::unordered_map<const Face*, Vec3> normalOf;
    for (size_t i = 0; i < built.size(); ++i) {
        normalOf[built[i].face] = loopNormal(indexed[i].loop, pts);
    }
    int edgeIndex = 0;
    for (auto& fb : built) {
        for (size_t l = 0; l < fb.loops.size(); ++l) {
            const std::vector<size_t>& loop = fb.loops[l];
            const size_t n = loop.size();
            for (size_t i = 0; i < n; ++i) {
                HalfEdge* he = fb.hes[l][i];
                if (he->edge != nullptr) continue;  // already paired from the other side

                Edge* edge = solid->allocEdge();
                edge->halfEdge = he;
                edge->topoId = fb.face->topoId.child("edge", edgeIndex++);
                edge->curve = makeLineCurve(he->origin->point, he->next->origin->point);
                he->edge = edge;

                auto it = directed.find(dirKey(loop[(i + 1) % n], loop[i]));
                if (it != directed.end()) {
                    std::vector<HalfEdge*> free;
                    for (HalfEdge* candidate : it->second) {
                        if (candidate->twin == nullptr && candidate != he)
                            free.push_back(candidate);
                    }
                    HalfEdge* twin = free.empty() ? nullptr : free.front();
                    if (free.size() > 1) {
                        // Faces of several bodies meet at this edge.
                        std::vector<Vec3> normals;
                        normals.reserve(free.size());
                        for (HalfEdge* candidate : free) {
                            normals.push_back(normalOf.at(candidate->face));
                        }
                        twin = free[radialTwin(pts[loop[(i + 1) % n]] - pts[loop[i]],
                                               normalOf.at(fb.face), normals)];
                    }
                    if (twin != nullptr) {
                        he->twin = twin;
                        twin->twin = he;
                        twin->edge = edge;
                    }
                }
            }
        }
    }

    // 6. Shells: connected components over twin adjacency.
    std::vector<int> component(built.size(), -1);
    std::unordered_map<Face*, size_t> faceIndex;
    for (size_t i = 0; i < built.size(); ++i) faceIndex[built[i].face] = i;

    int componentCount = 0;
    for (size_t seed = 0; seed < built.size(); ++seed) {
        if (component[seed] != -1) continue;
        const int comp = componentCount++;
        std::vector<size_t> stack{seed};
        component[seed] = comp;
        while (!stack.empty()) {
            const size_t cur = stack.back();
            stack.pop_back();
            for (const auto& loop : built[cur].hes) {
                for (HalfEdge* he : loop) {
                    if (he->twin == nullptr) continue;
                    auto it = faceIndex.find(he->twin->face);
                    if (it == faceIndex.end() || component[it->second] != -1) continue;
                    component[it->second] = comp;
                    stack.push_back(it->second);
                }
            }
        }
    }

    std::vector<Shell*> shells(componentCount, nullptr);
    for (size_t i = 0; i < built.size(); ++i) {
        Shell*& shell = shells[component[i]];
        if (shell == nullptr) {
            shell = solid->allocShell();
            shell->solid = solid.get();
        }
        shell->faces.push_back(built[i].face);
        built[i].face->shell = shell;
    }

    // 7. A vertex for each fan of faces round it. Bodies that touch along an
    // edge or at a point were welded there into one vertex that two shells
    // share, and a shell that pinches to a point shares one with itself: a
    // vertex no solid has, which the counts (vertex, edge and face) took for
    // a solid that is not one, or for no solid at all. The faces round a
    // vertex are walked from one to the next across their edges; each such
    // fan after the first takes a copy of the vertex, so what touches there
    // touches without sharing anything.
    {
        // In the order the faces were given, so the copies are made, and
        // numbered, the same way every time.
        std::vector<Vertex*> order;
        std::unordered_map<const Vertex*, std::vector<HalfEdge*>> leaving;
        for (auto& fb : built) {
            for (auto& loop : fb.hes) {
                for (HalfEdge* he : loop) {
                    auto& out = leaving[he->origin];
                    if (out.empty()) order.push_back(he->origin);
                    out.push_back(he);
                }
            }
        }
        std::unordered_map<const HalfEdge*, bool> seen;
        for (Vertex* vertex : order) {
            const auto& outgoing = leaving[vertex];
            if (outgoing.size() < 2) continue;
            bool kept = false;
            for (HalfEdge* first : outgoing) {
                if (seen[first]) continue;
                // This fan: from one outgoing half-edge to the next, across
                // the face before it (into the vertex by its twin, out again
                // by the twin's next).
                std::vector<HalfEdge*> fan;
                for (HalfEdge* he = first; he != nullptr && !seen[he];) {
                    seen[he] = true;
                    fan.push_back(he);
                    he = he->twin != nullptr ? he->twin->next : nullptr;
                }
                // And the other way, should the fan be open.
                for (HalfEdge* he = first->prev != nullptr ? first->prev->twin : nullptr;
                     he != nullptr && !seen[he];
                     he = he->prev != nullptr ? he->prev->twin : nullptr) {
                    seen[he] = true;
                    fan.push_back(he);
                }
                if (!kept) {
                    kept = true;  // the first fan keeps the vertex
                    continue;
                }
                Vertex* copy = solid->allocVertex();
                copy->point = vertex->point;
                for (HalfEdge* he : fan) he->origin = copy;
            }
        }
        // Each vertex's outgoing half-edge is one that starts at it.
        for (auto& v : solid->vertices()) v.halfEdge = nullptr;
        for (auto& fb : built) {
            for (auto& loop : fb.hes) {
                for (HalfEdge* he : loop) {
                    if (he->origin->halfEdge == nullptr) he->origin->halfEdge = he;
                }
            }
        }
    }

    return solid;
}

}  // namespace hz::model
