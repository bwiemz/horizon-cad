#include "horizon/modeling/SolidSewer.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <numeric>
#include <unordered_map>
#include <vector>

#include "horizon/geometry/curves/NurbsCurve.h"
#include "horizon/geometry/surfaces/NurbsSurface.h"
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
class VertexWelder {
public:
    explicit VertexWelder(double tol) : m_tol(tol), m_cell(tol * 2.0) {}

    size_t index(const Vec3& p) {
        const CellKey base = keyFor(p);
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dz = -1; dz <= 1; ++dz) {
                    const CellKey k{base.x + dx, base.y + dy, base.z + dz};
                    auto it = m_grid.find(k);
                    if (it == m_grid.end()) continue;
                    for (size_t idx : it->second) {
                        if (m_points[idx].distanceTo(p) <= m_tol) return idx;
                    }
                }
            }
        }
        const size_t idx = m_points.size();
        m_points.push_back(p);
        m_grid[base].push_back(idx);
        return idx;
    }

    const std::vector<Vec3>& points() const { return m_points; }

private:
    CellKey keyFor(const Vec3& p) const {
        return {static_cast<int64_t>(std::floor(p.x / m_cell)),
                static_cast<int64_t>(std::floor(p.y / m_cell)),
                static_cast<int64_t>(std::floor(p.z / m_cell))};
    }

    double m_tol;
    double m_cell;
    std::vector<Vec3> m_points;
    std::unordered_map<CellKey, std::vector<size_t>, CellKeyHash> m_grid;
};

struct IndexedFace {
    std::vector<size_t> loop;
    topo::TopologyID topoId;
    std::shared_ptr<geo::NurbsSurface> surface;
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

    for (auto& face : faces) {
        std::vector<size_t> newLoop;
        const size_t n = face.loop.size();
        for (size_t i = 0; i < n; ++i) {
            const size_t ia = face.loop[i];
            const size_t ib = face.loop[(i + 1) % n];
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
        face.loop = std::move(newLoop);
    }
}

// -- Surface synthesis --------------------------------------------------------

std::shared_ptr<geo::NurbsSurface> synthesizePlanarPatch(const std::vector<size_t>& loop,
                                                         const std::vector<Vec3>& pts) {
    // Newell normal.
    Vec3 n = Vec3::Zero;
    for (size_t i = 0; i < loop.size(); ++i) {
        const Vec3& a = pts[loop[i]];
        const Vec3& b = pts[loop[(i + 1) % loop.size()]];
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

    // 1. Weld vertices and index the face loops.
    VertexWelder welder(weldTol);
    std::vector<IndexedFace> indexed;
    indexed.reserve(faces.size());
    for (const auto& face : faces) {
        if (face.points.size() < 3) continue;
        IndexedFace f;
        f.loop.reserve(face.points.size());
        for (const auto& p : face.points) f.loop.push_back(welder.index(p));
        dedupeLoop(f.loop);
        f.topoId = face.topoId;
        f.surface = face.surface;
        if (f.loop.size() >= 3) indexed.push_back(std::move(f));
    }
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

    struct FaceBuild {
        Face* face = nullptr;
        std::vector<HalfEdge*> hes;
        std::vector<size_t> loop;
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
        fb.loop = f.loop;

        Wire* wire = solid->allocWire();
        fb.face->outerLoop = wire;

        const size_t n = f.loop.size();
        fb.hes.resize(n);
        for (size_t i = 0; i < n; ++i) fb.hes[i] = solid->allocHalfEdge();
        for (size_t i = 0; i < n; ++i) {
            HalfEdge* he = fb.hes[i];
            he->origin = vertexFor(f.loop[i]);
            he->face = fb.face;
            he->next = fb.hes[(i + 1) % n];
            he->prev = fb.hes[(i + n - 1) % n];
            if (he->origin->halfEdge == nullptr) he->origin->halfEdge = he;
            directed[dirKey(f.loop[i], f.loop[(i + 1) % n])].push_back(he);
        }
        wire->halfEdge = fb.hes[0];

        // Surface: reuse the provided patch, else synthesize the planar
        // bounding-rectangle patch used throughout the kernel.
        fb.face->surface = f.surface ? f.surface : synthesizePlanarPatch(f.loop, pts);

        built.push_back(std::move(fb));
    }

    // 5. Twin pairing + edges.
    int edgeIndex = 0;
    for (auto& fb : built) {
        const size_t n = fb.loop.size();
        for (size_t i = 0; i < n; ++i) {
            HalfEdge* he = fb.hes[i];
            if (he->edge != nullptr) continue;  // already paired from the other side

            Edge* edge = solid->allocEdge();
            edge->halfEdge = he;
            edge->topoId = fb.face->topoId.child("edge", edgeIndex++);
            edge->curve = makeLineCurve(he->origin->point, he->next->origin->point);
            he->edge = edge;

            auto it = directed.find(dirKey(fb.loop[(i + 1) % n], fb.loop[i]));
            if (it != directed.end()) {
                for (HalfEdge* candidate : it->second) {
                    if (candidate->twin == nullptr && candidate != he) {
                        he->twin = candidate;
                        candidate->twin = he;
                        candidate->edge = edge;
                        break;
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
            for (HalfEdge* he : built[cur].hes) {
                if (he->twin == nullptr) continue;
                auto it = faceIndex.find(he->twin->face);
                if (it == faceIndex.end() || component[it->second] != -1) continue;
                component[it->second] = comp;
                stack.push_back(it->second);
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

    return solid;
}

}  // namespace hz::model
