#include "horizon/modeling/SectionView.h"

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#include "horizon/modeling/SolidTessellator.h"
#include "horizon/render/SceneGraph.h"
#include "horizon/topology/HalfEdge.h"
#include "horizon/topology/Solid.h"

namespace hz::model {

using hz::math::Vec2;
using hz::math::Vec3;

namespace {

constexpr double kOnPlane = 1e-9;
constexpr double kChainTol = 1e-6;

/// 2D projection basis for a view (matches the ViewProjection convention).
struct Basis {
    Vec3 origin;
    Vec3 right;
    Vec3 upv;

    Vec2 project(const Vec3& p) const {
        const Vec3 d = p - origin;
        return Vec2{d.dot(right), d.dot(upv)};
    }
};

Basis makeBasis(const ViewProjection& view) {
    Basis b;
    b.origin = view.origin;
    const Vec3 dir = view.dir.normalized();
    b.right = dir.cross(view.up).normalized();
    b.upv = b.right.cross(dir).normalized();
    return b;
}

/// Chain unordered 2D segments into closed loops (endpoint matching).
/// Unclosed chains — tolerance casualties — are discarded.
std::vector<std::vector<Vec2>> chainLoops(std::vector<std::pair<Vec2, Vec2>> segments) {
    std::vector<std::vector<Vec2>> loops;
    std::vector<bool> used(segments.size(), false);

    auto near = [](const Vec2& a, const Vec2& b) {
        return std::abs(a.x - b.x) < kChainTol && std::abs(a.y - b.y) < kChainTol;
    };

    for (size_t i = 0; i < segments.size(); ++i) {
        if (used[i]) continue;
        used[i] = true;

        std::vector<Vec2> chain{segments[i].first, segments[i].second};
        bool extended = true;
        while (extended && !near(chain.front(), chain.back())) {
            extended = false;
            for (size_t j = 0; j < segments.size(); ++j) {
                if (used[j]) continue;
                if (near(segments[j].first, chain.back())) {
                    chain.push_back(segments[j].second);
                } else if (near(segments[j].second, chain.back())) {
                    chain.push_back(segments[j].first);
                } else {
                    continue;
                }
                used[j] = true;
                extended = true;
                break;
            }
        }

        if (chain.size() >= 4 && near(chain.front(), chain.back())) {
            chain.pop_back();  // drop the duplicated closing point
            loops.push_back(std::move(chain));
        }
    }
    return loops;
}

/// 45° cross-hatching of the loop set (even-odd fill rule).
std::vector<std::pair<Vec2, Vec2>> hatchLoops(const std::vector<std::vector<Vec2>>& loops,
                                              double spacing) {
    std::vector<std::pair<Vec2, Vec2>> hatch;
    if (loops.empty() || spacing <= 0.0) return hatch;

    // Rotate 45°: u along the hatch lines, v across them.
    const double kInvSqrt2 = 0.7071067811865476;
    auto toRot = [&](const Vec2& p) {
        return Vec2{(p.x + p.y) * kInvSqrt2, (p.y - p.x) * kInvSqrt2};
    };
    auto fromRot = [&](const Vec2& p) {
        return Vec2{(p.x - p.y) * kInvSqrt2, (p.x + p.y) * kInvSqrt2};
    };

    double vMin = 1e300;
    double vMax = -1e300;
    for (const auto& loop : loops) {
        for (const Vec2& p : loop) {
            const double v = toRot(p).y;
            vMin = std::min(vMin, v);
            vMax = std::max(vMax, v);
        }
    }

    for (double v = vMin + spacing * 0.5; v < vMax; v += spacing) {
        // Intersections of the scanline with every loop edge.
        std::vector<double> hits;
        for (const auto& loop : loops) {
            for (size_t i = 0; i < loop.size(); ++i) {
                const Vec2 a = toRot(loop[i]);
                const Vec2 b = toRot(loop[(i + 1) % loop.size()]);
                if ((a.y <= v) == (b.y <= v)) continue;  // no crossing
                const double t = (v - a.y) / (b.y - a.y);
                hits.push_back(a.x + t * (b.x - a.x));
            }
        }
        std::sort(hits.begin(), hits.end());
        for (size_t k = 0; k + 1 < hits.size(); k += 2) {
            hatch.emplace_back(fromRot(Vec2{hits[k], v}), fromRot(Vec2{hits[k + 1], v}));
        }
    }
    return hatch;
}

}  // namespace

DrawingView SectionGenerator::sectionView(const topo::Solid& solid, const Vec3& planePoint,
                                          const Vec3& planeNormal, double hatchSpacing) {
    DrawingView view;
    const Vec3 n = planeNormal.normalized();

    view.projection.origin = planePoint;
    view.projection.dir = n * (-1.0);  // look at the cut
    view.projection.up = std::abs(n.z) < 0.9 ? Vec3(0.0, 0.0, 1.0) : Vec3(0.0, 1.0, 0.0);
    const Basis basis = makeBasis(view.projection);

    const auto dist = [&](const Vec3& p) { return (p - planePoint).dot(n); };

    // -- Cut profile: intersect every tessellation triangle with the plane --
    const render::MeshData mesh = SolidTessellator::tessellate(solid);
    std::vector<std::pair<Vec2, Vec2>> cutSegments;
    for (size_t t = 0; t + 2 < mesh.indices.size(); t += 3) {
        Vec3 pts[3];
        double d[3];
        for (int k = 0; k < 3; ++k) {
            const uint32_t idx = mesh.indices[t + static_cast<size_t>(k)];
            pts[k] = Vec3(mesh.positions[3 * idx], mesh.positions[3 * idx + 1],
                          mesh.positions[3 * idx + 2]);
            d[k] = dist(pts[k]);
        }

        // Triangles lying in the plane contribute area, not cut boundary.
        if (std::abs(d[0]) < kOnPlane && std::abs(d[1]) < kOnPlane && std::abs(d[2]) < kOnPlane) {
            continue;
        }

        // Collect the (up to two) crossing points of this triangle.
        std::vector<Vec3> crossings;
        for (int k = 0; k < 3; ++k) {
            const int j = (k + 1) % 3;
            if (std::abs(d[k]) < kOnPlane) {
                crossings.push_back(pts[k]);
                continue;
            }
            if ((d[k] > 0.0) == (d[j] > 0.0) || std::abs(d[j]) < kOnPlane) continue;
            const double s = d[k] / (d[k] - d[j]);
            crossings.push_back(pts[k] + (pts[j] - pts[k]) * s);
        }
        // Deduplicate near-coincident crossings (vertex-on-plane cases).
        std::vector<Vec3> unique;
        for (const Vec3& c : crossings) {
            bool dup = false;
            for (const Vec3& u : unique) {
                if (c.distanceTo(u) < kChainTol) {
                    dup = true;
                    break;
                }
            }
            if (!dup) unique.push_back(c);
        }
        if (unique.size() == 2) {
            cutSegments.emplace_back(basis.project(unique[0]), basis.project(unique[1]));
        }
    }

    // Deduplicate coincident segments: a triangle EDGE lying exactly in the
    // cut plane is reported by both triangles sharing it, and the doubled
    // segments would close each side onto itself instead of chaining the
    // profile. Canonicalize (quantized, lexicographic endpoints) and unique.
    {
        struct Key {
            long long ax, ay, bx, by;
            bool operator<(const Key& o) const {
                return std::tie(ax, ay, bx, by) < std::tie(o.ax, o.ay, o.bx, o.by);
            }
        };
        auto quantize = [](double v) { return static_cast<long long>(std::llround(v * 1e7)); };
        std::vector<std::pair<Key, size_t>> keyed;
        keyed.reserve(cutSegments.size());
        for (size_t i = 0; i < cutSegments.size(); ++i) {
            Vec2 a = cutSegments[i].first;
            Vec2 b = cutSegments[i].second;
            if (std::tie(a.x, a.y) > std::tie(b.x, b.y)) std::swap(a, b);
            keyed.push_back({Key{quantize(a.x), quantize(a.y), quantize(b.x), quantize(b.y)}, i});
        }
        std::sort(keyed.begin(), keyed.end(),
                  [](const auto& l, const auto& r) { return l.first < r.first; });
        std::vector<std::pair<Vec2, Vec2>> deduped;
        deduped.reserve(cutSegments.size());
        for (size_t i = 0; i < keyed.size(); ++i) {
            if (i > 0 && !(keyed[i - 1].first < keyed[i].first)) continue;  // duplicate
            deduped.push_back(cutSegments[keyed[i].second]);
        }
        cutSegments = std::move(deduped);
    }

    view.sectionLoops = chainLoops(std::move(cutSegments));
    view.sectionHatch = hatchLoops(view.sectionLoops, hatchSpacing);

    // -- Retained outline: edges of the half behind the plane, clipped --
    // Section convention: hidden lines are omitted, so every retained edge
    // projects as visible.
    for (const auto& e : solid.edges()) {
        if (e.halfEdge == nullptr || e.halfEdge->twin == nullptr) continue;
        Vec3 a = e.halfEdge->origin->point;
        Vec3 b = e.halfEdge->twin->origin->point;
        double da = dist(a);
        double db = dist(b);
        if (da > kOnPlane && db > kOnPlane) continue;  // fully in front — cut away
        if (da > kOnPlane || db > kOnPlane) {
            // Clip at the plane.
            const double s = da / (da - db);
            const Vec3 hit = a + (b - a) * s;
            if (da > 0.0) {
                a = hit;
            } else {
                b = hit;
            }
        }
        if (a.distanceTo(b) < kChainTol) continue;

        ProjectedEdge pe;
        pe.a = basis.project(a);
        pe.b = basis.project(b);
        pe.sourceEdge = e.topoId;
        pe.visibility = ProjectedEdge::Visibility::Visible;
        view.edges.push_back(pe);
    }

    // -- Bounds over everything the view renders --
    bool first = true;
    auto grow = [&](const Vec2& p) {
        if (first) {
            view.boundsMin = p;
            view.boundsMax = p;
            first = false;
            return;
        }
        view.boundsMin.x = std::min(view.boundsMin.x, p.x);
        view.boundsMin.y = std::min(view.boundsMin.y, p.y);
        view.boundsMax.x = std::max(view.boundsMax.x, p.x);
        view.boundsMax.y = std::max(view.boundsMax.y, p.y);
    };
    for (const auto& loop : view.sectionLoops) {
        for (const Vec2& p : loop) grow(p);
    }
    for (const auto& e : view.edges) {
        grow(e.a);
        grow(e.b);
    }

    return view;
}

}  // namespace hz::model
