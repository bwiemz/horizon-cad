#include "horizon/modeling/Loft.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>

#include "RingStack.h"
#include "horizon/modeling/ProfileValidator.h"
#include "horizon/topology/TopologyID.h"

namespace hz::model {

using hz::math::Vec2;
using hz::math::Vec3;
using namespace hz::topo;

namespace {

Vec3 centroid(const std::vector<Vec3>& ring) {
    Vec3 c = Vec3::Zero;
    for (const auto& p : ring) c = c + p;
    return c * (1.0 / static_cast<double>(ring.size()));
}

// Newell's method: area-weighted normal of a (possibly non-planar) ring.
Vec3 newellNormal(const std::vector<Vec3>& ring) {
    Vec3 n = Vec3::Zero;
    const size_t M = ring.size();
    for (size_t i = 0; i < M; ++i) {
        const Vec3& a = ring[i];
        const Vec3& b = ring[(i + 1) % M];
        n.x += (a.y - b.y) * (a.z + b.z);
        n.y += (a.z - b.z) * (a.x + b.x);
        n.z += (a.x - b.x) * (a.y + b.y);
    }
    return n;
}

// Rotate @p ring so vertex @p offset becomes index 0.
std::vector<Vec3> rotated(const std::vector<Vec3>& ring, size_t offset) {
    std::vector<Vec3> out;
    out.reserve(ring.size());
    for (size_t i = 0; i < ring.size(); ++i) {
        out.push_back(ring[(i + offset) % ring.size()]);
    }
    return out;
}

// Best start-index offset aligning @p ring to @p reference (minimizes the sum
// of squared vertex distances over all rotations).
size_t bestAlignmentOffset(const std::vector<Vec3>& ring, const std::vector<Vec3>& reference) {
    const size_t N = ring.size();
    size_t best = 0;
    double bestCost = std::numeric_limits<double>::max();
    for (size_t offset = 0; offset < N; ++offset) {
        double cost = 0.0;
        for (size_t i = 0; i < N; ++i) {
            const Vec3 d = ring[(i + offset) % N] - reference[i];
            cost += d.dot(d);
        }
        if (cost < bestCost) {
            bestCost = cost;
            best = offset;
        }
    }
    return best;
}

}  // namespace

std::unique_ptr<topo::Solid> Loft::execute(const std::vector<LoftSection>& sections,
                                           const std::string& featureID, int twistSegments) {
    if (sections.size() < 2) return nullptr;

    // -----------------------------------------------------------------------
    // 1. Validate and extract each section as a 3D ring.
    // -----------------------------------------------------------------------
    std::vector<std::vector<Vec3>> rings;
    rings.reserve(sections.size());
    size_t N = 0;
    for (const auto& section : sections) {
        auto validation = ProfileValidator::validate(section.profile);
        if (!validation.isClosed) return nullptr;

        std::vector<Vec2> verts2D =
            ringstack::extractProfileVertices(validation.orderedEdges, 1e-6);
        if (verts2D.size() < 3) return nullptr;
        if (N == 0) {
            N = verts2D.size();
        } else if (verts2D.size() != N) {
            // Era-2 scope: sections must share a vertex count.
            return nullptr;
        }

        std::vector<Vec3> ring;
        ring.reserve(N);
        for (const auto& v : verts2D) ring.push_back(section.plane.localToWorld(v));
        rings.push_back(std::move(ring));
    }

    // -----------------------------------------------------------------------
    // 2. Consistent winding + start-index alignment.
    // -----------------------------------------------------------------------
    const Vec3 loftAxis = (centroid(rings.back()) - centroid(rings.front())).normalized();

    for (auto& ring : rings) {
        // Wind so the ring normal points along the loft axis (outward at top).
        if (newellNormal(ring).dot(loftAxis) < 0.0) {
            std::reverse(ring.begin(), ring.end());
        }
    }
    for (size_t L = 1; L < rings.size(); ++L) {
        size_t offset = bestAlignmentOffset(rings[L], rings[L - 1]);
        if (offset != 0) rings[L] = rotated(rings[L], offset);
    }

    // -----------------------------------------------------------------------
    // 3. Lateral faces.
    //
    // Between two sections each band is ruled: the quad's corners are joined
    // by straight lines, and the surface through them is the bilinear patch.
    // When the four corners are coplanar (aligned, similar sections) the band
    // is that flat quad and needs nothing more.  When they are not — any
    // twist, or sections that are not similar — the loop does not bound a
    // well-defined area, and the kernel's loop-based paths would each pick a
    // diagonal of their own: a square twisted 0.6 rad integrated to 180.8
    // while the displayed patch enclosed 150.7.  Such a level is instead cut
    // along its rulings into strips and each non-planar strip into two
    // triangles, so every facet is flat and every path sees the same solid.
    //
    // The diagonal alternates from strip to strip.  The volume a bilinear
    // patch bounds is exactly the mean of its two triangulations, so a pair
    // of strips split opposite ways bounds the ruled volume exactly; with an
    // even strip count the faceted loft's volume *is* the ruled loft's, and
    // `twistSegments` only decides how closely the facets hug the patch.
    // Each facet records the bilinear patch it approximates.
    // -----------------------------------------------------------------------
    double scale = 0.0;
    for (const auto& ring : rings) {
        for (const auto& p : ring) scale = std::max(scale, (p - rings.front().front()).length());
    }
    const double planarTol = 1e-9 * std::max(scale, 1.0);
    auto isPlanar = [planarTol](const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& d) {
        const Vec3 n = (b - a).cross(c - a);
        const double len = n.length();
        if (len < 1e-300) return true;
        return std::abs((d - a).dot(n * (1.0 / len))) <= planarTol;
    };

    std::vector<SolidSewer::InputFace> faces;
    for (size_t L = 0; L + 1 < rings.size(); ++L) {
        const auto& lower = rings[L];
        const auto& upper = rings[L + 1];
        bool levelPlanar = true;
        for (size_t i = 0; i < N && levelPlanar; ++i) {
            const size_t j = (i + 1) % N;
            levelPlanar = isPlanar(lower[i], lower[j], upper[j], upper[i]);
        }
        // Every column of a level shares its rulings with its neighbours, so
        // the whole level is cut at the same stations or none of it is.
        const int strips = levelPlanar ? 1 : 2 * ((std::max(1, twistSegments) + 1) / 2);

        for (size_t i = 0; i < N; ++i) {
            const size_t j = (i + 1) % N;
            const auto id = TopologyID::make(
                featureID, "lateral_" + std::to_string(L) + "_" + std::to_string(i));
            auto patch = ringstack::makeBilinearPatch(lower[i], lower[j], upper[i], upper[j]);
            if (levelPlanar) {
                SolidSewer::InputFace f;
                f.points = {lower[i], lower[j], upper[j], upper[i]};
                f.topoId = id;
                f.surface = patch;
                faces.push_back(std::move(f));
                continue;
            }
            int facet = 0;
            for (int s = 0; s < strips; ++s) {
                const double t0 = static_cast<double>(s) / strips;
                const double t1 = static_cast<double>(s + 1) / strips;
                const Vec3 a0 = lower[i] + (upper[i] - lower[i]) * t0;
                const Vec3 b0 = lower[j] + (upper[j] - lower[j]) * t0;
                const Vec3 a1 = lower[i] + (upper[i] - lower[i]) * t1;
                const Vec3 b1 = lower[j] + (upper[j] - lower[j]) * t1;
                std::vector<std::vector<Vec3>> loops;
                if (isPlanar(a0, b0, b1, a1)) {
                    loops.push_back({a0, b0, b1, a1});
                } else if (s % 2 == 0) {
                    loops.push_back({a0, b0, b1});
                    loops.push_back({a0, b1, a1});
                } else {
                    loops.push_back({a0, b0, a1});
                    loops.push_back({b0, b1, a1});
                }
                for (auto& loop : loops) {
                    SolidSewer::InputFace f;
                    f.points = std::move(loop);
                    f.topoId = id.child("facet", facet++);
                    f.analyticSurface = patch;
                    faces.push_back(std::move(f));
                }
            }
        }
    }

    // Caps.  orientOutward fixes the handedness of the whole soup at once.
    {
        SolidSewer::InputFace bottom;
        bottom.points.assign(rings.front().rbegin(), rings.front().rend());
        bottom.topoId = TopologyID::make(featureID, "cap_bottom");
        bottom.surface =
            ringstack::makeCapSurface(rings.front(), sections.front().plane.normal() * -1.0);
        faces.push_back(std::move(bottom));

        SolidSewer::InputFace top;
        top.points = rings.back();
        top.topoId = TopologyID::make(featureID, "cap_top");
        top.surface = ringstack::makeCapSurface(rings.back(), sections.back().plane.normal());
        faces.push_back(std::move(top));
    }
    ringstack::orientOutward(faces);

    auto solid = SolidSewer::sew(faces);
    if (solid == nullptr || !solid->checkManifold()) return nullptr;

    {
        int idx = 0;
        for (auto& e : const_cast<std::deque<Edge>&>(solid->edges())) {
            e.topoId = TopologyID::make(featureID, "edge" + std::to_string(idx));
            ++idx;
        }
    }
    ringstack::assignEdgeCurves(*solid);

    return solid;
}

}  // namespace hz::model
