#include "horizon/modeling/Draft.h"

#include <cmath>
#include <unordered_map>
#include <vector>

#include "RingStack.h"
#include "horizon/topology/Queries.h"

namespace hz::model {

using hz::math::Vec3;
using namespace hz::topo;

namespace {

// Outward-oriented Newell normal of a face (flipped to point away from the
// solid centroid).
Vec3 outwardFaceNormal(const Face& face, const Vec3& solidCentroid) {
    auto verts = faceVertices(&face);
    if (verts.size() < 3) return Vec3::Zero;

    Vec3 n = Vec3::Zero;
    Vec3 c = Vec3::Zero;
    const size_t M = verts.size();
    for (size_t i = 0; i < M; ++i) {
        const Vec3& a = verts[i]->point;
        const Vec3& b = verts[(i + 1) % M]->point;
        n.x += (a.y - b.y) * (a.z + b.z);
        n.y += (a.z - b.z) * (a.x + b.x);
        n.z += (a.x - b.x) * (a.y + b.y);
        c += a;
    }
    if (n.length() < 1e-12) return Vec3::Zero;
    n = n.normalized();
    c = c * (1.0 / static_cast<double>(M));
    if (n.dot(c - solidCentroid) < 0.0) n = -n;
    return n;
}

}  // namespace

std::unique_ptr<topo::Solid> Draft::execute(std::unique_ptr<topo::Solid> solid,
                                            const Vec3& pullDirRaw, const Vec3& neutralPoint,
                                            double angleRad) {
    if (!solid || solid->faceCount() == 0) return nullptr;
    const Vec3 pull = pullDirRaw.normalized();
    const double tanA = std::tan(angleRad);

    // Solid centroid (for outward normal orientation).
    Vec3 centroid = Vec3::Zero;
    size_t vcount = 0;
    for (const auto& v : solid->vertices()) {
        centroid += v.point;
        ++vcount;
    }
    if (vcount == 0) return nullptr;
    centroid = centroid * (1.0 / static_cast<double>(vcount));

    // Collect each vertex's incident lateral-face horizontal normals.
    // A lateral face's normal is roughly perpendicular to the pull direction.
    std::unordered_map<const Vertex*, std::vector<Vec3>> lateralNormals;
    for (const auto& face : solid->faces()) {
        Vec3 n = outwardFaceNormal(face, centroid);
        if (n.length() < 1e-9) continue;
        if (std::abs(n.dot(pull)) > 0.5) continue;  // cap face — skip

        // Horizontal component (perpendicular to pull).
        Vec3 h = (n - pull * n.dot(pull));
        if (h.length() < 1e-9) continue;
        h = h.normalized();
        for (const auto* v : faceVertices(&face)) {
            lateralNormals[v].push_back(h);
        }
    }

    // Move each lateral vertex by the mitered offset of its two incident
    // lateral normals, scaled by height * tan(angle).
    for (auto& v : const_cast<std::deque<Vertex>&>(solid->vertices())) {
        auto it = lateralNormals.find(&v);
        if (it == lateralNormals.end() || it->second.size() < 2) continue;

        // Use the two most-divergent normals (handles the general case; for a
        // prism vertex there are exactly two).
        const auto& normals = it->second;
        Vec3 n1 = normals[0];
        Vec3 n2 = normals[1];
        double worst = n1.dot(n2);
        for (size_t i = 0; i < normals.size(); ++i) {
            for (size_t j = i + 1; j < normals.size(); ++j) {
                double d = normals[i].dot(normals[j]);
                if (d < worst) {
                    worst = d;
                    n1 = normals[i];
                    n2 = normals[j];
                }
            }
        }

        const double height = (v.point - neutralPoint).dot(pull);
        const double delta = height * tanA;
        const double denom = 1.0 + n1.dot(n2);
        if (std::abs(denom) < 1e-9) continue;  // opposing faces — undefined miter
        Vec3 offset = (n1 + n2) * (delta / denom);
        v.point += offset;
    }

    // Rebind surfaces from the updated vertices so tessellation / mass
    // properties stay consistent.
    for (auto& face : const_cast<std::deque<Face>&>(solid->faces())) {
        auto verts = faceVertices(&face);
        if (verts.size() == 4) {
            face.surface = ringstack::makeBilinearPatch(verts[0]->point, verts[1]->point,
                                                        verts[3]->point, verts[2]->point);
        } else if (verts.size() >= 3) {
            std::vector<Vec3> ring;
            ring.reserve(verts.size());
            for (const auto* vv : verts) ring.push_back(vv->point);
            Vec3 n = outwardFaceNormal(face, centroid);
            face.surface = ringstack::makeCapSurface(ring, n);
        }
    }

    // Rebind edge curves from moved endpoints.
    ringstack::assignEdgeCurves(*solid);

    return solid;
}

}  // namespace hz::model
