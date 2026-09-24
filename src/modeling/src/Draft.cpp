#include "horizon/modeling/Draft.h"

#include <cmath>
#include <unordered_map>
#include <utility>
#include <vector>

#include "RingStack.h"
#include "horizon/topology/Queries.h"

namespace hz::model {

using hz::math::Vec3;
using namespace hz::topo;

namespace {

// A face's outward normal: its loop normal, turned over when its body is
// wound inside out (@p orientation -1). The winding says which side is
// outside on any shape; comparing with a centroid does not, and turned the
// walls of holes and the inner faces of an L-shaped part the wrong way.
Vec3 outwardFaceNormal(const Face& face, double orientation) {
    return loopNormal(&face) * orientation;
}

}  // namespace

std::unique_ptr<topo::Solid> Draft::execute(std::unique_ptr<topo::Solid> solid,
                                            const Vec3& pullDirRaw, const Vec3& neutralPoint,
                                            double angleRad) {
    if (!solid || solid->faceCount() == 0) return nullptr;
    const Vec3 pull = pullDirRaw.normalized();
    const double tanA = std::tan(angleRad);

    // Each body's winding, from its signed volume: +1 when its loops' normals
    // point outward, -1 when it is wound inside out.
    std::unordered_map<const Shell*, double> orientations;
    for (const auto& shell : solid->shells()) {
        orientations[&shell] = signedVolume(shell) < 0.0 ? -1.0 : 1.0;
    }
    const auto orientationOf = [&orientations](const Face& face) {
        const auto it = orientations.find(face.shell);
        return it != orientations.end() ? it->second : 1.0;
    };

    // Collect each vertex's incident lateral-face horizontal normals.
    // A lateral face's normal is roughly perpendicular to the pull direction.
    std::unordered_map<const Vertex*, std::vector<Vec3>> lateralNormals;
    for (const auto& face : solid->faces()) {
        Vec3 n = outwardFaceNormal(face, orientationOf(face));
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
            Vec3 n = outwardFaceNormal(face, orientationOf(face));
            face.surface = ringstack::makeCapSurface(ring, n);
        }
    }

    // Rebind edge curves from moved endpoints.
    ringstack::assignEdgeCurves(*solid);

    return solid;
}

}  // namespace hz::model
