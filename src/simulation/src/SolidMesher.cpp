#include "horizon/simulation/SolidMesher.h"

#include <algorithm>
#include <cmath>

#include "horizon/simulation/BoxMesher.h"
#include "horizon/topology/HalfEdge.h"
#include "horizon/topology/Solid.h"

namespace hz::sim {

namespace {

double component(const math::Vec3& v, int axis) {
    return axis == 0 ? v.x : (axis == 1 ? v.y : v.z);
}

}  // namespace

Aabb solidAabb(const topo::Solid& solid) {
    Aabb box;
    bool first = true;
    for (const topo::Vertex& v : solid.vertices()) {
        const math::Vec3& p = v.point;
        if (first) {
            box.min = p;
            box.max = p;
            first = false;
        } else {
            box.min.x = std::min(box.min.x, p.x);
            box.min.y = std::min(box.min.y, p.y);
            box.min.z = std::min(box.min.z, p.z);
            box.max.x = std::max(box.max.x, p.x);
            box.max.y = std::max(box.max.y, p.y);
            box.max.z = std::max(box.max.z, p.z);
        }
    }
    box.valid = !first;
    return box;
}

TetMesh meshSolidBoundingBox(const topo::Solid& solid, int nx, int ny, int nz) {
    const Aabb box = solidAabb(solid);
    if (!box.valid) return {};

    const double sx = box.max.x - box.min.x;
    const double sy = box.max.y - box.min.y;
    const double sz = box.max.z - box.min.z;

    TetMesh mesh = meshBox(sx, sy, sz, nx, ny, nz);
    // meshBox builds at the origin; shift it to the solid's location.
    for (Node& n : mesh.nodes) {
        n.position = n.position + box.min;
    }
    return mesh;
}

std::vector<int> nodesOnPlane(const TetMesh& mesh, int axis, double coord, double tol) {
    std::vector<int> out;
    for (int i = 0; i < static_cast<int>(mesh.nodes.size()); ++i) {
        if (std::abs(component(mesh.nodes[i].position, axis) - coord) <= tol) {
            out.push_back(i);
        }
    }
    return out;
}

}  // namespace hz::sim
