#include "horizon/simulation/BoxMesher.h"

namespace hz::sim {

TetMesh meshBox(double sizeX, double sizeY, double sizeZ, int nx, int ny, int nz) {
    TetMesh mesh;
    if (nx < 1 || ny < 1 || nz < 1 || sizeX <= 0.0 || sizeY <= 0.0 || sizeZ <= 0.0) {
        return mesh;
    }

    const int sx = nx + 1;
    const int sy = ny + 1;
    const int sz = nz + 1;
    auto index = [&](int i, int j, int k) { return i + sx * (j + sy * k); };

    // Grid nodes.
    mesh.nodes.reserve(static_cast<std::size_t>(sx) * sy * sz);
    for (int k = 0; k < sz; ++k) {
        for (int j = 0; j < sy; ++j) {
            for (int i = 0; i < sx; ++i) {
                mesh.nodes.push_back(
                    Node{math::Vec3(sizeX * i / nx, sizeY * j / ny, sizeZ * k / nz)});
            }
        }
    }

    // Corner offsets for a cell, labelled by (x, y, z) bits.
    static const int off[8][3] = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {1, 1, 0},
                                  {0, 0, 1}, {1, 0, 1}, {0, 1, 1}, {1, 1, 1}};
    // Six conforming tetrahedra sharing the cell's main diagonal (corners 0-7).
    static const int tets[6][4] = {{0, 1, 3, 7}, {0, 3, 2, 7}, {0, 2, 6, 7},
                                   {0, 6, 4, 7}, {0, 4, 5, 7}, {0, 5, 1, 7}};

    mesh.elements.reserve(static_cast<std::size_t>(nx) * ny * nz * 6);
    for (int ck = 0; ck < nz; ++ck) {
        for (int cj = 0; cj < ny; ++cj) {
            for (int ci = 0; ci < nx; ++ci) {
                int corner[8];
                for (int c = 0; c < 8; ++c) {
                    corner[c] = index(ci + off[c][0], cj + off[c][1], ck + off[c][2]);
                }
                for (const auto& t : tets) {
                    mesh.elements.push_back(
                        Tet4{{corner[t[0]], corner[t[1]], corner[t[2]], corner[t[3]]}});
                }
            }
        }
    }

    return mesh;
}

}  // namespace hz::sim
