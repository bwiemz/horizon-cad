// Picking on the CPU (Phase 132): a face by the ray through the cursor, an
// edge by how near it is drawn to the cursor, and not what is hidden.

#include <gtest/gtest.h>

#include <limits>

#include "horizon/math/Mat4.h"
#include "horizon/render/Camera.h"
#include "horizon/render/MeshPicker.h"

using hz::math::Mat4;
using hz::math::Vec3;
using hz::render::Camera;
using hz::render::MeshData;
using hz::render::MeshPicker;

namespace {

/// Two unit squares, one at z = 1 ("top") above one at z = 0 ("bottom"),
/// each two triangles, with the top square's four sides as edges.
MeshData stackedSquares() {
    MeshData mesh;
    const float z[2] = {1.0f, 0.0f};
    for (int s = 0; s < 2; ++s) {
        const auto base = static_cast<uint32_t>(mesh.positions.size() / 3);
        for (const auto& [x, y] : {std::pair{0.f, 0.f}, {1.f, 0.f}, {1.f, 1.f}, {0.f, 1.f}}) {
            mesh.positions.insert(mesh.positions.end(), {x, y, z[s]});
            mesh.normals.insert(mesh.normals.end(), {0.f, 0.f, 1.f});
        }
        mesh.indices.insert(mesh.indices.end(),
                            {base, base + 1, base + 2, base, base + 2, base + 3});
        mesh.triangleFaces.insert(mesh.triangleFaces.end(),
                                  {static_cast<uint32_t>(s), static_cast<uint32_t>(s)});
    }
    mesh.faceTags = {"box/top", "box/bottom"};
    const float c[4][2] = {{0, 0}, {1, 0}, {1, 1}, {0, 1}};
    for (int k = 0; k < 4; ++k) {
        MeshData::Edge edge;
        edge.tag = "box/edge" + std::to_string(k);
        edge.points = {c[k][0], c[k][1], 1.0f, c[(k + 1) % 4][0], c[(k + 1) % 4][1], 1.0f};
        mesh.edges.push_back(edge);
    }
    // One edge of the bottom square, hidden under the top one.
    MeshData::Edge under;
    under.tag = "box/under";
    under.points = {0.2f, 0.5f, 0.0f, 0.8f, 0.5f, 0.0f};
    mesh.edges.push_back(under);
    return mesh;
}

}  // namespace

TEST(MeshPickerTest, TheRayPicksTheNearestFace) {
    const MeshData mesh = stackedSquares();
    const auto hit =
        MeshPicker::pickFace(mesh, Mat4::identity(), Vec3(0.5, 0.5, 5), Vec3(0, 0, -1));
    ASSERT_TRUE(hit.has_value());
    EXPECT_EQ(mesh.faceTags[static_cast<size_t>(hit->face)], "box/top") << "not the one under it";
    EXPECT_NEAR(hit->distance, 4.0, 1e-9);
    EXPECT_NEAR(hit->point.z, 1.0, 1e-9);

    const auto up = MeshPicker::pickFace(mesh, Mat4::identity(), Vec3(0.5, 0.5, -5), Vec3(0, 0, 1));
    ASSERT_TRUE(up.has_value());
    EXPECT_EQ(mesh.faceTags[static_cast<size_t>(up->face)], "box/bottom") << "from below";

    EXPECT_FALSE(MeshPicker::pickFace(mesh, Mat4::identity(), Vec3(2, 2, 5), Vec3(0, 0, -1)))
        << "beside it";
    EXPECT_FALSE(MeshPicker::pickFace(mesh, Mat4::identity(), Vec3(0.5, 0.5, 5), Vec3(0, 0, 1)))
        << "behind the ray's origin";
}

TEST(MeshPickerTest, TheMeshIsPickedWhereItIsPlaced) {
    const MeshData mesh = stackedSquares();
    const Mat4 moved = Mat4::translation(Vec3(10, 0, 0));
    EXPECT_FALSE(MeshPicker::pickFace(mesh, moved, Vec3(0.5, 0.5, 5), Vec3(0, 0, -1)));
    EXPECT_TRUE(MeshPicker::pickFace(mesh, moved, Vec3(10.5, 0.5, 5), Vec3(0, 0, -1)));
}

// Looking straight down: an edge is picked within the tolerance of where it
// is drawn, the nearest one; and not when the face in front hides it.
TEST(MeshPickerTest, AnEdgeIsPickedNearWhereItIsDrawnUnlessHidden) {
    const MeshData mesh = stackedSquares();
    Camera camera;
    camera.lookAt(Vec3(0.5, 0.5, 10), Vec3(0.5, 0.5, 0), Vec3(0, 1, 0));
    const int w = 800;
    const int h = 600;
    // Where the point (1, 0.5, 1), on the right edge, lands on screen.
    const auto screenOf = [&](const Vec3& p) {
        const auto clip = camera.viewProjectionMatrix() * hz::math::Vec4(p, 1.0);
        const Vec3 ndc = clip.perspectiveDivide();
        return std::pair{(ndc.x + 1.0) * 0.5 * w, (1.0 - ndc.y) * 0.5 * h};
    };
    const auto [x, y] = screenOf(Vec3(1, 0.5, 1));
    const double far = std::numeric_limits<double>::infinity();

    const auto hit = MeshPicker::pickEdge(mesh, Mat4::identity(), camera, x + 3, y, w, h, 8, far);
    ASSERT_TRUE(hit.has_value());
    EXPECT_EQ(mesh.edges[static_cast<size_t>(hit->edge)].tag, "box/edge1");
    EXPECT_FALSE(MeshPicker::pickEdge(mesh, Mat4::identity(), camera, x + 20, y, w, h, 8, far))
        << "too far from it";

    // The edge under the top square: picked when nothing is in front, not
    // when the top face is (9 along the ray, the edge 10).
    const auto [ux, uy] = screenOf(Vec3(0.5, 0.5, 0));
    const auto hidden = MeshPicker::pickEdge(mesh, Mat4::identity(), camera, ux, uy, w, h, 8, 9.0);
    EXPECT_FALSE(hidden.has_value()) << "hidden by the face in front";
    const auto seen = MeshPicker::pickEdge(mesh, Mat4::identity(), camera, ux, uy, w, h, 8, far);
    ASSERT_TRUE(seen.has_value());
    EXPECT_EQ(mesh.edges[static_cast<size_t>(seen->edge)].tag, "box/under");
}
