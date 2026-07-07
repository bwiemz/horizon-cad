#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#include "horizon/math/Vec3.h"
#include "horizon/modeling/ExactPredicates.h"
#include "horizon/modeling/PrimitiveFactory.h"

using hz::math::Vec3;
using hz::model::ExactPredicates;
using hz::model::PrimitiveFactory;

// --- orient3D tests ---

TEST(ExactPredicatesTest, PointAbovePlane) {
    Vec3 planePoint(0, 0, 0);
    Vec3 planeNormal(0, 0, 1);
    Vec3 testPoint(0, 0, 5);
    EXPECT_EQ(ExactPredicates::orient3D(planePoint, planeNormal, testPoint), 1);
}

TEST(ExactPredicatesTest, PointBelowPlane) {
    Vec3 planePoint(0, 0, 0);
    Vec3 planeNormal(0, 0, 1);
    Vec3 testPoint(0, 0, -3);
    EXPECT_EQ(ExactPredicates::orient3D(planePoint, planeNormal, testPoint), -1);
}

TEST(ExactPredicatesTest, PointOnPlane) {
    Vec3 planePoint(0, 0, 0);
    Vec3 planeNormal(0, 0, 1);
    Vec3 testPoint(5, 7, 0);
    EXPECT_EQ(ExactPredicates::orient3D(planePoint, planeNormal, testPoint), 0);
}

TEST(ExactPredicatesTest, PointNearPlaneWithinTolerance) {
    Vec3 planePoint(0, 0, 0);
    Vec3 planeNormal(0, 0, 1);
    Vec3 testPoint(1, 2, 1e-12);
    EXPECT_EQ(ExactPredicates::orient3D(planePoint, planeNormal, testPoint), 0);
}

TEST(ExactPredicatesTest, OrientWithTiltedPlane) {
    Vec3 planePoint(1, 1, 1);
    Vec3 planeNormal = Vec3(1, 1, 1).normalized();
    Vec3 above(3, 3, 3);
    Vec3 below(-1, -1, -1);
    EXPECT_EQ(ExactPredicates::orient3D(planePoint, planeNormal, above), 1);
    EXPECT_EQ(ExactPredicates::orient3D(planePoint, planeNormal, below), -1);
}

// --- classifyPoint tests ---

TEST(ExactPredicatesTest, PointInsideBox) {
    auto box = PrimitiveFactory::makeBox(2.0, 2.0, 2.0);
    // Center of box is at (1, 1, 1).
    Vec3 inside(1.0, 1.0, 1.0);
    EXPECT_EQ(ExactPredicates::classifyPoint(inside, *box), -1);
}

TEST(ExactPredicatesTest, PointOutsideBox) {
    auto box = PrimitiveFactory::makeBox(2.0, 2.0, 2.0);
    Vec3 outside(5.0, 5.0, 5.0);
    EXPECT_EQ(ExactPredicates::classifyPoint(outside, *box), 1);
}

TEST(ExactPredicatesTest, PointOnBoxSurface) {
    auto box = PrimitiveFactory::makeBox(2.0, 2.0, 2.0);
    // A vertex of the box (on boundary).
    Vec3 onSurface(0.0, 0.0, 0.0);
    EXPECT_EQ(ExactPredicates::classifyPoint(onSurface, *box), 0);
}

// --- cached-mesh classification (tessellateSolid + classifyPointAgainstMesh) ---

TEST(ExactPredicatesTest, TessellateSolidProducesTriangles) {
    auto box = PrimitiveFactory::makeBox(2.0, 2.0, 2.0);
    auto mesh = ExactPredicates::tessellateSolid(*box);
    EXPECT_FALSE(mesh.empty());
    EXPECT_EQ(mesh.size() % 3u, 0u);  // whole triangles (3 corners each)
}

TEST(ExactPredicatesTest, ClassifyAgainstMeshMatchesClassifyPoint) {
    // The cached-mesh path must agree with the tessellate-per-call path so the
    // Boolean fast path (tessellate once, classify many) is behavior-preserving.
    auto box = PrimitiveFactory::makeBox(2.0, 2.0, 2.0);
    auto mesh = ExactPredicates::tessellateSolid(*box);

    const Vec3 inside(1.0, 1.0, 1.0);
    const Vec3 outside(5.0, 5.0, 5.0);
    const Vec3 boundary(0.0, 0.0, 0.0);

    EXPECT_EQ(ExactPredicates::classifyPointAgainstMesh(inside, mesh),
              ExactPredicates::classifyPoint(inside, *box));
    EXPECT_EQ(ExactPredicates::classifyPointAgainstMesh(outside, mesh),
              ExactPredicates::classifyPoint(outside, *box));
    EXPECT_EQ(ExactPredicates::classifyPointAgainstMesh(boundary, mesh),
              ExactPredicates::classifyPoint(boundary, *box));

    EXPECT_EQ(ExactPredicates::classifyPointAgainstMesh(inside, mesh), -1);
    EXPECT_EQ(ExactPredicates::classifyPointAgainstMesh(outside, mesh), 1);
    EXPECT_EQ(ExactPredicates::classifyPointAgainstMesh(boundary, mesh), 0);
}

// Curved solids (torus/cylinder/sphere) are stored as coarse box topology
// whose face loops enclose little-to-no volume — a torus's eight ring corners
// are all coplanar. tessellateSolid must fall back to smooth surface
// tessellation for them (it is the DrawingProjection occluder), not degenerate
// to a flat, zero-thickness loop mesh.
TEST(ExactPredicatesTest, TessellateCurvedSolidIsNotDegenerate) {
    auto extentZ = [](const std::vector<Vec3>& tris) {
        double lo = 1e300;
        double hi = -1e300;
        for (const Vec3& p : tris) {
            lo = std::min(lo, p.z);
            hi = std::max(hi, p.z);
        }
        return hi - lo;
    };

    auto torus = PrimitiveFactory::makeTorus(10.0, 3.0);
    auto torusMesh = ExactPredicates::tessellateSolid(*torus);
    ASSERT_FALSE(torusMesh.empty());
    // A flat loop mesh would have zero Z extent; the real torus spans 2*r = 6.
    EXPECT_GT(extentZ(torusMesh), 1.0)
        << "torus occluder collapsed to a flat mesh (loop triangulation, not surface)";

    auto cyl = PrimitiveFactory::makeCylinder(4.0, 12.0);
    auto cylMesh = ExactPredicates::tessellateSolid(*cyl);
    ASSERT_FALSE(cylMesh.empty());
    // A cylinder must round: its cross-section is far denser than a 4-gon prism.
    int distinctXY = 0;
    std::vector<std::pair<double, double>> seen;
    for (size_t i = 0; i < cylMesh.size(); ++i) {
        const auto xy = std::make_pair(cylMesh[i].x, cylMesh[i].y);
        bool dup = false;
        for (const auto& s : seen) {
            if (std::abs(s.first - xy.first) < 1e-6 && std::abs(s.second - xy.second) < 1e-6) {
                dup = true;
                break;
            }
        }
        if (!dup) {
            seen.push_back(xy);
            ++distinctXY;
        }
    }
    EXPECT_GT(distinctXY, 8) << "cylinder occluder is a coarse prism, not a rounded surface";
}
