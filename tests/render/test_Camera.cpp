#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "horizon/math/BoundingBox.h"
#include "horizon/math/Vec4.h"
#include "horizon/render/Camera.h"

using hz::math::BoundingBox;
using hz::math::Mat4;
using hz::math::Vec3;
using hz::math::Vec4;
using hz::render::Camera;
using hz::render::ProjectionType;

namespace {

bool finite(const Mat4& m) {
    for (const auto& row : m.m) {
        for (double v : row) {
            if (!std::isfinite(v)) return false;
        }
    }
    return true;
}

/// Where @p world lands in normalized device coordinates; inside the view
/// volume when every component is within [-1, 1].
Vec3 ndc(const Camera& camera, const Vec3& world) {
    return (camera.viewProjectionMatrix() * Vec4(world, 1.0)).perspectiveDivide();
}

bool same(const Vec3& a, const Vec3& b) {
    return a.x == b.x && a.y == b.y && a.z == b.z;
}

bool inView(const Camera& camera, const Vec3& world) {
    const Vec3 p = ndc(camera, world);
    return std::abs(p.x) <= 1.0 && std::abs(p.y) <= 1.0 && std::abs(p.z) <= 1.0;
}

Camera viewportCamera(bool orthographic) {
    Camera camera;
    camera.setIsometricView();
    camera.setAutoClipPlanes(true);
    if (orthographic) {
        camera.setOrthographic(100.0, 75.0, -10000.0, 10000.0);
    } else {
        camera.setPerspective(45.0, 4.0 / 3.0, 0.1, 10000.0);
    }
    return camera;
}

}  // namespace

// A wheel spun hard asks for a zoom of nothing, or less: the view stays as
// it was instead of collapsing to a point or turning inside out.
TEST(CameraTest, ZoomByNothingOrLessIsNoZoom) {
    for (bool ortho : {false, true}) {
        Camera camera = viewportCamera(ortho);
        const Vec3 eye = camera.eye();
        const double height = camera.orthoHeight();
        for (double factor : {0.0, -0.2, -5.0, std::numeric_limits<double>::quiet_NaN(),
                              std::numeric_limits<double>::infinity()}) {
            camera.zoom(factor);
            EXPECT_TRUE(same(camera.eye(), eye)) << factor;
            EXPECT_EQ(camera.orthoHeight(), height) << factor;
            EXPECT_TRUE(finite(camera.viewProjectionMatrix())) << factor;
        }
    }
}

// However far the view is zoomed in or out, it projects to finite numbers,
// and what it looks at stays in view.
TEST(CameraTest, ZoomStaysWithinLimitsAndKeepsTheTargetInView) {
    for (bool ortho : {false, true}) {
        Camera camera = viewportCamera(ortho);
        for (int i = 0; i < 2000; ++i) camera.zoom(0.5);
        const double nearest = (camera.eye() - camera.target()).length();
        EXPECT_GE(nearest, Camera::kMinDistance * 0.999);
        EXPECT_TRUE(finite(camera.viewProjectionMatrix()));
        EXPECT_TRUE(inView(camera, camera.target())) << "zoomed in, ortho " << ortho;
        if (ortho) {
            EXPECT_GE(camera.orthoHeight(), Camera::kMinDistance * 0.999);
        }

        for (int i = 0; i < 4000; ++i) camera.zoom(2.0);
        const double farthest = (camera.eye() - camera.target()).length();
        EXPECT_LE(farthest, Camera::kMaxDistance * 1.001);
        EXPECT_TRUE(finite(camera.viewProjectionMatrix()));
        EXPECT_TRUE(inView(camera, camera.target())) << "zoomed out, ortho " << ortho;
        if (ortho) {
            EXPECT_LE(camera.orthoHeight(), Camera::kMaxDistance * 1.001);
        }
    }
}

// An orthographic view zoomed out keeps its shape: width and height change
// together, even where one of them meets its limit.
TEST(CameraTest, OrthographicZoomKeepsTheAspect) {
    Camera camera = viewportCamera(true);
    const double aspect = camera.orthoWidth() / camera.orthoHeight();
    for (int i = 0; i < 200; ++i) camera.zoom(3.0);
    EXPECT_NEAR(camera.orthoWidth() / camera.orthoHeight(), aspect, 1e-9 * aspect);
    for (int i = 0; i < 400; ++i) camera.zoom(1.0 / 3.0);
    EXPECT_NEAR(camera.orthoWidth() / camera.orthoHeight(), aspect, 1e-9 * aspect);
}

// A part fitted is in view, whatever its size: a building, a watch screw,
// and a part a thousandth of a millimetre across. Fixed clip planes at 0.1
// and 10,000 hid the first and the last two.
TEST(CameraTest, FittedPartsOfAnySizeAreInView) {
    for (bool ortho : {false, true}) {
        for (double size : {1e-3, 0.05, 1.0, 250.0, 5e4, 1e7, 1e10}) {
            Camera camera = viewportCamera(ortho);
            const Vec3 offset(size * 3.0, -size, size * 0.5);
            const BoundingBox box(offset - Vec3(size, size, size) * 0.5,
                                  offset + Vec3(size, size, size) * 0.5);
            camera.fitAll(box);
            ASSERT_TRUE(finite(camera.viewProjectionMatrix())) << size;
            for (double x : {box.min().x, box.max().x}) {
                for (double y : {box.min().y, box.max().y}) {
                    for (double z : {box.min().z, box.max().z}) {
                        EXPECT_TRUE(inView(camera, Vec3(x, y, z)))
                            << "size " << size << " ortho " << ortho << " corner " << x << ',' << y
                            << ',' << z;
                    }
                }
            }
        }
    }
}

// A box with no finite size is not fitted: the camera stays where it was.
TEST(CameraTest, FittingAnInfiniteBoxLeavesTheView) {
    Camera camera = viewportCamera(false);
    const Vec3 eye = camera.eye();
    const double inf = std::numeric_limits<double>::infinity();
    camera.fitAll(BoundingBox(Vec3(-inf, 0.0, 0.0), Vec3(inf, 1.0, 1.0)));
    EXPECT_TRUE(same(camera.eye(), eye));
    EXPECT_TRUE(finite(camera.viewProjectionMatrix()));
}

// A pan of no finite size leaves the view where it was.
TEST(CameraTest, PanningByNoFiniteDistanceIsNoPan) {
    Camera camera = viewportCamera(false);
    const Vec3 eye = camera.eye();
    camera.pan(std::numeric_limits<double>::quiet_NaN(), 1.0);
    camera.pan(1.0, std::numeric_limits<double>::infinity());
    EXPECT_TRUE(same(camera.eye(), eye));
}

// Planes set are the planes used, unless the camera is asked to place its
// own; those it places follow its distance from the target.
TEST(CameraTest, AutomaticPlanesFollowTheDistance) {
    Camera camera;
    camera.setIsometricView();
    camera.setPerspective(45.0, 1.0, 0.1, 10000.0);
    EXPECT_EQ(camera.nearPlane(), 0.1);
    EXPECT_EQ(camera.farPlane(), 10000.0);

    camera.setAutoClipPlanes(true);
    const double d = (camera.eye() - camera.target()).length();
    EXPECT_LT(camera.nearPlane(), d);
    EXPECT_GT(camera.farPlane(), d);
    camera.zoom(1000.0);
    EXPECT_GT(camera.farPlane(), 1000.0 * d);
    EXPECT_LT(camera.nearPlane(), 1000.0 * d);
    camera.zoom(1e-9);
    EXPECT_LT(camera.nearPlane(), (camera.eye() - camera.target()).length());
}
