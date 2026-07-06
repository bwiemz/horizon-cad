#include <gtest/gtest.h>

#include <filesystem>

#include "horizon/math/Vec3.h"
#include "horizon/render/MaterialLibrary.h"
#include "horizon/render/PathTracer.h"

using hz::math::Vec3;
using hz::render::Material;
using hz::render::MeshData;
using hz::render::PathTracer;
using hz::render::RtCamera;
using hz::render::RtImage;
using hz::render::RtSettings;

namespace {

/// Unit cube mesh centered at the origin.
MeshData makeCubeMesh(double half = 5.0) {
    MeshData mesh;
    const double h = half;
    const double verts[8][3] = {{-h, -h, -h}, {h, -h, -h}, {h, h, -h}, {-h, h, -h},
                                {-h, -h, h},  {h, -h, h},  {h, h, h},  {-h, h, h}};
    for (const auto& v : verts) {
        mesh.positions.insert(
            mesh.positions.end(),
            {static_cast<float>(v[0]), static_cast<float>(v[1]), static_cast<float>(v[2])});
        mesh.normals.insert(mesh.normals.end(), {0.f, 0.f, 1.f});
    }
    const uint32_t quads[6][4] = {{0, 3, 2, 1}, {4, 5, 6, 7}, {0, 1, 5, 4},
                                  {2, 3, 7, 6}, {1, 2, 6, 5}, {3, 0, 4, 7}};
    for (const auto& q : quads) {
        mesh.indices.insert(mesh.indices.end(), {q[0], q[1], q[2], q[0], q[2], q[3]});
    }
    return mesh;
}

RtSettings smallSettings() {
    RtSettings s;
    s.width = 64;
    s.height = 48;
    s.samplesPerPixel = 8;
    s.maxBounces = 3;
    return s;
}

RtCamera lookAtOrigin() {
    RtCamera cam;
    cam.eye = Vec3(18.0, -18.0, 12.0);
    cam.target = Vec3(0.0, 0.0, 0.0);
    return cam;
}

double luminance(const Vec3& c) {
    return 0.2126 * c.x + 0.7152 * c.y + 0.0722 * c.z;
}

}  // namespace

TEST(PathTracerTest, EmptySceneRendersEnvironmentOnly) {
    PathTracer tracer;
    const RtImage image = tracer.render(lookAtOrigin(), smallSettings());
    ASSERT_EQ(image.pixels.size(), 64u * 48u);
    // Sky above is brighter than ground below.
    const double top = luminance(image.pixels[64 * 2 + 32]);
    const double bottom = luminance(image.pixels[64 * 45 + 32]);
    EXPECT_GT(top, bottom);
    EXPECT_GT(top, 0.1);
}

TEST(PathTracerTest, CubeCoversImageCenter) {
    PathTracer tracer;
    tracer.addMesh(makeCubeMesh(), hz::render::MaterialLibrary::find("Matte Plastic"));
    EXPECT_EQ(tracer.triangleCount(), 12u);

    const RtSettings settings = smallSettings();
    const RtImage image = tracer.render(lookAtOrigin(), settings);

    // The cube must shade differently from the pure environment at center.
    PathTracer empty;
    const RtImage background = empty.render(lookAtOrigin(), settings);
    const Vec3 c = image.pixels[64 * 24 + 32];
    const Vec3 b = background.pixels[64 * 24 + 32];
    EXPECT_GT(c.distanceTo(b), 0.01) << "cube did not affect the image center";

    // Corners still see the environment.
    EXPECT_LT(image.pixels[0].distanceTo(background.pixels[0]), 1e-9);
}

TEST(PathTracerTest, DeterministicAndThreadInvariant) {
    PathTracer tracer;
    tracer.addMesh(makeCubeMesh(), hz::render::MaterialLibrary::find("Polished Steel"));

    RtSettings settings = smallSettings();
    settings.multithreaded = true;
    const RtImage a = tracer.render(lookAtOrigin(), settings);
    const RtImage b = tracer.render(lookAtOrigin(), settings);
    settings.multithreaded = false;
    const RtImage c = tracer.render(lookAtOrigin(), settings);

    ASSERT_EQ(a.pixels.size(), c.pixels.size());
    for (size_t i = 0; i < a.pixels.size(); ++i) {
        EXPECT_DOUBLE_EQ(a.pixels[i].x, b.pixels[i].x);
        EXPECT_DOUBLE_EQ(a.pixels[i].x, c.pixels[i].x);
        EXPECT_DOUBLE_EQ(a.pixels[i].z, c.pixels[i].z);
    }
}

TEST(PathTracerTest, SunSideIsBrighterThanShadowSide) {
    PathTracer tracer;
    tracer.addMesh(makeCubeMesh(), hz::render::MaterialLibrary::find("Matte Plastic"));

    RtSettings settings = smallSettings();
    settings.samplesPerPixel = 32;
    settings.sunDirection = Vec3(0.0, 0.0, 1.0);  // straight down from above

    // Look at the cube's lit top face vs its unlit -Y side face.
    RtCamera topCam;
    topCam.eye = Vec3(0.0, 0.0, 30.0);
    topCam.target = Vec3(0.0, 0.0, 0.0);
    topCam.up = Vec3(0.0, 1.0, 0.0);
    const RtImage topView = tracer.render(topCam, settings);

    RtCamera sideCam;
    sideCam.eye = Vec3(0.0, -30.0, 0.0);
    sideCam.target = Vec3(0.0, 0.0, 0.0);
    const RtImage sideView = tracer.render(sideCam, settings);

    const double litTop = luminance(topView.pixels[64 * 24 + 32]);
    const double shadedSide = luminance(sideView.pixels[64 * 24 + 32]);
    EXPECT_GT(litTop, shadedSide * 1.5) << "direct sun should dominate";
}

TEST(PathTracerTest, MoreSamplesReduceVariance) {
    PathTracer tracer;
    tracer.addMesh(makeCubeMesh(), hz::render::MaterialLibrary::find("Brushed Aluminum"));

    RtSettings low = smallSettings();
    low.samplesPerPixel = 2;
    RtSettings high = smallSettings();
    high.samplesPerPixel = 64;

    // Variance proxy: mean absolute difference between two independent seeds.
    auto meanDiff = [&](const RtSettings& base) {
        RtSettings s1 = base;
        s1.seed = 11;
        RtSettings s2 = base;
        s2.seed = 42;
        const RtImage a = tracer.render(lookAtOrigin(), s1);
        const RtImage b = tracer.render(lookAtOrigin(), s2);
        double sum = 0.0;
        for (size_t i = 0; i < a.pixels.size(); ++i) sum += a.pixels[i].distanceTo(b.pixels[i]);
        return sum / static_cast<double>(a.pixels.size());
    };

    EXPECT_LT(meanDiff(high), meanDiff(low));
}

/// Review regression: NEE must evaluate the full BRDF — pure metals have no
/// diffuse lobe, so without specular NEE the sun had zero effect on them.
TEST(PathTracerTest, MetalsRespondToSunRadiance) {
    PathTracer tracer;
    tracer.addMesh(makeCubeMesh(), hz::render::MaterialLibrary::find("Polished Steel"));

    RtSettings dim = smallSettings();
    dim.samplesPerPixel = 16;
    dim.sunDirection = Vec3(0.2, -0.3, 1.0);
    dim.sunRadiance = Vec3(0.1, 0.1, 0.1);
    RtSettings bright = dim;
    bright.sunRadiance = Vec3(10.0, 10.0, 10.0);

    const RtImage a = tracer.render(lookAtOrigin(), dim);
    const RtImage b = tracer.render(lookAtOrigin(), bright);

    double diff = 0.0;
    for (size_t i = 0; i < a.pixels.size(); ++i) diff += a.pixels[i].distanceTo(b.pixels[i]);
    EXPECT_GT(diff, 1e-3) << "sun radiance had no effect on a pure metal";
}

/// Review regression: Material::alpha was silently ignored — Glass rendered
/// fully opaque. A dark object placed BEHIND a glass slab must show through.
TEST(PathTracerTest, GlassTransmitsLight) {
    RtSettings settings = smallSettings();
    settings.samplesPerPixel = 32;

    RtCamera cam;
    cam.eye = Vec3(0.0, -30.0, 0.0);
    cam.target = Vec3(0.0, 0.0, 0.0);

    const auto behind = hz::math::Mat4::translation(Vec3(0.0, 15.0, 0.0));

    PathTracer glassOnly;
    glassOnly.addMesh(makeCubeMesh(), hz::render::MaterialLibrary::find("Glass"));

    PathTracer glassWithRubberBehind;
    glassWithRubberBehind.addMesh(makeCubeMesh(), hz::render::MaterialLibrary::find("Glass"));
    glassWithRubberBehind.addMesh(makeCubeMesh(), hz::render::MaterialLibrary::find("Rubber"),
                                  behind);

    const RtImage a = glassOnly.render(cam, settings);
    const RtImage b = glassWithRubberBehind.render(cam, settings);

    // Through the glass, the dark rubber block replaces the bright sky:
    // the center pixel darkens noticeably. Were alpha ignored (opaque
    // glass), the object behind could not influence the pixel.
    const size_t center = 64 * 24 + 32;
    EXPECT_LT(luminance(b.pixels[center]), luminance(a.pixels[center]) * 0.8)
        << "object behind glass is invisible — transmission not working";
}

TEST(PathTracerTest, SavesPpm) {
    PathTracer tracer;
    tracer.addMesh(makeCubeMesh(), hz::render::MaterialLibrary::find("Wood"));
    const RtImage image = tracer.render(lookAtOrigin(), smallSettings());

    const auto path = std::filesystem::temp_directory_path() / "hz_pathtracer_test.ppm";
    ASSERT_TRUE(image.savePpm(path.string()));
    EXPECT_GT(std::filesystem::file_size(path), size_t{64 * 48 * 3});
    std::filesystem::remove(path);
}
