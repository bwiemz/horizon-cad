#include <gtest/gtest.h>

#include <cmath>
#include <deque>
#include <numbers>

#include "horizon/modeling/MassProperties.h"
#include "horizon/modeling/PrimitiveFactory.h"
#include "horizon/topology/Solid.h"

using namespace hz::model;
using hz::math::Vec3;

namespace {
void translate(hz::topo::Solid& solid, const Vec3& d) {
    for (auto& v : const_cast<std::deque<hz::topo::Vertex>&>(solid.vertices()))
        v.point = v.point + d;
}
}  // namespace

// ---------------------------------------------------------------------------
// Volume / area / centroid — exact for a box (planar faces)
// ---------------------------------------------------------------------------

TEST(MassPropertiesTest, BoxVolumeAndArea) {
    auto box = PrimitiveFactory::makeBox(2.0, 3.0, 4.0);  // [0,2]x[0,3]x[0,4]
    auto mp = MassPropertiesCalculator::compute(*box);
    ASSERT_TRUE(mp.valid);
    EXPECT_NEAR(mp.volume, 24.0, 1e-6);                              // 2*3*4
    EXPECT_NEAR(mp.surfaceArea, 2 * (2 * 3 + 3 * 4 + 2 * 4), 1e-6);  // 52
    EXPECT_NEAR(mp.centerOfMass.x, 1.0, 1e-6);
    EXPECT_NEAR(mp.centerOfMass.y, 1.5, 1e-6);
    EXPECT_NEAR(mp.centerOfMass.z, 2.0, 1e-6);
}

TEST(MassPropertiesTest, VolumeInvariantUnderTranslation) {
    auto box = PrimitiveFactory::makeBox(2.0, 2.0, 2.0);
    translate(*box, Vec3(100, -50, 30));
    auto mp = MassPropertiesCalculator::compute(*box);
    ASSERT_TRUE(mp.valid);
    EXPECT_NEAR(mp.volume, 8.0, 1e-6);
    EXPECT_NEAR(mp.centerOfMass.x, 101.0, 1e-6);
    EXPECT_NEAR(mp.centerOfMass.y, -49.0, 1e-6);
    EXPECT_NEAR(mp.centerOfMass.z, 31.0, 1e-6);
}

// ---------------------------------------------------------------------------
// Inertia tensor — closed form for a solid box about its center
//   Ixx = m(h²+d²)/12, etc.
// ---------------------------------------------------------------------------

TEST(MassPropertiesTest, BoxInertiaTensor) {
    const double w = 2.0, h = 4.0, d = 6.0;
    auto box = PrimitiveFactory::makeBox(w, h, d);
    auto mp = MassPropertiesCalculator::compute(*box);  // unit density → mass = volume
    ASSERT_TRUE(mp.valid);
    const double m = mp.volume;  // density 1
    EXPECT_NEAR(mp.inertia.at(0, 0), m * (h * h + d * d) / 12.0, 1e-4);
    EXPECT_NEAR(mp.inertia.at(1, 1), m * (w * w + d * d) / 12.0, 1e-4);
    EXPECT_NEAR(mp.inertia.at(2, 2), m * (w * w + h * h) / 12.0, 1e-4);
    // Products of inertia vanish for a box aligned to its centroidal axes.
    EXPECT_NEAR(mp.inertia.at(0, 1), 0.0, 1e-6);
    EXPECT_NEAR(mp.inertia.at(1, 2), 0.0, 1e-6);
    EXPECT_NEAR(mp.inertia.at(0, 2), 0.0, 1e-6);
}

// ---------------------------------------------------------------------------
// Material / mass
// ---------------------------------------------------------------------------

TEST(MassPropertiesTest, MaterialScalesMass) {
    auto box = PrimitiveFactory::makeBox(1.0, 1.0, 1.0);  // volume 1
    Material steel = Material::steel();
    auto mp = MassPropertiesCalculator::compute(*box, &steel);
    ASSERT_TRUE(mp.valid);
    EXPECT_NEAR(mp.density, 7850.0, 1e-9);
    EXPECT_NEAR(mp.mass, 7850.0, 1e-6);  // density * volume
    // Inertia scales with density too.
    auto unit = MassPropertiesCalculator::compute(*box);
    EXPECT_NEAR(mp.inertia.at(0, 0), 7850.0 * unit.inertia.at(0, 0), 1e-3);
}

TEST(MassPropertiesTest, PresetDensities) {
    EXPECT_NEAR(Material::steel().density, 7850.0, 1e-9);
    EXPECT_NEAR(Material::aluminum().density, 2700.0, 1e-9);
    EXPECT_NEAR(Material::titanium().density, 4500.0, 1e-9);
    EXPECT_NEAR(Material::absPlastic().density, 1050.0, 1e-9);
}

// ---------------------------------------------------------------------------
// Curved primitive: properties reflect the actual B-Rep polytope (an inscribed
// prism), so the centroid is symmetry-exact and the volume is bounded by the
// ideal round cylinder.
// ---------------------------------------------------------------------------

TEST(MassPropertiesTest, CylinderCentroidAndBounds) {
    auto cyl = PrimitiveFactory::makeCylinder(5.0, 10.0);  // [-5,5]×[-5,5]×[0,10]
    ASSERT_NE(cyl, nullptr);
    auto mp = MassPropertiesCalculator::compute(*cyl);
    ASSERT_TRUE(mp.valid);

    // Centroid is exact by symmetry regardless of facet count.
    EXPECT_NEAR(mp.centerOfMass.x, 0.0, 1e-9);
    EXPECT_NEAR(mp.centerOfMass.y, 0.0, 1e-9);
    EXPECT_NEAR(mp.centerOfMass.z, 5.0, 1e-9);

    // The inscribed B-Rep prism has positive volume bounded above by the ideal
    // round cylinder π r² h ≈ 785.4.
    const double roundVol = std::numbers::pi * 25.0 * 10.0;
    EXPECT_GT(mp.volume, 0.0);
    EXPECT_LE(mp.volume, roundVol + 1e-6);
    EXPECT_GT(mp.surfaceArea, 0.0);
}
