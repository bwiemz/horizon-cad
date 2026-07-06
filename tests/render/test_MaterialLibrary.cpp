#include <gtest/gtest.h>

#include <set>

#include "horizon/render/MaterialLibrary.h"

using hz::render::Material;
using hz::render::MaterialLibrary;

TEST(MaterialLibraryTest, ContainsRoadmapPresets) {
    // Roadmap §7.3 preset list.
    for (const char* name : {"Brushed Aluminum", "Polished Steel", "Matte Plastic", "Rubber",
                             "Glass", "Carbon Fiber", "Wood"}) {
        EXPECT_TRUE(MaterialLibrary::contains(name)) << name;
    }
    EXPECT_FALSE(MaterialLibrary::contains("Unobtainium"));
}

TEST(MaterialLibraryTest, PresetValuesAreWellFormed) {
    std::set<std::string> names;
    for (const auto& entry : MaterialLibrary::presets()) {
        EXPECT_TRUE(names.insert(entry.name).second) << "duplicate preset " << entry.name;
        const Material& m = entry.material;
        EXPECT_GE(m.metallic, 0.0f);
        EXPECT_LE(m.metallic, 1.0f);
        EXPECT_GE(m.roughness, 0.0f);
        EXPECT_LE(m.roughness, 1.0f);
        EXPECT_GT(m.alpha, 0.0f);
        EXPECT_LE(m.alpha, 1.0f);
        for (double c : {m.color.x, m.color.y, m.color.z}) {
            EXPECT_GE(c, 0.0);
            EXPECT_LE(c, 1.0);
        }
    }
}

TEST(MaterialLibraryTest, MetalsAndDielectricsAreDistinct) {
    const Material steel = MaterialLibrary::find("Polished Steel");
    const Material rubber = MaterialLibrary::find("Rubber");
    EXPECT_FLOAT_EQ(steel.metallic, 1.0f);
    EXPECT_FLOAT_EQ(rubber.metallic, 0.0f);
    EXPECT_LT(steel.roughness, rubber.roughness);

    // Glass is the only transparent preset.
    EXPECT_LT(MaterialLibrary::find("Glass").alpha, 1.0f);
    EXPECT_FLOAT_EQ(steel.alpha, 1.0f);
}

TEST(MaterialLibraryTest, UnknownNameYieldsDefaultMaterial) {
    const Material fallback = MaterialLibrary::find("Unobtainium");
    const Material def{};
    EXPECT_FLOAT_EQ(fallback.metallic, def.metallic);
    EXPECT_FLOAT_EQ(fallback.roughness, def.roughness);
    EXPECT_DOUBLE_EQ(fallback.color.x, def.color.x);
}
