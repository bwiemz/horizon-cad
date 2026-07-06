#include "horizon/render/MaterialLibrary.h"

namespace hz::render {

namespace {

Material make(const math::Vec3& color, float metallic, float roughness, float alpha = 1.0f) {
    Material m;
    m.color = color;
    m.metallic = metallic;
    m.roughness = roughness;
    m.alpha = alpha;
    return m;
}

}  // namespace

const std::vector<MaterialLibrary::Entry>& MaterialLibrary::presets() {
    // Roadmap §7.3 preset list. Metal base colors are measured F0 tints;
    // dielectric base colors are albedo.
    static const std::vector<Entry> kPresets = {
        {"Brushed Aluminum", make({0.91, 0.92, 0.92}, 1.0f, 0.45f)},
        {"Polished Steel", make({0.77, 0.78, 0.78}, 1.0f, 0.12f)},
        {"Matte Plastic", make({0.55, 0.57, 0.60}, 0.0f, 0.85f)},
        {"Rubber", make({0.12, 0.12, 0.13}, 0.0f, 0.95f)},
        {"Glass", make({0.85, 0.90, 0.92}, 0.0f, 0.05f, 0.35f)},
        {"Carbon Fiber", make({0.08, 0.08, 0.09}, 0.35f, 0.40f)},
        {"Wood", make({0.44, 0.30, 0.18}, 0.0f, 0.70f)},
    };
    return kPresets;
}

Material MaterialLibrary::find(const std::string& name) {
    for (const Entry& e : presets()) {
        if (e.name == name) return e.material;
    }
    return Material{};
}

bool MaterialLibrary::contains(const std::string& name) {
    for (const Entry& e : presets()) {
        if (e.name == name) return true;
    }
    return false;
}

}  // namespace hz::render
