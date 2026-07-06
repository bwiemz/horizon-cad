#pragma once

#include <string>
#include <vector>

#include "horizon/render/SceneGraph.h"

namespace hz::render {

/// Named PBR material presets (Phase 67, roadmap §7.3). Values follow the
/// metallic-roughness workflow the viewport shader consumes: dielectrics get
/// metallic 0 and rely on the fixed 4% F0; metals carry their reflectance
/// tint in the base color.
class MaterialLibrary {
public:
    struct Entry {
        std::string name;
        Material material;
    };

    /// The built-in presets, in display order.
    static const std::vector<Entry>& presets();

    /// Look up a preset by (case-sensitive) name. Returns the default
    /// Material when the name is unknown.
    static Material find(const std::string& name);

    /// True when @p name matches a preset.
    static bool contains(const std::string& name);
};

}  // namespace hz::render
