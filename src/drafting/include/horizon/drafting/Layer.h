#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace hz::draft {

struct LayerProperties {
    std::string name;
    uint32_t color = 0xFFFFFFFF;  // ARGB white
    double lineWidth = 1.0;
    int lineType = 1;  ///< Default = Continuous (see LineType.h).
    bool visible = true;
    bool locked = false;

    bool operator==(const LayerProperties&) const = default;
};

class LayerManager {
public:
    LayerManager();

    void addLayer(const LayerProperties& props);
    void removeLayer(const std::string& name);

    /// Rename a layer, keeping its properties; the current layer follows it.
    /// False, and nothing changed, for the default layer "0", a layer that
    /// does not exist, or a name that is empty or already taken.
    bool renameLayer(const std::string& from, const std::string& to);
    LayerProperties* getLayer(const std::string& name);
    const LayerProperties* getLayer(const std::string& name) const;
    std::vector<std::string> layerNames() const;

    const std::string& currentLayer() const { return m_currentLayer; }
    void setCurrentLayer(const std::string& name);

    void clear();

    /// The same layers, alike in every property, and the same current one.
    bool operator==(const LayerManager&) const = default;

private:
    std::unordered_map<std::string, LayerProperties> m_layers;
    std::string m_currentLayer;
};

}  // namespace hz::draft
