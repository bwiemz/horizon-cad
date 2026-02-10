#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <unordered_map>

namespace hz::draft {

struct LayerProperties {
    std::string name;
    uint32_t color = 0xFFFFFFFF;  // ARGB white
    double lineWidth = 1.0;
    bool visible = true;
    bool locked = false;
};

class LayerManager {
public:
    LayerManager();

    void addLayer(const LayerProperties& props);
    void removeLayer(const std::string& name);
    LayerProperties* getLayer(const std::string& name);
    const LayerProperties* getLayer(const std::string& name) const;
    std::vector<std::string> layerNames() const;

    const std::string& currentLayer() const { return m_currentLayer; }
    void setCurrentLayer(const std::string& name);

private:
    std::unordered_map<std::string, LayerProperties> m_layers;
    std::string m_currentLayer;
};

}  // namespace hz::draft
