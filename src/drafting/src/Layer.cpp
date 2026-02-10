#include "horizon/drafting/Layer.h"
#include <algorithm>

namespace hz::draft {

LayerManager::LayerManager() {
    // Create default layer "0"
    LayerProperties defaultLayer;
    defaultLayer.name = "0";
    defaultLayer.color = 0xFFFFFFFF;
    defaultLayer.lineWidth = 1.0;
    defaultLayer.visible = true;
    defaultLayer.locked = false;
    m_layers["0"] = defaultLayer;
    m_currentLayer = "0";
}

void LayerManager::addLayer(const LayerProperties& props) {
    m_layers[props.name] = props;
}

void LayerManager::removeLayer(const std::string& name) {
    // Prevent removing the current layer or the default layer
    if (name == m_currentLayer || name == "0") {
        return;
    }
    m_layers.erase(name);
}

LayerProperties* LayerManager::getLayer(const std::string& name) {
    auto it = m_layers.find(name);
    if (it != m_layers.end()) {
        return &it->second;
    }
    return nullptr;
}

const LayerProperties* LayerManager::getLayer(const std::string& name) const {
    auto it = m_layers.find(name);
    if (it != m_layers.end()) {
        return &it->second;
    }
    return nullptr;
}

std::vector<std::string> LayerManager::layerNames() const {
    std::vector<std::string> names;
    names.reserve(m_layers.size());
    for (const auto& [name, _] : m_layers) {
        names.push_back(name);
    }
    std::sort(names.begin(), names.end());
    return names;
}

void LayerManager::setCurrentLayer(const std::string& name) {
    if (m_layers.find(name) != m_layers.end()) {
        m_currentLayer = name;
    }
}

void LayerManager::clear() {
    m_layers.clear();
    LayerProperties defaultLayer;
    defaultLayer.name = "0";
    defaultLayer.color = 0xFFFFFFFF;
    defaultLayer.lineWidth = 1.0;
    defaultLayer.visible = true;
    defaultLayer.locked = false;
    m_layers["0"] = defaultLayer;
    m_currentLayer = "0";
}

}  // namespace hz::draft
