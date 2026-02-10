#include "horizon/ui/ToolManager.h"
#include "horizon/ui/Tool.h"

namespace hz::ui {

ToolManager::ToolManager() = default;
ToolManager::~ToolManager() = default;

void ToolManager::registerTool(std::unique_ptr<Tool> tool) {
    if (!tool) return;
    std::string toolName = tool->name();
    m_tools[toolName] = std::move(tool);
}

bool ToolManager::setActiveTool(const std::string& name) {
    auto it = m_tools.find(name);
    if (it == m_tools.end()) {
        return false;
    }

    // Deactivate the previous tool.
    if (m_activeTool) {
        m_activeTool->deactivate();
    }

    m_activeTool = it->second.get();
    return true;
}

Tool* ToolManager::activeTool() const {
    return m_activeTool;
}

std::vector<std::string> ToolManager::toolNames() const {
    std::vector<std::string> names;
    names.reserve(m_tools.size());
    for (const auto& [name, tool] : m_tools) {
        names.push_back(name);
    }
    return names;
}

}  // namespace hz::ui
