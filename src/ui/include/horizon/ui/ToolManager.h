#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace hz::ui {

class Tool;

/// Manages the registry of available interactive tools and tracks the active tool.
class ToolManager {
public:
    ToolManager();
    ~ToolManager();

    /// Register a tool.  Ownership is transferred to the manager.
    void registerTool(std::unique_ptr<Tool> tool);

    /// Activate the tool identified by the given name.
    /// Returns true if the tool was found and activated.
    bool setActiveTool(const std::string& name);

    /// Returns the currently active tool, or nullptr if none.
    Tool* activeTool() const;

    /// Returns the names of all registered tools.
    std::vector<std::string> toolNames() const;

private:
    std::unordered_map<std::string, std::unique_ptr<Tool>> m_tools;
    Tool* m_activeTool = nullptr;
};

}  // namespace hz::ui
