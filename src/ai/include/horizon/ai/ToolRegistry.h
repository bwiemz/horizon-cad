#pragma once

#include <functional>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <vector>

#include "horizon/ai/Conversation.h"

namespace hz::ai {

/// What a tool may touch: Read only looks; Modify changes the document
/// through its undo stack; External reaches outside it (writes a file), which
/// a host asks the user about first.
enum class ToolAccess { Read, Modify, External };

/// What a tool sends back to the model.
struct ToolResult {
    std::string content;  ///< text (usually JSON) the model reads
    bool isError = false;
    bool changedModel = false;  ///< it changed the part or the drawing
    /// Steps it added to the document's undo stack: one for a change (each
    /// change is one step), none for a query or a change that was refused.
    int undoSteps = 0;

    /// @p changedModel: it changed the document, as one undo step.
    static ToolResult ok(const nlohmann::json& value, bool changedModel = false);
    static ToolResult message(std::string text, bool changedModel = false);
    static ToolResult error(std::string message);
};

/// The tools a model may call, by name.
class ToolRegistry {
public:
    using Handler = std::function<ToolResult(const nlohmann::json& input)>;

    /// Add a tool. A later tool of the same name replaces the earlier one.
    void add(ToolSpec spec, ToolAccess access, Handler handler);

    std::vector<ToolSpec> specs() const;
    std::optional<ToolAccess> access(const std::string& name) const;
    bool has(const std::string& name) const { return find(name) != nullptr; }

    /// Run tool @p name. Never throws: an unknown tool, an input that is not
    /// an object or lacks a required property, and anything the handler
    /// throws all come back as an error result the model can read.
    ToolResult run(const std::string& name, const nlohmann::json& input) const;

private:
    struct Entry {
        ToolSpec spec;
        ToolAccess access = ToolAccess::Read;
        Handler handler;
    };
    const Entry* find(const std::string& name) const;

    std::vector<Entry> m_entries;
};

}  // namespace hz::ai
