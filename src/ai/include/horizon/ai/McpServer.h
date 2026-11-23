#pragma once

#include <iosfwd>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>

#include "horizon/ai/ToolRegistry.h"

namespace hz::ai {

/// A Model Context Protocol server over a ToolRegistry: JSON-RPC 2.0, one
/// message per line (the stdio transport). Lets an MCP client (Claude
/// Desktop, Claude Code, Cursor, and others) call Horizon CAD's tools.
///
/// Implements initialize (version negotiation), notifications/initialized,
/// ping, tools/list and tools/call. A tool's failure is a result with isError
/// set, which the client's model reads; a malformed message is a JSON-RPC
/// error. Never throws.
class McpServer {
public:
    /// The protocol revisions spoken, newest first.
    static constexpr const char* kProtocolVersions[] = {"2025-06-18", "2025-03-26", "2024-11-05"};

    McpServer(const ToolRegistry& tools, std::string name, std::string version);

    /// What the server tells the client's model about itself at
    /// initialization (how to use the tools).
    void setInstructions(std::string instructions) { m_instructions = std::move(instructions); }

    /// The reply to one message (a request, a notification, or a batch of
    /// them), or nothing for a notification.
    std::optional<nlohmann::json> handle(const nlohmann::json& message);
    /// The reply to one line of input: a parse error for a line that is not
    /// JSON, nothing for a blank line or a notification.
    std::optional<std::string> handleLine(const std::string& line);

    /// Serve until @p in ends: a line in, a line out (when there is a reply),
    /// flushed at once.
    void serve(std::istream& in, std::ostream& out);

    bool initialized() const { return m_initialized; }

private:
    std::optional<nlohmann::json> handleOne(const nlohmann::json& message);
    nlohmann::json initialize(const nlohmann::json& params);
    nlohmann::json listTools() const;
    nlohmann::json callTool(const nlohmann::json& params) const;

    const ToolRegistry& m_tools;
    std::string m_name;
    std::string m_version;
    std::string m_instructions;
    bool m_initialized = false;
};

}  // namespace hz::ai
