#pragma once

#include <memory>
#include <string>

#include "horizon/ai/ToolRegistry.h"

namespace hz::doc {
class Document;
}

namespace hz::mcp {

/// The document an MCP client works on, and the tools it is given: the CAD
/// tools (hz::ai::registerCadTools), and new_document, open_document,
/// save_document, export_document and get_session_info for its files.
///
/// Opening or starting a document replaces the one held, after the tool call
/// that asked for it returns (applyPending()): a tool's handler cannot be
/// replaced while it runs.
class McpSession {
public:
    /// Start with an empty part (or drawing).
    explicit McpSession(bool part = true);
    ~McpSession();

    McpSession(const McpSession&) = delete;
    McpSession& operator=(const McpSession&) = delete;

    /// Open @p path (.hzpart, .hcad or .dxf) as the document now. False, and
    /// why in @p error, when it cannot be read; the document is unchanged.
    bool open(const std::string& path, std::string* error = nullptr);

    ai::ToolRegistry& tools() { return m_tools; }
    doc::Document& document() { return *m_document; }
    const std::string& path() const { return m_path; }

    /// Put in place a document a tool asked for. Call after each message.
    void applyPending();

private:
    void bind();
    void registerDocumentTools();

    std::unique_ptr<doc::Document> m_document;
    std::string m_path;
    std::unique_ptr<doc::Document> m_pending;  ///< to be the document next
    std::string m_pendingPath;
    ai::ToolRegistry m_tools;
};

/// What the MCP server tells a client's model about Horizon CAD.
std::string serverInstructions();

}  // namespace hz::mcp
