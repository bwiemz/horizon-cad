#include "horizon/ai/McpServer.h"

#include <algorithm>
#include <istream>
#include <ostream>
#include <utility>

namespace hz::ai {

namespace {

using nlohmann::json;

// JSON-RPC 2.0 error codes.
constexpr int kParseError = -32700;
constexpr int kInvalidRequest = -32600;
constexpr int kMethodNotFound = -32601;
constexpr int kInvalidParams = -32602;

json errorReply(const json& id, int code, const std::string& message) {
    return {{"jsonrpc", "2.0"}, {"id", id}, {"error", {{"code", code}, {"message", message}}}};
}

json resultReply(const json& id, json result) {
    return {{"jsonrpc", "2.0"}, {"id", id}, {"result", std::move(result)}};
}

/// @p value as a reply. (Built with emplace: a json converts to many things,
/// an optional among them.)
std::optional<json> reply(json value) {
    std::optional<json> out;
    out.emplace(std::move(value));
    return out;
}

std::string dump(const json& value) {
    return value.dump(-1, ' ', false, json::error_handler_t::replace);
}

}  // namespace

McpServer::McpServer(const ToolRegistry& tools, std::string name, std::string version)
    : m_tools(tools), m_name(std::move(name)), m_version(std::move(version)) {}

std::optional<json> McpServer::handle(const json& message) {
    if (message.is_array()) {
        // A batch (JSON-RPC 2.0; MCP before 2025-06-18 allowed them).
        if (message.empty()) return reply(errorReply(nullptr, kInvalidRequest, "empty batch"));
        json replies = json::array();
        for (const auto& item : message) {
            auto answer = handleOne(item);
            if (m_afterEach) m_afterEach();
            if (answer) replies.push_back(std::move(*answer));
        }
        if (replies.empty()) return std::nullopt;
        return reply(std::move(replies));
    }
    auto answer = handleOne(message);
    if (m_afterEach) m_afterEach();
    return answer;
}

std::optional<json> McpServer::handleOne(const json& message) {
    if (!message.is_object() || message.value("jsonrpc", json()) != "2.0" ||
        !message.contains("method") || !message["method"].is_string()) {
        // A response from the client (to a request this server never makes)
        // or a malformed message.
        if (message.is_object() && (message.contains("result") || message.contains("error"))) {
            return std::nullopt;
        }
        const json id = message.is_object() && message.contains("id") ? message["id"] : json();
        return reply(errorReply(id, kInvalidRequest, "not a JSON-RPC 2.0 request"));
    }
    const std::string method = message["method"].get<std::string>();
    const bool notification = !message.contains("id");
    const json id = notification ? json() : message["id"];
    if (!notification && !id.is_string() && !id.is_number_integer() && !id.is_number_unsigned()) {
        return reply(errorReply(nullptr, kInvalidRequest, "the id must be a string or an integer"));
    }
    const json params = message.contains("params") ? message["params"] : json::object();
    if (!params.is_object()) {
        if (notification) return std::nullopt;
        return reply(errorReply(id, kInvalidParams, "params must be an object"));
    }

    if (notification) {
        if (method == "notifications/initialized") m_initialized = true;
        return std::nullopt;  // notifications/cancelled and others: nothing to do
    }
    if (method == "initialize") return reply(resultReply(id, initialize(params)));
    if (method == "ping") return reply(resultReply(id, json::object()));
    if (method == "tools/list") return reply(resultReply(id, listTools()));
    if (method == "tools/call") {
        if (!params.contains("name") || !params["name"].is_string()) {
            return reply(errorReply(id, kInvalidParams, "tools/call needs the tool's name"));
        }
        if (!m_tools.has(params["name"].get<std::string>())) {
            return reply(errorReply(id, kInvalidParams,
                                    "unknown tool: " + params["name"].get<std::string>()));
        }
        return reply(resultReply(id, callTool(params)));
    }
    return reply(errorReply(id, kMethodNotFound, "method not found: " + method));
}

json McpServer::initialize(const json& params) {
    // The client's version when it is one this server speaks, else the
    // newest: the client then decides whether it can go on.
    std::string version = kProtocolVersions[0];
    if (params.contains("protocolVersion") && params["protocolVersion"].is_string()) {
        const std::string wanted = params["protocolVersion"].get<std::string>();
        if (std::find(std::begin(kProtocolVersions), std::end(kProtocolVersions), wanted) !=
            std::end(kProtocolVersions)) {
            version = wanted;
        }
    }
    json result = {{"protocolVersion", version},
                   {"capabilities", {{"tools", {{"listChanged", false}}}}},
                   {"serverInfo", {{"name", m_name}, {"version", m_version}}}};
    if (!m_instructions.empty()) result["instructions"] = m_instructions;
    return result;
}

json McpServer::listTools() const {
    json tools = json::array();
    for (const auto& spec : m_tools.specs()) {
        json tool = {{"name", spec.name},
                     {"description", spec.description},
                     {"inputSchema", spec.inputSchema}};
        if (const auto access = m_tools.access(spec.name)) {
            // Hints a client may use to decide what to ask the user about.
            // Anything but a query may change or replace what is there (a
            // feature deleted, a document replaced, a file written over):
            // destructive, to be safe.
            tool["annotations"] = {{"readOnlyHint", *access == ToolAccess::Read},
                                   {"destructiveHint", *access != ToolAccess::Read},
                                   {"openWorldHint", *access == ToolAccess::External}};
        }
        tools.push_back(std::move(tool));
    }
    return {{"tools", std::move(tools)}};
}

json McpServer::callTool(const json& params) const {
    const std::string name = params["name"].get<std::string>();
    const json arguments = params.contains("arguments") ? params["arguments"] : json::object();
    const ToolResult result = m_tools.run(name, arguments);
    json content = json::array({{{"type", "text"}, {"text", result.content}}});
    for (const auto& image : result.images) {
        content.push_back(
            {{"type", "image"}, {"data", image.base64}, {"mimeType", image.mediaType}});
    }
    return {{"content", std::move(content)}, {"isError", result.isError}};
}

std::optional<std::string> McpServer::handleLine(const std::string& line) {
    if (line.find_first_not_of(" \t\r\n") == std::string::npos) return std::nullopt;
    const json message = json::parse(line, nullptr, false);
    if (message.is_discarded()) return dump(errorReply(nullptr, kParseError, "not JSON"));
    std::optional<json> answer;
    try {
        answer = handle(message);
    } catch (const std::exception& e) {
        // Nothing here throws by design; still, the server keeps serving.
        const json id = message.is_object() && message.contains("id") ? message["id"] : json();
        answer = reply(errorReply(id, -32603, std::string("internal error: ") + e.what()));
    }
    if (!answer) return std::nullopt;
    return dump(*answer);
}

void McpServer::serve(std::istream& in, std::ostream& out) {
    std::string line;
    while (std::getline(in, line)) {
        if (const auto reply = handleLine(line)) {
            out << *reply << '\n';
            out.flush();
        }
    }
}

}  // namespace hz::ai
