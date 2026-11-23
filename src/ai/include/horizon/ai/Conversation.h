#pragma once

#include <nlohmann/json.hpp>
#include <string>
#include <vector>

namespace hz::ai {

/// Who said it. Tool results travel in a User message, as the providers
/// expect; there is no separate "tool" role here.
enum class Role { User, Assistant };

/// One piece of a message, in no provider's format: each adapter writes and
/// reads its own wire format from these.
struct ContentBlock {
    enum class Kind {
        Text,        ///< `text`
        ToolCall,    ///< the model asks for tool `name` with `input`; `id` names the call
        ToolResult,  ///< the answer to call `id` (tool `name`): `text`, `isError`
        Opaque,      ///< a block of provider `provider` this code does not read but
                     ///< must send back unchanged (`raw`): a thinking block, a
                     ///< Gemini part carrying a thought signature
    };

    Kind kind = Kind::Text;
    std::string text;
    std::string id;
    std::string name;
    nlohmann::json input;
    bool isError = false;
    std::string provider;
    nlohmann::json raw;

    /// Not noexcept: its JSON members are built on construction, and building
    /// a JSON value may allocate. (Defaulted with its own exception
    /// specification, as C++20 allows; the implicit one would claim noexcept.)
    ContentBlock() noexcept(false) = default;

    static ContentBlock makeText(std::string text);
    static ContentBlock makeToolCall(std::string id, std::string name, nlohmann::json input);
    static ContentBlock makeToolResult(std::string callId, std::string name, std::string content,
                                       bool isError);
    static ContentBlock makeOpaque(std::string provider, nlohmann::json raw);
};

struct Message {
    Role role = Role::User;
    std::vector<ContentBlock> blocks;

    /// Its text blocks, joined by blank lines.
    std::string text() const;
    /// Its tool calls, in order.
    std::vector<const ContentBlock*> toolCalls() const;
};

/// A tool as offered to the model: its name, what it does, and a JSON Schema
/// for its input, kept to what all providers accept (type, properties,
/// required, items, enum, description, minimum, maximum).
struct ToolSpec {
    std::string name;
    std::string description;
    nlohmann::json inputSchema;
};

struct ChatRequest {
    std::string model;
    std::string system;
    std::vector<Message> messages;
    std::vector<ToolSpec> tools;
    int maxTokens = 16000;
};

enum class StopReason {
    EndTurn,    ///< the model finished
    ToolUse,    ///< it waits for the results of its tool calls
    MaxTokens,  ///< it was cut off
    Refusal,    ///< a safety system or the model declined
    Error,      ///< the request failed: see ChatResponse::error
    Other,
};

struct Usage {
    long long inputTokens = 0;
    long long outputTokens = 0;
};

struct ChatResponse {
    Message message{Role::Assistant, {}};
    StopReason stop = StopReason::Other;
    Usage usage;
    std::string error;          ///< why the request failed, for the user
    std::string refusalDetail;  ///< what the provider said about a refusal, if anything
};

}  // namespace hz::ai
