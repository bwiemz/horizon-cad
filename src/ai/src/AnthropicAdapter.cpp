// The Anthropic Messages API (POST /v1/messages), API version 2026-10-21.
//
// That version refuses temperature, top_p and top_k on every model, so none
// is ever sent. Thinking blocks (always on for the current models) come back
// in the content and must be sent back unchanged on later turns: they are
// kept as opaque blocks. A response is checked for a refusal before its
// content is read.

#include <memory>
#include <string>
#include <utility>

#include "AdapterUtil.h"
#include "horizon/ai/Provider.h"

namespace hz::ai {

namespace {

using nlohmann::json;
using namespace detail;

constexpr const char* kApiVersion = "2026-10-21";
constexpr const char* kFallbackBeta = "server-side-fallback-2026-07-01";
constexpr const char* kProvider = "anthropic";

bool isEffort(const std::string& effort) {
    return effort == "low" || effort == "medium" || effort == "high" || effort == "xhigh" ||
           effort == "max";
}

json contentOf(const Message& message) {
    json content = json::array();
    for (const auto& block : message.blocks) {
        switch (block.kind) {
            case ContentBlock::Kind::Text:
                // The API refuses an empty text block.
                if (!block.text.empty())
                    content.push_back({{"type", "text"}, {"text", block.text}});
                break;
            case ContentBlock::Kind::ToolCall:
                content.push_back(
                    {{"type", "tool_use"},
                     {"id", block.id},
                     {"name", block.name},
                     {"input", block.input.is_object() ? block.input : json::object()}});
                break;
            case ContentBlock::Kind::ToolResult: {
                const std::string text = block.text.empty() ? "(no output)" : block.text;
                json result = {{"type", "tool_result"}, {"tool_use_id", block.id}};
                if (block.images.empty()) {
                    result["content"] = text;
                } else {
                    // A result may carry images among its text (a view).
                    json parts = json::array({{{"type", "text"}, {"text", text}}});
                    for (const auto& image : block.images) {
                        parts.push_back({{"type", "image"},
                                         {"source",
                                          {{"type", "base64"},
                                           {"media_type", image.mediaType},
                                           {"data", image.base64}}}});
                    }
                    result["content"] = std::move(parts);
                }
                if (block.isError) result["is_error"] = true;
                content.push_back(std::move(result));
                break;
            }
            case ContentBlock::Kind::Opaque:
                // Another provider's blocks mean nothing here and are dropped.
                if (block.provider == kProvider && block.raw.is_object()) {
                    content.push_back(block.raw);
                }
                break;
        }
    }
    return content;
}

std::string errorMessageOf(const json& body) {
    if (!body.is_object()) return {};
    const auto it = body.find("error");
    if (it == body.end() || !it->is_object()) return {};
    std::string message = stringAt(*it, "message");
    const std::string type = stringAt(*it, "type");
    if (!type.empty()) message = message.empty() ? type : message + " (" + type + ")";
    return message;
}

class AnthropicAdapter final : public ProviderAdapter {
public:
    HttpRequest buildRequest(const ChatRequest& chat, const ProviderConfig& config) const override {
        HttpRequest request;
        request.url = joinUrl(config.baseUrl, "/v1/messages");
        request.headers = headers(config);
        const bool fallback = config.serverFallback && anthropicSupportsServerFallback(chat.model);
        if (fallback) request.headers.emplace_back("anthropic-beta", kFallbackBeta);

        json body = {{"model", chat.model},
                     {"max_tokens", chat.maxTokens > 0 ? chat.maxTokens : 16000}};
        if (!chat.system.empty()) body["system"] = chat.system;

        json messages = json::array();
        for (const auto& message : chat.messages) {
            json content = contentOf(message);
            if (content.empty()) continue;  // the API combines the turns either side
            messages.push_back({{"role", message.role == Role::User ? "user" : "assistant"},
                                {"content", std::move(content)}});
        }
        body["messages"] = std::move(messages);

        if (!chat.tools.empty()) {
            json tools = json::array();
            for (const auto& tool : chat.tools) {
                tools.push_back({{"name", tool.name},
                                 {"description", tool.description},
                                 {"input_schema", tool.inputSchema}});
            }
            body["tools"] = std::move(tools);
        }
        if (isEffort(config.effort)) body["output_config"] = {{"effort", config.effort}};
        if (fallback) body["fallbacks"] = "default";
        request.body = dump(body);
        return request;
    }

    ChatResponse parseResponse(const HttpResponse& http) const override {
        if (http.status == 0) return failed(httpErrorMessage("Anthropic", http, {}));
        const auto parsed = parseJson(http.body);
        if (!succeeded(http.status) || !parsed || !parsed->is_object() ||
            stringAt(*parsed, "type") == "error") {
            if (succeeded(http.status) && !parsed) {
                return failed("Anthropic sent a reply that is not JSON: " + snippet(http.body));
            }
            HttpResponse shown = http;
            if (succeeded(shown.status)) shown.status = 500;  // an error body with a 200
            return failed(
                httpErrorMessage("Anthropic", shown, parsed ? errorMessageOf(*parsed) : ""));
        }
        const json& body = *parsed;

        ChatResponse response;
        response.message.role = Role::Assistant;
        if (const auto usage = body.find("usage"); usage != body.end()) {
            response.usage.inputTokens = integerAt(*usage, "input_tokens");
            response.usage.outputTokens = integerAt(*usage, "output_tokens");
        }

        const std::string stop = stringAt(body, "stop_reason");
        if (stop == "refusal") {
            // Checked before the content is read: a classifier can decline
            // before any output (empty content) or part-way (partial output,
            // which is not the answer and is not kept).
            response.stop = StopReason::Refusal;
            if (const auto details = body.find("stop_details");
                details != body.end() && details->is_object()) {
                response.refusalDetail = stringAt(*details, "explanation");
                if (response.refusalDetail.empty()) {
                    response.refusalDetail = stringAt(*details, "category");
                }
            }
            return response;
        }

        if (const auto content = body.find("content");
            content != body.end() && content->is_array()) {
            // A server-side fallback marks where the declined attempt ended
            // with a `fallback` block. What the declined attempt made before
            // the last one is not the answer: its tool calls are not run, and
            // its thinking and tool calls are not sent back (the API checks).
            // Its text, the marker itself (where it stood) and everything
            // after are kept.
            std::size_t boundary = 0;
            for (std::size_t i = 0; i < content->size(); ++i) {
                if (stringAt((*content)[i], "type") == "fallback") boundary = i;
            }
            for (std::size_t i = 0; i < content->size(); ++i) {
                const json& block = (*content)[i];
                if (!block.is_object()) continue;
                const std::string type = stringAt(block, "type");
                if (i < boundary && type != "text") continue;
                if (type == "text") {
                    response.message.blocks.push_back(
                        ContentBlock::makeText(stringAt(block, "text")));
                } else if (type == "tool_use") {
                    const auto input = block.find("input");
                    response.message.blocks.push_back(
                        ContentBlock::makeToolCall(stringAt(block, "id"), stringAt(block, "name"),
                                                   input != block.end() ? *input : json::object()));
                } else {
                    // thinking, redacted_thinking, and anything newer: sent
                    // back as it came.
                    response.message.blocks.push_back(ContentBlock::makeOpaque(kProvider, block));
                }
            }
        }

        if (stop == "tool_use") {
            response.stop =
                response.message.toolCalls().empty() ? StopReason::EndTurn : StopReason::ToolUse;
        } else if (stop == "end_turn" || stop == "stop_sequence") {
            response.stop = StopReason::EndTurn;
        } else if (stop == "max_tokens") {
            response.stop = StopReason::MaxTokens;
        } else if (stop == "model_context_window_exceeded") {
            response.stop = StopReason::Error;
            response.error =
                "The conversation is longer than the model can read. Start a new conversation.";
        } else {
            response.stop =
                response.message.toolCalls().empty() ? StopReason::Other : StopReason::ToolUse;
        }
        return response;
    }

    HttpRequest buildListModels(const ProviderConfig& config) const override {
        HttpRequest request;
        request.method = "GET";
        request.url = joinUrl(config.baseUrl, "/v1/models?limit=1000");
        request.headers = headers(config);
        return request;
    }

    std::vector<std::string> parseListModels(const HttpResponse& http,
                                             std::string* error) const override {
        std::vector<std::string> models;
        const auto parsed = parseJson(http.body);
        if (!succeeded(http.status) || !parsed) {
            if (error) {
                *error = httpErrorMessage("Anthropic", http, parsed ? errorMessageOf(*parsed) : "");
            }
            return models;
        }
        if (const auto data = parsed->find("data"); data != parsed->end() && data->is_array()) {
            for (const auto& model : *data) {
                std::string id = stringAt(model, "id");
                if (!id.empty()) models.push_back(std::move(id));
            }
        }
        if (models.empty() && error) *error = "Anthropic listed no models.";
        return models;
    }

private:
    static std::vector<std::pair<std::string, std::string>> headers(const ProviderConfig& config) {
        std::vector<std::pair<std::string, std::string>> out = {
            {"content-type", "application/json"}, {"anthropic-version", kApiVersion}};
        if (!config.apiKey.empty()) out.emplace_back("x-api-key", config.apiKey);
        return out;
    }
};

}  // namespace

std::unique_ptr<ProviderAdapter> makeAnthropicAdapter() {
    return std::make_unique<AnthropicAdapter>();
}

}  // namespace hz::ai
