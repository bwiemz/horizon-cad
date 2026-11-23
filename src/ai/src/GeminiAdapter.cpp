// Google's Gemini API (POST {base}/v1beta/models/{model}:generateContent).
//
// A model turn's parts are sent back exactly as they came: the current
// models attach a thought signature to their parts and refuse a function
// call's follow-up without it. So a text or a function call read from a part
// keeps the part, and the part is what goes back. A thought part is kept
// as an opaque block. Function calls may come without an id; one is made up
// here, and left out of what is sent back.

#include <algorithm>
#include <atomic>
#include <cctype>
#include <memory>
#include <string>
#include <utility>

#include "AdapterUtil.h"
#include "horizon/ai/Provider.h"

namespace hz::ai {

namespace {

using nlohmann::json;
using namespace detail;

constexpr const char* kProvider = "gemini";
constexpr const char* kMadeUpId = "hzcall_";

std::atomic<unsigned long long> g_nextCallId{1};

std::string modelName(std::string model) {
    if (model.rfind("models/", 0) == 0) model.erase(0, 7);
    return model;
}

/// A JSON Schema as Gemini's `parameters` takes it: the same, with each type
/// in capitals (its Schema's enum).
json geminiSchema(const json& schema) {
    if (schema.is_array()) {
        json out = json::array();
        for (const auto& item : schema) out.push_back(geminiSchema(item));
        return out;
    }
    if (!schema.is_object()) return schema;
    json out = json::object();
    for (auto it = schema.begin(); it != schema.end(); ++it) {
        if (it.key() == "type" && it.value().is_string()) {
            std::string type = it.value().get<std::string>();
            std::transform(type.begin(), type.end(), type.begin(),
                           [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
            out["type"] = type;
        } else if (it.key() == "properties" && it.value().is_object()) {
            json properties = json::object();
            for (auto p = it.value().begin(); p != it.value().end(); ++p) {
                properties[p.key()] = geminiSchema(p.value());
            }
            out["properties"] = std::move(properties);
        } else if (it.key() == "items") {
            out["items"] = geminiSchema(it.value());
        } else {
            out[it.key()] = it.value();
        }
    }
    return out;
}

std::string errorMessageOf(const json& body) {
    if (!body.is_object()) return {};
    const auto it = body.find("error");
    if (it == body.end() || !it->is_object()) return {};
    std::string message = stringAt(*it, "message");
    const std::string status = stringAt(*it, "status");
    if (!status.empty()) message = message.empty() ? status : message + " (" + status + ")";
    return message;
}

bool isRefusal(const std::string& reason) {
    return reason == "SAFETY" || reason == "RECITATION" || reason == "BLOCKLIST" ||
           reason == "PROHIBITED_CONTENT" || reason == "SPII" || reason == "IMAGE_SAFETY" ||
           reason == "LANGUAGE";
}

/// A whole reply (generated content), or the error it is.
ChatResponse parseContent(const HttpResponse& http) {
    if (http.status == 0) return failed(httpErrorMessage("Gemini", http, {}));
    const auto parsed = parseJson(http.body);
    // Bound once, the optional tested before every use (CI's clang-tidy
    // checks it).
    if (!parsed || !succeeded(http.status) || !parsed->is_object() || parsed->contains("error")) {
        if (!parsed && succeeded(http.status)) {
            return failed("Gemini sent a reply that is not JSON: " + snippet(http.body));
        }
        HttpResponse shown = http;
        if (succeeded(shown.status)) shown.status = 500;
        return failed(httpErrorMessage("Gemini", shown, parsed ? errorMessageOf(*parsed) : ""));
    }
    const json& body = *parsed;

    ChatResponse response;
    response.message.role = Role::Assistant;
    if (const auto usage = body.find("usageMetadata"); usage != body.end()) {
        response.usage.inputTokens = integerAt(*usage, "promptTokenCount");
        response.usage.outputTokens =
            integerAt(*usage, "candidatesTokenCount") + integerAt(*usage, "thoughtsTokenCount");
    }
    if (const auto feedback = body.find("promptFeedback"); feedback != body.end()) {
        if (const std::string blocked = stringAt(*feedback, "blockReason"); !blocked.empty()) {
            response.stop = StopReason::Refusal;
            response.refusalDetail = "the request was blocked (" + blocked + ")";
            return response;
        }
    }
    const auto candidates = body.find("candidates");
    if (candidates == body.end() || !candidates->is_array() || candidates->empty() ||
        !(*candidates)[0].is_object()) {
        return failed("Gemini sent a reply with no candidates: " + snippet(http.body));
    }
    const json& candidate = (*candidates)[0];
    const std::string finish = stringAt(candidate, "finishReason");
    if (isRefusal(finish)) {
        response.stop = StopReason::Refusal;
        response.refusalDetail = "Gemini stopped the reply (" + finish + ")";
        return response;
    }

    if (const auto content = candidate.find("content"); content != candidate.end()) {
        if (const auto parts = content->find("parts");
            parts != content->end() && parts->is_array()) {
            for (const auto& part : *parts) {
                if (!part.is_object()) continue;
                if ((part.contains("thought") && part["thought"].is_boolean() &&
                     part["thought"].get<bool>()) ||
                    (!part.contains("text") && !part.contains("functionCall"))) {
                    response.message.blocks.push_back(ContentBlock::makeOpaque(kProvider, part));
                    continue;
                }
                ContentBlock block;
                if (const auto call = part.find("functionCall");
                    call != part.end() && call->is_object()) {
                    std::string id = stringAt(*call, "id");
                    if (id.empty()) id = kMadeUpId + std::to_string(g_nextCallId++);
                    const auto args = call->find("args");
                    block = ContentBlock::makeToolCall(
                        std::move(id), stringAt(*call, "name"),
                        args != call->end() && args->is_object() ? *args : json::object());
                } else {
                    block = ContentBlock::makeText(stringAt(part, "text"));
                }
                block.provider = kProvider;
                block.raw = part;
                response.message.blocks.push_back(std::move(block));
            }
        }
    }

    const bool calls = !response.message.toolCalls().empty();
    if (finish == "MAX_TOKENS") {
        response.stop = StopReason::MaxTokens;
    } else if (finish == "MALFORMED_FUNCTION_CALL" || finish == "UNEXPECTED_TOOL_CALL") {
        response.stop = StopReason::Error;
        response.error = "Gemini produced a tool call it could not complete (" + finish +
                         "). Try again, or rephrase the request.";
    } else {
        response.stop = calls ? StopReason::ToolUse : StopReason::EndTurn;
    }
    return response;
}

/// Whether @p part is a piece of text (with, perhaps, a thought signature):
/// one that joins the text before it. A function call is never one.
bool isTextPiece(const json& part) {
    if (!part.is_object() || !part.contains("text") || !part["text"].is_string()) return false;
    for (auto it = part.begin(); it != part.end(); ++it) {
        if (it.key() != "text" && it.key() != "thought" && it.key() != "thoughtSignature") {
            return false;
        }
    }
    return true;
}

bool isThought(const json& part) {
    return part.contains("thought") && part["thought"].is_boolean() && part["thought"].get<bool>();
}

/// A streamed reply: each event is a whole GenerateContentResponse with the
/// parts that are new. Put back together as one, and read by parseContent().
class ContentStreamReader final : public SseStreamReader {
protected:
    std::string onEvent(const SseEvent& event) override {
        const auto parsed = parseJson(event.data);
        if (!parsed || !parsed->is_object()) return {};
        const json& data = *parsed;
        if (const auto error = data.find("error"); error != data.end() && !error->is_null()) {
            m_error = data;
            return {};
        }
        if (const auto usage = data.find("usageMetadata");
            usage != data.end() && usage->is_object()) {
            m_usage = *usage;
        }
        if (const auto feedback = data.find("promptFeedback");
            feedback != data.end() && feedback->is_object()) {
            m_feedback = *feedback;
            if (!stringAt(*feedback, "blockReason").empty()) m_ended = true;
        }
        const auto candidates = data.find("candidates");
        if (candidates == data.end() || !candidates->is_array() || candidates->empty() ||
            !(*candidates)[0].is_object()) {
            return {};
        }
        const json& candidate = (*candidates)[0];
        if (const std::string finish = stringAt(candidate, "finishReason"); !finish.empty()) {
            m_finish = finish;
            m_ended = true;  // given on the last event only
        }
        const auto content = candidate.find("content");
        if (content == candidate.end() || !content->is_object()) return {};
        const auto parts = content->find("parts");
        if (parts == content->end() || !parts->is_array()) return {};
        std::string text;
        for (const auto& part : *parts) {
            if (!part.is_object()) continue;
            if (!isThought(part)) text += stringAt(part, "text");
            // Text comes in many small parts; one part for it, as a reply
            // sent whole has (a message's text blocks are joined by a blank
            // line). A signature on any piece goes on the whole, where a reply
            // sent whole carries it.
            if (isTextPiece(part) && !m_parts.empty() && isTextPiece(m_parts.back()) &&
                isThought(part) == isThought(m_parts.back())) {
                json& whole = m_parts.back();
                appendAt(whole, "text", stringAt(part, "text"));
                if (const auto signature = part.find("thoughtSignature"); signature != part.end()) {
                    whole["thoughtSignature"] = *signature;
                }
            } else {
                m_parts.push_back(part);
            }
        }
        return text;
    }

    ChatResponse finishStream(const HttpResponse& http) override {
        if (!m_error.is_null()) {
            long long status = 500;
            if (m_error["error"].is_object()) status = integerAt(m_error["error"], "code", 500);
            return streamError(m_error,
                               status >= 400 && status < 600 ? static_cast<int>(status) : 500);
        }
        if (!m_ended) return cutOff(http);
        json candidate = {{"content", {{"role", "model"}, {"parts", m_parts}}}};
        if (!m_finish.empty()) candidate["finishReason"] = m_finish;
        json body = {{"candidates", json::array({std::move(candidate)})}};
        if (!m_usage.is_null()) body["usageMetadata"] = m_usage;
        if (!m_feedback.is_null()) body["promptFeedback"] = m_feedback;
        HttpResponse whole;
        whole.status = 200;
        whole.body = dump(body);
        return parseContent(whole);
    }

    ChatResponse parseWhole(const HttpResponse& http) const override { return parseContent(http); }
    std::string provider() const override { return "Gemini"; }

private:
    json m_parts = json::array();
    std::string m_finish;
    json m_usage;
    json m_feedback;
    json m_error;  ///< null unless the stream reported one
    bool m_ended = false;
};

class GeminiAdapter final : public ProviderAdapter {
public:
    HttpRequest buildRequest(const ChatRequest& chat, const ProviderConfig& config) const override {
        return build(chat, config, false);
    }

    ChatResponse parseResponse(const HttpResponse& http) const override {
        return parseContent(http);
    }

    HttpRequest buildStreamingRequest(const ChatRequest& chat,
                                      const ProviderConfig& config) const override {
        return build(chat, config, true);
    }

    std::unique_ptr<StreamReader> makeStreamReader() const override {
        return std::make_unique<ContentStreamReader>();
    }

    HttpRequest buildListModels(const ProviderConfig& config) const override {
        HttpRequest request;
        request.method = "GET";
        request.url = joinUrl(config.baseUrl, "/v1beta/models?pageSize=1000");
        request.headers = headers(config);
        return request;
    }

    std::vector<std::string> parseListModels(const HttpResponse& http,
                                             std::string* error) const override {
        std::vector<std::string> models;
        const auto parsed = parseJson(http.body);
        if (!succeeded(http.status) || !parsed) {
            if (error)
                *error = httpErrorMessage("Gemini", http, parsed ? errorMessageOf(*parsed) : "");
            return models;
        }
        if (const auto list = parsed->find("models"); list != parsed->end() && list->is_array()) {
            for (const auto& model : *list) {
                const auto methods = model.find("supportedGenerationMethods");
                const bool generates =
                    methods == model.end() ||
                    (methods->is_array() && std::find(methods->begin(), methods->end(),
                                                      json("generateContent")) != methods->end());
                std::string name = modelName(stringAt(model, "name"));
                if (generates && !name.empty()) models.push_back(std::move(name));
            }
        }
        if (models.empty() && error) *error = "Gemini listed no models.";
        return models;
    }

private:
    HttpRequest build(const ChatRequest& chat, const ProviderConfig& config, bool stream) const {
        HttpRequest request;
        request.url = joinUrl(config.baseUrl,
                              "/v1beta/models/" + modelName(chat.model) +
                                  (stream ? ":streamGenerateContent?alt=sse" : ":generateContent"));
        request.headers = headers(config);

        json contents = json::array();
        for (const auto& message : chat.messages) {
            json parts = json::array();
            for (const auto& block : message.blocks) {
                const bool own = block.provider == kProvider && block.raw.is_object();
                switch (block.kind) {
                    case ContentBlock::Kind::Text:
                        if (own) {
                            parts.push_back(block.raw);
                        } else if (!block.text.empty()) {
                            parts.push_back({{"text", block.text}});
                        }
                        break;
                    case ContentBlock::Kind::ToolCall:
                        if (own) {
                            parts.push_back(block.raw);
                        } else {
                            json call = {
                                {"name", block.name},
                                {"args", block.input.is_object() ? block.input : json::object()}};
                            if (!block.id.empty() && block.id.rfind(kMadeUpId, 0) != 0) {
                                call["id"] = block.id;
                            }
                            parts.push_back({{"functionCall", std::move(call)}});
                        }
                        break;
                    case ContentBlock::Kind::ToolResult: {
                        json response = block.isError ? json{{"error", block.text}}
                                                      : json{{"result", block.text}};
                        json reply = {{"name", block.name}, {"response", std::move(response)}};
                        if (!block.id.empty() && block.id.rfind(kMadeUpId, 0) != 0) {
                            reply["id"] = block.id;
                        }
                        parts.push_back({{"functionResponse", std::move(reply)}});
                        // Its images beside it, in the same turn.
                        for (const auto& image : block.images) {
                            parts.push_back(
                                {{"inlineData",
                                  {{"mimeType", image.mediaType}, {"data", image.base64}}}});
                        }
                        break;
                    }
                    case ContentBlock::Kind::Opaque:
                        if (own) parts.push_back(block.raw);
                        break;
                }
            }
            if (parts.empty()) continue;
            contents.push_back({{"role", message.role == Role::User ? "user" : "model"},
                                {"parts", std::move(parts)}});
        }

        json body = {{"contents", std::move(contents)},
                     {"generationConfig",
                      {{"maxOutputTokens", chat.maxTokens > 0 ? chat.maxTokens : 16000}}}};
        if (!chat.system.empty())
            body["systemInstruction"] = {{"parts", {{{"text", chat.system}}}}};
        if (!chat.tools.empty()) {
            json declarations = json::array();
            for (const auto& tool : chat.tools) {
                declarations.push_back({{"name", tool.name},
                                        {"description", tool.description},
                                        {"parameters", geminiSchema(tool.inputSchema)}});
            }
            body["tools"] = json::array({{{"functionDeclarations", std::move(declarations)}}});
        }
        request.body = dump(body);
        return request;
    }

    static std::vector<std::pair<std::string, std::string>> headers(const ProviderConfig& config) {
        std::vector<std::pair<std::string, std::string>> out = {
            {"content-type", "application/json"}};
        if (!config.apiKey.empty()) out.emplace_back("x-goog-api-key", config.apiKey);
        return out;
    }
};

}  // namespace

std::unique_ptr<ProviderAdapter> makeGeminiAdapter() {
    return std::make_unique<GeminiAdapter>();
}

}  // namespace hz::ai
