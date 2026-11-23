// OpenAI's chat completions (POST {base}/chat/completions), which OpenAI
// serves and so do the local and hosted servers that copy it: Ollama, LM
// Studio, llama.cpp, vLLM, LocalAI, Jan, OpenRouter, Groq, Mistral, DeepSeek,
// xAI. The copies differ in small ways, read tolerantly here: a tool call
// without an id, arguments as an object rather than a string, a finish
// reason of "stop" beside tool calls, an error given as a bare string.

#include <algorithm>
#include <memory>
#include <string>
#include <utility>

#include "AdapterUtil.h"
#include "horizon/ai/Provider.h"

namespace hz::ai {

namespace {

using nlohmann::json;
using namespace detail;

std::string errorMessageOf(const json& body) {
    if (!body.is_object()) return {};
    const auto it = body.find("error");
    if (it == body.end()) return {};
    if (it->is_string()) return it->get<std::string>();
    if (!it->is_object()) return {};
    std::string message = stringAt(*it, "message");
    const std::string code = stringAt(*it, "code");
    if (!code.empty() && message.find(code) == std::string::npos) {
        message += message.empty() ? code : " (" + code + ")";
    }
    return message;
}

/// A message's content as text, whether a string or an array of parts.
std::string textOf(const json& content) {
    if (content.is_string()) return content.get<std::string>();
    std::string text;
    if (content.is_array()) {
        for (const auto& part : content) {
            if (stringAt(part, "type") == "text") text += stringAt(part, "text");
        }
    }
    return text;
}

class OpenAIAdapter final : public ProviderAdapter {
public:
    explicit OpenAIAdapter(bool official) : m_official(official) {}

    HttpRequest buildRequest(const ChatRequest& chat, const ProviderConfig& config) const override {
        HttpRequest request;
        request.url = joinUrl(config.baseUrl, "/chat/completions");
        request.headers = headers(config);

        json messages = json::array();
        if (!chat.system.empty())
            messages.push_back({{"role", "system"}, {"content", chat.system}});
        for (const auto& message : chat.messages) {
            if (message.role == Role::User) {
                // Results first: each must follow the assistant message that
                // asked for it.
                for (const auto& block : message.blocks) {
                    if (block.kind != ContentBlock::Kind::ToolResult) continue;
                    std::string content = block.text.empty() ? "(no output)" : block.text;
                    if (block.isError) content = "Error: " + content;
                    messages.push_back(
                        {{"role", "tool"}, {"tool_call_id", block.id}, {"content", content}});
                }
                const std::string text = message.text();
                if (!text.empty()) messages.push_back({{"role", "user"}, {"content", text}});
                continue;
            }
            json calls = json::array();
            for (const auto* call : message.toolCalls()) {
                calls.push_back(
                    {{"id", call->id},
                     {"type", "function"},
                     {"function",
                      {{"name", call->name},
                       {"arguments",
                        dump(call->input.is_object() ? call->input : json::object())}}}});
            }
            const std::string text = message.text();
            if (text.empty() && calls.empty()) continue;
            json assistant = {{"role", "assistant"}};
            assistant["content"] = text.empty() ? json(nullptr) : json(text);
            if (!calls.empty()) assistant["tool_calls"] = std::move(calls);
            messages.push_back(std::move(assistant));
        }

        json body = {{"model", chat.model}, {"messages", std::move(messages)}};
        const int maxTokens = chat.maxTokens > 0 ? chat.maxTokens : 16000;
        // OpenAI's own models take the newer name (its reasoning models refuse
        // the older); most servers that copy the API know only the older.
        body[m_official ? "max_completion_tokens" : "max_tokens"] = maxTokens;
        if (!chat.tools.empty()) {
            json tools = json::array();
            for (const auto& tool : chat.tools) {
                tools.push_back({{"type", "function"},
                                 {"function",
                                  {{"name", tool.name},
                                   {"description", tool.description},
                                   {"parameters", tool.inputSchema}}}});
            }
            body["tools"] = std::move(tools);
        }
        request.body = dump(body);
        return request;
    }

    ChatResponse parseResponse(const HttpResponse& http) const override {
        const char* who = m_official ? "OpenAI" : "The model server";
        if (http.status == 0) return failed(httpErrorMessage(who, http, {}));
        const auto parsed = parseJson(http.body);
        // Bound once, the optional tested before every use (CI's clang-tidy
        // checks it).
        if (!parsed || !succeeded(http.status) || !parsed->is_object() ||
            parsed->contains("error")) {
            if (!parsed && succeeded(http.status)) {
                return failed(std::string(who) +
                              " sent a reply that is not JSON: " + snippet(http.body));
            }
            HttpResponse shown = http;
            if (succeeded(shown.status)) shown.status = 500;
            return failed(httpErrorMessage(who, shown, parsed ? errorMessageOf(*parsed) : ""));
        }
        const json& body = *parsed;

        ChatResponse response;
        response.message.role = Role::Assistant;
        if (const auto usage = body.find("usage"); usage != body.end()) {
            response.usage.inputTokens = integerAt(*usage, "prompt_tokens");
            response.usage.outputTokens = integerAt(*usage, "completion_tokens");
        }
        const auto choices = body.find("choices");
        if (choices == body.end() || !choices->is_array() || choices->empty() ||
            !(*choices)[0].is_object()) {
            return failed(std::string(who) +
                          " sent a reply with no choices: " + snippet(http.body));
        }
        const json& choice = (*choices)[0];
        const json message = choice.contains("message") && choice["message"].is_object()
                                 ? choice["message"]
                                 : json::object();

        if (const std::string refusal = stringAt(message, "refusal"); !refusal.empty()) {
            response.stop = StopReason::Refusal;
            response.refusalDetail = refusal;
            return response;
        }
        if (const auto content = message.find("content"); content != message.end()) {
            std::string text = textOf(*content);
            if (!text.empty())
                response.message.blocks.push_back(ContentBlock::makeText(std::move(text)));
        }
        if (const auto calls = message.find("tool_calls");
            calls != message.end() && calls->is_array()) {
            int index = 0;
            for (const auto& call : *calls) {
                ++index;
                if (!call.is_object()) continue;
                const json function = call.contains("function") && call["function"].is_object()
                                          ? call["function"]
                                          : json::object();
                std::string id = stringAt(call, "id");
                if (id.empty()) id = "call_" + std::to_string(index);
                json input = json::object();
                if (const auto args = function.find("arguments"); args != function.end()) {
                    if (args->is_object()) {
                        input = *args;
                    } else if (args->is_string()) {
                        const std::string raw = args->get<std::string>();
                        if (!raw.empty()) {
                            const auto value = parseJson(raw);
                            input = value && value->is_object() ? *value
                                                                : json{{"_invalid_arguments", raw}};
                        }
                    }
                }
                response.message.blocks.push_back(ContentBlock::makeToolCall(
                    std::move(id), stringAt(function, "name"), std::move(input)));
            }
        }

        const std::string finish = stringAt(choice, "finish_reason");
        const bool calls = !response.message.toolCalls().empty();
        if (finish == "length") {
            response.stop = StopReason::MaxTokens;
        } else if (finish == "content_filter") {
            response.stop = StopReason::Refusal;
            response.refusalDetail = "the provider's content filter stopped the reply";
        } else {
            // "tool_calls", "function_call", and servers that say "stop" with
            // calls in the message.
            response.stop = calls ? StopReason::ToolUse : StopReason::EndTurn;
        }
        return response;
    }

    HttpRequest buildListModels(const ProviderConfig& config) const override {
        HttpRequest request;
        request.method = "GET";
        request.url = joinUrl(config.baseUrl, "/models");
        request.headers = headers(config);
        return request;
    }

    std::vector<std::string> parseListModels(const HttpResponse& http,
                                             std::string* error) const override {
        const char* who = m_official ? "OpenAI" : "The model server";
        std::vector<std::string> models;
        const auto parsed = parseJson(http.body);
        if (!succeeded(http.status) || !parsed) {
            if (error) *error = httpErrorMessage(who, http, parsed ? errorMessageOf(*parsed) : "");
            return models;
        }
        if (const auto data = parsed->find("data"); data != parsed->end() && data->is_array()) {
            for (const auto& model : *data) {
                std::string id = stringAt(model, "id");
                if (!id.empty()) models.push_back(std::move(id));
            }
        }
        std::sort(models.begin(), models.end());
        if (models.empty() && error) *error = std::string(who) + " listed no models.";
        return models;
    }

private:
    static std::vector<std::pair<std::string, std::string>> headers(const ProviderConfig& config) {
        std::vector<std::pair<std::string, std::string>> out = {
            {"content-type", "application/json"}};
        if (!config.apiKey.empty()) out.emplace_back("authorization", "Bearer " + config.apiKey);
        return out;
    }

    bool m_official;
};

}  // namespace

std::unique_ptr<ProviderAdapter> makeOpenAIAdapter(bool official) {
    return std::make_unique<OpenAIAdapter>(official);
}

}  // namespace hz::ai
