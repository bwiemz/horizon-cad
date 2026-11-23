// OpenAI's chat completions (POST {base}/chat/completions), which OpenAI
// serves and so do the local and hosted servers that copy it: Ollama, LM
// Studio, llama.cpp, vLLM, LocalAI, Jan, OpenRouter, Groq, Mistral, DeepSeek,
// xAI. The copies differ in small ways, read tolerantly here: a tool call
// without an id, arguments as an object rather than a string, a finish
// reason of "stop" beside tool calls, an error given as a bare string.

#include <algorithm>
#include <cstddef>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

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

/// A whole reply (a chat completion), or the error it is.
ChatResponse parseCompletion(const HttpResponse& http, bool official) {
    const char* who = official ? "OpenAI" : "The model server";
    if (http.status == 0) return failed(httpErrorMessage(who, http, {}));
    const auto parsed = parseJson(http.body);
    // Bound once, the optional tested before every use (CI's clang-tidy
    // checks it).
    if (!parsed || !succeeded(http.status) || !parsed->is_object() || parsed->contains("error")) {
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
        return failed(std::string(who) + " sent a reply with no choices: " + snippet(http.body));
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

/// A streamed chat completion, put back together as the completion it would
/// have been sent whole, and read by parseCompletion().
class CompletionStreamReader final : public SseStreamReader {
public:
    explicit CompletionStreamReader(bool official) : m_official(official) {}

protected:
    std::string onEvent(const SseEvent& event) override {
        if (event.data == "[DONE]") {
            m_ended = true;
            return {};
        }
        const auto parsed = parseJson(event.data);
        if (!parsed || !parsed->is_object()) return {};
        const json& data = *parsed;
        if (const auto error = data.find("error");
            error != data.end() && (error->is_object() || error->is_string())) {
            m_error = data;
            return {};
        }
        if (const auto usage = data.find("usage"); usage != data.end() && usage->is_object()) {
            m_usage = *usage;
        }
        const auto choices = data.find("choices");
        if (choices == data.end() || !choices->is_array() || choices->empty() ||
            !(*choices)[0].is_object()) {
            return {};
        }
        const json& choice = (*choices)[0];
        if (const std::string finish = stringAt(choice, "finish_reason"); !finish.empty()) {
            m_finish = finish;
        }
        const auto delta = choice.find("delta");
        if (delta == choice.end() || !delta->is_object()) return {};
        std::string text;
        if (const auto content = delta->find("content"); content != delta->end()) {
            text = textOf(*content);
            m_text += text;
        }
        m_refusal += stringAt(*delta, "refusal");
        if (const auto calls = delta->find("tool_calls");
            calls != delta->end() && calls->is_array()) {
            for (const auto& call : *calls) {
                if (call.is_object()) addCall(call);
            }
        }
        return text;
    }

    ChatResponse finishStream(const HttpResponse& http) override {
        if (!m_error.is_null()) return streamError(m_error, 500);
        if (!m_ended && m_finish.empty()) return cutOff(http);
        json message = {{"role", "assistant"}};
        message["content"] = m_text.empty() ? json(nullptr) : json(m_text);
        if (!m_refusal.empty()) message["refusal"] = m_refusal;
        if (!m_calls.empty()) {
            json calls = json::array();
            for (const auto& call : m_calls) {
                calls.push_back({{"id", call.id},
                                 {"type", "function"},
                                 {"function",
                                  {{"name", call.name},
                                   {"arguments",
                                    call.whole.is_object() ? call.whole : json(call.arguments)}}}});
            }
            message["tool_calls"] = std::move(calls);
        }
        json body = {{"choices", json::array({{{"index", 0},
                                               {"message", std::move(message)},
                                               {"finish_reason", m_finish}}})}};
        if (!m_usage.is_null()) body["usage"] = m_usage;
        HttpResponse whole;
        whole.status = 200;
        whole.body = dump(body);
        return parseCompletion(whole, m_official);
    }

    ChatResponse parseWhole(const HttpResponse& http) const override {
        return parseCompletion(http, m_official);
    }
    std::string provider() const override { return m_official ? "OpenAI" : "The model server"; }

private:
    struct Call {
        std::string id;
        std::string name;
        std::string arguments;  ///< the JSON text, as it came
        json whole;             ///< arguments sent as an object, by a server that does
    };

    /// A piece of a tool call: by its index, its id and name come first and
    /// its arguments in pieces after.
    void addCall(const json& piece) {
        const long long index = integerAt(piece, "index", -1);
        const std::string id = stringAt(piece, "id");
        std::size_t at = m_calls.size();
        if (index >= 0) {
            const auto slot = m_slot.find(index);
            // A new id at an index in use is another call: a server that
            // numbers every call 0.
            if (slot != m_slot.end() && (id.empty() || m_calls[slot->second].id.empty() ||
                                         m_calls[slot->second].id == id)) {
                at = slot->second;
            }
        }
        if (at == m_calls.size()) {
            m_calls.emplace_back();
            if (index >= 0) m_slot[index] = at;
        }
        Call& call = m_calls[at];
        if (call.id.empty()) call.id = id;
        const auto function = piece.find("function");
        if (function == piece.end() || !function->is_object()) return;
        // A name comes whole, once; a server that sends it again is not
        // naming another.
        if (const std::string name = stringAt(*function, "name");
            !name.empty() && name != call.name) {
            call.name += name;
        }
        if (const auto args = function->find("arguments"); args != function->end()) {
            if (args->is_string()) {
                call.arguments += args->get<std::string>();
            } else if (args->is_object()) {
                call.whole = *args;
            }
        }
    }

    bool m_official;
    std::string m_text;
    std::string m_refusal;
    std::string m_finish;
    json m_usage;
    json m_error;  ///< null unless the stream reported one
    bool m_ended = false;
    std::vector<Call> m_calls;
    std::map<long long, std::size_t> m_slot;  ///< a call's index -> where it is
};

class OpenAIAdapter final : public ProviderAdapter {
public:
    explicit OpenAIAdapter(bool official) : m_official(official) {}

    HttpRequest buildRequest(const ChatRequest& chat, const ProviderConfig& config) const override {
        return build(chat, config, false);
    }

    ChatResponse parseResponse(const HttpResponse& http) const override {
        return parseCompletion(http, m_official);
    }

    HttpRequest buildStreamingRequest(const ChatRequest& chat,
                                      const ProviderConfig& config) const override {
        return build(chat, config, true);
    }

    std::unique_ptr<StreamReader> makeStreamReader() const override {
        return std::make_unique<CompletionStreamReader>(m_official);
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
    HttpRequest build(const ChatRequest& chat, const ProviderConfig& config, bool stream) const {
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
                json shown = json::array();  // their images, which a tool message cannot hold
                for (const auto& block : message.blocks) {
                    if (block.kind != ContentBlock::Kind::ToolResult) continue;
                    std::string content = block.text.empty() ? "(no output)" : block.text;
                    if (block.isError) content = "Error: " + content;
                    messages.push_back(
                        {{"role", "tool"}, {"tool_call_id", block.id}, {"content", content}});
                    for (const auto& image : block.images) {
                        shown.push_back({{"type", "text"},
                                         {"text", "The image " + block.name + " (" + block.id +
                                                      ") returned:"}});
                        shown.push_back(
                            {{"type", "image_url"},
                             {"image_url",
                              {{"url", "data:" + image.mediaType + ";base64," + image.base64}}}});
                    }
                }
                const std::string text = message.text();
                if (!shown.empty()) {
                    // After the results, in a user message: the only place the
                    // chat API takes an image.
                    if (!text.empty()) shown.push_back({{"type", "text"}, {"text", text}});
                    messages.push_back({{"role", "user"}, {"content", std::move(shown)}});
                } else if (!text.empty()) {
                    messages.push_back({{"role", "user"}, {"content", text}});
                }
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
        if (stream) {
            body["stream"] = true;
            // The tokens used come in a last chunk of their own, when asked
            // for; a server that only copies the API may refuse the option.
            if (m_official) body["stream_options"] = {{"include_usage", true}};
        }
        request.body = dump(body);
        return request;
    }

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
