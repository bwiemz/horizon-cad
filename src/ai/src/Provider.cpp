#include "horizon/ai/Provider.h"

#include <algorithm>
#include <utility>

#include "AdapterUtil.h"

namespace hz::ai {

std::unique_ptr<ProviderAdapter> makeAnthropicAdapter();
std::unique_ptr<ProviderAdapter> makeOpenAIAdapter(bool official);
std::unique_ptr<ProviderAdapter> makeGeminiAdapter();

ProviderAdapter::~ProviderAdapter() = default;

ProviderConfig defaultConfig(ProviderKind kind) {
    ProviderConfig config;
    config.kind = kind;
    switch (kind) {
        case ProviderKind::Anthropic:
            config.baseUrl = "https://api.anthropic.com";
            config.model = "claude-opus-5-5";
            break;
        case ProviderKind::OpenAI:
            config.baseUrl = "https://api.openai.com/v1";
            config.model = "gpt-5.5";
            break;
        case ProviderKind::Gemini:
            config.baseUrl = "https://generativelanguage.googleapis.com";
            config.model = "gemini-4-argon";
            break;
        case ProviderKind::OpenAICompatible:
            // Ollama's default port; LM Studio is http://localhost:1234/v1.
            // No model: the user picks one of those the server has.
            config.baseUrl = "http://localhost:11434/v1";
            config.maxTokens = 8192;
            config.images = false;  // many local models read no images
            break;
    }
    return config;
}

const char* providerName(ProviderKind kind) {
    switch (kind) {
        case ProviderKind::Anthropic:
            return "anthropic";
        case ProviderKind::OpenAI:
            return "openai";
        case ProviderKind::Gemini:
            return "gemini";
        case ProviderKind::OpenAICompatible:
            return "openai-compatible";
    }
    return "anthropic";
}

std::optional<ProviderKind> providerFromName(std::string_view name) {
    for (const ProviderKind kind : kAllProviders) {
        if (name == providerName(kind)) return kind;
    }
    if (name == "claude") return ProviderKind::Anthropic;
    if (name == "gpt") return ProviderKind::OpenAI;
    if (name == "google") return ProviderKind::Gemini;
    if (name == "local" || name == "ollama" || name == "lmstudio") {
        return ProviderKind::OpenAICompatible;
    }
    return std::nullopt;
}

const char* providerTitle(ProviderKind kind) {
    switch (kind) {
        case ProviderKind::Anthropic:
            return "Anthropic Claude";
        case ProviderKind::OpenAI:
            return "OpenAI GPT";
        case ProviderKind::Gemini:
            return "Google Gemini";
        case ProviderKind::OpenAICompatible:
            return "Local or OpenAI-compatible";
    }
    return "";
}

std::vector<std::string> apiKeyEnvironment(ProviderKind kind) {
    switch (kind) {
        case ProviderKind::Anthropic:
            return {"ANTHROPIC_API_KEY"};
        case ProviderKind::OpenAI:
            return {"OPENAI_API_KEY"};
        case ProviderKind::Gemini:
            return {"GEMINI_API_KEY", "GOOGLE_API_KEY"};
        case ProviderKind::OpenAICompatible:
            return {};
    }
    return {};
}

std::unique_ptr<ProviderAdapter> makeAdapter(ProviderKind kind) {
    switch (kind) {
        case ProviderKind::Anthropic:
            return makeAnthropicAdapter();
        case ProviderKind::OpenAI:
            return makeOpenAIAdapter(true);
        case ProviderKind::Gemini:
            return makeGeminiAdapter();
        case ProviderKind::OpenAICompatible:
            return makeOpenAIAdapter(false);
    }
    return makeAnthropicAdapter();
}

bool anthropicSupportsServerFallback(std::string_view model) {
    static constexpr std::string_view kModels[] = {"claude-fable-5-2", "claude-fable-5-1",
                                                   "claude-opus-5-5", "claude-opus-5",
                                                   "claude-sonnet-5-5"};
    return std::find(std::begin(kModels), std::end(kModels), model) != std::end(kModels);
}

// --- detail ------------------------------------------------------------------

namespace detail {

std::string joinUrl(std::string_view base, std::string_view path) {
    while (!base.empty() && base.back() == '/') base.remove_suffix(1);
    std::string url(base);
    if (!path.empty() && path.front() != '/') url += '/';
    url += path;
    return url;
}

std::optional<nlohmann::json> parseJson(const std::string& body) {
    nlohmann::json value = nlohmann::json::parse(body, nullptr, false);
    if (value.is_discarded()) return std::nullopt;
    // emplace: a json converts to many things, an optional among them.
    std::optional<nlohmann::json> out;
    out.emplace(std::move(value));
    return out;
}

std::string stringAt(const nlohmann::json& object, const char* key) {
    if (!object.is_object()) return {};
    const auto it = object.find(key);
    if (it == object.end() || !it->is_string()) return {};
    return it->get<std::string>();
}

long long integerAt(const nlohmann::json& object, const char* key, long long otherwise) {
    if (!object.is_object()) return otherwise;
    const auto it = object.find(key);
    if (it == object.end()) return otherwise;
    if (it->is_number_integer()) return it->get<long long>();
    if (it->is_number_unsigned()) {
        const auto value = it->get<unsigned long long>();
        return value > 0x7fffffffffffffffULL ? otherwise : static_cast<long long>(value);
    }
    return otherwise;
}

std::string snippet(const std::string& body, std::size_t limit) {
    std::string out;
    for (const char c : body) {
        if (out.size() >= limit) {
            out += "...";
            break;
        }
        out += (c == '\n' || c == '\r' || c == '\t') ? ' ' : c;
    }
    return out;
}

std::string httpErrorMessage(std::string_view provider, const HttpResponse& response,
                             const std::string& providerMessage) {
    if (response.status == 0) {
        std::string message = "Could not reach " + std::string(provider);
        if (!response.networkError.empty()) message += ": " + response.networkError;
        return message;
    }
    std::string message =
        std::string(provider) + " said (HTTP " + std::to_string(response.status) + ")";
    message += providerMessage.empty()
                   ? (response.body.empty() ? "" : ": " + snippet(response.body))
                   : ": " + providerMessage;
    switch (response.status) {
        case 401:
        case 403:
            message += " - check the API key in the assistant's settings.";
            break;
        case 404:
            message += " - check the model name and the server address.";
            break;
        case 429:
            message += " - the rate or spending limit was reached; wait and try again.";
            break;
        default:
            if (response.status >= 500) message += " - the service had a problem; try again.";
            break;
    }
    return message;
}

ChatResponse failed(std::string message) {
    ChatResponse response;
    response.stop = StopReason::Error;
    response.error = std::move(message);
    return response;
}

std::string dump(const nlohmann::json& value) {
    return value.dump(-1, ' ', false, nlohmann::json::error_handler_t::replace);
}

}  // namespace detail

}  // namespace hz::ai
