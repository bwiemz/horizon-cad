#pragma once

#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "horizon/ai/Conversation.h"
#include "horizon/ai/Http.h"

namespace hz::ai {

/// The wire formats Horizon speaks. OpenAICompatible is every server that
/// takes OpenAI's chat completions: Ollama, LM Studio, llama.cpp, vLLM,
/// LocalAI, Jan, OpenRouter, Groq, Mistral, DeepSeek, xAI...
enum class ProviderKind { Anthropic, OpenAI, Gemini, OpenAICompatible };

inline constexpr ProviderKind kAllProviders[] = {ProviderKind::Anthropic, ProviderKind::OpenAI,
                                                 ProviderKind::Gemini,
                                                 ProviderKind::OpenAICompatible};

struct ProviderConfig {
    ProviderKind kind = ProviderKind::Anthropic;
    /// Where to send requests. Anthropic and Gemini: the host
    /// ("https://api.anthropic.com"); OpenAI and compatible servers: up to and
    /// including the version ("https://api.openai.com/v1",
    /// "http://localhost:11434/v1").
    std::string baseUrl;
    std::string apiKey;  ///< empty: send none (a local server needs none)
    std::string model;
    int maxTokens = 16000;
    /// Anthropic only: output_config.effort ("low", "medium", "high", "xhigh",
    /// "max"); empty for the model's default.
    std::string effort;
    /// Anthropic only: ask the API to re-run a request its safety classifiers
    /// decline on Anthropic's recommended fallback model, for the models that
    /// support it.
    bool serverFallback = true;
    /// Show the model pictures of the part (get_view_image): for a model that
    /// reads images. On for the cloud providers' models, which all do; off
    /// for a compatible server, whose model may not.
    bool images = true;
    /// Ask for the reply as it is written (server-sent events), so its text
    /// shows as it comes. Off for a server that streams badly.
    bool stream = true;
};

/// The settings a new user starts from for @p kind.
ProviderConfig defaultConfig(ProviderKind kind);

/// Stable names, for settings files and the MCP server's flags: "anthropic",
/// "openai", "gemini", "openai-compatible".
const char* providerName(ProviderKind kind);
std::optional<ProviderKind> providerFromName(std::string_view name);
/// How the provider is shown to a person: "Anthropic Claude", ...
const char* providerTitle(ProviderKind kind);
/// The environment variables that hold its API key, in the order they are
/// read; none for OpenAICompatible.
std::vector<std::string> apiKeyEnvironment(ProviderKind kind);

/// Reads one streamed reply (server-sent events) as it arrives. Made by the
/// adapter that built the request; one per request. Never throws.
class StreamReader {
public:
    virtual ~StreamReader();

    /// The next bytes of the body. Returns the text of the reply they add,
    /// to show as it comes ("" for none): the model's words only, never its
    /// thinking or a tool call.
    virtual std::string feed(std::string_view bytes) = 0;
    /// The reply, once @p http has finished: what the adapter's
    /// parseResponse() gives for the same reply sent whole. A failed status
    /// is read from @p http's body as parseResponse() reads it, and so is a
    /// body that is not a stream (a server that answered whole); its body
    /// need not have been fed. A stream that ended before its end is an
    /// error.
    virtual ChatResponse finish(const HttpResponse& http) = 0;
};

/// One provider's wire format. Pure: it builds requests and reads replies,
/// and sends nothing. Never throws.
class ProviderAdapter {
public:
    virtual ~ProviderAdapter();

    virtual HttpRequest buildRequest(const ChatRequest& request,
                                     const ProviderConfig& config) const = 0;
    virtual ChatResponse parseResponse(const HttpResponse& response) const = 0;

    /// buildRequest()'s request, asking for the reply as it is written.
    virtual HttpRequest buildStreamingRequest(const ChatRequest& request,
                                              const ProviderConfig& config) const = 0;
    /// A reader for the reply to one such request.
    virtual std::unique_ptr<StreamReader> makeStreamReader() const = 0;

    /// The request for the models the account can use, and their ids from
    /// its reply (empty, and why in @p error, when it failed).
    virtual HttpRequest buildListModels(const ProviderConfig& config) const = 0;
    virtual std::vector<std::string> parseListModels(const HttpResponse& response,
                                                     std::string* error) const = 0;
};

std::unique_ptr<ProviderAdapter> makeAdapter(ProviderKind kind);

/// Whether Anthropic's server-side fallback on refusal applies to @p model.
bool anthropicSupportsServerFallback(std::string_view model);

}  // namespace hz::ai
