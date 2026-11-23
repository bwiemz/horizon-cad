#pragma once

// What the provider adapters share: URLs, tolerant JSON reading, and error
// messages a person can act on. Internal to hz_ai.

#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <string_view>

#include "horizon/ai/Conversation.h"
#include "horizon/ai/Http.h"

namespace hz::ai::detail {

/// @p base without trailing slashes, then @p path (which starts with '/').
std::string joinUrl(std::string_view base, std::string_view path);

/// @p body as JSON, or nullopt when it is not JSON. Never throws.
std::optional<nlohmann::json> parseJson(const std::string& body);

/// @p object[@p key] when it is a string, else "".
std::string stringAt(const nlohmann::json& object, const char* key);
/// @p object[@p key] when it is an integer, else @p otherwise.
long long integerAt(const nlohmann::json& object, const char* key, long long otherwise = 0);

/// The body as text for an error message: at most @p limit characters, on one
/// line.
std::string snippet(const std::string& body, std::size_t limit = 300);

/// A response that failed, said for a person: no response at all, or
/// "<Provider> said (HTTP 401): <its message>" with a hint for the statuses
/// people can fix (a key, a model name, a limit).
std::string httpErrorMessage(std::string_view provider, const HttpResponse& response,
                             const std::string& providerMessage);

/// A ChatResponse that failed with @p message.
ChatResponse failed(std::string message);

/// Serialise @p value for a request body; invalid UTF-8 is replaced, never
/// thrown on.
std::string dump(const nlohmann::json& value);

/// Whether @p status is a success.
inline bool succeeded(int status) {
    return status >= 200 && status < 300;
}

}  // namespace hz::ai::detail
