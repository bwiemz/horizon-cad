#pragma once

// What the provider adapters share: URLs, tolerant JSON reading, and error
// messages a person can act on. Internal to hz_ai.

#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "horizon/ai/Conversation.h"
#include "horizon/ai/Http.h"
#include "horizon/ai/Provider.h"

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

/// Add @p more to the string @p object[@p key] where it is (one that is not a
/// string yet is made one): a reply's text grows by many small pieces, and
/// building it again for each would take time as the square of its length.
void appendAt(nlohmann::json& object, const char* key, std::string_view more);

/// Serialise @p value for a request body; invalid UTF-8 is replaced, never
/// thrown on.
std::string dump(const nlohmann::json& value);

/// Whether @p status is a success.
inline bool succeeded(int status) {
    return status >= 200 && status < 300;
}

/// One server-sent event.
struct SseEvent {
    std::string event;  ///< its `event:` field, "" when it has none
    std::string data;   ///< its `data:` lines, joined by '\n'
};

/// Splits a server-sent event stream into its events. Fed in pieces of any
/// size: a line, or the two bytes of a CRLF, may be split between two.
/// Comments (lines starting ':') and `id:` / `retry:` are passed over.
class SseParser {
public:
    /// The events @p bytes complete.
    std::vector<SseEvent> feed(std::string_view bytes);
    /// The stream has ended: the event it left open, if it has data (a
    /// server that does not end its last event with a blank line).
    std::vector<SseEvent> finish();

private:
    void endLine(std::vector<SseEvent>& out);

    std::string m_line;
    bool m_afterCr = false;  ///< the last byte ended a line with CR: skip an LF next
    bool m_started = false;  ///< past a byte-order mark, if there was one
    SseEvent m_event;
    bool m_hasData = false;
};

/// What every provider's stream reader shares: the events split out, a
/// failed status or a body that is not a stream read as a whole reply, and
/// a stream that stopped before its end. Never throws.
class SseStreamReader : public StreamReader {
public:
    std::string feed(std::string_view bytes) final;
    ChatResponse finish(const HttpResponse& http) final;

protected:
    /// One event; the reply's text it adds.
    virtual std::string onEvent(const SseEvent& event) = 0;
    /// The reply the events made, once the stream ended; @p http for its
    /// status and anything the network said.
    virtual ChatResponse finishStream(const HttpResponse& http) = 0;
    /// The adapter's parseResponse().
    virtual ChatResponse parseWhole(const HttpResponse& http) const = 0;
    /// Who answered, for messages ("Anthropic").
    virtual std::string provider() const = 0;

    /// The reply to a stream that stopped before its end.
    ChatResponse cutOff(const HttpResponse& http) const;
    /// The reply to an error a stream reported, read as parseResponse() reads
    /// an error body with @p status.
    ChatResponse streamError(const nlohmann::json& error, int status) const;

private:
    SseParser m_parser;
    bool m_fed = false;
    bool m_events = false;
    std::string m_broken;  ///< why an event could not be read, if one could not
};

}  // namespace hz::ai::detail
