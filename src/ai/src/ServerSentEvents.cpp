// Server-sent events (the HTML standard's "text/event-stream"), as every
// provider streams a reply, and what the providers' stream readers share.

#include <exception>
#include <string>
#include <utility>
#include <vector>

#include "AdapterUtil.h"

namespace hz::ai::detail {

namespace {

constexpr std::string_view kByteOrderMark = "\xEF\xBB\xBF";

}  // namespace

void appendAt(nlohmann::json& object, const char* key, std::string_view more) {
    nlohmann::json& value = object[key];
    if (!value.is_string()) value = std::string();
    value.get_ref<std::string&>().append(more);
}

std::vector<SseEvent> SseParser::feed(std::string_view bytes) {
    std::vector<SseEvent> out;
    if (!m_started) {
        // A byte-order mark is allowed before the first line, and is not part
        // of it. It may come split; wait for enough bytes to tell.
        m_line.append(bytes);
        if (m_line.size() < kByteOrderMark.size() &&
            kByteOrderMark.substr(0, m_line.size()) == m_line) {
            return out;
        }
        m_started = true;
        std::string held = std::move(m_line);
        m_line.clear();
        if (held.compare(0, kByteOrderMark.size(), kByteOrderMark) == 0) {
            held.erase(0, kByteOrderMark.size());
        }
        return feed(held);
    }
    for (const char c : bytes) {
        if (m_afterCr) {
            m_afterCr = false;
            if (c == '\n') continue;  // the rest of a CRLF
        }
        if (c == '\r' || c == '\n') {
            m_afterCr = c == '\r';
            endLine(out);
        } else {
            m_line += c;
        }
    }
    return out;
}

std::vector<SseEvent> SseParser::finish() {
    std::vector<SseEvent> out;
    if (!m_started) {
        m_started = true;
        if (m_line == kByteOrderMark.substr(0, m_line.size())) m_line.clear();
    }
    if (!m_line.empty()) endLine(out);
    if (m_hasData) {
        out.push_back(std::move(m_event));
        m_event = {};
        m_hasData = false;
    }
    return out;
}

void SseParser::endLine(std::vector<SseEvent>& out) {
    const std::string line = std::move(m_line);
    m_line.clear();
    if (line.empty()) {
        // A blank line ends the event. One with no data is no event.
        if (m_hasData) out.push_back(std::move(m_event));
        m_event = {};
        m_hasData = false;
        return;
    }
    if (line.front() == ':') return;  // a comment (a keep-alive)
    const std::size_t colon = line.find(':');
    const std::string field = line.substr(0, colon);
    std::string value;
    if (colon != std::string::npos) {
        value = line.substr(colon + 1);
        if (!value.empty() && value.front() == ' ') value.erase(0, 1);
    }
    if (field == "data") {
        if (m_hasData) m_event.data += '\n';
        m_event.data += value;
        m_hasData = true;
    } else if (field == "event") {
        m_event.event = std::move(value);
    }
}

std::string SseStreamReader::feed(std::string_view bytes) {
    m_fed = true;
    std::string text;
    try {
        for (const auto& event : m_parser.feed(bytes)) {
            m_events = true;
            text += onEvent(event);
        }
    } catch (const std::exception& e) {
        // A reader does not throw; an event it did not foresee still must
        // not reach the transport's event loop.
        if (m_broken.empty()) m_broken = e.what();
    }
    return text;
}

ChatResponse SseStreamReader::finish(const HttpResponse& http) {
    try {
        // An error is a JSON body, not a stream.
        if (http.status == 0 || !succeeded(http.status)) return parseWhole(http);
        if (!m_fed && !http.body.empty()) feed(http.body);
        for (const auto& event : m_parser.finish()) {
            m_events = true;
            onEvent(event);
        }
        if (!m_broken.empty()) return failed("The reply could not be read: " + m_broken);
        if (!m_events) {
            // Nothing came after the headers: the connection closed first.
            if (http.body.find_first_not_of(" \t\r\n") == std::string::npos) {
                return cutOff(http);
            }
            // A server that took no notice of the request to stream answers
            // whole.
            return parseWhole(http);
        }
        return finishStream(http);
    } catch (const std::exception& e) {
        return failed(std::string("The reply could not be read: ") + e.what());
    }
}

ChatResponse SseStreamReader::cutOff(const HttpResponse& http) const {
    std::string message = provider() + "'s reply stopped before it was complete";
    if (!http.networkError.empty()) message += " (" + http.networkError + ")";
    return failed(message + ". Try again.");
}

ChatResponse SseStreamReader::streamError(const nlohmann::json& error, int status) const {
    HttpResponse http;
    http.status = status;
    http.body = dump(error);
    return parseWhole(http);
}

}  // namespace hz::ai::detail
