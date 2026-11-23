#pragma once

// A transport for tests: it records what is sent, and replies from a queue,
// at once or when told to.

#include <deque>
#include <functional>
#include <nlohmann/json.hpp>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "horizon/ai/Http.h"

namespace hz::ai::test {

class FakeTransport : public HttpTransport {
public:
    std::vector<HttpRequest> requests;
    std::deque<HttpResponse> replies;
    /// Hold replies until releaseNext() rather than answering in send().
    bool hold = false;
    /// Keep held callbacks through cancel(), as a transport that cannot
    /// stop a request in time would: the session must ignore them.
    bool keepThroughCancel = false;
    int cancels = 0;
    /// Say it streams, and hand a reply's body over in pieces this long.
    bool streaming = false;
    std::size_t piece = 7;
    /// The pieces handed over, in order.
    std::vector<std::string> pieces;
    /// Called after each piece is handed over: to see what it did.
    std::function<void()> afterPiece;

    bool streams() const override { return streaming; }

    RequestId sendStreaming(const HttpRequest& request, std::function<void(std::string_view)> part,
                            std::function<void(HttpResponse)> done) override {
        return send(request,
                    [this, part = std::move(part), done = std::move(done)](HttpResponse response) {
                        if (response.status >= 200 && response.status < 300) {
                            for (std::size_t at = 0; at < response.body.size(); at += piece) {
                                pieces.push_back(response.body.substr(at, piece));
                                part(pieces.back());
                                if (afterPiece) afterPiece();
                            }
                        }
                        done(std::move(response));
                    });
    }

    RequestId send(const HttpRequest& request, std::function<void(HttpResponse)> done) override {
        requests.push_back(request);
        const RequestId id = ++m_nextId;
        if (hold) {
            m_pending.push_back({id, std::move(done)});
            return id;
        }
        done(next());
        return id;
    }

    void cancel(RequestId id) override {
        ++cancels;
        cancelled.push_back(id);
        if (!keepThroughCancel) {
            std::erase_if(m_pending, [id](const Pending& p) { return p.id == id; });
        }
    }

    std::vector<RequestId> cancelled;

    std::size_t pending() const { return m_pending.size(); }

    /// Answer the oldest held request with the next queued reply.
    void releaseNext() {
        if (m_pending.empty()) return;
        auto done = std::move(m_pending.front().done);
        m_pending.pop_front();
        done(next());
    }

    /// The JSON body of request @p index.
    nlohmann::json body(std::size_t index) const {
        return nlohmann::json::parse(requests.at(index).body);
    }

private:
    HttpResponse next() {
        if (replies.empty())
            return {500, R"({"type":"error","error":{"message":"no reply queued"}})", {}};
        HttpResponse reply = replies.front();
        replies.pop_front();
        return reply;
    }

    struct Pending {
        RequestId id;
        std::function<void(HttpResponse)> done;
    };
    std::deque<Pending> m_pending;
    RequestId m_nextId = 0;
};

/// An Anthropic reply: text and tool calls ({id, name, input}).
inline HttpResponse anthropicReply(const std::string& text,
                                   const std::vector<nlohmann::json>& calls = {},
                                   const std::string& stop = "") {
    nlohmann::json content = nlohmann::json::array();
    if (!text.empty()) content.push_back({{"type", "text"}, {"text", text}});
    for (const auto& call : calls) {
        content.push_back({{"type", "tool_use"},
                           {"id", call.at("id")},
                           {"name", call.at("name")},
                           {"input", call.value("input", nlohmann::json::object())}});
    }
    const std::string reason = !stop.empty() ? stop : (calls.empty() ? "end_turn" : "tool_use");
    return {200,
            nlohmann::json{{"type", "message"},
                           {"content", content},
                           {"stop_reason", reason},
                           {"usage", {{"input_tokens", 10}, {"output_tokens", 5}}}}
                .dump(),
            {}};
}

/// Server-sent events: each {"event": name, "data": json}, as a provider
/// streams them.
inline std::string sse(const std::vector<nlohmann::json>& events) {
    std::string out;
    for (const auto& event : events) {
        if (event.contains("event")) out += "event: " + event["event"].get<std::string>() + "\n";
        const auto& data = event["data"];
        out += "data: " + (data.is_string() ? data.get<std::string>() : data.dump()) + "\n\n";
    }
    return out;
}

/// anthropicReply(), streamed: the text in pieces of @p piece characters,
/// each tool call's input in two.
inline HttpResponse anthropicStream(const std::string& text,
                                    const std::vector<nlohmann::json>& calls = {},
                                    const std::string& stop = "", std::size_t piece = 4) {
    using nlohmann::json;
    const auto event = [](const json& data) {
        return json{{"event", data["type"]}, {"data", data}};
    };
    std::vector<json> events;
    events.push_back(event({{"type", "message_start"},
                            {"message",
                             {{"type", "message"},
                              {"role", "assistant"},
                              {"content", json::array()},
                              {"usage", {{"input_tokens", 10}, {"output_tokens", 1}}}}}}));
    int index = 0;
    if (!text.empty()) {
        events.push_back(event({{"type", "content_block_start"},
                                {"index", index},
                                {"content_block", {{"type", "text"}, {"text", ""}}}}));
        for (std::size_t at = 0; at < text.size(); at += piece) {
            events.push_back(
                event({{"type", "content_block_delta"},
                       {"index", index},
                       {"delta", {{"type", "text_delta"}, {"text", text.substr(at, piece)}}}}));
        }
        events.push_back(event({{"type", "content_block_stop"}, {"index", index}}));
        ++index;
    }
    for (const auto& call : calls) {
        events.push_back(event({{"type", "content_block_start"},
                                {"index", index},
                                {"content_block",
                                 {{"type", "tool_use"},
                                  {"id", call.at("id")},
                                  {"name", call.at("name")},
                                  {"input", json::object()}}}}));
        const std::string input = call.value("input", json::object()).dump();
        const std::size_t half = input.size() / 2;
        for (const std::string& part : {input.substr(0, half), input.substr(half)}) {
            events.push_back(
                event({{"type", "content_block_delta"},
                       {"index", index},
                       {"delta", {{"type", "input_json_delta"}, {"partial_json", part}}}}));
        }
        events.push_back(event({{"type", "content_block_stop"}, {"index", index}}));
        ++index;
    }
    const std::string reason = !stop.empty() ? stop : (calls.empty() ? "end_turn" : "tool_use");
    events.push_back(event({{"type", "message_delta"},
                            {"delta", {{"stop_reason", reason}}},
                            {"usage", {{"output_tokens", 5}}}}));
    events.push_back(event({{"type", "message_stop"}}));
    return {200, sse(events), {}};
}

}  // namespace hz::ai::test
