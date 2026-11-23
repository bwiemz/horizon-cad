#pragma once

// A transport for tests: it records what is sent, and replies from a queue,
// at once or when told to.

#include <deque>
#include <functional>
#include <nlohmann/json.hpp>
#include <string>
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

}  // namespace hz::ai::test
