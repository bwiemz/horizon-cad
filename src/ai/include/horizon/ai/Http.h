#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace hz::ai {

/// An HTTP request as an adapter builds it: no transport of its own, so the
/// adapters stay pure and testable, and the application chooses how to send.
struct HttpRequest {
    std::string method = "POST";
    std::string url;
    std::vector<std::pair<std::string, std::string>> headers;
    std::string body;

    /// The value of header @p name (case-insensitive), "" when it is not set.
    std::string header(const std::string& name) const;
};

struct HttpResponse {
    int status = 0;  ///< 0 when no response arrived
    std::string body;
    std::string networkError;  ///< why nothing arrived (refused, timed out, TLS...)
};

/// Sends requests and calls back with what came back. Asynchronous: send()
/// returns at once and @p done is called later, on the thread that owns the
/// transport (the GUI thread in the application). A transport may call
/// @p done before send() returns (a test's fake does).
class HttpTransport {
public:
    using RequestId = std::uint64_t;

    virtual ~HttpTransport();

    /// Send @p request; its id, for cancel().
    virtual RequestId send(const HttpRequest& request, std::function<void(HttpResponse)> done) = 0;

    /// Whether sendStreaming() hands over a reply's body as it arrives. When
    /// it does not, there is no point asking a server to stream.
    virtual bool streams() const { return false; }
    /// Like send(), and @p part is called with each piece of the body as it
    /// arrives, when the status is a success. @p done is still called last,
    /// with the whole body. By default the whole body is one piece, given
    /// just before @p done.
    virtual RequestId sendStreaming(const HttpRequest& request,
                                    std::function<void(std::string_view)> part,
                                    std::function<void(HttpResponse)> done);
    /// Abandon request @p id if it is still in flight: its callback is not
    /// called. Others sent through the same transport are not touched.
    virtual void cancel(RequestId id) = 0;
};

}  // namespace hz::ai
