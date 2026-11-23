#pragma once

#include <functional>
#include <string>
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
    std::string networkError;  ///< why nothing arrived (refused, timed out, TLS…)
};

/// Sends requests and calls back with what came back. Asynchronous: send()
/// returns at once and @p done is called later, on the thread that owns the
/// transport (the GUI thread in the application). A transport may call
/// @p done before send() returns (a test's fake does).
class HttpTransport {
public:
    virtual ~HttpTransport();

    virtual void send(const HttpRequest& request, std::function<void(HttpResponse)> done) = 0;
    /// Abandon every request in flight; their callbacks are not called.
    virtual void cancelAll() = 0;
};

}  // namespace hz::ai
