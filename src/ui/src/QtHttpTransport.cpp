#include "horizon/ui/QtHttpTransport.h"

#include <spdlog/spdlog.h>

#include <QByteArray>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QNetworkRequest>
#include <QTimer>
#include <QUrl>
#include <exception>
#include <memory>
#include <string>
#include <string_view>
#include <utility>

#if QT_CONFIG(ssl)
#include <QSslSocket>
#endif

namespace hz::ui {

namespace {

constexpr const char* kCancelled = "hzCancelled";

}  // namespace

QtHttpTransport::QtHttpTransport(QObject* parent)
    : QObject(parent), m_manager(new QNetworkAccessManager(this)) {}

QtHttpTransport::~QtHttpTransport() {
    cancelAll();
}

bool QtHttpTransport::supportsTls() {
#if QT_CONFIG(ssl)
    return QSslSocket::supportsSsl();
#else
    return false;
#endif
}

ai::HttpTransport::RequestId QtHttpTransport::send(const ai::HttpRequest& request,
                                                   std::function<void(ai::HttpResponse)> done) {
    return start(request, nullptr, std::move(done));
}

ai::HttpTransport::RequestId QtHttpTransport::sendStreaming(
    const ai::HttpRequest& request, std::function<void(std::string_view)> part,
    std::function<void(ai::HttpResponse)> done) {
    return start(request, std::move(part), std::move(done));
}

ai::HttpTransport::RequestId QtHttpTransport::start(const ai::HttpRequest& request,
                                                    std::function<void(std::string_view)> part,
                                                    std::function<void(ai::HttpResponse)> done) {
    const RequestId id = ++m_nextId;
    const QUrl url(QString::fromStdString(request.url));
    const auto failLater = [this, id, done](const std::string& why) {
        // Called back later, as a reply would be (the caller is not yet ready
        // for an answer while send() runs), unless it is cancelled first.
        m_failing.insert(id);
        QTimer::singleShot(0, this, [this, id, done, why] {
            if (!m_failing.remove(id)) return;
            ai::HttpResponse response;
            response.networkError = why;
            done(response);
        });
    };
    if (!url.isValid() ||
        (url.scheme() != QLatin1String("http") && url.scheme() != QLatin1String("https"))) {
        failLater("the server address is not an http or https URL");
        return id;
    }
    if (url.scheme() == QLatin1String("https") && !supportsTls()) {
        failLater(
            "this build of Horizon CAD cannot make secure (https) connections: no TLS library "
            "was found. A local server over http still works.");
        return id;
    }

    QNetworkRequest networkRequest(url);
    networkRequest.setTransferTimeout(m_timeoutMs);
    // A redirect is followed only to the same scheme, host and port: the
    // request carries an API key in its headers, which must never go to a
    // host the user did not choose. A model's API does not redirect elsewhere.
    networkRequest.setAttribute(QNetworkRequest::RedirectPolicyAttribute,
                                QNetworkRequest::SameOriginRedirectPolicy);
    for (const auto& [name, value] : request.headers) {
        networkRequest.setRawHeader(QByteArray::fromStdString(name),
                                    QByteArray::fromStdString(value));
    }
    QNetworkReply* reply = nullptr;
    if (request.method == "GET") {
        reply = m_manager->get(networkRequest);
    } else {
        reply = m_manager->post(networkRequest, QByteArray::fromStdString(request.body));
    }
    m_replies.insert(id, reply);
    // The body as it came; a stream's pieces are handed over as they come.
    const bool streamed = static_cast<bool>(part);
    auto received = std::make_shared<std::string>();
    const auto take = [reply, received, part = std::move(part)] {
        const QByteArray bytes = reply->readAll();
        if (bytes.isEmpty()) return;
        received->append(bytes.constData(), static_cast<std::size_t>(bytes.size()));
        const int status = reply->attribute(QNetworkRequest::HttpStatusCodeAttribute).toInt();
        // An error's body is not a stream: it is read whole, at the end.
        if (!part || status < 200 || status >= 300) return;
        try {
            part(std::string_view(bytes.constData(), static_cast<std::size_t>(bytes.size())));
        } catch (const std::exception& e) {
            spdlog::error("AI assistant: a reply's handler failed: {}", e.what());
        }
    };
    if (streamed) {
        connect(reply, &QNetworkReply::readyRead, this, [reply, take] {
            if (!reply->property(kCancelled).toBool()) take();
        });
    }
    connect(reply, &QNetworkReply::finished, this,
            [this, id, reply, received, take, done = std::move(done)] {
                m_replies.remove(id);
                reply->deleteLater();
                if (reply->property(kCancelled).toBool()) return;
                take();  // what is left: a stream's last piece too, before the end
                ai::HttpResponse response;
                response.status =
                    reply->attribute(QNetworkRequest::HttpStatusCodeAttribute).toInt();
                response.body = std::move(*received);
                const auto failure = reply->error();
                if (response.status == 0) {
                    // No HTTP answer at all. The error text names the host, never a
                    // header: no key reaches it.
                    response.networkError = failure == QNetworkReply::OperationCanceledError ||
                                                    failure == QNetworkReply::TimeoutError
                                                ? "no reply within the time limit"
                                                : reply->errorString().toStdString();
                } else if (failure == QNetworkReply::RemoteHostClosedError ||
                           failure == QNetworkReply::TimeoutError ||
                           failure == QNetworkReply::OperationCanceledError) {
                    // An answer that stopped part-way (a stream): its reader says
                    // the reply is incomplete, and why.
                    response.networkError = failure == QNetworkReply::RemoteHostClosedError
                                                ? "the server closed the connection"
                                                : "nothing more came within the time limit";
                }
                try {
                    done(response);
                } catch (const std::exception& e) {
                    // The session's callback does not throw; a transport never lets
                    // an exception into Qt's event loop.
                    spdlog::error("AI assistant: a reply's handler failed: {}", e.what());
                }
            });
    return id;
}

void QtHttpTransport::cancel(RequestId id) {
    m_failing.remove(id);
    // abort() finishes the reply at once; marked first, it calls no one.
    const QPointer<QNetworkReply> reply = m_replies.take(id);
    if (!reply) return;
    reply->setProperty(kCancelled, true);
    reply->abort();
}

void QtHttpTransport::cancelAll() {
    m_failing.clear();
    const auto ids = m_replies.keys();
    for (const RequestId id : ids) cancel(id);
}

}  // namespace hz::ui
