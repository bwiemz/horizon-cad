#pragma once

#include <QHash>
#include <QObject>
#include <QPointer>
#include <QSet>
#include <functional>

#include "horizon/ai/Http.h"

class QNetworkAccessManager;
class QNetworkReply;

namespace hz::ui {

/// The AI assistant's HTTP transport: Qt Network, asynchronous, on the thread
/// that owns it. A request that sends nothing for the timeout fails; a
/// cancelled one never calls back.
class QtHttpTransport : public QObject, public ai::HttpTransport {
    Q_OBJECT
public:
    explicit QtHttpTransport(QObject* parent = nullptr);
    ~QtHttpTransport() override;

    /// How long a request may go without receiving anything (default five
    /// minutes: a local model on a CPU can take that long to answer).
    void setTimeoutMs(int ms) { m_timeoutMs = ms; }
    int timeoutMs() const { return m_timeoutMs; }

    RequestId send(const ai::HttpRequest& request,
                   std::function<void(ai::HttpResponse)> done) override;
    void cancel(RequestId id) override;
    /// Abandon every request in flight.
    void cancelAll();

    /// Whether HTTPS can be used: this Qt has a TLS backend that loads.
    static bool supportsTls();

private:
    QNetworkAccessManager* m_manager;
    QHash<RequestId, QPointer<QNetworkReply>> m_replies;
    QSet<RequestId> m_failing;  ///< refused before sending: answered next pass
    RequestId m_nextId = 0;
    int m_timeoutMs = 300000;
};

}  // namespace hz::ui
