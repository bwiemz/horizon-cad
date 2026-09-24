#pragma once

#include <atomic>
#include <exception>
#include <functional>
#include <string>
#include <thread>
#include <utility>

namespace hz::ui {

/// Work run on a worker thread while the window stays usable: an import, an
/// interference check. Cancelling stops waiting for it; work that does not
/// look at the flag it is given runs on to its end, and its result is
/// dropped. The destructor waits for the worker.
template <typename Result>
class BackgroundTask {
public:
    using Work = std::function<Result(const std::atomic<bool>& cancelled)>;

    explicit BackgroundTask(Work work) : m_work(std::move(work)) {}
    ~BackgroundTask() {
        cancel();
        if (m_thread.joinable()) m_thread.join();
    }

    BackgroundTask(const BackgroundTask&) = delete;
    BackgroundTask& operator=(const BackgroundTask&) = delete;

    /// Run on a worker. @p onFinished is called on the worker when the work
    /// is done; post from it to the GUI thread.
    void start(std::function<void()> onFinished = {}) {
        m_onFinished = std::move(onFinished);
        m_thread = std::thread([this] { run(); });
    }

    void cancel() { m_cancelled = true; }
    bool cancelled() const { return m_cancelled.load(); }
    bool finished() const { return m_finished.load(); }

    /// The result once finished(), moved out, and what the work threw, if
    /// anything (then the result is default-made).
    Result take() { return std::move(m_result); }
    const std::string& error() const { return m_error; }

private:
    void run() {
        // Nothing may escape a worker thread: it would end the application.
        try {
            m_result = m_work(m_cancelled);
        } catch (const std::exception& e) {
            m_error = e.what();
        } catch (...) {
            m_error = "an unknown error";
        }
        m_finished = true;
        if (m_onFinished) m_onFinished();
    }

    Work m_work;
    Result m_result{};
    std::string m_error;
    std::function<void()> m_onFinished;
    std::atomic<bool> m_cancelled{false};
    std::atomic<bool> m_finished{false};
    std::thread m_thread;
};

}  // namespace hz::ui
