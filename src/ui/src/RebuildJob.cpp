#include "horizon/ui/RebuildJob.h"

#include <exception>
#include <utility>

#include "horizon/document/Document.h"
#include "horizon/document/UndoStack.h"
#include "horizon/fileio/NativeFormat.h"
#include "horizon/ui/WorkerThread.h"

namespace hz::ui {

RebuildJob::Stamp RebuildJob::stampOf(const doc::Document& doc) {
    return {doc.undoStack().revision(), doc.featureTree().revision()};
}

RebuildJob::RebuildJob(const doc::Document& doc)
    : m_snapshot(io::NativeFormat::documentToJson(doc, false)),
      m_exact(doc.exactCounterpart()),
      m_stamp(stampOf(doc)) {}

RebuildJob::~RebuildJob() {
    cancel();
    if (m_thread.joinable()) m_thread.join();
}

void RebuildJob::start(std::function<void()> onFinished) {
    m_onFinished = std::move(onFinished);
    startWorker(m_thread, [this] { run(); });
}

void RebuildJob::run() {
    // Nothing may escape a worker thread: std::terminate would take the
    // whole application with it.
    try {
        doc::Document copy;
        std::string error;
        if (io::NativeFormat::documentFromJson(m_snapshot, copy, &error)) {
            // The document's exact counterpart reused while its imports are
            // as they were: the copy reads them again from the part's STEP
            // text, and they are the same.
            doc::BuildOptions options;
            options.previous = m_exact;
            m_result = copy.buildWithDiagnostics(&m_control, options);
        } else {
            m_result.failedFeatureIndex = 0;
            m_result.failureMessage = "the model could not be copied for rebuilding: " + error;
        }
    } catch (const std::exception& e) {
        m_result = {};
        m_result.failedFeatureIndex = 0;
        m_result.failureMessage = std::string("the rebuild failed: ") + e.what();
    } catch (...) {
        m_result = {};
        m_result.failedFeatureIndex = 0;
        m_result.failureMessage = "the rebuild failed";
    }
    m_finished = true;
    if (m_onFinished) m_onFinished();
}

doc::BuildResult RebuildJob::takeResult() {
    return std::move(m_result);
}

}  // namespace hz::ui
