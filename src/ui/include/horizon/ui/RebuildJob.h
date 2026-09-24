#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <string>
#include <thread>

#include "horizon/document/FeatureTree.h"

namespace hz::doc {
class Document;
}  // namespace hz::doc

namespace hz::ui {

/// A model rebuild on a worker thread, so a long one does not freeze the
/// window. The document is snapshotted — as its native JSON — on the thread
/// that makes the job; the worker loads the snapshot into a document of its
/// own and rebuilds that, so nothing it reads can change under it. The result
/// is taken back on the GUI thread and applied only if the document has not
/// changed since the snapshot (see Stamp).
class RebuildJob {
public:
    /// What the snapshot was taken of. A result is current only while the
    /// document's undo history and feature tree are where they were.
    struct Stamp {
        std::uint64_t undoRevision = 0;
        std::uint64_t featureRevision = 0;
        bool operator==(const Stamp&) const = default;
    };
    static Stamp stampOf(const doc::Document& doc);

    explicit RebuildJob(const doc::Document& doc);
    /// Cancels the job and waits for the worker to stop: after the feature
    /// it is in, since a build stops only between features.
    ~RebuildJob();

    RebuildJob(const RebuildJob&) = delete;
    RebuildJob& operator=(const RebuildJob&) = delete;

    /// Run on a worker thread. @p onFinished is called on the worker when
    /// the build is done; post from it to the GUI thread.
    void start(std::function<void()> onFinished = {});

    /// Ask the build to stop at the next feature.
    void cancel() { m_control.cancel = true; }

    bool finished() const { return m_finished.load(); }
    int done() const { return m_control.done.load(); }
    int total() const { return m_control.total.load(); }
    const Stamp& stamp() const { return m_stamp; }

    /// The result, once finished(); it is moved out.
    doc::BuildResult takeResult();

private:
    void run();

    std::string m_snapshot;
    Stamp m_stamp;
    doc::BuildControl m_control;
    doc::BuildResult m_result;
    std::function<void()> m_onFinished;
    std::atomic<bool> m_finished{false};
    std::thread m_thread;
};

}  // namespace hz::ui
