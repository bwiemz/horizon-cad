#include "horizon/ui/WorkerThread.h"

#include <spdlog/spdlog.h>

#include <system_error>

namespace hz::ui {

void startWorker(std::thread& thread, const std::function<void()>& work) {
    try {
        thread = std::thread(work);
    } catch (const std::system_error& e) {
        spdlog::warn("No worker thread could be started ({}); running the work here", e.what());
        work();
    }
}

}  // namespace hz::ui
